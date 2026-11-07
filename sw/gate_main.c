// =====================================================================
// gate_main.c —— 门禁闭环编排（CPU 侧，无相机方案）
//
// 每帧的完整流程
//   ┌ 9 个窗循环 ────────────────────────────────────────────────┐
//   │ seek(win) → 等 win_done                                    │
//   │ fsrc_go  → 等 done            （窗口 128×96 → ACT 的 q8）   │
//   │ net_go(det, hold=1)                                        │
//   │   for 42 层：等 layer_done → 若是 head 层就读回 → 放行      │
//   │ 用读回的 cls/box 解码本窗候选 → 本窗 NMS                   │
//   └────────────────────────────────────────────────────────────┘
//   跨窗 NMS → 取面积最大
//   seek(候选所在窗) → 等 win_done
//   roi_go（窗内坐标 ×1.2）→ 等 done
//   net_go(rec, hold=0) → 等 net_done
//   d1_go → 等 done
//   读 64 维 embedding → 与模板逐个比 → 门禁状态机
//
// 为什么 det 必须 hold=1 逐层读
//   head 层的 ACT `out_base` **互相重叠**（H2c/H2b 都是 12480、H3c/H3b 都是
//   12384、H4c/H4b 都是 12288），后一个 head 会覆盖前一个。
//   所以只能在 layer_done 那一刻把该层读走 —— 这就是 cnn_soc 新增
//   `layer_done` 输出的唯一理由。
//
// 为什么每个像素一次 rd_go
//   cnn_soc 的 `rd_*` 是"一拍一个地址"、**没有背压**，CPU 跟不上。
//   cnn_mmio 只抓第一个 beat ⇒ 一次 rd_go 读一个像素（32 通道）。
//   代价：每像素约 10 拍，一窗 512 像素 ≈ 5 K 拍，9 窗 ≈ 46 K 拍 = 0.3 ms
//   （占 7.8 ms/帧 的 4%，可接受）。
// =====================================================================
#include "gate_map.h"
#include "gate_regs.h"
#include "gate_core.h"
#include "gate_api.h"

//=====================================================================
// 板级钩子（由 board 层实现；纯逻辑测试时给空实现即可）
//=====================================================================
__attribute__((weak)) void     gate_on_result(const gate_result_t *r) { (void)r; }
__attribute__((weak)) uint32_t gate_cycle(void) { return 0; }
__attribute__((weak)) void     gate_on_init(void) { }

//=====================================================================
// 内部状态
//=====================================================================
#define NWIN        (GATE_SW_NX * GATE_SW_NY)
#define SW_SX       ((GATE_BUF_W - GATE_DET_W) / (GATE_SW_NX - 1))   // 96
#define SW_SY       ((GATE_BUF_H - GATE_DET_H) / (GATE_SW_NY - 1))   // 72
#define PER_WIN     8                  // 每窗最多留几个候选（按 d 保留最大的几个）
#define MAX_CAND    (NWIN * PER_WIN)
#define CLS_MAX     (16 * 12 * 8)      // cls 一层最大字节数（scale0: 12×16×8）

// ★ 帧数适配：ROM 只有 4 帧（走近/对准/贴近/陌生人），循环播放。
//   PASS_N=3 时"连续 3 帧认出"永远等不到（序列是 2 好 1 坏 1 空）
//   ⇒ 取 PASS_N=2：2 好帧开门、随后 2 坏帧关门，正好演示完整循环。
//   真机 30fps 时应改回 3（或直接用 1 秒窗口中位数，见 doc §8）。
#define PASS_N      2                  // 连续 2 帧通过 ⇒ 开门
#define FAIL_N      2                  // 连续 2 帧不通过 ⇒ 关门
#define HOLD_FRAMES 4                  // 开门后保持多少帧（闩锁）

#define THR_D       0                  // face 判据阈值（与 golden 一致）
#define MIN_SIDE    8                  // 最小框边长（与 golden 一致）
#define IOU_MILLI   400                // NMS 阈值 0.40

static gdec_t   s_dec;
static uint8_t  s_cls[CLS_MAX];        // cls 层缓存（1 bank = 8 通道/像素）
static gbox_t   s_cand[MAX_CAND];
static int      s_ncand;               // 已收集的候选数（**顺序追加**，不留空洞）

static ggate_t  s_gate;                // 门禁状态机（在 gate_core.c 里，可单测）
static int      s_fr;                  // 当前帧号（% GATE_SWF）

//---------------------------------------------------------------------
// 小工具
//---------------------------------------------------------------------
// 轮询上限：约 2e6 次 × ~3 拍 ≈ 6e6 拍 ≈ 40 ms @150 MHz。
// 为什么不用 ctrl_timeout 位：那是 cnn_ctrl 的**粘滞**标志，一旦置起就再也清不掉
// （MMIO 没有清它的口），会让后续所有 wait 立刻返回 ⇒ 死循环。
// 所以这里用本地计数做安全网，与 RTL 状态无关。
#define POLL_LIMIT  2000000u

// 返回 0 = 等到；-1 = 超时（上层当作"这一步失败"，不往下走）
static int wait_bits(uint32_t bits, uint32_t clr)
{
    uint32_t n = 0;
    // Clear BEFORE issuing an operation, not here: a fast completion may already
    // be sticky by the time software enters this wait (especially net_done).
    while ((GATE_REG(R_STATUS) & bits) == 0) {
        if (++n > POLL_LIMIT)
            return -1;
    }
    GATE_REG(R_CLR) = clr;
    return 0;
}

// 读一个像素的若干 32 b 字（nw = bank 数 × 2）
static void rd_pixel(uint16_t pix, int nw, uint8_t *out)
{
    int k;
    uint32_t n;
    GATE_REG(R_CLR)    = X_RD;
    GATE_REG(R_RDADDR) = (uint32_t)pix | (1u << 16);   // nword = 1
    // CTRL also writes the level-sensitive hold bit. Preserve it while reading
    // detection heads, otherwise the following layers overwrite the head buffer.
    GATE_REG(R_CTRL)   = C_RD_GO | C_HOLD;
    n = 0;
    while ((GATE_REG(R_STATUS) & S_RD_READY) == 0)
        if (++n > 100000u) break;      // 读回失败就当全 0，别死等
    for (k = 0; k < nw; k++) {
        GATE_REG(R_RDIDX) = (uint32_t)k;
        ((uint32_t *)out)[k] = GATE_REG(R_RDDATA);
    }
    GATE_REG(R_CLR) = X_RD;
}

// ★ seek 必须带帧号：frame_idx 只在「最后一窗 + go」时前进，
//   全程用 seek 定位的话它永远停在 0，ROM 一直播第 0 帧（不报错，只是视频不动）。
static void seek_win(int win, int frame)
{
    GATE_REG(R_CLR)     = X_SW_WIN;
    GATE_REG(R_WINSEL)  = (uint32_t)win;
    GATE_REG(R_WINSEL2) = (uint32_t)(frame & 0xFF) << 18;
    GATE_REG(R_CTRL)    = C_SEEK;                  // seek_xy = 0 ⇒ 按 3×3 网格窗
    (void)wait_bits(S_SW_WIN, X_SW_WIN);
}

// 把窗口原点对到 (x0,y0)（任意位置，钳到缓冲内）—— 给 ROI 用
static void seek_at(int x0, int y0, int frame)
{
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x0 > GATE_BUF_W - GATE_DET_W) x0 = GATE_BUF_W - GATE_DET_W;
    if (y0 > GATE_BUF_H - GATE_DET_H) y0 = GATE_BUF_H - GATE_DET_H;
    GATE_REG(R_CLR)     = X_SW_WIN;
    GATE_REG(R_WINSEL2) = (uint32_t)x0 | ((uint32_t)y0 << 9)
                        | ((uint32_t)(frame & 0xFF) << 18);
    GATE_REG(R_CTRL)    = C_SEEK | C_SEEKXY;
    (void)wait_bits(S_SW_WIN, X_SW_WIN);
}

//---------------------------------------------------------------------
// 读 cls 层（1 bank ⇒ 每像素 8 字节）
//---------------------------------------------------------------------
static void read_cls(const gscale_t *sc)
{
    int p, n = (int)sc->oh * sc->ow;
    for (p = 0; p < n; p++)
        rd_pixel((uint16_t)(sc->cls_pix + p), 2, &s_cls[p * 8]);
}

//---------------------------------------------------------------------
// 读 box 层并顺手解码（cls 已在 s_cls 里）
//
// ★ 每窗只保留 **d 最大的 PER_WIN 个**，而不是"先到先留"。
//   为什么：head 层按 anchor 顺序产出（gy 外、gx 内、ai 最内），
//   如果按先到先留，一个"排在最后但分数很高"的脸会被丢掉。
//   保留策略：本地小数组按 d 降序插入，满了就和新值比，小的直接扔。
//---------------------------------------------------------------------
static void decode_box_layer(const gscale_t *sc, int win, int ox, int oy)
{
    int p, ai, n = (int)sc->oh * sc->ow;
    int nloc = 0;
    uint8_t raw[32];
    gbox_t loc[PER_WIN];
    const int32_t q = s_dec.q;

    for (p = 0; p < n; p++) {
        int gx = p % sc->ow, gy = p / sc->ow;
        // 中心定点：round(ax*A) = (2gx+1)*sx*2^(q-1)，与 hwnet 精确相等
        int32_t axq = (int32_t)(2 * gx + 1) * ((int32_t)sc->sx << (q - 1));
        int32_t ayq = (int32_t)(2 * gy + 1) * ((int32_t)sc->sy << (q - 1));

        rd_pixel((uint16_t)(sc->box_pix + p), sc->box_banks * 2, raw);

        for (ai = 0; ai < sc->na; ai++) {
            const int8_t *cl = (const int8_t *)&s_cls[p * 8];
            const int8_t *bx = (const int8_t *)&raw[ai * 4];
            int d = (int)cl[2 * ai + 1] - (int)cl[2 * ai];
            int16_t x1, y1, x2, y2;
            int k;

            if (d < THR_D)
                continue;
            g_decode(&s_dec, axq, ayq, sc->sizes[ai], sc->sizes[ai],
                     bx[0], bx[1], bx[2], bx[3], &x1, &y1, &x2, &y2);
            if (x2 - x1 < MIN_SIDE || y2 - y1 < MIN_SIDE)
                continue;

            // 按 d 降序插入（PER_WIN 很小，插入排序最省）
            if (nloc == PER_WIN && d <= loc[PER_WIN - 1].d)
                continue;                       // 连最差的那个都不如，直接扔
            k = (nloc < PER_WIN) ? nloc++ : PER_WIN - 1;
            while (k > 0 && loc[k - 1].d < (int16_t)d) {
                loc[k] = loc[k - 1];
                k--;
            }
            loc[k].lx1 = x1; loc[k].ly1 = y1; loc[k].lx2 = x2; loc[k].ly2 = y2;
            loc[k].x1 = (int16_t)(x1 + ox); loc[k].y1 = (int16_t)(y1 + oy);
            loc[k].x2 = (int16_t)(x2 + ox); loc[k].y2 = (int16_t)(y2 + oy);
            loc[k].d  = (int16_t)d;
            loc[k].win = (uint8_t)win;
        }
    }

    // 顺序追加到全局表（不留空洞）
    for (ai = 0; ai < nloc; ai++) {
        if (s_ncand >= MAX_CAND)
            break;
        s_cand[s_ncand++] = loc[ai];
    }
}

//---------------------------------------------------------------------
// 跑一窗的检测（返回本窗候选数）
//---------------------------------------------------------------------
static int run_window_det(int win, int frame)
{
    int si, s2, n0 = s_ncand;
    int ox = (win % GATE_SW_NX) * ((GATE_BUF_W - GATE_DET_W) / (GATE_SW_NX - 1));
    int oy = (win / GATE_SW_NX) * ((GATE_BUF_H - GATE_DET_H) / (GATE_SW_NY - 1));

    // ---- 窗口进 FB ----
    seek_win(win, frame);
    GATE_REG(R_CLR) = X_FSRC;
    GATE_REG(R_CTRL) = C_FSRC_GO;
    if (wait_bits(S_FSRC_DONE, X_FSRC) != 0)
        return 0;

    // ---- det 42 层，逐层 hold ----
    GATE_REG(R_CLR)     = X_ALL;
    GATE_REG(R_NETCFG)  = (uint32_t)GATE_NLD;        // woff = 0
    GATE_REG(R_NETCFG2) = 0u;                        // boff = 0, desc_base = 0
    GATE_REG(R_CTRL)    = C_NET_GO | C_HOLD;

    for (si = 0; si < GATE_NLD; si++) {
        int idx;
        if (wait_bits(S_LAYER, X_LAYER) != 0)
            return s_ncand - n0;       // 层超时：本窗候选就用已拿到的
        idx = (int)(GATE_REG(R_LAYER) & 0xFF);

        // 这一层是某个尺度的 cls / box 吗？（head 层 out_base 重叠 ⇒ 必须现在读）
        for (s2 = 0; s2 < 4; s2++) {
            if (idx == (int)GATE_SC[s2].cls)
                read_cls(&GATE_SC[s2]);
            else if (idx == (int)GATE_SC[s2].box)
                decode_box_layer(&GATE_SC[s2], win, ox, oy);
        }

        // 放行下一层（hold 是电平：先清再置）
        GATE_REG(R_CTRL) = 0;
        GATE_REG(R_CTRL) = C_HOLD;
    }
    (void)wait_bits(S_NET_DONE, X_NET);
    return s_ncand - n0;
}

//---------------------------------------------------------------------
// 识别 + 门禁：返回命中的模板下标（-1 = 未命中 / 无 ROI）
//---------------------------------------------------------------------
static int run_rec_and_gate(const gbox_t *best, int ox, int oy, int32_t *cos1000)
{
    int rx1, ry1, rx2, ry2, best_t = -1;
#ifndef GATE_MULTI_ID
    int t;
#endif

    // 与 seek_at 同样的钳位，保证 ROI 的局部坐标和 FB 内容一致
    if (ox < 0) ox = 0;
    if (oy < 0) oy = 0;
    if (ox > GATE_BUF_W - GATE_DET_W) ox = GATE_BUF_W - GATE_DET_W;
    if (oy > GATE_BUF_H - GATE_DET_H) oy = GATE_BUF_H - GATE_DET_H;
#ifndef GATE_MULTI_ID
    int32_t best_dot = 0;
#endif
    int32_t dot, nq2;
    uint8_t emb[64];

    *cos1000 = -1000;
    // 注意：传的是**选定窗**的原点，不是候选自己的窗
    if (!g_roi_of(best, ox, oy, GATE_DET_W, GATE_DET_H, 1200, MIN_SIDE,
                  &rx1, &ry1, &rx2, &ry2))
        return -1;

    // ---- ROI：窗内坐标 → 64×64（cnn_roi 读的是当前窗的 FB）----
    GATE_REG(R_ROI) = (uint32_t)rx1 | ((uint32_t)ry1 << 8)
                    | ((uint32_t)rx2 << 16) | ((uint32_t)ry2 << 24);
    GATE_REG(R_CLR) = X_ROI;
    GATE_REG(R_CTRL) = C_ROI_GO;
    if (wait_bits(S_ROI_DONE, X_ROI) != 0)
        return -1;

    // ---- rec 20 层（hold=0：不需要逐层读）----
    GATE_REG(R_CLR)     = X_ALL;
    GATE_REG(R_NETCFG)  = (uint32_t)GATE_NLR | ((uint32_t)GATE_NET_WOFF << 8);
    GATE_REG(R_NETCFG2) = (uint32_t)GATE_NET_BOFF | ((uint32_t)GATE_DESC_REC << 8);
    GATE_REG(R_CTRL)    = C_NET_GO;                  // hold = 0
    if (wait_bits(S_NET_DONE, X_NET) != 0)
        return -1;

    // ---- D1：1024 → 64 ----
    GATE_REG(R_D1B)  = (uint32_t)(GATE_D1_IN_BASE & 0xFFFF)
                     | ((uint32_t)(GATE_D1_OUT_BASE & 0xFFFF) << 16);
    GATE_REG(R_D1SH) = GATE_D1_SH_REQ;
    GATE_REG(R_CLR) = X_D1;
    GATE_REG(R_CTRL) = C_D1_GO;
    if (wait_bits(S_D1_DONE, X_D1) != 0)
        return -1;

    // ---- 读 64 维（out_base=0 ⇒ 像素 0 = dim 0..31，像素 1 = dim 32..63）----
    rd_pixel((uint16_t)(GATE_D1_OUT_BASE >> 2), 8, &emb[0]);
    rd_pixel((uint16_t)((GATE_D1_OUT_BASE >> 2) + 1), 8, &emb[32]);

    // ---- Optional integer adapter and identity-aware gallery ----
#ifdef GATE_MULTI_ID
    {
        int nearest = -1;
        int8_t adjusted[64];
        g_feature_transform64((const int8_t *)emb, adjusted,
                              GATE_FEATURE_BIAS, GATE_FEATURE_W,
                              GATE_FEATURE_SHIFT);
        best_t = g_match_gallery64(&GATE_TPL[0][0], GATE_NT2, GATE_TT,
                                    GATE_TPL_IDS, GATE_TPL_N, adjusted,
                                    GATE_KBITS, GATE_MARGIN_Q15, &nearest);
        if (nearest >= 0) {
            uint32_t den;
            g_accept(GATE_TPL[nearest], adjusted, GATE_EMB,
                     GATE_TT[nearest], GATE_KBITS, &dot, &nq2);
            den = g_isqrt64((uint64_t)GATE_NT2[nearest] * (uint32_t)nq2);
            if (den && dot > 0)
                *cos1000 = (int32_t)(((uint64_t)dot * 1000u) / den);
        }
    }
#else
    // ---- 与模板逐个比 ----
    for (t = 0; t < GATE_TPL_N; t++) {
        int ok = g_accept(GATE_TPL[t], (const int8_t *)emb, GATE_EMB,
                          GATE_TT[t], GATE_KBITS, &dot, &nq2);
        // 余弦 ×1000：仅显示（cos = dot / sqrt(nt2·nq2)）
        if (nq2 > 0 && dot > 0) {
            uint32_t den = g_isqrt64((uint64_t)GATE_NT2[t] * (uint64_t)nq2);
            int32_t c = den ? (int32_t)(((uint64_t)dot * 1000u) / den) : 0;
            if (c > *cos1000)
                *cos1000 = c;
        }
        // 多模板通过时按 dot^2/nt2 排序；公共查询范数可约去。
        if (ok && (best_t < 0 ||
                   g_cos_better(dot, GATE_NT2[t],
                                best_dot, GATE_NT2[best_t]))) {
            best_dot = dot;
            best_t = t;
        }
    }
#endif
    return best_t;
}

//---------------------------------------------------------------------
// 一帧
//---------------------------------------------------------------------
int gate_run_frame(uint32_t frame)
{
    int win, best, n, label = -1;
    uint8_t r_win_out = 0;
    int32_t cos1000 = -1000;
    gate_result_t r;
    uint32_t t0 = gate_cycle();

    s_dec.step_c = GATE_STEP_C;
    s_dec.q      = GATE_FIXQ;
    s_dec.lut    = GATE_EXPLUT;

    // ---- 9 窗检测 ----
    s_ncand = 0;
    {
        int fr = (GATE_SWF > 0) ? (int)(frame % (uint32_t)GATE_SWF) : 0;
        for (win = 0; win < NWIN; win++)
            run_window_det(win, fr);
        s_fr = fr;
    }

    // ---- 跨窗 NMS → 取面积最大 ----
    n = g_nms(s_cand, s_ncand, IOU_MILLI);
    best = g_pick_best(s_cand, n);

    if (best >= 0) {
        // ★★ 把窗口**对准脸中心**再裁 ROI（cnn_roi 只能从当前窗的 FB 裁）。
        //   为什么不用"候选自己那个窗"或"最宽松的 3×3 窗"：
        //   3×3 网格的窗可能正好把这个脸切在边上，实测 32×35 被切成 32×32 后
        //   cos 从 0.538 崩到 0.103（识别器对长宽比极敏感）。
        //   窗口能任意定位之后，脸永远在窗口正中，1.2 倍外扩一定装得下。
        int cx = (s_cand[best].x1 + s_cand[best].x2) / 2;
        int cy = (s_cand[best].y1 + s_cand[best].y2) / 2;
        seek_at(cx - GATE_DET_W / 2, cy - GATE_DET_H / 2, s_fr);
        label = run_rec_and_gate(&s_cand[best], cx - GATE_DET_W / 2,
                                 cy - GATE_DET_H / 2, &cos1000);
        r_win_out = s_cand[best].win;
    }

    // ---- 门禁 ----
    r.frame   = frame;
    r.win     = r_win_out;
    r.x1      = (best >= 0) ? s_cand[best].x1 : -1;
    r.y1      = (best >= 0) ? s_cand[best].y1 : -1;
    r.x2      = (best >= 0) ? s_cand[best].x2 : -1;
    r.y2      = (best >= 0) ? s_cand[best].y2 : -1;
    r.d       = (best >= 0) ? s_cand[best].d : 0;
    r.cos1000 = cos1000;
    r.label   = (int8_t)label;
    r.open    = g_gate_step(&s_gate, label);
    r.cyc     = gate_cycle() - t0;
    r.ncand   = (uint8_t)(s_ncand > 255 ? 255 : s_ncand);
    r.nafter  = (uint8_t)(n > 255 ? 255 : n);
    gate_on_result(&r);

    // 下一帧开头会 seek_win(0, fr) ⇒ 这里不用再回窗 0（省 12,288 拍）
    return r.open;
}

void gate_init(void)
{
    g_gate_init(&s_gate, PASS_N, FAIL_N, HOLD_FRAMES);
    s_ncand = 0;
    s_fr = 0;
    GATE_REG(R_CTRL) = C_HOLD;      // 默认 hold（det 逐层读）
    GATE_REG(R_CLR)  = X_ALL;
    gate_on_init();
}

int gate_selftest(void)
{
    return (GATE_REG(R_ID) == GATE_ID_EXPECT) ? 0 : -1;
}
