// =====================================================================
// gate_core.c —— 门禁编排的纯逻辑实现（与 Python golden 逐位一致）
// =====================================================================
#include "gate_core.h"

// ---------------------------------------------------------------------
// 框解码
//
// 对照 hwnet.decode_box：
//     cx = round(ax*A) + qx*STEP_C*aw
//     w  = (aw*A*lut[qw+128]) >> q
//     x1 = (cx - (w>>1)) >> q
//     x2 = (cx + (w>>1) + A - 1) >> q
//
// ★ 关键化简（本工程 FIXQ=20，A=1<<q）：
//       (aw*A*lut) >> q  ==  aw*lut        （精确相等，不是近似）
//   ⇒ 尺寸项不需要 64 位乘法。实测最大 aw*lut = 256*5,129,115 = 1.31e9，
//     仍在 int32 内（2.15e9），但 cx + (w>>1) + A 会到 ~1.0e9 —— 也安全。
//   若将来 q 改小或 oc 变大导致 aw*lut 溢出，这里必须换 64 位。
// ---------------------------------------------------------------------
void g_decode(const gdec_t *dec, int32_t ax_q, int32_t ay_q,
              int32_t aw, int32_t ah,
              int qx, int qy, int qw, int qh,
              int16_t *ox1, int16_t *oy1, int16_t *ox2, int16_t *oy2)
{
    const int32_t A = (int32_t)1 << dec->q;
    int32_t cx = ax_q + qx * dec->step_c * aw;
    int32_t cy = ay_q + qy * dec->step_c * ah;
    int32_t w  = aw * dec->lut[qw + 128];
    int32_t h  = ah * dec->lut[qh + 128];
    int32_t hw = w >> 1;
    int32_t hh = h >> 1;

    // Python 的 >> 对负数是 floor，C 的算术右移也是 floor ⇒ 一致
    *ox1 = (int16_t)((cx - hw) >> dec->q);
    *oy1 = (int16_t)((cy - hh) >> dec->q);
    *ox2 = (int16_t)((cx + hw + A - 1) >> dec->q);
    *oy2 = (int16_t)((cy + hh + A - 1) >> dec->q);
}

// ---------------------------------------------------------------------
// IoU ×1000
//   与 Python 的 `inter/union <= thr`（浮点）等价：
//   两边都是整数，所以 inter/union 是有理数；用
//       inter*1000 <= thr_milli*union
//   做精确整数比较，不会引入浮点舍入（thr=0.40 时 thr_milli=400）。
// ---------------------------------------------------------------------
int32_t g_iou_milli(int ax1, int ay1, int ax2, int ay2,
                    int bx1, int by1, int bx2, int by2)
{
    int iw = (ax2 < bx2 ? ax2 : bx2) - (ax1 > bx1 ? ax1 : bx1);
    int ih = (ay2 < by2 ? ay2 : by2) - (ay1 > by1 ? ay1 : by1);
    int32_t inter, uni;
    if (iw <= 0 || ih <= 0)
        return 0;
    inter = (int32_t)iw * ih;
    uni = (int32_t)(ax2 - ax1) * (ay2 - ay1)
        + (int32_t)(bx2 - bx1) * (by2 - by1) - inter;
    if (uni <= 0)
        return 0;
    return (int32_t)(((int64_t)inter * 1000) / uni);
}

// ---------------------------------------------------------------------
// 贪心 NMS（与 hwnet.nms 一致：按 d 降序，IoU <= thr 保留）
//   为了在 RV32 上避免浮点，比较用 g_iou_milli
// ---------------------------------------------------------------------
int g_nms(gbox_t *b, int n, int thr_milli)
{
    int i, j, m = 0;
    gbox_t t;
    // 插入排序（n <= 54，比调用 qsort 省栈、省代码）
    for (i = 1; i < n; i++) {
        t = b[i];
        for (j = i - 1; j >= 0 && b[j].d < t.d; j--)
            b[j + 1] = b[j];
        b[j + 1] = t;
    }
    for (i = 0; i < n; i++) {
        int keep = 1;
        for (j = 0; j < m; j++) {
            if (g_iou_milli(b[i].x1, b[i].y1, b[i].x2, b[i].y2,
                            b[j].x1, b[j].y1, b[j].x2, b[j].y2) > thr_milli) {
                keep = 0;
                break;
            }
        }
        if (keep)
            b[m++] = b[i];
    }
    return m;
}

// ---------------------------------------------------------------------
// 取判据 d 最大的候选（跨窗合并后的最终选择）
//
// ★ 不用"按面积"：实测跨 9 窗 + thr_d=0 时，贴窗边被裁成大方块的假框
//   （92×90）会把真脸（26×35）压掉，cos 从 0.54 掉到 0.16。
//   与 model/exp_slide_rom.py 的 golden 口径一致（max d）。
// ---------------------------------------------------------------------
int g_pick_best(const gbox_t *b, int n)
{
    int i, best = -1;
    int16_t best_d = -32768;
    for (i = 0; i < n; i++) {
        if (b[i].d > best_d) {
            best_d = b[i].d;
            best = i;
        }
    }
    return best;
}

// ---------------------------------------------------------------------
// 余弦判定（无除法、无开方）
//   dot²·2^kb >= T_t·nq2  —— 两项都在 1e14 量级 ⇒ 必须 64 位
//   （RV32 上 gcc 会调 __muldi3；每人一次，代价可忽略）
// ---------------------------------------------------------------------
int g_accept(const int8_t *t, const int8_t *q, int dim,
             uint32_t T_t, uint8_t kb, int32_t *dot_out, int32_t *nq2_out)
{
    int i;
    int32_t dot = 0, nq2 = 0;
    uint64_t lhs, rhs;
    for (i = 0; i < dim; i++) {
        int32_t ti = t[i], qi = q[i];
        dot += ti * qi;
        nq2 += qi * qi;
    }
    if (dot_out) *dot_out = dot;
    if (nq2_out) *nq2_out = nq2;
    if (dot <= 0)
        return 0;
    lhs = (uint64_t)((int64_t)dot * dot) << kb;
    rhs = (uint64_t)T_t * (uint64_t)nq2;
    return (lhs >= rhs) ? 1 : 0;
}

int g_cos_better(int32_t dot, uint32_t nt2,
                 int32_t best_dot, uint32_t best_nt2)
{
    uint64_t lhs, rhs;
    if (dot <= 0 || nt2 == 0)
        return 0;
    if (best_dot <= 0 || best_nt2 == 0)
        return 1;
    lhs = (uint64_t)((int64_t)dot * dot) * best_nt2;
    rhs = (uint64_t)((int64_t)best_dot * best_dot) * nt2;
    return lhs > rhs;
}

static int imin(int a, int b) { return a < b ? a : b; }

void g_feature_transform64(const int8_t *in, int8_t *out,
                            const int8_t *bias, const int8_t *matrix,
                            uint8_t shift)
{
    int16_t q[64];
    int i, j;
    for (i = 0; i < 64; i++) {
        int v = (int)in[i] - (bias ? bias[i] : 0);
        q[i] = (int16_t)(v < -128 ? -128 : (v > 127 ? 127 : v));
    }
    for (i = 0; i < 64; i++) {
        int32_t v = q[i];
        if (matrix) {
            v = 0;
            for (j = 0; j < 64; j++)
                v += (int32_t)q[j] * matrix[i * 64 + j];
            v >>= shift;
        }
        out[i] = (int8_t)(v < -128 ? -128 : (v > 127 ? 127 : v));
    }
}

int g_match_gallery64(const int8_t *templates, const uint32_t *norms,
                      const uint32_t *thresholds, const uint8_t *owners,
                      int ntemplates, const int8_t *query, uint8_t kbits,
                      uint16_t margin_q15, int *nearest_template)
{
    uint8_t passed[16] = {0};
    uint32_t nq2 = 0;
    int best = -1, second = -1, nearest = -1, i, j;
    int32_t bs = -32769, ss = -32769;
    if (nearest_template) *nearest_template = -1;
    if (ntemplates <= 0 || ntemplates > 127 || kbits > 8)
        return -1;
    for (j = 0; j < 64; j++)
        nq2 += (int32_t)query[j] * query[j];
    if (!nq2) return -1;
    for (i = 0; i < ntemplates; i++) {
        int id = owners ? owners[i] : i;
        int32_t dot = 0, score;
        uint32_t den;
        if (id > 127 || !norms[i]) continue;
        for (j = 0; j < 64; j++)
            dot += (int32_t)templates[i * 64 + j] * query[j];
        den = g_isqrt64((uint64_t)nq2 * norms[i]);
        score = den ? (int32_t)(((int64_t)dot * 32768) / den) : 0;
        if (score > 32768) score = 32768;
        if (score < -32768) score = -32768;
        if (dot > 0 && (((uint64_t)((int64_t)dot * dot) << kbits)
                        >= (uint64_t)thresholds[i] * nq2))
            passed[id >> 3] |= (uint8_t)(1u << (id & 7));
        if (id == best) {
            if (score > bs) { bs = score; nearest = i; }
        } else if (score > bs || (score == bs && id < best)) {
            second = best; ss = bs;
            best = id; bs = score; nearest = i;
        } else if (score > ss || (score == ss && id < second)) {
            second = id; ss = score;
        }
    }
    if (nearest_template) *nearest_template = nearest;
    if (best < 0 || !(passed[best >> 3] & (1u << (best & 7))))
        return -1;
    if (second >= 0 && bs - ss < margin_q15)
        return -1;
    return best;
}

// ---------------------------------------------------------------------
// 全局框 → 窗内 ROI（外扩；装不下就**等比缩小**，不硬裁）
// ---------------------------------------------------------------------
int g_roi_of(const gbox_t *b, int ox, int oy, int W, int H,
             int margin_milli, int min_side,
             int *rx1, int *ry1, int *rx2, int *ry2)
{
    int w = b->x2 - b->x1;
    int h = b->y2 - b->y1;
    int nw, nh, s, x1, y1, x2, y2;
    // 中心用"双像素"表示（cx2 = 2*cx），避免 (x1+x2)/2 把 .5 丢掉
    int cx2 = 2 * (b->x1 - ox) + w;
    int cy2 = 2 * (b->y1 - oy) + h;
    int maxw, maxh;

    if (w <= 0 || h <= 0)
        return 0;

    // 外扩后的宽高（四舍五入；等价于 float 的 round(w*margin)）
    nw = (w * margin_milli + 500) / 1000;
    nh = (h * margin_milli + 500) / 1000;

    // 以中心为基准，两边最多能给多宽（双像素单位）
    maxw = imin(cx2, 2 * W - cx2);
    maxh = imin(cy2, 2 * H - cy2);
    if (maxw <= 0 || maxh <= 0)
        return 0;

    // 装不下 ⇒ **等比缩小**（不硬裁，保住长宽比）
    s = 1000;
    if (nw > maxw)
        s = imin(s, maxw * 1000 / nw);
    if (nh > maxh)
        s = imin(s, maxh * 1000 / nh);
    nw = nw * s / 1000;
    nh = nh * s / 1000;

    // 半像素精度还原边界：x1 = floor((cx2-nw)/2)、x2 = ceil((cx2+nw)/2)
    x1 = (cx2 - nw) >> 1;
    x2 = (cx2 + nw + 1) >> 1;
    y1 = (cy2 - nh) >> 1;
    y2 = (cy2 + nh + 1) >> 1;
    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;
    if (x2 > W) x2 = W;
    if (y2 > H) y2 = H;
    if (x2 - x1 < min_side || y2 - y1 < min_side)
        return 0;
    *rx1 = x1; *ry1 = y1; *rx2 = x2; *ry2 = y2;
    return 1;
}

// ---------------------------------------------------------------------
// 64 位整数开方（逐位法，32 轮；只在**显示**余弦时用，不参与判定）
// ---------------------------------------------------------------------
uint32_t g_isqrt64(uint64_t v)
{
    uint64_t rem = 0, root = 0;
    int i;
    for (i = 31; i >= 0; i--) {
        root <<= 1;
        rem = (rem << 2) | ((v >> (i * 2)) & 3u);
        if (rem > root) {
            rem -= root + 1;
            root += 2;
        }
    }
    return (uint32_t)(root >> 1);
}

// ---------------------------------------------------------------------
// 门禁状态机（见 gate_core.h 的说明）
// ---------------------------------------------------------------------
void g_gate_init(ggate_t *g, uint8_t pass_n, uint8_t fail_n, uint8_t hold_n)
{
    g->open = 0; g->cnt = 0; g->fail = 0; g->hold = 0;
    g->pass_n = pass_n ? pass_n : 1;
    g->fail_n = fail_n ? fail_n : 1;
    g->hold_n = hold_n;
}

uint8_t g_gate_step(ggate_t *g, int label)
{
    if (label >= 0) {
        if (g->cnt < 255) g->cnt++;
        g->fail = 0;
    } else {
        g->cnt = 0;
        if (g->fail < 255) g->fail++;
    }
    if (g->fail >= g->fail_n) {
        g->open = 0;
        g->hold = 0;
    } else if (g->cnt >= g->pass_n) {
        g->open = 1;
        g->hold = g->hold_n;
    } else if (g->open) {
        if (g->hold)
            g->hold--;
        else
            g->open = 0;
    }
    return g->open;
}
