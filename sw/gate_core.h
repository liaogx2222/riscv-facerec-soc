// =====================================================================
// gate_core.h —— 门禁编排的**纯逻辑**部分（无 MMIO、无寄存器）
//
// 为什么单独拆出来：这些函数必须与 Python golden 逐位一致
//   （`model/hwnet.py` 的 decode_box / nms / cos_accept）。
//   拆成纯函数之后可以在 PC 上用 gcc 直接跑对拍（`sw/test_gate_core.c`），
//   不用碰 RTL、不用等仿真 —— 这是"先在 PC 上验语义，再上板"的分层。
//
// ★ 全部整数运算，无浮点（RV32I 没有 FPU）：
//     · 框解码靠 Q20 定点 + EXP_LUT
//     · IoU 用**千分比整数**比较（inter*1000 <= thr*union），与浮点等价
//     · 余弦判定用 64 位乘（dot*dot*2^8 约 2.7e14，32 位一定溢出）
// =====================================================================
#ifndef GATE_CORE_H
#define GATE_CORE_H

#include <stdint.h>

#define GATE_MAX_WIN     9      // 滑窗数（3×3）
#define GATE_MAX_PER_WIN 6      // 每窗最多留几个候选
#define GATE_MAX_CAND    (GATE_MAX_WIN * GATE_MAX_PER_WIN)

// 一个候选框。**同时**保留全局坐标（跨窗 NMS 用）和窗内坐标（ROI 用）。
typedef struct {
    int16_t x1, y1, x2, y2;      // 全局坐标（320×240 缓冲）
    int16_t lx1, ly1, lx2, ly2;  // 窗内坐标（0..127 / 0..95）
    int16_t d;                   // face_logit − bg_logit（判据）
    uint8_t win;                 // 来自哪个窗
} gbox_t;

// 解码器上下文
typedef struct {
    int32_t        step_c;       // STEP_C（Q_q 定点中心步长）
    int32_t        q;            // FIXQ
    const int32_t *lut;          // 256 项 EXP_LUT
} gdec_t;

// ---------------------------------------------------------------------
// 单 anchor 解码 —— 与 hwnet.decode_box 逐位一致
//   ax_q/ay_q = ax * 2^q（调用方按 (2*gx+1)*sx*2^(q-1) 预先算好）
//   qx..qh    = head 输出的 4 个回归量（int8，已符号扩展）
// 返回 int16 像素框
// ---------------------------------------------------------------------
void g_decode(const gdec_t *dec, int32_t ax_q, int32_t ay_q,
              int32_t aw, int32_t ah,
              int qx, int qy, int qw, int qh,
              int16_t *ox1, int16_t *oy1, int16_t *ox2, int16_t *oy2);

// IoU ×1000（整数；两框都假定 x2>x1、y2>y1）
int32_t g_iou_milli(int ax1, int ay1, int ax2, int ay2,
                    int bx1, int by1, int bx2, int by2);

// 贪心 NMS：按 d 降序，IoU <= thr_milli 才保留（与 hwnet.nms 的 `<=` 一致）
// 返回保留个数；原地压缩到 b[0..n-1]
int g_nms(gbox_t *b, int n, int thr_milli);

// 取**判据 d 最大**的那个，返回下标（n<=0 时返回 -1）
//
// ★ 为什么不是"按面积取最大"：单窗的 det_decode 末尾确实按面积排序
//   （"优先给大脸机会"），但那是**一个窗内、已经过 NMS** 的情形。
//   跨 9 个窗、thr_d=0 时，会出现"贴在窗边被裁成大方块"的假框
//   （实测 92×90 的假框把 26×35 的真脸压掉了，cos 从 0.54 掉到 0.16）。
//   ⇒ 多窗合并必须按**置信度**挑，与 model/exp_slide_rom.py 的 golden 一致。
int g_pick_best(const gbox_t *b, int n);

// 余弦判定（与 hwnet.cos_accept 同式）：
//   dot = Σ t·q ｜ nq2 = Σ q² ｜ 开门 ⇔ dot>0 且 dot²·2^kb >= T_t·nq2
// 返回 1 = 通过；dot/nq2 通过指针回传（可能为 NULL）
int g_accept(const int8_t *t, const int8_t *q, int dim,
             uint32_t T_t, uint8_t kb, int32_t *dot_out, int32_t *nq2_out);

// Compare two positive cosine scores sharing the same query vector.
// dot^2/nt2 preserves exact ranking without sqrt/division. dim <= 64,
// signed INT8 inputs keep the uint64 cross-products within range.
int g_cos_better(int32_t dot, uint32_t nt2,
                 int32_t best_dot, uint32_t best_nt2);

// Optional 64-D integer adapter and identity-aware multi-template gallery.
// matrix==NULL applies only bias subtraction. Output is saturated INT8.
void g_feature_transform64(const int8_t *in, int8_t *out,
                            const int8_t *bias, const int8_t *matrix,
                            uint8_t shift);
int g_match_gallery64(const int8_t *templates, const uint32_t *norms,
                      const uint32_t *thresholds, const uint8_t *owners,
                      int ntemplates, const int8_t *query, uint8_t kbits,
                      uint16_t margin_q15, int *nearest_template);

// 全局框 → 窗内 ROI（框坐标是全局的，ox/oy 是选定窗的原点）
//
// ★★ 关键：外扩后若装不进窗，**等比缩小**而不是硬裁。
//   硬裁会破坏长宽比（32×35 → 32×32），而识别器对 aspect 极敏感：
//   实测同一张脸 32×35 cos=0.538、32×34=0.382、32×33=0.261、32×32=0.103。
//   等比缩到 29×32 是 0.358（虽然还是掉，但比 0.103 好 3.5 倍）。
//   根治办法是让 ROI 从 320×240 缓冲裁（见 doc 的"已知限制"）。
// 返回 0 = ROI 太小（< min_side），1 = 可用
int g_roi_of(const gbox_t *b, int ox, int oy, int W, int H,
             int margin_milli, int min_side,
             int *rx1, int *ry1, int *rx2, int *ry2);

// 整数开方（仅用于**显示**余弦值，不参与判定）
uint32_t g_isqrt64(uint64_t v);

// ---------------------------------------------------------------------
// 门禁状态机（序列级判决）
//
// 为什么要"序列级"：单帧不可靠 —— ChokePoint 实测本人段有 cos 0.144 的帧、
// 陌生人段有 0.477 的帧，两者重叠。所以判决必须看连续几帧。
// 规则（与 model/video_demo.py 同一套）：
//   连续 pass_n 帧"认出" ⇒ 开门（闩锁，hold 帧内不因单帧抖动关门）
//   连续 fail_n 帧"没认出" ⇒ 关门
// ---------------------------------------------------------------------
typedef struct {
    uint8_t open;      // 当前门状态
    uint8_t cnt;       // 连续认出帧数
    uint8_t fail;      // 连续未认出帧数
    uint8_t hold;      // 开门后剩余闩锁帧
    uint8_t pass_n;    // 连续几帧认出才开门
    uint8_t fail_n;    // 连续几帧没认出才关门
    uint8_t hold_n;    // 开门后保持多少帧
} ggate_t;

void g_gate_init(ggate_t *g, uint8_t pass_n, uint8_t fail_n, uint8_t hold_n);
// label >= 0 表示认出了（命中模板下标）。返回 1 = 开门
uint8_t g_gate_step(ggate_t *g, int label);

#endif // GATE_CORE_H
