// =====================================================================
// gate_api.h —— gate_main.c 与板级胶水之间的**唯一契约**
//
// 为什么单独一个头：`gate_result_t` 两边都要用。C 语言**不做跨文件类型检查**，
// 如果两边各写一份结构体，字段顺序一改就是静默错位（这是最典型的
// "编译通过、运行乱码"）。所以结构体只允许在这里定义一次。
// =====================================================================
#ifndef GATE_API_H
#define GATE_API_H

#include <stdint.h>

typedef struct {
    uint32_t frame;             // 帧号
    uint8_t  win;               // 候选所在窗（0..8）
    int16_t  x1, y1, x2, y2;    // 全局框（320×240 缓冲坐标）；-1 = 无候选
    int16_t  d;                 // face 判据（face_logit − bg_logit）
    int32_t  cos1000;           // 余弦 ×1000（仅显示，不参与判定）
    int8_t   label;             // 命中的模板下标；-1 = 未命中
    uint8_t  open;              // 1 = 开门
    uint32_t cyc;               // 本帧总拍数
    uint8_t  ncand;             // 9 窗候选总数（诊断）
    uint8_t  nafter;            // 跨窗 NMS 之后剩几个（诊断）
} gate_result_t;

// ---- 由 gate_main.c 提供 ----
void gate_init(void);
int  gate_run_frame(uint32_t frame);   // 返回 1 = 开门
int  gate_selftest(void);              // 0 = ID 寄存器正确

// ---- 由板级（gate_board.c）实现 ----
void     gate_on_result(const gate_result_t *r);
void     gate_on_init(void);
uint32_t gate_cycle(void);

#endif // GATE_API_H
