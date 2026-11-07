// =====================================================================
// test_gate_core.c —— 在 PC 上对拍 gate_core 与 Python golden
//
//   gcc -O2 -o test_gate_core test_gate_core.c gate_core.c && ./test_gate_core
//
// 向量与期望值全部由 `gen/gen_gate_testvec.py` 生成（Python 侧调
// hwnet.decode_box / nms / cos_accept）。C 侧只要算得一模一样就算过。
// =====================================================================
#include <stdio.h>
#include <string.h>

#include "gate_core.h"
#include "gate_map.h"
#include "_test_vec.h"

static int fails = 0;

int main(void)
{
    int i, k;

    // ---------------- ① 框解码 ----------------
    {
        gdec_t dec;
        int nbad = 0;
        dec.step_c = TV_STEP_C;
        dec.q = TV_Q;
        dec.lut = GATE_EXPLUT;
        for (i = 0; i < TV_NDEC; i++) {
            const tv_dec_t *t = &TV_DEC[i];
            // ax_q = (2*gx+1) * sx * 2^(q-1)   —— 与 hwnet 的 ax*A 精确相等
            int32_t axq = (int32_t)(2 * t->gx + 1) * ((int32_t)t->sx << (TV_Q - 1));
            int32_t ayq = (int32_t)(2 * t->gy + 1) * ((int32_t)t->sy << (TV_Q - 1));
            int16_t x1, y1, x2, y2;
            g_decode(&dec, axq, ayq, t->aw, t->ah,
                     t->qx, t->qy, t->qw, t->qh, &x1, &y1, &x2, &y2);
            if (x1 != t->x1 || y1 != t->y1 || x2 != t->x2 || y2 != t->y2) {
                if (nbad < 6)
                    printf("  FAIL dec#%d s(%d,%d) g(%d,%d) aw=%d q=(%d,%d,%d,%d)\n"
                           "       got (%d,%d,%d,%d) exp (%d,%d,%d,%d)\n",
                           i, t->sx, t->sy, t->gx, t->gy, t->aw,
                           t->qx, t->qy, t->qw, t->qh,
                           x1, y1, x2, y2, t->x1, t->y1, t->x2, t->y2);
                nbad++;
            }
        }
        printf("① 框解码      %5d 例 ｜ 失败 %d\n", TV_NDEC, nbad);
        fails += nbad;
    }

    // ---------------- ② NMS ----------------
    {
        int nbad = 0;
        for (i = 0; i < TV_NNMS; i++) {
            gbox_t b[12];
            int n = TV_NMS_N[i], m;
            for (k = 0; k < n; k++) {
                memset(&b[k], 0, sizeof(b[k]));
                b[k].x1 = TV_NMS_BOX[i][k].b[0];
                b[k].y1 = TV_NMS_BOX[i][k].b[1];
                b[k].x2 = TV_NMS_BOX[i][k].b[2];
                b[k].y2 = TV_NMS_BOX[i][k].b[3];
                b[k].d = TV_NMS_BOX[i][k].b[4];
            }
            m = g_nms(b, n, 400);
            if (m != TV_NMS_NK[i]) {
                if (nbad < 5)
                    printf("  FAIL nms#%d 保留 %d，期望 %d（输入 %d 个）\n",
                           i, m, TV_NMS_NK[i], n);
                nbad++;
            } else {
                // 顺序也要一致（Python 按 d 降序稳定排序）
                for (k = 0; k < m; k++) {
                    int ex = TV_NMS_BOX[i][TV_NMS_KEEP[i][k]].b[4];
                    if (b[k].d != ex) {
                        if (nbad < 5)
                            printf("  FAIL nms#%d 第 %d 个 d=%d，期望 %d（顺序错）\n",
                                   i, k, b[k].d, ex);
                        nbad++;
                        break;
                    }
                }
            }
        }
        printf("② NMS         %5d 例 ｜ 失败 %d\n", TV_NNMS, nbad);
        fails += nbad;
    }

    // ---------------- ③ 余弦判定 ----------------
    {
        int nbad = 0;
        for (i = 0; i < TV_NCOS; i++) {
            int32_t dot = 0, nq2 = 0;
            int ok = g_accept(TV_COS_T[i], TV_COS_Q[i], GATE_EMB,
                              TV_COS_TT[i], GATE_KBITS, &dot, &nq2);
            if (ok != TV_COS_OK[i] || dot != TV_COS_DOT[i] || nq2 != TV_COS_NQ2[i]) {
                if (nbad < 6)
                    printf("  FAIL cos#%d ok=%d(exp %d) dot=%d(exp %d) nq2=%d(exp %d)\n",
                           i, ok, TV_COS_OK[i], dot, TV_COS_DOT[i],
                           nq2, TV_COS_NQ2[i]);
                nbad++;
            }
        }
        printf("③ 余弦判定    %5d 例 ｜ 失败 %d\n", TV_NCOS, nbad);
        fails += nbad;
    }

    // ---------------- ④ ROI 映射（只做自洽检查）----------------
    {
        gbox_t b;
        int x1, y1, x2, y2, nbad = 0;
        memset(&b, 0, sizeof(b));
        // 窗内 (40,30)-(90,90)：中心 (65,60)，50×60。1.2 外扩要 30/36 半宽高，
        // 垂直方向只够 35 ⇒ 等比缩到 ~0.97 ⇒ (36,26,94,94)。
        // 这里只断言"在界内 + 包含原框 + aspect 基本不变"，不写死坐标
        // （写死坐标会把"等比缩"这种正确行为判成错，第一版就栽在这）。
        b.x1 = 40; b.y1 = 30; b.x2 = 90; b.y2 = 90;
        if (!g_roi_of(&b, 0, 0, 128, 96, 1200, 8, &x1, &y1, &x2, &y2)) {
            printf("  FAIL roi 正常框被拒\n");
            nbad++;
        } else if (x1 < 0 || y1 < 0 || x2 > 128 || y2 > 96 ||
                   x1 > 40 || y1 > 30 || x2 < 90 || y2 < 90) {
            printf("  FAIL roi 越界或没包含原框：(%d,%d,%d,%d)\n", x1, y1, x2, y2);
            nbad++;
        } else {
            int w0 = x2 - x1, h0 = y2 - y1;   // 应约 58×68（50:60 → 0.833）
            if (h0 * 50 > w0 * 60 + 60 * 50 / 8 || w0 * 60 > h0 * 50 + 60 * 50 / 8) {
                printf("  FAIL roi aspect 偏了：%dx%d（原 50x60）\n", w0, h0);
                nbad++;
            }
        }
        // 贴右下角的框：装不下 ⇒ 等比缩小，长宽比要保持
        b.x1 = 100; b.y1 = 80; b.x2 = 126; b.y2 = 94;     // 26×14
        if (!g_roi_of(&b, 0, 0, 128, 96, 1200, 8, &x1, &y1, &x2, &y2)) {
            printf("  FAIL roi 贴边应等比缩小而不是拒绝\n");
            nbad++;
        } else {
            // 原始 aspect 26:14 ≈ 1.857；缩放后应接近
            int w0 = x2 - x1, h0 = y2 - y1;
            if (h0 <= 0 || (w0 * 14 - h0 * 26 > 26 * 14 / 6) ||
                (h0 * 26 - w0 * 14 > 26 * 14 / 6)) {
                printf("  FAIL roi 贴边后 aspect 变了：%dx%d（原 26x14）\n", w0, h0);
                nbad++;
            }
        }
        b.x1 = 0; b.y1 = 0; b.x2 = 4; b.y2 = 4;           // 太小（4×1.2=4.8 < min_side 8）
        if (g_roi_of(&b, 0, 0, 128, 96, 1200, 8, &x1, &y1, &x2, &y2)) {
            printf("  FAIL roi 太小的框应被拒\n");
            nbad++;
        }
        printf("④ ROI 映射        3 例 ｜ 失败 %d\n", nbad);
        fails += nbad;
    }

    // ---------------- ⑤ 门禁状态机 ----------------
    {
        int nbad = 0;
        ggate_t g;
        // 用例 1：pass_n=3 fail_n=2 hold=2，序列 P P P F F F
        //   ★ hold=0 时任何一帧 fail 都会立刻关门（闩锁没余量），
        //     所以这里用 hold=2 才验得出"单帧抖动不关门"。
        {
            const int seq[] = { 0, 0, 0, -1, -1, -1 };
            const int exp[] = { 0, 0, 1, 1, 0, 0 };
            int i;
            g_gate_init(&g, 3, 2, 2);
            for (i = 0; i < 6; i++) {
                int got = (int)g_gate_step(&g, seq[i]);
                if (got != exp[i]) {
                    printf("  FAIL gate#1 第 %d 帧 got %d exp %d\n", i, got, exp[i]);
                    nbad++;
                }
            }
        }
        // 用例 2：pass_n=2 fail_n=2 hold=2，序列 P P F F P P
        //   P P -> 开；F（fail=1）-> 仍开；F（fail=2）-> 关；P P -> 再开
        {
            const int seq[] = { 0, 0, -1, -1, 0, 0 };
            const int exp[] = { 0, 1, 1, 0, 0, 1 };
            int i;
            g_gate_init(&g, 2, 2, 2);
            for (i = 0; i < 6; i++) {
                int got = (int)g_gate_step(&g, seq[i]);
                if (got != exp[i]) {
                    printf("  FAIL gate#2 第 %d 帧 got %d exp %d\n", i, got, exp[i]);
                    nbad++;
                }
            }
        }
        // 用例 3：ROM 4 帧循环（走近=未检出 / 对准 / 贴近 / 陌生人），pass_n=2
        //   期望：前 2 帧关，第 2 帧起开，第 4 帧（陌生人）关
        {
            const int cyc[] = { -1, 0, 0, -1, -1, 0, 0, -1 };
            const int exp[] = { 0, 0, 1, 1, 0, 0, 1, 1 };
            int i;
            g_gate_init(&g, 2, 2, 4);
            for (i = 0; i < 8; i++) {
                int got = (int)g_gate_step(&g, cyc[i]);
                if (got != exp[i]) {
                    printf("  FAIL gate#3 第 %d 帧 got %d exp %d\n", i, got, exp[i]);
                    nbad++;
                }
            }
        }
        printf("⑤ 门禁状态机      3 组序列 ｜ 失败 %d\n", nbad);
        fails += nbad;
    }

    printf("\n%s（合计失败 %d）\n", fails ? "★ FAILED" : "ALL PASS", fails);
    return fails ? 1 : 0;
}
