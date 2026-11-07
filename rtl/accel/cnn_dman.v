//=====================================================================
// cnn_dman.v : 数据偏斜管理器（d_manager）—— **M2：偏斜总线 288 b → 2304 b**
//
// 作用：把线缓冲送来的窗口 win[TAPS][CH] 偏斜成 dly[TAPS][CH]，
//   使 TAPS 个 PE 在同一拍处理**同一输出像素**的不同 tap（脉动阵列标准 skew）。
// 延迟量推导：pe_col 链延迟 TAPS 拍，D[k](T-TAPS+k) 需等于像素 q 的 tap k
//   （q 的窗口在 T-TAPS 拍送入）=> delay_k = k。
//
// ★ M2 相对 CIFAR 的改动（基线 §6 M2）：
//   CIFAR 的总线是 **9 tap × 4 通道 = 288 b**（ICG=4）；
//   FaceRec_SoC 的 1×1 与 dw 需要 **32 通道/拍** ⇒ 总线拓宽到
//   **9 tap × 32 通道 × 8 b = 2304 b**。
//   寄存器代价：Σt × CH × AW = 36 × 32 × 8 = **9,216 FF**（器件 319,600 ⇒ 2.9%，基线 §4.3）。
//
// ★ 新增 `skew_en`（1×1 必须关掉偏斜）：
//     3×3 / dw：tap = **空间偏移** ⇒ 要 skew（skew_en=1）
//     1×1     ：tap = **输入通道** ⇒ dly[t] 必须是**同一像素** ⇒ skew_en=0（直通）
//   依据基线 §4.1 的 dly 语义表：1×1 那行写的是「通道 ch 在**同一像素**」。
//
// 布局：win[(t*CH + ch)*AW +: AW]，t = tap(0..TAPS-1)，ch = 通道(0..CH-1)
//=====================================================================
`timescale 1ns/1ps

module cnn_dman #(
    parameter TAPS = 9,
    parameter CH   = 32,
    parameter AW   = 8
)(
    input  wire                     clk,
    input  wire                     en,
    input  wire                     skew_en,   // 1=按 tap 偏斜（3×3/dw）；0=直通（1×1）
    input  wire [TAPS*CH*AW-1:0]    win,
    output wire [TAPS*CH*AW-1:0]    dly
);

    genvar t;
    generate
        for (t = 0; t < TAPS; t = t + 1) begin : g_tap
            if (t == 0) begin : g_nodelay
                // tap 0：直连（0 拍延迟）
                assign dly[(0*CH)*AW +: CH*AW] = win[(0*CH)*AW +: CH*AW];
            end else begin : g_delay
                // tap t：延迟 t 拍（整字 CH*AW 位一起移，减少声明数量、便于 PDS）
                reg [CH*AW-1:0] sr [0:t-1];
                integer i;
                always @(posedge clk) begin
                    if (en && skew_en) begin
                        sr[0] <= win[(t*CH)*AW +: CH*AW];
                        for (i = 1; i < t; i = i + 1)
                            sr[i] <= sr[i-1];
                    end
                end
                // ★ skew_en=0 时**旁路**（不推进移位链），dly 直接等于 win
                assign dly[(t*CH)*AW +: CH*AW] =
                    skew_en ? sr[t-1] : win[(t*CH)*AW +: CH*AW];
            end
        end
    endgenerate

endmodule
