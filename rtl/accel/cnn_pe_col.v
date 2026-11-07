//=====================================================================
// cnn_pe_col.v : 一个 lane（输出通道槽位）的 **TAPS 级 PE 链**
//
// 结构（参考 CNN_FPGA 的 pe_col）：TAPS 个 cnn_pe 串联，psum 逐级累加。
//   y_out(T) = sum_{k=0..TAPS-1} D[k](T-TAPS+k) * W[k]
// 即：配 cnn_dman 的「tap k 延迟 k 拍」偏斜后，
//   T 拍输出 = T-TAPS 拍送入窗口的 TAPS 个 tap 的点积（整链延迟 TAPS 拍）。
//
// ★ **9 个 tap 是「归约链」** —— 这是本设计最核心的一件事（基线 §4.1）：
//   链的 9 个 tap 必须能被加起来，所以「哪个维度当 tap」是架构选择：
//     3×3  → tap = 空间偏移 9 个
//     dw   → tap = 空间偏移 9 个（通道由 dslice 选）
//     1×1  → tap = **输入通道** 9 个（每 page 8 个有效，tap 8 权重置 0）
//   三种语义都只改 `cnn_pe_page` 里的 dslice 重排（wire permutation），
//   **本模块完全不变** —— 这就是「三种算子共用一个阵列」的落点。
//
// 权重在本迭代 (og,ig) 内固定（由 cnn_pe_page 的 wreg 保持），
// 激活每拍流过新像素 —— 经典 weight-stationary 脉动。
//=====================================================================
`timescale 1ns/1ps

module cnn_pe_col #(
    parameter TAPS = 9,
    parameter AW   = 8,
    parameter WW   = 8,
    parameter YW   = 32
)(
    input  wire                            clk,
    input  wire                            en,
    // 1 维激活输入（tap k 在 [k*AW +: AW]，已按 dman 偏斜）
    input  wire signed [TAPS*AW-1:0]       d,
    // 1 维权重输入（tap k 在 [k*WW +: WW]）
    input  wire signed [TAPS*WW-1:0]       w,
    // 链式输出（整链 TAPS 个 tap 的点积）
    output wire signed [YW-1:0]            y_out
);

    // 链式 psum：yc[0]=0 -> ... -> yc[TAPS]=y_out（展平为向量，工具最安全）
    wire signed [(TAPS+1)*YW-1:0] yc;
    assign yc[YW-1:0] = {YW{1'b0}};

    genvar t;
    generate
        for (t = 0; t < TAPS; t = t + 1) begin : g_pe
            cnn_pe #(
                .AW(AW), .WW(WW), .YW(YW)
            ) u_pe (
                .clk   (clk),
                .en    (en),
                .a     (d[t*AW +: AW]),
                .w     (w[t*WW +: WW]),
                .y_in  (yc[t*YW +: YW]),
                .y_out (yc[(t+1)*YW +: YW])
            );
        end
    endgenerate

    assign y_out = yc[TAPS*YW +: YW];

endmodule
