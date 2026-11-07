//=====================================================================
// cnn_pe_page.v : PE 页 —— ICG x OC_SLOT 个 pe_col + 权重寄存器组
//                 **M1：dslice 取数改为三算子可配置重排**
//
// 结构：ICG=4 个输入通道页 × OC_SLOT=8 个输出通道槽 = 32 条 TAPS 级 PE 链；
//       权重寄存器组 wreg = ICG*OC_SLOT*TAPS*WW = **2304 b = 288 B**
//       在每次 (og,ig) 迭代开始时整字装入（迭代内固定 = weight-stationary）。
//
// ★★★ 本文件是「三种算子共用一个阵列」的**唯一落点**（基线 §4.1）
//
//   阵列 = ICG(4) × OC_SLOT(8) × TAPS(9) = 288 MAC，9 个 tap 构成归约链。
//   三种算子**只差 lane(p,c) 的 dslice 从 dly 的哪个通道取数**：
//
//     | 模式 | dly[t][ch] 的语义        | lane (p,c) 取      | 通道号公式 |
//     |------|--------------------------|--------------------|-----------|
//     | 3×3  | 通道 ch 在**空间位置 t** | dly[t][p]          | ch = p        |
//     | dw   | 通道 ch 在**空间位置 t** | dly[t][8p+c]       | ch = 8*p + c  |
//     | 1×1  | 通道 ch 在**同一像素**   | dly[t][8p+t]       | ch = 8*p + t  |
//
//   **三种取数都是对同一条 32 通道 dly 总线的固定重排（wire permutation），
//     不是动态 mux** —— 综合成纯布线，不占 LUT/APM。
//   ⇒ 权重字宽（2304 b）与读口**完全不变**，这是整个架构最重要的简化。
//
//   ⚠ 1×1 只有 **8 个 tap 有效**（t=0..7，每 page 8 个输入通道），
//     tap 8 的 dslice 强制 0 ⇒ 权重置 0 即可（已证 `ceil(ic/32)==ceil(ic/36)`
//     对所有 ic 成立 ⇒ 用满 9 个 tap 零收益，见 `调研_脉动阵列适配性.md` §A3-I3）。
//   ⚠ 1×1 要求 `cnn_dman` 的 **skew_en=0**（tap 不是空间偏移）。
//
// wreg 字节布局（与 ROM 打包一致，**三种算子共用同一布局**，差别只在 ROM 里装什么）：
//   bit[((p*OC_SLOT+c)*TAPS + t)*WW +: WW]
//     3×3 装 W[og*8+c][ig*4+p][t] ｜ dw 装 W_dw[8p+c][t] ｜ 1×1 装 W[og*8+c][8p+t]
//=====================================================================
`timescale 1ns/1ps

module cnn_pe_page #(
    parameter ICG     = 4,       // 输入通道分组（页数）—— 3×3 每拍交付的通道数
    parameter OC_SLOT = 8,       // 输出通道槽位数
    parameter TAPS    = 9,       // 归约链长度
    parameter CH      = 32,      // dly 总线通道数（= ICG*OC_SLOT）
    parameter AW      = 8,
    parameter WW      = 8,
    parameter YW      = 32
)(
    input  wire                              clk,
    input  wire                              en,       // 流水线推进
    // 算子模式：0=3×3 conv ｜ 1=depthwise ｜ 2=1×1 pointwise ｜ 3=保留(=3×3)
    input  wire [1:0]                        op_mode,
    // 激活（已按 cnn_dman 偏斜/直通）：dly[(t*CH+ch)*AW +: AW]
    input  wire [TAPS*CH*AW-1:0]             dly,
    // 权重装载口：一次装满整个 wreg（来自 cnn_wrom 的 2304 b 字）
    input  wire                              wld_en,
    input  wire [ICG*OC_SLOT*TAPS*WW-1:0]    wld_data,
    // 页部分和：page_y[(p*OC_SLOT+c)*YW +: YW]（未加 bias、未跨 ig 累加）
    output wire [ICG*OC_SLOT*YW-1:0]         page_y
);

    localparam MODE_3X3 = 2'd0;
    localparam MODE_DW  = 2'd1;
    localparam MODE_PW  = 2'd2;

    //-----------------------------------------------------------------
    // 权重寄存器组（整字装载，迭代内保持）
    //-----------------------------------------------------------------
    reg [ICG*OC_SLOT*TAPS*WW-1:0] wreg;
    always @(posedge clk) begin
        if (wld_en)
            wreg <= wld_data;
    end

    //-----------------------------------------------------------------
    // ICG x OC_SLOT 条 PE 链
    //-----------------------------------------------------------------
    genvar p, c, t;
    generate
        for (p = 0; p < ICG; p = p + 1) begin : g_page
            for (c = 0; c < OC_SLOT; c = c + 1) begin : g_col
                wire signed [TAPS*AW-1:0] dslice;
                reg signed [TAPS*AW-1:0] dslice_q;
                always @(posedge clk) if(en) dslice_q <= dslice;
                wire signed [TAPS*WW-1:0] wslice;
                wire signed [YW-1:0]       ys;
                for (t = 0; t < TAPS; t = t + 1) begin : g_map
                    // ---- 三算子唯一的差别：本 lane 的第 t 个 tap 取哪个通道 ----
                    // 3×3 : 通道 p（每拍只用到 dly 的低 4 个通道 ⇒ 只读 1 个 ACT bank）
                    // dw  : 通道 8p+c（32 通道全用）
                    // 1×1 : 通道 8p+t（t<8；tap 8 无对应通道 ⇒ 给 0，权重也置 0）
                    wire [5:0] ch_33 = p;                       // 0..ICG-1
                    wire [5:0] ch_dw = (8*p) + c;               // 0..CH-1
                    wire [5:0] ch_pw = (8*p) + t;               // 0..CH-1（t<8 时有效）
                    wire       pw_ok = (t < 8);                 // 1×1 第 9 个 tap 无效
                    assign dslice[t*AW +: AW] =
                        (op_mode == MODE_3X3) ? dly[(t*CH + ch_33)*AW +: AW] :
                        (op_mode == MODE_DW ) ? dly[(t*CH + ch_dw)*AW +: AW] :
                        (pw_ok)               ? dly[(t*CH + ch_pw)*AW +: AW] :
                                                {AW{1'b0}};
                    // ---- 权重：三种算子布局相同，RTL 侧完全不区分 ----
                    assign wslice[t*WW +: WW] =
                        wreg[((p*OC_SLOT+c)*TAPS+t)*WW +: WW];
                end
                cnn_pe_col #(
                    .TAPS(TAPS), .AW(AW), .WW(WW), .YW(YW)
                ) u_col (
                    .clk  (clk),
                    .en   (en),
                    .d    (dslice_q),
                    .w    (wslice),
                    .y_out(ys)
                );
                assign page_y[(p*OC_SLOT+c)*YW +: YW] = ys;
            end
        end
    endgenerate

endmodule
