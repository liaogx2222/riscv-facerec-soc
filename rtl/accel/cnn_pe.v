//=====================================================================
// cnn_pe.v : 脉动阵列处理单元 —— 1 个寄存器 MAC（INT8 x INT8 -> 32b 累加）
//
// 位置：FaceRec_SoC 加速器阵列（ICG=4 页 x OC_SLOT=8 列 x TAPS=9 = 288 MAC）
//       psum 沿 tap 轴（9 个 PE）逐级脉动累加。
//
// 语义（与 hwnet.py 整数参考逐位一致）：
//   a_q <= a; y_out <= y_in + a_q*w          （a: 激活 int8；w: 权重 int8）
//
// ★ 来源：`CIFAR10_CNN_Accel/rtl/accel/cnn_pe.v`（已 108 MHz 验证）
//   FaceRec_SoC 沿用**同一个阵列形状**（基线 §1「引擎」：ICG(4)×OC_SLOT(8)×TAPS(9)=288 MAC），
//   所以本文件**逻辑不变**，只是换了工程。改它之前先想清楚：阵列形状是冻结项。
//
// ⚠ PDS 纪律（血泪，见 RISCV_CPU/AGENT.md §4.5/§4.6）
//   * `syn_hier="hard"` **必须加**：否则 Synplify 把 288 个结构相同的 PE 实例
//     合并成「一个实例 + 时间复用」—— 主工程实测 APM 从 432 塌到 56
//   * Verilog-2001：不用 logic/always_ff；拼接结果是无符号的，要当有符号用必须 $signed()
//=====================================================================
`timescale 1ns/1ps

module cnn_pe /* synthesis syn_hier="hard" */ #(
    parameter AW = 8,          // 激活位宽
    parameter WW = 8,          // 权重位宽
    parameter YW = 32          // psum 累加位宽
)(
    input  wire                clk,
    input  wire                en,        // 流水线推进（阵列级统一）
    input  wire signed [AW-1:0] a,
    input  wire signed [WW-1:0] w,
    input  wire signed [YW-1:0] y_in,     // 上游 PE 的 psum
    output reg  signed [YW-1:0] y_out     // 传下游
);

    //-----------------------------------------------------------------
    // 有符号 8x8 乘法（PDS 推断 18x18 APM 的点）
    //-----------------------------------------------------------------
    // Relocated from cnn_pe_page.dslice_q, with the same clock enable.
    // Do not preserve this FF in fabric: it should pack into APM X_REG.
    reg signed [AW-1:0] a_q;
    always @(posedge clk) if(en) a_q<=a;
    wire signed [AW+WW-1:0] prod;
    assign prod = a_q * w;

    always @(posedge clk) begin
        if (en)
            y_out <= y_in + prod;
    end

endmodule
