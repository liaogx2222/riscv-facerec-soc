//=====================================================================
// cnn_dwrom.v : **M7** dense 权重 ROM（D1：FC 1024→64）
//
// ★ 为什么独立一个 ROM（字宽不同）：
//   `net_spec.mf_layers()` 只覆盖 M0..M10（卷积部分），**不含 D1**。
//   只打包 M0..M10 时实测 rec 只有 122,400 B < 原始 163,392 B ⇒
//   少的正是 D1 的 65,536 个权重。**漏打包 = 上板识别输出全错**。
//   ⇒ D1 用 64 b/字的 dense 格式，单独一个 ROM，与 CIFAR 的
//     `dense1_wrom.mem` 同格式。
//
// 布局（每 (o, chunk) 一个 64b 字，`gen/gen_wrom_mem.py`）：
//   `word |= W[o, ck*8 + k] << (k*8)`，chunk = 8 个连续输入通道
//   ⇒ 字地址 = o * (ic/8) + ck
//
// 容量实测：D1 64×1024 × 8 chunk = 8192 字 × 64 b = 512 KB ≈ 14.6 DRM
//=====================================================================
`timescale 1ns/1ps

module cnn_dwrom #(
    parameter DEPTH     = 8192,         // 64b 字数
    parameter INIT_FILE = "rtl/accel/mem/cnn_dwrom.mem"
)(
    input  wire          clk,
    input  wire          rst_n,

    input  wire          rd_en,
    input  wire [12:0]   rd_addr,       // 64b 字地址 = o*(ic/8) + ck
    output reg  [63:0]   rd_data,       // rd_en 次拍有效
    output wire          rd_vld
);

    localparam AW = 13;                 // DEPTH=8192 → 13 位

    // ★ 扁平数组 + $readmemh 直接初始化（synplify 才认，勿回退）
    reg [63:0] mem [0:DEPTH-1];
    initial $readmemh(INIT_FILE, mem);

    always @(posedge clk) begin
        if (rd_en)
            rd_data <= mem[rd_addr[AW-1:0]];
    end

    reg rd_vld_r;
    always @(posedge clk) rd_vld_r <= rd_en;
    assign rd_vld = rd_vld_r;

endmodule
