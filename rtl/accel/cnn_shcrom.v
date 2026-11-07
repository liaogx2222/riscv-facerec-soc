//=====================================================================
// cnn_shcrom.v : shc（BN 折叠常数偏置）ROM —— **256 b/字**
//
// 打包（`gen/build_desc.py` ⑤，与描述符 brom_base 同一累加顺序）：
//   conv/pw 层：一字装 **4 个 og 组**，slot = og&3；每组 8 字节 =
//               shc[og*8+g]（g=0..7，即本 og 组 8 个输出通道的偏置）
//   dw 层    ：一字 **1 组**，32 字节按通道直排：字节 c = shc[og*32+c]
//   超出 oc_raw 的通道补 0（权重也是 0，不会被消费）。
//
// 读时序（与 outacc 的 r3 对齐）：
//   outacc 用 **og_s1**（管线第 1 级的 og）同拍发地址 ⇒ q 在 og_s2 拍有效，
//   正是 r3 锁存（bnr = r2 + shc）那一拍。
//=====================================================================
`timescale 1ns/1ps

module cnn_shcrom #(
    parameter DEPTH     = 256,                            // 字数（256b/字）
    parameter AW        = 8,
    parameter INIT_FILE = "rtl/accel/mem/cnn_shcrom.mem"
)(
    input  wire          clk,
    input  wire          rst_n,
    input  wire          rd_en,
    input  wire [AW-1:0] rd_addr,
    output reg  [255:0]  rd_data
);

    reg [255:0] rom [0:DEPTH-1];
    initial begin
        $readmemh(INIT_FILE, rom);
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            rd_data <= 256'd0;
        end else if (rd_en) begin
            rd_data <= rom[rd_addr];
        end
    end

endmodule
