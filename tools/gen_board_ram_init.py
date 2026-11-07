#!/usr/bin/env python3
# gen_board_ram_init.py : 生成 rtl/soc/simple_ram.v —— RAM 模块 + 内嵌固件初始化
#
# 背景: 远程平台 (1) 只收 verilog/ip/fdc 三类, 无法上传 .hex 数据文件;
#       (2) PDS 自动选层会把"未被实例化"的模块当顶层 (教训: 独立的
#           simple_ram_init 模块被选为 top, 约束全部报 object not found).
#       因此固件初始化直接内嵌进唯一的 simple_ram 模块, 由参数门控:
#         EMBED_INIT=0 (默认): 仿真行为不变 (tb 自己 $readmemh)
#         EMBED_INIT=1        : 板级综合, initial 逐字写 mem → BRAM init
#       (PDS 2022.2-SP6.4 已验证支持逐字 initial → BRAM 初始化推断)
#
# 用法: python benchmark/port/gen_board_ram_init.py [in.hex] [out.v]  (改固件后必须重跑!)
#   默认: benchmark/port/coremark.hex → rtl/soc/simple_ram.v (跑分包)
#   其它板包: python ... <你的固件.hex> <目标 simple_ram 变体.v> (模块名同为 simple_ram, 上传时替换)
import io, os, sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
HEX  = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "benchmark", "port", "coremark.hex")
OUT  = sys.argv[2] if len(sys.argv) > 2 else os.path.join(ROOT, "rtl", "soc", "simple_ram.v")

words = []
with io.open(HEX, encoding="utf-8") as f:
    for line in f:
        line = line.strip()
        if line and not line.startswith("//"):
            words.append(line)

ADDR_W = 17   # 128KB
DEPTH  = 1 << (ADDR_W - 2)
assert len(words) <= DEPTH, f"固件 {len(words)} 词超过 RAM {DEPTH} 词"

HDR = """//=====================================================================
// simple_ram : 真双端口 RAM —— **本文件由
//   benchmark/port/gen_board_ram_init.py 自动生成, 勿手改!**
//   (改模板去改生成脚本; 固件更新后重跑生成脚本)
//
// 双端口结构 (BRAM/DRM 推断友好, 取指与数据访问结构性零冲突):
//   I 端口: 只读, 同步 1 拍 (取指通道)
//   D 端口: 读+写(字节使能), 同步 1 拍 (数据通道)
// 固件 %d 词以 initial 逐字内嵌 (PDS 已验证支持 → BRAM init), EMBED_INIT=0
// 时不生效; 仿真 tb 可对 .mem 外部 $readmemh 覆盖.
//=====================================================================
`timescale 1ns/1ps

module simple_ram #(
    parameter ADDR_W     = 17,          // 128KB
    parameter INIT_FILE  = "",          // 仿真可选文件初始化
    parameter EMBED_INIT = 0            // 板级=1: 内嵌固件生效 (fpga_top 默认传 1)
)(
    input  wire        clk,
    // 取指通道 (只读, 同步)
    input  wire        i_valid,
    input  wire [31:0] i_addr,
    output reg  [31:0] i_rdata,
    // 数据通道 (读写, 同步, 字节使能)
    input  wire        d_valid,
    input  wire        d_write,
    input  wire [31:0] d_addr,
    input  wire [31:0] d_wdata,
    input  wire [ 3:0] d_wstrb,
    output reg  [31:0] d_rdata
);

    localparam DEPTH = 1 << (ADDR_W - 2);

    reg [31:0] mem [0:DEPTH-1];

    // D 写 (同步, 字节使能)
    integer i;
    always @(posedge clk) begin
        if (d_valid && d_write) begin
            for (i = 0; i < 4; i = i + 1) begin
                if (d_wstrb[i])
                    mem[d_addr[ADDR_W-1:2]][i*8 +: 8] <= d_wdata[i*8 +: 8];
            end
        end
    end

    // I 读 (同步)
    always @(posedge clk) begin
        if (i_valid)
            i_rdata <= mem[i_addr[ADDR_W-1:2]];
    end

    // D 读 (同步)
    always @(posedge clk) begin
        if (d_valid && !d_write)
            d_rdata <= mem[d_addr[ADDR_W-1:2]];
    end

    // 可选: 仿真从文件初始化
    initial begin
        if (INIT_FILE != "") begin
            $readmemh(INIT_FILE, mem);
        end
    end

    // 板级: 内嵌固件镜像 (未列出的字保持 0, 与 BRAM 默认初值一致)
    initial begin
        if (EMBED_INIT != 0) begin
""" % len(words)

with io.open(OUT, "w", encoding="utf-8", newline="\n") as f:
    f.write(HDR)
    for idx, w in enumerate(words):
        f.write("            mem[%d] = 32'h%s;\n" % (idx, w.rjust(8, "0")[-8:]))
    f.write("""        end
    end

endmodule
""")

print("[OK] %s: %d words embedded (EMBED_INIT gated)" % (OUT, len(words)))
