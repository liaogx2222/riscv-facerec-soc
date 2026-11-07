//=====================================================================
// cnn_wrom.v : **M7** 统一权重 ROM（三算子共用同一 2304 b 字宽）
//
// ⚠⚠ 两条结构纪律（CIFAR 两次教训，勿回退）：
//   1. 必须**扁平数组 + `initial $readmemh(FILE, mem)` 直接初始化**：
//      「init_tmp 中转 + for 切片分发到 N bank」iverilog 能跑、synplify 不识别
//      ⇒ 网表 INIT 全 0 ⇒ 上板结果全错（MNIST 实测 class=9，本应 3）。
//   2. 必须只有 **1 个读口**：多读口 ⇒ mem 变 2 读 1 写 ⇒ 超 DRM36K 双口能力
//      ⇒ 综合器复制两份 mem ⇒ BRAM 溢出 ⇒ Place-0114 放不下。
//
// ★ 三种算子**共用同一个字宽、同一个读口**：`cnn_pe_page` 的 `wld_data`
//   就是本模块的 `rd_data`（2304 b）。差别只在 ROM 里装什么，由
//   `gen/gen_wrom_mem.py` 打包（见其 docstring 的三算字段表）。
//
// 布局（与打包脚本逐位对应，lane L = p*OC_SLOT + c）：
//   bit[((L*TAPS) + t)*WW +: WW]
//     conv 3×3 = W[og*8+c][ig*4+p][t//3][t%3]
//     dw       = W[og*32 + (8p+c)][t//3][t%3]      ← og 是 **32 通道组**
//     pw 1×1   = W[og*8+c][8p+t][0][0]             ← t<8，tap 8 权重置 0
// 一个字装 288 个权重字节（32 lane × 9 tap）。
//
// ⚠ 权重 ROM **不需要字节序反转**：`'%0576X'` 天然 MSB 优先，正好等于
//   Verilog 的位序。字节序反转只出现在**激活字**（RTL 的「字节 s = 通道 s」是 LSB 优先）。
//
// 容量（2026-10-16 实测，`gen/gen_wrom_mem.py`）：
//   det 1026 字 + rec 425 字 = **1451 字** × 2304 b = 3.34 Mb ≈ 92.9 DRM
//   （另 D1 用独立的 `cnn_dwrom.v`：8192 字 × 64 b ≈ 14.6 DRM）
//=====================================================================
`timescale 1ns/1ps

module cnn_wrom #(
    parameter DEPTH     = 1451,         // 2304b 字数
    parameter INIT_FILE = "rtl/accel/mem/cnn_wrom.mem"
)(
    input  wire          clk,
    input  wire          rst_n,

    input  wire          rd_en,
    input  wire [10:0]   rd_addr,       // 2304b 字地址（weight_base + 偏移）
    output reg  [2303:0] rd_data,       // rd_en 次拍有效
    output wire          rd_vld
);

    localparam AW = 11;                 // DEPTH=1451 → 11 位（2048 ≥ 1451）

    reg [2303:0] mem [0:DEPTH-1];
    initial $readmemh(INIT_FILE, mem);

    // ★ 2026-10-23（run17 结论）：不能在**本模块内部**寄存地址。
    //   DRAM36K 自带地址寄存器，Synplify 会把多余的一级**吸收掉**
    //   （run16 加了也没用，路径仍是 APM → ADA0 直连）。
    //   正确做法：在**调用方 cnn_engine** 里寄存（见那里的 waddr_d1）。
    always @(posedge clk) begin
        if (rd_en)
            rd_data <= mem[rd_addr[AW-1:0]];
    end

    reg rd_vld_r;
    always @(posedge clk) rd_vld_r <= rd_en;
    assign rd_vld = rd_vld_r;

endmodule
