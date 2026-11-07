//=====================================================================
// regfile.v : RV32I 寄存器堆 (★ D2b: 4 读 2 写)
//
// 规格书第 7 节: 异步读、同步写、写优先
//   - 写优先: 同一拍读与写同地址时, 读返回新写入数据 (forwarding to same cycle)
//   - x0 恒为 0, 写入 x0 被忽略
//   - 复位清零 (可选, 便于仿真)
//
// ★★ D2b (2026-10-08): 2 读 1 写 → **4 读 2 写** (顺序双发射)
//   · 读口 1/2 = slot0 的 rs1/rs2;  读口 3/4 = slot1 的 rs1/rs2
//   · 写口 0 = WB slot0 (可含 load/jal 回写值); 写口 1 = WB slot1 (纯 ALU)
//
//   实现: **两份阵列** (mem / memb), 各带 2 个异步读口; 两个写口**都写两份**。
//   理由: PDS 对 "4 读口 + 2 写口" 的单阵列会退化成 FF/复制, 而"2 份 2R2W"
//   是它熟悉的 LUTRAM 形状 (32×32 很小, 复制代价可忽略)。
//   两份阵列内容恒等 ⇒ 任意读口取哪一份都对。
//
//   ⚠ 写冲突: 两个写口同拍写**同一寄存器**时, **slot1 (写口 1) 优先** ——
//     程序序上 slot1 在后。靠"后一条非阻塞赋值覆盖前一条"实现 (Verilog 语义)。
//     读穿透旁路同理: 先查写口 1, 再查写口 0。
//=====================================================================
`timescale 1ns/1ps

module regfile (
    input  wire        clk,
    input  wire        resetn,
    // 写端口 0 (WB slot0)
    input  wire        we,
    input  wire [ 4:0] waddr,
    input  wire [31:0] wdata,
    // 读端口 1/2 (ID slot0, 异步读)
    input  wire [ 4:0] raddr1,
    input  wire [ 4:0] raddr2,
    output wire [31:0] rdata1,
    output wire [31:0] rdata2,
    // ★ D2b 写端口 1 (WB slot1)
    input  wire        we1,
    input  wire [ 4:0] waddr1,
    input  wire [31:0] wdata1,
    // ★ D2b 读端口 3/4 (ID slot1, 异步读)
    input  wire [ 4:0] raddr3,
    input  wire [ 4:0] raddr4,
    output wire [31:0] rdata3,
    output wire [31:0] rdata4
);

    reg [31:0] mem  [0:31];   // 阵列 A: 服务读口 1/2
    reg [31:0] memb [0:31];   // 阵列 B: 服务读口 3/4 (内容与 A 恒等)

    // 复位清零 + 双写口 (写口 1 后写 ⇒ 同址时 slot1 优先)
    integer i;
    always @(posedge clk) begin
        if (!resetn) begin
            for (i = 0; i < 32; i = i + 1) begin
                mem[i]  <= 32'h0;
                memb[i] <= 32'h0;
            end
        end else begin
            if (we  && waddr  != 5'b0) begin mem[waddr]  <= wdata;  memb[waddr]  <= wdata;  end
            if (we1 && waddr1 != 5'b0) begin mem[waddr1] <= wdata1; memb[waddr1] <= wdata1; end
        end
    end

    // 异步读 + 写优先穿透 (x0 恒 0)
    assign rdata1 = (raddr1 == 5'b0) ? 32'h0
                  : (we1 && waddr1 == raddr1) ? wdata1
                  : (we  && waddr  == raddr1) ? wdata
                  : mem[raddr1];

    assign rdata2 = (raddr2 == 5'b0) ? 32'h0
                  : (we1 && waddr1 == raddr2) ? wdata1
                  : (we  && waddr  == raddr2) ? wdata
                  : mem[raddr2];

    assign rdata3 = (raddr3 == 5'b0) ? 32'h0
                  : (we1 && waddr1 == raddr3) ? wdata1
                  : (we  && waddr  == raddr3) ? wdata
                  : memb[raddr3];

    assign rdata4 = (raddr4 == 5'b0) ? 32'h0
                  : (we1 && waddr1 == raddr4) ? wdata1
                  : (we  && waddr  == raddr4) ? wdata
                  : memb[raddr4];

endmodule
