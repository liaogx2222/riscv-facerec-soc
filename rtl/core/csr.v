//=====================================================================
// csr.v : 机器模式 CSR 子集 + 陷阱提交逻辑 (M3a, 高阶任务2)
//
// 支持: mstatus(仅 MIE/MPIE/MPP WARL) / mtvec / mepc / mcause
//       读口任意周期可用(EX 组合), 写口 = csrrw/s/c 在 adv 退休拍提交.
// trap/mret 也在此提交 (EX 级判定, 与分支重定向同层):
//   trap: mepc←pc, mcause←cause, MPIE←MIE, MIE←0
//   mret: MIE←MPIE
// 赛题口径: 基础任务纯 RV32I, CSR/陷阱为高阶任务2扩展 (文档已声明).
//=====================================================================
`timescale 1ns/1ps

module csr (
    input  wire        clk,
    input  wire        resetn,

    // EX 读口 (含地址合法性)
    input  wire [11:0] rd_addr,
    output reg  [31:0] rd_data,
    output wire        rd_ok,

    // csrrw/s/c 提交 (adv 门控的退休拍脉冲)
    input  wire        wr_en,
    input  wire [11:0] wr_addr,
    input  wire [ 1:0] wr_mode,          // 00=write 01=set 10=clear
    input  wire [31:0] wr_data,

    // 陷阱
    input  wire        trap,
    input  wire [31:0] trap_cause,
    input  wire [31:0] trap_pc,
    input  wire        mret,

    output wire        mstatus_mie,
    output wire [31:0] mtvec_o,
    output wire [31:0] mepc_o           // mret 目标
);
    localparam C_MSTATUS = 12'h300;
    localparam C_MTVEC   = 12'h305;
    localparam C_MEPC    = 12'h341;
    localparam C_MCAUSE  = 12'h342;
    // ★ 2026-10-14: mhartid(0xF14, 规范强制只读 CSR) —— riscv-tests 官方
    //   环境序言 `csrr a0, mhartid; bnez a0, self` 依赖其可读(单核恒 0)。
    //   未实现 ⇒ 非法指令陷阱 ⇒ mtvec 未初始化(0) ⇒ 死循环 (官方套件全灭)。
    //   只读常数, 零写口, 对既有逻辑零影响 (CoreMark/回归均不读 0xF14)。
    localparam C_MHARTID = 12'hF14;
    // ★ 2026-10-14: mie/satp/pmpcfg0/pmpaddr0 —— riscv-tests 官方环境序言
    //   会写这四个 CSR (关中断 + PMP 全内存授权); 未实现 ⇒ 非法陷阱, 而
    //   mtvec 恰好已指向这段 ⇒ 陷阱死循环。按规范实现为 **WARL 暂存**
    //   (写任意/读回, 无任何功能副作用): 本核 PMP 不存在、中断源只有
    //   intc 且接受逻辑只看 mstatus.MIE (mie 位屏蔽为已备案的裁剪点)。
    localparam C_MIE      = 12'h304;
    localparam C_SATP     = 12'h180;
    localparam C_PMPADDR0 = 12'h3B0;
    localparam C_PMPCFG0  = 12'h3A0;
    localparam MSTATUS_WMASK = 32'h0000_1888;  // MIE(3)=0x8 MPIE(7)=0x80 MPP(12:11)=0x1800

    reg [31:0] mstatus, mtvec, mepc, mcause;
    reg [31:0] mie_q, satp_q, pmpaddr0_q, pmpcfg0_q;   // WARL 暂存

    wire known = (rd_addr == C_MSTATUS) || (rd_addr == C_MTVEC) ||
                 (rd_addr == C_MEPC)    || (rd_addr == C_MCAUSE) ||
                 (rd_addr == C_MHARTID) || (rd_addr == C_MIE) ||
                 (rd_addr == C_SATP)    || (rd_addr == C_PMPADDR0) ||
                 (rd_addr == C_PMPCFG0);
    assign rd_ok        = known;
    assign mstatus_mie  = mstatus[3];
    assign mtvec_o      = mtvec;
    assign mepc_o       = mepc;

    always @(*) begin
        case (rd_addr)
            C_MSTATUS:  rd_data = mstatus;
            C_MTVEC:    rd_data = mtvec;
            C_MEPC:     rd_data = mepc;
            C_MCAUSE:   rd_data = mcause;
            C_MHARTID:  rd_data = 32'h0;             // 单核, hartid=0
            C_MIE:      rd_data = mie_q;
            C_SATP:     rd_data = satp_q;
            C_PMPADDR0: rd_data = pmpaddr0_q;
            C_PMPCFG0:  rd_data = pmpcfg0_q;
            default:    rd_data = 32'h0;
        endcase
    end

    function [31:0] apply_mode;
        input [31:0] old, val;
        input [ 1:0] md;
        begin
            case (md)
                2'b01:   apply_mode = old | val;
                2'b10:   apply_mode = old & ~val;
                default: apply_mode = val;
            endcase
        end
    endfunction

    always @(posedge clk) begin
        if (!resetn) begin
            mstatus <= 32'h0; mtvec <= 32'h0;
            mepc    <= 32'h0; mcause <= 32'h0;
            mie_q <= 32'h0; satp_q <= 32'h0; pmpaddr0_q <= 32'h0; pmpcfg0_q <= 32'h0;
        end else begin
            if (wr_en && (wr_addr == C_MSTATUS))
                mstatus <= (mstatus & ~MSTATUS_WMASK) |
                           (apply_mode(mstatus, wr_data, wr_mode) & MSTATUS_WMASK);
            else if (wr_en && (wr_addr == C_MTVEC))  mtvec  <= apply_mode(mtvec, wr_data, wr_mode) & 32'hFFFF_FFFE;
            else if (wr_en && (wr_addr == C_MEPC))   mepc   <= apply_mode(mepc, wr_data, wr_mode) & 32'hFFFF_FFFF;
            else if (wr_en && (wr_addr == C_MCAUSE)) mcause <= apply_mode(mcause, wr_data, wr_mode);
            // ★ WARL 暂存组 (mie/satp/pmpcfg0/pmpaddr0): 写任意读回, 无副作用
            else if (wr_en && (wr_addr == C_MIE))      mie_q      <= apply_mode(mie_q, wr_data, wr_mode);
            else if (wr_en && (wr_addr == C_SATP))     satp_q     <= apply_mode(satp_q, wr_data, wr_mode);
            else if (wr_en && (wr_addr == C_PMPCFG0))  pmpcfg0_q  <= apply_mode(pmpcfg0_q, wr_data, wr_mode);
            else if (wr_en && (wr_addr == C_PMPADDR0)) pmpaddr0_q <= apply_mode(pmpaddr0_q, wr_data, wr_mode);

            if (trap) begin
                mepc    <= trap_pc;
                mcause  <= trap_cause;
                mstatus[7] <= mstatus[3];          // MPIE ← MIE
                mstatus[3] <= 1'b0;                // MIE  ← 0
            end
            if (mret)
                mstatus[3] <= mstatus[7];          // MIE ← MPIE
        end
    end
endmodule
