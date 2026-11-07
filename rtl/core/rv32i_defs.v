//=====================================================================
// rv32i_defs.v : RV32I 指令集常量与编码定义 (宏, 兼容任意 iverilog)
//
// 用 `define 宏提供 opcode/funct3/funct7/ALU 编码, 供 ctrl.v 及后续模块
// 使用。用法: `include "rv32i_defs.v" (不冲突, 宏无 timescale)。
//=====================================================================

// ---------- opcode [6:0] ----------
`define OP_LUI      7'b0110111
`define OP_AUIPC    7'b0010111
`define OP_JAL      7'b1101111
`define OP_JALR     7'b1100111
`define OP_BRANCH   7'b1100011
`define OP_LOAD     7'b0000011
`define OP_STORE    7'b0100011
`define OP_OP       7'b0110011
`define OP_OP_IMM   7'b0010011
`define OP_SYSTEM   7'b1110011
`define OP_FENCE    7'b0001111
`define OP_OP_32    7'b0111011

// ---------- funct3 [14:12] ----------
`define F3_LB       3'b000
`define F3_LH       3'b001
`define F3_LW       3'b010
`define F3_LBU      3'b100
`define F3_LHU      3'b101
`define F3_SB       3'b000
`define F3_SH       3'b001
`define F3_SW       3'b010

`define F3_BEQ      3'b000
`define F3_BNE      3'b001
`define F3_BLT      3'b100
`define F3_BGE      3'b101
`define F3_BLTU     3'b110
`define F3_BGEU     3'b111

`define F3_ADDSUB   3'b000
`define F3_SLL      3'b001
`define F3_SLT      3'b010
`define F3_SLTU     3'b011
`define F3_XOR      3'b100
`define F3_SR       3'b101
`define F3_OR       3'b110
`define F3_AND      3'b111

`define F3_FENCE    3'b000
`define F3_CSRRW    3'b001
`define F3_CSRRS    3'b010
`define F3_CSRRC    3'b011
`define F3_CSRRWI   3'b101
`define F3_CSRRSI   3'b110
`define F3_CSRRCI   3'b111

// ---------- funct7 [31:25] ----------
`define F7_0000000  7'b0000000
`define F7_0100000  7'b0100000
`define F7_MULDIV   7'b0000001   // RV32M (Zmmul 子集: 只用乘, DIV 译码为 illegal)

// ---------- ALU 操作编码 (alu.v 与 ctrl.v 共享) ----------
`define ALU_ADD     4'd0
`define ALU_SUB     4'd1
`define ALU_SLL     4'd2
`define ALU_SLT     4'd3
`define ALU_SLTU    4'd4
`define ALU_XOR     4'd5
`define ALU_SRL     4'd6
`define ALU_SRA     4'd7
`define ALU_OR      4'd8
`define ALU_AND     4'd9
`define ALU_BRANCH  4'd10
`define ALU_MUL     4'd11        // 乘族: 结果字/符号由 funct3 选 (MUL/MULH/MULHSU/MULHU)
