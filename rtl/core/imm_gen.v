//=====================================================================
// imm_gen.v : RV32I 立即数生成器 (ID 级)
//
// 按指令类型生成符号扩展的 32bit 立即数:
//   I型: load/jalr/op-imm  [31:20]
//   S型: store             {[31:25],[11:7]}
//   B型: branch            {[31],[7],[30:25],[11:8]}  (x2)
//   U型: lui/auipc         [31:12] << 12
//   J型: jal               {[31],[19:12],[20],[30:21]} (x2)
//
// 输入 instr 由 ID 级传入, 输出 imm。
//=====================================================================
`timescale 1ns/1ps

module imm_gen (
    input  wire [31:0] instr,
    output reg  [31:0] imm
);

    // 立即数字段 (按 RISC-V 定义, 符号扩展前)
    wire [31:0] imm_i = {{20{instr[31]}}, instr[31:20]};
    wire [31:0] imm_s = {{20{instr[31]}}, instr[31:25], instr[11:7]};
    wire [31:0] imm_b = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
    wire [31:0] imm_u = {instr[31:12], 12'b0};
    wire [31:0] imm_j = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};

    always @(*) begin
        case (instr[6:0])
            7'b0010011,          // OP-IMM
            7'b0000011,          // LOAD
            7'b1100111: imm = imm_i;          // JALR
            7'b0100011: imm = imm_s;          // STORE
            7'b1100011: imm = imm_b;          // BRANCH
            7'b0110111,          // LUI
            7'b0010111: imm = imm_u;          // AUIPC
            7'b1101111: imm = imm_j;          // JAL
            default:   imm = imm_i;
        endcase
    end

endmodule
