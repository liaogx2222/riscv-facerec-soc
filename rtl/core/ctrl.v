//=====================================================================
// ctrl.v : RV32I 主控制器 (ID 级译码)
//
// 输入 32bit 指令, 输出流水线控制信号 (规格书 2.2 节 ID/EX 定义):
//   alu_src    : 0 = rs2, 1 = imm (SLLI/SRLI/SRAI 的 shamt 走 imm)
//   alu_op     : ALU 操作编码, 见 rv32i_defs.v (与 ALU 内自定义编码一致)
//   branch/jal/jalr : 分支/跳转
//   mem_read/mem_write : 数据内存访问
//   reg_write/mem_to_reg : 寄存器写回
//   illegal    : 非法指令标志 (ecall/ebreak 在基础任务视为 illegal)
//
// 支持: LUI AUIPC JAL JALR
//       LB LH LW LBU LHU
//       SB SH SW
//       BEQ BNE BLT BGE BLTU BGEU
//       ADD SUB SLL SLT SLTU XOR SRL SRA OR AND
//       ADDI SLTI SLTIU XORI ORI ANDI SLLI SRLI SRAI
//       fence (NOP)
//=====================================================================
`timescale 1ns/1ps

`include "rv32i_defs.v"

module ctrl (
    input  wire [31:0] instr,

    output reg         alu_src,
    output reg  [ 3:0] alu_op,
    output reg         branch,
    output reg         jal,
    output reg         jalr,
    output reg         mem_read,
    output reg         mem_write,
    output reg         reg_write,
    output reg         mem_to_reg,
    output reg         alu_a_pc,       // AUIPC: ALU 的 a 输入用 pc (pc+imm)
    output reg         alu_a_zero,     // LUI: ALU 的 a 输入强制为 0 (U-type 无 rs1)
    output reg         illegal,

    // ---- M3a: SYSTEM/CSR 译码输出 ----
    output reg         csr_en,         // csrrw/s/c[i] 族
    output reg         csr_imm,        // 源操作数 = uimm(=rs1字段零扩展)
    output reg  [ 1:0] csr_mode,       // 00=write 01=set 10=clear
    output reg         sys_mret,
    output reg         sys_ecall,
    output reg         sys_ebreak
);

    // 指令位域分解
    wire [ 6:0] opcode = instr[6:0];
    wire [ 2:0] funct3 = instr[14:12];
    wire [ 6:0] funct7 = instr[31:25];

    // 默认值 (非法/无效指令: 全不激活, 视为 NOP bubble)
    always @(*) begin
        alu_src    = 1'b0;
        alu_op     = 4'b0;
        branch     = 1'b0;
        jal        = 1'b0;
        jalr       = 1'b0;
        mem_read   = 1'b0;
        mem_write  = 1'b0;
        reg_write  = 1'b0;
        mem_to_reg = 1'b0;
        alu_a_pc   = 1'b0;
        alu_a_zero = 1'b0;
        illegal    = 1'b0;
        csr_en     = 1'b0;
        csr_imm    = 1'b0;
        csr_mode   = 2'b00;
        sys_mret   = 1'b0;
        sys_ecall  = 1'b0;
        sys_ebreak = 1'b0;

        case (opcode)
            //-----------------------------------------------------------------
            // U-type
            //-----------------------------------------------------------------
            `OP_LUI: begin
                alu_src    = 1'b1;
                alu_op     = `ALU_ADD;     // 结果 = imm (移位后)
                reg_write  = 1'b1;
                alu_a_zero = 1'b1;         // U-type 无 rs1: 强制 a=0, 避免把
                                           // imm 位域当寄存器号读到非零值污染结果
            end
            `OP_AUIPC: begin
                alu_src   = 1'b1;
                alu_op    = `ALU_ADD;      // 结果 = pc + imm
                alu_a_pc  = 1'b1;          // ALU a 输入用 pc
                reg_write = 1'b1;
            end

            //-----------------------------------------------------------------
            // J-type
            //-----------------------------------------------------------------
            `OP_JAL: begin
                alu_src    = 1'b1;
                alu_op     = `ALU_ADD;     // 结果 = pc + 4 (pc_plus4)
                jal        = 1'b1;
                reg_write  = 1'b1;
                mem_to_reg = 1'b1;         // 写回 pc_plus4
            end

            //-----------------------------------------------------------------
            // I-type JALR (仅 funct3=0 合法)
            //-----------------------------------------------------------------
            `OP_JALR: begin
                if (funct3 == 3'b000) begin
                    alu_src    = 1'b1;
                    alu_op     = `ALU_ADD; // 结果 = pc + 4
                    jalr       = 1'b1;
                    reg_write  = 1'b1;
                    mem_to_reg = 1'b1;
                end else begin
                    illegal = 1'b1;
                end
            end

            //-----------------------------------------------------------------
            // B-type
            //-----------------------------------------------------------------
            `OP_BRANCH: begin
                case (funct3)
                    `F3_BEQ, `F3_BNE, `F3_BLT, `F3_BGE, `F3_BLTU, `F3_BGEU: begin
                        branch = 1'b1;
                        alu_op = `ALU_BRANCH;  // 分支比较在 EX 级按 funct3 判断
                    end
                    default: illegal = 1'b1;
                endcase
            end

            //-----------------------------------------------------------------
            // I-type LOAD
            //-----------------------------------------------------------------
            `OP_LOAD: begin
                case (funct3)
                    `F3_LB, `F3_LH, `F3_LW, `F3_LBU, `F3_LHU: begin
                        alu_src    = 1'b1;
                        alu_op     = `ALU_ADD;   // 地址 = rs1 + imm
                        mem_read   = 1'b1;
                        reg_write  = 1'b1;
                        mem_to_reg = 1'b1;
                    end
                    default: illegal = 1'b1;
                endcase
            end

            //-----------------------------------------------------------------
            // S-type
            //-----------------------------------------------------------------
            `OP_STORE: begin
                case (funct3)
                    `F3_SB, `F3_SH, `F3_SW: begin
                        alu_src    = 1'b1;
                        alu_op     = `ALU_ADD;   // 地址 = rs1 + imm
                        mem_write  = 1'b1;
                    end
                    default: illegal = 1'b1;
                endcase
            end

            //-----------------------------------------------------------------
            // R-type
            //-----------------------------------------------------------------
            `OP_OP: begin
                case ({funct7, funct3})
                    {`F7_0000000, `F3_ADDSUB}: begin
                        alu_src  = 1'b0;
                        alu_op   = `ALU_ADD;
                        reg_write= 1'b1;
                    end
                    {`F7_0100000, `F3_ADDSUB}: begin
                        alu_src  = 1'b0;
                        alu_op   = `ALU_SUB;
                        reg_write= 1'b1;
                    end
                    {`F7_0000000, `F3_SLL}: begin
                        alu_src  = 1'b0;
                        alu_op   = `ALU_SLL;
                        reg_write= 1'b1;
                    end
                    {`F7_0000000, `F3_SLT}: begin
                        alu_src  = 1'b0;
                        alu_op   = `ALU_SLT;
                        reg_write= 1'b1;
                    end
                    {`F7_0000000, `F3_SLTU}: begin
                        alu_src  = 1'b0;
                        alu_op   = `ALU_SLTU;
                        reg_write= 1'b1;
                    end
                    {`F7_0000000, `F3_XOR}: begin
                        alu_src  = 1'b0;
                        alu_op   = `ALU_XOR;
                        reg_write= 1'b1;
                    end
                    {`F7_0000000, `F3_SR}: begin
                        alu_src  = 1'b0;
                        alu_op   = `ALU_SRL;
                        reg_write= 1'b1;
                    end
                    {`F7_0100000, `F3_SR}: begin
                        alu_src  = 1'b0;
                        alu_op   = `ALU_SRA;
                        reg_write= 1'b1;
                    end
                    {`F7_0000000, `F3_OR}: begin
                        alu_src  = 1'b0;
                        alu_op   = `ALU_OR;
                        reg_write= 1'b1;
                    end
                    {`F7_0000000, `F3_AND}: begin
                        alu_src  = 1'b0;
                        alu_op   = `ALU_AND;
                        reg_write= 1'b1;
                    end
                    // ---- RV32M / Zmmul (S4): MUL/MULH/MULHSU/MULHU ----
                    //   DIV/DIVU/REM/REMU (funct3 4..7) 落 default=illegal,
                    //   除法继续走 soft_libgcc 纯 RV32I 实现
                    {`F7_MULDIV, `F3_ADDSUB}: begin
                        alu_src  = 1'b0;
                        alu_op   = `ALU_MUL;      // MUL
                        reg_write= 1'b1;
                    end
                    {`F7_MULDIV, `F3_SLL}: begin
                        alu_src  = 1'b0;
                        alu_op   = `ALU_MUL;      // MULH
                        reg_write= 1'b1;
                    end
                    {`F7_MULDIV, `F3_SLT}: begin
                        alu_src  = 1'b0;
                        alu_op   = `ALU_MUL;      // MULHSU
                        reg_write= 1'b1;
                    end
                    {`F7_MULDIV, `F3_SLTU}: begin
                        alu_src  = 1'b0;
                        alu_op   = `ALU_MUL;      // MULHU
                        reg_write= 1'b1;
                    end
                    default: illegal = 1'b1;
                endcase
            end

            //-----------------------------------------------------------------
            // I-type OP-IMM (立即数运算)
            //-----------------------------------------------------------------
            `OP_OP_IMM: begin
                // 注意: ADDI/SLTI/SLTIU/XORI/ORI/ANDI 的 [31:25] 属于立即数,
                //       不能用于 case 匹配 (只按 funct3 判断)
                case (funct3)
                    `F3_ADDSUB: begin                 // ADDI
                        alu_src  = 1'b1;
                        alu_op   = `ALU_ADD;
                        reg_write= 1'b1;
                    end
                    `F3_SLT: begin                    // SLTI
                        alu_src  = 1'b1;
                        alu_op   = `ALU_SLT;
                        reg_write= 1'b1;
                    end
                    `F3_SLTU: begin                   // SLTIU
                        alu_src  = 1'b1;
                        alu_op   = `ALU_SLTU;
                        reg_write= 1'b1;
                    end
                    `F3_XOR: begin                    // XORI
                        alu_src  = 1'b1;
                        alu_op   = `ALU_XOR;
                        reg_write= 1'b1;
                    end
                    `F3_OR: begin                     // ORI
                        alu_src  = 1'b1;
                        alu_op   = `ALU_OR;
                        reg_write= 1'b1;
                    end
                    `F3_AND: begin                    // ANDI
                        alu_src  = 1'b1;
                        alu_op   = `ALU_AND;
                        reg_write= 1'b1;
                    end
                    `F3_SLL: begin                    // SLLI: 要求 [31:25]=0000000
                        if (instr[31:25] == 7'b0000000) begin
                            alu_src  = 1'b1;
                            alu_op   = `ALU_SLL;
                            reg_write= 1'b1;
                        end else illegal = 1'b1;
                    end
                    `F3_SR: begin                     // SRLI [31:25]=0000000 / SRAI [31:25]=0100000
                        if (instr[31:25] == 7'b0000000) begin
                            alu_src  = 1'b1;
                            alu_op   = `ALU_SRL;
                            reg_write= 1'b1;
                        end else if (instr[31:25] == 7'b0100000) begin
                            alu_src  = 1'b1;
                            alu_op   = `ALU_SRA;
                            reg_write= 1'b1;
                        end else illegal = 1'b1;
                    end
                    default: illegal = 1'b1;
                endcase
            end

            //-----------------------------------------------------------------
            // SYSTEM / FENCE
            //-----------------------------------------------------------------
            `OP_SYSTEM: begin
                // M3a (高阶任务2): SYSTEM 结构化译码
                if (funct3 == 3'b000) begin
                    case (instr[31:20])
                        12'h000: sys_ecall  = 1'b1;
                        12'h001: sys_ebreak = 1'b1;
                        12'h302: sys_mret   = 1'b1;
                        default: illegal    = 1'b1;   // sret/wfi/其余
                    endcase
                end else begin
                    case (funct3)
                        3'b001, 3'b010, 3'b011: begin
                            csr_en = 1'b1; csr_mode = funct3[1:0] - 2'd1;
                            reg_write = 1'b1;
                        end
                        3'b101, 3'b110, 3'b111: begin
                            csr_en = 1'b1; csr_imm = 1'b1;
                            csr_mode = funct3[1:0] - 2'd1; reg_write = 1'b1;
                        end
                        default: illegal = 1'b1;      // funct3=100 保留
                    endcase
                end
            end
            `OP_FENCE: begin
                // fence 合法无副作用, 按 NOP 处理
                illegal = 1'b0;
            end

            default: illegal = 1'b1;
        endcase
    end

endmodule
