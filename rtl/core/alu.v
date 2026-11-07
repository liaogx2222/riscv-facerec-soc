//=====================================================================
// alu.v : RV32I ALU 单元
//
// 支持 ctrl.v 定义的全部 ALU 操作编码 (见 rv32i_defs.v):
//   ALU_ADD/SUB/SLL/SLT/SLTU/XOR/SRL/SRA/OR/AND + ALU_BRANCH (分支比较)
//
// 输入:
//   a, b              : 32bit 操作数 (**不含** load 旁路 —— 分支比较用它)
//   a_ld, b_ld        : ★ MEM 级 load 旁路专用操作数 (**晚到**, 只影响 result)
//   a_ld_en / b_ld_en : 旁路使能
//   alu_op            : 4bit 操作编码
//   funct3            : 分支条件编码 [2:0] (BEQ/BNE/BLT/BGE/BLTU/BGEU)
//   mul_result        : ★ 措施4 两级流水乘法器结果 (来自 mul_unit)
//
// 输出:
//   result       : 运算结果 (非分支)
//   branch_taken : 分支是否跳转 (alu_op==ALU_BRANCH 时有效)
//
// ★★★ 为什么 load 旁路必须单开 a_ld/b_ld 端口 (2026-10-08, 第四次修时序违例)
// --------------------------------------------------------------------------
//   历史教训, 按顺序读:
//     §5.3 让 hazard_unit 对 branch/jalr 保留停顿 —— 只保证**功能**正确;
//     §5.5 单开 a_br/b_br "硬结构切"       —— 被综合器 **CSE 合并回去**, 失效;
//     §5.7 把 dcache 侧做短 (tag 比较寄存化) —— 有效 (WNS −1.372→−0.906), 但路径仍在。
//   根因始终是同一件事:
//       `branch_target = (alu_a + id_ex_imm) & ~1`, 而 `alu_a` 含 2'b11(load 旁路)项
//     ⇒ **代码里就把 load 数据接进了分支目标加法器**, 再经 pred_ok 32 位比较
//       → branch_taken_eff → next_pc → pc → pf_buffer.next_issue。
//   本轮改法: load 值走**专用端口** a_ld/b_ld, 在 ALU **内部**才与 result 路径汇合;
//     a/b 端口本身是 3:1 纯寄存器源 ⇒ 分支比较与 branch_target **在逻辑上就
//     接不到** load 数据。再配 `syn_hier="hard"` 硬边界, 杜绝跨模块 CSE 把
//     `ax` 与 `a` 合并 (这正是 §5.5 失败的机理)。
//   功能等价: branch/jalr 的 forward_a/b 永不是 2'b11 (forwarding_unit 的
//     bypass_ok 已排除 ctrl_alu) ⇒ 在分支上恒有 ax==a、bx==b。
//=====================================================================
`timescale 1ns/1ps

`include "rv32i_defs.v"

module alu #(
    parameter COMPARE_ALT_INPUT = 0,
    parameter PAIR_LIGHT_RESULT = 0
) /* synthesis syn_hier="hard" */ (
    input  wire [31:0] a,
    input  wire [31:0] b,
    input  wire [31:0] a_ld,       // ★ load 旁路专用 (晚到)
    input  wire [31:0] b_ld,
    input  wire        a_ld_en,
    input  wire        b_ld_en,
    input  wire [ 3:0] alu_op,
    input  wire [ 2:0] funct3,     // 分支比较条件 (BEQ/BNE/BLT/BGE/BLTU/BGEU)
    input  wire [31:0] mul_result, // ★ 提速措施4: 两级流水乘法器的结果

    input  wire [31:0] cmp_a,
    input  wire [31:0] cmp_b,
    output reg  [31:0] result,     // 运算结果
    output reg  [31:0] addr_result, // speculative ID AGU only; compares predict 0
    output wire        branch_taken
);

    // ---- result 用"含 load 旁路"的操作数; 分支比较用**不含**旁路的 a/b ----
    wire [31:0] ax = a_ld_en ? a_ld : a;
    wire [31:0] bx = b_ld_en ? b_ld : b;

    //-----------------------------------------------------------------
    // ★ 单一差值向量 (§5.7 B): SUB / SLT / SLTU 全从它派生
    //   原先把 `a-b` / `$signed(a)<$signed(b)` / `a<b` 分开写, PDS 把它们共享成
    //   **部分共享进位链**: 实测 ALU 内部 3.585 ns 中有 0.496 ns 的 `_N3` 网 +
    //   0.113/0.295 两级, 全是 `N7.slt`(比较链) → `N39`(减法链) 的串联代价。
    //   有符号小于: 符号不同 → 看 a 的符号; 符号相同 → 看差值符号
    //   无符号小于: 就是借位 d[32]
    //-----------------------------------------------------------------
    wire [32:0] d_ab  = {1'b0, ax} - {1'b0, bx};
    // A comparison result can feed the other issue slot and then the ID
    // pre-read AGU in this same cycle. Do not serialize those comparisons
    // through the full-width SUB carry chain. Compare bytes in parallel and
    // combine their equal/less flags in a short lexicographic network.
    wire ltu_s, ltu_b;
    wire [31:0] cmp_ax = COMPARE_ALT_INPUT ? cmp_a : ax;
    wire [31:0] cmp_bx = COMPARE_ALT_INPUT ? cmp_b : bx;
    rv32_cmp_bytes u_cmp_result (.a(cmp_ax), .b(cmp_bx), .lt(ltu_s));
    rv32_cmp_bytes u_cmp_branch (.a(a), .b(b), .lt(ltu_b));
    wire lt_s = (cmp_ax[31] ^ cmp_bx[31]) ? cmp_ax[31] : ltu_s;
    wire lt_b = (a[31] ^ b[31]) ? a[31] : ltu_b;
    wire eq_b  = (a == b);

    always @(*) begin
        case (alu_op)
            `ALU_ADD:   addr_result = ax + bx;
            `ALU_SUB:   addr_result = d_ab[31:0];
            `ALU_SLL:   addr_result = ax << bx[4:0];
            `ALU_XOR:   addr_result = ax ^ bx;
            `ALU_SRL:   addr_result = ax >> bx[4:0];
            `ALU_SRA:   addr_result = $signed(ax) >>> bx[4:0];
            `ALU_OR:    addr_result = ax | bx;
            `ALU_AND:   addr_result = ax & bx;
            `ALU_MUL:   addr_result = mul_result;
            default:    addr_result = 32'h0;
        endcase
    end

    // Only the architectural result observes compare flags. The speculative
    // address output has no comparator data input, cutting compare -> ID AGU.
    generate if (PAIR_LIGHT_RESULT) begin : g_pair_light
        // The CPU pair guard permits only these producing operations for
        // same-cycle EX0 -> EX1 forwarding. Keep SUB/compare
        // out of that path; the independent ID preview remains complete.
        always @(*) begin
            case (alu_op)
                `ALU_ADD: result = ax + bx;
                `ALU_SLL: result = (PAIR_LIGHT_RESULT == 2) ? ax << bx[4:0] : 32'b0;
                `ALU_SRL: result = (PAIR_LIGHT_RESULT == 2) ? ax >> bx[4:0] : 32'b0;
                `ALU_SRA: result = (PAIR_LIGHT_RESULT == 2) ? $signed(ax) >>> bx[4:0] : 32'sb0;
                `ALU_XOR: result = ax ^ bx;
                `ALU_OR:  result = ax | bx;
                `ALU_AND: result = ax & bx;
                default:  result = 32'b0;
            endcase
        end
    end else begin : g_full_result
    always @(*) begin
        case (alu_op)
            `ALU_SLT:  result = {31'b0, lt_s};
            `ALU_SLTU: result = {31'b0, ltu_s};
            default:   result = addr_result;
        endcase
    end
    end endgenerate

    // 分支条件判断 (组合, 仅 alu_op==ALU_BRANCH 时有效)
    //   ★ 用 a/b (**不含** load 旁路) —— 见文件头说明
    assign branch_taken =
        (alu_op == `ALU_BRANCH) ? (
            (funct3 == `F3_BEQ) ?  eq_b :
            (funct3 == `F3_BNE) ? ~eq_b :
            (funct3 == `F3_BLT) ?  lt_b :
            (funct3 == `F3_BGE) ? ~lt_b :
            (funct3 == `F3_BLTU)?  ltu_b :
            (funct3 == `F3_BGEU)? ~ltu_b :
            1'b0
        ) : 1'b0;

endmodule

// Keep the byte comparison tree separate from arithmetic carry inference.
module rv32_cmp_bytes /* synthesis syn_hier="hard" */ (
    input wire [31:0] a,
    input wire [31:0] b,
    output wire lt
);
    wire [3:0] less, equal;
    genvar k;
    generate for (k = 0; k < 4; k = k + 1) begin : g_byte
        assign less[k] = a[k*8 +: 8] < b[k*8 +: 8];
        assign equal[k] = a[k*8 +: 8] == b[k*8 +: 8];
    end endgenerate
    assign lt = less[3]
              | (equal[3] & less[2])
              | (equal[3] & equal[2] & less[1])
              | (equal[3] & equal[2] & equal[1] & less[0]);
endmodule
