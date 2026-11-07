//=====================================================================
// mul_unit.v : RV32I Zmmul 乘族 —— **两级流水**乘法器（提速措施 4）
//
// 为什么需要它
// ------------
// Q2 实测：CPU 在 100 MHz 约束下只能到 72.41 MHz，关键路径是**单周期 32x32 乘法器**：
//     ex_mem_reg_write → 转发 mux → alu_a → [4 级级联 APM] → mem_wb_alu_result
//   4 个 18x18 APM 串联（N4_m1 → N14_m2 → N14_m3 → N14_m4）纯组合延迟约 **9.5 ns**，
//   加上转发 2.1 ns + 布线 1.7 ns ⇒ 13.8 ns，物理上不可能塞进 10 ns。
//   ⇒ 必须**在乘法器内部切一刀**，让每级 ≤ 5 ns。
//
// 结构（两级流水）
// ----------------
//   第 1 级：4 个 **17x17 部分积并行**（每个 1 个 18x18 APM，约 2 ns），结果寄存
//   第 2 级：两层进位保存压缩 + 一次末级进位传播，组合输出
//
//   数学：把 a、b 各拆成「高 16 位（按符号扩展）+ 低 16 位（零扩展）」
//         a = a_hi * 2^16 + a_lo ，  b = b_hi * 2^16 + b_lo
//         a * b = p_hh*2^32 + (p_lh + p_hl)*2^16 + p_ll
//   ⇒ **对有符号/无符号统一成立**（关键是 a_hi/b_hi 的符号位取自 a[31]/b[31]，
//     而 a[31]/b[31] 恰好就是 a/b 的符号 ⇒ 无需额外判断）。
//
// 符号规则（按 funct3）
//   MUL   (000) : a、b 均有符号 → 低 32 位
//   MULH  (001) : a、b 均有符号 → 高 32 位
//   MULHSU(010) : a 有符号、b 无符号 → 高 32 位
//   MULHU (011) : a、b 均无符号 → 高 32 位
//
// 时序契约（**CPU 侧必须配合**）
//   - `en`（= mul_go）只在**发射拍**为 1：那拍锁存 4 个部分积
//   - 下一拍 `result` 有效，由 CPU 的 MEM 转发和 MEM/WB 锁存取用，无需停顿 EX
//   - `en`=0 时部分积保持；可每拍接受一组新输入，上一组结果在锁存前有效
//
// ⚠ 与 rtl/core/alu.v 配套：alu.v 已删掉内部 4 个乘法器，`ALU_MUL` 走本模块。
//   板证版（单周期乘法）备份在 _trash/2026-09-22/rtl_baseline_27MHz/alu.v
//=====================================================================
`timescale 1ns/1ps

module mul_unit (
    input  wire        clk,
    input  wire        resetn,
    input  wire        en,          // 第 1 级锁存使能（仅发射拍为 1）
    input  wire [31:0] a,
    input  wire [31:0] b,
    input  wire [ 2:0] funct3,      // 000 MUL / 001 MULH / 010 MULHSU / 011 MULHU

    output wire [31:0] result
);

    // ---- 第 1 级：4 个 17x17 部分积（并行，各 1 个 18x18 APM）----
    wire a_signed = (funct3 != 3'b011);                        // MULHU 之外 a 都是有符号
    wire b_signed = (funct3 == 3'b000) | (funct3 == 3'b001);   // 只有 MUL/MULH 的 b 有符号

    wire signed [16:0] a_hi = a_signed ? {a[31], a[31:16]} : {1'b0, a[31:16]};
    wire signed [16:0] b_hi = b_signed ? {b[31], b[31:16]} : {1'b0, b[31:16]};
    wire signed [16:0] a_lo = {1'b0, a[15:0]};                 // 低半字恒为正值
    wire signed [16:0] b_lo = {1'b0, b[15:0]};

    wire signed [33:0] p_ll = a_lo * b_lo;
    wire signed [33:0] p_lh = a_lo * b_hi;
    wire signed [33:0] p_hl = a_hi * b_lo;
    wire signed [33:0] p_hh = a_hi * b_hi;

    reg signed [33:0] p_ll_q, p_lh_q, p_hl_q, p_hh_q;
    reg        [ 2:0] f3_q;

    always @(posedge clk) begin
        if (!resetn) begin
            p_ll_q <= 34'sd0; p_lh_q <= 34'sd0; p_hl_q <= 34'sd0; p_hh_q <= 34'sd0;
            f3_q   <= 3'b000;
        end else if (en) begin
            p_ll_q <= p_ll;  p_lh_q <= p_lh;  p_hl_q <= p_hl;  p_hh_q <= p_hh;
            f3_q   <= funct3;
        end
        // en=0: 保持（停的那拍结果稳定）
    end

    // ---- 第 2 级：两层进位保存压缩，最后仅一次进位传播 ----
    // 只使用积的低64位；移位前保留部分积的符号扩展。
    wire [63:0] term_ll = {{30{p_ll_q[33]}}, p_ll_q};
    wire [63:0] term_lh = {{14{p_lh_q[33]}}, p_lh_q, 16'b0};
    wire [63:0] term_hl = {{14{p_hl_q[33]}}, p_hl_q, 16'b0};
    wire [63:0] term_hh = {p_hh_q[31:0], 32'b0};
    wire [63:0] sum0 = term_ll ^ term_lh ^ term_hl;
    wire [63:0] carry0 = ((term_ll & term_lh) | (term_ll & term_hl)
                         | (term_lh & term_hl)) << 1;
    wire [63:0] sum1 = sum0 ^ carry0 ^ term_hh;
    wire [63:0] carry1 = ((sum0 & carry0) | (sum0 & term_hh)
                         | (carry0 & term_hh)) << 1;
    wire [63:0] prod = sum1 + carry1;

    // ---- 结果选择（与 alu.v 原逻辑一致）----
    //   MUL   → 低 32 位
    //   MULH  → 高 32 位（有符号）
    //   MULHSU→ 高 32 位（a 有符号 × b 无符号）
    //   MULHU → 高 32 位（无符号）
    //   ⚠ MULHSU 的 a 是有符号、b 是无符号 —— 上面的通用分解已经正确处理
    //     （a_hi 带符号、b_hi 零扩展），所以这里只需按 funct3 取高/低 32 位。
    assign result = (f3_q == 3'b000) ? prod[31:0] : prod[63:32];

endmodule
