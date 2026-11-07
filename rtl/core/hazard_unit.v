//=====================================================================
// hazard_unit.v : 冒险检测单元 (Load-Use 停顿 + 分支冲刷)
//
// 1. Load-Use 冒险: ID/EX 的 mem_read=1, 且 ID/EX.rd == IF/ID 指令的 rs1/rs2 (rd!=0)
//    -> 停顿 1 拍: PC 冻结, IF/ID 保持, ID/EX 插 bubble
//    ★ 2026-10-08 阶段1: MEM→EX load 旁路生效后, **一般消费者不再需要停顿**
//      （load 数据在其 MEM 拍即可从 EX/MEM 转发取用）。
//    ★★ 2026-10-11 (顺序双发冲 5, 交接 §5): **branch/jalr 例外也取消** ——
//      load 相邻的 branch/jalr 改走「EX 挂起 + MEM 级解析」(见 cpu_rv32i.v):
//      EX 拍投机走 BTB 预测但不解析, 操作数快照随流水进 MEM, MEM 拍用快照
//      比较解析, 失配走 MEM 级校正通路 (冲刷 if_id/id_ex, pc←解析目标)。
//      ⇒ LU_BYPASS_EN=1 时本模块的 load-use 停顿**恒为 0**
//        (停顿结构保留, 把 LU_BYPASS_EN 改 0 即整体回退到"全部停顿")。
// 2. 内存忙 (valid && !ready): EX/MEM 等待内存完成, PC/IF/ID/ID/EX 保持
// 3. 分支/跳转: EX 级 branch_taken -> 冲刷 IF/ID 和 ID/EX (flush)
//=====================================================================
`timescale 1ns/1ps

`include "rv32i_defs.v"

module hazard_unit (
    input  wire [ 4:0] id_ex_rd,       // EX 级目标寄存器 (slot0)
    input  wire        id_ex_mem_read, // EX 级读内存 (slot0)
    input  wire [ 4:0] id_ex1_rd,      // ★ P3': slot1 也可能是 load
    input  wire        id_ex1_mem_read,
    input  wire [ 4:0] if_id_rs1,      // ID 级源寄存器
    input  wire [ 4:0] if_id_rs2,
    input  wire        branch_taken,   // EX 级分支/跳转命中
    input  wire        mem_busy,       // 内存忙 (valid && !ready)

    output reg         load_use_stall, // Load-Use 停顿 (ID/EX 插 bubble)
    output reg         mem_stall,      // 内存忙停顿 (保持全部)
    output reg         flush           // 冲刷 (分支/跳转)
);

    // ★★ 阶段1 (2026-10-08) + 修时序违例
    //   MEM→EX load 旁路对**大多数消费者**生效 ⇒ 取消停顿。
    //   原理: load 的数据在其 **MEM 拍**就已可用 ——
    //     · 快路命中: dcache 的 p_dat* 是寄存器输出, 该拍开头即有效;
    //     · 慢路/miss: 该拍 d_rdy=0 ⇒ adv=0 整条流水线冻结, 下拍数据才有效;
    //   ⇒ 消费者在 EX 拍可直接从 EX/MEM 转发取用, 零停顿且绝不取到脏值。
    //
    //   ★★ 2026-10-11: **branch/jalr 例外取消** (冲 5 §5)。
    //     原例外原因: 分支判定/目标要走
    //       alu_a → ALU → branch_taken → next_pc → f_redirect → pf_buffer
    //     长链, 把 load 数据接进这条链后 PDS 实测 WNS = −1.249 ns。
    //     现改为: load 相邻的 branch/jalr 在 EX 拍**挂起不解析** (投机走
    //     BTB), 操作数快照进 MEM, MEM 拍用全寄存器源比较解析 —— load 数据
    //     从 "→ pc" 长链上物理摘除, 停顿随之取消 (CoreMark load_use 17.1K
    //     拍全部回收, 预期 +6.4%)。
    //
    //   回退（一行）: 把 LU_BYPASS_EN 改成 1'b0 ⇒ 恢复"全部 load-use 都停顿"
    //   (挂起/MEM 解析通路在停顿下自然不触发, 逐拍等价于旧版)。
    localparam LU_BYPASS_EN = 1'b1;

    // Load-Use 检测 (★ P3': slot0 与 slot1 的 load 都要查)
    wire lu0 = id_ex_mem_read  && (id_ex_rd  != 0) &&
               ((id_ex_rd  == if_id_rs1) || (id_ex_rd  == if_id_rs2));
    wire lu1 = id_ex1_mem_read && (id_ex1_rd != 0) &&
               ((id_ex1_rd == if_id_rs1) || (id_ex1_rd == if_id_rs2));
    // ★ 2026-10-11: LU_BYPASS_EN=1 ⇒ load_use 恒 0 (branch/jalr 走挂起+MEM 解析)
    wire load_use = (lu0 || lu1) && !LU_BYPASS_EN;

    always @(*) begin
        load_use_stall = load_use;
        mem_stall      = mem_busy;
        flush          = branch_taken;
    end

endmodule
