//=====================================================================
// forwarding_unit.v : 数据转发单元 (EX 级, ★ D2b 双槽版)
//
// 把 EX/MEM / MEM/WB 的写回结果转发给 EX 级两个槽的 ALU 输入, 消除数据冒险。
//
// ★★ D2b (2026-10-08) 为什么必须扩到"5 源"
// ---------------------------------------------------------------
//   单发射时, 消费者在 EX 拍能看到的上游只有两级:
//       EX/MEM (1 条前) 与 MEM/WB (2 条前)。
//   双发射后, 每个流水级装**一对**, 所以消费者之前有**四条**指令:
//       EX/MEM.slot1 → EX/MEM.slot0 → MEM/WB.slot1 → MEM/WB.slot0
//   (程序序: MEM/WB 更早)。P3/P4 保证 slot1 恒为纯 ALU 写回 ⇒ EX/MEM.slot1
//   与 MEM/WB.slot1 的转发值就是各自的 ALU 结果, 没有 load。
//
//   优先级 (越晚写越优先):
//       EX/MEM.slot1 > EX/MEM.slot0 > MEM/WB.slot1 > MEM/WB.slot0 > 寄存器堆
//
//   编码 (3 bit, 每源端口一个):
//     3'b000 寄存器堆 (id_ex_rs*_data)
//     3'b001 EX/MEM.slot0  ALU 结果
//     3'b010 EX/MEM.slot1  ALU 结果
//     3'b011 ★ MEM 级 load 旁路 (mem_load_ext) —— 仅 EX/MEM.slot0 可能是 load
//     3'b100 MEM/WB.slot0  写回值 (wb_wdata, 可含 load/jal 值)
//     3'b101 MEM/WB.slot1  写回值 (wb1_wdata, 恒为 ALU 结果)
//     3'b110 ★★ **对内转发** (仅 slot1 端口): slot0 的 **EX 级 alu_result**
//            —— slot0 与 slot1 同在 EX, slot0 在程序序上**更晚**于任何
//            EX/MEM·MEM/WB 指令 ⇒ **最高优先级**。
//            ⚠ 合法性由配对规则 P5' 保证: 只有当 slot0 是**纯 ALU 写回**
//              (非 load / 非 jal·jalr) 才允许配对 —— 因为
//                · slot0 是 load  ⇒ alu_result 是**地址**不是数据;
//                · slot0 是 jal/jalr ⇒ alu_result 是**跳转目标**, 而写回值是 pc+4;
//              (CSR 已由 P6x 排除; MUL 有 mul_go 停一拍, 停后 alu_result 有效 ⇒ 可)
//
// ★★ 为什么 load 旁路仍要走**独立编码**而不是并进主 mux
// ---------------------------------------------------------------
//   只用 hazard_unit 的停顿不够: 停顿让"load 数据 → 分支判定"功能上永不成立,
//   但组合路径依然在网表里, 综合器无法证明它是假路径。实测 WNS = −1.548 ns,
//   关键路径正是 dcache → mem_load_shifted → alu_a → ALU → branch_taken → pc。
//   把 `ctrl_alu`(= id_ex_branch | id_ex_jalr) 加进选择条件后:
//     · 选 mem_load_ext 需要 ctrl_alu = 0;
//     · branch_taken 有效需要 id_ex_branch = 1;
//   ⇒ 逻辑矛盾 ⇒ 综合器可安全剪断。
//   ★ 2026-10-11 配套机制变更 (冲 5 §5): 原先与本门配套的是 hazard_unit
//     对 branch/jalr 的 load-use 停顿; 现改为 cpu_rv32i 的「EX 挂起 + MEM
//     级解析」—— 分支在 EX 拍不解析 (branch_taken_eff/BTB 更新封锁),
//     挂起判定 = 本模块选择条件去掉 ctrl_alu 后的 raw 3'b011 (见
//     cpu_rv32i.v 挂起判定块, 与本文件优先级逐位对齐)。本门保留:
//     EX 级比较器/目标加法器仍绝不能见到 load 数据。
//   ★ 2026-10-11 §5d (P2 松绑): slot1 也可以是 branch/jalr 了 —— 其操作数
//     若需 load 旁路 (raw 011), 在 cpu_rv32i 的**配对规则**里直接拆对
//     (p_npend1, 与本文件 slot1 优先级逐位镜像) ⇒ 能发射的 slot1 分支
//     恒取不到 011, 本模块无需为 slot1 加结构门。
//
// ⚠⚠ 实现纪律 (2026-10-08 实测踩过):
//   本模块**禁止用 function 做选择**。iverilog 不会把"只在 function 体里读到的
//   信号"加进 `always @(*)` 的敏感表 ⇒ 转发选择**不更新** ⇒ 取数全错
//   (实测: cpu_loaduse_tb 四个 load 全返回 0 / c_run_tb 超时)。
//   这里改成把四条 if-else 链**显式展开**, 全部信号都在 always 块内直接可读。
//=====================================================================
`timescale 1ns/1ps

module forwarding_unit (
    // ---- 消费者: ID/EX (两个槽) ----
    input  wire [ 4:0] id_ex_rs1,
    input  wire [ 4:0] id_ex_rs2,
    input  wire        ctrl_alu,          // slot0 是 branch/jalr (结构门)
    input  wire [ 4:0] id_ex1_rs1,        // ★ D2b slot1
    input  wire [ 4:0] id_ex1_rs2,
    input  wire        id_ex1_valid,      // slot1 本拍有效才需要转发
    input  wire [ 4:0] id_ex_rd,          // ★ P5' 探针
    input  wire        id_ex_reg_write,

    // ---- 生产者: EX/MEM (一对) ----
    input  wire [ 4:0] ex_mem_rd,
    input  wire        ex_mem_reg_write,
    input  wire        ex_mem_mem_read,   // slot0 是 load
    // ★★ 2026-10-08 修复: 匹配条件**必须带 valid 门**。
    //   EX/MEM 被 branch_taken_eff / trap 压掉时 valid=0, 但 rd/reg_write/mem_read
    //   字段**仍是旧值** ⇒ 不加门会误命中, 转发到上一条指令的垃圾数据。
    //   实测: `sw ra, 28(sp)` 在 slot1 时存进了 0xffffff84 而不是 ra=0xa8。
    input  wire        ex_mem_valid,
    input  wire        ex_mem1_valid,
    input  wire [ 4:0] ex_mem1_rd,        // ★ D2b slot1
    input  wire        ex_mem1_reg_write,
    input  wire        ex_mem1_mem_read,  // ★ P3': slot1 也可能是 load

    // ---- 生产者: MEM/WB (一对) ----
    input  wire [ 4:0] mem_wb_rd,
    input  wire        mem_wb_reg_write,
    input  wire [ 4:0] mem_wb1_rd,        // ★ D2b slot1
    input  wire        mem_wb1_reg_write,

    output reg  [ 2:0] forward_a,         // slot0 rs1
    output reg  [ 2:0] forward_b,         // slot0 rs2
    output reg  [ 2:0] forward_a1,        // ★ D2b slot1 rs1
    output reg  [ 2:0] forward_b1         // ★ D2b slot1 rs2
);

    // load 旁路是否可用: EX/MEM.slot0 是 load, 且**消费者不是 branch/jalr**
    wire bypass_ok = ex_mem_valid && ex_mem_mem_read && !ctrl_alu;
    wire bypass_ok1 = ex_mem_valid && ex_mem_mem_read;   // ★ slot1 独立门

    always @(*) begin
        // ================= slot0 rs1 =================
        forward_a = 3'b000;
        // ★ P3': 程序序 = ex_mem1(最晚) > ex_mem0 > mem_wb1 > mem_wb0
        if      (ex_mem1_valid && ex_mem1_mem_read  && (ex_mem1_rd != 5'b0) && (ex_mem1_rd == id_ex_rs1))
            forward_a = 3'b011;
        else if (ex_mem1_valid && ex_mem1_reg_write && (ex_mem1_rd != 5'b0) && (ex_mem1_rd == id_ex_rs1))
            forward_a = 3'b010;
        else if (ex_mem_valid  && ex_mem_reg_write  && (ex_mem_rd  != 5'b0) && (ex_mem_rd  == id_ex_rs1))
            forward_a = bypass_ok ? 3'b011 : 3'b001;
        else if (mem_wb1_reg_write && (mem_wb1_rd != 5'b0) && (mem_wb1_rd == id_ex_rs1))
            forward_a = 3'b101;
        else if (mem_wb_reg_write  && (mem_wb_rd  != 5'b0) && (mem_wb_rd  == id_ex_rs1))
            forward_a = 3'b100;

        // ================= slot0 rs2 =================
        forward_b = 3'b000;
        // ★ P3': 程序序 = ex_mem1(最晚) > ex_mem0 > mem_wb1 > mem_wb0
        if      (ex_mem1_valid && ex_mem1_mem_read  && (ex_mem1_rd != 5'b0) && (ex_mem1_rd == id_ex_rs2))
            forward_b = 3'b011;
        else if (ex_mem1_valid && ex_mem1_reg_write && (ex_mem1_rd != 5'b0) && (ex_mem1_rd == id_ex_rs2))
            forward_b = 3'b010;
        else if (ex_mem_valid  && ex_mem_reg_write  && (ex_mem_rd  != 5'b0) && (ex_mem_rd  == id_ex_rs2))
            forward_b = bypass_ok ? 3'b011 : 3'b001;
        else if (mem_wb1_reg_write && (mem_wb1_rd != 5'b0) && (mem_wb1_rd == id_ex_rs2))
            forward_b = 3'b101;
        else if (mem_wb_reg_write  && (mem_wb_rd  != 5'b0) && (mem_wb_rd  == id_ex_rs2))
            forward_b = 3'b100;

        // ================= slot1 rs1 =================
        forward_a1 = 3'b000;
        if (id_ex1_valid) begin
            if      (id_ex_reg_write && (id_ex_rd != 5'b0) && (id_ex_rd == id_ex1_rs1))
                forward_a1 = 3'b110;
            else if (ex_mem1_valid && ex_mem1_mem_read  && (ex_mem1_rd != 5'b0) && (ex_mem1_rd == id_ex1_rs1))
                forward_a1 = 3'b011;
            else if (ex_mem1_valid && ex_mem1_reg_write && (ex_mem1_rd != 5'b0) && (ex_mem1_rd == id_ex1_rs1))
                forward_a1 = 3'b010;
            else if (ex_mem_valid  && ex_mem_reg_write  && (ex_mem_rd  != 5'b0) && (ex_mem_rd  == id_ex1_rs1))
                forward_a1 = bypass_ok1 ? 3'b011 : 3'b001;
            else if (mem_wb1_reg_write && (mem_wb1_rd != 5'b0) && (mem_wb1_rd == id_ex1_rs1))
                forward_a1 = 3'b101;
            else if (mem_wb_reg_write  && (mem_wb_rd  != 5'b0) && (mem_wb_rd  == id_ex1_rs1))
                forward_a1 = 3'b100;
        end

        // ================= slot1 rs2 =================
        forward_b1 = 3'b000;
        if (id_ex1_valid) begin
            if      (id_ex_reg_write && (id_ex_rd != 5'b0) && (id_ex_rd == id_ex1_rs2))
                forward_b1 = 3'b110;
            else if (ex_mem1_valid && ex_mem1_mem_read  && (ex_mem1_rd != 5'b0) && (ex_mem1_rd == id_ex1_rs2))
                forward_b1 = 3'b011;
            else if (ex_mem1_valid && ex_mem1_reg_write && (ex_mem1_rd != 5'b0) && (ex_mem1_rd == id_ex1_rs2))
                forward_b1 = 3'b010;
            else if (ex_mem_valid  && ex_mem_reg_write  && (ex_mem_rd  != 5'b0) && (ex_mem_rd  == id_ex1_rs2))
                forward_b1 = bypass_ok1 ? 3'b011 : 3'b001;
            else if (mem_wb1_reg_write && (mem_wb1_rd != 5'b0) && (mem_wb1_rd == id_ex1_rs2))
                forward_b1 = 3'b101;
            else if (mem_wb_reg_write  && (mem_wb_rd  != 5'b0) && (mem_wb_rd  == id_ex1_rs2))
                forward_b1 = 3'b100;
        end
    end

endmodule
