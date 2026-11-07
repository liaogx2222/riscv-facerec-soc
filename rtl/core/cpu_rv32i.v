// S5F: five-stage in-order dual-issue RV32I + Zmmul.
// Separate fetch/data channels, forwarding, store buffer and BTB/RAS.
// Registered pipeline state stalls on data-bus backpressure; RAM scope is
// decoded in parallel for each MEM lane before selecting load return data.
`timescale 1ns/1ps

`include "rv32i_defs.v"

module cpu_rv32i #(
    parameter BTB_ENABLE = 0,       // M2: 1=IF 级 BTB 预测; 0=纯 S2c (逐拍等价回退态)
    parameter S6B_ENABLE = 0,
    parameter IO_RAM_ADDR_W = 17
) (
    input  wire        clk,
    input  wire        resetn,

    // 双通道内存接口:
    //   F 通道: 取指, req(=valid 常1)/resp(电平, 桥侧队头==pc 即有效) + rdata;
    //           resp 由桥侧预取缓冲 (pf_buffer) 提供, 命中时几乎每拍可得;
    //           f_take = resp 有效拍被流水线消费 (冲刷拍也算消费), 桥据此弹出
    //   D 通道: 数据访问, 独立 2 拍 Valid-Ready 握手
    output wire        f_req,
    output wire [31:0] f_addr,
    output wire [31:0] f_redirect,    // S3: 早一拍重定向提示 (= 下一拍 pc)
    output wire        f_redirect_vld,// S6b: 本拍 f_redirect 有效 (= 下拍 pc 必为该值, 单发)
    output wire        f_registered_redirect_vld,
    output wire [31:0] f_registered_redirect_tgt,
    input  wire        f_resp_vld,
    input  wire [31:0] f_resp_data,
    // ★ 双发射取指: 队头的**下一个字** + 可用标志 (由 pf_buffer 提供)
    //   rv32i 下每个字就是一条指令 ⇒ {nxt, data} = 一拍两条指令
    input  wire [31:0] f_resp_data_nxt,
    input  wire        f_resp_have_nxt,
    // ★★ #3: 取指包第 2 条的**地址** (pf_buffer 给) —— 用于配对校验
    //   顺序包 = pc+4; "插入包" = BTB 在 pc 预测的跳转目标 T
    input  wire [31:0] f_resp_nxt_addr,
    output wire        f_take,
    // ★ D2b: 本拍同时消费取指队头的第 2 个字 (顺序双发射)
    output wire        f_take_nxt,
    //   D 通道: 数据访问 (读/写, 字节使能)
    output wire        d_valid,
    output wire        d_write,
    output wire [31:0] d_addr,
    output wire [31:0] d_load_addr, // read-data selection, independent of SB drain arbitration
    output wire        d_load_is_ram, // parallel lane classification, before the late address mux
    output wire [31:0] d_store_addr,
    output wire        d_store_is_ram,
    output wire [31:0] d_wdata,
    output wire [ 3:0] d_wstrb,
    input  wire        d_rdy,
    input  wire [31:0] d_rdata,
    // ★ IO 气泡握手 (2026-10-10): SoC 以 "~ram_sel" 驱动 —— 当前 D 通道占据者
    //   是非 RAM 区 (MMIO 外设) 访存时为高。用于 IO 退休次拍插 1 拍气泡 (见
    //   d_valid 生成处), 保证任意两次 IO 访存之间 d_valid 必有间隙。
    input  wire        io_active,
    // S6a-R 早一拍预读预告: adv 拍且 id_ex 为 load/store(即将被 EX/MEM 锁存)
    //   → dcache 端口B提前寄存读, 该操作 d_valid 首拍命中即出 rdy (1 拍访存).
    //   与 ex_mem 捕获条件严格同构; pre_a==d_addr 校验兜底, 最坏=不加速绝不出错.
    output wire        d_valid_pre,
    output wire [31:0] d_addr_pre,
    output wire [159:0] d_pretag_addresses,
    output wire [2:0] d_pretag_choice,

    // 中断输入 (M3b): 4 源 one-hot [0]SW [1]TIMER [2]UART_RX [3]KEY
    input  wire [ 3:0] irq,
    // 预留中断/异常接口 (基础任务暂不实现)
    input  wire        ext_irq,
    output wire        cpu_stall,

    // fence 钩子 (M1c): ID 级见 OP_FENCE 打一记全局 cache 无效化脉冲.
    //   误预测路径上的 fence 也会触发 —— write-through+纯无效化下只是
    //   多刷一次, 语义安全 (无脏行可丢).
    output reg         fence_flush,

    // ★★ #3: 预取器查询口 (BTB 的 `vf_*` 口, 基线里完全悬空)
    //   预取器在**发出请求 A 的那拍**要知道 BTB(A) 是否预测 taken, 才能决定
    //   icache 第 2 读口读 T 还是 A+4。**这只影响第 2 个字读哪里**;
    //   ★ §5f-3 R1c: 链步进已改为**跟随预测路径** (插入包 T+4 / slot1 预测 T1),
    //   与 CPU 取指流对齐 ⇒ 正确预测零 hint (旧"链恒顺序"注释作废)。
    input  wire [31:0] pf_lk_addr,
    output wire        pf_lk_taken,
    output wire [31:0] pf_lk_tgt,
    // ★ R1c-b: pf 的 slot1 前瞻 (BTB vf2 口, 查询 pf_lk_addr+4) —— pf 链随
    //   预测走需要看到顺序包 slot1 的预测, 否则 slot1 预测分支仍要 hint 清链。
    output wire        pf_lk1_taken,
    output wire [31:0] pf_lk1_tgt,
    // ★ DBG-1e: pf 前瞻查询口 (地址 = pf 的 next_issue) + 锁存前结果回传
    input  wire [31:0] pf_lk2_addr,
    output wire        pf_lk2_taken,
    output wire [31:0] pf_lk2_tgt,
    output wire        pf_lk3_taken,
    output wire [31:0] pf_lk3_tgt
);

    //-----------------------------------------------------------------
    // 流水线寄存器
    //-----------------------------------------------------------------
    // IF/ID
    reg        if_id_valid = 1'b0;
    reg [31:0] if_id_pc;
    reg [31:0] if_id_instr;
    // ★★ D2b: slot1 (取指包第 2 条 = pc+4)
    //   `if_id_pair=1` 才代表 instr1 有效 —— 否则它是 f_resp_data_nxt 的陈旧值
    //   (have_nxt=0 时该总线无意义), 必须靠这个位门掉。
    reg        if_id_pair = 1'b0;
    reg [31:0] if_id_instr1;
    // ★★ #3: slot1 的**真实 pc** —— 顺序包 = pc+4, 插入包 = T。
    //   (基线里 slot1 的 pc 隐含 = pc+4, 插入包下不成立; 影响 slot1 的 AUIPC
    //    与 pc_plus4, 也让诊断用的退休流显示错地址。)
    reg [31:0] if_id_pc1 = 32'h0;
    // M2 BTB 预测携带位 (随指令流水; 冲刷/插泡同拍清)
    reg        if_id_pred_taken = 1'b0;
    reg [31:0] if_id_pred_tgt = 32'h0;
    // ★★ P2 松绑 (2026-10-11 §5d): slot1 的 IF 级预测携带位 —— 仅「slot0
    //   未被预测 taken 且 slot1=pc+4」的**顺序包**里可能为 1 (插入包 slot1=T
    //   不预测, 见 pred_applied1; pair_now=0 时恒 0)。
    reg        if_id1_pred_taken = 1'b0;
    reg [31:0] if_id1_pred_tgt = 32'h0;

    // ID/EX (控制与数据占位, 下一任务由 ID 级接入)
    reg        id_ex_valid = 1'b0;
    reg [31:0] id_ex_pc;
    reg [31:0] id_ex_rs1_data;
    reg [31:0] id_ex_rs2_data;
    reg [31:0] id_ex_imm;
    reg [ 4:0] id_ex_rd;
    reg [ 4:0] id_ex_rs1;
    reg [ 4:0] id_ex_rs2;
    reg        id_ex_alu_src;
    reg        id_ex_alu_a_pc;         // AUIPC: ALU a 输入用 pc
    reg        id_ex_alu_a_zero;       // LUI: ALU a 输入强制 0 (U-type 无 rs1)
    reg [ 3:0] id_ex_alu_op;
    reg        id_ex_branch;
    reg        id_ex_jal;
    reg        id_ex_jalr;
    reg        id_ex_mem_read;
    reg        id_ex_mem_write;
    reg        id_ex_reg_write;
    reg        id_ex_mem_to_reg;
    reg [ 2:0] id_ex_funct3;         // EX 级分支条件 (funct3)
    reg [31:0] id_ex_mem_addr_pre;   // ★ 阶段0: ID 级投机访存地址, 下拍当 d_addr_pre
    // ---- M3a CSR/异常携带位 ----
    reg        id_ex_csr_en = 1'b0, id_ex_csr_imm = 1'b0;
    reg [ 1:0] id_ex_csr_mode = 2'b00;
    reg [11:0] id_ex_csr_addr = 12'h0;
    reg        id_ex_mret = 1'b0, id_ex_ecall = 1'b0, id_ex_ebreak = 1'b0;
    reg        id_ex_illegal = 1'b0;
    reg        id_ex_pred_taken = 1'b0;            // M2: EX 校验用
    reg [31:0] id_ex_pred_tgt = 32'h0;
    reg [30:0] id_ex_jbase0_q, id_ex_jbase1_q;

    // EX/MEM
    reg        ex_mem_valid = 1'b0;
    reg        ex_mem_is_mul = 1'b0;   // ★ MUL 晚结果 (v2 乘法器): 该指令的真值在 mul_result
    reg [31:0] ex_mem_alu_result;
    reg [31:0] ex_mem_store_data;
    reg [ 4:0] ex_mem_rd;
    reg [31:0] ex_mem_pc_plus4;
    reg        ex_mem_mem_read;
    reg        ex_mem_mem_write;
    reg        ex_mem_reg_write;
    reg        ex_mem_mem_to_reg;
    reg [ 2:0] ex_mem_funct3;        // 存取宽度/符号 (lb/lh/lw/lbu/lhu, sb/sh/sw)
    reg [ 1:0] ex_mem_byte_off;      // 字内字节偏移 (load 符号扩展用)
    reg [ 3:0] ex_mem_wstrb;         // 字内字节写使能 (store 用)
    reg        ex_mem_branch_taken;  // 打拍的分支命中 (下一拍重定向 PC)
    reg [31:0] ex_mem_branch_target; // 打拍的分支目标
    // ★★★ branch/jalr MEM 级解析携带位 (2026-10-11, 冲 5 §5):
    //   挂起分支在 EX 拍把「解析所需的一切」快照进 MEM。快照操作数对 load
    //   旁路位直接取 mem_load_ext (与转发码 3'b011 同源同拍 ⇒ 值正确),
    //   MEM 拍比较器输入全为寄存器 ⇒ "load 数据 → pc" 长链不重建。
    reg        ex_mem_br_pend = 1'b0;        // 挂起标志 (解析须 ex_mem_valid 同真)
    reg        ex_mem_br_is_jalr = 1'b0;     // 1=jalr (taken=1, 目标=op_a+imm); 0=branch
    reg [ 2:0] ex_mem_br_funct3 = 3'b0;      // branch 比较条件 (BEQ/BNE/BLT/BGE/BLTU/BGEU)
    reg [31:0] ex_mem_br_op_a = 32'h0;       // rs1 快照 (EX 转发值 / load 旁路值)
    reg [31:0] ex_mem_br_op_b = 32'h0;       // rs2 快照
    // ★★ 2026-10-14 (riscv-tests 发现的真 bug 修复): miss load 的旁路数据
    //   锁存寄存器。§5a 快照 = 分支 EX→MEM 边沿的瞬时 mem_load_ext (=d_rdata)。
    //   命中 load: 数据呈现拍 = adv 拍, 快照正确。**miss load**: d_rdy 拍数据
    //   才出现, 但该拍 adv 可能 =0 (miss 冻结期取指饥饿) ⇒ load 继续驻留 MEM,
    //   等 adv 拍锁快照时 dcache 输出已回退为旧值 ⇒ 分支解析用错操作数
    //   (rv32ui-p-lw case3 实测: 取到上一笔 load 的残留 0x00ff00ff)。
    //   修法: 只要 load 在 D 口呈现且 d_rdy=1 (**不管 adv**), 就把数据锁进
    //   mem_ld_lat; 快照 mux 优先用锁存值 —— 交付与 adv 同拍时 lat_v=0 仍走
    //   瞬态路径 (与原行为逐位一致), 提前交付时用锁存值 (修复本 bug)。
    reg  [31:0] mem_ld_lat = 32'h0;
    reg         mem_ld_lat_v = 1'b0;
    wire        ld_lat_cap = d_valid && d_rdy && !d_write &&
                             (ex_mem_mem_read || ex_mem1_mem_read);
    always @(posedge clk) begin
        if (!resetn) begin
            mem_ld_lat   <= 32'h0;
            mem_ld_lat_v <= 1'b0;
        end else if (ld_lat_cap && !adv) begin
            // 数据交付拍 adv=0 (miss 冻结) ⇒ 锁存, 供后续 adv 拍的快照取用。
            // d_rdy 拍的 mem_load_ext 即该 load 数据 (同源同拍)。
            mem_ld_lat   <= mem_load_ext;
            mem_ld_lat_v <= 1'b1;
        end else begin
            mem_ld_lat_v <= 1'b0;   // 每次推进/无交付都失效 (防跨 load 残留)
        end
    end
    reg [31:0] ex_mem_br_imm = 32'h0;        // 目标偏移 (imm)
    reg [31:0] ex_mem_br_pc = 32'h0;         // 分支自身 pc (目标基址 / BTB upd_pc)
    reg        ex_mem_br_pred_taken = 1'b0;  // IF 级预测 taken (比对基准)
    reg [31:0] ex_mem_br_pred_tgt = 32'h0;   // IF 级预测目标
    reg [ 1:0] ex_mem_br_btype = 2'b0;       // {ret, call} (BTB 延迟更新用)

    // MEM/WB
    reg        mem_wb_valid = 1'b0;
    reg [31:0] mem_wb_alu_result;
    reg [31:0] mem_wb_mem_rdata;
    // ★ 提速措施2: 在 MEM 级就把 load 值「移位+符号扩展」好并寄存。
    //   原写法把 32 位变长移位器 + 5 路扩展 mux 放在 mem_wb_mem_rdata **之后**，
    //   而该值经「寄存器堆写穿透旁路 → EX 转发 mux → EX 地址加法器 → d_addr_pre
    //   → dcache 阵列地址」一路组合下去，构成 14.6 ns 的关键路径（实测 68 MHz）。
    //   把这两级挪到寄存器**之前**，转发路径上就只剩纯 mux。
    reg [31:0] mem_wb_load_value;
    reg [31:0] mem_wb_pc_plus4;
    reg [ 4:0] mem_wb_rd;
    reg        mem_wb_reg_write;
    reg        mem_wb_mem_to_reg;
    reg [ 2:0] mem_wb_funct3;        // load 宽度/符号 (WB 级扩展)
    reg [ 1:0] mem_wb_byte_off;      // 字内偏移 (load 扩展)

    //=================================================================
    // ★★★ D2b (2026-10-08): 顺序双发射的 **slot1 流水线寄存器**
    //
    //   slot1 的配对规则 (逐步放宽史):
    //     基线    : 纯 ALU 写回 (P2/P3/P4/P5/P8 全禁) ⇒ MEM 级、D 口、
    //               dcache 预读、CSR/陷阱、分支重定向全部不用改
    //     P3'     : + 访存 (一对至多一条, P3 仍生效)
    //     §5c     : + MUL (第二乘法器 u_mul1, 结果晚一拍)
    //     §5d(本次): + 分支/跳转 (P2 松绑) —— slot1 BTB 第二查询口 (lk1)
    //               预测 + u_alu1 EX 级解析 + older-wins 重定向仲裁 (slot0
    //               程序序更老, 全部事件压过 slot1); load 相邻会挂起的
    //               slot1 分支在配对级拆对 (p_npend1, 挂起机制只支持 slot0)。
    //   ⇒ MEM 级、D 口、dcache 预读、CSR/陷阱沿用; 分支重定向扩展到 slot1。
    //=================================================================
    // ID/EX slot1
    reg        id_ex1_valid = 1'b0;
    reg [31:0] id_ex1_pc;
    reg [31:0] id_ex1_rs1_data;
    reg [31:0] id_ex1_rs2_data;
    reg [31:0] id_ex1_imm;
    reg [ 4:0] id_ex1_rd;
    reg [ 4:0] id_ex1_rs1;
    reg [ 4:0] id_ex1_rs2;
    reg        id_ex1_alu_src;
    reg        id_ex1_alu_a_pc;
    reg        id_ex1_alu_a_zero;
    reg [ 3:0] id_ex1_alu_op;
    reg        id_ex1_reg_write;
    // ★★ P3': slot1 允许访存
    reg        id_ex1_mem_read;
    reg        id_ex1_mem_write;
    reg        id_ex1_mem_to_reg;
    reg [ 2:0] id_ex1_funct3;
    reg [31:0] id_ex1_mem_addr_pre;
    // ★★ P2 松绑 (2026-10-11 §5d): slot1 分支/跳转字段 (P2 放行) + IF 预测携带位
    reg        id_ex1_branch = 1'b0;
    reg        id_ex1_jal = 1'b0;
    reg        id_ex1_jalr = 1'b0;
    reg        id_ex1_pred_taken = 1'b0;
    reg [31:0] id_ex1_pred_tgt = 32'h0;
    reg [30:0] id_ex1_jbase0_q, id_ex1_jbase1_q;

    // EX/MEM slot1 (纯 ALU 结果直通)
    reg        ex_mem1_valid = 1'b0;
    reg        ex_mem1_is_mul = 1'b0;  // ★ §5c slot1 MUL 晚结果标志 (真值在 mul_result1)
    reg [31:0] ex_mem1_alu_result;
    reg [ 4:0] ex_mem1_rd;
    reg        ex_mem1_reg_write;
    reg [31:0] ex_mem1_store_data;
    reg [31:0] ex_mem1_pc_plus4;
    reg        ex_mem1_mem_read;
    reg        ex_mem1_mem_write;
    reg        ex_mem1_mem_to_reg;
    reg [ 2:0] ex_mem1_funct3;
    reg [ 1:0] ex_mem1_byte_off;
    reg [ 3:0] ex_mem1_wstrb;

    // MEM/WB slot1
    reg        mem_wb1_valid = 1'b0;
    reg [31:0] mem_wb1_alu_result;
    reg [ 4:0] mem_wb1_rd;
    reg        mem_wb1_reg_write;
    reg [31:0] mem_wb1_load_value;
    reg [31:0] mem_wb1_pc_plus4;
    reg        mem_wb1_mem_read;
    reg        mem_wb1_mem_to_reg;
    wire        wb1_reg_write = mem_wb1_valid & mem_wb1_reg_write;
    wire [31:0] wb_payload0_q,wb_payload1_q;
    wire [31:0] wb1_wdata = wb_payload1_q;

    //-----------------------------------------------------------------
    // 内部连线
    //-----------------------------------------------------------------
    reg        pc_en       = 1'b0;  // PC 允许更新(非停顿)
    reg        if_id_clear = 1'b0;  // 冲刷 IF/ID
    reg        id_ex_clear = 1'b0;  // 冲刷 ID/EX
    reg [31:0] next_pc;
    reg [31:0] pc;                  // IF 级 PC

    // 分支/跳转 (由 EX 级 ALU branch_taken 驱动)
    wire       branch_taken;
    reg  [31:0] branch_target;

    // ★★★ branch/jalr MEM 级解析 (2026-10-11, 冲 5 §5): 前向声明
    //   load 相邻的 branch/jalr 在 EX 拍不解析 (投机走 BTB), 挂起到 MEM 拍用
    //   快照操作数比较解析; 失配走 MEM 级校正通路。驱动在「MEM 级解析」块
    //   (branch_taken 定义之后), 此处仅声明 (next_pc/flush/trap 门控在前文引用)。
    wire        ex_br_pend;        // EX 级判定: 本分支操作数需 load 旁路 ⇒ 挂起
    wire        mem_br_mismatch;   // MEM 级解析与 IF 预测失配 (校正事件, X 安全)
    // ★ §5f-2: syn_maxfan (fanout 37 → 官方复制优化, ADS 手册 5.13)
    (* syn_maxfan = 16 *) wire mem_br_take;  // X 安全后的解析 taken (校正目标选择用)
    wire [31:0] mem_br_tgt;        // 解析目标 (jalr=op_a+imm&~1; branch=pc+imm)

    // ★★ P2 松绑 (2026-10-11 §5d): slot1 分支/跳转 —— 前向声明
    //   (f_redirect_vld / flush_eff / next_pc 在驱动点之前引用)
    wire        branch_taken1;     // slot1 EX 级解析 taken (驱动在 slot1 分支块)
    reg  [31:0] branch_target1;    // slot1 解析目标 (branch/jal=pc+imm; jalr=op_a+imm&~1)
    wire        branch_taken_eff1; // slot1 EX 级校正事件 (older-wins: 让位 slot0 全部事件)
    wire        pred_applied1;     // slot1 IF 级预测应用 (仅配对顺序包, 驱动在 pair_now 处)
    reg         ex_redirect_q = 1'b0;
    reg         ex_redirect_lane1_q;
    reg [31:0]  ex_redirect_tgt0_q, ex_redirect_tgt1_q;

    // 转发控制 (★ D2b: 3 bit 编码 000=堆 001=EX/MEM.s0 010=EX/MEM.s1
    //                             011=load旁路 100=MEM/WB.s0 101=MEM/WB.s1)
    wire [2:0] forward_a, forward_b;
    wire pair_link0_q,pair_link1_q;
    wire [2:0] forward_a1, forward_b1;   // ★ D2b slot1
    // 停顿/冲刷
    wire       load_use_stall;   // Load-Use: ID/EX 插 bubble
    wire       mem_stall;        // 内存忙: 保持全部
    wire       hazard_flush;

    // ID 级控制信号 (由 ctrl.v 驱动)
    wire       id_alu_src;
    wire       id_alu_a_pc;
    wire       id_alu_a_zero;
    wire [ 3:0] id_alu_op;
    wire       id_branch;
    wire       id_jal;
    wire       id_jalr;
    wire       id_mem_read;
    wire       id_mem_write;
    wire       id_reg_write;
    wire       id_mem_to_reg;
    wire       id_illegal;
    wire       id_csr_en, id_csr_imm, id_mret, id_ecall, id_ebreak;
    wire [ 1:0] id_csr_mode;

    // ID 级读出的寄存器数据 (由 regfile.v 驱动)
    wire [31:0] id_rs1_data;
    wire [31:0] id_rs2_data;
    wire [31:0] id_imm;              // 由 imm_gen.v 驱动

    // ★★★ 阶段0 / Part A: ID 级投机访存地址 (寄存器化)
    //   目的: 让 dcache 的「提前一拍预告读」拿到**寄存器输出**的地址, 从而
    //   既保留「命中 1 拍」, 又切断旧的组合长路径:
    //     WB 寄存器 → 寄存器堆旁路 → EX 转发 mux → EX 加法器 → d_addr_pre
    //     → dcache 阵列地址   (实测 14.627 ns, 68.37 MHz 的根因)
    //   改为: ID 级算好 → 寄存进 ID/EX → EX 拍稳定输出给阵列译码 (≈4~5 ns)。
    //   ⚠ 本地址是**投机值**: 若 EX 级因转发改变了 rs1, 则它与真实地址不符,
    //     dcache 的 (pre_a == d_addr) 门会否决快路, 自动落回 2 拍慢路
    //     ⇒ 最坏 = 改动前, 回退安全是构造性的。
    //   字对齐与 EX 的 mem_addr_word 严格同构 (窄访存 lb/sb 也能命中快路)。
    //
    // ★★ 阶段2 (2026-10-08): **ID 级转发增强** —— 提高投机地址命中率
    //   背景: 实测快路命中率只有 ~62%, 剩余 38% 失配落回 2 拍慢路
    //   (dport_wait 32,472 拍 = 7.6%, 是全表最大单项)。失配根因是
    //   EX 级 rs1 被转发改写, 而 ID 期用的是寄存器堆旧值。
    //   做法: 消费者 C 在 ID 时, **当前 EX/MEM 里的指令 Q** 会在下一拍进
    //   MEM/WB, 而那一拍 C 正好在 EX —— C 的转发源就是 **Q 的写回值**。
    //   这个值在 ID 期就能算出来 (Q 的 ex_mem_* 就是寄存器) ⇒ 投机更准。
    //   ⚠ 2026-10-08 阶段2 后续: **把"Q 是 load"也纳入**（原为省时序而排除）。
    //     收益: 指针追逐型代码里基址常来自上一条 load, 这类失配占剩余的相当比例。
    //     代价: `mem_load_ext` 会被接到 ID 路径上 (dcache 阵列 → 移位/扩展 mux →
    //     q_wb_val mux → ID 加法器 → ID/EX 寄存器)。该路径**终点是寄存器**,
    //     不构成 "dcache → pc" 的组合链, 但**会加长 ID 级时序** —— 属"性能优先、
    //     时序后收"的取舍。
    //     有效性: mem_load_ext 在 d_rdy=1 时有效; 该拍 adv=1 才会锁存 ID/EX,
    //     慢路/miss 时 adv=0 ⇒ 预测值不会被锁存 ⇒ 安全。
    //   安全性不变: 预测错只是失配, dcache 的 (pre_a == d_addr) 门照旧否决。
    wire [ 4:0] if_id_rs1_idx = if_id_instr[19:15];
    // ★ MUL 晚结果: 前向声明 (供 id_q_fwd_val / ex_mem_fw_val / WB 锁存; 驱动在 u_mul)
    wire [31:0] mul_result;
    // ★ slot1 MUL (2026-10-11 §5c, P4a 放行): 第二乘法器实例的结果
    //   (前向声明; 驱动在 u_mul1, 机制与 slot0 MUL_LATE 完全镜像)
    wire [31:0] mul_result1;
    wire [31:0] alu_pred_addr0, alu_pred_addr1;
    wire        id_q_fwd_en   = ex_mem_reg_write && (ex_mem_rd != 5'b0) &&
                                (ex_mem_rd == if_id_rs1_idx);
    wire [31:0] id_q_fwd_val  = ex_mem_is_mul ? mul_result :
                                ex_mem_mem_to_reg
                              ? (ex_mem_mem_read ? mem_load_ext : ex_mem_pc_plus4)
                              : ex_mem_alu_result;
    // ★★★ 2026-10-09 关键修复: **补上"1 条前"(ID/EX) 的转发预测**。
    //   实测: 慢路 27,347 次里 **99.3% 是 `pre_a != d_addr`**(ID 投机地址失配),
    //   只有 278 次是"没预告"。根因就在这条链的**优先级搞错了阶段**:
    //     消费者 C 在 ID(T) ⇒ C 的 EX 在 T+1 ⇒ 那一拍 EX 级转发的**最高优先源**
    //     是"1 条前"的指令 (T 时在 ID/EX, T+1 时在 EX/MEM), 而"2 条前"的
    //     (T 时在 EX/MEM) 只排第二。
    //   原代码只看了 `ex_mem_*`(=2 条前), 一旦"1 条前"写了同一个寄存器,
    //   预测就用了**过期值** ⇒ 地址失配 ⇒ 落回 2 拍慢路。
    //   而"1 条前"的写回值在 T 拍**组合可得** (`alu_result` / `mem_load_ext` /
    //   `pc+4`), 与 EX 级真正会转发的东西**逐位同源** ⇒ 预测精确。
    //   ⚠ 取舍: `alu_result` 被接进 ID 加法器 ⇒ ID 级时序加长 (终点是寄存器,
    //     不构成 "dcache→pc" 组合链)。按"先跑分后时序"先拿下收益。
    wire [31:0] id_ex_fw_val = id_ex_mem_read
                             ? mem_load_ext
                             : (id_ex_mem_to_reg ? (id_ex_pc + 32'd4) : alu_pred_addr0);
    wire        id_a_fwd_en  = id_ex_valid && id_ex_reg_write && (id_ex_rd != 5'b0) &&
                               (id_ex_rd == if_id_rs1_idx);
    // ★★ 再补 **slot1 生产者**: EX 级转发优先级是
    //     id_ex1 > id_ex0 > ex_mem1 > ex_mem0 > MEM/WB(寄存器堆旁路已覆盖)
    //   上一版只补了 id_ex0 与 ex_mem0 ⇒ slot1 写同一寄存器时预测仍旧失配。
    wire [31:0] id_ex1_fw_val = id_ex1_mem_read ? mem_load_ext
                              : (id_ex1_mem_to_reg ? (id_ex1_pc + 32'd4) : alu_pred_addr1);
    wire        id_a1_fwd_en  = id_ex1_valid && id_ex1_reg_write && (id_ex1_rd != 5'b0) &&
                                (id_ex1_rd == if_id_rs1_idx);
    wire [31:0] id_m1_fw_val  = ex_mem1_mem_read ? mem_load_ext : ex_mem1_fw_val;
    // ★ 刀A3 实验结论 (2026-10-16, 已回滚): 撤 m1 load 分支实测 +7,443 ticks
    //   且综合后 WNS 反而恶化 (−7.40→−7.72, 消费端撤除后综合重排使 pf 环
    //   next_issue 链变长为新榜首) —— 双指标净负, 故恢复 m1 原始快路径。
    //   结论: AGU 预测链 (−7.40) 与 pf next_issue 环是并列长链, 单撤一端
    //   无收益; 后续需先切 pf 环 (§5f-1 遗留的 next_issue→BTB→issue_next)。
    wire        id_m1_fwd_en  = ex_mem1_valid && ex_mem1_reg_write && (ex_mem1_rd != 5'b0) &&
                                (ex_mem1_rd == if_id_rs1_idx);
    wire [31:0] id_rs1_pred   = id_a1_fwd_en ? id_ex1_fw_val
                              : id_a_fwd_en  ? id_ex_fw_val
                              : id_m1_fwd_en ? id_m1_fw_val
                              : id_q_fwd_en  ? id_q_fwd_val
                              :                id_rs1_data;

    // Only predict the 128 KiB RAM window. This is a speculative pre-read,
    // never the architectural effective address: dcache checks the complete
    // pre_a against d_addr before using its result. Projecting modulo 2^17
    // preserves every RAM address and shortens the late forwarding -> ID AGU
    // carry chain. Addresses outside this window safely use the slow path.
    wire [16:0] id_mem_addr_low = id_rs1_pred[16:0] + id_imm[16:0];
    wire [31:0] id_mem_addr_sum = {15'b0, id_mem_addr_low};
    wire [31:0] id_mem_addr     = {id_mem_addr_sum[31:2], 2'b00};

    //-----------------------------------------------------------------
    // ID 级子模块: ctrl / imm_gen / regfile
    //-----------------------------------------------------------------
    ctrl u_ctrl (
        .instr     (if_id_instr),
        .alu_src   (id_alu_src),
        .alu_a_pc  (id_alu_a_pc),
        .alu_a_zero(id_alu_a_zero),
        .alu_op    (id_alu_op),
        .branch    (id_branch),
        .jal       (id_jal),
        .jalr      (id_jalr),
        .mem_read  (id_mem_read),
        .mem_write (id_mem_write),
        .reg_write (id_reg_write),
        .mem_to_reg(id_mem_to_reg),
        .illegal   (id_illegal),
        .csr_en    (id_csr_en), .csr_imm(id_csr_imm), .csr_mode(id_csr_mode),
        .sys_mret  (id_mret), .sys_ecall(id_ecall), .sys_ebreak(id_ebreak)
    );

    imm_gen u_imm (
        .instr(if_id_instr),
        .imm  (id_imm)
    );

    //-----------------------------------------------------------------
    // ★★ D2b: slot1 的 ID 级子模块 (只取纯 ALU 需要的几根)
    //-----------------------------------------------------------------
    wire       id1_alu_src, id1_alu_a_pc, id1_alu_a_zero, id1_reg_write;
    wire       id1_mem_read, id1_mem_write, id1_mem_to_reg;
    // ★ P2 松绑 (§5d): slot1 的控制流译码输出 (原空接 —— P2 禁分支时代的遗留)
    wire       id1_branch, id1_jal, id1_jalr;
    wire [3:0] id1_alu_op;
    wire [31:0] id1_imm;
    wire [31:0] id1_rs1_data, id1_rs2_data;

    ctrl u_ctrl1 (
        .instr     (if_id_instr1),
        .alu_src   (id1_alu_src),
        .alu_a_pc  (id1_alu_a_pc),
        .alu_a_zero(id1_alu_a_zero),
        .alu_op    (id1_alu_op),
        .branch    (id1_branch), .jal(id1_jal), .jalr(id1_jalr),   // ★ §5d: P2 放行
        .mem_read  (id1_mem_read), .mem_write(id1_mem_write),
        .reg_write (id1_reg_write),
        .mem_to_reg(id1_mem_to_reg), .illegal(), .csr_en(), .csr_imm(), .csr_mode(),
        .sys_mret(), .sys_ecall(), .sys_ebreak()
    );

    imm_gen u_imm1 (
        .instr(if_id_instr1),
        .imm  (id1_imm)
    );

    // ★★ P3': slot1 投机访存地址 (与 slot0 同构)
    wire [ 4:0] if_id1_rs1_idx = if_id_instr1[19:15];
    wire        id1_q_fwd_en   = ex_mem_reg_write && (ex_mem_rd != 5'b0) &&
                                 (ex_mem_rd == if_id1_rs1_idx);
    // ★ 同 slot0: 补"1 条前"(ID/EX.slot0) 的转发预测
    wire [31:0] id1_ex_fw_val = id_ex_mem_read
                              ? mem_load_ext
                              : (id_ex_mem_to_reg ? (id_ex_pc + 32'd4) : alu_pred_addr0);
    wire        id1_a_fwd_en  = id_ex_valid && id_ex_reg_write && (id_ex_rd != 5'b0) &&
                                (id_ex_rd == if_id1_rs1_idx);
    wire [31:0] id1_ex1_fw_val = id_ex1_mem_read ? mem_load_ext
                               : (id_ex1_mem_to_reg ? (id_ex1_pc + 32'd4) : alu_pred_addr1);
    // ★ §5f-2 刀A2: EX1 是 load 时**不放行** —— "dcache 读→SB CAM→d_rdata→
    //   mem_load_ext→slot1 AGU→pre" 是实测榜首 (−7.98, 17.2ns 全组合链)。
    //   该情形 slot1 配对拆开 (走慢路), 代价 = load 相邻 slot1 访存次数 ×1 拍
    //   (DBG 实测另见台账)。若未来要找回: 需 load 数据打拍版转发, 勿直接恢复。
    wire        id1_a1_fwd_en  = id_ex1_valid && id_ex1_reg_write && !id_ex1_mem_read &&
                                 (id_ex1_rd != 5'b0) &&
                                 (id_ex1_rd == if_id1_rs1_idx);
    wire [31:0] id1_m1_fw_val  = ex_mem1_mem_read ? mem_load_ext : ex_mem1_fw_val;
    wire        id1_m1_fwd_en  = ex_mem1_valid && ex_mem1_reg_write && (ex_mem1_rd != 5'b0) &&
                                 (ex_mem1_rd == if_id1_rs1_idx);
    // ★★★ L1 (2026-10-09): slot1 投机地址**漏了最高优先源** —— 实测 11,113 次
    //   `pre_a != d_addr` 失配**全部**是 slot1 访存, 典型现场:
    //     slot0 = `lui x15,4` @0x1f00 ; slot1 = `lw x10,-0x2b4(x15)` @0x1f04
    //   预测取了 ID/EX slot1 (2 条前) 的旧值 ⇒ 地址算成 0x1c5c (应 0x3d4c)。
    //
    //   ★ 对齐依据 (forwarding_unit 的 slot1 优先级):
    //       id_ex(1条前) > ex_mem1(2条前) > ex_mem(3条前) > mem_wb1(4) > mem_wb(5)
    //     slot1 的**1 条前** = 同一取指包里的 slot0 —— 它在预测拍 (N) 还在 IF/ID,
    //     到 N+1 才进 ID/EX, 其 EX 结果就是本拍 `alu_result`。
    //     ⇒ 预测拍只能**廉价地"预演"** slot0 的 ALU 结果 (仅 ALU_ADD 族):
    //         lui   → imm            (alu_a_zero)
    //         auipc → pc+imm         (alu_a_pc)
    //         addi/add → rs1+imm     (= id_mem_addr_sum)
    //         jal/jalr → **pc+4**     (转发值不是跳转目标!)
    //     其余 (load/mul/csr/非 ADD) 不预测 ⇒ 退回原链 (最坏 = 改动前)。
    //   ⚠ 只支持**立即数型** (I/U/J 型): R 型 `add` 的第 2 操作数是 rs2 而非 imm,
    //     `id_imm` 在 R 型下是 rs2 的位域 ⇒ 会算错。故排除 opcode=0x33 (OP)。
    wire        id_q_is_alu   = id_reg_write && (id_alu_op == `ALU_ADD) &&
                                !id_mem_read && !id_csr_en &&
                                (if_id_instr[6:0] != 7'b0110011);
    wire [31:0] id_q_res      = (id_jal | id_jalr) ? (if_id_pc + 32'd4)
                              : id_alu_a_zero      ? id_imm
                              : id_alu_a_pc        ? (if_id_pc + id_imm)
                              :                      id_mem_addr_sum;
    wire        id1_q0_fwd_en = id_q_is_alu && if_id_valid &&
                                (if_id_instr[11:7] != 5'b0) &&
                                (if_id_instr[11:7] == if_id1_rs1_idx);
    // ★ §5f-2 刀A2 (2026-10-16): q0 源 (slot0 AGU 输出直通 slot1 AGU) 撤除
    //   —— DBG-V2 实测 +503 ticks (近零), 切断 "slot0 AGU→slot1 AGU" 两级
    //   串联组合链 (L1 的 q0 快路径在 O3 固件下几乎不再触发)。
    wire [31:0] id1_rs1_pred   = id1_a1_fwd_en ? id1_ex1_fw_val
                               : id1_a_fwd_en  ? id1_ex_fw_val
                               : id1_m1_fwd_en ? id1_m1_fw_val
                               : id1_q_fwd_en  ? id_q_fwd_val
                               :                 id1_rs1_data;
    wire [16:0] id1_mem_addr_low = id1_rs1_pred[16:0] + id1_imm[16:0];
    wire [31:0] id1_mem_addr_sum = {15'b0, id1_mem_addr_low};
    wire [31:0] id1_mem_addr     = {id1_mem_addr_sum[31:2], 2'b00};

    regfile u_rf (
        .clk    (clk),
        .resetn (resetn),
        .we     (wb_reg_write),      // 写回使能 (MEM/WB slot0)
        .waddr  (mem_wb_rd),
        .wdata  (wb_wdata),          // 写回数据
        .raddr1 (if_id_instr[19:15]),
        .raddr2 (if_id_instr[24:20]),
        .rdata1 (id_rs1_data),
        .rdata2 (id_rs2_data),
        // ★ D2b: 写口 1 = WB slot1; 读口 3/4 = ID slot1
        .we1    (wb1_reg_write), .waddr1(mem_wb1_rd), .wdata1(wb1_wdata),
        .raddr3 (if_id_instr1[19:15]), .raddr4(if_id_instr1[24:20]),
        .rdata3 (id1_rs1_data), .rdata4(id1_rs2_data)
    );

    //-----------------------------------------------------------------
    // 转发单元: EX 级 ALU 输入 (消除数据冒险)
    //-----------------------------------------------------------------
    // Register the complete forwarding selection with its ID/EX consumer.
    // pc_en && !id_ex_clear implies adv: both producer tag stages advance.
    // A clear gives zero-tag consumers; without clear/pc_en all stages hold.
    // Preserve exact future MEM validity gates, including an EX0 redirect
    // which suppresses MEM1 one cycle before the front-end redirect flush.
    reg [4:0] fwd_id_rs1,fwd_id_rs2,fwd_id1_rs1,fwd_id1_rs2,fwd_id_rd;
    reg fwd_id_ctrl,fwd_id_reg_write;
    wire fwd_id1_valid = if_id_valid & if_id_pair;
    always @(*) begin
        fwd_id_rs1=0;fwd_id_rs2=0;fwd_id1_rs1=0;fwd_id1_rs2=0;
        fwd_id_rd=0;fwd_id_ctrl=0;fwd_id_reg_write=0;
        if(if_id_valid)begin
            fwd_id_rs1=if_id_instr[19:15];fwd_id_rs2=if_id_instr[24:20];
            fwd_id_rd=if_id_instr[11:7];fwd_id_ctrl=id_branch|id_jalr;
            fwd_id_reg_write=id_reg_write;
        end
        if(if_id_valid & if_id_pair)begin
            fwd_id1_rs1=if_id_instr1[19:15];fwd_id1_rs2=if_id_instr1[24:20];
        end
    end
    wire fwd_mem0_valid_next = id_ex_valid && ~ex_mem_branch_taken
                                 && !trap_fire && !mem_br_mismatch && !ex_redirect_q;
    wire fwd_mem1_valid_next = id_ex1_valid & ~branch_taken_eff
                                  & ~trap_fire & ~mret_fire & ~mem_br_mismatch & ~ex_redirect_q;
    wire [2:0] fwd_next_a,fwd_next_b,fwd_next_a1,fwd_next_b1;
    reg [11:0] forward_select_q;
    always @(posedge clk) begin
        if(!resetn || id_ex_clear) forward_select_q <= 12'b0;
        else if(pc_en) forward_select_q <= {fwd_next_a,fwd_next_b,fwd_next_a1,fwd_next_b1};
    end
    assign {forward_a,forward_b,forward_a1,forward_b1} = forward_select_q;
    forwarding_unit u_fwd_next (
        .id_ex_rs1(fwd_id_rs1),
        .id_ex_rs2(fwd_id_rs2),
        .ex_mem_rd(id_ex_rd),
        .ex_mem_reg_write(id_ex_reg_write),
        .ex_mem_mem_read(id_ex_mem_read),
        .ctrl_alu(fwd_id_ctrl),
        .mem_wb_rd(ex_mem_rd),
        .mem_wb_reg_write(ex_mem_reg_write),
        .id_ex1_rs1(fwd_id1_rs1),
        .id_ex1_rs2(fwd_id1_rs2),
        .id_ex1_valid(fwd_id1_valid),
        .id_ex_rd(fwd_id_rd),
        .id_ex_reg_write(fwd_id_reg_write),
        .ex_mem1_rd(id_ex1_rd),
        .ex_mem1_reg_write(id_ex1_reg_write),
        .ex_mem1_mem_read(id_ex1_mem_read),
        .ex_mem_valid(fwd_mem0_valid_next),
        .ex_mem1_valid(fwd_mem1_valid_next),
        .mem_wb1_rd(ex_mem1_rd),
        .mem_wb1_reg_write(ex_mem1_reg_write),
        .forward_a(fwd_next_a),
        .forward_b(fwd_next_b),
        .forward_a1(fwd_next_a1),
        .forward_b1(fwd_next_b1)
    );

    //-----------------------------------------------------------------
    // 冒险单元: Load-Use 停顿 + 分支冲刷
    //   ★ 2026-10-11 (冲 5 §5): branch/jalr 的 load-use 停顿已取消 ——
    //   load 相邻分支改走「EX 挂起 + MEM 级解析」(ex_br_pend / mem_br_mismatch,
    //   见下方 MEM 级解析块), hazard_unit 里 if_id_ctrl_alu 例外已删除。
    //-----------------------------------------------------------------
    hazard_unit u_hazard (
        .id_ex_rd      (id_ex_rd),
        .id_ex_mem_read(id_ex_mem_read),
        .id_ex1_rd     (id_ex1_rd),
        .id_ex1_mem_read(id_ex1_mem_read),
        .if_id_rs1     (if_id_instr[19:15]),
        .if_id_rs2     (if_id_instr[24:20]),
        .branch_taken  (ex_mem_branch_taken),   // 打拍版, 与 PC 重定向同拍
        .mem_busy      (mem_busy),
        .load_use_stall(load_use_stall),
        .mem_stall     (mem_stall),
        .flush         (hazard_flush)
    );

    //-----------------------------------------------------------------
    // 双通道呈现 + 统一推进使能
    //   adv = 取指退休 且 数据通道空闲/退休 —— 全流水线原子推进,
    //   杜绝 "后级走、前级停" 造成的指令重复/丢失 (单端口病根).
    //   hazard_unit 的 mem_busy = 数据通道占用 (冻结 load-use/flush 清零,
    //   防止压在 ID/EX 的 load 被 bubble 抹掉).
    //-----------------------------------------------------------------
    // -----------------------------------------------------------------
    // F 通道前端: 请求恒有效, 桥侧预取缓冲给电平 resp
    //   f_take: resp 有效且本拍流水线装载 IF/ID (front_en) 或分支重定向
    //   (错路指令被冲刷也算消费, 弹出后不再回看) —— 桥据其弹出队头.
    // -----------------------------------------------------------------
    assign f_req  = 1'b1;
    assign f_addr = pc;
    assign f_take = f_resp_vld & pc_en;
    // ★ D2b: `f_take_nxt` 在下面 pair_now 定义处驱动 (需 flush_eff/lu_eff)
    // S6b: 预测/校正均为"退休拍单发"事件 (f_take/adv 天然门控——S3 反复发
    //   hint 的教训由事件结构规避). hint 值=next_pc, 与本拍末 pc 锁存值严格
    //   一致; S6B_ENABLE=0 时 vld 恒 0, pf 走原 resync 路 (逐拍零差异).
    assign f_redirect     = next_pc;
    // EX/MEM corrections already cross a register boundary. Expose only
    // those stable register values so SoC need not delay their hint again.
    assign f_registered_redirect_vld = (S6B_ENABLE != 0) &&
        ((ex_redirect_q === 1'b1) || (mm_pend_q === 1'b1));
    assign f_registered_redirect_tgt = (mm_pend_q === 1'b1) ? mm_tgt_q :
        (ex_redirect_lane1_q ? ex_redirect_tgt1_q : ex_redirect_tgt0_q);
    //   ⚠⚠⚠ **必须 X 安全 (2026-10-08 实测踩中, 死锁级)**:
    //     `branch_taken_eff` 在"EX 级是 branch 且其 id_ex_rs1_data 为 X"时是 X,
    //     而 `pred_applied | branch_taken_eff` 会把 X 一路带到 `req_hint` →
    //     pf_buffer 的 `hint_fire` → `iss_addr` 的三目 ⇒ **iss_addr = X**
    //     ⇒ `next_issue` 被永久锁成 X ⇒ 取指链彻底死掉 (实测 pc 卡在 0xcf0)。
    //     用 `=== 1'b1` 把 X 归到"不发 hint" ⇒ 退回 S6B=0 的 resync 通路,
    //     功能完全等价(只是少一次提前直取), 安全。
    //   ★ 2026-10-11: MEM 级分支校正 (mem_br_mismatch) 同为单发重定向事件,
    //     一并作为 hint (值 = next_pc, 与本拍末 pc 锁存值严格一致)。
    //   ★ 2026-10-11 §5d (P2 松绑): slot1 的两个单发重定向事件一并并入 ——
    //     pred_applied1 (slot1 预测跳转, 配对顺序包) 与 branch_taken_eff1
    //     (slot1 EX 校正)。
    assign f_redirect_vld = (S6B_ENABLE != 0) && pc_en
                            && ((pred_applied === 1'b1) || (pred_applied1 === 1'b1)
                                || (ex_redirect_q === 1'b1)
                                || (mm_pend_q === 1'b1));   // ★ 刀B: 失配校正晚 1 拍进 pf (与 pc 对齐)

    // ★★ P3': D 口由 slot0/slot1 中那条访存指令驱动 (P3 保证最多一条)
    // ★★★ 2026-10-09 修复 (既有 bug, P3' 时代遗留, 双发射放大后必踩):
    //   `mem0_act`/`mem1_act` 原本**不含 valid**。EX/MEM 被 `branch_taken_eff`
    //   压掉时 valid=0 但 `mem_read/mem_write` 字段仍是**被冲刷指令的旧值**
    //   ⇒ `mem0_act=1` 而 `ex_mem_valid=0` ⇒ `mem_sel1 = ~1 & 1 = 0`
    //   ⇒ **slot1 的真实访存被套上了 slot0(废指令)的地址/数据/字节使能**。
    //   实测 (smoke.hex): `lw x1,44(x2)` 读到 0x30xxxxxx (只有 1 个字节有定义)
    //   ⇒ ra 变 X ⇒ `ret` 目标 X ⇒ pc 全 X ⇒ 死锁。
    //   ⇒ 修法: 两个 `_act` 都带上各自的 valid。语义完全等价于
    //     `d_valid = mem0_act | mem1_act`, 且 `mem_sel1` 不会再选到废指令。
    wire mem0_act  = ex_mem_valid  & (ex_mem_mem_read  | ex_mem_mem_write);
    wire mem1_act  = ex_mem1_valid & (ex_mem1_mem_read | ex_mem1_mem_write);
    wire mem_data_active = mem0_act | mem1_act;
    wire mem_sel1  = ~mem0_act & mem1_act;
    // 当前占据者的 load/store 属性 (P3 保证一对至多一条访存)
    wire cur_is_store = mem_data_active & (mem_sel1 ? ex_mem1_mem_write : ex_mem_mem_write);
    wire cur_is_load  = mem_data_active & ~cur_is_store;

    //-----------------------------------------------------------------
    // ★★★ branch/jalr 的 MEM 级解析 —— 判定本体 (2026-10-11 冲 5 §5c 前移)
    //   挂起分支在 MEM 拍用快照操作数比较解析。判定 = **纯组合** (全快照
    //   寄存器源), **不含 adv** —— MEM 驻留第一拍 (mem_br_mism_c) 即有效:
    //     · 用途①: 错路配对 load 的 D 口压制 (mem1_ld_supp — §5f-2 刀1 已撤除)
    //       —— 否则 d_valid&d_rdy 可在 adv=0 拍 (f_resp_vld=0 取指饥饿)
    //       提前完成访存, MMIO 读副作用不可撤回;
    //     · 用途②: adv 拍的 mem_br_mismatch 校正事件 (冲刷/重定向/BTB,
    //       在 adv 定义处赋值)。
    //   · branch: funct3 语义比较 (复制 alu.v 分支语义, 输入全寄存器);
    //   · jalr  : taken=1, 目标 = (快照 rs1 + imm) & ~1。
    //   与 IF 级预测 (ex_mem_br_pred_*) 比对: 一致 → 照常写回 + BTB 延迟
    //   更新; 失配 → 校正 (trap/mret/CSR 让位, older-wins)。
    //   X 安全: 判定值 X 时按"已知才压制/校正/更新"归 0 (与 EX 级
    //   branch_taken X 语义一致: X ⇒ 沿预测路径继续)。
    //-----------------------------------------------------------------
    wire [32:0] mem_br_dif = {1'b0, ex_mem_br_op_a} - {1'b0, ex_mem_br_op_b};
    wire        mem_br_eq  = (ex_mem_br_op_a == ex_mem_br_op_b);
    wire        mem_br_lt  = (ex_mem_br_op_a[31] & ~ex_mem_br_op_b[31]) |
                             (~(ex_mem_br_op_a[31] ^ ex_mem_br_op_b[31]) & mem_br_dif[31]);
    wire        mem_br_ltu = mem_br_dif[32];
    reg         mem_br_cmp;
    always @(*) begin
        case (ex_mem_br_funct3)
            `F3_BEQ:  mem_br_cmp = mem_br_eq;
            `F3_BNE:  mem_br_cmp = ~mem_br_eq;
            `F3_BLT:  mem_br_cmp = mem_br_lt;
            `F3_BGE:  mem_br_cmp = ~mem_br_lt;
            `F3_BLTU: mem_br_cmp = mem_br_ltu;
            `F3_BGEU: mem_br_cmp = ~mem_br_ltu;
            default:  mem_br_cmp = 1'b0;   // funct3=010/011 非法编码 (同 alu.v)
        endcase
    end
    wire        mem_br_taken_raw = ex_mem_br_is_jalr |
                                   (~ex_mem_br_is_jalr & mem_br_cmp);
    wire        mem_br_take1  = (mem_br_taken_raw === 1'b1);          // X 安全
    wire        mem_br_known  = mem_br_take1 | (mem_br_taken_raw === 1'b0);
    wire [31:0] mem_br_tgt_raw = ex_mem_br_is_jalr
                               ? ((ex_mem_br_op_a + ex_mem_br_imm) & ~32'h1)
                               : (ex_mem_br_pc + ex_mem_br_imm);
    assign mem_br_take = mem_br_take1;
    assign mem_br_tgt  = mem_br_tgt_raw;
    wire        mem_br_mism_cmp = mem_br_known &
        (mem_br_take1 ? (~ex_mem_br_pred_taken || (ex_mem_br_pred_tgt != mem_br_tgt_raw))
                      : ex_mem_br_pred_taken);
    wire        mem_br_mism_c = ex_mem_valid && ex_mem_br_pend &&
                                (mem_br_mism_cmp === 1'b1);
    //-----------------------------------------------------------------
    // ★★★ Store Buffer (阶段 A, 2026-10-10, 设计: doc/CPU_乱序双发_设计方案_v1.md §2)
    //   store 退休即入队 (不再占 D 口等 d_rdy) ⇒ D 口让给 load ⇒ dport_wait 的
    //   store 侧消失。SB 满时冻结推进 (构造性回退, 排水腾位后自动恢复)。
    //   正确性三规则:
    //     ① load 命中 SB (字地址 CAM): 全字写单命中 → 转发 (不上 D 口, 1 拍完成);
    //        部分写/多命中 → 冻结等该项排空后再正常上 D 口 (write-through 下
    //        SB 未排空 = dcache/RAM 旧数据, 不等必读脏)。
    //     ② SB 有未排空 IO store ⇒ 所有 load 等待 (MMIO 写读副作用顺序)。
    //     ③ 排水按 FIFO 序; 等待中的 load 不挡排水 ⇒ 无死锁。
    //   顺序性: 顺序流水 ⇒ SB 内全部是比 MEM 占据者更老的 store, 等待读到的
    //   就是 load 程序点应有的值 (无 RAW/WAW 乱序问题; C 阶段重命名后须重审)。
    //   异常/中断: SB 是提交后缓冲 (store 已架构性退休), 陷阱/中断不冲刷 SB,
    //   M3a/M3b 语义零改动。fence 的 cache 无效化与 SB 无交互 (write-through
    //   无脏行, 排水晚到走原写穿路径)。
    //-----------------------------------------------------------------
    // ★ 二分开关 (仅仿真调试, 综合时不存在): -DSB_BYPASS ⇒ store 直通 D 口 (旧行为),
    //   SB 只保留寄存器但不接管 D 口、不入队 (用于隔离"SB 机制"与"其它交互")
`ifdef SB_BYPASS
    localparam SB_ON = 0;
`else
    localparam SB_ON = 1;
`endif
    localparam SB_DEPTH = 4;
    reg [31:2] sb_addr  [0:SB_DEPTH-1];
    reg [31:0] sb_data  [0:SB_DEPTH-1];
    reg [ 3:0] sb_strb  [0:SB_DEPTH-1];
    reg        sb_iof   [0:SB_DEPTH-1];
    reg        sb_val   [0:SB_DEPTH-1];
    reg [ 1:0] sb_wp, sb_rp;
    reg [ 2:0] sb_cnt;
    wire sb_full    = (sb_cnt == 3'd4);
    wire sb_empty   = (sb_cnt == 3'd0);
    // 任一未排空 IO store (IO 写读顺序等待用)
    wire sb_has_io  = (sb_val[0] & sb_iof[0]) | (sb_val[1] & sb_iof[1])
                    | (sb_val[2] & sb_iof[2]) | (sb_val[3] & sb_iof[3]);
    // 入队字段: 必须取占据者原字段, 不能用 mux 后的 d_* (排水呈现拍 d_* 是队头)
    wire [31:0] cur_st_addr = mem_sel1 ? ex_mem1_alu_result : ex_mem_alu_result;
    wire [31:0] cur_st_data = mem_sel1 ? ex_mem1_store_data : ex_mem_store_data;
    wire [ 3:0] cur_st_strb = mem_sel1 ? ex_mem1_wstrb      : ex_mem_wstrb;
    // 地址 CAM (4 项独立比较)。⚠ 比较地址必须用占据者原地址 cur_ld_addr
    //   (纯寄存器源): 若用 d_addr (含排水队头 mux), 则
    //   drain_present → d_addr → CAM → load_ready → drain_present 成组合环
    //   ⇒ vvp 振荡卡死 (2026-10-10 实测, mul_tb 无声挂死)。
    wire [31:0] cur_ld_addr = mem_sel1 ? ex_mem1_alu_result : ex_mem_alu_result;
    assign d_load_addr = cur_ld_addr;
    // Presented writes drain the SB, or use the current store in bypass mode.
    // This address has no late load/drain arbitration.
    assign d_store_addr = SB_ON ? {sb_addr[sb_rp], 2'b00} : cur_ld_addr;
    wire cur_addr_is_ram;
    assign d_load_is_ram=cur_addr_is_ram;
    assign d_store_is_ram=SB_ON ? ~sb_iof[sb_rp] : cur_addr_is_ram;
    // Compare independent register-sourced high address bits before the
    // late lane selector, as with the low-word CAM comparators.
    sb_ram_scope #(.RAM_AW(IO_RAM_ADDR_W)) u_sb_scope(
        .addr0(ex_mem_alu_result),.addr1(ex_mem1_alu_result),
        .lane1(mem_sel1),.is_ram(cur_addr_is_ram));
    // sb_iof is captured from the same full address at enqueue. Qualified
    // RAM matches are therefore equivalent to full-address CAM matches.
    // IO reads never forward; io_ord_wait still orders all buffered IO.
    wire sb_cmp0;
    sb_cam_match #(.WORD_BITS(IO_RAM_ADDR_W-2)) u_sb_cam0(.word_addr(sb_addr[0][IO_RAM_ADDR_W-1:2]),
        .lane0_addr(ex_mem_alu_result[IO_RAM_ADDR_W-1:2]),.lane1_addr(ex_mem1_alu_result[IO_RAM_ADDR_W-1:2]),
        .lane1(mem_sel1),.match(sb_cmp0));
    wire sb_m0 = sb_val[0] & ~sb_iof[0] & sb_cmp0;
    wire sb_cmp1;
    sb_cam_match #(.WORD_BITS(IO_RAM_ADDR_W-2)) u_sb_cam1(.word_addr(sb_addr[1][IO_RAM_ADDR_W-1:2]),
        .lane0_addr(ex_mem_alu_result[IO_RAM_ADDR_W-1:2]),.lane1_addr(ex_mem1_alu_result[IO_RAM_ADDR_W-1:2]),
        .lane1(mem_sel1),.match(sb_cmp1));
    wire sb_m1 = sb_val[1] & ~sb_iof[1] & sb_cmp1;
    wire sb_cmp2;
    sb_cam_match #(.WORD_BITS(IO_RAM_ADDR_W-2)) u_sb_cam2(.word_addr(sb_addr[2][IO_RAM_ADDR_W-1:2]),
        .lane0_addr(ex_mem_alu_result[IO_RAM_ADDR_W-1:2]),.lane1_addr(ex_mem1_alu_result[IO_RAM_ADDR_W-1:2]),
        .lane1(mem_sel1),.match(sb_cmp2));
    wire sb_m2 = sb_val[2] & ~sb_iof[2] & sb_cmp2;
    wire sb_cmp3;
    sb_cam_match #(.WORD_BITS(IO_RAM_ADDR_W-2)) u_sb_cam3(.word_addr(sb_addr[3][IO_RAM_ADDR_W-1:2]),
        .lane0_addr(ex_mem_alu_result[IO_RAM_ADDR_W-1:2]),.lane1_addr(ex_mem1_alu_result[IO_RAM_ADDR_W-1:2]),
        .lane1(mem_sel1),.match(sb_cmp3));
    wire sb_m3 = sb_val[3] & ~sb_iof[3] & sb_cmp3;
    // Forward only one full-word match, exactly as before. Parallel
    // one-hot qualification avoids a population-count carry chain and
    // priority strobe mux before the late-load data selection.
    wire [3:0] sb_single = {sb_m3 & ~(sb_m0 | sb_m1 | sb_m2),
                            sb_m2 & ~(sb_m0 | sb_m1 | sb_m3),
                            sb_m1 & ~(sb_m0 | sb_m2 | sb_m3),
                            sb_m0 & ~(sb_m1 | sb_m2 | sb_m3)};
    wire [3:0] sb_full_word = {(sb_strb[3] == 4'b1111), (sb_strb[2] == 4'b1111),
                               (sb_strb[1] == 4'b1111), (sb_strb[0] == 4'b1111)};
    // Data masks need only entry qualification. Load RAM scope is required
    // for hit/wait controls, and must not lengthen every selected data byte.
    wire sb_any_match = cur_addr_is_ram & (sb_m0 | sb_m1 | sb_m2 | sb_m3);
    // MMIO reads must reach the peripheral after older writes have drained.
    wire sb_fwd_hit = SB_ON & cur_is_load & cur_addr_is_ram & (|(sb_single & sb_full_word));
    // Data is consumed only when the qualifier proves a single match.
    wire [31:0] sb_fwd_data = ({32{sb_m0}} & sb_data[0]) |
                               ({32{sb_m1}} & sb_data[1]) |
                               ({32{sb_m2}} & sb_data[2]) |
                               ({32{sb_m3}} & sb_data[3]);
    wire sb_fwd_wait = cur_is_load & sb_any_match & ~sb_fwd_hit;
    wire io_ord_wait    = cur_is_load & sb_has_io & ~sb_fwd_hit;
    wire load_stall     = SB_ON & cur_is_load & ~sb_fwd_hit
                          & (sb_fwd_wait | io_ord_wait);
    // ★ P9v4 (2026-10-11) → §5f-2 刀1 (2026-10-15) 撤除: mem1_ld_supp 压制
    //   已整体删除 (见下方 load_present 处的不变量注释), 本门随之消失。
    wire load_stall_eff = load_stall;
    wire load_ready     = cur_is_load & ~sb_fwd_hit & ~load_stall;
    // ★★ IO 气泡 (2026-10-10): IO(非 RAM) 访存退休的**次拍**强制 1 拍
    //   d_valid=0 —— load 呈现拍冻结流水线, 排水呈现拍仅延迟一拍 (SB 自持)。
    //   ⇒ 任意两次 IO 访存之间 d_valid 必有 ≥1 拍间隙, SoC 侧上升沿型 accept 桥
    //   每笔恰好 accept 一次, 每拍 accept 桥也不会对同笔重复触发。
    //   ⚠ 被气泡泡住的若是 load 必须同时冻结 adv (否则 d_free=1 会带着未完成
    //     的访存推进 = 丢访问); 排水被泡住则无需冻结 (SB 自持重试)。
    reg  io_retire_q;
    // ★★★ P9v4 (2026-10-11) 的错路配对 load 压制 (mem1_ld_supp) 已于
    //   §5f-2 刀1 (2026-10-15) **整体删除** —— 它是综合关键路径的公共头
    //   (MEM 分支解析 → mem_br_take/mem_br_mism_c → 本门 → D 口 → d_rdata
    //   → 前递 → ALU → 重定向/AGU, 21ns 级串行链), 且错路 load 发射在本
    //   SoC 上是无害预读。
    //   ⚠⚠⚠ 不变量 (删除的合法性依据, 动 MMIO 前必读):
    //     本 SoC 所有 **读侧** MMIO 寄存器 (UART_STATUS / GPIO_DATA) 均为
    //     无副作用纯状态读, 错路 load 提前读它们 = 预取语义, 可随冲刷丢弃。
    //     **未来新增任何带副作用的读寄存器 (FIFO 弹出/应答清除类) 时,
     //     必须恢复本压制** (git 历史 P9v4 可查), 否则错路读会破坏架构状态。
    //   遗留影响 (已评估): 错路 RAM load 多占 D 口 1 拍 (下一个真访存顺延,
    //   非阻塞); 其 pre 预告污染一次 pre_a (下一 load 落慢路 1 拍);
    //   响应随冲刷丢弃, 不入架构状态。
    wire load_present  = load_ready;
    wire load_pres_eff = load_present;
    wire drain_present = SB_ON ? (~sb_empty & ~load_pres_eff)  // SB 接管: D 口空闲排水
                               : cur_is_store;                 // 旁路: store 直通 (旧行为)
    wire d_any         = load_pres_eff | drain_present;
    wire mem_gap       = io_retire_q & d_any;
    wire load_gap      = mem_gap & load_pres_eff;      // 被泡住的是 load ⇒ 冻结
    wire d_is_drain   = SB_ON & drain_present;   // 旁路模式恒 0 ⇒ 字段一律取当前槽
    assign d_valid  = d_any & ~mem_gap;
    assign d_write  = drain_present;
    assign d_addr   = d_is_drain ? {sb_addr[sb_rp], 2'b00}
                                    : (mem_sel1 ? ex_mem1_alu_result : ex_mem_alu_result);
    assign d_wdata  = d_is_drain ? sb_data[sb_rp]
                                    : (mem_sel1 ? ex_mem1_store_data : ex_mem_store_data);
    assign d_wstrb  = d_is_drain ? sb_strb[sb_rp]
                      : (cur_is_store ? cur_st_strb : 4'b0);

    wire mem_busy     = d_valid & ~d_rdy;     // 数据访问占用 EX/MEM
    wire d_free       = ~d_valid | d_rdy;     // 数据不挡 EX/MEM 推进
    wire adv_base     = f_resp_vld & d_free;  // 取指退休 且 数据通道空闲: 全流水线推进拍

    // ★★ 提速措施4 (v2, 2026-10-11): MUL **不再冻结流水线** —— 改"结果晚一拍"。
    //   原设计: EX 停 1 拍等两级乘法 ⇒ 每个 MUL 冻结全场 1 拍 (CoreMark 实测
    //   9,510 拍 = 4.0%, 是仅次于 load_use 的最大可修项)。
    //   新设计: 发射拍 (adv=1) 锁存部分积并**照常推进**; 结果在其 MEM 拍有效,
    //   经两处 mux 注入真值:
    //     · ex_mem_fw_val (转发码 3'b001/消费者在 EX) —— 寄存器源, 不加长时序;
    //     · mem_wb_alu_result 锁存 (MEM→WB)。
    //   吞吐: 背靠背 MUL 每 1 拍一个 (p_*_q 每拍可重锁, 上一拍结果当拍仍有效)。
    //   ⚠ ID 级投机地址若依赖 EX 级 MUL: alu_result 现在是**占位垃圾** ⇒ 预测
    //     失配落 2 拍慢路 (pre_a==d_addr 门兜底, 功能安全); MUL 结果进访存地址
    //     的场景很少 (基址+偏移为主)。
    wire id_ex_is_mul = id_ex_valid && (id_ex_alu_op == `ALU_MUL);
    // ★ slot1 MUL (2026-10-11 §5c): P4a 放行 —— slot1 也可以是乘族指令,
    //   用第二乘法器实例 u_mul1 (与 u_mul 结构相同), 结果同样晚一拍
    //   (mul_result1), 经 ex_mem1_is_mul / mem_wb1_alu_result 两处注入。
    wire id_ex1_is_mul = id_ex1_valid && (id_ex1_alu_op == `ALU_MUL);
    // ★ SB: store 入队冻结 (SB 满) —— 两个 MUL 路径共用, 提前声明
    wire sb_stall     = SB_ON & cur_is_store & sb_full;
`ifdef MUL_OLD_FREEZE
    // ★ 旧"EX 停 1 拍"路径 —— **已发现有既有 bug, 仅存档勿用**:
    //   mulstress 差分 (金标准=Python) RES2 错: `mul t0; mul t1; mul t2,t0,t1`
    //   (t2 的操作数一个距离1 经 3'b001 转发、一个距离2) 算错 —— CoreMark
    //   CRC 未踩到纯属该模式未出现。正确路径 = 下面的 MUL_LATE。
    wire mul_go       = adv_base & id_ex_is_mul & ~mul_busy_q;
    wire adv          = adv_base & ~mul_go & ~(mem_gap & load_pres_eff)
                        & ~sb_stall & ~load_stall_eff;
    reg mul_busy_q = 1'b0;
    always @(posedge clk) begin
        if (!resetn) mul_busy_q <= 1'b0;
        else         mul_busy_q <= mul_go;
    end
`else
    // ★★ 默认 (2026-10-11): MUL 不冻结, 结果晚一拍 —— mulstress 14/14 全对。
    wire mul_go       = adv & id_ex_is_mul;
    wire adv          = adv_base & ~(mem_gap & load_pres_eff)
                        & ~sb_stall & ~load_stall_eff;
`endif

    // ★ branch/jalr MEM 解析事件拍 (adv 门控, 2026-10-11 冲 5 §5c):
    //   重定向/冲刷/BTB 延迟更新都在这一拍发生; 判定本体 mem_br_mism_c
    //   已在 MEM 驻留第一拍有效 (见 D 口段上方), 此处只做事件化。
    wire mem_br_fire     = ex_mem_valid && ex_mem_br_pend && adv;
    assign mem_br_mismatch = mem_br_mism_c && adv;

    wire data_retire  = d_valid & d_rdy;      // 数据访问退休拍 (load 退休 / SB 排水退休)
    // ★ IO 气泡状态: IO 访存退休的次拍拉高 (配合 mem_gap 插 1 拍气泡)
    always @(posedge clk) begin
        if (!resetn)              io_retire_q <= 1'b0;
        else                      io_retire_q <= data_retire & io_active;
    end
    // ★ SB 队列控制: adv 拍压入占据者 store; 排水退休拍弹出队头
    //   (同拍压入+弹出安全: wp==rp ⟺ 空, 而弹出要求非空 ⇒ 槽位不冲突)
    //   ★ 2026-10-11: MEM 级分支校正拍 (~mem_br_mismatch) —— ex_mem0 是挂起
    //   分支 (非 store), 此拍 cur_is_store 必来自 ex_mem1 的错路配对 store
    //   (P9 已放行 SB 模式下与挂起分支配对), 入队必须拦截; 入队发生在失配
    //   拍末 (adv=1), 组合门可精确撤回。SB_BYPASS 调试模式下该 store 不配对
    //   (P9 门 c1_memw&~SB_ON), 本门天然不触发。
    wire sb_push = SB_ON & adv & cur_is_store & ~sb_full & ~mem_br_mismatch;
    wire sb_pop  = SB_ON & data_retire & drain_present;
    always @(posedge clk) begin
        if (!resetn) begin
            sb_val[0] <= 1'b0; sb_val[1] <= 1'b0;
            sb_val[2] <= 1'b0; sb_val[3] <= 1'b0;
            sb_wp  <= 2'd0; sb_rp <= 2'd0; sb_cnt <= 3'd0;
        end else begin
            if (sb_push) begin
                sb_addr[sb_wp] <= cur_st_addr[31:2];
                sb_data[sb_wp] <= cur_st_data;
                sb_strb[sb_wp] <= cur_st_strb;
                // d_addr/io_active may belong to an older draining store here.
                // Classify the instruction being enqueued, not that bus owner.
                sb_iof [sb_wp] <= (cur_st_addr[31:IO_RAM_ADDR_W] != 0);
                sb_val [sb_wp] <= 1'b1;
            end
            if (sb_pop)
                sb_val[sb_rp] <= 1'b0;
            if (sb_push)                                    sb_wp  <= sb_wp + 2'd1;
            if (sb_pop)                                     sb_rp  <= sb_rp + 2'd1;
            sb_cnt <= sb_cnt + {2'b0, sb_push} - {2'b0, sb_pop};
        end
    end

    assign cpu_stall = ~adv;

    //-----------------------------------------------------------------
    // fence 全局 cache 无效化脉冲 (M1c): 对 if_id 中的 OP_FENCE 取边沿
    //   (stall 期 if_id 保持多拍, 只打一发; 误预测路径触发也只多刷一次,
    //    write-through 后端无脏行, 纯无效化语义安全)
    //-----------------------------------------------------------------
    wire fence_id = if_id_valid && (if_id_instr[6:0] == `OP_FENCE)
                                 && (if_id_instr[14:12] == `F3_FENCE);
    reg  fence_seen_q = 1'b0;
    always @(posedge clk) begin
        if (!resetn) begin
            fence_seen_q <= 1'b0;
            fence_flush  <= 1'b0;
        end else begin
            fence_seen_q <= fence_id;
            fence_flush  <= fence_id && !fence_seen_q;
        end
    end

    //-----------------------------------------------------------------
    // 流水线控制
    //   adv=0: PC/IF/ID/ID/EX/EX/MEM/MEM/WB 全保持
    //   S2a: retire_hold 冻结已移除 (M0 时代 2 拍桥遗留, dcache 下冗余)
    //   S2c: 分支重定向提前到 EX 判定拍 (branch_taken_eff, adv 门控):
    //        冲刷/pc 更新/链重建都比"打拍到 EX/MEM"方案早一拍. 分支本身
    //        仍随 adv 正常流入 EX/MEM (jal 的 pc+4 写回路径不变); 被冲刷的
    //        错路指令在 if_id/id_ex 两端同时清. 访存冻结拍分支留在 id_ex,
    //        重定向随之顺延 (adv 门控保证不丢失).
    //   (M2 BTB 曾接入本区, 但 coremark 活锁 pc=0x19ac → 已回退;
    //    尝试版存档 /tmp/btb_attempt.v, 待波形调试后再上)
    //-----------------------------------------------------------------
    //   ★★★ 2026-10-11 (冲 5 §5): 挂起分支 (ex_br_pend) 在 EX 拍**不解析** ——
    //     其 branch_taken (比较器) / branch_target (jalr 目标) 都是结构门下的
    //     占位垃圾 ⇒ branch_taken_eff / BTB 更新一律封锁, 解析延迟到 MEM 拍。
    //     mem_br_mismatch 拍 (MEM 级校正): id_ex 里的指令是被冲刷的**错路**
    //     指令, 它的 EX 级解析 (若恰是分支) 必须让位 (older-wins) ⇒ 同样封锁。
    wire branch_taken_eff   = ((branch_taken & ~pred_ok) | back_to_seq)
                              & id_ex_valid & adv
                              & ~ex_br_pend & ~mem_br_mismatch & ~ex_redirect_q;
    //-----------------------------------------------------------------
    // M2 BTB (IF 预测 / EX 校正+更新), 见 develop.md §8.14 集成规则:
    //   - 预测只在 f_take(退休拍)生效, 只改 next_pc、不发冲刷;
    //   - 校正只走 branch_taken_eff 通路(adv 门控, 冻结拍顺延不丢失);
    //   - 更新 adv 门控单发 (S3 教训: 冻结拍反复动作=取指饿死);
    //   - BTB_ENABLE=0 时 pred_* 恒 0 → 上式退化为 S2c 原表达式(零差异).
    //-----------------------------------------------------------------
    wire          btb_pred_strong = pred_strong_q;   // g_btb generate 驱动 (未启用恒 0)
    wire [31:0]   btb_pred_tgt    = pred_tgt_q;
    // ★★ P2 松绑 (§5d): slot1 预测 (BTB 第二查询口 lk1, 按 slot1 的 pc=pc+4)
    wire          btb1_pred_strong = pred1_strong_q; // g_btb generate 驱动 (未启用恒 0)
    wire [31:0]   btb1_pred_tgt    = pred1_tgt_q;
    wire          pred_applied    = f_take & btb_pred_strong;
    wire          ctrl_resolved   = id_ex_valid && !ex_redirect_q && (id_ex_branch | id_ex_jal | id_ex_jalr);
    // ★ §5d: slot1 的控制流解析在跑标志 (BTB 更新口仲裁用)
    wire          ctrl_resolved1  = id_ex1_valid && !ex_redirect_q && (id_ex1_branch | id_ex1_jal | id_ex1_jalr);
    wire          jalr_pred_target_ok, jalr1_pred_target_ok;
    wire          pred_ok         = id_ex_pred_taken && branch_taken
                                    && (id_ex_jalr ? jalr_pred_target_ok
                                                   : (id_ex_pred_tgt == branch_target));
    wire          back_to_seq     = id_ex_pred_taken && ~branch_taken;  // 预测 taken 实际 not
    // ★★ §5d: slot1 的预测校验 (与 slot0 同构; 驱动在 slot1 分支块)
    wire          pred_ok1     = id_ex1_pred_taken && branch_taken1
                                 && (id_ex1_jalr ? jalr1_pred_target_ok
                                                : (id_ex1_pred_tgt == branch_target1));
    wire          back_to_seq1 = id_ex1_pred_taken && ~branch_taken1;   // 预测 taken 实际 not

    // BTB 预测位默认 0 (g_btb generate 内 always 驱动; 禁用时保持声明初值)
    reg           pred_strong_q = 1'b0;
    reg  [31:0]   pred_tgt_q    = 32'h0;
    // ★ §5d: slot1 预测位默认 0 (同上)
    reg           pred1_strong_q = 1'b0;
    reg  [31:0]   pred1_tgt_q    = 32'h0;

    //-----------------------------------------------------------------
    // M3a: 精确陷阱 (EX 级). 优先级 trap > mret > branch.
    //   trap 拍: 冲刷 if_id/id_ex, mepc←id_ex_pc, mcause←编码, pc←mtvec;
    //   EX/MEM 不收陷阱指令(valid 抑制), CSR 非法地址不写.
    //   中断采样点 = id_ex 有效退休沿(adv), 首条未执行指令精确入 trap.
    //   ★ 2026-10-11 (冲 5 §5): MEM 级分支校正 (mem_br_mismatch) 拍,
    //     id_ex 里的指令是被冲刷的错路指令 ⇒ 其陷阱/mret/CSR 写全是伪事件,
    //     必须让位给更老的分支 (older-wins) ⇒ 一律 & ~mem_br_mismatch.
    //     中断为电平采样, 让位只是晚 1~2 拍重采, 不丢失。
    //-----------------------------------------------------------------
    wire [31:0] csr_mtvec, csr_mepc, csr_rd;
    wire        csr_mie, csr_rd_ok;
    wire [31:0] irq_id_enc = irq[0] ? 32'd0 : irq[1] ? 32'd1 :
                             irq[2] ? 32'd2 : irq[3] ? 32'd3 : 32'd0;
    wire        irq_any    = (|irq) === 1'b1;          // z/x 安全: 悬空=0
    wire        exc_fire   = id_ex_valid && adv && ~mem_br_mismatch && ~ex_redirect_q &&
        (id_ex_illegal || (id_ex_csr_en && !csr_rd_ok) || id_ex_ecall || id_ex_ebreak);
    wire        int_fire   = id_ex_valid && adv && csr_mie && irq_any && !exc_fire
                             && ~mem_br_mismatch && ~ex_redirect_q;
    wire        trap_fire  = exc_fire || int_fire;
    wire        mret_fire  = id_ex_valid && adv && id_ex_mret && !trap_fire
                             && ~mem_br_mismatch && ~ex_redirect_q;
    wire [31:0] exc_cause  = id_ex_ecall ? 32'd11 : id_ex_ebreak ? 32'd3 : 32'd2;
    wire [31:0] trap_cause = int_fire ? (32'h8000_0000 | irq_id_enc) : exc_cause;
    // ★★ P2 松绑 (2026-10-11 §5d): slot1 分支 EX 级校正 —— **older-wins**:
    //   slot0 程序序更老 ⇒ slot0 的一切重定向/中止事件 (EX 分支校正 / trap /
    //   mret / MEM 挂起分支校正) 全部压过 slot1 (该拍 slot1 是错路指令, 其
    //   解析/事件一律作废)。slot1 自身的校正与 slot0 同构: 预测失配或未预测
    //   实际 taken ⇒ 冲刷 if_id+id_ex、pc←解析目标 (not-taken ⇒ pc+4)。
    //   ⚠ 挂起机制 (§5a) 只支持 slot0: slot1 分支若操作数需 load 旁路, 在
    //     配对规则里直接拆对 (p_npend1), 永不挂起 ⇒ 此处无 ex_br_pend1。
    //   X 安全: branch_taken1 为 X (操作数 X) 时本式为 X ⇒ 在 if() 里按假
    //     处理 / pair_now 用 === 排除 —— 与 slot0 branch_taken_eff 同纪律。
    assign branch_taken_eff1 = ((branch_taken1 & ~pred_ok1) | back_to_seq1)
                               & id_ex1_valid & adv
                               & ~branch_taken_eff & ~trap_fire & ~mret_fire
                               & ~mem_br_mismatch & ~ex_redirect_q;
    wire [31:0] rs1_fwd    = (forward_a == 3'b011) ? mem_load_ext      :
                             (forward_a == 3'b001) ? ex_mem_fw_val      :
                             (forward_a == 3'b010) ? ex_mem1_fw_val     :
                             (forward_a == 3'b100) ? fwd_mem_alu        :
                             (forward_a == 3'b101) ? wb1_wdata : id_ex_rs1_data;
    wire [31:0] csr_wval   = id_ex_csr_imm ? {27'b0, id_ex_rs1} : rs1_fwd;  // uimm=instr[19:15]=id_ex_rs1

    csr u_csr (
        .clk(clk), .resetn(resetn),
        .rd_addr(id_ex_csr_addr), .rd_data(csr_rd), .rd_ok(csr_rd_ok),
        .wr_en(id_ex_valid && adv && id_ex_csr_en && csr_rd_ok && !trap_fire
               && !mem_br_mismatch && !ex_redirect_q),
        .wr_addr(id_ex_csr_addr), .wr_mode(id_ex_csr_mode), .wr_data(csr_wval),
        .trap(trap_fire), .trap_cause(trap_cause), .trap_pc(id_ex_pc),
        .mret(mret_fire),
        .mstatus_mie(csr_mie), .mtvec_o(csr_mtvec), .mepc_o(csr_mepc)
    );

    // ★ 双发射预留: 取指通道已能同时给出两个字 (= 两条指令)
    //   `f_resp_data` = pc 处的字, `f_resp_data_nxt` = pc+4 处的字,
    //   `f_resp_have_nxt` = 后者是否已在预取队里。
    wire [31:0] if_instr_nxt = f_resp_data_nxt;
    wire        if_have_nxt  = f_resp_have_nxt;

    //-----------------------------------------------------------------
    // ★★★ D1 (2026-10-08): 双发射 **配对判定 + 配对率测量**
    //   目的: **在不实现双发射的前提下先测出"可达配对率 p"** —— 它直接决定
    //   CM/MHz 4.5 是否可达:  cycles ≈ instret/(1+p) + stalls。
    //     p=0.40 ⇒ IPC 1.11 ⇒ 4.16 ;  p=0.50 ⇒ 1.17 ⇒ 4.40 ;  p=0.60 ⇒ 1.23 ⇒ 4.62
    //   两个候选操作数就是取指通道同一拍给出的两个字:
    //     slot0 = f_resp_data (pc 处)   slot1 = f_resp_data_nxt (pc+4 处)
    //   配对规则 P1~P6 见 doc/CPU_双发射优化方案_v1.md §11.3。
    //   ⚠ 本块**只产生观测量**, 不接进任何控制/数据通路 ⇒ 逐位零差异。
    //-----------------------------------------------------------------
    wire [4:0] if0_rd  = f_resp_data[11:7];
    wire [4:0] if1_rs1 = f_resp_data_nxt[19:15];
    wire [4:0] if1_rs2 = f_resp_data_nxt[24:20];

    wire       c0_branch, c0_jal, c0_jalr, c0_memr, c0_memw, c0_regw;
    wire       c0_illegal, c0_ecall, c0_ebreak, c0_csr, c0_mret;
    wire [3:0] c0_alu_op;
    wire       c1_branch, c1_jal, c1_jalr, c1_memr, c1_memw, c1_csr, c1_illegal;
    wire [3:0] c1_alu_op;

    ctrl u_ctrl_pair0 (
        .instr(f_resp_data), .alu_src(), .alu_op(c0_alu_op), .branch(c0_branch),
        .jal(c0_jal), .jalr(c0_jalr), .mem_read(c0_memr), .mem_write(c0_memw),
        .reg_write(c0_regw), .mem_to_reg(), .alu_a_pc(), .alu_a_zero(),
        .illegal(c0_illegal), .csr_en(c0_csr), .csr_imm(), .csr_mode(),
        .sys_mret(c0_mret), .sys_ecall(c0_ecall), .sys_ebreak(c0_ebreak)
    );
    ctrl u_ctrl_pair1 (
        .instr(f_resp_data_nxt), .alu_src(), .alu_op(c1_alu_op), .branch(c1_branch),
        .jal(c1_jal), .jalr(c1_jalr), .mem_read(c1_memr), .mem_write(c1_memw),
        .reg_write(), .mem_to_reg(), .alu_a_pc(), .alu_a_zero(),
        .illegal(c1_illegal), .csr_en(c1_csr), .csr_imm(), .csr_mode(),
        .sys_mret(), .sys_ecall(), .sys_ebreak()
    );

    wire p_have   = if_have_nxt;                                  // P1
    // ★★ §5d (2026-10-11): P2 已放宽 —— slot1 允许分支/跳转 (BTB lk1 第二
    //    查询口预测 + EX 级解析 + older-wins 重定向, 见交接 §5d)。
    //    p_nctrl1 保留: D1 观测块的旧口径计数用 (不参与实际配对判定)。
    wire p_nctrl1 = ~(c1_branch | c1_jal | c1_jalr);              // P2
    wire p_mem    = ~((c0_memr | c0_memw) & (c1_memr | c1_memw)); // P3
    wire p_nmul1  = ~(c1_alu_op == `ALU_MUL);                     // P4a (§5c 已从配对判定移除, 仅 D1 观测)
    wire p_ncsr1  = ~c1_csr;                                      // P4b
    wire p_nraw   = ~(c0_regw && (if0_rd != 5'b0) &&              // P5
                      ((if0_rd == if1_rs1) || (if0_rd == if1_rs2)));
    wire p_clean0 = ~(c0_illegal | c0_ecall | c0_ebreak) &&       // P6
                    ~(f_resp_data[6:0] == `OP_FENCE);
    // ★★★ slot1 必须是**纯 ALU**: 排除非法编码, 且排除整个 OP_SYSTEM (0x73)
    //   —— ecall/ebreak/mret/csr 全在里面。
    //   ⚠⚠ 血泪: 最初只排除了 `c1_csr`(funct3!=0), 结果 **ecall(funct3=0) 漏网**
    //      ⇒ slot1 的 ecall 被当成 NOP 静默执行 (trap 逻辑只看 slot0) ⇒
    //        trap_test 少了一次陷阱、后续全乱 (实测: 0xa8 的 ecall 未触发)。
    //   `fence`(0x0F) 同理排除: `fence_id` 只看 if_id_instr(slot0),
    //   slot1 的 fence 会丢掉 icache 无效化语义。
    wire p_leg1   = ~c1_illegal
                    && (f_resp_data_nxt[6:0] != `OP_SYSTEM)
                    && (f_resp_data_nxt[6:0] != `OP_FENCE);

    wire p_rest    = p_have & p_clean0 & p_leg1;   // 与规则无关的前置
    wire p_rules   = p_nctrl1 & p_mem & p_nmul1 & p_ncsr1 & p_nraw;
    wire if_pair_ok = p_rest & p_rules;
    wire if_issuing = adv & f_resp_vld;      // 本拍真的发射一条指令
    // 逐条规则的拒绝计数 (只在"有第2字"的拍上统计)
    wire rej_ctrl1  = p_rest & ~p_nctrl1;
    wire rej_mem    = p_rest &  p_nctrl1 & ~p_mem;
    wire rej_mulcsr = p_rest &  p_nctrl1 &  p_mem & ~(p_nmul1 & p_ncsr1);
    wire rej_raw    = p_rest &  p_nctrl1 &  p_mem &  p_nmul1 & p_ncsr1 & ~p_nraw;
    //-----------------------------------------------------------------
    // ★★★ D2b 预备 (2026-10-08): **实际要落地的规则集**下的配对率
    //   D1 的 P3 只禁"两条都访存"; 但 D2b 第一版把 slot1 限制为
    //   **纯 ALU 写回** (不访存/不控制/不 CSR/MUL), 这样 MEM 级、D 口、
    //   CSR/陷阱、分支重定向**全部不用改** —— 是"能一次做对"的前提。
    //   额外补的三条 (都是为**省硬件**而非省逻辑):
    //     P2b: slot0 不得被 BTB 预测 taken —— 预测 taken 时 pc+4 是错路,
    //          配进去就是错的 (而且 slot1 与 slot0 同拍退休, 无法单独拦)。
    //     P6x: slot0 不得是 CSR/MRET —— 它们会 trap/mret 并压掉整个 ID/EX,
    //          而 slot1 与 slot0 同拍 ⇒ 必须一起压; 直接排除最省事。
    //     P7 : slot0.rd != slot1.rd —— 双写口同址冲突 + 转发源二义。
    //   这条计数 = **D2b 的可达配对率上限**, 判据同 D1: 纯观测、零差异。
    //-----------------------------------------------------------------
    wire [4:0] if0_rd_s  = f_resp_data[11:7];
    wire [4:0] if1_rd_s  = f_resp_data_nxt[11:7];
    wire p_nmem1s = ~(c1_memr | c1_memw);                       // P3': slot1 不访存
    // ★★ P5' (对内 RAW 放宽) —— **已实验并撤回 (2026-10-08, 见文档 §11.15)**
    //   做法是给 slot1 加一个"slot0 EX 结果"的转发源 (forwarding 编码 3'b110)。
    //   实测: 它会把 slot0 的 **X** 一路带进 `alu_result1` → `wb1_wdata` →
    //   **写回寄存器堆** ⇒ X 扩散 + 取指链死锁 (pc 卡在 0xcf0, instret 8215)。
    //   根因是"寄存器堆里本来就存在 X"(见 §11.15), 放宽 P5 只是把它放大。
    //   ⇒ 暂缓, 规则回到 P5 (`p_nraw`), 硬件通路也一并撤掉以免留死代码。
    // ★★ P5' (2026-10-08 修正): 允许**对内 RAW**, 但仅当 slot0 的结果**可转发**
    //   (slot0 是纯 ALU 写回; load 的 alu_result 是地址, jal/jalr 的是跳转目标)。
    //   ⚠⚠ 第一版把布尔式写反了:
    //        `~(... & p_raw0_ok & ...)`  ⇒ 当 p_raw0_ok=0 时整项为 0 ⇒ 反而**放行**,
    //        也就是"slot0 是 load 且 slot1 读它的 rd"这个最不能配对的情况被允许了
    //        ⇒ slot1 拿 load 旁路的 X 当操作数 ⇒ 结果 X 写回寄存器堆 ⇒ 死锁。
    //      正确式: 有 RAW **且** slot0 不可转发 ⇒ 才禁止。
    wire p_raw_present = c0_regw & (if0_rd_s != 5'b0) &
                         ((if0_rd_s == if1_rs1) || (if0_rd_s == if1_rs2));
    // ★★ P5'' (再放宽): slot0 是 jal/jalr 时也允许对内 RAW ——
    //   它们的架构写回值是 **pc+4**, EX 拍就能算 (见 `slot0_ex_fw_val`)。
    //   **load 仍禁止** (写回值要到 MEM 级)。
    // ★★★ MUL 也禁止 (2026-10-11, MUL_LATE 差分测试定位): 晚结果方案里
    //   MUL 的 `alu_result` 在对内转发的 EX 拍是**占位垃圾** (真值要 MEM 拍
    //   才有), slot1 会把垃圾当操作数 —— mulstress 差分: RES0/P5 模式的
    //   `mul→addi` 对内 RAW 读到 0。⇒ MUL 回到严格 P5 (对内 RAW 拆对)。
    wire c0_is_mul, pair_cmp1, pair_mul1, pair_light0;
    rv32_pair_classes u_pair_class0 (.instr(f_resp_data),
        .compare_op(),.multiply_op(c0_is_mul),.light_op(pair_light0));
    rv32_pair_classes u_pair_class1 (.instr(f_resp_data_nxt),
        .compare_op(pair_cmp1),.multiply_op(pair_mul1),.light_op());
    wire p_raw0_ok = ~c0_memr && ~c0_is_mul && ~(c1_branch | c1_jalr);
    // Spend a small issue-rate budget to avoid chaining two ALUs after a
    // late MEM load. p9b_bsrc mirrors the producer position at pair time.
    // Such dependencies issue separately; the slot1 RAW path can then use
    // the structurally load-free slot0 result for every instruction class.

    // APM input setup is tight. Issue an EX0-dependent MUL separately so
    // the multiplier input never serializes the preceding ALU in one cycle.


    wire p_nraw_s  = ~p_raw_present | (p_raw0_ok & ~p9b_bsrc & ~pair_mul_source & ~pair_cmp1 & ~pair_mul1 & pair_light0);
    // A multiply in the preceding issue group becomes a MEM-stage late
    // result when fetched slot0 executes. Do not chain its result through
    // slot0 and an intra-pair dependent slot1 in the same cycle.
    wire pair_mul_source0 = (id_alu_op == `ALU_MUL) &&
                             (if_id_instr[11:7] != 5'b0) &&
                             ((if_id_instr[11:7] == f_resp_data[19:15]) ||
                              (if_id_instr[11:7] == f_resp_data[24:20]));
    wire pair_mul_source1 = if_id_pair && (id1_alu_op == `ALU_MUL) &&
                             (if_id_instr1[11:7] != 5'b0) &&
                             ((if_id_instr1[11:7] == f_resp_data[19:15]) ||
                              (if_id_instr1[11:7] == f_resp_data[24:20]));
    wire pair_mul_source = pair_mul_source0 | pair_mul_source1;
    wire p_njal0  = ~(c0_jal | c0_jalr);                        // P2c
    // ★★★ #3: P2b 的**正确放宽** —— 不再问"slot0 是否被预测 taken", 而是问
    //   "**取指包第 2 条到底是谁**":
    //     预测 taken ⇒ 第 2 条必须正好是 BTB 目标 T (pf_buffer 读的就是 T)
    //     预测 not   ⇒ 第 2 条必须是 pc+4, 且 slot0 不能是无条件跳转
    //   校验不过就不配对 (退回单发射)。**构造性安全**: 配对成立 ⟺ slot1 是 slot0
    //   的架构后继。⚠ 这里用的是 **CPU 自己那一拍查的 BTB** (新鲜), 不是预取器的
    //   (陈旧) ⇒ 两边不一致时只是"这次不配对", 绝不会配错、也不会走错路。
    wire p_npred0 = (btb_pred_strong === 1'b1)
                    ? (f_resp_nxt_addr === btb_pred_tgt)
                    : ((f_resp_nxt_addr === (pc + 32'd4)) && (p_njal0 === 1'b1));
    wire p_ncsr0  = ~(c0_csr | c0_mret);                        // P6x
    // ★★ P7 (slot0.rd == slot1.rd) —— **2026-10-08 实测证明这条是多余的, 已删除**。
    //   双写口同址冲突 / 转发源二义 在硬件里**本来就按程序序处理好了**:
    //     · 寄存器堆写: `if (we && waddr!=0) ...; if (we1 && waddr1!=0) ...`
    //       —— 后一条非阻塞赋值覆盖前一条 ⇒ **slot1 (程序序更晚) 优先** ✓
    //     · 读穿透旁路: 先查写口 1, 再查写口 0 ⇒ slot1 优先 ✓
    //     · 转发优先级: `ex_mem1` 排在 `ex_mem0` 前、`mem_wb1` 排在 `mem_wb0` 前 ✓
    //   实测删掉后 cycles 331,336 → 更少, 四 CRC 不变。
    wire p_nrd0   = 1'b1;   // P7 已放宽 (保留名字, 便于与 §11.3 的规则表对照)
    wire p_nfence1= ~(f_resp_data_nxt[6:0] == `OP_FENCE);       // P8
    // ★★★ P9 (2026-10-11, 冲 5 §5; §5c 放宽): slot0 是 branch/jalr 且
    //   「EX 拍会挂起」且 slot1 是 **store（仅 SB_BYPASS 调试模式）** 时不配对。
    //   §5c 起放行 load-slot1: 错路配对 load 由 mem_br_mism_c (纯组合判定,
    //   MEM 驻留第一拍有效) 从第一拍就压住不上 D 口 (mem1_ld_supp — 已撤除, 见 D 口
    //   段) —— adv=0 拍提前访存的窗口已彻底关闭, 失配可精确撤回;
    //   SB 模式 store-slot1 也已放行 (入队在失配拍末, sb_push 组合门拦截);
    //   唯一残留 = SB_BYPASS 调试模式下 store 直通 D 口可能提前写内存 ⇒ 禁。
    //   slot1 为纯 ALU 时配对: 唯一副作用是写回, 捕获在失配拍末
    //   (adv=1), 用 ~mem_br_mismatch 门拦截 (见 MEM/WB slot1 块)。
    //   挂起判定与 EX 级公式**逐位镜像** (L=if_id_instr1 / M=if_id_instr,
    //   对应 EX 拍的 ex_mem1 / ex_mem0):
    //     挂起 ⟺ L 是命中 load, 或 L 不写该寄存器而 M 是命中 load
    //   —— L (程序序更晚) 的匹配写会盖掉 M, 此时分支在 EX 拍根本不挂起,
    //   不能因 M 命中就误拦配对。
    wire [ 4:0] if0_rs1 = f_resp_data[19:15];
    wire [ 4:0] if0_rs2 = f_resp_data[24:20];
    wire p9_l_ld  = if_id_pair && (id1_mem_read || id1_alu_op == `ALU_MUL) && (if_id_instr1[11:7] != 5'b0) &&
                    ((if_id_instr1[11:7] == if0_rs1) || (if_id_instr1[11:7] == if0_rs2));
    wire p9_l_win = if_id_pair && id1_reg_write && (if_id_instr1[11:7] != 5'b0) &&
                    ((if_id_instr1[11:7] == if0_rs1) || (if_id_instr1[11:7] == if0_rs2));
    wire p9_m_ld  = (id_mem_read || id_alu_op == `ALU_MUL) && (if_id_instr[11:7] != 5'b0) &&
                    ((if_id_instr[11:7] == if0_rs1) || (if_id_instr[11:7] == if0_rs2));
    wire p9_pend  = if_id_valid && (p9_l_ld | (~p9_l_win & p9_m_ld));
    wire p_npend0 = ~((c0_branch | c0_jalr) && p9_pend &&
                      (c1_memw & ~SB_ON));
    // ★★ P9 扩展 (2026-10-11 §5d; 2026-10-14 逐操作数修正): slot1 分支/跳转
    //   的挂起拆对 —— 挂起机制 (EX 挂起 + MEM 级解析, §5a) 只支持 slot0,
    //   slot1 分支的操作数若在 EX 拍需 load 旁路 (raw 011) 必须拆对。
    //   ★ 修正史: 原镜像用「L 写了任一操作数 (l_win)」一笔带过 —— 但分支有
    //     **两个操作数**: L(lui t2) 只写 rs2 时, rs1 仍需 M(load) 的 011,
    //     该配对被错误放行 (rv32ui-p-lw case3 实测: slot1 分支 EX 直取陈旧
    //     mem_load_ext 判错方向)。现按 forwarding_unit 的 slot1 链**逐操作数**
    //     重写: rsX 需 011 ⟺ B(本对 slot0) 不写 rsX ∧ [L 是命中 load
    //     ∨ (L 不写 rsX ∧ M 是命中 load)]。(B 写 ⇒ 110 对内转发, 安全。)
    //   判定与 forwarding_unit 的 slot1 优先级逐位镜像 (L=if_id_instr1 /
    //     M=if_id_instr ↔ EX 拍的 ex_mem1 / ex_mem0)。
    //   jal 无寄存器操作数, 永不挂起 ⇒ 只拦 branch/jalr (见 p_npend1)。
    wire p9b_bwa = c0_regw && (f_resp_data[11:7] != 5'b0) && (f_resp_data[11:7] == if1_rs1);
    wire p9b_bwb = c0_regw && (f_resp_data[11:7] != 5'b0) && (f_resp_data[11:7] == if1_rs2);
    wire p9b_lda = if_id_pair && (id1_mem_read || id1_alu_op == `ALU_MUL) && (if_id_instr1[11:7] != 5'b0) && (if_id_instr1[11:7] == if1_rs1);
    wire p9b_ldb = if_id_pair && (id1_mem_read || id1_alu_op == `ALU_MUL) && (if_id_instr1[11:7] != 5'b0) && (if_id_instr1[11:7] == if1_rs2);
    wire p9b_lwa = if_id_pair && id1_reg_write && (if_id_instr1[11:7] != 5'b0) && (if_id_instr1[11:7] == if1_rs1);
    wire p9b_lwb = if_id_pair && id1_reg_write && (if_id_instr1[11:7] != 5'b0) && (if_id_instr1[11:7] == if1_rs2);
    wire p9b_mda = (id_mem_read || id_alu_op == `ALU_MUL) && (if_id_instr[11:7] != 5'b0) && (if_id_instr[11:7] == if1_rs1);
    wire p9b_mdb = (id_mem_read || id_alu_op == `ALU_MUL) && (if_id_instr[11:7] != 5'b0) && (if_id_instr[11:7] == if1_rs2);
    // ★★ 2026-10-15 (Rev.C, 108M 时序): **传递依赖** 拆对 —— B(slot0) 的操作数
    //   若来自 L/M 位的 load (B 在 EX 拍走 011 转发取 mem_load_ext), 则 B 的
    //   结果 → slot1 分支/jalr 的操作数 (110) → branch_target1 → f_redirect
    //   是 **两级转发串联** 的 20ns 组合链 (108M PnR 实测最差路径)。镜像只查
    //   一级不够: jalr rs1=B.rd ≠ L/M 的 load rd, 但值链仍是 load→ALU→目标。
    //   ⇒ B 写的寄存器若是 "load 滋养" 的, 同样拆对。更深的生产者 (pair-2
    //   之前) 在分支 EX 拍已进 WB (寄存器源), 路径天然短, 无需递归。
    wire p9b_bsrca = id_mem_read  && (if_id_instr[11:7]  != 5'b0) &&
                     ((if_id_instr[11:7]  == f_resp_data[19:15]) ||
                      (if_id_instr[11:7]  == f_resp_data[24:20]));
    wire p9b_bsrcb = if_id_pair && id1_mem_read && (if_id_instr1[11:7] != 5'b0) &&
                     ((if_id_instr1[11:7] == f_resp_data[19:15]) ||
                      (if_id_instr1[11:7] == f_resp_data[24:20]));
    wire p9b_bsrc  = p9b_bsrca | p9b_bsrcb;   // B 的操作数被 L/M 位 load 滋养
    wire p9_pend1  = if_id_valid &&
                     ((p9b_lda | (~p9b_lwa & p9b_mda)) & ~p9b_bwa) |
                     ((p9b_ldb | (~p9b_lwb & p9b_mdb)) & ~p9b_bwb) |
                     (p9b_bsrc & (p9b_bwa | p9b_bwb));
    wire p_npend1  = ~((c1_branch | c1_jalr) && p9_pend1);
    wire p_clean0s = p_clean0 & p_ncsr0 & p_npred0;
    // ★★ P3' 已放宽: slot1 允许访存 (P3 仍生效: 一对最多一条访存)
    // ★★ §5c (2026-10-11): P4a 已放宽 —— slot1 允许 MUL (第二乘法器 u_mul1,
    //    结果晚一拍, 与 slot0 MUL_LATE 镜像); P4b (slot1 CSR) 仍禁止。
    //    p_nmul1 保留: D1 观测块的旧口径计数用 (不参与实际配对判定)。
    // ★★ §5d (2026-10-11): P2 已放宽 —— p_nctrl1 退出实际配对判定,
    //    新增 p_npend1 (slot1 分支挂起拆对, 见上)。
    wire if_pair_ok_s = p_rest & p_clean0s & p_mem
                        & p_ncsr1 & p_nraw_s & p_nrd0 & p_nfence1
                        & p_npend0 & p_npend1;
    // 分组拒绝计数 (便于定位"哪条最贵"; 链序 = 实际规则集的排列)
    wire rej_s_mem   = p_rest & p_clean0s & p_npend0 & p_npend1 & ~p_mem;
    wire rej_s_raw   = p_rest & p_clean0s & p_npend0 & p_npend1 & p_mem
                       & p_ncsr1 & ~p_nraw_s;
    // ★ §5d: 占用原 rej_s_rd0 槽位 (P7 已放宽, 恒 0) —— slot1 分支挂起拆对
    wire rej_s_pen1  = p_rest & p_clean0s & p_mem & p_ncsr1 & p_nraw_s
                       & p_npend0 & ~p_npend1;
    wire rej_s_clean = p_rest & ~p_clean0s;
    // ★ 2026-10-11: mem_br_mismatch (MEM 级分支校正) 并入冲刷族 ——
    //   冲 if_id/id_ex、置 pc_en、禁 pair_now, 与 EX 级 branch_taken_eff 同构,
    //   只是发生晚一级 (分支已到 MEM)。
    // ★ §5d (P2 松绑): slot1 的 EX 级分支校正 (branch_taken_eff1) 同样并入
    //   冲刷族 —— 冲 if_id/id_ex、pc←解析目标; older-wins 已在 eff1 定义处
    //   让位给 slot0 全部事件。
    // ★ 刀B: mm_pend_q 并入冲刷族 —— mm 拍清 "按顺序方向取的那条错路指令"
    //   (且保证 pc_en=1, 见 mm 寄存器注释)
    wire flush_eff = trap_fire | mret_fire | ex_redirect_q | mem_br_mismatch
                   | mm_pend_q
                   ;
    wire lu_eff    = load_use_stall & adv;
    wire front_en  = adv & ~lu_eff;

    //-----------------------------------------------------------------
    // ★★★ D2b: 本拍**真的发射一对** (slot0 + slot1)
    //   `if_pair_ok_s` 已含 `if_issuing = adv & f_resp_vld`, 这里再排除
    //   冲刷拍 (flush_eff 会清 IF/ID, 不装对) 与 load-use 插泡拍 (lu_eff)。
    //   ⇒ 与 IF/ID 的装载条件、PC 的 +8 步进**严格同条件**, 三者必须一致。
    //-----------------------------------------------------------------
    //   ⚠ 必须显式带上 `if_issuing` (= adv & f_resp_vld): `if_pair_ok_s` 的
    //     `p_rest` **不含**发射门, 漏掉会让 pair_now 在非发射拍为 1
    //     (虽然下游 pc_en/pop 都门住了, 但计数会虚高: 实测 70,594 vs 66,641)。
    //
    //   ⚠⚠⚠ **X 安全 (2026-10-08 实测踩中, 死锁级)**:
    //     `flush_eff = trap_fire | mret_fire | branch_taken_eff`, 而
    //     `branch_taken_eff` 可能为 **X** —— 当 EX 级是一条 branch 且它的
    //     `id_ex_rs1_data` 是 X 时 (id_ex 里可能压着一条被冲刷/未初始化路径的
    //     指令), `alu_branch_taken` 就是 X。
    //     旧代码里 `next_pc` 走的是 `if (branch_taken_eff) ...` —— **X 被当作假**,
    //     所以这个 X 一直是**良性**的 (v1 全程 CRC 一致就是证据)。
    //     但 `next_pc = pc + (pair_now ? 8 : 4)` 是**三目**:
    //       `X ? 8 : 4` ⇒ X ⇒ pc = X ⇒ 整机死锁 (实测 instret 卡在 8214)。
    //     ⇒ 配对判定**必须用 `===` 把 X 归到"不配对"**(保守且安全:
    //       真冲刷时不配对; 非冲刷时只是少配一次, 只损失一点性能)。
    wire pair_now = (if_pair_ok_s === 1'b1) & (if_issuing === 1'b1)
                    & (flush_eff === 1'b0) & (lu_eff === 1'b0);

    // ★★ #3: 一拍弹 2 格的两种情形:
    //   ① 真配对        —— slot1 与 slot0 同拍退休;
    //   ② **跳过插入项** —— 预取器在预测 taken 时把 T 插在 A 后面, 但 CPU 这一拍
    //      判定**不跳** (自己的 BTB 说 not-taken) ⇒ 这个插入项是废的, 必须
    //      一起弹掉, 否则队头会停在 T 上, 与 pc+4 失配 ⇒ resync。
    wire pkt_is_jump = (f_resp_have_nxt === 1'b1) && (f_resp_nxt_addr !== (pc + 32'd4));
    assign f_take_nxt = pair_now | (pkt_is_jump & ~btb_pred_strong);

    // ★★ §5d (P2 松绑): slot1 的 IF 级预测应用 —— **仅配对顺序包**:
    //   pair_now 已含 p_npred0 ⇒ 此包要么「slot0 未被预测 taken 且非
    //   jal/jalr 且第 2 条=pc+4」(顺序包), 要么「slot0 预测 taken 且第 2
    //   条=T」(插入包); 再加 pkt_seq (第 2 条地址==pc+4, 即 lk1 的查询地址)
    //   把插入包排除 ⇒ slot1 预测只在顺序包生效。插入包的 slot1 (=T) 不
    //   预测 —— 它作为后续包的 slot0 时再走正常 lk 口 (v1 取舍)。
    //   预测效果: next_pc ← btb1_pred_tgt (下拍直接取 slot1 分支的目标)。
    wire pkt_seq = (f_resp_nxt_addr === (pc + 32'd4));   // 顺序包 (slot1 的 pc = lk1 查询地址)
    assign pred_applied1 = pair_now & pkt_seq & (btb1_pred_strong === 1'b1);

    always @(*) begin
        pc_en       = flush_eff | front_en;
        if_id_clear = flush_eff;
        id_ex_clear = flush_eff | lu_eff;
    end

    // ★★ §5f-2 刀B (2026-10-16): MEM 失配重定向**打拍** —— 实测综合榜首
    //   (−7.87) 是 "dcache 读 → mem_load_ext → 前递 → MEM 分支解析(32b 比较)
    //   → next_pc → pc" 的 load→branch→pc 全组合链。把失配校正事件寄存一拍:
    //   失配拍锁存 mm_pend_q/mm_tgt_q, **次拍** pc 才改向 ⇒ 解析→pc 的 D 链切断。
    //   语义变化与对齐:
    //     · 失配拍 pc 照常吃顺序 next_pc (取的错路指令由 flush 清, 见 flush_eff
    //       的失配拍项), mm 拍 pc ← mm_tgt (T 方向);
    //     · **mm 拍 flush_eff=1** (本刀给 flush_eff 加了 mm_pend_q 项) ⇒ pc_en
    //       恒 1, mm_pend_q 单拍脉冲必然被消费 —— 无需保持逻辑 (也规避了
    //       "mm_pend_q→flush_eff→pc_en→保持条件" 的组合环);
    //     · mm 拍 flush 同时清掉 "mm 拍按顺序方向取的那条错路指令" (失配拍
    //       if_id_clear 已清一轮, mm 拍取指用顺序 pc 会再进一条 ⇒ 必须再清);
    //     · f_redirect_vld 的失配项同步换成 mm_pend_q (pf 侧 R1a 本就打拍,
    //       两者对齐: pf 与 pc 同拍晚 1 拍改向, hint 值 = mm_tgt = T ✓);
    //     · 代价 = 每次 MEM 失配 +1 拍 (实测见 CoreMark ticks 台账); EX 级
    //       校正 (branch_taken_eff*) 不打拍, 仍走组合 next_pc;
    //     · mm 拍 ID/EX/EX1 全是 bubble (失配拍 if_id/id_ex_clear 已清) ⇒
    //       mm 拍不可能有新 trap/mret/EX 校正事件, 伪事件封锁窗无需扩。
    reg        mm_pend_q;
    reg [31:0] mm_tgt_q;
    // EX still cancels its paired younger slot immediately. The front end
    // receives a registered correction one cycle later. During that cycle
    // both younger EX slots, exceptions, CSR and predictor updates are killed.
    // Targets latch every cycle: late prediction equality must not drive their CE.
    // The redirect pulse forces PC/IFID/IDEX update even when fetch is stalled.
    always @(posedge clk) begin
        if (!resetn) begin
            ex_redirect_q <= 1'b0; ex_redirect_lane1_q <= 1'b0;
            ex_redirect_tgt0_q <= 32'b0; ex_redirect_tgt1_q <= 32'b0;
        end else begin
            ex_redirect_q <= ((branch_taken_eff === 1'b1) || (branch_taken_eff1 === 1'b1))
                             && !trap_fire && !mret_fire && !mm_pend_q;
            ex_redirect_lane1_q <= (branch_taken_eff1 === 1'b1);
            ex_redirect_tgt0_q <= branch_taken ? branch_target : (id_ex_pc + 32'd4);
            ex_redirect_tgt1_q <= branch_taken1 ? branch_target1 : (id_ex1_pc + 32'd4);
        end
    end
    always @(posedge clk) begin
        if (!resetn) begin
            mm_pend_q <= 1'b0;
            mm_tgt_q  <= 32'h0;
        end else begin
            mm_pend_q <= mem_br_mismatch;
            // taken ⇒ 解析目标 (jalr 用补丁后操作数算的目标 / branch 用 pc+imm);
            // not-taken ⇒ 分支 pc+4 (与原 next_pc 尾项逐位一致, 仅平移一拍)
            if (mem_br_mismatch)
                mm_tgt_q <= mem_br_take ? mem_br_tgt : (ex_mem_br_pc + 32'd4);
        end
    end

    //-----------------------------------------------------------------
    // PC 更新 (同步复位低有效, 复位 PC = 0)
    //-----------------------------------------------------------------
    wire [31:0] pc_seq4, packet_seq4;
    pc_inc4 u_pc_seq4 (.a(pc), .y(pc_seq4));
    pc_inc4 u_packet_seq4 (.a(f_resp_nxt_addr), .y(packet_seq4));
    always @(posedge clk) begin
        if (!resetn)
            pc <= 32'h0000_0000;
        else if (pc_en)
            pc <= next_pc;
    end

    always @(*) begin
        // ★★ #3: 配对拍的下一条 = **第 2 条的地址 + 4**
        //   顺序包 (nx = pc+4) ⇒ pc+8 (与基线逐位一致);
        //   插入包 (nx = T)   ⇒ T+4  (T 已被 slot1 消费, 不能再取 T)
        next_pc = pair_now ? packet_seq4 : pc_seq4;
        // ★★★ 2026-10-09 修复: **配对拍绝不能被 BTB 预测覆盖**。
        //   配对时 slot1 = 第 2 条(可能就是插入项 T)已被消费 ⇒ 下一条必须是
        //   `第 2 条地址 + 4`; 若此时还被 `btb_pred_tgt`(=T) 覆盖, pc 会**跳回 T**
        //   ⇒ 目标指令被执行两次 ⇒ 栈指针漂移 / 结果全错。
        //   (实测: `bne`@0xb0 配对插入项 0x90 后 pc 又回到 0x90, 退休流里
        //    `0x90` 连续出现两次。)
        if (!pair_now && pred_applied)
            next_pc = btb_pred_tgt;           // M2: IF 预测跳转 (被退休指令拍生效)
        // ★★ §5d (P2 松绑): slot1 预测 —— 放在 slot0 预测之后 (退化场景
        //   T==pc+4 时 slot0 的预测目标已被配对本身消费, slot1 的目标才是
        //   下一条; 正常顺序包里 pred_applied=0, 此行即生效)。
        if (pred_applied1)
            next_pc = btb1_pred_tgt;          // §5d: slot1 预测跳转 (配对顺序包)
        if (ex_redirect_q)
            next_pc = ex_redirect_lane1_q ? ex_redirect_tgt1_q : ex_redirect_tgt0_q;
                                              // S2c: EX 重定向 / M2: 误预测校正
                                              // ★ RVC: 顺序返回地址也是 pc+长度
        // ★★ §5d (P2 松绑): slot1 的 EX 级校正 —— 排在 slot0 之后 (older-
        //   wins: 同拍双校正时 slot0 压轴), mret/trap/MEM 校正仍在其后
        //   (更老事件优先; eff1 定义处已用 ~门让位, 这里是双保险)。
        if (mret_fire)
            next_pc = csr_mepc;               // M3a: mret 返回
        if (trap_fire)
            next_pc = csr_mtvec & 32'hFFFF_FFFC;  // M3a: Direct 模式入向量
        // ★★★ 2026-10-11 (冲 5 §5): MEM 级分支校正 —— **最高优先级**。
        //   挂起分支在 MEM 拍解析失配: taken ⇒ pc←解析目标 (jalr 用补丁后
        //   操作数算的目标 / branch 用 pc+imm); not-taken ⇒ pc←分支 pc+4。
        //   同拍 trap/mret 已被 ~mem_br_mismatch 封锁 (错路指令的伪事件),
        //   这里再压轴一次保证优先级; mem_br_take 已 X 安全 (=== 归一)。
        // ★ 刀B: 失配校正移到 mm 拍生效 (mm_pend_q 压轴, older-wins 语义不变;
        //   mm 拍同拍 pred_applied/EX 校正已被 flush 门控/气泡化压制, 见上分析)
        if (mm_pend_q)
            next_pc = mm_tgt_q;
    end

    //-----------------------------------------------------------------
    // IF/ID 流水线寄存器
    //-----------------------------------------------------------------
    always @(posedge clk) begin
        if (!resetn) begin
            if_id_valid <= 1'b0;
            if_id_pc    <= 32'h0;
            if_id_instr <= 32'h0;
            if_id_pair  <= 1'b0;
            if_id_instr1<= 32'h0;
            if_id_pc1   <= 32'h0;
            if_id_pred_taken <= 1'b0;
            if_id_pred_tgt   <= 32'h0;
            if_id1_pred_taken <= 1'b0;     // ★ §5d: slot1 预测位随复位清
            if_id1_pred_tgt   <= 32'h0;
        end else if (if_id_clear) begin
            if_id_valid <= 1'b0;           // 冲刷: 插 bubble, 数据保持
            if_id_pair  <= 1'b0;           // ★ D2b: 配对位随冲刷清
            if_id_pred_taken <= 1'b0;      // M2: 预测位随冲刷清
            if_id1_pred_taken <= 1'b0;     // ★ §5d: slot1 预测位随冲刷清
        end else if (pc_en) begin          // 非停顿: 装载
            if_id_valid <= 1'b1;
            if_id_pc    <= pc;
            if_id_instr <= f_resp_data;
            if_id_pair  <= pair_now;               // ★ D2b
            if_id_instr1<= f_resp_data_nxt;        // ★ D2b (pair_now=0 时无意义)
            if_id_pc1   <= f_resp_nxt_addr;        // ★ #3 (pair_now=0 时无意义)
            if_id_pred_taken <= pred_applied;   // M2
            if_id_pred_tgt   <= btb_pred_tgt;
            if_id1_pred_taken <= pred_applied1; // ★ §5d (pair_now=0 时恒 0)
            if_id1_pred_tgt   <= btb1_pred_tgt; // ★ §5d (pred_applied1=0 时无意义)
        end
        // 停顿: 保持
    end

    //-----------------------------------------------------------------
    // ID/EX 流水线寄存器
    //-----------------------------------------------------------------
    // For an even predicted target T, (rs1 + imm) & ~1 == T iff
    // rs1 == T-imm or rs1 == (T|1)-imm (all arithmetic modulo 2^32).
    // The low bit selects the candidate; subtract in ID, compare in EX.
    // This leaves the real JALR target and all forwarding/issue rules intact.
    wire [31:0] id_jbase0 = if_id_pred_tgt - id_imm;
    wire [31:0] id_jbase1 = (if_id_pred_tgt | 32'd1) - id_imm;
    wire [31:0] id1_jbase0 = if_id1_pred_tgt - id1_imm;
    wire [31:0] id1_jbase1 = (if_id1_pred_tgt | 32'd1) - id1_imm;
    always @(posedge clk) begin
        if (!resetn || id_ex_clear) begin
            id_ex_jbase0_q <= 31'b0; id_ex_jbase1_q <= 31'b0;
            id_ex1_jbase0_q <= 31'b0; id_ex1_jbase1_q <= 31'b0;
        end else if (pc_en) begin
            id_ex_jbase0_q <= id_jbase0[31:1];
            id_ex_jbase1_q <= id_jbase1[31:1];
            id_ex1_jbase0_q <= id1_jbase0[31:1];
            id_ex1_jbase1_q <= id1_jbase1[31:1];
        end
    end
    always @(posedge clk) begin
        if (!resetn) begin
            id_ex_valid <= 1'b0;
            id_ex_pc         <= 32'h0;
            id_ex_rs1_data   <= 32'h0;
            id_ex_rs2_data   <= 32'h0;
            id_ex_imm        <= 32'h0;
            id_ex_mem_addr_pre <= 32'h0;
            id_ex_rd         <=  5'h0;
            id_ex_rs1        <=  5'h0;
            id_ex_rs2        <=  5'h0;
            id_ex_alu_src    <=  1'b0;
            id_ex_alu_a_pc   <=  1'b0;
            id_ex_alu_a_zero <=  1'b0;
            id_ex_alu_op     <=  4'h0;
            id_ex_branch     <=  1'b0;
            id_ex_jal        <=  1'b0;
            id_ex_jalr       <=  1'b0;
            id_ex_mem_read   <=  1'b0;
            id_ex_mem_write  <=  1'b0;
            id_ex_reg_write  <=  1'b0;
            id_ex_mem_to_reg <=  1'b0;
            id_ex_funct3     <=  3'b0;
            id_ex_csr_en <= 1'b0; id_ex_csr_imm <= 1'b0; id_ex_csr_mode <= 2'b00;
            id_ex_csr_addr <= 12'h0; id_ex_mret <= 1'b0; id_ex_ecall <= 1'b0;
            id_ex_ebreak <= 1'b0; id_ex_illegal <= 1'b0;
            id_ex_pred_taken <= 1'b0; id_ex_pred_tgt <= 32'h0;
        end else if (id_ex_clear) begin
            // 冲刷: 插完整 bubble (valid 及全部控制信号清零)
            id_ex_valid     <= 1'b0;
            id_ex_mem_addr_pre <= 32'h0;   // ★ 阶段0: bubble 时清零
            id_ex_rd        <=  5'h0;
            id_ex_rs1       <=  5'h0;
            id_ex_rs2       <=  5'h0;
            id_ex_alu_src   <= 1'b0;
            id_ex_alu_a_pc  <= 1'b0;
            id_ex_alu_a_zero<= 1'b0;
            id_ex_alu_op    <= 4'b0;
            id_ex_branch    <= 1'b0;
            id_ex_jal       <= 1'b0;
            id_ex_jalr      <= 1'b0;
            id_ex_mem_read  <= 1'b0;
            id_ex_mem_write <= 1'b0;
            id_ex_reg_write <= 1'b0;
            id_ex_mem_to_reg<= 1'b0;
            id_ex_funct3    <= 3'b0;
            id_ex_csr_en <= 1'b0; id_ex_csr_imm <= 1'b0; id_ex_csr_mode <= 2'b00;
            id_ex_csr_addr <= 12'h0; id_ex_mret <= 1'b0; id_ex_ecall <= 1'b0;
            id_ex_ebreak <= 1'b0; id_ex_illegal <= 1'b0;
            id_ex_pred_taken <= 1'b0; id_ex_pred_tgt <= 32'h0;
        end else if (pc_en) begin          // 非停顿: 装载 ID 级
            id_ex_valid     <= if_id_valid;
            id_ex_pc        <= if_id_pc;
            id_ex_pred_taken <= if_id_pred_taken;   // M2 (bubble 时 if_id_pred_taken 已为 0)
            id_ex_pred_tgt   <= if_id_pred_tgt;
            if (if_id_valid) begin
                id_ex_rs1_data  <= id_rs1_data;   // regfile 异步读
                id_ex_rs2_data  <= id_rs2_data;
                id_ex_imm       <= id_imm;        // imm_gen
                id_ex_mem_addr_pre <= id_mem_addr; // ★ 阶段0: ID 级投机地址
                id_ex_rd        <= if_id_instr[11:7];
                id_ex_rs1       <= if_id_instr[19:15];
                id_ex_rs2       <= if_id_instr[24:20];
                id_ex_alu_src   <= id_alu_src;
                id_ex_alu_a_pc  <= id_alu_a_pc;
                id_ex_alu_a_zero<= id_alu_a_zero;
                id_ex_alu_op    <= id_alu_op;
                id_ex_branch    <= id_branch;
                id_ex_jal       <= id_jal;
                id_ex_jalr      <= id_jalr;
                id_ex_mem_read  <= id_mem_read;
                id_ex_mem_write <= id_mem_write;
                id_ex_reg_write <= id_reg_write;
                id_ex_mem_to_reg<= id_mem_to_reg;
                id_ex_funct3    <= if_id_instr[14:12];
                id_ex_csr_en    <= id_csr_en;
                id_ex_csr_imm   <= id_csr_imm;
                id_ex_csr_mode  <= id_csr_mode;
                id_ex_csr_addr  <= if_id_instr[31:20];
                id_ex_mret      <= id_mret;
                id_ex_ecall     <= id_ecall;
                id_ex_ebreak    <= id_ebreak;
                id_ex_illegal   <= id_illegal;
            end else begin
                // bubble: 冲刷后 IF/ID 仍持有陈旧 instr, 若直接解码会把 rd/
                // reg_write/alu 结果泄漏进 EX/MEM 并被转发 (曾致 ret 返回地址
                // 被转发成栈值 0x1fe78). 这里对无效 IF/ID 插干净 bubble.
                id_ex_rs1_data  <= 32'h0;
                id_ex_rs2_data  <= 32'h0;
                id_ex_imm       <= 32'h0;
                id_ex_mem_addr_pre <= 32'h0;   // ★ 阶段0
                id_ex_rd        <=  5'h0;
                id_ex_rs1       <=  5'h0;
                id_ex_rs2       <=  5'h0;
                id_ex_alu_src   <= 1'b0;
                id_ex_alu_a_pc  <= 1'b0;
                id_ex_alu_a_zero<= 1'b0;
                id_ex_alu_op    <= 4'h0;
                id_ex_branch    <= 1'b0;
                id_ex_jal       <= 1'b0;
                id_ex_jalr      <= 1'b0;
                id_ex_mem_read  <= 1'b0;
                id_ex_mem_write <= 1'b0;
                id_ex_reg_write <= 1'b0;
                id_ex_mem_to_reg<= 1'b0;
                id_ex_funct3    <= 3'b0;
            id_ex_csr_en <= 1'b0; id_ex_csr_imm <= 1'b0; id_ex_csr_mode <= 2'b00;
            id_ex_csr_addr <= 12'h0; id_ex_mret <= 1'b0; id_ex_ecall <= 1'b0;
            id_ex_ebreak <= 1'b0; id_ex_illegal <= 1'b0;
            end
        end
        // 停顿: 保持
    end

    //-----------------------------------------------------------------
    // ★★ D2b: ID/EX **slot1** 流水线寄存器
    //   与 slot0 同条件 (reset / id_ex_clear / pc_en), 只有装载时多一个
    //   `if_id_pair` 门: 没配对时 instr1 是陈旧数据, 绝不能让它的 rd/reg_write
    //   泄漏进 EX/MEM (与当年 slot0 的 "bubble 必须清干净" 同一个坑)。
    //-----------------------------------------------------------------
    always @(posedge clk) begin
        if (!resetn) begin
            id_ex1_valid <= 1'b0;
            id_ex1_pc <= 32'h0; id_ex1_rs1_data <= 32'h0; id_ex1_rs2_data <= 32'h0;
            id_ex1_imm <= 32'h0; id_ex1_rd <= 5'h0; id_ex1_rs1 <= 5'h0;
            id_ex1_rs2 <= 5'h0; id_ex1_alu_src <= 1'b0; id_ex1_alu_a_pc <= 1'b0;
            id_ex1_alu_a_zero <= 1'b0; id_ex1_alu_op <= 4'h0; id_ex1_reg_write <= 1'b0;
            id_ex1_mem_read <= 1'b0; id_ex1_mem_write <= 1'b0; id_ex1_mem_to_reg <= 1'b0;
            id_ex1_funct3 <= 3'h0; id_ex1_mem_addr_pre <= 32'h0;
            id_ex1_branch <= 1'b0; id_ex1_jal <= 1'b0; id_ex1_jalr <= 1'b0;   // ★ §5d
            id_ex1_pred_taken <= 1'b0; id_ex1_pred_tgt <= 32'h0;              // ★ §5d
        end else if (id_ex_clear) begin
            id_ex1_valid <= 1'b0;
            id_ex1_rd <= 5'h0; id_ex1_rs1 <= 5'h0; id_ex1_rs2 <= 5'h0;
            id_ex1_alu_src <= 1'b0; id_ex1_alu_a_pc <= 1'b0; id_ex1_alu_a_zero <= 1'b0;
            id_ex1_alu_op <= 4'h0; id_ex1_reg_write <= 1'b0;
            id_ex1_mem_read <= 1'b0; id_ex1_mem_write <= 1'b0; id_ex1_mem_to_reg <= 1'b0;
            id_ex1_funct3 <= 3'h0; id_ex1_mem_addr_pre <= 32'h0;
            id_ex1_branch <= 1'b0; id_ex1_jal <= 1'b0; id_ex1_jalr <= 1'b0;   // ★ §5d
            id_ex1_pred_taken <= 1'b0; id_ex1_pred_tgt <= 32'h0;              // ★ §5d
        end else if (pc_en) begin
            id_ex1_valid <= if_id_valid & if_id_pair;
            if (if_id_valid & if_id_pair) begin
                id_ex1_pc        <= if_id_pc1;          // ★ #3: slot1 的真实 pc (顺序=pc+4, 插入=T)
                id_ex1_branch    <= id1_branch;         // ★ §5d: P2 放行
                id_ex1_jal       <= id1_jal;
                id_ex1_jalr      <= id1_jalr;
                id_ex1_pred_taken<= if_id1_pred_taken;  // ★ §5d: slot1 IF 预测携带
                id_ex1_pred_tgt  <= if_id1_pred_tgt;
                id_ex1_rs1_data  <= id1_rs1_data;
                id_ex1_rs2_data  <= id1_rs2_data;
                id_ex1_imm       <= id1_imm;
                id_ex1_rd        <= if_id_instr1[11:7];
                id_ex1_rs1       <= if_id_instr1[19:15];
                id_ex1_rs2       <= if_id_instr1[24:20];
                id_ex1_alu_src   <= id1_alu_src;
                id_ex1_alu_a_pc  <= id1_alu_a_pc;
                id_ex1_alu_a_zero<= id1_alu_a_zero;
                id_ex1_alu_op    <= id1_alu_op;
                id_ex1_reg_write <= id1_reg_write;
                id_ex1_mem_read  <= id1_mem_read;
                id_ex1_mem_write <= id1_mem_write;
                id_ex1_mem_to_reg<= id1_mem_to_reg;
                id_ex1_funct3    <= if_id_instr1[14:12];
                id_ex1_mem_addr_pre <= id1_mem_addr;
            end else begin
                id_ex1_rs1_data  <= 32'h0; id_ex1_rs2_data <= 32'h0; id_ex1_imm <= 32'h0;
                id_ex1_rd <= 5'h0; id_ex1_rs1 <= 5'h0; id_ex1_rs2 <= 5'h0;
                id_ex1_alu_src <= 1'b0; id_ex1_alu_a_pc <= 1'b0; id_ex1_alu_a_zero <= 1'b0;
                id_ex1_alu_op <= 4'h0; id_ex1_reg_write <= 1'b0;
                id_ex1_mem_read <= 1'b0; id_ex1_mem_write <= 1'b0; id_ex1_mem_to_reg <= 1'b0;
                id_ex1_funct3 <= 3'h0; id_ex1_mem_addr_pre <= 32'h0;
                id_ex1_branch <= 1'b0; id_ex1_jal <= 1'b0; id_ex1_jalr <= 1'b0;   // ★ §5d
                id_ex1_pred_taken <= 1'b0; id_ex1_pred_tgt <= 32'h0;              // ★ §5d
            end
        end
        // 停顿: 保持
    end

    //-----------------------------------------------------------------
    // EX/MEM 流水线寄存器 (接入 alu.v + 转发)
    //-----------------------------------------------------------------
    // ALU 组合输出
    wire [31:0] alu_result;
    wire [31:0] alu_addr_result;
    wire [31:0] alu_addr_result1;
    wire        alu_branch_taken;

    //-----------------------------------------------------------------
    // ★★ 阶段1: MEM 级 load 值「移位 + 符号/零扩展」(组合)
    //   这两级原在 WB 级 (mem_wb_mem_rdata 之后); 措施2 已挪到 MEM 级寄存器之前。
    //   本阶段把它**提前到转发源声明之前**，好让 EX/MEM 转发在 load 时直接取用。
    //-----------------------------------------------------------------
    // ★★ P3': load 可能来自 slot0 或 slot1 (P3 保证只有一条)
    //   只 mux funct3/byte_off, **数据通路 d_rdata 不变** ⇒ 不加关键路径级数
    wire [ 2:0] ld_f3  = ex_mem_mem_read ? ex_mem_funct3   : ex_mem1_funct3;
    wire [ 1:0] ld_off = ex_mem_mem_read ? ex_mem_byte_off : ex_mem1_byte_off;
    wire [31:0] ram_load_value;
    wire [31:0] sb_load_value[0:3];
    load_format u_load_ram(.word_in(d_rdata),.byte_off(ld_off),.funct3(ld_f3),.value(ram_load_value));
    genvar lf;
    generate for(lf=0;lf<4;lf=lf+1) begin:g_load_format
        load_format u_format(.word_in(sb_data[lf]),.byte_off(ld_off),.funct3(ld_f3),.value(sb_load_value[lf]));
    end endgenerate
    // sb_fwd_hit guarantees exactly one matching full-word entry. Therefore
    // the extension is identical, while CAM and formatting run in parallel.
    wire [31:0] sb_load_value_selected = ({32{sb_m0}} & sb_load_value[0]) |
        ({32{sb_m1}} & sb_load_value[1]) | ({32{sb_m2}} & sb_load_value[2]) |
        ({32{sb_m3}} & sb_load_value[3]);
    wire [31:0] mem_load_ext=sb_fwd_hit?sb_load_value_selected:ram_load_value;

    //-----------------------------------------------------------------
    // ★★★ 2026-10-08 (第四次修时序违例): **把 load 旁路 mux 搬进 ALU**
    //   背景: 关键路径 (WNS −0.906 @ 108 MHz, `project/report_timing/q5_top.rtr`) 是
    //     dcache 数据阵列 → d_rdata → 移位/扩展 → 转发 mux → **branch_target 加法器**
    //       → pred_ok 32 位比较 → branch_taken_eff → next_pc → pc → pf.next_issue
    //   根因: `branch_target = (alu_a + id_ex_imm) & ~1`, 而 `alu_a` 含 2'b11
    //   (load 旁路) 项 ⇒ **代码里就把 load 数据接进了分支目标**。
    //   功能上不可能: branch_target 只在 branch/jal/jalr 用, 而 jalr 属 ctrl_alu
    //   ⇒ hazard_unit 必有停顿 ⇒ forward_* 不会是 2'b11。但"代码连了就是连了",
    //   且 §5.5 已证明: 只要两条路逻辑等价, 综合器就会 **CSE 合并回去**,
    //   所以单开 a_br/b_br 那种"改名"式硬切无效。
    //   ⇒ 本轮唯一可靠的办法: 让 load 值**物理上**进不了分支逻辑。
    //     把 4:1 转发 mux 拆成两半:
    //       · `alu_a`/`alu_b` = **3:1, 纯寄存器源** ⇒ 喂 ALU 的 a/b 端口,
    //         同时喂 `branch_target` 加法器 ⇒ 分支路径逻辑上就没有 load 数据;
    //       · `mem_load_ext` 走**专用端口** a_ld/b_ld (晚到), 在 ALU **内部**
    //         才与 result 路径汇合 ⇒ 只影响 result。
    //     ALU 侧加 `syn_hier="hard"` 硬边界 ⇒ 杜绝跨模块 CSE 把 ax 与 a 合并。
    //   阶段1 的收益 (取消 load-use 停顿) 完全保留 —— 旁路仍然生效, 只是换了入口。
    //-----------------------------------------------------------------
    wire [31:0] fwd_mem_alu = wb_wdata;            // MEM/WB 写回数据 (含 load 值)

    // ★★★ 2026-10-08 修复 (既有 bug): EX/MEM 转发的值**不能**直接取 `ex_mem_alu_result`。
    //   对 **jal/jalr**, 架构写回值是 **pc+4** (mem_to_reg=1 且 mem_read=0),
    //   而 `ex_mem_alu_result` 里装的是**跳转目标** ✗。
    //   单发射时这条转发几乎不命中 (jal 之后立刻就是目标, x1 的消费者在很后面),
    //   一直被寄存器堆掩盖; P3' 让 `sw` 进 slot1 后, sw 的 EX 拍 EX/MEM 里
    //   正好压着调用者的 `jal` ⇒ 把跳转目标当 `ra` 存进了栈
    //   (实测 `sw ra,28(sp)` 存成 0xffffff84 而不是 0xa8)。
    // ★ slot0 在 EX 级的"可转发写回值": jal/jalr 是 pc+4, 其余是 ALU 结果
    wire [31:0] slot0_ex_fw_val = (id_ex_jal | id_ex_jalr) ? (id_ex_pc + 32'd4)
                                                           : alu_result;
    // p9_pend1 rejects a slot1 branch/JALR that consumes a slot0 result
    // dependent on MEM load bypass. Make that invariant structural: control
    // flow gets a separate slot0 ALU with no load-data input. The p_nraw_s
    // guard extends the same load-free invariant to slot1 arithmetic RAW.
    wire [31:0] slot0_ctrl_alu_result;
    wire [31:0] slot0_ctrl_fw_val = (id_ex_jal | id_ex_jalr)
                                  ? (id_ex_pc + 32'd4) : slot0_ctrl_alu_result;
    // ★ MUL 晚结果 (v2 乘法器): MEM 级占据者是 MUL 时, 其真值在 mul_result
    //   (p_*_q 寄存器输出, 该拍有效), ex_mem_alu_result 只是占位垃圾。
    //   (mul_result 在 ID 投机段前向声明, 行 ~333)
    wire [31:0] ex_mem_fw_val  = ex_mem_is_mul ? mul_result :
                                 (ex_mem_mem_to_reg  && !ex_mem_mem_read)
                                 ? ex_mem_pc_plus4  : ex_mem_alu_result;
    // ★ §5c: slot1 MUL 晚结果 —— 与 slot0 的 ex_mem_fw_val 同构 (is_mul 优先)
    wire [31:0] ex_mem1_fw_val = ex_mem1_is_mul ? mul_result1 :
                                 (ex_mem1_mem_to_reg && !ex_mem1_mem_read)
                                 ? ex_mem1_pc_plus4  : ex_mem1_alu_result;

    // ---- 5:1 转发 (不含 load 旁路): **纯寄存器源** ----
    //   ⚠⚠ D2b 血泪: forwarding_unit 现在会为 slot0 的端口返回 3'b010/3'b101
    //      (EX/MEM.slot1 / MEM/WB.slot1) —— 单发射时代只可能出 001/100。
    //      漏掉这两个码 ⇒ 静默落回寄存器堆旧值 ⇒ 取数全错、X 满屏。
    //      **转发编码必须与解码 mux 严格一一对应, 多一个码都不行。**
    wire [31:0] rs1_nb = (forward_a == 3'b001) ? ex_mem_fw_val     :
                         (forward_a == 3'b010) ? ex_mem1_fw_val    :
                         (forward_a == 3'b100) ? fwd_mem_alu       :
                         (forward_a == 3'b101) ? wb1_wdata         : id_ex_rs1_data;
    wire [31:0] rs2_nb = (forward_b == 3'b001) ? ex_mem_fw_val     :
                         (forward_b == 3'b010) ? ex_mem1_fw_val    :
                         (forward_b == 3'b100) ? fwd_mem_alu       :
                         (forward_b == 3'b101) ? wb1_wdata         : id_ex_rs2_data;

    wire [31:0] alu_a = id_ex_alu_a_zero ? 32'h0 :
                        (id_ex_alu_a_pc ? id_ex_pc : rs1_nb);
    wire [31:0] alu_b = id_ex_alu_src ? id_ex_imm : rs2_nb;

    // ---- load 旁路: 专用端口 (只在 ALU 的 result 路径生效) ----
    wire        alu_a_ld_en = (forward_a == 3'b011) && !id_ex_alu_a_zero && !id_ex_alu_a_pc;
    wire        alu_b_ld_en = (forward_b == 3'b011) && !id_ex_alu_src;

    // MUL is register-register whenever mul_go is asserted. Bypass the
    // unrelated PC/zero/immediate selection, preserving every forwarding source.
    wire [31:0] mul_a = (forward_a == 3'b011) ? mem_load_ext : rs1_nb;
    //   ⚠⚠ store_shift 用的 rs2 **不能**被 `id_ex_alu_src` 门掉！
    //     S 型指令的 alu_src=1 (地址 = rs1+imm), 但**写数据仍是 rs2** ⇒
    //     必须保留旁路。否则 `sb x2, 0(x1)` 紧跟 `lw x2,...` 会写错值。
    //     (2026-10-08 实测: 加错这道门 ⇒ CoreMark 在 4156 条指令后死锁。)
    wire [31:0] alu_rs2   = (forward_b == 3'b011) ? mem_load_ext : rs2_nb;

    //-----------------------------------------------------------------
    // ★★ 提速措施4: 两级流水乘法器（替代 alu.v 内部的 4 个单周期 32x32 乘法器）
    //   Q2 实测那 4 个乘法器串成 4 级 APM，纯组合约 9.5 ns ⇒ CPU 只能到 72.41 MHz。
    //   这里改成「第 1 级 4 个 17x17 部分积并行 → 第 2 级加法树」。v2 (2026-10-11):
    //   MUL 不再停顿 EX 级 —— 发射拍锁存, 结果晚一拍经 ex_mem_fw_val/WB 锁存取用。
    //   MUL inputs include load bypass and retain MEM/MUL/WB forwarding.
    // ★ mul_result 已在前向声明 (ex_mem_fw_val 处); 例化在此驱动。
    mul_unit u_mul (
        .clk      (clk),
        .resetn   (resetn),
        .en       (mul_go),          // 发射拍锁存部分积 (MUL 不再停顿, 结果晚一拍)
        .a        (mul_a),
        .b        (alu_rs2),
        .funct3   (id_ex_funct3),
        .result   (mul_result)
    );

    //   ⚠ slot1 MUL 的第二乘法器 (u_mul1) 例化在下方 alu_rs2_1 声明之后
    //     (Verilog 先声明后使用), 见 "slot1 MUL" 块。

    // ★ a/b 端口 = **3:1 纯寄存器源** (分支比较与 branch_target 都只依赖它);
    //   load 旁路走 a_ld/b_ld 专用端口, 只在 ALU 内部的 result 路径生效。
    alu u_alu (
        .a            (alu_a),
        .b            (alu_b),
        .a_ld         (mem_load_ext),
        .b_ld         (mem_load_ext),
        .a_ld_en      (alu_a_ld_en),
        .b_ld_en      (alu_b_ld_en),
        .alu_op       (id_ex_alu_op),
        .funct3       (id_ex_funct3),          // EX 级保存的分支条件
        .mul_result   (mul_result),            // ★ 措施4
        .result       (alu_result),
        .addr_result  (alu_addr_result),
        .branch_taken ()
    );

    // The pair guard rejects late-MUL-fed slot0 -> slot1 RAW. This copy
    // therefore uses the normal MEM register value without a MUL bypass.
    // The architectural slot0 ALU retains all real forwarding sources.
    wire [31:0] pair_mem0_val = pair_link0_q
                               ? ex_mem_pc_plus4 : ex_mem_alu_result;
    wire [31:0] pair_mem1_val = pair_link1_q
                               ? ex_mem1_pc_plus4 : ex_mem1_alu_result;
    wire [31:0] pair_rs1 = (forward_a == 3'b001) ? pair_mem0_val :
                          (forward_a == 3'b010) ? pair_mem1_val :
                          (forward_a == 3'b100) ? fwd_mem_alu :
                          (forward_a == 3'b101) ? wb1_wdata : id_ex_rs1_data;
    wire [31:0] pair_rs2 = (forward_b == 3'b001) ? pair_mem0_val :
                          (forward_b == 3'b010) ? pair_mem1_val :
                          (forward_b == 3'b100) ? fwd_mem_alu :
                          (forward_b == 3'b101) ? wb1_wdata : id_ex_rs2_data;
    wire [31:0] pair_alu_a = id_ex_alu_a_zero ? 32'b0 :
                            (id_ex_alu_a_pc ? id_ex_pc : pair_rs1);
    wire [31:0] pair_alu_b = id_ex_alu_src ? id_ex_imm : pair_rs2;
    alu #(.PAIR_LIGHT_RESULT(2)) u_alu_ctrl_fwd (
        .a(pair_alu_a), .b(pair_alu_b), .a_ld(32'b0), .b_ld(32'b0),
        .a_ld_en(1'b0), .b_ld_en(1'b0), .alu_op(id_ex_alu_op),
        .funct3(id_ex_funct3), .mul_result(32'b0),
        .result(slot0_ctrl_alu_result), .addr_result(alu_pred_addr0), .branch_taken(alu_branch_taken)
    );

    // ID addresses are speculative and checked against the complete actual
    // address by dcache. Keep their EX preview free of late load/MUL data;
    // direct MEM load prediction remains available without an intervening ALU.
    // This is only a speculative ID address preview. Never serialize EX0
    // through EX1 here; unmatched intra-pair sources predict their ID value.
    // Actual EX1 arithmetic retains 110 and cache validates the full address.
    wire [31:0] pred1_rs1 = (forward_a1 == 3'b001) ? pair_mem0_val :
                           (forward_a1 == 3'b010) ? pair_mem1_val :
                           (forward_a1 == 3'b100) ? fwd_mem_alu :
                           (forward_a1 == 3'b101) ? wb1_wdata : id_ex1_rs1_data;
    wire [31:0] pred1_rs2 = (forward_b1 == 3'b001) ? pair_mem0_val :
                           (forward_b1 == 3'b010) ? pair_mem1_val :
                           (forward_b1 == 3'b100) ? fwd_mem_alu :
                           (forward_b1 == 3'b101) ? wb1_wdata : id_ex1_rs2_data;
    wire [31:0] pred1_a = id_ex1_alu_a_zero ? 32'b0 :
                         (id_ex1_alu_a_pc ? id_ex1_pc : pred1_rs1);
    wire [31:0] pred1_b = id_ex1_alu_src ? id_ex1_imm : pred1_rs2;
    alu u_alu1_pred (
        .a(pred1_a), .b(pred1_b), .a_ld(32'b0), .b_ld(32'b0),
        .a_ld_en(1'b0), .b_ld_en(1'b0), .alu_op(id_ex1_alu_op),
        .funct3(id_ex1_funct3), .mul_result(32'b0),
        .result(), .addr_result(alu_pred_addr1), .branch_taken()
    );

    //-----------------------------------------------------------------
    // ★★ D2b slot1: 第二个 ALU (★ §5d: P2 放行后也做分支比较 —— 原注释
    //   "slot1 无分支条件/恒非 branch" 已失效; load 旁路不受 ctrl_alu 结构
    //   门限制这一点仍成立, 因为 slot1 分支的操作数已由配对规则保证不需
    //   load 旁路 (p_npend1/p_raw0_ok/p_njal0), "dcache → ALU → branch"
    //   假路径在 slot1 侧依旧不存在)
    //   操作数选择与 slot0 同构, 但转发源多两级 (EX/MEM.slot1 与 MEM/WB.slot1),
    //   因为双发射下"前面的指令"有 4 条。
    //-----------------------------------------------------------------
    wire [31:0] rs1_nb1 = (forward_a1 == 3'b110) ? slot0_ctrl_fw_val  :
                          (forward_a1 == 3'b001) ? ex_mem_fw_val      :
                          (forward_a1 == 3'b010) ? ex_mem1_fw_val     :
                          (forward_a1 == 3'b100) ? fwd_mem_alu        :
                          (forward_a1 == 3'b101) ? wb1_wdata          : id_ex1_rs1_data;
    wire [31:0] rs2_nb1 = (forward_b1 == 3'b110) ? slot0_ctrl_fw_val  :
                          (forward_b1 == 3'b001) ? ex_mem_fw_val      :
                          (forward_b1 == 3'b010) ? ex_mem1_fw_val     :
                          (forward_b1 == 3'b100) ? fwd_mem_alu        :
                          (forward_b1 == 3'b101) ? wb1_wdata          : id_ex1_rs2_data;
    wire [31:0] alu_a1 = id_ex1_alu_a_zero ? 32'h0 :
                         (id_ex1_alu_a_pc ? id_ex1_pc : rs1_nb1);
    wire [31:0] alu_b1 = id_ex1_alu_src ? id_ex1_imm : rs2_nb1;
    wire        alu_a1_ld_en = (forward_a1 == 3'b011) && !id_ex1_alu_a_zero && !id_ex1_alu_a_pc;
    wire        alu_b1_ld_en = (forward_b1 == 3'b011) && !id_ex1_alu_src;
    wire [31:0] alu_result1;
    wire        alu_branch_taken1;   // ★ §5d: slot1 分支比较输出 (P2 放行)
    wire [31:0] ctrl_rs1_nb1 = (forward_a1 == 3'b001) ? pair_mem0_val :
                              (forward_a1 == 3'b010) ? pair_mem1_val :
                              (forward_a1 == 3'b100) ? fwd_mem_alu :
                              (forward_a1 == 3'b101) ? wb1_wdata : id_ex1_rs1_data;
    wire [31:0] ctrl_rs2_nb1 = (forward_b1 == 3'b001) ? pair_mem0_val :
                              (forward_b1 == 3'b010) ? pair_mem1_val :
                              (forward_b1 == 3'b100) ? fwd_mem_alu :
                              (forward_b1 == 3'b101) ? wb1_wdata : id_ex1_rs2_data;
    wire [31:0] ctrl_alu_a1 = id_ex1_alu_a_zero ? 32'b0 :
                              (id_ex1_alu_a_pc ? id_ex1_pc : ctrl_rs1_nb1);
    wire [31:0] ctrl_alu_b1 = id_ex1_alu_src ? id_ex1_imm : ctrl_rs2_nb1;

    // Slot1 SLT/SLTU rejects intra-pair RAW. Omit EX0 only from comparison
    // operands, preserving real MEM/load/MUL/WB and normal arithmetic 110.
    wire [31:0] cmp1_rs1 = (forward_a1 == 3'b001) ? ex_mem_fw_val :
                           (forward_a1 == 3'b010) ? ex_mem1_fw_val :
                           (forward_a1 == 3'b011) ? mem_load_ext :
                           (forward_a1 == 3'b100) ? fwd_mem_alu :
                           (forward_a1 == 3'b101) ? wb1_wdata : id_ex1_rs1_data;
    wire [31:0] cmp1_rs2 = (forward_b1 == 3'b001) ? ex_mem_fw_val :
                           (forward_b1 == 3'b010) ? ex_mem1_fw_val :
                           (forward_b1 == 3'b011) ? mem_load_ext :
                           (forward_b1 == 3'b100) ? fwd_mem_alu :
                           (forward_b1 == 3'b101) ? wb1_wdata : id_ex1_rs2_data;
    wire [31:0] cmp1_b = id_ex1_alu_src ? id_ex1_imm : cmp1_rs2;
    alu #(.COMPARE_ALT_INPUT(1)) u_alu1 (
        .a            (alu_a1),
        .b            (alu_b1),
        .a_ld         (mem_load_ext),
        .b_ld         (mem_load_ext),
        .a_ld_en      (alu_a1_ld_en),
        .b_ld_en      (alu_b1_ld_en),
        .alu_op       (id_ex1_alu_op),
        .funct3       (id_ex1_funct3),    // ★ §5d: slot1 分支条件 (原 3'b000 空接)
        .mul_result   (mul_result1),       // ★ §5c: slot1 MUL (P4a 放行)
        .cmp_a        (cmp1_rs1),
        .cmp_b        (cmp1_b),
        .result       (alu_result1),
        .addr_result  (alu_addr_result1),
        .branch_taken ()
    );

    alu u_alu1_ctrl (
        .a(ctrl_alu_a1), .b(ctrl_alu_b1), .a_ld(32'b0), .b_ld(32'b0),
        .a_ld_en(1'b0), .b_ld_en(1'b0), .alu_op(id_ex1_alu_op),
        .funct3(id_ex1_funct3), .mul_result(32'b0),
        .result(), .addr_result(), .branch_taken(alu_branch_taken1)
    );

    // 分支/跳转重定向 (EX 级): 目标地址
    //   branch:  pc + imm (B型)
    //   jal:     pc + imm (J型)
    //   jalr:    alu_a + imm (rs1 已转发)
    always @(*) begin
        branch_target = 32'h0;
        if (id_ex_branch)
            branch_target = id_ex_pc + id_ex_imm;
        else if (id_ex_jal)
            branch_target = id_ex_pc + id_ex_imm;
        else if (id_ex_jalr)
            branch_target = (pair_alu_a + id_ex_imm) & ~32'h1; // late MUL resolves from a MEM snapshot
    end

    // branch_taken: ALU 分支结果 或 jal/jalr
    //   ⚠ 挂起分支 (ex_br_pend) 的本值是结构门下的占位垃圾 —— 上层
    //   branch_taken_eff / BTB 更新均已封锁, 此处不额外门控 (观测端已滤)。
    assign branch_taken = (id_ex_branch && alu_branch_taken) || id_ex_jal || id_ex_jalr;

    //-----------------------------------------------------------------
    // ★★ §5d (P2 松绑): slot1 分支/跳转的 EX 级解析 —— 与 slot0 同构。
    //   branch_target1 的 jalr 项用对内转发 (3'b110) 的 alu_a1: 配对规则
    //   保证 slot1 分支的操作数**绝不需要 load 旁路** (p_npend1 拆对 load
    //   相邻 / p_raw0_ok 拆对 slot0=load·MUL / p_njal0 拆对 slot0=jal·jalr)
    //   ⇒ alu_a1 恒为寄存器源, "load 数据 → pc" 长链在 slot1 侧不存在。
    //   ★ 2026-10-15 修正: "寄存器源"须是**传递闭包** —— B 的操作数被 L/M 位
    //     load 滋养时 (011), B 的结果仍是瞬态值, 经 110 → jalr 目标 → 重定向
    //     形成 20ns 链 (108M 实测最差路径)。p9_pend1 已扩展传递依赖拆对,
    //     本注释的"恒为寄存器源"在拆对语义下成立。
    //   branch_taken1 对挂起场景无需门控 —— slot1 分支永不挂起 (见上)。
    //   上层消费: pred_ok1/back_to_seq1/branch_taken_eff1 (older-wins)。
    //-----------------------------------------------------------------
    assign branch_taken1 = (id_ex1_branch && alu_branch_taken1)
                           || id_ex1_jal || id_ex1_jalr;

    assign jalr_pred_target_ok = !id_ex_pred_tgt[0] &&
        ((pair_alu_a[0] == id_ex_imm[0])
            ? (pair_alu_a[31:1] == id_ex_jbase0_q)
            : (pair_alu_a[31:1] == id_ex_jbase1_q));
    assign jalr1_pred_target_ok = !id_ex1_pred_tgt[0] &&
        ((ctrl_alu_a1[0] == id_ex1_imm[0])
            ? (ctrl_alu_a1[31:1] == id_ex1_jbase0_q)
            : (ctrl_alu_a1[31:1] == id_ex1_jbase1_q));

    always @(*) begin
        branch_target1 = 32'h0;
        if (id_ex1_branch)
            branch_target1 = id_ex1_pc + id_ex1_imm;             // B 型: pc + imm
        else if (id_ex1_jal)
            branch_target1 = id_ex1_pc + id_ex1_imm;             // J 型: pc + imm
        else if (id_ex1_jalr)
            branch_target1 = (ctrl_alu_a1 + id_ex1_imm) & ~32'h1; // no transitive load bypass
    end

    //-----------------------------------------------------------------
    // ★★★ 挂起判定 (EX 级, 2026-10-11 冲 5 §5):
    //   branch/jalr 的操作数是否需要「MEM 级 load 旁路」(转发码 raw 3'b011)。
    //   与 forwarding_unit 的优先级逐位对齐 (ex_mem1 程序序晚于 ex_mem0):
    //     raw 011 ⟺ m1_load 命中, 或 m1 未命中而 m0_load 命中
    //   结构门 (ctrl_alu) 把 m0 的 011 压成 001 (load 的**地址**), m1 的 011
    //   在 rs1_nb/rs2_nb 里无解码落回寄存器堆旧值 —— 两者都是错值 ⇒ 凡 raw
    //   011 一律挂起, EX 级比较/目标作废 (jal 目标与寄存器无关, 不挂起)。
    wire m1_ld_a  = ex_mem1_valid && ex_mem1_mem_read  && (ex_mem1_rd != 5'b0) && (ex_mem1_rd == id_ex_rs1);
    wire m1_ld_b  = ex_mem1_valid && ex_mem1_mem_read  && (ex_mem1_rd != 5'b0) && (ex_mem1_rd == id_ex_rs2);
    wire m1_any_a = ex_mem1_valid && ex_mem1_reg_write && (ex_mem1_rd != 5'b0) && (ex_mem1_rd == id_ex_rs1);
    wire m1_any_b = ex_mem1_valid && ex_mem1_reg_write && (ex_mem1_rd != 5'b0) && (ex_mem1_rd == id_ex_rs2);
    wire m0_ld_a  = ex_mem_valid && ex_mem_mem_read && ex_mem_reg_write &&
                    (ex_mem_rd != 5'b0) && (ex_mem_rd == id_ex_rs1);
    wire m0_ld_b  = ex_mem_valid && ex_mem_mem_read && ex_mem_reg_write &&
                    (ex_mem_rd != 5'b0) && (ex_mem_rd == id_ex_rs2);
    wire br_load_a = m1_ld_a | (~m1_any_a & m0_ld_a);
    wire br_load_b = m1_ld_b | (~m1_any_b & m0_ld_b);
    wire m1_mul_a = ex_mem1_valid && ex_mem1_is_mul && ex_mem1_reg_write &&
                     (ex_mem1_rd != 5'b0) && (ex_mem1_rd == id_ex_rs1);
    wire m1_mul_b = ex_mem1_valid && ex_mem1_is_mul && ex_mem1_reg_write &&
                     (ex_mem1_rd != 5'b0) && (ex_mem1_rd == id_ex_rs2);
    wire m0_mul_a = ex_mem_valid && ex_mem_is_mul && ex_mem_reg_write &&
                     (ex_mem_rd != 5'b0) && (ex_mem_rd == id_ex_rs1);
    wire m0_mul_b = ex_mem_valid && ex_mem_is_mul && ex_mem_reg_write &&
                     (ex_mem_rd != 5'b0) && (ex_mem_rd == id_ex_rs2);
    wire br_mul_a = m1_mul_a | (~m1_any_a & m0_mul_a);
    wire br_mul_b = m1_mul_b | (~m1_any_b & m0_mul_b);
    wire br_pend_a = br_load_a | br_mul_a;
    wire br_pend_b = br_load_b | br_mul_b;
    assign ex_br_pend = id_ex_valid && (id_ex_branch || id_ex_jalr) &&
                        (br_pend_a || br_pend_b);

    //-----------------------------------------------------------------
    // ★ branch/jalr MEM 级解析的**判定本体已前移**到 D 口段之前
    //   (mem_br_dif/mem_br_cmp/mem_br_mism_c 等, 2026-10-11 §5c) ——
    //   因为错路配对 load 的 D 口压制 (mem1_ld_supp — §5f-2 刀1 已撤除) 曾需要它在 MEM 驻留
    //   第一拍就有效, 不能等 adv。此处只保留挂起判定 (EX 级) 与快照捕获。
    //-----------------------------------------------------------------

    //-----------------------------------------------------------------
    // M2 BTB 本体 (32 项全相连, btb.v). 禁用时不例化, pred_* 保持 0.
    //   lk: IF 按 pc 组合查询; upd: EX 判定拍(adv 门控)单发, 两向更新.
    //   查询口在 btb 内对同拍更新做前递 (同地址读到更新后的槽).
    //-----------------------------------------------------------------
    reg          pf_lk_taken_r = 1'b0;
    reg  [31:0]  pf_lk_tgt_r   = 32'h0;
    assign pf_lk_taken = pf_lk_taken_r;
    assign pf_lk_tgt   = pf_lk_tgt_r;
    // ★ R1c-b: pf slot1 前瞻 (vf2 口, 与 pf_lk 同判据 ctr[1]; ret 不做 RAS
    //   覆盖 —— pf 侧 ret 目标漂移由 hint 对齐判定/resync 兜底, 与 slot0 同级)
    reg          pf_lk1_taken_r = 1'b0;
    reg  [31:0]  pf_lk1_tgt_r   = 32'h0;
    // ★ DBG-1e
    reg          pf_lk2_taken_r = 1'b0;
    reg  [31:0]  pf_lk2_tgt_r   = 32'h0;
    reg          pf_lk3_taken_r = 1'b0;
    reg  [31:0]  pf_lk3_tgt_r   = 32'h0;
    assign pf_lk1_taken = pf_lk1_taken_r;
    assign pf_lk1_tgt   = pf_lk1_tgt_r;
    assign pf_lk2_taken = pf_lk2_taken_r;
    assign pf_lk2_tgt   = pf_lk2_tgt_r;
    assign pf_lk3_taken = pf_lk3_taken_r;
    assign pf_lk3_tgt   = pf_lk3_tgt_r;

    generate if (BTB_ENABLE != 0) begin : g_btb
        wire        lk_hit;
        wire [31:0] lk_tgt;
        wire [ 1:0] lk_ctr;
        wire [ 1:0] lk_btype;
        // ★ R1c-b: pf slot1 前瞻口 (vf2) 线网
        wire        vf2_hit;
        wire [31:0] vf2_tgt;
        wire [ 1:0] vf2_ctr;
        // ★ DBG-1e
        wire        vf3_hit;
        wire [31:0] vf3_tgt;
        wire [ 1:0] vf3_ctr;
        wire        vf4_hit;
        wire [31:0] vf4_tgt;
        wire [ 1:0] vf4_ctr;
        // ★ §5f-3: 循环出口预测覆盖线网
        wire        lk_loop_nt, lk1_loop_nt, vf_loop_nt, vf2_loop_nt;
        // ★★ §5d (P2 松绑): slot1 预测的第二查询口 (按 slot1 的 pc=pc+4)
        wire        lk1_hit;
        wire [31:0] lk1_tgt;
        wire [ 1:0] lk1_ctr;
        wire [ 1:0] lk1_btype;
        wire        vf_hit;
        wire [31:0] vf_tgt;
        wire [ 1:0] vf_ctr;
        wire [ 1:0] vf_btype;
        // ★★ S1 预测器强化 (2026-10-11): **RAS (16 项返回栈)**。
        //   动机: `ret` = `jalr x0,0(ra)` 的目标是寄存器值, BTB 项里存的
        //   "上次返回地址"在递归/多调用点场景必错 —— CoreMark 误判 9.8% 的主力。
        //   规则: EX 级解析拍 push (call: jal/jalr 且 rd=x1/x5, 压 pc+4) /
        //   pop (ret: jalr 且 rs1=x1/x5, rd=x0); IF 预测命中 btype=ret 项时
        //   无条件预测 taken (返回必跳), 目标取 RAS 栈顶 (peek 不消耗, EX 弹)。
        //   栈空/深度超 16 → 预测落空, 走 EX 重定向兜底 (功能安全)。
        wire        ras_peek_v;
        wire [31:0] ras_peek;
        wire        ras_push, ras_pop;
        //   call = jal/jalr 且 rd=x1(xra)/x5(xt0); ret = jalr 且 rs1=x1/x5, rd=x0
        wire        is_call = (id_ex_jal | id_ex_jalr) &&
                              ((id_ex_rd == 5'd1) || (id_ex_rd == 5'd5));
        wire        is_ret  = id_ex_jalr &&
                              ((id_ex_rs1 == 5'd1) || (id_ex_rs1 == 5'd5)) &&
                              (id_ex_rd == 5'd0);
        // ★★ §5d (P2 松绑): slot1 的 call/ret 判定 (P2 放行后 slot1 也可以
        //   是 jal/jalr)
        wire        is_call1 = (id_ex1_jal | id_ex1_jalr) &&
                               ((id_ex1_rd == 5'd1) || (id_ex1_rd == 5'd5));
        wire        is_ret1  = id_ex1_jalr &&
                               ((id_ex1_rs1 == 5'd1) || (id_ex1_rs1 == 5'd5)) &&
                               (id_ex1_rd == 5'd0);
        // ★★ 2026-10-11 (冲 5 §5): 挂起分支的 BTB 更新**延迟到 MEM 解析拍**。
        //   EX 拍的 branch_taken/branch_target 对挂起分支是占位垃圾, 不能入表;
        //   MEM 拍用解析真值更新 (upd_pc/ Taken/Tgt/Btype 全取快照)。
        //   ★★ §5d: 更新口三源仲裁 (单更新口, 更老优先, 丢更新只损预测质量):
        //     MEM 挂起分支 > EX slot0 > EX slot1。
        //   slot1 侧让位/封锁条件: slot0 重定向 (eff0) / trap / mret 拍 slot1
        //   是错路指令, 其解析与更新作废; MEM 校正拍同理。
        wire btb_upd_mem = mem_br_fire & mem_br_known;
        wire btb_upd_ex  = ctrl_resolved & adv & ~ex_br_pend
                           & ~mem_br_mismatch & ~btb_upd_mem;
        wire btb_upd_ex1 = ctrl_resolved1 & adv & ~branch_taken_eff
                           & ~trap_fire & ~mret_fire & ~mem_br_mismatch
                           & ~btb_upd_mem & ~btb_upd_ex;
        // Prepare payload arbitration without advance. Whenever the
        // unchanged event enable is 1, adv is 1 and these select exactly
        // the legacy MEM > EX0 > EX1 source. Invalid-cycle data is unused.
        wire btb_data_mem = ex_mem_valid && ex_mem_br_pend && mem_br_known;
        wire btb_data_ex = ctrl_resolved & ~ex_br_pend & ~mem_br_mism_c & ~btb_data_mem;
        btb u_btb (
            .clk(clk), .resetn(resetn),
            .lk_addr(pc), .lk_hit(lk_hit), .lk_tgt(lk_tgt), .lk_ctr(lk_ctr),
            .lk1_addr(pc + 32'd4), .lk1_hit(lk1_hit), .lk1_tgt(lk1_tgt),
            .lk1_ctr(lk1_ctr), .lk1_btype(lk1_btype),
            .vf_addr(pf_lk_addr), .vf_hit(vf_hit), .vf_tgt(vf_tgt), .vf_ctr(vf_ctr),
            // ★ R1c-b: pf slot1 前瞻 = 链地址 +4 (顺序包 slot1 的 pc)
            .vf2_addr(pf_lk_addr + 32'd4), .vf2_hit(vf2_hit),
            // ★ DBG-1e: 前瞻口 (查 pf 的 next_issue / +4)
            .vf3_addr(pf_lk2_addr), .vf3_hit(vf3_hit),
            .vf3_tgt(vf3_tgt), .vf3_ctr(vf3_ctr),
            .vf4_addr(pf_lk2_addr + 32'd4), .vf4_hit(vf4_hit),
            .vf4_tgt(vf4_tgt), .vf4_ctr(vf4_ctr),
            .vf2_tgt(vf2_tgt), .vf2_ctr(vf2_ctr),
            // ★ §5f-3: 循环出口预测覆盖 (四口"预测不跳")
            .lk_loop_nt(lk_loop_nt), .lk1_loop_nt(lk1_loop_nt),
            .vf_loop_nt(vf_loop_nt), .vf2_loop_nt(vf2_loop_nt),
            .upd_en(btb_upd_mem | btb_upd_ex | btb_upd_ex1),
            .upd_pc   (btb_data_mem ? ex_mem_br_pc   : btb_data_ex ? id_ex_pc      : id_ex1_pc),
            .upd_taken(btb_data_mem ? mem_br_take1   : btb_data_ex ? branch_taken  : branch_taken1),
            .upd_tgt  (btb_data_mem ? mem_br_tgt_raw : btb_data_ex ? branch_target : branch_target1),
            .upd_btype(btb_data_mem ? ex_mem_br_btype : btb_data_ex ? {is_ret, is_call}
                                                                  : {is_ret1, is_call1}),
            .lk_btype(lk_btype), .vf_btype(vf_btype)
        );
        // ★★ §5d: RAS 覆盖 slot1 的 call/ret —— 同拍双 RAS 操作时 slot0
        //   (程序序更老) 优先, slot1 让位 (丢更新只损预测质量不出错)。
        //   slot1 的封锁条件与 BTB 更新口一致 (slot0 事件 ⇒ slot1 错路)。
        wire s0_ras_act = adv & id_ex_valid & (is_call | is_ret)
                          & ~trap_fire & ~mem_br_mismatch & ~ex_redirect_q;
        wire s1_ras_act = adv & id_ex1_valid & (is_call1 | is_ret1)
                          & ~branch_taken_eff & ~trap_fire & ~mem_br_mismatch & ~ex_redirect_q;
        reg ras_push_q, ras_pop_q;
        reg [31:0] ras_push_data_q;
        always @(posedge clk) begin
            if (!resetn) begin
                ras_push_q <= 1'b0; ras_pop_q <= 1'b0;
                ras_push_data_q <= 32'b0;
            end else begin
                // Legacy RAS ignores X-valued commands in procedural ifs.
                ras_push_q <= (ras_push === 1'b1);
                ras_pop_q <= (ras_pop === 1'b1);
                ras_push_data_q <= s0_ras_act ? (id_ex_pc + 32'd4) : (id_ex1_pc + 32'd4);
            end
        end
        ras #(.FORWARD_PENDING(1)) u_ras (
            .clk(clk), .resetn(resetn),
            .push(ras_push_q),
            .push_data(ras_push_data_q),
            .pop(ras_pop_q),
            .peek_v(ras_peek_v), .peek(ras_peek)
        );
        // ★ 2026-10-11: MEM 校正拍 id_ex 是错路指令, 其 call/ret 不得动 RAS
        //   (挂起分支自身的 push/pop 在其 EX 拍照常进行 —— push 数据=pc+4
        //    与解析结果无关, pop 消耗的栈顶已在 IF peek 时使用过)。
        assign ras_push = (s0_ras_act & is_call) | (~s0_ras_act & s1_ras_act & is_call1);
        assign ras_pop  = (s0_ras_act & is_ret)  | (~s0_ras_act & s1_ras_act & is_ret1);
        always @(*) begin
            // ★ ret 项: 无条件预测 taken (返回必跳), 目标取 RAS 栈顶;
            //   栈空则不预测 (落回 EX 重定向兜底)。其余项: ctr[1] 强 taken。
            //   ★ §5f-3: 循环出口覆盖 —— lp 高置信且本圈跳满 ⇒ 不跳 (覆盖 bimodal)。
            pred_strong_q = lk_hit && ((lk_btype == 2'b10) ? ras_peek_v
                                                          : (lk_ctr[1] && !lk_loop_nt));
            pred_tgt_q    = (lk_btype == 2'b10) ? ras_peek : lk_tgt;
            // ★★ §5d (P2 松绑): slot1 预测 —— 判据与 slot0 同构 (lk1 口,
            //   ret 项同样取 RAS 栈顶)。应用条件见 pred_applied1 (仅配对顺
            //   序包); 插入包的 slot1(=T) 不经此口预测。
            pred1_strong_q = lk1_hit && ((lk1_btype == 2'b10) ? ras_peek_v
                                                              : (lk1_ctr[1] && !lk1_loop_nt));
            pred1_tgt_q    = (lk1_btype == 2'b10) ? ras_peek : lk1_tgt;
            // ★ #3: 预取器查询口 —— 与 lk 口同判据 (ctr[1] = strongly taken)
            //   ★ §5f-3: 循环出口覆盖同步作用于 pf 视角 (否则 CPU 正确不跳而
            //   pf 照旧插入 T ⇒ resync 抵消收益)
            pf_lk_taken_r = vf_hit && vf_ctr[1] && !vf_loop_nt;
            pf_lk_tgt_r   = vf_tgt;
            // ★ R1c-b: pf slot1 前瞻 (vf2 口 = 链地址+4), 判据同 slot0
            pf_lk1_taken_r = vf2_hit && vf2_ctr[1] && !vf2_loop_nt;
            pf_lk1_tgt_r   = vf2_tgt;
            // ★ DBG-1e: 前瞻结果 (组合出 pf, pf 侧锁存; loop_nt 不接——前瞻口
            //   服务插入预测, 循环覆盖信号对锁存预测的语义待定版再补)
            pf_lk2_taken_r = vf3_hit && vf3_ctr[1];
            pf_lk2_tgt_r   = vf3_tgt;
            pf_lk3_taken_r = vf4_hit && vf4_ctr[1];
            pf_lk3_tgt_r   = vf4_tgt;
        end
    end endgenerate

    //-----------------------------------------------------------------
    // 窄访存对齐 (EX 级): 字节/半字 load/store
    //   sw/sh/sb 按地址低位把 store 数据左移对齐到字
    //   ex_mem_store_data 存对齐后的数据 (MEM 直接写整字, wstrb 选字节)
    //   地址清低 2 位对齐到字
    //-----------------------------------------------------------------
    // Actual memory addresses are always forwarded rs1 + sign-extended
    // immediate. Keep unrelated ALU shift/result selection out of this path.
    // mul_a contains every normal/MEM/load/WB forwarding source for rs1.
    wire [31:0] mem_addr_sum = mul_a + id_ex_imm;
    wire [1:0] mem_byte_off = mem_addr_sum[1:0];
    reg  [31:0] store_shift;                          // 对齐后的 store 数据
    reg  [ 3:0] mem_wstrb_pre;                        // EX 级算 wstrb
    wire [31:0] mem_addr_word;                        // 字对齐地址
    always @(*) begin
        case (id_ex_funct3)
            `F3_SB: begin
                store_shift   = alu_rs2 << {mem_byte_off, 3'b000};
                mem_wstrb_pre = 4'b0001 << mem_byte_off;
            end
            `F3_SH: begin
                // 半字需 2 字节对齐 (off[0]=0), 数据左移 2 的整数倍
                // off[1]=1 时半字放 [31:16], 需左移 16 (off[1] 放 bit4);
                // 原写 bit3 只移 8 / bit2 只移 4, 高字节错位 (0x2222 写成 0x0022)
                store_shift   = alu_rs2 << {1'b0, mem_byte_off[1], 4'b0000};
                mem_wstrb_pre = 4'b0011 << {2'b00, mem_byte_off[1], 1'b0};
            end
            `F3_SW: begin
                store_shift   = alu_rs2;
                mem_wstrb_pre = 4'b1111;
            end
            default: begin
                store_shift   = alu_rs2;
                mem_wstrb_pre = 4'b0000;   // load: 不写
            end
        endcase
    end
    assign mem_addr_word = {mem_addr_sum[31:2], 2'b00};

    // ★★ P3': slot1 的 store 对齐 / 字地址
    wire [31:0] alu_rs2_1 = (forward_b1 == 3'b011) ? mem_load_ext : rs2_nb1;

    //-----------------------------------------------------------------
    // ★★ slot1 MUL (2026-10-11 §5c, P4a 放行): 第二乘法器实例, 与 u_mul
    //   结构相同、完全独立 (同拍 (MUL,MUL) 对各自锁存互不干扰)。机制与
    //   slot0 MUL_LATE 镜像: 发射拍 (mul_go1 = adv) 锁存部分积, 结果晚一拍
    //   (mul_result1), 经 ex_mem1_is_mul → mem_wb1_alu_result 与
    //   ex_mem1_fw_val (转发码 3'b010) 两处注入。
    //   ★ 用含 load 旁路的寄存器操作数 (mul_a1 / alu_rs2_1) —— slot1 的 load
    //     旁路本就不受 ctrl_alu 结构门限制 (P2: slot1 恒非分支)。
    //   ⚠ 对内 RAW (slot0=MUL) 仍被 P5'' 拆对 (p_raw0_ok=~c0_is_mul):
    //     mul_result 在对内转发的 EX 拍是占位垃圾, 与 slot0 时代同理。
    //   mul_go1 无需 ~mem_br_mismatch 门: slot0 挂起分支失配时本指令随
    //   ex_mem1 捕获门消失, 乘法器里残留的部分积下次发射自然覆盖 (无副作用)。
    //-----------------------------------------------------------------
    // pair_mul1 excludes 110 only for MUL. Architectural non-MUL slot1
    // arithmetic still consumes its original same-pair forwarding source.
    wire [31:0] mul_a1 = (forward_a1 == 3'b001) ? ex_mem_fw_val :
                         (forward_a1 == 3'b010) ? ex_mem1_fw_val :
                         (forward_a1 == 3'b011) ? mem_load_ext :
                         (forward_a1 == 3'b100) ? fwd_mem_alu :
                         (forward_a1 == 3'b101) ? wb1_wdata : id_ex1_rs1_data;
    wire [31:0] mul_b1 = (forward_b1 == 3'b001) ? ex_mem_fw_val :
                         (forward_b1 == 3'b010) ? ex_mem1_fw_val :
                         (forward_b1 == 3'b011) ? mem_load_ext :
                         (forward_b1 == 3'b100) ? fwd_mem_alu :
                         (forward_b1 == 3'b101) ? wb1_wdata : id_ex1_rs2_data;
    wire mul_go1      = adv & id_ex1_is_mul;
    mul_unit u_mul1 (
        .clk      (clk),
        .resetn   (resetn),
        .en       (mul_go1),         // 发射拍锁存部分积 (与 slot0 同机制)
        .a        (mul_a1),
        .b        (mul_b1),
        .funct3   (id_ex1_funct3),
        .result   (mul_result1)
    );
    wire [31:0] mem_addr_sum1 = ((forward_a1 == 3'b011) ? mem_load_ext : rs1_nb1) + id_ex1_imm;
    wire [1:0]  mem_byte_off1   = mem_addr_sum1[1:0];
    reg  [31:0] store_shift1;
    reg  [ 3:0] mem_wstrb_pre1;
    wire [31:0] mem_addr_word1  = {mem_addr_sum1[31:2], 2'b00};
    always @(*) begin
        case (id_ex1_funct3)
            `F3_SB: begin
                store_shift1   = alu_rs2_1 << {mem_byte_off1, 3'b000};
                mem_wstrb_pre1 = 4'b0001 << mem_byte_off1;
            end
            `F3_SH: begin
                store_shift1   = alu_rs2_1 << {1'b0, mem_byte_off1[1], 4'b0000};
                mem_wstrb_pre1 = 4'b0011 << {2'b00, mem_byte_off1[1], 1'b0};
            end
            `F3_SW: begin
                store_shift1   = alu_rs2_1;
                mem_wstrb_pre1 = 4'b1111;
            end
            default: begin
                store_shift1   = alu_rs2_1;
                mem_wstrb_pre1 = 4'b0000;
            end
        endcase
    end

    // S6a-R: 预告 = 本拍 adv 且 id_ex 是 load/store (与 ex_mem 捕获同构)
    // ★ 阶段0 / Part A: 预告地址改用**寄存器化**的 ID 级投机地址
    //   (原为 `assign d_addr_pre = mem_addr_word;` —— EX 加法器组合直出,
    //    构成 14.627 ns 关键路径的尾部, 是措施3 砍掉快路的唯一原因)。
    //   现在阵列地址只由 ID/EX 寄存器驱动 ⇒ 加法器整条移出该路径。
    //   投机失配 (EX 级 rs1 被转发) 时 dcache 的 (pre_a == d_addr) 门自动
    //   否决快路, 落回 2 拍慢路 ⇒ 行为最坏等于改动前。
    // ★★★ L1 修复: `pre0` 必须带 `id_ex_valid`。
    //   原来不带 ⇒ ID/EX slot0 是**气泡**(但 mem_read/mem_write 残留旧值) 时
    //   `d_addr_pre` 会选 slot0 的**陈旧地址**, 而 `d_valid_pre` 是由 slot1 驱动的
    //   ⇒ 预告地址张冠李戴 ⇒ 必失配。
    wire pre0 = id_ex_valid  && (id_ex_mem_read  | (SB_ON ? 1'b0 : id_ex_mem_write));
    wire pre1 = id_ex1_valid && (id_ex1_mem_read | (SB_ON ? 1'b0 : id_ex1_mem_write));
    // ★★★ SB 排水预告 (2026-10-11, 阶段A收益化):
    //   排水 store 没有 pre ⇒ dcache 快路 (fast_hit) 永不命中 ⇒ 每个排水都走
    //   2 拍慢路 (accept + hit_beat) 且等待拍冻结流水线。CoreMark 实测
    //   drain_wait=19,580 拍 (18,866 个 store × ~1.04 拍), 是 SB 回退 +9.1% 的主因
    //   (旁路下 store 直通吃快路只占 D 口 1 拍且与推进重叠)。
    //   修法: EX 级没有 load (⇒ 下一拍 D 口不会被 load 占用) 且 SB 非空时,
    //   用 pre 端口预告**队头地址** ⇒ 排水 accept 拍吃快路, 恢复 1 拍 store。
    //   同时 SB 模式下 store 不再发 pre (它已不呈现 D 口, 发了也浪费且会压制排水预告;
    //   旁路模式保留原行为)。
    //   安全性: pre 失配自动落慢路 (fast 的 pre_a==d_addr 门, 行为最坏=改动前);
    //   load 的 pre 永远优先 (ex_has_load 时排水预告让位); p_dat 每拍重读阵列,
    //   dcache 的 RAW 前递 (预告拍合并写) 对排水同样生效; sb_addr 在非空时必已写入。
    wire ex_has_load = (id_ex_valid && id_ex_mem_read) || (id_ex1_valid && id_ex1_mem_read);
    // ★ v3: 预告地址 = "**下一拍将呈现 D 口的排水项**"预测 (2026-10-11)。
    //   v1/v2 只在 ~sb_empty 时预告队头 —— 但排水 accept 恰在压入后一拍, 而**压入拍
    //   sb_empty 仍为 1** (非阻塞更新) ⇒ "空→压入"后的第一笔排水永远无预告 (慢路);
    //   连发流则队头本拍弹掉、预告地址过期 ⇒ 隔一慢一。实测 (v1) 仍剩 14,089 拍
    //   无预告 + 9,467 拍预告不符。
    //   预测规则:
    //     · SB 非空且本拍队头不弹  ⇒ 现队头;
    //     · 本拍队头弹 (drain_pop_now) ⇒ 次项 (cnt>1) / 本拍压入项 (cnt==1 且压入);
    //     · SB 空 ⇒ 本拍压入项 (push_now) 才是下一拍排水; 否则无排水 (预告无害)。
    //   IO 区地址 ([31]=1) 不进 dcache (soc 侧 ram_sel 门控), 预告它只会污染 pre_a
    //   ⇒ 不预告。
    wire drain_pop_now = drain_present & d_valid & d_rdy;   // = sb_pop
    wire push_now      = adv & cur_is_store & ~sb_full;
    wire [31:0] pre_next_head =
        ~sb_empty ? (drain_pop_now
                       ? ((sb_cnt > 3'd1)           ? {sb_addr[sb_rp + 2'd1], 2'b00}
                          : (push_now ? cur_st_addr : {sb_addr[sb_rp], 2'b00}))
                       : {sb_addr[sb_rp], 2'b00})
                  : (push_now ? cur_st_addr : {sb_addr[sb_rp], 2'b00});
    // ★ 2026-10-11: MEM 校正拍 id_ex 是错路指令, 其 load 预告撤回 (预告本身
    //   投机安全, 撤回只为保持 pre_a 语义干净 —— 与 !trap_fire 门控同理)。
    // ★ §5f-2 刀1: 撤除 !mem_br_mismatch 组合门 (与 mem1_ld_supp 同源同链);
    //   错路 load 的 pre = 无害预读 (见 load_present 处不变量)。
    //   ~ex_mem_branch_taken 是寄存器版重定向位 (不在此组合链上), 保留。
    wire load_pre_v  = adv && (pre0 || pre1) && ~ex_mem_branch_taken
                       && !trap_fire;
    wire drain_pre_v = SB_ON && ~load_pre_v && ~pre_next_head[31];
    // Tags for five register-sourced candidates are compared in parallel.
    // Preserve the existing data-RAM address and all validity/RAW handling.
    assign d_pretag_addresses={cur_st_addr,{sb_addr[sb_rp + 2'd1],2'b00},
        {sb_addr[sb_rp],2'b00},id_ex1_mem_addr_pre,id_ex_mem_addr_pre};
    wire [2:0] drain_pretag_choice=~sb_empty
        ? (drain_pop_now ? ((sb_cnt>3'd1)?3'd3:(push_now?3'd4:3'd2)) :3'd2)
        : (push_now?3'd4:3'd2);
    assign d_pretag_choice=load_pre_v?(pre0?3'd0:3'd1)
        :(drain_pre_v?drain_pretag_choice:3'd1);
`ifdef FACEREC_SIM
    always @(posedge clk) if(resetn && d_valid_pre)
        if(d_pretag_addresses[d_pretag_choice*32+:32] !== d_addr_pre)
            $fatal(1,"Parallel tag choice disagrees with original hint address");
`endif
    assign d_valid_pre = load_pre_v || drain_pre_v;
    assign d_addr_pre  = load_pre_v    ? (pre0 ? id_ex_mem_addr_pre : id_ex1_mem_addr_pre)
                       : (drain_pre_v ? pre_next_head
                                      : id_ex1_mem_addr_pre);

    // Link classification shares the EX/MEM boundary; pair payloads remain
    // exactly the original mux while avoiding its late load-type input.
    pair_value_reg #(.WIDTH(1)) u_pair_link0(.clk(clk),.resetn(resetn),.en(adv),
        .d(id_ex_mem_to_reg && !id_ex_mem_read),.q(pair_link0_q));
    pair_value_reg #(.WIDTH(1)) u_pair_link1(.clk(clk),.resetn(resetn),.en(adv),
        .d(id_ex1_mem_to_reg && !id_ex1_mem_read),.q(pair_link1_q));

    always @(posedge clk) begin
        if (!resetn) begin
            ex_mem_valid      <= 1'b0;
            ex_mem_is_mul     <= 1'b0;
            ex_mem_alu_result <= 32'h0;
            ex_mem_store_data <= 32'h0;
            ex_mem_rd         <=  5'h0;
            ex_mem_pc_plus4   <= 32'h0;
            ex_mem_mem_read   <= 1'b0;
            ex_mem_mem_write  <= 1'b0;
            ex_mem_reg_write  <= 1'b0;
            ex_mem_mem_to_reg <= 1'b0;
            ex_mem_funct3     <= 3'b0;
            ex_mem_byte_off   <= 2'b0;
            ex_mem_wstrb      <= 4'b0;
            ex_mem_branch_taken <= 1'b0;
            ex_mem_branch_target <= 32'h0;
            // ★ 2026-10-11: MEM 级解析携带位复位
            ex_mem_br_pend <= 1'b0;      ex_mem_br_is_jalr <= 1'b0;
            ex_mem_br_funct3 <= 3'b0;    ex_mem_br_op_a <= 32'h0;
            ex_mem_br_op_b <= 32'h0;     ex_mem_br_imm <= 32'h0;
            ex_mem_br_pc <= 32'h0;       ex_mem_br_pred_taken <= 1'b0;
            ex_mem_br_pred_tgt <= 32'h0; ex_mem_br_btype <= 2'b0;
        end else if (adv) begin
            // 分支/跳转在 EX/MEM 触发时, EX/MEM 装 bubble (不取 ID/EX 的被冲刷指令)
            // 否则分支+1 指令会流入 EX/MEM 执行 (flush 来不及拦)
            // ★ 2026-10-11: MEM 级分支校正拍 (mem_br_mismatch), id_ex 里是
            //   错路指令, 一并压泡 (其挂起标志/字段随 valid 无效而失去意义)。
            ex_mem_valid      <= id_ex_valid && ~ex_mem_branch_taken
                                 && !trap_fire && !mem_br_mismatch && !ex_redirect_q;
            // ⚠ M2 修复: 原式用裸 branch_taken(= "必冲刷" 的 S2c 假设)。BTB 下
            //   预测正确时 taken 但不重定向, 裸信号会把**正确路径**指令在下一拍
            //   误压成 bubble(main 的 sw 32(sp) 就这样丢了→后续 load 读 X→活锁)。
            //   只有真实重定向(branch_taken_eff)才需要压 EX/MEM。禁用态
            //   branch_taken_eff ≡ id_ex_valid&&branch_taken&&adv, 与原式逐拍等价。
            ex_mem_branch_taken  <= branch_taken_eff;
            ex_mem_branch_target <= branch_target;
            ex_mem_is_mul        <= id_ex_is_mul;   // ★ 真值在 MEM 拍的 mul_result
            ex_mem_alu_result <= (id_ex_mem_read || id_ex_mem_write) ? mem_addr_word :
                                   id_ex_csr_en ? csr_rd : alu_result;   // M3a: CSR 旧值走 WB
            ex_mem_store_data <= store_shift;        // 对齐后的 store 数据
            ex_mem_rd         <= id_ex_rd;
            ex_mem_pc_plus4   <= id_ex_pc + 32'd4;
            ex_mem_mem_read   <= id_ex_mem_read;
            ex_mem_mem_write  <= id_ex_mem_write;
            ex_mem_reg_write  <= id_ex_reg_write;
            ex_mem_mem_to_reg <= id_ex_mem_to_reg;
            ex_mem_funct3     <= id_ex_funct3;
            ex_mem_byte_off   <= mem_byte_off;       // 原始字节偏移
            ex_mem_wstrb      <= mem_wstrb_pre;
            // ★★★ 2026-10-11 (冲 5 §5): 挂起分支快照 —— 解析所需全量携带进 MEM。
            //   快照操作数: load 旁路位 (br_pend_*) 直接取 mem_load_ext —— 与
            //   转发码 3'b011 的取值同源同拍 (该 load 正在本拍呈现 D 口/SB 转发),
            //   其余位取现有 5:1 转发 mux; MEM 拍比较器输入全为寄存器。
            //   ⚠ 快照拍 = 分支 EX→MEM 推进拍 (adv), 此时 ex_mem(0/1) 仍持有
            //   该 load (推进发生在拍末), mem_load_ext 必为其数据。
            ex_mem_br_pend       <= ex_br_pend & ~mem_br_mismatch & ~ex_redirect_q;
            ex_mem_br_is_jalr    <= ex_br_pend & id_ex_jalr;
            ex_mem_br_funct3     <= id_ex_funct3;
            // ★ 2026-10-14: load 旁路值改用 mem_ld_lat (miss 提前交付锁存,
            //   见 mem_ld_lat_v 声明处) —— 修复 miss load 相邻分支解析错值。
            // Snapshot late MUL directly, separately from the load latch;
            // a leftover mem_ld_lat must never override a MUL operand.
            ex_mem_br_op_a       <= br_mul_a ? (m1_mul_a ? mul_result1 : mul_result) :
                                     br_load_a ? (mem_ld_lat_v ? mem_ld_lat : mem_load_ext) : rs1_nb;
            ex_mem_br_op_b       <= br_mul_b ? (m1_mul_b ? mul_result1 : mul_result) :
                                     br_load_b ? (mem_ld_lat_v ? mem_ld_lat : mem_load_ext) : rs2_nb;
            ex_mem_br_imm        <= id_ex_imm;
            ex_mem_br_pc         <= id_ex_pc;
            ex_mem_br_pred_taken <= id_ex_pred_taken;
            ex_mem_br_pred_tgt   <= id_ex_pred_tgt;
            // btype 现算 (is_ret/is_call 在 g_btb generate 块内, 不可直接引用)
            ex_mem_br_btype      <=
                { id_ex_jalr && ((id_ex_rs1 == 5'd1) || (id_ex_rs1 == 5'd5)) && (id_ex_rd == 5'd0),
                 (id_ex_jal | id_ex_jalr) && ((id_ex_rd == 5'd1) || (id_ex_rd == 5'd5)) };
        end
        // 停顿: 保持
    end

    //-----------------------------------------------------------------
    // ★★ D2b: EX/MEM **slot1** (★ §5d: P2 放行后也可为分支/跳转 —— 字段
    //   通用, 无需改动; 分支 reg_write=0 / jal·jalr 走 mem_to_reg 的 pc+4
    //   写回路径, 均与既有字段兼容)
    //   ⚠⚠ 必须同拍压掉: slot1 与 slot0 **同属一对、同拍退休**, 所以只要
    //     slot0 被中止 (重定向 / trap / mret), slot1 必须一起压 ——
    //     否则错路/被中止的指令仍会写回寄存器堆 (精确异常被破坏)。
    //-----------------------------------------------------------------
    always @(posedge clk) begin
        if (!resetn) begin
            ex_mem1_valid <= 1'b0; ex_mem1_alu_result <= 32'h0;
            ex_mem1_is_mul <= 1'b0;   // ★ §5c
            ex_mem1_rd <= 5'h0;    ex_mem1_reg_write <= 1'b0;
            ex_mem1_store_data <= 32'h0; ex_mem1_pc_plus4 <= 32'h0;
            ex_mem1_mem_read <= 1'b0; ex_mem1_mem_write <= 1'b0;
            ex_mem1_mem_to_reg <= 1'b0; ex_mem1_funct3 <= 3'h0;
            ex_mem1_byte_off <= 2'h0; ex_mem1_wstrb <= 4'h0;
        end else if (adv) begin
            // ★ 2026-10-11: mem_br_mismatch 拍 id_ex/id_ex1 全是错路指令,
            //   slot1 也必须压泡 —— 否则错路 load/store 次拍会呈现 D 口
            //   (不可撤回的访存副作用) / 错路结果会写回寄存器堆。
            ex_mem1_valid      <= id_ex1_valid & ~branch_taken_eff
                                  & ~trap_fire & ~mret_fire & ~mem_br_mismatch & ~ex_redirect_q;
            ex_mem1_alu_result <= (id_ex1_mem_read || id_ex1_mem_write)
                                  ? mem_addr_word1 : alu_result1;
            ex_mem1_is_mul     <= id_ex1_is_mul;   // ★ §5c: 真值在 MEM 拍的 mul_result1
            ex_mem1_rd         <= id_ex1_rd;
            ex_mem1_reg_write  <= id_ex1_reg_write;
            ex_mem1_store_data <= store_shift1;
            ex_mem1_pc_plus4   <= id_ex1_pc + 32'd4;
            ex_mem1_mem_read   <= id_ex1_mem_read;
            ex_mem1_mem_write  <= id_ex1_mem_write;
            ex_mem1_mem_to_reg <= id_ex1_mem_to_reg;
            ex_mem1_funct3     <= id_ex1_funct3;
            ex_mem1_byte_off   <= mem_byte_off1;
            ex_mem1_wstrb      <= mem_wstrb_pre1;
        end
        // 停顿: 保持
    end

    //-----------------------------------------------------------------
    // MEM/WB 流水线寄存器
    //-----------------------------------------------------------------
    // Select the final WB value before the existing MEM/WB boundary.
    // Legacy component registers below remain as an independent simulation
    // oracle; their unused data copies are removable by synthesis.
    wire [31:0] wb_payload0_d = ex_mem_mem_to_reg
        ? (ex_mem_mem_read ? mem_load_ext : ex_mem_pc_plus4)
        : (ex_mem_is_mul ? mul_result : ex_mem_alu_result);
    wire [31:0] wb_payload1_d = ex_mem1_mem_to_reg
        ? (ex_mem1_mem_read ? mem_load_ext : ex_mem1_pc_plus4)
        : (ex_mem1_is_mul ? mul_result1 : ex_mem1_alu_result);
    pair_value_reg #(.WIDTH(32)) u_wb_payload0(.clk(clk),.resetn(resetn),.en(adv),
        .d(wb_payload0_d),.q(wb_payload0_q));
    pair_value_reg #(.WIDTH(32)) u_wb_payload1(.clk(clk),.resetn(resetn),.en(adv),
        .d(wb_payload1_d),.q(wb_payload1_q));
    reg        mem_wb_mem_read;      // 区分 load 与 jal (写回源)
    always @(posedge clk) begin
        if (!resetn) begin
            mem_wb_valid      <= 1'b0;
            mem_wb_alu_result <= 32'h0;
            mem_wb_mem_rdata  <= 32'h0;
            mem_wb_load_value <= 32'h0;
            mem_wb_pc_plus4   <= 32'h0;
            mem_wb_rd         <=  5'h0;
            mem_wb_reg_write  <= 1'b0;
            mem_wb_mem_to_reg <= 1'b0;
            mem_wb_mem_read   <= 1'b0;
            mem_wb_funct3     <= 3'b0;
            mem_wb_byte_off   <= 2'b0;
        end else if (adv) begin
            mem_wb_valid      <= ex_mem_valid;
            mem_wb_alu_result <= ex_mem_is_mul ? mul_result : ex_mem_alu_result;  // ★ MUL 晚结果
            mem_wb_mem_rdata  <= d_rdata;
            mem_wb_load_value <= mem_load_ext;   // ★ 措施2: MEM 级已扩展好的 load 值
            mem_wb_pc_plus4   <= ex_mem_pc_plus4;
            mem_wb_rd         <= ex_mem_rd;
            mem_wb_reg_write  <= ex_mem_reg_write;
            mem_wb_mem_to_reg <= ex_mem_mem_to_reg;
            mem_wb_mem_read   <= ex_mem_mem_read;
            mem_wb_funct3     <= ex_mem_funct3;
            mem_wb_byte_off   <= ex_mem_byte_off;
        end
        // 停顿: 保持
    end

    //-----------------------------------------------------------------
    // ★★ D2b: MEM/WB **slot1** (纯 ALU 直通)
    //-----------------------------------------------------------------
    always @(posedge clk) begin
        if (!resetn) begin
            mem_wb1_valid <= 1'b0; mem_wb1_alu_result <= 32'h0;
            mem_wb1_rd <= 5'h0;    mem_wb1_reg_write <= 1'b0;
            mem_wb1_load_value <= 32'h0; mem_wb1_pc_plus4 <= 32'h0;
            mem_wb1_mem_read <= 1'b0;    mem_wb1_mem_to_reg <= 1'b0;
        end else if (adv) begin
            // ★ 2026-10-11: MEM 校正拍 (mem_br_mismatch), ex_mem1 里的 slot1
            //   是挂起分支的错路配对指令 (纯 ALU, P9 保证) ⇒ 写回拦截。
            //   捕获发生在失配拍末 (adv=1), 组合门可精确撤回。
            mem_wb1_valid      <= ex_mem1_valid & ~mem_br_mismatch;
            mem_wb1_alu_result <= ex_mem1_is_mul ? mul_result1 : ex_mem1_alu_result;  // ★ §5c slot1 MUL 晚结果
            mem_wb1_rd         <= ex_mem1_rd;
            mem_wb1_reg_write  <= ex_mem1_reg_write;
            mem_wb1_load_value <= mem_load_ext;
            mem_wb1_pc_plus4   <= ex_mem1_pc_plus4;
            mem_wb1_mem_read   <= ex_mem1_mem_read;
            mem_wb1_mem_to_reg <= ex_mem1_mem_to_reg;
        end
        // 停顿: 保持
    end

    //-----------------------------------------------------------------
    // 写回 (WB 级): 组合选数, 送给 regfile 写端口
    //   mem_to_reg=1: mem_read=1 -> 存储器读数据 (窄 load 符号/零扩展)
    //                 mem_read=0 -> pc_plus4 (jal/jalr)
    //   mem_to_reg=0: ALU 结果
    //   valid = 0 (bubble) 时禁止写回
    //-----------------------------------------------------------------
    // load 符号/零扩展: 从整字读回按字节偏移取出 lb/lh/lbu/lhu 并扩展
    //   读回整字 mem_wb_mem_rdata, 目标字节在 [byte_off*8]
    //   lb/lbu: 取 8bit 符号/零扩展
    //   lh/lhu: 取 16bit 符号/零扩展
    //   lw:     整字
    //-----------------------------------------------------------------
    wire        wb_reg_write;
    wire [31:0] wb_wdata;
    // ★★ 提速措施2 + 阶段1: MEM 级 load 移位/扩展 (mem_load_shifted /
    //   mem_load_byte / mem_load_half / mem_load_ext) 已**上移到 EX 转发源
    //   之前声明** (见本文件 "阶段1: MEM 级 load 值" 段)。
    //   这样同时满足两件事:
    //     · 措施2 的收益不变 —— 它仍在寄存器**之前**组合算好, 再寄存进
    //       mem_wb_load_value, 转发路径上不含移位器/扩展 mux;
    //     · 阶段1 的新收益 —— EX/MEM 转发在 load 时可直接取 mem_load_ext,
    //       从而取消 load-use 停顿 (MEM→EX load 旁路)。

    // ---- 以下为原 WB 级扩展（保留作对照/仿真参考，已不在关键路径上）----
    wire [31:0] load_word_shifted;     // 读回字右移到目标字节到 bit0
    wire [ 7:0] load_byte;
    wire [15:0] load_half;
    reg  [31:0] mem_load_value;        // 扩展后的 load 数据

    assign load_word_shifted = mem_wb_mem_rdata >> {mem_wb_byte_off, 3'b000};
    assign load_byte  = load_word_shifted[7:0];
    assign load_half  = load_word_shifted[15:0];

    always @(*) begin
        case (mem_wb_funct3)
            `F3_LB:  mem_load_value = {{24{load_byte[7]}}, load_byte};
            `F3_LBU: mem_load_value = {24'b0, load_byte};
            `F3_LH:  mem_load_value = {{16{load_half[15]}}, load_half};
            `F3_LHU: mem_load_value = {16'b0, load_half};
            default: mem_load_value = mem_wb_mem_rdata;   // lw
        endcase
    end

    assign wb_reg_write = mem_wb_valid & mem_wb_reg_write;
    // ★ 提速措施2: 改用 MEM 级已扩展好并寄存的 mem_wb_load_value
    assign wb_wdata = wb_payload0_q;

    //-----------------------------------------------------------------
    // 未用输入抑制 (骨架阶段占位)
    //-----------------------------------------------------------------
    wire _unused = ext_irq;

    // S8 测量面板第二组事件已在 pm_events2_q 处寄存器化 (见上方 assign)

endmodule

// Pairing needs three instruction classes, not the encoded execution ALU op.
// Match ctrl's exact case/default semantics, including invalid and X/Z fields.
module rv32_pair_classes /* synthesis syn_hier="hard" */ (
    input wire [31:0] instr,
    output reg compare_op,
    output reg multiply_op,
    output reg light_op
);
    always @(*) begin
        compare_op=1'b0;
        multiply_op=1'b0;
        // ctrl defaults alu_op to ALU_ADD, including invalid encodings.
        light_op=1'b1;
        case(instr[6:0])
            `OP_OP: begin
                case({instr[31:25],instr[14:12]})
                    {`F7_0000000,`F3_SLT},
                    {`F7_0000000,`F3_SLTU}: begin
                        compare_op=1'b1; light_op=1'b0;
                    end
                    {`F7_0100000,`F3_ADDSUB}: light_op=1'b0;
                    {`F7_MULDIV,`F3_ADDSUB},
                    {`F7_MULDIV,`F3_SLL},
                    {`F7_MULDIV,`F3_SLT},
                    {`F7_MULDIV,`F3_SLTU}: begin
                        multiply_op=1'b1; light_op=1'b0;
                    end
                    default: begin end
                endcase
            end
            `OP_OP_IMM: begin
                case(instr[14:12])
                    `F3_SLT,`F3_SLTU: begin
                        compare_op=1'b1; light_op=1'b0;
                    end
                    default: begin end
                endcase
            end
            `OP_BRANCH: begin
                case(instr[14:12])
                    `F3_BEQ,`F3_BNE,`F3_BLT,`F3_BGE,`F3_BLTU,`F3_BGEU:
                        light_op=1'b0;
                    default: begin end
                endcase
            end
            default: begin end
        endcase
    end
endmodule
