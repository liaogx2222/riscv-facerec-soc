//=====================================================================
// btb.v : 分支目标缓冲 + 方向预测 (M2, IF 级查询 / EX 级校验更新)
//
// 256 项索引表 (2026-10-11, 128→256 容量): {tag=pc[16:10], target[31:0], ctr[1:0]}
//   ctr==2'b00 → 空槽; 预测条件 = tag 命中 && ctr[1](strongly taken)
//   index = pc[9:2] (字对齐, 覆盖 1KB 窗口/项), tag 7 位覆盖 64KB
//
//   IF  lk_*  : 每拍按 pc 组合查表 → 预测 taken 时改 next_pc
//   IF  lk1_* : ★ P2 松绑 (2026-10-11 §5d): 第二查询口 —— 按 slot1 的
//               pc=pc+4 查询 (仅顺序双发包用; FF 阵列组合读, 加口廉价)。
//               与 lk 同拍更新不重叠的架构论证同 lk 口 (查询=IF 级 pc,
//               更新=EX 级 id_ex_pc, 同拍相等架构不可达)。
//   EX  vf_*  : 按 id_ex_pc 复核 (与 branch_target 相等才可免冲刷)
//   UPD upd_* : EX 判定拍 (adv 门控) 一次性更新:
//         taken:  命中→ctr 饱和+1 & 写 target; 未命中→分配 ctr=2'b10
//         !taken: 命中→ctr-1 (1→0 即失效逐出); 未命中不动
//   查询/复核口对同拍更新做"有效槽位"前递 (组合算出更新后的槽内容再比对).
//   ★ 2026-10-06 前递已删除（架构死逻辑，证明见下方查询口注释）——
//   代价: 同拍更新对同拍查询不可见（架构上两者地址恒不同，无影响）。
//=====================================================================
`timescale 1ns/1ps

module btb (
    input  wire        clk,
    input  wire        resetn,

    input  wire [31:0] lk_addr,
    output wire        lk_hit,
    output wire [31:0] lk_tgt,
    output wire [ 1:0] lk_ctr,
    // ★ P2 松绑 (2026-10-11 §5d): 第二查询口 (slot1 预测, 按 pc+4 查询)
    input  wire [31:0] lk1_addr,
    output wire        lk1_hit,
    output wire [31:0] lk1_tgt,
    output wire [ 1:0] lk1_ctr,
    output wire [ 1:0] lk1_btype,

    input  wire [31:0] vf_addr,
    output wire        vf_hit,
    output wire [31:0] vf_tgt,
    output wire [ 1:0] vf_ctr,

    input  wire        upd_en,
    input  wire [31:0] upd_pc,
    input  wire        upd_taken,
    input  wire [31:0] upd_tgt,
    // ★ S1 预测器强化 (2026-10-11): 调用/返回类型位 —— RAS 用的标记
    //   2'b01=call (jal/jalr 且 rd=x1/x5), 2'b10=ret (jalr 且 rs1=x1/x5, rd=x0),
    //   2'b00=其它。ret 的目标是寄存器值, BTB 里存的"上次目标"必错 ⇒
    //   命中 ret 项时预测目标改从 RAS 栈顶取 (cpu_rv32i 侧接)。
    input  wire [ 1:0] upd_btype,
    output wire [ 1:0] lk_btype,
    output wire [ 1:0] vf_btype,
    // ★ §5f-3 R1c-b (2026-10-15): pf 的 **slot1 前瞻口** —— 链随预测走需要看到
    //   顺序包 slot1 (=iss_addr+4) 的预测, 否则 slot1 预测分支仍要 hint 清链
    //   (实测 38.9K 次/轮 = R1c 残余气泡大头)。查询域 = pf 链地址+4, 与 upd
    //   写域 (EX id_ex_pc) 同拍相等的论证同 vf 口 (小循环跨圈重访读到的只是
    //   上一圈更新值, 与 vf 同级陈旧, 预测器自愈)。
    input  wire [31:0] vf2_addr,
    output wire        vf2_hit,
    output wire [31:0] vf2_tgt,
    output wire [ 1:0] vf2_ctr,
    // ★ §5f-3 循环出口预测器 (2026-10-15): 四查询口的"预测不跳"覆盖信号 ——
    //   紧循环 bimodal 饱和后每次出口必误预测 (实测占误预测 79%), trip 计数
    //   学习圈数后在本圈跳满时覆盖为不跳。四口必须同时可见: 否则 CPU 正确
    //   不跳而 pf 照旧插入 T ⇒ 链失配 resync, 抵消收益。
    output wire        lk_loop_nt,
    output wire        lk1_loop_nt,
    output wire        vf_loop_nt,
    output wire        vf2_loop_nt,
    // ★ DBG-1e: pf 前瞻口 (查 pf next_issue / +4) —— 结果只进 pf 锁存器,
    //   不反馈 next_issue ⇒ next_issue 组合环断
    input  wire [31:0] vf3_addr,
    output wire        vf3_hit,
    output wire [31:0] vf3_tgt,
    output wire [ 1:0] vf3_ctr,
    input  wire [31:0] vf4_addr,
    output wire        vf4_hit,
    output wire [31:0] vf4_tgt,
    output wire [ 1:0] vf4_ctr
);
    reg [ 6:0] tag [0:255];
    reg [31:0] tgt [0:255] /* synthesis syn_ramstyle="select_ram" */;
    reg [ 1:0] ctr [0:255];
    reg [ 1:0] btype[0:255];

    // ---- 更新槽的"下一内容"(组合) ----
    // ★ 2026-10-08 阶段2: 32 项 → **128 项**（index 5→7 位, tag 10→8 位）。
    //   动机: flush(误预测校正) 9,672 拍 = 2.3%。原 32 项目录太小,
    //   CoreMark 的控制流指令远超 32 条 ⇒ 别名冲突造成额外误预测。
    //   128 项仍用 FF 阵列 (128×42 bit ≈ 5.4Kb, FF 余量极大),
    //   **tag 变窄 2 位 ⇒ 比较反而更快**, 是纯收益的尺寸改动。
    wire [7:0] ui = upd_pc[9:2];
    wire [6:0] ut = upd_pc[16:10];
    // Event data is consumed only when m_ev_q is asserted. Prepare it
    // independently of the late update enable; valid and write timing stay.
    wire         uh  = (tag[ui] == ut) && (ctr[ui] != 2'b00);
    wire [ 1:0] nc   = uh ? (upd_taken ? (ctr[ui]==2'b11 ? 2'b11 : ctr[ui]+2'b01)
                                         : (ctr[ui]==2'b01 ? 2'b00 : ctr[ui]-2'b01))
                       : (upd_taken ? 2'b01 : ctr[ui]);               // 新项先弱不跳，连续 taken 后再预测跳转
    wire [ 7:0] nt   = (!uh && upd_taken) ? ut : tag[ui];
    wire [31:0] ng   = (uh && upd_taken) ? upd_tgt
                       : (!uh && upd_taken) ? upd_tgt : tgt[ui];

    // ---- 通用查询口 (idx/tag 各一份) ----
    // ★ 2026-10-06: 删除同拍更新前递 (lfwd: 查询口读到本拍更新槽内容)。
    //   架构证明: lk_addr=IF 级 pc, upd_pc=EX 级 id_ex_pc, 同拍相等要求
    //   pc 回落至 EX 退休指令地址 —— 而一切 pc 回落 (分支/陷阱/预测) 都
    //   伴随重定向冲刷 EX、停顿则 pc 与流水线同冻结 (相对距离不变) ⇒
    //   同拍 lk==upd 架构不可达 ⇒ 前递是死逻辑, 删除后行为逐位等价。
    //   动机: S6b 重定向链 ALU 判决→ng→前递→lk_hit→next_pc→
    //   f_redirect→pf_next_issue 共 21 级, CIFAR 工程 108M 综合违例
    //   −0.038ns (1 端点); 删除后 ng 只进寄存器写, 链被切断。
    // IF 查询
    wire [7:0] li = lk_addr[9:2];
    wire [6:0] lt = lk_addr[16:10];
    assign lk_hit   = (ctr[li] != 2'b00) && (tag[li] == lt);
    assign lk_tgt   = tgt[li];
    assign lk_ctr   = ctr[li];
    assign lk_btype = btype[li];

    // ★ P2 松绑: IF 第二查询口 (slot1 的 pc = pc+4; FF 阵列组合读, 加口廉价)
    wire [7:0] l1i = lk1_addr[9:2];
    wire [6:0] l1t = lk1_addr[16:10];
    assign lk1_hit   = (ctr[l1i] != 2'b00) && (tag[l1i] == l1t);
    assign lk1_tgt   = tgt[l1i];
    assign lk1_ctr   = ctr[l1i];
    assign lk1_btype = btype[l1i];

    // EX 复核（本 CPU 未使用，保留接口）
    wire [7:0] vi = vf_addr[9:2];
    wire [6:0] vt = vf_addr[16:10];
    assign vf_hit   = (ctr[vi] != 2'b00) && (tag[vi] == vt);
    assign vf_tgt   = tgt[vi];
    assign vf_ctr   = ctr[vi];
    assign vf_btype = btype[vi];

    // ★ R1c-b: pf slot1 前瞻 (vf 口同款组合读, FF 阵列加口廉价)
    wire [7:0] v2i = vf2_addr[9:2];
    wire [6:0] v2t = vf2_addr[16:10];
    assign vf2_hit = (ctr[v2i] != 2'b00) && (tag[v2i] == v2t);
    assign vf2_tgt = tgt[v2i];
    assign vf2_ctr = ctr[v2i];
    // ★ DBG-1e: 前瞻口 (同款组合读, FF 阵列加口廉价)
    wire [7:0] v3i = vf3_addr[9:2];
    wire [6:0] v3t = vf3_addr[16:10];
    assign vf3_hit = (ctr[v3i] != 2'b00) && (tag[v3i] == v3t);
    assign vf3_tgt = tgt[v3i];
    assign vf3_ctr = ctr[v3i];
    wire [7:0] v4i = vf4_addr[9:2];
    wire [6:0] v4t = vf4_addr[16:10];
    assign vf4_hit = (ctr[v4i] != 2'b00) && (tag[v4i] == v4t);
    assign vf4_tgt = tgt[v4i];
    assign vf4_ctr = ctr[v4i];

    //=================================================================
    // ★★★ §5f-3 循环出口预测器 (L-BILD 简化版, 2026-10-15)
    //   动机: bimodal 饱和后循环出口每圈必误预测 1 次 —— 归因实测占全部
    //   误预测 79% (loop_exit 46% + 短圈 predt_nt 33%)。trip 计数学习
    //   "每圈跳数", 本圈跳满即覆盖为不跳。
    //   表: 32 项 {v, tag=pc[13:7], trip[8], cnt[8], conf[2]}, 索引 pc[6:2]。
    //   覆盖条件: hit && conf[1](高置信) && trip!=0(已学过) && cnt>=trip。
    //   更新 (复用 upd 口, 每次分支解析一拍):
    //     taken  : 命中 cnt++ (饱和); 若 cnt>=trip 且已高置信 = trip 过期
    //              (变圈数循环) ⇒ conf 清零防连环误预测; 未命中且为**后向
    //              条件分支** ⇒ 分配 {trip=0,cnt=1}。
    //     !taken : cnt=0; 正确退出 (conf[1] && cnt==trip) ⇒ trip/conf 保持;
    //              否则重学 trip<=cnt (cnt>=2 才置高置信, 短计数不学)。
    //   约束: 仅条件分支 (btype=00) 参与, call/ret 排除; jal 永不退出由
    //   trip!=0 防护。紧循环 pf 视角滞后 1 拍只会"更 taken", 出口拍退化为
    //   resync ≈ 原误预测代价 (平手不亏); 松循环 (体≥10 条) 零成本净赚。
    //   复位: **不复位** (同 tag/tgt 的 RSCO 论证), initial 给仿真初值,
    //   片上 Pango FF 上电为 0 与之一致。
    //=================================================================
    reg        lp_v   [0:31];
    reg [6:0]  lp_tag [0:31];
    reg [7:0]  lp_trip[0:31];
    reg [7:0]  lp_cnt [0:31];
    reg [1:0]  lp_conf[0:31];

    // ---- 四查询口 (覆盖判定全 === 收口, X 归 0 = 不覆盖, 退回 bimodal) ----
    wire [4:0] lp_li  = lk_addr[6:2];
    wire       lk_lph = (lp_v[lp_li] === 1'b1) && (lp_tag[lp_li] === lk_addr[13:7]);
    assign lk_loop_nt = (lk_lph === 1'b1) && (lp_conf[lp_li] === 2'b11) &&
                        (lp_trip[lp_li] !== 8'h0) &&
                        ((lp_cnt[lp_li] >= lp_trip[lp_li]) === 1'b1);

    wire [4:0] lp_l1i = lk1_addr[6:2];
    wire       lk1_lph = (lp_v[lp_l1i] === 1'b1) && (lp_tag[lp_l1i] === lk1_addr[13:7]);
    assign lk1_loop_nt = (lk1_lph === 1'b1) && (lp_conf[lp_l1i] === 2'b11) &&
                         (lp_trip[lp_l1i] !== 8'h0) &&
                         ((lp_cnt[lp_l1i] >= lp_trip[lp_l1i]) === 1'b1);

    wire [4:0] lp_vi  = vf_addr[6:2];
    wire       vf_lph = (lp_v[lp_vi] === 1'b1) && (lp_tag[lp_vi] === vf_addr[13:7]);
    assign vf_loop_nt = (vf_lph === 1'b1) && (lp_conf[lp_vi] === 2'b11) &&
                        (lp_trip[lp_vi] !== 8'h0) &&
                        ((lp_cnt[lp_vi] >= lp_trip[lp_vi]) === 1'b1);

    wire [4:0] lp_v2i = vf2_addr[6:2];
    wire       v2_lph = (lp_v[lp_v2i] === 1'b1) && (lp_tag[lp_v2i] === vf2_addr[13:7]);
    assign vf2_loop_nt = (v2_lph === 1'b1) && (lp_conf[lp_v2i] === 2'b11) &&
                         (lp_trip[lp_v2i] !== 8'h0) &&
                         ((lp_cnt[lp_v2i] >= lp_trip[lp_v2i]) === 1'b1);

    // ---- 更新下一状态 (组合) ----
    wire [4:0] lp_ui  = upd_pc[6:2];
    wire       lp_uh  = (lp_v[lp_ui] === 1'b1) && (lp_tag[lp_ui] === upd_pc[13:7]);
    wire       lp_cc  = (upd_btype === 2'b00);                    // 条件分支才参与
    wire       lp_bwd = (upd_tgt < upd_pc);                       // 后向 = 循环特征
    wire       lp_stale = lp_uh && (lp_conf[lp_ui] === 2'b11) &&
                          (lp_trip[lp_ui] !== 8'h0) &&
                          ((lp_cnt[lp_ui] >= lp_trip[lp_ui]) === 1'b1);
    wire [7:0] lp_nc  = lp_uh ? ((lp_cnt[lp_ui] == 8'hFF) ? 8'hFF
                                                  : lp_cnt[lp_ui] + 8'h01) : 8'h01;
    wire       lp_corr  = lp_uh && (lp_conf[lp_ui] === 2'b11) &&
                          (lp_trip[lp_ui] !== 8'h0) &&
                          ((lp_cnt[lp_ui] === lp_trip[lp_ui]) === 1'b1); // 正确退出
    wire [7:0] lp_ntrip = lp_corr ? lp_trip[lp_ui]
                          : (((lp_cnt[lp_ui] >= 8'd2) === 1'b1) ? lp_cnt[lp_ui]
                                                                : lp_trip[lp_ui]);
    wire [1:0] lp_nconf = lp_stale ? 2'b00
                          : (((lp_cnt[lp_ui] >= 8'd2) === 1'b1) ? 2'b11
                                                                : lp_conf[lp_ui]);

    // ★★ §5f-2 刀D (2026-10-16): 主表更新**打一拍** —— 刀C 只切了 lp 表
    //   写链, 实测新榜首 = "EX 解析 → upd 仲裁 → nc 饱和计算(阵列读+tag比较)
    //   → 主表 ctr/tag/tgt 写口" (WNS −8.42 全家桶)。训练事件锁存一拍次拍写
    //   ⇒ 写口 D 链只剩 寄存器→FF, 超短。预测器训练晚 1 拍零功能影响
    //   (lk/vf/vf2 读口不受写拍影响; 连续更新同项后拍覆盖, 与原同拍非阻塞
    //   语义一致)。
    reg        m_ev_q;
    reg        m_ev_taken_q, m_ev_hit_q;
    reg [7:0]  m_ev_idx_q;
    reg [1:0]  m_ev_nc_q, m_ev_btype_q;
    reg [7:0]  m_ev_nt_q;
    reg [31:0] m_ev_ng_q;
    always @(posedge clk) begin
        if (!resetn) begin
            m_ev_q <= 1'b0; m_ev_idx_q <= 8'h0; m_ev_nc_q <= 2'b00;
            m_ev_nt_q <= 8'h0; m_ev_ng_q <= 32'h0; m_ev_taken_q <= 1'b0;
            m_ev_hit_q <= 1'b0; m_ev_btype_q <= 2'b00;
        end else begin
            m_ev_q       <= upd_en;
            m_ev_taken_q <= upd_taken;
            m_ev_hit_q   <= uh;
            m_ev_idx_q   <= ui;
            m_ev_nc_q    <= nc;
            m_ev_nt_q    <= nt;
            m_ev_ng_q    <= ng;
            m_ev_btype_q <= upd_btype;
        end
    end

    // ---- 更新提交 ----
    // ★★ §5f-2 刀C (2026-10-16): 循环表训练路径**打一拍** —— 实测综合榜首
    //   (−8.5) 是 "EX 分支解析 → BTB 更新仲裁 → lp 训练组合 (cnt/trip/conf
    //   状态机+比较链) → lp 阵列写口" 的长链。训练事件全量锁存一拍, 次拍写
    //   ⇒ 写口 D 链只剩 寄存器→ram, 超短。lp 表是预测器训练统计, 晚 1 拍
    //   **零功能影响**; 连续两拍更新同一项时, 后拍写覆盖前拍 (与原同拍
    //   非阻塞读旧值语义一致, 训练统计本就允许丢计数)。
    //   ⚠ 主表 (tag/tgt/ctr/btype) 更新不打拍 —— 不在关键路径, 最小改动。
    reg        lp_ev_q;
    reg        lp_ev_taken_q, lp_ev_uh_q, lp_ev_stale_q;
    reg        lp_ev_cc_q, lp_ev_bwd_q, lp_ev_corr_q;
    reg [4:0]  lp_ev_idx_q;
    reg [6:0]  lp_ev_tag_q;
    reg [7:0]  lp_ev_nc_q, lp_ev_ntrip_q;
    reg [1:0]  lp_ev_nconf_q;
    always @(posedge clk) begin
        if (!resetn) begin
            lp_ev_q <= 1'b0;
            lp_ev_idx_q <= 5'h0; lp_ev_tag_q <= 7'h0;
            lp_ev_nc_q <= 8'h0; lp_ev_ntrip_q <= 8'h0; lp_ev_nconf_q <= 2'b00;
            lp_ev_taken_q <= 1'b0; lp_ev_uh_q <= 1'b0; lp_ev_stale_q <= 1'b0;
            lp_ev_cc_q <= 1'b0; lp_ev_bwd_q <= 1'b0; lp_ev_corr_q <= 1'b0;
        end else begin
            lp_ev_q        <= upd_en;
            lp_ev_taken_q  <= upd_taken;
            lp_ev_idx_q    <= lp_ui;
            lp_ev_tag_q    <= upd_pc[13:7];
            lp_ev_uh_q     <= lp_uh;
            lp_ev_stale_q  <= lp_stale;
            lp_ev_cc_q     <= lp_cc;
            lp_ev_bwd_q    <= lp_bwd;
            lp_ev_corr_q   <= lp_corr;
            lp_ev_nc_q     <= lp_nc;
            lp_ev_ntrip_q  <= lp_ntrip;
            lp_ev_nconf_q  <= lp_nconf;
        end
    end
    // ★★ 2026-10-08 修时序: **tag/tgt 阵列不复位, 只复位 ctr**。
    //   为什么可以 (构造性论证):
    //     · 查询口 `lk_hit = (ctr[li] != 2'b00) && (tag[li] == lt)`
    //       —— `ctr==0` 时 tag/tgt **根本不被读**; `lk_tgt`/`vf_tgt` 同理只在
    //       命中 (即 ctr!=0) 时才被上层使用。
    //     · 分配槽位时 ctr/tag/tgt 是**同一拍一起写**的 (见 nc/nt/ng 的
    //       `upd_en && !uh && upd_taken` 分支) ⇒ 不存在"ctr 非 0 而 tag 是垃圾"
    //       的窗口; `uh` 又要求 `ctr!=0`, 所以首次更新的 `tag` 值不影响判定。
    //   收益: 复位扇出 **5376 FF → 256 FF** (占全设计 8440 FF 的 64% → 3%)。
    //   实测这条复位串行链 (RSCO, 0.079 ns/跳) 正是当前 WNS +0.306 ns 的关键路径
    //   (`u_fpga/por_cnt` → `u_btb/tgt[*][*]/RS`) —— **加任何新逻辑前必须先削它**,
    //   否则 RVC 多出的 FF 会把复位扇出撑大、这条路径直接违例。
    //   `initial` 只为仿真给初值 (片上 Pango FF 上电即为 0, 与之一致; 项目里
    //   simple_ram.v 等已在用 initial)。
    integer b_i, b_r;
    initial begin
        for (b_i = 0; b_i < 256; b_i = b_i + 1) begin
            tag[b_i] = 8'h0; tgt[b_i] = 32'h0; ctr[b_i] = 2'b00; btype[b_i] = 2'b00;
        end
        // ★ 循环表仿真初值 (片上上电为 0, 不复位 — 同 tag/tgt 的 RSCO 论证)
        for (b_i = 0; b_i < 32; b_i = b_i + 1) begin
            lp_v[b_i] = 1'b0; lp_tag[b_i] = 7'h0; lp_trip[b_i] = 8'h0;
            lp_cnt[b_i] = 8'h0; lp_conf[b_i] = 2'b00;
        end
    end

    always @(posedge clk) begin
        if (!resetn) begin
            for (b_r = 0; b_r < 256; b_r = b_r + 1) ctr[b_r] <= 2'b00;
        end else if (m_ev_q) begin
            // ★ 刀D: 主表更新走锁存事件 (次拍写, 见上方锁存块注释)
            ctr[m_ev_idx_q] <= m_ev_nc_q;
            tag[m_ev_idx_q] <= m_ev_nt_q;
            tgt[m_ev_idx_q] <= m_ev_ng_q;
            // ★ btype 随 taken 更新写 (call/ret 类型恒定于同一 pc; !taken 保持)
            if (m_ev_taken_q) btype[m_ev_idx_q] <= m_ev_btype_q;
            // ★ 刀C: 循环表更新走锁存事件 (次拍写, 见上方锁存块注释)
            if (lp_ev_q) begin
                if (lp_ev_taken_q) begin
                    if (lp_ev_uh_q) begin
                        lp_cnt[lp_ev_idx_q] <= lp_ev_nc_q;
                        if (lp_ev_stale_q) lp_conf[lp_ev_idx_q] <= 2'b00; // trip 过期防连环误预测
                    end else if (lp_ev_cc_q && lp_ev_bwd_q) begin
                        lp_v[lp_ev_idx_q]    <= 1'b1;
                        lp_tag[lp_ev_idx_q]  <= lp_ev_tag_q;
                        lp_trip[lp_ev_idx_q] <= 8'h0;
                        lp_cnt[lp_ev_idx_q]  <= 8'h01;
                        lp_conf[lp_ev_idx_q] <= 2'b00;
                    end
                end else if (lp_ev_uh_q) begin
                    lp_cnt[lp_ev_idx_q] <= 8'h00;
                    if (!lp_ev_corr_q) begin                      // 正确退出保持 trip/conf
                        lp_trip[lp_ev_idx_q] <= lp_ev_ntrip_q;
                        lp_conf[lp_ev_idx_q] <= lp_ev_nconf_q;
                    end
                end
            end
        end
    end
endmodule
