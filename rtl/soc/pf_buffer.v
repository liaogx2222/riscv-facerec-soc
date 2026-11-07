//=====================================================================
// pf_buffer.v : 取指预取缓冲 (M0'/M1/S6b/D2a) —— 独立可测模块
//
// CPU 侧 (电平 resp + take 消费, 与 run29 CPU 的 adv/冻结语义适配):
//   cpu_req/cpu_addr : CPU 请求 (cpu_req 常 1, cpu_addr = pc)
//   resp             : 电平 —— 队头(或本拍到达旁路)地址 == cpu_addr 即为 1,
//                      未被消费的拍保持为 1 且数据稳定 (CPU 冻结无损)
//   cpu_take         : 本拍消费 resp (仅允许在 resp=1 拍; 冲刷拍也算消费)
//
// S6b 重定向直取 (req_hint/hint_addr, 退休拍单发):
//   CPU 在预测/校正拍预告"下拍 cpu_addr=hint_addr", 本拍即按 hint 起新链
//   (= 把 resync 提前一拍且发地址换成 hint) → 目标响应在 pc 已改向的下一拍
//   旁路命中, taken 取指空隙 1 拍 → 0 拍.
//   ⚠ §5f-1 R1a (2026-10-15): 上游 soc_top 已将 f_redirect/f_redirect_vld 成对
//   锁存一拍再驱动本模块的 req_hint/hint_addr (切重定向域组合环). 故实际间隙
//   回到 1 拍 (零间隙优化为时序让路); 本模块接口与三纪律不变.
//   ★ §5f-1 R1b-v2 (2026-10-15): issue 路径**弃用 icache 的组合可用性探测**
//   (ready2/ready3) —— 可用性改**地址规则预测** (a2_ok/t_ok, 无阵列读), 切断
//   "next_issue→BTB→iss_addr2→icache tag 异步读→ready2→issue_next" 组合环的
//   icache-tag 腿. 命中时地址规则精确 (A+4 与 A 同行 ⇒ A 命中则 A+4 必命中);
//   行末跨界/行填充/插入目标 miss 的少数失配由既有 short_push/resync 兜底
//   (#4b 机制). 口2/口3 miss 不触发 icache 填充 (仅口1 miss 填充) ⇒ 乐观插入
//   安全: T miss 只损失一次配对, 无停顿. ready2/ready3 端口保留但不再参与控制.
//   ★★ §5f-3 R1c (2026-10-15): **链随预测走** —— 插入包链推进改 T+4 (跟随
//   预测路径), 与 CPU 配对消费 (A,T) 后 next_pc=T+4 严丝合缝 ⇒ 正确预测的
//   分支零 hint 零气泡 (R1a 的 +1 气泡/分支 根源消除). 配套两项:
//   ① 插入包 2 字化 (A,T), 第3口/word3/push3 废弃, landing_slot 不再多留槽;
//   ② hint 语义改"对齐则无害" (hint_addr==下一交付地址 ⇒ 空操作), 否则会把
//   对齐的队列炸掉. 误预测/预测竞态仍走 hint 清链重启 + resync 兜底.
//   ⚠ 三条结构纪律 (S3 血泪 + 组合环铁律):
//     1) hint 绝不进 hit/resync/resp 的判定链 (addr_eff 仍=cpu_addr)——
//        gnt 是组合直出, 若 iss/exp_push 反哺 hit 判定会零延迟振荡;
//     2) 只在真发得出去 (do_issue) 的那拍才清链/改 exp_push, 否则退化为
//        现状 resync, 绝不少发一拍;
//     3) hint_fire 经 hint_taken 输出反馈, CPU 侧无需(单发事件自洽), 仅 tb 断言.
//
// 后端 (M1 起: tagged 变延迟, 单发在飞 —— 对接 icache 命中/miss 混合延迟):
//   ready            : 后端可接受请求 (miss 填充期间拉低)
//   gnt/gaddr        : 组合请求脉冲 (gnt=1 拍后端锁存 gaddr; 单发在飞)
//   resp_vld/resp_addr/resp_rdata : 响应, 延迟不定但**按序返回**;
//                      本模块用 resp_addr 与期望链地址 (exp_push) 过滤,
//                      冲刷后旧请求的迟到响应自动丢弃
//
// ★★★ D2a (2026-10-08): 后端**一拍可回 2 个字** ⇒ 填充率翻倍
//
//   动机（实测）：单字后端下 `填充率 0.98 字/拍 ≈ 消耗率 0.88 字/拍`
//   ⇒ 队列**接近饱和**、`avg_cnt` 长期只有 0.54 字 ⇒ 手上永远只有一个字
//   ⇒ `have_nxt`(第二个字可用) 只有 15.7% ⇒ 双发射无从谈起。
//   要让 `have_nxt` 上去，填充率必须**显著超过**消耗率（约 2×）。
//
//   做法：icache 加第二个寄存读口，一拍给出 {resp_rdata, resp_nxt_data}；
//   本模块一拍**压 2 条**，链步进 8 字节。
//
//   ⚠⚠ 两个必须守住的纪律（v1/v2 都在这上面翻过车）：
//
//   1) `next_issue` 必须**在发出拍推进**（`next_issue <= issue_next`），
//      不能改成"按 push 推进"。因为 `do_issue` 在 `landing_slot` 挡住时会
//      反复对**同一个** `next_issue` 求值；只有在发出拍推进，重复求值才是
//      幂等的。改成 push 驱动 ⇒ 发出→响应之间 `next_issue` 是旧值 ⇒
//      同一地址被重复请求 ⇒ 有效填充率折半 ⇒ 白干。
//
//   2) 步进必须**在发出拍可预测**。icache 回 2 字的条件是
//        `a[4:2] != 7`（行内末字取不到 a+4）且 `a < RAM_LIMIT`（非 RAM 区
//        直通 0，无第二字）
//      —— 这两个条件 pf_buffer **在发出拍就能算**，所以步进是可预测的：
//        预测 8 / 4 字节。命中时预测 100% 准确；**只有 icache 行填充**
//        （`st==S_FILL` 时 resp_nxt_vld 被压 0）会让实际步进比预测小 4 字节，
//        此时 `exp_push` 与 `next_issue` 错位 ⇒ 响应被丢弃 ⇒ 由既有的
//        `resync` 通路自动重建链。代价 ≈ 每次 miss 一次重同步（全跑 ~500 次，
//        ~1000 拍 = 0.25%），换来的是**不需要给 icache 加状态**。
//
//   溢出安全：`landing_slot` 保证 `cnt_after <= DEPTH-2`，即每个被发出的
//   请求在下一拍都有**至少 2 个落点**；push 只可能来自已发出的请求 ⇒
//   `cnt` 永不越过 DEPTH（DEPTH=4 时 cnt 在 2~3 之间振荡，正是我们要的）。
//
//   行为:
//   - DEPTH 深 {addr,data} FIFO, 存放 4 字节步进连续地址链 (投机预取)
//   - 队头/旁路命中 cpu_addr → resp 电平 (稳态 1 指令/拍)
//   - 失配 (重定向/跑飞/掏空到非对齐) → 整队冲刷, 从 cpu_addr 重建
//   - 发出条件: 后端 ready && 在飞位腾出 && 有落点 (cnt_after<=DEPTH-2)
//     (resync/hint 拍清链重建, 落点恒成立)
//=====================================================================
`timescale 1ns/1ps

module pf_buffer #(
    parameter DEPTH     = 4,                  // 仅支持 DEPTH=4 (AW 写死)
    parameter RAM_LIMIT = 32'h0002_0000       // 必须与 icache.RAM_LIMIT 一致
)(
    input  wire        clk,
    input  wire        resetn,

    input  wire        cpu_req,
    input  wire [31:0] cpu_addr,
    input  wire        req_hint,              // S6b: 本拍有效 (CPU 保证单发)
    input  wire [31:0] hint_addr,
    output wire        hint_taken,            // hint 被本拍消费 (已按 hint 起链)
    output wire        resp,
    output wire [31:0] resp_data,
    // ★★ D2a: 队头的**下一个字** + 是否可用 (取指包的第 2 条)
    //   链按序 +4/+8 到达 (由 exp_push 保证) ⇒ f_addr[rptr+1] == head_addr+4。
    //   cnt>=2 时两级都在队里; 旁路拍 (cnt==0) have_nxt=0 ⇒ 上层按需停顿。
    //   ⚠ 不要试图用 per-entry "包首位" 替换 `cnt>=2`：2 字推送必然让 wptr
    //     越过 rptr+1，故 `entry[rptr] 是包首` ⟹ `cnt>=2`，两者恒等价。
    output wire [31:0] resp_data_nxt,
    // ★★ #3: 第 2 条的**实际地址** (CPU 配对校验用; 顺序包=pc+4, 插入包=T)
    output wire [31:0] resp_nxt_addr,
    output wire        have_nxt,
    // ★★ #3: BTB 在 `iss_addr` 的预测 (来自 CPU 的 BTB `vf` 口)
    //   只用来决定 **icache 第 2 读口读哪个地址**:
    //     预测 taken ⇒ 读 T ("插入项"); 否则读 A+4 (链的下一格)
    //   ⚠ 它**不改变链的步进** —— 链永远是 A → A+4 → A+8 … (顺序)。
    input  wire        pred_taken,
    input  wire [31:0] pred_tgt,
    // ★ R1c-b: slot1 前瞻 (BTB vf2 口, 查询 iss_addr+4) —— 顺序包 slot1 是
    //   强 taken 分支时, CPU 配对消费 (A, A+4) 后 next_pc = T1, 链随预测走
    //   需要跟进到 T1 (否则 slot1 预测分支仍要 hint 清链, 实测 38.9K 次/轮)。
    input  wire        pred1_taken,
    input  wire [31:0] pred1_tgt,
    // ★ DBG-1e: BTB 前瞻查询结果 (vf3/vf4 口查上一拍 next_issue / +4) ——
    //   顺序流拍用锁存版预测做插入/步进决策 (环断); 突变拍 (hint/resync/
    //   short_push 后一拍) 用当拍 vf/vf2 (地址源是寄存器, 本就在环外)。
    input  wire        pnext_taken,
    input  wire [31:0] pnext_tgt,
    input  wire        pnext1_taken,
    input  wire [31:0] pnext1_tgt,
    // ★ DBG-1e: issue_next 输出 (供 CPU vf3/vf4 前瞻查询) —— 查询地址必须是
    //   **下一请求地址** (=issue_next): 拍 i 锁存 vf3(i)=预测(issue_next(i)),
    //   拍 i+1 发出请求 i+1 (=issue_next(i)) 时用 pnext_q —— 严格对齐。
    //   ⚠ 查询地址取自环输出没问题 (结果只进锁存器, 不反馈) ⇒ 环仍断。
    output wire [31:0] pf_next_issue_o,
    // ★★ #3: 第 2 读口地址 → icache (组合, 与 gaddr 同拍有效)
    output wire [31:0] req_addr2,
    // ★★ #4: 第 3 读口地址 —— **只给"插入项"用**, 不占链的带宽:
    //   pred_taken ⇒ 第 2 口读 T(插入项), 第 3 口读 A+4(链的下一格) ⇒ 3 字全到手
    //   否则第 2 口读 A+4(链), 第 3 口闲置 ⇒ 与基线逐位相同 (2 字/请求)
    output wire [31:0] req_addr3,
    input  wire [31:0] resp_3rd_data,
    input  wire        resp_3rd_vld,
    // ★★★ #4c: icache 的**组合可用性** (精确预测, 取代启发式)
    //   ★ §5f-1 R1b-v2 (2026-10-15): 本模块控制路径**不再使用**这两路组合探测
    //   (可用性改地址规则 a2_ok/t_ok, 见 issue 路径注释) —— 端口保留仅为
    //   tb 兼容; icache tag 异步读就此退出 issue→next_issue 组合环。
    input  wire        ready2,
    input  wire        ready3,
    input  wire        cpu_take,
    // ★★ D2b: 本拍**同时消费第 2 个字** (顺序双发射, 一拍退休两条)
    //   只有 cpu_take=1 且 have_nxt=1 时才允许置 1 ⇒ 弹出 0/1/2 格。
    input  wire        cpu_take_nxt,

    input  wire        ready,                 // 后端可接受 (高阻于 miss 期)
    output wire        gnt,                   // 组合请求: 有效拍后端锁存 gaddr
    output wire [31:0] gaddr,
    input  wire        resp_vld,
    input  wire [31:0] resp_addr,
    input  wire [31:0] resp_rdata,
    // ★★ D2a: 后端一拍给两个字 ⇒ 本模块一拍压 2 条
    input  wire [31:0] resp_nxt_data,
    input  wire        resp_nxt_vld,

    // ★★ 观测端口 (D1b/D2a/D2b 诊断用)
    //   ⚠ 必须是**端口**: PDS 不支持 soc_top 侧写 `u_pf.do_issue` 这种跨层引用
    //     (报 Verilog-4083: Parameter ... can not be found)。此前这几条是
    //     层次引用, iverilog 能过但 PDS 过不去。
    output wire [4:0]  pm_cnt_occ,     // 当前队长 cnt (DEPTH<=16)
    output wire [4:0]  pm_evt,          // {cnt>=2, 2字响应, pop, push_ok, do_issue}
    output wire        pm_resync,       // ★ 诊断: 链重同步拍
    output wire        pm_hint_fire     // ★ 诊断: hint 直取生效拍
);
    // $clog2(DEPTH) —— PDS 拒 $clog2 (SystemVerilog), 用条件表达式
    localparam AW = (DEPTH > 8) ? 4 : (DEPTH > 4) ? 3 : 2;  // 4->2, 8->3, 16->4

    reg [31:0]   f_addr [0:DEPTH-1];
    reg [31:0]   f_data [0:DEPTH-1];
    reg [AW-1:0] wptr, rptr;
    reg [AW:0]   cnt;                         // 0..DEPTH
    reg [31:0]   next_issue;                  // 下一个待发地址 (发出拍推进)
    assign pf_next_issue_o = issue_next;      // ★ DBG-1e: 前瞻查询地址 = 下一请求
    // ★ DBG-1e: 前瞻预测锁存 (vf3/vf4 查上一拍 next_issue 的结果)
    reg          pnext_taken_q, pnext1_taken_q;
    reg  [31:0]  pnext_tgt_q, pnext1_tgt_q;
    // ★ DBG-1e: 顺序流判定 —— 上一拍无 hint/resync/short_push 事件 ⇒ 本拍
    //   iss_addr = 上一拍 next_issue ⇒ pnext_q 对齐, 用锁存预测; 否则本拍
    //   iss_addr 来自寄存器突变源 (hint_addr/addr_eff/nxt_addr_q, 环外)
    //   ⇒ 用当拍 vf/vf2 (查 iss_addr, 环外路径)。
    reg          short_push_q;
    always @(posedge clk) begin
        if (!resetn) begin
            pnext_taken_q  <= 1'b0; pnext1_taken_q <= 1'b0;
            pnext_tgt_q    <= 32'h0; pnext1_tgt_q  <= 32'h0;
            short_push_q   <= 1'b0;
        end else begin
            pnext_taken_q  <= pnext_taken;
            pnext_tgt_q    <= pnext_tgt;
            pnext1_taken_q <= pnext1_taken;
            pnext1_tgt_q   <= pnext1_tgt;
            short_push_q   <= short_push;   // short_push 后一拍 iss_addr 仍是突变值
        end
    end
    // ★ 当拍判定: hint/resync 当拍 iss_addr 即突变 (f_redirect_q/addr_eff),
    //   必须当拍切到 vf; short_push 的突变在后一拍 (next_issue 拉回在沿上),
    //   故加上 short_push_q。
    //   环安全: req_hint_eff/hint_hit/resync/short_push 的扇入 = 队列头比较/
    //   cpu_req/响应有效 —— 无一路从 next_issue 来 ⇒ 本判定不进环。
    wire        use_now = (req_hint_eff === 1'b1) || (resync === 1'b1) ||
                          (short_push === 1'b1) || (short_push_q === 1'b1);
    reg [31:0]   exp_push;                    // 下一个期望到达的链地址
    reg          out_vld_q;                   // 在飞请求 (单发)
    reg  [31:0]  out_addr_q;                  // 在飞请求地址
    reg  [31:0]  nxt_addr_q;                  // ★ #3: 在飞请求的第 2 字地址
    reg          pt_q;                        // ★ #3: 在飞请求的 pred_taken (push 拍要用)
    reg          p2_q;                        // ★ #4: 在飞请求的 pred2 (push 拍要用)
    reg          p1_q;                        // ★ R1c-b: 在飞请求是 slot1 链随包
    reg  [31:0]  nxt1_addr_q;                 // ★ R1c: 在飞请求的链随地址 (=issue_next)

    //-----------------------------------------------------------------
    // 组合: 到达/命中/冲刷/发出
    //   ⚠ 命中/冲刷链上禁止引入 !resync (push_ok 必须不含 resync):
    //   resync→hit→have_head→push_now(含!resync)→resync 是组合环 (v2 曾犯).
    //   ⚠ S6b: hint 同理不得进 hit 判定 —— 只作用于 iss/exp 的寄存输入.
    //-----------------------------------------------------------------
    wire push_ok   = resp_vld && (resp_addr == exp_push);     // 到达且属于当前链
    wire [31:0] head_addr = (cnt != 0) ? f_addr[rptr] : resp_addr;
    wire have_head = (cnt != 0) || push_ok;                   // 旁路: 本拍到达即消费
    // ★ RVC 第2步: 链是 **4 字节步进**的字地址, 而 cpu_addr 可能 2 字节对齐
    //   ⇒ 命中比较只比 [31:2]。(rv32i 下 cpu_addr[1:0] 恒 0 ⇒ 逐位等价)
    wire [31:0] addr_eff = {cpu_addr[31:2], 2'b00};
    wire hit       = cpu_req && have_head && (head_addr == addr_eff);
    wire pop       = hit && cpu_take;
    // ★★ D2b: 双发射一拍消费 2 个字
    wire pop_nxt   = pop && cpu_take_nxt && have_nxt;
    wire [AW:0] pop_n = {1'b0, pop} + {1'b0, pop_nxt};
    // push_now 只用 push_ok/!resync (与原 v2 一致). ⚠ 勿加 !req_hint 门控:
    //   hint 拍若旁路命中并 pop, 弹1压0 → cnt_after 下溢 3'b111=7 (首轮踩中).
    //   hint 的组合影响全部收敛到 hint_fire=do_issue&&req_hint (do_issue 派生链
    //   不经 hit/resync/push_now/pop → 无组合环), chain_rst 拍整体清链覆盖计数.
    wire push_now  = push_ok && !resync;                      // 冲刷拍丢弃 (仅寄存路径)
    // ★ D2a: 一拍压 1 或 2 条
    // ★ R1c (链随预测走): 插入包 **2 字化** (A,T) —— 第3口/word3/push3 全部废弃:
    //   链下一格 = T+4 由下一请求正常取 (旧设计第 3 口预取的 A+4 是错路字)。
    wire [AW:0] push_inc = ((push_now === 1'b1) && (resp_nxt_vld === 1'b1)) ? 3'd2
                         : ((push_now === 1'b1) ? 3'd1 : 3'd0);
    // ★ #4b 修复 (保留): 预测 2 字实收 1 字 ⇒ next_issue 已越过缺失格 ⇒
    //   本拍**不许发新请求**, 把 next_issue 拉回缺失格补发。
    //   ★ R1c: 缺失格地址统一 = **nxt_addr_q** (本请求第 2 字地址):
    //     顺序包 = A+4 (与旧 resp_addr+4 逐位一致);
    //     插入包 = T —— T miss 时补发 T (CPU 下拍正要取 T; 链下一格 T+4
    //     尚未发出, next_issue 从 T+4 拉回 T, T 送达后后续自然补 T+4)。
    wire        short_push= (push_now === 1'b1) && (resp_nxt_vld === 1'b0) &&
                            ((pt_q === 1'b1) || (p2_q === 1'b1) || (p1_q === 1'b1));
    // ★ R1c: "链推进 2 字"的实际判据 = 第 2 字送达 (顺序包 A+4 / 插入包 T 同式)
    wire        actual_a8 = (resp_nxt_vld === 1'b1);

    // 空链但地址流仍与 pc 对齐 (消费掏空/等待落地), 或在飞请求即 pc 本身
    // (重建成链窗口 / miss 填充在飞) → 不冲刷, 就绪后续发 (每 pc 恰发一次)
    wire wait_ok   = (cnt == 0) && !push_ok &&
                     ((next_issue == addr_eff) || (out_vld_q && (out_addr_q == addr_eff)));
    wire resync    = cpu_req && !hit && !wait_ok;

    // ---- S6b: hint 拍 = "地址换成 next_pc 的提前 resync" (可发才吃) ----
    //   ★★ R1c (链随预测走): hint 语义改为**对齐则无害**。链跟随预测路径后,
    //   正确预测的分支队列天然对齐 (配对消费 (A,T) 后 head = T+4 = 配对拍
    //   next_pc, L1369; 单发消费 A 后 head = T = 预测 pc) —— 此时 hint 必须
    //   是**空操作**, 否则会把对齐的队列炸掉重取 (比 R1a 的 +1 气泡更糟)。
    //   判据: hint_addr == 下一交付地址 (队头 / 在飞响应 / next_issue)
    //   ⇒ 抑制; 不等 ⇒ 真重定向 (误预测/预测竞态), 按原语义清链重启。
    //   (=== 为情形相等, 恒 0/1, X 安全; 双 X 归为 hit=抑制, resync 兜底)
    wire [31:0] m_nxt = (cnt != 0)           ? f_addr[rptr]
                      : (out_vld_q === 1'b1) ? out_addr_q
                      :                        next_issue;
    wire hint_hit     = (m_nxt === hint_addr);
    wire req_hint_eff = (req_hint === 1'b1) && (hint_hit === 1'b0);
    //   landing_slot 在 resync|req_hint_eff 拍恒成立 (同拍清链, 新链从 hint_addr 起)
    wire [AW:0] cnt_after = cnt + push_inc - pop_n;
    wire out_free     = !out_vld_q || resp_vld;
    // ★ D2a: 发出请求时预留 2 个落点 (响应可能带 2 字)
    //   ★ R1c: 插入包 2 字化 ⇒ 不再多留一槽 (was: DEPTH-3)。
    // Reserve response slots without borrowing this edge's CPU pop.
    // Architectural pop/count updates stay exact; only near-full issue may wait.
    wire landing_slot = (resync || req_hint_eff) ? 1'b1
                      :                            ((cnt + push_inc) <= (DEPTH-2));
    reg do_issue;
    always @(*)
        do_issue = cpu_req && ready && out_free && landing_slot && !short_push;

    wire hint_fire = do_issue && req_hint_eff;        // 本拍真按 hint 发出去了 (观测)
    // ★★★ 2026-10-09 修复: **清链用 `req_hint_eff`, 不是 `hint_fire`**。
    //   后端单发在飞时 (`out_free=0`, 约一半的拍) hint 吃不掉 ⇒ 退化成 resync,
    //   而 resync 的地址是 `addr_eff`(**当前 pc**, 即分支自身) 而不是 `hint_addr`
    //   (**分支目标**) ⇒ 清错地方 ⇒ 下一拍还要再 resync。
    //   配合"`next_issue` 只在真发出那拍推进"的修复, 未发出时 `next_issue` 停在
    //   `hint_addr`, 下一拍自动补发 ⇒ **绝不少发一拍**。
    wire chain_rst = resync || req_hint_eff;          // 清链重定向 (hint 优先地址)
    assign hint_taken = hint_fire;

    //   ⚠ 三目全部用 `=== 1'b1` 收口: 上游 hint_fire/resync 可能是 X
    //     (见 cpu_rv32i 的 f_redirect_vld 注释), 一旦 X 进 iss_addr 就会被
    //     `next_issue` 永久锁死 ⇒ 取指链死锁。
    wire [31:0] iss_addr = (req_hint_eff === 1'b1) ? hint_addr
                         : (resync    === 1'b1) ? addr_eff
                         :                        next_issue;
    wire [31:0] hint4, hint8, restart4, restart8, stream4, stream8;
    pc_step #(.STEP(32'd4)) u_hint4(.a(hint_addr), .y(hint4));
    pc_step #(.STEP(32'd8)) u_hint8(.a(hint_addr), .y(hint8));
    pc_step #(.STEP(32'd4)) u_restart4(.a(addr_eff), .y(restart4));
    pc_step #(.STEP(32'd8)) u_restart8(.a(addr_eff), .y(restart8));
    pc_step #(.STEP(32'd4)) u_stream4(.a(next_issue), .y(stream4));
    pc_step #(.STEP(32'd8)) u_stream8(.a(next_issue), .y(stream8));
    wire [31:0] iss_addr4 = (req_hint_eff === 1'b1) ? hint4
                           : (resync === 1'b1) ? restart4 : stream4;
    wire [31:0] iss_addr8 = (req_hint_eff === 1'b1) ? hint8
                           : (resync === 1'b1) ? restart8 : stream8;

    // ★★ §5f-1 R1b-v2 (2026-10-15): 可用性预测改**地址规则** (X 安全, 无阵列读)
    //   —— 切断 issue 路径上的 icache tag 组合读 (ready2/ready3 探测)。
    //   命中时精确: A+4 与 A 同行 ⇒ A 命中则 A+4 必命中; 行末跨界/行填充/
    //   插入目标 miss 的少数失配由 short_push/resync 兜底 (#4b)。
    //   口2/口3 miss 不触发填充 ⇒ 乐观插入安全 (T miss 只损失一次配对)。
    //   ⇒ issue 路径只剩 next_issue→BTB(单阵列读)→加法; icache tag 读只余
    //     rd2_ok/rd3_ok 寄存边界 (响应侧), 重定向域组合环彻底切断。
    // Replace add-then-range-check with two constant comparisons. Preserve
    // 32-bit wraparound: fffffffc..ffffffff become 0..3 after +4.
    // Small parameter limits retain the original expression.
    function after4_in_ram;
        input [31:0] a;
        reg [31:0] a4;
        begin
            if (RAM_LIMIT >= 32'd4)
                after4_in_ram = ((a < (RAM_LIMIT - 32'd4)) ||
                                (a >= 32'hffff_fffc)) === 1'b1;
            else begin
                a4 = a + 32'd4;
                after4_in_ram = (a4 < RAM_LIMIT) === 1'b1;
            end
        end
    endfunction
    wire        a2_ok    = (req_hint_eff === 1'b1) ? after4_in_ram(hint_addr)
                           : (resync === 1'b1) ? after4_in_ram(addr_eff)
                           : after4_in_ram(next_issue);
    // ★ DBG-1e: 预测源混合 —— 顺序流拍用锁存前瞻预测 (环断), 突变拍用当拍
    //   vf/vf2 (地址源在环外)。锁存目标的合法性用锁存值现算 (无环)。
    wire        t_ok_now = after4_in_ram(pred_tgt);
    // On restart use sequential fetch, keeping live BTB queries out of
    // the next lookahead lookup. Normal streams keep registered prediction.
    wire        ins_sel_taken = !use_now && pnext_taken_q;
    wire [31:0] ins_sel_tgt   = pnext_tgt_q;
    wire [31:0] ins_sel_tgt4;
    pc_step #(.STEP(32'd4)) u_target4(.a(ins_sel_tgt), .y(ins_sel_tgt4));
    wire        ins_sel1_taken= !use_now && pnext1_taken_q;
    wire [31:0] ins_sel1_tgt  = pnext1_tgt_q;
    wire        t_ok_sel = after4_in_ram(ins_sel_tgt);
    // ★ R1b-v2: 插入包条件 = BTB 预测 taken 且目标+4 在 RAM 区
    //   (t_ok 现在同时保证链随预测走的下一格 T+4 合法)
    wire        ins_ok_i = (ins_sel_taken === 1'b1) && t_ok_sel;

    // ★★ §5f-1 R1b-v2: `pred2` 改**地址规则** a2_ok (was: icache 组合探测 ready2)。
    //   旧 #4c 精确探测把 icache tag 异步读串进了 issue→next_issue 组合环
    //   (150M 不可收敛); 地址规则命中时同样精确, 少数失配 short_push 兜底。
    wire        pred2      = a2_ok;
    // ★★ #3: 第 2 读口的地址 —— 预测 taken 时读 **T (插入项)**, 否则读 A+4
    // ★★ #3b: **自适应插入** —— 只在队列有余量时才把第 2 读口花在"插入项"上。
    //   动机 (实测 200k 拍): 固定插入时 `issue=54,604` 但 `push=44,145`
    //   ⇒ **10,458 次响应被丢弃 (19%)**, 因为插入项占掉了本该给"链的下一格"的
    //   第 2 读口 ⇒ 链填充率从 2 字/请求掉到 1 字/请求 ⇒ 取指饿死。
    //   ⇒ 队列浅 (cnt < 4) 时优先喂链 (读 A+4); 队列深时才有余量插 T。
    wire [31:0] iss_addr2  = (ins_sel_taken === 1'b1) ? ins_sel_tgt : iss_addr4;
    // ★★ #3: **链步进 —— R1c 链随预测走**:
    //   插入包 (ins_ok_i): 链推进 = **T+4** (跟随预测路径)。CPU 配对消费
    //     (A,T) 后 next_pc = T+4 (cpu_rv32i L1369), 单发消费 A 后预测 pc = T
    //     (队头即插入项 T) —— 队列与取指流严丝合缝 ⇒ 正确预测零 hint 零气泡。
    //     旧设计 (链恒顺序, 步进 8) 依赖 hint 每分支清链重启 —— R1a 打拍后
    //     这就是 +1 气泡/分支 的根源, R1c 从根上消除。
    //   ★ R1c-b: 顺序包 slot1 (=A+4) 是强 taken 分支时, 链推进 = **T1**
    //     (pred1_tgt; CPU 配对消费 (A,A+4) 后 next_pc = T1)。队列
    //     [A, A+4, T1, ...] 与"配对→T1"和"单发 A→A+4→(预测)→T1"两条消费
    //     路径都吻合 ⇒ hint 被"对齐则无害"判定吸收。判据与 slot0 同
    //     (vf2 ctr[1]); ret 不做 RAS 覆盖 (漂移由 hint/resync 兜底, 与 vf 同级)。
    //   顺序包: 步进 8/4 (与基线逐位一致)。
    //   预测/实际失配 (BTB 竞态/T miss/行填充) 由 short_push + resync 兜底:
    //   head 与 cpu_addr 每拍比对, 失配即重建 —— 安全网与链策略解耦。
    //   ★ 优先级: slot0 插入 (T+4) > slot1 预测 (T1) > 顺序步进 —— CPU 侧
    //     插入包的 slot1 不预测 (pkt_seq=0), 两者架构互斥。
    wire        ins1_ok   = (ins_sel1_taken === 1'b1) &&
                            ((ins_sel1_tgt < RAM_LIMIT) === 1'b1);
    // ★ DBG-1e: issue_next 只依赖 iss_addr + 寄存预测 (pnext_q/use_q_pred_q)
    //   或环外当拍查询 (突变拍, 地址源=寄存器) ⇒ next_issue 组合环断。
    //   ⚠ loop_nt 覆盖暂未接前瞻口 (定版再补), 变圈循环的插入抑制暂缺。
    wire [31:0] issue_next = (ins_ok_i === 1'b1) ? ins_sel_tgt4
                           : (ins1_ok  === 1'b1) ? ins_sel1_tgt
                           : (pred2 ? iss_addr8 : iss_addr4);

    assign req_addr2 = iss_addr2;             // ★ #3: 组合送 icache 第 2 读口
    // ★ R1c: 第 3 口废弃 (插入包 2 字化, 不再预取错路字 A+4)。连线保留仅为
    //   接口/tb 兼容, resp_3rd_* 不再参与任何控制 (综合可剪)。
    assign req_addr3 = iss_addr4;
    assign gnt   = do_issue;                  // 组合发出: 本拍 ready 拍即被后端锁存,
    assign gaddr = iss_addr;                  //   命中可做到 1 拍/请求满速流水

    assign resp      = hit;
    assign resp_data = (cnt != 0) ? f_data[rptr] : resp_rdata;

    // ★★ D2a/D2b: 队头的下一个字 = 取指包的第 2 条
    wire [AW-1:0] rptr_n = rptr + 2'd1;
    //   两种来源:
    //     · cnt>=2 : 第 2 条已在队里 (f_data[rptr+1] 由 exp_push 保证 = head+4)
    //     · 旁路拍 (cnt==0 且本拍有 2 字响应到达): 第 2 条就是本拍响应的
    //       `resp_nxt_data` —— **不能**读 f_data[rptr+1], 因为写入发生在时钟沿,
    //       本拍读到的还是上一圈的陈旧值。
    wire bypass2 = (cnt == 0) && push_ok && resp_nxt_vld;
    assign have_nxt      = (cnt >= 2) || bypass2;
    assign resp_data_nxt = (cnt != 0) ? f_data[rptr_n] : resp_nxt_data;
    // ★ #3: 第 2 条的**实际地址** (CPU 用它校验"第 2 条是不是 slot0 的后继")
    assign resp_nxt_addr = (cnt != 0) ? f_addr[rptr_n] : nxt_addr_q;

    //-----------------------------------------------------------------
    // 观测端口 (纯电平, 不参与任何控制)
    //-----------------------------------------------------------------
    assign pm_cnt_occ = cnt;
    assign pm_resync  = resync;
    assign pm_hint_fire = hint_fire;
    assign pm_evt     = { (cnt >= 2), (push_ok && resp_nxt_vld), pop, push_ok, do_issue };

    //-----------------------------------------------------------------
    // 状态更新
    //-----------------------------------------------------------------
    always @(posedge clk) begin
        if (!resetn) begin
            wptr <= 0; rptr <= 0; cnt <= 0;
            out_vld_q <= 1'b0; out_addr_q <= 32'h0;
            nxt_addr_q <= 32'h0; pt_q <= 1'b0; p2_q <= 1'b0; p1_q <= 1'b0; nxt1_addr_q <= 32'h0;
            next_issue <= 32'hFFFF_FFFC;      // 非法对齐值 → 首个请求必走 resync
            exp_push   <= 32'h0;
        end else begin
            // 在飞跟踪: 发出即占, 响应按序销账 (无论是否被过滤丢弃)
            if (do_issue) begin
                out_vld_q  <= 1'b1;
                out_addr_q <= iss_addr;
                nxt_addr_q <= iss_addr2;      // ★ #3: 本请求第 2 字的地址
                // ★ R1b-v2: 乐观插入 —— BTB taken 且目标在 RAM 区即当插入包
                //   (was: 还需 ready2 探测 T 命中; T miss 由 short_push 补发 T)
                pt_q       <= ins_ok_i;
                p2_q       <= pred2;          // ★ #4: 本请求是否预测"2 字"
                // ★ R1c/R1c-b: 在飞请求的链随属性 —— p1_q=slot1 链随包 (与
                //   插入包互斥), nxt1_addr_q=本请求的链推进地址 (exp_push 用)
                p1_q         <= (ins_ok_i === 1'b0) && (ins1_ok === 1'b1);
                nxt1_addr_q  <= issue_next;
            end else if (resp_vld)
                out_vld_q  <= 1'b0;

            if (push_now) begin               // 到达响应入队 (旁路消费同拍也入队)
                f_addr[wptr] <= resp_addr;
                f_data[wptr] <= resp_rdata;
                if (resp_nxt_vld) begin        // ★ 第二字同拍入队
                    f_addr[wptr + 2'd1] <= nxt_addr_q;   // ★ #3: 插入项=T / 顺序=A+4
                    f_data[wptr + 2'd1] <= resp_nxt_data;
                end
                // ★ R1c: 第 3 字不再入队 (插入包 2 字化, push3/word3 废弃)
                wptr     <= wptr + push_inc[AW-1:0];
                // 期望链地址推进 (★ R1c 统一式): short_push 拍 = 缺失格
                // nxt_addr_q (顺序=A+4 / 插入=T, next_issue 同拍拉回此处);
                // 正常拍 = **nxt1_addr_q** (发出拍锁存的链随地址) ——
                //   顺序包: = resp_addr+8/4 (与基线逐位一致);
                //   插入包: = T+4; slot1 链随包: = T1。
                //   ⚠ 回卷目标 (自旋环 T1/A+4 指向链上更早地址) 时, 旧顺序
                //   算术 resp_addr+8 会与回卷响应永久失配 ⇒ 响应全弃 ⇒ 饿死
                //   (irq 死锁实证), 必须用锁存的链随地址。
                exp_push <= (short_push === 1'b1) ? nxt_addr_q : nxt1_addr_q;
            end

            if (chain_rst) begin
                wptr <= 0; rptr <= 0; cnt <= 0;
                exp_push   <= iss_addr;       // 发出 iss_addr, 其响应到达时应能入队
                next_issue <= (do_issue === 1'b1) ? issue_next : iss_addr;  // ★ bugfix
            end else begin
                if (pop_nxt)      rptr <= rptr + 2'd2;
                else if (pop)     rptr <= rptr + 2'd1;
                cnt <= cnt_after;
                // ★★ 纪律 1: 只在**发出拍**推进 next_issue ⇒ 重复求值幂等
                if (short_push)   next_issue <= nxt_addr_q;          // ★ #4b/R1c: 补发缺失格 (顺序=A+4, 插入=T)
                else if (do_issue) next_issue <= issue_next;
            end
        end
    end
endmodule
