//=====================================================================
// icache.v : 2 路组相联指令 Cache (高阶任务1 / M1a)
//
// ★ 2026-10-08 阶段2: **4KB → 8KB**（64 set → 128 set）。
//   动机: 实测 fetch 税 13,907 拍 (3.3%) 几乎全部来自 icache 行填充
//   (1,487 次 × ~9 拍)。固件 ~15.7KB 而原 icache 仅 4KB (覆盖率 26%),
//   冲突缺失占相当比例 ⇒ 加倍后覆盖率 52%, 冲突缺失显著减少。
//   纯阵列尺寸改动, 无新增逻辑深度; BRAM 增量约 1~2 块 (DRM 70/415 已用)。
//
// 几何: 8KB 数据, 行 32B (8 字), 128 set × 2 way
//       index = addr[11:5], word = addr[4:2], tag = addr[31:12]
//
// 前端 (对接 pf_buffer v2 后端契约):
//   req_vld/req_addr : 组合请求 (ready 拍被锁存进流水级1)
//   ready            : IDLE 且 (级1 本拍不会判 miss) 且 无挂起 flush
//   resp_vld/resp_addr/resp_data : 命中延迟恰 1 拍 (与 BRAM 同步读同构);
//     miss = 检出 + 行填充到目标字 (≤11 拍); 非 RAM 区直通 0 (防死锁)
//
// 命中流水: T 拍锁存请求并并行索引 4 块存储 (tag×2 + data×2),
//   T+1 拍比较 tag + 2选1 选择 data. 命中路径每拍可收 1 请求 →
//   与 pf_buffer 1 指令/拍满速对接.
//
// Miss: 行突发填充 —— 连续 8 拍向 RAM I 口发 line_base+4k (1 拍同步回),
//   逐字写入受害 way (LFSR 随机替换, 进入填充时锁定), 末字到达拍写
//   tag+valid 并回 IDLE; 目标字到达拍即发 resp. 期间 ready=0.
//
// flush (M1c fence 钩子): 全局无效化. write-through 后端无脏行, 直接清
//   valid 即安全; 填充忙时挂起, 回 IDLE 后执行 (不丢请求语义).
//
//=====================================================================
// ★★★ 2026-10-08 (#1「取指包 =(pc,T)」): **第二读口改为"上层指定地址"**
//
//   动机 (见 doc/CPU_双发射优化方案_v1.md §11.21/§11.22):
//   D2b 之后 `slot0 不净` 的 **100% 来自 "BTB 在 pc 预测 taken"** (50,767 条
//   = 15.1% 指令)。配对是 `(pc, pc+4)`, 而预测 taken 时正确后继是 **T**,
//   `pc+4` 是错路 ⇒ slot1 只能是废的。要让 slot1 装 T, 取指包必须变成
//   `(pc, T)` ⇒ 第 2 读口要读的**不是** `req_addr+4`, 而是
//   `iss_addr2 = pred_taken ? pred_tgt : req_addr+4` (由 pf_buffer 给出)。
//
//   ⚠ 为什么必须**独立读 tag**: 预测 taken 时第 2 字在**另一条 cache 行**
//     (另一个 set / 另一个 tag), 不能复用第 1 字的 tag 比较结果。
//
//   ⚠⚠ 端口数纪律 (Pango BRAM 只有 2 个真双口端口):
//     · `dat0/dat1` 原本已是 **2 个寄存读口 + 1 个填充写口** (q_dat0/q_dat0n),
//       本改动只是把第 2 个读口的**地址**换掉, **端口数不变** ⇒ BRAM 推断不变 ✓
//     · `tag0/tag1` 原本是 1 寄存读 + 1 写; 再加一个读口就变 3 端口 ⇒
//       BRAM 推不出来, 会退化成 FF 阵列 (2560 FF/way)。这里改用 **dcache v6
//       的同款办法**: tag 阵列**全部组合读** (深度 128 → 分布式 RAM,
//       RAM64X1D 天然 1 写 2 读), 把"tag 比较"从**输出级**提前到**接收拍**,
//       比较结果寄存成 `rd1_hit/rd1_way`/`rd2_ok/rd2_way`。
//       副作用是**时序更好**: tag 比较 + `v0[idx]` 128:1 mux 整条移出
//       `resp_data` 出口, 出口只剩一个寄存器选通。
//=====================================================================
`timescale 1ns/1ps

module icache (
    input  wire        clk,
    input  wire        resetn,

    input  wire        req_vld,
    input  wire [31:0] req_addr,
    // ★★ #1: 第二读口地址 (pf_buffer 给)
    //   顺序包 = req_addr+4; 预测 taken 包 = BTB 预测目标 T。
    //   必须在 `req_vld && ready` 那拍与 req_addr **同拍**有效。
    input  wire [31:0] req_addr2,
    // ★★ #4: **第三读口** —— 插入包需要 3 个字 (A、A+4、T):
    //   第 2 口给"插入项/后继", 第 3 口给"链的下一格"。没有它就只能二选一
    //   ⇒ 链填充率折半 ⇒ 取指饿死 (实测 §11.25.3)。
    input  wire [31:0] req_addr3,
    output wire        ready,
    output wire        resp_vld,
    output wire [31:0] resp_addr,
    output wire [31:0] resp_data,
    // ★★ D2a (2026-10-08): **一拍给两个字** —— 双发射取指
    //   `resp_data_nxt` = `req_addr2` 处的字; `resp_nxt_vld` 表示有效。
    output wire [31:0] resp_data_nxt,
    output wire        resp_nxt_vld,
    // ★★ #4: 第 3 个字 + 有效
    output wire [31:0] resp_data_3rd,
    output wire        resp_3rd_vld,
    // ★★★ #4c: **组合**告诉上层"req_addr2 / req_addr3 这一拍是否真的能取到字"。
    //   动机: 上层原来用启发式 (`a[4:2]!=7 && a<RAM_LIMIT`) 预测第 2 字是否可用,
    //   一旦预测"2 字"而实际只回 1 字 (miss / 跨行未命中), 链步进就与预期错位 ⇒
    //   响应地址不符被丢弃 ⇒ **链永久跳过一格** ⇒ CPU 停在那一格 ⇒ resync。
    //   实测这就是 94% "紧跟配对" 的 resync 的来源。
    //   ⇒ 把真实可用性组合送给上层, 让它**精确**决定步进 (不再有错位)。
    //   ⚠ 无组合环: `ready2/3` 只依赖 req_addr2/3, 而后者不依赖它们。
    //   ⚠ **§5f-1 R1b (2026-10-15) 实验**: 曾把 ready2/3 改寄存输出以切重定向域
    //   组合环, 但实测导致 pf_buffer 插入包/步进预测变陈旧 → 取指链错位 →
    //   CoreMark 功能损坏 (28K 预告不符 / 72K resync) → **已回退**, 维持组合输出。
    // ★ #4c: ready2/3 保持**组合输出** (§5f-1 R1b 实验表明寄存会破坏 pf_buffer
    //   插入包/步进预测 → 取指链错位 → 功能损坏, 已回退; 详见会话记录)
    output wire        ready2,
    output wire        ready3,
    input  wire        flush,

    // 后端 (simple_ram I 口, 1 拍同步读)
    output wire        r_valid,
    output wire [31:0] r_addr,
    input  wire [31:0] r_data,
    input wire early_fill_allowed,

    // 观测: 填充忙 (PM 归因用)
    output wire        miss_busy
);
    localparam RAM_LIMIT = 32'h0002_0000;   // 128KB (RAM_ADDR_W=17)

    //-----------------------------------------------------------------
    // 存储阵列 (PDS 推断: tag = 分布式 RAM, data = BRAM)
    //-----------------------------------------------------------------
    reg [19:0] tag0 [0:127], tag1 [0:127];  // addr[31:12]
    reg [127:0] v0, v1;                     // per-way 位 valid (128 set)
    reg [31:0] dat0 [0:1023];               // {index[6:0], word[2:0]}
    reg [31:0] dat1 [0:1023];

    reg [15:0] lfsr = 16'hACE5;             // 随机替换
    always @(posedge clk)
        lfsr <= {lfsr[14:0], lfsr[15]^lfsr[13]^lfsr[12]^lfsr[10]};
    wire       victim = lfsr[0];

    //-----------------------------------------------------------------
    // 两个读口 (★ 均为**接收拍组合**索引 + 寄存结果)
    //   口1 = req_addr (取指包第 1 条), 口2 = req_addr2 (第 2 条)
    //   数据口: 仍为**寄存读** (BRAM 必须寄存读! 组合读大数组会退化 LUTRAM)
    //   tag 口: **组合读** —— 见文件头"端口数纪律"
    //-----------------------------------------------------------------
    wire [ 6:0] ridx   = req_addr[11:5];
    wire [ 6:0] ridx_n = req_addr2[11:5];
    wire [ 6:0] ridx_m = req_addr3[11:5];
    wire [19:0] rtag   = req_addr[31:12];
    wire [19:0] rtag_n = req_addr2[31:12];
    wire [19:0] rtag_m = req_addr3[31:12];
    wire        ram_ok_r  = (req_addr  < RAM_LIMIT);
    wire        ram_ok_rn = (req_addr2 < RAM_LIMIT);
    wire        ram_ok_rm = (req_addr3 < RAM_LIMIT);

    // 组合读 tag (小阵列 → 分布式 RAM, 1 写 2 读)
    wire [19:0] t0c = tag0[ridx];
    wire [19:0] t1c = tag1[ridx];
    wire [19:0] t0n = tag0[ridx_n];
    wire [19:0] t1n = tag1[ridx_n];
    //   ⚠ 用 `==` (与原版逐位一致): `v0[ridx]` 为 0 时 `&&` 直接把 X 收成 0
    //     (valid 只在同拍与 tag 一起写 ⇒ "valid=1 而 tag 是 X" 架构不可达)。
    wire        h0c = v0[ridx] && (t0c == rtag);
    wire        h1c = v1[ridx] && (t1c == rtag);
    wire        h0n = v0[ridx_n] && (t0n == rtag_n);
    wire        h1n = v1[ridx_n] && (t1n == rtag_n);
    wire [19:0] t0m = tag0[ridx_m];
    wire [19:0] t1m = tag1[ridx_m];
    wire        h0m = v0[ridx_m] && (t0m == rtag_m);
    wire        h1m = v1[ridx_m] && (t1m == rtag_m);
    // ★ #4c: **组合**告诉上层 req_addr2/req_addr3 这一拍是否真的能取到字.
    //   (§5f-1 R1b 实验: 改寄存输出会破坏 pf_buffer 的步进/插入包预测 → 回退)
    assign ready2 = ram_ok_rn && (h0n || h1n);
    assign ready3 = ram_ok_rm && (h0m || h1m);

    reg  [31:0] q_dat0, q_dat1, q_dat0n, q_dat1n;
    reg  [31:0] q_dat0m, q_dat1m;            // ★ #4: 第 3 读口数据
    reg         rd1_hit, rd1_way;            // 口1 命中 (接收拍算好, 寄存)
    reg         rd2_ok,  rd2_way;            // 口2 命中 (接收拍算好, 寄存)
    reg         rd3_ok,  rd3_way;            // ★ #4: 口3 命中 (接收拍算好, 寄存)

    //-----------------------------------------------------------------
    // 状态与流水级1
    //-----------------------------------------------------------------
    localparam S_IDLE = 1'b0, S_FILL = 1'b1;
    reg        st;
    reg        s1_vld;                      // 级1 持有有效请求 (未响应)
    reg [31:0] a1;                          // 级1 地址 (命中比较 / 填充基准)
    reg        ram_ok1;                     // 级1 地址在 RAM 区
    reg        fway;                        // 填充受害 way (进入时锁定)
    reg [ 3:0] fc;                          // 填充读计数 0..7
    reg        rv_q;                        // 上拍 r_valid (字到达标志)
    reg [ 3:0] fc_q;                        // 到达字序号
    reg        flush_req;

    wire [ 6:0] idx  = a1[11:5];
    wire [ 2:0] word = a1[4:2];
    wire [19:0] tag  = a1[31:12];

    // ★ #1: 命中判定改用寄存好的 rd1_hit / rd1_way (tag 比较已移出输出级)
    wire        hit_any   = rd1_hit;
    wire [31:0] hit_data  = rd1_way ? q_dat1 : q_dat0;
    wire [31:0] hit_data_n= rd2_way ? q_dat1n : q_dat0n;

    // 级1 判 miss → 本拍起拒收 (同拍即填充起点, 无空转)
    wire will_miss = (st == S_IDLE) && s1_vld && ram_ok1 && !hit_any;
    wire early_start=will_miss&&(early_fill_allowed===1'b1);
    assign ready   = (st == S_IDLE) && !will_miss && !flush_req;

    // 响应: 命中(级1) | 非RAM直通0(级1) | 填充目标字到达
    wire fill_arrive = (st == S_FILL) && rv_q && (fc_q != 4'h8);
    // Isolate RAM return from decode/pairing and prefetch CE paths.
    // Retain every acknowledged miss reply, including during cache flush.
    wire fill_target_arrive=fill_arrive&&(fc_q==4'd0);
    reg fill_resp_q;
    reg [31:0] fill_data_q,fill_addr_q;
    always @(posedge clk)begin
        if(!resetn)begin fill_resp_q<=0;fill_data_q<=0;fill_addr_q<=0;end
        else begin
            fill_resp_q<=fill_target_arrive;
            if(fill_target_arrive)begin fill_data_q<=r_data;fill_addr_q<=a1;end
        end
    end
    assign resp_vld=((st==S_IDLE)&&s1_vld&&(!ram_ok1||hit_any))||fill_resp_q;
    assign resp_addr=fill_resp_q?fill_addr_q:a1;
    assign resp_data=fill_resp_q?fill_data_q:(ram_ok1?hit_data:32'h0);
    // ★ D2a/#1: 第二个字 (地址 = 上层给的 req_addr2)
    //   有效条件: 级1 命中 且 口2 也在接收拍命中 (rd2_ok 已含 RAM 区检查)
    //   填充期一律无效 (resp_nxt_vld=0) ⇒ 上层 have_nxt=0, 走 resync 兜底。
    assign resp_data_nxt = rd2_ok ? hit_data_n : 32'h0;
    assign resp_nxt_vld  = (st == S_IDLE) && s1_vld && ram_ok1 &&
                           hit_any && rd2_ok;
    // ★★ #4: 第 3 个字 (地址 = 上层给的 req_addr3)
    assign resp_data_3rd = rd3_ok ? (rd3_way ? q_dat1m : q_dat0m) : 32'h0;
    assign resp_3rd_vld  = (st == S_IDLE) && s1_vld && ram_ok1 &&
                           hit_any && rd3_ok;

    // RAM I 口: 仅填充期使用
    assign miss_busy = st || early_start;                       // S_FILL = 行填充中
    assign r_valid = early_start || ((st == S_FILL) && (fc != 4'h8));
    wire [2:0] refill_issue_word=word+(early_start?3'd0:fc[2:0]);
    wire [2:0] refill_arrive_word=word+fc_q[2:0];
    assign r_addr  = {a1[31:5], refill_issue_word, 2'b00};   // fc[2:0]: 4 位全拼会整体左移 1 位 (33bit 截断 bug)

    //-----------------------------------------------------------------
    always @(posedge clk) begin
        if (!resetn) begin
            st <= S_IDLE; s1_vld <= 1'b0; a1 <= 32'h0; ram_ok1 <= 1'b0;
            fway <= 1'b0; fc <= 4'h0; rv_q <= 1'b0; fc_q <= 4'h0;
            flush_req <= 1'b0; v0 <= 128'h0; v1 <= 128'h0;
            rd1_hit <= 1'b0; rd1_way <= 1'b0; rd2_ok <= 1'b0; rd2_way <= 1'b0;
            rd3_ok  <= 1'b0; rd3_way <= 1'b0;
        end else begin
            rv_q <= r_valid;
            fc_q <= early_start?4'd0:fc;
            if (flush) flush_req <= 1'b1;

            // ---- 两个读口: 数据寄存读 + tag 比较结果寄存 (接收拍算) ----
            //   ⚠ 用 `=== 1'b1` 收口: `req_addr2` 可能来自 X (上游 iss_addr 三目),
            //     一旦 X 进 rd2_ok 就会沿 resp_nxt_vld → have_nxt → 配对判定扩散。
            //     归成 0 = "第 2 字不可用" ⇒ 上层 have_nxt=0, 只损失一次配对, 安全。
            q_dat0  <= dat0[{ridx,   req_addr[4:2]}];
            q_dat1  <= dat1[{ridx,   req_addr[4:2]}];
            q_dat0n <= dat0[{ridx_n, req_addr2[4:2]}];
            q_dat1n <= dat1[{ridx_n, req_addr2[4:2]}];
            q_dat0m <= dat0[{ridx_m, req_addr3[4:2]}];
            q_dat1m <= dat1[{ridx_m, req_addr3[4:2]}];
            rd1_hit <= (req_vld && ready && (h0c || h1c)) === 1'b1;
            rd1_way <= (h1c) === 1'b1;
            rd2_ok  <= (req_vld && ready && ram_ok_rn && (h0n || h1n)) === 1'b1;
            rd2_way <= (h1n) === 1'b1;
            rd3_ok  <= (req_vld && ready && ram_ok_rm && (h0m || h1m)) === 1'b1;
            rd3_way <= (h1m) === 1'b1;

            case (st)
            S_IDLE: begin
                // flush 只清 valid, 绝不动 s1_vld (清了会吞掉已 ack 请求的响应,
                // pf_buffer 等不到 resp 永久挂起 —— M0 教训同源). write-through
                // 后端下在级请求命中判定被清掉的 valid = 判 miss → 重新填充, 正确.
                if (flush_req) begin
                    v0 <= 128'h0; v1 <= 128'h0;
                    flush_req <= 1'b0;
                end
                if (req_vld && ready) begin    // 锁存新请求进级1
                    a1      <= req_addr;
                    ram_ok1 <= ram_ok_r;
                    s1_vld  <= 1'b1;
                end else if (will_miss) begin
                    s1_vld  <= 1'b1;           // miss 请求保留 (填充目标)
                end else begin
                    s1_vld  <= 1'b0;           // 已响应/无请求
                end
                if (will_miss) begin
                    st   <= S_FILL;
                    fc   <= early_start?4'h1:4'h0;
                    fway <= victim;
                end
            end
            S_FILL: begin
                if (fill_arrive) begin         // 到达字写入受害 way
                    if (fway) dat1[{idx,refill_arrive_word}] <= r_data;
                    else      dat0[{idx,refill_arrive_word}] <= r_data;
                    if (fc_q == 4'h7) begin    // 末字: 写 tag+valid, 归位
                        if (fway) begin tag1[idx] <= tag; v1[idx] <= 1'b1; end
                        else        begin tag0[idx] <= tag; v0[idx] <= 1'b1; end
                        st     <= S_IDLE;
                        s1_vld <= 1'b0;
                        fc_q   <= 4'h8;        // 防本拍后误判
                    end
                end
                if (fc != 4'h8) fc <= fc + 4'h1;
            end
            endcase
        end
    end
endmodule
