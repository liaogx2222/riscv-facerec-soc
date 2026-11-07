//=====================================================================
// dcache.v : 2 路组相联数据 Cache (高阶任务1 / M1b + S6a-R 命中 1 拍化)
//
// ★★★ 2026-10-07 阶段0 / Part A：**恢复端口B 快路, 但预告地址改为寄存器化**
//   背景：措施3 (2026-09-22) 把端口B 整条砍掉, 因为 `d_addr_pre` 是
//   `assign d_addr_pre = mem_addr_word;` —— EX 地址加法器**组合直出**,
//   把「WB → 寄存器堆旁路 → EX 转发 mux → EX 加法器 → 阵列地址」接成
//   14.627 ns 的关键路径。砍掉后命中 1→2 拍, 代价 15.8% 周期 (dport_wait)。
//   本次做法：**不动端口B 的任何逻辑**, 只把 CPU 侧的 `d_addr_pre` 源从
//   「EX 组合」换成「ID/EX 寄存器」(cpu_rv32i.v 新增 id_ex_mem_addr_pre,
//   = ID 级算好的字对齐 rs1+imm)。阵列地址从此只由寄存器驱动 ⇒
//   加法器整条移出该路径 (新路径 ≈4~5 ns), 而命中 1 拍的收益全部拿回。
//   安全性：`fast` 的 (pre_a == d_addr) 门本就存在 —— ID 投机地址一旦被
//   EX 转发改写就自动失配, 逐拍落回 2 拍慢路 ⇒ **最坏 = 措施3 版, 绝不更差**。
//
// 有效几何: 2KB, 行32B (8字), 32 set × 2 way。
// 数组声明64行，但当前索引addr[9:5]只有5位，最高索引位恒0。
// 本轮保持既有地址映射与访存周期，不扩容。
//
// 前端握手 (S8 实测: dcache miss 仅 0.4% → 收益全在"命中少一拍协议税"):
//   * 端口A (慢路, 原 2 拍版逐拍不变): accept 拍寄存读 d_addr, T+1 比较出
//     rdy —— pre 不成立的场合(pre_ok=0 / 地址不符 / 复位首笔 / 外设转 RAM)
//     全部落回此路, 永远正确.
//   * 端口B (快路, S6a-R): CPU 在"该访存即将于下拍进 EX/MEM"的 adv 拍给
//     d_valid_pre/d_addr_pre → 端口B当拍寄存读, 操作 d_valid 首拍 p_* 恰
//     有效 → 命中当拍出 rdy+data (1 拍访存). 门:
//     fast = d_valid && !acc && !filling && pre_ok && (pre_a==d_addr).
//   * pre_ok: pre 发出拍须非填充 —— fill 末拍(adv=1)真 CPU 会发下一笔预告,
//     而该拍端口B读到"填充写到一半的 way + 旧 tag", tag 门可能骗过而数据错
//     (victim 旧 tag==下一笔 tag 的角落) → 用 filling 一拍否决, 该笔落回慢路.
//   * RAW 前递: 快/慢命中拍的 store 合并写与端口B同边读撞同一字时, 用合并
//     值覆盖 p_dat (背对背 store→load 同字, v1 的鬼坑显式封死).
//
// 写策略: write-through + write-allocate (不变). accept/快命中拍写通直达 RAM;
//   miss 8 拍行突发读回(自带新数据); 命中 store 合并写命中 way.
// flush(fence): 挂起到空闲, 只清 valid; 绝不清在飞请求.
//
// ★★★ 2026-10-08 v6：**把 tag 比较从 d_rdata 数据通路上彻底摘掉**
//   背景：阶段1 的 MEM→EX load 旁路让 d_rdata 进了 EX 级 ALU 的操作数。PDS 实测
//   关键路径 = dcache tag RAM(tco 1.644) → tag 比较(0.613, 3 级) → fh0(fanout 70,
//   0.476) → f_old/d_rdata 两级 mux → d_rdata → 移位 → alu_b → ALU → 结果,
//   15.247 ns / 20 级, WNS −1.372 @ 108 MHz。
//   做法：**把端口B 的 tag 读改成组合读 + 比较结果寄存**。
//     · tag 阵列 64×22 改为组合读 ⇒ 综合成分布式 RAM (RAM64X1×22×2 ≈ 44 LUT,
//       门禁规则: 深度 <256 的数组按 flops/LUTRAM 处理, 多口无害)；
//     · 预告拍(T)用组合读出的 pt0/pt1 与 `d_addr_pre[31:10]` 比较, 结果存
//       `pre_hit`/`pre_way` ⇒ 到 accept 拍(T+1) 时它们已是**纯寄存器**；
//     · `f_old = pre_way ? p_dat1 : p_dat0`、`d_rdata = accept ? f_old : hit_old`
//       —— 全部由寄存器选通 ⇒ 数据通路只剩 **1 个 LUT6 级**。
//   安全性 (逐条)：
//     · 预测错/miss ⇒ pre_hit=0 ⇒ fast_hit=0 ⇒ d_rdy=0 ⇒ 全流水线冻结,
//       该拍 d_rdata 不会被任何寄存器锁存, 落回慢路 ⇒ 绝不取到脏值；
//     · flush: pre_hit 是**预告拍**算的, 而 v0/v1 可能在该拍之后被清 ⇒ 用
//       `!flush && !flush_req` 把 fast_hit 保守否决 (fence 极少, 落回慢路无害)；
//     · 填充中/填充末拍: `!filling` 已在 pre_hit 的条件里 (旧 pre_ok 同款)。
//
// 历史: v1 组合读阵列(42错)→v2 单口mux(破坏慢路,不可行)→v3 双口(pre_ok/
//   RAW 后慢路 0 错, 但单元 tb 建模不出 EX 提前拍, 快路误判)→ v3-R(慢路=原
//   逻辑逐拍复制, 验证主战场移到 SoC 级 coremark 三 CRC)→ v4(措施3: 去端口B)
//   → v5(阶段0: 恢复端口B, 预告地址改寄存器源) → **v6(本版: tag 比较寄存化)**.
//   慢路 (端口A) 逐拍逻辑自 v3-R 起从未改动。2 拍版备份:
//   `_trash/2026-10-07/stage0_before/dcache.v`。
//=====================================================================
`timescale 1ns/1ps

module dcache #(
    parameter SEPARATE_FAST_ADDR = 0,
    parameter PARALLEL_PRETAG = 0
)(
    input  wire        clk,
    input  wire        resetn,

    input  wire        d_valid,
    input  wire        d_write,
    input  wire [31:0] d_addr,
    input  wire [31:0] d_read_addr,
    input  wire [31:0] d_write_addr,
    input  wire [31:0] d_wdata,      // 已按字节左移对齐 (EX 级完成)
    input  wire [ 3:0] d_wstrb,
    output wire        d_rdy,
    output wire [31:0] d_rdata,
    input  wire        flush,

    // S6a-R: EX 级早一拍预告 (CPU 与 ex_mem 捕获同构生成)
    input  wire        d_valid_pre,
    input  wire [31:0] d_addr_pre,
    input wire [159:0] d_pretag_addresses,
    input wire [2:0] d_pretag_choice,

    // 后端 (simple_ram D 口)
    output wire        m_valid,
    output wire        m_write,
    output wire [31:0] m_addr,
    output wire [31:0] m_wdata,
    output wire [ 3:0] m_wstrb,
    input  wire [31:0] m_rdata,

    // S8 测量面板 (纯观测, 组合脉冲, 不进时序路径)
    output wire        stat_hit,      // 命中退休拍 (含快路)
    output wire        stat_miss,     // miss 起意拍
    output wire        stat_fast,     // ★ 快路(S6a-R)命中脉冲 (观测: dport_wait 归因)
    output wire        stat_slow,     // ★ 慢路(2 拍)命中脉冲
    output wire        stat_nopre,    // ★ 慢路归因: 没有预告 (pre_ok=0)
    output wire        stat_mism,     // ★ 慢路归因: 预告地址不符 (pre_a != d_addr)
    output wire        stat_fmiss     // ★ 慢路归因: 真 miss
);
    // The routed critical path starts at the tag BRAM clock-to-Q. Keep
    // these small dual-read tag arrays in distributed RAM, with the same
    // registered slow read and combinational pre-read cycle protocol.
    reg [21:0] tag0 [0:63] /* synthesis syn_ramstyle="select_ram" */;
    reg [21:0] tag1 [0:63] /* synthesis syn_ramstyle="select_ram" */;
    reg [63:0] v0, v1;
    reg [31:0] dat0 [0:511], dat1 [0:511];

    reg [15:0] lfsr = 16'hBEEF;
    always @(posedge clk)
        lfsr <= {lfsr[14:0], lfsr[15]^lfsr[13]^lfsr[11]^lfsr[2]};
    wire victim = lfsr[0];

    //-----------------------------------------------------------------
    // 流水级1 + 填充状态
    //-----------------------------------------------------------------
    reg        acc;                          // 请求持有到 rdy (慢路)
    reg [31:0] a1;  reg we1;  reg [31:0] wd1;  reg [3:0] ws1;
    reg        filling;
    reg [ 3:0] fc;
    reg        rv_q;
    reg [ 3:0] fc_q;
    reg        fway;
    reg [31:0] tgt_word;
    reg        flush_req;
    // A miss completes one edge after the final refill word is captured.
    // Keep hits unchanged; no main-RAM data reaches the CPU reply mux.
    (* syn_preserve=1 *) reg fill_reply_q;
    (* syn_preserve=1 *) reg [31:0] fill_reply_data_q;

    wire [ 5:0] idx  = a1[9:5];
    wire [ 2:0] word = a1[4:2];
    wire [21:0] tag  = a1[31:10];

    //-----------------------------------------------------------------
    // 端口 A/B 读口 + pre 流水 + RAW 前递
    //   ★ v6: 端口B 的 tag 改为**组合读**, 命中/命中 way 在预告拍算完并寄存。
    //     ⇒ d_rdata 的选择只剩寄存器选通, tag 比较不再挂在数据出口上。
    //-----------------------------------------------------------------
    wire [5:0] ridx  = d_addr[9:5];
    wire [5:0] pridx = d_addr_pre[9:5];
    reg  [21:0] q_tag0, q_tag1;              // A: 慢路比较级 (寄存)
    reg  [31:0] q_dat0, q_dat1;
    reg  [31:0] p_dat0, p_dat1;              // B: 快路数据级 (寄存)
    reg         pre_ok;                      // 预告有效 (发拍非填充)
    reg  [31:0] pre_a;
    reg         pre_hit;                     // ★ v6: 快路命中 (预告拍算好, 寄存)
    reg         pre_way;                     // ★ v6: 命中 way (1=way1)

    // ---- 预告拍: tag 组合读 + 比较 (结果下一拍即用, 不进数据通路) ----
    wire pvh0,pvh1;
    generate if(PARALLEL_PRETAG)begin:g_parallel_pretag
        wire [4:0] hit_way0,hit_way1;
        genvar pp;
        for(pp=0;pp<5;pp=pp+1)begin:g_hint
            wire [31:0] a=d_pretag_addresses[pp*32+:32];
            dcache_pre_tag_match u_match(.tag0(tag0[a[9:5]]),.tag1(tag1[a[9:5]]),
                .valid0(v0[a[9:5]]),.valid1(v1[a[9:5]]),.request_tag(a[31:10]),
                .hit0(hit_way0[pp]),.hit1(hit_way1[pp]));
        end
        assign pvh0=hit_way0[d_pretag_choice];
        assign pvh1=hit_way1[d_pretag_choice];
    end else begin:g_legacy_pretag
        wire [21:0] pt0=tag0[pridx],pt1=tag1[pridx];
        wire [21:0] pft=d_addr_pre[31:10];
        assign pvh0=v0[pridx]&&(pt0==pft);
        assign pvh1=v1[pridx]&&(pt1==pft);
    end endgenerate

    wire hit0 = v0[idx] && (q_tag0 == tag);
    wire hit1 = v1[idx] && (q_tag1 == tag);
    wire hit_any  = hit0 || hit1;
    wire [31:0] hit_old = hit0 ? q_dat0 : q_dat1;

    wire accept = d_valid && !acc;

    // ---- 快路评估 (d_valid 首拍, 端口B数据) ----
    // Compare independent register-sourced candidates before selecting a bit.
    // Late SB arbitration no longer precedes a full-width address compare.
    wire fast_addr_match = SEPARATE_FAST_ADDR
        ? ((d_write && (pre_a == d_write_addr)) || (!d_write && (pre_a == d_read_addr)))
        : (pre_a == d_addr);
    wire         fast   = accept && !filling && pre_ok && fast_addr_match;
    //   flush 保守否决: pre_hit 是预告拍算的, v0/v1 之后可能被 fence 清掉
    // ★★★ 2026-10-09: **X 安全**。`pre_a` 现在可能来自 ID 级投机转发的组合值
    //   (`alu_result`), 而寄存器堆里本来就可能存在 X (§11.15) ⇒ `pre_a` 可能是 X
    //   ⇒ `(pre_a == d_addr)` 是 X ⇒ `fast` 是 X ⇒ `d_rdy` 是 X ⇒ **全流水线死锁**。
    //   用 `(fast === 1'b1)` 把 X 归到"快路不成立" ⇒ 落回 2 拍慢路 (永远正确),
    //   只损失那一次加速机会。
    wire         fast_hit = (fast === 1'b1) && pre_hit && !flush && !flush_req;
    wire [ 5:0]  fidx   = d_addr[9:5];
    wire [ 2:0]  fword  = d_addr[4:2];
    // ★ v6: 选通信号全是寄存器 (pre_way / accept) ⇒ 一个 LUT6 级即可完成
    wire [31:0]  f_old  = pre_way ? p_dat1 : p_dat0;

    function [31:0] bytemux;
        input [31:0] old, nw;
        input [ 3:0] m;
        begin
            bytemux = { m[3]?nw[31:24]:old[31:24], m[2]?nw[23:16]:old[23:16],
                        m[1]?nw[15:8] :old[15:8],  m[0]?nw[7:0]  :old[7:0]  };
        end
    endfunction

    // ---- RAW 前递描述: 本边界的 store 合并写 (快|慢) ----
    wire        mb_fast = fast_hit && d_write;
    wire        mb_slow = hit_beat && we1;
    wire        mb_en   = mb_fast || mb_slow;
    wire [ 8:0] mb_slot = mb_fast ? {fidx, fword} : {idx, word};
    wire [31:0] mb_data = mb_fast ? bytemux(f_old,   d_wdata, d_wstrb)
                                  : bytemux(hit_old, wd1,      ws1);
    wire        mb_way0 = mb_fast ? ~pre_way : hit0;

    always @(posedge clk) begin
        q_tag0 <= tag0[ridx];  q_tag1 <= tag1[ridx];
        q_dat0 <= dat0[{ridx, d_addr[4:2]}];
        q_dat1 <= dat1[{ridx, d_addr[4:2]}];
        p_dat0 <= dat0[{pridx, d_addr_pre[4:2]}];
        p_dat1 <= dat1[{pridx, d_addr_pre[4:2]}];
        pre_ok  <= d_valid_pre && !filling;
        // ★★★ L1 修复: `pre_a` 必须与 `pre_ok` **同拍锁存**。
        //   原来 `pre_a <= d_addr_pre` 是无条件的, 而 `pre_ok` 有条件 ⇒
        //   一旦某拍 `d_valid_pre=0` (ID/EX 是气泡) 而 pre_ok 仍是 1,
        //   `pre_a` 就被**无关指令**的地址覆盖 ⇒ accept 拍必然 `pre_a != d_addr`
        //   ⇒ 快路必失配 (落回 2 拍慢路)。
        if (d_valid_pre && !filling) pre_a <= d_addr_pre;
        // ★ v6: 命中 + 命中 way 在**预告拍**算完 (tag 组合读), 结果寄存
        pre_hit <= d_valid_pre && !filling && (pvh0 || pvh1);
        pre_way <= pvh1;
        if (d_valid_pre && mb_en && {pridx, d_addr_pre[4:2]} == mb_slot) begin
            if (mb_way0) p_dat0 <= mb_data;
            else         p_dat1 <= mb_data;
        end
    end

    //-----------------------------------------------------------------
    // 后端直通 (组合, 保证 accept/快命中拍 RAM 采到本拍数据)
    //-----------------------------------------------------------------
    wire [31:0] line_addr = {a1[31:5], fc[2:0], 2'b00};
    assign m_valid = accept || filling;
    assign m_write = accept && d_write;          // write-through (快慢同拍提交)
    assign m_addr  = accept ? d_addr : line_addr;
    assign m_wdata = d_wdata;
    assign m_wstrb = d_wstrb;

    //-----------------------------------------------------------------
    // 响应: 快命中(首拍) | 慢命中(accept+1) | fill_done
    //-----------------------------------------------------------------
    wire fill_done = filling && rv_q && (fc_q == 4'h7);
    wire hit_beat  = acc && !filling && !fill_reply_q && hit_any; // 慢路 (pre 不成立/快 miss 复核)
    assign d_rdy   = fill_reply_q || (!filling && (fast_hit || hit_beat));
    wire [2:0] word_sel = a1[4:2];
    // ★ v6 (2026-10-08 修时序): 选通由 `fast_hit` 改为 `accept`。
    //   accept 拍: f_old 已是"预告拍算好并寄存的命中 way"数据 (pre_way 选通);
    //   accept+1 拍: 慢路 hit_old。安全性: accept 且非 fast_hit 时 d_rdy=0
    //   ⇒ 整条流水线冻结 ⇒ 该拍 d_rdata 不被锁存、无提交。
    //   这样 tag 比较 (0.613 ns / 3 级) 与 fh0 的 fanout-70 网络 (0.476 ns)
    //   彻底移出 d_rdata 路径。
    // On a valid ready beat: fast_hit implies !acc, hit_beat implies acc,
    // and fill_done uses the separate filling arm. Select by registered
    // phase instead of accept to remove SB/request arbitration from data.
    // Idle/non-ready data is not consumed by the CPU.
    assign d_rdata = fill_reply_q ? fill_reply_data_q : (acc ? hit_old : f_old);

    assign stat_hit  = hit_beat || fast_hit;
    assign stat_fast = fast_hit;
    assign stat_slow = hit_beat;
    assign stat_nopre = accept && !fast_hit && !pre_ok;
    assign stat_mism  = accept && !fast_hit && pre_ok && (pre_a != d_addr);
    assign stat_fmiss = accept && !fast_hit && pre_ok && (pre_a == d_addr);
    assign stat_miss = acc && !filling && !fill_reply_q && !hit_any && !fast;  // 慢路 miss 定夺拍 (快 miss 时 acc 尚 0)

    //-----------------------------------------------------------------
    always @(posedge clk) begin
        if (!resetn) begin
            acc <= 1'b0; filling <= 1'b0; fc <= 4'h0; rv_q <= 1'b0; fc_q <= 4'h0;
            fway <= 1'b0; a1 <= 32'h0; we1 <= 1'b0; wd1 <= 32'h0; ws1 <= 4'h0;
            tgt_word <= 32'h0; flush_req <= 1'b0;
            fill_reply_q<=1'b0;fill_reply_data_q<=32'b0;
            v0 <= 64'h0; v1 <= 64'h0;
        end else begin
            rv_q <= m_valid && !m_write;
            fc_q <= fc;
            if (flush) flush_req <= 1'b1;

            if (accept) begin                    // ---- accept 拍: 级1 装载 ----
                acc <= fast_hit ? 1'b0 : 1'b1;   // 快命中当拍退休, 不占用持有
                a1  <= d_addr;
                we1 <= d_write;
                wd1 <= d_wdata;
                ws1 <= d_wstrb;
                fc  <= 4'h0;
            end

            if (fill_reply_q) begin
                // The held request retires here, without a second write or
                // a new lookup/refill. acc blocks another acceptance.
                fill_reply_q<=1'b0;
                acc<=1'b0;
            end else if (!filling) begin
                if (fast_hit) begin              // ---- 快命中 store: 首拍合并写 ----
                    if (d_write) begin
                        // ★ v6: way 由寄存的 pre_way 给出 (fast_hit 已保证它就是命中 way)
                        if (~pre_way) dat0[{fidx,fword}] <= bytemux(f_old, d_wdata, d_wstrb);
                        else          dat1[{fidx,fword}] <= bytemux(f_old, d_wdata, d_wstrb);
                    end
                end else if (hit_beat) begin     // ---- 慢路命中 rdy 拍 ----
                    if (we1) begin               // 命中 store: 合并写命中 way
                        if (hit0) dat0[{idx,word}] <= bytemux(hit_old, wd1, ws1);
                        else      dat1[{idx,word}] <= bytemux(hit_old, wd1, ws1);
                    end
                    acc <= 1'b0;
                end else if (acc) begin          // ---- miss: 起填充 ----
                    filling <= 1'b1;
                    fway    <= victim;
                    fc      <= 4'h0;
                end
            end else begin
                if (rv_q && (fc_q <= 4'h7)) begin // ---- 填充: 到达字入 way ----
                    if (fc_q == word_sel) tgt_word <= m_rdata;
                    if (fway) dat1[{idx,fc_q[2:0]}] <= m_rdata;
                    else      dat0[{idx,fc_q[2:0]}] <= m_rdata;
                    if (fill_done) begin
                        if (fway) begin tag1[idx] <= tag; v1[idx] <= 1'b1; end
                        else        begin tag0[idx] <= tag; v0[idx] <= 1'b1; end
                        filling <= 1'b0;
                        acc     <= 1'b1;
                        fill_reply_q<=1'b1;
                        fill_reply_data_q <= (word_sel==3'd7) ? m_rdata : tgt_word;
                    end
                end
                if (fc != 4'h8 && !fill_done) fc <= fc + 4'h1;
            end

            if (flush_req && !filling && !acc) begin
                v0 <= 64'h0; v1 <= 64'h0;
                flush_req <= 1'b0;
            end
        end
    end
endmodule

// Keep tag comparison ahead of the late three-bit arbitration choice.
module dcache_pre_tag_match /* synthesis syn_hier="hard" */(
    input wire [21:0] tag0,tag1,request_tag,
    input wire valid0,valid1,output wire hit0,hit1);
    assign hit0=valid0&&(tag0==request_tag);
    assign hit1=valid1&&(tag1==request_tag);
endmodule
