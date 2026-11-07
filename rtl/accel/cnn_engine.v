//=====================================================================
// cnn_engine.v : **数据通路穿衣** —— 把 M1/M2/M3/M4/M5/M7/M9 的模块按
//                 `cnn_seq` 的控制向量连成一条完整数据通路。
//
//   权重： cnn_wrom ──2304b──> cnn_pe_page.wld_data
//   激活： cnn_act(读) ──256b──> cnn_linebuf.newword
//          cnn_linebuf.win ──2304b──> cnn_dman ──> cnn_pe_page.dly
//   输出： cnn_pe_page.page_y ──> cnn_outacc ──> cnn_act(写)
//
// ★ 拍点对齐（三处，本质都是"同步读要提前一拍"）
//   ① 权重：`wrom.rd_en` 与 (og,ig) 变化**同拍**拉高 ⇒ `rd_data` 次拍有效
//      ⇒ `pe_page.wld_en` 由 `rd_en` 寄存一拍得到。
//   ② 行缓冲：`act_re/act_raddr` 与 `seq.lb_load_o` **同拍**发出
//      ⇒ `act_rdata` 次拍有效 ⇒ linebuf 的 `wld/waddr/ldst` 用**寄存一拍**的版本。
//   ③ `rot` 不寄存：linebuf 与 seq 的 `rot_p` 必须同拍前进。
//
// ★ 算子相关（全部由 `op_mode` / `mode_pw` 显式给，**不从别的字段推导**）
//     op_mode : 0=3×3 conv ｜ 1=dw ｜ 2=1×1 pw   （来自描述符 TYPE）
//     mode_pw : 1×1 ⇒ linebuf 退化为 1×1 窗口 + dman 关偏斜
//     grp     : 读哪 32 个输入通道 —— conv 取 0、dw 取 og、pw 取 ig
//     bank_mask_h : dw/pw 要读 4 个 bank（32 通道/拍）；3×3 只读 1 个（4 通道）
//   ⚠ conv 的 ic > 32 时需要**按 32 通道组多次装载**（每 ig 组一次），本版未实现；
//     本版限定 conv 的 ic ≤ 32（此时 ig 循环复用同一份行缓冲数据）。
//
// ★ 不用 `linebuf.clr`：串行调度里**每个窗口行都先装载后使用**，残留数据永不被读到
//   ⇒ 免掉整组清零（那会拒绝 BRAM 推断，见 M3 的门禁 WARN）。
//
// ★ ACT 地址（与 `gen/act_alloc.py` 同口径）
//     字地址 W → bank = W&3、bank_idx = W>>2（读 32 通道 = 4 个 bank 同一索引）
//     读：raddr = (in_base + pix*in_wpg + grp*4) >> 2
//     写：cnn_outacc 内部算 out_base + pix*out_wpg + og_off（og_off = dw?og*4:og）
//     ⇒ in_wpg / out_wpg 都必须是**补齐后的字步长**（4 的倍数），
//       由 `gen/act_alloc.py` 的 stride 给，`gen/build_desc.py` 保证一致。
//=====================================================================
`timescale 1ns/1ps

module cnn_engine #(
    parameter ICG       = 4,
    parameter OC_SLOT   = 8,
    parameter TAPS      = 9,
    parameter CH        = 32,
    parameter MAXW      = 128,
    parameter WAW       = 7,
    parameter YW        = 32,
    parameter ACC_AW    = 16,
    parameter WROM_AW   = 11,
    parameter ACC_DEPTH = 1024,
    parameter ACC_AW_C  = 10,
    parameter WROM_FILE = "rtl/accel/mem/cnn_wrom.mem",
    parameter SHC_FILE  = "rtl/accel/mem/cnn_shcrom.mem"
)(
    input  wire                  clk,
    input  wire                  rst_n,

    // ---- 描述符字段（由 cnn_desc_dec 或固件提供）----
    input  wire                  start,
    input  wire [1:0]            op_mode,       // 0=3×3 conv ｜ 1=dw ｜ 2=1×1 pw
    input  wire [7:0]            n_og,
    input  wire [7:0]            n_ig,
    input  wire [7:0]            ih,
    input  wire [7:0]            iw,
    input  wire [7:0]            oh,
    input  wire [7:0]            ow,
    input  wire                  stride2,
    input  wire                  dw_mode,       // = (op_mode == DW)
    input  wire                  mode_pw,       // = (op_mode == PW)
    input  wire                  lin_out,
    input  wire [13:0]           weight_base,
    input  wire [15:0]           in_base,
    input  wire [7:0]            in_wpg,        // ★ 输入图补齐后的字步长（4 的倍数）
    input  wire [15:0]           out_base,
    input  wire [7:0]            out_wpg,       // ★ 输出图补齐后的字步长（4 的倍数）
    input  wire [7:0]            sh_req,
    input  wire [7:0]            brom_base,     // ★ shc ROM 字基址（描述符 brom_base）

    output wire                  busy,
    output wire                  layer_done,

    // ---- ACT 外口 ----
    output wire [3:0]            act_we,
    output wire [15:0]            act_waddr,     // ★ 固定 16 位（与 act_raddr 同理：
                                        //   ACT 字地址 ≥1024 时 bank 索引 ≥256，
                                        //   按 ACC_AW=8 收窄会回绕 —— 踩过）
    output wire [15:0]            act_raddr,     // ★ 固定 16 位（ACT 字地址 >>2）
    output wire [CH*8-1:0]       act_wdata,
    output wire                  act_re,
    output wire [3:0]            act_bank_mask, // ★ conv 单 bank 读；dw/pw 全 bank
    input  wire [CH*8-1:0]       act_rdata
);

    localparam OP_3X3 = 2'd0, OP_DW = 2'd1, OP_PW = 2'd2;
    // 输入通道组号（决定 ACT 读的**字偏移**）：
    //   dw ：og 组 = 32 通道 = 4 字 ⇒ 偏移 og*4
    //   pw ：ig 组 = 32 通道 = 4 字 ⇒ 偏移 ig*4
    //   conv：每拍 4 通道 = **半字**；通道 4ig 在字 (4ig)>>3 = ig>>1 ⇒ 偏移 ig>>1
    //         （低/高半字由 ig&1 在装载重排时选，见 lb_neww）
    wire [7:0] grp = dw_mode ? (og_c << 2)
                   : mode_pw ? (ig_c << 2)
                   :           {6'd0, ig_c[7:1]};
    // ★ **三种算子都 skew_en=1**。推导（pe 链 tap t 在 T-9+t 拍被消费）：
    //     y_out(T) = Σ_t dly[t](T-9+t)·w[t]
    //   skew 让 dly[t](τ)=win[t](τ-t) ⇒ tap t 在 T-9+t 拍读到的正是
    //   **T-9 拍送入的那个窗口**的第 t 个 tap —— 与 tap 维是空间偏移
    //   (3×3/dw) 还是输入通道 (pw) **无关**，偏斜的只是时间轴。
    //   skew_en=0 时 tap t 读到 T-9+t 拍的 win ⇒ 9 个 tap 混 9 个像素（错）。
    //   （"1×1 必须 skew_en=0"是早期错误结论，见 2026-10-17 日志。）

    // ---------------------------------------------------------------
    // 调度器
    // ---------------------------------------------------------------
    wire [7:0]  og_c, ig_c, row_c, col_c, ld_row_c;
    wire [15:0] pix_c;                 // ★ seq.pix_o 是 16 位像素号（8 位会让
                                       //   ctrl_pipe 的拼接宽度错位 ⇒ o_vld 永远 0）
    wire        ig_first_c, ig_last_c, seq_lb_load, seq_lb_rot, seq_row_vld, ld_k_c;
    wire [1:0]  seq_lb_ldst;
    wire [7:0]  seq_lb_waddr;
    wire [13:0] waddr_c;

    cnn_seq #(.ROW_W(8), .COL_W(8)) u_seq (
        .clk(clk), .rst_n(rst_n), .start(start),
        .n_og(n_og), .n_ig(n_ig), .ih(ih), .iw(iw), .oh(oh), .ow(ow),
        .stride2(stride2), .weight_base(weight_base),
        .busy(busy), .layer_done(layer_done),
        .og_o(og_c), .ig_o(ig_c), .ig_first_o(ig_first_c), .ig_last_o(ig_last_c),
        .row_o(row_c), .row_vld_o(seq_row_vld), .col_o(col_c), .pix_o(pix_c),
        .lb_load_o(seq_lb_load), .lb_ldst_o(seq_lb_ldst),
        .lb_waddr_o(seq_lb_waddr), .lb_rot_o(seq_lb_rot),
        .waddr_o(waddr_c), .ld_row_o(ld_row_c), .ld_k_o(ld_k_c)
    );

    // ---------------------------------------------------------------
    // 权重 ROM：与 (og,ig) 变化同拍发读 ⇒ 次拍 rd_data 有效
    // ---------------------------------------------------------------
    reg [7:0] og_r, ig_r;
    reg       wr_rd, wld_en_r;
    // ★ `start` 那拍 og=ig=0，而 og_r/ig_r 也被复位成 0 ⇒ **比较不出变化**
    //   ⇒ 权重永远不会被装载（整条通路是 X）。必须把 start 也算作一次"变化"。
    wire      ogig_chg = (busy & ((og_c != og_r) || (ig_c != ig_r)));

    // ★ 权重装载**延迟 4 拍**触发（含 wrom 读）：S_NEXT 换 (og,ig) 后，
    //   上一迭代最后像素的 tap k 要到 RUN 结束后第 k 拍才乘完（tap0..8
    //   跨 9 拍）。seq 已插 S_DRAIN(9 拍)；这里把装载再推 4 拍：
    //     DRAIN 首拍(+3) 变化 → rd_en(+7) → 数据(+8) → wld_en_r(+9)
    //     → wreg 于 +10 拍更新：晚于在途像素乘完(+7)、早于新迭代首拍
    //     (+13+iw) ⇒ 两边都安全。start 走旁路（首迭代有 2*iw 拍预装载余量）。
    reg  [5:0] ogig_sr;
    always @(posedge clk) begin
        if (!rst_n) ogig_sr <= 6'd0;
        else        ogig_sr <= {ogig_sr[4:0], ogig_chg};
    end
    // ★ `start` 必须**打一拍**再参与：seq 的 `wb`（weight_base）是在 start 那拍的
    //   边沿才锁存的 ⇒ 同一拍发 rd_en 会用**旧的 wb=0**，读到 ROM 字 0
    //   （= det 段的权重）。合成测试的 weight_base=0 恰好掩盖了这个 bug。
    reg       start_d;
    always @(posedge clk) begin
        if (!rst_n) start_d <= 1'b0;
        else        start_d <= start;
    end
    wire      ogig_new = start_d | ogig_sr[5];

    // ⚠ 这里**不能**在 start 拍把 wr_rd/wld_en_r 一并清掉 —— 那会把
    //   `ogig_new`（含 start 拉高）产生的装载脉冲整掉，wreg 永远是 X。
    //   og_r/ig_r 也不必在 start 复位：跨层时它们停在上层的末值，而 og_c 从 0 起
    //   ⇒ 自然不相等 ⇒ 自动触发一次装载。
    // ★ run16：wrom 内部地址寄存一拍 ⇒ 读延迟 1→2 拍 ⇒ 装载脉冲再推 1 拍。
    //   （还顺便给数据留出 1 拍余量：数据在 ogig_new+2 有效，装载在 +3。）
    reg       wr_rd2, wld_pre;
    always @(posedge clk) begin
        if (!rst_n) begin
            og_r <= 8'd0; ig_r <= 8'd0; wr_rd <= 1'b0; wr_rd2 <= 1'b0; wld_pre <= 1'b0; wld_en_r <= 1'b0;
        end else begin
            wr_rd    <= ogig_new;
            wr_rd2   <= wr_rd;
            wld_pre  <= wr_rd2;
            wld_en_r <= wld_pre;
            og_r     <= og_c;
            ig_r     <= ig_c;
        end
    end

    wire [5:0] og_s1_c;                    // outacc 管线级 1 的 og
    wire [5:0] og_s0_c;                    // outacc 管线级 0 的 og（shc ROM 地址用）
    // ★★ 2026-10-23（run18）：权重 ROM 的 rd_en/rd_addr **在引擎侧寄存**。
    //   run17 头号路径 = `u_seq` 地址乘法 APM → `u_wrom/mem_*/ADA0`：
    //   **0 级逻辑，走线 5.98 ns**（扇出到 93 个 DRAM，物理跨度大）。
    //   ⚠ 不能放在 cnn_wrom 内部：DRAM 自带地址寄存器，会被吸收（run16 已证）。
    //   代价：读延迟 +1 拍；wld_en 链本已是 3 级（wr_rd/wr_rd2/wld_en_r）
    //   ⇒ 数据在 +2 有效、装载在 +3，仍有 1 拍余量。
    reg        ogig_d1;
    reg [13:0] waddr_d1;
    // Keep this additional edge in fabric: the first address edge may be
    // packed into the address-generating DSP output register.
    (* syn_preserve=1 *) reg [13:0] waddr_d2;
    reg ogig_d2;
    always @(posedge clk)begin
        if(!rst_n)begin waddr_d2<=0;ogig_d2<=0;end
        else begin waddr_d2<=waddr_d1;ogig_d2<=ogig_d1;end
    end
    always @(posedge clk) begin
        if (!rst_n) begin
            ogig_d1  <= 1'b0;
            waddr_d1 <= 14'd0;
        end else begin
            ogig_d1  <= ogig_new;
            waddr_d1 <= waddr_c;
        end
    end

    wire [2303:0] wdata_r;
    cnn_wrom #(.INIT_FILE(WROM_FILE)) u_wrom (
        .clk(clk), .rst_n(rst_n),
        .rd_en(ogig_d2), .rd_addr(waddr_d2[WROM_AW-1:0]), .rd_data(wdata_r)
    );

    // The existing load-enable chain samples weights four edges after
    // ogig_new. Address d2 -> ROM -> fabric data now consumes three of
    // these edges; load timing and group-tail protection remain unchanged.
    (* syn_preserve=1 *) reg [2303:0] wdata_pipe_q;
    always @(posedge clk)wdata_pipe_q<=wdata_r;

    // ---------------------------------------------------------------
    // shc ROM：地址用 **og_s1**（outacc 管线级 1 的 og）同拍发读 ⇒
    //   q 在 og_s2 拍有效 = r3 锁存拍（bnr = r2 + shc）。每拍都在读，
    //   og 推进时地址自动跟上，无需预取/装载管理。
    //   conv/pw：字偏移 og>>2（一字 4 组）；dw：字偏移 og（一字 1 组）。
    // ---------------------------------------------------------------
    wire [255:0] shc_q;
    // 2026-10-30：地址改用管线级 0，ROM q 提前一拍到达；
    // 再在引擎侧打一拍送给 outacc，使 r3 锁存拍不变。
    wire [7:0]   shc_addr = brom_base + (dw_mode ? og_s0_c : {4'd0, og_s0_c[5:2]});
    cnn_shcrom #(.INIT_FILE(SHC_FILE)) u_shcrom (
        .clk(clk), .rst_n(rst_n),
        .rd_en(busy), .rd_addr(shc_addr), .rd_data(shc_q)
    );
    // 2026-10-30 (shcpipe)：指令路径已提前一拍，
    //   256 b 读数据在引擎侧再寄存一拍（同 weightpipe 的 wdata_pipe_q 做法），
    //   把 DRAM tco 1.644 ns + 净 1.052 ns 从 r3 关键路径上摎掉。
    //   busy 与 outacc 的 en 同源，保证与 pix_pipe 同步推进。
    (* syn_preserve=1 *) reg [255:0] shc_q_r;
    always @(posedge clk) if (busy) shc_q_r <= shc_q;

    // ---------------------------------------------------------------
    // ACT 读：与 seq 的装载同拍发地址 ⇒ rdata 次拍有效
    // ---------------------------------------------------------------
    // ★ ACT 读的是**输入行 ld_row 的第 lb_waddr 列**那个像素
    //   （lb_waddr 是列号，不是像素号 —— 漏掉行号会整行读错）
    // ★★ 2026-10-21：in_off 必须 **17 位**。det 真实尺寸的 ACT 占用到 flat 73,728
    //   （输入物化区 [0,49152) + 特征图峰值 [49152,73728)）⇒ 超过 16 位字地址空间
    //   （65,536）⇒ 层 2（B1p，in_base=61440）起读输入整体回绕到字 0（det 输入区），
    //   数值全错但不报错。写路径（outacc 的 awaddr_c 是 18 位）本就是对的。
    //   ⇒ det 全尺寸此前从未验证过（tb_realnet 的 det PASS 是 ÷4 量级， footprint 不超限）。
    // ★★ 2026-10-23：地址预计算（拆串联乘法）
    //   原式 `in_base + (ld_row_c*iw + waddr)*in_wpg + grp` 是**两级乘法串联**，
    //   PDS 布局布线后实测这条路径 **14.96 ns**（5 级逻辑，logic 6.56 / route 4.66）
    //   是全加速器头号瓶颈。
    //   `iw * in_wpg` 是**每层常数**（描述符给出，整层不变）⇒ 打一拍预计算，
    //   每拍只剩两个**并行**乘法（8×16 与 8×8，各 1 个 APM）+ 三输入加法。
    //   ⚠ iw/in_wpg 只在层间变化，而引擎的首次 ACT 读在 start 后若干拍
    //     （WARM 等）才发出 ⇒ 1 拍延迟安全（已用 realnet 回归验证）。
    reg [15:0] iw_wpg_r;
    always @(posedge clk) begin
        if (!rst_n) iw_wpg_r <= 16'd0;
        else        iw_wpg_r <= iw * in_wpg;
    end

    // ★★ 2026-10-23（run10）：ACT 读地址**加一级流水**
    //   run8 实测这条路径 9.73 ns（logic 5.09 / route 4.36），是唯一的瓶颈；
    //   地址算完直接驱动 ACT 的 64 个 DRAM 实例（扇出大、走线长）。
    //   ⇒ 在引擎内**寄存一拍**，路径拆成「乘加链 → 寄存器」与「寄存器 → DRAM 地址」两段。
    //
    //   代价（已在 seq 里补偿）：act_rdata 从「请求后第 1 拍」变成**第 2 拍**，
    //   所以：
    //     ① linebuf 写控制（wld/ldst/waddr/rot）从延迟 1 拍改为**延迟 2 拍**（本处两级寄存）；
    //     ② `cnn_seq` 的 `warm_cyc` 对**非 rot 入口** +1（最后一笔载荷晚 1 拍落盘）。
    //     ③ `lb_neww` 的 bank/半字选择子也随之寄存（conv 重排用）。
    //
    //   ⚠ 不能只寄存 `ld_row*iw_wpg_r`：ld_row 变化恰好在该行首笔载荷那拍
    //     ⇒ 首笔拿到上一行的偏移（实测整网 699 项失败）。
    //-----------------------------------------------------------------
    // 级 1：地址 + 选择子寄存
    reg [16:0] in_off_r;
    reg [1:0]  conv_bank_r;
    reg        conv_half_r;
    reg        s1_load;
    reg [1:0]  s1_ldst;
    reg [7:0]  s1_waddr;

    // Stage 0 snapshots a complete request. Products are reduced modulo
    // 2^17 exactly as the original final 17-bit address expression.
    // Stage 1 adds only registered terms; no multiplier cascade remains.
    // Snapshot request selection before the multiplier input setup path.
    (* syn_preserve=1 *) reg [7:0] request_row_q;
    reg [7:0] request_wpg_q, request_group_q;
    (* syn_preserve=1 *) reg [7:0] request_column_q;
    reg [15:0] request_stride_q, request_base_q;
    reg request_load_q, request_half_q;
    reg [1:0] request_ldst_q;
    always @(posedge clk) begin
        if(!rst_n) begin
            request_row_q<=0;request_column_q<=0;request_wpg_q<=0;request_group_q<=0;
            request_stride_q<=0;request_base_q<=0;request_load_q<=0;
            request_half_q<=0;request_ldst_q<=0;
        end else begin
            request_row_q<=ld_row_c;request_column_q<=seq_lb_waddr;
            request_wpg_q<=in_wpg;request_group_q<=grp;
            request_stride_q<=iw_wpg_r;request_base_q<=in_base;
            request_load_q<=busy & seq_lb_load;request_half_q<=ig_c[0];
            request_ldst_q<=seq_lb_ldst;
        end
    end
    reg [16:0] row_product_q, column_product_q, base_group_q;
    reg s0_load, s0_half;
    reg [1:0] s0_ldst;
    reg [7:0] s0_waddr;
    always @(posedge clk) begin
        if(!rst_n) begin
            row_product_q<=0;column_product_q<=0;base_group_q<=0;
            s0_load<=0;s0_half<=0;s0_ldst<=0;s0_waddr<=0;
        end else begin
            row_product_q<=request_row_q * request_stride_q;
            column_product_q<=request_column_q * request_wpg_q;
            base_group_q<={1'b0,request_base_q}+request_group_q;
            s0_load<=request_load_q;s0_half<=request_half_q;
            s0_ldst<=request_ldst_q;s0_waddr<=request_column_q;
        end
    end
    wire [16:0] in_off = row_product_q + column_product_q + base_group_q;

    always @(posedge clk) begin
        if (!rst_n) begin
            in_off_r    <= 17'd0;
            conv_bank_r <= 2'd0;
            conv_half_r <= 1'b0;
            s1_load     <= 1'b0;
            s1_ldst     <= 2'd0;
            s1_waddr    <= 8'd0;
        end else begin
            in_off_r    <= in_off;
            conv_bank_r <= in_off[1:0];
            conv_half_r <= s0_half;
            s1_load     <= s0_load;
            s1_ldst     <= s0_ldst;
            s1_waddr    <= s0_waddr;
        end
    end

    assign act_re    = s1_load;
    assign act_raddr = in_off_r[16:2];         // >> 2 = bank 索引（≤18,431 ⇒ 15 位）
    // ★ conv 只读 1 个 bank（目标字的 64b）；dw/pw 读 4 个 bank（32 通道）
    assign act_bank_mask = (op_mode == OP_3X3) ? (4'b0001 << in_off_r[1:0])
                                               : 4'b1111;

    // 级 2：linebuf 写控制（与 act_rdata 同拍）
    // ★★ v2.0（2026-10-25）：**rot 不再延迟**（与文件头 ③ 的原始约定一致）
    //   run10 为了把「写控制与 act_rdata 对齐」把 rot 也一起延了 2 拍，
    //   结果 linebuf 的 rot_p 比 seq 晚 2 拍。旧 FSM 里 S_WARM 有 8 拍，
    //   把这段滞后完全盖住了，所以一直没暴露。
    //   去掉 S_WARM 后，RUN 第 0/1 拍读到的还是**上一行**的窗口 ⇒ 每行前 2 列错
    //   （实测 det ÷4 层1：像素 16/17 = 行1 列0/1 错，正好 2 列）。
    //   ⇒ rot 必须**同拍**进 linebuf（rot 不参与 ACT 读流水，只推进 rot_p）。
    reg        lb_load_d;
    reg [1:0]  lb_ldst_d;
    reg [7:0]  lb_waddr_d;
    always @(posedge clk) begin
        if (!rst_n) begin
            lb_load_d <= 1'b0; lb_ldst_d <= 2'd0; lb_waddr_d <= 8'd0;
        end else begin
            lb_load_d <= s1_load;
            lb_ldst_d <= s1_ldst;
            lb_waddr_d<= s1_waddr;
        end
    end

    // ---------------------------------------------------------------
    // Historical alignment derivation below; current control has 10 stages.
    // MAC entry and bank-read stages are reflected in NSTG_CTRL.
    //win(W) 的点积在 page_y 上于 W+9 拍出现，
    //   但 outacc 的 r2 **每拍无条件采样 page_y**（不门控 vld）—— 它在
    //   page_vld(P) 后第 1 拍（P+1）才真正锁存 page_y(P+1)。
    //   ⇒ 控制要在 P+1 拍对上 page_y(P+1)=win(P+1-9) ⇒ page_vld 须在
    //     W+8 拍发（= 控制延迟 8），否则整图**后移一个像素**（实测踩过：
    //     9 级时像素 k 的值写进 k+1 的地址）。
    //   ⚠ 每个 busy 拍都要移位（dman/PE 链在 LOAD/DRAIN 期也在推进）。
    // ---------------------------------------------------------------
    localparam PPW = 1 + 16 + 8 + 1 + 1 + 8;  // {vld, pix_out, col, igf, igl, og}
    localparam NSTG_CTRL = 13;
    reg [NSTG_CTRL*PPW-1:0] ctrl_pipe;
    always @(posedge clk) begin
        if (!rst_n)
            ctrl_pipe <= {NSTG_CTRL*PPW{1'b0}};
        else if (busy)
            ctrl_pipe <= {ctrl_pipe[(NSTG_CTRL-1)*PPW-1:0],
                          seq_row_vld, pix_c, col_c, ig_first_c, ig_last_c, og_c};
    end
    wire               o_vld   = ctrl_pipe[NSTG_CTRL*PPW-1];
    wire [15:0]        o_pix   = ctrl_pipe[NSTG_CTRL*PPW-2 -: 16];
    wire [7:0]         o_col   = ctrl_pipe[NSTG_CTRL*PPW-18 -: 8];
    wire               o_igf   = ctrl_pipe[NSTG_CTRL*PPW-26];
    wire               o_igl   = ctrl_pipe[NSTG_CTRL*PPW-27];
    wire [7:0]         o_og    = ctrl_pipe[NSTG_CTRL*PPW-35 +: 8];   // ★ og 在 stage 的**低 8 位**

    // ---------------------------------------------------------------
    // 行缓冲 / 偏斜 / PE 页 / 输出累加
    // ---------------------------------------------------------------
    wire [TAPS*CH*8-1:0] win, dly;
    // ★ conv 装载重排：ACT 读出的 256b 里，本 ig 组的 4 通道在
    //   bank (in_off&3) 的半字 (ig&1) ⇒ 挪到低 4 字节（lane ch=p 取 0..3），
    //   其余清零。dw/pw 不重排（32 通道全用）。
    // ★ 选择子已随地址一起寄存（级 1）⇒ 与 act_rdata（级 2 有效）同拍
    // Split the measured ACT DRM -> channel mux -> distributed RAM path.
    // Raw data and all write metadata advance together; rot stays immediate.
    reg [255:0] lb_raw_q;
    reg lb_load_q,lb_half_q;
    reg [1:0] lb_ldst_q,lb_bank_q,lb_mode_q;
    reg [7:0] lb_waddr_q;
    always @(posedge clk)lb_raw_q<=act_rdata;
    always @(posedge clk)begin
        if(!rst_n)begin
            lb_load_q<=0;lb_half_q<=0;lb_ldst_q<=0;lb_bank_q<=0;
            lb_mode_q<=0;lb_waddr_q<=0;
        end else begin
            lb_load_q<=lb_load_d;lb_ldst_q<=lb_ldst_d;lb_waddr_q<=lb_waddr_d;
            lb_bank_q<=conv_bank_r;lb_half_q<=conv_half_r;lb_mode_q<=op_mode;
        end
    end
    wire [255:0] lb_neww = (lb_mode_q == OP_3X3) ?
        {{192{1'b0}}, lb_raw_q[64*lb_bank_q + 32*lb_half_q +: 32]}
        : lb_raw_q;

    cnn_linebuf #(.CH(CH), .AW(8), .MAXW(MAXW), .TAPS(TAPS), .WAW(WAW)) u_lb (
        .clk(clk), .rst_n(rst_n), .clr(1'b0),
        // ★ win_top_zero 只在**输出行 0** 时生效（窗口顶行 = 输入行 -1，越界补零）
        //   恒接 1 会把所有输出行的顶行都清零 ⇒ 3×3 只剩 2 行参与卷积，全层数值错
        .win_top_zero(row_c == 8'd0),
        // ★ 底行越界 ⇒ 掩零。窗口底行对应的**输入行**：
        //     stride-1 ⇒ r+1；stride-2 ⇒ 2r+1（3×3 窗口跨 2r-1/2r/2r+1）。
        //   ⚠ 原条件写死 `!stride2`，注释称"stride-2 底行 = 2r+1 ≤ ih-1 恒存在"
        //   —— **错的**：ih 为奇数（或 ih<2）时末输出行的 2r+1 == ih（不存在）。
        //   det 层26 E1d 真实 **4x3 → 2x2 s2（ih=3）**必踩：底行补零被漏掉
        //   ⇒ 该输出行少了一行贡献（表现为部分通道偏小）；ih=1 时更会读到
        //   ACT 越界（X/垃圾）并在 288 PE 网络里传播。
        .win_bot_zero((stride2 ? ({1'b0, row_c} << 1) : {1'b0, row_c}) + 9'd1
                      >= {1'b0, ih}),
        .mode_pw(mode_pw),            // 1×1 ⇒ 窗口退化为 1×1（9 tap 同一列字）
        .rot_step2(stride2),
        .wld(lb_load_q), .ldst(lb_ldst_q), .waddr(lb_waddr_q),
        .newword(lb_neww),
        // ★ stride-2 的**列**也要隔列采样：窗口中心列 = 2*col（行方向由 rot_step2 处理）
        //   漏了这条 ⇒ 输出列 c 拿到的是输入列 c 的窗口（只隔了行、没隔列）
        .rot(seq_lb_rot), .iw(iw),
        .c(stride2 ? {col_c[6:0], 1'b0} : col_c), .rot_p_o(), .win(win)
    );

    // Data-only skew/MAC pipelines advance during idle as well. Their idle
    // contents are never committed: ctrl_pipe validity and outacc remain busy
    // gated, and valid windows traverse the full pipeline before consumption.
    // This removes busy from more than 9000 data-register enable pins.
    cnn_dman #(.TAPS(TAPS), .CH(CH), .AW(8)) u_dman (
        .clk(clk), .en(1'b1),
        .skew_en(1'b1),              // ★ 三算子都要偏斜（判别实验已证：pw 旁路更差，头注释那句是旧误结论）
        .win(win), .dly(dly)
    );

    wire [ICG*OC_SLOT*YW-1:0] page_y;
    cnn_pe_page #(.ICG(ICG), .OC_SLOT(OC_SLOT), .TAPS(TAPS), .CH(CH),
                  .AW(8), .WW(8), .YW(YW)) u_page (
        .clk(clk), .en(1'b1), .op_mode(op_mode), .dly(dly),
        .wld_en(wld_en_r), .wld_data(wdata_pipe_q), .page_y(page_y)
    );

    cnn_outacc #(.ICG(ICG), .OC_SLOT(OC_SLOT), .YW(YW),
                 .ACC_DEPTH(ACC_DEPTH), .ACC_AW(ACC_AW_C)) u_acc (
        .clk(clk), .rst_n(rst_n), .en(busy),
        // ★ 控制向量已经过 9 级管线，与 page_y 严格对齐
        .page_y(page_y), .page_vld(o_vld),
        .pix_idx(o_pix),             // ★ acc RAM 索引 = **像素号**（不是列号！）
                                     //   多 ig 时行 0 col0 与行 7 col0 共用列槽
                                     //   ⇒ ig=1 读回别行的部分和（踩过）
        .pix_out(o_pix),             // ACT 写地址用 **像素号** row*ow+col
        .ig_first(o_igf), .ig_last(o_igl), .dw_mode(dw_mode),
        .lin_out(lin_out), .sh_req(sh_req), .shc_r(shc_q_r),
        .out_base(out_base), .out_wpg(out_wpg), .og(o_og),
        .act_we(act_we), .act_waddr(act_waddr), .act_wdata(act_wdata),
        .og_s0(og_s0_c), .og_s1(og_s1_c), .acc_idle()
    );

endmodule
