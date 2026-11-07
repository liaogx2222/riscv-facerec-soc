//=====================================================================
// cnn_seq.v : 引擎调度器 —— og/ig 双循环 + 行/列扫描 + 权重/行缓冲寻址
//
// 职责：把一层的描述符字段变成**逐拍的控制向量**（不含数据通路）。
//   数据通路交给各自验证过的模块：cnn_wrom(取权重) → cnn_pe_page(算) →
//   cnn_dman(偏斜) ← cnn_linebuf(窗口) → cnn_outacc(累加/requant) → ACT。
//
// ★ 输出是**组合的**（从 (st, og, ig, r, x, rot_p) 实时译出），与控制向量同拍有效
//   ⇒ 与 `gen/gen_seq_vec.py` 的逐拍模型一 一对应、可对拍。
//   （寄存器式输出会引入"控制晚一拍"的歧义，本工程 TB/模型都会被坑过。）
//
// ★ 迭代边界（与 `cnn_desc_dec.v` 同口径）
//     n_og = ceil(oc/8)【dw 用 ceil(oc/32)】  n_ig = ceil(ic/4)【dw=1 / pw 用 ceil(ic/32)】
//     权重字地址 waddr = weight_base + og*n_ig + ig
//     ⇒ 与 `gen/gen_wrom_mem.py` 的打包顺序（for og: for ig:）严格一致
//
// ★ 扫描时序（**串行装载版**）
//     og → ig → row →
//       预装载 2 行（row0→slot1、row1→slot2；此时 rot_p=0，窗口 tr=1/2 正是它们）
//       第 r 行：r≥2 且 r+1<ih ⇒ LOAD 输入行 (r+1) 到槽 ((r+2)&3)，iw 拍
//                RUN ow+1 拍（前 ow 拍有效，最后 1 拍行冲刷 row_vld=0）
//                rot（stride-2 时 rot_p += 2）
//     ⇒ 串行总拍 = 2*iw(预装) + Σ_row [iw·need_load + ow+1]
//       CIFAR 的"装载/计算重叠"是 -47% 的优化，**本版故意先串行**（可验证），
//       重叠版作为下一步（那时行装载要与 RUN 并发，需重排态）。
//
// ★ 环槽语义（与 `cnn_linebuf.v` 一致）：窗口 tr 行 = row_buf[(rot_p+tr)&3]，
//   输出行 r 的窗口 = 输入行 (r-1, r, r+1)；r==0 时 tr=0 行由 win_top_zero 置零。
//=====================================================================
`timescale 1ns/1ps

module cnn_seq #(
    parameter ROW_W = 8,
    parameter COL_W = 8,
    // ★★ v2.0（2026-10-25）：S_WARM 从 8/9 拍压到 **3 拍**（`WARM_N=2`）
    //   ⚠ 口径：退出条件是 `wm_cnt == warm_cyc` ⇒ **实占 warm_cyc+1 拍**
    //
    //   旧结论「S_WARM 是偏斜链固有开销、不可压」经推导是错的：
    //     y_out(T) = Σ_t dly[t](T-9+t)·w[t]，dly[t](τ)=win[t](τ-t)
    //              = Σ_t win[t](T-9)·w[t]  ⇒ 只用到「窗口实例 T-9」
    //   窗口实例 < 0 的那些拍（T < 9）对应控制向量 v_row_vld=0，
    //   outacc 在 o_vld=0 时既不写 acc_mem 也不写 ACT ⇒ **整段丢弃**。
    //   而窗口实例 0 在 RUN 第 0 拍送入链首（tap0 直连、tap t 在 +t 拍取），
    //   每拍 tap t 都精确取到 win(该实例)，与「链里之前装了什么」无关
    //   ⇒ 预热对**被采样的**拍没有作用，只是把链里的旧字冲掉。
    //
    //   真正需要的那几拍是**流水落稳**：
    //     · ACT 读 → linebuf 写 有 2 拍延迟（s1_* → lb_*_d）
    //     · 小 iw（÷4 的 4x3 层）时 S_PRE 只有 8 拍，最后一列的写刚好压着 RUN 首拍
    //   实测（det ÷4 前 9 层，含 4x3/2x1 极小层）：
    //     warm_cyc=0（1 拍）FAIL ｜ warm_cyc=1（2 拍）PASS ｜ warm_cyc=2（3 拍）PASS
    //   ⇒ 取 WARM_N=2（3 拍，留 1 拍余量）。收益 ≈ 5~7 拍/行迭代。
    //
    //   WARM_N=0 时直接把所有「进 S_WARM」改成「进 S_RUN」（回退/实验用）。
    parameter integer WARM_N = 7
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  start,        // 描述符已解码：开始跑这一层
    input  wire [7:0]            n_og,
    input  wire [7:0]            n_ig,
    input  wire [7:0]            ih,           // 输入行数
    input  wire [7:0]            iw,           // 输入列数
    input  wire [7:0]            oh,           // 输出行数
    input  wire [7:0]            ow,           // 输出列数
    input  wire                  stride2,
    input  wire [13:0]           weight_base,

    output reg                   busy,          // 本层未跑完
    output reg                   layer_done,    // 本层跑完（单拍脉冲）

    // ---- 逐拍控制向量（组合）----
    output wire [7:0]            og_o,
    output wire [7:0]            ig_o,
    output wire                  ig_first_o,
    output wire                  ig_last_o,
    output wire [ROW_W-1:0]      row_o,
    output wire                  row_vld_o,
    output wire [COL_W-1:0]      col_o,
    output wire [15:0]           pix_o,    // ★ 输出**像素号** = row*ow+col（ACT 写地址用）
    output wire                  lb_load_o,
    output wire [1:0]            lb_ldst_o,
    output wire [7:0]            lb_waddr_o,
    output wire                  lb_rot_o,
    output wire [13:0]           waddr_o,
    output wire [7:0]            ld_row_o,    // 本拍装载的是哪个**输入行**（stride-2 有两行）
    output wire                  ld_k_o       // 本行第几次装载
);

    localparam S_IDLE = 3'd0, S_PRE = 3'd1, S_LOAD = 3'd2,
               S_RUN  = 3'd3, S_ROT = 3'd4, S_NEXT = 3'd5, S_DRAIN = 3'd6,
               S_WARM = 3'd7;
    // ★ SKIP_WARM=1 时所有「进 S_WARM」改「直接进 S_RUN」（见模块头参数说明）
    localparam S_WU = (WARM_N == 0) ? S_RUN : S_WARM;
    localparam DRAIN_CYC = 18;      // ★ PE 链 9 拍：换权重前必须排空（见下）
    // ★ 启动偏斜期：dman 的 9 级偏斜链要 9 拍才能让 dly[0..8] 全部落到
    //   **同一个已装载 slot**。小图（iw<9）装载拍数不足 ⇒ tap0..3 还是 X
    //   ⇒ sum X ⇒ 整层错（真实层 rec 的 2×2 段必踩；8×8 恰好够所以一直掩盖）。
    //   ⇒ 每次装载段结束后插 WARM_CYC 拍：不出像素、不装载、col 保持 0
    //   （= RUN 首列）⇒ linebuf 输出稳定 ⇒ 偏斜链填满同 slot 的字。
    // ★ 旧推导（8/9 拍）已作废，见参数区 WARM_N 的说明。
    wire [3:0] warm_cyc = WARM_N[3:0];   // ★ v2.0：统一 warm_cyc=2（S_WARM 实占 3 拍；原来 7/9）
    reg        warm_from_rot;                    // 本次 WARM 是否由 S_ROT 进入（stride-1）
    reg [2:0] st;
    // ow is a layer constant. Advance the row base once at S_ROT,
    // replacing the RUN-time r*ow multiplier with a 16-bit increment.
    reg [15:0] pixel_row_base;
    always @(posedge clk) begin
        if(!rst_n) pixel_row_base<=0;
        else if(st==S_IDLE || st==S_PRE || st==S_NEXT) pixel_row_base<=0;
        else if(st==S_ROT) pixel_row_base<=pixel_row_base+{8'b0,ow};
    end

    reg [4:0] dr_cnt;
    reg       drain_last;    // 本次 DRAIN 是层末排空（结束前最后一次）
    reg [3:0] wm_cnt;        // 启动偏斜期计数
    // ★ 装载/计算重叠：DRAIN 期为**下一迭代**预装载 2 行的进度
    reg       dr_last;       // DRAIN 最后一拍（linebuf 锁存中，不再发装载）
    reg [7:0] dbeat_c;       // 预装载列号 0..iw-1
    reg       dbeat_r;       // 预装载行号 0/1
    reg       pre_full;      // 本迭代的 2 行预装载已在 DRAIN 里全部完成
`ifdef SEQ_IT
    // 诊断：每次进入预装载段（= 一个 (og,ig) 迭代的开始）打一行
    always @(posedge clk) if (rst_n && st == S_PRE && x == 8'd0 && pre_cnt == 8'd0)
        $display("  [IT] og=%0d ig=%0d rot_p=%0d n_og=%0d n_ig=%0d", og, ig, rot_p, n_og, n_ig);
`endif

`ifdef SEQ_STAT
    // ★ 每态拍数统计（只在 SEQ_STAT 下编译；用于"先测再优"，不参与综合）
    //   用法：iverilog -DSEQ_STAT ... ⇒ 每层结束时打印**本层**增量 + 累计
    integer c_pre, c_load, c_warm, c_run, c_rot, c_drain, c_next, rows, iters;
    integer p_pre, p_load, p_warm, p_run, p_rot, p_drain, p_next, p_rows, p_iters;
    initial begin
        c_pre=0; c_load=0; c_warm=0; c_run=0; c_rot=0; c_drain=0; c_next=0;
        rows=0; iters=0;
        p_pre=0; p_load=0; p_warm=0; p_run=0; p_rot=0; p_drain=0; p_next=0;
        p_rows=0; p_iters=0;
    end
    always @(posedge clk) if (rst_n && busy) begin
        case (st)
        S_PRE   : c_pre   = c_pre   + 1;
        S_LOAD  : c_load  = c_load  + 1;
        S_WARM  : c_warm  = c_warm  + 1;
        S_RUN   : c_run   = c_run   + 1;
        S_ROT   : begin c_rot = c_rot + 1; rows = rows + 1; end
        S_DRAIN : c_drain = c_drain + 1;
        S_NEXT  : begin c_next = c_next + 1; iters = iters + 1; end
        default : ;
        endcase
    end
    always @(posedge clk) if (rst_n && layer_done) begin
        $display("  [STAT] L%0d: PRE=%0d LOAD=%0d WARM=%0d RUN=%0d ROT=%0d DRAIN=%0d NEXT=%0d rows=%0d it=%0d | SUM=%0d",
                 og, c_pre-p_pre, c_load-p_load, c_warm-p_warm, c_run-p_run,
                 c_rot-p_rot, c_drain-p_drain, c_next-p_next,
                 rows-p_rows, iters-p_iters,
                 c_pre+c_load+c_warm+c_run+c_rot+c_drain+c_next);
        p_pre=c_pre; p_load=c_load; p_warm=c_warm; p_run=c_run;
        p_rot=c_rot; p_drain=c_drain; p_next=c_next;
        p_rows=rows; p_iters=iters;
    end
`endif

    reg [7:0]  og, ig;
    // Group counts are layer constants and og/ig have settled long before
    // S_NEXT. Remove increment/compare from the NEXT -> column-clear cone.
    (* syn_preserve=1 *) reg ig_more_q,og_more_q;
    always @(posedge clk)begin
        if(!rst_n)begin ig_more_q<=0;og_more_q<=0;end
        else begin ig_more_q<=(ig+8'd1<n_ig);og_more_q<=(og+8'd1<n_og);end
    end
`ifdef FACEREC_SIM
    always @(posedge clk)if(rst_n && st==S_NEXT)begin
        if(ig_more_q!==(ig+8'd1<n_ig) || og_more_q!==(og+8'd1<n_og))
            $fatal(1,"Group limit changed too close to NEXT");
    end
`endif

    reg [7:0]  r;                 // 输出行
    reg [7:0]  x;                 // 列 / 装载游标
    reg [7:0]  pre_cnt;           // 预装载进度（0,1 → 完成时 2）
    reg [1:0]  rot_p;
    reg [13:0] wb;                // 本层权重基址
    reg        ld_k;              // 本行第几次装载（stride-2 要装 2 行）

    // ---------------------------------------------------------------
    // 组合控制向量
    // ---------------------------------------------------------------
    // ★ 本行要装载几次：stride-1 装 1 行（输入行 r+1）；**stride-2 装 2 行**
    //   （输入行 2r / 2r+1 —— 相邻输出行的 3×3 窗口在 stride-2 下有 1 行重叠）
    wire       ld_total = stride2;                 // 0 ⇒ 1 次；1 ⇒ 2 次
    // ★ ld_row 在**预装载段**（r==0）必须是 pre_cnt（0 或 1）
    //   —— 预装载装的是输入行 0 和 1，不是 r+1
    // ★ ovl_pre 时 ACT 读行 = 预装载的行号（0/1），不能用 r 推。
    //   pre_full = 两行都已在 DRAIN 里装完（置位后本迭代不再发装载）
    wire       ovl_pre  = (st == S_DRAIN) && !drain_last && !dr_last && !pre_full;
    // ★★★ 「装载/计算重叠」之二：**stride-1 把下一行的装载整体并入 RUN**。
    //   依据（已核）：RUN 期窗口读 slot (rot_p+0/1/2)（pw 只读 +1），
    //   而下一行要写的是 slot (rot_p+3) ⇒ **恒为空闲槽**，写不影响本次计算。
    //   RUN 有 ow+1 拍、装载只需 iw 拍，stride-1 时 ow == iw ⇒ 恰好装得下。
    //   ⇒ S_LOAD 对 stride-1 可以整段省掉（ROT 后直接进 S_WARM）。
    //   ⚠ stride-2 不适用：ow = iw/2 ⇒ RUN 装不下 2 行，仍走 S_LOAD。
    wire       run_load_s1 = (st == S_RUN) && !stride2 && (r + 8'd2 < ih) && (x < iw);
    // ★★ opt2（2026-10-24）：stride-2 的**第一行装载并入 RUN**。
    //   依据：RUN 期窗口读 slot (rot_p+0/1/2)，空闲槽 = rot_p+3；
    //   而 S_LOAD 在 S_ROT 之后（rot_p 已 +2）读 ld_slot = (rot_p+2)+1+ld_k
    //   ⇒ **ld_k=0 的目标槽正好就是 RUN 期的空闲槽**。
    //   RUN 有 ow 拍、每拍装 1 列 ⇒ 先装前 ow 列，S_LOAD 从第 ow 列续。
    //   收益：每个 stride-2 输出行省 `ow` 拍 ≈ S_LOAD 的 1/4（约 10.7 K 拍/帧，4.4%）。
    //   ⚠ 末行不装（没有下一行）；ow ≥ iw 时不并入（否则 S_LOAD 起始列越界）。
    wire       run_load_s2 = (st == S_RUN) &&  stride2 && (r + 8'd1 < oh) && (ow < iw);
    wire       run_load    = run_load_s1 | run_load_s2;
    wire [7:0] ld_row   = run_load_s2 ? ((r << 1) + 8'd2)
                       : run_load_s1 ? (r + 8'd2)
                       : ovl_pre ? dbeat_r
                       : (r == 8'd0) ? pre_cnt
                       : (stride2 ? ((r << 1) + {7'd0, ld_k}) : (r + 8'd1));
    wire [1:0] ld_slot  = stride2 ? (rot_p + 2'd1 + {1'b0, ld_k}) : (rot_p + 2'd2);

    assign og_o       = og;
    assign ig_o       = ig;
    assign ig_first_o = (ig == 8'd0);
    assign ig_last_o  = ((ig + 8'd1) >= n_ig);
    assign waddr_o    = wb + (og * n_ig) + ig;
    assign row_o      = r;

    reg  v_row_vld;  reg [ROW_W-1:0] v_row;
    reg  [COL_W-1:0] v_col;
    reg  [15:0]     v_pix;
    reg  v_load, v_rot;  reg [1:0] v_ldst;  reg [7:0] v_lbwa;

    always @(*) begin
        // 默认
        v_row_vld = 1'b0;  v_row = r;  v_col = 8'd0;  v_pix = 16'd0;
        v_load = 1'b0; v_rot = 1'b0; v_ldst = 2'd0; v_lbwa = 8'd0;

        case (st)
        S_PRE: begin
            v_load = 1'b1;
            // ★ 预装载的槽位必须**相对 rot_p**（与 RUN 的窗口映射 rot_p+tr 一致）。
            //   写死 1/2 只在 ig=0 结束后的 rot_p==0（= oh 是 4 的倍数）时对；
            //   oh=2 的层（rec M4p 起）rot_p=2 ⇒ 写 slot1/2 而 RUN 读 slot3
            //   ⇒ ig≥1 的整组输入读成 0（踩过，表现为 ig=1 的 sum 全 0）。
            v_ldst = (rot_p + 2'd1 + {1'b0, (pre_cnt != 8'd0)}) & 2'd3;
            v_lbwa = x;
        end
        S_LOAD: begin
            // r==0 的装载已在 S_PRE 做完 ⇒ 这里只管 r>=1 的装载
            //   stride-1 装 1 行，stride-2 装 2 行（ld_k 计数）
            if (r >= 8'd1) begin
                v_load = 1'b1;
                v_ldst = ld_slot;
                v_lbwa = x;
            end
        end
        S_RUN: begin
            v_row_vld = (x < ow);
            v_row     = r;
            v_col     = x;
            // ★ 输出像素号 = row*ow + col（ACT 写地址要用它；acc RAM 用 col）
            v_pix     = pixel_row_base + {8'd0, x};
            // ★ 装载/计算重叠：同拍把下一行写进空闲槽 (rot_p+3)&3
            if (run_load) begin
                v_load = 1'b1;
                v_ldst = (rot_p + 2'd3) & 2'd3;
                v_lbwa = x;
            end
        end
        S_ROT: begin
            v_rot = 1'b1;
        end
        S_DRAIN: begin
            // ★★★ 「装载/计算重叠」：PRE 移进 DRAIN。
            //   DRAIN 的 9 拍里 PE 链在排空、linebuf 写口与 ACT 读口都空闲
            //   ⇒ 顺带把**下一迭代**的 2 行预装载发掉（最多 DRAIN_CYC-1 拍，
            //   留最后 1 拍给 linebuf 锁存，保证进 S_WARM 时窗口已稳定）。
            //   ACT 读 in_base 区、outacc 同时写 out_base 区 ⇒ 两区不重叠，无冲突。
            if (ovl_pre) begin
                v_load = 1'b1;
                v_ldst = (rot_p + 2'd1 + {1'b0, dbeat_r}) & 2'd3;
                v_lbwa = dbeat_c;
            end
        end
        default: ;
        endcase
    end

    assign row_vld_o = v_row_vld;
    assign col_o     = v_col;
    assign pix_o     = v_pix;
    assign lb_load_o = v_load;
    assign lb_rot_o  = v_rot;
    assign lb_ldst_o = v_ldst;
    assign lb_waddr_o= v_lbwa;
    assign ld_row_o  = ld_row;
    assign ld_k_o    = ld_k;

    // ---------------------------------------------------------------
`ifdef FACEREC_SIM
    always @(posedge clk) if(rst_n && st==S_RUN && v_row_vld)
        if(v_pix !== (({8'd0,r}*{8'd0,ow}+{8'd0,x}) & 16'hffff))
            $fatal(1,"CNN pixel row-base disagrees with original address formula");
`endif
    // 状态转移
    // ---------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            st <= S_IDLE; busy <= 1'b0; layer_done <= 1'b0;
            og <= 8'd0; ig <= 8'd0; r <= 8'd0; x <= 8'd0;
            pre_cnt <= 8'd0; rot_p <= 2'd0; wb <= 14'd0; ld_k <= 1'b0;
            dr_cnt <= 4'd0; drain_last <= 1'b0;
            dbeat_c <= 8'd0; dbeat_r <= 1'b0; pre_full <= 1'b0; dr_last <= 1'b0;
            warm_from_rot <= 1'b0;
        end else begin
            layer_done <= 1'b0;

            case (st)
            //----------------------------------------------------------
            S_IDLE: begin
                if (start) begin
                    busy     <= 1'b1;
                    og       <= 8'd0;  ig  <= 8'd0;
                    r        <= 8'd0;  x   <= 8'd0;
                    pre_cnt  <= 8'd0;  rot_p <= 2'd0;  ld_k <= 1'b0;
                    wm_cnt   <= 4'd0;
                    wb       <= weight_base;
                    st       <= S_PRE;
                end
            end

            //----------------------------------------------------------
            S_PRE: begin                          // 预装载 2 行
                if (x == iw - 8'd1) begin
                    x <= 8'd0;
                    if (pre_cnt == 8'd0) pre_cnt <= 8'd1;
                    else begin
                        pre_cnt <= 8'd2;
                        r <= 8'd0;
                        wm_cnt <= 4'd0;
                        warm_from_rot <= 1'b0;
                        st <= S_WU;
                    end
                end else x <= x + 8'd1;
            end

            //----------------------------------------------------------
            S_LOAD: begin
                // r>=1 才装载；r==0 已在预装载里做完
                if (r >= 8'd1) begin
                    if (x == iw - 8'd1) begin
                        x <= 8'd0;
                        // ★ stride-2 越界行：第 2 行的输入行号 (2r+1) 在 ih 为奇数时
                        //   == ih（不存在）⇒ 直接结束本次装载，免得 ACT 读越界
                        if (!ld_k && ld_total && ((r << 1) + 8'd1 < ih)) begin
                            ld_k <= 1'b1;              // stride-2：再装第二行
                        end else begin
                            ld_k <= 1'b0;
                            wm_cnt <= 4'd0;
                            warm_from_rot <= 1'b0;
                            st   <= S_WU;
                        end
                    end else x <= x + 8'd1;
                end else begin
                    x <= 8'd0;
                    wm_cnt <= 4'd0;
                    warm_from_rot <= 1'b0;
                    st <= S_WU;
                end
            end

            //----------------------------------------------------------
            S_WARM: begin
                // 启动偏斜期：不出像素（v_row_vld=0，默认）、不装载、col=0
                //   ⇒ linebuf 输出稳定 9 拍 ⇒ dly[0..8] 全部 = 同一 slot 字
                // ⚠ v_col 默认 0 = RUN 首列 ⇒ warm 期与 RUN 首拍同 slot ✓
                if (wm_cnt == warm_cyc) begin
                    wm_cnt <= 4'd0;
                    x      <= 8'd0;
                    st     <= S_RUN;
                end else wm_cnt <= wm_cnt + 4'd1;
            end

            //----------------------------------------------------------
            // ★ 2026-10-24（opt1）：试去掉行末冲刷拍（原 ow+1 拍 → ow 拍）
            //   依据：偏斜链的模型是 dly[t](τ)=win[t](τ-t)，像素只在**采样那一拍**需要
            //   窗口正确，之后的 9 拍传播期窗口变不变都不影响该像素。
            //   收益：每行 1 拍 × 7,292 个行迭代 ≈ 7.3 K 拍（2.9%）。
            //   判据：÷4 rec 整网逐位一致（不一致就回退）。
            S_RUN: begin                          // ow 拍
                if (x == ow - 8'd1) begin
                    x <= 8'd0;
                    st <= S_ROT;
                end else x <= x + 8'd1;
            end

            //----------------------------------------------------------
            S_ROT: begin
                rot_p <= rot_p + (stride2 ? 2'd2 : 2'd1);
                ld_k  <= 1'b0;
                if (r == oh - 8'd1) begin
                    st <= S_NEXT;
                end else begin
                    r  <= r + 8'd1;
                    // ★ stride-1 的装载已在上一行 RUN 里做完 ⇒ 直接进 S_WARM
                    st <= stride2 ? S_LOAD : S_WU;
                    if (!stride2) begin
                        wm_cnt <= 4'd0;
                        warm_from_rot <= 1'b1;   // 装载已在 RUN 里做完 ⇒ 8 拍够
                    end else begin
                        // ★ opt2：第一行的前 ow 列已在 RUN 里装完 ⇒ 从第 ow 列续
                        x <= (ow < iw) ? ow : 8'd0;
                    end
                end
            end

            //----------------------------------------------------------
            S_NEXT: begin                         // 换 ig / 换 og / 结束
                // ★ rot_p **不复位**：linebuf 的物理 rot_p 不会跟着复位，
                //   预装载槽位 (rot_p+1)/(rot_p+2) 与窗口映射 (rot_p+tr)
                //   用同一个连续演化的 rot_p ⇒ 天然一致（CIFAR 的"相对寻址"）。
                //   复位它 = 第 2 个 ig 迭代起窗口整体错位（踩过）。
                if (ig_more_q) begin
                    ig <= ig + 8'd1;
                    r <= 8'd0; x <= 8'd0; pre_cnt <= 8'd0; ld_k <= 1'b0;
                    dr_cnt <= 4'd0;
                    dbeat_c <= 8'd0; dbeat_r <= 1'b0; pre_full <= 1'b0;
                    st <= S_DRAIN;
                end else if (og_more_q) begin
                    og <= og + 8'd1; ig <= 8'd0;
                    r <= 8'd0; x <= 8'd0; pre_cnt <= 8'd0; ld_k <= 1'b0;
                    dr_cnt <= 4'd0;
                    dbeat_c <= 8'd0; dbeat_r <= 1'b0; pre_full <= 1'b0;
                    st <= S_DRAIN;
                end else begin
                    // ★ 层末也要排空：最后像素的 tap 在链上要跨 RUN 结束后
                    //   8 拍才乘完、再经 outacc 2 拍写 ACT ⇒ busy/layer_done
                    //   必须等 DRAIN 完才落，否则在途像素全部卡死在管线里。
                    drain_last <= 1'b1;
                    dr_cnt     <= 4'd0;
                    st         <= S_DRAIN;
                end
            end

            //----------------------------------------------------------
            // ★ 排空态：本迭代最后像素的 9 个 tap 在链上要到 RUN 结束后
            //   第 8 拍才乘完（tap k 在 T0+k 拍乘）。此期间换权重会让
            //   在途像素混用新旧权重 ⇒ 必须等 DRAIN_CYC 拍再进 LOAD
            //   （引擎在 DRAIN 首拍触发权重装载，延迟 4 拍落到 wreg，
            //     早于新迭代首拍、晚于在途像素乘完，两边都安全）。
            //----------------------------------------------------------
            S_DRAIN: begin
                dr_last <= (dr_cnt == DRAIN_CYC - 4'd2);   // 最后一拍前置标记
                // 预装载进度：每拍一列，行末换行；第二行末列发完 ⇒ pre_full
                if (ovl_pre) begin
                    if (dbeat_c + 8'd1 == iw) begin
                        dbeat_c <= 8'd0;
                        if (dbeat_r) pre_full <= 1'b1;
                        else         dbeat_r <= 1'b1;
                    end else
                        dbeat_c <= dbeat_c + 8'd1;
                end
                if (dr_cnt == DRAIN_CYC - 4'd1) begin
                    if (drain_last) begin
                        busy       <= 1'b0;
                        layer_done <= 1'b1;
                        drain_last <= 1'b0;
                        st         <= S_IDLE;
                    end else if (pre_full) begin
                        // ★ 2 行已在 DRAIN 里装完 ⇒ **跳过 S_PRE** 直接进 S_WARM
                        dbeat_c <= 8'd0; dbeat_r <= 1'b0; pre_full <= 1'b0;
                        wm_cnt  <= 4'd0;  x <= 8'd0;
                        warm_from_rot <= 1'b0;
                        st      <= S_WU;
                    end else begin
                        // 未装完 ⇒ 回 S_PRE **从断点续装**（避免重装已装的列）
                        pre_cnt <= {7'd0, dbeat_r};
                        x       <= dbeat_c;
                        st      <= S_PRE;
                    end
                end else
                    dr_cnt <= dr_cnt + 4'd1;
            end

            default: st <= S_IDLE;
            endcase
        end
    end

endmodule
