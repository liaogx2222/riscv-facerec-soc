//=====================================================================
// cnn_soc.v : FaceRec 加速器顶层（2026-10-20 新增）
//
//  帧写口 ──> 帧缓冲(FW×FH×24b) ──> cnn_roi ──┐
//                                              ├─> ACT(4 bank) ──> cnn_engine ──> ACT
//  cnn_ctrl(逐层调度) ────────────────────────┘
//  ACT 读回口（M11-lite，自动递增 256b/拍）
//
// ★ 本层解决的三个"集成才暴露"的问题
//   ① **ACT 三方仲裁**：写口有 engine 与 roi 两个主；读口有 engine 与读回两个主。
//      二者**时间上互斥**（ROI 先跑完、再跑网络；读回在网络空闲时），
//      所以用简单的模式选择即可，不需要总线仲裁器。
//   ② **帧缓冲**（M10 的最小核心）：CPU/相机逐像素写，ROI 以 1 像素/拍同步读。
//      ⚠ ROI 恒把输出写到 **bank 索引 = 像素号、bank = 0..in_wpg-1** —— 与第 0 层
//        `in_base=0`、`in_wpg=4` 的读地址 `raddr=(0+pix*4)>>2=pix` 严格对应。
//   ③ **描述符 RAM 与 ROM 偏移**：两套网络共用一块描述符 RAM（`desc_base` 区分），
//      ROM 偏移（`woff/boff`）由本层按"网络放进 ROM 的位置"给。
//
// ⚠ 未做（后续）：与识别首层的**流式融合**（ROI 直接喂 linebuf，省掉 ACT 往返）；
//   CPU 侧的寄存器/MMIO 封装（本层用并行口，接 Wishbone 是薄薄一层）。
//=====================================================================
`timescale 1ns/1ps

module cnn_soc #(
    parameter FW     = 128,      // 帧宽
    parameter FH     = 96,       // 帧高
    parameter OUT    = 64,       // ROI 输出边长（= 识别输入边长）
    parameter AAW    = 16,       // ★ ACT 地址位宽（必须 16：bank 索引可 ≥1024）
    parameter ADEPTH = 5376,     // ACT 每 bank 字数（÷4 用 336）
    parameter DAW    = 10,       // 描述符 RAM 地址位宽
    parameter NDESC  = 512,
    parameter B0DEPTH = ADEPTH,
    parameter DESC_FILE = "rtl/accel/mem/cnn_desc.mem"
)(
    input  wire              clk,
    input  wire              rst_n,

    // ---- 帧写口（M10 简化：CPU/相机逐像素写）----
    input  wire              fw_we,
    input  wire [7:0]        fw_x,
    input  wire [7:0]        fw_y,
    input  wire [23:0]       fw_d,

    // ---- ROI 启动 ----
    input  wire              roi_go,
    input  wire [7:0]        x1, y1, x2, y2,

    // ---- 帧→ACT 物化启动（M15：全流程第一棒，det 首层输入）----
    input  wire              fsrc_go,
    output wire              fsrc_busy,
    output wire              fsrc_done,

    // ---- D1 dense（M16：FC 1024→64，embedding）----
    input  wire              d1_go,
    input  wire [15:0]       d1_in_base,    // M10 输出的 ACT flat 基址
    input  wire [15:0]       d1_out_base,   // embedding 写出的 ACT flat 基址
    input  wire [7:0]        d1_sh_req,
    output wire              d1_busy,
    output wire              d1_done,

    // ---- 网络启动 ----
    input  wire              net_go,
    input  wire [7:0]        nlayer,
    input  wire              net_hold,       // 高 = 每层 done 后暂停（供 CPU 读回该层输出）
    input  wire [DAW-1:0]    desc_base,
    input  wire [13:0]       woff,
    input  wire [7:0]        boff,

    // ---- ACT 读回（M11-lite：1 个 256b 字/拍，地址自动递增）----
    input  wire              rd_go,
    input  wire [15:0]       rd_word,        // 起始 64b 字地址
    input  wire [15:0]       rd_nword,       // 读多少个 64b 字
    output reg               rd_valid,
    output reg  [255:0]      rd_data,
    output reg               rd_busy,
    output reg               rd_done,

    // ---- 状态 ----
    output wire              roi_busy,
    output wire              roi_done,
    output wire              net_busy,
    output wire              net_done,
    output wire              ctrl_timeout,
    output wire [7:0]        layer_idx,
    // ★ 每层完成**单拍脉冲**（= cnn_ctrl 看到的 eng_done）。
    //   为什么必须给 CPU：head 层的 out_base **互相重叠**
    //   （H2c_p/H2b_p 都是 768、H3c/H3b 都是 384、H4c/H4b 都是 0），
    //   后一个 head 会覆盖前一个 ⇒ CPU 必须在 layer_done 那一层立刻读回。
    //   `layer_idx` 是电平（停在刚完成的层），单靠它轮询会漏掉脉冲。
    output wire              layer_done
);

    //-----------------------------------------------------------------
    // 帧缓冲（写口来自 CPU/相机；读口给 ROI，同步读 1 拍）
    //-----------------------------------------------------------------
    reg [23:0] FB [0:FW*FH-1];
    always @(posedge clk) if (fw_we) FB[fw_y * FW + fw_x] <= fw_d;

    // 帧读口 = ROI 与 fsrc **分时复用**（两者不会同时跑，SoC 流程保证）
    wire [7:0]  roi_fx, roi_fy;      // cnn_roi 驱动的请求
    wire        roi_fen;
    wire [7:0]  fs_x, fs_y;          // cnn_fsrc 驱动的请求
    wire        fs_en;
    wire [7:0]  fr_x, fr_y;
    wire        fr_en;
    reg  [23:0] fr_data;
    integer     fi;
    wire        fsrc_act = fsrc_busy | fsrc_go;
    assign fr_x  = fsrc_act ? fs_x  : roi_fx;
    assign fr_y  = fsrc_act ? fs_y  : roi_fy;
    assign fr_en = fsrc_act ? fs_en : roi_fen;
    always @(posedge clk) begin
        if (fr_en) begin
            fi = fr_y * FW + fr_x;
            fr_data <= (fi < FW*FH) ? FB[fi] : 24'd0;
        end
    end

    //-----------------------------------------------------------------
    // ROI
    //-----------------------------------------------------------------
    wire        aw_we;
    wire [15:0] aw_addr;
    wire [63:0] aw_data;
    wire [23:0] roi_rgb;
    wire [15:0] roi_pix;
    wire        roi_ovld;
    wire [23:0] roi_aq;

    cnn_roi #(.OUT(OUT), .FW(FW), .FH(FH), .WP4(4)) u_roi (
        .clk(clk), .rst_n(rst_n), .start(roi_go),
        .x1(x1), .y1(y1), .x2(x2), .y2(y2),
        .fr_x(roi_fx), .fr_y(roi_fy), .fr_en(roi_fen), .fr_data(fr_data),
        .o_valid(roi_ovld), .o_pix(roi_pix), .o_rgb(roi_rgb), .o_aq(roi_aq),
        .aw_we(aw_we), .aw_addr(aw_addr), .aw_data(aw_data),
        .busy(roi_busy), .done(roi_done)
    );

    // ---- 帧→ACT 物化（M15）----
    wire [3:0]  fw_we4;
    wire [15:0] fw_addr;
    wire [255:0] fw_data;
    cnn_fsrc #(.FW(FW), .FH(FH)) u_fsrc (
        .clk(clk), .rst_n(rst_n), .start(fsrc_go),
        .fr_x(fs_x), .fr_y(fs_y), .fr_en(fs_en), .fr_data(fr_data),
        .aw_we(fw_we4), .aw_addr(fw_addr), .aw_data(fw_data),
        .busy(fsrc_busy), .done(fsrc_done)
    );

    // ---- D1 dense（M16）----
    wire        d1_re;
    wire [15:0] d1_raddr;
    wire [3:0]  d1_bmask;
    wire [3:0]  d1_we;
    wire [15:0] d1_waddr;
    wire [255:0] d1_wdata;
    cnn_d1 #(.IC(1024), .OC(64), .IW(8),
             .W_FILE("rtl/accel/mem/cnn_dwrom.mem"),
             .SHC_FILE("rtl/accel/mem/cnn_d1shc.mem")) u_d1 (
        .clk(clk), .rst_n(rst_n), .start(d1_go),
        .in_base(d1_in_base), .out_base(d1_out_base), .sh_req(d1_sh_req),
        .d_re(d1_re), .d_raddr(d1_raddr), .d_bmask(d1_bmask), .d_rdata(act_rdata),
        .d_we(d1_we), .d_waddr(d1_waddr), .d_wdata(d1_wdata),
        .busy(d1_busy), .done(d1_done)
    );

    wire [255:0] act_rdata;

    //-----------------------------------------------------------------
    // 控制器
    //-----------------------------------------------------------------
    wire        eng_rst_n, eng_start;
    wire [1:0]  op_mode;
    wire [7:0]  n_og, n_ig, ih, iw, oh, ow, in_wpg, out_wpg, sh_req, brom_base;
    wire        stride2, dw_mode, mode_pw, lin_out;
    wire [13:0] weight_base;
    wire [15:0] in_base, out_base;
    wire        eng_busy, eng_done;
    assign layer_done = eng_done;

    wire [DAW-1:0] c_dram_addr;
    wire           c_dram_re;
    reg  [31:0]    c_dram_data;
    reg  [31:0]    DMEM [0:NDESC-1];
    initial $readmemh(DESC_FILE, DMEM);

    always @(posedge clk) if (c_dram_re) c_dram_data <= DMEM[c_dram_addr];

    cnn_ctrl #(.DAW(DAW), .TMO(0)) u_ctrl (
        .clk(clk), .rst_n(rst_n), .go(net_go), .nlayer(nlayer),
        .hold(net_hold), .desc_base(desc_base),
        .wbase_off(woff), .bbase_off(boff),
        .dram_addr(c_dram_addr), .dram_re(c_dram_re), .dram_data(c_dram_data),
        .eng_rst_n(eng_rst_n), .eng_start(eng_start),
        .op_mode(op_mode), .n_og(n_og), .n_ig(n_ig), .ih(ih), .iw(iw),
        .oh(oh), .ow(ow), .stride2(stride2), .dw_mode(dw_mode),
        .mode_pw(mode_pw), .lin_out(lin_out),
        .weight_base(weight_base), .in_base(in_base), .out_base(out_base),
        .in_wpg(in_wpg), .out_wpg(out_wpg), .sh_req(sh_req), .brom_base(brom_base),
        .eng_busy(eng_busy), .eng_done(eng_done),
        .busy(net_busy), .net_done(net_done), .layer_idx(layer_idx),
        .timeout(ctrl_timeout)
    );

    //-----------------------------------------------------------------
    // 引擎
    //-----------------------------------------------------------------
    wire [3:0]   e_we, e_bmask;
    wire [15:0]  e_waddr, e_raddr;
    wire [255:0] e_wdata;
    wire         e_re;

    cnn_engine #(.CH(32), .MAXW(128), .WAW(7), .ACC_AW(AAW),
                 .ACC_DEPTH(1024), .ACC_AW_C(10), .WROM_AW(11)) u_eng (
        .clk(clk), .rst_n(eng_rst_n), .start(eng_start), .op_mode(op_mode),
        .n_og(n_og), .n_ig(n_ig), .ih(ih), .iw(iw), .oh(oh), .ow(ow),
        .stride2(stride2), .dw_mode(dw_mode), .mode_pw(mode_pw), .lin_out(lin_out),
        .weight_base(weight_base), .in_base(in_base), .in_wpg(in_wpg),
        .out_base(out_base), .out_wpg(out_wpg), .sh_req(sh_req),
        .brom_base(brom_base),
        .busy(eng_busy), .layer_done(eng_done),
        .act_we(e_we), .act_waddr(e_waddr), .act_wdata(e_wdata),
        .act_re(e_re), .act_raddr(e_raddr), .act_bank_mask(e_bmask),
        .act_rdata(act_rdata)
    );

    //-----------------------------------------------------------------
    // ACT 读回（M11-lite）
    //-----------------------------------------------------------------
    localparam R_IDLE = 2'd0, R_RUN = 2'd1, R_DONE = 2'd2;
    reg [1:0]  rst_rd;
    reg [15:0] rd_cnt;
    reg [15:0] rd_base;
    reg [15:0] rd_len;
    reg        r_re;
    reg [15:0] r_raddr;

    always @(posedge clk) begin
        if (!rst_n) begin
            rst_rd <= R_IDLE; rd_cnt <= 16'd0; rd_base <= 16'd0; rd_len <= 16'd0;
            r_re <= 1'b0; r_raddr <= 16'd0;
            rd_valid <= 1'b0; rd_data <= 256'd0; rd_busy <= 1'b0; rd_done <= 1'b0;
        end else begin
            rd_done  <= 1'b0;
            rd_valid <= 1'b0;
            case (rst_rd)
            R_IDLE: begin
                r_re <= 1'b0;
                if (rd_go) begin
                    rd_busy <= 1'b1;
                    rd_base <= rd_word;
                    rd_len  <= rd_nword;
                    rd_cnt  <= 16'd0;
                    // ★ 地址与 re 在**进入 R_RUN 前**就给出，保证 c=0 拍就在读 rd_word
                    r_re    <= 1'b1;
                    r_raddr <= rd_word;
                    rst_rd  <= R_RUN;
                end
            end
            R_RUN: begin
                // ★ 同步读 1 拍：c 拍采到的是 (c-1) 号地址的数据
                //   ⇒ c=0 那次采的是垃圾，不报 valid；地址要提前 +1
                r_re     <= 1'b1;
                r_raddr  <= rd_base + rd_cnt + 16'd1;
                rd_data  <= act_rdata;
                rd_valid <= (rd_cnt != 16'd0);
                if (rd_cnt == rd_len) begin
                    r_re   <= 1'b0;
                    rst_rd <= R_DONE;
                end else
                    rd_cnt <= rd_cnt + 16'd1;
            end
            R_DONE: begin
                rd_done <= 1'b1;
                rd_busy <= 1'b0;
                rst_rd  <= R_IDLE;
            end
            endcase
        end
    end

    //-----------------------------------------------------------------
    // ACT 三方仲裁（时间上互斥 ⇒ 简单模式选择）
    //-----------------------------------------------------------------
    // 写：fsrc（帧→ACT）运行期优先，其次 ROI（只写 bank0），否则引擎写
    //   ⚠ 三者时间互斥：fsrc 只在 det 前跑一次，ROI 只在 det 完成后跑一次
    wire        roi_act = roi_busy | roi_go;
    wire [3:0]   a_we    = fsrc_act ? (fw_we4 & 4'b0001)
                         : (d1_act ? d1_we : (roi_act ? aw_we : e_we));
    // Packed image input is bank0-only, above the 6144-word feature workspace.
    wire [15:0]  a_waddr = fsrc_act ? (fw_addr + 16'd6144)
                         : (d1_act ? d1_waddr : (roi_act ? aw_addr : e_waddr));
    wire [255:0] a_wdata = fsrc_act ? fw_data
                         : (d1_act ? d1_wdata
                                   : (roi_act ? {192'd0, aw_data} : e_wdata));

    // 读：D1 运行期优先，其次读回口独占，否则引擎读（全部时间互斥）
    wire         d1_act  = d1_busy | d1_go;
    wire         rd_act  = (rst_rd != R_IDLE);
    wire         a_re    = d1_act ? d1_re   : (rd_act ? r_re : e_re);
    wire [15:0]  a_raddr = d1_act ? d1_raddr : (rd_act ? r_raddr : e_raddr);
    wire [3:0]   a_bmask = d1_act ? d1_bmask : (rd_act ? 4'b1111 : e_bmask);

    cnn_act #(.BANKS(4), .CH_BANK(8), .AW(8), .DEPTH(ADEPTH), .AAW(AAW), .B0DEPTH(B0DEPTH)) u_act (
        .clk(clk),
        .we(a_we), .waddr(a_waddr), .wdata(a_wdata),
        .raddr(a_raddr), .bank_mask(a_bmask), .rdata(act_rdata)
    );

endmodule
