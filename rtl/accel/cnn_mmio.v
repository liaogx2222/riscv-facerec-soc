//=====================================================================
// cnn_mmio.v —— 把 cnn_soc + cnn_slidewin 暴露成 **32 位内存映射寄存器**
//
// 为什么需要这一层
//   1. cnn_soc 有 ~470 个端口（位宽合计远超 FBB676 的引脚数），CPU 不可能直接连；
//   2. CPU 的总线是「一拍一个 32 位读/写」，加速器的 `*_go` / `*_done` 是**单拍脉冲**
//      ⇒ 直接用 CPU 轮询会**漏掉脉冲**（轮询一次要 3~10 拍）。所以这里把 done 全部
//        做成**粘滞位 + 写 1 清除**，CPU 想什么时候看就什么时候看。
//   3. `rd_data` 是 256 b，CPU 一次只能拿 32 b ⇒ 需要一个**捕获寄存器**：
//      rd 流是「一拍一个地址」（不能背压），CPU 跟不上，所以只抓**第一个有效 beat**。
//      ★ 由此定下使用约定：`rd_nword` 只写 1（一个地址 = 一个像素的 32 通道），
//        地址 = 像素地址（不是扁平 64b 字地址）。
//
// 寄存器表（字节偏移；`addr[11:0]`）
//   0x000 CTRL     W  [0]net_go [1]fsrc_go [2]roi_go [3]d1_go [4]rd_go [5]sw_go
//                     [6]net_hold(电平) [7]rd_clr
//                     ★ [0..5] 是**自清脉冲**：写 1 后下一拍自动归零，不会重复触发
//   0x004 STATUS   R  [0]net_busy  [1]net_done*
//                     [2]fsrc_busy [3]fsrc_done*
//                     [4]roi_busy  [5]roi_done*
//                     [6]d1_busy   [7]d1_done*
//                     [8]rd_busy   [9]rd_done*   [10]rd_ready
//                     [11]layer_done*  [12]ctrl_timeout
//                     [13]sw_win_done* [14]sw_frame_done* [15]sw_busy
//                     （* = 粘滞位，靠 0x030 CLR 清）
//   0x008 ROI      W  x1[7:0] | y1[15:8] | x2[23:16] | y2[31:24]
//   0x00C D1B      W  in_base[15:0] | out_base[31:16]
//   0x010 D1SH     W  sh_req[7:0]
//   0x014 NETCFG   W  nlayer[7:0] | woff[21:8]
//   0x018 NETCFG2  W  boff[7:0] | desc_base[17:8]
//   0x01C RDADDR   W  rd_word[15:0] | rd_nword[31:16]
//   0x020 RDIDX    W  [2:0] 选 rd_cap 的哪 32 b
//   0x024 RDDATA   R  rd_cap[32*idx +: 32]
//   0x028 ID       R  0x46524331 ("FRC1")
//   0x02C LAYER    R  layer_idx[7:0]
//   0x030 CLR      W  [0]fsrc [1]roi [2]d1 [3]net [4]rd [5]layer
//                     [6]sw_win [7]sw_frame [8]all
//   0x034 SWSTAT   R  win_idx[3:0] | frame_idx[11:8]
//=====================================================================
`timescale 1ns/1ps

module cnn_mmio #(
    // ---- cnn_soc ----
    parameter FW       = 128,
    parameter FH       = 96,
    parameter OUT      = 64,
    parameter AAW      = 16,
    parameter ADEPTH   = 6144,
    parameter B0DEPTH  = 18432,
    parameter DAW      = 10,
    parameter NDESC    = 512,
    // ---- 滑窗帧源 ----
    parameter SW_EN    = 1,          // 0 = 不用滑窗（fw_* 留给外部相机）
    parameter SW_FW    = 320,
    parameter SW_FH    = 240,
    parameter SW_NX    = 3,
    parameter SW_NY    = 3,
    parameter SW_NFR   = 4,
    parameter SW_DW    = 8,
    parameter SW_HOLD  = 0,          // CPU owns seek; 0 disables autonomous advance
    parameter SW_MEM   = "rtl/accel/mem/slidewin.mem"
)(
    input  wire        clk,
    input  wire        rst_n,
    // ---- 32 位内存映射总线（与 RISCV_CPU 的 soc_top 同风格）----
    input  wire        sel,          // 本模块片选（地址高位已在外层译好）
    input  wire        we,           // 1 = 写
    input  wire [11:0] addr,         // accepted write address
    input  wire [11:0] raddr,        // independent, side-effect-free read address
    input  wire [31:0] wdata,
    output reg  [31:0] rdata
);

    //=================================================================
    // 寄存器
    //=================================================================
    reg [7:0]    r_nlayer, r_shreq, r_boff;
    reg [13:0]   r_woff;
    reg [DAW-1:0] r_desc;
    reg [7:0]    r_x1, r_y1, r_x2, r_y2;
    reg [15:0]   r_d1in, r_d1out;
    reg [15:0]   r_rdword, r_rdn;
    reg [2:0]    r_rdidx;
    reg          r_hold;
    reg [3:0]    r_winsel;
    reg [8:0]    r_winx, r_winy;
    reg [7:0]    r_frame;
    reg          r_seekxy;

    // 自清脉冲
    reg          p_net, p_fsrc, p_roi, p_d1, p_rd, p_sw, p_seek;

    // 粘滞 done 位
    reg          s_fsrc, s_roi, s_d1, s_net, s_rd, s_lay, s_win, s_frm;

    // rd 捕获
    reg [255:0]  rd_cap;
    reg          rd_ready;

    // CLR 解码（组合，只在本拍写时有效）
    wire wr = sel & we;
    wire clr_fsrc = wr & (addr == 12'h030) & (wdata[0] | wdata[8]);
    wire clr_roi  = wr & (addr == 12'h030) & (wdata[1] | wdata[8]);
    wire clr_d1   = wr & (addr == 12'h030) & (wdata[2] | wdata[8]);
    wire clr_net  = wr & (addr == 12'h030) & (wdata[3] | wdata[8]);
    wire clr_rd   = wr & (addr == 12'h030) & (wdata[4] | wdata[8]);
    wire clr_lay  = wr & (addr == 12'h030) & (wdata[5] | wdata[8]);
    wire clr_win  = wr & (addr == 12'h030) & (wdata[6] | wdata[8]);
    wire clr_frm  = wr & (addr == 12'h030) & (wdata[7] | wdata[8]);

    //=================================================================
    // 加速器 + 滑窗源
    //=================================================================
    wire        fw_we, fsrc_busy, fsrc_done, d1_busy, d1_done;
    wire        net_busy, net_done, layer_done, ctrl_timeout;
    wire        roi_busy, roi_done;
    wire        rd_valid, rd_busy, rd_done;
    wire [255:0] rd_data;
    wire [7:0]  layer_idx;
    wire [3:0]  sw_win;
    wire [7:0]  sw_frame;
    wire        sw_win_done, sw_frame_done;

    wire        fw_we_ext;
    wire [7:0]  fw_x, fw_y;
    wire [23:0] fw_d;

    cnn_soc #(
        .FW(FW), .FH(FH), .OUT(OUT), .AAW(AAW), .ADEPTH(ADEPTH), .B0DEPTH(B0DEPTH),
        .DAW(DAW), .NDESC(NDESC)
    ) u_soc (
        .clk(clk), .rst_n(rst_n),
        .fw_we(fw_we), .fw_x(fw_x), .fw_y(fw_y), .fw_d(fw_d),
        .roi_go(p_roi), .x1(r_x1), .y1(r_y1), .x2(r_x2), .y2(r_y2),
        .fsrc_go(p_fsrc), .fsrc_busy(fsrc_busy), .fsrc_done(fsrc_done),
        .d1_go(p_d1), .d1_in_base(r_d1in), .d1_out_base(r_d1out),
        .d1_sh_req(r_shreq), .d1_busy(d1_busy), .d1_done(d1_done),
        .net_go(p_net), .nlayer(r_nlayer), .net_hold(r_hold),
        .desc_base(r_desc), .woff(r_woff), .boff(r_boff),
        .rd_go(p_rd), .rd_word(r_rdword), .rd_nword(r_rdn),
        .rd_valid(rd_valid), .rd_data(rd_data),
        .rd_busy(rd_busy), .rd_done(rd_done),
        .roi_busy(roi_busy), .roi_done(roi_done),
        .net_busy(net_busy), .net_done(net_done),
        .ctrl_timeout(ctrl_timeout),
        .layer_idx(layer_idx), .layer_done(layer_done)
    );

    generate
    if (SW_EN != 0) begin : g_sw
        cnn_slidewin #(
            .FW(SW_FW), .FH(SW_FH),
            .WIN_W(FW), .WIN_H(FH),
            .NX(SW_NX), .NY(SW_NY),
            .NFRAME(SW_NFR), .DW(SW_DW),
            .HOLD_CYC(SW_HOLD), .MEM_FILE(SW_MEM), .BANKED_ROM(1)
        ) u_sw (
            .clk(clk), .rst_n(rst_n), .go(p_sw),
            .seek(p_seek), .win_sel(r_winsel),
            .seek_xy(r_seekxy), .win_x(r_winx), .win_y(r_winy),
            .frame_sel(r_frame),
            .fw_we(fw_we), .fw_x(fw_x), .fw_y(fw_y), .fw_d(fw_d),
            .win_idx(sw_win), .frame_idx(sw_frame),
            .win_done(sw_win_done), .frame_done(sw_frame_done)
        );
        assign fw_we_ext = 1'b0;
    end else begin : g_ext
        // 无滑窗：fw_* 留给外部（相机/上级模块）驱动
        assign fw_we   = fw_we_ext;
        assign fw_x    = 8'd0;
        assign fw_y    = 8'd0;
        assign fw_d    = 24'd0;
        assign sw_win  = 4'd0;
        assign sw_frame= 8'd0;
        assign sw_win_done  = 1'b0;
        assign sw_frame_done= 1'b0;
    end
    endgenerate

    //=================================================================
    // 写寄存器 + 自清脉冲
    //=================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            r_nlayer <= 8'd42; r_shreq <= 8'd0; r_boff <= 8'd0; r_woff <= 14'd0;
            r_desc   <= {DAW{1'b0}};
            r_x1 <= 8'd0; r_y1 <= 8'd0; r_x2 <= 8'd127; r_y2 <= 8'd95;
            r_d1in <= 16'd0; r_d1out <= 16'd0;
            r_rdword <= 16'd0; r_rdn <= 16'd1; r_rdidx <= 3'd0;
            r_hold <= 1'b1;                 // ★ 默认 hold：CPU 逐层读回
            r_winsel <= 4'd0; r_winx <= 9'd0; r_winy <= 9'd0; r_seekxy <= 1'b0;
            r_frame <= 8'd0;
            p_net <= 1'b0; p_fsrc <= 1'b0; p_roi <= 1'b0;
            p_d1  <= 1'b0; p_rd   <= 1'b0; p_sw  <= 1'b0; p_seek <= 1'b0;
        end else begin
            // ---- 自清：默认全部归零 ----
            p_net <= 1'b0; p_fsrc <= 1'b0; p_roi <= 1'b0;
            p_d1  <= 1'b0; p_rd   <= 1'b0; p_sw  <= 1'b0; p_seek <= 1'b0;

            if (wr) begin
                case (addr)
                12'h000: begin
                    p_net  <= wdata[0];
                    p_fsrc <= wdata[1];
                    p_roi  <= wdata[2];
                    p_d1   <= wdata[3];
                    p_rd   <= wdata[4];
                    p_sw   <= wdata[5];
                    r_hold <= wdata[6];
                    p_seek <= wdata[8];
                    r_seekxy <= wdata[9];
                end
                12'h008: begin
                    r_x1 <= wdata[7:0];   r_y1 <= wdata[15:8];
                    r_x2 <= wdata[23:16]; r_y2 <= wdata[31:24];
                end
                12'h00C: begin r_d1in <= wdata[15:0]; r_d1out <= wdata[31:16]; end
                12'h010: r_shreq <= wdata[7:0];
                12'h014: begin r_nlayer <= wdata[7:0]; r_woff <= wdata[21:8]; end
                12'h018: begin r_boff <= wdata[7:0]; r_desc <= wdata[17:8]; end
                12'h01C: begin r_rdword <= wdata[15:0]; r_rdn <= wdata[31:16]; end
                12'h020: r_rdidx <= wdata[2:0];
                12'h038: r_winsel <= wdata[3:0];
                12'h03C: begin
                    r_winx  <= wdata[8:0];
                    r_winy  <= wdata[17:9];
                    r_frame <= wdata[25:18];
                end
                default: ;
                endcase
            end
        end
    end

    //=================================================================
    // 粘滞 done 位（set 优先于 clear：同拍既完成又被清时不丢事件）
    //=================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            s_fsrc <= 1'b0; s_roi <= 1'b0; s_d1 <= 1'b0;
            s_net  <= 1'b0; s_rd  <= 1'b0; s_lay <= 1'b0;
            s_win  <= 1'b0; s_frm <= 1'b0;
        end else begin
            if (clr_fsrc) s_fsrc <= 1'b0;
            if (clr_roi)  s_roi  <= 1'b0;
            if (clr_d1)   s_d1   <= 1'b0;
            if (clr_net)  s_net  <= 1'b0;
            if (clr_rd)   s_rd   <= 1'b0;
            if (clr_lay)  s_lay  <= 1'b0;
            if (clr_win)  s_win  <= 1'b0;
            if (clr_frm)  s_frm  <= 1'b0;
            // set 放后面 ⇒ 同拍优先
            if (fsrc_done)      s_fsrc <= 1'b1;
            if (roi_done)       s_roi  <= 1'b1;
            if (d1_done)        s_d1   <= 1'b1;
            if (net_done)       s_net  <= 1'b1;
            if (rd_done)        s_rd   <= 1'b1;
            if (layer_done)     s_lay  <= 1'b1;
            if (sw_win_done)    s_win  <= 1'b1;
            if (sw_frame_done)  s_frm  <= 1'b1;
        end
    end

    //=================================================================
    // rd 捕获：只抓**第一个有效 beat**
    //   rd 流不能背压（一拍一个地址），CPU 跟不上 ⇒ 必须有捕获寄存器。
    //   约定 rd_nword = 1（一个地址 = 一个像素的 32 通道）。
    //=================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            rd_cap <= 256'd0; rd_ready <= 1'b0;
        end else begin
            if (p_rd || clr_rd) rd_ready <= 1'b0;
            if (rd_valid && !rd_ready) begin
                rd_cap   <= rd_data;
                rd_ready <= 1'b1;
            end
        end
    end

    //=================================================================
    // 读
    //=================================================================
    wire [31:0] st;
    assign st[0]  = net_busy;
    assign st[1]  = s_net;
    assign st[2]  = fsrc_busy;
    assign st[3]  = s_fsrc;
    assign st[4]  = roi_busy;
    assign st[5]  = s_roi;
    assign st[6]  = d1_busy;
    assign st[7]  = s_d1;
    assign st[8]  = rd_busy;
    assign st[9]  = s_rd;
    assign st[10] = rd_ready;
    assign st[11] = s_lay;
    assign st[12] = ctrl_timeout;
    assign st[13] = s_win;
    assign st[14] = s_frm;
    assign st[15] = (SW_EN != 0) ? ~(sw_win_done | sw_frame_done) : 1'b0;
    assign st[31:16] = 16'd0;

    always @(*) begin
        rdata = 32'h0;
        // The SoC qualifies read data with its read-page decode. Do not gate
        // this path with the independently arbitrated write transaction.
            case (raddr)
            12'h000: rdata = {24'd0, 1'b0, r_hold, 6'd0};
            12'h004: rdata = st;
            12'h008: rdata = {r_y2, r_x2, r_y1, r_x1};
            12'h00C: rdata = {r_d1out, r_d1in};
            12'h010: rdata = {24'd0, r_shreq};
            12'h014: rdata = {10'd0, r_woff, r_nlayer};
            12'h018: rdata = {14'd0, r_desc, r_boff};
            12'h01C: rdata = {r_rdn, r_rdword};
            12'h020: rdata = {29'd0, r_rdidx};
            12'h024: rdata = rd_cap[32*r_rdidx +: 32];
            12'h028: rdata = 32'h4652_4331;         // "FRC1"
            12'h02C: rdata = {24'd0, layer_idx};
            12'h034: rdata = {20'd0, sw_frame[3:0], sw_win};
            12'h038: rdata = {28'd0, r_winsel};
            12'h03C: rdata = {6'd0, r_frame, r_winy, r_winx};
            default: rdata = 32'h0;
            endcase
    end

endmodule
