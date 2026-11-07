//=====================================================================
// cnn_slidewin.v : **原图滑窗帧源**（金字塔层上滑 128×96 窗）
//
// 背景（见 doc/滑窗与并行可行性_20261025.md）：
//   整幅 800×600 压成 128×96 时脸只剩 7~14 px，检测基本是噪声（cos 中位 0.122）。
//   改成在 ÷2.5 的 320×240 上滑 3×3 = 9 个 128×96 窗 ⇒ 检出 6/6、cos 0.622，
//   而且每窗还是 128×96 ⇒ **检测引擎一行都不用改**。
//
// 本模块就是那个「窗口地址生成器 + 像素搬运」：
//   从一块 FW×FH 的缓冲里，按 NX×NY 的网格依次取出 WIN_W×WIN_H 的窗口，
//   一像素一拍，直接驱动 cnn_soc 的 `fw_we/fw_x/fw_y/fw_d` 口
//   （与 cnn_framesrc 完全兼容，可原位替换）。
//
// ★ 为什么窗口串行就够：9 窗 × 126,000 拍 = 1,134,000 拍 = 7.4 ms，
//   30 fps 预算 33.3 ms ⇒ 占空比 22%。**不需要并行、不需要多引擎。**
//
// ★ 缓冲布局：按 y*FW+x 线性存放，帧首尾相接（STRIDE = FW*FH，无需对齐：
//   8 位宽度下 PDS 对非 2 的幂深度正常映射，实测 307,200×8b → 86 DRM 无 DRC）。
//   窗口 (x0,y0) 的像素地址 = base + y0*FW + x0 + yc*FW + xc，
//   地址用**增量寄存器**走（行末 +FW-WIN_W+1），不综合出乘法器。
//
// ★★ 时序（踩过两次的坑，改之前先读）：
//   数据链 = paddr(寄存) → raddr(寄存) → rq(ROM 同步读) → fw_d(寄存) = **4 级**
//   所以坐标延 **3 级**（v1/v2 + 输出寄存 = 3）：
//       fw_d(t) ↔ xc(t-3)   ｜   fw_x(t) = x2(t-1) = xc(t-3) ✓
//   两个失败模式：
//     · 坐标只延 2 级 ⇒ 数据和坐标错开 1 像素（拿到邻居的值）
//     · 窗口起始地址在**进窗后一拍**才设 ⇒ paddr 比 xc 慢一拍，
//       行末跳转 (FW-WIN_W+1) 就跳错位置 ⇒ 每行最后一列读到垃圾
//   所以 paddr 必须在「进入该窗的那一拍」就设好（见 hold 分支）。
//
// 握手：每窗灌完给 `win_done`（单拍），整帧 NWIN 窗灌完额外给 `frame_done`。
//   `go` 单拍脉冲 = 提前进入下一窗（CPU 在 det 完成后给）。
//=====================================================================
`timescale 1ns/1ps

module cnn_slidewin_source #(
    parameter FW       = 320,          // 缓冲宽（÷2.5 后的原图宽）
    parameter FH       = 240,          // 缓冲高
    parameter WIN_W    = 128,          // 窗口宽（= 检测输入宽，不能改）
    parameter WIN_H    = 96,           // 窗口高
    parameter NX       = 3,            // 横向窗口数
    parameter NY       = 3,            // 纵向窗口数
    parameter SX       = (FW - WIN_W) / (NX - 1),   // 横向步长（3×3 ⇒ 96）
    parameter SY       = (FH - WIN_H) / (NY - 1),   // 纵向步长（3×3 ⇒ 72）
    parameter NFRAME   = 4,            // 缓冲里存几帧
    parameter DW       = 8,            // ROM 位宽：8 = 灰度（R=G=B），24 = RGB
    parameter STRIDE   = FW * FH,      // 每帧在 ROM 里占的字数（帧首尾相接）
    parameter HOLD_CYC = 200_000,      // 每窗停留拍数（等 CPU 解码；go 可提前打断）
    parameter MEM_FILE = "rtl/accel/mem/slidewin.mem",
    parameter BANKED_ROM = 0
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        go,             // 单拍：进下一窗（可接 det 的 done）
    // ★ 随机定位：直接载入第 win_sel 个窗（当前帧）。
    //   为什么需要：CPU 要先把 9 个窗全跑一遍才知道哪个窗里的脸最大，
    //   然后必须**回到那个窗**才能从 FB 裁 ROI（cnn_roi 读的是 FB，
    //   不是 320×240 缓冲）。只靠 go 逐个轮要转 9 次。
    //   另外每帧开头也靠 seek(0) 回到第 0 窗。
    input  wire        seek,
    input  wire [3:0]  win_sel,
    // ★★ 任意原点窗（seek 时二选一：CTRL[9]=0 用 win_sel，=1 用 win_x/win_y）
    //   为什么需要：ROI 只能从当前窗的 FB 裁，而 3×3 固定窗会把
    //   "贴在窗边的脸"切掉几行 —— 实测 32×35 被切成 32×32 后
    //   cos 从 0.538 崩到 0.103（识别器对长宽比极敏感）。
    //   把窗口对准脸中心就能保证 1.2 倍外扩完全落在窗内。
    input  wire        seek_xy,
    input  wire [8:0]  win_x,
    input  wire [8:0]  win_y,
    // ★ seek 时同时指定**帧号**。
    //   为什么必须有：frame_idx 只在「最后一窗 + go」时才前进，
    //   而 CPU 全程用 seek 定位 ⇒ frame_idx 永远停在 0，ROM 一直播第 0 帧。
    //   （这个 bug 不会报错，只是"视频不动"。）
    input  wire [7:0]  frame_sel,
    output reg         fw_we,
    output reg  [7:0]  fw_x,
    output reg  [7:0]  fw_y,
    output wire [23:0] fw_d,
    output reg  [3:0]  win_idx,        // 当前窗口号 0..NWIN-1
    output reg  [7:0]  frame_idx,      // 当前帧号
    output reg         win_done,       // 一窗灌完的单拍脉冲
    output reg         frame_done      // 整帧（NWIN 窗）灌完的单拍脉冲
);

    localparam NPIX  = WIN_W * WIN_H;      // 每窗像素数
    localparam NWIN  = NX * NY;            // 每帧窗口数
    localparam NTOT  = NFRAME * STRIDE;    // ROM 总字数
    localparam AW    = 20;                 // 地址位宽（够 6 帧 × 76800）

    //-----------------------------------------------------------------
    // 帧缓冲 ROM（同步读 ⇒ 可落 BRAM）
    //-----------------------------------------------------------------
    // ROM storage and output registers are defined alongside the read port.

    //-----------------------------------------------------------------
    // 窗口偏移表：woff = y0*FW + x0（常数 mux，不产生乘法器）
    //-----------------------------------------------------------------
    reg [AW-1:0] woff;        // 当前窗
    reg [AW-1:0] woff_nxt;    // 下一窗（win_idx+1；最后一个窗回绕成 0）
    reg [AW-1:0] woff_sel;    // seek 目标窗
    integer wi;
    always @(*) begin
        woff     = {AW{1'b0}};
        woff_nxt = {AW{1'b0}};
        woff_sel = {AW{1'b0}};
        for (wi = 0; wi < NWIN; wi = wi + 1) begin
            if (win_idx == wi)
                woff = (((wi / NX) * SY) * FW) + ((wi % NX) * SX);
            if (win_idx + 1 == wi)
                woff_nxt = (((wi / NX) * SY) * FW) + ((wi % NX) * SX);
            if (win_sel == wi)
                woff_sel = (((wi / NX) * SY) * FW) + ((wi % NX) * SX);
        end
    end

    //-----------------------------------------------------------------
    // 像素地址
    //-----------------------------------------------------------------
    reg [AW-1:0] base;                 // 当前帧起始
    reg [AW-1:0] paddr;                // 当前要读的像素地址（与 xc/yc 同拍）
    reg [AW-1:0] raddr;                // 送 RAM 的地址（打一拍）
    wire [DW-1:0] frame_pixel;
    always @(posedge clk)raddr<=paddr;
    generate if(BANKED_ROM && DW==8 && NTOT==307200 &&
                MEM_FILE=="rtl/accel/mem/slidewin.mem")begin:g_banked
        cnn_slidewin_rom4_banked #(.AW(AW)) u_rom(.clk(clk),.address(raddr),.pixel(frame_pixel));
    end else begin:g_original
        reg[DW-1:0]rom[0:NTOT-1];
        initial $readmemh(MEM_FILE,rom);
        reg[DW-1:0]read_s1,read_s2;
        always @(posedge clk)begin read_s1<=rom[raddr];read_s2<=read_s1;end
        assign frame_pixel=read_s2;
    end endgenerate

    wire [AW-1:0] base_nxt = (base + STRIDE >= NTOT) ? {AW{1'b0}} : base + STRIDE;

    //-----------------------------------------------------------------
    // 窗口内像素游标
    //-----------------------------------------------------------------
    reg [15:0] pc;                     // 窗内像素号 0..NPIX-1
    reg [7:0]  xc, yc;

    reg        loading;                // 1 = 正在灌这一窗
    reg [23:0] hcnt;                   // 停留计数
    reg        lp1, lp2, lp3;          // last_pix 的 3 级延迟（对齐写落盘）
    reg        lw_q;                   // 窗口结束时锁存 last_win

    wire last_pix = loading && (pc == NPIX - 1);
    wire last_win = (win_idx == NWIN - 1);

    always @(posedge clk) begin
        if (!rst_n) begin
            base <= {AW{1'b0}}; paddr <= {AW{1'b0}};
            pc <= 16'd0; xc <= 8'd0; yc <= 8'd0;
            loading <= 1'b1; hcnt <= 24'd0;
            win_idx <= 4'd0; frame_idx <= 8'd0;
            win_done <= 1'b0; frame_done <= 1'b0;
            lp1 <= 1'b0; lp2 <= 1'b0; lp3 <= 1'b0; lw_q <= 1'b0;
        end else begin
            // ★ last_win 在窗口结束时锁存：win_idx 下一窗就变，而 done 要晚 4 拍
            if (last_pix) lw_q <= last_win;
            win_done   <= lp3;              // 每窗都给（含最后一窗）
            frame_done <= lp3 && lw_q;      // 最后一窗额外给 frame_done
            lp1 <= last_pix;
            lp2 <= lp1;
            lp3 <= lp2;

            if (loading) begin
                // ---- 地址推进：行末跳到下一行同列 ----
                if (xc == WIN_W - 8'd1)
                    paddr <= paddr + (FW - WIN_W + 1);
                else
                    paddr <= paddr + 1;

                if (last_pix) begin
                    pc      <= 16'd0;
                    xc      <= 8'd0;
                    yc      <= 8'd0;
                    loading <= 1'b0;
                    hcnt    <= 24'd0;
                end else begin
                    pc <= pc + 16'd1;
                    if (xc == WIN_W - 8'd1) begin
                        xc <= 8'd0;
                        yc <= (yc == WIN_H - 8'd1) ? 8'd0 : yc + 8'd1;
                    end else xc <= xc + 8'd1;
                end
            end else begin
                // ---- 停留：等 go 或超时 ----
                hcnt <= hcnt + 24'd1;
                if (seek && seek_xy) begin
                    // 任意原点（给 ROI 用）：直接从 (win_x, win_y) 开始装一窗
                    loading <= 1'b1;
                    hcnt    <= 24'd0;
                    win_idx   <= 4'd15;                // 标记"不是 3×3 网格里的窗"
                    base      <= frame_sel * STRIDE;
                    frame_idx <= frame_sel;
                    paddr     <= frame_sel * STRIDE + (win_y * FW) + win_x;
                end else if (seek) begin
                    // 直接载入目标窗（不动帧号）
                    loading <= 1'b1;
                    hcnt    <= 24'd0;
                    win_idx   <= win_sel;
                    base      <= frame_sel * STRIDE;
                    frame_idx <= frame_sel;
                    paddr     <= frame_sel * STRIDE + woff_sel;
                end else if (go || ((HOLD_CYC != 0) && (hcnt >= HOLD_CYC))) begin
                    loading <= 1'b1;
                    hcnt    <= 24'd0;
                    if (last_win) begin
                        // 换帧：窗 0 偏移 = 0
                        win_idx   <= 4'd0;
                        base      <= base_nxt;
                        paddr     <= base_nxt;
                        frame_idx <= (frame_idx == NFRAME - 1) ? 8'd0
                                                              : frame_idx + 8'd1;
                    end else begin
                        win_idx <= win_idx + 4'd1;
                        paddr   <= base + woff_nxt;
                    end
                end
            end
        end
    end

    //-----------------------------------------------------------------
    // 输出级：与数据链（4 级）对齐 ⇒ 坐标延 3 级
    //-----------------------------------------------------------------
    reg        v1, v2;
    reg [7:0]  x1, x2, y1, y2;

    always @(posedge clk) begin
        if (!rst_n) begin
            v1 <= 1'b0; v2 <= 1'b0;
            x1 <= 8'd0; x2 <= 8'd0; y1 <= 8'd0; y2 <= 8'd0;
        end else begin
            v1 <= loading;
            x1 <= xc;  y1 <= yc;
            v2 <= v1;
            x2 <= x1;  y2 <= y1;
        end
    end

    // 灰度时三通道复制（检测网络吃 24b RGB，灰度等价于 R=G=B）
    generate
        if (DW == 24) begin : g_rgb
            assign fw_d=frame_pixel;
            always @(posedge clk) begin
                fw_we <= v2;
                fw_x  <= x2;
                fw_y  <= y2;

            end
        end else begin : g_gray
            assign fw_d={frame_pixel,frame_pixel,frame_pixel};
            always @(posedge clk) begin
                fw_we <= v2;
                fw_x  <= x2;
                fw_y  <= y2;

            end
        end
    endgenerate

endmodule

// Final packet boundary after ROM-bank selection. Every visible field and
// done pulse advances together, including reset/start/seek transitions.
module cnn_slidewin #(
    parameter FW=320,FH=240,WIN_W=128,WIN_H=96,NX=3,NY=3,
    parameter SX=(FW-WIN_W)/(NX-1),SY=(FH-WIN_H)/(NY-1),
    parameter NFRAME=4,DW=8,STRIDE=FW*FH,HOLD_CYC=200_000,
    parameter MEM_FILE="rtl/accel/mem/slidewin.mem",BANKED_ROM=0
)(
    input wire clk,rst_n,go,seek,input wire[3:0]win_sel,
    input wire seek_xy,input wire[8:0]win_x,win_y,input wire[7:0]frame_sel,
    output reg fw_we,output reg[7:0]fw_x,fw_y,
    output reg[23:0]fw_d /* synthesis syn_preserve=1 */,
    output reg[3:0]win_idx,output reg[7:0]frame_idx,
    output reg win_done,frame_done
);
    wire we_s,wd_s,fd_s;wire[7:0]x_s,y_s,fi_s;
    wire[23:0]d_s;wire[3:0]wi_s;
    cnn_slidewin_source #(.FW(FW),.FH(FH),.WIN_W(WIN_W),.WIN_H(WIN_H),
        .NX(NX),.NY(NY),.SX(SX),.SY(SY),.NFRAME(NFRAME),.DW(DW),
        .STRIDE(STRIDE),.HOLD_CYC(HOLD_CYC),.MEM_FILE(MEM_FILE),.BANKED_ROM(BANKED_ROM)) u_source(
        .clk(clk),.rst_n(rst_n),.go(go),.seek(seek),.win_sel(win_sel),
        .seek_xy(seek_xy),.win_x(win_x),.win_y(win_y),.frame_sel(frame_sel),
        .fw_we(we_s),.fw_x(x_s),.fw_y(y_s),.fw_d(d_s),
        .win_idx(wi_s),.frame_idx(fi_s),.win_done(wd_s),.frame_done(fd_s));
    always @(posedge clk)begin
        fw_d<=d_s;fw_x<=x_s;fw_y<=y_s;
        win_idx<=wi_s;frame_idx<=fi_s;
        if(!rst_n)begin fw_we<=0;win_done<=0;frame_done<=0;end
        else begin fw_we<=we_s;win_done<=wd_s;frame_done<=fd_s;end
    end
endmodule
