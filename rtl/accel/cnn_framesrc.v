//=====================================================================
// cnn_framesrc.v : **预存帧播放器**（无相机时的视频源）
//
// 场景：比赛现场没有相机，只能把帧预先烧进片上 ROM，循环播放当"视频"。
//   ⇒ 本模块替代「CPU 通过 MMIO 逐像素写帧」那条路，直接驱动 cnn_soc 的
//     `fw_we/fw_x/fw_y/fw_d` 口（一像素一拍，24b RGB，R 在高字节）。
//
// ★ 为什么几乎不花钱：
//   整条流水线 202,023 拍/帧 = 1.312 ms，而 30 fps 的预算是 33.3 ms
//   ⇒ 占空比只有 3.9%。所以**不需要相机接口、不需要双缓冲、不需要降采样**，
//     一个 ROM + 一个计数器就够。资源只有 ROM 本身：
//       24b × 12288 像素 = 36,864 B/帧 = **9 DRM/帧**
//       8 帧 = 72 DRM（总 DRM 222 → 294/415 = 71%）
//       12 帧 = 108 DRM（330/415 = 80%）
//
// ★ 换帧时机：两种都能用
//   · 自由跑：`HOLD_CYC` 拍后自动换下一帧（演示用，不依赖 CPU）
//   · 握手：外部给 `go` 单拍脉冲（CPU 在 net_done 后给），处理完立刻换帧
//
// ★ ROM 必须是**同步读**（1 拍延迟）才能落 BRAM，所以内部有一级流水：
//     raddr 寄存 → rom_q 寄存 → 输出级把 fw_we/fw_x/fw_y 一起延 2 拍对齐
//
// 帧数据格式（与 `gen/gen_fullflow.py` 的 flow_frame.hex 一致）：
//   每行一个 24b 字 `%06X` = {R[7:0], G[7:0], B[7:0]}，按 y*FW+x 顺序，
//   帧与帧首尾相接（共 NFRAME*FW*FH 行）。
//=====================================================================
`timescale 1ns/1ps

module cnn_framesrc #(
    parameter FW       = 128,
    parameter FH       = 96,
    parameter NFRAME   = 8,
    // ★ 每帧在 ROM 里占的**对齐跨度**。NPIX=12,288 不是 2 的幂，
    //   直接拼成 98,304 深会让 PDS 的 DRAM 级联报 DRC-0007（COUTA/CINA），
    //   而且只映射到 25/72 DRM。按 16,384 对齐后级联干净。
    parameter STRIDE   = 16384,
    parameter HOLD_CYC = 3_000_000,   // 自由跑时每帧停留拍数（154MHz ⇒ ~19.5ms ⇒ 51fps）
    parameter MEM_FILE = "rtl/accel/mem/framesrc.mem"
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       go,             // 单拍脉冲：提前换下一帧（可接 net_done）
    output reg        fw_we,
    output reg  [7:0] fw_x,
    output reg  [7:0] fw_y,
    output reg  [23:0] fw_d,
    output reg  [7:0] frame_idx,      // 当前帧号（给 LED/调试用）
    output reg        frame_done      // 一帧灌完的单拍脉冲（可作 net_go 的触发）
);

    localparam NPIX  = FW * FH;
    localparam NTOT  = NFRAME * STRIDE;   // 含每帧尾部填充

    //-----------------------------------------------------------------
    // 帧 ROM（同步读 ⇒ 可落 BRAM）
    //-----------------------------------------------------------------
    reg [23:0] rom [0:NTOT-1];
    initial begin
        $readmemh(MEM_FILE, rom);
    end

    reg [17:0] base;                  // 当前帧在 ROM 里的起始地址（累加 STRIDE，不用乘法）
    reg [15:0] pc;                    // 帧内像素号 0..NPIX-1
    reg [17:0] raddr;
    reg [23:0] rq;

    // 像素坐标：用两个小计数器，避免综合出除法/取模
    reg [7:0] xc, yc;

    always @(posedge clk) begin
        raddr <= base + pc;
        rq    <= rom[raddr];
    end

    //-----------------------------------------------------------------
    // 换帧控制
    //-----------------------------------------------------------------
    reg        loading;               // 1 = 正在灌这一帧
    reg [23:0] hcnt;                  // 停留计数
    reg        lp1, lp2, lp3;         // last_pix 的 3 级延迟（对齐写落盘）

    wire last_pix = loading && (pc == NPIX - 1);

    always @(posedge clk) begin
        if (!rst_n) begin
            base <= 18'd0; pc <= 16'd0;
            xc <= 8'd0; yc <= 8'd0;
            loading <= 1'b1; hcnt <= 24'd0;
            frame_idx <= 8'd0; frame_done <= 1'b0;
            lp1 <= 1'b0; lp2 <= 1'b0; lp3 <= 1'b0;
        end else begin
            frame_done <= lp3;
            lp1 <= last_pix;
            lp2 <= lp1;
            lp3 <= lp2;

            if (loading) begin
                // ---- 灌帧：每拍一个像素 ----
                if (last_pix) begin
                    pc      <= 16'd0;
                    xc      <= 8'd0;
                    yc      <= 8'd0;
                    loading <= 1'b0;
                    hcnt    <= 24'd0;
                end else begin
                    pc <= pc + 16'd1;
                    if (xc == FW - 8'd1) begin
                        xc <= 8'd0;
                        yc <= (yc == FH - 8'd1) ? 8'd0 : yc + 8'd1;
                    end else xc <= xc + 8'd1;
                end
            end else begin
                // ---- 停留：等 go 或超时后换下一帧 ----
                hcnt <= hcnt + 24'd1;
                if (go || (hcnt >= HOLD_CYC)) begin
                    base <= (base + STRIDE >= NTOT) ? 18'd0 : base + STRIDE;
                    frame_idx <= (frame_idx == NFRAME - 1) ? 8'd0 : frame_idx + 8'd1;
                    loading <= 1'b1;
                    hcnt <= 24'd0;
                end
            end
        end
    end

    //-----------------------------------------------------------------
    // 输出级：与 rom 同步读的 1 拍延迟对齐（raddr 寄存 + rq 寄存 = 2 拍）
    //   ★ 四个信号必须**同拍对齐**，否则写进去的像素错位（FB 索引与数据不配）
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

    always @(posedge clk) begin
        fw_we <= v2;
        fw_x  <= x2;
        fw_y  <= y2;
        fw_d  <= rq;
    end

endmodule
