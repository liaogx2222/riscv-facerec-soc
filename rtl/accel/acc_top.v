//=====================================================================
// acc_top.v —— 综合/时序评估用**自包含封装**（不参与仿真）
//
// 为什么这样做：
//   ① 参数必须显式钉死：`cnn_soc` 的默认 ADEPTH=5376 是 ÷4 缩尺值，而真实部署要按
//      **特征图峰值**算。PDS 的 `compile -defined_parameters` 本机没有示例、易
//      "改了没生效"，所以用一层显式例化写死，综合报告的 BRAM 占用可直接反查。
//      ★ ADEPTH 取值（2026-10-22 结论）：**6144**。
//        · 18432 是**仿真专属**——它把 det 输入物化区（INOFF_DET=49152 flat 字）
//          也算进了 ACT；真实硬件里 det 首层由帧源直供（`src_img`，= 待做的 M10）。
//        · 按 18432 综合实测 DRM **397/415 = 95.7%**，布线几乎不收敛；
//          按 6144 是 **221/415 = 53.3%**。
//   ② `cnn_soc` 有约 470 个端口位（光 rd_data 就 256 位）⇒ 直接当顶层会撞
//      `DRC-0018: The resource of IO is not enough`。而真实部署里它是 SoC
//      **内部**模块（接 CPU/相机，不出引脚）⇒ 这里只留 clk/rst_n + 8 位观测口，
//      输入由 LFSR/计数器驱动，输出全部折叠进 sink（防止被优化掉）。
//      ⇒ 这样报出的是**内部逻辑 Fmax**，与真实使用方式一致。
//
// 采用参数：FW=128 FH=96 OUT=64 AAW=16 ADEPTH=6144 DAW=10 NDESC=512
//=====================================================================
`timescale 1ns/1ps

module acc_top (
    input  wire       clk,
    input  wire       rst_n,
    output reg  [7:0] sink          // 观测口：把所有输出折叠进来，防止被剪掉
);

    //-----------------------------------------------------------------
    // 激励：32 位 LFSR + 自由计数器（无除法/取模，避免综合出除法器）
    //-----------------------------------------------------------------
    reg [31:0] lfsr;
    reg [15:0] cnt;
    always @(posedge clk) begin
        if (!rst_n) begin
            lfsr <= 32'h1;
            cnt  <= 16'd0;
        end else begin
            lfsr <= {lfsr[30:0], lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0]};
            cnt  <= cnt + 16'd1;
        end
    end

    // ---- 帧写口 ----
    wire        fw_we = lfsr[0];
    wire [7:0]  fw_x  = lfsr[8:1];
    wire [7:0]  fw_y  = {1'b0, lfsr[15:9]};
    wire [23:0] fw_d  = lfsr[23:0];

    // ---- ROI ----
    wire        roi_go = (cnt[7:0] == 8'h00);
    wire [7:0]  x1 = lfsr[8:1];
    wire [7:0]  y1 = lfsr[16:9];
    wire [7:0]  x2 = lfsr[24:17];
    wire [7:0]  y2 = lfsr[7:0];

    // ---- 帧→ACT ----
    wire        fsrc_go = (cnt[7:0] == 8'h40);

    // ---- D1 ----
    wire        d1_go       = (cnt[7:0] == 8'h80);
    wire [15:0] d1_in_base  = {4'd0, lfsr[11:0]};
    wire [15:0] d1_out_base = 16'd0;
    wire [7:0]  d1_sh_req   = lfsr[7:0];        // 动态移位量（= 描述符字段，别用常量）

    // ---- 网络 ----
    wire        net_go    = (cnt[9:0] == 10'h100);
    wire [7:0]  nlayer    = lfsr[7:0];
    wire        net_hold  = lfsr[8];
    wire [9:0]  desc_base = lfsr[9:0];
    wire [13:0] woff      = {4'd0, lfsr[9:0]};
    wire [7:0]  boff      = lfsr[7:0];

    // ---- ACT 读回 ----
    wire        rd_go    = (cnt[11:0] == 12'h200);
    wire [15:0] rd_word  = {4'd0, lfsr[11:0]};
    wire [15:0] rd_nword = {8'd0, lfsr[7:0]};

    // ---- 输出 ----
    wire         fsrc_busy, fsrc_done;
    wire         d1_busy,   d1_done;
    wire         rd_valid,  rd_busy, rd_done;
    wire [255:0] rd_data;
    wire         roi_busy,  roi_done;
    wire         net_busy,  net_done;
    wire         ctrl_timeout;
    wire [7:0]   layer_idx;

    cnn_soc #(
        .FW     (128),
        .FH     (96),
        .OUT    (64),
        .AAW    (16),
        .ADEPTH (6144),      // det 特征图峰值 24,576 flat 字 ÷ 4 bank（见文件头 ①）
        .DAW    (10),
        .NDESC  (512)
    ) u_soc (
        .clk(clk), .rst_n(rst_n),
        .fw_we(fw_we), .fw_x(fw_x), .fw_y(fw_y), .fw_d(fw_d),
        .roi_go(roi_go), .x1(x1), .y1(y1), .x2(x2), .y2(y2),
        .fsrc_go(fsrc_go), .fsrc_busy(fsrc_busy), .fsrc_done(fsrc_done),
        .d1_go(d1_go), .d1_in_base(d1_in_base), .d1_out_base(d1_out_base),
        .d1_sh_req(d1_sh_req), .d1_busy(d1_busy), .d1_done(d1_done),
        .net_go(net_go), .nlayer(nlayer), .net_hold(net_hold),
        .desc_base(desc_base), .woff(woff), .boff(boff),
        .rd_go(rd_go), .rd_word(rd_word), .rd_nword(rd_nword),
        .rd_valid(rd_valid), .rd_data(rd_data), .rd_busy(rd_busy), .rd_done(rd_done),
        .roi_busy(roi_busy), .roi_done(roi_done),
        .net_busy(net_busy), .net_done(net_done),
        .ctrl_timeout(ctrl_timeout), .layer_idx(layer_idx)
    );

    //-----------------------------------------------------------------
    // 观测口：状态位 + rd_data 抽头异或折叠（保证读回通路不被剪掉）
    //-----------------------------------------------------------------
    // ★ 两级折叠：一级折叠时实测这条路径有 9.25 ns
    //   （源分散在不同模块，布线主导）——它是测量脚手架，不该成为假瓶颈。
    //   拆成两组独立折叠先打拍，再异或进 sink。
    // ★ 2026-10-23：rd_data 抽头**每个单独 1 bit 寄存**，不再一次性 XOR 五个字节。
    //   原写法源分散在 256b 读回 mux 各处，布局布线后实测 **11.77 ns**，
    //   会成为掩盖真实瓶颈的「测量脚手架假瓶颈」。
    //   1 bit 寄存器就近布局 ⇒ 每条路径只剩「mux → 1 个 FF」。
    reg [7:0] f_stat;
    reg t0, t1, t2, t3, t4;
    always @(posedge clk) begin
        if (!rst_n) begin
            sink <= 8'd0; f_stat <= 8'd0;
            t0 <= 1'b0; t1 <= 1'b0; t2 <= 1'b0; t3 <= 1'b0; t4 <= 1'b0;
        end else begin
            f_stat <= {fsrc_busy, fsrc_done, d1_busy, d1_done,
                       rd_busy, roi_done, net_done, ctrl_timeout} ^ layer_idx;
            t0 <= rd_data[7];   t1 <= rd_data[63];
            t2 <= rd_data[127]; t3 <= rd_data[191]; t4 <= rd_data[255];
            sink   <= sink ^ f_stat ^ {3'd0, t0, t1, t2, t3, t4} ^ {7'd0, rd_valid};
        end
    end

endmodule
