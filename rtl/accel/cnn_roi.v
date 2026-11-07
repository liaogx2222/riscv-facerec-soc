//=====================================================================
// cnn_roi.v : 硬件 ROI 裁剪 + 整数双线性缩放（M14，2026-10-20 新增）
//
// ★ 语义**逐位复刻** model/data_rec.py::roi_crop_resize_int（冻结语义）：
//     sx(ox) = 256*x1 + 2*sw*(2*ox+1) - 128          （align_corners=False，Q8）
//     sx     = clip(sx, x1*256, (x2-1)*256)          ← 夹到**裁剪框**边界
//     x0 = sx>>8 ; fx = sx&255 ; x0b = min(x0+1, x2-1)
//     sy/y0/fy/y0b 同理
//     v = ( a00*(256-fy)*(256-fx) + a01*(256-fy)*fx
//         + a10*fy*(256-fx)     + a11*fy*fx  + 32768 ) >> 16
//     out = clip(v, 0, 255)
//
// ★ 为什么 out=64 时没有除法（这是能做成硬件的前提）：
//     Python 是 `(2*o+1)*sw*128 // out`；out=64 ⇒ 128/64 = 2，
//     且 (2*o+1)*sw*128 恒被 64 整除 ⇒ 等价于 `(2*o+1)*sw*2`（**无舍入**）。
//     ⇒ 硬件只需「初值 + 每步 +4*sw」的累加器。
//
// ★ 为什么夹取必须用**裁剪框**边界而不是整图边界：
//     align_corners=False 的映射会略微超出 [x1,x2)，cv2 在裁剪框处夹取。
//     夹到整图边界会在 x1>0 时读到框外像素（实测差 82 LSB，见 data_rec 注释）。
//
// ★ 结构（每拍 1 个输出像素，3 通道并行）：
//     帧存储 ──(1 像素/拍)──> 2 行缓冲（各 FW×24b，环形选择 sel0）
//                              └─> 双线性插值（水平 2 级 → 垂直 1 级）
//     行缓冲**跨输出行复用**：y0 单调不减 ⇒ 相邻输出行常只前进 0/1 行
//     ⇒ 每张脸约 4096 + sh*sw 拍（不是 4096*4）。
//
// ⚠ R3：**逐脸串行**，不给多张脸各配缓冲（单脸 64×64 RGB8 = 12 KiB）。
//=====================================================================
`timescale 1ns/1ps

module cnn_roi #(
    parameter OUT = 64,          // 输出边长（★ 必须是 2 的幂：靠 128/OUT 消除法）
    parameter FW  = 128,         // 帧宽
    parameter FH  = 96,          // 帧高
    parameter AW  = 8,           // 坐标位宽（FW/FH ≤ 256）
    parameter WW  = 7,           // 行缓冲地址位宽（FW=128 ⇒ 7）
    parameter WP4 = 4            // ★ = in_wpg（每像素的 ACT 字数；ic=3 ⇒ 4）
)(
    input  wire              clk,
    input  wire              rst_n,
    input  wire              start,        // 单拍脉冲：开始处理一张脸
    input  wire [AW-1:0]     x1, y1,       // 裁剪框（右开区间 [x1,x2) × [y1,y2)）
    input  wire [AW-1:0]     x2, y2,

    // ---- 帧读口（同步读，1 像素/拍；data 次拍有效）----
    output wire [AW-1:0]     fr_x,         // ★ 组合输出：必须与 kcnt 同拍
    output wire [AW-1:0]     fr_y,         //   （曾做成寄存器 ⇒ 地址比数据晚一拍，踩过）
    output wire              fr_en,
    input  wire [23:0]       fr_data,      // {R[23:16], G[15:8], B[7:0]}

    // ---- 输出（每拍 1 像素，3 通道并行）----
    output reg               o_valid,
    output reg  [15:0]       o_pix,        // 0 .. OUT*OUT-1
    output reg  [23:0]       o_rgb,        // 插值后的 uint8 RGB
    output reg  [23:0]       o_aq,         // ★ 量化后：clip(rgb-127,-128,127) 的 int8×3

    // ---- ACT 写入（每像素 WP4 拍：b0=ch0..7，b1..b3=0）----
    //   依据：rec 输入 ic=3 ⇒ conv 的 n_ig=1 且权重在 ch≥3 处为 0
    //   ⇒ word1..3（ch8..31）**无需写**（读不到 / 权重为 0），省 3/4 写带宽。
    output wire [3:0]        aw_we,        // ★ 1 拍写满 4 个字 ⇒ 4 个 bank 全使能
    output wire [15:0]       aw_addr,      // bank 索引 = 像素号
    output wire [255:0]      aw_data,      // {bank3,bank2,bank1,bank0}：bank0 = {ch7..ch0}

    output reg               busy,
    output reg               done          // 单拍脉冲
);

    // ★ 映射系数：sx = 256*x1 + (2o+1)*sw*(128/OUT) - 128
    //   Python 是 `(2*o+1)*sw*128//out`；OUT 为 2 的幂且整除 128 时 128/OUT 是整数
    //   ⇒ 无舍入。原来把 (128/OUT) 写死成 2（只对 OUT=64 成立）⇒ OUT=16 时整幅错。
    localparam K = 128 / OUT;

    localparam S_IDLE = 3'd0, S_ROW = 3'd1, S_FILL0 = 3'd2, S_FILL1 = 3'd3,
               S_EMIT = 3'd4, S_DRN = 3'd6, S_DONE = 3'd5,
               S_YCLAMP=4'd7,S_SETUP=4'd8,S_SEED=4'd9,S_YNEIGHBOR=4'd10;

    reg [3:0]  st;
    reg [AW-1:0] bx1, by1, bx2, by2;      // 已夹取的框
    reg [AW-1:0] sw, sh;                   // 框宽高
    reg [AW-1:0] oy;                       // 输出行
    reg [7:0]  ox;                         // 输出列（OUT ≤ 256）
    reg [AW-1:0] kcnt;                     // 行缓冲填充游标

    // ---- 坐标累加器（sx/sy）----
    reg signed [17:0] sx_acc, sy_acc;

    reg signed [17:0] sx_origin_q,sy_origin_q,sx_seed_q,sy_clip_q;
    reg [17:0] sx_step_q,sy_step_q;
    reg [AW-1:0] bx_last_q,by_last_q;
    reg p0_v,p2d_v,p2e_v;
    reg [17:0] p0_sx;
    reg [15:0] p0_pix,p2d_pix,p2e_pix;
    reg [7:0] p0_fy;
    reg p0_sel0;

    // ---- 行缓冲 ----
    reg [23:0] lb0 [0:FW-1];
    reg [23:0] lb1 [0:FW-1];
    reg [AW-1:0] tag0, tag1;
    reg          vld0, vld1;
    reg          sel0;                     // 0 ⇒ 行 0 在 lb0；1 ⇒ 行 0 在 lb1
    reg          fill_sel;                 // 当前填充目标：0 ⇒ lb0
    reg          fill1_req;                // ★ S_ROW 时定下的「行 1 需填充」（寄存，
                                           //   否则在 S_FILL0 里用未更新的 tag 重算会错）
    reg [2:0]    drn;                      // S_DRN 排空计数
    reg          last_row;                 // 行末时记下「是否最后一行」

    // 框夹取（组合，与 Python 同序）：x1=clip(x1,0,W-1); x2=clip(x2,x1+1,W)
    //   ⚠ 必须用**夹后**的 x1/x2 去算 sw/sh 与 sx 初值 —— 用原始值会在越界框上出错
    wire [AW-1:0] cx1 = (x1 > FW - 1) ? (FW - 1) : x1;
    wire [AW-1:0] cx2 = (x2 < cx1 + 8'd1) ? (cx1 + 8'd1) :
                        (x2 > FW) ? FW[AW-1:0] : x2;
    wire [AW-1:0] cy1 = (y1 > FH - 1) ? (FH - 1) : y1;
    wire [AW-1:0] cy2 = (y2 < cy1 + 8'd1) ? (cy1 + 8'd1) :
                        (y2 > FH) ? FH[AW-1:0] : y2;
    wire [AW-1:0] csw = cx2 - cx1;
    wire [AW-1:0] csh = cy2 - cy1;

    // 帧读口（组合）：地址与 fr_en 必须**同拍**给出，data 次拍有效
    assign fr_en = (st == S_FILL0) || (st == S_FILL1);
    assign fr_x  = bx1 + kcnt;
    assign fr_y  = (st == S_FILL0) ? y0 : y0b;

    // ---------------------------------------------------------------
    // 组合：当前输出行的 y0 / y0b
    // ---------------------------------------------------------------
    reg signed [17:0] syhi;
    reg signed [17:0] sylo;
    wire signed [17:0] sy_c = (sy_acc < sylo) ? sylo :
                              (sy_acc > syhi) ? syhi : sy_acc;
    reg [AW-1:0] y0;
    reg [7:0] fy;
    reg [AW-1:0] y0b;

    // 复用/换缓冲判定
    wire tag_sel0_ok = (sel0 ? vld1 : vld0) && ((sel0 ? tag1 : tag0) == y0);
    wire tag_alt_ok  = (sel0 ? vld0 : vld1) && ((sel0 ? tag0 : tag1) == y0);
    wire do_swap     = !tag_sel0_ok && tag_alt_ok;
    wire nsel0       = sel0 ^ do_swap;
    wire need_fill0  = !tag_sel0_ok && !do_swap;
    // 行 1 的目标缓冲 = nsel0^1
    wire t1_ok = (nsel0 ? vld0 : vld1) && ((nsel0 ? tag0 : tag1) == y0b);
    wire need_fill1 = !t1_ok;

    // ---------------------------------------------------------------
    // 组合：当前输出列的 x0 / x0b / fx
    // ---------------------------------------------------------------
    reg signed [17:0] sxhi;
    reg signed [17:0] sxlo;
    wire signed [17:0] sx_c = (sx_acc < sxlo) ? sxlo :
                              (sx_acc > sxhi) ? sxhi : sx_acc;
    wire [AW-1:0] x0  = sx_c[AW+7:8];
    wire [7:0]    fx  = sx_c[7:0];
    wire [AW-1:0] x0b = (x0 + 8'd1 > bx2 - 8'd1) ? (bx2 - 8'd1) : (x0 + 8'd1);

    //-----------------------------------------------------------------
    // ★★ 输出流水（3 级附加，S_EMIT 仍 1 拍/像素）
    //
    //   原实现是**一拍走完**：
    //     sx_acc → 夹取比较 → x0 → x0b 比较 → 行缓冲读 → 取字节
    //     → 双线性（4 个级联 APM）→ q8 → ACT 写地址
    //   PDS 布局布线后实测 **22.27 ns**（16 级逻辑，逻辑 11.0 / 布线 7.5 ns）
    //   ，是整个加速器唯一的瓶颈（第 2〜6 名都是 3.6 ns 的 cnn_d1）。
    //
    //   切法（每级都 < 6.7 ns）：
    //     级 1（S_EMIT）：算 x0/x0b/fx/fy + 锁存 pix、sel0
    //     级 2：用级 1 的坐标读行缓冲（同步打拍）
    //     级 3：双线性（cnn_bl ×3）结果打拍
    //     级 4：q8 + 写 ACT（组合输出，与 p3_* 同拍）
    //   ⚠ 行末必须等 3 拍排空（S_DRN）才能进 S_ROW/S_FILL：
    //     否则 FILL 会覆盖级 2 还没读完的行缓冲。
    //-----------------------------------------------------------------
    reg             p1_v, p2_v, p2b_v, p2c_v, p3_v;
    reg  [15:0]     p1_pix, p2_pix, p2b_pix, p2c_pix, p3_pix;
    reg  [AW-1:0]   p1_x0, p1_x0b;
    reg  [7:0]      p1_fx, p1_fy, p2_fx, p2_fy;
    reg             p1_sel0;
    reg  [23:0]     p2_r0a, p2_r0b, p2_r1a, p2_r1b;
    reg  [7:0]      p3_vr, p3_vg, p3_vb;

    // 级 2：行缓冲读（索引 = 绝对坐标 - x1；两个缓冲二选一）
    wire [WW-1:0] i00 = (p1_x0  - bx1);
    wire [WW-1:0] i01 = (p1_x0b - bx1);
    wire [23:0] r0a = p1_sel0 ? lb1[i00] : lb0[i00];
    wire [23:0] r0b = p1_sel0 ? lb1[i01] : lb0[i01];
    wire [23:0] r1a = p1_sel0 ? lb0[i00] : lb1[i00];
    wire [23:0] r1b = p1_sel0 ? lb0[i01] : lb1[i01];

    // 三通道双线性
    // 整数双线性插值：**独立子模块 3 次例化**（原为 function 3 处调用）。
    // ★ 为什么不用 function：PDS/Synplify 把「有局部变量且多处调用」的 function
    //   做 static 内联（局部变量全设计共享一份）⇒ 综合后 tie-0（审计规则 [B]）。
    wire [7:0] vr, vg, vb;
    // ★ cnn_bl 内部已打一拍（水平→垂直）：4 个级联 APM 拆成 2+2
    wire bl_v;
    cnn_bl u_bl_r (.clk(clk), .v_i(p2_v), .v_o(bl_v),
                   .a(p2_r0a[23:16]), .b(p2_r0b[23:16]), .c(p2_r1a[23:16]), .d(p2_r1b[23:16]),
                   .fx(p2_fx), .fy(p2_fy), .y(vr));
    cnn_bl u_bl_g (.clk(clk), .v_i(p2_v), .v_o(),
                   .a(p2_r0a[15:8]),  .b(p2_r0b[15:8]),  .c(p2_r1a[15:8]),  .d(p2_r1b[15:8]),
                   .fx(p2_fx), .fy(p2_fy), .y(vg));
    cnn_bl u_bl_b (.clk(clk), .v_i(p2_v), .v_o(),
                   .a(p2_r0a[7:0]),   .b(p2_r0b[7:0]),   .c(p2_r1a[7:0]),   .d(p2_r1b[7:0]),
                   .fx(p2_fx), .fy(p2_fy), .y(vb));


    // 量化：a_q = clip(rgb - 127, -128, 127)
    // q8(v) = clip(v-127, -128, 127) 的 8 位补码（= hwnet.quant_input）
    // ★ 纯表达式、无函数局部变量：PDS/Synplify 会把「有局部变量且多处调用」的
    //   function 做 static 内联 ⇒ 综合成 tie-0（审计规则 [B]，静默错值）。
    //   v ∈ [0,254] ⇒ v-127 ∈ [-127,127]，8 位截断即正确；v=255 ⇒ 128 要夹到 127。
    function [7:0] q8;
        input [7:0] p;
        begin
            q8 = (p == 8'd255) ? 8'h7F : (p - 8'd127);
        end
    endfunction

    // ★ 字节序：byte0=R(=ch0) byte1=G byte2=B（与 gen_roi_vec 的
    //   `aq[0]|(aq[1]<<8)|(aq[2]<<16)` 逐位一致；反了会 R/B 互换）
    wire [23:0] v_aq = {q8(vr), q8(vg), q8(vb)};   // {B, G, R} 才是 24 位 View
    wire [23:0] v_aq_w = {q8(vb), q8(vg), q8(vr)}; // byte0=R byte1=G byte2=B
    wire [23:0] v_rgb = {vr, vg, vb};

    // ★ ACT 写口（组合）：由**级 4**（p3_*）驱动，与取数据同拍。
    //   byte0=R byte1=G byte2=B（与 gen_roi_vec 的 aq[0]|(aq[1]<<8)|(aq[2]<<16) 逐位一致）
    wire [23:0] aw_aq = {q8(p3_vb), q8(p3_vg), q8(p3_vr)};
    assign aw_we   = p3_v ? 4'b1111 : 4'd0;
    assign aw_addr = p3_pix;
    assign aw_data = {192'd0, aw_aq};      // 上位 192 位清零 + bank0

    // ---------------------------------------------------------------
    // 时序
    // ---------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            st <= S_IDLE; busy <= 1'b0; done <= 1'b0; o_valid <= 1'b0;
            sxhi<=0;sxlo<=0;syhi<=0;sylo<=0;
            sx_origin_q<=0;sy_origin_q<=0;sx_seed_q<=0;sy_clip_q<=0;
            sx_step_q<=0;sy_step_q<=0;bx_last_q<=0;by_last_q<=0;
            y0<=0;y0b<=0;fy<=0;
            p0_v<=0;p2d_v<=0;p2e_v<=0;p0_sx<=0;p0_pix<=0;
            p2d_pix<=0;p2e_pix<=0;p0_fy<=0;p0_sel0<=0;

            
            o_pix <= 16'd0; o_rgb <= 24'd0; o_aq <= 24'd0;
            sel0 <= 1'b0; vld0 <= 1'b0; vld1 <= 1'b0;
            tag0 <= {AW{1'b0}}; tag1 <= {AW{1'b0}};
            kcnt <= {AW{1'b0}}; oy <= {AW{1'b0}}; ox <= 8'd0;
            sx_acc <= 18'sd0; sy_acc <= 18'sd0; fill_sel <= 1'b0; fill1_req <= 1'b0;
            bx1 <= {AW{1'b0}}; by1 <= {AW{1'b0}}; bx2 <= {AW{1'b0}}; by2 <= {AW{1'b0}};
            sw <= {AW{1'b0}}; sh <= {AW{1'b0}};
            p1_v <= 1'b0; p2_v <= 1'b0; p2b_v <= 1'b0; p2c_v <= 1'b0; p3_v <= 1'b0;
            p1_pix <= 16'd0; p2_pix <= 16'd0; p2b_pix <= 16'd0; p2c_pix <= 16'd0; p3_pix <= 16'd0;
            p1_x0 <= {AW{1'b0}}; p1_x0b <= {AW{1'b0}};
            p1_fx <= 8'd0; p1_fy <= 8'd0; p2_fx <= 8'd0; p2_fy <= 8'd0;
            p1_sel0 <= 1'b0;
            p2_r0a <= 24'd0; p2_r0b <= 24'd0; p2_r1a <= 24'd0; p2_r1b <= 24'd0;
            p3_vr <= 8'd0; p3_vg <= 8'd0; p3_vb <= 8'd0;
            drn <= 2'd0; last_row <= 1'b0;
        end else begin
            done    <= 1'b0;
            // ---- 流水默认推进（S_EMIT 里只置 p1_v，其余各级逐拍右移）----
            p0_v<=0;
            p1_v<=p0_v;
            if(p0_v) begin
                p1_pix<=p0_pix;p1_x0<=p0_sx[AW+7:8];
                p1_x0b<=(p0_sx[AW+7:8]+8'd1>bx_last_q)?bx_last_q:(p0_sx[AW+7:8]+8'd1);
                p1_fx<=p0_sx[7:0];p1_fy<=p0_fy;p1_sel0<=p0_sel0;
            end
            p2d_v<=p2c_v;p2d_pix<=p2c_pix;
            p2e_v<=p2d_v;p2e_pix<=p2d_pix;
            p2_v  <= p1_v;  p2_pix  <= p1_pix;
            p2b_v <= p2_v;  p2b_pix <= p2_pix;      // 与 cnn_bl 第 2 级输出对齐
            p2c_v <= p2b_v; p2c_pix <= p2b_pix;     // 与 cnn_bl 第 3 级输出对齐（run13：内部延迟 2 拍）
            p3_v  <= p2e_v; p3_pix  <= p2e_pix;
            p2_fx <= p1_fx; p2_fy <= p1_fy;
            if (p1_v) begin
                p2_r0a <= r0a; p2_r0b <= r0b; p2_r1a <= r1a; p2_r1b <= r1b;
            end
            // ★ run13：cnn_bl 内部延迟从 1 拍变 2 拍 ⇒ 捕获用 **p2c_v**
            if (p2e_v) begin
                p3_vr <= vr; p3_vg <= vg; p3_vb <= vb;
            end
            // ---- 观测口：与 ACT 写口（级 4）对齐 ----
            o_valid <= p3_v;
            o_pix   <= p3_pix;
            o_rgb   <= {p3_vr, p3_vg, p3_vb};
            o_aq    <= {q8(p3_vr), q8(p3_vg), q8(p3_vb)};

            case (st)
            //----------------------------------------------------------
            S_IDLE: begin
                
                if (start) begin
                    busy <= 1'b1;
                    // 框夹取（与 Python 同序）：x1=clip(x1,0,W-1); x2=clip(x2,x1+1,W)
                    bx1 <= cx1;  bx2 <= cx2;
                    by1 <= cy1;  by2 <= cy2;
                    // Widths and coordinate seeds are computed in following setup stages.
                    oy  <= {AW{1'b0}};
                    vld0 <= 1'b0; vld1 <= 1'b0;
                    sel0 <= 1'b0;
                    // sx 初值 = 256*x1 + 2*sw - 128（全部用**夹后**的值）
                    st <= S_SETUP;
                end
            end

            //----------------------------------------------------------
            S_SETUP: begin
                sw<=bx2-bx1;sh<=by2-by1;
                sxlo<={10'd0,bx1}<<<8;sylo<={10'd0,by1}<<<8;
                sxhi<=({10'd0,bx2}-18'sd1)<<<8;
                syhi<=({10'd0,by2}-18'sd1)<<<8;
                bx_last_q<=bx2-8'd1;by_last_q<=by2-8'd1;
                sx_origin_q<=({10'd0,bx1}<<<8)-18'sd128;
                sy_origin_q<=({10'd0,by1}<<<8)-18'sd128;
                st<=S_SEED;
            end
            S_SEED: begin
                sx_seed_q<=sx_origin_q+sw*K;
                sx_acc<=sx_origin_q+sw*K;sy_acc<=sy_origin_q+sh*K;
                sx_step_q<=sw*(2*K);sy_step_q<=sh*(2*K);
                st<=S_YCLAMP;
            end
            S_YCLAMP: begin sy_clip_q<=sy_c;st<=S_YNEIGHBOR;end
            S_YNEIGHBOR: begin
                y0<=sy_clip_q[AW+7:8];fy<=sy_clip_q[7:0];
                y0b<=(sy_clip_q[AW+7:8]+8'd1>by_last_q)?by_last_q:(sy_clip_q[AW+7:8]+8'd1);
                st<=S_ROW;
            end

            S_ROW: begin
                // sy 已由上一行末尾（或初值）给出 ⇒ 这里算 y0/y0b 的复用与填充
                sel0      <= nsel0;
                fill1_req <= need_fill1;
                if (need_fill0) begin
                    kcnt     <= {AW{1'b0}};
                    fill_sel <= nsel0;              // 填"行 0"缓冲
                    st       <= S_FILL0;
                end else if (need_fill1) begin
                    kcnt     <= {AW{1'b0}};
                    fill_sel <= nsel0 ^ 1'b1;       // 填"行 1"缓冲
                    st       <= S_FILL1;
                end else begin
                    ox     <= 8'd0;
                    sx_acc <= sx_seed_q;
                    st     <= S_EMIT;
                end
            end

            //----------------------------------------------------------
            // 填行 0（目标 = sel0 更新后的 sel0，即 nsel0）
            S_FILL0: begin
                                if (kcnt > {AW{1'b0}}) begin
                    // 上一拍的读数据现在有效
                    if (fill_sel == 1'b0) lb0[kcnt - 8'd1] <= fr_data;
                    else                  lb1[kcnt - 8'd1] <= fr_data;
                end
                if (kcnt == sw) begin
                    
                    if (fill_sel == 1'b0) begin tag0 <= y0; vld0 <= 1'b1; end
                    else                  begin tag1 <= y0; vld1 <= 1'b1; end
                    if (fill1_req) begin
                        kcnt     <= {AW{1'b0}};
                        fill_sel <= sel0 ^ 1'b1;    // 行 1 的目标缓冲（sel0 已稳定）
                        st       <= S_FILL1;
                    end else begin
                        ox     <= 8'd0;
                        sx_acc <= sx_seed_q;
                        st     <= S_EMIT;
                    end
                end else kcnt <= kcnt + 8'd1;
            end

            //----------------------------------------------------------
            S_FILL1: begin
                                if (kcnt > {AW{1'b0}}) begin
                    if (fill_sel == 1'b0) lb0[kcnt - 8'd1] <= fr_data;
                    else                  lb1[kcnt - 8'd1] <= fr_data;
                end
                if (kcnt == sw) begin
                    
                    if (fill_sel == 1'b0) begin tag0 <= y0b; vld0 <= 1'b1; end
                    else                  begin tag1 <= y0b; vld1 <= 1'b1; end
                    ox     <= 8'd0;
                    sx_acc <= sx_seed_q;
                    st     <= S_EMIT;
                end else kcnt <= kcnt + 8'd1;
            end

            //----------------------------------------------------------
            S_EMIT: begin
                // ★ 流水**级 1**：算坐标并锁存（后续 3 级在上面的默认推进里）
                //   1 拍/像素 不变；ACT 写发生在级 4。
                p0_v<=1;
                p0_pix<={8'd0,oy}*OUT+{8'd0,ox};
                p0_sx<=sx_c;p0_fy<=fy;p0_sel0<=sel0;
                sx_acc  <= sx_acc + sx_step_q;       // +(128/OUT)*2*sw
                if (ox == OUT - 8'd1) begin
                    ox <= 8'd0;
                    last_row <= (oy == OUT - 8'd1);
                    if (oy != OUT - 8'd1) begin
                        oy     <= oy + 8'd1;
                        sy_acc <= sy_acc + sy_step_q;   // +(128/OUT)*2*sh
                    end
                    drn <= 2'd0;
                    st  <= S_DRN;                        // 行末先排空流水
                end else ox <= ox + 8'd1;
            end

            //----------------------------------------------------------
            // ★ 排空：行末等 3 拍，让级 2 把行缓冲读完、级 3/4 写完 ACT
            //   （否则 S_ROW/S_FILL 会覆盖行缓冲，读到新行的数据）
            S_DRN: begin
                if (drn == 3'd6) begin        // run13：cnn_bl 多 1 级 ⇒ 排空 4 拍
                    st <= last_row ? S_DONE : S_YCLAMP;
                end else
                    drn <= drn + 2'd1;
            end

            //----------------------------------------------------------
            S_DONE: begin
                done <= 1'b1;
                busy <= 1'b0;
                
                st <= S_IDLE;
            end

            default: st <= S_IDLE;
            endcase
        end
    end

endmodule
