// Seven metadata stages; four cycles longer than the reference module.
// E0 control/read, E1 pair sums, E2 reduction, E3 acc, E4 shift,
// E5 signed clamp, E6 per-channel bias/clamp. Mode/shape are iteration constants.
`timescale 1ns/1ps

module cnn_outacc #(
    parameter ICG       = 4,
    parameter OC_SLOT   = 8,
    parameter YW        = 32,
    parameter ACC_DEPTH = 1024,
    parameter ACC_AW    = 10
)(
    input  wire                        clk,
    input  wire                        rst_n,
    input  wire                        en,        // ROW_RUN 流水线使能
    // 页部分和（与 pix_idx 对齐）
    input  wire [ICG*OC_SLOT*YW-1:0]   page_y,
    input  wire                        page_vld,
    input  wire [ACC_AW-1:0]           pix_idx,   // acc RAM 索引 = **列号**
    input  wire [15:0]                  pix_out,   // ACT 写地址用的**像素号**
    // 迭代属性（整个迭代内稳定）
    input  wire                        ig_first,  // ig==0：acc 直接写（不累加）
    input  wire                        ig_last,   // 最后一个 ig：走 requant 输出
    input  wire                        dw_mode,   // ★ 1 = depthwise：**不求和、不用 acc RAM**
    input  wire                        lin_out,   // 1 = 线性输出（clamp [-128,127]）；0 = ReLU
    input  wire [5:0]                  sh_req,    // requant 右移量
    // 按通道常数偏置（32 通道 × int8 = BN 折叠后的 offset）
    //   conv/pw：shc_r[64*(og&3) +: 64] = shc[og*8+g]（g=0..7，本 og 组 8 通道）
    //     —— 来自 shc ROM 的字（slot og&3），索引用 og_s2&3
    //   dw     ：一次填满 32 通道 `shc_r[(p*8+c)*8 +: 8] = shc[8p+c]`
    input  wire [ICG*OC_SLOT*8-1:0]    shc_r,
    // 输出特征图寻址
    input  wire [15:0]                 out_base,  // ACT 字基址
    input  wire [5:0]                  out_wpg,   // 每像素字数 = ceil(oc/8)
    input  wire [5:0]                  og,        // 当前输出通道组（conv/pw 用；dw 忽略）
    // ACT RAM 写口（组合；4 个 bank 各 64b）
    output wire [3:0]                  act_we,    // ★ 每 bank 独立写使能
    output wire [15:0]                 act_waddr,
    output wire [5:0]                  og_s0,     // ★ 管线级 0 的 og（shc ROM 地址提前 1 拍，2026-10-30）
    output wire [5:0]                  og_s1,     // ★ 管线级 1 的 og（保留，shcpipe 不再使用）
    output wire [ICG*OC_SLOT*8-1:0]    act_wdata, // 32 通道 × 8b
    // 管线排空（FSM 切迭代前等待）
    output wire                        acc_idle
);

    localparam NL   = ICG*OC_SLOT;    // 32 个 lane
    localparam PW   = ACC_AW + 16 + 3 + 8;   // {og, igf, igl, vld, pix_out, pix_idx}
    localparam NSTG = 7;

    reg [NSTG*PW-1:0] pix_pipe;

    // acc RAM：OC_SLOT × 32b（**只有 conv/pw 用**；dw 旁路）
    reg [OC_SLOT*32-1:0] acc_mem [0:ACC_DEPTH-1];
    reg [OC_SLOT*32-1:0] accold_q;

    // ★ 位偏移（移入顺序 {og, igf, igl, vld, pix_out, pix_idx}，MSB→LSB）：
    //     pix_idx : [0, ACC_AW)   pix_out : [ACC_AW, ACC_AW+16)
    //     vld : ACC_AW+16   igl : +1   igf : +2   og : [ACC_AW+19, ACC_AW+27)
    //   ⚠ 加字段后必须同步挪偏移，否则 vld/pix 全错位（本次踩过：输出全 X / 地址错乱）
    localparam PO = ACC_AW;                 // pix_out 起始位
    localparam PV = ACC_AW + 16;            // vld / igl / igf 起始位
    localparam PG = ACC_AW + 19;            // og 起始位
    wire [ACC_AW-1:0] pix_s0 = pix_pipe[2*PW +: ACC_AW];
    wire               vld_s0 = pix_pipe[2*PW + PV];
    wire               igf_s0 = pix_pipe[2*PW + PV + 2];
    wire [ACC_AW-1:0] pix_s2  = pix_pipe[6*PW +: ACC_AW];       // acc RAM 用（列号）
    wire [15:0]        pixo_s2 = pix_pipe[6*PW + PO +: 16];     // ACT 写用（像素号）
    wire               vld_s2 = pix_pipe[6*PW + PV];
    wire               igl_s2 = pix_pipe[6*PW + PV + 1];
    // ★ og 必须随像素一起走 3 级管线：ACT 写发生在 o_vld+2，跨 og 边界时
    //   输入端的 og 已经是新值 ⇒ 最后几个像素的写地址/写 bank 会用错 og。
    wire [5:0]         og_s2  = pix_pipe[6*PW + PG +: 6];
    // ★ og_s1（管线第 1 级）：引擎用它同拍发 shc ROM 读地址 ⇒
    //   ROM q 在 og_s2 拍有效，正是 r3 锁存拍（bnr = r2 + shc）。
    assign og_s1 = pix_pipe[5*PW + PG +: 6];
    // 一拍更早的 og：供 shc ROM 发地址，使 ROM q 在 og_s1 拍有效。
    assign og_s0 = pix_pipe[4*PW + PG +: 6];

    //-----------------------------------------------------------------
    // 每 slot 的「4 page 求和」（**只有 conv/pw 用**）
    //-----------------------------------------------------------------
    reg [OC_SLOT*32-1:0] sum_p;
    reg [OC_SLOT*32-1:0] sum01_q,sum23_q;
    reg [ICG*OC_SLOT*YW-1:0] raw1_q,raw2_q;
    reg [OC_SLOT*32-1:0] accold1_q,accold2_q;
    always @(posedge clk) if(en) begin
        raw1_q<=page_y;raw2_q<=raw1_q;
        accold1_q<=accold_q;accold2_q<=accold1_q;
    end
    genvar c;
    generate
        for (c = 0; c < OC_SLOT; c = c + 1) begin : g_psum
            always @(posedge clk) if(en) begin
                sum01_q[c*32 +: 32] <=
                    $signed(page_y[(0*OC_SLOT+c)*YW +: YW]) +
                    $signed(page_y[(1*OC_SLOT+c)*YW +: YW]);
                sum23_q[c*32 +: 32] <=
                    $signed(page_y[(2*OC_SLOT+c)*YW +: YW]) +
                    $signed(page_y[(3*OC_SLOT+c)*YW +: YW]);
                sum_p[c*32 +: 32] <= $signed(sum01_q[c*32 +: 32])+
                                      $signed(sum23_q[c*32 +: 32]);
            end
        end
    endgenerate

    // Q_MIN：lin 允许 -128；relu 下界 0（⚠ 写 `-8'sd128` 会溢出，用位模式 8'h80）
    wire signed [7:0] q_min = lin_out ? 8'sh80 : 8'sd0;

    //-----------------------------------------------------------------
    // 32 个 lane 的 requant 链
    //-----------------------------------------------------------------
    reg  [NL*8-1:0] r2_r;      // C2
    reg  [NL*8-1:0] r3_r;      // C3
    wire [NL*32-1:0] lane_in;  // 组合（C1）

    genvar p, g;
    generate
        for (p = 0; p < ICG; p = p + 1) begin : g_page
            for (g = 0; g < OC_SLOT; g = g + 1) begin : g_lane
                // lane L = p*OC_SLOT + g
                //   dw     ：本 page 本 slot 的原始部分和（**不与其他 page 相加**）
                //   conv/pw：4 个 page 的部分和（对 slot g 求和，与 p 无关）
                assign lane_in[(p*OC_SLOT+g)*32 +: 32] = dw_mode ?
                    $signed(raw2_q[(p*OC_SLOT+g)*YW +: YW]) :
                    $signed(sum_p[g*32 +: 32]);

                //---- acc_new（组合，C1）：dw 旁路 acc RAM；conv/pw 走累加 ----
                wire signed [31:0] accn = dw_mode ? $signed(lane_in[(p*OC_SLOT+g)*32 +: 32]) :
                    (igf_s0 ? $signed(sum_p[g*32 +: 32])
                            : $signed(accold2_q[g*32 +: 32]) +
                              $signed(sum_p[g*32 +: 32]));

                //---- stage r2（C1 求值，C2 有效）----
                reg signed [31:0] accn_q,shift_q;
                always @(posedge clk) if(en) begin
                    accn_q<=accn;
                    shift_q<=accn_q >>> sh_req;
                end
                wire signed [31:0] a1s = shift_q;
                always @(posedge clk) begin
                    if (!rst_n)
                        r2_r[(p*OC_SLOT+g)*8 +: 8] <= 8'd0;
                    else if (en) begin
                        if (a1s > 32'sd127)
                            r2_r[(p*OC_SLOT+g)*8 +: 8] <= 8'sd127;
                        else if (a1s < -32'sd128)
                            r2_r[(p*OC_SLOT+g)*8 +: 8] <= -8'sd128;
                        else
                            r2_r[(p*OC_SLOT+g)*8 +: 8] <= a1s[7:0];
                    end
                end

                //---- stage r3（C2 求值，C3 有效）：+ shc + clamp ----
                //   ⚠ 不能写 `r2_r[...][7]`（部分选择后不能再带下标，iverilog 报错）⇒ 用中间线网
                wire signed [7:0] r2s, shcs;
                wire signed [8:0] bnr;
                assign r2s  = r2_r[(p*OC_SLOT+g)*8 +: 8];
                // ★ dw：lane (p,g) = 输出通道 8p+g ⇒ 32 字节通道直排；
                //   conv/pw：lane (p,g) 的输出通道 = og_s2*8+g（与 p 无关）
                //     ⇒ 用 ROM 字内 slot (og_s2&3) 的 8 字节
                assign shcs = dw_mode ? shc_r[(p*OC_SLOT+g)*8 +: 8]
                                      : shc_r[(((og_s2 & 3) * 8 + g)) * 8 +: 8];
                assign bnr  = r2s + shcs;
                always @(posedge clk) begin
                    if (!rst_n)
                        r3_r[(p*OC_SLOT+g)*8 +: 8] <= 8'd0;
                    else if (en) begin
                        if (bnr > 9'sd127)
                            r3_r[(p*OC_SLOT+g)*8 +: 8] <= 8'sd127;
                        else if (bnr < q_min)
                            r3_r[(p*OC_SLOT+g)*8 +: 8] <= q_min;
                        else
                            r3_r[(p*OC_SLOT+g)*8 +: 8] <= bnr[7:0];
                    end
                end
            end
        end
    endgenerate

    //-----------------------------------------------------------------
    // 主流水线：像素管线 + 每 slot 的 acc 旧值读出
    //-----------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            pix_pipe <= {NSTG*PW{1'b0}};
        end else if (en) begin
            // ★ 位移入顺序 = {igf, igl, vld, pix_out, pix_idx}
        //   ⇒ LSB(=位 [0 +: ACC_AW]) 是 pix_idx（acc RAM 用），
        //     位 [ACC_AW +: 16] 是 pix_out（ACT 写地址用）。
        //   ⚠ 写成 {igf,igl,vld,pix,pixout} 会把 pix 的位置抢给 pixout ⇒ 全 X
        // ★ 位移入宽度必须**恰等于 PW**（少一位都会让后续级错位）：
        //   {og(8b: 2'b00 拼接), igf, igl, vld, pix_out, pix_idx} = 8+1+1+1+16+10
        pix_pipe <= {pix_pipe[(NSTG-1)*PW-1:0],
                        2'b00, og[5:0], ig_first, ig_last, page_vld, pix_out, pix_idx};
        end
    end

    //-----------------------------------------------------------------
    // acc RAM（**只有 conv/pw 写**；dw 旁路）
    //-----------------------------------------------------------------
    // 每 slot 的 acc_new（供 RAM 写用）—— 必须在 always 之前声明
    wire [OC_SLOT*32-1:0] accn_c;
    genvar g2;
    generate
        for (g2 = 0; g2 < OC_SLOT; g2 = g2 + 1) begin : g_accn
            assign accn_c[g2*32 +: 32] = igf_s0 ? $signed(sum_p[g2*32 +: 32])
                : ($signed(accold2_q[g2*32 +: 32]) + $signed(sum_p[g2*32 +: 32]));
        end
    endgenerate

    always @(posedge clk) begin
        if (en && vld_s0 && !dw_mode)
            acc_mem[pix_s0] <= accn_c;
        accold_q <= acc_mem[pix_idx];
    end

    //-----------------------------------------------------------------
    // ACT 写口（组合；4 个 bank）
    //-----------------------------------------------------------------
    // ★ 地址里的通道组偏移，dw 与 conv/pw **差 4 倍**：
    //   conv/pw：og = **8 通道组** ⇒ 写 1 个字，偏移 = og
    //   dw     ：og = **32 通道组**（一拍出全部 32 通道）⇒ 写 4 个字，偏移 = og*4
    //   （dw 的通道 [32·og, 32·og+32) ⇒ 字下标 [4·og, 4·og+4)，4 个 bank 一次写完）
    // ⚠ 漏掉这 4 倍会**整体写错位置**（dw 层全部错位），不是小错。
    wire [7:0]  og_off = dw_mode ? {og_s2, 2'b00} : {2'b00, og_s2};
    // Prepare the address during existing data stages; ACT commit remains E6.
    wire [15:0] addr_pix_s4=pix_pipe[4*PW+PO+:16];
    wire [5:0] addr_og_s5=pix_pipe[5*PW+PG+:6];
    wire [7:0] addr_og_off_s5=dw_mode?{addr_og_s5,2'b00}:{2'b00,addr_og_s5};
    // Configuration is stable while the layer is busy. Capture it before
    // the existing E5 address product stage, keeping the FF out of APM Y_REG.
    (* syn_preserve=1 *) reg [5:0] out_wpg_q;
    always @(posedge clk)if(en)out_wpg_q<=out_wpg;
    // Precompute the base/group addend at E5 beside the product. Keeping
    // this fabric boundary removes dw_mode -> carry chain -> APM C/Z setup.
    wire [5:0] addr_og_s4=pix_pipe[4*PW+PG+:6];
    wire [7:0] addr_og_off_s4=dw_mode?{addr_og_s4,2'b00}:{2'b00,addr_og_s4};
    (* syn_preserve=1 *) reg [17:0] addr_addend_q;
    reg [17:0] addr_product_q,awaddr_q;
    always @(posedge clk) if(en) begin
        addr_product_q<={2'b0,addr_pix_s4}*{12'b0,out_wpg_q};
        addr_addend_q<={2'b0,out_base}+{10'b0,addr_og_off_s4};
        awaddr_q<=addr_product_q+addr_addend_q;
    end
    wire [17:0] awaddr_c=awaddr_q;
    // dw：一拍写满 4 个 bank（32 个输出通道）
    // conv/pw：只写 og 对应的那 1 个 bank（通道 [og*8, og*8+8)）
    assign act_we    = (vld_s2 & igl_s2) ? (dw_mode ? 4'b1111 : (4'b0001 << og_s2[1:0]))
                                         : 4'b0000;
    // ★ `act_waddr` 是 **bank 索引 = 字地址 >> 2**（`cnn_act` 的 waddr 是 bank 索引）
    //   `act_wdata` 的 lane (p,c) 落在 bank p 的槽 c：
    //     dw     ：bank p = 通道 [8p,8p+8)  ⇒ 4 个 bank 一次写完 32 通道 ✓
    //     conv/pw：4 个 p 的 lane 值相同（都等于 Σ_p' page_y[p'][c]）⇒ 写到
    //              bank (og&3) 的正好是通道 og*8+c ✓
    assign act_waddr = awaddr_c[17:2];
    assign act_wdata = r3_r;

    // acc_idle：只看 NSTG 级 vld 位
    wire [NSTG-1:0] vld_vec;
    genvar gv;
    generate
        for (gv = 0; gv < NSTG; gv = gv + 1) begin : g_vbit
            assign vld_vec[gv] = pix_pipe[gv*PW + PV];
        end
    endgenerate
    assign acc_idle = (vld_vec == {NSTG{1'b0}});

endmodule
