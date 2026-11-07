//=====================================================================
// cnn_d1.v —— **D1 dense（FC 1024→64）硬件模块**（M16，全流程最后一棒）
//
// 为什么单独一个模块（不进 cnn_engine）：
//   · D1 的输入是 M10 的**整图展平**（1024 = 16 像素 × 64 通道），描述符的
//     ih/iw 是 8 位字段 ⇒ 无法用 iw=1024 表达；且权重布局 [64][1024] 与
//     pw 的 [oc][ic] 打包方式不同（dwrom 是 64b/字的 dense ROM）。
//   · 与 cnn_fsrc / cnn_roi 同一模式：独立小模块 + ACT 经 SoC 仲裁接入。
//
// 数据流（三段式，全部时间互斥走 ACT 口）：
//   ① LOAD  32 个 beat × 8 拍 = 256 拍：每 beat 从 ACT 读回 256b（4 个 64b 字），
//           再摊成 4 拍逐字写进 VBUF（**单写口**，见下方 VBUF 说明）
//           ⇒ VBUF[0..127]（= 展平后的 1024 个 int8，字 ck 的 byte k = 通道 ck*8+k）
//   ② COMP  64 个输出 × 129 拍 = 8,256 拍，每拍 8 个 int8 MAC：
//           acc = Σ_{ck,k} W[o][ck*8+k] · VBUF[ck][k]
//           ⇒ y = clip(acc >>> sh_req) + shc[o]（与 hwnet.dense_int 逐位同序）
//           （129 = 128 + 1：VBUF 同步读有 1 拍延迟，多跑一拍收最后一个字）
//   ③ WR    2 拍把 64 个 int8 写回 ACT（out_base 起，4 字/拍）
//
// 权重/shc 与上板同一份文件：rtl/accel/mem/cnn_dwrom.mem（8192×64b）、
//   cnn_d1shc.mem（64 字节，由 gen_fullflow 生成）。
// 周期 ≈ 8,523 拍（tb/d1_tb.v 实测）。
//=====================================================================
`timescale 1ns/1ps

module cnn_d1 #(
    parameter IC = 1024,             // 输入通道总数（= M10 展平）
    parameter OC = 64,               // 输出 = embedding 维数
    parameter IW = 8,                // 字宽（通道/字）
    parameter W_FILE  = "rtl/accel/mem/cnn_dwrom.mem",
    parameter SHC_FILE = "rtl/accel/mem/cnn_d1shc.mem"
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire [15:0] in_base,      // ACT flat 字基址（M10 输出，须 4 对齐）
    input  wire [15:0] out_base,     // ACT flat 字基址（embedding 写出，须 4 对齐）
    input  wire [7:0]  sh_req,       // requant 右移量（D1 = 8）
    // ---- ACT 读口（与引擎/读回分时复用）----
    output reg         d_re,
    output reg  [15:0] d_raddr,
    output reg  [3:0]  d_bmask,
    input  wire [255:0] d_rdata,
    // ---- ACT 写口 ----
    output reg  [3:0]  d_we,
    output reg  [15:0] d_waddr,
    output reg  [255:0] d_wdata,
    output reg         busy,
    output reg         done
);

    localparam NW  = IC / IW;             // 128 个 64b 字（= 1024 通道 / 8）
    localparam NB  = NW / 4;              // 32 个 ACT beat（每 beat 256b = 4 字）
    localparam NCH = OC / IW;             // 8 个输出字（64 通道 / 8）

    localparam S_IDLE = 3'd0, S_LOAD = 3'd1, S_COMP = 3'd2, S_FIN  = 3'd3,
                   S_WR   = 3'd4, S_DONE = 3'd5, S_FIN2 = 3'd6, S_FIN3=3'd7;
    reg [2:0] st;

    //-----------------------------------------------------------------
    // ★★ VBUF：**单写口 + 同步读**的 128×64 阵列
    //
    //   为什么是这个形态（PDS 150MHz 优化的关键，别退回旧写法）：
    //   · 最初是「一拍写 4 个**不同地址**的字 + 组合读」。Synplify 无法把
    //     这种多写口阵列推断成 RAM，只能铺成一棵 128:1×64b 的 mux 树 ⇒
    //     实测 ckcnt[0..6] 每根线扇出 **7400**、单根线延时 3.58 ns，
    //     整条路径 ~16 ns（≈48 MHz）。
    //   · 先试过「拆 4 个 bank + 4:1 mux」：工具又把 bank 合并回单块，扇出不变。
    //   · 最终：LOAD 摊成「每拍写 1 个字」⇒ 单写口；读改成同步读 + 1 拍流水。
    //     结果：LUT 15,841 → 7,910，ckcnt 扇出 7400 → 45。
    //   语义不变：字序号 w 就是 VBUF[w]。
    //-----------------------------------------------------------------
    reg [63:0] VBUF [0:NW-1];
    // ★★ 2026-10-23（run11）：EMB 必须保留为独立寄存器。
    //   PDS/Synplify 会把「只被一个下游使用、时钟相同」的 EMB 合并进 d_wdata，
    //   把「动态右移 + 两级夹取 + +shc + 打包」全并成一条组合路径（实测 23 级 / 7.77 ns）。
    //   syn_preserve 禁止该优化。
    (* syn_preserve = 1 *) reg signed [7:0] EMB [0:OC-1];   // 输出 embedding（不得合并）
    reg [63:0] D1W [0:OC*(IC/IW)-1];      // dense 权重 ROM（64b/字）
    reg signed [7:0] SHC [0:OC-1];        // 每输出通道常数偏置

    reg [2:0]   ph;                       // LOAD 相位（8 拍/beat）
    // ★★ 2026-10-23（run17）：计数器用**实际位宽**，不用 integer（32 位）。
    //   run16 头号路径 = `ckcnt[27] → 宽比较器/进位链 → APM`（7 级 + 走线 4.78 ns）。
    //   ckcnt 最大 130 ⇒ 8 位；bcnt 最大 31 ⇒ 6 位；ocnt/wcnt 最大 63/1 ⇒ 8 位。
    reg [5:0]   bcnt;                     // LOAD 的 beat 序号（0..NB-1 = 31）
    reg [255:0] beat_q;                   // 本 beat 的 ACT 读回数据
    reg [7:0]   ocnt, ckcnt, wcnt;        // 0..63 / 0..130 / 0..1
    // COMP 流水寄存器：VBUF/D1W 的读数据都在地址发出后下一拍有效
    reg [63:0]  v_q, w_q;
    // D1W/VBUF synchronous outputs feed registered DSP multiplier inputs.
    reg [63:0] v_mul_q,w_mul_q;
    // ★★ 2026-10-23（run13）：acc 从 64 位降到 **32 位**。
    //   上界：1024 个输入 × (128×128) = 16.78 M < 2^24 ⇒ 32 位（含符号）绐绐有余。
    //   run12 实测：64 位进位链把「APM → acc[63]」拉成 19 级 / 6.8 ns，是 D1 头号路径。
    reg signed [31:0] acc;
    reg signed [18:0] sum8_q;              // 求和树结果寄存（run14）
    reg signed [31:0] yq;                  // 移位后的中间值（分拆用）
    reg [7:0]  sh_req_r;                   // sh_req 是**每层常数** ⇒ 寄存一拍
    reg signed [7:0] y_clip_q,shc_clip_q;
    wire signed [8:0] bias_sum={y_clip_q[7],y_clip_q}+{shc_clip_q[7],shc_clip_q};

    //-----------------------------------------------------------------
    // ★ 8 个 int8 乘积 + **平衡树求和**（显式三级）
    //
    //   原写法 `acc = acc + p0 + p1 + ... + p7` 是**串行 8 级 64 位加法**：
    //   PDS 布局布线后实测这段占了关键路径的 ~8 ns
    //   （每级进位链 0.09 ns × 64 位 + 路由）。
    //   改成：先把 8 个乘积用**平衡树**求和（3 级窄加法，结果 ≤ 19 位），
    //   再与 acc 相加（**只 1 次 64 位加法**）。
    //   整数加法可结合且无溢出（每项 ≤ 16129，八项和 ≤ 129032 < 2^18）⇒ **逐位等价**。
    //   ⚠ 本段按 IW=8 展开（D1 固定 8 通道/字）。
    //-----------------------------------------------------------------
    wire signed [15:0] p0 = $signed(w_mul_q[ 7: 0]) * $signed(v_mul_q[ 7: 0]);
    wire signed [15:0] p1 = $signed(w_mul_q[15: 8]) * $signed(v_mul_q[15: 8]);
    wire signed [15:0] p2 = $signed(w_mul_q[23:16]) * $signed(v_mul_q[23:16]);
    wire signed [15:0] p3 = $signed(w_mul_q[31:24]) * $signed(v_mul_q[31:24]);
    wire signed [15:0] p4 = $signed(w_mul_q[39:32]) * $signed(v_mul_q[39:32]);
    wire signed [15:0] p5 = $signed(w_mul_q[47:40]) * $signed(v_mul_q[47:40]);
    wire signed [15:0] p6 = $signed(w_mul_q[55:48]) * $signed(v_mul_q[55:48]);
    wire signed [15:0] p7 = $signed(w_mul_q[63:56]) * $signed(v_mul_q[63:56]);

    // ★ 流水级 A→B 的乘积寄存器（把「APM 乘法」与「树求和+累加」分成两拍）
    reg signed [15:0] pq0, pq1, pq2, pq3, pq4, pq5, pq6, pq7;
    wire signed [16:0] s01 = pq0 + pq1;
    wire signed [16:0] s23 = pq2 + pq3;
    wire signed [16:0] s45 = pq4 + pq5;
    wire signed [16:0] s67 = pq6 + pq7;
    wire signed [17:0] t0123 = s01 + s23;
    wire signed [17:0] t4567 = s45 + s67;
    wire signed [18:0] sum8   = t0123 + t4567;

    always @(posedge clk) begin
        if (!rst_n) sh_req_r <= 8'd0;
        else        sh_req_r <= sh_req;
    end

    initial begin
        $readmemh(W_FILE, D1W);
        $readmemh(SHC_FILE, SHC);
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            st <= S_IDLE; busy <= 1'b0; done <= 1'b0;
            d_re <= 1'b0; d_raddr <= 16'd0; d_bmask <= 4'd0;
            d_we <= 4'd0; d_waddr <= 16'd0; d_wdata <= 256'd0;
            ph <= 3'd0; bcnt <= 0; ocnt <= 0; ckcnt <= 0; wcnt <= 0; sum8_q <= 19'sd0;
            y_clip_q<=0;shc_clip_q<=0;
        end else begin
            done <= 1'b0;
            d_re <= 1'b0;
            d_we <= 4'd0;
            case (st)
            //----------------------------------------------------------
            S_IDLE: if (start) begin
                busy <= 1'b1; st <= S_LOAD; ph <= 3'd0; bcnt <= 0;
            end
            //----------------------------------------------------------
            // ① LOAD：8 拍/beat（32 beat × 8 = 256 拍）
            //   ph=0 发 ACT 读请求；ph=2 收数据；ph=3..6 逐字写 VBUF；ph=7 推进 beat
            //   ★ ACT 读数据在**请求后第 2 拍**有效（d_re/d_raddr 是寄存器输出，
            //     再经 sdpram 同步读 1 拍）⇒ 收在 ph=2，别提前也别推后。
            //   ★ 摊成 4 拍写是为了让 VBUF 只有 1 个写口（可推断成 RAM）。
            S_LOAD: begin
                ph <= ph + 3'd1;
                case (ph)
                3'd0: begin
                    d_re    <= 1'b1;
                    d_raddr <= (in_base >> 2) + {10'd0, bcnt};
                    d_bmask <= 4'b1111;
                end
                3'd2: beat_q <= d_rdata;
                3'd3: VBUF[bcnt*4 + 0] <= beat_q[63:0];
                3'd4: VBUF[bcnt*4 + 1] <= beat_q[127:64];
                3'd5: VBUF[bcnt*4 + 2] <= beat_q[191:128];
                3'd6: VBUF[bcnt*4 + 3] <= beat_q[255:192];
                3'd7: begin
                    if (bcnt == NB - 1) begin
                        st <= S_COMP; ocnt <= 0; ckcnt <= 0; acc = 32'sd0;
                    end else
                        bcnt <= bcnt + 1;
                end
                default: ;
                endcase
            end
            //----------------------------------------------------------
            // ② COMP：每拍 8 MAC（1 个权重字 × 1 个输入字）
            //   同步读流水：本拍发地址 ckcnt，数据下拍有效
            //   ⇒ 本拍累加的是索引 ckcnt-1 ⇒ ckcnt 跑 0..NW，在 ckcnt==NW 收尾
            S_COMP: begin
                // ★★ 3 级流水（每级都 < 6.7 ns）：
                //   级 A  ：本拍发 VBUF/D1W 地址（BRAM 同步读，下拍 v_q/w_q 有效）
                //   级 A→B：乘积打一拍（pq* ← APM 输出）
                //   级 B  ：树求和 + 与 acc 相加
                //   ⇒ 本拍 pq* 对应索引 **ckcnt-2** ⇒ ckcnt 跑 0..NW+1
                if (ckcnt < NW)
                    v_q <= VBUF[ckcnt[6:0]];
                w_q <= D1W[{ocnt[6:0], 7'd0} + ckcnt[7:0]];
                v_mul_q<=v_q;w_mul_q<=w_q;
                pq0 <= p0;  pq1 <= p1;  pq2 <= p2;  pq3 <= p3;
                pq4 <= p4;  pq5 <= p5;  pq6 <= p6;  pq7 <= p7;
                // ★★ run14：**求和树结果再寄存一拍**，把「APM → 树求和 → 32 位进位链」
                //   拆成两段：
                //     · pq* → sum8（3 级 19 位加）→ sum8_q          ≈ 1.5~2 ns
                //     · sum8_q → acc（32 位进位链）→ acc            ≈ 2~2.5 ns
                //   run13 实测合在一拍是 3.69 ns 逻辑 + 3.38 ns 走线（头号路径）。
                //   代价：每输出 +1 拍（共 +64 拍）。
                sum8_q <= sum8;
                if (ckcnt >= 4)
                    acc = acc + {{13{sum8_q[18]}}, sum8_q};   // 19 位有符号 → 32 位
                if (ckcnt == NW + 3) begin
                    ckcnt <= 0;
                    st    <= S_FIN;
                end else begin
                    ckcnt <= ckcnt + 1;
                end
            end
            //----------------------------------------------------------
            // ②b FIN：requant + 写 EMB。
            //   ★ 必须从累加拍里拆出来：否则「最后一次累加（宽加法）
            //     + 动态右移（桶形移位器）+ 两级夹取」全挤在一拍。
            //   ★ acc 在这里用**阻塞**清零：同一变量不能混用阻塞/非阻塞（Verilog-4042）。
            // ★★ 2026-10-23（run11）：FIN 拆为**两拍**
            //   原来一拍里：64 位动态右移（桶形，约 8 级）+ 两级夹取 + +shc
            //   ⇒ 布局布线后这段单独就是头号路径。
            //   现在：S_FIN 只做移位（存 yq），S_FIN2 做夹取 + shc + 写 EMB。
            //   代价：每输出 +1 拍（64 输出 ⇒ +64 拍，占 8.7K 拍的 0.7%）。
            S_FIN: begin
                yq  = acc >>> sh_req_r;
                acc = 32'sd0;
                st  <= S_FIN2;
            end
            //----------------------------------------------------------
            S_FIN2: begin
                if(yq>32'sd127)y_clip_q<=8'sd127;
                else if(yq< -32'sd128)y_clip_q<=8'sh80;
                else y_clip_q<=yq[7:0];
                shc_clip_q<=SHC[ocnt];
                st<=S_FIN3;
            end
            S_FIN3: begin
                if(bias_sum>9'sd127)EMB[ocnt]<=8'sd127;
                else if(bias_sum< -9'sd128)EMB[ocnt]<=8'sh80;
                else EMB[ocnt]<=bias_sum[7:0];
                if (ocnt == OC - 1) begin
                    st <= S_WR; wcnt <= 0;
                end else begin
                    ocnt <= ocnt + 1;
                    st   <= S_COMP;
                end
            end
            //----------------------------------------------------------
            // ③ WR：2 拍 × 4 字 = 64 通道（byte k = 通道 wordidx*8+k）
            S_WR: begin
                d_we    <= 4'b1111;
                d_waddr <= (out_base >> 2) + {8'd0, wcnt};   // wcnt 已收窄为 8 位
                d_wdata <= wr_word;
                if (wcnt == NCH/4 - 1) begin
                    // ★ 不要在这里清 d_we / busy：
                    //   ① d_we 被后赋值覆盖会把本拍的写灭掉；
                    //   ② busy 一低，SoC 仲裁（d1_act = busy|go）就切走，
                    //      而末个写是**寄存器输出**，要在 busy 期间才落盘
                    //      （与 cnn_fsrc 的 S_DRN 同族：晚一拍在 S_DONE 清）。
                    st   <= S_DONE;
                end else
                    wcnt <= wcnt + 1;
            end
            //----------------------------------------------------------
            S_DONE: begin
                done <= 1'b1;
                busy <= 1'b0;      // ★ 晚一拍清：末个寄存器写已在本拍落盘
                st   <= S_IDLE;
            end
            endcase
        end
    end

    // 4 个 64b 字（= 32 通道）打包成 256b 写数据。
    // ★ 用**唯一组合进程 + 模块级数组**，不用 function：PDS/Synplify 对「有局部变量
    //   且多处调用」的 function 做 static 内联 ⇒ tie-0（审计规则 [B]）。
    //   布局：字 ww（0..3）的 byte kk = 通道 (wcnt*4+ww)*8 + kk。
    reg [255:0] wr_word;
    integer ww, kk;
    always @(*) begin
        wr_word = 256'd0;
        for (ww = 0; ww < 4; ww = ww + 1)
            for (kk = 0; kk < IW; kk = kk + 1)
                wr_word[ww*64 + kk*8 +: 8] = EMB[(wcnt*4 + ww)*IW + kk];
    end
endmodule
