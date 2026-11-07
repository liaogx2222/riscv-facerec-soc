//=====================================================================
// cnn_fsrc.v —— 帧缓冲 → ACT 物化写入器（M15，全流程第一棒）
//
// 为什么需要它：det 首层（B0, ic=3）的输入在**真实硬件**上由帧源直供
// linebuf（描述符的 `src_img=1`），但 RTL 的 `src_img` 目前**只解码不使用**
// （`cnn_desc_dec` 解出后没有消费者，引擎所有层输入都读 ACT）⇒ 仿真里必须
// 先把整帧物化进 ACT，det 才能跑。
//
// 语义（与 roi_frame/ROI 链逐位同源）：
//   · 遍历帧的每个像素 p = y*FW + x（行主序，与 ACT 字序一致）
//   · ACT 写：bank 索引 = p，256b 口一次写满 4 个 bank
//       bank0 = {40'd0, q8(B), q8(G), q8(R)}   （byte0 = ch0 = R）
//       bank1..3 = 0                            （ic=3 ⇒ 只有 word0 有值）
//   · q8(v) = clip(v − 127, −128, 127) 的 8 位补码 = `hwnet.quant_input`
//   ⇒ det B0 读 in_base=0、in_wpg=4 时，raddr = (0 + pix*4 + 0)>>2 = pix ✓
//
// 时序：帧读口是**同步读**（fr_data 比 fr_en 晚 1 拍）⇒ 请求拍 c、数据拍 c+1、
//   ACT 写在 c+2（aw_* 寄存器打一拍）。每像素 1 拍，共 FW*FH+3 拍。
//
// ⚠ 与 cnn_roi **分时复用**帧读口（cnn_soc 里 mux）：两者不会同时跑。
//=====================================================================
`timescale 1ns/1ps

module cnn_fsrc #(
    parameter FW = 128,
    parameter FH = 96
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    // 帧读口（与 cnn_roi 共享，cnn_soc 负责 mux）
    output reg  [7:0]  fr_x,
    output reg  [7:0]  fr_y,
    output reg         fr_en,
    input  wire [23:0] fr_data,      // {R[23:16], G[15:8], B[7:0]}
    // ACT 写口（256b：4 个 bank 同一索引）
    output reg  [3:0]  aw_we,
    output reg  [15:0] aw_addr,
    output reg  [255:0] aw_data,
    output reg         busy,
    output reg         done
);

    localparam S_IDLE = 2'd0, S_RUN = 2'd1, S_DRN = 2'd3, S_DONE = 2'd2;
    reg [1:0] st;

    reg [7:0]  cx, cy;                // 本拍请求的像素坐标
    reg [1:0]  drn;                   // S_DRN 排水计数
    reg        p_en;                  // 上一拍是否发了读请求
    reg [15:0] p_pix;                 // 上一拍请求的像素号（本拍数据有效）

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

    // 组合输出：S_RUN 期间每拍发一个读请求
    always @(*) begin
        fr_en = (st == S_RUN);
        fr_x  = cx;
        fr_y  = cy;
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            st <= S_IDLE; cx <= 8'd0; cy <= 8'd0; drn <= 2'd0;
            p_en <= 1'b0; p_pix <= 16'd0;
            aw_we <= 4'd0; aw_addr <= 16'd0; aw_data <= 256'd0;
            busy <= 1'b0; done <= 1'b0;
        end else begin
            done <= 1'b0;
            aw_we <= 4'd0;
            case (st)
            S_IDLE: begin
                if (start) begin
                    st <= S_RUN; cx <= 8'd0; cy <= 8'd0; drn <= 2'd0;
                    p_en <= 1'b0; busy <= 1'b1;
                end
            end
            S_RUN: begin
                // 记下本拍请求（数据下一拍有效）
                p_en  <= 1'b1;
                p_pix <= {8'd0, cy} * FW + {8'd0, cx};
                // 写上拍请求的数据（fr_data 本拍有效）
                if (p_en) begin
                    aw_we   <= 4'b1111;
                    aw_addr <= p_pix;
                    aw_data <= {192'd0, 40'd0,
                                q8(fr_data[7:0]),    // B → bank0 byte2
                                q8(fr_data[15:8]),   // G → bank0 byte1
                                q8(fr_data[23:16])}; // R → bank0 byte0（ch0）
                end
                // 推进坐标
                if (cx == FW - 1) begin
                    cx <= 8'd0;
                    if (cy == FH - 1) begin
                        // ★ 末像素已发请求，但它的 ACT 写还要 2 拍才落盘
                        //   （aw_* 寄存 1 拍 + ACT 采样 1 拍）⇒ 必须保持 busy
                        //   把这两拍拖完（busy 一低，仲裁就切回引擎 ⇒ 写丢失）
                        st <= S_DRN; drn <= 2'd0;
                    end else
                        cy <= cy + 8'd1;
                end else
                    cx <= cx + 8'd1;
            end
            S_DRN: begin
                // 把最后注册的写放出去（仲裁仍选 fsrc，因为 busy 还高）
                if (p_en) begin
                    aw_we   <= 4'b1111;
                    aw_addr <= p_pix;
                    aw_data <= {192'd0, 40'd0,
                                q8(fr_data[7:0]), q8(fr_data[15:8]),
                                q8(fr_data[23:16])};
                end
                p_en <= 1'b0;
                if (drn == 2'd1) st <= S_DONE;
                else             drn <= drn + 2'd1;
            end
            S_DONE: begin
                st <= S_IDLE; done <= 1'b1; busy <= 1'b0;
            end
            endcase
        end
    end
endmodule
