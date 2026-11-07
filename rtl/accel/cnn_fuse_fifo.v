//=====================================================================
// cnn_fuse_fifo.v : **M9** dw → pw 融合通路的小 FIFO + 反压
//
// 为什么需要它（基线 §4.4「★ dw → pw 融合」）：
//   UltraFace / MobileFaceNet 里 **dw 之后紧跟 pw**。
//   `cnn_pe_page` 的 dw 映射**一拍产出同一个像素的 32 通道**
//   —— 正是 pw 需要的输入格式 ⇒ 直接把 dw 输出接进 pw 输入，
//   **省掉一次 ACT 往返**（写 ACT + 再读回来）。
//
// ⚠⚠ 两侧速率不匹配，**必须反压**：
//   dw 产出：1 像素/拍（32 通道）
//   pw 消耗：⌈oc/8⌉ 拍/像素（oc=32 ⇒ 4 拍）
//   ⇒ pw 慢 4 倍 ⇒ 没有反压 FIFO 会无限涨。本模块满时 `in_ready=0`
//     ⇒ 控制器必须据此 stall 上游 dw。
//     ⚠ 这一条基线没写：**融合不是零成本，dw 会因反压而停顿**，
//       端到端拍数应以 **max(dw, pw) 之和** 估，不是两者相加（基线 §4.6 目前是相加）。
//
// 容量：32 通道 × 深 4 × 8 b = 8192 b，用寄存器实现（深度小，不用 BRAM）。
// 读口：**组合读**（深度 ≤4 时比同步读省一拍延迟，且不会退化成分布式 RAM）
//=====================================================================
`timescale 1ns/1ps

module cnn_fuse_fifo #(
    parameter CH     = 32,
    parameter AW     = 8,
    parameter DEPTH  = 4,
    parameter PW     = 2            // 地址位宽（DEPTH=4 ⇒ 2）
)(
    input  wire                  clk,
    input  wire                  rst_n,
    // ---- dw 侧（写）----
    input  wire                  in_we,
    input  wire [CH*AW-1:0]      in_data,
    output wire                  in_ready,     // ★ 反压：未满才能写
    // ---- pw 侧（读）----
    input  wire                  out_re,
    output wire [CH*AW-1:0]      out_data,
    output wire                  out_vld       // 非空
);

    reg [CH*AW-1:0] mem [0:DEPTH-1];
    reg [PW-1:0]    wr_p, rd_p;
    reg [PW:0]      cnt;             // 0..DEPTH 需要多一位

    wire full  = (cnt == DEPTH);
    wire empty = (cnt == 0);

    assign in_ready = ~full;
    assign out_vld  = ~empty;
    assign out_data = mem[rd_p];

    always @(posedge clk) begin
        if (!rst_n) begin
            wr_p <= {PW{1'b0}};
            rd_p <= {PW{1'b0}};
            cnt  <= {(PW+1){1'b0}};
        end else begin
            case ({in_we && in_ready, out_re && out_vld})
                2'b10: begin                      // 只写
                    mem[wr_p] <= in_data;
                    wr_p      <= wr_p + 1'b1;
                    cnt       <= cnt + 1'b1;
                end
                2'b01: begin                      // 只读
                    rd_p      <= rd_p + 1'b1;
                    cnt       <= cnt - 1'b1;
                end
                2'b11: begin                      // 同拍读写：指针都进，cnt 不变
                    mem[wr_p] <= in_data;
                    wr_p      <= wr_p + 1'b1;
                    rd_p      <= rd_p + 1'b1;
                end
                default: ;
            endcase
        end
    end

endmodule
