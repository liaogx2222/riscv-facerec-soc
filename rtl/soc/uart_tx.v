//=====================================================================
// uart_tx.v : UART 发送器 (8N1, 基础任务结果输出用)
//
// 接口:
//   tx_data / tx_start : CPU 写入一字节并启动发送
//   tx_busy            : 1 = 正在发送 (CPU 轮询等待)
//   tx (串行输出)      : 空闲为高
//
// 波特率 = clk_freq / baud / 16 (过采样) 或 直接 clk_freq/baud (每 bit 一拍)
// 本实现: 每 bit 一拍 (clk_freq/baud 分频), 简单且够用
//=====================================================================
`timescale 1ns/1ps

module uart_tx #(
    parameter CLK_FREQ = 50_000_000,   // 系统时钟 Hz
    parameter BAUD     = 115_200       // 波特率
)(
    input  wire        clk,
    input  wire        resetn,
    input  wire        tx_start,       // 启动发送 (tx_data 须稳定)
    input  wire [ 7:0] tx_data,
    output reg         tx_busy,        // 发送中
    output reg         tx              // 串行输出
);

    localparam BIT_CYCLES = CLK_FREQ / BAUD;

    // 发送状态机: IDLE -> START -> DATA[7:0] -> STOP
    localparam IDLE  = 3'd0;
    localparam START = 3'd1;
    localparam DATA  = 3'd2;
    localparam STOP  = 3'd3;

    reg [ 2:0] state;
    reg [ 2:0] bit_cnt;          // 数据位计数
    reg [31:0] cyc_cnt;          // 每 bit 计时
    reg [ 7:0] shift;            // 待发数据

    always @(posedge clk) begin
        if (!resetn) begin
            state    <= IDLE;
            bit_cnt  <= 0;
            cyc_cnt  <= 0;
            tx_busy  <= 1'b0;
            tx       <= 1'b1;    // 空闲高
            shift    <= 8'h0;
        end else begin
            case (state)
                IDLE: begin
                    tx <= 1'b1;
                    tx_busy <= 1'b0;
                    if (tx_start) begin
                        shift   <= tx_data;
                        state   <= START;
                        cyc_cnt <= 0;
                        tx_busy <= 1'b1;
                    end
                end
                START: begin
                    tx <= 1'b0;              // 起始位
                    if (cyc_cnt == BIT_CYCLES-1) begin
                        cyc_cnt <= 0;
                        bit_cnt <= 0;
                        state   <= DATA;
                    end else cyc_cnt <= cyc_cnt + 1;
                end
                DATA: begin
                    tx <= shift[0];
                    if (cyc_cnt == BIT_CYCLES-1) begin
                        cyc_cnt <= 0;
                        if (bit_cnt == 7) begin
                            state <= STOP;
                        end else begin
                            shift   <= shift >> 1;
                            bit_cnt <= bit_cnt + 1;
                        end
                    end else cyc_cnt <= cyc_cnt + 1;
                end
                STOP: begin
                    tx <= 1'b1;              // 停止位
                    if (cyc_cnt == BIT_CYCLES-1) begin
                        cyc_cnt <= 0;
                        tx_busy <= 1'b0;
                        state   <= IDLE;
                    end else cyc_cnt <= cyc_cnt + 1;
                end
                default: state <= IDLE;
            endcase
        end
    end

endmodule
