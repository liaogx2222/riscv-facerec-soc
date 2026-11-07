//=====================================================================
// uart_rx.v : 8N1 UART 接收器 (M3b) — 与 uart_tx 对称
//
// BIT_CYCLES = round(CLK_FREQ/BAUD)。起始位下降沿 → 半位复查(滤毛刺) →
// 每位中心采 1 位(LSB first, 左移入) → 停止位复查 → 出字节。
// rx_valid 为"就绪电平"(读 RXDATA 脉冲清); 就绪未读又有新字节到 → overrun。
// ⚠ 计数纪律: 递减到 0 动作、动作拍重装 BIT_CYCLES-1 → 每 bit 恰占
//   BIT_CYCLES 拍 (与 uart_tx 同基) 且采样落在位窗中段、零漂移。若误装
//   BIT_CYCLES 则每位多占一拍 → 板(434拍/位)侥幸能用、仿真(2拍/位)必炸。
//=====================================================================
`timescale 1ns/1ps

module uart_rx #(
    parameter CLK_FREQ = 50_000_000,
    parameter BAUD     = 115_200
)(
    input  wire        clk,
    input  wire        resetn,
    input  wire        rxd,            // 高闲置; SoC 层已将悬空/z 归高
    input  wire        rx_read,        // 读 RXDATA 单拍脉冲
    output reg  [ 7:0] rx_data,
    output reg         rx_valid,       // 字节就绪(轮询位/中断源)
    output reg         rx_overrun
);
    localparam BIT_CYCLES = (CLK_FREQ + BAUD/2) / BAUD;
    // 起始复查半位计数: 递减到 0 才动作 → 复查拍 = 检测拍 + BC + 1.
    // 用 (BIT_CYCLES-1)/2 才落在起始位正中; 用 BIT_CYCLES/2 会在 2拍/位(仿真
    // 加速串口)时复查踩进 bit0, 收到 0x55 这类 bit0=1 的字节被当毛刺丢帧.
    localparam START_BC = (BIT_CYCLES - 1) / 2;

    localparam S_IDLE=3'd0, S_START=3'd1, S_BITS=3'd2, S_STOP=3'd3;
    reg [2:0]       st;
    reg [15:0]      bc;
    reg [2:0]       bitn;
    reg [7:0]       sh;
    wire            rxd_hi = (rxd === 1'b0) ? 1'b0 : 1'b1;

    always @(posedge clk) begin
        if (!resetn) begin
            st <= S_IDLE; bc <= 0; bitn <= 0; sh <= 0;
            rx_data <= 8'h0; rx_valid <= 1'b0; rx_overrun <= 1'b0;
        end else begin
            case (st)
            S_IDLE: if (!rxd_hi) begin                 // 起始下降沿
                        st <= S_START; bc <= START_BC;
                    end
            S_START: if (bc != 0) bc <= bc - 1'b1;
                     else if (!rxd_hi) begin           // 复查为低: 真起始
                        st <= S_BITS; bc <= BIT_CYCLES-1; bitn <= 3'd0; sh <= 8'h0;
                     end else st <= S_IDLE;            // 毛刺, 回空闲
            S_BITS: if (bc != 0) bc <= bc - 1'b1;
                    else begin
                        sh   <= {rxd_hi, sh[7:1]};     // LSB first, 移满 8 位
                        bc   <= BIT_CYCLES-1;          // 动作拍已占 1 拍 → 每 bit 恰 BIT_CYCLES 拍
                        if (bitn == 3'd7) st <= S_STOP;
                        else              bitn <= bitn + 3'd1;
                    end
            S_STOP: if (bc != 0) bc <= bc - 1'b1;
                    else begin
                        st <= S_IDLE;
                        if (rxd_hi) begin              // 停止位有效才收
                            rx_data <= sh;
                            if (rx_valid) rx_overrun <= 1'b1;   // 上一字节未走
                            else          rx_valid   <= 1'b1;
                        end
                    end
            default: st <= S_IDLE;
            endcase
            if (rx_read) begin
                rx_valid   <= 1'b0;
                rx_overrun <= 1'b0;
            end
        end
    end
endmodule
