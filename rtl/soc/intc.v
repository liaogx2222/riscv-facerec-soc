//=====================================================================
// intc.v : CLINT 式简化中断控制器 (M3b, 高阶任务2)
//
// 源(固定位号, 对齐 cpu irq[3:0] one-hot / mcause=0x8000_0000|id):
//   [0] SW     软件写触发        [1] TIMER 低 32 位与 cmp 相等单拍
//   [2] UART_RX rx_valid 上升沿   [3] KEY   外部输入下降沿(本板可不用)
//
// MMIO(soc_top 译码到本模块, wr/rd 为桥 accept 单拍脉冲):
//   widx 0: MIE   rw, 低 4 位          1: MIPS  读=挂起; 写 1 清对应位(W1C)
//   widx 2: SWIRQ 写值 v → mip |= v[3:0](软件自触发, 测试/核间敲门)
//   widx 3: TIMER_CMP 写=32 位阈值并清 fired 重武装; 读回阈值
// irq[i] = mip[i] & mie[i]  (mstatus.MIE 总闸在 CPU 的 csr 里)
//=====================================================================
`timescale 1ns/1ps

module intc (
    input  wire        clk,
    input  wire        resetn,

    input  wire        wr,
    input  wire [ 1:0] widx,
    input  wire [31:0] wdata,
    output wire [31:0] rdata,             // 组合读

    input  wire [31:0] time_lo,      // 周期计数器低 32 位(soc 的 cyc_cnt)
    input  wire        evt_uart_rx,  // uart_rx.rx_valid 电平(读 RXDATA 自动降)
    input  wire        key_n,        // 低有效按键(闲置拉高)
    output wire [ 3:0] irq
);
    reg  [3:0] mie, mip;
    reg  [31:0] cmp;
    reg         timer_fired;
    reg         cmp_valid;                   // cmp 写过才允许比较匹配
    reg         rx_q, key_q;
    reg  [3:0]  mip_n;

    wire evt_timer = cmp_valid && (time_lo == cmp) && !timer_fired;
    wire evt_key   = key_q && !key_n;               // 下降沿
    wire [3:0] evt_pulse = {evt_key, evt_uart_rx, evt_timer, 1'b0};

    assign irq = mip & mie;

    // mip 下一拍 = 旧值|事件, 再叠加同拍 MMIO 写 (显式合并, 防同拍双非阻塞赋值
    // 丢事件; uart 为电平源, W1C 后若仍未读 RXDATA 下一拍自然重新置位)
    always @(*) begin
        mip_n = mip | evt_pulse;
        if (wr && widx == 2'd1) mip_n = mip_n & ~wdata[3:0];   // W1C
        if (wr && widx == 2'd2) mip_n = mip_n |  wdata[3:0];   // SWIRQ
    end

    always @(posedge clk) begin
        if (!resetn) begin
            mie <= 4'h0; mip <= 4'h0; cmp <= 32'h0; cmp_valid <= 1'b0;
            timer_fired <= 1'b0; rx_q <= 1'b0; key_q <= 1'b1;
        end else begin
            rx_q  <= evt_uart_rx;
            key_q <= key_n;
            mip   <= mip_n;
            if (evt_timer && !(wr && widx == 2'd3)) timer_fired <= 1'b1;
            if (wr) begin
                case (widx)
                    2'd0: mie <= wdata[3:0];
                    2'd3: begin cmp <= wdata; timer_fired <= 1'b0; cmp_valid <= 1'b1; end
                    default: ;
                endcase
            end
        end
    end
    // 组合读
    reg [31:0] rdata_c;
    always @(*) begin
        case (widx)
            2'd0:    rdata_c = {28'h0, mie};
            2'd1:    rdata_c = {28'h0, mip};
            2'd2:    rdata_c = {28'h0, mip & mie};
            default: rdata_c = cmp;
        endcase
    end
    assign rdata = rdata_c;
endmodule
