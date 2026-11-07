// CPU/UART 101.25MHz; independent CNN domain 202.5MHz. Handshake CDC MMIO.
// S5F in-order dual issue + complete FaceRec CNN; ROM source and UART.
`timescale 1ns/1ps

module facerec_top #(
    parameter integer CLK_HZ         = 101_250_000,
    parameter integer CNN_CLK_HZ     = 202_500_000,
    parameter integer BAUD           = 115_200,
    parameter         LED_ACTIVE_LOW = 1,
    // 固件：仿真用 RAM_INIT_FILE（$readmemh），板级用 RAM_EMBED_INIT（内嵌进 simple_ram）
    parameter         RAM_INIT_FILE  = "",
    parameter         RAM_EMBED_INIT = 1,        // 板级默认：用内嵌固件（见 gen/gen_simple_ram.py）
    parameter integer RAM_ADDR_W     = 16        // 64KB：菜单镜像 11,691 词（46.6 KB）
)(
    input  wire       sys_clk,      // 27 MHz（D18）
    input  wire       key_n,        // 复位按键（C22，低有效）
    input  wire       uart_rxd,     // AC26
    output wire       uart_txd,     // AB26
    output wire [3:0] led_out       // U16/U15/V17/V16（低电平点亮）
);

    //-----------------------------------------------------------------
    // 时钟 / 复位
    //   默认：PLL 出 101.25 MHz(CPU) + 202.5 MHz(CNN)，约束见
    //         constraints/product/board.fdc
    //   FACEREC_NO_PLL：27 MHz 单时钟（PLL 旁路，两域同源），约束见
    //         constraints/27mhz/board.fdc —— 这是历史上唯一签核过的配置
    //         （sys_clk→sys_clk WNS +14.080 ns / TNS 0 / WHS +41 ps）。
    //   FACEREC_SIM：仿真时同样旁路，避免依赖 Pango 原语。
    //-----------------------------------------------------------------
    wire clk_sys, clk_cnn, pll_locked;

`ifdef FACEREC_NO_PLL
    assign clk_sys    = sys_clk;
    assign clk_cnn    = sys_clk;
    assign pll_locked = 1'b1;
`elsif FACEREC_SIM
    assign clk_sys    = sys_clk;
    assign clk_cnn    = sys_clk;
    assign pll_locked = 1'b1;
`else
    pll u_pll (
        .clkin1 (sys_clk),
        .clkout0(clk_sys),
        .clkout1(clk_cnn),
        .lock   (pll_locked)
    );
`endif

    reg [15:0] por_cnt = 16'h0000;
    wire       por_done = &por_cnt;
    always @(posedge clk_sys)
        if (!por_done && pll_locked)
            por_cnt <= por_cnt + 16'h0001;

    reg [2:0] key_syn = 3'b111;
    always @(posedge clk_sys)
        key_syn <= {key_syn[1:0], key_n};

    wire resetn = por_done & key_syn[2] & pll_locked;

    //-----------------------------------------------------------------
    // SoC（含门禁加速器）
    //-----------------------------------------------------------------
    wire       cpu_stall;
    wire [7:0] gpio;

    soc_top #(
        .CLK_FREQ      (CLK_HZ),
        .BAUD          (BAUD),
        .RAM_ADDR_W    (RAM_ADDR_W),
        .RAM_INIT_FILE (RAM_INIT_FILE),
        .RAM_EMBED_INIT(RAM_EMBED_INIT),
        .UART_BEACON   (0),
        .BTB_ENABLE    (1),
        .S6B_ENABLE    (1),
        .QLN_EN        (0),          // 不用 QLN
        .ACCEL_EN      (0),
        .FACEREC_EN    (1),
        .FACEREC_ASYNC (1),
        .PIPE_MMIO     (1)
    ) u_soc (
        .clk      (clk_sys),
        .cnn_clk  (clk_cnn),
        .resetn   (resetn),
        .uart_tx  (uart_txd),
        .uart_rxd (uart_rxd),
        .key_in   (1'b1),
        .gpio_out (gpio),
        .cpu_stall(cpu_stall)
    );

    //-----------------------------------------------------------------
    // LED: reset released, heartbeat, door state, UART activity (active low).
    //-----------------------------------------------------------------
    reg [23:0] heartbeat = 24'h0;
    always @(posedge clk_sys) heartbeat <= heartbeat + 24'h1;

    wire [7:0] gpio_led = {4'b0, ~uart_txd, gpio[0], heartbeat[23], resetn};

    genvar g;
    generate
        if (LED_ACTIVE_LOW) begin : g_led_n
            for (g = 0; g < 4; g = g + 1) begin : b
                assign led_out[g] = ~gpio_led[g];
            end
        end else begin : g_led_p
            assign led_out = gpio_led[3:0];
        end
    endgenerate

endmodule
