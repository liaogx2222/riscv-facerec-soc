// PFD27MHz, VCO=27*10*3=810MHz; /8 CPU101.25, /4 CNN202.5.
// All divider values are integers; both output duties are 50%.
`timescale 1ns/1ps
module pll(input wire clkin1, output wire clkout0, output wire clkout1, output wire lock);
    wire clk_raw,cnn_raw;
    GTP_GPLL #(
        .CLKIN_FREQ(27.0), .STATIC_RATIOI(1), .STATIC_RATIOM(10),
        .STATIC_RATIOF(3.0), .STATIC_RATIO0(8.0),
        .STATIC_RATIO1(4), .STATIC_RATIO2(9), .STATIC_RATIO3(9),
        .STATIC_RATIO4(9), .STATIC_RATIO5(9), .STATIC_RATIO6(9),
        .STATIC_DUTY0(8), .STATIC_DUTY1(4), .STATIC_DUTYF(3),
        .INTERNAL_FB("CLKOUTF"), .EXTERNAL_FB("DISABLE")
    ) u_gpll (
        .CLKIN1(clkin1), .CLKIN2(1'b0), .CLKIN_SEL(1'b0), .CLKFB(1'b0),
        .CLKOUT0(clk_raw), .CLKOUT1(cnn_raw), .LOCK(lock), .RST(1'b0), .PLL_PWD(1'b0),
        .DPS_CLK(1'b0), .DPS_EN(1'b0), .DPS_DIR(1'b0),
        .CLKOUT0_SYN(1'b0), .CLKOUT1_SYN(1'b0), .CLKOUT2_SYN(1'b0),
        .CLKOUT3_SYN(1'b0), .CLKOUT4_SYN(1'b0), .CLKOUT5_SYN(1'b0),
        .CLKOUT6_SYN(1'b0), .CLKOUTF_SYN(1'b0),
        .APB_CLK(1'b0), .APB_RST_N(1'b0), .APB_SEL(1'b0),
        .APB_EN(1'b0), .APB_WRITE(1'b0), .APB_ADDR(5'b0), .APB_WDATA(16'b0)
    );
    GTP_CLKBUFG u_bufg(clkout0, clk_raw);
    GTP_CLKBUFG u_bufg_cnn(clkout1, cnn_raw);
endmodule
