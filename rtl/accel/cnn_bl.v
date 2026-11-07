// Integer bilinear interpolation, four registered arithmetic stages.
// Products never cascade through another multiplier or adder in one cycle.
// y = (a*(256-fx)*(256-fy)+b*fx*(256-fy)+c*(256-fx)*fy+d*fx*fy+32768)>>16.
`timescale 1ns/1ps
module cnn_bl(input wire clk,v_i,input wire [7:0] a,b,c,d,fx,fy,
              output wire v_o,output wire [7:0] y);
    wire [8:0] wx0=9'd256-{1'b0,fx},wy0=9'd256-{1'b0,fy};
    reg [16:0] ha_q,hb_q,hc_q,hd_q;
    reg [17:0] h0_q,h1_q;
    reg [8:0] wy1_q,wy2_q;
    reg [7:0] fy1_q,fy2_q;
    reg [26:0] m0_q,m1_q;
    reg [27:0] sum_q;
    reg [3:0] valid_q;
    always @(posedge clk) begin
        ha_q<=a*wx0;hb_q<=b*{1'b0,fx};
        hc_q<=c*wx0;hd_q<=d*{1'b0,fx};
        wy1_q<=wy0;fy1_q<=fy;
        h0_q<={1'b0,ha_q}+{1'b0,hb_q};
        h1_q<={1'b0,hc_q}+{1'b0,hd_q};
        wy2_q<=wy1_q;fy2_q<=fy1_q;
        m0_q<=h0_q*wy2_q;m1_q<=h1_q*{1'b0,fy2_q};
        sum_q<={1'b0,m0_q}+{1'b0,m1_q}+28'd32768;
        valid_q<={valid_q[2:0],v_i};
    end
    assign v_o=valid_q[3];
    assign y=(sum_q[27:16]>12'd255)?8'hff:sum_q[23:16];
endmodule
