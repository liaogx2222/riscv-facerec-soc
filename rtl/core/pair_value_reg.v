`timescale 1ns/1ps
// Same EX/MEM boundary as the architectural payload. WIDTH also permits
// capturing only the link classification while sharing existing data regs.
module pair_value_reg /* synthesis syn_hier="hard" */ #(
    parameter WIDTH = 32
) (
    input wire clk,
    input wire resetn,
    input wire en,
    input wire [WIDTH-1:0] d,
    output reg [WIDTH-1:0] q
);
    always @(posedge clk) begin
        if (!resetn) q <= {WIDTH{1'b0}};
        else if (en) q <= d;
    end
endmodule
