`timescale 1ns/1ps
// Preserve the add-before-select boundary when pair eligibility arrives late.
module pc_inc4 /* synthesis syn_hier="hard" */ (
    input wire [31:0] a,
    output wire [31:0] y
);
    assign y = a + 32'd4;
endmodule
