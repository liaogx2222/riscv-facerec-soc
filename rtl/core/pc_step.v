`timescale 1ns/1ps
// Keep constant address arithmetic before late recovery/issue selection.
module pc_step #(
    parameter [31:0] STEP = 32'd4
) /* synthesis syn_hier="hard" */ (
    input wire [31:0] a,
    output wire [31:0] y
);
    assign y = a + STEP;
endmodule
