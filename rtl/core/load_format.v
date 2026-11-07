// Format independent RAM/SB words before the late forwarding choice.
`include "rv32i_defs.v"
module load_format /* synthesis syn_hier="hard" */(
    input wire [31:0] word_in,input wire [1:0] byte_off,
    input wire [2:0] funct3,output reg [31:0] value);
    wire [31:0] shifted=word_in >> {byte_off,3'b000};
    always @* case(funct3)
        `F3_LB: value={{24{shifted[7]}},shifted[7:0]};
        `F3_LBU:value={24'b0,shifted[7:0]};
        `F3_LH: value={{16{shifted[15]}},shifted[15:0]};
        `F3_LHU:value={16'b0,shifted[15:0]};
        default:value=word_in;
    endcase
endmodule
