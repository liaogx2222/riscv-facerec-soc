// Compare qualified RAM word indexes before the late lane selector.
module sb_cam_match /* synthesis syn_hier="hard" */ #(parameter WORD_BITS=30)(
 input wire [WORD_BITS-1:0] word_addr,lane0_addr,lane1_addr,
 input wire lane1,output wire match);
 wire match0,match1;
 sb_word_eq #(.WORD_BITS(WORD_BITS)) eq0(.a(word_addr),.b(lane0_addr),.eq(match0));
 sb_word_eq #(.WORD_BITS(WORD_BITS)) eq1(.a(word_addr),.b(lane1_addr),.eq(match1));
 assign match=lane1?match1:match0;
endmodule
module sb_word_eq /* synthesis syn_hier="hard" */ #(parameter WORD_BITS=30)(
 input wire [WORD_BITS-1:0] a,b,output wire eq);
 assign eq=a==b;
endmodule

module sb_ram_scope /* synthesis syn_hier="hard" */ #(parameter RAM_AW=17)(
 input wire [31:0] addr0,addr1,input wire lane1,output wire is_ram);
 wire ram0,ram1;
 sb_high_zero #(.N(32-RAM_AW)) z0(.a(addr0[31:RAM_AW]),.zero(ram0));
 sb_high_zero #(.N(32-RAM_AW)) z1(.a(addr1[31:RAM_AW]),.zero(ram1));
 assign is_ram=lane1?ram1:ram0;
endmodule
module sb_high_zero /* synthesis syn_hier="hard" */ #(parameter N=15)(
 input wire [N-1:0] a,output wire zero);
 assign zero=~|a;
endmodule
