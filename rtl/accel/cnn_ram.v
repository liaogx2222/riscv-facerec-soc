// Exact-depth simple dual-port RAM. Non-power-of-two depths use two
// power-of-two segments; synchronous read/hold and read-before-write are kept.
`timescale 1ns/1ps
module cnn_sdpram #(parameter W=64,D=8192,AW=13)(
 input clk,input we,input [AW-1:0] waddr,input [W-1:0] wdata,
 input re,input [AW-1:0] raddr,output wire [W-1:0] rdata);
 localparam SPLIT=(D & (D-1))!=0;
 localparam HD=SPLIT?(1<<($clog2(D)-1)):D;
 localparam TD=SPLIT?(D-HD):1;
 localparam HAW=$clog2(HD)>0?$clog2(HD):1;
 localparam TAW=$clog2(TD)>0?$clog2(TD):1;
 // Inactive arrays have no hardware users and are eliminated in synthesis.
 reg [W-1:0] mem[0:D-1],head[0:HD-1],tail[0:TD-1];
 generate if(!SPLIT) begin:g_pow2
  reg [W-1:0] q;
  always @(posedge clk) begin
   if(we) mem[waddr]<=wdata;
   if(re) q<=mem[raddr];
  end
  assign rdata=q;
 end else begin:g_split
  reg [W-1:0] hq,tq; reg sel_q,invalid_q;
  always @(posedge clk) begin
   if(we && waddr<HD) head[waddr[HAW-1:0]]<=wdata;
   if(we && waddr>=HD && waddr<D) tail[waddr[TAW-1:0]]<=wdata;
   if(re) begin
    sel_q<=raddr>=HD; invalid_q<=raddr>=D;
    // Read both physical segments. Only the registered selected output is
    // observable; remove the address/range decode from the DRM read CE.
    hq<=head[raddr[HAW-1:0]];
    tq<=tail[raddr[TAW-1:0]];
   end
  end
  assign rdata=invalid_q?{W{1'bx}}:(sel_q?tq:hq);
 end endgenerate
`ifdef FACEREC_SIM
 // TB initialization/readback only. Simulation and hardware use the same
 // segmented port datapath above; these hooks add no alternative RAM model.
 task sim_write;
  input integer address;input [W-1:0] value;
  begin
   if(address>=0 && address<D) begin
    if(!SPLIT) mem[address]=value;
    else if(address<HD) head[address]=value;
    else tail[address-HD]=value;
   end
  end
 endtask
 function [W-1:0] sim_read;
  input integer address;
  begin
   if(address<0 || address>=D) sim_read={W{1'bx}};
   else if(!SPLIT) sim_read=mem[address];
   else if(address<HD) sim_read=head[address];
   else sim_read=tail[address-HD];
  end
 endfunction
`endif
endmodule
