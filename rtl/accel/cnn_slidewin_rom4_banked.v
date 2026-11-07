// Four 320x240 gray frames, split into 64K-word groups with explicit init files.
// The two data stages match the original rq + fw_d enabled-edge latency.
module cnn_slidewin_rom4_banked #(parameter AW=20)(
    input wire clk,input wire[AW-1:0] address,output wire[7:0] pixel);
    reg[AW-17:0] select_s1,select_s2;
    always @(posedge clk)begin
        select_s1<=address[AW-1:16];select_s2<=select_s1;
    end
    wire[39:0] words;
    genvar bank;
    generate for(bank=0;bank<5;bank=bank+1)begin:g_bank
        localparam DEPTH=(bank==4)?45056:65536;
        reg[7:0]rom[0:DEPTH-1];
        if(bank==0)begin:load0 initial $readmemh("rtl/accel/mem/slidewin_bank0.mem",rom);end
        if(bank==1)begin:load1 initial $readmemh("rtl/accel/mem/slidewin_bank1.mem",rom);end
        if(bank==2)begin:load2 initial $readmemh("rtl/accel/mem/slidewin_bank2.mem",rom);end
        if(bank==3)begin:load3 initial $readmemh("rtl/accel/mem/slidewin_bank3.mem",rom);end
        if(bank==4)begin:load4 initial $readmemh("rtl/accel/mem/slidewin_bank4.mem",rom);end
        reg[7:0] read_s1;
        // This edge must stay in fabric, after the cascaded DRM outputs.
        (* syn_preserve=1 *) reg[7:0] read_s2;
        always @(posedge clk)begin read_s1<=rom[address[15:0]];read_s2<=read_s1;end
        assign words[bank*8+:8]=read_s2;
    end endgenerate
    assign pixel=words[select_s2*8+:8];
endmodule
