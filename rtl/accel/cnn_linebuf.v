// Banked 4-row line buffer, derived from FaceRec_SoC cnn_linebuf.
// Four logical rows x three interleaved column banks; one read per bank.
// Payload RAM is reset-free; per-word validity implements synchronous clear.
// 3x3 and PW windows remain combinational, with the original cycle protocol.
// Eight-bit coordinate quotient/remainder uses a complete constant lookup,
// avoiding runtime signed division/modulo unsupported by PDS ADS.
// PW selects the center row and wraps c to WAW bits as in the source design.
// Source provenance and equivalence/resource evidence are in doc/ and build/.
`timescale 1ns/1ps

(* syn_romstyle="logic" *) module cnn_linebuf #(
    parameter CH   = 32,
    parameter AW   = 8,
    parameter MAXW = 128,
    parameter TAPS = 9,
    parameter WAW  = 7,           // 列地址位宽（MAXW=128 ⇒ 7）
    parameter RAW  = 9            // 行缓冲地址位宽（4*MAXW=512 ⇒ 9）
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    clr,          // 层首/复位：整组清零 + rot_p 归零
    input  wire                    win_top_zero, // 输出行 r==0：窗口顶行（输入行 -1）强制零
    input  wire                    win_bot_zero, // ★ 窗口底行（输入行 r+1 ≥ ih）强制零
                                                 //   —— stride-1 末输出行的底行越界，
                                                 //     调度器仍会装载（读 ACT 越界地址，
                                                 //     数据被这里掩掉），拍数不变
    input  wire                    mode_pw,      // ★ 1 = 1×1 层：窗口退化为 1×1（9 tap 同一列字）
    input  wire                    rot_step2,    // ★ 1 = 每次 rot 前进 2 行（stride-2）；0 = 前进 1 行
    input  wire                    wld,          // 装载一拍（写 row_buf[ldst][waddr]）
    input  wire [1:0]              ldst,         // 装载目标物理行（0..3）
    input  wire [WAW-1:0]          waddr,        // 装载列地址
    input  wire [CH*AW-1:0]        newword,      // 装载数据（CH 字节）
    input  wire                    rot,          // 行旋转
    input  wire [7:0]              iw,           // 本层行宽（运行时；★M8 拓到 8 位）
    input  wire [7:0]              c,            // 当前输出像素列（0..iw-1）
    output wire [1:0]              rot_p_o,      // 行环指针（控制器算装载目标用）
    output reg  [TAPS*CH*AW-1:0]   win           // 组合窗口输出
);

    // Complete eight-bit quotient/remainder table: no runtime div/mod.
    function [8:0] divmod3;
        input [7:0] x;
        begin
            case(x)
                8'd0: divmod3=9'd0;
                8'd1: divmod3=9'd1;
                8'd2: divmod3=9'd2;
                8'd3: divmod3=9'd4;
                8'd4: divmod3=9'd5;
                8'd5: divmod3=9'd6;
                8'd6: divmod3=9'd8;
                8'd7: divmod3=9'd9;
                8'd8: divmod3=9'd10;
                8'd9: divmod3=9'd12;
                8'd10: divmod3=9'd13;
                8'd11: divmod3=9'd14;
                8'd12: divmod3=9'd16;
                8'd13: divmod3=9'd17;
                8'd14: divmod3=9'd18;
                8'd15: divmod3=9'd20;
                8'd16: divmod3=9'd21;
                8'd17: divmod3=9'd22;
                8'd18: divmod3=9'd24;
                8'd19: divmod3=9'd25;
                8'd20: divmod3=9'd26;
                8'd21: divmod3=9'd28;
                8'd22: divmod3=9'd29;
                8'd23: divmod3=9'd30;
                8'd24: divmod3=9'd32;
                8'd25: divmod3=9'd33;
                8'd26: divmod3=9'd34;
                8'd27: divmod3=9'd36;
                8'd28: divmod3=9'd37;
                8'd29: divmod3=9'd38;
                8'd30: divmod3=9'd40;
                8'd31: divmod3=9'd41;
                8'd32: divmod3=9'd42;
                8'd33: divmod3=9'd44;
                8'd34: divmod3=9'd45;
                8'd35: divmod3=9'd46;
                8'd36: divmod3=9'd48;
                8'd37: divmod3=9'd49;
                8'd38: divmod3=9'd50;
                8'd39: divmod3=9'd52;
                8'd40: divmod3=9'd53;
                8'd41: divmod3=9'd54;
                8'd42: divmod3=9'd56;
                8'd43: divmod3=9'd57;
                8'd44: divmod3=9'd58;
                8'd45: divmod3=9'd60;
                8'd46: divmod3=9'd61;
                8'd47: divmod3=9'd62;
                8'd48: divmod3=9'd64;
                8'd49: divmod3=9'd65;
                8'd50: divmod3=9'd66;
                8'd51: divmod3=9'd68;
                8'd52: divmod3=9'd69;
                8'd53: divmod3=9'd70;
                8'd54: divmod3=9'd72;
                8'd55: divmod3=9'd73;
                8'd56: divmod3=9'd74;
                8'd57: divmod3=9'd76;
                8'd58: divmod3=9'd77;
                8'd59: divmod3=9'd78;
                8'd60: divmod3=9'd80;
                8'd61: divmod3=9'd81;
                8'd62: divmod3=9'd82;
                8'd63: divmod3=9'd84;
                8'd64: divmod3=9'd85;
                8'd65: divmod3=9'd86;
                8'd66: divmod3=9'd88;
                8'd67: divmod3=9'd89;
                8'd68: divmod3=9'd90;
                8'd69: divmod3=9'd92;
                8'd70: divmod3=9'd93;
                8'd71: divmod3=9'd94;
                8'd72: divmod3=9'd96;
                8'd73: divmod3=9'd97;
                8'd74: divmod3=9'd98;
                8'd75: divmod3=9'd100;
                8'd76: divmod3=9'd101;
                8'd77: divmod3=9'd102;
                8'd78: divmod3=9'd104;
                8'd79: divmod3=9'd105;
                8'd80: divmod3=9'd106;
                8'd81: divmod3=9'd108;
                8'd82: divmod3=9'd109;
                8'd83: divmod3=9'd110;
                8'd84: divmod3=9'd112;
                8'd85: divmod3=9'd113;
                8'd86: divmod3=9'd114;
                8'd87: divmod3=9'd116;
                8'd88: divmod3=9'd117;
                8'd89: divmod3=9'd118;
                8'd90: divmod3=9'd120;
                8'd91: divmod3=9'd121;
                8'd92: divmod3=9'd122;
                8'd93: divmod3=9'd124;
                8'd94: divmod3=9'd125;
                8'd95: divmod3=9'd126;
                8'd96: divmod3=9'd128;
                8'd97: divmod3=9'd129;
                8'd98: divmod3=9'd130;
                8'd99: divmod3=9'd132;
                8'd100: divmod3=9'd133;
                8'd101: divmod3=9'd134;
                8'd102: divmod3=9'd136;
                8'd103: divmod3=9'd137;
                8'd104: divmod3=9'd138;
                8'd105: divmod3=9'd140;
                8'd106: divmod3=9'd141;
                8'd107: divmod3=9'd142;
                8'd108: divmod3=9'd144;
                8'd109: divmod3=9'd145;
                8'd110: divmod3=9'd146;
                8'd111: divmod3=9'd148;
                8'd112: divmod3=9'd149;
                8'd113: divmod3=9'd150;
                8'd114: divmod3=9'd152;
                8'd115: divmod3=9'd153;
                8'd116: divmod3=9'd154;
                8'd117: divmod3=9'd156;
                8'd118: divmod3=9'd157;
                8'd119: divmod3=9'd158;
                8'd120: divmod3=9'd160;
                8'd121: divmod3=9'd161;
                8'd122: divmod3=9'd162;
                8'd123: divmod3=9'd164;
                8'd124: divmod3=9'd165;
                8'd125: divmod3=9'd166;
                8'd126: divmod3=9'd168;
                8'd127: divmod3=9'd169;
                8'd128: divmod3=9'd170;
                8'd129: divmod3=9'd172;
                8'd130: divmod3=9'd173;
                8'd131: divmod3=9'd174;
                8'd132: divmod3=9'd176;
                8'd133: divmod3=9'd177;
                8'd134: divmod3=9'd178;
                8'd135: divmod3=9'd180;
                8'd136: divmod3=9'd181;
                8'd137: divmod3=9'd182;
                8'd138: divmod3=9'd184;
                8'd139: divmod3=9'd185;
                8'd140: divmod3=9'd186;
                8'd141: divmod3=9'd188;
                8'd142: divmod3=9'd189;
                8'd143: divmod3=9'd190;
                8'd144: divmod3=9'd192;
                8'd145: divmod3=9'd193;
                8'd146: divmod3=9'd194;
                8'd147: divmod3=9'd196;
                8'd148: divmod3=9'd197;
                8'd149: divmod3=9'd198;
                8'd150: divmod3=9'd200;
                8'd151: divmod3=9'd201;
                8'd152: divmod3=9'd202;
                8'd153: divmod3=9'd204;
                8'd154: divmod3=9'd205;
                8'd155: divmod3=9'd206;
                8'd156: divmod3=9'd208;
                8'd157: divmod3=9'd209;
                8'd158: divmod3=9'd210;
                8'd159: divmod3=9'd212;
                8'd160: divmod3=9'd213;
                8'd161: divmod3=9'd214;
                8'd162: divmod3=9'd216;
                8'd163: divmod3=9'd217;
                8'd164: divmod3=9'd218;
                8'd165: divmod3=9'd220;
                8'd166: divmod3=9'd221;
                8'd167: divmod3=9'd222;
                8'd168: divmod3=9'd224;
                8'd169: divmod3=9'd225;
                8'd170: divmod3=9'd226;
                8'd171: divmod3=9'd228;
                8'd172: divmod3=9'd229;
                8'd173: divmod3=9'd230;
                8'd174: divmod3=9'd232;
                8'd175: divmod3=9'd233;
                8'd176: divmod3=9'd234;
                8'd177: divmod3=9'd236;
                8'd178: divmod3=9'd237;
                8'd179: divmod3=9'd238;
                8'd180: divmod3=9'd240;
                8'd181: divmod3=9'd241;
                8'd182: divmod3=9'd242;
                8'd183: divmod3=9'd244;
                8'd184: divmod3=9'd245;
                8'd185: divmod3=9'd246;
                8'd186: divmod3=9'd248;
                8'd187: divmod3=9'd249;
                8'd188: divmod3=9'd250;
                8'd189: divmod3=9'd252;
                8'd190: divmod3=9'd253;
                8'd191: divmod3=9'd254;
                8'd192: divmod3=9'd256;
                8'd193: divmod3=9'd257;
                8'd194: divmod3=9'd258;
                8'd195: divmod3=9'd260;
                8'd196: divmod3=9'd261;
                8'd197: divmod3=9'd262;
                8'd198: divmod3=9'd264;
                8'd199: divmod3=9'd265;
                8'd200: divmod3=9'd266;
                8'd201: divmod3=9'd268;
                8'd202: divmod3=9'd269;
                8'd203: divmod3=9'd270;
                8'd204: divmod3=9'd272;
                8'd205: divmod3=9'd273;
                8'd206: divmod3=9'd274;
                8'd207: divmod3=9'd276;
                8'd208: divmod3=9'd277;
                8'd209: divmod3=9'd278;
                8'd210: divmod3=9'd280;
                8'd211: divmod3=9'd281;
                8'd212: divmod3=9'd282;
                8'd213: divmod3=9'd284;
                8'd214: divmod3=9'd285;
                8'd215: divmod3=9'd286;
                8'd216: divmod3=9'd288;
                8'd217: divmod3=9'd289;
                8'd218: divmod3=9'd290;
                8'd219: divmod3=9'd292;
                8'd220: divmod3=9'd293;
                8'd221: divmod3=9'd294;
                8'd222: divmod3=9'd296;
                8'd223: divmod3=9'd297;
                8'd224: divmod3=9'd298;
                8'd225: divmod3=9'd300;
                8'd226: divmod3=9'd301;
                8'd227: divmod3=9'd302;
                8'd228: divmod3=9'd304;
                8'd229: divmod3=9'd305;
                8'd230: divmod3=9'd306;
                8'd231: divmod3=9'd308;
                8'd232: divmod3=9'd309;
                8'd233: divmod3=9'd310;
                8'd234: divmod3=9'd312;
                8'd235: divmod3=9'd313;
                8'd236: divmod3=9'd314;
                8'd237: divmod3=9'd316;
                8'd238: divmod3=9'd317;
                8'd239: divmod3=9'd318;
                8'd240: divmod3=9'd320;
                8'd241: divmod3=9'd321;
                8'd242: divmod3=9'd322;
                8'd243: divmod3=9'd324;
                8'd244: divmod3=9'd325;
                8'd245: divmod3=9'd326;
                8'd246: divmod3=9'd328;
                8'd247: divmod3=9'd329;
                8'd248: divmod3=9'd330;
                8'd249: divmod3=9'd332;
                8'd250: divmod3=9'd333;
                8'd251: divmod3=9'd334;
                8'd252: divmod3=9'd336;
                8'd253: divmod3=9'd337;
                8'd254: divmod3=9'd338;
                8'd255: divmod3=9'd340;
                default: divmod3=9'bxxxxxxxxx;
            endcase
        end
    endfunction

    localparam DW = CH*AW;
    localparam BANK_DEPTH = (MAXW+2)/3;
    localparam BAW = BANK_DEPTH>1 ? $clog2(BANK_DEPTH) : 1;
    reg [1:0] rot_p;
    assign rot_p_o=rot_p;
    always @(posedge clk) begin
        if (!rst_n || clr) rot_p<=0;
        else if (rot) rot_p<=rot_p+(rot_step2 ? 2'd2 : 2'd1);
    end

    // Each physical row has three interleaved column banks. Three adjacent
    // columns use different banks, so every payload array needs one read.
    wire [7:0] ci=mode_pw ? c[WAW-1:0] : c;
    wire [8:0] read_divmod=divmod3(ci);
    wire [6:0] cq=read_divmod[8:2];
    wire [1:0] cr=read_divmod[1:0];
    // Move every RAM operation one edge together. Delaying just the read
    // address would change same-address read/write and clear semantics.
    reg memory_reset_q,memory_clear_q,write_load_q;
    reg [1:0] write_row_q;
    // The original binary address shadow let ADS retime this table into
    // a synchronous DRM ROM. Capture its decoded result in fabric instead.
    wire [8:0] write_decode=divmod3(waddr);
    (* syn_preserve=1, syn_romstyle="logic" *) reg [8:0] write_decode_q;
    (* syn_preserve=1 *) reg write_in_range_q;
    reg [DW-1:0] write_word_q;
    always @(posedge clk)begin
        memory_reset_q<=rst_n;memory_clear_q<=clr;write_load_q<=wld;
        write_row_q<=ldst;write_decode_q<=write_decode;
        write_in_range_q<=waddr<MAXW;write_word_q<=newword;
    end
    wire [8:0] write_divmod=write_decode_q;
    wire [6:0] write_idx=write_divmod[8:2];
    wire [1:0] write_bank=write_divmod[1:0];
    wire [12*DW-1:0] bank_word;
    wire [11:0] bank_valid;
    genvar row,bank;
    generate for(row=0;row<4;row=row+1) begin:g_row
        // Keep the scalar validity address off the 256-bit payload fanout.
        wire [3*BAW-1:0] valid_read_indexes;
        // ADS compiles away equivalent instances despite syn_hier="hard".
        // Official syn_noprune is verified by the decoder attribute probe.
        (* syn_noprune=1 *) cnn_linebuf_read_addr #(.WAW(WAW),.BANK_DEPTH(BANK_DEPTH),.BAW(BAW)) u_valid_addr(
            .c(c),.mode_pw(mode_pw),.indexes(valid_read_indexes));
        wire [3*BAW-1:0] row_read_indexes;
        cnn_linebuf_read_addr #(.WAW(WAW),.BANK_DEPTH(BANK_DEPTH),.BAW(BAW)) u_addr(
            .c(c),.mode_pw(mode_pw),.indexes(row_read_indexes));
        (* syn_preserve=1 *) reg [3*BAW-1:0] payload_indexes_q;
        (* syn_preserve=1 *) reg [3*BAW-1:0] validity_indexes_q;
        always @(posedge clk)begin
            payload_indexes_q<=row_read_indexes;
            validity_indexes_q<=valid_read_indexes;
        end
        for(bank=0;bank<3;bank=bank+1) begin:g_bank
            reg [DW-1:0] payload[0:BANK_DEPTH-1] /* synthesis syn_ramstyle="select_ram" */;
            reg [BANK_DEPTH-1:0] valid;
            wire wen=write_load_q && write_row_q==row && write_bank==bank && write_in_range_q;
            always @(posedge clk)
                if(memory_reset_q && !memory_clear_q && wen) payload[write_idx[BAW-1:0]]<=write_word_q;
            always @(posedge clk) begin
                if(!memory_reset_q || memory_clear_q) valid<=0;
                else if(wen) valid[write_idx[BAW-1:0]]<=1'b1;
            end
            reg [DW-1:0] read_q;
            reg read_valid_q;
            // Sample payload and its validity at the same edge. Applying
            // the zero mask afterwards preserves read-before-write and
            // four-state semantics, without a wide synchronous-reset cone.
            always @(posedge clk) begin
                read_q <= payload[payload_indexes_q[bank*BAW+:BAW]];
                read_valid_q <= valid[validity_indexes_q[bank*BAW+:BAW]];
            end
            assign bank_word[(row*3+bank)*DW +: DW]=read_q;
            assign bank_valid[row*3+bank]=read_valid_q;
        end
    end endgenerate

    // Register decoded selectors/masks alongside the bank data. Boundary
    // comparison and coordinate arithmetic no longer precede DSP input setup.
    localparam NREADROWS=(TAPS+2)/3;
    reg [1:0] row_d[0:NREADROWS-1],row_q[0:NREADROWS-1];
    reg [1:0] col_d[0:2],col_q[0:2];
    integer init_i;
    reg [TAPS-1:0] keep_d,keep_q;
    integer t,tr,tc,bank_tmp,scol;
    reg [1:0] physical_row,column_bank;
    reg [3:0] data_index;
    always @(*) begin
        keep_d=0;tr=0;tc=0;physical_row=0;column_bank=0;
        bank_tmp=0;scol=0;data_index=0;
        for(init_i=0;init_i<NREADROWS;init_i=init_i+1)row_d[init_i]=0;
        for(init_i=0;init_i<3;init_i=init_i+1)col_d[init_i]=0;
        for(t=0;t<TAPS;t=t+1) begin
            tr=t/3;tc=t%3;
            if(mode_pw) begin
                physical_row=(rot_p+1)&3;column_bank=cr;
                data_index={physical_row,1'b0}+physical_row+column_bank;
                row_d[tr]=physical_row;col_d[tc]=column_bank;keep_d[t]=1'b1;
            end else begin
                physical_row=(rot_p+tr)&3;
                bank_tmp=cr+tc-1;
                if(bank_tmp<0) bank_tmp=bank_tmp+3;
                if(bank_tmp>=3) bank_tmp=bank_tmp-3;
                column_bank=bank_tmp[1:0];scol=ci+tc-1;
                data_index={physical_row,1'b0}+physical_row+column_bank;
                row_d[tr]=physical_row;col_d[tc]=column_bank;
                // Procedural if preserves the reference's four-state mask
                // behavior, including unknown boundary control inputs.
                if((win_top_zero && tr==0)||(win_bot_zero && tr==2)||scol<0||scol>=iw)
                    keep_d[t]=1'b0;
                else keep_d[t]=1'b1;
            end
        end
    end
    reg [1:0] row_index_q[0:NREADROWS-1],col_index_q[0:2];
    reg [TAPS-1:0] keep_index_q;
    integer k;
    always @(posedge clk) begin
        for(k=0;k<NREADROWS;k=k+1)begin row_index_q[k]<=row_d[k];row_q[k]<=row_index_q[k];end
        for(k=0;k<3;k=k+1)begin col_index_q[k]<=col_d[k];col_q[k]<=col_index_q[k];end
        keep_index_q<=keep_d;keep_q<=keep_index_q;
    end
    reg [1:0] col_pipe_q[0:2];
    reg [TAPS-1:0] keep_pipe_q;
    integer pipe_col;
    always @(posedge clk)begin
        for(pipe_col=0;pipe_col<3;pipe_col=pipe_col+1)col_pipe_q[pipe_col]<=col_q[pipe_col];
        keep_pipe_q<=keep_q;
    end
    wire [TAPS*DW-1:0] window_word;
    genvar rr,bb,cc;
    generate for(rr=0;rr<NREADROWS;rr=rr+1)begin:g_read_row
        wire [3*DW-1:0] logical_banks;
        reg [3*DW-1:0] logical_banks_q;
        always @(posedge clk)logical_banks_q<=logical_banks;
        for(bb=0;bb<3;bb=bb+1)begin:g_read_bank
            cnn_lb_row_pick #(.DW(DW)) u_pick(
                .words({bank_word[(9+bb)*DW+:DW],bank_word[(6+bb)*DW+:DW],
                        bank_word[(3+bb)*DW+:DW],bank_word[bb*DW+:DW]}),
                .valids({bank_valid[9+bb],bank_valid[6+bb],bank_valid[3+bb],bank_valid[bb]}),
                .sel(row_q[rr]),.value(logical_banks[bb*DW+:DW]));
        end
        for(cc=0;cc<3;cc=cc+1)begin:g_read_col
            if(rr*3+cc<TAPS)begin:g_tap
                cnn_lb_col_pick #(.DW(DW)) u_pick(.words(logical_banks_q),
                    .sel(col_pipe_q[cc]),.value(window_word[(rr*3+cc)*DW+:DW]));
            end
        end
    end endgenerate
    integer o;
    always @(*) begin
        win=0;
        for(o=0;o<TAPS;o=o+1) begin
            if(!keep_pipe_q[o]) win[o*DW+:DW]=0;
            else win[o*DW+:DW]=window_word[o*DW+:DW];
        end
    end
endmodule

// Final bank indexes are constant functions of c and PW wrap semantics.
module cnn_linebuf_read_addr /* synthesis syn_hier="hard" */ #(
    parameter WAW=7,BANK_DEPTH=43,BAW=6)(
    input wire [7:0] c,input wire mode_pw,output wire [3*BAW-1:0] indexes);
    function [BAW-1:0] bound;
        input integer n;
        begin if(n>=0 && n<BANK_DEPTH)bound=n;else bound=0;end
    endfunction
    function [3*BAW-1:0] decode_read;
        input [7:0] coordinate;input pw;
        begin
        case({pw,coordinate})
            9'd0:decode_read={bound(-1),bound(0),bound(0)};
            9'd1:decode_read={bound(0),bound(0),bound(0)};
            9'd2:decode_read={bound(0),bound(0),bound(1)};
            9'd3:decode_read={bound(0),bound(1),bound(1)};
            9'd4:decode_read={bound(1),bound(1),bound(1)};
            9'd5:decode_read={bound(1),bound(1),bound(2)};
            9'd6:decode_read={bound(1),bound(2),bound(2)};
            9'd7:decode_read={bound(2),bound(2),bound(2)};
            9'd8:decode_read={bound(2),bound(2),bound(3)};
            9'd9:decode_read={bound(2),bound(3),bound(3)};
            9'd10:decode_read={bound(3),bound(3),bound(3)};
            9'd11:decode_read={bound(3),bound(3),bound(4)};
            9'd12:decode_read={bound(3),bound(4),bound(4)};
            9'd13:decode_read={bound(4),bound(4),bound(4)};
            9'd14:decode_read={bound(4),bound(4),bound(5)};
            9'd15:decode_read={bound(4),bound(5),bound(5)};
            9'd16:decode_read={bound(5),bound(5),bound(5)};
            9'd17:decode_read={bound(5),bound(5),bound(6)};
            9'd18:decode_read={bound(5),bound(6),bound(6)};
            9'd19:decode_read={bound(6),bound(6),bound(6)};
            9'd20:decode_read={bound(6),bound(6),bound(7)};
            9'd21:decode_read={bound(6),bound(7),bound(7)};
            9'd22:decode_read={bound(7),bound(7),bound(7)};
            9'd23:decode_read={bound(7),bound(7),bound(8)};
            9'd24:decode_read={bound(7),bound(8),bound(8)};
            9'd25:decode_read={bound(8),bound(8),bound(8)};
            9'd26:decode_read={bound(8),bound(8),bound(9)};
            9'd27:decode_read={bound(8),bound(9),bound(9)};
            9'd28:decode_read={bound(9),bound(9),bound(9)};
            9'd29:decode_read={bound(9),bound(9),bound(10)};
            9'd30:decode_read={bound(9),bound(10),bound(10)};
            9'd31:decode_read={bound(10),bound(10),bound(10)};
            9'd32:decode_read={bound(10),bound(10),bound(11)};
            9'd33:decode_read={bound(10),bound(11),bound(11)};
            9'd34:decode_read={bound(11),bound(11),bound(11)};
            9'd35:decode_read={bound(11),bound(11),bound(12)};
            9'd36:decode_read={bound(11),bound(12),bound(12)};
            9'd37:decode_read={bound(12),bound(12),bound(12)};
            9'd38:decode_read={bound(12),bound(12),bound(13)};
            9'd39:decode_read={bound(12),bound(13),bound(13)};
            9'd40:decode_read={bound(13),bound(13),bound(13)};
            9'd41:decode_read={bound(13),bound(13),bound(14)};
            9'd42:decode_read={bound(13),bound(14),bound(14)};
            9'd43:decode_read={bound(14),bound(14),bound(14)};
            9'd44:decode_read={bound(14),bound(14),bound(15)};
            9'd45:decode_read={bound(14),bound(15),bound(15)};
            9'd46:decode_read={bound(15),bound(15),bound(15)};
            9'd47:decode_read={bound(15),bound(15),bound(16)};
            9'd48:decode_read={bound(15),bound(16),bound(16)};
            9'd49:decode_read={bound(16),bound(16),bound(16)};
            9'd50:decode_read={bound(16),bound(16),bound(17)};
            9'd51:decode_read={bound(16),bound(17),bound(17)};
            9'd52:decode_read={bound(17),bound(17),bound(17)};
            9'd53:decode_read={bound(17),bound(17),bound(18)};
            9'd54:decode_read={bound(17),bound(18),bound(18)};
            9'd55:decode_read={bound(18),bound(18),bound(18)};
            9'd56:decode_read={bound(18),bound(18),bound(19)};
            9'd57:decode_read={bound(18),bound(19),bound(19)};
            9'd58:decode_read={bound(19),bound(19),bound(19)};
            9'd59:decode_read={bound(19),bound(19),bound(20)};
            9'd60:decode_read={bound(19),bound(20),bound(20)};
            9'd61:decode_read={bound(20),bound(20),bound(20)};
            9'd62:decode_read={bound(20),bound(20),bound(21)};
            9'd63:decode_read={bound(20),bound(21),bound(21)};
            9'd64:decode_read={bound(21),bound(21),bound(21)};
            9'd65:decode_read={bound(21),bound(21),bound(22)};
            9'd66:decode_read={bound(21),bound(22),bound(22)};
            9'd67:decode_read={bound(22),bound(22),bound(22)};
            9'd68:decode_read={bound(22),bound(22),bound(23)};
            9'd69:decode_read={bound(22),bound(23),bound(23)};
            9'd70:decode_read={bound(23),bound(23),bound(23)};
            9'd71:decode_read={bound(23),bound(23),bound(24)};
            9'd72:decode_read={bound(23),bound(24),bound(24)};
            9'd73:decode_read={bound(24),bound(24),bound(24)};
            9'd74:decode_read={bound(24),bound(24),bound(25)};
            9'd75:decode_read={bound(24),bound(25),bound(25)};
            9'd76:decode_read={bound(25),bound(25),bound(25)};
            9'd77:decode_read={bound(25),bound(25),bound(26)};
            9'd78:decode_read={bound(25),bound(26),bound(26)};
            9'd79:decode_read={bound(26),bound(26),bound(26)};
            9'd80:decode_read={bound(26),bound(26),bound(27)};
            9'd81:decode_read={bound(26),bound(27),bound(27)};
            9'd82:decode_read={bound(27),bound(27),bound(27)};
            9'd83:decode_read={bound(27),bound(27),bound(28)};
            9'd84:decode_read={bound(27),bound(28),bound(28)};
            9'd85:decode_read={bound(28),bound(28),bound(28)};
            9'd86:decode_read={bound(28),bound(28),bound(29)};
            9'd87:decode_read={bound(28),bound(29),bound(29)};
            9'd88:decode_read={bound(29),bound(29),bound(29)};
            9'd89:decode_read={bound(29),bound(29),bound(30)};
            9'd90:decode_read={bound(29),bound(30),bound(30)};
            9'd91:decode_read={bound(30),bound(30),bound(30)};
            9'd92:decode_read={bound(30),bound(30),bound(31)};
            9'd93:decode_read={bound(30),bound(31),bound(31)};
            9'd94:decode_read={bound(31),bound(31),bound(31)};
            9'd95:decode_read={bound(31),bound(31),bound(32)};
            9'd96:decode_read={bound(31),bound(32),bound(32)};
            9'd97:decode_read={bound(32),bound(32),bound(32)};
            9'd98:decode_read={bound(32),bound(32),bound(33)};
            9'd99:decode_read={bound(32),bound(33),bound(33)};
            9'd100:decode_read={bound(33),bound(33),bound(33)};
            9'd101:decode_read={bound(33),bound(33),bound(34)};
            9'd102:decode_read={bound(33),bound(34),bound(34)};
            9'd103:decode_read={bound(34),bound(34),bound(34)};
            9'd104:decode_read={bound(34),bound(34),bound(35)};
            9'd105:decode_read={bound(34),bound(35),bound(35)};
            9'd106:decode_read={bound(35),bound(35),bound(35)};
            9'd107:decode_read={bound(35),bound(35),bound(36)};
            9'd108:decode_read={bound(35),bound(36),bound(36)};
            9'd109:decode_read={bound(36),bound(36),bound(36)};
            9'd110:decode_read={bound(36),bound(36),bound(37)};
            9'd111:decode_read={bound(36),bound(37),bound(37)};
            9'd112:decode_read={bound(37),bound(37),bound(37)};
            9'd113:decode_read={bound(37),bound(37),bound(38)};
            9'd114:decode_read={bound(37),bound(38),bound(38)};
            9'd115:decode_read={bound(38),bound(38),bound(38)};
            9'd116:decode_read={bound(38),bound(38),bound(39)};
            9'd117:decode_read={bound(38),bound(39),bound(39)};
            9'd118:decode_read={bound(39),bound(39),bound(39)};
            9'd119:decode_read={bound(39),bound(39),bound(40)};
            9'd120:decode_read={bound(39),bound(40),bound(40)};
            9'd121:decode_read={bound(40),bound(40),bound(40)};
            9'd122:decode_read={bound(40),bound(40),bound(41)};
            9'd123:decode_read={bound(40),bound(41),bound(41)};
            9'd124:decode_read={bound(41),bound(41),bound(41)};
            9'd125:decode_read={bound(41),bound(41),bound(42)};
            9'd126:decode_read={bound(41),bound(42),bound(42)};
            9'd127:decode_read={bound(42),bound(42),bound(42)};
            9'd128:decode_read={bound(42),bound(42),bound(43)};
            9'd129:decode_read={bound(42),bound(43),bound(43)};
            9'd130:decode_read={bound(43),bound(43),bound(43)};
            9'd131:decode_read={bound(43),bound(43),bound(44)};
            9'd132:decode_read={bound(43),bound(44),bound(44)};
            9'd133:decode_read={bound(44),bound(44),bound(44)};
            9'd134:decode_read={bound(44),bound(44),bound(45)};
            9'd135:decode_read={bound(44),bound(45),bound(45)};
            9'd136:decode_read={bound(45),bound(45),bound(45)};
            9'd137:decode_read={bound(45),bound(45),bound(46)};
            9'd138:decode_read={bound(45),bound(46),bound(46)};
            9'd139:decode_read={bound(46),bound(46),bound(46)};
            9'd140:decode_read={bound(46),bound(46),bound(47)};
            9'd141:decode_read={bound(46),bound(47),bound(47)};
            9'd142:decode_read={bound(47),bound(47),bound(47)};
            9'd143:decode_read={bound(47),bound(47),bound(48)};
            9'd144:decode_read={bound(47),bound(48),bound(48)};
            9'd145:decode_read={bound(48),bound(48),bound(48)};
            9'd146:decode_read={bound(48),bound(48),bound(49)};
            9'd147:decode_read={bound(48),bound(49),bound(49)};
            9'd148:decode_read={bound(49),bound(49),bound(49)};
            9'd149:decode_read={bound(49),bound(49),bound(50)};
            9'd150:decode_read={bound(49),bound(50),bound(50)};
            9'd151:decode_read={bound(50),bound(50),bound(50)};
            9'd152:decode_read={bound(50),bound(50),bound(51)};
            9'd153:decode_read={bound(50),bound(51),bound(51)};
            9'd154:decode_read={bound(51),bound(51),bound(51)};
            9'd155:decode_read={bound(51),bound(51),bound(52)};
            9'd156:decode_read={bound(51),bound(52),bound(52)};
            9'd157:decode_read={bound(52),bound(52),bound(52)};
            9'd158:decode_read={bound(52),bound(52),bound(53)};
            9'd159:decode_read={bound(52),bound(53),bound(53)};
            9'd160:decode_read={bound(53),bound(53),bound(53)};
            9'd161:decode_read={bound(53),bound(53),bound(54)};
            9'd162:decode_read={bound(53),bound(54),bound(54)};
            9'd163:decode_read={bound(54),bound(54),bound(54)};
            9'd164:decode_read={bound(54),bound(54),bound(55)};
            9'd165:decode_read={bound(54),bound(55),bound(55)};
            9'd166:decode_read={bound(55),bound(55),bound(55)};
            9'd167:decode_read={bound(55),bound(55),bound(56)};
            9'd168:decode_read={bound(55),bound(56),bound(56)};
            9'd169:decode_read={bound(56),bound(56),bound(56)};
            9'd170:decode_read={bound(56),bound(56),bound(57)};
            9'd171:decode_read={bound(56),bound(57),bound(57)};
            9'd172:decode_read={bound(57),bound(57),bound(57)};
            9'd173:decode_read={bound(57),bound(57),bound(58)};
            9'd174:decode_read={bound(57),bound(58),bound(58)};
            9'd175:decode_read={bound(58),bound(58),bound(58)};
            9'd176:decode_read={bound(58),bound(58),bound(59)};
            9'd177:decode_read={bound(58),bound(59),bound(59)};
            9'd178:decode_read={bound(59),bound(59),bound(59)};
            9'd179:decode_read={bound(59),bound(59),bound(60)};
            9'd180:decode_read={bound(59),bound(60),bound(60)};
            9'd181:decode_read={bound(60),bound(60),bound(60)};
            9'd182:decode_read={bound(60),bound(60),bound(61)};
            9'd183:decode_read={bound(60),bound(61),bound(61)};
            9'd184:decode_read={bound(61),bound(61),bound(61)};
            9'd185:decode_read={bound(61),bound(61),bound(62)};
            9'd186:decode_read={bound(61),bound(62),bound(62)};
            9'd187:decode_read={bound(62),bound(62),bound(62)};
            9'd188:decode_read={bound(62),bound(62),bound(63)};
            9'd189:decode_read={bound(62),bound(63),bound(63)};
            9'd190:decode_read={bound(63),bound(63),bound(63)};
            9'd191:decode_read={bound(63),bound(63),bound(64)};
            9'd192:decode_read={bound(63),bound(64),bound(64)};
            9'd193:decode_read={bound(64),bound(64),bound(64)};
            9'd194:decode_read={bound(64),bound(64),bound(65)};
            9'd195:decode_read={bound(64),bound(65),bound(65)};
            9'd196:decode_read={bound(65),bound(65),bound(65)};
            9'd197:decode_read={bound(65),bound(65),bound(66)};
            9'd198:decode_read={bound(65),bound(66),bound(66)};
            9'd199:decode_read={bound(66),bound(66),bound(66)};
            9'd200:decode_read={bound(66),bound(66),bound(67)};
            9'd201:decode_read={bound(66),bound(67),bound(67)};
            9'd202:decode_read={bound(67),bound(67),bound(67)};
            9'd203:decode_read={bound(67),bound(67),bound(68)};
            9'd204:decode_read={bound(67),bound(68),bound(68)};
            9'd205:decode_read={bound(68),bound(68),bound(68)};
            9'd206:decode_read={bound(68),bound(68),bound(69)};
            9'd207:decode_read={bound(68),bound(69),bound(69)};
            9'd208:decode_read={bound(69),bound(69),bound(69)};
            9'd209:decode_read={bound(69),bound(69),bound(70)};
            9'd210:decode_read={bound(69),bound(70),bound(70)};
            9'd211:decode_read={bound(70),bound(70),bound(70)};
            9'd212:decode_read={bound(70),bound(70),bound(71)};
            9'd213:decode_read={bound(70),bound(71),bound(71)};
            9'd214:decode_read={bound(71),bound(71),bound(71)};
            9'd215:decode_read={bound(71),bound(71),bound(72)};
            9'd216:decode_read={bound(71),bound(72),bound(72)};
            9'd217:decode_read={bound(72),bound(72),bound(72)};
            9'd218:decode_read={bound(72),bound(72),bound(73)};
            9'd219:decode_read={bound(72),bound(73),bound(73)};
            9'd220:decode_read={bound(73),bound(73),bound(73)};
            9'd221:decode_read={bound(73),bound(73),bound(74)};
            9'd222:decode_read={bound(73),bound(74),bound(74)};
            9'd223:decode_read={bound(74),bound(74),bound(74)};
            9'd224:decode_read={bound(74),bound(74),bound(75)};
            9'd225:decode_read={bound(74),bound(75),bound(75)};
            9'd226:decode_read={bound(75),bound(75),bound(75)};
            9'd227:decode_read={bound(75),bound(75),bound(76)};
            9'd228:decode_read={bound(75),bound(76),bound(76)};
            9'd229:decode_read={bound(76),bound(76),bound(76)};
            9'd230:decode_read={bound(76),bound(76),bound(77)};
            9'd231:decode_read={bound(76),bound(77),bound(77)};
            9'd232:decode_read={bound(77),bound(77),bound(77)};
            9'd233:decode_read={bound(77),bound(77),bound(78)};
            9'd234:decode_read={bound(77),bound(78),bound(78)};
            9'd235:decode_read={bound(78),bound(78),bound(78)};
            9'd236:decode_read={bound(78),bound(78),bound(79)};
            9'd237:decode_read={bound(78),bound(79),bound(79)};
            9'd238:decode_read={bound(79),bound(79),bound(79)};
            9'd239:decode_read={bound(79),bound(79),bound(80)};
            9'd240:decode_read={bound(79),bound(80),bound(80)};
            9'd241:decode_read={bound(80),bound(80),bound(80)};
            9'd242:decode_read={bound(80),bound(80),bound(81)};
            9'd243:decode_read={bound(80),bound(81),bound(81)};
            9'd244:decode_read={bound(81),bound(81),bound(81)};
            9'd245:decode_read={bound(81),bound(81),bound(82)};
            9'd246:decode_read={bound(81),bound(82),bound(82)};
            9'd247:decode_read={bound(82),bound(82),bound(82)};
            9'd248:decode_read={bound(82),bound(82),bound(83)};
            9'd249:decode_read={bound(82),bound(83),bound(83)};
            9'd250:decode_read={bound(83),bound(83),bound(83)};
            9'd251:decode_read={bound(83),bound(83),bound(84)};
            9'd252:decode_read={bound(83),bound(84),bound(84)};
            9'd253:decode_read={bound(84),bound(84),bound(84)};
            9'd254:decode_read={bound(84),bound(84),bound(85)};
            9'd255:decode_read={bound(84),bound(85),bound(85)};
            9'd256:decode_read={bound(((8'd0 & ((1<<WAW)-1))/3)),bound(((8'd0 & ((1<<WAW)-1))/3)),bound(((8'd0 & ((1<<WAW)-1))/3))};
            9'd257:decode_read={bound(((8'd1 & ((1<<WAW)-1))/3)),bound(((8'd1 & ((1<<WAW)-1))/3)),bound(((8'd1 & ((1<<WAW)-1))/3))};
            9'd258:decode_read={bound(((8'd2 & ((1<<WAW)-1))/3)),bound(((8'd2 & ((1<<WAW)-1))/3)),bound(((8'd2 & ((1<<WAW)-1))/3))};
            9'd259:decode_read={bound(((8'd3 & ((1<<WAW)-1))/3)),bound(((8'd3 & ((1<<WAW)-1))/3)),bound(((8'd3 & ((1<<WAW)-1))/3))};
            9'd260:decode_read={bound(((8'd4 & ((1<<WAW)-1))/3)),bound(((8'd4 & ((1<<WAW)-1))/3)),bound(((8'd4 & ((1<<WAW)-1))/3))};
            9'd261:decode_read={bound(((8'd5 & ((1<<WAW)-1))/3)),bound(((8'd5 & ((1<<WAW)-1))/3)),bound(((8'd5 & ((1<<WAW)-1))/3))};
            9'd262:decode_read={bound(((8'd6 & ((1<<WAW)-1))/3)),bound(((8'd6 & ((1<<WAW)-1))/3)),bound(((8'd6 & ((1<<WAW)-1))/3))};
            9'd263:decode_read={bound(((8'd7 & ((1<<WAW)-1))/3)),bound(((8'd7 & ((1<<WAW)-1))/3)),bound(((8'd7 & ((1<<WAW)-1))/3))};
            9'd264:decode_read={bound(((8'd8 & ((1<<WAW)-1))/3)),bound(((8'd8 & ((1<<WAW)-1))/3)),bound(((8'd8 & ((1<<WAW)-1))/3))};
            9'd265:decode_read={bound(((8'd9 & ((1<<WAW)-1))/3)),bound(((8'd9 & ((1<<WAW)-1))/3)),bound(((8'd9 & ((1<<WAW)-1))/3))};
            9'd266:decode_read={bound(((8'd10 & ((1<<WAW)-1))/3)),bound(((8'd10 & ((1<<WAW)-1))/3)),bound(((8'd10 & ((1<<WAW)-1))/3))};
            9'd267:decode_read={bound(((8'd11 & ((1<<WAW)-1))/3)),bound(((8'd11 & ((1<<WAW)-1))/3)),bound(((8'd11 & ((1<<WAW)-1))/3))};
            9'd268:decode_read={bound(((8'd12 & ((1<<WAW)-1))/3)),bound(((8'd12 & ((1<<WAW)-1))/3)),bound(((8'd12 & ((1<<WAW)-1))/3))};
            9'd269:decode_read={bound(((8'd13 & ((1<<WAW)-1))/3)),bound(((8'd13 & ((1<<WAW)-1))/3)),bound(((8'd13 & ((1<<WAW)-1))/3))};
            9'd270:decode_read={bound(((8'd14 & ((1<<WAW)-1))/3)),bound(((8'd14 & ((1<<WAW)-1))/3)),bound(((8'd14 & ((1<<WAW)-1))/3))};
            9'd271:decode_read={bound(((8'd15 & ((1<<WAW)-1))/3)),bound(((8'd15 & ((1<<WAW)-1))/3)),bound(((8'd15 & ((1<<WAW)-1))/3))};
            9'd272:decode_read={bound(((8'd16 & ((1<<WAW)-1))/3)),bound(((8'd16 & ((1<<WAW)-1))/3)),bound(((8'd16 & ((1<<WAW)-1))/3))};
            9'd273:decode_read={bound(((8'd17 & ((1<<WAW)-1))/3)),bound(((8'd17 & ((1<<WAW)-1))/3)),bound(((8'd17 & ((1<<WAW)-1))/3))};
            9'd274:decode_read={bound(((8'd18 & ((1<<WAW)-1))/3)),bound(((8'd18 & ((1<<WAW)-1))/3)),bound(((8'd18 & ((1<<WAW)-1))/3))};
            9'd275:decode_read={bound(((8'd19 & ((1<<WAW)-1))/3)),bound(((8'd19 & ((1<<WAW)-1))/3)),bound(((8'd19 & ((1<<WAW)-1))/3))};
            9'd276:decode_read={bound(((8'd20 & ((1<<WAW)-1))/3)),bound(((8'd20 & ((1<<WAW)-1))/3)),bound(((8'd20 & ((1<<WAW)-1))/3))};
            9'd277:decode_read={bound(((8'd21 & ((1<<WAW)-1))/3)),bound(((8'd21 & ((1<<WAW)-1))/3)),bound(((8'd21 & ((1<<WAW)-1))/3))};
            9'd278:decode_read={bound(((8'd22 & ((1<<WAW)-1))/3)),bound(((8'd22 & ((1<<WAW)-1))/3)),bound(((8'd22 & ((1<<WAW)-1))/3))};
            9'd279:decode_read={bound(((8'd23 & ((1<<WAW)-1))/3)),bound(((8'd23 & ((1<<WAW)-1))/3)),bound(((8'd23 & ((1<<WAW)-1))/3))};
            9'd280:decode_read={bound(((8'd24 & ((1<<WAW)-1))/3)),bound(((8'd24 & ((1<<WAW)-1))/3)),bound(((8'd24 & ((1<<WAW)-1))/3))};
            9'd281:decode_read={bound(((8'd25 & ((1<<WAW)-1))/3)),bound(((8'd25 & ((1<<WAW)-1))/3)),bound(((8'd25 & ((1<<WAW)-1))/3))};
            9'd282:decode_read={bound(((8'd26 & ((1<<WAW)-1))/3)),bound(((8'd26 & ((1<<WAW)-1))/3)),bound(((8'd26 & ((1<<WAW)-1))/3))};
            9'd283:decode_read={bound(((8'd27 & ((1<<WAW)-1))/3)),bound(((8'd27 & ((1<<WAW)-1))/3)),bound(((8'd27 & ((1<<WAW)-1))/3))};
            9'd284:decode_read={bound(((8'd28 & ((1<<WAW)-1))/3)),bound(((8'd28 & ((1<<WAW)-1))/3)),bound(((8'd28 & ((1<<WAW)-1))/3))};
            9'd285:decode_read={bound(((8'd29 & ((1<<WAW)-1))/3)),bound(((8'd29 & ((1<<WAW)-1))/3)),bound(((8'd29 & ((1<<WAW)-1))/3))};
            9'd286:decode_read={bound(((8'd30 & ((1<<WAW)-1))/3)),bound(((8'd30 & ((1<<WAW)-1))/3)),bound(((8'd30 & ((1<<WAW)-1))/3))};
            9'd287:decode_read={bound(((8'd31 & ((1<<WAW)-1))/3)),bound(((8'd31 & ((1<<WAW)-1))/3)),bound(((8'd31 & ((1<<WAW)-1))/3))};
            9'd288:decode_read={bound(((8'd32 & ((1<<WAW)-1))/3)),bound(((8'd32 & ((1<<WAW)-1))/3)),bound(((8'd32 & ((1<<WAW)-1))/3))};
            9'd289:decode_read={bound(((8'd33 & ((1<<WAW)-1))/3)),bound(((8'd33 & ((1<<WAW)-1))/3)),bound(((8'd33 & ((1<<WAW)-1))/3))};
            9'd290:decode_read={bound(((8'd34 & ((1<<WAW)-1))/3)),bound(((8'd34 & ((1<<WAW)-1))/3)),bound(((8'd34 & ((1<<WAW)-1))/3))};
            9'd291:decode_read={bound(((8'd35 & ((1<<WAW)-1))/3)),bound(((8'd35 & ((1<<WAW)-1))/3)),bound(((8'd35 & ((1<<WAW)-1))/3))};
            9'd292:decode_read={bound(((8'd36 & ((1<<WAW)-1))/3)),bound(((8'd36 & ((1<<WAW)-1))/3)),bound(((8'd36 & ((1<<WAW)-1))/3))};
            9'd293:decode_read={bound(((8'd37 & ((1<<WAW)-1))/3)),bound(((8'd37 & ((1<<WAW)-1))/3)),bound(((8'd37 & ((1<<WAW)-1))/3))};
            9'd294:decode_read={bound(((8'd38 & ((1<<WAW)-1))/3)),bound(((8'd38 & ((1<<WAW)-1))/3)),bound(((8'd38 & ((1<<WAW)-1))/3))};
            9'd295:decode_read={bound(((8'd39 & ((1<<WAW)-1))/3)),bound(((8'd39 & ((1<<WAW)-1))/3)),bound(((8'd39 & ((1<<WAW)-1))/3))};
            9'd296:decode_read={bound(((8'd40 & ((1<<WAW)-1))/3)),bound(((8'd40 & ((1<<WAW)-1))/3)),bound(((8'd40 & ((1<<WAW)-1))/3))};
            9'd297:decode_read={bound(((8'd41 & ((1<<WAW)-1))/3)),bound(((8'd41 & ((1<<WAW)-1))/3)),bound(((8'd41 & ((1<<WAW)-1))/3))};
            9'd298:decode_read={bound(((8'd42 & ((1<<WAW)-1))/3)),bound(((8'd42 & ((1<<WAW)-1))/3)),bound(((8'd42 & ((1<<WAW)-1))/3))};
            9'd299:decode_read={bound(((8'd43 & ((1<<WAW)-1))/3)),bound(((8'd43 & ((1<<WAW)-1))/3)),bound(((8'd43 & ((1<<WAW)-1))/3))};
            9'd300:decode_read={bound(((8'd44 & ((1<<WAW)-1))/3)),bound(((8'd44 & ((1<<WAW)-1))/3)),bound(((8'd44 & ((1<<WAW)-1))/3))};
            9'd301:decode_read={bound(((8'd45 & ((1<<WAW)-1))/3)),bound(((8'd45 & ((1<<WAW)-1))/3)),bound(((8'd45 & ((1<<WAW)-1))/3))};
            9'd302:decode_read={bound(((8'd46 & ((1<<WAW)-1))/3)),bound(((8'd46 & ((1<<WAW)-1))/3)),bound(((8'd46 & ((1<<WAW)-1))/3))};
            9'd303:decode_read={bound(((8'd47 & ((1<<WAW)-1))/3)),bound(((8'd47 & ((1<<WAW)-1))/3)),bound(((8'd47 & ((1<<WAW)-1))/3))};
            9'd304:decode_read={bound(((8'd48 & ((1<<WAW)-1))/3)),bound(((8'd48 & ((1<<WAW)-1))/3)),bound(((8'd48 & ((1<<WAW)-1))/3))};
            9'd305:decode_read={bound(((8'd49 & ((1<<WAW)-1))/3)),bound(((8'd49 & ((1<<WAW)-1))/3)),bound(((8'd49 & ((1<<WAW)-1))/3))};
            9'd306:decode_read={bound(((8'd50 & ((1<<WAW)-1))/3)),bound(((8'd50 & ((1<<WAW)-1))/3)),bound(((8'd50 & ((1<<WAW)-1))/3))};
            9'd307:decode_read={bound(((8'd51 & ((1<<WAW)-1))/3)),bound(((8'd51 & ((1<<WAW)-1))/3)),bound(((8'd51 & ((1<<WAW)-1))/3))};
            9'd308:decode_read={bound(((8'd52 & ((1<<WAW)-1))/3)),bound(((8'd52 & ((1<<WAW)-1))/3)),bound(((8'd52 & ((1<<WAW)-1))/3))};
            9'd309:decode_read={bound(((8'd53 & ((1<<WAW)-1))/3)),bound(((8'd53 & ((1<<WAW)-1))/3)),bound(((8'd53 & ((1<<WAW)-1))/3))};
            9'd310:decode_read={bound(((8'd54 & ((1<<WAW)-1))/3)),bound(((8'd54 & ((1<<WAW)-1))/3)),bound(((8'd54 & ((1<<WAW)-1))/3))};
            9'd311:decode_read={bound(((8'd55 & ((1<<WAW)-1))/3)),bound(((8'd55 & ((1<<WAW)-1))/3)),bound(((8'd55 & ((1<<WAW)-1))/3))};
            9'd312:decode_read={bound(((8'd56 & ((1<<WAW)-1))/3)),bound(((8'd56 & ((1<<WAW)-1))/3)),bound(((8'd56 & ((1<<WAW)-1))/3))};
            9'd313:decode_read={bound(((8'd57 & ((1<<WAW)-1))/3)),bound(((8'd57 & ((1<<WAW)-1))/3)),bound(((8'd57 & ((1<<WAW)-1))/3))};
            9'd314:decode_read={bound(((8'd58 & ((1<<WAW)-1))/3)),bound(((8'd58 & ((1<<WAW)-1))/3)),bound(((8'd58 & ((1<<WAW)-1))/3))};
            9'd315:decode_read={bound(((8'd59 & ((1<<WAW)-1))/3)),bound(((8'd59 & ((1<<WAW)-1))/3)),bound(((8'd59 & ((1<<WAW)-1))/3))};
            9'd316:decode_read={bound(((8'd60 & ((1<<WAW)-1))/3)),bound(((8'd60 & ((1<<WAW)-1))/3)),bound(((8'd60 & ((1<<WAW)-1))/3))};
            9'd317:decode_read={bound(((8'd61 & ((1<<WAW)-1))/3)),bound(((8'd61 & ((1<<WAW)-1))/3)),bound(((8'd61 & ((1<<WAW)-1))/3))};
            9'd318:decode_read={bound(((8'd62 & ((1<<WAW)-1))/3)),bound(((8'd62 & ((1<<WAW)-1))/3)),bound(((8'd62 & ((1<<WAW)-1))/3))};
            9'd319:decode_read={bound(((8'd63 & ((1<<WAW)-1))/3)),bound(((8'd63 & ((1<<WAW)-1))/3)),bound(((8'd63 & ((1<<WAW)-1))/3))};
            9'd320:decode_read={bound(((8'd64 & ((1<<WAW)-1))/3)),bound(((8'd64 & ((1<<WAW)-1))/3)),bound(((8'd64 & ((1<<WAW)-1))/3))};
            9'd321:decode_read={bound(((8'd65 & ((1<<WAW)-1))/3)),bound(((8'd65 & ((1<<WAW)-1))/3)),bound(((8'd65 & ((1<<WAW)-1))/3))};
            9'd322:decode_read={bound(((8'd66 & ((1<<WAW)-1))/3)),bound(((8'd66 & ((1<<WAW)-1))/3)),bound(((8'd66 & ((1<<WAW)-1))/3))};
            9'd323:decode_read={bound(((8'd67 & ((1<<WAW)-1))/3)),bound(((8'd67 & ((1<<WAW)-1))/3)),bound(((8'd67 & ((1<<WAW)-1))/3))};
            9'd324:decode_read={bound(((8'd68 & ((1<<WAW)-1))/3)),bound(((8'd68 & ((1<<WAW)-1))/3)),bound(((8'd68 & ((1<<WAW)-1))/3))};
            9'd325:decode_read={bound(((8'd69 & ((1<<WAW)-1))/3)),bound(((8'd69 & ((1<<WAW)-1))/3)),bound(((8'd69 & ((1<<WAW)-1))/3))};
            9'd326:decode_read={bound(((8'd70 & ((1<<WAW)-1))/3)),bound(((8'd70 & ((1<<WAW)-1))/3)),bound(((8'd70 & ((1<<WAW)-1))/3))};
            9'd327:decode_read={bound(((8'd71 & ((1<<WAW)-1))/3)),bound(((8'd71 & ((1<<WAW)-1))/3)),bound(((8'd71 & ((1<<WAW)-1))/3))};
            9'd328:decode_read={bound(((8'd72 & ((1<<WAW)-1))/3)),bound(((8'd72 & ((1<<WAW)-1))/3)),bound(((8'd72 & ((1<<WAW)-1))/3))};
            9'd329:decode_read={bound(((8'd73 & ((1<<WAW)-1))/3)),bound(((8'd73 & ((1<<WAW)-1))/3)),bound(((8'd73 & ((1<<WAW)-1))/3))};
            9'd330:decode_read={bound(((8'd74 & ((1<<WAW)-1))/3)),bound(((8'd74 & ((1<<WAW)-1))/3)),bound(((8'd74 & ((1<<WAW)-1))/3))};
            9'd331:decode_read={bound(((8'd75 & ((1<<WAW)-1))/3)),bound(((8'd75 & ((1<<WAW)-1))/3)),bound(((8'd75 & ((1<<WAW)-1))/3))};
            9'd332:decode_read={bound(((8'd76 & ((1<<WAW)-1))/3)),bound(((8'd76 & ((1<<WAW)-1))/3)),bound(((8'd76 & ((1<<WAW)-1))/3))};
            9'd333:decode_read={bound(((8'd77 & ((1<<WAW)-1))/3)),bound(((8'd77 & ((1<<WAW)-1))/3)),bound(((8'd77 & ((1<<WAW)-1))/3))};
            9'd334:decode_read={bound(((8'd78 & ((1<<WAW)-1))/3)),bound(((8'd78 & ((1<<WAW)-1))/3)),bound(((8'd78 & ((1<<WAW)-1))/3))};
            9'd335:decode_read={bound(((8'd79 & ((1<<WAW)-1))/3)),bound(((8'd79 & ((1<<WAW)-1))/3)),bound(((8'd79 & ((1<<WAW)-1))/3))};
            9'd336:decode_read={bound(((8'd80 & ((1<<WAW)-1))/3)),bound(((8'd80 & ((1<<WAW)-1))/3)),bound(((8'd80 & ((1<<WAW)-1))/3))};
            9'd337:decode_read={bound(((8'd81 & ((1<<WAW)-1))/3)),bound(((8'd81 & ((1<<WAW)-1))/3)),bound(((8'd81 & ((1<<WAW)-1))/3))};
            9'd338:decode_read={bound(((8'd82 & ((1<<WAW)-1))/3)),bound(((8'd82 & ((1<<WAW)-1))/3)),bound(((8'd82 & ((1<<WAW)-1))/3))};
            9'd339:decode_read={bound(((8'd83 & ((1<<WAW)-1))/3)),bound(((8'd83 & ((1<<WAW)-1))/3)),bound(((8'd83 & ((1<<WAW)-1))/3))};
            9'd340:decode_read={bound(((8'd84 & ((1<<WAW)-1))/3)),bound(((8'd84 & ((1<<WAW)-1))/3)),bound(((8'd84 & ((1<<WAW)-1))/3))};
            9'd341:decode_read={bound(((8'd85 & ((1<<WAW)-1))/3)),bound(((8'd85 & ((1<<WAW)-1))/3)),bound(((8'd85 & ((1<<WAW)-1))/3))};
            9'd342:decode_read={bound(((8'd86 & ((1<<WAW)-1))/3)),bound(((8'd86 & ((1<<WAW)-1))/3)),bound(((8'd86 & ((1<<WAW)-1))/3))};
            9'd343:decode_read={bound(((8'd87 & ((1<<WAW)-1))/3)),bound(((8'd87 & ((1<<WAW)-1))/3)),bound(((8'd87 & ((1<<WAW)-1))/3))};
            9'd344:decode_read={bound(((8'd88 & ((1<<WAW)-1))/3)),bound(((8'd88 & ((1<<WAW)-1))/3)),bound(((8'd88 & ((1<<WAW)-1))/3))};
            9'd345:decode_read={bound(((8'd89 & ((1<<WAW)-1))/3)),bound(((8'd89 & ((1<<WAW)-1))/3)),bound(((8'd89 & ((1<<WAW)-1))/3))};
            9'd346:decode_read={bound(((8'd90 & ((1<<WAW)-1))/3)),bound(((8'd90 & ((1<<WAW)-1))/3)),bound(((8'd90 & ((1<<WAW)-1))/3))};
            9'd347:decode_read={bound(((8'd91 & ((1<<WAW)-1))/3)),bound(((8'd91 & ((1<<WAW)-1))/3)),bound(((8'd91 & ((1<<WAW)-1))/3))};
            9'd348:decode_read={bound(((8'd92 & ((1<<WAW)-1))/3)),bound(((8'd92 & ((1<<WAW)-1))/3)),bound(((8'd92 & ((1<<WAW)-1))/3))};
            9'd349:decode_read={bound(((8'd93 & ((1<<WAW)-1))/3)),bound(((8'd93 & ((1<<WAW)-1))/3)),bound(((8'd93 & ((1<<WAW)-1))/3))};
            9'd350:decode_read={bound(((8'd94 & ((1<<WAW)-1))/3)),bound(((8'd94 & ((1<<WAW)-1))/3)),bound(((8'd94 & ((1<<WAW)-1))/3))};
            9'd351:decode_read={bound(((8'd95 & ((1<<WAW)-1))/3)),bound(((8'd95 & ((1<<WAW)-1))/3)),bound(((8'd95 & ((1<<WAW)-1))/3))};
            9'd352:decode_read={bound(((8'd96 & ((1<<WAW)-1))/3)),bound(((8'd96 & ((1<<WAW)-1))/3)),bound(((8'd96 & ((1<<WAW)-1))/3))};
            9'd353:decode_read={bound(((8'd97 & ((1<<WAW)-1))/3)),bound(((8'd97 & ((1<<WAW)-1))/3)),bound(((8'd97 & ((1<<WAW)-1))/3))};
            9'd354:decode_read={bound(((8'd98 & ((1<<WAW)-1))/3)),bound(((8'd98 & ((1<<WAW)-1))/3)),bound(((8'd98 & ((1<<WAW)-1))/3))};
            9'd355:decode_read={bound(((8'd99 & ((1<<WAW)-1))/3)),bound(((8'd99 & ((1<<WAW)-1))/3)),bound(((8'd99 & ((1<<WAW)-1))/3))};
            9'd356:decode_read={bound(((8'd100 & ((1<<WAW)-1))/3)),bound(((8'd100 & ((1<<WAW)-1))/3)),bound(((8'd100 & ((1<<WAW)-1))/3))};
            9'd357:decode_read={bound(((8'd101 & ((1<<WAW)-1))/3)),bound(((8'd101 & ((1<<WAW)-1))/3)),bound(((8'd101 & ((1<<WAW)-1))/3))};
            9'd358:decode_read={bound(((8'd102 & ((1<<WAW)-1))/3)),bound(((8'd102 & ((1<<WAW)-1))/3)),bound(((8'd102 & ((1<<WAW)-1))/3))};
            9'd359:decode_read={bound(((8'd103 & ((1<<WAW)-1))/3)),bound(((8'd103 & ((1<<WAW)-1))/3)),bound(((8'd103 & ((1<<WAW)-1))/3))};
            9'd360:decode_read={bound(((8'd104 & ((1<<WAW)-1))/3)),bound(((8'd104 & ((1<<WAW)-1))/3)),bound(((8'd104 & ((1<<WAW)-1))/3))};
            9'd361:decode_read={bound(((8'd105 & ((1<<WAW)-1))/3)),bound(((8'd105 & ((1<<WAW)-1))/3)),bound(((8'd105 & ((1<<WAW)-1))/3))};
            9'd362:decode_read={bound(((8'd106 & ((1<<WAW)-1))/3)),bound(((8'd106 & ((1<<WAW)-1))/3)),bound(((8'd106 & ((1<<WAW)-1))/3))};
            9'd363:decode_read={bound(((8'd107 & ((1<<WAW)-1))/3)),bound(((8'd107 & ((1<<WAW)-1))/3)),bound(((8'd107 & ((1<<WAW)-1))/3))};
            9'd364:decode_read={bound(((8'd108 & ((1<<WAW)-1))/3)),bound(((8'd108 & ((1<<WAW)-1))/3)),bound(((8'd108 & ((1<<WAW)-1))/3))};
            9'd365:decode_read={bound(((8'd109 & ((1<<WAW)-1))/3)),bound(((8'd109 & ((1<<WAW)-1))/3)),bound(((8'd109 & ((1<<WAW)-1))/3))};
            9'd366:decode_read={bound(((8'd110 & ((1<<WAW)-1))/3)),bound(((8'd110 & ((1<<WAW)-1))/3)),bound(((8'd110 & ((1<<WAW)-1))/3))};
            9'd367:decode_read={bound(((8'd111 & ((1<<WAW)-1))/3)),bound(((8'd111 & ((1<<WAW)-1))/3)),bound(((8'd111 & ((1<<WAW)-1))/3))};
            9'd368:decode_read={bound(((8'd112 & ((1<<WAW)-1))/3)),bound(((8'd112 & ((1<<WAW)-1))/3)),bound(((8'd112 & ((1<<WAW)-1))/3))};
            9'd369:decode_read={bound(((8'd113 & ((1<<WAW)-1))/3)),bound(((8'd113 & ((1<<WAW)-1))/3)),bound(((8'd113 & ((1<<WAW)-1))/3))};
            9'd370:decode_read={bound(((8'd114 & ((1<<WAW)-1))/3)),bound(((8'd114 & ((1<<WAW)-1))/3)),bound(((8'd114 & ((1<<WAW)-1))/3))};
            9'd371:decode_read={bound(((8'd115 & ((1<<WAW)-1))/3)),bound(((8'd115 & ((1<<WAW)-1))/3)),bound(((8'd115 & ((1<<WAW)-1))/3))};
            9'd372:decode_read={bound(((8'd116 & ((1<<WAW)-1))/3)),bound(((8'd116 & ((1<<WAW)-1))/3)),bound(((8'd116 & ((1<<WAW)-1))/3))};
            9'd373:decode_read={bound(((8'd117 & ((1<<WAW)-1))/3)),bound(((8'd117 & ((1<<WAW)-1))/3)),bound(((8'd117 & ((1<<WAW)-1))/3))};
            9'd374:decode_read={bound(((8'd118 & ((1<<WAW)-1))/3)),bound(((8'd118 & ((1<<WAW)-1))/3)),bound(((8'd118 & ((1<<WAW)-1))/3))};
            9'd375:decode_read={bound(((8'd119 & ((1<<WAW)-1))/3)),bound(((8'd119 & ((1<<WAW)-1))/3)),bound(((8'd119 & ((1<<WAW)-1))/3))};
            9'd376:decode_read={bound(((8'd120 & ((1<<WAW)-1))/3)),bound(((8'd120 & ((1<<WAW)-1))/3)),bound(((8'd120 & ((1<<WAW)-1))/3))};
            9'd377:decode_read={bound(((8'd121 & ((1<<WAW)-1))/3)),bound(((8'd121 & ((1<<WAW)-1))/3)),bound(((8'd121 & ((1<<WAW)-1))/3))};
            9'd378:decode_read={bound(((8'd122 & ((1<<WAW)-1))/3)),bound(((8'd122 & ((1<<WAW)-1))/3)),bound(((8'd122 & ((1<<WAW)-1))/3))};
            9'd379:decode_read={bound(((8'd123 & ((1<<WAW)-1))/3)),bound(((8'd123 & ((1<<WAW)-1))/3)),bound(((8'd123 & ((1<<WAW)-1))/3))};
            9'd380:decode_read={bound(((8'd124 & ((1<<WAW)-1))/3)),bound(((8'd124 & ((1<<WAW)-1))/3)),bound(((8'd124 & ((1<<WAW)-1))/3))};
            9'd381:decode_read={bound(((8'd125 & ((1<<WAW)-1))/3)),bound(((8'd125 & ((1<<WAW)-1))/3)),bound(((8'd125 & ((1<<WAW)-1))/3))};
            9'd382:decode_read={bound(((8'd126 & ((1<<WAW)-1))/3)),bound(((8'd126 & ((1<<WAW)-1))/3)),bound(((8'd126 & ((1<<WAW)-1))/3))};
            9'd383:decode_read={bound(((8'd127 & ((1<<WAW)-1))/3)),bound(((8'd127 & ((1<<WAW)-1))/3)),bound(((8'd127 & ((1<<WAW)-1))/3))};
            9'd384:decode_read={bound(((8'd128 & ((1<<WAW)-1))/3)),bound(((8'd128 & ((1<<WAW)-1))/3)),bound(((8'd128 & ((1<<WAW)-1))/3))};
            9'd385:decode_read={bound(((8'd129 & ((1<<WAW)-1))/3)),bound(((8'd129 & ((1<<WAW)-1))/3)),bound(((8'd129 & ((1<<WAW)-1))/3))};
            9'd386:decode_read={bound(((8'd130 & ((1<<WAW)-1))/3)),bound(((8'd130 & ((1<<WAW)-1))/3)),bound(((8'd130 & ((1<<WAW)-1))/3))};
            9'd387:decode_read={bound(((8'd131 & ((1<<WAW)-1))/3)),bound(((8'd131 & ((1<<WAW)-1))/3)),bound(((8'd131 & ((1<<WAW)-1))/3))};
            9'd388:decode_read={bound(((8'd132 & ((1<<WAW)-1))/3)),bound(((8'd132 & ((1<<WAW)-1))/3)),bound(((8'd132 & ((1<<WAW)-1))/3))};
            9'd389:decode_read={bound(((8'd133 & ((1<<WAW)-1))/3)),bound(((8'd133 & ((1<<WAW)-1))/3)),bound(((8'd133 & ((1<<WAW)-1))/3))};
            9'd390:decode_read={bound(((8'd134 & ((1<<WAW)-1))/3)),bound(((8'd134 & ((1<<WAW)-1))/3)),bound(((8'd134 & ((1<<WAW)-1))/3))};
            9'd391:decode_read={bound(((8'd135 & ((1<<WAW)-1))/3)),bound(((8'd135 & ((1<<WAW)-1))/3)),bound(((8'd135 & ((1<<WAW)-1))/3))};
            9'd392:decode_read={bound(((8'd136 & ((1<<WAW)-1))/3)),bound(((8'd136 & ((1<<WAW)-1))/3)),bound(((8'd136 & ((1<<WAW)-1))/3))};
            9'd393:decode_read={bound(((8'd137 & ((1<<WAW)-1))/3)),bound(((8'd137 & ((1<<WAW)-1))/3)),bound(((8'd137 & ((1<<WAW)-1))/3))};
            9'd394:decode_read={bound(((8'd138 & ((1<<WAW)-1))/3)),bound(((8'd138 & ((1<<WAW)-1))/3)),bound(((8'd138 & ((1<<WAW)-1))/3))};
            9'd395:decode_read={bound(((8'd139 & ((1<<WAW)-1))/3)),bound(((8'd139 & ((1<<WAW)-1))/3)),bound(((8'd139 & ((1<<WAW)-1))/3))};
            9'd396:decode_read={bound(((8'd140 & ((1<<WAW)-1))/3)),bound(((8'd140 & ((1<<WAW)-1))/3)),bound(((8'd140 & ((1<<WAW)-1))/3))};
            9'd397:decode_read={bound(((8'd141 & ((1<<WAW)-1))/3)),bound(((8'd141 & ((1<<WAW)-1))/3)),bound(((8'd141 & ((1<<WAW)-1))/3))};
            9'd398:decode_read={bound(((8'd142 & ((1<<WAW)-1))/3)),bound(((8'd142 & ((1<<WAW)-1))/3)),bound(((8'd142 & ((1<<WAW)-1))/3))};
            9'd399:decode_read={bound(((8'd143 & ((1<<WAW)-1))/3)),bound(((8'd143 & ((1<<WAW)-1))/3)),bound(((8'd143 & ((1<<WAW)-1))/3))};
            9'd400:decode_read={bound(((8'd144 & ((1<<WAW)-1))/3)),bound(((8'd144 & ((1<<WAW)-1))/3)),bound(((8'd144 & ((1<<WAW)-1))/3))};
            9'd401:decode_read={bound(((8'd145 & ((1<<WAW)-1))/3)),bound(((8'd145 & ((1<<WAW)-1))/3)),bound(((8'd145 & ((1<<WAW)-1))/3))};
            9'd402:decode_read={bound(((8'd146 & ((1<<WAW)-1))/3)),bound(((8'd146 & ((1<<WAW)-1))/3)),bound(((8'd146 & ((1<<WAW)-1))/3))};
            9'd403:decode_read={bound(((8'd147 & ((1<<WAW)-1))/3)),bound(((8'd147 & ((1<<WAW)-1))/3)),bound(((8'd147 & ((1<<WAW)-1))/3))};
            9'd404:decode_read={bound(((8'd148 & ((1<<WAW)-1))/3)),bound(((8'd148 & ((1<<WAW)-1))/3)),bound(((8'd148 & ((1<<WAW)-1))/3))};
            9'd405:decode_read={bound(((8'd149 & ((1<<WAW)-1))/3)),bound(((8'd149 & ((1<<WAW)-1))/3)),bound(((8'd149 & ((1<<WAW)-1))/3))};
            9'd406:decode_read={bound(((8'd150 & ((1<<WAW)-1))/3)),bound(((8'd150 & ((1<<WAW)-1))/3)),bound(((8'd150 & ((1<<WAW)-1))/3))};
            9'd407:decode_read={bound(((8'd151 & ((1<<WAW)-1))/3)),bound(((8'd151 & ((1<<WAW)-1))/3)),bound(((8'd151 & ((1<<WAW)-1))/3))};
            9'd408:decode_read={bound(((8'd152 & ((1<<WAW)-1))/3)),bound(((8'd152 & ((1<<WAW)-1))/3)),bound(((8'd152 & ((1<<WAW)-1))/3))};
            9'd409:decode_read={bound(((8'd153 & ((1<<WAW)-1))/3)),bound(((8'd153 & ((1<<WAW)-1))/3)),bound(((8'd153 & ((1<<WAW)-1))/3))};
            9'd410:decode_read={bound(((8'd154 & ((1<<WAW)-1))/3)),bound(((8'd154 & ((1<<WAW)-1))/3)),bound(((8'd154 & ((1<<WAW)-1))/3))};
            9'd411:decode_read={bound(((8'd155 & ((1<<WAW)-1))/3)),bound(((8'd155 & ((1<<WAW)-1))/3)),bound(((8'd155 & ((1<<WAW)-1))/3))};
            9'd412:decode_read={bound(((8'd156 & ((1<<WAW)-1))/3)),bound(((8'd156 & ((1<<WAW)-1))/3)),bound(((8'd156 & ((1<<WAW)-1))/3))};
            9'd413:decode_read={bound(((8'd157 & ((1<<WAW)-1))/3)),bound(((8'd157 & ((1<<WAW)-1))/3)),bound(((8'd157 & ((1<<WAW)-1))/3))};
            9'd414:decode_read={bound(((8'd158 & ((1<<WAW)-1))/3)),bound(((8'd158 & ((1<<WAW)-1))/3)),bound(((8'd158 & ((1<<WAW)-1))/3))};
            9'd415:decode_read={bound(((8'd159 & ((1<<WAW)-1))/3)),bound(((8'd159 & ((1<<WAW)-1))/3)),bound(((8'd159 & ((1<<WAW)-1))/3))};
            9'd416:decode_read={bound(((8'd160 & ((1<<WAW)-1))/3)),bound(((8'd160 & ((1<<WAW)-1))/3)),bound(((8'd160 & ((1<<WAW)-1))/3))};
            9'd417:decode_read={bound(((8'd161 & ((1<<WAW)-1))/3)),bound(((8'd161 & ((1<<WAW)-1))/3)),bound(((8'd161 & ((1<<WAW)-1))/3))};
            9'd418:decode_read={bound(((8'd162 & ((1<<WAW)-1))/3)),bound(((8'd162 & ((1<<WAW)-1))/3)),bound(((8'd162 & ((1<<WAW)-1))/3))};
            9'd419:decode_read={bound(((8'd163 & ((1<<WAW)-1))/3)),bound(((8'd163 & ((1<<WAW)-1))/3)),bound(((8'd163 & ((1<<WAW)-1))/3))};
            9'd420:decode_read={bound(((8'd164 & ((1<<WAW)-1))/3)),bound(((8'd164 & ((1<<WAW)-1))/3)),bound(((8'd164 & ((1<<WAW)-1))/3))};
            9'd421:decode_read={bound(((8'd165 & ((1<<WAW)-1))/3)),bound(((8'd165 & ((1<<WAW)-1))/3)),bound(((8'd165 & ((1<<WAW)-1))/3))};
            9'd422:decode_read={bound(((8'd166 & ((1<<WAW)-1))/3)),bound(((8'd166 & ((1<<WAW)-1))/3)),bound(((8'd166 & ((1<<WAW)-1))/3))};
            9'd423:decode_read={bound(((8'd167 & ((1<<WAW)-1))/3)),bound(((8'd167 & ((1<<WAW)-1))/3)),bound(((8'd167 & ((1<<WAW)-1))/3))};
            9'd424:decode_read={bound(((8'd168 & ((1<<WAW)-1))/3)),bound(((8'd168 & ((1<<WAW)-1))/3)),bound(((8'd168 & ((1<<WAW)-1))/3))};
            9'd425:decode_read={bound(((8'd169 & ((1<<WAW)-1))/3)),bound(((8'd169 & ((1<<WAW)-1))/3)),bound(((8'd169 & ((1<<WAW)-1))/3))};
            9'd426:decode_read={bound(((8'd170 & ((1<<WAW)-1))/3)),bound(((8'd170 & ((1<<WAW)-1))/3)),bound(((8'd170 & ((1<<WAW)-1))/3))};
            9'd427:decode_read={bound(((8'd171 & ((1<<WAW)-1))/3)),bound(((8'd171 & ((1<<WAW)-1))/3)),bound(((8'd171 & ((1<<WAW)-1))/3))};
            9'd428:decode_read={bound(((8'd172 & ((1<<WAW)-1))/3)),bound(((8'd172 & ((1<<WAW)-1))/3)),bound(((8'd172 & ((1<<WAW)-1))/3))};
            9'd429:decode_read={bound(((8'd173 & ((1<<WAW)-1))/3)),bound(((8'd173 & ((1<<WAW)-1))/3)),bound(((8'd173 & ((1<<WAW)-1))/3))};
            9'd430:decode_read={bound(((8'd174 & ((1<<WAW)-1))/3)),bound(((8'd174 & ((1<<WAW)-1))/3)),bound(((8'd174 & ((1<<WAW)-1))/3))};
            9'd431:decode_read={bound(((8'd175 & ((1<<WAW)-1))/3)),bound(((8'd175 & ((1<<WAW)-1))/3)),bound(((8'd175 & ((1<<WAW)-1))/3))};
            9'd432:decode_read={bound(((8'd176 & ((1<<WAW)-1))/3)),bound(((8'd176 & ((1<<WAW)-1))/3)),bound(((8'd176 & ((1<<WAW)-1))/3))};
            9'd433:decode_read={bound(((8'd177 & ((1<<WAW)-1))/3)),bound(((8'd177 & ((1<<WAW)-1))/3)),bound(((8'd177 & ((1<<WAW)-1))/3))};
            9'd434:decode_read={bound(((8'd178 & ((1<<WAW)-1))/3)),bound(((8'd178 & ((1<<WAW)-1))/3)),bound(((8'd178 & ((1<<WAW)-1))/3))};
            9'd435:decode_read={bound(((8'd179 & ((1<<WAW)-1))/3)),bound(((8'd179 & ((1<<WAW)-1))/3)),bound(((8'd179 & ((1<<WAW)-1))/3))};
            9'd436:decode_read={bound(((8'd180 & ((1<<WAW)-1))/3)),bound(((8'd180 & ((1<<WAW)-1))/3)),bound(((8'd180 & ((1<<WAW)-1))/3))};
            9'd437:decode_read={bound(((8'd181 & ((1<<WAW)-1))/3)),bound(((8'd181 & ((1<<WAW)-1))/3)),bound(((8'd181 & ((1<<WAW)-1))/3))};
            9'd438:decode_read={bound(((8'd182 & ((1<<WAW)-1))/3)),bound(((8'd182 & ((1<<WAW)-1))/3)),bound(((8'd182 & ((1<<WAW)-1))/3))};
            9'd439:decode_read={bound(((8'd183 & ((1<<WAW)-1))/3)),bound(((8'd183 & ((1<<WAW)-1))/3)),bound(((8'd183 & ((1<<WAW)-1))/3))};
            9'd440:decode_read={bound(((8'd184 & ((1<<WAW)-1))/3)),bound(((8'd184 & ((1<<WAW)-1))/3)),bound(((8'd184 & ((1<<WAW)-1))/3))};
            9'd441:decode_read={bound(((8'd185 & ((1<<WAW)-1))/3)),bound(((8'd185 & ((1<<WAW)-1))/3)),bound(((8'd185 & ((1<<WAW)-1))/3))};
            9'd442:decode_read={bound(((8'd186 & ((1<<WAW)-1))/3)),bound(((8'd186 & ((1<<WAW)-1))/3)),bound(((8'd186 & ((1<<WAW)-1))/3))};
            9'd443:decode_read={bound(((8'd187 & ((1<<WAW)-1))/3)),bound(((8'd187 & ((1<<WAW)-1))/3)),bound(((8'd187 & ((1<<WAW)-1))/3))};
            9'd444:decode_read={bound(((8'd188 & ((1<<WAW)-1))/3)),bound(((8'd188 & ((1<<WAW)-1))/3)),bound(((8'd188 & ((1<<WAW)-1))/3))};
            9'd445:decode_read={bound(((8'd189 & ((1<<WAW)-1))/3)),bound(((8'd189 & ((1<<WAW)-1))/3)),bound(((8'd189 & ((1<<WAW)-1))/3))};
            9'd446:decode_read={bound(((8'd190 & ((1<<WAW)-1))/3)),bound(((8'd190 & ((1<<WAW)-1))/3)),bound(((8'd190 & ((1<<WAW)-1))/3))};
            9'd447:decode_read={bound(((8'd191 & ((1<<WAW)-1))/3)),bound(((8'd191 & ((1<<WAW)-1))/3)),bound(((8'd191 & ((1<<WAW)-1))/3))};
            9'd448:decode_read={bound(((8'd192 & ((1<<WAW)-1))/3)),bound(((8'd192 & ((1<<WAW)-1))/3)),bound(((8'd192 & ((1<<WAW)-1))/3))};
            9'd449:decode_read={bound(((8'd193 & ((1<<WAW)-1))/3)),bound(((8'd193 & ((1<<WAW)-1))/3)),bound(((8'd193 & ((1<<WAW)-1))/3))};
            9'd450:decode_read={bound(((8'd194 & ((1<<WAW)-1))/3)),bound(((8'd194 & ((1<<WAW)-1))/3)),bound(((8'd194 & ((1<<WAW)-1))/3))};
            9'd451:decode_read={bound(((8'd195 & ((1<<WAW)-1))/3)),bound(((8'd195 & ((1<<WAW)-1))/3)),bound(((8'd195 & ((1<<WAW)-1))/3))};
            9'd452:decode_read={bound(((8'd196 & ((1<<WAW)-1))/3)),bound(((8'd196 & ((1<<WAW)-1))/3)),bound(((8'd196 & ((1<<WAW)-1))/3))};
            9'd453:decode_read={bound(((8'd197 & ((1<<WAW)-1))/3)),bound(((8'd197 & ((1<<WAW)-1))/3)),bound(((8'd197 & ((1<<WAW)-1))/3))};
            9'd454:decode_read={bound(((8'd198 & ((1<<WAW)-1))/3)),bound(((8'd198 & ((1<<WAW)-1))/3)),bound(((8'd198 & ((1<<WAW)-1))/3))};
            9'd455:decode_read={bound(((8'd199 & ((1<<WAW)-1))/3)),bound(((8'd199 & ((1<<WAW)-1))/3)),bound(((8'd199 & ((1<<WAW)-1))/3))};
            9'd456:decode_read={bound(((8'd200 & ((1<<WAW)-1))/3)),bound(((8'd200 & ((1<<WAW)-1))/3)),bound(((8'd200 & ((1<<WAW)-1))/3))};
            9'd457:decode_read={bound(((8'd201 & ((1<<WAW)-1))/3)),bound(((8'd201 & ((1<<WAW)-1))/3)),bound(((8'd201 & ((1<<WAW)-1))/3))};
            9'd458:decode_read={bound(((8'd202 & ((1<<WAW)-1))/3)),bound(((8'd202 & ((1<<WAW)-1))/3)),bound(((8'd202 & ((1<<WAW)-1))/3))};
            9'd459:decode_read={bound(((8'd203 & ((1<<WAW)-1))/3)),bound(((8'd203 & ((1<<WAW)-1))/3)),bound(((8'd203 & ((1<<WAW)-1))/3))};
            9'd460:decode_read={bound(((8'd204 & ((1<<WAW)-1))/3)),bound(((8'd204 & ((1<<WAW)-1))/3)),bound(((8'd204 & ((1<<WAW)-1))/3))};
            9'd461:decode_read={bound(((8'd205 & ((1<<WAW)-1))/3)),bound(((8'd205 & ((1<<WAW)-1))/3)),bound(((8'd205 & ((1<<WAW)-1))/3))};
            9'd462:decode_read={bound(((8'd206 & ((1<<WAW)-1))/3)),bound(((8'd206 & ((1<<WAW)-1))/3)),bound(((8'd206 & ((1<<WAW)-1))/3))};
            9'd463:decode_read={bound(((8'd207 & ((1<<WAW)-1))/3)),bound(((8'd207 & ((1<<WAW)-1))/3)),bound(((8'd207 & ((1<<WAW)-1))/3))};
            9'd464:decode_read={bound(((8'd208 & ((1<<WAW)-1))/3)),bound(((8'd208 & ((1<<WAW)-1))/3)),bound(((8'd208 & ((1<<WAW)-1))/3))};
            9'd465:decode_read={bound(((8'd209 & ((1<<WAW)-1))/3)),bound(((8'd209 & ((1<<WAW)-1))/3)),bound(((8'd209 & ((1<<WAW)-1))/3))};
            9'd466:decode_read={bound(((8'd210 & ((1<<WAW)-1))/3)),bound(((8'd210 & ((1<<WAW)-1))/3)),bound(((8'd210 & ((1<<WAW)-1))/3))};
            9'd467:decode_read={bound(((8'd211 & ((1<<WAW)-1))/3)),bound(((8'd211 & ((1<<WAW)-1))/3)),bound(((8'd211 & ((1<<WAW)-1))/3))};
            9'd468:decode_read={bound(((8'd212 & ((1<<WAW)-1))/3)),bound(((8'd212 & ((1<<WAW)-1))/3)),bound(((8'd212 & ((1<<WAW)-1))/3))};
            9'd469:decode_read={bound(((8'd213 & ((1<<WAW)-1))/3)),bound(((8'd213 & ((1<<WAW)-1))/3)),bound(((8'd213 & ((1<<WAW)-1))/3))};
            9'd470:decode_read={bound(((8'd214 & ((1<<WAW)-1))/3)),bound(((8'd214 & ((1<<WAW)-1))/3)),bound(((8'd214 & ((1<<WAW)-1))/3))};
            9'd471:decode_read={bound(((8'd215 & ((1<<WAW)-1))/3)),bound(((8'd215 & ((1<<WAW)-1))/3)),bound(((8'd215 & ((1<<WAW)-1))/3))};
            9'd472:decode_read={bound(((8'd216 & ((1<<WAW)-1))/3)),bound(((8'd216 & ((1<<WAW)-1))/3)),bound(((8'd216 & ((1<<WAW)-1))/3))};
            9'd473:decode_read={bound(((8'd217 & ((1<<WAW)-1))/3)),bound(((8'd217 & ((1<<WAW)-1))/3)),bound(((8'd217 & ((1<<WAW)-1))/3))};
            9'd474:decode_read={bound(((8'd218 & ((1<<WAW)-1))/3)),bound(((8'd218 & ((1<<WAW)-1))/3)),bound(((8'd218 & ((1<<WAW)-1))/3))};
            9'd475:decode_read={bound(((8'd219 & ((1<<WAW)-1))/3)),bound(((8'd219 & ((1<<WAW)-1))/3)),bound(((8'd219 & ((1<<WAW)-1))/3))};
            9'd476:decode_read={bound(((8'd220 & ((1<<WAW)-1))/3)),bound(((8'd220 & ((1<<WAW)-1))/3)),bound(((8'd220 & ((1<<WAW)-1))/3))};
            9'd477:decode_read={bound(((8'd221 & ((1<<WAW)-1))/3)),bound(((8'd221 & ((1<<WAW)-1))/3)),bound(((8'd221 & ((1<<WAW)-1))/3))};
            9'd478:decode_read={bound(((8'd222 & ((1<<WAW)-1))/3)),bound(((8'd222 & ((1<<WAW)-1))/3)),bound(((8'd222 & ((1<<WAW)-1))/3))};
            9'd479:decode_read={bound(((8'd223 & ((1<<WAW)-1))/3)),bound(((8'd223 & ((1<<WAW)-1))/3)),bound(((8'd223 & ((1<<WAW)-1))/3))};
            9'd480:decode_read={bound(((8'd224 & ((1<<WAW)-1))/3)),bound(((8'd224 & ((1<<WAW)-1))/3)),bound(((8'd224 & ((1<<WAW)-1))/3))};
            9'd481:decode_read={bound(((8'd225 & ((1<<WAW)-1))/3)),bound(((8'd225 & ((1<<WAW)-1))/3)),bound(((8'd225 & ((1<<WAW)-1))/3))};
            9'd482:decode_read={bound(((8'd226 & ((1<<WAW)-1))/3)),bound(((8'd226 & ((1<<WAW)-1))/3)),bound(((8'd226 & ((1<<WAW)-1))/3))};
            9'd483:decode_read={bound(((8'd227 & ((1<<WAW)-1))/3)),bound(((8'd227 & ((1<<WAW)-1))/3)),bound(((8'd227 & ((1<<WAW)-1))/3))};
            9'd484:decode_read={bound(((8'd228 & ((1<<WAW)-1))/3)),bound(((8'd228 & ((1<<WAW)-1))/3)),bound(((8'd228 & ((1<<WAW)-1))/3))};
            9'd485:decode_read={bound(((8'd229 & ((1<<WAW)-1))/3)),bound(((8'd229 & ((1<<WAW)-1))/3)),bound(((8'd229 & ((1<<WAW)-1))/3))};
            9'd486:decode_read={bound(((8'd230 & ((1<<WAW)-1))/3)),bound(((8'd230 & ((1<<WAW)-1))/3)),bound(((8'd230 & ((1<<WAW)-1))/3))};
            9'd487:decode_read={bound(((8'd231 & ((1<<WAW)-1))/3)),bound(((8'd231 & ((1<<WAW)-1))/3)),bound(((8'd231 & ((1<<WAW)-1))/3))};
            9'd488:decode_read={bound(((8'd232 & ((1<<WAW)-1))/3)),bound(((8'd232 & ((1<<WAW)-1))/3)),bound(((8'd232 & ((1<<WAW)-1))/3))};
            9'd489:decode_read={bound(((8'd233 & ((1<<WAW)-1))/3)),bound(((8'd233 & ((1<<WAW)-1))/3)),bound(((8'd233 & ((1<<WAW)-1))/3))};
            9'd490:decode_read={bound(((8'd234 & ((1<<WAW)-1))/3)),bound(((8'd234 & ((1<<WAW)-1))/3)),bound(((8'd234 & ((1<<WAW)-1))/3))};
            9'd491:decode_read={bound(((8'd235 & ((1<<WAW)-1))/3)),bound(((8'd235 & ((1<<WAW)-1))/3)),bound(((8'd235 & ((1<<WAW)-1))/3))};
            9'd492:decode_read={bound(((8'd236 & ((1<<WAW)-1))/3)),bound(((8'd236 & ((1<<WAW)-1))/3)),bound(((8'd236 & ((1<<WAW)-1))/3))};
            9'd493:decode_read={bound(((8'd237 & ((1<<WAW)-1))/3)),bound(((8'd237 & ((1<<WAW)-1))/3)),bound(((8'd237 & ((1<<WAW)-1))/3))};
            9'd494:decode_read={bound(((8'd238 & ((1<<WAW)-1))/3)),bound(((8'd238 & ((1<<WAW)-1))/3)),bound(((8'd238 & ((1<<WAW)-1))/3))};
            9'd495:decode_read={bound(((8'd239 & ((1<<WAW)-1))/3)),bound(((8'd239 & ((1<<WAW)-1))/3)),bound(((8'd239 & ((1<<WAW)-1))/3))};
            9'd496:decode_read={bound(((8'd240 & ((1<<WAW)-1))/3)),bound(((8'd240 & ((1<<WAW)-1))/3)),bound(((8'd240 & ((1<<WAW)-1))/3))};
            9'd497:decode_read={bound(((8'd241 & ((1<<WAW)-1))/3)),bound(((8'd241 & ((1<<WAW)-1))/3)),bound(((8'd241 & ((1<<WAW)-1))/3))};
            9'd498:decode_read={bound(((8'd242 & ((1<<WAW)-1))/3)),bound(((8'd242 & ((1<<WAW)-1))/3)),bound(((8'd242 & ((1<<WAW)-1))/3))};
            9'd499:decode_read={bound(((8'd243 & ((1<<WAW)-1))/3)),bound(((8'd243 & ((1<<WAW)-1))/3)),bound(((8'd243 & ((1<<WAW)-1))/3))};
            9'd500:decode_read={bound(((8'd244 & ((1<<WAW)-1))/3)),bound(((8'd244 & ((1<<WAW)-1))/3)),bound(((8'd244 & ((1<<WAW)-1))/3))};
            9'd501:decode_read={bound(((8'd245 & ((1<<WAW)-1))/3)),bound(((8'd245 & ((1<<WAW)-1))/3)),bound(((8'd245 & ((1<<WAW)-1))/3))};
            9'd502:decode_read={bound(((8'd246 & ((1<<WAW)-1))/3)),bound(((8'd246 & ((1<<WAW)-1))/3)),bound(((8'd246 & ((1<<WAW)-1))/3))};
            9'd503:decode_read={bound(((8'd247 & ((1<<WAW)-1))/3)),bound(((8'd247 & ((1<<WAW)-1))/3)),bound(((8'd247 & ((1<<WAW)-1))/3))};
            9'd504:decode_read={bound(((8'd248 & ((1<<WAW)-1))/3)),bound(((8'd248 & ((1<<WAW)-1))/3)),bound(((8'd248 & ((1<<WAW)-1))/3))};
            9'd505:decode_read={bound(((8'd249 & ((1<<WAW)-1))/3)),bound(((8'd249 & ((1<<WAW)-1))/3)),bound(((8'd249 & ((1<<WAW)-1))/3))};
            9'd506:decode_read={bound(((8'd250 & ((1<<WAW)-1))/3)),bound(((8'd250 & ((1<<WAW)-1))/3)),bound(((8'd250 & ((1<<WAW)-1))/3))};
            9'd507:decode_read={bound(((8'd251 & ((1<<WAW)-1))/3)),bound(((8'd251 & ((1<<WAW)-1))/3)),bound(((8'd251 & ((1<<WAW)-1))/3))};
            9'd508:decode_read={bound(((8'd252 & ((1<<WAW)-1))/3)),bound(((8'd252 & ((1<<WAW)-1))/3)),bound(((8'd252 & ((1<<WAW)-1))/3))};
            9'd509:decode_read={bound(((8'd253 & ((1<<WAW)-1))/3)),bound(((8'd253 & ((1<<WAW)-1))/3)),bound(((8'd253 & ((1<<WAW)-1))/3))};
            9'd510:decode_read={bound(((8'd254 & ((1<<WAW)-1))/3)),bound(((8'd254 & ((1<<WAW)-1))/3)),bound(((8'd254 & ((1<<WAW)-1))/3))};
            9'd511:decode_read={bound(((8'd255 & ((1<<WAW)-1))/3)),bound(((8'd255 & ((1<<WAW)-1))/3)),bound(((8'd255 & ((1<<WAW)-1))/3))};
            default:decode_read={3*BAW{1'bx}};
        endcase
        end
    endfunction
    assign indexes=decode_read(c,mode_pw);
endmodule

// Shared 4:1 row selection precedes the three column selections. Keeping
// this hierarchy stops a separate 12:1 mux being rebuilt for every tap.
module cnn_lb_row_pick /* synthesis syn_hier="hard" */ #(parameter DW=256)(
    input wire [4*DW-1:0] words,input wire [3:0] valids,
    input wire [1:0] sel,output wire [DW-1:0] value);
    wire [DW-1:0] raw=words[sel*DW+:DW];
    assign value=valids[sel]?raw:{DW{1'b0}};
endmodule
module cnn_lb_col_pick /* synthesis syn_hier="hard" */ #(parameter DW=256)(
    input wire [3*DW-1:0] words,input wire [1:0] sel,
    output wire [DW-1:0] value);
    assign value=words[sel*DW+:DW];
endmodule
