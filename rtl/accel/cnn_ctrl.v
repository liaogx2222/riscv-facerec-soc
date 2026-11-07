//=====================================================================
// cnn_ctrl.v : 逐层调度控制器（M8 的"控制器"部分，2026-10-20 新增）
//
// 职责：从描述符 RAM 顺序取 8 字/层 → 解码 → **按层间复位契约**清引擎 → 发 start
//       → 等 layer_done → 下一层 → 全部跑完给 net_done。
//
// ★ 为什么必须有它：`tb_realnet` 里"逐层 for 循环 + 每层 rst_n 低 2 拍"这段
//   之前只存在于测试台。没有它，引擎在板上跑不了整个网络。
//
// ★★ 三条冻结契约（都来自实测，改前必读）
//   ① **层间复位**：每层 start 前 `eng_rst_n` 低 ≥2 拍。ACT 是 RAM、无复位
//      ⇒ 特征图不丢；wrom/shcrom 无状态。代价 2 拍/层（全链 0.06%）。
//      不复位 ⇒ 层 10 起 FAIL（linebuf/dman/outacc/pe_page 的跨层残留）。
//   ② **in_wpg 由 ic 推导**：`in_wpg = ceil(ceil(ic/8)/4)*4`。
//      不能取"上一层 out_wpg" —— det 的 head 层输入来自 `src`（分支），
//      主链前一层与 src 层的 oc 不同（踩过：in_wpg 大 4 倍 ⇒ 8 个 head 层错）。
//   ③ **oh/ow 由 ih/iw + stride2 推导**：`oh = stride2 ? ceil(ih/2) : ih`。
//      描述符里只有 ih/iw（省位宽）。
//
// 用法：`go` 单拍 + `nlayer`，然后等 `net_done` 单拍。
//=====================================================================
`timescale 1ns/1ps

`include "cnn_desc.vh"

module cnn_ctrl #(
    parameter DAW = 8,           // 描述符 RAM 地址位宽（层索引 × DESC_WORDS）
    parameter TMO = 24           // 单层超时（拍数），0 = 不检查
)(
    input  wire              clk,
    input  wire              rst_n,

    input  wire              go,          // 单拍：启动一个网络
    input  wire [7:0]        nlayer,      // 层数（含 FC）
    input  wire [DAW-1:0]    desc_base,   // 本网络描述符在 RAM 里的起始字地址（层号×DESC_WORDS）
    input  wire              hold,        // 高 = 本层 done 后**暂停放行**（供读回/调试）
    // ★ 本网络在 ROM 里的基址偏移：描述符里的 weight_base/brom_base 是**网络内相对值**，
    //   而 ROM 是「det 在前、rec 在后」⇒ rec 必须 +det 段终值（det 为 0）。
    //   这条**不能**由描述符携带（它是"这个网络放进 ROM 哪个位置"的系统级决定）。
    input  wire [13:0]       wbase_off,
    input  wire [7:0]        bbase_off,

    // ---- 描述符 RAM（同步读，1 拍延迟；地址 = 层号*DESC_WORDS + k）----
    output reg  [DAW-1:0]    dram_addr,
    output reg               dram_re,
    input  wire [31:0]       dram_data,

    // ---- 引擎（eng_rst_n 由本模块产生）----
    output reg               eng_rst_n,
    output reg               eng_start,
    output reg  [1:0]        op_mode,
    output reg  [7:0]        n_og,
    output reg  [7:0]        n_ig,
    output reg  [7:0]        ih,
    output reg  [7:0]        iw,
    output reg  [7:0]        oh,
    output reg  [7:0]        ow,
    output reg               stride2,
    output reg               dw_mode,
    output reg               mode_pw,
    output reg               lin_out,
    output reg  [13:0]       weight_base,
    output reg  [15:0]       in_base,
    output reg  [15:0]       out_base,
    output reg  [7:0]        in_wpg,
    output reg  [7:0]        out_wpg,
    output reg  [7:0]        sh_req,
    output reg  [7:0]        brom_base,
    input  wire              eng_busy,
    input  wire              eng_done,

    // ---- 状态 ----
    output reg               busy,
    output reg               net_done,    // 单拍
    output reg  [7:0]        layer_idx,
    output reg               timeout      // 有层超时（粘滞，供调试）
);

    localparam S_IDLE = 3'd0, S_FETCH = 3'd1, S_DEC = 3'd2,
               S_RST   = 3'd3, S_START = 3'd4, S_WAIT = 3'd5, S_DONE = 3'd6;

    reg [2:0]  st;
    reg [3:0]  k;                 // 描述符字游标 0..DESC_WORDS
    reg [255:0] dsh;              // 拼装中的描述符
    reg [7:0]  nlay;
    reg [23:0] tmo_cnt;
    reg        done_seen;         // 本层 eng_done 已到（hold 期间保持）

    // ---- 解码（组合）----
    wire [3:0]  d_type;
    wire        d_lin, d_s2, d_bank, d_fuse, d_srcimg;
    wire [7:0]  d_brom, d_ih, d_iw, d_shreq;
    wire [9:0]  d_ic;
    wire [8:0]  d_oc;
    wire [9:0]  d_nchunk;
    wire [13:0] d_owpg, d_wb;
    wire [15:0] d_inb, d_outb;
    wire        d_3x3, d_dw, d_pw, d_fc;
    wire [7:0]  d_nog, d_nig;
    wire [15:0] d_nwords;

    cnn_desc_dec u_dec (
        .desc(dsh),
        .type_o(d_type), .lin_out_o(d_lin), .stride2_o(d_s2),
        .bank_mask_o(d_bank), .fuse_o(d_fuse), .src_img_o(d_srcimg),
        .brom_base_o(d_brom),
        .ih_o(d_ih), .iw_o(d_iw), .ic_o(d_ic),
        .oc_o(d_oc), .nchunk_o(d_nchunk),
        .out_wpg_o(d_owpg),
        .in_base_o(d_inb), .out_base_o(d_outb),
        .weight_base_o(d_wb),
        .sh_req_o(d_shreq),
        .is_3x3_o(d_3x3), .is_dw_o(d_dw), .is_pw_o(d_pw), .is_fc_o(d_fc),
        .n_og_o(d_nog), .n_ig_o(d_nig), .n_words_o(d_nwords)
    );

    // ★ in_wpg = ceil(ceil(ic/8)/4)*4（由 ic 推导，**不能**取上一层 out_wpg）
    wire [9:0] ic8 = (d_ic + 10'd7) >> 3;          // ceil(ic/8)
    wire [9:0] icw = (ic8 + 10'd3) >> 2;           // ceil(ic8/4)
    wire [7:0] inwpg_c = {icw[5:0], 2'b00};        // ×4

    // ★ oh/ow = stride2 ? ceil(ih/2) : ih
    wire [7:0] oh_c = d_s2 ? ((d_ih + 8'd1) >> 1) : d_ih;
    wire [7:0] ow_c = d_s2 ? ((d_iw + 8'd1) >> 1) : d_iw;

    // 地址：层号 × DESC_WORDS + k
    // ★ 必须显式零扩展：`k[DAW-1:0]` 在 DAW>4 时是**越界部分选择 ⇒ 返回 X**（踩过）
    wire [DAW-1:0] base_addr = desc_base + layer_idx * DESC_WORDS;
    wire [DAW-1:0] k_addr    = k;

    always @(posedge clk) begin
        if (!rst_n) begin
            st <= S_IDLE; busy <= 1'b0; net_done <= 1'b0;
            dram_addr <= {DAW{1'b0}}; dram_re <= 1'b0;
            eng_rst_n <= 1'b0; eng_start <= 1'b0;
            op_mode <= 2'd0; n_og <= 8'd0; n_ig <= 8'd0;
            ih <= 8'd0; iw <= 8'd0; oh <= 8'd0; ow <= 8'd0;
            stride2 <= 1'b0; dw_mode <= 1'b0; mode_pw <= 1'b0; lin_out <= 1'b0;
            weight_base <= 14'd0; in_base <= 16'd0; out_base <= 16'd0;
            in_wpg <= 8'd0; out_wpg <= 8'd0; sh_req <= 8'd0; brom_base <= 8'd0;
            layer_idx <= 8'd0; k <= 4'd0; dsh <= 256'd0; nlay <= 8'd0;
            tmo_cnt <= 24'd0; timeout <= 1'b0; done_seen <= 1'b0;
        end else begin
            net_done <= 1'b0;
            eng_start <= 1'b0;

            case (st)
            //----------------------------------------------------------
            S_IDLE: begin
                dram_re <= 1'b0;
                if (go) begin
                    busy      <= 1'b1;
                    nlay      <= nlayer;
                    layer_idx <= 8'd0;
                    k         <= 4'd0;
                    timeout   <= 1'b0;
                    st        <= S_FETCH;
                end
            end

            //----------------------------------------------------------
            S_FETCH: begin                       // 8 字（同步读 ⇒ 下一拍取）
                dram_re   <= 1'b1;
                dram_addr <= base_addr + k_addr;
                // ★ 描述符 RAM 是**同步读**（地址 → 数据 1 拍）：
                //   在 k 拍发出的地址 base+(k-2)，其数据在 k 拍才有效
                //   ⇒ 落位到 dsh[k-2]（曾按 0 拍采样 ⇒ 整包错位一个字，踩过）
                if (k > 4'd1)
                    dsh[(k - 4'd2) * 32 +: 32] <= dram_data;
                if (k == DESC_WORDS + 4'd1) begin     // 多跑 2 拍把最后 2 字冲出来
                    k       <= 4'd0;
                    dram_re <= 1'b0;
                    st      <= S_DEC;
                end else
                    k <= k + 4'd1;
            end

            //----------------------------------------------------------
            S_DEC: begin                         // 把解码结果锁进引擎端口
                op_mode     <= d_dw ? 2'd1 : d_pw ? 2'd2 : 2'd0;
                dw_mode     <= d_dw;
                mode_pw     <= d_pw;
                lin_out     <= d_lin;
                stride2     <= d_s2;
                n_og        <= d_nog;
                n_ig        <= d_nig;
                ih          <= d_ih;
                iw          <= d_iw;
                oh          <= oh_c;
                ow          <= ow_c;
                weight_base <= d_wb + wbase_off;   // ★ 加本网络的 ROM 偏移
                brom_base   <= d_brom + bbase_off;
                in_base     <= d_inb;
                out_base    <= d_outb;
                in_wpg      <= inwpg_c;
                out_wpg     <= d_owpg[7:0];
                sh_req      <= d_shreq;
                eng_rst_n   <= 1'b0;             // ★ 契约①：先拉低
                tmo_cnt     <= 24'd0;
                st          <= S_RST;
            end

            //----------------------------------------------------------
            S_RST: begin                         // ★ 契约①：低 ≥2 拍
                if (tmo_cnt == 24'd1) begin
                    eng_rst_n <= 1'b1;
                    tmo_cnt   <= 24'd0;
                    st        <= S_START;
                end else
                    tmo_cnt <= tmo_cnt + 24'd1;
            end

            //----------------------------------------------------------
            S_START: begin
                eng_start <= 1'b1;
                st        <= S_WAIT;
            end

            //----------------------------------------------------------
            S_WAIT: begin
                if (eng_done) done_seen <= 1'b1;
                if (done_seen && !hold) begin
                    done_seen <= 1'b0;
                    if (layer_idx + 8'd1 >= nlay) begin
                        st <= S_DONE;
                    end else begin
                        layer_idx <= layer_idx + 8'd1;
                        k         <= 4'd0;
                        st        <= S_FETCH;
                    end
                end else if (TMO != 0 && tmo_cnt >= TMO) begin
                    timeout <= 1'b1;
                    st      <= S_DONE;           // 超时也收口（避免死等）
                end else
                    tmo_cnt <= tmo_cnt + 24'd1;
            end

            //----------------------------------------------------------
            S_DONE: begin
                net_done <= 1'b1;
                busy     <= 1'b0;
                st       <= S_IDLE;
            end

            default: st <= S_IDLE;
            endcase
        end
    end

endmodule
