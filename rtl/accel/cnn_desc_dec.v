//=====================================================================
// cnn_desc_dec.v : Layer Descriptor 解码器（**M8 / 引擎第 1 步**）
//
// 输入：Command Queue 送来的 8×32b 描述符（`cnn_desc.vh` 定义字段）
// 输出：控制器要用的全部字段
//
// ★★ **位选必须全部来自 `cnn_desc.vh` 的 localparam**，不要在别处硬编码。
//   对拍：`gen/build_desc.py` 用**同一份层表**生成描述符包，并同时产出
//   `tb/desc_vec.txt.hex`（62 条 = det 42 + rec 20，每条 8 字 desc + 17 个期望字段）。
//   `tb/cnn_desc_dec_tb.v` 读它逐字段比对 ⇒ **打包侧与解码侧同源同验**。
//
// 派生字段（这里算一次，控制器直接用）：
//   · `is_3x3 / is_dw / is_pw / is_fc` —— 由 TYPE 译码
//   · `n_og` = ceil(oc/8)（conv/pw）或 ceil(oc/32)（dw）★ dw 的 og 是 **32 通道组**
//   · `n_ig` = ceil(ic/4)（conv）｜ 1（dw）｜ ceil(ic/32)（pw）
//   · `n_words` = n_og × n_ig —— 该层在权重 ROM 里占几个 2304b 字
//     ⇒ `cnn_outacc`/`cnn_pe_page` 的迭代边界全靠它，错了会整体错位
//=====================================================================
`timescale 1ns/1ps

`include "cnn_desc.vh"

module cnn_desc_dec (
    input  wire [DESC_WORDS*32-1:0] desc,

    // ---- w0 ----
    output wire [3:0]  type_o,
    output wire        lin_out_o,
    output wire        stride2_o,
    output wire        bank_mask_o,
    output wire        fuse_o,
    output wire        src_img_o,
    output wire [7:0]  brom_base_o,
    // ---- w1 ----
    output wire [7:0]  ih_o,
    output wire [7:0]  iw_o,
    output wire [9:0]  ic_o,
    // ---- w2 ----
    output wire [8:0]  oc_o,
    output wire [9:0]  nchunk_o,
    // ---- w3 ----
    output wire [13:0] out_wpg_o,
    // ---- w4 ----
    output wire [15:0] in_base_o,
    output wire [15:0] out_base_o,
    // ---- w5 ----
    output wire [13:0] weight_base_o,
    // ---- w6 ----
    output wire [7:0]  sh_req_o,

    // ---- 派生 ----
    output wire        is_3x3_o,
    output wire        is_dw_o,
    output wire        is_pw_o,
    output wire        is_fc_o,
    output wire [7:0]  n_og_o,
    output wire [7:0]  n_ig_o,
    output wire [15:0] n_words_o
);

    // 逐字切片
    wire [31:0] d0 = desc[0*32 +: 32];
    wire [31:0] d1 = desc[1*32 +: 32];
    wire [31:0] d2 = desc[2*32 +: 32];
    wire [31:0] d3 = desc[3*32 +: 32];
    wire [31:0] d4 = desc[4*32 +: 32];
    wire [31:0] d5 = desc[5*32 +: 32];
    wire [31:0] d6 = desc[6*32 +: 32];

    // ---- 原样字段 ----
    assign type_o      = d0[D0_TYPE_HI  :D0_TYPE_LO];
    assign lin_out_o   = d0[D0_LIN_OUT];
    assign stride2_o   = d0[D0_STRIDE];
    assign bank_mask_o = d0[D0_BANK_MASK];
    assign fuse_o      = d0[D0_FUSE];
    assign src_img_o   = d0[D0_SRC_IMG];
    assign brom_base_o = d0[D0_BROM_HI:D0_BROM_LO];

    assign ih_o        = d1[D1_IH_HI:D1_IH_LO];
    assign iw_o        = d1[D1_IW_HI:D1_IW_LO];
    assign ic_o        = d1[D1_IC_HI:D1_IC_LO];

    assign oc_o        = d2[D2_OC_HI:D2_OC_LO];
    assign nchunk_o    = d2[D2_NCH_HI:D2_NCH_LO];

    assign out_wpg_o   = d3[D3_OWPG_HI:D3_OWPG_LO];

    assign in_base_o   = d4[D4_IN_HI:D4_IN_LO];
    assign out_base_o  = d4[D4_OUT_HI:D4_OUT_LO];

    assign weight_base_o = d5[D5_WB_HI:D5_WB_LO];

    assign sh_req_o    = d6[D6_SHREQ_HI:D6_SHREQ_LO];

    // ---- 派生 ----
    assign is_3x3_o = (type_o == DTYPE_CONV);
    assign is_dw_o  = (type_o == DTYPE_DW);
    assign is_pw_o  = (type_o == DTYPE_PW);
    assign is_fc_o  = (type_o == DTYPE_FC);

    // n_og：conv/pw 按 8 通道一组；**dw 按 32 通道一组**（一拍出 32 个输出通道）
    //   ⚠ 这一条与 `cnn_outacc.v` 的 dw 写地址 `og*4`、`act_we=4'b1111` 严格对应
    wire [8:0] oc_ceil8  = {1'b0, oc_o[8:3]} + ((oc_o[2:0] == 3'b000) ? 9'd0 : 9'd1);
    wire [8:0] oc_ceil32 = {1'b0, oc_o[8:5]} + ((oc_o[4:0] == 5'b00000) ? 9'd0 : 9'd1);
    wire [8:0] ic_ceil4  = {1'b0, ic_o[9:2]} + ((ic_o[1:0] == 2'b00) ? 9'd0 : 9'd1);
    wire [8:0] ic_ceil32 = {1'b0, ic_o[9:5]} + ((ic_o[4:0] == 5'b00000) ? 9'd0 : 9'd1);

    assign n_og_o = is_dw_o ? oc_ceil32[7:0] : oc_ceil8[7:0];
    assign n_ig_o = is_dw_o ? 8'd1 : (is_3x3_o ? ic_ceil4[7:0] : ic_ceil32[7:0]);

    // 该层的权重字数 = n_og × n_ig
    //   conv = ⌈oc/8⌉×⌈ic/4⌉ ｜ pw = ⌈oc/8⌉×⌈ic/32⌉ ｜ dw = ⌈oc/32⌉
    assign n_words_o = {8'b0, n_og_o} * {8'b0, n_ig_o};

endmodule
