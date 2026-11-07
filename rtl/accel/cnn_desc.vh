//=====================================================================
// cnn_desc.vh : Layer Descriptor 字段布局（FaceRec_SoC 版，**M8 扩展**）
//
// 描述符 = 8 × 32b，经 Command Queue 入队（每层一项），控制器逐项取出、解码、驱动子引擎。
// 所有字段值由固件按模型（`model/w/layer_cfg.json` 的层表）写出。
//
// ★ 相对 CIFAR 版（`CIFAR10_CNN_Accel/rtl/accel/cnn_desc.vh`）的改动（基线 §7）
//   · TYPE 新增 **3=DW / 4=PW(1×1)**（原只有 0=CONV / 1=POOL / 2=FC / 15=END）
//   · 新增 `stride`（1/2）、`lin_out`、`bank_mask`、`fuse`
//   · `ih/iw` 6 → **8 位**（检测有 64×48 的层）
//   · `in_base/out_base` 14 → **16 位**（M13）
//   · `weight_base` 12 → **14 位**
//   · **删除 `sh_bn`** —— M5 取消了 BN 乘加级（requant 链改为
//     `>>>sh_req → clamp → +shc → clamp`，与 `hwnet.conv_int` 同口径）
//
// ---------------------------------------------------------------------------
// w0
//   [3:0]   TYPE        0=CONV(3×3 g=1) ｜ 1=POOL ｜ 2=FC ｜ **3=DW** ｜ **4=PW(1×1)** ｜ 15=END
//   [4]     lin_out     1 = 线性输出（clamp [-128,127]）｜ 0 = ReLU（clamp [0,127]）
//   [5]     stride      1 = s2 ｜ 0 = s1     ★ stride-2 与 stride-1 权重相同，只是输出隔行隔列采样
//   [6]     bank_mask   1 = 读 4 个 ACT bank（dw/pw 需 32 通道/拍）
//                       0 = 只读 1 个 bank（3×3 只需 4 通道/拍）
//   [7]     fuse        1 = 本层输出**直接进下一级**（dw→pw 融合），不写 ACT、省一次往返
//   [8]     src_img     1 = 输入取 IMG（网络第一层）｜ 0 = 取 ACT
//   [9]     in_sh_d2    保留（FaceRec_SoC 无 dense 影子）
//   [15:10] 保留（0）
//   [23:16] brom_base   8 位：`shc` 常数表基址（brom_addr = bbase + og，按通道偏置）
//   [31:24] 保留（0）
//
// w1: [7:0]ih  [15:8]iw  [25:16]ic                    ← ih/iw 拓到 8 位
// w2: [8:0]oc  [21:12]nchunk（dense 用；conv/dw/pw 不用）
// w3: [13:0]out_wpg（输出每像素字数 = ceil(oc/8)）  [31:16]保留
// w4: [15:0]in_base  [31:16]out_base                 ← 拓到 16 位
// w5: [13:0]weight_base（Weight RAM 2304b 字地址）    ← 拓到 14 位
// w6: [7:0]sh_req  [31:8]保留（**sh_bn 已取消**）
// w7: 保留（0）
//=====================================================================
`ifndef CNN_DESC_VH
`define CNN_DESC_VH

// ---- TYPE ------------------------------------------------------------
localparam [3:0] DTYPE_CONV = 4'd0;    // 3×3, g=1（ICG=4 通道/拍）
localparam [3:0] DTYPE_POOL = 4'd1;    // 2×2 s2 max（UltraFace 不用，保留）
localparam [3:0] DTYPE_FC   = 4'd2;    // 全连接（识别的 D1）
localparam [3:0] DTYPE_DW   = 4'd3;    // ★ depthwise 3×3（19 层）
localparam [3:0] DTYPE_PW   = 4'd4;    // ★ 1×1 pointwise（20 层）
localparam [3:0] DTYPE_END  = 4'd15;   // 队列结束哨兵

// ---- w0 --------------------------------------------------------------
localparam D0_TYPE_LO    = 0;
localparam D0_TYPE_HI    = 3;
localparam D0_LIN_OUT    = 4;
localparam D0_STRIDE     = 5;
localparam D0_BANK_MASK  = 6;
localparam D0_FUSE       = 7;
localparam D0_SRC_IMG    = 8;
localparam D0_IN_SH_D2   = 9;
localparam D0_BROM_LO    = 16;
localparam D0_BROM_HI    = 23;

// ---- w1 --------------------------------------------------------------
localparam D1_IH_LO      = 0;
localparam D1_IH_HI      = 7;
localparam D1_IW_LO      = 8;
localparam D1_IW_HI      = 15;
localparam D1_IC_LO      = 16;
localparam D1_IC_HI      = 25;

// ---- w2 --------------------------------------------------------------
localparam D2_OC_LO      = 0;
localparam D2_OC_HI      = 8;
localparam D2_NCH_LO     = 12;
localparam D2_NCH_HI     = 21;

// ---- w3 --------------------------------------------------------------
localparam D3_OWPG_LO    = 0;
localparam D3_OWPG_HI    = 13;

// ---- w4 --------------------------------------------------------------
localparam D4_IN_LO      = 0;
localparam D4_IN_HI      = 15;
localparam D4_OUT_LO     = 16;
localparam D4_OUT_HI     = 31;

// ---- w5 --------------------------------------------------------------
localparam D5_WB_LO      = 0;
localparam D5_WB_HI      = 13;

// ---- w6 --------------------------------------------------------------
localparam D6_SHREQ_LO   = 0;
localparam D6_SHREQ_HI   = 7;

// ---- 描述符字数 ------------------------------------------------------
localparam DESC_WORDS    = 8;

`endif // CNN_DESC_VH
