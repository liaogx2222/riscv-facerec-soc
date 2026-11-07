//=====================================================================
// cnn_act.v : **M4：ACT 按通道组分 4 个 bank**（新模块）
//
// 为什么必须 bank 化（基线 §4.2，勿回退）：
//
//   · 1×1 层需要 **32 通道/拍**。若把 ACT 字加宽到 256 b（32 通道），
//     生产者每像素只写 8 通道 ⇒ **部分写** ⇒ 按 CIFAR 血泪
//     （"只有单条整字写才推断成块 RAM"）会落成**分布式 RAM**。
//   · 改成 **4 个独立 bank**（bank b 存通道 [8b, 8b+8)，字宽 64 b）后：
//       写：每像素写 8 通道 ⇒ 命中 **1 个 bank 的 1 个字** ⇒ **单条整字写** ✓ BRAM 安全
//       读：一拍并行读 4 个 bank ⇒ **32 通道/拍** ✓
//       容量：4 bank × (像素数 × 64 b) = 像素数 × 256 b
//             —— 与"32 通道字"**等容量，零额外存储** ✓
//       带宽自适应：3×3 层只需 4 通道（都在 bank 0）⇒ `bank_mask=4'b0001` ✓
//
// ★ 与 CIFAR 的差别：CIFAR 只有一块 64 b 的 `cnn_sdpram`（8 通道/拍），
//   3×3 靠 ICG=4 分组分 4 拍喂；本模块把 4 块 bank 并行化，一拍给 32 通道。
//
// `bank_mask` 语义：bit b = 1 表示**读** bank b；未选中的 bank 输出强制 0
//   （不读 ⇒ 保持旧值，必须屏蔽掉，否则下游会拿到陈旧数据）。
//
// ⚠ 容量（DEPTH）**待 ACT 内存规划器定** —— 与描述符的 `in_base`/`out_base` 同一处缺口。
//   参考量级（基线 §5）：检测 ACT 峰值 144 KB、识别 48 KB。
//   单张 64×48×32 特征图 = 3072 像素 × 256 b = 96 KB ⇒ 本模块 DEPTH ≥ 3072 像素。
//=====================================================================
`timescale 1ns/1ps

module cnn_act #(
    parameter BANKS   = 4,        // 通道组数
    parameter CH_BANK = 8,        // 每组通道数（字宽 = CH_BANK*8）
    parameter AW      = 8,        // 通道内位宽
    parameter DEPTH   = 8192,     // 像素数（字深）
    parameter AAW     = 13,
    parameter B0DEPTH = DEPTH
)(
    input  wire                              clk,
    // ---- 写口：每 bank 独立写使能（单条整字写）----
    //   conv/pw：we = 4'b0001 << (og&3) —— 只写 1 个 bank（8 个输出通道）
    //   dw     ：we = 4'b1111            —— 一拍写满 4 个 bank（32 个输出通道）
    input  wire [BANKS-1:0]                  we,
    input  wire [AAW-1:0]                    waddr,    // 像素地址
    input  wire [BANKS*CH_BANK*AW-1:0]       wdata,    // bank b 在 [b*64 +: 64]
    // ---- 读口：一拍并行读多个 bank ----
    input  wire [AAW-1:0]                    raddr,
    input  wire [BANKS-1:0]                  bank_mask,
    output wire [BANKS*CH_BANK*AW-1:0]       rdata     // bank b 在 [b*64 +: 64]
);

    //-----------------------------------------------------------------
    // ★★ 2026-10-23（run12）：**写口就近打一拍**
    //   run11 实测头号路径 = `u_acc` 的地址乘法 APM → ACT 写地址
    //   （`gopdrm_36k/ADA0[4]`）：**只有 2 级逻辑（共 2.95 ns）但走线 4.05 ns**，
    //   因为写地址寄存器在 outacc 里，离 ACT 的 64 个 DRAM 实例太远。
    //   ⇒ 在 ACT 内部再打一拍（PDS 会把它放在 DRAM 旁），把「远距离扇出」
    //     变成「寄存器 → 相邻 DRAM」。
    //   代价：写入整体晚 1 拍。**不需要改任何写方的排空计数**——
    //   写使能脉冲被本级寄存器锁住，写方之后拉低也照样落盘；
    //   且所有下游（引擎读回、tb 读回口）都在 done 后若干拍才读。
    //-----------------------------------------------------------------
    reg [BANKS-1:0]            we_r;
    reg [AAW-1:0]              waddr_r;
    reg [BANKS*CH_BANK*AW-1:0] wdata_r;
    always @(posedge clk) begin
        we_r    <= we;
        waddr_r <= waddr;
        wdata_r <= wdata;
    end

    genvar b;
    generate
        for (b = 0; b < BANKS; b = b + 1) begin : g_bank
            wire [CH_BANK*AW-1:0] rd;
            cnn_sdpram #(
                .W(CH_BANK*AW), .D((b == 0) ? B0DEPTH : DEPTH), .AW(AAW)
            ) u_bank (
                .clk   (clk),
                .we    (we_r[b]),                  // ★ 单条整字写，只有选中的 bank 被写
                .waddr (waddr_r),
                .wdata (wdata_r[(b*CH_BANK)*AW +: CH_BANK*AW]),
                .re    (bank_mask[b]),
                .raddr (raddr),
                .rdata (rd)
            );
            // 未选中的 bank 强制 0（不读 ⇒ 保持旧值，必须屏蔽）
            assign rdata[(b*CH_BANK)*AW +: CH_BANK*AW] =
                bank_mask[b] ? rd : {(CH_BANK*AW){1'b0}};
        end
    endgenerate

endmodule
