// =====================================================================
// gate_regs.h —— cnn_mmio 的寄存器映射 + MMIO 访问宏
//
// 与 rtl/accel/cnn_mmio.v 的表**必须一字不差**。改一边就要改另一边。
//
// ★ 三条使用铁律（都是 cnn_mmio 的设计约束，违反会静默出错）
//   1. `*_go` 是**自清脉冲**：写 1 后下一拍自动归零。写 0 不会"取消"，
//      要等 done。
//   2. `*_done` 是**粘滞位**，必须写 CLR 清掉再等下一次 ——
//      否则第二次会立刻"看到"上一次的完成。
//   3. `rd_*` 一次只抓**第一个 beat**，所以 `rd_nword` 必须写 1；
//      地址是**像素地址**（= ACT 扁平字地址 >> 2），一个地址给该像素
//      32 个通道（4 bank × 8）。
// =====================================================================
#ifndef GATE_REGS_H
#define GATE_REGS_H

#include <stdint.h>

#ifndef GATE_MMIO_BASE
// ★ 与 RISCV_CPU/rtl/soc/soc_top.v 的 FACEREC_EN 窗口一致（独立一页）
//   为什么不用 0x4000_0000：QLN 已占那个窗口并按 0x30 分流，
//   而本寄存器表要用到 0x03C。
#define GATE_MMIO_BASE  0x60000000u
#endif

// 板级其它 MMIO（与 soc_top 一致）
#define GATE_UART_BASE  0x10000000u   // +0x0 写 = 发一字节
#define GATE_TIMER_BASE 0x30000000u   // +0x0 读 = 周期计数器低 32 位

#define GATE_REG(o)  (*(volatile uint32_t *)(GATE_MMIO_BASE + (o)))

// ---- 寄存器偏移 ----
#define R_CTRL      0x000
#define R_STATUS    0x004
#define R_ROI       0x008
#define R_D1B       0x00C
#define R_D1SH      0x010
#define R_NETCFG    0x014
#define R_NETCFG2   0x018
#define R_RDADDR    0x01C
#define R_RDIDX     0x020
#define R_RDDATA    0x024
#define R_ID        0x028
#define R_LAYER     0x02C
#define R_CLR       0x030
#define R_SWSTAT    0x034
#define R_WINSEL    0x038
#define R_WINSEL2   0x03C

// ---- CTRL 位 ----
#define C_NET_GO    0x001
#define C_FSRC_GO   0x002
#define C_ROI_GO    0x004
#define C_D1_GO     0x008
#define C_RD_GO     0x010
#define C_SW_GO     0x020
#define C_HOLD      0x040
#define C_SEEK      0x100
#define C_SEEKXY    0x200

// ---- STATUS 位 ----
#define S_NET_BUSY  0x0001
#define S_NET_DONE  0x0002
#define S_FSRC_BUSY 0x0004
#define S_FSRC_DONE 0x0008
#define S_ROI_BUSY  0x0010
#define S_ROI_DONE  0x0020
#define S_D1_BUSY   0x0040
#define S_D1_DONE   0x0080
#define S_RD_BUSY   0x0100
#define S_RD_DONE   0x0200
#define S_RD_READY  0x0400
#define S_LAYER     0x0800
#define S_TIMEOUT   0x1000
#define S_SW_WIN    0x2000
#define S_SW_FRAME  0x4000
#define S_SW_BUSY   0x8000

// ---- CLR 位 ----
#define X_FSRC      0x01
#define X_ROI       0x02
#define X_D1        0x04
#define X_NET       0x08
#define X_RD        0x10
#define X_LAYER     0x20
#define X_SW_WIN    0x40
#define X_SW_FRAME  0x80
#define X_ALL       0x100

#define GATE_ID_EXPECT 0x46524331u   // "FRC1"

#endif // GATE_REGS_H
