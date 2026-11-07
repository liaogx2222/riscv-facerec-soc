/*=====================================================================
 * link_menu.ld —— 「CoreMark + 人脸识别」双功能菜单固件
 *
 *   和 sw/link.ld 的区别只有两处：
 *     ① 允许用到 64 KB（0x0000_0000 ~ 0x0000_FFFF）
 *        —— CoreMark 的 -O3 展开体 (~35 KB) + 门禁 (~13 KB) 装不进 32 KB
 *     ② 栈顶抬到 0xFFF0
 *
 *   代价：板级 simple_ram 的 ADDR_W 要从 15 改成 16（32 KB → 64 KB，
 *   多约 8 个 DRM）。改完必须重新综合，否则 INIT 超深会被截断。
 *=====================================================================*/
OUTPUT_ARCH(riscv)
ENTRY(_start)

SECTIONS {
    . = 0x0;
    .text : {
        *(.text.start)
        *(.text*)
    }
    .rodata : { *(.rodata*) *(.srodata*) }
    .data   : { *(.data*) *(.sdata*) }

    . = ALIGN(16);
    __bss_start = .;
    .bss    : { *(.bss*) *(.sbss*) *(COMMON) }
    . = ALIGN(16);
    __bss_end = .;

    _stack_top = 0xFFF0;
    ASSERT(__bss_end + 8192 <= _stack_top, "Firmware leaves less than 8KB stack")

    /DISCARD/ : { *(.eh_frame) *(.comment) *(.riscv.attributes) }
}
