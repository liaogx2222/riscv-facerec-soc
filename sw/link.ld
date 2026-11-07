/*=====================================================================
 * link.ld —— FaceRec 门禁固件的裸机链接脚本
 *
 *   _start 必须在 0（CPU 复位 PC = 0）
 *   主存 16 KB（0x0000_0000 ~ 0x0000_3FFF），栈顶 0x3FF0
 *
 *   ⚠ 不要把 RAM 放大到 256 KB：板级 `simple_ram` 每 32 KB 吃 8 个 DRM，
 *     256 KB 就是 64 DRM，会把 DRM 预算（415 个）里近 1/6 吃掉。
 *     本固件 text ≈ 10.7 KB + bss ≈ 3.0 KB ⇒ 16 KB 足够。
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

    /* 栈（不放东西，只在 start.S 里用地址 0x3FF0）*/
    _stack_top = 0x7FF0;
    ASSERT(__bss_end + 4096 <= _stack_top, "Firmware leaves less than 4KB stack")

    /DISCARD/ : { *(.eh_frame) *(.comment) *(.riscv.attributes) }
}
