/*=====================================================================
 * start_cmloop.S —— 抓包固件的裸机入口（必须在地址 0）
 *
 * 与 sw/start.S 的唯一差别：bss 用**字**清零，不是逐字节。
 *
 * 为什么在意这 4 倍
 *   这份镜像的全部意义是"让 CoreMark 尽早开始跑"，因为抓包相位被钉在
 *   上电后很短的时间内。CoreMark 的 bss（2000 元素的 list/matrix 数组）
 *   有二三十 KB，逐字节清要一两毫秒，正好落在抓包窗口附近；改成字清零
 *   压到几百微秒，CPU 就能稳定地跑进 CoreMark 再被抓到。
 *
 *   两次清零的语义等价：__bss_start 与 __bss_end 由链接脚本给出，
 *   均按 4 字节对齐（link_menu.ld 里 .bss 带 ALIGN(4)）。
 *=====================================================================*/
    .section .text.start
    .globl _start
_start:
    la   sp, _stack_top

    la   t0, __bss_start
    la   t1, __bss_end
    andi t0, t0, -4                 /* 对齐到字，防止起点未对齐时踩到前一节 */
1:  bgeu t0, t1, 2f
    sw   zero, 0(t0)
    addi t0, t0, 4
    j    1b
2:
    jal  ra, main                   /* 进 C 世界：无限循环跑 CoreMark */
3:  j    3b                          /* main 返回也不许跑飞 */
