/*=====================================================================
 * start.S —— FaceRec 门禁固件的裸机入口（CPU 复位 PC = 0，本文件必须在地址 0）
 *
 *   ① 设栈指针（16 KB RAM 顶 = 0x3FF0；⚠ 不能用 0x4000，那是上界外 1 字，
 *      任何 0(sp) 访问会被地址截断回绕到 0x0 = 代码区 ⇒ 静默踩栈）
 *   ② 清 .bss（C 标准保证静态变量初值 0；不靠编译器自动做，自己清）
 *   ③ 进 C 的 main()
 *   ④ main 返回也不许跑飞
 *
 * 为什么 bss 要自己清
 *   `sw/gate_main.c` 里的 s_cls / s_cand / s_ncand 都是静态的。
 *   虽然每个都"写后读"，但一旦以后有人加了"读后写"的静态变量，
 *   不清 bss 就会读到复位后的随机值（BRAM 初值不确定），
 *   而且这种 bug 只在板上出现、仿真里（$readmemh 清零）看不到。
 *=====================================================================*/
    .section .text.start
    .globl _start
_start:
    la   sp, _stack_top

    la   t0, __bss_start
    la   t1, __bss_end
1:  bgeu t0, t1, 2f
    sb   zero, 0(t0)
    addi t0, t0, 1
    j    1b
2:
    jal  ra, main               /* 进 C 世界（gate_board.c）*/
3:  j    3b                      /* main 返回也不许跑飞 */
