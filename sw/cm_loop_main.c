/*=====================================================================
 * cm_loop_main.c —— ILA 抓包专用固件：上电即无限循环跑 CoreMark 2K 性能段
 *
 * 为什么需要这份固件
 *   平台没有 Trigger Setup 入口，抓包的相位由 Debugger 下载位流那一刻决定，
 *   也就是上电后很短的时间内。菜单固件上电先跑 uart_delay_ms(1500)，
 *   所以无论按多少次 Run，512 拍永远落在那段延时循环里（0x0946~0x0959），
 *   看不到 CoreMark。要让波形落在算法段，只能让 CoreMark 从上电就跑。
 *
 * 为什么把打印全部丢掉
 *   一轮 CoreMark 在 27 MHz 下约 194,583 拍 ≈ 7.2 ms；而它的输出约 400 字符，
 *   115200 波特率下要 35 ms —— 打印比计算慢 5 倍。若照常打印，CPU 有 83%
 *   的时间停在串口发送里，抓包大概率抓到 uart_putc 的忙等循环。
 *   这里把 ee_printf 整个替换成空实现，只保留每个迭代一个字节的心跳：
 *     '.' = 该轮 coremark_main 返回 0（三组 CRC 全部校验通过）
 *     'X' = 返回非 0（校验失败，看得到就是有问题）
 *
 * 与跑分固件的差异（只影响观测，不影响被测代码）
 *   - 不链接 uart_out.c / menu_main.c / gate_*.c，只留 CoreMark 本体
 *   - 迭代次数 ITERATIONS=1（2K performance 段，与 perf_sim 同一口径）
 *   - start_cmloop.S 用字清零 bss（原 start.S 是逐字节，慢 4 倍），
 *     让 CPU 在上电后几十微秒内就进入 CoreMark，抓包相位更稳
 *=====================================================================*/
#include <stdint.h>

#define UART_TX    (*(volatile uint32_t *)0x10000000u)
#define UART_STAT  (*(volatile uint32_t *)0x10000004u)

extern int coremark_main(void);

/* CoreMark 的打印出口。coremark.h 在 HAS_PRINTF=0 时把它叫 ee_printf，
   若哪天打开 HAS_PRINTF 就会变成 printf —— 两个都定义，链接不会失配。 */
int ee_printf(const char *fmt, ...)
{
    (void)fmt;
    return 0;
}

int printf(const char *fmt, ...)
{
    (void)fmt;
    return 0;
}

/* 单字节裸发：不走 uart_out.c 的"整行 22 字节 + 600 ms 静默"策略，
   那套是为菜单可读性设计的，这里只要一个心跳。 */
static void cm_putc(char c)
{
    uint32_t guard = 2000000u;
    while ((UART_STAT & 1u) && --guard) {
        /* 忙等有上限：busy 若恒为 1 也不许卡死 */
    }
    UART_TX = (uint32_t)(uint8_t)c;
}

int main(void)
{
    for (;;) {
        int rc = coremark_main();
        cm_putc(rc == 0 ? '.' : 'X');
    }
}
