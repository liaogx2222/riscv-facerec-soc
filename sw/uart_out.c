// =====================================================================
// uart_out.c —— 整行发货的串口输出（设计说明见 uart_out.h）
//
// 寄存器（与 rtl/soc/soc_top.v 一致）
//   0x1000_0000  写 = TXDATA（启动发送）
//   0x1000_0004  读 = STATUS，bit0 = tx_busy
//   0x3000_0000  读 = 自由运行周期计数器低 32 位（clk_sys = 101.25 MHz）
// =====================================================================
#include "uart_out.h"

#define UART_TX    (*(volatile uint32_t *)0x10000000u)
#define UART_STAT  (*(volatile uint32_t *)0x10000004u)
#define TIMER_LO   (*(volatile uint32_t *)0x30000000u)

#define CPU_HZ     101250000u
#define MS_TICKS   (CPU_HZ / 1000u)

uint32_t g_uart_gap_ms   = UART_GAP_DEFAULT_MS;
uint32_t g_uart_intra_ms = UART_INTRA_DEFAULT_MS;

static char s_seg[UART_SEG_MAX];   /* 正在攒的这一段 */
static int  s_segn;                /* 已经攒了几个字符 */
static int  s_tim = 0;             /* 0=未判定 1=有计数器 2=没有 */

void uart_set_pace(uint32_t gap_ms, uint32_t intra_ms)
{
    g_uart_gap_ms   = gap_ms;
    g_uart_intra_ms = intra_ms;
}

static int tim_ok(void)
{
    if (s_tim == 0) {
        uint32_t a = TIMER_LO, n;
        for (n = 0; n < 32u; n++) {
            if (TIMER_LO != a) break;
        }
        s_tim = (TIMER_LO != a) ? 1 : 2;
    }
    return s_tim == 1;
}

void uart_delay_ms(uint32_t ms)
{
    if (ms == 0u) return;
    if (tim_ok()) {
        uint32_t t0 = TIMER_LO;
        uint32_t span = ms * MS_TICKS;
        uint32_t guard = 0;
        while ((uint32_t)(TIMER_LO - t0) < span) {
            if (++guard > span) break;     /* 计数器停摆时的安全网 */
        }
    } else {
        volatile uint32_t i;
        uint32_t n = ms * 15000u;          /* 约 1 ms / 15000 次空转 */
        for (i = 0; i < n; i++) { }
    }
}

/* 阻塞到 TX 空闲再写一个字节。一个字节 86.8 µs（115200 8N1）。 */
static void put_raw(char c)
{
    uint32_t n = 0;
    while ((UART_STAT & 1u) && (++n < UART_POLL_MAX)) { }
    UART_TX = (uint32_t)(uint8_t)c;
}

/* 发一段：n 个可见字符 + CRLF，然后静默 g_uart_gap_ms。
 *
 * 段内**全速**（g_uart_intra_ms = 0 时）是刻意的：
 *   平台一次读 24 字节，只要这 24 字节在"一次轮询"的间隙里全部到齐，
 *   读到的就是完整一段。段拉得越短，被轮询撞进中间的概率越小
 *   （22 字节 1.9 ms vs 轮询 500 ms ⇒ 0.3%）。
 *   早期的 20 ms/字节是"持续慢速"，反而让缓冲区长期满着、切割点随机。 */
static void emit_seg(int n)
{
    int i;

    for (i = 0; i < n; i++) {
        put_raw(s_seg[i]);
        if (g_uart_intra_ms) uart_delay_ms(g_uart_intra_ms);
    }
    put_raw('\r');
    put_raw('\n');
    s_segn = 0;

    if (g_uart_gap_ms) uart_delay_ms(g_uart_gap_ms);
}

void uart_putc(char c)
{
    int cut, rest, i, n;

    if (c == '\r') return;             /* 换行统一由 emit_seg 补 CRLF */

    if (c == '\n') {
        if (s_segn) emit_seg(s_segn);
        else {                          /* 空行也要真的换一行 */
            put_raw('\r');
            put_raw('\n');
            if (g_uart_gap_ms) uart_delay_ms(g_uart_gap_ms);
        }
        return;
    }

    s_seg[s_segn++] = c;
    if ((uint32_t)s_segn < UART_SEG_MAX) return;

    /* 满了：回退到最近的空格断开，保证不在词中间切。
       找不到空格（超长单词）才硬切。空格至少要在第 8 个字符之后，
       否则断出来会是一个孤零零的两三字残句。 */
    n   = s_segn;
    cut = n;
    for (i = n - 1; i >= 8; i--) {
        if (s_seg[i] == ' ') { cut = i; break; }
    }

    if (cut == n) {
        emit_seg(n);
        return;
    }

    /* ⚠ emit_seg() 会把 s_segn 清 0，所以剩余长度必须在**调用前**算好。
       第一版写成 `emit_seg(cut); rest = s_segn - cut - 1;`，
       rest 变成负数、s_segn 变负、下一拍写 s_seg[负数] —— 内存踩坏。
       这类错在板上表现为"菜单出两行就卡死"，很难往串口模块上想。 */
    rest = n - cut - 1;
    emit_seg(cut);                      /* 发 [0, cut)，断点那个空格丢掉 */
    for (i = 0; i < rest; i++) s_seg[i] = s_seg[cut + 1 + i];
    s_segn = rest;
}

void uart_puts(const char *s)
{
    while (*s) uart_putc(*s++);
}

void uart_flush(void)
{
    if (s_segn) emit_seg(s_segn);
}
