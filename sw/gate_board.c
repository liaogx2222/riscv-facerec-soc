// =====================================================================
// gate_board.c —— 板级胶水：UART 输出 + main 循环
//
// 与 gate_main.c 的分工：
//   gate_main.c  纯编排（只依赖 MMIO 寄存器，不依赖板子）
//   gate_board.c 板子相关的部分（UART 地址、打印格式、启动循环）
// 这样换板子只改这一个文件。
//
// UART（与 RISCV_CPU 的 soc_top 一致）
//   0x1000_0000 + 0x0  写 = 发一个字节
//   0x1000_0000 + 0x4  读 = bit0 busy
//
// ★ 输出节流（提交版）
//   115200 8N1 ≈ 11.5 字节/ms，一行约 90 字节 ≈ 8 ms。
//   如果上电后立刻把 banner + 连续帧结果全灌出去，会出现两种"看起来像坏了"的
//   现象：(1) 远程平台/USB 串口的接收缓冲丢字节；(2) 打开串口、开始保存日志
//   的动作还没完成，前几百毫秒的输出直接丢失。
//   所以这里每一行打印之间都插了延时（GATE_*_GAP_MS），启动也先等 2 s 再说话。
// =====================================================================
#include <stdint.h>

#include "gate_map.h"
#include "gate_regs.h"
#include "gate_core.h"
#include "gate_api.h"
#include "uart_out.h"

#define UART_TX   (*(volatile uint32_t *)GATE_UART_BASE)
#define UART_STAT (*(volatile uint32_t *)(GATE_UART_BASE + 4))   // bit0 = busy

//---------------------------------------------------------------------
// 节流参数（单位 ms）
//   行间 300 ms ⇒ 约 3 行/秒：人眼能跟、串口日志也来得及落盘。
//---------------------------------------------------------------------
#define GATE_INIT_GAP_MS    50u   /* banner 行与行之间（已逐字节节流，留一点余量） */
#define GATE_FRAME_GAP_MS  100u   /* 相邻两帧结果之间                     */
#define GATE_STAT_GAP_MS    50u   /* 8 帧汇总与下一帧之间                 */
/* 上电保活行数：每 2 s 一行，共 N 行（把"能看见"的窗口拉到 2N 秒）。
   出正式提交版时可改成 1，只为诊断时保留 6。                            */
#ifndef GATE_ALIVE_N
#define GATE_ALIVE_N         6u
#endif

//---------------------------------------------------------------------
// 毫秒延时：优先用 SoC 的周期计数器（0x3000_0000，clk_sys 自由运行）
//   定标：CLK_HZ = 101.25 MHz，与 facerec_top 的 CLK_HZ 参数一致。
//   定时器万一没接（读回恒定值），退回空转延时，绝不在这里死等。
//---------------------------------------------------------------------
#define GATE_CPU_HZ   101250000u
#define GATE_MS_TICKS (GATE_CPU_HZ / 1000u)

static int s_tim_state = 0;            /* 0=未测 1=有 2=没有 */

static int tim_alive(void)
{
    if (s_tim_state == 0) {
        volatile uint32_t *t = (volatile uint32_t *)GATE_TIMER_BASE;
        uint32_t a = *t, n;
        for (n = 0; n < 32u; n++) {
            if (*t != a)
                break;
        }
        s_tim_state = (*t != a) ? 1 : 2;
    }
    return s_tim_state == 1;
}

static void delay_ms(uint32_t ms)
{
    if (ms == 0u)
        return;
    if (tim_alive()) {
        volatile uint32_t *t = (volatile uint32_t *)GATE_TIMER_BASE;
        uint32_t t0   = *t;
        uint32_t span = ms * GATE_MS_TICKS;
        uint32_t guard = 0;
        while ((uint32_t)(*t - t0) < span) {
            if (++guard > span)          /* 计数器停摆时的安全网 */
                break;
        }
    } else {
        volatile uint32_t i;             /* 约 1 ms / 15000 次空转 */
        uint32_t n = ms * 15000u;
        for (i = 0; i < n; i++) { }
    }
}

static void delay_loops(uint32_t n)
{
    volatile uint32_t i;
    for (i = 0; i < n; i++) { }
}

//---------------------------------------------------------------------
// freestanding 的 memset / memcpy
//   为什么需要：`-nostdlib` 下没有 libc，而 gcc 会把"循环填字节"优化成
//   `memset` 调用（本文件的 pn() 就踩到了）⇒ 链接期 undefined reference。
//   给两个最小实现，顺便让 gate_core 里可能的 memcpy 也有落点。
//---------------------------------------------------------------------
void *memset(void *d, int c, unsigned int n)
{
    unsigned char *p = (unsigned char *)d;
    while (n--) *p++ = (unsigned char)c;
    return d;
}

void *memcpy(void *d, const void *s, unsigned int n)
{
    unsigned char *p = (unsigned char *)d;
    const unsigned char *q = (const unsigned char *)s;
    while (n--) *p++ = *q++;
    return d;
}

//---------------------------------------------------------------------
// 最小打印（不依赖 libc 的 printf，省 flash/栈）
//---------------------------------------------------------------------
// ★★ 必须等 busy 清掉再写。
//   soc_top 的 UART 是一个发送寄存器 + 启动位，**没有 FIFO**：
//   若 CPU 连续写，后一个字节会把前一个覆盖掉。
//   实测（tb/facerec_smoke_tb）：不加流控时 400k 拍只发出 3 个字节，
//   整个 banner 几乎全丢。很容易误判成"串口线路坏了"。
static void pc(char c)
{
    // 统一走 uart_out.c：busy 轮询有上限（防死等）+ 每字节停 10 ms（防丢字节）。
    // 老实现是本文件内的 20000 次上限轮询；抽出去后菜单/门禁/CoreMark 三条
    // 输出路径共用同一套节流策略。速率可运行时改（菜单 3 键）。
    uart_putc(c);
}

static void ps(const char *s) { while (*s) pc(*s++); }

static void pn(int32_t v, int width)
{
    char buf[12];
    int n = 0, neg = 0;
    uint32_t u;
    if (v < 0) { neg = 1; u = (uint32_t)(-v); } else u = (uint32_t)v;
    do { buf[n++] = (char)('0' + u % 10u); u /= 10u; } while (u);
    while (neg && n < width) buf[n++] = ' ';
    if (neg) buf[n++] = '-';
    while (n < width) buf[n++] = ' ';
    while (n) pc(buf[--n]);
}

static void phex(uint32_t v, int digits)
{
    static const char *H = "0123456789ABCDEF";
    int i;
    for (i = digits - 1; i >= 0; i--)
        pc(H[(v >> (i * 4)) & 0xF]);
}

//---------------------------------------------------------------------
// 结果回调
//---------------------------------------------------------------------
// 统计（比 gate_on_result 先声明）
static uint32_t s_nframe, s_nopen, s_nlabel, s_nfail;
static uint32_t s_cyc_sum;        /* 累计加速器周期，用于给出"均 cyc / fps" */

static const char *tpl_name(int i)
{
#ifdef GATE_MULTI_ID
    if (i < 0 || i >= GATE_PERSON_N)
        return "?";
    return GATE_PERSON_NAMES[i];
#else
    if (i < 0 || i >= GATE_TPL_N)
        return "?";
    return GATE_TPL_NAMES[i];
#endif
}

void gate_on_result(const gate_result_t *r)
{
    /* 一行 ≤ 70 字节、全 ASCII。
     * 字段顺序就是答辩要念的顺序：
     *   第几帧 → 哪个滑窗 → 检索到几个候选 → 检测框
     *   → 相似度 → 认成谁 → PASS/REJ → 门状态 → 加速器周期数
     * 去掉的冗余（上一版每帧 100+ 字节，中文占 3 字节/字且被切开会变 �）：
     *   "cand"/"/2"、"box"、"d=xx"、"【认证通过】"、"门=开启/保持"、" cyc"。
     * d（距离）不再打印 —— cos 已经是同一件事的归一化表达。 */
    ps("f");
    pn((int32_t)r->frame, 0);
    ps(" w");
    pn((int32_t)r->win, 0);
    ps(" c");
    pn((int32_t)r->ncand, 0);
    if (r->x1 >= 0) {
        ps(" (");
        pn(r->x1, 0); ps(","); pn(r->y1, 0); ps(",");
        pn(r->x2, 0); ps(","); pn(r->y2, 0); ps(")");
        ps(" cos");
        if (r->cos1000 < 0) { ps("---"); }
        else { pn(r->cos1000 / 1000, 0); ps("."); pn(r->cos1000 % 1000, 3); }
    } else {
        ps(" -");
    }
    ps(" ");
    if (r->label >= 0) {
        ps(tpl_name(r->label));
    } else {
        ps("unreg");
    }
    ps(r->label >= 0 ? " PASS " : " REJ  ");
    ps(r->open ? "g1 " : "g0 ");
    pn((int32_t)r->cyc, 7);
    ps("c\r\n");

    if (r->label >= 0) s_nlabel++;
    else               s_nfail++;
    s_cyc_sum += r->cyc;

    // 帧间留空档：逐字节节流已提供间距，这里只补一点，让接收端有机会排空
    delay_ms(GATE_FRAME_GAP_MS);
}

//---------------------------------------------------------------------
// 门禁结果统计（给答辩用的汇总）
//---------------------------------------------------------------------
uint32_t gate_cycle(void)
{
    // SoC 的周期计数器（0x3000_0000 读 = 低 32 位）
    return *(volatile uint32_t *)GATE_TIMER_BASE;
}

void gate_on_init(void)
{
    /* 两行 82 字节说完配置。上一版用了 4 行中文、约 160 字节，是最容易丢的一段。 */
    ps("[GATE] ");
    pn(GATE_BUF_W, 0); ps("x"); pn(GATE_BUF_H, 0);
    ps(" det"); pn(GATE_DET_W, 0); ps("x"); pn(GATE_DET_H, 0);
    ps(" sw"); pn(GATE_SW_NX, 0); ps("x"); pn(GATE_SW_NY, 0);
    ps(" tpl "); ps(tpl_name(0));
    ps(" tau"); pn(GATE_TAU_X1000, 0);
    ps("\r\n");
    delay_ms(GATE_INIT_GAP_MS);

    /* 一行 ≤ 22 字符：平台一次只取 24 字节（见 uart_out.h），超了就被劈开 */
    ps("[GATE] uart 115200\r\n");
    ps("gap=");
    pn((int32_t)g_uart_gap_ms, 0);
    ps(" b");
    pn((int32_t)g_uart_intra_ms, 0);
    ps("\r\n");
    delay_ms(GATE_INIT_GAP_MS);
}

//---------------------------------------------------------------------
// CNN 域健康诊断（2026-11-01）
//   0x2000_0000 的读由 clk_sys 直接给出、**不经过 CNN 握手桥**，
//   所以加速器完全不应答时这里照样能打印 —— 这是把"死机"变成"报告"的关键。
//   bit8 cnn_clk 是否在跑（间隔 3 ms 连读两次，不同即在跑）
//   bit9 fr_resetn（CNN 域复位是否已释放）
//   bit10 握手桥当前忙 / bit11 上电以来是否完成过任何一笔
//---------------------------------------------------------------------
static void diag_print(void)
{
    uint32_t d0 = *(volatile uint32_t *)0x20000000u;
    uint32_t d1;
    int clk_ok, rst_ok, busy, done_ok;

    delay_ms(3u);
    d1      = *(volatile uint32_t *)0x20000000u;
    clk_ok  = (int)(((d0 ^ d1) >> 8) & 1u);
    rst_ok  = (int)((d1 >> 9) & 1u);
    busy    = (int)((d1 >> 10) & 1u);
    done_ok = (int)((d1 >> 11) & 1u);

    ps("[D] clk");   ps(clk_ok ? "1" : "0");
    ps(" rst");      ps(rst_ok ? "1" : "0");
    ps(" busy");     ps(busy ? "1" : "0");
    ps(" done");     ps(done_ok ? "1" : "0");
    ps(" 0x");       phex(d1, 8);
    ps("\r\n");
}

//---------------------------------------------------------------------
// LED 状态（低电平点亮，位含义见 rtl/board/facerec_top.v）
//   bit0 → LED1  CPU 心跳（亮灭交替 = CPU 在跑）
//   bit1 → LED2  自检结果（常亮 = 加速器 ID 正确）
//   bit2 → LED3  门状态（亮 = 门开）
//   LED0 由硬件直连 resetn，固件管不到。
// 平台串口不可靠时，这 4 个灯就是唯一的可信输出。
//---------------------------------------------------------------------
static uint32_t s_led = 0u;

static void led_flush(void)
{
    *(volatile uint32_t *)0x20000000u = s_led;
}

int gate_app_run(int quick)
{
    uint32_t f;

    // quick=1：由菜单固件调起，跳过"上电保活 12 s"和启动横幅，
    //          直接自检 → 出帧。quick=0：独立上电的完整流程。
    if (!quick) {
        uint32_t n;

        // ★ 关掉 RTL 连续信标（写 0x2000_0004 = 任意值）。
        //   信标每 20 ms 发一个 'U'，不先关掉就会和这里的第一个字符抢 TX。
        //   写不进去也不影响后续：CPU 第一次写 UART 时硬件也会自动关信标。
        *(volatile uint32_t *)0x20000004u = 1u;
        delay_loops(120000u);            // ~1 ms，等信标最后一个字节发完

        // 能执行到这里就说明 CPU 出来了：立刻点灯，之后一直闪
        s_led = 1u;
        led_flush();

        // 这一行是"CPU 真的在跑"的第一个证据：除了 UART 的写/状态位之外
        // 不依赖任何东西（不读 ID、不读定时器）。
        ps("\r\n[BOOT] cpu alive\r\n");

    // ★ 保活：每 2 s 一行，共 GATE_ALIVE_N 行；期间 LED1 以 1 Hz 翻转
    //   ⇒ 即使串口一个字都收不到，只要 LED1 在闪就能确认 CPU 在执行。
    for (n = 1u; n <= GATE_ALIVE_N; n++) {
        uint32_t k;
        for (k = 0u; k < 4u; k++) {          /* 2 s 拆成 4×500 ms */
            s_led ^= 1u;
            led_flush();
            delay_ms(500u);
        }
        if (n == 1u) {
            ps(tim_alive() ? "[BOOT] tim ok 101.25MHz\r\n"
                           : "[BOOT] tim none (spin)\r\n");
        }
        ps("[A]");
        pn((int32_t)(n * 2u), 2);
        ps("s\r\n");
        diag_print();          /* 每一轮都带上 CNN 域状态，任何时候接上都看得到 */
    }
    delay_ms(GATE_INIT_GAP_MS);

    // ★★ CNN 域健康诊断（2026-11-01）—— 已在保活循环里逐轮打印，这里再打一次收尾
    //   0x2000_0000 的读由 clk_sys 直接给出、**不经过 CNN 握手桥**，
    //   所以加速器完全不应答时这里照样能打印 —— 这是把"死机"变成"报告"的关键。
    diag_print();
    delay_ms(GATE_INIT_GAP_MS);
    }   /* if (!quick) */

    if (gate_selftest() != 0) {
        uint32_t id = GATE_REG(R_ID);
        uint32_t d2 = *(volatile uint32_t *)0x20000000u;
        ps("[ERR] id=0x");
        phex(id, 8);
        ps(" want 46524331\r\n");
        // 0xDEADBEEF = RTL 的读超时哨兵：桥在 0.65 ms 内没有任何应答
        ps(id == 0xDEADBEEFu ? "[ERR] timeout: bridge no ack 0.65ms\r\n"
                             : "[ERR] bridge acked, value wrong\r\n");
        ps("[DIAG] done="); phex(d2, 8);
        ps("\r\n");
        // ★ 自检失败：LED1 快闪 ~4 Hz、LED2 保持灭。
        //   不看串口也能一眼认出"自检没过"，与正常运行的慢闪区分开。
        s_led = 0u;                      /* bit1=0 ⇒ LED2 灭 */
        led_flush();
        for (;;) {
            s_led ^= 1u;
            led_flush();
            delay_ms(120u);
        }
    }

    // 自检通过：LED2 常亮，LED1 继续按帧翻转
    s_led = 1u | 2u;
    led_flush();

    ps("[GATE] ok id 46524331\r\n");
    delay_ms(GATE_INIT_GAP_MS);

    gate_init();

    for (f = 0;; f++) {
        uint8_t open = (uint8_t)gate_run_frame(f);
        // LED：bit0 每帧翻转（≈1.6 Hz 慢闪）= CPU 在跑并出帧
        //      bit1 常亮 = 自检过        bit2 = 门状态
        s_led = 2u | (open ? 4u : 0u) | (f & 1u);
        led_flush();
        s_nframe++;
        if (open) s_nopen++;
        if (f % 8 == 7) {
            uint32_t avg = s_cyc_sum / s_nframe;
            /* 一行 41 字节。avg 是**加速器周期**，与串口速率无关 ——
             * 答辩讲吞吐要引用这一项（fps），不要用日志时间戳算帧率。 */
            ps("[s] f"); pn((int32_t)s_nframe, 0);
            ps(" open"); pn((int32_t)s_nopen, 0);
            ps(" id");   pn((int32_t)s_nlabel, 0);
            ps(" rej");  pn((int32_t)s_nfail, 0);
            ps(" avg");  pn((int32_t)avg, 0);
            ps("c ");    pn((int32_t)(101250000u / avg), 0);
            ps("fps\r\n");
            delay_ms(GATE_STAT_GAP_MS);
        }
    }
    return 0;
}

/* ---- 旧入口：独立上电直接跑门禁（菜单固件用 -DWITH_MENU 关掉这段）---- */
#ifndef WITH_MENU
int main(void)
{
    return gate_app_run(0);
}
#endif
