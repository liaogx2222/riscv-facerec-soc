// =====================================================================
// menu_main.c —— 串口菜单固件（CoreMark / 人脸识别 二选一）
//
// 交互
//   上电打印菜单 → 串口输入 '1' 跑 CoreMark v1.0 → 跑完回到菜单
//                串口输入 '2' 跑人脸识别门禁（连续出帧，复位才退出）
//                串口输入 '3' 循环切换串口速率档位
//
// 为什么单独一个文件
//   门禁固件自己的 main 在 gate_board.c 里（`gate_app_run(0)`），
//   菜单固件把同一份门禁逻辑以 `gate_app_run(1)` 调起来（跳过启动保活循环），
//   两边共用 gate_main.c / gate_core.c，一行推理代码都不用改。
//
// 输出为什么要"按整行发货"（2026-11-02 凌晨定案）
//   平台的串口工具**每次只取 24 字节**（实测：稳态段连续十余条 RX 的负载
//   恰好 24 字符，间隔 480~510 ms，偶发 127 / 860 ms）。
//   原来的"每字节停 20 ms"让产速（25 B/500 ms）正好卡在读取上限，
//   缓冲区长期是满的，读指针落在句中的位置完全随机
//   ⇒ 现象就是每条 RX 都断在词中间（"…2=Gate 3" / "=UART 20ms"）。
//   调慢不管用 —— 切割点照样随机。现在的做法见 uart_out.h：
//     ① 一行不超过 22 可视字符（+CRLF = 24，正好一次读取）；
//     ② 段内全速 1.9 ms 发完（比一次轮询短两个数量级，撞进去的概率 0.3%）；
//     ③ 段后静默 600 ms（> 轮询 500 ms），保证缓冲区被排空一次。
//   ⇒ 每条 RX = 一整行，且断点一定在词边界。
//
// 节流档位
//   全部经 uart_out.c 的 uart_putc()，菜单按 3 循环切换
//   （段间静默 ms / 段内每字节 ms）：600/0 → 600/1 → 800/1 → 400/0 → 200/0。
//   第一下按是"段内慢发"（怕接收端按字节丢时的救场）。
//   CoreMark 的打印发生在 start_time()/stop_time() 之外，
//   节流不会污染跑分（分数只看 stop-start 之间的周期数）。
// =====================================================================
#include <stdint.h>

#include "uart_out.h"

/* 菜单里要不要出现键 4（ILA）。
 *
 * 为什么要这个开关：ILA 的 MMIO（0x2000_0008..14）在
 *   candidates/cpu101_cnn202_ila  里才有；cpu101_cnn202_menu 的内部逻辑是
 *   ILA 之前的版本，没有这几个寄存器。
 * 如果两种 RTL 用同一份固件，menu 版的键 4 会写进未映射地址、
 * 再读回垃圾、白等 0.1 s、然后打印 30 行乱码 —— 演示时误按就是事故。
 * ⇒ 由 build_menu_fw.py 的 --ila 决定，不带就是 0，菜单只到 3。 */
#ifndef WITH_ILA
#define WITH_ILA 0
#endif

/* ---- UART 接收（rtl/soc/soc_top.v）---- */
#define UART_RXDATA  (*(volatile uint32_t *)0x10000008u)   /* 读即清 */
#define UART_RXSTAT  (*(volatile uint32_t *)0x1000000Cu)   /* bit0=valid bit1=overrun */

/* ---- GPIO（0x2000_0000 = LED[3:0]，低电平点亮）---- */
#define GPIO_OUT     (*(volatile uint32_t *)0x20000000u)
#define GPIO_BCN_OFF (*(volatile uint32_t *)0x20000004u)

#if WITH_ILA
/* ---- 片上逻辑分析仪 ILA（rtl/soc/ila_core.v，挂 0x2000_00xx）----
 *   +0x08 W: CTRL {0=arm 1=soft 2=trig_sel}   R: {wr_ptr, done, armed}
 *   +0x0C W: 读索引 0..255                    R: 该索引样本
 *   +0x10 W: 触发 PC                          R: 回读
 *   +0x14                                     R: 实时 probe
 *
 * 为什么自己做一个 ILA 而不是用 PDS 的 ips_debug_core：
 *   厂商 Debug Core 的数据回读口是 JTAG Hub（cdt_dbg.exe），
 *   远程平台只给一条 UART，抓得到、读不出。
 *   这个核把采集缓冲区挂成 MMIO，回读走同一条串口，端到端只要一个通道。 */
#define ILA_CTRL     (*(volatile uint32_t *)0x20000008u)
#define ILA_IDX      (*(volatile uint32_t *)0x2000000Cu)
#define ILA_PC       (*(volatile uint32_t *)0x20000010u)
#define ILA_LIVE     (*(volatile uint32_t *)0x20000014u)

#define ILA_DEPTH    256u
#define ILA_DUMP_N    48u    /* 只打印触发点附近 48 拍：够画波形，又不至于刷 60 s
                              * （35 B/s 的硬上限下，96 拍要 33 s，48 拍 15 s） */
#endif /* WITH_ILA */

/* bit0 → LED1（CPU 心跳）  bit1 → LED2  bit2 → LED3 */
static void led_set(uint32_t v)
{
    GPIO_OUT = v & 7u;
}

/* ---- 由别的编译单元提供 ---- */
extern int      coremark_main(void);        /* core_main.c，用 -Dmain=coremark_main 编进来 */
extern int      gate_app_run(int quick);    /* gate_board.c，quick=1 跳过启动保活循环 */
extern uint32_t coremark_last_cycles(void); /* core_portme.c，跑分区间的真实周期数 */
extern const char *coremark_flags_str(void); /* core_portme.c，官方行要的 compiler+flags */

#ifndef ITERATIONS
#define ITERATIONS 6000
#endif

/* 小工具：无符号十进制输出（不依赖 printf，省代码） */
static void pu32(uint32_t v)
{
    char b[11];
    int  n = 0;
    if (v == 0u) { uart_putc('0'); return; }
    while (v) { b[n++] = (char)('0' + (v % 10u)); v /= 10u; }
    while (n) uart_putc(b[--n]);
}

/* 官方 CoreMark 1.0 汇总行 + CoreMark/MHz。
 *
 * 为什么需要在这里补：HAS_FLOAT=0 时 core_main.c 里
 *     CoreMark 1.0 : <iter/s> / <compiler> <flags> / static
 * 整段包在 `#if HAS_FLOAT` 里，根本不会打印（格式用了 %f）。
 * 开 HAS_FLOAT 需要软浮点，会把镜像撑超 64 KB 的栈余量（已实测失败）。
 * ⇒ 这里用 64 位整数按**官方格式**补一行，compiler/flags 取自
 *   core_portme.c 的 coremark_flags_str()，与真实编译选项同源。
 *
 * 上一版的两个错，一并修掉：
 *   ① 分隔符用了 ':'，官方是 '/'；
 *   ② 分数写死取整（525），官方是带 3 位小数的 525.990。 */
static void print_score(void)
{
    uint32_t cyc = coremark_last_cycles();
    uint64_t milli, ips_x1000;
    uint32_t frac;

    if (cyc == 0u) return;

    /* milli    = iterations × 1e9 / cycles      ⇒ CoreMark/MHz × 1000 */
    milli     = ((uint64_t)ITERATIONS * 1000000000ull) / (uint64_t)cyc;
    /* ips_x1000= iterations × 101.25e6 × 1000 / cycles ⇒ (iter/s) × 1000 */
    ips_x1000 = ((uint64_t)ITERATIONS * 101250000000ull) / (uint64_t)cyc;

    uart_puts("CoreMark 1.0 : ");
    pu32((uint32_t)(ips_x1000 / 1000u));
    uart_putc('.');
    frac = (uint32_t)(ips_x1000 % 1000u);
    uart_putc((char)('0' + (frac / 100u)));
    uart_putc((char)('0' + ((frac / 10u) % 10u)));
    uart_putc((char)('0' + (frac % 10u)));
    uart_puts(" / ");
    uart_puts(coremark_flags_str());
    uart_puts(" / static\r\n");

    uart_puts("[CM] CM/MHz=");
    pu32((uint32_t)(milli / 1000u));
    uart_putc('.');
    frac = (uint32_t)(milli % 1000u);
    uart_putc((char)('0' + (frac / 100u)));
    uart_putc((char)('0' + ((frac / 10u) % 10u)));
    uart_putc((char)('0' + (frac % 10u)));
    uart_puts("\r\n");
}

/* ---- 串口节奏档位：{段间静默 ms, 段内每字节 ms} ----
 * 默认 600/0：一行 22 字符用 1.9 ms 冲完，然后静默 600 ms（> 平台 500 ms 轮询）
 * ⇒ 每次轮询都拿到完整的一整行。
 * 按 3 **先往更保守走**（段内加 1 ms），因为如果默认档还掉字节，
 * 说明接收端在按字节丢，必须当场能救 —— 否则只能回来改代码重跑一遍 45 分钟的实现。 */
static const uint32_t s_pace_tab[][2] = {
    { 600u, 0u },   /* 0 ★默认 */
    { 600u, 1u },   /* 1 段内慢发 */
    { 800u, 1u },   /* 2 最保守 */
    { 400u, 0u },   /* 3 快一点 */
    { 200u, 0u },   /* 4 本机录屏 */
};
static int s_pace_idx = 0;

static void print_gap(void)
{
    pu32(g_uart_gap_ms);
    uart_puts("ms b");
    pu32(g_uart_intra_ms);
}

/* 菜单压成三行、每行都不超过 22 字符（一段 = 一次 RX），
 * 中文全部去掉 —— 一个汉字 3 字节，被批次边界切开会显示成 �。 */
static void print_menu(void)
{
    uart_puts("\r\nFaceRec SoC\r\n");
    uart_puts("1=CoreMark 2=Gate\r\n");
#if WITH_ILA
    uart_puts("3=UART 4=ILA ");
#else
    uart_puts("3=UART ");
#endif
    pu32(g_uart_gap_ms);
    uart_puts("\r\n> ");
    uart_flush();                        /* 提示符不带换行，得手动发出去 */
}

static int rx_get(void)
{
    if (UART_RXSTAT & 1u)
        return (int)(UART_RXDATA & 0xFFu);
    return -1;
}

#if WITH_ILA
/* =====================================================================
 * ILA：抓一段真实的流水线波形并回传
 *
 * 触发源用「分支误预测」而不是 PC 匹配，理由：
 *   误预测是流水线效率的直接杀手（冲刷 + 重新取指），
 *   而且它天然发生在**循环稳态中**，不像函数入口那样只能抓到前导码。
 *
 * 采样窗口 256 拍（2.53 µs @101.25 MHz），停下时缓冲区内是
 *   触发点前 192 拍 + 触发点后 64 拍
 * ——「前因」比「后果」更值钱，所以前段留得多。
 * ===================================================================== */

/* 定点循环：load + ALU + store + 一个**数据相关的不规则分支**
 * 目的是一次把 load-use 冒险、分支解析、误预测、双发射配对都制造出来，
 * 这样一屏波形里能看到全部机制。
 * noinline 是为了让地址稳定（PC 触发模式会用到）。 */
static volatile uint32_t s_buf[16];
static volatile uint32_t s_acc;

__attribute__((noinline)) static void bench_loop(uint32_t n)
{
    uint32_t i, acc = 0x1234u;

    for (i = 0; i < n; i++) {
        uint32_t v = s_buf[i & 15u];
        acc ^= (v + i);
        /* 位号随迭代变化 ⇒ BHT 追不上，必然误预测 */
        if (i & (1u << ((i >> 4) & 3u)))
            acc += 3u;
        s_buf[(i * 7u) & 15u] = acc;
    }
    s_acc = acc;
}

/* 打印 nd 位十六进制 */
static void ph(uint32_t v, int nd)
{
    static const char H[] = "0123456789abcdef";
    int sh;
    for (sh = (nd - 1) * 4; sh >= 0; sh -= 4)
        uart_putc(H[(v >> sh) & 0xFu]);
}

/* 每拍归一个类，日志里人眼直接能读 */
static char ila_cls(uint32_t s)
{
    if (s & (1u << 30)) return 'M';      /* 误预测 */
    if (s & (1u << 21)) return 'L';      /* load-use */
    if (s & (1u << 19)) return 'D';      /* 数据口等待 */
    if (s & (1u << 18)) return 'F';      /* 取指未就绪 */
    if (s & (1u << 28)) return '2';      /* 一拍退休两条 */
    if (s & (1u << 17)) return 'R';      /* 退休一条 */
    return '.';
}

static void ila_run(void)
{
    uint32_t i, k = 0, got = 0u, t0, st;
    uint32_t retire = 0, fnr = 0, dw = 0, lu = 0, mis = 0, r2 = 0, pair = 0;
    uint32_t start, ins;

    uart_puts("[ILA] arm\r\ntrig=mispredict\r\n");
    ILA_CTRL = 0x5u;                     /* CTRL[2]=1 误预测触发 + CTRL[0]=1 布防 */

    /* 一边跑定点循环一边等触发（循环本身就在制造误预测） */
    t0 = *(volatile uint32_t *)0x30000000u;
    do {
        bench_loop(200u);
        st = ILA_CTRL;
    } while (!(st & 0x2u) &&
             (uint32_t)(*(volatile uint32_t *)0x30000000u - t0) < 10125000u);

    if (!(st & 0x2u)) {
        uart_puts("[ILA] no trigger\r\nsoft fallback\r\n");
        ILA_CTRL = 0x2u;                 /* 只置 soft：0x2 不会重新布防 */
        st = ILA_CTRL;
    }

    /* 第一遍：定位触发点 + 统计整个 256 拍窗口 */
    for (i = 0; i < ILA_DEPTH; i++) {
        uint32_t s;
        ILA_IDX = i;
        s = ILA_IDX;
        if (s & 0x80000000u) { k = i; got = 1u; }
        if (s & (1u << 17)) retire++;
        if (s & (1u << 18)) fnr++;
        if (s & (1u << 19)) dw++;
        if (s & (1u << 21)) lu++;
        if (s & (1u << 30)) mis++;
        if (s & (1u << 28)) r2++;
        if (s & (1u << 27)) pair++;
    }

    /* 指令数 = 有退休的拍 + 双退休的拍（retire2 那拍本身也算一条） */
    ins = retire + r2;

    /* 统计行：每行 ≤ 22 字符 —— 一次 RX 正好一整行（原理见 uart_out.h） */
    uart_puts("[ILA] win=256 trig=");
    ph(k, 2);
    if (!got) uart_putc('!');
    uart_puts("\r\nretire=");   ph(retire, 3);
    uart_puts(" retire2=");     ph(r2, 3);
    uart_puts("\r\nifstall=");  ph(fnr, 3);
    uart_puts(" dwait=");       ph(dw, 3);
    uart_puts("\r\nloaduse=");  ph(lu, 3);
    uart_puts(" pair=");        ph(pair, 3);
    uart_puts("\r\nmispred=");  ph(mis, 3);
    uart_puts(" ins=");         ph(ins, 3);
    uart_puts("\r\nIPC=");
    ph(ins / 256u, 1);
    uart_putc('.');
    ph((ins % 256u) * 1000u / 256u, 3);
    uart_puts("\r\n");

    /* 第二遍：打印触发点附近 ILA_DUMP_N 拍。
       **两个采样拼一行**（"xxxxxxxx M xxxxxxxx M" = 21 字符）：
       10 字符一行的话 48 行要 30 s，拼成 21 字符一行直接砍半到 15 s。 */
    start = (k + ILA_DEPTH - (ILA_DUMP_N / 2u)) % ILA_DEPTH;
    uart_puts("[ILA] hex pc|flg x2\r\n");
    for (i = 0; i < ILA_DUMP_N; i += 2u) {
        int j;
        for (j = 0; j < 2; j++) {
            uint32_t idx = (start + i + (uint32_t)j) % ILA_DEPTH;
            uint32_t s;
            ILA_IDX = idx;
            s = ILA_IDX;
            if (j) uart_putc(' ');
            ph(s, 8);
            uart_putc(' ');
            uart_putc(ila_cls(s));
        }
        uart_puts("\r\n");
    }

    /* 收尾信号：抓到了就把 LED3 点亮，肉眼可判 */
    led_set(2u | (got ? 4u : 0u));
    uart_delay_ms(300u);
    led_set(2u);
}
#endif /* WITH_ILA */

int main(void)
{
    uint32_t hb_t0;
    int      hb = 0;

    /* 关掉 RTL 连续信标：菜单要用串口，不能和 'U' 抢线 */
    GPIO_BCN_OFF = 1u;
    uart_delay_ms(2u);

    /* LED2 常亮 = 固件已在菜单（与门禁运行态区分） */
    led_set(2u);

    /* 等平台终端接上再开口。
       2026-11-02 实测：上一版一复位就发菜单，平台那边还没开始读，
       第一行被啃掉半句（日志里第一行只剩 "SoC | 1=CoreMark 2=Gate 3"，
       开头的 "FaceRec " 丢了）。1.5 s 足够串口工具把读循环挂上。 */
    uart_delay_ms(1500u);

    print_menu();

    hb_t0 = *(volatile uint32_t *)0x30000000u;

    for (;;) {
        int c = rx_get();

        if (c >= 0) {
            /* 回显：确认按键真的收到了（4 字节） */
            uart_puts("[K]");
            uart_putc((char)c);
            uart_puts("\r\n");

            if (c == '1') {
                int rc;
                led_set(1u);                    /* LED2 灭，表示在跑分 */
                uart_puts("[RUN] CoreMark\r\n");
                uart_puts("6000 iter ~12s\r\n");
                rc = coremark_main();
                if (rc != 0) {
                    uart_puts("[RUN] rc=");
                    uart_putc((char)('0' + (rc & 7)));
                    uart_puts("\r\n");
                }
                print_score();
                led_set(2u);
                print_menu();
            } else if (c == '2') {
                uart_puts("[RUN] FaceRec\r\n");
                /* 不返回：内部是无限出帧循环，按复位键回菜单 */
                gate_app_run(1);
            } else if (c == '3') {
                /* 节奏循环切换：{段间静默, 段内每字节} ms。
                   第一次按是**段内慢发**（600/1），因为要救"按字节丢"的场。 */
                s_pace_idx = (s_pace_idx + 1) %
                             (int)(sizeof(s_pace_tab) / sizeof(s_pace_tab[0]));
                uart_set_pace(s_pace_tab[s_pace_idx][0], s_pace_tab[s_pace_idx][1]);
                uart_puts("[UART] gap=");
                print_gap();
                uart_puts("\r\n");
                print_menu();
#if WITH_ILA
            } else if (c == '4') {
                uart_puts("[RUN] ILA capture\r\n");
                ila_run();
                print_menu();
#endif
            } else if (c == '\r' || c == '\n' || c == ' ') {
                /* 回车/空格不做事 */
            } else {
#if WITH_ILA
                uart_puts("[K] use 1/2/3/4\r\n");
#else
                uart_puts("[K] use 1/2/3\r\n");
#endif
            }
            hb_t0 = *(volatile uint32_t *)0x30000000u;
        }

        /* 空闲心跳：LED1 每 500 ms 翻转一次，证明 CPU 还活着 */
        {
            uint32_t now = *(volatile uint32_t *)0x30000000u;
            if ((uint32_t)(now - hb_t0) >= 50625000u) {   /* 0.5 s @101.25MHz */
                hb_t0 = now;
                hb ^= 1;
                led_set(2u | (uint32_t)hb);
            }
        }
    }
}
