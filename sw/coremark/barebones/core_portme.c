/*
Copyright 2018 Embedded Microprocessor Benchmark Consortium (EEMBC)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

Original Author: Shay Gal-on
*/
/* ==========================================================================
 * core_portme.c : CoreMark v1.0 -> RV32I 裸机 SoC 移植 (计时 / 初始化)
 *
 * - barebones_clock(): 读 SoC 硬件周期计数器 (0x3000_0000, 64bit,
 *   读低半锁存高半), tick = CPU cycle, 真实计时 (跑分用).
 *   32bit 差值要求单次计时 < 2^32/SYSCLK_HZ ≈ 159s, 足够.
 * - portable_init(): 启动自检 + 打印横幅.
 * - portable_fini(): 把 "完成标志 0xCAFE" 写回 RESULT_BASE, 供 testbench 判定跑完.
 * ========================================================================== */
#include "coremark.h"
#include "core_portme.h"

#if VALIDATION_RUN
volatile ee_s32 seed1_volatile = 0x3415;
volatile ee_s32 seed2_volatile = 0x3415;
volatile ee_s32 seed3_volatile = 0x66;
#endif
#if PERFORMANCE_RUN
volatile ee_s32 seed1_volatile = 0x0;
volatile ee_s32 seed2_volatile = 0x0;
volatile ee_s32 seed3_volatile = 0x66;
#endif
#if PROFILE_RUN
volatile ee_s32 seed1_volatile = 0x8;
volatile ee_s32 seed2_volatile = 0x8;
volatile ee_s32 seed3_volatile = 0x8;
#endif

/* 固定迭代次数 (SEED_METHOD=SEED_VOLATILE 时, main 取 seed4_volatile 作为 iterations).
 * 必须非零, 以跳过 core_main 的自动校准循环. */
volatile ee_s32 seed4_volatile = ITERATIONS;
volatile ee_s32 seed5_volatile = 0;
#if COREMARK_DUAL
static unsigned int coremark_run_index;
#endif

/* --------------------------------------------------------------------------
 * 计时: 软件计数器
 * -------------------------------------------------------------------------- */
#define TCK_PER_SEC  SYSCLK_HZ /* tick = 1 个 CPU cycle */

/* SoC 硬件周期计数器 (rtl/soc/soc_top.v): 自由运行, 读 0x...00 返回低 32 位
 * 并把当时的 63:32 快照进影子寄存器, 读 0x...04 取回快照. */
#define TIMER_BASE   0x30000000u
static volatile ee_u32 *const timer_lo = (volatile ee_u32 *)TIMER_BASE;
static volatile ee_u32 *const timer_hi = (volatile ee_u32 *)(TIMER_BASE + 4);

CORETIMETYPE
barebones_clock(void)
{
    (void)timer_hi;             /* 先读高 (返回上次低半读取时的快照, 此处丢弃) */
    return *timer_lo;           /* 读低半, 同时锁存新高半快照 */
}

#define GETMYTIME(_t)              (*_t = barebones_clock())
#define MYTIMEDIFF(fin, ini)       ((fin) - (ini))
#define TIMER_RES_DIVIDER          1
#define SAMPLE_TIME_IMPLEMENTATION 1
#define EE_TICKS_PER_SEC           (TCK_PER_SEC / TIMER_RES_DIVIDER)

static CORETIMETYPE start_time_val, stop_time_val;

void
start_time(void)
{
    /* ★★★ S58 (2026-10-21, Astra M0): **ROI 对齐清零**。
     *
     * 问题: pm_cnt[*] 从 resetn 起一直累计, 而 Total cycles 读的是
     *   cyc_cnt 的 (stop - start) —— 只覆盖迭代区间。两个口径不同, 于是
     *   `Total instructions` 含 crt0/初始化, IPC 虚高; tb 面板里的
     *   pair_ok/load_use/规则拒绝等事件率也全部被 343,976 拍的全仿真基数
     *   污染 (被误当成 263,108 拍的占比)。
     *
     * 修法: 0x3000_003C 是 timer 区内的空闲**只写**地址, 硬件在这一拍把
     *   所有 pm_cnt 清零 ⇒ 事件计数与 cyc_cnt 的 start 严格同点。
     *   cyc_cnt 本身不清零 (时间戳要用绝对差值)。
     */
    *(volatile unsigned int *)0x3000003Cu = 1u;
    GETMYTIME(&start_time_val);
}

void
stop_time(void)
{
    /* ★★★ S58: **先冻结 pm_cnt，再读时间戳**。
     * ROI 到此结束，但 CoreMark 之后还要跑 perf_report 的 ee_printf
     * （实测约 24.6k 条指令 / 64k 拍）—— 不冻结的话所有事件计数会被它污染，
     * 我们又回到"分子含收尾、分母只有 ROI"的老问题上。 */
    *(volatile unsigned int *)0x30000038u = 1u;
    GETMYTIME(&stop_time_val);
}

CORE_TICKS
get_time(void)
{
    CORE_TICKS elapsed = (CORE_TICKS)(MYTIMEDIFF(stop_time_val, start_time_val));
    return elapsed;
}

/* 给菜单固件用：把本次跑分区间内的真实周期数交出去。
 * HAS_FLOAT=0 时 core_main 不会打印官方的
 *   "CoreMark 1.0 : <iterations/sec> : <compiler> : <flags>"
 * 那一行（它包在 #if HAS_FLOAT 里），所以由调用方用整数运算补一条
 * 等价的汇总行，保证报名材料里能直接引用 CoreMark/MHz。 */
ee_u32
coremark_last_cycles(void)
{
    return (ee_u32)MYTIMEDIFF(stop_time_val, start_time_val);
}

/* 给菜单固件用：官方 "CoreMark 1.0 : …" 那一行需要的 `<compiler> <flags>` 串。
 *
 * HAS_FLOAT=0 时 core_main.c 不会打印那一行（整段包在 #if HAS_FLOAT 里，
 * 因为格式是 %f）。由调用方拿这个串按官方格式自己补一行：
 *     CoreMark 1.0 : <iterations/sec> / <compiler> <flags> / static
 * 这样 COMPILER_VERSION / COMPILER_FLAGS 只有一处定义，不会两处对不上。 */
const char *
coremark_flags_str(void)
{
    return COMPILER_VERSION " " COMPILER_FLAGS;
}

secs_ret
time_in_secs(CORE_TICKS ticks)
{
    /* 真实换算: cycles / SYSCLK_HZ = 秒 (core_main 要求 >=10s) */
    secs_ret retval = ((secs_ret)ticks) / (secs_ret)EE_TICKS_PER_SEC;
    return retval;
}

ee_u32 default_num_contexts = 1;

/* --------------------------------------------------------------------------
 * 完成标志: 跑完写入 RAM, testbench 据此判定执行结束
 * -------------------------------------------------------------------------- */
#ifndef WITH_MENU
#define RESULT_BASE 0x00001000
static volatile unsigned int *done = (volatile unsigned int *)(RESULT_BASE);
#endif

/* --------------------------------------------------------------------------
 * 平台初始化
 * -------------------------------------------------------------------------- */
void
portable_init(core_portable *p, int *argc, char *argv[])
{
    (void)argc;
    (void)argv;

    /* 启动自检: 指针宽度应为 4 字节 */
    if (sizeof(ee_ptr_int) != sizeof(ee_u8 *))
    {
        ee_printf("ERROR: ee_ptr_int size mismatch! (%lu vs %lu)\n",
                  (unsigned long)sizeof(ee_ptr_int),
                  (unsigned long)sizeof(ee_u8 *));
    }
    if (sizeof(ee_u32) != 4)
    {
        ee_printf("ERROR: ee_u32 size mismatch! (%lu bytes)\n",
                  (unsigned long)sizeof(ee_u32));
    }

#if COREMARK_DUAL
    seed1_volatile = coremark_run_index ? 0x3415 : 0;
    seed2_volatile = coremark_run_index ? 0x3415 : 0;
    seed3_volatile = 0x66;
    ee_printf("RV32I CoreMark %s start\n", coremark_run_index ? "validation" : "performance");
    coremark_run_index++;
#else
    ee_printf("RV32I CoreMark validation start\n");
#endif
    p->portable_id = 1;
}

/* --------------------------------------------------------------------------
 * 平台收尾: 写完成标志 + 性能计数器报表
 * -------------------------------------------------------------------------- */
void
portable_fini(core_portable *p)
{
    /* ★ 性能计数器报表 (2026-10-10): 经 SoC 扩展读窗读 pm_cnt，全部在计时区
     *   之外读取，不影响跑分。计数器语义见 rtl/soc/soc_top.v pm_cnt 注释。
     *   地址映射: 0x3000_0000 页 +08/+0C = pm_cnt[2]/[3] (原窗口),
     *             +80..+FC = pm_cnt[16..46] (扩展窗, d_addr[6:2] 索引)。 */
    {
        unsigned int cycles     = (unsigned int)(stop_time_val - start_time_val);
        unsigned int retire_pos = *(volatile unsigned int *)0x30000008u; /* pm_cnt[2]  退休正拍     */
        unsigned int dual2      = *(volatile unsigned int *)0x300000C8u; /* pm_cnt[34] 双退休拍     */
        unsigned int instret    = retire_pos + dual2;                    /* 真实指令数              */
        unsigned int ic_miss    = *(volatile unsigned int *)0x30000088u; /* pm_cnt[18] I$ miss 次数 */
        unsigned int dc_miss    = *(volatile unsigned int *)0x300000ECu; /* pm_cnt[43] D$ 真 miss   */
        unsigned int dc_fast    = *(volatile unsigned int *)0x300000F4u; /* pm_cnt[45] D$ 快路命中  */
        unsigned int dc_slow    = *(volatile unsigned int *)0x300000F8u; /* pm_cnt[46] D$ 慢路命中  */
        unsigned int br_cnt     = *(volatile unsigned int *)0x30000028u; /* pm_cnt[10] 分支解决     */
        unsigned int br_miss    = *(volatile unsigned int *)0x30000030u; /* pm_cnt[12] 分支误判修正 */
        unsigned int fe_stall   = *(volatile unsigned int *)0x3000000Cu; /* pm_cnt[3]  取指未就绪拍 */
        unsigned int dc_acc     = dc_fast + dc_slow + dc_miss;

        ee_printf("\n===== Perf counters =====\n");
        ee_printf("Total cycles: %u\n", cycles);
        ee_printf("Total instructions: %u\n", instret);
        /* Derive ratios on the host from these raw counters. With 10000
         * iterations, multiplying a 32-bit count by 1000/100000 overflows.
         * Keep benchmark reporting free of extra 64-bit runtime routines. */
        ee_printf("I-Cache Miss: %u\n", ic_miss);
        ee_printf("D-Cache Miss: %u\n", dc_miss);
        ee_printf("D-Cache Access: %u\n", dc_acc);
        ee_printf("Branch count: %u\n", br_cnt);
        ee_printf("Branch miss: %u\n", br_miss);
        ee_printf("Stalled frontend cycles: %u\n", fe_stall);
    }

#if COREMARK_DUAL
    ee_printf("RV32I CoreMark phase done\n");
    if (coremark_run_index < 2) {
        p->portable_id = 0;
        return;
    }
#else
    ee_printf("RV32I CoreMark validation done\n");
#endif
#ifndef WITH_MENU
    *(volatile unsigned int *)0x20000000 = 0xFF; /* GPIO=0xFF -> LED 常亮, 板级可见完成标志 */
    done[0] = 0xCAFE;   /* 完成标志 -> testbench 读取判定 */
#endif
    /* WITH_MENU（菜单固件）：不写 0x1000 的完成标志 —— 那个地址落在
     * 合并镜像的 .text 里，会把指令改掉（独立跑分包才用得上它），
     * 也不动 GPIO，LED 状态留给菜单自己管。 */
    p->portable_id = 0;
}
