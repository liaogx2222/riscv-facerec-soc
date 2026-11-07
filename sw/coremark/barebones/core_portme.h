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
 * core_portme.h : CoreMark v1.0 -> RV32I 裸机 SoC 移植
 *
 * 目标平台: EEMBC CoreMark v1.0 裸机 (RV32I), 跑在本竞赛 SoC:
 *   RAM  @ 0x0000_0000 ~ 0x0001_FFFF (128KB)
 *   UART @ 0x1000_0000  (+0 TXDATA, +4 STATUS bit0=tx_busy)
 *   GPIO @ 0x2000_0000
 *
 * 关键决策:
 *   - HAS_FLOAT 0:  RV32I 纯整型, CoreMark 算法全是整数, 关浮点避免 %f.
 *   - MEM_STATIC:   用静态数组 (TOTAL_DATA_SIZE 字节), 不用 malloc/大栈.
 *   - SEED_VOLATILE + 固定 ITERATIONS: 跳过自动校准循环, 保证跑完.
 *   - 时钟为软件计数器 (非真实计时), 用于功能性验证, 报告 ticks 仅作展示.
 * ========================================================================== */

#ifndef CORE_PORTME_H
#define CORE_PORTME_H

/************************/
/* Data types and settings */
/************************/

/* HAS_FLOAT = 0
 *
 * 曾经想开成 1，让官方自己打印 `Total time (secs): 11.407` 那种小数。
 * 实测不行：`%f` 的变参是 double，`v * 1000.0` 会拉进 __muldf3/__adddf3/
 * __fixunsdfsi 一整套软浮点，镜像撑爆 64 KB 里留给栈的余量（链接脚本断言
 * "Firmware leaves less than 8KB stack" 直接失败）。
 *
 * 改为：keep HAS_FLOAT=0，但在 core_main.c 的**整数分支**里用 64 位整数把
 * 3 位小数补回来（原来那里直接 %d 打印整数秒，把 11.407 截成 11，
 * Iterations/Sec 因此虚高 3.6%）。见 core_main.c 的 "Total time (secs)" 处。
 *
 * ee_printf.c 里的 %f 实现保留（用 #if HAS_FLOAT 包着，此配置下不参与编译），
 * 换到有 FPU 的目标上把这里改回 1 即可直接用。 */
#define HAS_FLOAT       0
#define HAS_TIME_H      0
#define USE_CLOCK       0
#define HAS_STDIO       0
#define HAS_PRINTF      0

/* Total data size = 2000 -> 2K validation run (seedcrc=0x18f2, known_id=4).
 * seeds 0x3415/0x3415/0x66 + size=2000/3=666/算法, 匹配官方已知 CRC 表:
 *   list=0xe3c1 matrix=0x0747 state=0x8d84
 * 若未通过命令行 -DTOTAL_DATA_SIZE 指定, 这里给默认, 让 seedcrc 命中已知集合. */
#ifndef TOTAL_DATA_SIZE
#define TOTAL_DATA_SIZE 2000
#endif

/* 固定迭代次数: 避免 core_main 的自动校准循环 (iterations==0 时无限循环).
 * crclist/crcmatrix/crcstate 均在 i==0 记录, 与迭代次数无关, 所以功能验证
 * 用 ITERATIONS=1 即可判对错; 上板测 CoreMark/MHz 时改回 10000. */
#ifndef ITERATIONS
#define ITERATIONS 1
#endif

/* CPU 主频 Hz: 必须与 fpga_top 的 CLK_HZ 一致 (板 27MHz 晶振).
 * 供 core_portme.c 硬件计时器换算 tick->秒. */
#ifndef SYSCLK_HZ
#define SYSCLK_HZ 27000000
#endif

/* tick 频率（Hz）—— core_main.c 的整数分支用它把 Total time 的小数补全。
 * 必须与 core_portme.c 里的 EE_TICKS_PER_SEC 相同：
 *     EE_TICKS_PER_SEC = TCK_PER_SEC / TIMER_RES_DIVIDER
 *     TCK_PER_SEC      = SYSCLK_HZ
 *     TIMER_RES_DIVIDER= 1
 * ⇒ 一个 tick 就是一个 CPU 周期。改 TIMER_RES_DIVIDER 时这里要同步。 */
#ifndef EE_TICKS_PER_HZ
#define EE_TICKS_PER_HZ  (SYSCLK_HZ)
#endif

#ifndef COMPILER_VERSION
#ifdef __GNUC__
#define COMPILER_VERSION "GCC"__VERSION__
#else
#define COMPILER_VERSION "unknown"
#endif
#endif
#ifndef COMPILER_FLAGS
/* ★ 必须与 tools/build_menu_fw.py 里的 COMMON + CM_OPT 逐项一致。
 *   原来这里硬编码的是 `-O2`，而实际编译用的是
 *   `-O3 -funroll-loops -funroll-all-loops -fpeel-loops -falign-*=4`
 *   —— 官方输出与真实编译选项对不上，评委对着看就是硬伤。 */
#define FLAGS_STR ""
#ifdef CM_RVC
#define COMPILER_FLAGS \
    "-march=rv32ic_zmmul -mno-div -mabi=ilp32 -O3 -funroll-loops -funroll-all-loops -fpeel-loops -falign-functions=4 -falign-loops=4 -fno-common -ffreestanding -DVALIDATION_RUN=1"
#else
#define COMPILER_FLAGS \
    "-march=rv32i_zmmul -mno-div -mabi=ilp32 -O3 -funroll-loops -funroll-all-loops -fpeel-loops -falign-functions=4 -falign-loops=4 -fno-common -ffreestanding -DVALIDATION_RUN=1"
#endif
#endif
#ifndef MEM_LOCATION
#define MEM_LOCATION "Static"
#endif

/* Data types */
typedef signed short   ee_s16;
typedef unsigned short ee_u16;
typedef signed int     ee_s32;
typedef double         ee_f32;
typedef unsigned char  ee_u8;
typedef unsigned int   ee_u32;
typedef ee_u32         ee_ptr_int;   /* RV32I 指针 = 32 位 */
typedef unsigned int   ee_size_t;
#define NULL ((void *)0)

#define align_mem(x) (void *)(4 + (((ee_ptr_int)(x) - 1) & ~3))

/* Timing type */
#define CORETIMETYPE ee_u32
typedef ee_u32 CORE_TICKS;

/* 种子来源: 用 volatile 变量 (SEED_VOLATILE). ITERATIONS 由本文件固定. */
#ifndef SEED_METHOD
#define SEED_METHOD SEED_VOLATILE
#endif

/* 内存分配方式: 静态数组 */
#ifndef MEM_METHOD
#define MEM_METHOD MEM_STATIC
#endif

#ifndef MULTITHREAD
#define MULTITHREAD 1
#define USE_PTHREAD 0
#define USE_FORK    0
#define USE_SOCKET  0
#endif

#ifndef MAIN_HAS_NOARGC
#define MAIN_HAS_NOARGC 1
#endif

#ifndef MAIN_HAS_NORETURN
#define MAIN_HAS_NORETURN 0
#endif

extern ee_u32 default_num_contexts;

typedef struct CORE_PORTABLE_S
{
    ee_u8 portable_id;
} core_portable;

void portable_init(core_portable *p, int *argc, char *argv[]);
void portable_fini(core_portable *p);

#if !defined(PROFILE_RUN) && !defined(PERFORMANCE_RUN) \
    && !defined(VALIDATION_RUN)
#if (TOTAL_DATA_SIZE == 1200)
#define VALIDATION_RUN 1
#elif (TOTAL_DATA_SIZE == 2000)
#define PERFORMANCE_RUN 1
#else
#define VALIDATION_RUN 1
#endif
#endif

int ee_printf(const char *fmt, ...);

#endif /* CORE_PORTME_H */
