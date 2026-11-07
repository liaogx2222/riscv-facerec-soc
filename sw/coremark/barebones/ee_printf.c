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
 * ee_printf.c : CoreMark v1.0 -> RV32I 裸机 (经 UART 输出)
 *
 * 本 SoC 无标准库, 故自实现最小 printf.
 * 支持 CoreMark VALIDATION 报告所需的全部格式: %s %d %u %lu %x/%X
 * (含宽度/补零, 如 %04x), %c, %f(定点小数), \n -> \r\n.
 * ========================================================================== */
#include <coremark.h>
#include <stdarg.h>
#include "uart_out.h"

#define ZEROPAD   (1 << 0)
#define SIGN      (1 << 1)
#define PLUS      (1 << 2)
#define SPACE      (1 << 3)
#define LEFT      (1 << 4)
#define UPPERCASE (1 << 5)

#define is_digit(c) ((c) >= '0' && (c) <= '9')

static char digits[]       = "0123456789abcdefghijklmnopqrstuvwxyz";
static char upper_digits[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";

static void uart_send_char(char c)
{
    /* 统一走 sw/uart_out.c：
     *   - busy 轮询有上限（老代码是 `while (*status & 1);` 死等，
     *     万一 busy 位读回恒 1 就永远卡在这里，现象是"串口一个字都没有"）
     *   - 每 8 个字符停 2 ms、行末再停 2 ms，避免平台侧接收缓冲丢字节
     * CoreMark 的打印都在 start_time()/stop_time() 之外，节流不影响跑分。 */
    uart_putc(c);
}

static int skip_atoi(const char **s)
{
    int i = 0;
    while (is_digit(**s))
        i = i * 10 + *((*s)++) - '0';
    return i;
}

static int strnlen_c(const char *s, int count)
{
    int i;
    for (i = 0; *s != '\0' && i < count; ++s, ++i) ;
    return i;
}

/* 把 num 按 base 转成字符, 写进 *str, 返回结束指针 */
static char *
fmt_number(char *str, unsigned long num, int base, int width, int prec, int flags)
{
    char tmp[32];
    char *dig = (flags & UPPERCASE) ? upper_digits : digits;
    int i = 0, sign = 0, neg = 0;

    if (flags & SIGN)
    {
        if ((long)num < 0)
        {
            neg = 1;
            num = -(long)num;
            width--;
        }
    }

    if (num == 0)
        tmp[i++] = '0';
    else
    {
        while (num != 0)
        {
            tmp[i++] = dig[num % (unsigned)base];
            num = num / (unsigned)base;
        }
    }

    if (i > prec)
        prec = i;

    int fill = width - prec;

    if (flags & SIGN)
    {
        if (neg) { sign = '-'; fill--; }
        else if (flags & PLUS) { sign = '+'; fill--; }
        else if (flags & SPACE) { sign = ' '; fill--; }
    }
    if (fill < 0) fill = 0;

    char pad = (flags & ZEROPAD) ? '0' : ' ';

    if (!(flags & LEFT))
    {
        if (sign) *str++ = sign;
        while (fill-- > 0) *str++ = pad;
    }
    else
    {
        if (sign) *str++ = sign;
    }

    while (i < prec--) *str++ = '0';
    while (i-- > 0) *str++ = tmp[i];
    while (fill-- > 0) *str++ = ' ';

    return str;
}

/* --------------------------------------------------------------------------
 * %f —— 定点小数（默认 3 位）
 *
 * 现状：HAS_FLOAT=0，整段不参与编译。
 *   本工程的 tick→秒换算在 core_main.c 的整数分支里用 64 位整数补 3 位小数
 *   （见那里的 2026-11-01 注释）。为什么不靠 %f：`v * 1000.0` 会拉进
 *   __muldf3 / __adddf3 / __fixunsdfsi 一整套软浮点，镜像撑爆 64 KB 里
 *   留给栈的余量，链接脚本断言 "Firmware leaves less than 8KB stack" 直接失败。
 *   换到 FPU 目标或 RAM 宽裕的板子，把 core_portme.h 的 HAS_FLOAT 改回 1 即可启用。
 *
 * 实现取巧处
 *   只做一次 `v * 10^prec + 0.5`（四舍五入）再强转成 unsigned long，
 *   剩下全是整数取模输出 —— 比完整 %f 省一半软浮点。
 * -------------------------------------------------------------------------- */
#if HAS_FLOAT
static char *
fmt_float(char *str, double v, int prec)
{
    unsigned long div = 1, scaled, ip, fp;
    char tmp[24];
    int n = 0, i;

    if (prec < 0) prec = 3;
    if (prec > 6) prec = 6;
    for (i = 0; i < prec; i++) div *= 10u;

    if (v < 0.0) { *str++ = '-'; v = -v; }

    scaled = (unsigned long)(v * (double)div + 0.5);   /* 四舍五入到 10^-prec */
    ip = scaled / div;
    fp = scaled % div;

    if (ip == 0) tmp[n++] = '0';
    while (ip) { tmp[n++] = (char)('0' + (int)(ip % 10u)); ip /= 10u; }
    while (n) *str++ = tmp[--n];

    if (prec > 0)
    {
        *str++ = '.';
        while (div > 1u)
        {
            div /= 10u;
            *str++ = (char)('0' + (int)((fp / div) % 10u));
        }
    }
    return str;
}
#endif  /* HAS_FLOAT */

int
ee_printf(const char *fmt, ...)
{
    char buf[256];
    char *str = buf;
    const char *f = fmt;
    va_list args;

    va_start(args, fmt);

    while (*f)
    {
        if (*f != '%')
        {
            *str++ = *f++;
            if (str - buf >= 250)
            {
                char *_p = buf;
                while (_p < str) uart_send_char(*_p++);
                str = buf;
            }
            continue;
        }

        int flags = 0, width = -1, prec = -1, base = 10;
        char qual = 0;
        f++;

    repeat_flags:
        switch (*f)
        {
        case '0': flags |= ZEROPAD; f++; goto repeat_flags;
        case '-': flags |= LEFT;    f++; goto repeat_flags;
        case '+': flags |= PLUS;    f++; goto repeat_flags;
        case ' ': flags |= SPACE;   f++; goto repeat_flags;
        default:  break;
        }

        if (is_digit(*f))
            width = skip_atoi(&f);
        else if (*f == '*')
        { f++; width = va_arg(args, int); }

        if (*f == '.')
        {
            f++;
            if (is_digit(*f))
                prec = skip_atoi(&f);
            else if (*f == '*')
            { f++; prec = va_arg(args, int); }
            if (prec < 0) prec = 0;
        }

        if (*f == 'l' || *f == 'L')
        { qual = *f; f++; }

        switch (*f)
        {
        case 's':
        {
            char *s = va_arg(args, char *);
            if (!s) s = "(null)";
            int len = (prec < 0) ? strnlen_c(s, 255) : prec;
            if (width > 0 && !(flags & LEFT) && width > len)
            { int p = width - len; while (p-- > 0) *str++ = ' '; }
            int i; for (i = 0; i < len; i++) *str++ = s[i];
            if (width > 0 && (flags & LEFT) && width > len)
            { int p = width - len; while (p-- > 0) *str++ = ' '; }
            break;
        }
        case 'c':
            *str++ = (char)va_arg(args, int);
            break;
        case 'f':
#if HAS_FLOAT
            /* 变参里 float 会被提升成 double，必须按 double 取 */
            str = fmt_float(str, va_arg(args, double), prec);
#else
            *str++ = '%'; *str++ = 'f';
#endif
            break;
        case 'o': base = 8;  break;
        case 'x': base = 16; break;
        case 'X': base = 16; flags |= UPPERCASE; break;
        case 'u': base = 10; break;
        case 'd': case 'i': base = 10; flags |= SIGN; break;
        case '%': *str++ = '%'; break;
        default:  *str++ = '%'; *str++ = *f; break;
        }

        if (*f != 's' && *f != 'c' && *f != '%' && *f != 'f')
        {
            unsigned long val = 0;
            if (qual == 'l')
                val = va_arg(args, unsigned long);
            else if (flags & SIGN)
                val = (unsigned long)(long)va_arg(args, int);
            else
                val = va_arg(args, unsigned int);
            str = fmt_number(str, val, base, width, prec, flags);
        }

        f++;
    }

    va_end(args);

    {
        char *_p = buf;
        while (_p != str) uart_send_char(*_p++);
    }
    return (int)(str - buf);
}
