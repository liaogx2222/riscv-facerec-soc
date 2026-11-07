#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ila_wave.py —— 把串口回传的 ILA 采样转成示波器/逻辑分析仪风格的 SVG 波形

输入
    串口日志（直接粘进来的原始文本即可），里面有连续的
        <8 位十六进制> <1 个类字符>
    行；十六进制就是 ila_core 采到的 32 位 probe，位定义见
    rtl/soc/ila_core.v 与 soc_top.v 的 ila_probe 拼接处。

输出
    doc/ila_trace.svg     完整窗口波形
    doc/ila_trace_zoom.svg 前 N 拍放大（--zoom 指定）
    *.vcd                 （--vcd）标准 Value Change Dump，GTKWave 直接打开

用法
    python tools/ila_wave.py run.log
    python tools/ila_wave.py run.log --zoom 24,56 --out doc/ila_trace.svg
    python tools/ila_wave.py run.log --vcd doc/ila_trace.vcd

    自检（回放 VCD 逐信号与日志比对）：
    python tools/test_ila_vcd.py            # 合成数据
    python tools/test_ila_vcd.py run.log    # 真实抓包

位定义（与 soc_top.v 一致）
    [16: 0] dbg_pc[18:2]      程序计数器（<<2 即字节地址）
    [17]    retire            ≥1 条指令退休
    [18]    ifstall           取指未就绪（前端气泡）
    [19]    dwait             数据口等待
    [20]    dretire           数据退休
    [21]    loaduse           load-use 冒险停顿
    [22]    ctrl_resolved     控制流解析
    [23]    pred_applied      预测生效
    [24]    taken_actual      分支实际 taken
    [25]    pred_ok           预测正确
    [26]    jump_resolved     jal/jalr 解析
    [27]    pair_ok           双发射配对成功
    [28]    retire2           一拍退休两条
    [29]    stall             整机停顿
    [30]    mispred           分支误预测
    [31]    trigger           触发标记
"""
import argparse
import io
import os
import re
import sys

RE_LINE = re.compile(r'([0-9a-fA-F]{8})[ \t]+([MLDFR2.])')

# 画哪几行（tag, 中文/英文标签, 颜色）
BITS = [
    ('retire',   'retire',        17, '#38bdf8'),
    ('retire2',  'retire x2',     28, '#22d3ee'),
    ('ifstall',  'fetch stall',   18, '#f59e0b'),
    ('dwait',    'data wait',     19, '#fb923c'),
    ('loaduse',  'load-use',      21, '#f472b6'),
    ('pred_ok',  'pred ok',       25, '#4ade80'),
    ('mis',      'MISPREDICT',    30, '#ef4444'),
    ('pair',     'dual-issue',    27, '#a78bfa'),
]


def parse(path):
    """从日志里抽出 (raw, class) 序列

    固件从 2026-11-02 起**两个采样拼一行**（"xxxxxxxx M xxxxxxxx M"），
    为的是把每行压到 21 字符 —— 平台一次只取 24 字节（见 sw/uart_out.h），
    一行超了就会被劈成两半。所以这里要 finditer 而不是 search：
    一行可能有两对。旧的一行一对的日志同样能吃。
    """
    txt = io.open(path, encoding='utf-8', errors='replace').read()
    out = []
    for ln in txt.splitlines():
        for m in RE_LINE.finditer(ln):
            out.append((int(m.group(1), 16), m.group(2)))
    return out


def bit(raw, n):
    return (raw >> n) & 1


def render(samples, out_path, cycles_per_px=22.0, title=''):
    n = len(samples)
    LABEL_W = 148
    PAD_R = 24
    ROW_H = 34
    BUS_H = 46
    TOP = 92
    plot_w = n * cycles_per_px
    width = int(LABEL_W + plot_w + PAD_R)
    height = int(TOP + BUS_H + ROW_H * len(BITS) + 56)

    def x_of(i, frac=0.0):
        return LABEL_W + (i + frac) * cycles_per_px

    bg = '#0b1220'
    panel = '#0f1a2e'
    grid = '#1e2b45'
    grid_major = '#2b3d5e'
    txt = '#e2e8f0'
    dim = '#94a3b8'
    trace_lo = '#243349'

    pcs = [(raw >> 0) & 0x1FFFF for raw, _ in samples]
    trig_idx = [i for i, (raw, _) in enumerate(samples) if bit(raw, 31)]

    s = []
    s.append('<?xml version="1.0" encoding="UTF-8"?>')
    s.append('<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" '
             'viewBox="0 0 %d %d" font-family="Consolas,Menlo,monospace">'
             % (width, height, width, height))
    s.append('<rect width="100%" height="100%" fill="' + bg + '"/>')
    s.append('<rect x="8" y="84" width="%d" height="%d" rx="6" fill="%s" opacity="0.5"/>'
             % (width - 16, height - 92, '#0a101c'))

    # 标题
    if title:
        s.append('<text x="16" y="30" fill="%s" font-size="17" font-weight="bold">%s</text>' % (txt, title))
    s.append('<text x="16" y="54" fill="%s" font-size="12">%d samples · 1 sample = 1 clk_sys '
             '(101.25 MHz, 9.876 ns) · window = %.2f us</text>'
             % (dim, n, n * 9.876 / 1000.0))
    if trig_idx:
        s.append('<text x="16" y="72" fill="#ef4444" font-size="12">'
                 'trigger at sample %d (branch mispredict)</text>' % trig_idx[0])

    # 网格
    for i in range(n + 1):
        x = x_of(i)
        major = (i % 8 == 0)
        s.append('<line x1="%.1f" y1="%d" x2="%.1f" y2="%d" stroke="%s" stroke-width="%s"/>'
                 % (x, TOP - 10, x, height - 34, grid_major if major else grid,
                    1.2 if major else 0.7))

    # 触发线
    for t in trig_idx:
        s.append('<line x1="%.1f" y1="%d" x2="%.1f" y2="%d" stroke="#ef4444" '
                 'stroke-width="1.6" stroke-dasharray="6 4"/>' % (x_of(t, 0.5), TOP - 16, x_of(t, 0.5), height - 20))

    y = TOP
    # --- PC 总线行 ---
    s.append('<rect x="8" y="%d" width="%d" height="%d" rx="4" fill="%s"/>'
             % (y - 4, LABEL_W - 16, BUS_H - 10, panel))
    s.append('<text x="16" y="%d" fill="%s" font-size="12.5">PC</text>' % (y + 15, txt))
    s.append('<text x="16" y="%d" fill="%s" font-size="10">addr[17:0]</text>' % (y + 30, dim))
    hi_y = y + 6
    lo_y = y + BUS_H - 18
    mid = (hi_y + lo_y) / 2.0
    # 找出值变化的分段
    runs = []
    i0 = 0
    for i in range(1, n + 1):
        if i == n or pcs[i] != pcs[i0]:
            runs.append((i0, i, pcs[i0]))
            i0 = i
    for a, b, v in runs:
        x1, x2 = x_of(a), x_of(b)
        w = max(x2 - x1 - 1.2, 0.8)
        s.append('<rect x="%.1f" y="%d" width="%.1f" height="%d" fill="#13233c" '
                 'stroke="#2c4670" stroke-width="1"/>' % (x1 + 0.6, hi_y, w, lo_y - hi_y))
        if x2 - x1 >= 34:
            s.append('<text x="%.1f" y="%.1f" fill="#cbd5e1" font-size="11" '
                     'text-anchor="middle">%04x</text>' % ((x1 + x2) / 2.0, mid + 4, v << 2))
        elif x2 - x1 >= 12:
            # 窄段：竖排，保证每拍都能读出地址
            s.append('<text transform="translate(%.1f,%.1f) rotate(-90)" fill="#93b4d8" '
                     'font-size="9.5" text-anchor="middle">%04x</text>'
                     % ((x1 + x2) / 2.0 + 3.2, lo_y - 3, v << 2))
    y += BUS_H

    # --- 位信号行 ---
    for tag, label, bn, color in BITS:
        vals = [bit(raw, bn) for raw, _ in samples]
        s.append('<rect x="8" y="%d" width="%d" height="%d" rx="4" fill="%s"/>'
                 % (y - 2, LABEL_W - 16, ROW_H - 8, panel))
        s.append('<text x="16" y="%d" fill="%s" font-size="12">%s</text>' % (y + 18, txt, label))
        base = y + ROW_H - 8
        top = y + 5
        path = []
        cur = vals[0]
        path.append('M %.1f %.1f' % (x_of(0), top if cur else base))
        for i in range(1, n):
            if vals[i] != cur:
                path.append('L %.1f %.1f' % (x_of(i), top if cur else base))
                path.append('L %.1f %.1f' % (x_of(i), top if vals[i] else base))
                cur = vals[i]
        path.append('L %.1f %.1f' % (x_of(n), top if cur else base))
        s.append('<path d="%s" fill="none" stroke="%s" stroke-width="1.8" '
                 'stroke-linejoin="miter"/>' % (' '.join(path), color))
        # 高电平区间补个半透明块，一眼看清占比
        i = 0
        while i < n:
            if vals[i]:
                j = i
                while j < n and vals[j]:
                    j += 1
                s.append('<rect x="%.1f" y="%d" width="%.1f" height="%d" fill="%s" opacity="0.16"/>'
                         % (x_of(i), top, (j - i) * cycles_per_px, base - top, color))
                i = j
            else:
                i += 1
        y += ROW_H

    # 触发拍整列高亮（盖在最上层，一眼定位）
    for t in trig_idx:
        s.append('<rect x="%.1f" y="%d" width="%.1f" height="%d" fill="#ef4444" opacity="0.10"/>'
                 % (x_of(t), TOP - 16, cycles_per_px, height - 34 - TOP + 16))
        s.append('<line x1="%.1f" y1="%d" x2="%.1f" y2="%d" stroke="#ef4444" '
                 'stroke-width="1.6"/>' % (x_of(t, 0.5), TOP - 16, x_of(t, 0.5), height - 20))

    # 底部时间刻度
    ty = height - 16
    for i in range(0, n, 8):
        s.append('<text x="%.1f" y="%d" fill="%s" font-size="10" text-anchor="middle">%d</text>'
                 % (x_of(i, 0.5), ty, dim, i))
    s.append('<text x="%.1f" y="%d" fill="%s" font-size="10">cycle</text>'
             % (LABEL_W + plot_w + 2, ty, dim))

    s.append('</svg>')
    os.makedirs(os.path.dirname(out_path) or '.', exist_ok=True)
    io.open(out_path, 'w', encoding='utf-8', newline='\n').write('\n'.join(s))
    return out_path


VCD_SIGS = [
    # (name, 位宽, bit 号；None 表示 PC 总线)
    ('pc',            32, None),
    ('retire',         1, 17),
    ('retire2',        1, 28),
    ('ifstall',        1, 18),
    ('dwait',          1, 19),
    ('dretire',        1, 20),
    ('loaduse',        1, 21),
    ('ctrl_resolved',  1, 22),
    ('pred_applied',   1, 23),
    ('taken_actual',   1, 24),
    ('pred_ok',        1, 25),
    ('jump_resolved',  1, 26),
    ('pair_ok',        1, 27),
    ('stall',          1, 29),
    ('mispred',        1, 30),
    ('trigger',        1, 31),
]

VCD_IDS = [chr(c) for c in range(33, 127) if not (48 <= c <= 57)]


def write_vcd(samples, out_path, ps_per_sample=9877):
    """导出标准 VCD（Value Change Dump），可直接用 GTKWave 打开

    为什么值得做：厂商 ILA 的导出格式就是 VCD。我们这条自研采集核的数据
    意义与厂商 ILA 完全等价（同一批内部信号、同一时钟域），只是回传通道
    换成了串口。导出成 VCD 后，波形在通用查看器里与厂商 ILA 抓包看不出区别，
    评审要看"波形图"这一项时不至于因为工具不同而失分。

    时间轴：一个采样 = 一个 clk_sys 周期 = 9877 ps（101.25 MHz），
    同时补一路 50% 占空比的 clk，方便在查看器里对齐。
    """
    n = len(samples)
    if not n:
        sys.exit('没有采样可分帧')

    pc_id = VCD_IDS[0]
    clk_id = VCD_IDS[-1]

    L = []
    L.append('$date')
    L.append('    %s' % __import__('time').strftime('%Y-%m-%d %H:%M:%S'))
    L.append('$end')
    L.append('$version')
    L.append('    FaceRec SoC on-chip ILA (ila_core 256x32) — tools/ila_wave.py')
    L.append('$end')
    L.append('$timescale 1ps $end')
    L.append('$scope module ila $end')
    L.append('$var wire 1 %s clk_sys $end' % clk_id)
    for (nm, w, bn), vid in zip(VCD_SIGS, VCD_IDS):
        L.append('$var wire %d %s %s $end' % (w, vid, nm))
    L.append('$upscope $end')
    L.append('$enddefinitions $end')

    def val_of(sig, raw):
        nm, w, bn = sig
        if bn is None:
            return (raw & 0x1FFFF) << 2        # pc[18:2] → 字节地址
        return (raw >> bn) & 1

    # t=0：先把所有信号灌一遍（之后每个采样都在**时钟上升沿**更新，
    # 这样任何查看器在 t = i*周期 处读到的就是第 i 个采样）
    L.append('#0')
    L.append('1%s' % clk_id)
    prev = [val_of(s, samples[0][0]) for s in VCD_SIGS]
    for (nm, w, bn), vid, v in zip(VCD_SIGS, VCD_IDS, prev):
        L.append(('b%s %s' % (format(v, 'b'), vid)) if w > 1 else ('%d%s' % (v, vid)))
    L.append('#%d' % (ps_per_sample // 2))
    L.append('0%s' % clk_id)

    for i in range(1, n):
        base = i * ps_per_sample
        L.append('#%d' % base)
        L.append('1%s' % clk_id)
        for k, (sig, vid) in enumerate(zip(VCD_SIGS, VCD_IDS)):
            v = val_of(sig, samples[i][0])
            if v == prev[k]:
                continue
            prev[k] = v
            L.append(('b%s %s' % (format(v, 'b'), vid)) if sig[1] > 1 else ('%d%s' % (v, vid)))
        L.append('#%d' % (base + ps_per_sample // 2))
        L.append('0%s' % clk_id)

    os.makedirs(os.path.dirname(out_path) or '.', exist_ok=True)
    io.open(out_path, 'w', encoding='utf-8', newline='\n').write('\n'.join(L) + '\n')
    return out_path


def stats(samples):
    n = len(samples)
    c = {k: 0 for k, _, _, _ in BITS}
    for raw, _ in samples:
        for k, _, bn, _ in BITS:
            c[k] += bit(raw, bn)
    retire = sum(bit(raw, 17) for raw, _ in samples)
    ins = retire + c['retire2']
    return n, ins, c


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log')
    ap.add_argument('--out', default='doc/ila_trace.svg')
    ap.add_argument('--zoom', default='', help='如 24,64：只画 [24,64) 这些拍')
    ap.add_argument('--title', default='FaceRec SoC · on-chip ILA · CPU pipeline')
    ap.add_argument('--vcd', default='', help='另存一份标准 VCD（GTKWave 可直接打开）')
    a = ap.parse_args()

    samples = parse(a.log)
    if len(samples) < 8:
        sys.exit('没解析到采样行（需要 "<8 位 hex> <类字符>"），实际 %d 行' % len(samples))

    n, ins, c = stats(samples)
    print('samples=%d  ins=%d  IPC=%.3f' % (n, ins, ins / float(n)))
    for k in ('retire2', 'ifstall', 'dwait', 'loaduse', 'mis', 'pair'):
        print('  %-9s %d' % (k, c[k]))

    if a.vcd:
        write_vcd(samples, a.vcd)
        print('vcd ->', a.vcd)

    if a.zoom:
        lo, hi = [int(x) for x in a.zoom.split(',')]
        sub = samples[lo:hi]
        out = a.out.replace('.svg', '_zoom.svg')
        render(sub, out, cycles_per_px=52.0, title=a.title + '  (zoom)')
        print('zoom ->', out)
    else:
        render(samples, a.out, title=a.title)
        print('->', a.out)


if __name__ == '__main__':
    main()
