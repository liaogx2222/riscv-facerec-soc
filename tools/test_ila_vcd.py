#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""test_ila_vcd.py —— 校验 tools/ila_wave.py --vcd 导出的 VCD 是否与串口采样一致

做法：把导出的 VCD 按标准语义回放（时间戳处理完所有变化后再拍快照），
逐信号逐拍与日志里解析出来的 32 位 probe 比对；顺便检查
  · clk 是否 48 个采样对应 48 个完整周期
  · 触发标记（bit31）是否落在同一拍
  · 时间轴是否 i × 9877 ps（101.25 MHz）

用法：
    python tools/test_ila_vcd.py                      # 用内置合成数据自测
    python tools/test_ila_vcd.py build/ila/run.log    # 用真实抓包自测
"""
import io
import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import ila_wave as W                                     # noqa: E402

PS = 9877


def make_fake_log(path):
    """合成一段覆盖所有类字符的采样：R / 2 / F / D / L / M"""
    pc = 0x1a0c0
    rows = []
    for i in range(40):
        fl = 1 << 17
        if i == 12:
            fl |= 1 << 30                                  # M：误预测 + 触发
            pc -= 0x30
        elif i % 9 == 4:
            fl |= 1 << 21                                  # L：load-use
        elif i % 7 == 2:
            fl |= 1 << 28                                  # 2：双退休
        elif i % 11 == 6:
            fl |= 1 << 19                                  # D：数据等待
        elif i % 13 == 3:
            fl |= 1 << 18                                  # F：取指未就绪
        raw = ((pc >> 2) & 0x1FFFF) | fl | (0x80000000 if i == 12 else 0)
        cls = ('M' if fl >> 30 & 1 else
               'L' if fl >> 21 & 1 else
               '2' if fl >> 28 & 1 else
               'D' if fl >> 19 & 1 else
               'F' if fl >> 18 & 1 else 'R')
        rows.append('%08x %s' % (raw, cls))
        pc += 4
    lines = ['[ILA] hex pc|flg x2']
    for i in range(0, len(rows), 2):
        lines.append(' '.join(rows[i:i + 2]))
    io.open(path, 'w', encoding='utf-8', newline='').write('\r\n'.join(lines) + '\r\n')


def replay(vcd_path):
    """按 VCD 标准语义回放，返回 (每拍快照 dict 列表, clk 上升沿个数, 时间戳集合)"""
    txt = io.open(vcd_path, encoding='utf-8').read().splitlines()
    ids, i = {}, 0
    while not txt[i].startswith('$enddefinitions'):
        m = re.match(r'\$var wire (\d+) (\S+) (\S+) \$end', txt[i])
        if m:
            ids[m.group(2)] = (m.group(3), int(m.group(1)))
        i += 1

    snaps, cur, rises, stamps = [], {}, 0, []
    t = None

    def commit():
        if t is not None and t % PS == 0:
            snaps.append((t, dict(cur)))

    for ln in txt[i + 1:]:
        if ln.startswith('#'):
            commit()
            t = int(ln[1:])
            stamps.append(t)
            continue
        if ln.startswith('b'):
            v, k = ln[1:].split()
            cur[ids[k][0]] = int(v, 2)
        elif ln and ln[0] in '01':
            k = ids[ln[1:]][0]
            if k == 'clk_sys' and ln[0] == '1':
                rises += 1
            cur[k] = int(ln[0])
    commit()

    # 时间戳 t 处的快照要包含该时刻的变化 ⇒ 用下一时刻的 before 状态
    fixed = []
    for idx, (t, st) in enumerate(snaps):
        fixed.append((t, st))
    return fixed, rises, stamps


def gtkwave_roundtrip(vcd_path):
    """如果本机有 GTKWave，用**它自带的解析器**再验一次（vcd2fst）

    这一步的意义：前面那些检查都是本脚本自己的解析器，万一两边对规范理解一致
    但都错，就查不出来。用 GTKWave 的 vcd2fst 能过，就等于"GTKWave 肯定打得开"。
    没装 GTKWave 就跳过，不报错。
    """
    cand = []
    for p in (os.environ.get('GTKWAVE_BIN', ''),):
        if p:
            cand.append(os.path.join(p, 'vcd2fst.exe'))
    _which = shutil.which('vcd2fst')
    if _which:
        cand.append(_which)
    exe = next((c for c in cand if os.path.isfile(c)), None)
    if not exe:
        print('  · 跳过 GTKWave 往返校验（没找到 vcd2fst.exe）')
        return True
    fst = vcd_path + '.fst'
    r = subprocess.run([exe, vcd_path, fst], capture_output=True)
    ok = r.returncode == 0 and os.path.isfile(fst) and os.path.getsize(fst) > 0
    msg = (r.stdout + r.stderr).decode('utf-8', 'replace').strip()
    print('  %s GTKWave vcd2fst：rc=%d%s' % ('✓' if ok else '✗', r.returncode,
                                            ('  ' + msg[:120]) if msg else ''))
    if ok:
        print('    → %s（%d 字节），GTKWave 可直接打开' % (os.path.basename(fst),
                                                         os.path.getsize(fst)))
    return ok


def check(log_path, vcd_path):
    samples = W.parse(log_path)
    snaps, rises, stamps = replay(vcd_path)
    snaps.sort(key=lambda x: x[0])
    n = len(samples)

    print('日志采样 %d 拍；VCD 快照 %d 拍；clk 上升沿 %d 次'
          % (n, len(snaps), rises))
    ok = True
    if len(snaps) < n:
        print('  ✗ 快照数不足')
        return False

    for nm, w, bn in W.VCD_SIGS:
        exp = [(((raw & 0x1FFFF) << 2) if bn is None else ((raw >> bn) & 1))
               for raw, _ in samples]
        # VCD 在 t=i*PS 处写入第 i 拍的值；快照跨度为 [i*PS, (i+1)*PS)
        got = []
        for k in range(n):
            st = None
            for t, s in snaps:
                if t <= k * PS:
                    st = s
                else:
                    break
            got.append(0 if st is None else st.get(nm, 0))
        diff = [(k, a, b) for k, (a, b) in enumerate(zip(got, exp)) if a != b]
        if diff:
            ok = False
            print('  ✗ %-14s 不一致 %d 处，前 3：%s' % (nm, len(diff), diff[:3]))

    # 触发拍
    trig_src = [k for k, (raw, _) in enumerate(samples) if (raw >> 31) & 1]
    trig_vcd = []
    for k in range(n):
        st = None
        for t, s in snaps:
            if t <= k * PS:
                st = s
            else:
                break
        if st and st.get('trigger'):
            trig_vcd.append(k)
    if trig_src != trig_vcd:
        ok = False
        print('  ✗ 触发拍不一致：源 %s / VCD %s' % (trig_src, trig_vcd))
    else:
        print('  ✓ 触发拍对齐：%s' % trig_src)

    # 时间轴
    bad_t = [k for k in range(n) if k * PS not in [t for t, _ in snaps]]
    if bad_t:
        ok = False
        print('  ✗ 时间轴缺拍：%s' % bad_t[:5])
    if rises != n:
        ok = False
        print('  ✗ clk 上升沿 %d ≠ 采样数 %d' % (rises, n))

    if not gtkwave_roundtrip(vcd_path):
        ok = False

    print('结论：%s' % ('全部一致 ✓' if ok else '有不一致 ✗'))
    return ok


def main():
    if len(sys.argv) > 1:
        log = sys.argv[1]
    else:
        tmp = os.path.join(ROOT, 'build', 'ila', '_selftest.log')
        os.makedirs(os.path.dirname(tmp), exist_ok=True)
        make_fake_log(tmp)
        log = tmp
        print('自测：用合成日志', log)

    vcd = log.replace('.log', '.vcd')
    r = subprocess.run([sys.executable, os.path.join(HERE, 'ila_wave.py'),
                        log, '--out', log.replace('.log', '.svg'), '--vcd', vcd],
                       capture_output=True)
    if r.returncode != 0:
        sys.exit('生成 VCD 失败：' + r.stderr.decode('utf-8', 'replace'))

    ok = check(log, vcd)
    print('VCD:', vcd, '(%d 字节)' % os.path.getsize(vcd))
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
