#!/usr/bin/env python3
"""cmp_sbit.py —— 位流逐字节比对（忽略 gen_bit_stream 写进头部的生成时间戳）

为什么不能直接比 md5：`.sbit` 头部有一段 TLV，`c` 标签是生成日期
（`MM/DD/YY`），`d` 标签是生成时间（`HH:MM:SS`）。同一份布局布线换个时刻
重跑，除这 15 个字节外**逐字节相同**。所以判"复现成功"要看：

  1. 尺寸相同
  2. 差异字节集合 ⊆ 日期/时间字段
  3. 其余字节全部一致

用法:
    python tools/cmp_sbit.py A.sbit B.sbit
退出码 0 = 除时间戳外等价；1 = 存在实质差异。
"""
import argparse
import hashlib
import sys
from pathlib import Path

# TLV 中这两个标签是"生成时刻"，不是设计内容。
# 头部格式：\x00 <tag> \x00  <1 字节长度>  <payload>
TIME_TAGS = (0x63, 0x64)   # 'c' = 生成日期 MM/DD/YY, 'd' = 生成时间 HH:MM:SS
FIELD_MAX = 24             # 合法 payload 上限，超出视为非该字段


def stamp_fields(blob, verbose=False):
    """返回头部生成时间戳字段占用的字节偏移集合。

    头部 TLV 形如 00 63 00 09 "11/02/26"：三字节标签 + 1 字节长度 + payload。
    """
    offs = set()
    head = blob[:4096]
    for tid in TIME_TAGS:
        tag = bytes((0x00, tid, 0x00))
        i = head.find(tag)
        if i < 0:
            continue
        lp = i + len(tag)            # 长度字节位置
        if lp >= len(head):
            continue
        n = head[lp]
        if not 0 < n <= FIELD_MAX or lp + 1 + n > len(head):
            continue
        start = lp + 1               # payload 起点
        if verbose:
            print("  时间戳字段 tag=0x%02x @%d len=%d payload=%r"
                  % (tid, i, n, head[start:start + n]))
        offs.update(range(start, start + n))
    return offs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ref", type=Path)
    ap.add_argument("new", type=Path)
    ap.add_argument("--verbose", action="store_true")
    a = ap.parse_args()

    ref, new = a.ref.read_bytes(), a.new.read_bytes()
    print("ref %s  %d B  md5 %s" % (a.ref.name, len(ref), hashlib.md5(ref).hexdigest()))
    print("new %s  %d B  md5 %s" % (a.new.name, len(new), hashlib.md5(new).hexdigest()))
    if len(ref) != len(new):
        print("FAIL 尺寸不同，%+d B" % (len(new) - len(ref)))
        return 1

    diffs = [i for i in range(len(ref)) if ref[i] != new[i]]
    stamp = stamp_fields(ref, a.verbose) | stamp_fields(new, a.verbose)
    body = [i for i in diffs if i not in stamp]

    print("差异字节 %d 个；其中落在生成时间戳字段内 %d 个，字段外 %d 个"
          % (len(diffs), len(diffs) - len(body), len(body)))
    if a.verbose and diffs:
        for i in diffs[:20]:
            where = "时间戳" if i in stamp else "主体"
            print("  offset %-10d %-6s %02x -> %02x" % (i, where, ref[i], new[i]))

    if not body:
        print("RESULT 除生成时间戳外逐字节相同（设计内容等价）")
        return 0
    print("RESULT 存在实质性差异，偏移前几个: %s" % body[:12])
    return 1


if __name__ == "__main__":
    sys.exit(main())
