#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""manifest.py —— 给交付树打一份输入指纹清单

用途：任何一次 P&R 结果出来后，能回答"这是哪一版 RTL / 约束 / 固件 / ROM 跑出来的"。
只读，不改任何文件。

用法:
    python tools/manifest.py                 # 打印
    python tools/manifest.py --write         # 写入 MANIFEST.md
    python tools/manifest.py --json          # 输出 JSON（给 CI 比对）
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]     # release/
GROUPS = [
    ("RTL (core)",        "rtl/core/*.v"),
    ("RTL (soc)",         "rtl/soc/*.v"),
    ("RTL (accel)",       "rtl/accel/*.v"),
    ("RTL (board)",       "rtl/board/*.v"),
    ("CNN ROM (.mem)",    "rtl/accel/mem/*.mem"),
    ("约束",              "constraints/**/*.fdc"),
    ("约束 (签核)",       "constraints/**/*.sdc"),
    ("约束 (钉位)",       "constraints/**/*.pcf"),
    ("固件源码",          "sw/*.[cSh]"),
    ("测试台",            "tb/*.v"),
]


def h(p):
    d = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            d.update(b)
    return d.hexdigest()[:16]


def collect():
    out = {}
    for label, pat in GROUPS:
        files = sorted(ROOT.glob(pat))
        if not files:
            continue
        out[label] = [{"file": str(f.relative_to(ROOT)).replace("\\", "/"),
                       "bytes": f.stat().st_size, "sha256_16": h(f)} for f in files]
    return out


def rollup(entries):
    """该组整体指纹：把所有 (相对路径, sha) 排序后一起 hash。"""
    d = hashlib.sha256()
    for e in sorted(entries, key=lambda x: x["file"]):
        d.update(e["file"].encode() + e["sha256_16"].encode())
    return d.hexdigest()[:16]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()

    data = collect()
    if a.json:
        print(json.dumps({k: {"rollup": rollup(v), "files": v} for k, v in data.items()},
                         indent=2, ensure_ascii=False))
        return 0

    lines = ["# 输入指纹清单", "",
             "由 `tools/manifest.py --write` 生成。任何一次 P&R 的结论都应该能对回这里。", "",
             "| 组 | 文件数 | 整体指纹（sha256 前 16 位） |", "|---|---|---|"]
    for k, v in data.items():
        lines.append("| %s | %d | `%s` |" % (k, len(v), rollup(v)))
    lines += ["", "## 明细", ""]
    for k, v in data.items():
        lines += ["### %s" % k, "", "| 文件 | 字节 | sha256/16 |", "|---|---|---|"]
        for e in v:
            lines.append("| `%s` | %d | `%s` |" % (e["file"], e["bytes"], e["sha256_16"]))
        lines.append("")
    text = "\n".join(lines)

    if a.write:
        (ROOT / "MANIFEST.md").write_text(text, encoding="utf-8")
        total = sum(len(v) for v in data.values())
        print("written MANIFEST.md  (%d files, %d groups)" % (total, len(data)))
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
