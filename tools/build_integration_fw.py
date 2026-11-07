#!/usr/bin/env python3
"""build_integration_fw.py —— 构建整机集成回归镜像 tb/integration.mem

这条链路由 tb/tb_system_integration.v 使用：CPU 配置 CNN 描述符 → 帧源 →
滑窗 → PE → embedding → 突发回读，最后打印 SYSTEM_INTEGRATION_PASS。

源码：sw/integration.c + sw/start.S，链接脚本 sw/link.ld（RAM 基址 0x0）。
镜像补齐到 8192 词（32 KB），与测试台的 RAM_ADDR_W=15 对应。

用法（在 release/ 下）:
    python tools/build_integration_fw.py            # 只编译，写 tb/integration.mem
    python tools/build_integration_fw.py --check    # 编译后与现有镜像逐字节比对
"""
import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]        # = release/
TC_NAME = "xpack-riscv-none-elf-gcc-15.2.0-1/bin"
OUT = ROOT / "build/fw"
IMAGE = ROOT / "tb/integration.mem"
PAD_WORDS = 8192


def _default_tc():
    for p in [ROOT] + list(ROOT.parents):
        cand = p / "RISCV_CPU/tools/xpack-toolchain" / TC_NAME
        if (cand / "riscv-none-elf-gcc.exe").is_file():
            return cand
    return Path("RISCV_CPU/tools/xpack-toolchain") / TC_NAME


TC = Path(os.environ.get("RISCV_TOOLCHAIN", str(_default_tc())))

FLAGS = [
    "-march=rv32i_zmmul", "-mno-div", "-mabi=ilp32", "-Os",
    "-Wall", "-Wextra", "-ffreestanding", "-fno-builtin",
    "-ffunction-sections", "-fdata-sections", "-msmall-data-limit=0",
    "-nostdlib", "-nostartfiles", "-Isw", "-Tsw/link.ld",
    "-Wl,--gc-sections",
]


def run(cmd):
    subprocess.run([str(c) for c in cmd], cwd=ROOT, check=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true",
                    help="编译结果必须与 tb/integration.mem 完全一致")
    a = ap.parse_args()

    if not (TC / "riscv-none-elf-gcc.exe").is_file():
        raise SystemExit("找不到工具链: %s（设 RISCV_TOOLCHAIN 环境变量）" % TC)

    OUT.mkdir(parents=True, exist_ok=True)
    elf = OUT / "integration.elf"
    run([TC / "riscv-none-elf-gcc.exe"] + FLAGS +
        ["-Wl,-Map,%s" % (OUT / "integration.map"), "-o", elf,
         "sw/start.S", "sw/integration.c", "-lgcc"])
    run([TC / "riscv-none-elf-size.exe", elf])

    dis = subprocess.check_output(
        [str(TC / "riscv-none-elf-objdump.exe"), "-d", str(elf)], text=True)
    (OUT / "integration.dis").write_text(dis, encoding="utf-8")
    bad = re.search(r"\t(?:divu?|remu?)\s", dis)
    assert not bad, "固件里出现 DIV/REM（CPU 不支持）: " + bad.group(0)

    run([TC / "riscv-none-elf-objcopy.exe", "-O", "binary", elf, OUT / "integration.bin"])
    data = (OUT / "integration.bin").read_bytes()
    data += bytes((-len(data)) % 4)
    words = [int.from_bytes(data[i:i + 4], "little") for i in range(0, len(data), 4)]
    assert len(words) <= PAD_WORDS, "镜像超过 %d 词（32 KB）" % PAD_WORDS

    text = "".join("%08X\n" % w for w in words + [0] * (PAD_WORDS - len(words)))
    (OUT / "integration.mem").write_text(text, encoding="utf-8")
    (OUT / "integration.json").write_text(json.dumps(dict(
        bytes=len(data), words=len(words), pad_words=PAD_WORDS,
        sha256=hashlib.sha256(data).hexdigest()), indent=2), encoding="utf-8")

    print("镜像 %d 词 (%.1f KB)，补齐到 %d 词"
          % (len(words), len(words) * 4 / 1024.0, PAD_WORDS))

    if a.check:
        cur = IMAGE.read_text(encoding="utf-8")
        assert cur == text, "编译结果与 %s 不一致" % IMAGE
        print("与 %s 逐字节一致 ✅" % IMAGE.relative_to(ROOT))
    return 0


if __name__ == "__main__":
    sys.exit(main())
