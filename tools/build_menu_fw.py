#!/usr/bin/env python3
"""build_menu_fw.py (release 版) —— 构建「CoreMark + 人脸识别」双功能菜单固件

产出一个合并镜像（串口 '1' 跑 CoreMark，'2' 跑门禁人脸识别），并把 word
表内嵌进 rtl/soc/simple_ram.v 的 initial 块。

用法（在 release/ 下）:
    set RISCV_TOOLCHAIN=<...>/xpack-riscv-none-elf-gcc-15.2.0-1/bin
    python tools/build_menu_fw.py                 # 编译 + 内嵌到 rtl/soc/simple_ram.v
    python tools/build_menu_fw.py --no-ram        # 只编译，产出 elf/bin/mem

要点（与赛场跑分口径一致，别随手改）:
  * CoreMark 算法文件必须用官方跑分 flag（-O3 + 展开），门禁用 -Os
  * core_main.c 用 -Dmain=coremark_main 改名，避免和菜单 main 冲突
  * ITERATIONS=6000 @101.25 MHz ≈ 11.6 s，满足赛题"单次跑分 ≥10 s"且
    32 位 tick 不溢出
  * CPU 是 RV32I+Zmmul，没有 DIV/REM —— 编译后断言反汇编里不出现它们
"""
import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]        # = release/
TC_NAME = "xpack-riscv-none-elf-gcc-15.2.0-1/bin"


def _default_tc():
    """在祖先目录里找 RISCV_CPU/tools/xpack-toolchain/<版本>/bin。"""
    for p in [ROOT] + list(ROOT.parents):
        cand = p / "RISCV_CPU/tools/xpack-toolchain" / TC_NAME
        if (cand / "riscv-none-elf-gcc.exe").is_file():
            return cand
    return Path("RISCV_CPU/tools/xpack-toolchain") / TC_NAME


TC = Path(os.environ.get("RISCV_TOOLCHAIN", str(_default_tc())))

OUT = ROOT / "build/fw"
RAM = ROOT / "rtl/soc/simple_ram.v"

ITERATIONS = 6000
SYSCLK_HZ  = 101_250_000

COMMON = [
    "-march=rv32i_zmmul", "-mno-div", "-mabi=ilp32",
    "-ffreestanding", "-fno-builtin", "-ffunction-sections", "-fdata-sections",
    "-msmall-data-limit=0", "-fno-common", "-fno-stack-protector", "-std=gnu99",
    "-Wall", "-Wextra",
]
CM_OPT   = ["-O3", "-funroll-loops", "-funroll-all-loops", "-fpeel-loops",
            "-falign-functions=4", "-falign-loops=4"]
GATE_OPT = ["-Os"]
DEFS = [
    "-DTOTAL_DATA_SIZE=2000", "-DVALIDATION_RUN=1",
    "-DITERATIONS=%d" % ITERATIONS, "-DSYSCLK_HZ=%d" % SYSCLK_HZ,
    "-DWITH_MENU=1",
]
INCS = ["-Isw", "-Isw/coremark", "-Isw/coremark/barebones"]

# (源文件, 编译选项, 额外宏)
UNITS = [
    ("sw/start.S",                                    [],       []),
    ("sw/gate_core.c",                                GATE_OPT, []),
    ("sw/gate_main.c",                                GATE_OPT, []),
    ("sw/gate_board.c",                               GATE_OPT, []),
    ("sw/menu_main.c",                                GATE_OPT, []),
    ("sw/uart_out.c",                                 GATE_OPT, []),
    ("sw/coremark/core_list_join.c",                  CM_OPT,   []),
    ("sw/coremark/core_matrix.c",                     CM_OPT,   []),
    ("sw/coremark/core_state.c",                      CM_OPT,   []),
    ("sw/coremark/core_util.c",                       CM_OPT,   []),
    ("sw/coremark/core_main.c",                       CM_OPT,   ["-Dmain=coremark_main"]),
    ("sw/coremark/barebones/core_portme.c",           CM_OPT,   []),
    ("sw/coremark/barebones/ee_printf.c",             CM_OPT,   []),
]


def run(cmd):
    subprocess.run([str(c) for c in cmd], cwd=ROOT, check=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-ram", action="store_true",
                    help="只编译，不写 rtl/soc/simple_ram.v")
    a = ap.parse_args()

    if not (TC / "riscv-none-elf-gcc.exe").is_file():
        raise SystemExit("找不到工具链: %s（设 RISCV_TOOLCHAIN 环境变量）" % TC)

    OUT.mkdir(parents=True, exist_ok=True)
    gcc = TC / "riscv-none-elf-gcc.exe"

    objs = []
    for i, (src, opt, extra) in enumerate(UNITS):
        obj = OUT / ("u%02d_%s.o" % (i, Path(src).stem))
        run([gcc] + COMMON + opt + DEFS + extra + INCS + ["-c", src, "-o", obj])
        objs.append(obj)

    elf = OUT / "menu.elf"
    run([gcc] + COMMON + ["-nostdlib", "-nostartfiles",
                          "-Tsw/link_menu.ld", "-Wl,--gc-sections",
                          "-Wl,-Map,%s" % (OUT / "menu.map"),
                          "-o", elf] + objs + ["-lgcc"])

    run([TC / "riscv-none-elf-size.exe", elf])

    dis = subprocess.check_output(
        [str(TC / "riscv-none-elf-objdump.exe"), "-d", str(elf)], text=True)
    (OUT / "menu.dis").write_text(dis, encoding="utf-8")
    bad = re.search(r"\t(?:divu?|remu?)\s", dis)
    assert not bad, "固件里出现 DIV/REM（CPU 不支持）: " + bad.group(0)

    run([TC / "riscv-none-elf-objcopy.exe", "-O", "binary", elf, OUT / "menu.bin"])
    data = (OUT / "menu.bin").read_bytes()
    data += bytes((-len(data)) % 4)
    words = [int.from_bytes(data[i:i + 4], "little") for i in range(0, len(data), 4)]

    (OUT / "menu_embed.mem").write_text("".join("%08X\n" % w for w in words),
                                       encoding="utf-8")
    (OUT / "menu.json").write_text(json.dumps(dict(
        bytes=len(data), words=len(words), iterations=ITERATIONS,
        sysclk_hz=SYSCLK_HZ,
        expect_seconds=round(ITERATIONS * 195429 / SYSCLK_HZ, 2),
        sha256=hashlib.sha256(data).hexdigest()), indent=2), encoding="utf-8")

    print("\n固件 %d 词 (%.1f KB)，预计 CoreMark 跑 %.1f s"
          % (len(words), len(words) * 4 / 1024.0, ITERATIONS * 195429 / SYSCLK_HZ))

    if a.no_ram:
        return 0

    assert len(words) <= 16384, "超过 64 KB RAM"
    run([sys.executable, ROOT / "tools/gen_board_ram_init.py",
         OUT / "menu_embed.mem", RAM])
    s = RAM.read_text(encoding="utf-8")
    s = s.replace("parameter ADDR_W     = 17,", "parameter ADDR_W     = 16,")
    RAM.write_text(s, encoding="utf-8", newline="\n")
    print("已写入 %s（ADDR_W=16，64 KB）" % RAM)
    return 0


if __name__ == "__main__":
    sys.exit(main())
