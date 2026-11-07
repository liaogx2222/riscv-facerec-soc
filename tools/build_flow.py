#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""build_flow.py —— 一条命令走完 RTL → 网表 → 位流 → 校验

三个引擎阶段（顺序固定，不能混用）:

    阶段        引擎            产物                                耗时
    ----------  --------------  ----------------------------------  --------
    1 综合      2022.2-SP6.4    build/syn_<n>/synthesize/*_syn.vm    ~14 min
    2 布局布线  2025.2          build/pnr/<n>/prj_tasks/pnr_1/...    ~40 min
    3 校验      python          timing_summary.json + 码流 md5      秒级

用法（在 release/ 下）:

    # 全流程（默认名字 = today）
    python tools/build_flow.py

    # 指定运行名 + 自定 RTL 树
    python tools/build_flow.py my_run --rtl-root some/rtl

    # 已有网表，只跑布局布线
    python tools/build_flow.py my_run --skip-synth \\
        --netlist build/syn_old/synthesize/facerec_top_syn.vm

    # 顺带与一个参考码流做「除时间戳外逐字节」比对
    python tools/build_flow.py my_run --ref-sbit ../timing_opt/out/facerec_top_menu_line_202p5.sbit

退出码: 0 = All Constraints Met；2 = 有时序违例；1 = 流程失败。
"""
import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]        # release/
TOOLS = ROOT / "tools"
PY = sys.executable

# 注意两个阶段对运行名的用法不同，别写错（写错会另建一个空目录，看起来"跑完了"）：
#   综合  synth_vm.py <n>   → build/syn_<n>/
#   实现  v2_flow.py <n>    → build/pnr/<n>/       ← 没有 pnr_ 前缀
# 所以这里统一只传 <name>，两个脚本各自加前缀。


def step(title):
    print("\n" + "=" * 72)
    print(title)
    print("=" * 72)


def run(cmd, log=None):
    t0 = time.time()
    print("  $ " + " ".join(str(c) for c in cmd))
    if log:
        with open(log, "w", encoding="utf-8") as out:
            rc = subprocess.call([str(c) for c in cmd], cwd=str(ROOT),
                                 stdout=out, stderr=subprocess.STDOUT)
    else:
        rc = subprocess.call([str(c) for c in cmd], cwd=str(ROOT))
    print("    rc=%s  %.0fs" % (rc, time.time() - t0))
    return rc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("name", nargs="?", default=time.strftime("%m%d_%H%M"))
    ap.add_argument("--rtl-root", type=Path, default=ROOT / "rtl",
                    help="要综合的 RTL 根目录（默认交付树 rtl/）")
    ap.add_argument("--skip-synth", action="store_true")
    ap.add_argument("--netlist", type=Path, help="--skip-synth 时指定已有 .vm")
    ap.add_argument("--skip-pnr", action="store_true", help="只做综合")
    ap.add_argument("--ref-sbit", type=Path, help="参考码流；给出后做时间戳感知比对")
    ap.add_argument("--direct-pnr", type=int, default=0, help=">0 时改用多种子直连 pnr")
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()

    name = a.name.replace("_", "").isalnum() and a.name or "run"
    syn_name = "syn_" + name
    vm = ROOT / "build" / syn_name / "synthesize" / "facerec_top_syn.vm"

    # ---- 阶段 1：综合 -----------------------------------------------------
    if a.skip_synth:
        vm = a.netlist or vm
        if not vm.is_file():
            print("找不到网表: %s" % vm)
            return 1
        step("阶段 1/3 综合 —— 跳过，复用 %s" % vm)
    else:
        step("阶段 1/3 综合（PDS 2022.2）")
        rc = run([PY, TOOLS / "synth_vm.py", name, "--rtl-root", a.rtl_root],
                 ROOT / "build" / ("%s.synth.log" % syn_name))
        if rc != 0 or not vm.is_file():
            print("综合失败，见 build/%s.synth.log" % syn_name)
            return 1

    if a.skip_pnr:
        print("\n网表: %s" % vm)
        return 0

    # ---- 阶段 2：布局布线 + 出码流 ---------------------------------------
    step("阶段 2/3 布局布线 + 出码流（PDS 2025.2）")
    cmd = [PY, TOOLS / "v2_flow.py", name, "--netlist", vm]
    if a.direct_pnr:
        cmd += ["--direct-pnr", str(a.direct_pnr), "--seed", str(a.seed)]
    rc = run(cmd)

    run_dir = ROOT / "build" / "pnr" / name
    rtr = run_dir / "prj_tasks/pnr_1/report_timing/facerec_top.rtr"
    sbit = run_dir / "prj_tasks/pnr_1/generate_bitstream/facerec_top.sbit"

    # ---- 阶段 3：校验 -----------------------------------------------------
    step("阶段 3/3 校验")
    js = run_dir / "timing_summary.json"
    info = None
    if js.is_file():
        info = json.loads(js.read_text(encoding="utf-8"))
        print("  Design Summary : %s" % info.get("design"))
        for p in info.get("performance", []):
            print("  %-8s 请求 %.2f MHz → 实测 %.2f MHz    slack %+0.3f ns"
                  % (p["clock"], p["requested_mhz"], p["estimated_mhz"], p["slack"]))
    print("  报告 : %s" % (rtr if rtr.is_file() else "(缺失)"))
    print("  码流 : %s" % (sbit if sbit.is_file() else "(缺失)"))

    if a.ref_sbit and sbit.is_file():
        print("\n  与参考码流比对（忽略生成时间戳）:")
        rc2 = run([PY, TOOLS / "cmp_sbit.py", "--verbose", a.ref_sbit, sbit])
        if rc2 != 0:
            print("  ⚠ 码流与参考不等价 —— 检查 RTL / 约束 / 固件是否与参考同源")

    ok = info is not None and (info.get("design") or "").startswith("All Constraints Met")
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
