#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""synth_vm.py —— 用 2022.2 综合 release/rtl，产出 post_syn 网表供 v2_flow.py 使用

为什么必须两步：PDS 2025.2 的合成器在本设计上段错误（rtl-data-opt: signal 11），
所以 RTL 一律由 2022.2 综合成门级 Verilog（`facerec_top_syn.vm`），
再用 `v2_flow.py --netlist` 交给 2025.2 做 dev_map/place/route/report。

★ 关键陷阱：RTL 里的 ROM 初始化写的是**相对路径** `rtl/accel/mem/*.mem`，
  PDS 按**当前工作目录**解析。工作目录里没有 `rtl/` 时它会静默地把
  所有权重/描述符/帧 ROM 读成空（只报 C: Verilog-3001），综合照样"成功"，
  但 DRM/FF 数量腰斩 —— 网表是假的。所以每次都把 RTL 覆盖式拷到 `<run>/rtl/`。

用法:
    python tools/synth_vm.py release            # 综合 release/rtl
    python tools/synth_vm.py release --rtl-root ../timing_opt/candidates/xxx/rtl

产物: build/syn_<name>/synthesize/facerec_top_syn.vm
"""
import argparse
import glob
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

PDS = os.environ.get("PDS_SHELL_2022", r"E:\PDS_2022.2-SP6.4\bin\pds_shell.exe")
ROOT = Path(__file__).resolve().parent.parent          # release/
FDC = ROOT / "constraints" / "product" / "board.fdc"


def p(x):
    return str(x).replace("\\", "/")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("name")
    ap.add_argument("--rtl-root", type=Path, default=ROOT / "rtl")
    ap.add_argument("--fdc", type=Path, default=FDC)
    ap.add_argument("--paths", type=int, default=200)
    a = ap.parse_args()

    src = a.rtl_root.resolve()
    work = ROOT / "build" / ("syn_" + a.name)
    work.mkdir(parents=True, exist_ok=True)

    # ROM 相对路径要能解析：把 RTL 原样铺到 <work>/rtl（覆盖式，不 rmtree）
    stage = work / "rtl"
    shutil.copytree(src, stage, dirs_exist_ok=True)
    nfile = sum(len(fs) for _, _, fs in os.walk(stage))
    print("staged %d files -> %s" % (nfile, stage))

    # soc_top.v 用 `include "rv32i_defs.v"`，PDS 按 work_dir 找包含文件，
    # 而源文件在 rtl/core/ 下 —— 必须再拷一份到 work 根目录，否则报
    # Verilog-4104 "Include file rv32i_defs.v is not exists"。
    defs = list(Path(stage).rglob("rv32i_defs.v"))
    if not defs:
        print("rv32i_defs.v not found under", stage)
        return 2
    shutil.copy2(defs[0], work / "rv32i_defs.v")

    vfiles = sorted(glob.glob(str(stage) + "/**/*.v", recursive=True))
    if not vfiles:
        print("no .v under", stage)
        return 2

    lines = ["add_design {%s}" % p(v) for v in vfiles]
    lines += [
        "add_constraint {%s}" % p(a.fdc.resolve()),
        "set_arch -family Logos2 -device PG2L200H -speedgrade -6 -package FBB676",
        "compile -top_module {facerec_top} -system_verilog -top_module facerec_top",
        "synthesize -ads -max_path %d -selected_syn_tool_opt 2" % a.paths,
        "exit",
    ]
    tcl = work / "synth.tcl"
    tcl.write_text("\n".join(lines) + "\n", encoding="utf-8")

    env = dict(os.environ)
    env["LM_LICENSE_FILE"] = "27001@LAPTOP-JRJMGR56"       # 2022.2 的变量名
    t0 = time.time()
    log = work / "synth.log"
    with open(log, "w", encoding="utf-8") as out:
        rc = subprocess.call([PDS, "-file", str(tcl), "-work_dir", str(work)],
                             cwd=str(work), env=env, stdout=out, stderr=subprocess.STDOUT)
    txt = log.read_text(encoding="utf-8", errors="replace")
    vm = work / "synthesize" / "facerec_top_syn.vm"
    errs = re.findall(r"^E: .*$", txt, re.M)
    missed = re.findall(r'Cannot open specified data file "([^"]+)"', txt)
    ok = rc == 0 and vm.is_file() and not missed
    print("rc=%s  %.0fs  vm=%s  %s  errors=%d  missed_mem=%d"
          % (rc, time.time() - t0, vm.is_file(),
             ("%.1f MB" % (vm.stat().st_size / 1e6)) if vm.is_file() else "-",
             len(errs), len(missed)))
    for e in errs[:5]:
        print("  " + e)
    for m in missed[:5]:
        print("  MISSING MEM: " + m)
    if ok:
        print("NETLIST %s" % p(vm))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
