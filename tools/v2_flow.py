#!/usr/bin/env python3
"""v2_flow.py —— 2025.2 布局布线签核流程（post_syn 导入 2022.2 综合网表）

为什么分成两段：PDS 2025.2 的综合器在本设计上段错误（rtl-data-opt 阶段
signal 11），所以综合必须用 2022.2 产出 `facerec_top_syn.vm`，再由 2025.2
以 `-project_type post_syn` 导入。只有 2025.2 的布线级 WNS 是签核结论。

四个会话（与 timing_opt/runs/v2_netlist_d1pin3 已验证流程一致）:
    build.tcl  create_project(post_syn) + SDC + add_design .vm + dev_map
    place.tcl  open_project + PCF 钉位 + place
    route.tcl  route + gen_bit_stream
    rt.tcl     report_timing

用法（在 release/ 下）:
    python tools/v2_flow.py rel_signoff \\
        --netlist ../timing_opt/runs/syn_release_clean/synthesize/facerec_top_syn.vm

产物: build/pnr/<name>/{build,place,route,rt}.log
      build/pnr/<name>/prj_tasks/pnr_1/report_timing/facerec_top.rtr   ← 签核报告
"""
import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]        # = release/
PDS = Path(os.environ.get("PDS_2025", r"E:\pango\PDS_2025.2-ads\bin\pds_shell.exe"))
# 许可文件与 hostid 绑定，不随仓库分发；请通过 PANGO_LICENSE_FILE 指定。
LIC = os.environ.get("PANGO_LICENSE_FILE", "")
HEAD = "set_arch -family Logos2 -device PG2L200H -speedgrade -6 -package FBB676"


def posix(p):
    return str(p).replace("\\", "/")


def run_tcl(run_dir, tcl_name, body, log_name, timeout=10800):
    (run_dir / tcl_name).write_text("\n".join(body) + "\nexit\n", encoding="utf-8")
    out_path = run_dir / log_name
    env = dict(os.environ)
    env["PANGO_LICENSE_FILE"] = LIC
    t0 = time.time()
    with out_path.open("w", encoding="utf-8") as out:
        try:
            rc = subprocess.call([str(PDS), "-file", str(run_dir / tcl_name),
                                  "-work_dir", str(run_dir)],
                                 cwd=run_dir, env=env, stdout=out,
                                 stderr=subprocess.STDOUT, timeout=timeout)
        except subprocess.TimeoutExpired:
            rc = -9
    txt = out_path.read_text(encoding="utf-8", errors="replace")
    errs = re.findall(r"^E: .*$", txt, re.M)
    ok = rc == 0 and not errs
    print("  %-10s rc=%-3s %6.1fs %s" % (tcl_name, rc, time.time() - t0,
                                        "OK" if ok else "FAIL " + str(errs[:2])))
    return ok, txt


def summary(rtr):
    """抽出签核需要的三行：Design Summary + 两个时钟的 WNS。"""
    txt = rtr.read_text(encoding="utf-8", errors="replace")
    design = re.search(r"Design Summary\s*:\s*(.+)", txt)
    rows = re.findall(r"^\s*(clk_sys|clk_cnn)\s+(clk_sys|clk_cnn)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)",
                      txt, re.M)
    perf = re.findall(r"^\s*(clk_sys|clk_cnn)\s+(\d+\.\d+) MHz\s+(\d+\.\d+) MHz\s+"
                      r"(\d+\.\d+)\s+(\d+\.\d+)\s+(-?\d+\.\d+)", txt, re.M)
    return dict(design=design[1].strip() if design else None,
                setup=[dict(launch=a, capture=b, wns=float(c), tns=float(d))
                       for a, b, c, d in rows],
                performance=[dict(clock=c, requested_mhz=float(r), estimated_mhz=float(e),
                                  requested_ns=float(p), estimated_ns=float(q), slack=float(s))
                             for c, r, e, p, q, s in perf])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("name")
    ap.add_argument("--netlist", required=True, type=Path, help="2022.2 综合出的 .vm")
    ap.add_argument("--sdc", type=Path,
                    default=ROOT / "constraints/product/signoff/timing_clean.sdc")
    ap.add_argument("--pcf", type=Path,
                    default=ROOT / "constraints/product/signoff/d1pin6.pcf")
    ap.add_argument("--reuse", action="store_true", help="沿用已有 .vm（不重新拷贝）")
    ap.add_argument("--direct-pnr", type=int, default=0, metavar="N",
                    help="用 pnr 命令直接跑 place+route，N=多种子次数（>1 走多种子并行）")
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()

    assert a.netlist.is_file(), a.netlist
    assert a.sdc.is_file(), a.sdc
    assert a.pcf.is_file(), a.pcf
    assert a.name.replace("_", "").isalnum()
    print("许可文件: %s" % LIC)

    run = ROOT / "build/pnr" / a.name
    run.mkdir(parents=True, exist_ok=True)
    if not a.reuse:
        shutil.copy2(a.netlist, run / "facerec_top_syn.vm")
    for src, dst in [(a.sdc, "timing_clean.sdc"), (a.pcf, "d1pin6.pcf")]:
        shutil.copy2(src, run / dst)

    print("run dir: %s" % run)
    ok, _ = run_tcl(run, "build.tcl", [
        "create_project {%s/flow.pds} -project_type post_syn -family Logos2 "
        "-device PG2L200H -speedgrade -6 -package FBB676 -force -in_process" % posix(run),
        HEAD,
        "add_constraint {%s/timing_clean.sdc}" % posix(run),   # 必须在 add_design 之前
        "add_design {%s/facerec_top_syn.vm} -verilog" % posix(run),
        "dev_map",
    ], "build.log", timeout=3600)
    if not ok:
        return 1

    if a.direct_pnr:
        # 多种子：gplace/groute 各跑 N 个种子，工具挑最好的一个落盘。
        # 布局是随机的，单种子在 clk_cnn 上会有 ±0.3 ns 的抖动。
        ok, _ = run_tcl(run, "place.tcl", [
            "open_project {%s/flow.pds}" % posix(run),
            "add_constraint {%s/d1pin6.pcf}" % posix(run),
            "pnr -gplace_seed %d -gplace_times %d -parallel 2 "
            "-groute_times %d -seed_step 1 -generate_bitstream FALSE"
            % (a.seed, a.direct_pnr, a.direct_pnr),
            "gen_bit_stream",
        ], "place.log", timeout=14400)
    else:
        ok, _ = run_tcl(run, "place.tcl", [
            "open_project {%s/flow.pds}" % posix(run),
            "add_constraint {%s/d1pin6.pcf}" % posix(run),
            "launch_tasks [get_tasks {pnr_1}] -to_action place",
            "wait_on_tasks [get_tasks {pnr_1}] -to_action place",
        ], "place.log")
    if not ok:
        return 1

    if not a.direct_pnr:
        ok, _ = run_tcl(run, "route.tcl", [
            "open_project {%s/flow.pds}" % posix(run),
            "launch_tasks [get_tasks {pnr_1}] -to_action route",
            "wait_on_tasks [get_tasks {pnr_1}] -to_action route",
            "launch_tasks [get_tasks {pnr_1}] -to_action gen_bit_stream",
            "wait_on_tasks [get_tasks {pnr_1}] -to_action gen_bit_stream",
        ], "route.log")
        if not ok:
            return 1

    # route/gen_bit_stream 阶段已经自动跑过 report_timing，报告是最新的。
    # 想刷新一次时不能用 `clean -task X -action Y`：2025.2 的 clean 接受
    # `-task` 或 `-action` 之一（同时给会报 Flow-0153），正确写法是
    #   clean -task [get_tasks {pnr_1}]              ← 清掉该 task 的动作状态
    # 刷新成功就用新报告；失败也不算失败，沿用 route 阶段的报告。
    run_tcl(run, "rt.tcl", [
        "open_project {%s/flow.pds}" % posix(run),
        "clean -task [get_tasks {pnr_1}]",
        "launch_tasks [get_tasks {pnr_1}] -to_action report_timing",
        "wait_on_tasks [get_tasks {pnr_1}] -to_action report_timing",
    ], "rt.log")

    rtr = run / "prj_tasks/pnr_1/report_timing/facerec_top.rtr"
    assert rtr.is_file(), "缺少签核报告: %s" % rtr
    info = summary(rtr)
    (run / "timing_summary.json").write_text(json.dumps(info, indent=2), encoding="utf-8")
    print("\nDesign Summary : %s" % info["design"])
    for p in info["performance"]:
        print("  %-8s 请求 %.2f MHz / %.4f ns → 实测 %.2f MHz / %.4f ns，slack %+0.3f ns"
              % (p["clock"], p["requested_mhz"], p["requested_ns"],
                 p["estimated_mhz"], p["estimated_ns"], p["slack"]))
    print("报告: %s" % rtr)

    sbit = run / "prj_tasks/pnr_1/generate_bitstream/facerec_top.sbit"
    if sbit.is_file():
        print("码流: %s (%d B, md5 %s)"
              % (sbit, sbit.stat().st_size, hashlib.md5(sbit.read_bytes()).hexdigest()))
    return 0 if (info["design"] or "").startswith("All Constraints Met") else 2


if __name__ == "__main__":
    sys.exit(main())
