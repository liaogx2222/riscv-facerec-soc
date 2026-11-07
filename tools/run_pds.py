"""Freeze each run (RTL, ROM, constraints, firmware) before invoking PDS."""
from pathlib import Path
import argparse, hashlib, json, os, re, shutil, subprocess, time
from clock_configuration import configuration,constrained_configuration
ROOT=Path(__file__).resolve().parents[1]   # = release/
p=argparse.ArgumentParser(); p.add_argument('name'); p.add_argument('--period',type=float); p.add_argument('--group-cpu',action='store_true'); p.add_argument('--profile',choices=['gate','coremark'],default='gate'); p.add_argument('--iterations',type=int,default=10000); p.add_argument('--benchmark-mode',choices=['validation','dual'],default='dual'); p.add_argument('--benchmark-base',type=Path,help='Isolated benchmark output base inside build/'); p.add_argument('--bitstream',action='store_true'); p.add_argument('--prepare-only',action='store_true'); p.add_argument('--rtl-root',type=Path,help='Isolated candidate RTL; current-source checks remain against production rtl/'); p.add_argument('--constraint-file',type=Path,help='Isolated candidate board constraint source'); p.add_argument('--synthesis-paths',type=int,default=200,help='Report enough synthesis paths to diagnose independent timing cones'); p.add_argument('--seed',type=int,help='Explicit documented PDS gplace_seed; frozen with the run');p.add_argument('--single-clock',type=float,metavar='MHZ',help='PLL bypass: one clock at MHZ drives both domains (define FACEREC_NO_PLL)'); a=p.parse_args()
assert a.name.replace('_','').isalnum() and (a.period is None or a.period>0)
assert a.synthesis_paths>0
assert a.seed is None or 0<=a.seed<2**31
out=ROOT/'build/pds'/a.name
out.mkdir(parents=True,exist_ok=False)
rtl_root=(a.rtl_root or ROOT/'rtl').resolve()
assert rtl_root.is_dir()
clock=configuration(rtl_root)
if a.single_clock:
    # PLL bypassed: both domains are driven by the pad clock, so the only
    # legal period is 1/single_clock. The PLL-based binding below does not apply.
    assert a.period is None or abs(a.period-1000.0/a.single_clock)<0.00001
    a.period=1000.0/a.single_clock
    clock=dict(clock_hz=int(round(a.single_clock*1e6)),period_ns=a.period,single_clock=True)
else:
    a.period=clock['period_ns'] if a.period is None else a.period
    assert abs(a.period-clock['period_ns'])<0.00001,'Constraint period must match actual PLL; change hardware frequency first'
shutil.copytree(rtl_root,out/'rtl')
inputs={str(Path('rtl')/x.relative_to(rtl_root)):hashlib.sha256(x.read_bytes()).hexdigest() for x in rtl_root.rglob('*') if x.is_file()}
constraint_file=(a.constraint_file or ROOT/'constraints/product/board.fdc').resolve()
assert constraint_file.is_file()
if a.single_clock:
    fdc_text=constraint_file.read_text(encoding='utf-8')
    assert re.search(r'create_clock\s+-name\s+sys_clk\s+-period\s+[\d.]+',fdc_text),'Single-clock build needs a sys_clk create_clock'
    got=float(re.search(r'create_clock\s+-name\s+sys_clk\s+-period\s+([\d.]+)',fdc_text)[1])
    assert abs(got-a.period)<0.00001,'27MHz FDC period %.6f does not match %.6f'%(got,a.period)
else:
    assert constrained_configuration(rtl_root,constraint_file)['consistent'],'Board FDC must match actual PLL'
if a.rtl_root or a.constraint_file:
    (out/'variant.json').write_text(json.dumps(dict(rtl_root=str(rtl_root),constraint_file=str(constraint_file),note='Candidate input; not selected production until hashes match.'),indent=2))
inputs['constraints/board.fdc']=hashlib.sha256(constraint_file.read_bytes()).hexdigest()
shutil.copy2(constraint_file,out/'input_board.fdc')
benchmark_image=None
profile_ram_addr_w=int(re.search(r'parameter\s+integer\s+RAM_ADDR_W\s*=\s*(\d+)',
                                 (out/'rtl/board/facerec_top.v').read_text(encoding='utf-8'))[1])
if a.profile=='coremark':
    base=ROOT/'build/coremark' if a.benchmark_mode=='validation' else ROOT/'build/coremark/dual'
    if a.benchmark_base:
        base=a.benchmark_base.resolve();assert base.is_relative_to(ROOT/'build')
    image=base/str(a.iterations)/'simple_ram.v'
    benchmark_image=image.relative_to(ROOT).as_posix()
    assert image.exists(), 'Run build_coremark.py for these iterations first'
    benchmark_build=json.loads((image.parent/'build.json').read_text())
    assert benchmark_build['clock_hz']==clock['clock_hz'],'Rebuild CoreMark for actual PLL frequency'
    profile_ram_addr_w=benchmark_build['ram_addr_w'];assert profile_ram_addr_w in [16,17]
    inputs[str(image.relative_to(ROOT))]=hashlib.sha256(image.read_bytes()).hexdigest()
    shutil.copy2(image,out/'rtl/soc/simple_ram.v')
    top=out/'rtl/board/facerec_top.v'; s=top.read_text(encoding='utf-8')
    pat=r'(parameter\s+integer\s+RAM_ADDR_W\s*=\s*)\d+'
    assert re.search(pat,s),'RAM_ADDR_W parameter not found in facerec_top'
    top.write_text(re.sub(pat,lambda m:m.group(1)+str(profile_ram_addr_w),s,count=1),encoding='utf-8',newline='\n')
if a.single_clock:
    top=out/'rtl/board/facerec_top.v'; s=top.read_text(encoding='utf-8')
    hz=str(int(round(a.single_clock*1e6)))
    s2=re.sub(r'(parameter\s+integer\s+CLK_HZ\s*=\s*)[\d_]+',lambda m:m.group(1)+hz,s,count=1)
    assert s2!=s,'CLK_HZ default not found'
    s=s2
    s2=re.sub(r'(parameter\s+integer\s+CNN_CLK_HZ\s*=\s*)[\d_]+',lambda m:m.group(1)+hz,s,count=1)
    assert s2!=s,'CNN_CLK_HZ default not found'
    top.write_text(s2,encoding='utf-8',newline='\n')
    inputs['rtl/board/facerec_top.v']=hashlib.sha256(s2.encode()).hexdigest()
(out/'input_hashes.json').write_text(json.dumps(inputs,indent=2))
(out/'profile.json').write_text(json.dumps(dict(profile=a.profile,clock_hz=clock['clock_hz'],ram_addr_w=profile_ram_addr_w,iterations=a.iterations if a.profile=='coremark' else None,benchmark_mode=a.benchmark_mode if a.profile=='coremark' else None,benchmark_image=benchmark_image),indent=2))
shutil.copy2(rtl_root/'core/rv32i_defs.v',out/'rv32i_defs.v')
# The source constraints have already been validated against every PLL output.
# Preserve each domain's own period; replacing all periods with the CPU period
# silently over-constrains an independently clocked CNN.
fdc=constraint_file.read_text(encoding='utf-8')
if a.group_cpu:
    for inst in ['u_cpu','u_pf','u_ic','u_dc']:
        fdc+=f'\ndefine_attribute {{u_soc/{inst}}} {{PAP_GROUP}} {{cpu_group}}\n'
(out/'timing.fdc').write_text(fdc,encoding='utf-8')
if a.seed is not None:
    (out/'implementation_options.json').write_text(json.dumps(dict(gplace_seed=a.seed),indent=2))
files=(rtl_root/'files.f').read_text().splitlines()
tcl=['set_arch -family Logos2 -device PG2L200H -speedgrade -6 -package FBB676']
tcl += ['add_design {'+(out/x).as_posix()+'}' for x in files]
tcl += ['add_constraint {'+(out/'timing.fdc').as_posix()+'}',
        'compile -top_module facerec_top -system_verilog'
        + (' -compiler_directives "+define+FACEREC_NO_PLL"' if a.single_clock else ''),
        f'synthesize -ads -selected_syn_tool_opt 2 -max_path {a.synthesis_paths}','dev_map',
        'pnr -mode performance -number_of_router_thread 4'+(f' -gplace_seed {a.seed}' if a.seed is not None else ''),
        'report_timing -delay_type min_max -max_path 30']
if a.bitstream: tcl+=['gen_bit_stream -write_drm_data TRUE']
tcl+=['exit']
(out/'flow.tcl').write_text('\n'.join(tcl)+'\n')
manifest={str(x.relative_to(out)):hashlib.sha256(x.read_bytes()).hexdigest() for x in out.rglob('*') if x.is_file()}
(out/'source_hashes.json').write_text(json.dumps(manifest,indent=2))
if a.prepare_only:
    print(f'Frozen inputs prepared; execute PDS with -file {out/"flow.tcl"} -work_dir {out}')
    raise SystemExit(0)
pds=os.environ.get('PDS_SHELL','E:/PDS_2022.2-SP6.4/bin/pds_shell.exe')
start=time.time()
with (out/'console.log').open('w') as log:
    from pds_power import active_pds
    with active_pds():
        r=subprocess.run([pds,'-file',str(out/'flow.tcl'),'-work_dir',str(out)],cwd=out,stdout=log,stderr=subprocess.STDOUT)
(out/'exit.json').write_text(json.dumps(dict(returncode=r.returncode,seconds=time.time()-start,started_unix=start,period=a.period),indent=2))
print(f'PDS exit={r.returncode}; reports: {out}')
raise SystemExit(r.returncode)
