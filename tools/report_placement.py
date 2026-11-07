"""Ask official report_place_timing for paths from an immutable placement DB."""
from pathlib import Path
import argparse,hashlib,json,re,shutil,subprocess,time,sys
ROOT=Path(__file__).resolve().parents[1]   # = release/
p=argparse.ArgumentParser();p.add_argument('run');p.add_argument('--max-path',type=int,default=100);a=p.parse_args()
assert a.run.replace('_','').isalnum() and a.max_path>0
run=ROOT/'build/pds'/a.run;db=run/'place_route/facerec_top_plc.adf'
assert db.exists() and not db.with_suffix('.adf.lock').exists(),'Placement checkpoint unavailable or live'
out=ROOT/'build/placement_timing'/a.run;out.mkdir(parents=True,exist_ok=False)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
source_hash=sha(db);saved=out/'placement.adf';shutil.copy2(db,saved)
shutil.copy2(run/'timing.fdc',out/'timing.fdc')
pcf=run/'device_map/facerec_top.pcf';assert pcf.exists();shutil.copy2(pcf,out/'pins.pcf')
flow=out/'report.tcl';database_report=out/'placement.rpt'
# PDS writes the readable report to <design>.rtr in work_dir. The explicitly
# named -report_file is a compressed timing database, not the text report.
report=out/'facerec_top.rtr'
flow.write_text('\n'.join(['set_arch -family Logos2 -device PG2L200H -speedgrade -6 -package FBB676',
    'add_constraint {'+(out/'timing.fdc').as_posix()+'}',
    'add_constraint {'+(out/'pins.pcf').as_posix()+'}',
    'report_place_timing -force_to_run -input_db_file {'+saved.as_posix()+'} -specify_db {'+saved.as_posix()+'} -report_file {'+database_report.as_posix()+'} -delay_type min_max -max_path '+str(a.max_path),
    'exit'])+'\n')
started=time.time()
with (out/'console.log').open('w') as log:
    result=subprocess.run(['E:/PDS_2022.2-SP6.4/bin/pds_shell.exe','-file',str(flow),'-work_dir',str(out)],cwd=out,stdout=log,stderr=subprocess.STDOUT)
assert sha(db)==source_hash,'Original placement checkpoint changed'
record=dict(scope='Placement-stage timing estimates for screening/path diagnosis, not final routed qualification',
    returncode=result.returncode,seconds=time.time()-started,run=a.run,source_checkpoint_sha256=source_hash,
    command_sha256=sha(flow),log_sha256=sha(out/'console.log'),report_exists=report.exists(),
    report_sha256=sha(report) if report.exists() else None,
    report=report.relative_to(ROOT).as_posix(),
    timing_database_sha256=sha(database_report) if database_report.exists() else None)
if report.exists():
    text=report.read_text(errors='replace')
    record['slacks']=[float(x) for x in re.findall(r'Slack \((?:MET|VIOLATED)\)\s+(-?[\d.]+)',text)]
    record['route_delay_values_ns']=[float(x) for x in re.findall(r'Route:\s*([\d.]+)ns',text)]
    record['clocks']={row[0]:dict(requested_mhz=float(row[1]),estimated_mhz=float(row[2]),setup_slack_ns=float(row[5]))
        for row in re.findall(r'^\s*(clk_\w+)\s+([\d.]+) MHz\s+([\d.]+) MHz\s+([\d.]+)\s+([\d.]+)\s+(-?[\d.]+)',text,re.M)}
    record['readable_paths_with_nonzero_estimated_route_delay']=bool(record['slacks'] and any(x>0 for x in record['route_delay_values_ns']))
(out/'scope.json').write_text(json.dumps(record,indent=2))
print(json.dumps({k:v for k,v in record.items() if k not in ['slacks','route_delay_values_ns']},indent=2))
print((out/'console.log').read_text(errors='replace')[-5000:])
if result.returncode==0 and record.get('readable_paths_with_nonzero_estimated_route_delay'):
    subprocess.run([sys.executable,str(ROOT/'tools/analyze_routed_paths.py'),a.run,
                    '--stage','placement','--report',str(report)],check=True)
raise SystemExit(result.returncode or (0 if record.get('readable_paths_with_nonzero_estimated_route_delay') else 1))
