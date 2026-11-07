"""Execute a run frozen with run_pds.py --prepare-only, preserving exit evidence."""
from pathlib import Path
import argparse, datetime, hashlib, json, os, subprocess, time
ROOT=Path(__file__).resolve().parents[1]   # = release/
p=argparse.ArgumentParser(); p.add_argument('name'); p.add_argument('--through',choices=['full','map','place'],default='full'); a=p.parse_args()
assert a.name.replace('_','').isalnum()
out=ROOT/'build/pds'/a.name
for name,sha in json.loads((out/'source_hashes.json').read_text()).items():
    path=out/name.replace('\\','/')
    assert path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest()==sha, f'Frozen input changed: {name}'
assert not (out/'console.log').exists(), 'Run already started; use saved DB recovery instead'
flow=out/'flow.tcl'
if a.through in ['map','place']:
    lines=flow.read_text().splitlines()
    stop=next(i for i,line in enumerate(lines) if line.startswith('pnr '))
    assert lines[stop-1]=='dev_map'
    tail=[] if a.through=='map' else [lines[stop]+' -place_only']
    flow=out/f'flow_through_{a.through}.tcl'
    flow.write_text('\n'.join(lines[:stop]+tail+['exit'])+'\n')
cmd=[os.environ.get('PDS_SHELL','E:/PDS_2022.2-SP6.4/bin/pds_shell.exe'),'-file',str(flow),'-work_dir',str(out)]
start=time.time()
with (out/'console.log').open('w') as log:
    proc=subprocess.Popen(cmd,cwd=out,stdout=log,stderr=subprocess.STDOUT)
    (out/'start.json').write_text(json.dumps(dict(pid=proc.pid,command=cmd,through=a.through,
        executed_flow_sha256=hashlib.sha256(flow.read_bytes()).hexdigest(),
        started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat()),indent=2))
    from pds_power import active_pds
    with active_pds():code=proc.wait()
(out/'exit.json').write_text(json.dumps(dict(returncode=code,seconds=time.time()-start,started_unix=start,through=a.through),indent=2))
print(f'PDS exit={code}; reports: {out}')
if code==0 and a.through=='place':
    import sys
    screen=subprocess.run([sys.executable,str(ROOT/'tools/report_placement.py'),a.name],cwd=ROOT)
    raise SystemExit(screen.returncode)
raise SystemExit(code)
