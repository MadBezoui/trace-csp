"""Run a public I-210 ALINEA/HERO controller with local path/time overrides.

The upstream script is executed from a temporary copy. A short smoke run is
not a reproduction of the full seven-hour published benchmark.
"""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile


def find_tripinfo(root,seed,seconds):
    name=f'trace_tripinfo_{seed}_{seconds}.xml'
    candidates=(root/'Output'/name,root/f'Output_randomseed_{seed}'/name)
    for candidate in candidates:
        if candidate.is_file(): return candidate
    raise FileNotFoundError(f'completed tripinfo missing: {candidates}')


def main():
    p=argparse.ArgumentParser(); p.add_argument('--seconds',type=int,default=300)
    p.add_argument('--seed',type=int,default=5); p.add_argument('--output',required=True); p.add_argument('--tripinfo',action='store_true')
    p.add_argument('--algorithm',choices=('ALINEA','HERO'),default='ALINEA')
    p.add_argument('--repository',default='data/raw/i210'); p.add_argument('--parse-existing',action='store_true')
    a=p.parse_args()
    if a.seconds<=0: p.error('seconds must be positive')
    base=Path(a.repository).resolve()/'Ramp Metering'/f'run_fewIncidents_{a.algorithm}_benchmark'
    configs=list(base.rglob('I210.sumocfg.xml'))
    if len(configs)!=1: raise RuntimeError(f'expected one SUMO config under {base}, found {len(configs)}')
    root=configs[0].parent
    source=(base/'run.py').read_text()
    changes=[(r'(?m)^simProjDir = .*$',f'simProjDir = {str(root)+os.sep!r}'),
             (r'(?m)^randomSeed = .*$',f'randomSeed = {a.seed}'),
             (r'(?m)^totalTime = .*$',f'totalTime = {a.seconds}'),
             (r'isTripInfoSaved=True, tripInfoFile="tripInfo.xml"',f'isTripInfoSaved={a.tripinfo}, tripInfoFile="trace_tripinfo_{a.seed}_{a.seconds}.xml"')]
    for pattern,replacement in changes:
        source,count=re.subn(pattern,replacement,source)
        if count!=1: raise RuntimeError(f'upstream script layout changed: {pattern}')
    if a.algorithm=='HERO':
        source=source.split('# Save measurements from SUMO API.')[0]
    if a.parse_existing:
        if not a.tripinfo: p.error('--parse-existing requires --tripinfo')
        run=subprocess.CompletedProcess([],0,'','')
    else:
        with tempfile.TemporaryDirectory(prefix='trace-i210-') as d:
            script=Path(d)/'run.py'; script.write_text(source)
            env=os.environ.copy(); env['PYTHONPATH']=str(base)+os.pathsep+env.get('PYTHONPATH','')
            env['PATH']=str(Path(sys.executable).parent)+os.pathsep+env.get('PATH','')
            run=subprocess.run([sys.executable,str(script)],cwd=base,env=env,capture_output=True,text=True)
    output=Path(a.output); output.parent.mkdir(parents=True,exist_ok=True)
    report={'scenario':f'NCHRP I-210 {a.algorithm} benchmark','upstream_revision':'eb8bbf2781f4a5c437ea9283901b2df78e6fd135',
            'seconds_requested':a.seconds,'seed':a.seed,'returncode':run.returncode,'recovered_existing':a.parse_existing,'last_stdout':run.stdout[-3000:],'last_stderr':run.stderr[-3000:]}
    if a.tripinfo and run.returncode==0:
        import xml.etree.ElementTree as ET
        tripinfo=find_tripinfo(root,a.seed,a.seconds)
        count=0; duration=0.; waiting=0.
        for _,element in ET.iterparse(tripinfo,events=('end',)):
            if element.tag=='tripinfo':
                count+=1; duration+=float(element.get('duration','0')); waiting+=float(element.get('waitingTime','0'))
                element.clear()
        report['arrived_trips']=count; report['completed_trip_duration_s']=duration; report['completed_trip_waiting_s']=waiting
        report['tripinfo_path']=str(tripinfo)
    output.write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if not k.startswith('last_')},indent=2))
    if run.returncode: raise SystemExit(run.returncode)

if __name__=='__main__': main()
