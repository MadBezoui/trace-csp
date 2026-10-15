"""Manifest of the NCHRP I-210 benchmark files read by the SUMO harness.

The benchmark was obtained as the GitHub archive of the pinned revision (no git metadata in
the copy), so its identity is recorded as the SHA-256 of every file the harness can read:
the controller configuration of the ALINEA benchmark folder and the whole SUMO scenario.
With --check, the input hashes stored in result files are compared with the manifest.
Writes results/i210-benchmark-manifest.json.
"""
import argparse
import json
from pathlib import Path
from trace_csp.provenance import manifest
from trace_csp.sumo_i210 import BENCHMARK

REVISION='eb8bbf2781f4a5c437ea9283901b2df78e6fd135'
SOURCE=f'https://codeload.github.com/NCHRP-BSM-Traffic-Measures/CAT-Data-For-Freeway-Operations/tar.gz/{REVISION}'


def main():
    p=argparse.ArgumentParser(); p.add_argument('--repository',default='data/raw/i210')
    p.add_argument('--output',default='results/i210-benchmark-manifest.json'); p.add_argument('--check',nargs='*',default=[])
    a=p.parse_args()
    repo=Path(a.repository).resolve(); base=repo/BENCHMARK; root=next(base.rglob('I210.sumocfg.xml')).parent
    scenario=[f for f in root.iterdir() if f.name!='Output' and not f.name.endswith('.ipynb')]
    files=manifest([*sorted(base.glob('*.json')),base/'freewayControl.py',*scenario],repo)
    Path(a.output).write_text(json.dumps({'source':SOURCE,'revision':REVISION,'files':files},indent=1))
    print(len(files),'files in the manifest')
    bad=0
    for path in a.check:
        report=json.loads(Path(path).read_text())
        for name,digest in report.get('provenance',{}).get('inputs',{}).items():
            if name.startswith('results/'): continue
            key=str(Path(name).resolve().relative_to(repo)) if Path(name).is_absolute() else None
            key=key or next((k for k in files if name.endswith(k)),None)
            if key not in files or files[key]!=digest: bad+=1; print('mismatch',path,name)
    if a.check: print('checked',len(a.check),'result files,',bad,'mismatches')
    raise SystemExit(1 if bad else 0)


if __name__=='__main__': main()
