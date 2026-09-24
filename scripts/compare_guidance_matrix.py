"""Run paired CP-SAT, warm-start and learned-guidance smoke comparisons."""
import argparse
import json
import subprocess
import sys
from pathlib import Path
from statistics import median


def main():
    p=argparse.ArgumentParser();p.add_argument('--input',required=True);p.add_argument('--output-dir',default='results')
    p.add_argument('--tag',required=True);p.add_argument('--architecture',default='hgt',choices=('hgt','mlp','bipartite'))
    p.add_argument('--seeds',type=int,default=5);p.add_argument('--limit',type=int,default=30)
    p.add_argument('--splits',nargs='+',default=['test_id','test_ood_structure']); p.add_argument('--checkpoint-dir')
    a=p.parse_args()
    if a.seeds<1 or a.limit<1:p.error('seeds and limit must be positive')
    output=Path(a.output_dir);output.mkdir(parents=True,exist_ok=True)
    summary={'scope':'first held-out synthetic instances per split; end-to-end deadline included',
             'architecture':a.architecture,'limit':a.limit,'runs':[]}
    for split in a.splits:
        for seed in range(a.seeds):
            checkpoint=Path(a.checkpoint_dir or output)/f'{a.architecture}-{a.tag}-seed{seed}.pt'
            path=output/f'solver-{a.tag}-{a.architecture}-seed{seed}-{split}.json'
            cmd=[sys.executable,'-m','trace_csp.cli','experiment','--input',a.input,'--split',split,
                 '--limit',str(a.limit),'--checkpoint',str(checkpoint),'--output',str(path)]
            run=subprocess.run(cmd,capture_output=True,text=True)
            if run.returncode: raise RuntimeError(f'{split} seed {seed}: {run.stderr}')
            result=json.loads(path.read_text())
            stats={}
            for controller in result['controllers']:
                rows=[row for row in result['runs'] if row['controller']==controller]
                stats[controller]={'attempted':len(rows),'feasible_at_deadline':sum(row['feasible'] for row in rows),
                    'verified_feasible':sum(row['verified_feasible'] for row in rows),
                    'median_end_to_end_s':median(row['end_to_end_wall_s'] for row in rows),
                    'median_tts':median(row['tts'] for row in rows if row['tts'] is not None) if any(row['tts'] is not None for row in rows) else None}
            summary['runs'].append({'split':split,'seed':seed,'controllers':stats,'paired_runtime':result['paired_runtime']})
            print(json.dumps({'split':split,'seed':seed,'controllers':stats}),flush=True)
    (output/f'guidance-comparison-{a.tag}.json').write_text(json.dumps(summary,indent=2))

if __name__=='__main__':main()
