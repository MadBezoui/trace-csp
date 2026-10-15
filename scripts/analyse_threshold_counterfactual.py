"""Same-state analysis of the storage margin on the I-210 evaluation seeds.

The 80 % and 100 % runs follow different closed-loop trajectories, so the difference of
their certificate counts is a comparison between two policies, not the number of conflicts
caused by the margin. This script answers the narrower same-state question. For every period
that the interval certificate proved infeasible in an 80 % run, it rebuilds the logged
instance, raises the limits of the metered ramps to their full storage and records whether
- the certificate still fires (the limits are infeasible even at full storage), or else
- a CP-SAT feasibility solve finds a plan meeting the full-storage limits (the conflict is
  due to the margin), proves the full-storage limits infeasible, or stays unresolved.
It also counts certified periods matched by seed and time in the two trajectories.
Writes results/i210-final/threshold-counterfactual.json.
"""
import argparse
import json
from dataclasses import replace
from pathlib import Path
from ortools.sat.python import cp_model
from trace_csp.csp.builder import build
from trace_csp.graph.physics import interval_core
from replay_i210_decisions import instance_of

SEEDS=(6,7,8,9,10)


def feasible(instance,seconds,workers):
    built=build(instance,assume=False)
    solver=cp_model.CpSolver(); solver.parameters.max_time_in_seconds=seconds; solver.parameters.num_search_workers=workers
    status=solver.solve(built.model)
    return {cp_model.OPTIMAL:'feasible',cp_model.FEASIBLE:'feasible',cp_model.INFEASIBLE:'infeasible'}.get(status,'unresolved')


def main():
    p=argparse.ArgumentParser(); p.add_argument('--seconds',type=float,default=60); p.add_argument('--workers',type=int,default=4)
    p.add_argument('--output',default='results/i210-final/threshold-counterfactual.json'); a=p.parse_args()
    layout=json.loads(Path('results/i210-layout.json').read_text())
    full=tuple(i['storage'] if i['kind']=='metered' else 10**6 for i in layout['inflows'])
    rows=[]; matched={'both':0,'only_80':0,'only_100':0}
    for seed in SEEDS:
        r80=json.loads(Path(f'results/i210-final/shield-halt-f0.8-s{seed}.json').read_text())
        r100=json.loads(Path(f'results/i210-final/shield-halt-f1.0-s{seed}.json').read_text())
        c80={d['t'] for d in r80['decisions'] if d.get('trigger')=='certificate'}
        c100={d['t'] for d in r100['decisions'] if d.get('trigger')=='certificate'}
        matched['both']+=len(c80&c100); matched['only_80']+=len(c80-c100); matched['only_100']+=len(c100-c80)
        for d in r80['decisions']:
            if d.get('trigger')!='certificate': continue
            i,_=instance_of(r80,d)
            i=replace(i,corridor=replace(i.corridor,queue_max=full))
            core=interval_core(i)
            outcome='certified' if core else feasible(i,a.seconds,a.workers)
            rows.append({'seed':seed,'t':d['t'],'core_80':d['certified_core'],'time_zero_80':d['certified_core'].split('/')[2]=='0',
                         'core_100':core,'outcome_at_full_storage':outcome})
            print(seed,d['t'],outcome,flush=True)
    summary={k:sum(r['outcome_at_full_storage']==k for r in rows) for k in ('certified','feasible','infeasible','unresolved')}
    summary.update(periods=len(rows),time_zero=sum(r['time_zero_80'] for r in rows),matched_periods=matched,
                   solver_seconds=a.seconds,solver_workers=a.workers)
    Path(a.output).write_text(json.dumps({'summary':summary,'periods':rows},indent=1))
    print(json.dumps(summary))


if __name__=='__main__': main()
