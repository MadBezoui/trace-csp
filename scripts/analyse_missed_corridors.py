"""Characterize the proven-infeasible corridors that the interval certificate misses.

For each missed corridor: family, incident location relative to ramps, the solver's
core size, and whether ANY single queue guard is sufficient (checked by CP-SAT). If
no single guard is sufficient, infeasibility is genuinely multi-constraint and no
single-guard certificate can explain it.
"""
import json
from multiprocessing import Pool
from trace_csp.corridor import generate_corridor
from trace_csp.csp.builder import build
from trace_csp.graph.physics import core_is_sufficient,interval_reach


def analyse(row):
    i=generate_corridor(row['seed'],cells=row['cells'],horizon=row['horizon'])
    guards=sorted(build(i).assumption_names.values())
    single=[g for g in guards if core_is_sufficient(i,[g],seconds=30)]
    limits=i.corridor.queue_max; bounds=interval_reach(i)
    margin=max(bounds[f"q/{g.split('/',1)[1]}"][0]-limits[int(g.split('/')[3])] for g in guards)
    loc=[j for j,row_ in enumerate(i.scenarios[0].capacity_factor[0]) if row_<1000]
    return {'seed':row['seed'],'family':i.family,'incident_cells':loc,'ramp_cells':list(i.corridor.ramp_segment),
            'solver_core':row['solver_core'],'solver_core_size':len(row['solver_core']),
            'sufficient_single_guards':single,'best_interval_margin':margin}


if __name__=='__main__':
    rows=[json.loads(l) for l in open('results/final/certificates-10x6.jsonl')]
    missed=[r for r in rows if r['status']=='INFEASIBLE' and not r['interval_core']]
    with Pool(2) as p: out=p.map(analyse,missed)
    json.dump(out,open('results/final/missed-corridors.json','w'),indent=1)
    from collections import Counter
    print(len(out),'missed; families',Counter(o['family'] for o in out))
    print('single sufficient guard exists:',sum(bool(o['sufficient_single_guards']) for o in out),
          'solver core sizes',Counter(o['solver_core_size'] for o in out))
