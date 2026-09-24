"""Paired analysis of synthetic closed-loop episodes.

Halted episodes are reported separately. Total time spent is compared only on
cells where both controllers complete every period, so early halts cannot look
like lower congestion.
"""
import argparse
import glob
import json
from collections import defaultdict
import numpy as np
from trace_csp.data import open_jsonl
from trace_csp.evaluation.statistics import paired_summary,holm


def main():
    p=argparse.ArgumentParser(); p.add_argument('--pattern',required=True); p.add_argument('--output',required=True)
    p.add_argument('--reference',default='cp_sat')
    a=p.parse_args()
    cells=defaultdict(dict)
    for path in sorted(glob.glob(a.pattern)):
        with open_jsonl(path,'r') as f:
            for line in f:
                r=json.loads(line); key=(r['family'],r['demand_multiplier'],r['incident_factor'],r['seed'])
                cells[key][r['controller']]={k:r[k] for k in ('steps','requested_steps','halt_reason','total_time_spent',
                    'throughput','worst_ramp_waiting','ramp_waiting_gini','fallback_count','feasible_at_deadline')}
    controllers=sorted({c for v in cells.values() for c in v})
    overall={}; by_family=defaultdict(dict)
    for c in controllers:
        rows=[v[c] for v in cells.values() if c in v]
        overall[c]={'episodes':len(rows),'halted':sum(r['halt_reason'] is not None for r in rows),
                    'completed_steps':sum(r['steps'] for r in rows),'requested_steps':sum(r['requested_steps'] for r in rows),
                    'fallbacks':sum(r['fallback_count'] for r in rows)}
        for fam in sorted({k[0] for k in cells}):
            fr=[v[c] for k,v in cells.items() if k[0]==fam and c in v]
            by_family[fam][c]={'halted':sum(r['halt_reason'] is not None for r in fr),'episodes':len(fr)}
    tests={}
    for metric in ('total_time_spent','worst_ramp_waiting','throughput'):
        for c in controllers:
            if c==a.reference: continue
            pairs=[(v[a.reference][metric],v[c][metric]) for v in cells.values()
                   if a.reference in v and c in v and v[a.reference]['halt_reason'] is None and v[c]['halt_reason'] is None]
            tests[f'{metric}:{c}-vs-{a.reference}']=paired_summary([y for _,y in pairs],[x for x,_ in pairs]) if len(pairs)>=2 else None
    for metric in ('total_time_spent','worst_ramp_waiting','throughput'):
        keys=[k for k in tests if k.startswith(metric+':') and tests[k]]
        for k,adj in zip(keys,holm([tests[k]['wilcoxon_p'] for k in keys])): tests[k]['holm_p']=adj
    both={c:sum(1 for v in cells.values() if a.reference in v and c in v and v[a.reference]['halt_reason'] is None and v[c]['halt_reason'] is None)
          for c in controllers}
    report={'scope':'synthetic CTM closed loop; not SUMO','cells':len(cells),'overall':overall,'halts_by_family':by_family,
            'paired_on_jointly_completed':both,'paired_tests_controller_minus_reference':tests}
    with open(a.output,'w') as f: json.dump(report,f,indent=2)
    print(json.dumps({k:report[k] for k in ('cells','overall','paired_on_jointly_completed')},indent=1))
    for k,v in tests.items():
        if v: print(k,'n=%d mean=%.1f ci=[%.1f,%.1f] holm_p=%.3g'%(v['n'],v['mean_difference'],*v['ci95'],v['holm_p']))


if __name__=='__main__': main()
