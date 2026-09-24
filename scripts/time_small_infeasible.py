"""Time CP-SAT (final encoding) and the certificates on held-out small infeasible instances."""
import argparse
import json
from time import perf_counter
import numpy as np
from trace_csp.data import read_instances
from trace_csp.csp.solver import solve
from trace_csp.graph.physics import certified_core,interval_core


def main():
    p=argparse.ArgumentParser(); p.add_argument('--output',required=True)
    p.add_argument('--instances',default='data/processed/synthetic-12000-v3.jsonl.gz')
    a=p.parse_args(); rows=[]
    for i in read_instances(a.instances):
        if i.split!='test_id' or i.family!='compound': continue
        t=perf_counter(); r=solve(i); cp=perf_counter()-t
        t=perf_counter(); c=certified_core(i); tc=perf_counter()-t
        t=perf_counter(); b=interval_core(i); tb=perf_counter()-t
        rows.append({'id':i.instance_id,'status':r.status,'cpsat_s':cp,'capacity_s':tc,'interval_s':tb,
                     'core':r.unsat_core,'capacity_core':c,'interval_core':b})
    summary={'instances':len(rows),'status':{s:sum(r['status']==s for r in rows) for s in {r['status'] for r in rows}},
             'median_cpsat_s':float(np.median([r['cpsat_s'] for r in rows])),
             'median_capacity_s':float(np.median([r['capacity_s'] for r in rows])),
             'median_interval_s':float(np.median([r['interval_s'] for r in rows])),
             'capacity_certified':sum(r['capacity_core'] is not None for r in rows),
             'interval_certified':sum(r['interval_core'] is not None for r in rows),'rows':rows}
    with open(a.output,'w') as f: json.dump(summary,f,indent=1)
    print(json.dumps({k:v for k,v in summary.items() if k!='rows'},indent=1))


if __name__=='__main__': main()
