"""Coverage of the joint merge-cell refinement on the 800 corridors of results/final."""
import json
from time import perf_counter
import numpy as np
from trace_csp.corridor import generate_corridor
from trace_csp.graph.physics import interval_core,core_is_sufficient

rows=[json.loads(l) for l in open('results/final/certificates-10x6.jsonl')]
out=[]
for r in rows:
    i=generate_corridor(r['seed'],cells=r['cells'],horizon=r['horizon'])
    t=perf_counter(); core=interval_core(i,joint_merge=True); dt=perf_counter()-t
    new=core is not None and not r['interval_core']
    out.append({'seed':r['seed'],'status':r['status'],'joint_core':core,'joint_s':dt,'new':new,
                'new_sufficient':core_is_sufficient(i,[core],seconds=30) if new else None})
json.dump(out,open('results/final/joint-merge-coverage.json','w'),indent=1)
inf=[o for o in out if o['status']=='INFEASIBLE']; feas=[o for o in out if o['status'] in ('OPTIMAL','FEASIBLE')]
print('infeasible certified',sum(o['joint_core'] is not None for o in inf),'/',len(inf),
      '| false on proven-feasible',sum(o['joint_core'] is not None for o in feas),'/',len(feas),
      '| unknown fired',sum(o['joint_core'] is not None for o in out if o['status']=='UNKNOWN'),
      '| new',sum(o['new'] for o in out),'all new sufficient',all(o['new_sufficient'] for o in out if o['new']),
      '| median ms %.2f'%(1000*np.median([o['joint_s'] for o in out])))
