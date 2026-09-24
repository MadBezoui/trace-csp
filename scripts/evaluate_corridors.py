"""Certificate completeness and cost on multi-ramp corridors (parallel, streamed)."""
import argparse
import json
from multiprocessing import Pool
from time import perf_counter
from trace_csp.corridor import generate_corridor
from trace_csp.csp.solver import solve
from trace_csp.graph.physics import certified_core,interval_core,core_is_sufficient


def run(args):
    seed,cells,horizon,seconds=args
    i=generate_corridor(seed,cells=cells,horizon=horizon,time_limit_s=seconds)
    t=perf_counter(); cap=certified_core(i); t_cap=perf_counter()-t
    t=perf_counter(); box=interval_core(i); t_box=perf_counter()-t
    r=solve(i)
    row={'seed':seed,'cells':cells,'horizon':horizon,'family':i.family,'status':r.status,'solver_wall_s':r.wall_time,
         'first_feasible_s':r.first_feasible_s,'capacity_core':cap,'interval_core':box,'capacity_s':t_cap,'interval_s':t_box,
         'solver_core':r.unsat_core}
    if box: row['interval_core_sufficient']=core_is_sufficient(i,[box],seconds=30)
    joint=interval_core(i,joint_merge=True); row['joint_core']=joint
    if joint and joint!=box: row['joint_core_sufficient']=core_is_sufficient(i,[joint],seconds=30)
    return row


def main():
    p=argparse.ArgumentParser(); p.add_argument('--output',required=True); p.add_argument('--count',type=int,default=800)
    p.add_argument('--start',type=int,default=100000); p.add_argument('--cells',type=int,default=10)
    p.add_argument('--horizon',type=int,default=6); p.add_argument('--seconds',type=float,default=10.)
    p.add_argument('--workers',type=int,default=8)
    a=p.parse_args()
    jobs=[(s,a.cells,a.horizon,a.seconds) for s in range(a.start,a.start+a.count)]
    with Pool(a.workers) as pool, open(a.output,'w') as f:
        for row in pool.imap_unordered(run,jobs):
            f.write(json.dumps(row)+'\n'); f.flush()


if __name__=='__main__': main()
