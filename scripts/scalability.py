"""Scalability of the shield on long corridors: certificate cost, time to the first
verified incumbent, and the lexicographic stage reached within a 5 s deadline.

Shield setting: relaxation stages first (worst and total exceedance), then the
deviation from a reference plan, then the traffic objectives. The reference is the
held previous metering action (a stand-in for ALINEA's proposal).
"""
import argparse
import json
from time import perf_counter
from trace_csp.corridor import generate_corridor
from trace_csp.csp.builder import build
from trace_csp.csp.solver import solve
from trace_csp.csp.verifier import verify_plan
from trace_csp.graph.physics import interval_core


def main():
    p=argparse.ArgumentParser(); p.add_argument('--output',required=True)
    p.add_argument('--sizes',nargs='+',type=int,default=[25,50,100,200]); p.add_argument('--instances',type=int,default=10)
    p.add_argument('--deadline',type=float,default=5.); p.add_argument('--workers',type=int,default=2)
    a=p.parse_args(); rows=[]
    for cells in a.sizes:
        for k in range(a.instances):
            seed=300000+1000*cells+k
            i=generate_corridor(seed,cells=cells,horizon=6,time_limit_s=a.deadline,fd='pems-i210')
            from dataclasses import replace
            i=replace(i,config=replace(i.config,workers=a.workers))
            ref=list(i.state.previous.ramp)
            t=perf_counter(); build(i,relax='lexicographic',reference=ref,assume=False); t_build=perf_counter()-t
            t=perf_counter(); core=interval_core(i); t_cert=perf_counter()-t
            t=perf_counter(); r=solve(i,relax='lexicographic',reference=ref); t_solve=perf_counter()-t
            first=r.first_feasible_s
            t=perf_counter(); ok=bool(r.actions) and verify_plan(i,r.actions,queue_slack=10**9).safe; t_verify=perf_counter()-t
            from trace_csp.csp.controller import Controller
            ctl=Controller(mode='degrade'); ctl.reference=ref
            t=perf_counter(); held=ctl.best_held_action(i); t_fallback=perf_counter()-t
            fallback_excess=verify_plan(i,(held,)*i.config.horizon,queue_slack=10**9).max_queue_excess
            stages=[x['stage'] for x in r.trace]
            rows.append({'cells':cells,'ramps':len(i.corridor.ramp_segment),'seed':seed,'family':i.family,'variables':None,
                         'build_s':t_build,'certificate_s':t_cert,'certified':core is not None,'status':r.status,
                         'first_incumbent_s':first,'verify_s':t_verify,'verified':ok,'solve_wall_s':t_solve,
                         'last_stage_with_incumbent':stages[-1] if stages else None,'max_excess':r.max_queue_excess,
                         'fallback_s':t_fallback,'fallback_excess':fallback_excess})
            print(json.dumps({k_:rows[-1][k_] for k_ in ('cells','family','certified','status','first_incumbent_s','last_stage_with_incumbent')}),flush=True)
    json.dump(rows,open(a.output,'w'),indent=1)


if __name__=='__main__': main()
