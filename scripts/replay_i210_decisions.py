"""Rebuild every logged shield decision of an I-210 run and check it independently.

For each decision the CTM instance is rebuilt from the logged state, demand and exit
shares, then
- the executed command is verified, held over the update period, with the exact verifier
  (hard queue limits for nominal and fallback modes, reported exceedance otherwise);
- the interval certificate is recomputed and compared with the logged one;
- the requested command is compared with the executed command.
No SUMO run is needed. Usage: replay_i210_decisions.py results/i210-final/shield-...json
"""
import argparse
import json
from collections import Counter
from dataclasses import replace
from trace_csp.domain import Action,State,Corridor,Config,Instance
from trace_csp.csp.verifier import verify
from trace_csp.graph.physics import interval_core
from trace_csp.sumo_i210 import demand_scenarios

NOMINAL=('trace-csp','fallback-reference','shifted','fallback')


def corridor_of(d):
    d=dict(d); d['release_table']=tuple(tuple(tuple(p) for p in table) for table in d['release_table'])
    return Corridor(**{k:tuple(v) if isinstance(v,list) else v for k,v in d.items()})


def config_of(d):
    d=dict(d); d['vsl_values']=tuple(d['vsl_values']); return Config(**d)


def replay(report):
    base=corridor_of(report['model']['corridor']); cfg=config_of(report['model']['config'])
    vsl=(cfg.vsl_values[0],)*len(base.jam); block=cfg.control_block; counts=Counter()
    for row in report['decisions']:
        c=replace(base,exit_permille=tuple(row['exit_permille']))
        s=row['state']; prev=Action(tuple(s['previous']),vsl)
        i=Instance(c,State(tuple(s['vehicles']),tuple(s['queues']),0,prev),
                   demand_scenarios(row['boundary_in'],row['ramp_demand'],cfg.horizon,block,len(c.jam)),cfg)
        executed=Action(tuple(row['executed']),vsl)
        nominal=row['mode'] in NOMINAL
        cert=verify(i,executed,queue_slack=0 if nominal else 10**9)
        counts['decisions']+=1
        counts['verified' if cert.safe else 'rejected']+=1
        if row['requested'] is not None and row['requested']!=row['executed']: counts['requested_differs_from_executed']+=1
        if row.get('trigger')=='certificate' and interval_core(i)!=row['certified_core']: counts['certificate_mismatch']+=1
        if not nominal: counts['exceedance_'+('positive' if cert.max_queue_excess>0 else 'zero')]+=1
    return dict(counts)


def main():
    p=argparse.ArgumentParser(); p.add_argument('reports',nargs='+'); a=p.parse_args()
    for path in a.reports:
        print(path,json.dumps(replay(json.load(open(path)))))


if __name__=='__main__': main()
