"""Rebuild every logged shield decision of an I-210 run and check it independently.

For each decision the CTM instance is rebuilt from the logged state, demand and exit
shares, and the exact verifier checks the executed command held over its 30-s period:
- nominal and fallback commands against the hard queue limits;
- degraded commands against the queue limits plus the exceedance the decision declared
  (planned_excess for an optimized plan, first_period_excess for a held command), so the
  declared value must bound the replayed violation. The declared planned exceedance must
  also agree with the recorded exceedance stage: not above its objective, and equal to it
  when the stage is marked as proved minimal.
The interval certificate is recomputed and compared with the logged one, and the requested
command must equal the executed one. Missing fields count as failures.

Runs recorded before release 1.2.0 did not log first_period_excess. With
--allow-undeclared-hold such held commands are verified without a declared bound and counted
separately instead of failing.

This replay establishes that each executed command is consistent with its recorded model
inputs and declared bound. It does not re-prove the optimality of a full-horizon plan, and
it says nothing about the microscopic simulation itself. The process exits with status 1
if any decision fails. Usage: replay_i210_decisions.py results/i210-final/shield-...json
"""
import argparse
import json
import sys
from collections import Counter
from dataclasses import replace
from trace_csp.domain import Action,State,Corridor,Config,Instance
from trace_csp.csp.verifier import verify
from trace_csp.graph.physics import interval_core
from trace_csp.sumo_i210 import demand_scenarios

NOMINAL=('trace-csp','fallback-reference','shifted','fallback')
RELAXED=('degraded','soft-mpc')
REQUIRED=('mode','executed','requested','state','boundary_in','ramp_demand','exit_permille')


def corridor_of(d):
    d=dict(d); d['release_table']=tuple(tuple(tuple(p) for p in table) for table in d['release_table'])
    return Corridor(**{k:tuple(v) if isinstance(v,list) else v for k,v in d.items()})


def config_of(d):
    d=dict(d); d['vsl_values']=tuple(d['vsl_values']); return Config(**d)


def instance_of(report,row):
    base=corridor_of(report['model']['corridor']); cfg=config_of(report['model']['config'])
    vsl=(cfg.vsl_values[0],)*len(base.jam); s=row['state']
    c=replace(base,exit_permille=tuple(row['exit_permille']))
    i=Instance(c,State(tuple(s['vehicles']),tuple(s['queues']),0,Action(tuple(s['previous']),vsl)),
               demand_scenarios(row['boundary_in'],row['ramp_demand'],cfg.horizon,cfg.control_block,len(c.jam)),cfg)
    return i,Action(tuple(row['executed']),vsl)


def declared_bound(row,allow_undeclared_hold=False):
    """Exceedance the decision declared for its command, or a failure message."""
    if row['mode'] in NOMINAL: return 0,None
    if row['mode'] in RELAXED:
        bound=row.get('planned_excess')
        if not isinstance(bound,int): return None,'missing planned_excess'
        stage=next((r for r in row.get('stages') or [] if r.get('stage')=='max_excess'),None)
        if stage and 'objective' in stage:
            if bound>round(stage['objective']): return None,'planned_excess above the exceedance stage objective'
            if row.get('exceedance_proved_minimal') and (stage.get('status')!='OPTIMAL' or bound!=round(stage['objective'])):
                return None,'exceedance marked minimal without a matching optimal stage'
        elif row.get('exceedance_proved_minimal'): return None,'exceedance marked minimal without a stage record'
        return bound,None
    if row['mode']=='degraded-hold':
        bound=row.get('first_period_excess')
        if isinstance(bound,int): return bound,None
        if allow_undeclared_hold and 'first_period_excess' not in row: return 10**9,None
        return None,'missing first_period_excess for a held command'
    return None,f"unknown mode {row['mode']}"


def replay(report,allow_undeclared_hold=False):
    counts=Counter(); failures=[]
    for row in report['decisions']:
        counts['decisions']+=1
        missing=[k for k in REQUIRED if k not in row]
        if missing: failures.append((row.get('t'),'missing fields '+', '.join(missing))); continue
        bound,problem=declared_bound(row,allow_undeclared_hold)
        if problem: failures.append((row['t'],problem)); continue
        i,executed=instance_of(report,row)
        cert=verify(i,executed,queue_slack=bound)
        if cert.safe: counts['verified']+=1
        else: failures.append((row['t'],'verifier rejected the executed command: '+'; '.join(cert.violations[:3])))
        if row['requested'] is not None and row['requested']!=row['executed']: failures.append((row['t'],'requested command differs from executed command'))
        if row.get('trigger')=='certificate' and interval_core(i)!=row.get('certified_core'): failures.append((row['t'],'certificate differs from the logged one'))
        if row['mode'] in RELAXED: counts['declared_bound_positive' if bound>0 else 'declared_bound_zero']+=1
        if bound==10**9: counts['held_without_declared_bound']+=1
    counts['failures']=len(failures)
    return dict(counts),failures


def main():
    p=argparse.ArgumentParser(); p.add_argument('reports',nargs='+')
    p.add_argument('--allow-undeclared-hold',action='store_true',help='accept held commands logged before first_period_excess existed')
    a=p.parse_args(); failed=False
    for path in a.reports:
        report=json.load(open(path))
        if not report.get('decisions') or 'state' not in report['decisions'][0]:
            print(path,'no shield decisions to replay'); continue
        counts,failures=replay(report,a.allow_undeclared_hold); failed|=bool(failures)
        print(path,json.dumps(counts))
        for t,why in failures[:10]: print(f'  t={t}: {why}')
    sys.exit(1 if failed else 0)


if __name__=='__main__': main()
