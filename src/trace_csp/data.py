"""Seeded synthetic instances and portable JSONL; no pickle inputs.

Sensor and communication families perturb modeled demand uncertainty. They are
proxies, not a sensor-bias or delayed-actuator communication simulator.
"""
from dataclasses import asdict
from pathlib import Path
import gzip
import json
import random
from trace_csp.domain import Corridor,State,Action,Scenario,Config,Instance

FAMILIES=('normal','mild_incident','severe_incident','sensor_fault','demand_shift','communication_fault','compound','structural_ood')


def generate_instance(seed:int,horizon=2,family=None,demand_multiplier=1.0,incident_factor=None,
                      stress_queue_conflict=False,stress_ramp_index=0):
    rng=random.Random(seed); family=family or FAMILIES[seed%len(FAMILIES)]
    if family not in FAMILIES: raise ValueError('unknown scenario family')
    if stress_queue_conflict and family not in ('demand_shift','structural_ood','severe_incident'):
        raise ValueError('queue-conflict stress is reserved for OOD challenge families')
    n=3 if family=='structural_ood' else 2
    c=Corridor((500,)*n,(2,)*n,(80,)*n,(12,)*n,(0,n-1),(4,4),(20,20))
    if stress_queue_conflict and not 0<=stress_ramp_index<len(c.ramp_max):
        raise ValueError('invalid stress ramp index')
    cfg=Config(horizon=horizon,vsl_values=(60,80,100),time_limit_s=2,seed=seed)
    vehicles=tuple(rng.randint(15,35) for _ in range(n))
    queues=(18,18) if family=='compound' else tuple(rng.randint(1,8) for _ in range(2))
    if stress_queue_conflict:
        queues=list(queues); queues[stress_ramp_index]=18; queues=tuple(queues)
    state=State(vehicles,queues,0,Action((2,2),(100,)*n))
    base=[max(0,round(rng.randint(3,7)*demand_multiplier*(1.4 if family=='demand_shift' else 1))) for _ in range(horizon)]
    ramps=[tuple(rng.randint(6,8) if family=='compound' else rng.randint(1,3) for _ in range(2)) for _ in range(horizon)]
    if stress_queue_conflict:
        ramps=[tuple(max(6,value) if j==stress_ramp_index else value for j,value in enumerate(row)) for row in ramps]
    base_factor={'mild_incident':800,'severe_incident':500,'compound':450}.get(family,1000)
    factor=base_factor*(incident_factor if incident_factor is not None else 1000)//1000
    location=0 if family=='compound' else n-1 if seed%5==0 else rng.randrange(n)
    scenarios=[]
    for offset in (-1,1):
        uncertainty=3 if family=='sensor_fault' else 2 if family in ('communication_fault','compound') else 1
        main=tuple(max(0,x+offset*uncertainty+(2 if family=='communication_fault' and t>0 else 0))
                   for t,x in enumerate(base))
        rd=tuple(tuple(max(0,x+(1 if offset>0 else 0)) for x in row) for row in ramps)
        caps=[]
        for t in range(horizon):
            if family=='normal' and t>0: stage_factor=1000
            elif family=='mild_incident' and t>0: stage_factor=min(1000,factor+100)
            else: stage_factor=factor
            row=[]
            for j in range(n):
                if family=='compound' and j==location: row.append(0)
                elif j==location or family=='compound': row.append(max(0,stage_factor-50*(offset>0)))
                else: row.append(1000)
            caps.append(tuple(row))
        caps=tuple(caps)
        if stress_queue_conflict:
            cell=c.ramp_segment[stress_ramp_index]
            caps=tuple(tuple(0 if j==cell else value for j,value in enumerate(row)) for row in caps)
        scenarios.append(Scenario(1,main,rd,caps))
    # Split by entire synthetic date group, never neighbouring windows.
    group_index=seed//10; slot=group_index%10
    split='train' if slot<6 else 'validation' if slot<8 else 'test_id'
    if family=='structural_ood': split='test_ood_structure'
    elif family=='demand_shift': split='test_ood_demand'
    elif family=='severe_incident' and seed%5==0: split='test_ood_incident'
    # OOD events are assigned separate days to keep groups disjoint.
    group=f'{split.split("test_ood_")[-1] if split.startswith("test_ood_") else "id"}-date-{group_index:05d}'
    instance_id=f'synthetic-{seed:07d}' + (f'-queue-conflict-ramp-{stress_ramp_index}' if stress_queue_conflict else '')
    return Instance(c,state,tuple(scenarios),cfg,instance_id,group,family,split)


def instance_from_dict(d):
    corridor=dict(d['corridor'])
    if 'release_table' in corridor: corridor['release_table']=tuple(tuple(tuple(p) for p in table) for table in corridor['release_table'])
    c=Corridor(**{k:tuple(v) if isinstance(v,list) else v for k,v in corridor.items()})
    s=d['state']; a=Action(tuple(s['previous']['ramp']),tuple(s['previous']['vsl']))
    state=State(tuple(s['vehicles']),tuple(s['queues']),s['boundary_queue'],a)
    ws=tuple(Scenario(w['weight'],tuple(w['mainline']),tuple(map(tuple,w['ramps'])),tuple(map(tuple,w['capacity_factor']))) for w in d['scenarios'])
    cfg=dict(d['config']); cfg['vsl_values']=tuple(cfg['vsl_values'])
    return Instance(c,state,ws,Config(**cfg),**{k:d[k] for k in ('instance_id','group','family','split')})


def open_jsonl(path,mode):
    path=Path(path)
    if 'w' in mode: path.parent.mkdir(parents=True,exist_ok=True)
    return gzip.open(path,mode+'t',encoding='utf-8') if path.suffix=='.gz' else path.open(mode,encoding='utf-8')


def write_instances(path,instances):
    with open_jsonl(path,'w') as f:
        for row in instances: f.write(json.dumps(asdict(row),sort_keys=True)+'\n')


def read_instances(path):
    with open_jsonl(path,'r') as f:
        for line in f:
            if line.strip(): yield instance_from_dict(json.loads(line))
