"""Seeded multi-ramp corridors with spillback conflicts.

Placeholder triangular fundamental diagram for 500 m, two-lane cells:
free speed 100 km/h, capacity 4,000 veh/h (11 veh per 10 s), jam 150 veh,
wave speed 16 km/h. These values await PeMS calibration.
"""
import random
from trace_csp.domain import Corridor,State,Action,Scenario,Config,Instance

CORRIDOR_FAMILIES=('normal','incident','surge','compound')

# Cell parameters for 500 m two-lane cells and 10 s steps.
# 'pems-i210': medians of per-station fits on observed (>=75%) I-210 mainline
# PeMS data, Jan+Apr 2026 weekdays (results/pems-fd-i210-calibration.csv):
# 107 km/h free speed, 1,580 veh/h/lane, 20 km/h wave, 90 veh/km/lane jam.
FUNDAMENTAL_DIAGRAMS={'placeholder':{'jam':150,'capacity':11,'wave':16,'congested':(95,140),'free':(10,45)},
                      'pems-i210':{'jam':90,'capacity':9,'wave':20,'congested':(57,84),'free':(6,27)}}


def generate_corridor(seed,cells=10,horizon=6,family=None,time_limit_s=10.,fd='placeholder'):
    rng=random.Random(seed); family=family or CORRIDOR_FAMILIES[seed%len(CORRIDOR_FAMILIES)]
    if family not in CORRIDOR_FAMILIES: raise ValueError('unknown corridor family')
    if cells<3: raise ValueError('at least three cells required')
    ramps=tuple(range(1,cells,2))
    p=FUNDAMENTAL_DIAGRAMS[fd]; jam=p['jam']
    c=Corridor((500,)*cells,(2,)*cells,(jam,)*cells,(p['capacity'],)*cells,ramps,(6,)*len(ramps),(15,)*len(ramps),wave_speed_kmh=p['wave'])
    cfg=Config(horizon=horizon,vsl_values=(60,80,100),max_dv=20,max_dm=2,spatial_dv=20,time_limit_s=time_limit_s,seed=seed)
    incident=family in ('incident','compound'); surge=family in ('surge','compound')
    location=rng.randrange(2,cells) if incident else None
    vehicles=tuple(rng.randint(*p['congested']) if incident and location-3<=j<=location else rng.randint(*p['free']) for j in range(cells))
    queues=tuple(rng.randint(8,14) if surge else rng.randint(0,10) for _ in ramps)
    speed=rng.choice(cfg.vsl_values)
    state=State(vehicles,queues,0,Action(tuple(rng.randint(2,5) for _ in ramps),(speed,)*cells))
    main=[rng.randint(6,10) for _ in range(horizon)]
    ramp=[[rng.randint(2,4) if surge else rng.randint(1,3) for _ in ramps] for _ in range(horizon)]
    severity=rng.choice((0,250,500)) if incident else 1000
    scenarios=[]
    for weight,delta,extra in ((1,-1,0),(2,0,0),(1,1,1)):
        caps=tuple(tuple(max(0,severity-150*extra) if j==location else 1000 for j in range(cells)) for _ in range(horizon))
        scenarios.append(Scenario(weight,tuple(max(0,x+delta) for x in main),
                                  tuple(tuple(max(0,x+extra) for x in row) for row in ramp),caps))
    group_index=seed//10; slot=group_index%10
    split='train' if slot<6 else 'validation' if slot<8 else 'test_id'
    return Instance(c,state,tuple(scenarios),cfg,f'corridor{cells}-h{horizon}-{seed:06d}',f'corridor-date-{group_index:05d}',family,split)
