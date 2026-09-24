import random
from dataclasses import replace
from trace_csp.corridor import generate_corridor
from trace_csp.domain import Action
from trace_csp.traffic.ctm import step
from trace_csp.csp.verifier import verify_plan
from trace_csp.csp.solver import solve
from trace_csp.graph.physics import interval_reach


def with_offramps(seed,cells=6,horizon=3):
    i=generate_corridor(seed,cells=cells,horizon=horizon,time_limit_s=5)
    rng=random.Random(seed); c=i.corridor
    exits=tuple(rng.choice((0,0,120,250,400)) for _ in range(cells))
    ramp_min=(c.ramp_max[0],)+(0,)*(len(c.ramp_max)-1)  # first ramp unmetered
    prev=replace(i.state.previous,ramp=(c.ramp_max[0],)+i.state.previous.ramp[1:])
    return replace(i,corridor=replace(c,exit_permille=exits,ramp_min=ramp_min),state=replace(i.state,previous=prev))


def test_ctm_and_verifier_agree_and_conserve_with_offramps():
    for seed in range(12):
        i=with_offramps(seed); c=i.corridor; rng=random.Random(seed)
        plan=tuple(Action(tuple(rng.randint(lo,hi) for lo,hi in zip(c.ramp_min,c.ramp_max)),i.state.previous.vsl) for _ in range(i.config.horizon))
        cert=verify_plan(i,plan,queue_slack=10**6)
        assert 'mass conservation' not in cert.violations
        for wi,w in enumerate(i.scenarios):
            s=i.state
            for t,a in enumerate(plan):
                tr=step(c,s,a,w,t,i.config); s=tr.state
                assert s==cert.trajectories[wi][t+1]
                assert sum(tr.offramp)>=0


def test_solver_replays_exactly_and_respects_unmetered_ramp():
    for seed in range(6):
        i=with_offramps(seed); r=solve(i,relax='lexicographic')
        assert r.actions is not None
        assert all(a.ramp[0]==i.corridor.ramp_max[0] for a in r.actions)


def test_interval_bounds_contain_solver_states_with_offramps():
    for seed in range(6):
        i=with_offramps(seed); r=solve(i,relax='lexicographic')
        for name,(lo,hi) in interval_reach(i).items(): assert lo<=r.values[name]<=hi,name


def test_congestion_surrogate_objective_matches_verifier():
    for seed in range(4):
        i=with_offramps(seed); i=replace(i,config=replace(i.config,congestion_penalty=True))
        r=solve(i,relax='lexicographic')
        cert=verify_plan(i,r.actions,queue_slack=10**6)
        assert r.status=='OPTIMAL' and len(cert.objectives)==5
        weights=[w.weight for w in i.scenarios]
        over=sum(weights[int(k.split('/')[1])]*v for k,v in r.values.items() if k.startswith('over_critical/'))
        assert over==cert.objectives[0]


def with_drop(seed):
    i=with_offramps(seed); return replace(i,corridor=replace(i.corridor,capacity_drop_permille=150))


def test_capacity_drop_is_consistent_across_ctm_solver_verifier_and_intervals():
    for seed in range(8):
        i=with_drop(seed); c=i.corridor; rng=random.Random(seed)
        plan=tuple(Action(tuple(rng.randint(lo,hi) for lo,hi in zip(c.ramp_min,c.ramp_max)),i.state.previous.vsl) for _ in range(i.config.horizon))
        cert=verify_plan(i,plan,queue_slack=10**6)
        for wi,w in enumerate(i.scenarios):
            s=i.state
            for t,a in enumerate(plan):
                s=step(c,s,a,w,t,i.config).state; assert s==cert.trajectories[wi][t+1]
        r=solve(i,relax='lexicographic')  # raises if the solver and the verifier disagree
        for name,(lo,hi) in interval_reach(i).items(): assert lo<=r.values[name]<=hi,name


def test_joint_merge_bounds_contain_solver_states():
    for maker in (with_offramps,with_drop):
        for seed in range(8):
            i=maker(seed); r=solve(i,relax='lexicographic')
            for name,(lo,hi) in interval_reach(i,joint_merge=True).items(): assert lo<=r.values[name]<=hi,(seed,name)
    from trace_csp.corridor import generate_corridor
    for seed in range(20):
        i=generate_corridor(seed,cells=6,horizon=4,time_limit_s=5); r=solve(i,relax='lexicographic')
        for name,(lo,hi) in interval_reach(i,joint_merge=True).items(): assert lo<=r.values[name]<=hi,(seed,name)
