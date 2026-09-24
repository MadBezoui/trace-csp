from dataclasses import replace
from itertools import product
import pytest

from trace_csp.domain import Corridor, State, Action, Scenario, Config, Instance
from trace_csp.traffic.ctm import step
from trace_csp.csp.solver import solve
from trace_csp.csp.verifier import verify, verify_plan


def tiny(horizon=2, scenarios=None):
    c = Corridor(length_m=(500,), lanes=(1,), jam=(20,), capacity=(6,),
                 ramp_segment=(0,), ramp_max=(2,), queue_max=(5,))
    cfg = Config(horizon=horizon, dt_s=10, vsl_values=(60, 100), max_dv=40,
                 max_dm=2, spatial_dv=40, time_limit_s=5)
    s = State((10,), (3,), 0, Action((1,), (100,)))
    scenarios = scenarios or (Scenario(1, (2,)*horizon, ((1,),)*horizon, ((1000,),)*horizon),)
    return Instance(c, s, scenarios, cfg)


def test_ctm_hand_computed_and_conservation():
    i = tiny()
    out = step(i.corridor, i.state, Action((2,), (100,)), i.scenarios[0], 0, i.config)
    # Send=floor(10*100/180)=5; receive=floor(10*20/180)=1.
    # Ramp has priority: release 1, boundary admits 0, exit 5.
    assert out.state.vehicles == (6,)
    assert out.state.queues == (3,)
    assert out.state.boundary_queue == 2
    assert out.outflow == (5,)
    assert sum(out.state.vehicles + out.state.queues) + out.state.boundary_queue + 5 == 16


def test_cfl_and_dimensions_rejected():
    with pytest.raises(ValueError, match='CFL'):
        replace(tiny(), config=replace(tiny().config, dt_s=100))
    with pytest.raises(ValueError):
        replace(tiny(), state=replace(tiny().state, vehicles=(1, 2)))
    with pytest.raises(ValueError):
        replace(tiny(), scenarios=())


def test_incident_zero_capacity_retains_vehicles():
    i = tiny(1, (Scenario(1, (2,), ((1,),), ((0,),)),))
    out = step(i.corridor, i.state, Action((2,), (100,)), i.scenarios[0], 0, i.config)
    assert out.state.vehicles == (10,)
    assert out.state.queues == (4,)
    assert out.state.boundary_queue == 2
    assert out.outflow == (0,)


def test_verifier_rejects_bad_controls_and_overflow():
    i = tiny()
    assert not verify(i, Action((9,), (100,))).safe
    assert not verify(i, Action((1,), (999,))).safe
    assert not verify(i, Action((), (100,))).safe
    flood = replace(i, scenarios=(Scenario(1, (2,2), ((9,),(9,)), ((0,),(0,))),))
    cert = verify(flood, Action((2,), (100,)))
    assert not cert.safe
    assert any('queue' in v for v in cert.violations)


def test_cp_sat_matches_exhaustive_and_independent_replay():
    i = tiny()
    outcomes = []
    for values in product(product(range(3), (60,100)), repeat=2):
        actions = tuple(Action((m,), (v,)) for m,v in values)
        cert = verify_plan(i, actions)
        if cert.safe:
            outcomes.append(cert.objectives)
    r = solve(i)
    assert r.status == 'OPTIMAL'
    assert r.objectives == min(outcomes)
    assert verify_plan(i, r.actions).safe
    assert r.trajectories[0][-1] == verify_plan(i, r.actions).trajectories[0][-1]


def test_robust_controls_shared_and_infeasible_explained():
    i = tiny(1, (Scenario(1, (0,), ((20,),), ((0,),)),))
    r = solve(i)
    assert r.status == 'INFEASIBLE'
    assert r.actions is None
    assert r.unsat_core


def test_timeout_never_reads_unassigned_variables():
    i = replace(tiny(), config=replace(tiny().config,time_limit_s=0))
    r = solve(i)
    assert r.status == 'UNKNOWN'
    assert r.actions is None


def test_infeasibility_core_excludes_irrelevant_assumptions():
    from trace_csp.data import generate_instance
    r=solve(generate_instance(6,horizon=1))
    assert r.status=='INFEASIBLE'
    assert len(r.unsat_core)==1
    assert '/1/0' in r.unsat_core[0]


def test_solver_exposes_constraint_activity_certificate():
    r=solve(tiny(horizon=1))
    assert r.audit['solver_status']=='OPTIMAL'
    assert r.audit['independently_verified'] is True
    assert any(name.startswith('initial:') for name in r.audit['active_constraints'])
    assert r.audit['hard_violations']==[]
    assert r.audit['optimality_gap']==0
