from dataclasses import replace
from trace_csp.data import generate_instance
from trace_csp.csp.solver import solve
from trace_csp.csp.controller import Controller
from trace_csp.csp.verifier import verify


def test_relaxation_has_zero_excess_exactly_when_hard_problem_is_feasible():
    for seed in range(16):
        i=generate_instance(seed); hard=solve(i); relaxed=solve(i,relax='lexicographic')
        assert relaxed.actions is not None
        assert (relaxed.max_queue_excess==0)==(hard.actions is not None)


def test_degraded_controller_acts_on_certified_infeasibility_and_reports_excess():
    i=generate_instance(6,family='compound')
    assert Controller().act(i).action is None
    d=Controller(mode='degrade').act(i)
    assert d.mode=='degraded' and d.audit['certified_core'] and d.audit['planned_max_queue_excess']>0
    assert verify(i,d.action,queue_slack=d.audit['max_queue_excess_first_step']).safe


def test_degraded_controller_matches_hard_controller_when_feasible():
    i=generate_instance(0,family='normal')
    assert Controller(mode='degrade').act(i).action==Controller().act(i).action


def test_soft_mpc_returns_verified_action_with_reported_excess():
    d=Controller(mode='soft').act(generate_instance(6,family='compound'))
    assert d.mode=='soft-mpc' and d.audit['max_queue_excess_first_step']>=0
