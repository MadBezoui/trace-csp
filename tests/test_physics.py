from trace_csp.data import generate_instance
from trace_csp.csp.solver import solve
from trace_csp.graph.physics import certified_core,core_is_sufficient,queue_lower_bounds


def test_certified_core_is_solver_sufficient_on_compound_conflict():
    i=generate_instance(6,family='compound')
    assert solve(i).status=='INFEASIBLE'
    core=certified_core(i)
    assert core is not None and core_is_sufficient(i,[core]) is True


def test_no_certificate_on_feasible_instance_and_empty_guard_set_is_insufficient():
    i=generate_instance(0,family='normal')
    assert solve(i).actions is not None
    assert certified_core(i) is None
    assert core_is_sufficient(i,[]) is False


def test_queue_bound_never_exceeds_optimal_solver_queues():
    for seed in range(8):
        i=generate_instance(seed); r=solve(i)
        if r.actions is None: continue
        for name,bound in queue_lower_bounds(i).items(): assert r.values[name]>=bound


def test_interval_bounds_contain_solver_trajectories_on_corridors():
    from trace_csp.corridor import generate_corridor
    from trace_csp.graph.physics import interval_reach
    for seed in (1,2,5,9):
        i=generate_corridor(seed,cells=6,horizon=3,time_limit_s=5)
        r=solve(i)
        if r.actions is None: continue
        for name,(lo,hi) in interval_reach(i).items(): assert lo<=r.values[name]<=hi,name


def test_interval_certificate_is_solver_sufficient_and_tightens_capacity_bound():
    from trace_csp.corridor import generate_corridor
    from trace_csp.graph.physics import interval_core
    found=0
    for seed in range(60):
        i=generate_corridor(seed,cells=6,horizon=4,family='incident',time_limit_s=5)
        core=interval_core(i)
        if core is None: continue
        assert core_is_sufficient(i,[core]) is True
        found+=certified_core(i) is None
    assert found>0
