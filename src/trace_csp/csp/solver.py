from dataclasses import dataclass, field
from time import perf_counter
from ortools.sat.python import cp_model
from trace_csp.domain import Action, State
from trace_csp.csp.builder import build
from trace_csp.csp.verifier import verify_plan


@dataclass
class SolveResult:
    status: str
    actions: tuple[Action, ...] | None = None
    objectives: tuple[int, ...] | None = None
    trajectories: tuple = ()
    wall_time: float = 0
    first_feasible_s: float | None = None
    best_bound: float | None = None
    objective_value: float | None = None
    objective_stage: str | None = None
    gap: float | None = None
    branches: int = 0
    conflicts: int = 0
    unsat_core: list[str] = field(default_factory=list)
    values: dict[str,int] = field(default_factory=dict)
    trace: list[dict] = field(default_factory=list)
    audit: dict = field(default_factory=dict)
    max_queue_excess: int | None = None
    # One record per lexicographic stage: proof status, incumbent value and best bound.
    stages: list[dict] = field(default_factory=list)


class Trace(cp_model.CpSolverSolutionCallback):
    def __init__(self,start,stage,rows):
        super().__init__(); self.start=start; self.stage=stage; self.rows=rows
    def on_solution_callback(self):
        self.rows.append({'elapsed_s':perf_counter()-self.start,'stage':self.stage,'objective':self.objective_value,'bound':self.best_objective_bound})


def solve(instance,hints=None,priorities=None,fixed=None,relax=None,soft_weight=100,reference=None,reference_weighting='uniform'):
    start=perf_counter(); result=SolveResult('UNKNOWN')
    if instance.config.time_limit_s==0: return result
    built=build(instance,hints,priorities,fixed,relax=relax,soft_weight=soft_weight,assume=False,reference=reference,reference_weighting=reference_weighting)
    solver=cp_model.CpSolver(); cfg=instance.config
    solver.parameters.num_search_workers=cfg.workers; solver.parameters.random_seed=cfg.seed
    if priorities: solver.parameters.search_branching=cp_model.FIXED_SEARCH
    all_optimal=True
    for stage,obj in built.objectives:
        remaining=cfg.time_limit_s-(perf_counter()-start)
        if remaining<=0:
            all_optimal=False; result.stages.append({'stage':stage,'status':'NOT_STARTED'}); break
        built.model.minimize(obj); solver.parameters.max_time_in_seconds=remaining
        status=solver.solve(built.model,Trace(start,stage,result.trace))
        result.branches+=solver.num_branches; result.conflicts+=solver.num_conflicts
        record={'stage':stage,'status':solver.status_name(status),'elapsed_s':perf_counter()-start}
        if status in (cp_model.OPTIMAL,cp_model.FEASIBLE): record.update(objective=solver.objective_value,bound=solver.best_objective_bound)
        result.stages.append(record)
        if status not in (cp_model.OPTIMAL,cp_model.FEASIBLE):
            all_optimal=False
            if result.actions is None:
                result.status=solver.status_name(status)
                if status==cp_model.INFEASIBLE:
                    # An optimization solve returns every assumption as a 'core' in
                    # this OR-Tools version. Re-solve as pure feasibility first.
                    remaining=cfg.time_limit_s-(perf_counter()-start)
                    if remaining>0:
                        core_model=build(instance,fixed=fixed,assume=True)
                        core_solver=cp_model.CpSolver()
                        core_solver.parameters.num_search_workers=1
                        core_solver.parameters.max_time_in_seconds=remaining
                        core_status=core_solver.solve(core_model.model)
                        if core_status==cp_model.INFEASIBLE:
                            result.unsat_core=[core_model.assumption_names.get(k,str(k)) for k in core_solver.sufficient_assumptions_for_infeasibility()]
            break
        values={name:solver.value(v) for name,v in built.variables.items()}
        actions=tuple(Action(tuple(values[f'm/{t}/{r}'] for r in range(len(instance.corridor.ramp_max))),tuple(values[f'v/{t}/{j}'] for j in range(len(instance.corridor.jam)))) for t in range(cfg.horizon))
        slack=(values['excess/max'] if relax else 0)
        cert=verify_plan(instance,actions,queue_slack=slack)
        if relax and cert.max_queue_excess>values['excess/max']: raise RuntimeError('solver/verifier exceedance mismatch')
        if not cert.safe: raise RuntimeError('solver/verifier mismatch: '+str(cert.violations))
        # Compare independently replayed states against every solver state.
        states=[]
        for wi,ws in enumerate(built.states):
            trajectory=[]
            for t,(ns,qs,b) in enumerate(ws):
                state=State(tuple(solver.value(x) for x in ns),tuple(solver.value(x) for x in qs),solver.value(b),instance.state.previous if t==0 else actions[t-1])
                if state!=cert.trajectories[wi][t]: raise RuntimeError('solver/verifier trajectory mismatch')
                trajectory.append(state)
            states.append(tuple(trajectory))
        result.actions=actions; result.trajectories=tuple(states); result.objectives=cert.objectives; result.values=values
        result.max_queue_excess=cert.max_queue_excess
        result.status='FEASIBLE'; result.objective_stage=stage
        result.objective_value=solver.objective_value; result.best_bound=solver.best_objective_bound
        result.gap=abs(result.objective_value-result.best_bound)/max(1,abs(result.objective_value))
        if status!=cp_model.OPTIMAL:
            all_optimal=False; break
        built.model.add(obj==int(round(solver.objective_value)))
    if result.actions is not None and all_optimal: result.status='OPTIMAL'
    active=[]
    if result.actions is not None and not relax:
        from trace_csp.graph.factor_graph import factor_graph
        from trace_csp.graph.labels import labels_for
        graph=factor_graph(built); labels=labels_for(instance,built,graph,result)
        active=[name for name,y,valid in zip(graph['constraint_names'],labels['active'],labels['active_mask']) if y and valid]
    result.audit={'solver_status':result.status,'objective_stage':result.objective_stage,
                  'optimality_gap':result.gap,'independently_verified':result.actions is not None,
                  'active_constraints':active,'hard_violations':[] if result.actions else None,
                  'violated_soft_constraints':[],'unsat_core':result.unsat_core}
    result.wall_time=perf_counter()-start
    result.first_feasible_s=result.trace[0]['elapsed_s'] if result.trace else None
    return result
