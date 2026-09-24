from dataclasses import replace
from trace_csp.csp.solver import solve
from trace_csp.guidance.priorities import scores_to_priority


def guided_lns(instance,incumbent,graph,scores,time_limit_s=.5,release_fraction=.3):
    if incumbent.actions is None: return incumbent
    controls=[n for n in incumbent.values if n.startswith(('m/','v/'))]
    ranked=scores_to_priority(graph,scores,controls)
    release=set(ranked[:max(1,round(len(ranked)*release_fraction))])
    fixed={n:incumbent.values[n] for n in controls if n not in release}
    trial=solve(replace(instance,config=replace(instance.config,time_limit_s=time_limit_s)),hints=incumbent.actions,priorities=ranked,fixed=fixed)
    if trial.actions is not None and trial.objectives<incumbent.objectives: return trial
    return incumbent
