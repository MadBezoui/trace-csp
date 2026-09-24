from trace_csp.traffic.ctm import step
from trace_csp.graph.factor_graph import kind_of


def candidate_values(instance):
    """Previous-action candidate before repair; label only known state/control scopes."""
    values={}; a=instance.state.previous
    for t in range(instance.config.horizon):
        for key,row in [('m',a.ramp),('v',a.vsl)]:
            values.update({f'{key}/{t}/{j}':v for j,v in enumerate(row)})
    for wi,w in enumerate(instance.scenarios):
        s=instance.state
        for t in range(instance.config.horizon+1):
            values.update({f'n/{wi}/{t}/{j}':v for j,v in enumerate(s.vehicles)})
            values.update({f'q/{wi}/{t}/{j}':v for j,v in enumerate(s.queues)})
            values[f'b/{wi}/{t}']=s.boundary_queue
            if t<instance.config.horizon: s=step(instance.corridor,s,a,w,t,instance.config).state
    return values


def labels_for(instance,built,graph,result):
    labels={k:[] for k in ('active','violation','core','active_mask','violation_mask','core_mask')}
    candidate=candidate_values(instance); names=graph['variable_names']
    for ci,c in enumerate(built.model.proto.constraints):
        linear=kind_of(c)=='linear'; active=violation=0; am=vm=False
        if linear:
            scope=[names[v] for v in c.linear.vars]; lo,hi=c.linear.domain[0],c.linear.domain[-1]
            am=result.actions is not None and all(v in result.values for v in scope)
            vm=all(v in candidate for v in scope)
            if am:
                lhs=sum(co*result.values[n] for co,n in zip(c.linear.coeffs,scope))
                active=int(lhs==lo or lhs==hi)
            if vm:
                lhs=sum(co*candidate[n] for co,n in zip(c.linear.coeffs,scope))
                violation=int(not lo<=lhs<=hi)
        name=graph['constraint_names'][ci]
        cm=bool(c.enforcement_literal) and (result.status in ('OPTIMAL','FEASIBLE') or (result.status=='INFEASIBLE' and bool(result.unsat_core)))
        for key,val in [('active',active),('violation',violation),('core',int(name in result.unsat_core)),('active_mask',am),('violation_mask',vm),('core_mask',cm)]: labels[key].append(val)
    return labels
