"""Learning-free constraint scores from CTM monotonicity.

Queue lower bound: release <= min(reachable ramp rate, queue+demand, cell capacity),
and q + d - min(M, q + d, R) is nondecreasing in q. Propagating that bound gives a
sound (not complete) proof that one queue guard alone is an infeasibility core.
"""
from trace_csp.graph.labels import candidate_values
from trace_csp.domain import release_capacity


def command_range(instance,j,t):
    """Metering commands reachable at step t under smoothness; commands change only at
    the start of an actuation period, so at most t//control_block+1 changes occur."""
    c,s,cfg=instance.corridor,instance.state,instance.config
    changes=t//cfg.control_block+1; p=s.previous.ramp[j]
    return max(c.ramp_min[j],p-cfg.max_dm*changes),min(c.ramp_max[j],p+cfg.max_dm*changes)


def release_range(instance,j,t):
    """Smallest and largest release capacity over the reachable commands at step t."""
    lo,hi=command_range(instance,j,t); c,cfg=instance.corridor,instance.config
    if not c.release_table[j]: return lo,hi
    values=[release_capacity(c,cfg,j,u,t) for u in range(lo,hi+1)]
    return min(values),max(values)


def queue_lower_bounds(instance):
    """Lower bound on q/w/t/j for every scenario, time and ramp (all controls)."""
    c,s,cfg=instance.corridor,instance.state,instance.config
    bounds={}
    for wi,w in enumerate(instance.scenarios):
        q=list(s.queues)
        for j in range(len(q)): bounds[f'q/{wi}/0/{j}']=q[j]
        for t in range(cfg.horizon):
            for j,cell in enumerate(c.ramp_segment):
                reachable=release_range(instance,j,t)[1]
                receive=c.capacity[cell]*w.capacity_factor[t][cell]//1000
                arrivals=q[j]+w.ramps[t][j]
                q[j]=arrivals-min(reachable,arrivals,receive)
                bounds[f'q/{wi}/{t+1}/{j}']=q[j]
    return bounds


def certified_core(instance):
    """Earliest queue guard whose lower bound exceeds its limit, else None."""
    bounds=queue_lower_bounds(instance); limits=instance.corridor.queue_max
    hits=[(int(k.split('/')[2]),k) for k,v in bounds.items() if v>limits[int(k.split('/')[3])]]
    return 'queue/'+min(hits)[1][2:] if hits else None


def physics_scores(instance,graph):
    """Scores in [0,1] per constraint: equality, candidate replay, queue-bound margin."""
    bounds=queue_lower_bounds(instance); candidate=candidate_values(instance)
    limits=instance.corridor.queue_max; core=certified_core(instance)
    scores={'active':[],'violation':[],'core':[]}
    for name,f in zip(graph['constraint_names'],graph['constraint_features']):
        scores['active'].append(1. if f[4]==f[5] else .5 if name.startswith(('queue/','ramp_smooth','vsl_smooth')) else 0.)
        if name.startswith('queue/'):
            _,wi,t,j=name.split('/'); key=f'q/{wi}/{t}/{j}'; limit=limits[int(j)]
            scores['violation'].append(float(candidate[key]>limit))
            margin=(bounds[key]-limit)/max(1,limit)
            scores['core'].append(1. if name==core else min(.99,max(0.,.5+margin/2)))
        elif name.startswith(('spatial_upper','spatial_lower')):
            _,t,j=name.split('/'); v=instance.state.previous.vsl; j=int(j)
            scores['violation'].append(float(abs(v[j]-v[j-1])>instance.config.spatial_dv)); scores['core'].append(0.)
        else:
            scores['violation'].append(0.); scores['core'].append(0.)
    return scores


def core_is_sufficient(instance,guards,seconds=5.):
    """Solver check: enforcing only these queue guards leaves the model infeasible."""
    from ortools.sat.python import cp_model
    from trace_csp.csp.builder import build
    built=build(instance); model=built.model; model.clear_assumptions()
    flags={name:built.variables['assume:'+name] for name in built.assumption_names.values()}
    if any(g not in flags for g in guards): raise ValueError('unknown guard')
    for g in guards: model.add(flags[g]==1)
    solver=cp_model.CpSolver(); solver.parameters.num_search_workers=1; solver.parameters.max_time_in_seconds=seconds
    status=solver.solve(model)
    return {cp_model.INFEASIBLE:True,cp_model.OPTIMAL:False,cp_model.FEASIBLE:False}.get(status)


def interval_reach(instance,joint_merge=False):
    """Sound interval over-approximation of every reachable CTM state.

    Controls range over values reachable under ramp and speed smoothness
    (spatial consistency is dropped, which only enlarges the set); a ramp's release
    capacity ranges over the device release table of those commands. Each map is
    evaluated at the interval endpoints where it is monotone:
    S rises with (n, v), R falls with n, r = min(q+d, u, R) rises with all,
    f = min(S_up, R - r), q' = max(0, q+d-min(u, R)), z' = max(0, z+b-(R_1-r_1)).
    Returns {state_name: (lo, hi)} using the solver's variable names.
    """
    c,s,cfg=instance.corridor,instance.state,instance.config
    n_cells=len(c.jam); ramp_at=dict((cell,j) for j,cell in enumerate(c.ramp_segment))
    speeds=sorted(cfg.vsl_values); out={}
    from trace_csp.domain import critical_occupancy
    def send(n,v,j,cap_j): return min(n*v*cfg.dt_s*10//(36*c.length_m[j]),cap_j)
    def cap_range(j,lo,hi):
        dropped=cap[j]*(1000-c.capacity_drop_permille)//1000; crit=critical_occupancy(c,cfg,j)
        # Discharge capacity is not monotone in n: bound it over the whole occupancy interval.
        return (dropped if hi>crit else cap[j], cap[j] if lo<=crit else dropped)
    def recv(n,j): return min((c.jam[j]-n)*c.wave_speed_kmh*cfg.dt_s*10//(36*c.length_m[j]),cap[j])
    for wi,w in enumerate(instance.scenarios):
        n=[(x,x) for x in s.vehicles]; q=[(x,x) for x in s.queues]; z=(s.boundary_queue,)*2
        def record(t):
            for j,b in enumerate(n): out[f'n/{wi}/{t}/{j}']=b
            for j,b in enumerate(q): out[f'q/{wi}/{t}/{j}']=b
            out[f'b/{wi}/{t}']=z
        record(0)
        for t in range(cfg.horizon):
            cap=[k*f//1000 for k,f in zip(c.capacity,w.capacity_factor[t])]
            u=[release_range(instance,j,t) for j in range(len(c.ramp_segment))]
            changes=t//cfg.control_block+1
            v=[(max(speeds[0],p-cfg.max_dv*changes),min(speeds[-1],p+cfg.max_dv*changes)) for p in s.previous.vsl]
            v=[(min(x for x in speeds if x>=lo),max(x for x in speeds if x<=hi)) for lo,hi in v]
            S=[(send(n[j][0],v[j][0],j,cap_range(j,*n[j])[0]),send(n[j][1],v[j][1],j,cap_range(j,*n[j])[1])) for j in range(n_cells)]
            R=[(recv(n[j][1],j),recv(n[j][0],j)) for j in range(n_cells)]
            r=[(0,0)]*n_cells; qn=list(q)
            for cell,j in ramp_at.items():
                d=w.ramps[t][j]
                r[cell]=(min(q[j][0]+d,u[j][0],R[cell][0]),min(q[j][1]+d,u[j][1],R[cell][1]))
                qn[j]=(max(0,q[j][0]+d-min(u[j][1],R[cell][1])),max(0,q[j][1]+d-min(u[j][0],R[cell][0])))
            room=[(max(0,R[j][0]-r[j][1]),max(0,R[j][1]-r[j][0])) for j in range(n_cells)]
            # leave_j = min(S_j, room_{j+1}*1000 // share_j) and through_j = leave_j*share_j // 1000
            # are nondecreasing in S_j and room_{j+1}.
            share=[1000-e for e in c.exit_permille]
            leave=[(min(S[j][0],room[j+1][0]*1000//share[j]),min(S[j][1],room[j+1][1]*1000//share[j])) for j in range(n_cells-1)]+[S[-1]]
            inflow=[(min(z[0]+w.mainline[t],room[0][0]),min(z[1]+w.mainline[t],room[0][1]))]
            inflow+=[(leave[j][0]*share[j]//1000,leave[j][1]*share[j]//1000) for j in range(n_cells-1)]
            z0=z
            z=(max(0,z[0]+w.mainline[t]-room[0][1]),max(0,z[1]+w.mainline[t]-room[0][0]))
            entry=[(inflow[j][0]+r[j][0],inflow[j][1]+r[j][1]) for j in range(n_cells)]
            if joint_merge:
                # Mainline inflow and ramp release share the receiving capacity R_j:
                # E=f+r <= R, and f >= min(floor(share*S_up/1000), rho-delta) gives
                # E >= min(floor(share*S_up/1000)+r, R-delta) (delta=1 absorbs the diverge floor).
                for j in range(n_cells):
                    if j==0: joint_lo=min(z0[0]+w.mainline[t]+r[0][0],R[0][0])
                    else:
                        delta=0 if share[j-1]==1000 else 1
                        joint_lo=min(S[j-1][0]*share[j-1]//1000+r[j][0],R[j][0]-delta)
                    entry[j]=(max(entry[j][0],joint_lo,0),min(entry[j][1],R[j][1]))
            n=[(max(0,n[j][0]+entry[j][0]-leave[j][1]),min(c.jam[j],n[j][1]+entry[j][1]-leave[j][0])) for j in range(n_cells)]
            q=qn; record(t+1)
    return out


def interval_core(instance,joint_merge=False):
    """Earliest queue guard whose interval lower bound exceeds its limit, else None."""
    bounds=interval_reach(instance,joint_merge); limits=instance.corridor.queue_max
    hits=[(int(k.split('/')[2]),k) for k,(lo,_) in bounds.items() if k.startswith('q/') and lo>limits[int(k.split('/')[3])]]
    return 'queue/'+min(hits)[1][2:] if hits else None
