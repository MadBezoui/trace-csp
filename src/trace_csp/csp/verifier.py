"""Independent arithmetic replay; deliberately does not import CTM or OR-Tools.

Only the data definitions are shared with the rest of the package: every flow rule,
including the capacity drop and the device release table, is re-implemented here.
Certification is restricted to the supplied finite scenarios and this model.
Violations name the state time point (0 is the measured initial state).
"""
from dataclasses import dataclass, field
from fractions import Fraction
from trace_csp.domain import Action, State, Instance


@dataclass
class Certificate:
    safe: bool
    violations: list[str] = field(default_factory=list)
    objectives: tuple[int, ...] | None = None
    trajectories: tuple[tuple[State, ...], ...] = ()
    scope: str = 'discrete-model/supplied-scenarios'
    max_queue_excess: int = 0


def verify(i: Instance, action: Action, queue_slack=0) -> Certificate:
    """Verify one command held over a whole actuation period (Config.control_block steps)."""
    return verify_plan(i, (action,)*i.config.control_block, require_horizon=False, queue_slack=queue_slack)


def verify_plan(i: Instance, actions, require_horizon=True, queue_slack=0) -> Certificate:
    """queue_slack>0 accepts a declared queue exceedance; the realized maximum is reported."""
    c,cfg = i.corridor,i.config
    errors=[]; worst_excess=0
    if actions is None or len(actions)==0 or len(actions)>cfg.horizon or (require_horizon and len(actions)!=cfg.horizon):
        return Certificate(False,['invalid plan length'])
    prev=i.state.previous; block=cfg.control_block
    for t,a in enumerate(actions):
        if not isinstance(a,Action) or len(a.ramp)!=len(c.ramp_max) or len(a.vsl)!=len(c.jam):
            return Certificate(False,['invalid action shape'])
        if any(type(x) is not int or not lo<=x<=limit for x,lo,limit in zip(a.ramp,c.ramp_min,c.ramp_max)):
            return Certificate(False,['ramp domain'])
        if any(type(x) is not int or x not in cfg.vsl_values for x in a.vsl):
            return Certificate(False,['vsl domain'])
        if any(abs(x-y)>cfg.max_dm for x,y in zip(a.ramp,prev.ramp)): errors.append(f't{t}: ramp smoothness')
        if any(abs(x-y)>cfg.max_dv for x,y in zip(a.vsl,prev.vsl)): errors.append(f't{t}: speed smoothness')
        if any(abs(x-y)>cfg.spatial_dv for x,y in zip(a.vsl,a.vsl[1:])): errors.append(f't{t}: spatial speed')
        if t%block and a!=prev: errors.append(f't{t}: command changed within its actuation period')
        prev=a
    trajectories=[]; costs=[]; fairness=[]
    critical=[c.capacity[j]*36*c.length_m[j]//(max(cfg.vsl_values)*cfg.dt_s*10) for j in range(len(c.jam))]; congestion=[]
    def discharge(j,cap,n): return cap*(1000-c.capacity_drop_permille)//1000 if n>critical[j] else cap
    def release_cap(r,command,t): return c.release_table[r][command][t%block] if c.release_table[r] else command
    for wi,w in enumerate(i.scenarios):
        s=i.state; states=[s]; wait=[0]*len(c.ramp_max); cost=0; over=0
        # Queue limits hold at every state time point, including the measured one.
        worst_excess=max(worst_excess,*(x-limit for x,limit in zip(s.queues,c.queue_max)),0)
        if any(x>limit+queue_slack for x,limit in zip(s.queues,c.queue_max)): errors.append(f's{wi}/t0: queue bound')
        for t,a in enumerate(actions):
            # Exact rational calculations independently reproduce floor quantization.
            cap=[c.capacity[j]*w.capacity_factor[t][j]//1000 for j in range(len(c.jam))]
            supply=[min(cap[j],int(Fraction(c.wave_speed_kmh*cfg.dt_s*5,18*c.length_m[j])*(c.jam[j]-s.vehicles[j]))) for j in range(len(c.jam))]
            arrivals=[0]*len(c.jam); queues=[]
            for r,j in enumerate(c.ramp_segment):
                entered=min(release_cap(r,a.ramp[r],t),s.queues[r]+w.ramps[t][r],supply[j])
                arrivals[j]=entered
                queues.append(s.queues[r]+w.ramps[t][r]-entered)
            exits=[]; passing=[]
            for j in range(len(c.jam)):
                send=min(discharge(j,cap[j],s.vehicles[j]),int(Fraction(a.vsl[j]*cfg.dt_s*5,18*c.length_m[j])*s.vehicles[j]))
                if j+1==len(c.jam): exits.append(send); break
                keep=Fraction(1000-c.exit_permille[j],1000)
                out=min(send,int((supply[j+1]-arrivals[j+1])/keep))
                exits.append(out); passing.append(int(out*keep))
            inflow=min(s.boundary_queue+w.mainline[t],supply[0]-arrivals[0])
            counts=tuple(s.vehicles[j]+(inflow if j==0 else passing[j-1])+arrivals[j]-exits[j] for j in range(len(c.jam)))
            diverted=sum(x-y for x,y in zip(exits,passing))
            b=s.boundary_queue+w.mainline[t]-inflow
            if any(x<0 or x>limit for x,limit in zip(counts,c.jam)): errors.append(f's{wi}/t{t+1}: density bound')
            worst_excess=max(worst_excess,*(x-limit for x,limit in zip(queues,c.queue_max)),0)
            if any(x<0 or x>limit+queue_slack for x,limit in zip(queues,c.queue_max)): errors.append(f's{wi}/t{t+1}: queue bound')
            if b<0: errors.append(f's{wi}/t{t+1}: upstream queue')
            before=sum(s.vehicles)+sum(s.queues)+s.boundary_queue+w.mainline[t]+sum(w.ramps[t])
            if sum(counts)+sum(queues)+b+exits[-1]+diverted!=before: errors.append('mass conservation')
            s=State(counts,tuple(queues),b,a); states.append(s)
            cost+=sum(counts)+sum(queues)+b
            over+=sum(max(0,x-k) for x,k in zip(counts,critical))
            wait=[x+y for x,y in zip(wait,queues)]
        if cfg.fairness_limit is not None and wait and max(wait)-min(wait)>cfg.fairness_limit:
            errors.append(f's{wi}: fairness bound')
        trajectories.append(tuple(states)); costs.append(cost); fairness.append(max(wait,default=0)); congestion.append(over)
    weights=[w.weight for w in i.scenarios]; total=sum(weights)
    risk=min((100-cfg.alpha_percent)*total*z+100*sum(w*max(v-z,0) for w,v in zip(weights,costs)) for z in costs)
    smooth=0; prev=i.state.previous
    for a in actions:
        smooth+=sum(abs(x-y) for x,y in zip(a.ramp+a.vsl,prev.ramp+prev.vsl)); prev=a
    obj=(sum(w*x for w,x in zip(weights,congestion)),) if cfg.congestion_penalty else ()
    obj+=(sum(w*x for w,x in zip(weights,costs)),risk)
    if cfg.fairness: obj+=(sum(w*x for w,x in zip(weights,fairness)),)
    obj+=(smooth,)
    return Certificate(not errors,errors,obj,tuple(trajectories),max_queue_excess=worst_excess)
