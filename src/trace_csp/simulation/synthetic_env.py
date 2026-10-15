"""Paired closed-loop experiments within the discrete CTM, not a field model."""
from dataclasses import replace
from time import perf_counter
from trace_csp.data import generate_instance
from trace_csp.domain import Action
from trace_csp.traffic.ctm import step
from trace_csp.csp.verifier import verify
from trace_csp.csp.controller import Controller,alinea


def gini(values):
    x=sorted(values); total=sum(x); n=len(x)
    return 0. if not total else sum((2*j-n-1)*v for j,v in enumerate(x,1))/(n*total)


def period_incident_factor(family,initial_factor,period):
    if initial_factor is None: return None
    if family=='normal' and period>0: return 1000
    if family=='mild_incident': return min(1000,initial_factor+100*period)
    return initial_factor


def run_episode(seed,controller='cp_sat',periods=12,family=None,demand_multiplier=1.,incident_factor=None,predictor=None,
                corridor_cells=None,corridor_horizon=4,time_limit_s=None,ungated_baselines=False,soft_weight=100):
    if periods<1: raise ValueError('periods must be positive')
    supported=('fixed','alinea','hero_proxy','cp_sat','warm_cp_sat','learned_cp_sat','degraded_cp_sat','soft_mpc')
    if controller not in supported: raise ValueError('unknown controller')
    if controller=='learned_cp_sat' and predictor is None: raise ValueError('learned predictor required')
    def make(k,fam,factor):
        if corridor_cells:
            from trace_csp.corridor import generate_corridor
            x=generate_corridor(k,cells=corridor_cells,horizon=corridor_horizon,family=fam)
        else: x=generate_instance(k,horizon=2,family=fam,demand_multiplier=demand_multiplier,incident_factor=factor)
        return replace(x,config=replace(x.config,time_limit_s=time_limit_s)) if time_limit_s is not None else x
    first=make(seed,family,incident_factor)
    state=first.state; initial=sum(state.vehicles)+sum(state.queues)+state.boundary_queue
    mode={'degraded_cp_sat':'degrade','soft_mpc':'soft'}.get(controller,'hard')
    optimiser=Controller(reuse=controller not in ('cp_sat','degraded_cp_sat'),predictor=predictor if controller=='learned_cp_sat' else None,mode=mode,soft_weight=soft_weight)
    excess_total=0; excess_max=0; violations=0; total_time=0; exited=0; arriving=0; completed=0; fallback=0; maxqueue=0; waiting=[0]*len(first.corridor.ramp_max)
    halt_reason=None; halt_elapsed_s=None; records=[]
    for t in range(periods):
        period_factor=period_incident_factor(first.family,incident_factor,t)
        base=make(seed+t,first.family,period_factor)
        i=replace(base,state=state,instance_id=f'{first.instance_id}/period-{t}')
        control_started=perf_counter()
        if controller in ('cp_sat','warm_cp_sat','learned_cp_sat','degraded_cp_sat','soft_mpc'):
            decision=optimiser.act(i); action=decision.action; mode=decision.mode
        else:
            c=i.corridor
            if controller=='fixed':
                rate=tuple(min(m,max(lo,round(m/2))) for lo,m in zip(c.ramp_min,c.ramp_max))
                action=Action(rate,state.previous.vsl)
            else:
                action=alinea(i)
                if controller=='hero_proxy':
                    action=Action(tuple(c.ramp_max[r] if state.queues[r]>=.7*c.queue_max[r] else action.ramp[r] for r in range(len(c.ramp_max))),action.vsl)
            cert=verify(i,action)
            if not cert.safe and ungated_baselines: mode=controller+'-unverified'
            elif not cert.safe:
                # Apply the same verifier/fallback gate to every method.
                decision=Controller().act(replace(i,config=replace(i.config,time_limit_s=0)))
                action=decision.action; mode='baseline-fallback' if action else decision.mode
            else: mode=controller
        if action is None:
            halt_reason='no-certified-action'; halt_elapsed_s=perf_counter()-control_started; break
        control_elapsed_s=perf_counter()-control_started
        deadline_met=control_elapsed_s<=i.config.time_limit_s
        actual=i.scenarios[(seed+t)%len(i.scenarios)]
        transition=step(i.corridor,state,action,actual,0,i.config)
        state=transition.state; completed+=1
        arrived=actual.mainline[0]+sum(actual.ramps[0]); arriving+=arrived
        exited+=transition.outflow[-1]
        total_time+=(sum(state.vehicles)+sum(state.queues)+state.boundary_queue)*i.config.dt_s
        maxqueue=max(maxqueue,*state.queues)
        over=[max(0,q-k) for q,k in zip(state.queues,i.corridor.queue_max)]
        excess_total+=sum(over); excess_max=max(excess_max,*over); violations+=any(over)
        waiting=[a+b for a,b in zip(waiting,state.queues)]
        fallback+=mode not in (controller,'trace-csp','soft-mpc',controller+'-unverified')
        records.append({'period':t,'period_incident_factor':period_factor,
                        'realized_capacity_factor':list(actual.capacity_factor[0]),
                        'control_elapsed_s':control_elapsed_s,'deadline_met':deadline_met,
                        'mode':mode,'action':{'ramp':list(action.ramp),'vsl':list(action.vsl)},
                        'queue':list(state.queues),'vehicles':list(state.vehicles),'boundary_queue':state.boundary_queue,
                        'outflow':transition.outflow[-1]})
    final=sum(state.vehicles)+sum(state.queues)+state.boundary_queue
    return {'scope':'synthetic CTM closed loop','seed':seed,'soft_weight':soft_weight,'family':first.family,'controller':controller,
            'demand_multiplier':demand_multiplier,'incident_factor':incident_factor,'steps':completed,
            'requested_steps':periods,'halt_reason':halt_reason,'halt_elapsed_s':halt_elapsed_s,
            'feasible_at_deadline':sum(row['deadline_met'] for row in records),
            # Periods whose realized state exceeds a hard queue limit (ungated or relaxed actions).
            'hard_violations':violations,'fallback_count':fallback,
            'initial_vehicles':initial,'arriving_vehicles':arriving,'final_vehicles':final,'exited_vehicles':exited,
            'total_time_spent':total_time,'queue_excess_vehicle_steps':excess_total,'max_queue_excess':excess_max,'throughput':exited,'max_queue':maxqueue,'worst_ramp_waiting':max(waiting,default=0),
            'ramp_waiting_gini':gini(waiting),'records':records}


def experiment_matrix(controllers=('fixed','alinea','hero_proxy','cp_sat','warm_cp_sat','learned_cp_sat'),
                      seeds=range(30),periods=12,families=('normal',),demand_levels=(1.,),incident_levels=(1000,),predictor=None):
    return [run_episode(seed,controller,periods,family,demand,incident,predictor)
            for family in families for demand in demand_levels for incident in incident_levels
            for seed in seeds for controller in controllers]
