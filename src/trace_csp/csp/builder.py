"""CP-SAT encoding of CTM with scenario-shared controls and strict hard limits."""
from dataclasses import dataclass
from ortools.sat.python import cp_model
from trace_csp.domain import Instance


@dataclass
class BuiltModel:
    model: object
    variables: dict
    controls: dict
    states: list
    objectives: list
    assumption_names: dict


def build(instance: Instance, hints=None, priorities=None, fixed=None, relax=None, soft_weight=100, assume=True, reference=None, reference_weighting='uniform'):
    """relax=None keeps hard queue guards; 'lexicographic' minimizes the worst queue
    exceedance, then total exceedance, then the nominal objectives; 'soft' adds a
    weighted exceedance penalty to TTS (softened MPC baseline)."""
    if relax not in (None,'lexicographic','soft'): raise ValueError('unknown relaxation')
    c,s,cfg=instance.corridor,instance.state,instance.config
    model=cp_model.CpModel(); variables={}; controls={}; assumptions={}
    n,r,h=len(c.jam),len(c.ramp_max),cfg.horizon
    bound=max(1,sum(s.vehicles)+sum(s.queues)+s.boundary_queue+max(sum(w.mainline)+sum(map(sum,w.ramps)) for w in instance.scenarios))
    def var(name,lo=0,hi=bound):
        x=model.new_int_var(lo,hi,name); variables[name]=x; return x
    def linear(expr,name,guard=False):
        ct=model.add(expr).with_name(name)
        if guard:
            flag=model.new_bool_var('assume:'+name); variables[flag.name]=flag
            ct.only_enforce_if(flag); assumptions[flag.index]=name
            # Assumptions are needed only to extract cores; fixing the flags is much faster to optimize.
            if assume: model.add_assumption(flag)
            else: model.add(flag==1)
        return ct
    def minimum(name,expressions,hi=bound):
        x=var(name,0,hi); model.add_min_equality(x,expressions).with_name(name+':min'); return x
    def absolute(name,expr,hi):
        x=var(name,0,hi); model.add_abs_equality(x,expr).with_name(name+':abs'); return x
    ramps=[]; vsls=[]; smooth=[]
    for t in range(h):
        ms=[var(f'm/{t}/{j}',c.ramp_min[j],c.ramp_max[j]) for j in range(r)]
        vs=[]
        for j in range(n):
            x=model.new_int_var_from_domain(cp_model.Domain.from_values(cfg.vsl_values),f'v/{t}/{j}')
            variables[x.name]=x; vs.append(x)
        for j,x in enumerate(ms):
            controls[x.name]=x
            d=absolute(f'dm/{t}/{j}',x-(ramps[t-1][j] if t else s.previous.ramp[j]),c.ramp_max[j])
            linear(d<=cfg.max_dm,f'ramp_smooth/{t}/{j}'); smooth.append(d)
        for j,x in enumerate(vs):
            controls[x.name]=x
            d=absolute(f'dv/{t}/{j}',x-(vsls[t-1][j] if t else s.previous.vsl[j]),max(cfg.vsl_values))
            linear(d<=cfg.max_dv,f'vsl_smooth/{t}/{j}'); smooth.append(d)
            if j:
                linear(x-vs[j-1]<=cfg.spatial_dv,f'spatial_upper/{t}/{j}')
                linear(vs[j-1]-x<=cfg.spatial_dv,f'spatial_lower/{t}/{j}')
        ramps.append(ms); vsls.append(vs)
    costs=[]; fairs=[]; all_states=[]; excess=[]
    overs=[]
    # Critical occupancy: largest n whose free-flow sending at the top speed stays within capacity.
    critical=[c.capacity[j]*36*c.length_m[j]//(max(cfg.vsl_values)*cfg.dt_s*10) for j in range(n)]
    for wi,w in enumerate(instance.scenarios):
        states=[]; scenario_cost=[]; waits=[[] for _ in range(r)]
        for t in range(h+1):
            ns=[var(f'n/{wi}/{t}/{j}',0,c.jam[j]) for j in range(n)]
            qs=[var(f'q/{wi}/{t}/{j}') for j in range(r)]
            b=var(f'b/{wi}/{t}')
            states.append((ns,qs,b))
            for j,q in enumerate(qs):
                if relax:
                    slack=var(f'excess/{wi}/{t}/{j}'); excess.append((instance.scenarios[wi].weight,slack))
                    linear(q-slack<=c.queue_max[j],f'queue/{wi}/{t}/{j}')
                else: linear(q<=c.queue_max[j],f'queue/{wi}/{t}/{j}',guard=True)
            if t==0:
                for x,val in zip(ns+qs+[b],list(s.vehicles)+list(s.queues)+[s.boundary_queue]): linear(x==val,'initial:'+x.name)
            else:
                scenario_cost+=ns+qs+[b]
                if cfg.congestion_penalty:
                    for j in range(n):
                        o=var(f'over_critical/{wi}/{t}/{j}',0,c.jam[j]); linear(o>=ns[j]-critical[j],f'critical/{wi}/{t}/{j}')
                        overs.append((instance.scenarios[wi].weight,o))
                for j,q in enumerate(qs): waits[j].append(q)
        for t in range(h):
            ns,qs,b=states[t]; nn,qn,bn=states[t+1]
            sends=[]; receives=[]
            for j in range(n):
                prefix=f'{wi}/{t}/{j}'; cap=c.capacity[j]*w.capacity_factor[t][j]//1000
                raw=var('raw_send/'+prefix,0,c.jam[j])
                if len(cfg.vsl_values)==1:
                    # Fixed speed: the sending term is affine in the occupancy (same floor).
                    model.add_division_equality(raw,ns[j]*(cfg.vsl_values[0]*cfg.dt_s*10),36*c.length_m[j]).with_name('sending/'+prefix)
                else:
                    prod=var('product/'+prefix,0,c.jam[j]*max(cfg.vsl_values))
                    model.add_multiplication_equality(prod,[ns[j],vsls[t][j]]).with_name('speed_product/'+prefix)
                    model.add_division_equality(raw,prod*cfg.dt_s*10,36*c.length_m[j]).with_name('sending/'+prefix)
                if c.capacity_drop_permille:
                    # Congested indicator (n > critical) switches the discharge capacity.
                    jammed=model.new_bool_var('congested/'+prefix); variables[jammed.name]=jammed
                    crit=c.capacity[j]*36*c.length_m[j]//(max(cfg.vsl_values)*cfg.dt_s*10)
                    model.add(ns[j]>=crit+1).only_enforce_if(jammed); model.add(ns[j]<=crit).only_enforce_if(jammed.Not())
                    sends.append(minimum('send/'+prefix,[raw,cap-(cap-cap*(1000-c.capacity_drop_permille)//1000)*jammed]))
                else: sends.append(minimum('send/'+prefix,[raw,cap]))
                space=var('raw_receive/'+prefix,0,c.jam[j])
                model.add_division_equality(space,(c.jam[j]-ns[j])*c.wave_speed_kmh*cfg.dt_s*10,36*c.length_m[j]).with_name('receiving/'+prefix)
                receives.append(minimum('receive/'+prefix,[space,cap]))
            release=[minimum(f'release/{wi}/{t}/{j}',[ramps[t][j],qs[j]+w.ramps[t][j],receives[cell]]) for j,cell in enumerate(c.ramp_segment)]
            rin=dict(zip(c.ramp_segment,release))
            flows=[]; through=[]
            for j in range(n):
                if j+1==n: flows.append(sends[j]); break
                room=receives[j+1]-rin.get(j+1,0); share=1000-c.exit_permille[j]
                if share==1000:
                    flows.append(minimum(f'flow/{wi}/{t}/{j}',[sends[j],room])); through.append(flows[-1]); continue
                # FIFO diverge, identical integer rounding to traffic.ctm and the verifier.
                space=var(f'room/{wi}/{t}/{j+1}',0,c.jam[j+1]); linear(space==room,f'room/{wi}/{t}/{j+1}')
                limit=var(f'diverge_limit/{wi}/{t}/{j}',0,c.jam[j+1]*1000)
                model.add_division_equality(limit,space*1000,share).with_name(f'diverge_limit/{wi}/{t}/{j}')
                flows.append(minimum(f'flow/{wi}/{t}/{j}',[sends[j],limit]))
                x=var(f'through/{wi}/{t}/{j}',0,c.jam[j])
                model.add_division_equality(x,flows[-1]*share,1000).with_name(f'through/{wi}/{t}/{j}'); through.append(x)
            admitted=minimum(f'admit/{wi}/{t}',[b+w.mainline[t],receives[0]-rin.get(0,0)])
            for j in range(n): linear(nn[j]==ns[j]+(admitted if j==0 else through[j-1])+rin.get(j,0)-flows[j],f'conservation/{wi}/{t}/{j}')
            for j in range(r): linear(qn[j]==qs[j]+w.ramps[t][j]-release[j],f'ramp_conservation/{wi}/{t}/{j}')
            linear(bn==b+w.mainline[t]-admitted,f'boundary_conservation/{wi}/{t}')
        cost=var(f'cost/{wi}',0,bound*h); linear(cost==sum(scenario_cost),f'tts/{wi}'); costs.append(cost)
        wait_totals=[]
        for j in range(r):
            total=var(f'wait/{wi}/{j}',0,bound*h); linear(total==sum(waits[j]),f'waiting/{wi}/{j}'); wait_totals.append(total)
        fair=var(f'fair/{wi}',0,bound*h)
        model.add_max_equality(fair,wait_totals or [0]).with_name(f'fair_max/{wi}'); fairs.append(fair)
        if cfg.fairness_limit is not None:
            for j in range(r):
                for k in range(j):
                    linear(wait_totals[j]-wait_totals[k]<=cfg.fairness_limit,f'fair_upper/{wi}/{j}/{k}')
                    linear(wait_totals[k]-wait_totals[j]<=cfg.fairness_limit,f'fair_lower/{wi}/{j}/{k}')
        all_states.append(states)
    weights=[w.weight for w in instance.scenarios]
    z=var('risk/var',0,bound*h); tail=[]
    for wi,cost in enumerate(costs):
        e=var(f'risk/excess/{wi}',0,bound*h); model.add_max_equality(e,[cost-z,0]).with_name(f'cvar/{wi}'); tail.append(e)
    tts=sum(w*x for w,x in zip(weights,costs))
    objectives=[('tts',tts),('cvar',(100-cfg.alpha_percent)*sum(weights)*z+100*sum(w*x for w,x in zip(weights,tail)))]
    # Congestion precedes TTS: over a short horizon TTS is nearly indifferent to metering.
    if overs: objectives.insert(0,('congestion',sum(w*x for w,x in overs)))
    if reference is not None:
        # Shield mode: stay as close as possible to a reference metering plan (e.g. ALINEA),
        # before any traffic objective; hard limits (or the relaxation stages) still come first.
        dev=[]
        for t in range(h):
            for j in range(r):
                d=var(f'deviation/{t}/{j}',0,c.ramp_max[j]); model.add_abs_equality(d,ramps[t][j]-int(reference[j])).with_name(f'deviation/{t}/{j}'); dev.append(d)
        # 'increasing' weights later deviations more, so a needed correction is made now
        # rather than postponed to the end of the horizon (anti-procrastination).
        weight=[t+1 if reference_weighting=='increasing' else 1 for t in range(h) for _ in range(r)]
        objectives.insert(0,('deviation',sum(w_*d for w_,d in zip(weight,dev))))
    if relax:
        worst=var('excess/max'); model.add_max_equality(worst,[x for _,x in excess]).with_name('excess_max')
        total=sum(w*x for w,x in excess)
        if relax=='lexicographic': objectives=[('max_excess',worst),('total_excess',total)]+objectives
        else:
            k=[name for name,_ in objectives].index('tts'); objectives[k]=('soft_tts',tts+soft_weight*total)
    if cfg.fairness: objectives.append(('fairness',sum(w*x for w,x in zip(weights,fairs))))
    objectives.append(('smoothness',sum(smooth)))
    if hints:
        for t,a in enumerate(hints[:h]):
            for prefix,values in [('m',a.ramp),('v',a.vsl)]:
                for j,val in enumerate(values):
                    name=f'{prefix}/{t}/{j}'
                    if name in controls: model.add_hint(controls[name],int(val))
    if fixed:
        for name,value in fixed.items():
            if name not in controls: raise ValueError('only control variables can be fixed')
            linear(controls[name]==int(value),'lns_fix:'+name)
    if priorities:
        order=list(dict.fromkeys(priorities))+[name for name in controls if name not in priorities]
        if any(name not in controls for name in order): raise ValueError('unknown branching variable')
        model.add_decision_strategy([controls[name] for name in order],cp_model.CHOOSE_FIRST,cp_model.SELECT_MIN_VALUE)
    return BuiltModel(model,variables,controls,all_states,objectives,assumptions)
