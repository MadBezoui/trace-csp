from dataclasses import dataclass, replace, field
from time import perf_counter
from trace_csp.domain import Action
from trace_csp.csp.solver import solve,SolveResult
from trace_csp.csp.verifier import verify,Certificate


@dataclass
class Decision:
    action: Action | None
    mode: str
    certificate: Certificate
    solver_result: object
    reason: str
    elapsed_s: float
    audit: dict = field(default_factory=dict)


def alinea(instance):
    c,s,cfg=instance.corridor,instance.state,instance.config
    ramp=[]
    for r,j in enumerate(c.ramp_segment):
        target=round(.25*c.jam[j]); change=round(.15*(target-s.vehicles[j]))
        rate=max(c.ramp_min[r],min(c.ramp_max[r],s.previous.ramp[r]+max(-cfg.max_dm,min(cfg.max_dm,change))))
        ramp.append(rate)
    return Action(tuple(ramp),s.previous.vsl)


def verification_margin(instance):
    """Budget kept for verification and fallback so a solver incumbent is not late."""
    return min(.5,.1*instance.config.time_limit_s)


class Controller:
    """mode='hard': verified action or no-certified-action.
    mode='degrade': on certified or proven infeasibility, solve the lexicographic
    queue relaxation; as a last resort hold the previous action. The realized
    modeled queue exceedance is always reported.
    mode='soft': softened MPC baseline with a weighted exceedance penalty."""
    def __init__(self,reuse=True,predictor=None,mode='hard',soft_weight=100):
        self.reference=None  # optional reference ramp rates (shield mode), set before act()
        self.reference_weighting='uniform'
        if mode not in ('hard','degrade','soft'): raise ValueError('unknown controller mode')
        self.previous=None; self.signature=None; self.reuse=reuse; self.predictor=predictor
        self.mode=mode; self.soft_weight=soft_weight
    def _relaxed(self,instance,start,relax,core,result=None):
        remaining=max(0.,instance.config.time_limit_s-verification_margin(instance)-(perf_counter()-start))
        bounded=replace(instance,config=replace(instance.config,time_limit_s=remaining))
        relaxed=solve(bounded,relax=relax,soft_weight=self.soft_weight,reference=self.reference,reference_weighting=self.reference_weighting) if remaining>0 else SolveResult('UNKNOWN')
        candidates=[('degraded' if relax=='lexicographic' else 'soft-mpc',relaxed.actions[0])] if relaxed.actions else []
        candidates.append(('degraded-hold',self.best_held_action(instance)))
        for mode,a in candidates:
            cert=verify(instance,a,queue_slack=10**9)
            if cert.safe and (mode=='degraded-hold' or perf_counter()-start<=instance.config.time_limit_s):
                self.previous=relaxed.actions if mode!='degraded-hold' else None; self.signature=None
                audit={'mode':mode,'solver_status':(result or relaxed).status,'relaxed_status':relaxed.status,
                       'certified_core':core,'max_queue_excess_first_step':cert.max_queue_excess,
                       'planned_max_queue_excess':relaxed.max_queue_excess,'certificate_scope':cert.scope,'fallback_activated':True}
                return Decision(a,mode,cert,relaxed,'hard queue limits unattainable; exceedance reported',perf_counter()-start,audit)
        return None
    def best_held_action(self,instance):
        """Verify-first fallback: among holding the applied action and holding the reference
        (shield mode), return the one whose held plan has the smaller verified worst exceedance."""
        from trace_csp.domain import Action
        from trace_csp.csp.verifier import verify_plan
        options=[instance.state.previous]
        if self.reference is not None:
            options.append(Action(tuple(int(x) for x in self.reference),instance.state.previous.vsl))
        scored=[]
        for a in options:
            cert=verify_plan(instance,(a,)*instance.config.horizon,queue_slack=10**9)
            if cert.safe: scored.append((cert.max_queue_excess,len(scored),a))
        return min(scored)[2] if scored else instance.state.previous
    def act(self,instance):
        start=perf_counter(); signature=(instance.corridor,instance.config.horizon,instance.config.vsl_values)
        if self.mode=='soft':
            return self._relaxed(instance,start,'soft',None) or self._halt(SolveResult('UNKNOWN'),start)
        core=None
        if self.mode=='degrade':
            from trace_csp.graph.physics import interval_core
            core=interval_core(instance)
            if core is not None:
                return self._relaxed(instance,start,'lexicographic',core) or self._halt(SolveResult('INFEASIBLE'),start)
        shifted=None
        if self.reuse and self.previous and self.signature==signature:
            shifted=self.previous[1:]+self.previous[-1:]
        priorities=self.predictor(instance) if self.predictor is not None else None
        remaining=max(0.,instance.config.time_limit_s-verification_margin(instance)-(perf_counter()-start))
        bounded=replace(instance,config=replace(instance.config,time_limit_s=remaining))
        result=solve(bounded,hints=shifted,priorities=priorities,reference=self.reference,reference_weighting=self.reference_weighting) if remaining>0 else SolveResult('UNKNOWN')
        candidates=[]
        if result.actions: candidates.append(('trace-csp',result.actions[0]))
        if shifted: candidates.append(('shifted',shifted[0]))
        candidates.append(('fallback',alinea(instance)))
        candidates.append(('fallback',instance.state.previous))
        for mode,a in candidates:
            cert=verify(instance,a)
            if cert.safe and (mode!='trace-csp' or perf_counter()-start<=instance.config.time_limit_s):
                self.previous=result.actions if mode=='trace-csp' else None
                self.signature=signature
                reason='verified against current scenarios' if mode=='trace-csp' else 'solver unavailable, late, or rejected; fallback verified'
                audit=dict(result.audit)
                audit.update({'mode':mode,'solver_status':result.status,'reason':reason,
                              'certificate_scope':cert.scope,'fallback_activated':mode!='trace-csp'})
                return Decision(a,mode,cert,result,reason,perf_counter()-start,audit)
        if self.mode=='degrade':
            degraded=self._relaxed(instance,start,'lexicographic',result.unsat_core or None,result)
            if degraded: return degraded
        return self._halt(result,start)
    def _halt(self,result,start):
        self.previous=None
        reason='timeout/infeasibility/rejected fallback; external intervention required'
        certificate=Certificate(False,['all candidate controls rejected'])
        audit=dict(result.audit)
        audit.update({'mode':'no-certified-action','solver_status':result.status,'reason':reason,
                      'certificate_scope':certificate.scope,'fallback_activated':True})
        return Decision(None,'no-certified-action',certificate,result,reason,perf_counter()-start,audit)
