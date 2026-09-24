"""Integer triangular cell transmission model with ramp-priority merging.

Each sending/receiving quantity is computed from the pre-step state.
Unadmitted upstream arrivals remain in an explicit boundary queue.
"""
from dataclasses import dataclass
from trace_csp.domain import Action, Corridor, State, Scenario, Config, discharge_capacity


@dataclass(frozen=True)
class Transition:
    state: State
    outflow: tuple[int, ...]
    ramp_release: tuple[int, ...]
    admitted: int
    offramp: tuple[int, ...] = ()


def step(c: Corridor, state: State, action: Action, demand: Scenario, t: int, cfg: Config) -> Transition:
    n = len(c.jam)
    if len(action.ramp)!=len(c.ramp_segment) or len(action.vsl)!=n:
        raise ValueError('action dimensions')
    if any(type(x) is not int or not lo<=x<=m for x,lo,m in zip(action.ramp,c.ramp_min,c.ramp_max)) or any(v not in cfg.vsl_values for v in action.vsl):
        raise ValueError('action outside domain')
    capacity = [q*f//1000 for q,f in zip(c.capacity,demand.capacity_factor[t])]
    sending = [min(discharge_capacity(c,cfg,i,capacity[i],state.vehicles[i]),state.vehicles[i]*action.vsl[i]*cfg.dt_s*10//(36*c.length_m[i])) for i in range(n)]
    receiving = [min(capacity[i],(c.jam[i]-state.vehicles[i])*c.wave_speed_kmh*cfg.dt_s*10//(36*c.length_m[i])) for i in range(n)]
    release = tuple(min(action.ramp[r],state.queues[r]+demand.ramps[t][r],receiving[i]) for r,i in enumerate(c.ramp_segment))
    ramp_in = dict(zip(c.ramp_segment,release))
    room = [receiving[i]-ramp_in.get(i,0) for i in range(n)]
    # FIFO diverge: the cell outflow is limited so that its continuing share fits downstream.
    flow = tuple(min(sending[i],room[i+1]*1000//(1000-c.exit_permille[i])) if i+1<n else sending[i] for i in range(n))
    through = tuple(flow[i]*(1000-c.exit_permille[i])//1000 for i in range(n-1))
    offramp = tuple(flow[i]-through[i] for i in range(n-1))
    admitted = min(state.boundary_queue+demand.mainline[t],room[0])
    vehicles = tuple(state.vehicles[i]+(admitted if i==0 else through[i-1])+ramp_in.get(i,0)-flow[i] for i in range(n))
    queues = tuple(q+d-m for q,d,m in zip(state.queues,demand.ramps[t],release))
    return Transition(State(vehicles,queues,state.boundary_queue+demand.mainline[t]-admitted,action),flow,release,admitted,offramp)
