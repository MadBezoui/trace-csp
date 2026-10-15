"""Typed inputs. Counts/flows are integer vehicle units, lengths metres, speeds km/h."""
from dataclasses import dataclass
from math import isfinite


def integers(values, minimum=0):
    return all(type(x) is int and x >= minimum for x in values)


@dataclass(frozen=True)
class Action:
    ramp: tuple[int, ...]
    vsl: tuple[int, ...]


@dataclass(frozen=True)
class Corridor:
    length_m: tuple[int, ...]
    lanes: tuple[int, ...]
    jam: tuple[int, ...]
    capacity: tuple[int, ...]
    ramp_segment: tuple[int, ...]
    ramp_max: tuple[int, ...]
    queue_max: tuple[int, ...]
    wave_speed_kmh: int = 20
    vehicle_scale: int = 1
    # Per-mille share of each cell's downstream outflow that leaves by an off-ramp
    # before the next cell (ignored for the last cell). Empty means no off-ramps.
    exit_permille: tuple[int, ...] = ()
    # Minimum metering rate per ramp; ramp_min == ramp_max models an unmetered ramp.
    ramp_min: tuple[int, ...] = ()
    # Capacity drop: above its critical occupancy a cell discharges at most
    # capacity*(1000-capacity_drop_permille)//1000 (0 = triangular CTM).
    capacity_drop_permille: int = 0
    # Device release table per ramp: () means a command u releases at most u vehicles in
    # every step; otherwise entry [u][k] is the release capacity of command u at step k of
    # its actuation period (Config.control_block steps), e.g. green phases of a meter.
    release_table: tuple[tuple[tuple[int, ...], ...], ...] = ()

    def __post_init__(self):
        n, r = len(self.jam), len(self.ramp_segment)
        if not n or any(len(v) != n for v in (self.length_m,self.lanes,self.capacity)):
            raise ValueError('inconsistent corridor dimensions')
        if any(len(v) != r for v in (self.ramp_max,self.queue_max)):
            raise ValueError('inconsistent ramp dimensions')
        for v in (self.length_m,self.lanes,self.jam,self.capacity):
            if not integers(v, 1): raise ValueError('positive integer corridor quantities required')
        if not integers(self.ramp_max) or not integers(self.queue_max):
            raise ValueError('nonnegative ramp bounds required')
        if not integers(self.ramp_segment) or len(set(self.ramp_segment)) != r or any(i >= n for i in self.ramp_segment):
            raise ValueError('one ramp per valid segment is supported')
        if not integers((self.wave_speed_kmh,self.vehicle_scale),1): raise ValueError('invalid scale or wave speed')
        if not self.exit_permille: object.__setattr__(self,'exit_permille',(0,)*n)
        if not self.ramp_min: object.__setattr__(self,'ramp_min',(0,)*r)
        if len(self.exit_permille)!=n or not integers(self.exit_permille) or any(b>=1000 for b in self.exit_permille):
            raise ValueError('exit shares must be per-mille integers below 1000')
        if len(self.ramp_min)!=r or not integers(self.ramp_min) or any(a>b for a,b in zip(self.ramp_min,self.ramp_max)):
            raise ValueError('invalid minimum ramp rates')
        if not integers((self.capacity_drop_permille,)) or self.capacity_drop_permille>=1000: raise ValueError('invalid capacity drop')
        if not self.release_table: object.__setattr__(self,'release_table',((),)*r)
        if len(self.release_table)!=r: raise ValueError('one release table entry per ramp required')
        for table,top in zip(self.release_table,self.ramp_max):
            if table and (len(table)!=top+1 or len({len(row) for row in table})!=1 or not all(integers(row) for row in table)):
                raise ValueError('release table must give one nonnegative pattern per command 0..ramp_max')


@dataclass(frozen=True)
class State:
    vehicles: tuple[int, ...]
    queues: tuple[int, ...]
    boundary_queue: int
    previous: Action


@dataclass(frozen=True)
class Scenario:
    weight: int
    mainline: tuple[int, ...]
    ramps: tuple[tuple[int, ...], ...]
    capacity_factor: tuple[tuple[int, ...], ...]  # 0..1000


@dataclass(frozen=True)
class Config:
    horizon: int = 3
    dt_s: int = 10
    vsl_values: tuple[int, ...] = (40,50,60,70,80,90,100,110)
    max_dv: int = 20
    max_dm: int = 2
    spatial_dv: int = 20
    time_limit_s: float = 1.0
    workers: int = 1
    seed: int = 0
    alpha_percent: int = 90
    fairness: bool = True
    fairness_limit: int | None = None
    # Surrogate for capacity drop (not represented by the triangular CTM): penalize
    # vehicles above each cell's critical occupancy, before TTS in the lexicographic order.
    congestion_penalty: bool = False
    # Steps per actuation period: commands may change only every control_block steps,
    # and an issued command is held, and verified, over a whole period.
    control_block: int = 1

    def __post_init__(self):
        if not integers((self.horizon,self.dt_s,self.workers,self.control_block),1): raise ValueError('positive configuration required')
        if self.horizon%self.control_block: raise ValueError('horizon must be a multiple of the control block')
        if not self.vsl_values or not integers(self.vsl_values,1) or len(set(self.vsl_values)) != len(self.vsl_values):
            raise ValueError('invalid speed domain')
        if not integers((self.max_dv,self.max_dm,self.spatial_dv,self.seed,self.alpha_percent)) or self.alpha_percent >= 100:
            raise ValueError('invalid limits')
        if not isfinite(self.time_limit_s) or self.time_limit_s < 0: raise ValueError('invalid deadline')
        if self.fairness_limit is not None and not integers((self.fairness_limit,)): raise ValueError('invalid fairness bound')


@dataclass(frozen=True)
class Instance:
    corridor: Corridor
    state: State
    scenarios: tuple[Scenario, ...]
    config: Config
    instance_id: str = 'anonymous'
    group: str = 'synthetic-0'
    family: str = 'normal'
    split: str = 'train'

    def __post_init__(self):
        c, s, cfg = self.corridor,self.state,self.config
        n,r,h = len(c.jam),len(c.ramp_segment),cfg.horizon
        if len(s.vehicles)!=n or len(s.queues)!=r or len(s.previous.ramp)!=r or len(s.previous.vsl)!=n:
            raise ValueError('state dimension mismatch')
        if not integers(s.vehicles+s.queues+(s.boundary_queue,)):
            raise ValueError('nonnegative integer state required')
        # A queue above storage is a valid observed state; it makes the hard problem infeasible at t=0.
        if any(a>b for a,b in zip(s.vehicles,c.jam)):
            raise ValueError('initial state outside physical bounds')
        if not integers(s.previous.ramp) or any(not lo<=a<=hi for a,lo,hi in zip(s.previous.ramp,c.ramp_min,c.ramp_max)) or any(v not in cfg.vsl_values for v in s.previous.vsl):
            raise ValueError('invalid previous action')
        if any(max(max(cfg.vsl_values),c.wave_speed_kmh)*cfg.dt_s*10 > 36*l for l in c.length_m):
            raise ValueError('CFL condition violated: speed * dt must not exceed cell length')
        if any(table and len(table[0])!=cfg.control_block for table in c.release_table):
            raise ValueError('release patterns must span one control block')
        if not self.scenarios: raise ValueError('at least one uncertainty scenario required')
        for w in self.scenarios:
            if not integers((w.weight,),1) or len(w.mainline)!=h or len(w.ramps)!=h or len(w.capacity_factor)!=h:
                raise ValueError('scenario horizon/weight mismatch')
            if not integers(w.mainline) or any(len(v)!=r or not integers(v) for v in w.ramps):
                raise ValueError('invalid scenario demand')
            if any(len(v)!=n or not integers(v) or any(x>1000 for x in v) for v in w.capacity_factor):
                raise ValueError('invalid incident factors')


def critical_occupancy(c,cfg,j):
    """Largest occupancy whose free-flow sending at the top speed stays within nominal capacity."""
    return c.capacity[j]*36*c.length_m[j]//(max(cfg.vsl_values)*cfg.dt_s*10)


def release_capacity(c,cfg,r,command,t):
    """Largest release of ramp r at plan step t under a metering command."""
    table=c.release_table[r]
    return table[command][t%cfg.control_block] if table else command


def discharge_capacity(c,cfg,j,cap,n):
    """Sending capacity of cell j at occupancy n given its incident-adjusted capacity cap."""
    return cap*(1000-c.capacity_drop_permille)//1000 if n>critical_occupancy(c,cfg,j) else cap
