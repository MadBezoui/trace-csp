"""Closed-loop TRACE-CSP ramp metering on the NCHRP I-210 SUMO benchmark.

Every 30 s (the meter update period) the controller builds a CTM instance of the
metered section (trace_csp.sumo_i210), solves it and issues its first command.
Commands are held over the whole period, in the model (Config.control_block = 30 s / dt)
and in SUMO. A metered ramp's command is the number of green phases per period; the
model releases at most the vehicles allowed by the greens that start in each 10-s step
(sumo_i210.meter_release_table), and the meter executes exactly that signal plan. In
shield mode, a command equal to ALINEA's keeps ALINEA's plan, which is the same plan.

State estimation: --state detectors (default) converts 30-s loop occupancy to cell
density with the PeMS-calibrated effective length and interpolates cells without
loops; ramp queues come from the ramp lane-area (E2) detectors used by HERO.
--state oracle reads vehicle counts from SUMO (perfect observability, ablation).
Entry and exit flows are counted at edge entries (equivalent to loop detectors).

Other disclosed simplifications:
- Demand and exit shares are the measured counts of the last 30 s; scenarios scale
  them by 0.85 / 1.00 / 1.15. Incidents are not known to the controller.

Each decision is logged with the observed state, the measured demand and exit shares,
the requested, reference and executed commands, the per-stage solver proofs and the
vehicles released in the previous period, so scripts/replay_i210_decisions.py can
rebuild and re-verify it.
"""
import argparse
import json
import sys
from pathlib import Path
from time import perf_counter
import xml.etree.ElementTree as ET

from dataclasses import replace,asdict
from trace_csp.domain import Action,State,Config,Instance
from trace_csp.csp.controller import Controller
from trace_csp.sumo_i210 import extract_layout,corridor_from_layout,meter_commands,demand_scenarios,BENCHMARK
from trace_csp.provenance import provenance

PERIOD_S=30


class Counter:
    """Counts vehicles newly present on a set of edges (entries), sampled every step."""
    def __init__(self,traci,edges):
        self.traci=traci; self.edges=list(edges); self.seen=set(); self.count=0
    def step(self):
        now=set()
        for e in self.edges: now.update(self.traci.edge.getLastStepVehicleIDs(e))
        self.count+=len(now-self.seen); self.seen=now
    def take(self):
        c=self.count; self.count=0; return c


class LoopCounter(Counter):
    """Counts vehicles newly detected by a set of induction loops."""
    def step(self):
        now=set()
        for e in self.edges: now.update(self.traci.inductionloop.getLastStepVehicleIDs(e))
        self.count+=len(now-self.seen); self.seen=now


def greens(meter):
    """Green phases in the meter's current plan, i.e. the command it executes."""
    plan=meter.meteringPlan; g=meter.phaseIDs[0]
    return sum(x==g and (k==0 or plan[k-1]!=g) for k,x in enumerate(plan))



def main():
    p=argparse.ArgumentParser(); p.add_argument('--seconds',type=int,default=1800); p.add_argument('--seed',type=int,default=5)
    p.add_argument('--output',required=True); p.add_argument('--repository',default='data/raw/i210')
    p.add_argument('--horizon',type=int,default=6); p.add_argument('--time-limit',type=float,default=10.)
    p.add_argument('--mode',choices=('degrade','hard','soft','alinea','hero','shield'),default='degrade')
    p.add_argument('--state',choices=('detectors','oracle'),default='detectors'); p.add_argument('--effective-length-m',type=float,default=9.8)
    p.add_argument('--calibration'); p.add_argument('--no-congestion',action='store_true')
    p.add_argument('--scenario-root',help='directory holding I210.sumocfg.xml (default: ALINEA benchmark scenario)')
    p.add_argument('--storage-fraction',type=float,default=0.8,help='modeled ramp storage as a share of the benchmark maxVehNum')
    p.add_argument('--capacity-drop',type=int,default=0,help='modeled capacity drop above critical occupancy, per mille')
    p.add_argument('--approach-count',choices=('all','halting'),default='all',help='vehicles counted on the ramp approach upstream of the E2 area')
    p.add_argument('--reference-weighting',choices=('uniform','increasing'),default='uniform')
    p.add_argument('--max-rate-change',type=int,help='largest change of a meter command (green phases per period) between periods (default: unconstrained)'); p.add_argument('--workers',type=int,default=8)
    a=p.parse_args()
    base=Path(a.repository).resolve()/BENCHMARK; root=next(base.rglob('I210.sumocfg.xml')).parent
    if a.scenario_root: root=Path(a.scenario_root).resolve()
    sys.path.insert(0,str(base)); import freewayControl as fc
    import traci
    from sumolib import checkBinary
    layout=extract_layout(a.repository); cells=layout['cells']; dt=layout['dt_s']; block=PERIOD_S//dt
    if a.horizon%block: p.error(f'horizon must be a multiple of {block} steps')
    inputs=[a.calibration,base/'freewayControl.py',base/'meterConfig.json',root/'I210.sumocfg.xml']
    run_provenance=provenance(inputs)
    out=Path(a.output); out.parent.mkdir(parents=True,exist_ok=True)
    tripinfo=out.with_suffix('.tripinfo.xml')
    (root/'Output').mkdir(exist_ok=True)  # detector outputs of the scenario are written there
    traci.start([checkBinary('sumo'),'-c',str(root/'I210.sumocfg.xml'),'--step-length','1','--seed',str(a.seed),
                 '--no-warnings','true','--collision.action','none','--tripinfo-output',str(tripinfo)])
    env=fc.SimulationEnvironment(traci.getConnection())
    meters_cfg=json.loads((base/'meterConfig.json').read_text()); ramps_cfg=json.loads((base/'onrampConfig.json').read_text())
    area=json.loads((base/'onrampLaneAreaConfig.json').read_text())
    detectors={e.get('id'):(e.get('lane') or e.get('lanes')) for e in ET.parse(root/'I210.detector.xml').getroot()}
    meters={}
    for name,cfg in meters_cfg.items():
        m=fc.RampMeter(env,cfg['meterID'],[cfg['greenPhaseID'],cfg['redPhaseID']],numRampLane=ramps_cfg[name]['numRampLane'],
                       numVehPerGreenPerLane=cfg['numVehPerGreenPerLane'],greenPhaseLen=cfg['greenPhaseLen'],redPhaseLen=cfg['minRedPhaseLen'],
                       minRedPhaseLen=cfg['minRedPhaseLen'],maxRedPhaseLen=cfg['maxRedPhaseLen'])
        m.generateMeteringPlan([m.greenPhaseLen,m.redPhaseLen],PERIOD_S); meters[name]=m
    red_for={name:meter_commands(cfg,PERIOD_S) for name,cfg in meters_cfg.items()}
    ramp_edges={name:sorted({l.rsplit('_',1)[0] for d in cfg['rampLaneAreaDetectors'] for l in detectors[d].split()}) for name,cfg in area.items()}
    inflows=[(j,c['inflow']) for j,c in enumerate(cells) if c['inflow']]
    # SUMO inserts mainline demand on the first section edge (S25_M); nothing enters from upstream.
    boundary=Counter(traci,[layout['first_edge']])
    exits=[Counter(traci,c['exits']) for c in cells]
    through=[Counter(traci,[cells[j+1]['edges'][0]]) for j in range(len(cells)-1)]+[Counter(traci,[])]
    alinea={}; hero=None
    loops=lambda f,key:{k:fc.InductionLoop(env,v[key],v['period']) for k,v in json.loads((base/f).read_text()).items()}
    metering=loops('onrampMeteringInductionLoopConfig.json','mainlineInductionLoops')
    entry=loops('onrampEntryInductionLoopConfig.json','rampEntryInductionLoops')
    mainline_loops=loops('mainlineInductionLoopConfig.json','mainlineInductionLoops')
    lanearea={k:fc.LaneAreaDetector(env,v['rampLaneAreaDetectors'],v['period']) for k,v in area.items()}
    sensors=[*metering.values(),*entry.values(),*mainline_loops.values(),*lanearea.values()]
    exit_loops=json.loads((base/'onrampExitInductionLoopConfig.json').read_text())
    released={k:LoopCounter(traci,v['rampExitInductionLoops']) for k,v in exit_loops.items() if k in meters}
    if a.mode in ('alinea','shield'):
        # Upstream ALINEA with its original gains, setpoint, detectors and queue override.
        alinea={k:fc.ALINEA(env,meters[k],meteringUpdateFreq=30,gain=70,mainlineSetpoint=15,hasRampQueueOverride=True) for k in meters}
    if a.mode=='hero':
        # Upstream HERO: queue-informed ALINEA (gain 30, 80% storage) plus the coordinator.
        alinea={k:fc.QueueInformedALINEA(env,meters[k],meteringUpdateFreq=30,gain=30,mainlineSetpoint=15,
                                         allowedMaxRampQueueLen=0.8*ramps_cfg[k]['maxVehNum'],hasRampQueueOverride=True) for k in meters}
        hero=fc.HeuristicRampMeteringCoordinator(list(alinea.values()))
    all_edges=traci.edge.getIDList()
    upstream_edges={name:[e for e in all_edges if e.startswith(edges[0].rsplit('_',1)[0]+'_') and e not in edges and int(e.rsplit('_',1)[1])<int(min(edges).rsplit('_',1)[1])]
                    for name,edges in ramp_edges.items()}
    series=[]
    # Ramp demand is counted where vehicles enter the ramp system (origin and detected edges).
    arrivals=[Counter(traci,upstream_edges[inf['name']]+ramp_edges[inf['name']]) if inf['kind']=='metered' else Counter(traci,inf['edges']) for _,inf in inflows]
    cell_of={e:j for j,c in enumerate(cells) for e in c['edges']}
    loop_cells=[]
    for group in [*mainline_loops.values(),*metering.values()]:
        edge=traci.lane.getEdgeID(traci.inductionloop.getLaneID(group.inductionLoopIDs[0]))
        if edge in cell_of: loop_cells.append((cell_of[edge],group))
    controller=Controller(mode={'shield':'degrade'}.get(a.mode,a.mode if a.mode in ('hard','degrade','soft') else 'hard')); controller.reference_weighting=a.reference_weighting; previous=None; log=[]; worst_excess=0; excess_vs=0
    vht=0; backlog=0; ramp_vs=0; spill_vs=0; halt_vs=0; queue_excess_vs=0; ramp_excess={k:0 for k in ramp_edges}
    corridor=corridor_from_layout(layout,period_s=PERIOD_S); model=None
    for t in range(a.seconds):
        for m in meters.values(): m.run()
        env.update()
        for c in [boundary,*arrivals,*exits,*through,*released.values()]: c.step()
        for sensor in sensors: sensor.run()
        vht+=traci.vehicle.getIDCount(); backlog+=len(traci.simulation.getPendingVehicles())
        for name,edges in ramp_edges.items():
            onramp=sum(traci.edge.getLastStepVehicleNumber(e) for e in edges); ramp_vs+=onramp
            over=max(0,onramp-ramps_cfg[name]['maxVehNum'])
            worst_excess=max(worst_excess,over); excess_vs+=over; ramp_excess[name]+=over
            spill_vs+=sum(traci.edge.getLastStepVehicleNumber(e) for e in upstream_edges[name])
            halting=sum(traci.edge.getLastStepHaltingNumber(e) for e in upstream_edges[name]); halt_vs+=halting
            # Queue as the shield counts it (area plus halting approach vehicles) above the full storage.
            queue_excess_vs+=max(0,onramp+halting-ramps_cfg[name]['maxVehNum'])
        if env.step%PERIOD_S: continue
        series.append({'t':env.step,'pending':len(traci.simulation.getPendingVehicles()),
                       'ramp_area':{k:int(v.numVeh) for k,v in lanearea.items()},
                       'ramp_upstream':{k:sum(traci.edge.getLastStepVehicleNumber(e) for e in es) for k,es in upstream_edges.items()},
                       'queue_indicator':{k:bool(v.queueIndicator) for k,v in entry.items()},
                       'first_cell_halting':traci.edge.getLastStepHaltingNumber(layout['first_edge'])})
        if hero:
            hero.updateMeteringPlans([{'nearbyDownstreamMainlineOcc':metering[k].edgeOcc,'rampQueueLen':lanearea[k].numVeh,
                                       'rampDemand':entry[k].edgeFlow,'rampQueueIndicator':entry[k].queueIndicator} for k in alinea])
            continue
        if alinea:
            for k,ctl in alinea.items():
                ctl.updateMeteringPlan(nearbyDownstreamMainlineOcc=metering[k].edgeOcc,rampQueueIndicator=entry[k].queueIndicator)
            if a.mode!='shield': continue
        started=perf_counter()
        b=boundary.take(); d=[x.take() for x in arrivals]
        ex=[x.take() for x in exits]; th=[x.take() for x in through]
        shares=tuple(min(999,round(1000*e/(e+h))) if e+h else 0 for e,h in zip(ex,th))
        corridor=corridor_from_layout(layout,exit_permille=shares,calibration=a.calibration,period_s=PERIOD_S)
        corridor=replace(corridor,queue_max=tuple(int(k*a.storage_fraction) if k<10**6 else k for k in corridor.queue_max),capacity_drop_permille=a.capacity_drop)
        if a.state=='oracle':
            n=tuple(min(corridor.jam[j],sum(traci.edge.getLastStepVehicleNumber(e) for e in c['edges'])) for j,c in enumerate(cells))
            q=tuple(sum(traci.edge.getLastStepVehicleNumber(e) for e in ramp_edges[inf['name']]+upstream_edges[inf['name']]) if inf['kind']=='metered' else 0 for _,inf in inflows)
        else:
            density={}
            for j,group in loop_cells: density.setdefault(j,[]).append(group.edgeOcc/100/(a.effective_length_m/1000))
            known=sorted((j,sum(v)/len(v)) for j,v in density.items())
            def at(j):
                below=[x for x in known if x[0]<=j]; above=[x for x in known if x[0]>=j]
                if not below: return above[0][1]
                if not above: return below[-1][1]
                (j0,k0),(j1,k1)=below[-1],above[0]
                return k0 if j0==j1 else k0+(k1-k0)*(j-j0)/(j1-j0)
            n=tuple(max(0,min(corridor.jam[j],round(at(j)*c['lanes']*c['length']/1000))) for j,c in enumerate(cells))
            # Ramp-entrance queue detection is assumed: vehicles on the approach upstream of the E2 area are counted.
            approach=traci.edge.getLastStepHaltingNumber if a.approach_count=='halting' else traci.edge.getLastStepVehicleNumber
            q=tuple(int(lanearea[inf['name']].numVeh)+sum(approach(e) for e in upstream_edges[inf['name']]) if inf['kind']=='metered' else 0 for _,inf in inflows)
        prev=previous or Action(tuple(corridor.ramp_max),(layout['vmax_kmh'],)*len(cells))
        cfg=Config(horizon=a.horizon,dt_s=dt,vsl_values=(layout['vmax_kmh'],),max_dv=0,max_dm=a.max_rate_change if a.max_rate_change is not None else max(corridor.ramp_max),spatial_dv=0,
                   time_limit_s=a.time_limit,seed=a.seed,workers=a.workers,congestion_penalty=not a.no_congestion,control_block=block)
        inst=Instance(corridor,State(n,q,0,prev),demand_scenarios(b,d,a.horizon,block,len(cells)),cfg,f'i210-{env.step}')
        if model is None: model={'corridor':asdict(replace(corridor,exit_permille=())),'config':asdict(cfg)}
        if a.mode=='shield':
            # Reference: the command of ALINEA's plan for this period (green phases per period).
            controller.reference=[greens(meters[inf['name']]) if inf['kind']=='metered' else corridor.ramp_max[k] for k,(_,inf) in enumerate(inflows)]
        dec=controller.act(inst)
        action=dec.action or prev
        executed=[]; red=[]
        for k,((j,inf),u) in enumerate(zip(inflows,action.ramp)):
            if inf['kind']!='metered': executed.append(u); continue
            m=meters[inf['name']]
            # In shield mode a halt or an unchanged command keeps ALINEA's plan.
            if not (a.mode=='shield' and (dec.action is None or u==controller.reference[k])):
                m.redPhaseLen=red_for[inf['name']][u]; m.generateMeteringPlan([m.greenPhaseLen,m.redPhaseLen],PERIOD_S)
            m.timer=0; executed.append(greens(m)); red.append(m.redPhaseLen)
        if dec.action is not None and tuple(executed)!=action.ramp: raise RuntimeError('executed command differs from the verified one')
        previous=Action(tuple(executed),action.vsl)
        audit=dec.audit
        log.append({'t':env.step,'mode':dec.mode,'trigger':audit.get('trigger'),'reason':dec.reason,'solve_s':perf_counter()-started,
                    'status':audit.get('solver_status'),'relaxed_status':audit.get('relaxed_status'),
                    'stages':audit.get('stages'),'hard_stages':audit.get('hard_stages'),
                    'certified_core':audit.get('certified_core'),'planned_excess':audit.get('planned_max_queue_excess'),
                    'exceedance_proved_minimal':audit.get('exceedance_proved_minimal'),'deviation_proved_minimal':audit.get('deviation_proved_minimal'),
                    'requested':list(dec.action.ramp) if dec.action else None,'reference':controller.reference,'executed':executed,'red_s':red,
                    'released_last_period':{k:c.take() for k,c in released.items()},
                    'state':{'vehicles':list(n),'queues':list(q),'previous':list(prev.ramp)},'boundary_in':b,'ramp_demand':d,'exit_permille':list(shares)})
    in_network=traci.vehicle.getIDCount(); pending_end=len(traci.simulation.getPendingVehicles())
    traci.close()
    count=duration=waiting=0
    for _,el in ET.iterparse(tripinfo,events=('end',)):
        if el.tag=='tripinfo':
            count+=1; duration+=float(el.get('duration','0')); waiting+=float(el.get('waitingTime','0')); el.clear()
    modes={}
    for row in log: modes[row['mode']]=modes.get(row['mode'],0)+1
    report={'scenario':'NCHRP I-210 closed loop (common measurement harness)','mode':a.mode,'state':a.state,'calibration':a.calibration,'congestion_penalty':not a.no_congestion,'scenario_root':str(root),'horizon':a.horizon,'time_limit_s':a.time_limit,'workers':a.workers,'seed':a.seed,'seconds':a.seconds,'cells':len(cells),
            'arrived_trips':count,'completed_trip_duration_s':duration,'completed_trip_waiting_s':waiting,
            'network_vehicle_seconds':vht,'insertion_backlog_vehicle_seconds':backlog,'vehicles_in_network_end':in_network,'pending_vehicles_end':pending_end,
            'ramp_vehicle_seconds':ramp_vs,'ramp_approach_vehicle_seconds':spill_vs,'ramp_approach_halting_vehicle_seconds':halt_vs,
            'controller_queue_excess_vehicle_seconds':queue_excess_vs,'actuation':f'commands held {PERIOD_S} s; meter command = green phases per period','control_period_s':PERIOD_S,'storage_fraction':a.storage_fraction,'capacity_drop_permille':a.capacity_drop,'approach_count':a.approach_count,'reference_weighting':a.reference_weighting,'max_rate_change':a.max_rate_change,'ramp_excess_by_ramp':ramp_excess,            'ramp_storage_excess_vehicle_seconds':excess_vs,'max_ramp_storage_excess':worst_excess,'modes':modes,
            'median_decision_s':sorted(r['solve_s'] for r in log)[len(log)//2] if log else None,
            'max_decision_s':max((r['solve_s'] for r in log),default=None),'model':model,
            'provenance':{**run_provenance,'load_average_end':list(__import__('os').getloadavg())},'decisions':log,'series':series}
    out.write_text(json.dumps(report,indent=1))
    print(json.dumps({k:v for k,v in report.items() if k!='decisions'},indent=1))


if __name__=='__main__': main()
