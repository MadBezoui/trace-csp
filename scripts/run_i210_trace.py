"""Closed-loop TRACE-CSP ramp metering on the NCHRP I-210 SUMO benchmark.

Every 30 s the controller builds a CTM instance of the metered section
(trace_csp.sumo_i210), solves it, and converts the first planned release rate to
a red-phase length with the upstream RampMeter conversion used by ALINEA.

State estimation: --state detectors (default) converts 30-s loop occupancy to cell
density with the PeMS-calibrated effective length and interpolates cells without
loops; ramp queues come from the ramp lane-area (E2) detectors used by HERO.
--state oracle reads vehicle counts from SUMO (perfect observability, ablation).
Entry and exit flows are counted at edge entries (equivalent to loop detectors).

Other disclosed simplifications:
- Demand and exit shares are the measured counts of the last 30 s; scenarios scale
  them by 0.85 / 1.00 / 1.15. Incidents are not known to the controller.
- The model plans in 10 s steps; the first step's rate is held for 30 s.
"""
import argparse
import json
import sys
from pathlib import Path
from time import perf_counter
import xml.etree.ElementTree as ET

from dataclasses import replace
from trace_csp.domain import Action,State,Scenario,Config,Instance
from trace_csp.csp.controller import Controller
from trace_csp.sumo_i210 import extract_layout,corridor_from_layout,BENCHMARK


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
    p.add_argument('--max-rate-change',type=int,help='largest metering-rate change per 10 s step (default: unconstrained)'); p.add_argument('--workers',type=int,default=8)
    a=p.parse_args()
    base=Path(a.repository).resolve()/BENCHMARK; root=next(base.rglob('I210.sumocfg.xml')).parent
    if a.scenario_root: root=Path(a.scenario_root).resolve()
    sys.path.insert(0,str(base)); import freewayControl as fc
    import traci
    from sumolib import checkBinary
    layout=extract_layout(a.repository); cells=layout['cells']; dt=layout['dt_s']
    out=Path(a.output); out.parent.mkdir(parents=True,exist_ok=True)
    tripinfo=out.with_suffix('.tripinfo.xml')
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
        m.generateMeteringPlan([m.greenPhaseLen,m.redPhaseLen],30); meters[name]=m
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
    vht=0; backlog=0; ramp_vs=0; spill_vs=0; ramp_excess={k:0 for k in ramp_edges}
    corridor=corridor_from_layout(layout)
    for t in range(a.seconds):
        for m in meters.values(): m.run()
        env.update()
        for c in [boundary,*arrivals,*exits,*through]: c.step()
        for sensor in sensors: sensor.run()
        vht+=traci.vehicle.getIDCount(); backlog+=len(traci.simulation.getPendingVehicles())
        for name,edges in ramp_edges.items():
            onramp=sum(traci.edge.getLastStepVehicleNumber(e) for e in edges); ramp_vs+=onramp
            over=max(0,onramp-ramps_cfg[name]['maxVehNum'])
            worst_excess=max(worst_excess,over); excess_vs+=over; ramp_excess[name]+=over
            spill_vs+=sum(traci.edge.getLastStepVehicleNumber(e) for e in upstream_edges[name])
        if env.step%30: continue
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
        corridor=corridor_from_layout(layout,exit_permille=shares,calibration=a.calibration)
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
                   time_limit_s=a.time_limit,seed=a.seed,workers=a.workers,congestion_penalty=not a.no_congestion)
        scen=tuple(Scenario(w,tuple([round(f*b/3)]*a.horizon),tuple([tuple(round(f*x/3) for x in d)]*a.horizon),
                            tuple([(1000,)*len(cells)]*a.horizon)) for w,f in ((1,.85),(2,1.),(1,1.15)))
        inst=Instance(corridor,State(n,q,0,prev),scen,cfg,f'i210-{env.step}')
        if a.mode=='shield':
            # Reference: ALINEA's rate for this period, in vehicles per model step.
            controller.reference=[min(corridor.ramp_max[k],max(corridor.ramp_min[k],round(alinea[inf['name']].meteredFlowImplemented*dt/3600)))
                                  if inf['kind']=='metered' else corridor.ramp_max[k] for k,(_,inf) in enumerate(inflows)]
        dec=controller.act(inst)
        action=dec.action or prev
        for (j,inf),u in zip(inflows,action.ramp):
            if inf['kind']!='metered': continue
            if a.mode=='shield' and controller.reference is not None and u==controller.reference[inflows.index((j,inf))]:
                continue  # unchanged by the shield: keep ALINEA's own metering plan
            m=meters[inf['name']]
            lo=m.convertRedPhaseLenToMeteredFlow(m.maxRedPhaseLen,30); hi=m.convertRedPhaseLenToMeteredFlow(m.minRedPhaseLen,30)
            flow=min(hi,max(lo,u*3600//dt))
            m.redPhaseLen=m.convertMeteredFlowToRedPhaseLen(flow,30); m.generateMeteringPlan([m.greenPhaseLen,m.redPhaseLen],30); m.timer=0
        previous=action if dec.action else None
        log.append({'t':env.step,'mode':dec.mode,'solve_s':perf_counter()-started,'status':dec.audit.get('solver_status'),
                    'certified_core':dec.audit.get('certified_core'),'planned_excess':dec.audit.get('planned_max_queue_excess'),
                    'ramp':list(action.ramp),'reference':controller.reference,'boundary_in':b,'exit_permille':list(shares)})
    in_network=traci.vehicle.getIDCount()
    traci.close()
    count=duration=waiting=0
    for _,el in ET.iterparse(tripinfo,events=('end',)):
        if el.tag=='tripinfo':
            count+=1; duration+=float(el.get('duration','0')); waiting+=float(el.get('waitingTime','0')); el.clear()
    modes={}
    for row in log: modes[row['mode']]=modes.get(row['mode'],0)+1
    report={'scenario':'NCHRP I-210 closed loop (common measurement harness)','mode':a.mode,'state':a.state,'calibration':a.calibration,'congestion_penalty':not a.no_congestion,'scenario_root':str(root),'horizon':a.horizon,'time_limit_s':a.time_limit,'workers':a.workers,'seed':a.seed,'seconds':a.seconds,'cells':len(cells),
            'arrived_trips':count,'completed_trip_duration_s':duration,'completed_trip_waiting_s':waiting,
            'network_vehicle_seconds':vht,'insertion_backlog_vehicle_seconds':backlog,'vehicles_in_network_end':in_network,
            'ramp_vehicle_seconds':ramp_vs,'ramp_approach_vehicle_seconds':spill_vs,'storage_fraction':a.storage_fraction,'capacity_drop_permille':a.capacity_drop,'approach_count':a.approach_count,'reference_weighting':a.reference_weighting,'max_rate_change':a.max_rate_change,'ramp_excess_by_ramp':ramp_excess,            'ramp_storage_excess_vehicle_seconds':excess_vs,'max_ramp_storage_excess':worst_excess,'modes':modes,
            'median_decision_s':sorted(r['solve_s'] for r in log)[len(log)//2] if log else None,
            'max_decision_s':max((r['solve_s'] for r in log),default=None),'decisions':log,'series':series}
    out.write_text(json.dumps(report,indent=1))
    print(json.dumps({k:v for k,v in report.items() if k!='decisions'},indent=1))


if __name__=='__main__': main()
