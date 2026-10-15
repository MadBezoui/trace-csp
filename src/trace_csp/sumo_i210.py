"""CTM layout of the metered I-210 section extracted from the NCHRP SUMO network.

Mainline general-purpose edges S{first}_M..S{last}_M are grouped into cells of at
least the CFL length. Each cell holds at most one inflow: a metered on-ramp
(meterConfig.json) or an HOV-to-GP merge, modeled as an unmetered ramp. Off-ramps
and GP-to-HOV diverges become per-mille exit shares at the cell's downstream end.

Meter commands. Every update period the upstream RampMeter repeats a cycle of a fixed
green phase and a red phase, so the device can only issue N = period // (green + red)
green phases per period. The model command of a metered ramp is this N, held over the
period, and its release capacity at each model step is the number of green phases that
start within that step, times the lanes and the vehicles allowed per green
(meter_release_table). The upstream ALINEA conversion always issues the red phase
period // N - green, so ALINEA's commands lie in the same set.
"""
from math import ceil
from pathlib import Path
import json
import re
import xml.etree.ElementTree as ET

BENCHMARK='Ramp Metering/run_fewIncidents_ALINEA_benchmark'


def _number(edge_id): return int(re.match(r'S(\d+)',edge_id).group(1))


def extract_layout(repository='data/raw/i210',first=25,last=67,dt_s=10,vmax_kmh=105):
    import sumolib
    base=Path(repository)/BENCHMARK
    root=next(base.rglob('I210.sumocfg.xml')).parent
    net=sumolib.net.readNet(str(root/'I210.net(zipper).xml'))
    detectors={e.get('id'):(e.get('lane') or e.get('lanes')) for e in ET.parse(root/'I210.detector.xml').getroot()}
    area=json.loads((base/'onrampLaneAreaConfig.json').read_text())
    meters=json.loads((base/'meterConfig.json').read_text())
    onramps=json.loads((base/'onrampConfig.json').read_text())
    metered={}
    for name,cfg in area.items():
        edges={lane.rsplit('_',1)[0] for d in cfg['rampLaneAreaDetectors'] for lane in detectors[d].split()}
        metered[_number(min(edges))]=name
    min_length=ceil(vmax_kmh*dt_s/3.6)
    mainline=sorted((e for e in net.getEdges() if re.fullmatch(r'S\d+_M',e.getID()) and first<=_number(e.getID())<=last),key=lambda e:_number(e.getID()))
    def inflow_of(edge):
        k=_number(edge.getID()); ins=[i.getID() for i in edge.getIncoming()]
        if k in metered: return {'kind':'metered','name':metered[k],'meter':meters[metered[k]]['meterID'],
                                 'edges':[i for i in ins if '_O' in i],'storage':onramps[metered[k]]['maxVehNum'],
                                 'lanes':onramps[metered[k]]['numRampLane'],'meter_config':meters[metered[k]]}
        if any(i.endswith('_H') for i in ins): return {'kind':'hov_merge','edges':[i for i in ins if i.endswith('_H')]}
        if any('_O' in i for i in ins): return {'kind':'unmetered','edges':[i for i in ins if '_O' in i]}
        return None
    info=[(e.getID(),e.getLength(),e.getLaneNumber(),inflow_of(e),[o.getID() for o in e.getOutgoing() if not o.getID().endswith('_M')]) for e in mainline]
    # Partition by dynamic programming: every cell >= min_length with at most one inflow;
    # among valid partitions keep the one with the most cells (finest resolution).
    m=len(info); best=[None]*(m+1); best[0]=(0,None)
    for end in range(1,m+1):
        for start in range(end-1,-1,-1):
            if best[start] is None: continue
            length=sum(x[1] for x in info[start:end]); inflows=sum(x[3] is not None for x in info[start:end])
            if inflows>1: break
            if length>=min_length and (best[end] is None or best[start][0]+1>best[end][0]): best[end]=(best[start][0]+1,start)
    if best[m] is None: raise ValueError(f'no valid partition with cells >= {min_length} m')
    bounds=[]; end=m
    while end: start=best[end][1]; bounds.append((start,end)); end=start
    cells=[]
    for start,end in reversed(bounds):
        part=info[start:end]
        cells.append({'edges':[x[0] for x in part],'length':sum(x[1] for x in part),'lane_m':sum(x[1]*x[2] for x in part),
                      'min_lanes':min(x[2] for x in part),'inflow':next((x[3] for x in part if x[3]),None),
                      'exits':[o for x in part for o in x[4]]})
    for c in cells: c['lanes']=round(c['lane_m']/c['length'])
    return {'first_edge':mainline[0].getID(),'last_edge':mainline[-1].getID(),'dt_s':dt_s,'vmax_kmh':vmax_kmh,
            'min_cell_m':min_length,'cells':cells}


def meter_plan(green_s,red_s,period_s):
    """Phase sequence of one update period, as built by the upstream RampMeter: whole
    cycles of green then red, the remainder filled with red. True marks green seconds."""
    cycle=green_s+red_s; plan=[]
    for _ in range(period_s//cycle): plan+= [True]*green_s+[False]*red_s
    return plan+[False]*(period_s%cycle)


def meter_commands(meter,period_s=30):
    """Realizable commands N (green phases per period) and the red phase issuing each."""
    g=meter['greenPhaseLen']; commands={}
    for red in range(meter['minRedPhaseLen'],meter['maxRedPhaseLen']+1):
        commands.setdefault(period_s//(g+red),period_s//(period_s//(g+red))-g)
    return {n:red for n,red in sorted(commands.items()) if n>0}


def green_starts(plan,dt_s):
    """Green phases starting within each model step of the period."""
    starts=[k for k,x in enumerate(plan) if x and (k==0 or not plan[k-1])]
    return tuple(sum(dt_s*j<=k<dt_s*(j+1) for k in starts) for j in range(len(plan)//dt_s))


def meter_release_table(meter,lanes,dt_s=10,period_s=30):
    """Release capacity per model step for each command 0..max N (row 0 is unused)."""
    if period_s%dt_s: raise ValueError('the update period must be a multiple of the model step')
    commands=meter_commands(meter,period_s); per=meter['numVehPerGreenPerLane']*lanes
    rows=[(0,)*(period_s//dt_s)]*(max(commands)+1)
    for n,red in commands.items():
        rows[n]=tuple(per*x for x in green_starts(meter_plan(meter['greenPhaseLen'],red,period_s),dt_s))
    return min(commands),max(commands),tuple(rows)


def corridor_from_layout(layout,capacity_vphpl=1580,jam_vpkmpl=90,wave_kmh=20,exit_permille=None,calibration=None,
                         period_s=30):
    """Integer Corridor for the layout; exit shares default to zero until estimated.

    calibration: path to results/pems-i210e-calibration.json; per-cell capacity and
    jam density replace the scalar defaults and the corridor median wave speed is used.
    period_s: meter update period. Metered ramps take the device commands of
    meter_release_table and the model must hold them over period_s//dt steps.
    """
    per_cell=None
    if calibration:
        report=json.loads(Path(calibration).read_text()); per_cell=report['cells']
        wave_kmh=round(report['corridor_median']['wave_speed_kmh'])
    from trace_csp.domain import Corridor
    dt=layout['dt_s']; cells=layout['cells']
    ramps=[(j,c['inflow']) for j,c in enumerate(cells) if c['inflow']]
    big=10**6
    ramp_max=[]; ramp_min=[]; storage=[]; tables=[]
    for _,inflow in ramps:
        if inflow['kind']=='metered':
            lo,hi,table=meter_release_table(inflow['meter_config'],inflow['lanes'],dt,period_s)
            ramp_min.append(lo); ramp_max.append(hi); tables.append(table); storage.append(inflow['storage'])
        else:
            cap=max(1,round(capacity_vphpl*2*dt/3600)); ramp_max.append(cap); ramp_min.append(cap); tables.append(()); storage.append(big)
    cap=[p['capacity_vphpl'] for p in per_cell] if per_cell else [capacity_vphpl]*len(cells)
    jam=[p['jam_density_vpkmpl'] for p in per_cell] if per_cell else [jam_vpkmpl]*len(cells)
    return Corridor(tuple(round(c['length']) for c in cells),tuple(c['lanes'] for c in cells),
                    tuple(max(1,round(k*c['lanes']*c['length']/1000)) for k,c in zip(jam,cells)),
                    tuple(max(1,round(q*c['min_lanes']*dt/3600)) for q,c in zip(cap,cells)),
                    tuple(j for j,_ in ramps),tuple(ramp_max),tuple(storage),wave_speed_kmh=wave_kmh,
                    exit_permille=tuple(exit_permille) if exit_permille else (),ramp_min=tuple(ramp_min),release_table=tuple(tables))


def demand_scenarios(boundary,ramps,horizon,steps_per_period,cells):
    """Scenarios of the shield: weights 1:2:1 on 0.85, 1 and 1.15 times the demand counted
    over the last update period, spread evenly over its model steps and held over the horizon."""
    from trace_csp.domain import Scenario
    return tuple(Scenario(w,tuple([round(f*boundary/steps_per_period)]*horizon),
                          tuple([tuple(round(f*x/steps_per_period) for x in ramps)]*horizon),
                          tuple([(1000,)*cells]*horizon)) for w,f in ((1,.85),(2,1.),(1,1.15)))
