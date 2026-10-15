"""Map I-210 E PeMS stations to the SUMO corridor, fit cell parameters, validate on held-out dates.

Calibration dates: Jan + Apr 2026 weekdays. Validation dates: May + Aug 2026 weekdays.
Only rows with >= 75% observed samples are used.
"""
import json
import re
import numpy as np
import pandas as pd
from trace_csp.data_sources.fd import fit_triangular
from trace_csp.sumo_i210 import extract_layout

META='data/raw/pems/d07_text_meta_2023_12_22.txt'
NET='data/raw/i210/Ramp Metering/run_fewIncidents_ALINEA_benchmark/SUMO (few incidents)/SUMO (few incidents)/I210.net(zipper).xml'


def triangular(k,p):
    q=np.minimum(p['free_speed_kmh']*k,p['capacity_vphpl'])
    if p.get('wave_speed_kmh') and p.get('jam_density_vpkmpl'): q=np.minimum(q,p['wave_speed_kmh']*np.maximum(0,p['jam_density_vpkmpl']-k))
    return q


def main():
    import sumolib
    net=sumolib.net.readNet(NET)
    mainline=sorted([e for e in net.getEdges() if re.fullmatch(r'S\d+_M',e.getID())],key=lambda e:int(e.getID()[1:-2]))
    start={}; acc=0.
    for e in mainline: start[e.getID()]=acc; acc+=e.getLength()
    meta=pd.read_csv(META,sep='\t'); meta=meta[(meta.Fwy==210)&(meta.Dir=='E')]
    mapped=pd.read_csv('data/interim/pems/i210e_station_edges.csv')
    ml=mapped[(mapped.type=='ML')&mapped.corridor_m.notna()]
    slope,intercept=np.polyfit(ml.abs_pm,ml.corridor_m,1)
    meta=meta.assign(corridor_m=slope*meta.Abs_PM+intercept)
    meta=meta[(meta.corridor_m>=0)&(meta.corridor_m<=acc)]
    def edge_at(x):
        for e in reversed(mainline):
            if start[e.getID()]<=x: return e.getID()
    meta['edge']=meta.corridor_m.map(edge_at)
    layout=extract_layout(); cell_of={e:j for j,c in enumerate(layout['cells']) for e in c['edges']}
    meta['cell']=meta.edge.map(cell_of)
    meta.to_csv('data/interim/pems/i210e_station_positions.csv',index=False)

    fits=pd.read_csv('results/pems-fd-i210-calibration.csv')
    fits=fits.merge(meta[['ID','corridor_m','cell','edge']],left_on='station',right_on='ID',how='inner')
    corridor_fits=fits[fits.corridor_m.notna()]
    overall={k:float(corridor_fits[k].median()) for k in ('free_speed_kmh','capacity_vphpl','wave_speed_kmh','jam_density_vpkmpl','effective_length_m')}
    cells=[]
    for j in range(len(layout['cells'])):
        f=corridor_fits[corridor_fits.cell==j]
        cells.append({'cell':j,'stations':f.station.tolist(),**{k:(float(f[k].median()) if f[k].notna().any() else overall[k]) for k in overall}})

    df=pd.read_parquet('data/interim/pems/i210_5min.parquet',columns=['station','type','dir','date','pct_obs','flow','occ','speed'])
    df=df[(df.type=='ML')&(df.dir=='E')&(df.pct_obs>=75)&df.station.isin(corridor_fits.station)]
    d=pd.to_datetime(df.date); df=df[(d.dt.dayofweek<5)&d.dt.month.isin([5,8])]
    lanes=dict(zip(meta.ID,meta.Lanes))
    # The fit normalized flow by the lanes reporting flow on 2026-04-14; validation uses the
    # metadata lane count. Stop if the two disagree for a validated station.
    fitted_lanes=dict(zip(corridor_fits.station,corridor_fits.lanes))
    disagree={int(st):(int(fitted_lanes[st]),int(lanes[st])) for st in df.station.unique() if fitted_lanes[st]!=lanes[st]}
    if disagree: raise SystemExit(f'lane counts differ between fit and metadata (fit, metadata): {disagree}')
    placeholder={'free_speed_kmh':100.,'capacity_vphpl':1980.,'wave_speed_kmh':16.,'jam_density_vpkmpl':150.}
    errors={'calibrated':[],'placeholder':[],'corridor_median':[]}; per_station=[]
    for st,g in df.groupby('station'):
        p=corridor_fits[corridor_fits.station==st].iloc[0].to_dict()
        q=g.flow.to_numpy()*12/lanes[st]; k=g.occ.to_numpy()/(p['effective_length_m']/1000)
        row={'station':int(st),'n':len(g),'lanes':int(lanes[st])}
        for name,params in (('calibrated',p),('corridor_median',overall),('placeholder',placeholder)):
            err=triangular(k,params)-q; errors[name].append(err); row[name+'_rmse_vphpl']=float(np.sqrt(np.mean(err**2)))
        per_station.append(row)
    summary={name:{'rmse_vphpl':float(np.sqrt(np.mean(np.concatenate(e)**2))),'bias_vphpl':float(np.mean(np.concatenate(e)))} for name,e in errors.items()}
    report={'calibration_dates':'2026 Jan+Apr weekdays','validation_dates':'2026 May+Aug weekdays','min_pct_observed':75,
            'postmile_to_sumo':{'slope_m_per_mile':float(slope),'intercept_m':float(intercept)},
            'corridor_median':overall,'cells':cells,'validation_flow_error':summary,'validation_by_station':per_station,
            'validation_samples':int(len(df))}
    with open('results/pems-i210e-calibration.json','w') as f: json.dump(report,f,indent=1)
    print(json.dumps({k:report[k] for k in ('corridor_median','validation_flow_error','validation_samples')},indent=1))
    print('cells with local stations:',sum(bool(c['stations']) for c in cells),'/',len(cells))


if __name__=='__main__': main()
