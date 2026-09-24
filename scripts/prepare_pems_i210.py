"""Prepare Caltrans PeMS District 7 data for the I-210 calibration.

Steps (each writes one file):
1. extract freeway-210 rows from the raw daily 5-minute files
   -> data/interim/pems/i210_5min.parquet
2. fit a triangular fundamental diagram per mainline station and lane on observed
   (>= 75%) weekday records of January and April 2026
   -> results/pems-fd-i210-calibration.csv
3. project I-210 eastbound stations onto the SUMO network
   -> data/interim/pems/i210e_station_edges.csv

Raw inputs (not redistributed): data/raw/pems/d07_text_station_5min_*.txt and
d07_text_meta_2023_12_22.txt. The next step is scripts/calibrate_i210.py.
"""
import argparse
import glob
import re
from multiprocessing import Pool
from pathlib import Path
import pandas as pd
from trace_csp.data_sources.fd import fit_triangular

RAW=Path('data/raw/pems'); INTERIM=Path('data/interim/pems')
COLUMNS=['ts','station','district','fwy','dir','type','length','samples','pct_obs','flow','occ','speed']
NET='data/raw/i210/Ramp Metering/run_fewIncidents_ALINEA_benchmark/SUMO (few incidents)/SUMO (few incidents)/I210.net(zipper).xml'
META=RAW/'d07_text_meta_2023_12_22.txt'
LANE_DAY=RAW/'d07_text_station_5min_2026_04_14.txt'


def extract_file(path):
    rows=[]
    with open(path) as f:
        for line in f:
            parts=line.rstrip('\n').split(',')
            if len(parts)>3 and parts[3]=='210': rows.append(parts[:12])
    df=pd.DataFrame(rows,columns=COLUMNS)
    for c in ('station','district','fwy','samples'): df[c]=pd.to_numeric(df[c])
    for c in ('length','pct_obs','flow','occ','speed'): df[c]=pd.to_numeric(df[c].replace('',None))
    return df


def extract(workers):
    files=sorted(glob.glob(str(RAW/'d07_text_station_5min_*.txt')))
    with Pool(workers) as p: parts=p.map(extract_file,files)
    df=pd.concat(parts,ignore_index=True); df['ts']=pd.to_datetime(df.ts,format='%m/%d/%Y %H:%M:%S')
    df['date']=df.ts.dt.date; df['tod']=df.ts.dt.time
    INTERIM.mkdir(parents=True,exist_ok=True); df.to_parquet(INTERIM/'i210_5min.parquet')
    print(f'{len(df):,} I-210 rows from {len(files)} days')


def station_lanes():
    """Number of lanes with any reported flow, per station, on one reference day."""
    raw=pd.read_csv(LANE_DAY,header=None,low_memory=False)
    raw=raw[raw[3]==210]
    lane_flow=[12+5*k+1 for k in range(8)]
    return raw.groupby(1)[lane_flow].apply(lambda d:int(d.notna().any().sum()))


def fit_diagrams():
    lanes=station_lanes()
    df=pd.read_parquet(INTERIM/'i210_5min.parquet')
    df=df[(df.type=='ML')&(df.pct_obs>=75)]
    d=pd.to_datetime(df.date)
    cal=df[d.dt.month.isin([1,4])&(d.dt.dayofweek<5)]
    rows=[]
    for st,g in cal.groupby('station'):
        L=lanes.get(st,0)
        if L<1: continue
        try: fit=fit_triangular(g.flow*12/L,g.speed*1.609344,g.occ)
        except ValueError: continue
        fit.update(station=int(st),direction=g.dir.iloc[0],lanes=L); rows.append(fit)
    out=pd.DataFrame(rows); Path('results').mkdir(exist_ok=True)
    out.to_csv('results/pems-fd-i210-calibration.csv',index=False)
    print(f'{len(out)} stations fitted, {out.wave_speed_kmh.notna().sum()} with a congested branch')


def project_stations():
    import sumolib
    net=sumolib.net.readNet(NET)
    meta=pd.read_csv(META,sep='\t')
    m=meta[(meta.Fwy==210)&(meta.Dir=='E')&meta.Latitude.notna()]
    mainline=sorted([e for e in net.getEdges() if re.fullmatch(r'S\d+_M',e.getID())],key=lambda e:int(e.getID()[1:-2]))
    start={}; acc=0.
    for e in mainline: start[e.getID()]=acc; acc+=e.getLength()
    suffix={'ML':'_M','HV':'_H','OR':'_O','FR':'_F','FF':'_F'}
    rows=[]
    for _,s in m.iterrows():
        x,y=net.convertLonLat2XY(s.Longitude,s.Latitude)
        near=net.getNeighboringEdges(x,y,300)
        cands=[(d,e) for e,d in near if suffix.get(s.Type,'_M') in e.getID()]
        if not cands: continue
        d,e=min(cands,key=lambda c:c[0])
        ml=[(dd,ee) for ee,dd in near if re.fullmatch(r'S\d+_M',ee.getID())]
        pos=None
        if ml:
            dd,ee=min(ml,key=lambda c:c[0])
            pos=start[ee.getID()]+sumolib.geomhelper.polygonOffsetWithMinimumDistanceToPoint((x,y),ee.getShape())
        rows.append({'station':s.ID,'type':s.Type,'abs_pm':s.Abs_PM,'name':s.Name,'lanes':s.Lanes,'edge':e.getID(),'dist_m':round(d,1),'corridor_m':pos})
    out=pd.DataFrame(rows).sort_values('abs_pm'); out.to_csv(INTERIM/'i210e_station_edges.csv',index=False)
    print(f'{len(out)} I-210 E stations projected onto the SUMO network')


def main():
    p=argparse.ArgumentParser(); p.add_argument('--steps',nargs='+',choices=('extract','fit','project'),default=['extract','fit','project'])
    p.add_argument('--workers',type=int,default=8)
    a=p.parse_args()
    if 'extract' in a.steps: extract(a.workers)
    if 'fit' in a.steps: fit_diagrams()
    if 'project' in a.steps: project_stations()


if __name__=='__main__': main()
