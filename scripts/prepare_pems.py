"""Prepare a registered PeMS CSV without downloading or guessing columns."""
import argparse
import json
from pathlib import Path
from trace_csp.data_sources.pems import summarize_pems


def main():
    p=argparse.ArgumentParser(); p.add_argument('--input',required=True); p.add_argument('--output',required=True)
    for column in ('timestamp','station','flow','speed','occupancy'): p.add_argument('--'+column+'-column',required=True)
    p.add_argument('--flow-unit',choices=('veh_per_hour','vehicles_per_interval'),required=True)
    p.add_argument('--speed-unit',choices=('mph','kmh'),required=True)
    p.add_argument('--occupancy-unit',choices=('fraction','percent'),required=True)
    a=p.parse_args()
    mapping={key:getattr(a,key+'_column') for key in ('timestamp','station','flow','speed','occupancy')}
    result=summarize_pems(a.input,mapping,a.flow_unit,a.speed_unit,a.occupancy_unit)
    output=Path(a.output); output.parent.mkdir(parents=True,exist_ok=True); output.write_text(json.dumps(result,indent=2))
    print(json.dumps({'status':result['status'],'source_rows':result['source_rows'],'stations':len(result['stations'])}))

if __name__=='__main__': main()
