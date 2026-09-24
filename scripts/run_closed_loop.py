"""Stream paired synthetic CTM episodes; --full uses the preregistered matrix."""
import argparse
import json
from pathlib import Path
from itertools import product
import yaml
from trace_csp.data import open_jsonl
from trace_csp.data import FAMILIES
from trace_csp.pipeline import load_predictor
from trace_csp.simulation.synthetic_env import run_episode


def main():
    p=argparse.ArgumentParser(); p.add_argument('--output',required=True)
    p.add_argument('--seeds',type=int,default=3); p.add_argument('--periods',type=int,default=4)
    p.add_argument('--controllers',nargs='+',default=['fixed','alinea','cp_sat','warm_cp_sat'])
    p.add_argument('--families',nargs='+',default=['normal'])
    p.add_argument('--demand-levels',nargs='+',type=float,default=[1.])
    p.add_argument('--incident-factors',nargs='+',type=int,default=[1000])
    p.add_argument('--checkpoint'); p.add_argument('--full',action='store_true')
    p.add_argument('--corridor-cells',type=int); p.add_argument('--corridor-horizon',type=int,default=4); p.add_argument('--time-limit',type=float); p.add_argument('--ungated-baselines',action='store_true'); p.add_argument('--soft-weight',type=int,default=100)
    a=p.parse_args()
    if a.seeds<1 or a.periods<1: p.error('positive seeds and periods required')
    if a.full:
        cfg=yaml.safe_load((Path(__file__).resolve().parents[1]/'configs/experiments/full.yaml').read_text())
        families=cfg['families']; demands=cfg['demand_levels']; incidents=cfg['incident_severity_multipliers_per_mille']
        controllers=cfg['controllers']; seeds=range(cfg['seeds']); periods=cfg['periods']
    else:
        families=a.families; demands=a.demand_levels; incidents=a.incident_factors
        controllers=a.controllers; seeds=range(a.seeds); periods=a.periods
    if 'learned_cp_sat' in controllers and not a.checkpoint: p.error('learned_cp_sat requires --checkpoint')
    predictor=load_predictor(a.checkpoint) if a.checkpoint else None
    count=0; halted=0
    with open_jsonl(a.output,'w') as out:
        for family,demand,incident,seed,controller in product(families,demands,incidents,seeds,controllers):
            row=run_episode(seed,controller,periods,family,demand,incident,predictor,a.corridor_cells,a.corridor_horizon,a.time_limit,a.ungated_baselines,a.soft_weight)
            out.write(json.dumps(row,separators=(',',':'))+'\n'); count+=1; halted+=row['halt_reason'] is not None
    print(json.dumps({'scope':'synthetic CTM closed loop','episodes':count,'halted':halted,'output':a.output}))

if __name__=='__main__': main()
