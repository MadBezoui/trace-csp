"""Train matched architecture/seed probes on one immutable labelled dataset."""
import argparse
import gc
import json
from pathlib import Path
from time import perf_counter
from trace_csp.pipeline import train_probe


def main():
    p=argparse.ArgumentParser();p.add_argument('--input',required=True);p.add_argument('--output-dir',default='results')
    p.add_argument('--tag',required=True);p.add_argument('--architectures',nargs='+',choices=('mlp','bipartite','hgt'),default=['mlp','bipartite','hgt'])
    p.add_argument('--seeds',type=int,default=5);p.add_argument('--epochs',type=int,default=2)
    a=p.parse_args()
    if a.seeds<1 or a.epochs<1: p.error('seeds and epochs must be positive')
    output=Path(a.output_dir);output.mkdir(parents=True,exist_ok=True)
    for architecture in a.architectures:
        for seed in range(a.seeds):
            checkpoint=output/f'{architecture}-{a.tag}-seed{seed}.pt'
            report=output/f'train-{architecture}-{a.tag}-seed{seed}.json'
            if checkpoint.exists() and report.exists():
                print(json.dumps({'architecture':architecture,'seed':seed,'status':'existing'}),flush=True)
                continue
            start=perf_counter()
            result=train_probe(a.input,checkpoint,architecture,a.epochs,seed)
            result.update({'dataset':a.input,'epochs':a.epochs,'training_wall_s':perf_counter()-start})
            report.write_text(json.dumps(result,indent=2))
            print(json.dumps({'architecture':architecture,'seed':seed,'status':'trained',
                              'training_wall_s':result['training_wall_s']}),flush=True)
            del result;gc.collect()

if __name__=='__main__':main()
