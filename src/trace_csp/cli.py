"""Command-line experiment entry points; all artifacts are local."""
import argparse
import json
from trace_csp.data import generate_instance,write_instances,read_instances
from trace_csp.pipeline import solve_offline,train_probe,compare_controllers,evaluate_probe,evaluate_frequency_baseline


def main(argv=None):
    p=argparse.ArgumentParser(prog='trace-csp'); sub=p.add_subparsers(dest='command',required=True)
    gen=sub.add_parser('generate'); gen.add_argument('--count',type=int,default=100); gen.add_argument('--start',type=int,default=0)
    gen.add_argument('--horizon',type=int,default=2); gen.add_argument('--output',required=True)
    off=sub.add_parser('solve-offline'); off.add_argument('--input',required=True); off.add_argument('--output',required=True)
    off.add_argument('--limit',type=int); off.add_argument('--seconds',type=float)
    tr=sub.add_parser('train'); tr.add_argument('--input',required=True); tr.add_argument('--checkpoint',required=True)
    tr.add_argument('--architecture',choices=('mlp','bipartite','hgt'),default='hgt'); tr.add_argument('--epochs',type=int,default=10); tr.add_argument('--seed',type=int,default=0)
    ev=sub.add_parser('evaluate-model'); ev.add_argument('--input',required=True); ev.add_argument('--checkpoint',required=True)
    ev.add_argument('--split',required=True); ev.add_argument('--output',required=True)
    freq=sub.add_parser('evaluate-frequency'); freq.add_argument('--train',required=True); freq.add_argument('--test',required=True)
    freq.add_argument('--split',required=True); freq.add_argument('--output',required=True)
    exp=sub.add_parser('experiment'); exp.add_argument('--input',required=True); exp.add_argument('--output',required=True)
    exp.add_argument('--checkpoint'); exp.add_argument('--limit',type=int); exp.add_argument('--split')
    a=p.parse_args(argv)
    if a.command=='generate':
        if a.count<1: p.error('count must be positive')
        write_instances(a.output,(generate_instance(k,horizon=a.horizon) for k in range(a.start,a.start+a.count)))
        summary={'generated':a.count,'output':a.output}
    elif a.command=='solve-offline': summary=solve_offline(a.input,a.output,a.limit,a.seconds)
    elif a.command=='train': summary=train_probe(a.input,a.checkpoint,a.architecture,a.epochs,a.seed)
    elif a.command=='evaluate-model':
        summary=evaluate_probe(a.input,a.checkpoint,a.split)
        with open(a.output,'w') as f: json.dump(summary,f,indent=2)
        summary={k:summary[k] for k in ('architecture','seed','split','instances','metrics')}
    elif a.command=='evaluate-frequency':
        summary=evaluate_frequency_baseline(a.train,a.test,a.split)
        with open(a.output,'w') as f: json.dump(summary,f,indent=2)
        summary={k:summary[k] for k in ('architecture','split','instances','metrics')}
    else:
        rows=list(read_instances(a.input)); rows=[x for x in rows if x.split==a.split] if a.split else rows; rows=rows[:a.limit] if a.limit else rows
        names=('cp_sat','warm_cp_sat','learned_cp_sat') if a.checkpoint else ('cp_sat','warm_cp_sat')
        summary=compare_controllers(rows,names,a.checkpoint)
        with open(a.output,'w') as f: json.dump(summary,f,indent=2)
        summary={k:summary[k] for k in ('scope','seeds','controllers')}
    print(json.dumps(summary,indent=2))

if __name__=='__main__': main()
