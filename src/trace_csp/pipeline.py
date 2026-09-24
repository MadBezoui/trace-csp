"""Offline labels, training and matched synthetic solver comparisons."""
from dataclasses import asdict, replace
from pathlib import Path
import json
import random
from time import perf_counter
import numpy as np

from trace_csp.data import read_instances,open_jsonl
from trace_csp.csp.builder import build
from trace_csp.csp.solver import solve,SolveResult
from trace_csp.graph.factor_graph import factor_graph
from trace_csp.graph.labels import labels_for
from trace_csp.evaluation.metrics import binary_metrics
from trace_csp.evaluation.statistics import paired_summary


def solve_offline(input_path,output_path,limit=None,seconds=None):
    summary={'attempted':0,'feasible':0,'optimal':0,'infeasible':0,'unknown':0}
    with open_jsonl(output_path,'w') as out:
        for i in read_instances(input_path):
            if limit is not None and summary['attempted']>=limit: break
            if seconds is not None: i=replace(i,config=replace(i.config,time_limit_s=seconds))
            r=solve(i); b=build(i); graph=factor_graph(b); labels=labels_for(i,b,graph,r)
            record={'id':i.instance_id,'group':i.group,'split':i.split,'family':i.family,
                    'status':r.status,'objectives':r.objectives,'solver_wall_s':r.wall_time,'first_feasible_s':r.first_feasible_s,
                    'gap':r.gap,'branches':r.branches,'conflicts':r.conflicts,'trace':r.trace,
                    'unsat_core':r.unsat_core,'graph':graph,'labels':labels}
            out.write(json.dumps(record,separators=(',',':'))+'\n')
            summary['attempted']+=1
            if r.actions: summary['feasible']+=1
            if r.status=='OPTIMAL': summary['optimal']+=1
            if r.status=='INFEASIBLE': summary['infeasible']+=1
            if r.status=='UNKNOWN': summary['unknown']+=1
    return summary


def read_labelled(path):
    with open_jsonl(path,'r') as f:
        for line in f:
            if line.strip(): yield json.loads(line)


def model_class(name):
    from trace_csp.learning.constraint_transformer import GlobalMLP,BipartiteGNN,ConstraintAttentionNet
    return {'mlp':GlobalMLP,'bipartite':BipartiteGNN,'hgt':ConstraintAttentionNet}[name]


def train_probe(input_path,checkpoint,architecture='hgt',epochs=10,seed=0,lr=.001):
    import torch
    from trace_csp.learning.constraint_transformer import graph_to_tensors
    from trace_csp.learning.losses import multitask_loss
    from trace_csp.learning.calibrate import fit_temperature
    if epochs<1: raise ValueError('epochs must be positive')
    rows=[r for r in read_labelled(input_path) if r['status'] in ('OPTIMAL','FEASIBLE','INFEASIBLE')]
    train=[r for r in rows if r['split']=='train']; valid=[r for r in rows if r['split']=='validation']
    if not train or not valid: raise ValueError('both train and validation groups with feasible labels required')
    tg={r['group'] for r in train}; vg={r['group'] for r in valid}
    if tg & vg: raise ValueError('date group leakage')
    torch.manual_seed(seed); random.seed(seed); np.random.seed(seed); torch.set_num_threads(1)
    model=model_class(architecture)(); opt=torch.optim.AdamW(model.parameters(),lr=lr)
    def tensors(row): return graph_to_tensors(row['graph'],row['labels'])
    for epoch in range(epochs):
        model.train(); rng=random.Random(seed+epoch); order=list(train); rng.shuffle(order)
        for row in order:
            t=tensors(row); opt.zero_grad(); loss=multitask_loss(model(t),t); loss.backward(); opt.step()
    model.eval(); scores={k:[] for k in ('active','violation','core')}; truth={k:[] for k in scores}; logits={k:[] for k in scores}
    with torch.no_grad():
        for row in valid:
            t=tensors(row); output=model(t)
            for key in scores:
                mask=t[key+'_mask']; z=output[key][mask].cpu().numpy().tolist()
                logits[key]+=z; truth[key]+=t[key][mask].cpu().numpy().tolist()
    from scipy.special import expit
    temperature={}; report={}
    for key in scores:
        if logits[key]:
            temperature[key]=fit_temperature(logits[key],truth[key])
            report[key]=binary_metrics(expit(np.asarray(logits[key])/temperature[key]),truth[key])
        else: temperature[key]=1.; report[key]=None
    checkpoint=Path(checkpoint); checkpoint.parent.mkdir(parents=True,exist_ok=True)
    torch.save({'architecture':architecture,'state_dict':model.state_dict(),'temperature':temperature,'seed':seed,
                'train_groups':sorted(tg),'validation_groups':sorted(vg)},checkpoint)
    return {'architecture':architecture,'n_train':len(train),'n_validation':len(valid),
            'train_groups':sorted(tg),'validation_groups':sorted(vg),'metrics':report,'checkpoint':str(checkpoint)}


def load_predictor(checkpoint):
    import torch
    from trace_csp.learning.constraint_transformer import graph_to_tensors
    from trace_csp.guidance.priorities import scores_to_priority,calibrated_constraint_scores
    bundle=torch.load(checkpoint,map_location='cpu',weights_only=True)
    model=model_class(bundle['architecture'])(); model.load_state_dict(bundle['state_dict']); model.eval()
    def predict(instance):
        built=build(instance); graph=factor_graph(built); data=graph_to_tensors(graph)
        with torch.no_grad(): out=model(data)
        scores=calibrated_constraint_scores(out,bundle['temperature']).cpu().numpy()
        return scores_to_priority(graph,scores,tuple(built.controls))
    return predict


def evaluate_probe(input_path,checkpoint,split):
    """Evaluate a frozen checkpoint on untouched held-out groups."""
    import torch
    from scipy.special import expit
    from trace_csp.learning.constraint_transformer import graph_to_tensors
    if split!='test_id' and not split.startswith('test_ood_'):
        raise ValueError('evaluation requires a held-out test split')
    bundle=torch.load(checkpoint,map_location='cpu',weights_only=True)
    model=model_class(bundle['architecture'])(); model.load_state_dict(bundle['state_dict']); model.eval()
    torch.set_num_threads(1)
    logits={k:[] for k in ('active','violation','core')}; truth={k:[] for k in logits}
    groups=set(); count=0
    with torch.no_grad():
        for row in read_labelled(input_path):
            if row['split']!=split or row['status'] not in ('OPTIMAL','FEASIBLE','INFEASIBLE'): continue
            groups.add(row['group']); count+=1
            t=graph_to_tensors(row['graph'],row['labels']); output=model(t)
            for key in logits:
                mask=t[key+'_mask']
                logits[key].extend(output[key][mask].cpu().numpy().tolist())
                truth[key].extend(t[key][mask].cpu().numpy().tolist())
    if not count: raise ValueError('no labelled test instances in requested split')
    if groups.intersection(bundle['train_groups']) or groups.intersection(bundle['validation_groups']):
        raise ValueError('held-out group overlap with model development data')
    metrics={key:binary_metrics(expit(np.asarray(z)/bundle['temperature'][key]),truth[key]) if z else None
             for key,z in logits.items()}
    return {'architecture':bundle['architecture'],'seed':bundle['seed'],'split':split,
            'instances':count,'groups':sorted(groups),'metrics':metrics}


def evaluate_frequency_baseline(train_path,test_path,split):
    """Laplace-smoothed training frequency for each named constraint."""
    from collections import defaultdict
    if split!='test_id' and not split.startswith('test_ood_'):
        raise ValueError('evaluation requires a held-out test split')
    keys=('active','violation','core')
    counts={key:defaultdict(lambda:[0,0]) for key in keys}
    totals={key:[0,0] for key in keys}
    train_groups=set()
    for row in read_labelled(train_path):
        if row['split']!='train' or row['status'] not in ('OPTIMAL','FEASIBLE','INFEASIBLE'): continue
        train_groups.add(row['group'])
        for key in keys:
            for name,y,valid in zip(row['graph']['constraint_names'],row['labels'][key],row['labels'][key+'_mask']):
                if valid:
                    counts[key][name][0]+=y; counts[key][name][1]+=1
                    totals[key][0]+=y; totals[key][1]+=1
    if not train_groups: raise ValueError('no labelled training data')
    scores={key:[] for key in keys}; truth={key:[] for key in keys}
    groups=set(); instances=0
    for row in read_labelled(test_path):
        if row['split']!=split or row['status'] not in ('OPTIMAL','FEASIBLE','INFEASIBLE'): continue
        groups.add(row['group']); instances+=1
        for key in keys:
            for name,y,valid in zip(row['graph']['constraint_names'],row['labels'][key],row['labels'][key+'_mask']):
                if valid:
                    positive,labelled=counts[key].get(name,totals[key])
                    scores[key].append((positive+1)/(labelled+2)); truth[key].append(y)
    if not instances: raise ValueError('no labelled test instances in requested split')
    if groups & train_groups: raise ValueError('held-out group overlap with training data')
    return {'architecture':'constraint_frequency','split':split,'instances':instances,'groups':sorted(groups),
            'metrics':{key:binary_metrics(scores[key],truth[key]) if scores[key] else None for key in keys}}


def compare_controllers(instances,controllers=('cp_sat','warm_cp_sat','learned_cp_sat'),checkpoint=None):
    from trace_csp.csp.verifier import verify_plan
    instances=list(instances)
    if 'learned_cp_sat' in controllers and checkpoint is None: raise ValueError('learned controller requires checkpoint')
    predictor=load_predictor(checkpoint) if 'learned_cp_sat' in controllers else None
    runs=[]; previous={name:None for name in controllers}; signatures={name:None for name in controllers}; previous_group={name:None for name in controllers}
    for i in instances:
        signature=(i.corridor,i.config.horizon,i.config.vsl_values)
        for name in controllers:
            if name not in ('cp_sat','warm_cp_sat','learned_cp_sat'): raise ValueError('unknown controller')
            hints=previous[name] if name!='cp_sat' and signatures[name]==signature and previous_group[name]==i.group else None
            started=perf_counter()
            priority=predictor(i) if name=='learned_cp_sat' else None
            preprocessing_s=perf_counter()-started
            remaining=max(0.,i.config.time_limit_s-(perf_counter()-started))
            bounded=replace(i,config=replace(i.config,time_limit_s=remaining))
            r=solve(bounded,hints=hints,priorities=priority) if remaining>0 else SolveResult('UNKNOWN')
            cert=verify_plan(i,r.actions) if r.actions else None
            elapsed=perf_counter()-started
            verified=bool(cert and cert.safe)
            deadline_met=elapsed<=i.config.time_limit_s
            feasible=verified and deadline_met
            previous[name]=r.actions if feasible else None; signatures[name]=signature; previous_group[name]=i.group
            runs.append({'id':i.instance_id,'split':i.split,'family':i.family,'controller':name,'status':r.status,
                         'feasible':feasible,'verified_feasible':verified,'deadline_met':deadline_met,
                         'wall_s':elapsed,'end_to_end_wall_s':elapsed,'solver_wall_s':r.wall_time,'solver_budget_s':remaining,
                         'preprocessing_s':preprocessing_s,'first_feasible_s':(preprocessing_s+r.first_feasible_s) if r.first_feasible_s is not None else None,
                         'tts':r.objectives[int(i.config.congestion_penalty)] if r.objectives else None,'gap':r.gap,'branches':r.branches,'conflicts':r.conflicts})
    paired={}
    if 'cp_sat' in controllers:
        baseline={row['id']:row for row in runs if row['controller']=='cp_sat'}
        for name in controllers:
            if name=='cp_sat': continue
            rows=[(baseline[row['id']],row) for row in runs if row['controller']==name]
            pairs=[(a['wall_s'],b['wall_s']) for a,b in rows if a['feasible'] and b['feasible']]
            paired[name]=paired_summary(*map(list,zip(*pairs))) if len(pairs)>=2 else None
    return {'scope':'synthetic open-loop matched solver instances','seeds':len(instances),'controllers':list(controllers),'runs':runs,'paired_runtime':paired}
