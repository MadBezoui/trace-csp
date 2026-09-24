"""Learning-free physics baseline, non-trivial label subsets and solver-checked cores.

Core labels are non-unique, so AUPRC against one extracted core undercounts valid
alternatives. Here every method's top-ranked queue guard is re-checked by CP-SAT.
"""
import argparse
import json
from pathlib import Path
from time import perf_counter
import numpy as np
from scipy.special import expit
from trace_csp.data import read_instances
from trace_csp.pipeline import read_labelled,model_class
from trace_csp.evaluation.metrics import binary_metrics
from trace_csp.graph.physics import physics_scores,certified_core,core_is_sufficient

SPLITS=('test_id','test_ood_demand','test_ood_structure','test_ood_incident')


def pairs(instances_path,labelled_path,split):
    for i,row in zip(read_instances(instances_path),read_labelled(labelled_path)):
        if i.instance_id!=row['id']: raise ValueError('instance/label order mismatch')
        if row['split']==split and row['status'] in ('OPTIMAL','FEASIBLE','INFEASIBLE'): yield i,row


def load_models(output,tag,architectures,seeds):
    import torch
    models={}
    for a in architectures:
        for s in range(seeds):
            b=torch.load(output/f'{a}-{tag}-seed{s}.pt',map_location='cpu',weights_only=True)
            m=model_class(b['architecture'])(); m.load_state_dict(b['state_dict']); m.eval(); models[(a,s)]=(m,b['temperature'])
    return models


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--instances',required=True); p.add_argument('--labels',required=True)
    p.add_argument('--challenge-instances',required=True); p.add_argument('--challenge-labels',required=True)
    p.add_argument('--output-dir',default='results/v3'); p.add_argument('--tag',default='v3')
    p.add_argument('--architectures',nargs='+',default=['mlp','bipartite','hgt']); p.add_argument('--seeds',type=int,default=5)
    p.add_argument('--core-check-limit',type=int,default=150)
    a=p.parse_args()
    import torch
    from trace_csp.learning.constraint_transformer import graph_to_tensors
    torch.set_num_threads(1)
    output=Path(a.output_dir); models=load_models(output,a.tag,a.architectures,a.seeds)
    methods=['physics']+[f'{m}-seed{s}' for m,s in models]
    report={'scope':'synthetic v3; physics = CTM replay + monotone queue bound, no training','splits':{}}
    for split in SPLITS:
        src=(a.instances,a.labels) if split=='test_id' else (a.challenge_instances,a.challenge_labels)
        scores={m:{k:[] for k in ('active_ineq','violation','core')} for m in methods}
        truth={k:[] for k in ('active_ineq','violation','core')}
        topcore={m:{'valid':0,'checked':0,'matches_label':0} for m in methods+['solver_label']}
        timing={'physics_certificate_s':[],'cpsat_infeasible_s':[]}
        certified=infeasible=0
        for i,row in pairs(*src,split):
            g=row['graph']; L=row['labels']
            t0=perf_counter(); phys=physics_scores(i,g); core=certified_core(i); t1=perf_counter()
            out={'physics':phys}
            data=graph_to_tensors(g)
            with torch.no_grad():
                for (m,s),(model,temp) in models.items():
                    o=model(data); out[f'{m}-seed{s}']={k:expit(o[k].numpy()/temp[k]).tolist() for k in ('active','violation','core')}
            eq=[f[4]==f[5] for f in g['constraint_features']]
            for c in range(len(g['constraint_names'])):
                if L['active_mask'][c] and not eq[c]:
                    truth['active_ineq'].append(L['active'][c])
                    for m in methods: scores[m]['active_ineq'].append(out[m]['active'][c])
                for key in ('violation','core'):
                    if L[key+'_mask'][c]:
                        truth[key].append(L[key][c])
                        for m in methods: scores[m][key].append(out[m][key][c])
            if row['status']=='INFEASIBLE':
                infeasible+=1; certified+=core is not None
                timing['physics_certificate_s'].append(t1-t0); timing['cpsat_infeasible_s'].append(row['solver_wall_s'])
                if topcore['physics']['checked']<a.core_check_limit:
                    guards=[(c,n) for c,n in enumerate(g['constraint_names']) if n.startswith('queue/')]
                    label=set(row['unsat_core'])
                    for m in methods:
                        best=max(guards,key=lambda x:(out[m]['core'][x[0]],-x[0]))[1]
                        ok=core_is_sufficient(i,[best])
                        topcore[m]['checked']+=1; topcore[m]['valid']+=bool(ok); topcore[m]['matches_label']+=best in label
                    topcore['solver_label']['checked']+=1; topcore['solver_label']['valid']+=bool(core_is_sufficient(i,sorted(label)))
                    topcore['solver_label']['matches_label']+=1
        metrics={m:{k:(binary_metrics(v,truth[k]) if v else None) for k,v in scores[m].items()} for m in methods}
        summary={}
        for key in truth:
            def auprc(m): return metrics[m][key]['auprc'] if metrics[m][key] else None
            summary[key]={'positives':int(sum(truth[key])),'n':len(truth[key]),'physics':auprc('physics')}
            for arch in a.architectures:
                vals=[auprc(f'{arch}-seed{s}') for s in range(a.seeds)]
                summary[key][arch]={'median':float(np.median(vals)) if None not in vals else None,'min':min(vals) if None not in vals else None,'max':max(vals) if None not in vals else None}
        report['splits'][split]={'auprc':summary,
            'top1_core_sufficiency':{m:v for m,v in topcore.items() if v['checked']},
            'infeasible':infeasible,'physics_certified':certified,
            'median_physics_certificate_s':float(np.median(timing['physics_certificate_s'])) if infeasible else None,
            'median_cpsat_infeasible_s':float(np.median(timing['cpsat_infeasible_s'])) if infeasible else None}
        print(json.dumps({split:report['splits'][split]},indent=1),flush=True)
    (output/f'physics-comparison-{a.tag}.json').write_text(json.dumps(report,indent=2))


if __name__=='__main__': main()
