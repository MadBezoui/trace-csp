"""Frozen held-out model and constraint-frequency comparisons."""
import argparse
import json
from pathlib import Path
from statistics import median
from trace_csp.evaluation.data_qa import digest
from trace_csp.pipeline import evaluate_probe,evaluate_frequency_baseline


def main():
    p=argparse.ArgumentParser();p.add_argument('--train',required=True);p.add_argument('--challenge',required=True)
    p.add_argument('--output-dir',default='results');p.add_argument('--tag',required=True)
    p.add_argument('--architectures',nargs='+',choices=('mlp','bipartite','hgt'),default=['mlp','bipartite','hgt'])
    p.add_argument('--seeds',type=int,default=5)
    a=p.parse_args()
    if a.seeds<1:p.error('seeds must be positive')
    output=Path(a.output_dir);output.mkdir(parents=True,exist_ok=True)
    jobs=[('test_id',a.train)]+[(s,a.challenge) for s in ('test_ood_demand','test_ood_structure','test_ood_incident')]
    report={'scope':'synthetic frozen held-out evaluation; OOD is one engineered queue-conflict challenge',
            'train_labels_sha256':digest(a.train),'challenge_labels_sha256':digest(a.challenge),'models':{}}
    for architecture in ['frequency']+a.architectures:
        split_report={}
        for split,source in jobs:
            rows=[]
            for seed in ([None] if architecture=='frequency' else range(a.seeds)):
                if seed is None:
                    row=evaluate_frequency_baseline(a.train,source,split)
                    path=output/f'eval-frequency-{a.tag}-{split}.json'
                else:
                    row=evaluate_probe(source,output/f'{architecture}-{a.tag}-seed{seed}.pt',split)
                    path=output/f'eval-{architecture}-{a.tag}-seed{seed}-{split}.json'
                path.write_text(json.dumps(row,indent=2));rows.append(row)
                print(json.dumps({'architecture':architecture,'seed':seed,'split':split,
                                  'core_auprc':row['metrics']['core']['auprc']}),flush=True)
            metrics={}
            for key in ('active','violation','core'):
                values=[row['metrics'][key]['auprc'] for row in rows]
                metrics[key]={'auprc_by_seed':values,'median_auprc':median(values) if all(v is not None for v in values) else None,
                              'min_auprc':min(values) if all(v is not None for v in values) else None,
                              'max_auprc':max(values) if all(v is not None for v in values) else None,
                              'prevalence':rows[0]['metrics'][key]['prevalence'],
                              'positives':rows[0]['metrics'][key]['positives']}
            split_report[split]={'instances':rows[0]['instances'],'metrics':metrics}
        report['models'][architecture]=split_report
    (output/f'model-comparison-{a.tag}.json').write_text(json.dumps(report,indent=2))

if __name__=='__main__':main()
