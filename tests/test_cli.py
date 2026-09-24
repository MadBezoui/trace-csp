import json
import subprocess
import sys
from pathlib import Path
from trace_csp.data import generate_instance, write_instances, read_instances
from trace_csp.pipeline import solve_offline, train_probe, compare_controllers, evaluate_probe, evaluate_frequency_baseline


def test_offline_training_and_paired_experiment(tmp_path):
    raw=tmp_path/'raw.jsonl.gz'; labelled=tmp_path/'labelled.jsonl.gz'; checkpoint=tmp_path/'model.pt'
    write_instances(raw,(generate_instance(k,horizon=1) for k in range(90)))
    summary=solve_offline(raw,labelled,limit=90)
    assert summary['attempted']==90
    assert summary['feasible']>0
    rows=[json.loads(line) for line in __import__('gzip').open(labelled,'rt')]
    assert len(rows)==90 and all('graph' in x and 'labels' in x for x in rows)
    report=train_probe(labelled,checkpoint,architecture='mlp',epochs=1,seed=0)
    assert checkpoint.exists() and report['n_train']>0 and report['n_validation']>0
    assert not (set(report['train_groups']) & set(report['validation_groups']))
    held_out=evaluate_probe(labelled,checkpoint,'test_id')
    assert held_out['split']=='test_id' and held_out['instances']>0
    assert set(held_out['groups']).isdisjoint(report['train_groups']+report['validation_groups'])
    assert held_out['metrics']['active']['n']>0
    frequency=evaluate_frequency_baseline(labelled,labelled,'test_id')
    assert frequency['metrics']['core']['n']>0
    assert frequency['groups']==held_out['groups']
    exp=compare_controllers([generate_instance(k,horizon=1) for k in range(4)],controllers=('cp_sat','warm_cp_sat'))
    assert exp['seeds']==4 and set(exp['controllers'])=={'cp_sat','warm_cp_sat'}
    assert len(exp['runs'])==8


def test_cli_smoke(tmp_path):
    p=tmp_path/'instances.jsonl.gz'
    process=subprocess.run([sys.executable,'-m','trace_csp.cli','generate','--count','3','--horizon','1','--output',str(p)],capture_output=True,text=True)
    assert process.returncode==0,process.stderr
    assert len(list(read_instances(p)))==3


def test_comparison_reports_preprocessing_time():
    row=compare_controllers([generate_instance(0,horizon=1)],controllers=('cp_sat',))['runs'][0]
    assert row['end_to_end_wall_s']>=row['solver_wall_s']>=0


def test_comparison_excludes_verified_but_late_actions(monkeypatch):
    import time
    from dataclasses import replace
    import trace_csp.pipeline as pipeline
    from trace_csp.csp.solver import solve
    instance=generate_instance(0,horizon=1)
    genuine=solve(instance)
    instance=replace(instance,config=replace(instance.config,time_limit_s=.001))
    monkeypatch.setattr(pipeline,'load_predictor',lambda _:lambda _i:(time.sleep(.01) or []))
    monkeypatch.setattr(pipeline,'solve',lambda *_args,**_kwargs:genuine)
    row=compare_controllers([instance],controllers=('learned_cp_sat',),checkpoint='unused')['runs'][0]
    assert row['preprocessing_s']>instance.config.time_limit_s
    assert not row['deadline_met']
    assert not row['feasible']
