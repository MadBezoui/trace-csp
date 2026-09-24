from dataclasses import replace
import numpy as np
import pytest
from trace_csp.data import generate_instance, write_instances, read_instances
from trace_csp.csp.solver import solve
from trace_csp.csp.builder import build
from trace_csp.graph.factor_graph import factor_graph
from trace_csp.graph.labels import labels_for
from trace_csp.csp.controller import Controller
from trace_csp.evaluation.statistics import paired_summary, holm


def test_serialization_and_group_splits(tmp_path):
    rows=[generate_instance(k) for k in range(50)]
    path=tmp_path/'instances.jsonl.gz'; write_instances(path,rows)
    assert list(read_instances(path))==rows
    groups={}
    for row in rows:
        assert groups.setdefault(row.group,row.split)==row.split
    assert generate_instance(3)==generate_instance(3)
    assert generate_instance(3)!=generate_instance(4)


def test_graph_is_real_constraint_scope_and_masks_missing_labels():
    i=generate_instance(0,horizon=1)
    b=build(i); g=factor_graph(b); r=solve(i); labels=labels_for(i,b,g,r)
    assert len(g['variable_features'])==len(b.variables)
    assert max(g['edge_index'][0])<len(g['variable_features'])
    assert max(g['edge_index'][1])<len(g['constraint_features'])
    assert len(labels['active'])==len(g['constraint_features'])
    assert any(labels['active_mask'])
    assert not all(labels['active_mask'])  # nonlinear slack is not fabricated
    assert all(np.isfinite(np.array(g['variable_features'])).flat)


def test_multicell_multiscenario_solution_replays():
    for seed in range(5):
        i=generate_instance(seed,horizon=2)
        r=solve(i)
        assert r.status in ('OPTIMAL','FEASIBLE')
        assert r.actions is not None
        assert len(r.trajectories)==len(i.scenarios)


def test_controller_timeout_verifies_fallback_and_refuses_impossible():
    i=generate_instance(0,horizon=1)
    i=replace(i,config=replace(i.config,time_limit_s=0))
    d=Controller().act(i)
    assert d.action is not None and d.certificate.safe
    assert d.mode=='fallback'
    w=i.scenarios[0]
    bad=replace(i,scenarios=(replace(w,ramps=((100,100),),capacity_factor=((0,0),)),))
    d=Controller().act(bad)
    assert d.action is None and d.mode=='no-certified-action'


def test_paired_statistics_and_holm():
    result=paired_summary([4,5,6,7],[1,2,3,4],seed=1)
    assert result['mean_difference']==3
    assert result['ci95']==[3,3]
    assert holm([.01,.04,.03])==pytest.approx([.03,.06,.06])
    with pytest.raises(ValueError): paired_summary([1],[1,2])


def test_compound_event_generates_real_infeasibility_and_core_label():
    i=generate_instance(6,horizon=1)
    r=solve(i)
    assert r.status=='INFEASIBLE'
    assert any('queue/' in x for x in r.unsat_core)
    b=build(i); g=factor_graph(b); lab=labels_for(i,b,g,r)
    assert sum(lab['core'])>0


def test_stressed_ood_examples_have_proven_conflicts():
    for family in ('demand_shift','structural_ood','severe_incident'):
        for ramp in (0,1):
            i=generate_instance(20000,horizon=2,family=family,stress_queue_conflict=True,stress_ramp_index=ramp)
            r=solve(i)
            assert r.status=='INFEASIBLE'
            assert len(r.unsat_core)==1
            assert r.unsat_core[0].endswith(f'/{ramp}')
            assert i.split.startswith('test_ood_')


def test_experiment_families_and_incident_levels_are_distinct():
    seed=25
    for factor in (1000,800,500):
        rows=[generate_instance(seed,family=family,incident_factor=factor) for family in
              ('normal','mild_incident','severe_incident','sensor_fault','communication_fault')]
        assert len({row.scenarios for row in rows})==len(rows)
    compound=[generate_instance(seed,family='compound',incident_factor=factor).scenarios
              for factor in (1000,800,500)]
    assert len(set(compound))==3
    all_cases=[generate_instance(seed,family=family,incident_factor=factor) for family in
               ('normal','mild_incident','severe_incident','sensor_fault','demand_shift',
                'communication_fault','compound','structural_ood') for factor in (1000,800,500)]
    assert len({(row.corridor,row.state,row.scenarios) for row in all_cases})==24


def test_controller_never_uses_solution_after_deadline(monkeypatch):
    import time
    import trace_csp.csp.controller as controller_module
    instance=generate_instance(0,horizon=1)
    instance=replace(instance,config=replace(instance.config,time_limit_s=.001))
    genuine=solve(replace(instance,config=replace(instance.config,time_limit_s=2)))
    def late_solve(*args,**kwargs):
        time.sleep(.01)
        return genuine
    monkeypatch.setattr(controller_module,'solve',late_solve)
    decision=Controller().act(instance)
    assert decision.mode!='trace-csp'
    assert decision.action is None or decision.certificate.safe


def test_fallback_audit_records_reason_and_solver_status():
    i=generate_instance(0,horizon=1)
    i=replace(i,config=replace(i.config,time_limit_s=0))
    d=Controller().act(i)
    assert d.audit['mode']=='fallback'
    assert d.audit['solver_status']=='UNKNOWN'
    assert d.audit['reason']
