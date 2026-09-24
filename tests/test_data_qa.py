from trace_csp.data import generate_instance,write_instances
from trace_csp.pipeline import solve_offline
from trace_csp.evaluation.data_qa import audit_dataset


def test_audit_catches_group_leak_and_counts_core_labels(tmp_path):
    raw=tmp_path/'r.jsonl.gz'; lab=tmp_path/'l.jsonl.gz'
    write_instances(raw,(generate_instance(k,horizon=1) for k in range(20)))
    solve_offline(raw,lab)
    report=audit_dataset(raw,lab)
    assert report['instances']==20
    assert report['labelled']==20
    assert report['status']['INFEASIBLE']>0
    assert report['core_positives']>0
    assert report['group_leaks']==[]
    assert report['label_balance']['train']['core']['labelled']>0
