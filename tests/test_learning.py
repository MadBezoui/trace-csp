import numpy as np
import torch
from trace_csp.data import generate_instance
from trace_csp.csp.builder import build
from trace_csp.csp.solver import solve
from trace_csp.graph.factor_graph import factor_graph
from trace_csp.graph.labels import labels_for
from trace_csp.learning.constraint_transformer import ConstraintAttentionNet, GlobalMLP, BipartiteGNN, graph_to_tensors
from trace_csp.learning.losses import multitask_loss
from trace_csp.learning.calibrate import fit_temperature
from trace_csp.evaluation.metrics import binary_metrics
from trace_csp.guidance.priorities import scores_to_priority
from trace_csp.guidance.neural_lns import guided_lns


def example():
    i=generate_instance(0,horizon=1); b=build(i); g=factor_graph(b); r=solve(i)
    return i,b,g,labels_for(i,b,g,r)


def test_models_emit_one_logit_per_constraint_and_have_gradients():
    _,_,g,y=example(); t=graph_to_tensors(g,y)
    for model in (GlobalMLP(),BipartiteGNN(),ConstraintAttentionNet()):
        out=model(t)
        assert all(out[k].shape==(len(g['constraint_features']),) for k in ('active','violation','core'))
        loss=multitask_loss(out,t)
        assert torch.isfinite(loss)
        loss.backward()
        assert any(p.grad is not None for p in model.parameters())


def test_unavailable_labels_do_not_change_loss():
    _,_,g,y=example(); t=graph_to_tensors(g,y); out=GlobalMLP()(t)
    original=multitask_loss(out,t).item()
    missing=~t['active_mask']; t['active'][missing]=1-t['active'][missing]
    assert abs(multitask_loss(out,t).item()-original)<1e-6


def test_temperature_and_metrics_handle_imbalance():
    scores=np.array([.9,.8,.2,.1]); labels=np.array([1,0,0,0])
    metrics=binary_metrics(scores,labels)
    assert metrics['auprc']>.25
    assert metrics['prevalence']==.25
    assert metrics['recall_at_1']==1
    cal=fit_temperature(np.array([3.,2.,-1.,-2.]),labels)
    assert cal>0 and np.isfinite(cal)


def test_average_precision_and_top_k_are_invariant_to_tie_order():
    first=binary_metrics([.5,.5],[1,0],k=1)
    reversed_rows=binary_metrics([.5,.5],[0,1],k=1)
    for report in (first,reversed_rows):
        assert report['auprc']==.5
        assert report['precision_at_k']==.5
        assert report['recall_at_k']==.5


def test_guidance_keeps_all_constraints_and_lns_feasible():
    i,b,g,y=example()
    scores=np.arange(len(g['constraint_features']),dtype=float)
    order=scores_to_priority(g,scores,tuple(b.controls))
    assert set(order)==set(b.controls)
    warm=solve(i)
    result=guided_lns(i,warm,g,scores,time_limit_s=.5)
    assert result.actions is not None
    assert result.status in ('FEASIBLE','OPTIMAL')


def test_temperature_is_applied_to_logits_before_sigmoid():
    import torch
    from trace_csp.guidance.priorities import calibrated_constraint_scores
    logits={'active':torch.tensor([2.,-1.]),'violation':torch.tensor([0.,3.]),'core':torch.tensor([-2.,1.])}
    temperature={'active':2.,'violation':1.,'core':4.}
    expected=torch.sigmoid(logits['active']/2)+torch.sigmoid(logits['violation'])+torch.sigmoid(logits['core']/4)
    actual=calibrated_constraint_scores(logits,temperature)
    assert torch.allclose(actual,expected)
