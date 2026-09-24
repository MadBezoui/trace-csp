import torch
import torch.nn.functional as F


def masked_bce(logits,targets,mask,focal=False):
    if not mask.any(): return logits.sum()*0
    selected=logits[mask]; truth=targets[mask]
    prevalence=truth.mean().detach()
    pos_weight=((1-prevalence)/(prevalence+1e-6)).clamp(min=1,max=20)
    raw=F.binary_cross_entropy_with_logits(selected,truth,pos_weight=pos_weight,reduction='none')
    if focal:
        probability=torch.sigmoid(selected)
        pt=probability*truth+(1-probability)*(1-truth)
        raw=raw*(1-pt).pow(2)
    return raw.mean()


def multitask_loss(outputs,labels,weights=None):
    weights=weights or {'active':1,'violation':1,'core':1,'ranking':.1}
    loss=sum(weights.get(k,1)*masked_bce(outputs[k],labels[k],labels[k+'_mask'],focal=k=='violation') for k in ('active','violation','core'))
    mask=labels['active_mask'] & labels['core_mask']
    if mask.any():
        score=outputs['active'][mask]; y=labels['active'][mask]
        positives=score[y>0]; negatives=score[y==0]
        if len(positives) and len(negatives):
            loss=loss+weights.get('ranking',.1)*F.softplus(negatives[:,None]-positives[None,:]).mean()
    return loss
