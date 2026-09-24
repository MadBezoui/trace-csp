import numpy as np
from scipy.stats import rankdata


def binary_metrics(probabilities,labels,k=None,bins=10):
    p=np.asarray(probabilities,dtype=float); y=np.asarray(labels,dtype=int)
    if p.shape!=y.shape or p.ndim!=1 or len(y)==0 or not np.isfinite(p).all() or not np.isin(y,[0,1]).all() or np.any((p<0)|(p>1)):
        raise ValueError('valid binary scores and labels required')
    npos=int(y.sum()); nneg=len(y)-npos; k=max(1,min(len(y),k or npos or 1))
    order=np.argsort(-p,kind='stable')
    ranked_p=p[order]; ranked_y=y[order]
    # Average precision changes only at distinct score thresholds. Treating
    # tied rows one by one makes the result depend on their input order.
    ends=np.r_[np.flatnonzero(np.diff(ranked_p)!=0),len(y)-1]
    cumulative=np.cumsum(ranked_y)[ends]
    gained=np.diff(np.r_[0,cumulative])
    auprc=float(np.sum((cumulative/(ends+1))*gained)/npos) if npos else None
    cutoff=ranked_p[k-1]
    above=p>cutoff; tied=p==cutoff
    top_positives=float(y[above].sum()+(k-int(above.sum()))*y[tied].mean())
    auc=float((rankdata(p)[y==1].sum()-npos*(npos+1)/2)/(npos*nneg)) if npos and nneg else None
    ece=0.
    for b in range(bins):
        mask=(p>=b/bins)&((p<(b+1)/bins) if b<bins-1 else (p<=1))
        if mask.any(): ece+=mask.mean()*abs(p[mask].mean()-y[mask].mean())
    return {'n':len(y),'positives':npos,'prevalence':float(npos/len(y)),'auroc':auc,'auprc':auprc,'precision_at_k':top_positives/k,
            'recall_at_k':top_positives/npos if npos else None,'recall_at_1':float(y[p==p.max()].mean()/npos) if npos else None,
            'brier':float(((p-y)**2).mean()),'ece':float(ece),'critical_false_negative_rate':float(((p[y==1]<.5).sum()/npos)) if npos else None}
