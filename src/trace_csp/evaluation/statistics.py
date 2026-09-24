import numpy as np
from scipy.stats import wilcoxon


def paired_summary(a,b,seed=0,bootstrap_samples=2000):
    a,b=np.asarray(a,dtype=float),np.asarray(b,dtype=float)
    if a.ndim!=1 or a.shape!=b.shape or len(a)<2 or not np.isfinite(a).all() or not np.isfinite(b).all():
        raise ValueError('finite matched pairs (n>=2) required')
    d=a-b; rng=np.random.default_rng(seed)
    means=np.mean(rng.choice(d,(bootstrap_samples,len(d)),replace=True),axis=1)
    sd=float(np.std(d,ddof=1)); nonzero=d[d!=0]
    p=1.0 if not len(nonzero) else float(wilcoxon(d,method='auto').pvalue)
    return {'n':len(d),'mean_difference':float(d.mean()),'median_difference':float(np.median(d)),'std_difference':sd,
            'ci95':[float(v) for v in np.quantile(means,[.025,.975])],'wilcoxon_p':p,
            'cohen_dz':float(d.mean()/sd) if sd else None}


def holm(pvalues):
    p=np.asarray(pvalues,dtype=float)
    if p.ndim!=1 or not np.isfinite(p).all() or np.any((p<0)|(p>1)): raise ValueError('invalid p-values')
    order=np.argsort(p); out=np.empty(len(p)); previous=0.
    for rank,idx in enumerate(order):
        previous=max(previous,min(1.,(len(p)-rank)*p[idx])); out[idx]=previous
    return out.tolist()
