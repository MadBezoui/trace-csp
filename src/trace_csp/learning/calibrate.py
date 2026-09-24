import numpy as np
from scipy.optimize import minimize_scalar
from scipy.special import expit


def fit_temperature(logits,labels):
    logits=np.asarray(logits,dtype=float); labels=np.asarray(labels,dtype=float)
    if logits.shape!=labels.shape or logits.ndim!=1 or len(logits)==0 or not np.isfinite(logits).all() or not np.isin(labels,[0,1]).all():
        raise ValueError('finite binary validation labels required')
    def loss(t):
        p=np.clip(expit(logits/t),1e-9,1-1e-9)
        return -(labels*np.log(p)+(1-labels)*np.log1p(-p)).mean()
    return float(minimize_scalar(loss,bounds=(.05,20),method='bounded').x)
