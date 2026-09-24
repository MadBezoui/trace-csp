import numpy as np
from trace_csp.data_sources.fd import fit_triangular


def test_recovers_synthetic_triangular_diagram():
    rng=np.random.default_rng(0); vf,w,kj,g=100.,18.,150.,.0065
    kc=w*kj/(vf+w); k=rng.uniform(2,140,5000)
    q=np.where(k<=kc,vf*k,w*(kj-k)); v=q/k; o=k*g
    fit=fit_triangular(q,v,o)
    assert abs(fit['free_speed_kmh']-vf)<1 and abs(fit['effective_length_m']-6.5)<.1
    assert abs(fit['wave_speed_kmh']-w)<1 and abs(fit['jam_density_vpkmpl']-kj)<3
    assert abs(fit['capacity_vphpl']-vf*kc)/(vf*kc)<.03
