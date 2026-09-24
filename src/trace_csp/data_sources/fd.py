"""Per-station triangular fundamental diagram from observed detector data.

Inputs are per-lane flow (veh/h), speed (km/h) and occupancy (fraction).
Density uses an effective vehicle length estimated in free flow (k = q / v,
g = occ / k), so no fixed g-factor is assumed. Only observed samples should be
passed; PeMS imputed rows (% observed = 0) must be removed beforehand.
"""
import numpy as np
from scipy.stats import theilslopes


def fit_triangular(flow_vph,speed_kmh,occ,capacity_quantile=.99,min_congested=50):
    q=np.asarray(flow_vph,float); v=np.asarray(speed_kmh,float); o=np.asarray(occ,float)
    ok=np.isfinite(q)&np.isfinite(v)&np.isfinite(o)&(q>0)&(v>0)&(o>0)
    q,v,o=q[ok],v[ok],o[ok]
    if len(q)<200: raise ValueError('not enough observed samples')
    free=o<=np.quantile(o,.3)
    v_free=float(np.median(v[free]))
    g_km=float(np.median(o[free]*v[free]/q[free]))  # effective length (km) = occ / (q / v)
    k=o/g_km
    q_cap=float(np.quantile(q,capacity_quantile))
    k_crit=q_cap/v_free
    congested=(v<.7*v_free)&(k>k_crit)
    w=k_jam=None
    if congested.sum()>=min_congested:
        slope,intercept,_,_=theilslopes(q[congested],k[congested])
        if slope<0:
            w=float(-slope); k_jam=float(intercept/w)
    return {'n':int(len(q)),'n_congested':int(congested.sum()),'free_speed_kmh':v_free,
            'capacity_vphpl':q_cap,'effective_length_m':g_km*1000,'critical_density_vpkmpl':k_crit,
            'wave_speed_kmh':w,'jam_density_vpkmpl':k_jam}
