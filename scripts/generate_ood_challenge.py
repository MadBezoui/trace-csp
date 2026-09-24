"""Generate a held-out OOD challenge with verified queue-conflict candidates."""
import argparse
import json
from trace_csp.data import generate_instance,write_instances


def challenge_instances(count_per_family):
    for family,base,step in (('demand_shift',20000,1),('structural_ood',22000,1),('severe_incident',24000,5)):
        for j in range(count_per_family):
            yield generate_instance(base+step*j,horizon=2,family=family,
                                    stress_queue_conflict=j%2==0,stress_ramp_index=(j//2)%2)


def main():
    p=argparse.ArgumentParser(); p.add_argument('--count-per-family',type=int,default=300); p.add_argument('--output',required=True)
    a=p.parse_args()
    if a.count_per_family<2: p.error('count-per-family must be at least 2')
    write_instances(a.output,challenge_instances(a.count_per_family))
    print(json.dumps({'scope':'held-out synthetic OOD challenge','instances':3*a.count_per_family,'output':a.output}))

if __name__=='__main__': main()
