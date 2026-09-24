"""Streaming integrity audit of generated instances and solver labels."""
from collections import Counter,defaultdict
import hashlib
from itertools import zip_longest
from trace_csp.data import read_instances
from trace_csp.pipeline import read_labelled


def digest(path):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''): h.update(chunk)
    return h.hexdigest()


def audit_dataset(instances_path,labelled_path):
    groups=defaultdict(set); ids=Counter(); statuses=Counter(); splits=Counter(); families=Counter()
    balance=defaultdict(lambda:defaultdict(lambda:{'positives':0,'labelled':0}))
    count=0; core_positives=0
    for instance,label in zip_longest(read_instances(instances_path),read_labelled(labelled_path)):
        if instance is None or label is None: raise ValueError('instance/label count mismatch')
        count+=1; core_positives+=sum(label['labels']['core'])
        if instance.instance_id!=label['id'] or instance.group!=label['group'] or instance.split!=label['split']:
            raise ValueError('instance and label order or metadata mismatch')
        ids[instance.instance_id]+=1; groups[instance.group].add(instance.split)
        statuses[label['status']]+=1; splits[instance.split]+=1; families[instance.family]+=1
        for key in ('active','violation','core'):
            mask=label['labels'][key+'_mask']; values=label['labels'][key]
            balance[instance.split][key]['positives']+=sum(value for value,valid in zip(values,mask) if valid)
            balance[instance.split][key]['labelled']+=sum(mask)
        n=len(label['graph']['constraint_features'])
        if any(len(label['labels'][k])!=n for k in label['labels']): raise ValueError('label/graph size mismatch')
        if label['status']=='INFEASIBLE' and any(label['labels']['active_mask']): raise ValueError('fabricated active labels')
    return {'instances':count,'labelled':count,'status':dict(statuses),'split':dict(splits),'family':dict(families),
            'group_leaks':sorted(g for g,ss in groups.items() if len(ss)>1),
            'duplicate_ids':sorted(k for k,v in ids.items() if v>1),
            'core_positives':core_positives,
            'label_balance':{split:dict(tasks) for split,tasks in balance.items()},
            'source_sha256':digest(instances_path),'labels_sha256':digest(labelled_path)}
