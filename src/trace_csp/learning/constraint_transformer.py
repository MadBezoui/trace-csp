"""Constraint heads over the exact CP-SAT factor graph."""
import torch
from torch import nn
from torch_geometric.nn import HGTConv, SAGEConv

METADATA=(['variable','constraint'],[('variable','scope','constraint'),('constraint','rev_scope','variable')])


def graph_to_tensors(g,labels=None):
    edges=torch.tensor(g['edge_index'],dtype=torch.long)
    if edges.numel()==0: edges=torch.empty((2,0),dtype=torch.long)
    data={'variable':torch.tensor(g['variable_features'],dtype=torch.float32),
          'constraint':torch.tensor(g['constraint_features'],dtype=torch.float32),
          'edges':edges,'reverse_edges':edges.flip(0).contiguous()}
    if labels:
        for k,v in labels.items(): data[k]=torch.tensor(v,dtype=torch.bool if k.endswith('_mask') else torch.float32)
    return data


class Heads(nn.Module):
    def __init__(self,hidden):
        super().__init__(); self.layers=nn.ModuleDict({k:nn.Linear(hidden,1) for k in ('active','violation','core')})
    def forward(self,x): return {k:layer(x).squeeze(-1) for k,layer in self.layers.items()}


class GlobalMLP(nn.Module):
    def __init__(self,hidden=32):
        super().__init__(); self.var=nn.Linear(8,hidden); self.net=nn.Sequential(nn.Linear(8+hidden,hidden),nn.ReLU(),nn.Linear(hidden,hidden),nn.ReLU()); self.head=Heads(hidden)
    def forward(self,data):
        global_var=self.var(data['variable']).relu().mean(dim=0).expand(len(data['constraint']),-1)
        return self.head(self.net(torch.cat((data['constraint'],global_var),dim=1)))


class BipartiteGNN(nn.Module):
    def __init__(self,hidden=32,layers=2):
        super().__init__(); self.var=nn.Linear(8,hidden); self.con=nn.Linear(8,hidden)
        self.v2c=nn.ModuleList(SAGEConv((hidden,hidden),hidden) for _ in range(layers))
        self.c2v=nn.ModuleList(SAGEConv((hidden,hidden),hidden) for _ in range(layers))
        self.head=Heads(hidden)
    def forward(self,data):
        v=self.var(data['variable']).relu(); c=self.con(data['constraint']).relu()
        for vc,cv in zip(self.v2c,self.c2v):
            new_c=vc((v,c),data['edges']).relu()+c
            v=cv((c,v),data['reverse_edges']).relu()+v
            c=new_c
        return self.head(c)


class ConstraintAttentionNet(nn.Module):
    def __init__(self,hidden=32,heads=2,layers=2):
        super().__init__(); self.var=nn.Linear(8,hidden); self.con=nn.Linear(8,hidden)
        self.layers=nn.ModuleList(HGTConv(hidden,hidden,METADATA,heads=heads) for _ in range(layers))
        self.head=Heads(hidden)
    def forward(self,data):
        x={'variable':self.var(data['variable']).relu(),'constraint':self.con(data['constraint']).relu()}
        edges={METADATA[1][0]:data['edges'],METADATA[1][1]:data['reverse_edges']}
        for layer in self.layers:
            updated=layer(x,edges)
            x={key:torch.relu(updated[key]+x[key]) for key in x}
        return self.head(x['constraint'])
