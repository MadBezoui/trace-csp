"""Extract exact variable scopes from the CP-SAT protobuf, including auxiliaries."""
import math


def signed_log(x):
    return math.copysign(math.log1p(min(abs(x),1e12)),x)/30


def kind_of(p):
    if hasattr(p,'WhichOneof'): return p.WhichOneof('constraint')
    return next((k for k in ('linear','lin_max','int_prod','int_div','bool_or','bool_and') if getattr(p,'has_'+k,lambda:False)()),'other')


def scope_of(p):
    kind=kind_of(p); coeff={}
    if kind=='linear':
        coeff.update(zip(p.linear.vars,p.linear.coeffs))
    elif kind in ('lin_max','int_prod','int_div'):
        expr=getattr(p,kind)
        for e in [expr.target,*expr.exprs]:
            for v,c in zip(e.vars,e.coeffs): coeff[v]=coeff.get(v,0)+c
    for lit in p.enforcement_literal: coeff[lit if lit>=0 else -lit-1]=1
    return coeff


def factor_graph(built):
    p=built.model.proto
    vf=[]; cf=[]; edges=[[],[]]; ea=[]; names=[]; kinds=[]
    vnames=[v.name for v in p.variables]
    for v in p.variables:
        parts=v.name.split('/'); category=parts[0]
        vf.append([signed_log(v.domain[0]),signed_log(v.domain[-1]),float(category in ('m','v')),float(category=='n'),float(category=='q'),float(category=='b'),float(v.name.startswith('assume:')),signed_log(len(v.domain))])
    for ci,c in enumerate(p.constraints):
        kind=kind_of(c); names.append(c.name or f'{kind}:{ci}'); kinds.append(kind)
        scope=scope_of(c)
        lo,hi=(c.linear.domain[0],c.linear.domain[-1]) if kind=='linear' else (0,0)
        cf.append([float(kind=='linear'),float(kind=='lin_max'),float(kind=='int_prod'),float(kind=='int_div'),signed_log(lo),signed_log(hi),signed_log(len(scope)),float(bool(c.enforcement_literal))])
        for vi,coefficient in sorted(scope.items()):
            edges[0].append(vi); edges[1].append(ci); ea.append([signed_log(coefficient),float(coefficient<0)])
    return {'variable_features':vf,'constraint_features':cf,'edge_index':edges,'edge_features':ea,'variable_names':vnames,'constraint_names':names,'constraint_kinds':kinds}
