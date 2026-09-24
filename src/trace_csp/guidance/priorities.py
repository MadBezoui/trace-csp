
def scores_to_priority(graph,scores,control_names):
    """Rank controls using incident constraint scores; all controls remain."""
    names=graph['variable_names']; totals={name:0. for name in control_names}
    for vi,ci in zip(*graph['edge_index']):
        name=names[vi]
        if name in totals: totals[name]+=float(scores[ci])
    return sorted(totals,key=lambda n:(-totals[n],n))


def calibrated_constraint_scores(logits,temperature):
    return sum((logits[k]/temperature[k]).sigmoid() for k in ('active','violation','core'))
