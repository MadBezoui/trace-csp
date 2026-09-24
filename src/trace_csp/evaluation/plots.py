"""Simple exportable diagnostics; captions retain experiment scope."""
from collections import defaultdict
import json
import os
from pathlib import Path
import statistics
import tempfile
cache=Path(tempfile.gettempdir())/'trace-csp-plot-cache'
cache.mkdir(parents=True,exist_ok=True)
os.environ.setdefault('MPLCONFIGDIR',str(cache))
os.environ.setdefault('XDG_CACHE_HOME',str(cache))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def create_report(solver_experiment,episodes,output_dir):
    path=Path(output_dir); path.mkdir(parents=True,exist_ok=True)
    by=defaultdict(list)
    for row in solver_experiment['runs']: by[row['controller']].append(row)
    report={'scope':solver_experiment['scope'],'controllers':{},'episode_scope':'synthetic CTM closed loop','episodes':len(episodes)}
    colors=plt.get_cmap('tab10')
    fig,ax=plt.subplots(figsize=(7,4.5),constrained_layout=True)
    for j,(name,rows) in enumerate(by.items()):
        times=sorted(row.get('end_to_end_wall_s',row['wall_s']) for row in rows)
        ax.step(times,np.arange(1,len(times)+1)/len(times),where='post',label=name,color=colors(j))
        report['controllers'][name]={'attempted':len(rows),'feasible':sum(row['feasible'] for row in rows),
            'median_end_to_end_s':float(statistics.median(times)),'median_tts':float(statistics.median(row['tts'] for row in rows if row['tts'] is not None)) if any(row['tts'] is not None for row in rows) else None}
    ax.set(xlabel='End-to-end runtime (s)',ylabel='Cumulative fraction',title='Matched synthetic solver cases')
    ax.grid(alpha=.25); ax.legend(frameon=False)
    fig.savefig(path/'runtime_ecdf.pdf',bbox_inches='tight'); fig.savefig(path/'runtime_ecdf.png',dpi=180,bbox_inches='tight'); plt.close(fig)
    if episodes:
        ep=episodes[0]; records=ep['records']; fig,ax=plt.subplots(figsize=(7,4.5),constrained_layout=True)
        if records:
            queues=np.asarray([r['queue'] for r in records])
            for j in range(queues.shape[1]): ax.plot(np.arange(len(queues)),queues[:,j],marker='o',label=f'Ramp {j}')
        ax.set(xlabel='Control period',ylabel='Queue (integer vehicles)',title=f'Synthetic CTM — {ep["controller"]}, seed {ep["seed"]}')
        ax.grid(alpha=.25)
        if records: ax.legend(frameon=False)
        fig.savefig(path/'queue_trajectory.pdf',bbox_inches='tight'); fig.savefig(path/'queue_trajectory.png',dpi=180,bbox_inches='tight'); plt.close(fig)
    (path/'report.json').write_text(json.dumps(report,indent=2))
    return report
