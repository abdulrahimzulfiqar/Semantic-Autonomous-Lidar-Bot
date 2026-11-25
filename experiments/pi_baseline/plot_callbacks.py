#!/usr/bin/env python3
"""Plot same-host callback receipt traces around a recorded pilot failure."""
import argparse
import json
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evidence',type=Path,required=True)
    a=p.parse_args();root=a.evidence
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    actor=json.loads((root/'pi_results/actor/worker.json').read_text())
    trace=json.loads((root/'pi_results/actor/callback_trace.json').read_text())['rows']
    probe=json.loads((root/'pi_results/arrival_probe.json').read_text())['rows']
    fault=next(e['wall'] for e in actor['events'] if e['event']=='fault')
    left,right=fault-1.4,fault+.15
    fig,axes=plt.subplots(2,1,figsize=(9,5.2),sharex=True,gridspec_kw=dict(height_ratios=[3,1]),layout='constrained')
    for name,rows,key,color in [('Independent probe',probe['clock'],'wall','#218739'),
        ('Driver node',trace['driver_clock'],'begin_wall','#2a6cbd'),
        ('Mission node',trace['actor_clock'],'begin_wall','#d05325')]:
        rows=[r for r in rows if left<=r[key]<=right]
        axes[0].step([r[key]-fault for r in rows],[r['stamp'] for r in rows],where='post',label=name,color=color,linewidth=1.8)
    axes[0].set_ylabel('Received ROS clock (simulated s)');axes[0].legend(loc='upper left')
    for i,key in enumerate(['executor_timer','window_heartbeat']):
        spans=[(max(left,r['begin_wall'])-fault,min(right,r['end_wall'])-max(left,r['begin_wall'])) for r in trace.get(key,[]) if r['end_wall']>=left and r['begin_wall']<=right]
        axes[1].broken_barh(spans,(i-.3,.6),facecolors=['#9160af','#567c8d'][i])
    axes[1].set_yticks([0,1],['Control timer','Window heartbeat']);axes[1].set_xlabel('Pi monotonic wall time relative to fault (s)')
    for axis in axes:
        axis.axvline(0,color='#ab2424',linestyle='--',label='Guard fault');axis.grid(alpha=.2);axis.set_xlim(left-fault,right-fault)
    axes[0].set_title(root.name+': diagnostic clock receipt and callback execution')
    fig.savefig(root/'callback_timing.png',dpi=160)
    plt.close(fig)


if __name__=='__main__':main()
