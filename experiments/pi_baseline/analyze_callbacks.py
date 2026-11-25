#!/usr/bin/env python3
"""Correlate same-Pi observer and actor callback traces; never score a mission."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def largest_gap(rows,wall_key):
    ordered=sorted(rows,key=lambda r:r[wall_key])
    if len(ordered)<2:return None
    left,right=max(zip(ordered,ordered[1:]),key=lambda ab:ab[1][wall_key]-ab[0][wall_key])
    return dict(duration_wall_s=right[wall_key]-left[wall_key],left=left,right=right)


def correlate(trace,probe,begin,end):
    rows={k:[r for r in values if begin<=r['begin_wall']<=end] for k,values in trace['rows'].items()}
    arrivals={k:[r for r in values if begin<=r['wall']<=end] for k,values in probe['rows'].items()}
    clocks=rows.get('actor_clock',[]);independent=arrivals.get('clock',[])
    gap=largest_gap(clocks,'begin_wall')
    if gap:
        a,b=gap['left']['begin_wall'],gap['right']['begin_wall']
        during=[r for r in independent if a<r['wall']<b]
        gap.update(independent_clock_callbacks_during_gap=len(during),independent_clock_samples_during_gap=during,
            actor_ros_time_step=gap['right']['stamp']-gap['left']['stamp'])
    maxima={name:max(values,key=lambda r:r['end_wall']-r['begin_wall']) for name,values in rows.items() if values}
    durations={name:dict(calls=len(values),max_wall_s=max(r['end_wall']-r['begin_wall'] for r in values),
        p95_wall_s=float(np.percentile([r['end_wall']-r['begin_wall'] for r in values],95))) for name,values in rows.items() if values}
    matching={round(r['stamp']*1e9):r['wall'] for r in independent}
    offsets=[r['begin_wall']-matching[round(r['stamp']*1e9)] for r in clocks if round(r['stamp']*1e9) in matching]
    return dict(scope='Same-Pi callback entry/return correlation; not wire arrival, cross-host clock synchronization, causal proof or mission scoring',
        begin_wall=begin,end_wall=end,durations=durations,longest_calls=maxima,
        largest_actor_clock_callback_gap=gap,largest_independent_clock_callback_gap=largest_gap(independent,'wall'),
        largest_driver_clock_callback_gap=largest_gap(rows.get('driver_clock',[]),'begin_wall'),
        matching_clock_offsets=dict(samples=len(offsets),definition='Actor entry minus independent-probe callback wall time; signed difference, not network latency',
            minimum_wall_s=min(offsets,default=None),p95_wall_s=float(np.percentile(offsets,95)) if offsets else None,maximum_wall_s=max(offsets,default=None)),
        retained_actor_rows=sum(len(v) for v in rows.values()),retained_independent_rows=sum(len(v) for v in arrivals.values()))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evidence',type=Path,required=True);a=p.parse_args();root=a.evidence
    paths=dict(worker=root/'pi_results/actor/worker.json',trace=root/'pi_results/actor/callback_trace.json',probe=root/'pi_results/arrival_probe.json')
    data={k:json.loads(v.read_text()) for k,v in paths.items()}
    worker=data['worker'];begin=worker.get('dispatch_wall',0.)
    faults=[r['wall'] for r in worker['events'] if r['event']=='fault' and r['wall']>=begin]
    end=min(faults) if faults else max(r['end_wall'] for v in data['trace']['rows'].values() for r in v)
    result=correlate(data['trace'],data['probe'],begin,end)
    result.update(actor_passed=worker['passed'],actor_error=worker.get('error'),
        inputs_sha256={k:hashlib.sha256(v.read_bytes()).hexdigest() for k,v in paths.items()},
        analyzer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (root/'callback_analysis.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
