#!/usr/bin/env python3
"""Derive local replay timing and sampled per-process costs from frozen evidence."""
import argparse
import hashlib
import json
from pathlib import Path
import tarfile
import numpy as np
from replay_timing import timing_summary,clock_path_summary


def analyze(folder):
    paths={name:folder/'results'/name for name in ['feed.json','observations.json','resources.json','result.json']}
    values={k:json.loads(p.read_text()) for k,p in paths.items()}
    with tarfile.open(folder/'sources.tar.gz') as t:
        manifest=json.load(t.extractfile('replay_manifest.json'))
        for name,want in manifest['files_sha256'].items():
            if hashlib.sha256(t.extractfile(name).read()).hexdigest()!=want:raise ValueError('frozen_replay_source_changed:'+name)
        with np.load(t.extractfile('observations.npz'),allow_pickle=False) as data:
            stamps=data['stamps'];first=int(np.searchsorted(stamps,stamps[0]+1.))
            expected=[round(float(v)*1e9) for v in stamps[first:]]
    feed=values['feed.json'];observations=values['observations.json']
    if feed['scan_stamps_ns']!=expected:raise ValueError('replay_interval_does_not_match_frozen_recording')
    samples=[v for v in values['resources.json']['samples'] if feed['begin_wall']<=v['wall_monotonic']<=feed['end_wall']]
    by_pid={}
    for row in samples:
        for proc in row['processes']:by_pid.setdefault(proc['pid'],[]).append((row['wall_monotonic'],proc))
    costs=[]
    for pid,rows in by_pid.items():
        first,last=rows[0],rows[-1];dt=last[0]-first[0]
        delta=last[1]['cpu_s']-first[1]['cpu_s']
        costs.append(dict(pid=pid,argv=first[1]['argv'],samples=len(rows),sample_span_wall_s=dt,
            cpu_delta_s=delta,cpu_percent_one_core=100*delta/dt if dt>0 else None,
            sampled_pss_max_kib=max(v['memory_kib']['Pss'] for _,v in rows),
            sampled_rss_max_kib=max(v['memory_kib']['Rss'] for _,v in rows)))
    updates=[v for v in observations['amcl_updates'] if feed['begin_wall']<=v['wall']<=feed['end_wall']]
    report=dict(timing=timing_summary(feed,observations),role_passed=values['result.json']['passed'],
        clock_path=clock_path_summary(feed,observations),
        amcl_updates_during_replay=len(updates),process_costs=costs,
        resource_scope='Replay interval only; sampled process CPU delta over first/last in-interval sample, 100% = one core. Replay publisher and observer costs listed separately. No EKF, Nav2, route preprocessing or mission workload.',
        reconstruction=feed['scope'],recording_sha256=manifest['recording_sha256'],
        analysis_sources_sha256={name:hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
            for name in ['analyze_local_replay.py','replay_timing.py']},
        evidence_sha256={name:hashlib.sha256(p.read_bytes()).hexdigest() for name,p in paths.items()})
    report['passed']=bool(report['role_passed'] and report['timing']['passed'])
    (folder/'independent_timing_summary.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evidence',type=Path,required=True)
    a=p.parse_args();result=analyze(a.evidence);print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['passed'] else 1)
