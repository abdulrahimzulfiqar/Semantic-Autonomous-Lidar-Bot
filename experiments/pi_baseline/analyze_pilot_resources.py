#!/usr/bin/env python3
"""Sampled motion-interval costs for a retained pilot, with timing scope explicit."""
import argparse
import hashlib
import json
from pathlib import Path


def process_costs(samples,begin,end):
    selected=[r for r in samples if begin<=r['wall_monotonic']<=end]
    by_pid={}
    for row in selected:
        for p in row['processes']:by_pid.setdefault(p['pid'],[]).append((row['wall_monotonic'],p))
    processes=[]
    for pid,rows in by_pid.items():
        dt=rows[-1][0]-rows[0][0];cpu=rows[-1][1]['cpu_s']-rows[0][1]['cpu_s']
        processes.append(dict(pid=pid,argv=rows[0][1]['argv'],samples=len(rows),sample_span_wall_s=dt,
            cpu_delta_s=cpu,cpu_percent_one_core=100*cpu/dt if dt>0 else None,
            sampled_pss_max_kib=max(p['memory_kib']['Pss'] for _,p in rows),
            sampled_rss_max_kib=max(p['memory_kib']['Rss'] for _,p in rows)))
    sets=[{p['pid'] for p in row['processes']} for row in selected]
    stable=bool(sets and sets[0] and all(s==sets[0] for s in sets))
    span=selected[-1]['wall_monotonic']-selected[0]['wall_monotonic'] if selected else 0.
    total=sum(p['cpu_delta_s'] for p in processes)
    return dict(processes=processes,samples=len(selected),sample_span_wall_s=span,
        stable_process_set=stable,cpu_percent_one_core_total=100*total/span if span>0 and stable else None,
        sampled_concurrent_domain_pss_max_kib=max((sum(p['memory_kib']['Pss'] for p in row['processes']) for row in selected),default=None))


def analyze(folder):
    role_results = folder/'pi_results'
    if not role_results.exists():
        role_results = folder/'results'
    paths=dict(worker=role_results/'actor/worker.json',resources=role_results/'resources.json')
    if (role_results/'arrival_probe.json').exists():
        paths['probe'] = role_results/'arrival_probe.json'
    data={k:json.loads(p.read_text()) for k,p in paths.items()};worker=data['worker']
    sends=[e for e in worker['events'] if e['event']=='send']
    endings=[e for e in worker['events'] if e['event']=='fault' or (e['event']=='result' and e['outcome']=='succeeded')]
    if not sends or not endings:raise ValueError('no_motion_interval_for_resource_summary')
    begin=sends[0]['wall'];end=endings[-1]['wall']
    samples = data['resources']['samples']
    report=process_costs(samples,begin,end)
    firmware = [int(row['firmware_throttled'].split('=')[1], 16)
                for row in samples if (row.get('firmware_throttled') or '').startswith('throttled=')]
    report['firmware_power'] = dict(
        sampled_readings=len(firmware),
        active_undervoltage_readings=sum(bool(value & 1) for value in firmware),
        active_throttle_readings=sum(bool(value & 4) for value in firmware),
        first_flags=hex(firmware[0]) if firmware else None,
        last_flags=hex(firmware[-1]) if firmware else None,
        peak_cpu_temp_c=max((int(row['temperature_millidegrees_c']) / 1000
                            for row in samples if row.get('temperature_millidegrees_c')), default=None),
    )
    probe=data.get('probe', {}).get('rows', {});rates={}
    measured_topics = ('scan','clock','imu') if 'probe' in data else ()
    for kind in measured_topics:
        rows=[r for r in probe.get(kind,[]) if begin<=r['wall']<=end]
        span=rows[-1]['wall']-rows[0]['wall'] if len(rows)>1 else 0.
        rates[kind]=dict(callbacks=len(rows),receipt_span_wall_s=span,
            callbacks_per_wall_s=(len(rows)-1)/span if span else None,
            header_sim_per_wall_s=(rows[-1]['stamp']-rows[0]['stamp'])/span if span else None,
            max_receipt_gap_wall_s=max((b['wall']-a['wall'] for a,b in zip(rows,rows[1:])),default=None))
    report.update(scope='One pilot from dispatch to last native success/fault response, excluding preprocessing/rest verification/teardown. CPU uses actual sample span and 100% denotes one core. Relay and optional trace probe are separate processes. The sampler owner is excluded.',
        callback_tracing=(role_results/'actor/callback_trace.json').exists(),
        begin_wall=begin,end_wall=end,interval_wall_s=end-begin,actor_passed=worker['passed'],
        preprocessing_wall_s=worker['preprocessing_wall_s'],observed_rates=rates,
        inputs_sha256={k:hashlib.sha256(p.read_bytes()).hexdigest() for k,p in paths.items()},
        analyzer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    if role_results.name == 'results':
        feed_path = role_results/'replay_feed.json'
        feed = json.loads(feed_path.read_text())
        report['inputs_sha256']['replay_feed'] = hashlib.sha256(feed_path.read_bytes()).hexdigest()
        commands = [row[1] for row in feed['commands'] if begin <= row[0] <= end]
        report['local_replay'] = dict(completed=worker.get('replay_interval_completed',False),
            controller_outputs=len(commands), maximum_abs_linear_m_s=max(map(abs,commands),default=None),
            nonzero_linear_outputs=sum(abs(value)>.001 for value in commands),
            clip_inputs=feed['clip_inputs'], maximum_publication_lateness_wall_s=feed['maximum_publication_lateness_wall_s'])
    (folder/'resource_interval_summary.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evidence',type=Path,required=True)
    a=p.parse_args();print(json.dumps(analyze(a.evidence),indent=2))
