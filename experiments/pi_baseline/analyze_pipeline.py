#!/usr/bin/env python3
"""Compare phase 09 input, producer and actor timing around a navigation fault."""
import argparse
import hashlib
import json
from pathlib import Path


def first_receipts(rows):
    receipts = {}
    for row in sorted(rows, key=lambda item: item['wall']):
        receipts.setdefault(row['stamp_ns'], row)
    return receipts


def maximum_gap(values):
    return max((b-a for a, b in zip(values, values[1:])), default=None)


def timing(rows):
    wall_times = sorted(row['wall'] for row in rows)
    stamps = sorted({row['stamp_ns'] for row in rows})
    return dict(callbacks=len(rows), unique_stamps=len(stamps),
                max_sample_gap_ns=maximum_gap(stamps),
                max_callback_gap_s=maximum_gap(wall_times))


def compare_hosts(source_rows, pi_rows, fault_wall=None):
    source = first_receipts(source_rows)
    received = first_receipts(pi_rows)
    matched = source.keys() & received.keys()
    result = dict(simulator=timing(source_rows), pi=timing(pi_rows), matched_stamps=len(matched),
                  missing_on_pi_ns=sorted(source.keys()-received.keys()),
                  absent_from_source_probe_ns=sorted(received.keys()-source.keys()))
    if fault_wall is not None:
        result['matched_after_pi_fault_ns'] = sorted(stamp for stamp in matched if received[stamp]['wall']>fault_wall)
        result['latest_pi_stamp_before_fault_ns'] = max(
            (stamp for stamp in matched if received[stamp]['wall']<=fault_wall), default=None)
    return result


def compare_pi(rows, actor_rows, fault_wall):
    independent = first_receipts(rows)
    actor = first_receipts(actor_rows)
    shared = independent.keys() & actor.keys()
    # Only local Pi publishers belong here. Source DDS timestamps and the
    # probe's wall clock share a host, while the probe/actor offsets use
    # monotonic time. A zero DDS stamp means metadata was unavailable.
    native_delays = [row['unix_ns']-row['dds_source_ns'] for row in rows
                     if 0 < row.get('dds_source_ns', 0) <= row.get('unix_ns', 0)]
    native_source_times = sorted({row['dds_source_ns'] for row in rows if row.get('dds_source_ns', 0)>0})
    probe_before_fault = [row for row in rows if fault_wall is not None and row['wall']<=fault_wall]
    actor_before_fault = [row for row in actor_rows if fault_wall is not None and row['wall']<=fault_wall]
    return dict(probe=timing(rows), actor=timing(actor_rows), matched_stamps=len(shared),
                native_dds_source_max_gap_ns=maximum_gap(native_source_times),
                native_source_to_probe_samples=len(native_delays),
                native_source_to_probe_max_ns=max(native_delays, default=None),
                probe_to_actor_max_s=max((actor[s]['wall']-independent[s]['wall'] for s in shared), default=None),
                latest_probe_stamp_before_fault_ns=max((r['stamp_ns'] for r in probe_before_fault), default=None),
                latest_actor_stamp_before_fault_ns=max((r['stamp_ns'] for r in actor_before_fault), default=None))


def analyze(root):
    digests = {}

    def read(relative):
        data = (root/relative).read_bytes()
        digests[relative] = hashlib.sha256(data).hexdigest()
        return json.loads(data)

    producer = read('simulator_results/results/pipeline_arrival.json')
    independent = read('pi_results/arrival_probe.json')
    callback = read('pi_results/actor/callback_trace.json')['rows']
    worker = read('pi_results/actor/worker.json')
    fault = next((e for e in worker['events'] if e['event']=='fault'), None)
    end = fault['sim'] if fault else worker.get('failure_clock_sim', worker['final_sim'])
    start_ns, end_ns = round((end-2)*1e9), round(end*1e9)

    def bounded(rows):
        return [row for row in rows if start_ns<=row['stamp_ns']<=end_ns]

    sources, probes = producer['rows'], independent['rows']
    fault_wall = fault['wall'] if fault else None
    transport = {kind:compare_hosts(bounded(sources.get(kind, [])), bounded(probes.get(kind, [])), fault_wall)
                 for kind in ('wheel_odom','imu_input')}
    actor_grids = {kind:[dict(stamp_ns=round(row['stamp']*1e9), wall=row['begin_wall'])
                         for row in callback.get(kind, []) if 'stamp' in row] for kind in ('grid_global','grid_local')}
    native = {kind:compare_pi(bounded(probes.get(kind, [])), bounded(rows), fault_wall)
              for kind, rows in actor_grids.items()}
    native['filtered_odom'] = compare_pi(bounded(probes.get('filtered_odom', [])), [], fault_wall)
    for parent, child in [('map','odom'),('odom','base_footprint')]:
        tf = [row for row in probes.get('tf', []) if (row['parent'], row['child'])==(parent,child)]
        actor = [dict(stamp_ns=round(stamp*1e9), wall=row['begin_wall'])
                 for row in callback.get('tf_ingest', []) for a,b,stamp in row.get('stamps', [])
                 if (a,b)==(parent,child)]
        native[parent+'->'+child] = compare_pi(bounded(tf), bounded(actor), fault_wall)
    return dict(scope='Bounded diagnostic based on observed ROS stamps and local callback times. Cross-host data compare stamp coverage without inferring one-way delay.',
                first_stamp_ns=start_ns, last_stamp_ns=end_ns, actor_error=worker.get('error'),
                motion_started=any(row['event']=='send' for row in worker['events']),
                cross_host=transport, local_pi=native, inputs_sha256=digests,
                analyzer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', type=Path, required=True)
    args = parser.parse_args()
    report = analyze(args.evidence)
    (args.evidence/'pipeline_timing_analysis.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
