#!/usr/bin/env python3
"""Summarize retained failed pilots; never substitute this for mission scoring."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
from gcu_navigation_metrics import rest_window


def summarize(folder):
    files = {
        'actor': folder / 'pi_results/actor/worker.json',
        'observer': folder / 'simulator_results/results/observer.json',
        'resources': folder / 'pi_results/resources.json',
        'orchestration': folder / 'orchestration.json',
    }
    data = {name: json.loads(path.read_text()) for name, path in files.items()}
    actor, observer = data['actor'], data['observer']
    failure = actor.get('failure_clock_sim')
    truth = observer['truth']
    motion = [row for row in truth if row['sim_time'] >= actor.get('dispatch_sim', math.inf)]
    samples = data['resources']['samples']
    temperatures = [float(row['temperature_millidegrees_c']) / 1000 for row in samples
                    if row.get('temperature_millidegrees_c')]
    pss = [sum(p['memory_kib']['Pss'] for p in row['processes']) for row in samples]
    poses = actor.get('poses', [])
    candidate_gaps = [b['sim_time'] - a['sim_time'] for a, b in zip(poses, poses[1:])]
    # A rejected observation is absent from poses; report the final TF candidate
    # separately rather than pretending it was admitted by the crossing monitor.
    candidates = [row['query_stamp'] for row in actor.get('tf_timing', [])
                  if poses and row['query_stamp'] is not None and row['query_stamp'] > poses[-1]['sim_time']]
    return {
        'attempt': folder.name,
        'mission_passed': bool(actor['passed'] and data['orchestration']['passed']),
        'error': actor.get('error'),
        'dispatch_sim': actor.get('dispatch_sim'),
        'failure_sim': failure,
        'preprocessing_wall_s': actor.get('preprocessing_wall_s'),
        'motion_net_displacement_m': math.dist(motion[0]['truth'][:2], motion[-1]['truth'][:2]) if motion else None,
        'post_failure_sampled_rest': rest_window(truth, earliest=failure) if failure is not None else None,
        'classified_contacts': len(observer['contacts']),
        'contact_publishers': observer['contact_publishers'],
        'largest_admitted_pose_gap_sim_s': max(candidate_gaps, default=None),
        'first_unadmitted_tf_gap_sim_s': candidates[0] - poses[-1]['sim_time'] if candidates else None,
        'resource_samples': len(samples),
        'sampled_whole_attempt_domain_pss_max_kib': max(pss, default=None),
        'sampled_whole_attempt_temperature_max_c': max(temperatures, default=None),
        'resource_scope': 'Domain-218 processes, entire attempt including startup/preprocessing; approximately 1 wall-second samples. Not a successful-motion budget or an unsampled peak.',
        'inputs_sha256': {name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in files.items()},
    }


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--attempts', nargs='+', default=['pilot_08', 'pilot_09'])
    parser.add_argument('--output',type=Path,help='Derived summary path; defaults to latest_pilot_summary.json in evidence directory')
    args = parser.parse_args()
    result = {
        'scope': 'Failed diagnostic pilots; no completed Phase 09 benchmark or Pi capacity claim',
        'attempts': [summarize(args.evidence / name) for name in args.attempts],
    }
    (args.output or args.evidence / 'latest_pilot_summary.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
