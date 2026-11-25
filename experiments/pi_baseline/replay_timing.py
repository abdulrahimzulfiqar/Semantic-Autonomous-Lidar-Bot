"""Pure helpers for declared-rate SE(2) scan/odometry replay diagnostics."""
import math
import numpy as np


def validate_recording(data):
    stamps = np.asarray(data['stamps'])
    if stamps.ndim != 1 or len(stamps) < 2 or not np.all(np.isfinite(stamps)) or np.any(np.diff(stamps) <= 0):
        raise ValueError('recording_stamps_not_strictly_increasing')
    for key in ['odom', 'extrinsics']:
        if data[key].shape != (len(stamps), 3) or not np.all(np.isfinite(data[key])):
            raise ValueError('invalid_recording_' + key)
    if data['ranges'].shape != (len(stamps), len(data['angles'])):
        raise ValueError('invalid_scan_geometry')
    if not np.allclose(data['extrinsics'], data['extrinsics'][0], atol=1e-8, rtol=0):
        raise ValueError('changing_recorded_extrinsic')
    angles = data['angles']
    if len(angles) < 2 or not np.all(np.isfinite(angles)) or not np.allclose(np.diff(angles), angles[1]-angles[0]):
        raise ValueError('invalid_scan_angles')
    if not np.all(np.isfinite(data['initial_pose'])) or len(data['initial_pose']) != 3:
        raise ValueError('invalid_initial_pose')


def interpolated_pose(stamps, poses, stamp):
    """Interpolate within recorded SE(2); reject extrapolation."""
    if not math.isfinite(stamp) or stamp < stamps[0] or stamp > stamps[-1]:
        raise ValueError('replay_extrapolation')
    right = min(max(int(np.searchsorted(stamps, stamp, side='right')), 1), len(stamps)-1)
    left = right-1
    alpha = (stamp-stamps[left])/(stamps[right]-stamps[left])
    a, b = poses[left], poses[right]
    turn = math.atan2(math.sin(b[2]-a[2]), math.cos(b[2]-a[2]))
    return [float((1-alpha)*a[0]+alpha*b[0]), float((1-alpha)*a[1]+alpha*b[1]), float(a[2]+alpha*turn)]


def timing_summary(feed, observation):
    """These are input/timing gates, never a localization-accuracy score."""
    begin, end = feed['begin_wall'], feed['end_wall']
    rows = [r for r in observation['tf'] if begin <= r['wall'] <= end]
    scans = [r for r in observation['scans'] if begin <= r['wall'] <= end+.5]
    clock = [r for r in observation['clocks'] if begin <= r['wall'] <= end]
    valid = [r for r in rows if 'stamp' in r]
    increasing = []
    for r in valid:
        if not increasing or r['stamp'] > increasing[-1]['stamp']:
            increasing.append(r)
    gaps = [b['stamp']-a['stamp'] for a, b in zip(increasing, increasing[1:])]
    stamps = {r['stamp_ns'] for r in scans}
    expected = set(feed['scan_stamps_ns'])
    delays = [r['late_wall_s'] for r in feed['scan_publications']]
    ages = [r['clock']-r['stamp'] for r in valid]
    clock_gaps = [b['wall']-a['wall'] for a, b in zip(clock, clock[1:])]
    clock_stamps = [r['stamp'] for r in clock]
    clock_covers_recording = bool(clock_stamps and expected and
        clock_stamps[0] <= min(expected)/1e9+.15 and
        clock_stamps[-1] >= max(expected)/1e9-.15 and
        all(b >= a for a, b in zip(clock_stamps, clock_stamps[1:])))
    result = dict(scope='Local AMCL/TF replay timing only; no EKF, controller, mission or localization-accuracy claim',
        replay_rate=feed['rate'],recording_duration_sim_s=feed['duration_sim_s'],
        measured_replay_wall_s=end-begin,expected_scans=len(expected),received_unique_scans=len(expected & stamps),
        tf_queries=len(rows),tf_query_failures=len(rows)-len(valid),
        max_pose_observation_gap_sim_s=max(gaps, default=None),
        max_tf_age_sim_s=max(ages, default=None),max_scan_publication_lateness_wall_s=max(delays, default=None),
        max_clock_receipt_gap_wall_s=max(clock_gaps, default=None),
        clock_covers_recording=clock_covers_recording,
        thresholds=dict(scan_publication_lateness_wall_s=.1,tf_age_sim_s=.15,pose_observation_gap_sim_s=.15))
    result['passed'] = bool(rows and len(rows)==len(valid) and stamps >= expected and gaps and
        max(gaps)<=.15+1e-9 and max(ages)<=.15+1e-9 and min(ages)>=-1e-9 and
        max(delays)<=.1 and clock_covers_recording and clock_gaps and
        max(clock_gaps)<=.15/feed['rate'] and
        abs((end-begin)*feed['rate']-feed['duration_sim_s'])<=.1)
    return result


def clock_path_summary(feed, observation):
    """Locate source updates inside the largest observed receipt gap."""
    source=feed.get('clock_publications',[])
    received=[v for v in observation['clocks'] if feed['begin_wall']<=v['wall']<=feed['end_wall']]
    if len(source)<2 or len(received)<2:return None
    gaps=[b['wall']-a['wall'] for a,b in zip(received,received[1:])]
    index=int(np.argmax(gaps));left,right=received[index],received[index+1]
    during=[v for v in source if left['wall']<v['wall']<right['wall']]
    emitted={v['stamp_ns'] for v in source}
    seen={round(v['stamp']*1e9) for v in observation['clocks'] if v['wall']>=feed['begin_wall']}
    return dict(source_publications=len(source),matched_receipt_stamps=len(emitted & seen),
        max_source_publication_return_gap_wall_s=max(b['wall']-a['wall'] for a,b in zip(source,source[1:])),
        largest_receipt_gap_wall_s=gaps[index],gap_start=left,gap_end=right,
        source_publications_returned_during_receipt_gap=len(during),
        source_stamps_during_gap_ns=[v['stamp_ns'] for v in during],
        scope='Same-host monotonic clock, publication-call return versus subscriber callback. Does not isolate middleware from callback scheduling or establish wire latency.')
