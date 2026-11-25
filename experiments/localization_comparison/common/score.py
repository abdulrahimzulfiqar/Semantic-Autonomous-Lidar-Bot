#!/usr/bin/env python3
"""Evaluator-only paired scoring. No alignment fitted to ground truth."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np


def metrics(poses,truth,stamps):
    p=np.asarray(poses,dtype=float);truth=np.asarray(truth,dtype=float)
    if p.shape!=truth.shape or p.ndim!=2 or p.shape[1]!=3 or not np.isfinite(p).all() or not np.isfinite(truth).all():raise ValueError('invalid_paired_poses')
    error=p[:,:2]-truth[:,:2];distance=np.linalg.norm(error,axis=1)
    yaw=np.arctan2(np.sin(p[:,2]-truth[:,2]),np.cos(p[:,2]-truth[:,2]))
    return dict(frames=len(p),position_rmse_m=float(np.sqrt(np.mean(distance**2))),position_p95_m=float(np.percentile(distance,95)),
        position_max_m=float(distance.max()),heading_rmse_deg=float(np.degrees(np.sqrt(np.mean(yaw**2)))),
        heading_p95_deg=float(np.degrees(np.percentile(np.abs(yaw),95))),
        fraction_over_025m_or_5deg=float(np.mean((distance>.25)|(np.abs(yaw)>math.radians(5)))),
        final_position_error_m=float(distance[-1]),max_observation_gap_sim_s=float(np.max(np.diff(stamps))))


def score(capture,estimate):
    capture=Path(capture);estimate=Path(estimate);manifest=json.loads((capture/'capture.json').read_text())
    for name,digest in manifest['files_sha256'].items():
        if hashlib.sha256((capture/name).read_bytes()).hexdigest()!=digest:raise ValueError('capture_input_changed:'+name)
    reference=np.load(capture/'evaluation.npz');observations=np.load(capture/'observations.npz');candidate=json.loads(estimate.read_text())
    stamps=observations['stamps'];rows=candidate['poses']
    if not any(Path(name).name=='observations.npz' and digest==manifest['files_sha256']['observations.npz']
               for name,digest in candidate['input_sha256'].items()):raise ValueError('candidate_observation_provenance_differs')
    if not np.array_equal(reference['stamps'],stamps) or not np.array_equal([r['stamp'] for r in rows],stamps):raise ValueError('paired_timestamps_differ')
    if len(stamps)<2 or np.any(np.diff(stamps)<=0):raise ValueError('nonmonotonic_observations')
    # No dropping estimator failures, weak frames or large errors from scoring.
    a=metrics(reference['amcl'],reference['truth'],stamps);b=metrics([r['pose'] for r in rows],reference['truth'],stamps)
    b.update(accepted_fraction=sum(r['accepted'] for r in rows)/len(rows),lost_frames=sum(r['lost'] for r in rows),
        weak_geometry_frames=sum(r['reason']=='weak_geometry' for r in rows),rejected_frames=sum(not r['accepted'] for r in rows),
        update_wall_p50_ms=float(1000*np.median([r['wall_s'] for r in rows])),update_wall_p95_ms=float(1000*np.percentile([r['wall_s'] for r in rows],95)))
    # Independent odometry-only diagnostic, with the same declared start prior.
    # No ICP and no use of truth to fit this transform.
    odom=observations['odom'];seed=observations['initial_pose'];yaw0=seed[2]-odom[0,2]
    c,s=math.cos(yaw0),math.sin(yaw0)
    xy=(odom[:,:2]-odom[0,:2])@np.array([[c,s],[-s,c]])+seed[:2]
    yaw=np.arctan2(np.sin(odom[:,2]+yaw0),np.cos(odom[:,2]+yaw0))
    odometry=metrics(np.column_stack([xy,yaw]),reference['truth'],stamps)
    return dict(scope='Open-loop shared-input tracking comparison, AMCL-controlled trajectory; not closed-loop candidate navigation, global recovery or Pi performance',
        amcl=a,structural_2d=b,odometry_only_diagnostic=odometry,raw_amcl_measurement_messages=len(reference['amcl_measurement_stamps']),
        initial_pose=observations['initial_pose'].tolist(),received_scans=manifest['received_scans'],paired_scans=len(stamps),
        paired_fraction=len(stamps)/manifest['received_scans'],source_inputs_sha256=manifest['files_sha256'],
        candidate_sha256=hashlib.sha256(estimate.read_bytes()).hexdigest(),truth_alignment='None; fixed shared map frame',
        candidate_tracking_gate=bool(b['position_p95_m']<=.25 and b['heading_p95_deg']<=5 and not b['lost_frames']),
        decision='Measure, do not assume a winner; closed-loop and recovery gates remain separate')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--capture',type=Path,required=True);p.add_argument('--estimate',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=score(a.capture,a.estimate)
    a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');print(json.dumps(result,indent=2))
