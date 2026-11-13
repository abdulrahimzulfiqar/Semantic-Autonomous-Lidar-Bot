#!/usr/bin/env python3
"""Replay the cancel-topic test from frozen raw evidence, not its pass flag."""
import argparse,json
from pathlib import Path
import numpy as np
from gcu_cancel_metrics import score_cancellation
from gcu_plan_validation import validate_path
from osmag_constraint_mask import build_mask
from osmag_mission_contract import make_contract,sha
from verify_gcu_segmented_crossing import verify


def score_cancel_run(report,arrays,planner,stop_scorer=score_cancellation):
    stored=report['contract'];fixture=report['fixture'];snapshot=report['snapshot']
    contract=make_contract(planner,stored['start'],stored['goal'][:2],stored['goal'][2],mission_id=stored['mission_id'],
        now=stored['created_at'],pose_stamp=stored['pose_stamp'],route_revision=stored['route_revision'],closure_revision=stored['closure_revision'])
    assert contract.map_sha256==stored['map_sha256']
    assert contract.passage_ids==tuple(stored['passage_ids'])==tuple(fixture['expected_passages'])
    assert contract.area_ids==tuple(stored['area_ids'])==tuple(fixture['expected_areas'])
    assert np.allclose([s.point for s in contract.segments],[s['point'] for s in stored['segments']],rtol=0,atol=1e-6)
    assert sha({'path':stored['path'],'areas':stored['area_ids'],'passages':stored['passage_ids'],'segments':stored['segments']})==stored['route_sha256']
    incidence={n:(v['osmAG:from'],v['osmAG:to']) for n,v in planner.passages.items()}
    mask=build_mask(planner.grid,planner.resolution,planner.origin,planner.leaves,planner.passage_segments,incidence,
        allowed_areas=stored['area_ids'],open_passages=stored['passage_ids'],disabled_passages=stored['disabled_passages'],robot_pose=stored['start'],
        mission_id=stored['mission_id'],route_revision=stored['route_revision'],mask_revision=snapshot['mask_revision'])
    assert np.array_equal(arrays['physical'],planner.grid) and np.array_equal(arrays['mask'],mask.mask) and np.array_equal(arrays['combined'],mask.combined)
    assert mask.report()['mask_sha256']==snapshot['mask_sha256']
    assert validate_path(mask.combined,planner.resolution,planner.origin,stored['path'],.20)['valid']
    assert validate_path(mask.combined,planner.resolution,planner.origin,[v['truth'][:2] for v in report['truth']],.18)['valid']
    assert len(report['preflight'])==len(contract.segments)
    for v in report['preflight'].values():assert v['status']==4 and validate_path(mask.combined,planner.resolution,planner.origin,v['points'],.20)['valid']
    assert not report['contacts'] and len(report['contact_publishers'])==13 and all(v==1 for v in report['contact_publishers'].values())
    assert report['ownership']=={'cmd_vel':['controller_server'],'navigate_status':1,'direct_goal_publishers':0,'old_bridge_subscribers':0}
    assert np.percentile([r['position_error_m'] for r in report['localization']],95)<=.25
    assert np.percentile([r['yaw_error_deg'] for r in report['localization']],95)<=5
    assert max(abs(r['v']) for r in report['commands'])<=.10001
    assert all(0<=r['clock']-r['query_stamp']<=.15+1e-9 and r['query_stamp']<=r['latest_stamp'] for r in report['tf_timing'])
    return dict(verification_passed=True,**stop_scorer(report))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evidence',type=Path,required=True);p.add_argument('--trial',default='01');a=p.parse_args()
    result=verify(a.evidence,a.trial,'cancel',mission_scorer=score_cancel_run)
    name='historical_verification_replay.json' if result['historical_source_files'] else 'verification.json'
    (a.evidence/name).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
