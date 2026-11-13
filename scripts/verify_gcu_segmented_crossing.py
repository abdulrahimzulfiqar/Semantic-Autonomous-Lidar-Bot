#!/usr/bin/env python3
"""Replay physical, topology, handoff and rest evidence for the short crossing."""
import argparse,hashlib,io,json,math,tarfile
from dataclasses import replace
from pathlib import Path
import numpy as np
from gcu_plan_validation import validate_path
from gcu_navigation_metrics import rest_window,coverage,velocities
from osmag_paper_planner import PaperPlanner
from osmag_mission_contract import make_contract,CrossingMonitor,sha


def handoff_rest_report(truth,handoff_times):
    """Measure each handoff separately; a clean first handoff proves no later one."""
    measured=velocities(truth);report=[]
    for at in handoff_times:
        slow_start=None;previous=None;longest=0.
        for row in measured:
            t=row['sim_time']
            if not at-.5<=t<=at+2:continue
            if row['v']<=.01 and row['w']<=.02:
                if slow_start is None or (previous is not None and t-previous>.15):slow_start=t
                longest=max(longest,t-slow_start)
            else:slow_start=None
            previous=t
        report.append({'next_goal_sim':at,'longest_rest_sim_s':longest})
    return report


def verify(folder,trial='06',case_prefix='crossing',mission_scorer=None):
    manifest=json.loads((folder/'manifest.json').read_text());root=Path(__file__).resolve().parents[1]
    for n,digest in manifest['artifacts_sha256'].items():assert hashlib.sha256((folder/n).read_bytes()).hexdigest()==digest,n
    archived_sources=[]
    for n,digest in manifest['source_sha256'].items():
        if hashlib.sha256((root/n).read_bytes()).hexdigest()!=digest:
            with tarfile.open(folder/'source_snapshot.tar.gz') as snapshot:
                assert hashlib.sha256(snapshot.extractfile(n).read()).hexdigest()==digest,n
            archived_sources.append(n)
    with tarfile.open(folder/('attempt'+trial+'.tar.gz')) as t:
        prefix='results/'+case_prefix+'_'+trial+'/'
        report=json.loads(t.extractfile(prefix+'report.json').read());run=json.loads(t.extractfile(prefix+'run.json').read())
        arrays=np.load(io.BytesIO(t.extractfile(prefix+'constraint.npz').read()))
        runtime_manifest='crossing_manifest_attempt'+trial+'.json'
        if runtime_manifest not in t.getnames():runtime_manifest='nominal_manifest_'+case_prefix+trial+'.json'
        if runtime_manifest not in t.getnames():runtime_manifest='crossing_manifest.json'
        if runtime_manifest in t.getnames():
            runtime=json.loads(t.extractfile(runtime_manifest).read())['files_sha256']
            for name,digest in manifest['source_sha256'].items():
                member=Path(name).name
                if member in t.getnames():
                    assert hashlib.sha256(t.extractfile(member).read()).hexdigest()==digest,name
                    assert runtime[member]==digest,name
        assert run['passed']==bool(report['passed'] and run['checker_exit']==0 and all(v==0 for v in run['process_exits']) and not run['audit']['critical'] and run['truth_shutdown']['passed'] and run['simulator_shutdown']['passed'])
    result={'verification_passed':False,'trial_passed':bool(report['passed'] and run['passed']),
        'motion_attempted':any(e['event']=='send' for e in report['events']),'historical_source_files':archived_sources}
    result['runtime_error']=report.get('error');result['shutdown_passed']=run['simulator_shutdown']['passed']
    if not result['trial_passed']:
        result['verification_passed']=True;result['scope']='Failed trial retained; no successful crossing/arrival claim.';return result
    assert run['process_exits']==[0]*5 and not run['audit']['critical'] and run['truth_shutdown']['passed']
    if (folder/'cleanup.json').exists():assert not json.loads((folder/'cleanup.json').read_text())['remaining_test_processes']
    result.update((mission_scorer or score_mission)(report,arrays,PaperPlanner(root/'simulation/gcu_ground_floor',execution_padding=.02 if report.get('fixture',{}).get('constraint_aware_route') else None)))
    return result


def score_mission(report,arrays,planner):
    """Replay one leg independently of ROS; session callers audit process continuity."""
    assert report['passed']
    result={}
    stored=report['contract']
    contract=make_contract(planner,stored['start'],stored['goal'][:2],stored['goal'][2],mission_id=stored['mission_id'],now=stored['created_at'],pose_stamp=stored['pose_stamp'],route_revision=stored['route_revision'],closure_revision=stored['closure_revision'])
    metric=report.get('execution_mode')=='metric_final_goal'
    if metric:
        from gcu_metric_baseline import metric_contract
        contract=metric_contract(contract)
    assert contract.map_sha256==stored['map_sha256']
    assert np.array_equal(arrays['physical'],planner.grid)
    snapshot=report['snapshot'];physical_sha=hashlib.sha256(np.ascontiguousarray(arrays['physical'],dtype=np.int16).tobytes()).hexdigest()
    assert physical_sha==snapshot['physical_sha256']
    metadata={k:snapshot[k] for k in ['resolution','origin','mission_id','route_revision','closure_revision','mask_revision','radius','padding']}
    metadata.update(allowed=snapshot['allowed_areas'],open=snapshot['open_passages'],disabled=snapshot['disabled_passages'])
    assert hashlib.sha256(arrays['mask'].tobytes()+json.dumps(metadata,sort_keys=True).encode()+physical_sha.encode()).hexdigest()==snapshot['mask_sha256']
    assert sha({'path':stored['path'],'areas':stored['area_ids'],'passages':stored['passage_ids'],'segments':stored['segments']})==stored['route_sha256']
    if 'current_contract_mask_validation' in report:
        assert validate_path(arrays['combined'],planner.resolution,planner.origin,stored['path'],snapshot['radius']+snapshot['padding'])['valid']
    # Replanning in a different pyproj/GEOS version may change last-bit portal
    # coordinates. Verify the stored digest exactly and compare geometry in m.
    expected=tuple(report['fixture']['expected_passages']) if report.get('fixture') else ('-93',)
    assert contract.passage_ids==tuple(stored['passage_ids'])==expected and contract.area_ids==tuple(stored['area_ids'])
    assert np.allclose([s.point for s in contract.segments],[s['point'] for s in stored['segments']],rtol=0,atol=1e-6)
    truth=report['truth'];monitor=CrossingMonitor(replace(contract,start=tuple(truth[0]['truth'][:2]),pose_stamp=truth[0]['sim_time']),planner.leaves,planner.passage_segments)
    for row in truth[1:]:monitor.observe(row['truth'][:2],row['sim_time'])
    assert monitor.confirmed==len(expected)
    assert validate_path(arrays['combined'],planner.resolution,planner.origin,[v['truth'][:2] for v in truth],.18)['valid']
    final=truth[-1]['truth'];position=math.dist(final[:2],stored['goal'][:2]);heading=math.degrees(abs(math.atan2(math.sin(final[2]-stored['goal'][2]),math.cos(final[2]-stored['goal'][2]))))
    assert position<=.20 and heading<=10 and not report['contacts']
    events=report['events'];sends=[e for e in events if e['event']=='send'];assert len(sends)==(1 if metric else len(expected)+1)
    for index in range(0 if metric else len(expected)):
        old=next(e['uuid'] for e in events if e['event']=='accepted' and e['token']==sends[index]['token'])
        end=next(e for e in events if e['event']=='result' and e['uuid']==old);assert sends[index+1]['wall']<end['wall']
    terminal=next(e for e in events if e['event']=='result' and e['outcome']=='succeeded');assert rest_window(truth,earliest=terminal['sim']) is not None
    cov=coverage(truth,report['localization']);assert cov['matched_fraction']>=.9 and cov['max_truth_gap_sim_s']<=.15
    assert np.percentile([r['position_error_m'] for r in report['localization']],95)<=.25
    assert np.percentile([r['yaw_error_deg'] for r in report['localization']],95)<=5
    assert max(abs(r['v']) for r in report['commands'])<=.10001
    assert report['ownership']=={'cmd_vel':['controller_server'],'navigate_status':1,'direct_goal_publishers':0,'old_bridge_subscribers':0}
    assert len(report['contact_publishers'])==13 and all(v==1 for v in report['contact_publishers'].values())
    if 'tf_timing' in report:
        assert all(0<=row['clock']-row['query_stamp']<=.15+1e-9 and row['query_stamp']<=row['latest_stamp'] for row in report['tf_timing'])
    handoff_rests=handoff_rest_report(truth,[s['sim'] for s in sends[1:]])
    longest=max((v['longest_rest_sim_s'] for v in handoff_rests),default=0.)
    if not report.get('fixture') or report['fixture'].get('check_handoff_rest',False):assert longest<1.
    result.update(verification_passed=True,position_error_m=position,heading_error_deg=heading,
        confirmed_passages=[e['passage_id'] for e in monitor.events],handoff_before_terminal=True,
        longest_handoff_rest_sim_s=longest,handoff_rest_checks=handoff_rests,coverage=cov)
    if metric:result.update(handoff_before_terminal=None,execution_mode='metric_final_goal',single_navigation_goal=True)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evidence',type=Path,required=True);p.add_argument('--trial',default='06');p.add_argument('--case-prefix',default='crossing');a=p.parse_args()
    result=verify(a.evidence,a.trial,a.case_prefix)
    name='historical_verification_replay.json' if result['historical_source_files'] else 'verification.json'
    (a.evidence/name).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
