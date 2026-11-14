#!/usr/bin/env python3
"""Independently replay actual windowed motion, admission, footprint and shutdown."""
import argparse
import hashlib
import io
import json
import math
from pathlib import Path
import tarfile
import numpy as np
import yaml

from gcu_plan_validation import validate_path
from gcu_window_loss_metrics import score_window_loss
from osmag_paper_planner import PaperPlanner
from verify_gcu_segmented_crossing import score_mission


def verify(folder,trial='02',case_prefix='rolling_nominal',report_file='report.json',results_prefix=None):
    root=Path(__file__).resolve().parents[1]
    with tarfile.open(folder/('attempt'+trial+'.tar.gz')) as t:
        read=lambda p:json.loads(t.extractfile(p).read())
        prefix='results/'+(results_prefix or case_prefix+'_'+trial)+'/'
        run=read(prefix+'run.json');report=read(prefix+report_file)
        if report_file=='report.json' and report.get('source_loss_and_recovery'):
            pair=report['source_loss_and_recovery']
            fault=verify(folder,trial,case_prefix,pair['source_loss_file'],results_prefix)
            recovery=verify(folder,trial,case_prefix,pair['recovery_file'],results_prefix)
            assert run['passed'] and report['passed'] and fault['verification_passed'] and recovery['verification_passed']
            fault_raw=read(prefix+pair['source_loss_file']);recovery_raw=read(prefix+pair['recovery_file'])
            restored=next(e for e in report['events'] if e['event']=='recovered_after_confirmed_stop')
            assert not any(e['event']=='send' and fault_raw['window_loss_request']['wall']<e['wall']<restored['wall'] for e in report['events'])
            first_new=next(e for e in recovery_raw['events'] if e['event']=='send')
            first_old=next(e for e in fault_raw['events'] if e['event']=='send')
            assert first_new['wall']>=restored['wall'] and int(first_new['token'].split(':')[0])>int(first_old['token'].split(':')[0])
            old_pids={p['role']:p['pid'] for p in fault_raw['resource_at_dispatch']['processes']}
            new_pids={p['role']:p['pid'] for p in recovery_raw['resource_at_stop']['processes']}
            assert old_pids==new_pids,'native process replacement during recovery'
            return dict(verification_passed=True,source_loss_and_recovery=True,source_loss=fault,recovery=recovery,
                same_native_processes=True,scope='Physical stop, source restoration and guarded same-session recovery')
        # Older rolling launchers labelled interactive runs "rolling_nominal".
        # Identify the recorded session structure, not that historical label.
        session=report if report_file=='report.json' else read(prefix+'report.json')
        interactive='seed_fixture' in session and isinstance(session.get('missions'),list)
        if interactive:
            assert report_file!='report.json' and session['passed'] and session['final_state']['stop_confirmed']
            assert any(m['file']==report_file and m['outcome']=='succeeded' for m in session['missions'])
        assert run['passed'] and report['passed'] and run['process_exits']==[0]*(6 if interactive else 5)
        assert not run['audit']['critical'] and run['truth_shutdown']['passed'] and run['simulator_shutdown']['passed']
        runtime=read('crossing_manifest.json')['files_sha256']
        for name,want in runtime.items():
            if name.endswith(('.py','.yaml','.osm','.sdf','.urdf','.json')):
                assert hashlib.sha256(t.extractfile(name).read()).hexdigest()==want,name
        params=yaml.safe_load(t.extractfile('assets/nav2_sensor_constraints.yaml').read())
        for name in ['global_costmap','local_costmap']:
            assert params[name][name]['ros__parameters']['static_layer']['map_topic']=='/osmag/planning_window'
        assert not report['contacts'] and len(report['contact_publishers'])==13 and all(v==1 for v in report['contact_publishers'].values())
        arrays=np.load(io.BytesIO(t.extractfile(prefix+report.get('constraint_file','constraint.npz')).read()))
        source_loss=report.get('expected_window_source_loss',False)
        if source_loss:
            result=score_window_loss(report)
            assert np.percentile([v['position_error_m'] for v in report['localization']],95)<=.25
            assert np.percentile([v['yaw_error_deg'] for v in report['localization']],95)<=5
            assert validate_path(arrays['combined'],report['snapshot']['resolution'],report['snapshot']['origin'],[r['truth'][:2] for r in report['truth']],.18)['valid']
        else:
            result=score_mission(report,arrays,PaperPlanner(root/'simulation/gcu_ground_floor',execution_padding=.02),allow_proxy_repeats=True)
            assert not any('costmap' in i['name'] for i in report['planning_map_subscribers'])
        manager=report['rolling'];windows=manager['windows'];by_revision={}
        publications=manager.get('source_publications',[])
        if publications:
            assert all(b['revision']>=a['revision'] and b['sim']>=a['sim'] for a,b in zip(publications,publications[1:]))
        if source_loss:
            failed=manager['readiness_failures'][0];old=failed['states']['committed']
            assert old['source_age']>.5 and set(old['observations'])=={'global','local'} and not old['errors']
            assert all(0<=failed['sim']-v['stamp']<=.5 for v in old['observations'].values())
        assert windows and all(w['window']['shape']==[128,128] for w in windows)
        short=report.get('fixture',{}).get('allow_short_window_mission',False)
        if not source_loss and not short:assert len(windows)>=3
        for index,w in enumerate(windows):
            snap=w['snapshot'];win=w['window'];by_revision[win['revision']]=w
            data=np.load(io.BytesIO(t.extractfile(prefix+w['file']).read()))
            assert data['physical'].shape==data['mask'].shape==data['global'].shape==(128,128)
            physical_hash=hashlib.sha256(np.ascontiguousarray(data['physical'],dtype=np.int16).tobytes()).hexdigest()
            assert physical_hash==snap['physical_sha256']
            meta={k:snap[k] for k in ['resolution','origin','mission_id','route_revision','closure_revision','mask_revision','radius','padding']}
            meta.update(allowed=snap['allowed_areas'],open=snap['open_passages'],disabled=snap['disabled_passages'])
            assert hashlib.sha256(data['mask'].tobytes()+json.dumps(meta,sort_keys=True).encode()+physical_hash.encode()).hexdigest()==snap['mask_sha256']
            for key,m in w['native'].items():
                assert m['frame']=='map';rows,cols=np.mgrid[:data[key].shape[0],:data[key].shape[1]]
                sr=np.floor((m['origin'][1]+(rows+.5)*m['resolution']-win['origin'][1])/win['resolution']).astype(int)
                sc=np.floor((m['origin'][0]+(cols+.5)*m['resolution']-win['origin'][0])/win['resolution']).astype(int)
                inside=(sr>=0)&(sr<128)&(sc>=0)&(sc<128)
                blocked=np.zeros_like(inside);blocked[inside]=data['mask'][sr[inside],sc[inside]]==100
                assert not np.any(blocked&(data[key]>=0)&(data[key]<100))
            for check in w['preflights'].values():
                assert check['status']==4 and validate_path(data['combined'],win['resolution'],win['origin'],check['points'],.20)['valid']
            end=windows[index+1]['committed_sim'] if index+1<len(windows) else math.inf
            truth=[p['truth'][:2] for p in report['truth'] if w['published_sim']<=p['sim_time']<=end]
            if truth:assert validate_path(data['combined'],win['resolution'],win['origin'],truth,.18)['valid']
            if w['common_native_plan']:
                assert validate_path(data['combined'],win['resolution'],win['origin'],w['common_native_plan'],.20)['valid']
                previous=windows[index-1];old=np.load(io.BytesIO(t.extractfile(prefix+previous['file']).read()))
                assert validate_path(old['combined'],previous['window']['resolution'],previous['window']['origin'],w['common_native_plan'],.20)['valid']
        admitted=[e for e in manager['events'] if e['event']=='goal_admitted']
        sends=[e for e in report['events'] if e['event']=='send']
        assert len(admitted)==len(sends)
        if not short:assert any(e['proxy'] for e in sends)
        for event,send in zip(admitted,sends):
            goal=event['goal'];receipt=event['receipt'];w=by_revision[goal['window_revision']]
            assert goal['window_sha256']==receipt['window_sha256']==w['window']['sha256']
            assert 0<=receipt['observed_at']-receipt['source_last_publication']<=.5
            assert send['window_revision']==goal['window_revision'] and send['proxy']==goal['proxy']
            assert np.allclose(send['point'],goal['point'],atol=1e-8,rtol=0)
        if report.get('fixture',{}).get('physical_obstacle'):
            from gcu_physical_obstacle import overlay_box
            evaluation=overlay_box(arrays['combined'],report['snapshot']['resolution'],report['snapshot']['origin'],report['fixture']['physical_obstacle'])
            assert validate_path(evaluation,report['snapshot']['resolution'],report['snapshot']['origin'],[v['truth'][:2] for v in report['truth']],.18)['valid']
            result['physical_obstacle_sweep_passed']=True
    return result | dict(verification_passed=True,physical_motion=True,window_count=len(windows),
        proxy_goals=sum(e['proxy'] for e in sends),native_global_shape=[128,128],
        source_loss=source_loss,scope='Bounded actual-physics windowed mission; static route preprocessing/AMCL retained')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evidence',type=Path,required=True)
    p.add_argument('--trial',default='02');p.add_argument('--case-prefix',default='rolling_nominal')
    p.add_argument('--results-prefix',help='Exact results directory inside the frozen workspace')
    p.add_argument('--report-file',default='report.json',help='Individual mission report for interactive evidence')
    a=p.parse_args()
    result=verify(a.evidence,a.trial,a.case_prefix,a.report_file,a.results_prefix)
    (a.evidence/'verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
