#!/usr/bin/env python3
"""Verify a complete automated review session and each successful raw mission."""
import argparse,hashlib,io,json,tarfile
from pathlib import Path
import numpy as np
from osmag_paper_planner import PaperPlanner
from gcu_navigation_metrics import rest_window
from verify_gcu_segmented_crossing import score_mission


def verify_review(folder,trial='05'):
    manifest=json.loads((folder/'manifest.json').read_text())
    for n,h in manifest['artifacts_sha256'].items():assert hashlib.sha256((folder/n).read_bytes()).hexdigest()==h,n
    prefix='results/review_'+trial+'/'
    with tarfile.open(folder/('attempt'+trial+'.tar.gz')) as t:
        run=json.loads(t.extractfile(prefix+'run.json').read());session=json.loads(t.extractfile(prefix+'report.json').read());driver=json.loads(t.extractfile(prefix+'interface_driver.json').read())
        assert run['passed'] and session['passed'] and driver['passed']
        assert run['scenario']=='interactive' and run['process_exits']==[0]*6 and not run['audit']['critical']
        assert run['truth_shutdown']['passed'] and run['simulator_shutdown']['passed']
        runtime=json.loads(t.extractfile('crossing_manifest.json').read())['files_sha256']
        for n,h in manifest['source_sha256'].items():
            name=Path(n).name
            assert hashlib.sha256(t.extractfile(name).read()).hexdigest()==runtime[name]==h,name
        assert not session['contacts'] and len(session['contact_publishers'])==13 and all(n==1 for n in session['contact_publishers'].values())
        assert session['final_state']['stop_confirmed'] and session['final_state']['unresolved_requests']==0
        required={'no_automatic_goal_before_click','wrong_frame_outside_and_courtyard_goals_rejected_without_dispatch','occupied_goal_rejected_without_dispatch','busy_goal_rejected','cancel_topic_stops_active_mission','recover_after_cancel_and_arrive','consecutive_return_with_reversed_heading','inactive_navigator_does_not_start_motion','recover_after_navigator_reactivation','all_passages_closed_rejects_goal_without_dispatch'}
        assert required<=set(driver['checks'])
        planner=PaperPlanner(Path(__file__).resolve().parents[1]/'simulation/gcu_ground_floor',execution_padding=.02)
        missions=[]
        for item in session['missions']:
            r=json.loads(t.extractfile(prefix+item['file']).read())
            if item['outcome']=='succeeded':
                arrays=np.load(io.BytesIO(t.extractfile(prefix+r['constraint_file']).read()))
                missions.append({'file':item['file'],**score_mission(r,arrays,planner)})
            elif item['outcome']=='cancelled':
                assert r['state']['stop_confirmed'] and r['state']['unresolved_requests']==0 and r['state']['reason']=='user_cancel'
                fault=next(e for e in r['events'] if e['event']=='fault' and e['reason']=='user_cancel')
                assert not any(e['event']=='send' and e['wall']>fault['wall'] for e in r['events'])
                recovery=next(e for e in session['events'] if e['event']=='recovered_after_confirmed_stop' and e['wall']>fault['wall'])
                next_send=next(e for e in session['events'] if e['event']=='send' and e['wall']>recovery['wall'])
                stopped=rest_window([v for v in session['truth'] if fault['sim']<=v['sim_time']<=next_send['sim']],earliest=fault['sim'])
                assert stopped and stopped['start_sim']-fault['sim']<=1.
            elif item['outcome']=='failed':
                assert item['reason']=='navigation_action_rejected' and r['state']['stop_confirmed'] and r['state']['unresolved_requests']==0
                assert any(e['event']=='accepted' and not e['accepted'] for e in r['events'])
                assert all(abs(c['v'])+abs(c['w'])<=1e-5 for c in r['commands'])
            else:raise AssertionError('Unexpected review mission outcome')
        assert len(missions)==3 and sum(m['outcome']=='cancelled' for m in session['missions'])==1
        assert any(r['reason']=='no_feasible_graph_route' for r in session['rejections'])
        closed=next(e for e in session['events'] if e['event']=='closure_received')
        assert not any(e['event']=='send' and e['wall']>closed['wall'] for e in session['events'])
    assert not json.loads((folder/'cleanup.json').read_text())['remaining_test_processes']
    return {'verification_passed':True,'trial_passed':True,'scope':'Automated interface and motion acceptance; human visual review remains pending',
        'interface_checks':driver['checks'],'verified_arrivals':missions,'native_rejection_without_motion':True,'cancel_recovery_verified':True,'clean_native_exits':6}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evidence',type=Path,required=True);p.add_argument('--trial',default='05');a=p.parse_args()
    result=verify_review(a.evidence,a.trial);(a.evidence/'verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
