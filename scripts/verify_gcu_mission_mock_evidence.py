#!/usr/bin/env python3
"""Verify archived 06B1 ROS events/source without starting ROS or a VM."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import tarfile


def sha(payload):return hashlib.sha256(payload).hexdigest()


def verify(evidence):
    root=Path(__file__).resolve().parents[1]
    manifest=json.loads((evidence/'manifest.json').read_text())
    assert sha((evidence/manifest['archive']).read_bytes())==manifest['sha256']
    with tarfile.open(evidence/manifest['archive']) as archive:
        assert set(archive.getnames())==set(manifest['members'])
        def read(name):return archive.extractfile(name).read()
        def load(name):return json.loads(read(name))
        for name,digest in manifest['members'].items():assert sha(read(name))==digest,name
        initial=load('attempt_01/results/attempt_01/run.json')
        assert not initial['passed'] and initial['checker_exit']==1
        assert initial==json.loads((evidence/'initial_run.json').read_text())
        failed=load('attempt_01/results/attempt_01/report.json')
        assert failed['error']=="RuntimeError('mock_check_timeout')"
        assert failed['statuses'][-1]['state']=='cancelling'
        assert failed['statuses'][-1]['reason']=='navigation_action_rejected'
        run=load('attempt_02/results/attempt_02/run.json')
        report=load('attempt_02/results/attempt_02/report.json')
        assert run==json.loads((evidence/'run.json').read_text())
        assert report==json.loads((evidence/'report.json').read_text())
        assert run['passed'] and run['child_exits']==[0,0] and run['inputs_unchanged']
        assert not run['child_audit']['critical'] and run['elapsed_wall_s']<180
        matched=[]
        for attempt in ['attempt_01','attempt_02']:
            frozen=load(attempt+'/results/'+attempt+'/run.json')['source_sha256']
            for name,digest in frozen.items():
                assert sha(read(attempt+'/'+name))==digest,name
                if attempt=='attempt_02':
                    path=(root/'simulation/gcu_ground_floor'/name.removeprefix('assets/')
                          if name.startswith('assets/') else root/'scripts'/name)
                    if name!='manifest.json':
                        assert path.is_file() and sha(path.read_bytes())==digest,name
                        matched.append(str(path.relative_to(root)))
        assert report['passed'] and len(report['cases'])==10
        assert all(c['passed'] for c in report['cases'])
        assert not report['gazebo_started'] and not report['controller_started']
        assert report['velocity_publishers']=={'/cmd_vel':0,'/cmd_vel_nav':0}
        assert report['command_messages']==report['real_navigation_status_publishers']==0
        events=report['adapter_events'];missions=[i for i,e in enumerate(events) if e['event']=='mission']
        first=events[missions[0]:missions[1]]
        sends=[e for e in first if e['event']=='send'];acks=[e for e in first if e['event']=='accepted']
        assert [e['segment'] for e in sends]==[0,1]
        old=next(e for e in first if e['event']=='result' and e['uuid']==acks[0]['uuid'])
        assert old['outcome']=='aborted' and sends[1]['wall']<old['wall']<acks[1]['wall']
        crossing=next(e for e in first if e['event']=='crossing')
        assert crossing['passage_id']=='-93' and crossing['wall']>sends[1]['wall']
        assert abs(sends[0]['point'][1]-events[missions[0]]['contract']['start'][1])<1e-7
        assert any(s['state']=='succeeded' and s['confirmed_crossings']==1 and s['stop_confirmed'] for s in report['statuses'])
        assert any(s['state']=='failed' and s['reason']=='action_ack_timeout' and not s['stop_confirmed'] for s in report['statuses'])
        assert any(e['event']=='cancel_reply' and e['goals_canceling']>0 for e in events)
        assert all(e['physical_motion_enabled'] is False and e['mock_only'] for e in report['statuses'])
    cleanup=json.loads((evidence/'process_cleanup.json').read_text())
    assert cleanup['passed'] and not cleanup['matching_processes']
    local=json.loads((evidence/'local_validation.json').read_text());log=(evidence/local['log']).read_bytes()
    assert sha(log)==local['log_sha256'] and b'Ran 80 tests' in gzip.decompress(log) and b'\nOK\n' in gzip.decompress(log)
    for name,digest in local['source_sha256'].items():assert sha((root/name).read_bytes())==digest,name
    result={'passed':True,'checkpoint':'06B1','unit_tests_passed':80,'ros_mock_cases_passed':10,
            'runtime_files_verified':len(manifest['members']),'current_sources_matched':matched,
            'accepted_session_wall_s':run['elapsed_wall_s'],'initial_failed_attempt_preserved':True,
            'old_result_before_new_acceptance_verified':True,'process_cleanup_verified':True,
            'verifier_sha256':sha(Path(__file__).read_bytes()),
            'scope':'Mock ROS transport/state evidence; TF and action outcomes are synthetic. No real Nav2, physical motion or costmap constraint claim.'}
    (evidence/'verification.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evidence',type=Path,required=True)
    print(json.dumps(verify(p.parse_args().evidence),indent=2))
