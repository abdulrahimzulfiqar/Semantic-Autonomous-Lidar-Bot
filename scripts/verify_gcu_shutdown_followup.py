#!/usr/bin/env python3
"""Replay the bounded 06B2A shutdown controls; never infer a general fix."""
import argparse,hashlib,io,json,tarfile
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from gcu_costmap_observation import inspect
from gcu_plan_validation import validate_path


def verify(folder):
    manifest=json.loads((folder/'manifest.json').read_text())
    for name,digest in manifest['files_sha256'].items():
        assert hashlib.sha256((folder/name).read_bytes()).hexdigest()==digest,name
    root=Path(__file__).resolve().parents[1]
    for name,digest in manifest['source_sha256'].items():
        assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
    summary=[]
    with tarfile.open(folder/'runtime_evidence.tar.gz') as archive:
        def raw(name):return archive.extractfile(name).read()
        def data(name):return json.loads(raw(name))
        runtime=data('shutdown_followup_manifest.json');assert not runtime['remaining_test_processes']
        # Match the runtime checker/helper to the frozen original sensor trial:
        # these runs changed diagnostic observation, not the navigation profile.
        for name in ['check_gcu_sensor_constraints.py','run_gcu_sensor_constraints.py','gcu_costmap_observation.py','gcu_process_audit.py']:
            assert hashlib.sha256(raw(name)).hexdigest()==runtime['files_sha256'][name]
            assert raw(name)==(root/'scripts'/name).read_bytes()
        fixture=data('fixture/fixture.json')
        for trial in ['04','05','06']:
            prefix='results/diagnostic_'+trial+'/'
            run=data(prefix+'run.json');report=data(prefix+'report.json');capture=data(prefix+'shutdown_probe_capture.json')
            assert run['passed'] and report['passed'] and run['checker_exit']==0
            assert run['process_exits']==[0,0,0,0] and not run['audit']['critical']
            assert run['truth_shutdown']['passed'] and run['simulator_shutdown']['passed']
            assert not run['simulator_shutdown']['still_alive_pids'] and not capture['captured']
            assert report['navigate_status_publishers']==0 and len(report['scan_publishers'])==1
            assert all(abs(v)<=1e-10 for row in report['commands'] for v in row[1:])
            assert set(report['lifecycle_states'].values())=={'inactive'} and len(report['lifecycle_states'])==2
            assert len(report['revisions'])==2
            observations=[]
            for row in report['revisions']:
                arrays=np.load(io.BytesIO(raw(prefix+'revision_'+str(row['revision'])+'.npz')))
                snap=SimpleNamespace(mask=arrays['mask'],resolution=fixture['snapshot']['resolution'],origin=fixture['snapshot']['origin'])
                assert hashlib.sha256(snap.mask.tobytes()).hexdigest()==row['mask_array_sha256']
                native={}
                for key,m in row['costmaps'].items():
                    assert m['stamp']>=row['refresh_finished']>=row['published_at']
                    native[key]=inspect(snap,arrays[key],m['resolution'],m['origin'],m['map_from_grid'],
                        [report['localized_start'],fixture['goal']],full=key=='global')
                    assert native[key]['missing_cells']==0
                    assert native[key]['required_cells']==row['observed'][key]['required_cells']
                assert row['scan_count_after_refresh']>=3 and 0<=row['latest_scan_age']<=.3
                assert len([s for s in report['scans'] if min(s['stamp'],s['received'])>=row['refresh_finished']])>=row['scan_count_after_refresh']
                combined=np.where((arrays['physical']==0)&(snap.mask==0),0,np.where(arrays['physical']<0,-1,100))
                assert row['plan']['status']==4 and row['plan']['points']
                assert validate_path(combined,snap.resolution,snap.origin,row['plan']['points'],.20)['valid']
                observations.append(native)
            assert observations[1]['local']['required_cells']>0
            assert report['revisions'][0]['mask_array_sha256']!=report['revisions'][1]['mask_array_sha256']
            summary.append({'trial':trial,'passed':True,'elapsed_wall_s':run['elapsed_wall_s'],
                'local_blocked_cells_after_refresh':observations[1]['local']['required_cells'],
                'process_exits':run['process_exits']})
    return {'verification_passed':True,'stationary_controls_passed':3,'trials':summary,'motion_tested':False,
            'hang_reproduced':False,'root_cause_established':False,'general_shutdown_fix_claimed':False,
            'remaining_test_processes':0,'next':'One isolated constrained crossing; retain shutdown diagnostics and strict exit auditing.'}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evidence',type=Path,required=True);a=p.parse_args()
    result=verify(a.evidence);(a.evidence/'verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
