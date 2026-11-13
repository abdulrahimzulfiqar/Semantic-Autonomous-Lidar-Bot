#!/usr/bin/env python3
"""Replay 06B2A evidence, preserving the distinction from phase acceptance."""
import argparse,hashlib,io,json,tarfile
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from gcu_costmap_observation import inspect
from gcu_plan_validation import validate_path


def verify(folder):
    manifest=json.loads((folder/'manifest.json').read_text())
    for name,digest in manifest['artifacts_sha256'].items():
        if hashlib.sha256((folder/name).read_bytes()).hexdigest()!=digest:raise ValueError('changed_artifact:'+name)
    for name,digest in manifest['source_sha256'].items():
        if hashlib.sha256((Path(__file__).resolve().parents[1]/name).read_bytes()).hexdigest()!=digest:raise ValueError('changed_source:'+name)
    def read(archive,name):
        data=archive.extractfile(name)
        if data is None:raise ValueError('missing_archive_member:'+name)
        return data.read()
    with tarfile.open(folder/'attempt03.tar.gz') as archive:
        report=json.loads(read(archive,'results/stationary_03/report.json'))
        run=json.loads(read(archive,'results/stationary_03/run.json'))
        fixture=json.loads(read(archive,'fixture/fixture.json'))
        assert report['passed'] and not run['passed']
        assert report['navigate_status_publishers']==0 and len(report['scan_publishers'])==1
        assert not any(abs(v)>1e-10 for row in report['commands'] for v in row[1:])
        assert report['lifecycle_states']=={'controller_server':'inactive','planner_server':'inactive'}
        assert len(report['revisions'])==2
        rows=[]
        for row in report['revisions']:
            rev=row['revision'];arrays=np.load(io.BytesIO(read(archive,'results/stationary_03/revision_'+str(rev)+'.npz')))
            snap=SimpleNamespace(mask=arrays['mask'],resolution=fixture['snapshot']['resolution'],origin=fixture['snapshot']['origin'])
            assert hashlib.sha256(snap.mask.tobytes()).hexdigest()==row['mask_array_sha256']
            observations={}
            for key,m in row['costmaps'].items():
                assert m['stamp']>=row['refresh_finished']>=row['published_at']
                observations[key]=inspect(snap,arrays[key],m['resolution'],m['origin'],m['map_from_grid'],
                    [report['localized_start'],fixture['goal']],full=key=='global')
                assert observations[key]['missing_cells']==0
                assert observations[key]['required_cells']==row['observed'][key]['required_cells']
            assert row['scan_count_after_refresh']>=3 and 0<=row['latest_scan_age']<=.3
            fresh=[s for s in report['scans'] if s['stamp']>=row['refresh_finished'] and s['received']>=row['refresh_finished']]
            assert len(fresh)>=row['scan_count_after_refresh']
            # Original physical occupancy and current logical restrictions both
            # constrain the independent full-circle path check.
            combined=np.where((arrays['physical']==0)&(snap.mask==0),0,np.where(arrays['physical']<0,-1,100))
            assert row['plan']['status']==4 and row['plan']['points']
            assert validate_path(combined,snap.resolution,snap.origin,row['plan']['points'],.20)['valid']
            rows.append({'revision':rev,'observations':observations,'scan_count_after_refresh':row['scan_count_after_refresh']})
        assert rows[1]['observations']['local']['required_cells']>0
        assert report['revisions'][0]['mask_array_sha256']!=report['revisions'][1]['mask_array_sha256']
        assert run['truth_shutdown']['passed'] and not run['simulator_shutdown']['passed']
        assert any('exit code -9' in item['message'] for item in run['audit']['critical'])
    assert not json.loads((folder/'environment.json').read_text())['remaining_test_processes']
    return {'verification_passed':True,'checkpoint_accepted':False,'sensor_and_plan_checks_passed':True,
            'motion_tested':False,'blocking_gate':'Gazebo stop acknowledged but server failed to exit; killed during launch cleanup.',
            'replayed_revisions':rows,'processes_remaining':0}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evidence',type=Path,required=True);a=p.parse_args()
    result=verify(a.evidence);(a.evidence/'verification.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
