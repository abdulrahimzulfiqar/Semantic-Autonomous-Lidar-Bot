#!/usr/bin/env python3
"""Replay native rolling-window arrays and plans without ROS or Gazebo."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import tarfile
import numpy as np
from gcu_plan_validation import validate_path


def verify(folder,trial='06'):
    with tarfile.open(folder/('attempt'+trial+'.tar.gz')) as t:
        read=lambda p:json.loads(t.extractfile(p).read())
        manifest=read('manifest.json')
        for name,want in manifest['files_sha256'].items():
            assert hashlib.sha256(t.extractfile(name).read()).hexdigest()==want,name
        assert not any(n.endswith(('.pgm','.png')) for n in manifest['files_sha256'])
        prefix='results/native_'+trial+'/'
        run=read(prefix+'run.json');r=read(prefix+'report.json')
        assert run['passed'] and run['child_exits']==[0,0,0] and run['inputs_unchanged']
        assert not run['child_audit']['critical'] and r['passed']
        assert len(r['cases'])==5 and not r['commands'] and r['cmd_vel_publishers']==0
        assert r['full_map_subscribers']==0 and r['native_window_subscribers']==2
        assert r['lifecycle_cleaned'] and all(v=='unconfigured' for v in r['lifecycle_cleaned'].values())
        for name in ['planner.log','local.log']:
            log=t.extractfile(prefix+name).read().decode(errors='replace')
            assert 'Unable to start transition' not in log
            assert 'SEVERE WARNING' not in log
        origins=[]
        for case in r['cases']:
            snap=case['window'];target=case['target'];assert target['window_sha256']==snap['sha256']
            assert target['window_revision']==snap['revision']==case['revision']
            arrays=np.load(io.BytesIO(t.extractfile(prefix+f"revision_{case['revision']:02d}.npz").read()))
            assert arrays['physical'].shape==arrays['mask'].shape==arrays['global'].shape==(128,128)
            assert snap['grid_bytes']==16384 and target['proxy']
            for blocked in [arrays['mask']==100,arrays['physical']==100]:
                values=arrays['global'][blocked]
                assert np.all((values<0)|(values==100))
            for key in ['global','local']:
                meta=case['costmaps'][key];assert meta['frame']=='map'
                rows,cols=np.mgrid[:arrays[key].shape[0],:arrays[key].shape[1]]
                x=meta['origin'][0]+(cols+.5)*meta['resolution']
                y=meta['origin'][1]+(rows+.5)*meta['resolution']
                sr=np.floor((y-snap['origin'][1])/snap['resolution']).astype(int)
                sc=np.floor((x-snap['origin'][0])/snap['resolution']).astype(int)
                inside=(sr>=0)&(sr<128)&(sc>=0)&(sc<128)
                required=np.zeros_like(inside);required[inside]=arrays['mask'][sr[inside],sc[inside]]==100
                assert not np.any(required&(arrays[key]>=0)&(arrays[key]<100))
            origins.append(tuple(snap['origin']))
            if case['closed']:
                assert case['action_status']==6 and not case['path']
                assert case['admission'] is None and case['admission_rejection']=='proxy_prefix_blocked_by_constraint'
            else:
                assert case['admission']['window_sha256']==snap['sha256']
                assert case['admission']['window_revision']==case['revision']
                assert 0<=case['admission']['observed_at']-case['admission']['source_last_publication']<=.5
                assert case['action_status']==4 and case['path']
                assert validate_path(arrays['combined'],snap['resolution'],snap['origin'],case['path'],.20)['valid']
        assert len(set(origins[:3]))==3 and origins[0]==origins[3]==origins[4]
    return dict(verification_passed=True,scope='Native plan-only evidence; no physical motion claim',
        revisions=5,distinct_initial_window_origins=3,closure_and_reopen=True,
        global_grid_shape=[128,128],full_map_subscribers=0,observed_velocity_commands=0,
        native_process_exits=[0,0,0])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evidence',type=Path,required=True)
    p.add_argument('--trial',default='06');a=p.parse_args();r=verify(a.evidence,a.trial)
    (a.evidence/'verification.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
