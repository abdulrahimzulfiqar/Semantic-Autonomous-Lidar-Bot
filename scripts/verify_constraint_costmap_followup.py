#!/usr/bin/env python3
"""Replay the Nov 8 native keepout fix evidence, without ROS or a VM."""
import argparse,gzip,hashlib,io,json,tarfile
from pathlib import Path
import numpy as np
from shapely.geometry import box
from osmag_constraint_mask import build_mask,ConsumptionGate
from gcu_plan_validation import validate_path


def verify(evidence,plot=False):
    root=Path(__file__).resolve().parents[1];sha=lambda b:hashlib.sha256(b).hexdigest()
    manifest=json.loads((evidence/'manifest.json').read_text())
    assert sha((evidence/manifest['archive']).read_bytes())==manifest['sha256']
    grids=[]
    with tarfile.open(evidence/manifest['archive']) as t:
        assert set(t.getnames())==set(manifest['members'])
        def read(name):return t.extractfile(name).read()
        def load(name):return json.loads(read(name))
        for name,digest in manifest['members'].items():assert sha(read(name))==digest,name
        diagnostic=load('attempt_03/results/attempt_03/report.json')
        assert not diagnostic['passed'] and diagnostic['lifecycle_shutdown']['passed']
        failure=load('attempt_03/results/attempt_03/failed_revision_2.json')
        with np.load(io.BytesIO(read('attempt_03/results/attempt_03/failed_revision_2.npz'))) as arrays:
            assert np.sum((arrays['mask']==100)&(arrays['global']==0))==116
            assert failure['costmaps']['local']['message_count']>100
            cfg=failure['costmaps']['local'];rr,cc=np.mgrid[:arrays['local'].shape[0],:arrays['local'].shape[1]]
            sr=np.floor((cfg['origin'][1]+(rr+.5)*cfg['resolution']+4)/.05).astype(int)
            sc=np.floor((cfg['origin'][0]+(cc+.5)*cfg['resolution']+4)/.05).astype(int)
            inside=(sr>=0)&(sr<160)&(sc>=0)&(sc<160)
            assert np.all(arrays['local'][inside][arrays['mask'][sr[inside],sc[inside]]==100]==100)
        report=load('attempt_04/results/attempt_04/report.json');run=load('attempt_04/results/attempt_04/run.json')
        assert report==json.loads((evidence/'report.json').read_text())
        assert run==json.loads((evidence/'run.json').read_text())
        assert run['passed'] and run['child_exits']==[0,0,0] and run['inputs_unchanged']
        assert not run['child_audit']['critical'] and run['elapsed_wall_s']<180
        assert report['passed'] and len(report['cases'])==3
        assert report['lifecycle_shutdown']=={'passed':True,'states':{'planner_server':'inactive','costmap/costmap':'inactive'}}
        assert report['velocity_publishers']=={'/cmd_vel':0,'/cmd_vel_nav':0}
        assert report['navigation_status_publishers']==report['command_messages']==0
        for attempt in ['attempt_03','attempt_04']:
            frozen=load(attempt+'/results/'+attempt+'/run.json')['source_sha256']
            for name,digest in frozen.items():
                assert sha(read(attempt+'/'+name))==digest,name
                if attempt=='attempt_04' and name.endswith('.py'):assert sha((root/'scripts'/name).read_bytes())==digest,name
        for rev,row in enumerate(report['cases'],1):
            assert row['revision']==rev and row['passed'] and row['closed']==(rev==2)
            with np.load(io.BytesIO(read(f'attempt_04/results/attempt_04/revision_{rev}.npz'))) as arrays:
                snap=build_mask(arrays['physical'],.05,(-4.,-4.),{'a':box(-2,-2,0,2),'b':box(0,-2,2,2)},
                    {'p':[(0,-1.5),(0,1.5)]},{'p':('a','b')},allowed_areas=['a','b'],
                    open_passages=[] if rev==2 else ['p'],disabled_passages=['p'] if rev==2 else [],
                    robot_pose=(-1,0),mission_id='native_fixture',mask_revision=rev,closure_revision=rev-1)
                np.testing.assert_array_equal(snap.mask,arrays['mask'])
                assert snap.mask_sha256==row['snapshot']['mask_sha256']
                probes=[(-1,0),(1,0)] if rev==2 else [(-1,0),(0,0),(1,0)]
                gate=ConsumptionGate(snap,row['mask_published_at'],probes)
                now=max(v['observed_at'] for v in row['consumed'].values())
                for key in ['global','local']:
                    cfg=row['costmaps'][key]
                    gate.observe(key,arrays[key],cfg['resolution'],cfg['origin'],cfg['stamp'],now)
                assert gate.ready(now)
                if rev==2:assert row['action_status']==6 and not row['path']
                else:
                    assert row['action_status']==4 and validate_path(snap.combined,.05,(-4,-4),row['path'],.20)['valid']
                assert row['physical_wall_global']==100
                grids.append(arrays['global'].copy())
    local=json.loads((evidence/'local_validation.json').read_text());log=(evidence/local['log']).read_bytes()
    assert sha(log)==local['log_sha256'] and b'Ran 95 tests' in gzip.decompress(log) and b'\nOK\n' in gzip.decompress(log)
    for name,digest in local['source_sha256'].items():assert sha((root/name).read_bytes())==digest,name
    cleanup=json.loads((evidence/'process_cleanup.json').read_text());assert cleanup['passed'] and not cleanup['matching_processes']
    result={'passed':True,'date':'2026-11-08','scope':'Synthetic native plan-only keepout updates; no physical motion or sensor-layer refresh certification.',
            'unit_tests':95,'native_revisions_recomputed':3,'action_statuses':[r['action_status'] for r in report['cases']],
            'child_exits':run['child_exits'],'session_wall_s':run['elapsed_wall_s'],'diagnostic_global_missing_cells':116,
            'diagnostic_local_mask_was_applied':True,'runtime_files_verified':len(manifest['members']),
            'cleanup_verified':True,'verifier_sha256':sha(Path(__file__).read_bytes())}
    (evidence/'verification.json').write_text(json.dumps(result,indent=2)+'\n')
    if plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(1,3,figsize=(10,4),constrained_layout=True)
        for i,(ax,grid,row) in enumerate(zip(axes,grids,report['cases'])):
            ax.imshow(grid,origin='lower',extent=[-4,4,-4,4],cmap='gray_r',vmin=0,vmax=100)
            if row['path']:ax.plot(*np.asarray(row['path']).T,color='#008b8b',linewidth=2)
            ax.plot(-1,0,'o',color='#d07000');ax.plot(1,0,'x',color='#db2222')
            ax.set(title=['Open: plan succeeds','Closed: no route','Reopened: plan succeeds'][i],xlim=(-2.3,2.3),ylim=(-2.3,2.3),xlabel='map x (m)')
            ax.set_aspect('equal')
        axes[0].set_ylabel('map y (m)')
        fig.suptitle('Native Humble global costmaps after verified mask consumption\nSynthetic stationary scene; no controller or robot motion')
        fig.savefig(evidence/'native_revisions.png',dpi=150);plt.close(fig)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evidence',type=Path,required=True);p.add_argument('--plot',action='store_true')
    a=p.parse_args();print(json.dumps(verify(a.evidence,a.plot),indent=2))
