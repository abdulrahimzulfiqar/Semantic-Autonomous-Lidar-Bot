#!/usr/bin/env python3
"""Verify 06C1 evidence honestly: offline pass, native integration UNFINISHED."""
import argparse,gzip,hashlib,io,json,tarfile
from pathlib import Path
import numpy as np
from osmag_constraint_mask import build_mask,ConsumptionGate
from shapely.geometry import box


def verify(evidence):
    root=Path(__file__).resolve().parents[1];sha=lambda b:hashlib.sha256(b).hexdigest()
    archived={};archived_matches=[]
    snapshot_path=evidence/'source_before_followup_manifest.json'
    if snapshot_path.is_file():
        snapshot=json.loads(snapshot_path.read_text())
        assert sha((evidence/snapshot['archive']).read_bytes())==snapshot['sha256']
        with tarfile.open(evidence/snapshot['archive']) as old:
            assert set(old.getnames())==set(snapshot['members'])
            for name,digest in snapshot['members'].items():
                assert sha(old.extractfile(name).read())==digest,name
                archived[name]=digest
    def match(path,digest):
        name=str(path.relative_to(root))
        if sha(path.read_bytes())!=digest:
            assert archived.get(name)==digest,('historical source mismatch',name)
            archived_matches.append(name)
    checkpoint=json.loads((evidence/'checkpoint.json').read_text())
    for name,digest in checkpoint['source_sha256'].items():match(root/name,digest)
    for name,digest in checkpoint['artifacts_sha256'].items():assert sha((evidence/name).read_bytes())==digest,name
    offline=json.loads((evidence/'results.json').read_text())
    assert offline['passed'] and len(offline['cases'])==7 and all(r['passed'] for r in offline['cases'])
    for name,digest in offline['source_sha256'].items():match(root/'scripts'/name,digest)
    assert b'Ran 95 tests' in gzip.decompress((evidence/'local_tests.log.gz').read_bytes())
    assert b'\nOK\n' in gzip.decompress((evidence/'local_tests.log.gz').read_bytes())
    manifest=json.loads((evidence/'native_manifest.json').read_text())
    assert sha((evidence/manifest['archive']).read_bytes())==manifest['sha256']
    with tarfile.open(evidence/manifest['archive']) as t:
        assert set(t.getnames())==set(manifest['members'])
        def read(name):return t.extractfile(name).read()
        for name,digest in manifest['members'].items():assert sha(read(name))==digest,name
        for attempt in ['attempt_01','attempt_02']:
            run=json.loads(read(attempt+'/results/'+attempt+'/run.json'))
            assert run['passed'] is False
            for name,digest in run['source_sha256'].items():
                assert sha(read(attempt+'/'+name))==digest,name
                if attempt=='attempt_02' and name.endswith('.py'):
                    match(root/'scripts'/name,digest)
        native=json.loads(read('attempt_02/results/attempt_02/report.json'))
        assert native['passed'] is False and len(native['cases'])==1
        assert native==json.loads((evidence/'native_report.json').read_text())
        run=json.loads((evidence/'native_run.json').read_text())
        assert run['child_exits']==[1,0,-6,0] and run['child_audit']['critical']
        assert native['last_consumption_errors']['global']=='mask_constraints_not_consumed'
        assert native['last_consumption_errors']['local']=='costmap_not_fresh_after_mask'
        with np.load(io.BytesIO(read('attempt_02/results/attempt_02/revision_1.npz'))) as arrays:
            snap=build_mask(arrays['physical'],.05,(-4.,-4.),{'a':box(-2,-2,0,2),'b':box(0,-2,2,2)},
                {'p':[(0,-1.5),(0,1.5)]},{'p':('a','b')},allowed_areas=['a','b'],open_passages=['p'],
                disabled_passages=[],robot_pose=(-1,0),mission_id='native_fixture')
            np.testing.assert_array_equal(snap.mask,arrays['mask'])
            row=native['cases'][0];assert row['passed'] and row['action_status']==4
            assert snap.mask_sha256==row['snapshot']['mask_sha256']
            gate=ConsumptionGate(snap,0.,[(-1,0),(0,0),(1,0)])
            for key in ['global','local']:
                cfg=row['costmaps'][key];obs=row['consumed'][key]
                # The fixture used an identity map<-odom transform. This replays
                # cell consumption, not the unrecorded mask publication instant.
                gate.observe(key,arrays[key],cfg['resolution'],cfg['origin'],cfg['stamp'],obs['observed_at'])
    cleanup=json.loads((evidence/'process_cleanup.json').read_text())
    assert cleanup['passed'] and not cleanup['matching_processes']
    result={'evidence_integrity_passed':True,'offline_masks_passed':True,'unit_tests':95,'offline_gcu_cases':7,
        'native_open_revision_recomputed':True,'native_integration_passed':False,'process_cleanup_verified':True,
        'runtime_files_verified':len(manifest['members']),'verifier_sha256':sha(Path(__file__).read_bytes()),
        'archived_source_matches':sorted(set(archived_matches)),
        'resume':'Instrument failed close-update raw grids/timestamps and investigate standalone local-costmap teardown. Reopen and motion acceptance remain pending.'}
    output='historical_verification_replay.json' if archived_matches else 'verification.json'
    (evidence/output).write_text(json.dumps(result,indent=2)+'\n');return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evidence',type=Path,required=True)
    print(json.dumps(verify(p.parse_args().evidence),indent=2))
