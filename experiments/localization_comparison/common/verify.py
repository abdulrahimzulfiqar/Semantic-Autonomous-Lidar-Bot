#!/usr/bin/env python3
"""Replay hashes and paired scores from one frozen Phase 08 capture workspace."""
import argparse
import hashlib
import json
from pathlib import Path
import tarfile
import tempfile
from score import score


def verify(folder,case):
    folder=Path(folder);prefix='results/paired_capture_'+case+'_01/'
    with tarfile.open(folder/'attempt01.tar.gz') as t, tempfile.TemporaryDirectory() as temporary:
        read=lambda name:json.loads(t.extractfile(name).read())
        run=read('results/phase8_'+case+'_01/run.json')
        assert run['passed'] and run['process_exits']==[0]*5 and not run['audit']['critical']
        assert run['truth_shutdown']['passed'] and run['simulator_shutdown']['passed']
        runtime=read('crossing_manifest.json')['files_sha256']
        for name,want in runtime.items():
            assert hashlib.sha256(t.extractfile(name).read()).hexdigest()==want,name
        capture=read(prefix+'capture.json');candidate=read(prefix+'structural_estimate.json')
        code=folder.parent/'candidate_source_snapshot.tar.gz'
        with tarfile.open(code) as src:
            for name in ['tracker.py','replay.py','config.json']:
                expected=next(v for k,v in candidate['input_sha256'].items() if Path(k).name==name)
                assert hashlib.sha256(src.extractfile('structural_2d/'+name).read()).hexdigest()==expected,name
            assert hashlib.sha256(src.extractfile('common/capture.py').read()).hexdigest()==capture['recorder_sha256']
        osm_sha=hashlib.sha256(t.extractfile('assets/gcu_ground_floor.osm').read()).hexdigest()
        assert next(v for k,v in candidate['input_sha256'].items() if Path(k).name=='gcu_ground_floor.osm')==osm_sha
        assert capture['rows']>=100 and capture['max_scan_gap_sim_s']<=.15
        # Both use the same supplied start, not a pose fitted to the evaluator.
        fixture=read('assets/mission_fixture.json')
        assert capture['initial_pose']==fixture['spawn']
        out=Path(temporary)
        for name in ['capture.json','observations.npz','evaluation.npz','structural_estimate.json']:
            (out/name).write_bytes(t.extractfile(prefix+name).read())
        result=score(out,out/'structural_estimate.json')
        assert result['paired_fraction']>=.95
        original=read('results/phase8_'+case+'_01/report.json')
        assert original['passed'] and not original['contacts'] and original['state']['stop_confirmed']
    result.update(verification_passed=True,baseline_navigation_passed=True,
        candidate_gate_is_tracking_only=True,archive_sha256=hashlib.sha256((folder/'attempt01.tar.gz').read_bytes()).hexdigest())
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evidence',type=Path,required=True);p.add_argument('--case',choices=['clean','clutter'],required=True);a=p.parse_args()
    result=verify(a.evidence,a.case)
    (a.evidence/'paired_scores.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({key:result[key] for key in ['verification_passed','amcl','structural_2d','odometry_only_diagnostic','candidate_tracking_gate']},indent=2))
