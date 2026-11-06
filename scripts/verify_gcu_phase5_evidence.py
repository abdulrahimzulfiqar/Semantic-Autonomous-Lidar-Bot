#!/usr/bin/env python3
"""Verify Phase 05 archived hashes, process gates and measured navigation traces."""
import argparse
import gzip
import hashlib
import io
import json
import math
from pathlib import Path
import tarfile

import numpy as np
from PIL import Image
import yaml
from gcu_navigation_metrics import coverage, rest_window
from gcu_plan_validation import validate_path


def verify(evidence, live_run):
    root=Path(__file__).resolve().parents[1]
    archived_sources={}
    snapshot_manifest=evidence/'preview_v1_source_manifest.json'
    if snapshot_manifest.is_file():
        snapshot=json.loads(snapshot_manifest.read_text())
        snapshot_path=evidence/snapshot['archive']
        assert hashlib.sha256(snapshot_path.read_bytes()).hexdigest()==snapshot['sha256']
        with tarfile.open(snapshot_path) as sources:
            assert set(sources.getnames())==set(snapshot['members'])
            for name,digest in snapshot['members'].items():
                payload=sources.extractfile(name).read()
                assert hashlib.sha256(payload).hexdigest()==digest,name
                archived_sources[name]=digest
    matched=[]; archived_matched=[]
    def match_source(path,digest):
        relative=str(path.relative_to(root))
        if hashlib.sha256(path.read_bytes()).hexdigest()==digest:
            matched.append(relative)
        else:
            assert archived_sources.get(relative)==digest,('source/runtime mismatch',relative)
            archived_matched.append(relative)
    manifest=json.loads((evidence/'runtime_evidence_manifest.json').read_text())
    archive_path=evidence/manifest['archive']
    assert hashlib.sha256(archive_path.read_bytes()).hexdigest()==manifest['sha256']
    with tarfile.open(archive_path) as archive:
        assert set(archive.getnames())==set(manifest['members'])
        for name,digest in manifest['members'].items():
            assert hashlib.sha256(archive.extractfile(name).read()).hexdigest()==digest,name
        def read(name):return archive.extractfile(name).read()
        def load(name):return json.loads(read(name))
        summary=json.loads((evidence/'metric_results.json').read_text())
        assert summary['passed'] and summary['matrix_passed']
        comparison=load('metric/bridge_comparison_03/report.json')
        assert comparison['passed'] and comparison['matched_timestamps']>=50 and comparison['span_sim_s']>=1.
        assert all(max(abs(a-b) for a,b in zip(pair['new'],pair['reference']))<1e-12
                   for pair in comparison['samples'])
        base='metric/'+summary['final_group']
        assert load('runtime_final/current_input_verification.json')['passed']
        assert len(summary['matrix'])==16
        frozen=load(base+'/frozen_inputs.json')
        assert frozen==load(base+'/matrix_01/frozen_inputs.json')
        for name,digest in frozen.items():
            snapshot='runtime_final/'+name
            if snapshot in manifest['members']:
                assert hashlib.sha256(read(snapshot)).hexdigest()==digest,name
        def runner(name):
            result=load(name+'/run.json')
            assert result['passed'] and all(v==0 for v in result['launch_exits']),name
            audit=result.get('child_process_audit',result.get('child_audit'))
            assert not audit['critical'],name
        cfg=yaml.safe_load(read('runtime_final/assets/gcu_ground_floor.yaml'))
        def grid(fixture=None):
            name='runtime_final/assets/'+('navigation_fixtures/'+fixture+'/map.pgm' if fixture else 'gcu_ground_floor.pgm')
            image=np.flipud(np.asarray(Image.open(io.BytesIO(read(name)))))
            return np.where(image==254,0,np.where(image==0,100,-1))
        original=grid()
        checked=[]
        def measured(name, report, fixture=None):
            raw=load(name+'/'+report)
            assert raw['passed']
            assert len(raw['contact_publishers'])==13 and all(v==1 for v in raw['contact_publishers'].values())
            if raw['positive_control']:
                assert raw['contact_event_count']>0
                return
            assert raw['contact_event_count']==0 and not raw['contacts']
            assert raw['velocity_publishers']==[{'name':'controller_server','namespace':'/'}]
            peak=max((abs(command['v']) for command in raw['commands']),default=0.)
            assert peak<=.100001 and abs(peak-raw['max_command_mps'])<1e-9
            pairs=raw['localization']
            position=[math.dist(p['truth'][:2],p['estimated'][:2]) for p in pairs]
            angle=[abs(math.degrees(math.atan2(math.sin(p['truth'][2]-p['estimated'][2]),
                                              math.cos(p['truth'][2]-p['estimated'][2])))) for p in pairs]
            assert abs(np.percentile(position,95)-raw['localization_p95_position_m'])<1e-9
            assert abs(np.percentile(angle,95)-raw['localization_p95_heading_deg'])<1e-9
            assert np.percentile(position,95)<=.25 and np.percentile(angle,95)<=5.
            cov=coverage(raw['truth'],pairs)
            assert cov==raw['sampling_coverage'] and cov['matched_fraction']>=.9 and cov['max_truth_gap_sim_s']<=.15
            assert rest_window([v for v in raw['truth'] if v['sim_time']>=raw['rest_confirmed_sim']-1.3]) is not None
            swept=validate_path(grid(fixture) if fixture else original,cfg['resolution'],cfg['origin'][:2],
                                [p['truth'][:2] for p in raw['truth']])
            assert swept['valid'],name
            if raw['expected']=='success':
                assert raw['action_status']==4 and math.dist(raw['final_truth'][:2],raw['goal'][:2])<=.2
                difference=raw['final_truth'][2]-raw['goal'][2]
                assert abs(math.degrees(math.atan2(math.sin(difference),math.cos(difference))))<=10.
            elif raw['expected']=='cancelled':
                assert raw['action_status']==5 and raw['cancel_ack_count']>0
                assert rest_window(raw['truth'],earliest=raw['cancel_request_sim'])['start_sim']-raw['cancel_request_sim']<=1.
            else:
                assert not any(abs(v['v'])+abs(v['w'])>1e-5 for v in raw['commands'])
                if raw['via_click_bridge']:
                    assert raw['action_status'] is None and raw['bridge_outcome']=='rejected' and not raw['display_paths'][-1]['points']
                else:assert raw['action_status']==6
            checked.append(name+'/'+report)
        for row in summary['matrix']:
            name=base+'/matrix_01/'+row['name'];runner(name)
            fixture='detour' if row['name'].startswith('obstacle_detour') else 'blocked' if row['name'].startswith('blocked_passage') else None
            measured(name,'report.json',fixture)
        serial=shutdown=0
        for trial in summary['reliability']['trials']:
            assert trial['exit']==0
            name=base+'/'+trial['name'];runner(name);measured(name,'report.json')
            if trial['name'].startswith('serial_'):
                measured(name,'followup_report.json');serial+=1
            else:shutdown+=1
        assert serial==3 and shutdown==5
        live='preview/'+live_run;runner(live);preview=load(live+'/report.json')
        live_runtime=load(live+'/run.json')
        assert live_runtime['inputs_unchanged']
        for name,digest in load('preview_runtime/source_hashes.json').items():
            assert live_runtime['preview_source_sha256'][name]==digest,name
        assert preview['passed'] and len(preview['cases'])==13
        assert preview['velocity_publishers']==0 and not preview['command_messages']
        assert all(v==0 for v in preview['navigation_action_publishers'].values())
        assert preview['duplicate_tf_fixture_detected']
        assert preview['tf_edge_publishers']=={'AGmap->map':1,'map->odom':1,'odom->base_footprint':1}
        assert preview['maximum_truth_drift_m']<.01
        preview_map=yaml.safe_load(read('preview_runtime/assets/gcu_ground_floor.yaml')) if 'preview_runtime/assets/gcu_ground_floor.yaml' in manifest['members'] else cfg
        for row in preview['cases']:
            assert row['passed']
            if row['status']['state']=='ready':
                assert validate_path(original,preview_map['resolution'],preview_map['origin'][:2],row['path']['points'])['valid']
            else:assert not row['path']['points']
        for name,digest in frozen.items():
            if name.startswith('support_src/'):
                path=root/'simulation/gcu_ground_floor/support/gcu_truth_bridge'/name.removeprefix('support_src/')
            elif name=='assets/nav2_autonomous.yaml':
                path=root/'simulation/gcu_ground_floor/nav2_handoff.yaml'
            elif name=='run_gcu_navigation_test.py':path=root/'scripts/run_gcu_phase5_navigation_test.py'
            elif name=='run_gcu_navigation_suite.py':path=root/'scripts/run_gcu_phase5_navigation_suite.py'
            elif '/' not in name and name.endswith('.py'):path=root/'scripts'/name
            elif name.startswith('assets/'):path=root/'simulation/gcu_ground_floor'/name.removeprefix('assets/')
            else:continue
            if path.is_file():
                match_source(path,digest)
        for name,digest in live_runtime['preview_source_sha256'].items():
            path=root/'scripts'/name if '/' not in name and name.endswith('.py') else (
                root/'simulation/gcu_ground_floor/support/gcu_tf_probe'/name.removeprefix('probe_src/')
                if name.startswith('probe_src/') else None)
            if path is not None and path.is_file():
                match_source(path,digest)
        offline=json.loads((evidence/'preview_acceptance_final/results.json').read_text())
        assert offline['passed'] and len(offline['cases'])==8
        for name,digest in offline['source_sha256'].items():
            match_source(root/'scripts'/name,digest)
        for name,digest in offline['input_sha256'].items():
            assert hashlib.sha256(read('preview_runtime/assets/'+name)).hexdigest()==digest,name
        local=json.loads((evidence/'local_validation.json').read_text())
        log=(evidence/local['log']).read_bytes()
        assert hashlib.sha256(log).hexdigest()==local['log_sha256']
        assert local['unit_tests_passed']==34 and b'Ran 34 tests' in gzip.decompress(log)
    result={'passed':True,'archive_files_verified':len(manifest['members']),
            'matrix_cases':16,'serial_two_goal_runs':serial,'clean_shutdown_controls':shutdown,
            'measured_traces_recomputed':len(checked),'live_preview_cases':13,
            'truth_bridge_native_payload_matches':comparison['matched_timestamps'],
            'repository_source_files_matched':sorted(set(matched)),
            'archived_v1_source_files_matched':sorted(set(archived_matched)),
            'scope':'Recorded simulation evidence; no author-runtime, continuous-space optimum or hardware performance claim.'}
    output='independent_verification_v1_replay.json' if archived_matched else 'independent_verification.json'
    (evidence/output).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence',type=Path,required=True)
    parser.add_argument('--live-run',default='live_05')
    args=parser.parse_args();verify(args.evidence,args.live_run)
