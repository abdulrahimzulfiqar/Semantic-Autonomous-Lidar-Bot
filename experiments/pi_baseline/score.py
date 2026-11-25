#!/usr/bin/env python3
"""Independently score a split-host pilot using laptop-only truth and frozen inputs."""
import argparse
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import sys
import tarfile
import tempfile

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
import numpy as np
from osmag_paper_planner import PaperPlanner
from gcu_plan_validation import validate_path
from gcu_navigation_metrics import rest_window,coverage
from osmag_mission_contract import MissionContract,MissionSettings,CrossingMonitor,SegmentGoal


def score(folder):
    outcome=json.loads((folder/'orchestration.json').read_text())
    observer=json.loads((folder/'simulator_results/results/observer.json').read_text())
    worker_path=folder/'pi_results/actor/worker.json'
    report=dict(passed=False,scope='Single processor-in-loop pilot correctness; no repeated benchmark or real-time hardware claim',
        role_runs_passed=bool(outcome['passed']),criteria_source='Inherited Phase 06/07 pilot correctness gates',
        thresholds=dict(goal_position_m=.20,goal_heading_deg=10,localization_p95_m=.25,
            localization_p95_deg=5,matched_fraction=.9,max_truth_gap_sim_s=.15,
            maximum_linear_command_m_s=.10001,swept_radius_m=.18,rest_duration_sim_s=1.))
    if not worker_path.exists():
        report['error']='no_worker_report';return report
    worker=json.loads(worker_path.read_text())
    if not worker['passed']:
        report['error']=worker.get('error','actor_failed');return report
    with tempfile.TemporaryDirectory() as temp:
        assets=Path(temp)
        with tarfile.open(folder/'pi_sources.tar.gz') as t:
            manifest=json.loads(t.extractfile('phase9_manifest.json').read())
            for name,want in manifest['files_sha256'].items():
                if hashlib.sha256(t.extractfile(name).read()).hexdigest()!=want:
                    raise ValueError('frozen_pi_source_changed:'+name)
            for name in t.getnames():
                if name.startswith('assets/'):
                    t.extract(name,assets,filter='data')
        planner=PaperPlanner(assets/'assets',execution_padding=.02)
        begin=worker['dispatch_sim'];end=worker['final_sim']
        source_scans = [sample for sample in observer.get('scan_receipts', [])
                        if begin <= sample['stamp'] <= end]
        if len(source_scans) >= 2:
            wall_span = source_scans[-1]['received_wall'] - source_scans[0]['received_wall']
            if wall_span > 0:
                sim_span = source_scans[-1]['stamp'] - source_scans[0]['stamp']
                report['observed_sim_per_wall_s'] = sim_span / wall_span
                report['simulator_rate_source'] = 'Source scan span divided by simulator observer wall receipt span'
        truth=[v for v in observer['truth'] if begin<=v['sim_time']<=end+2.]
        matched=[v for v in observer['matched'] if truth and truth[0]['sim_time']<=v['sim_time']<=truth[-1]['sim_time']]
        if not truth or not matched:
            report['error']='missing_independent_observations';return report
        final=truth[-1]['truth'];goal=worker['goal'];delta=final[2]-goal[2]
        raw=worker['contract']
        contract=MissionContract(**{**raw,'settings':MissionSettings(**raw['settings']),
            'segments':tuple(SegmentGoal(**v) for v in raw['segments'])})
        oracle=replace(contract,start=tuple(truth[0]['truth'][:2]),pose_stamp=truth[0]['sim_time'])
        monitor=CrossingMonitor(oracle,planner.leaves,planner.passage_segments)
        for row in truth[1:]:monitor.observe(row['truth'][:2],row['sim_time'])
        report.update(position_error_m=math.dist(final[:2],goal[:2]),
            heading_error_deg=math.degrees(abs(math.atan2(math.sin(delta),math.cos(delta)))),
            localization_p95_m=float(np.percentile([v['position_error_m'] for v in matched],95)),
            localization_p95_deg=float(np.percentile([v['yaw_error_deg'] for v in matched],95)),
            coverage=coverage(truth,matched),truth_crossings=monitor.events,
            confirmed_passages=monitor.confirmed,expected_passages=len(contract.passage_ids),
            truth_rest=rest_window(truth,earliest=worker['final_sim']-1.),
            swept_truth=validate_path(planner.grid,planner.resolution,planner.origin,
                [v['truth'][:2] for v in truth],.18),
            contacts=observer['contacts'],contact_publishers=observer['contact_publishers'],
            executed_length_m=sum(math.dist(a['truth'][:2],b['truth'][:2]) for a,b in zip(truth,truth[1:])),
            sim_duration_s=end-begin,actor_wall_duration_s=worker['elapsed_wall_s'])
        commands=observer['commands']
        report['command_bounds_passed']=bool(commands and any(abs(v['v'])>.001 for v in commands)
            and max(abs(v['v']) for v in commands)<=.10001)
        report['passed']=bool(outcome['passed'] and report['position_error_m']<=.2 and report['heading_error_deg']<=10
            and report['localization_p95_m']<=.25 and report['localization_p95_deg']<=5
            and report['coverage']['matched_fraction']>=.9 and report['coverage']['max_truth_gap_sim_s']<=.15
            and report['swept_truth']['valid'] and report['truth_rest'] is not None
            and not report['contacts'] and all(v==1 for v in report['contact_publishers'].values())
            and len(report['contact_publishers'])==13 and report['command_bounds_passed']
            and monitor.confirmed==len(contract.passage_ids))
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evidence',type=Path,required=True)
    a=p.parse_args();result=score(a.evidence)
    (a.evidence/'independent_score.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 1)
