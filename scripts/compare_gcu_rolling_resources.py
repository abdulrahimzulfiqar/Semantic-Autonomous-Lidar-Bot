#!/usr/bin/env python3
"""One matched-query exploratory comparison; not a peak/whole-stack benchmark."""
import argparse
import copy
import hashlib
import io
import json
import math
from pathlib import Path
import re
import tarfile
import yaml
import numpy as np
from osmag_paper_planner import PaperPlanner
from verify_gcu_segmented_crossing import score_mission


def load(path,prefix):
    with tarfile.open(path) as t:
        report=json.load(t.extractfile('results/'+prefix+'/report.json'))
        run=json.load(t.extractfile('results/'+prefix+'/run.json'))
        params=yaml.safe_load(t.extractfile('assets/nav2_sensor_constraints.yaml').read())
        fixture=json.load(t.extractfile('assets/mission_fixture.json'))
        assert run['passed'] and report['passed'] and run['process_exits']==[0]*5
        assert not run['audit']['critical'] and run['truth_shutdown']['passed'] and run['simulator_shutdown']['passed']
        pinned=json.load(t.extractfile('crossing_manifest.json'))['files_sha256']
        for name,digest in pinned.items():
            assert hashlib.sha256(t.extractfile(name).read()).hexdigest()==digest,name
        arrays=np.load(io.BytesIO(t.extractfile('results/'+prefix+'/'+report.get('constraint_file','constraint.npz')).read()))
        score_mission(report,arrays,PaperPlanner(Path(__file__).resolve().parents[1]/'simulation/gcu_ground_floor',execution_padding=.02),
            allow_proxy_repeats=bool(fixture.get('rolling_window')))
        identical={}
        for name in ['gcu_amcl.launch.py','assets/amcl.yaml','assets/contact_v2/world.sdf','assets/contact_v2/robot.sdf']:
            text=t.extractfile(name).read().decode()
            identical[name]=re.sub(r'/home/ubuntu/edgeosmag_phase7_[^/\s\"<>]+','WORKSPACE',text)
    return report,run,params,fixture,identical


def summarize(report,run):
    start=report['resource_at_dispatch'];end=report['resource_at_stop'];elapsed=end['wall']-start['wall']
    resources={};before={p['pid']:p for p in start['processes']}
    for p in end['processes']:
        old=before[p['pid']];assert old['role']==p['role']
        resources[p['role']]=dict(rss_dispatch_kib=old['rss_kib'],rss_stop_kib=p['rss_kib'],
            cpu_delta_s=p['cpu_s']-old['cpu_s'],cpu_percent_of_one_core=100*(p['cpu_s']-old['cpu_s'])/elapsed)
    truth=report['truth']
    return dict(executed_length_m=sum(math.dist(a['truth'][:2],b['truth'][:2]) for a,b in zip(truth,truth[1:])),
        position_error_m=report['position_error_m'],heading_error_deg=report['heading_error_deg'],
        dispatch_to_truth_rest_sim_s=report['truth_rest']['confirmed_sim']-report['dispatch_sim'],
        full_wall_s=run['elapsed_wall_s'],resource_interval_wall_s=elapsed,
        native_grid_shapes=report['native_grid_shapes_at_stop'],resources=resources,
        scope='Two RSS samples per process, shared pages not deduplicated; CPU sampled dispatch through post-score rest. Not peaks, PSS, evaluator, Gazebo or whole-stack RAM.')


def compare(rolling,baseline):
    rr,rn,rp,rf,ri=rolling;br,bn,bp,bf,bi=baseline
    for key in ['spawn','goal','input_sha256','expected_passages','fuse_imu_yaw_rate','align_local_grid']:
        assert rf[key]==bf[key],key
    assert ri==bi,'sensor, physics or localization mismatch'
    def normalize(value):
        if isinstance(value,str):return re.sub(r'/home/ubuntu/edgeosmag_phase7_[^/\s\"<>]+','WORKSPACE',value)
        if isinstance(value,dict):return {k:normalize(v) for k,v in value.items()}
        if isinstance(value,list):return [normalize(v) for v in value]
        return value
    rnorm=normalize(copy.deepcopy(rp));bnorm=normalize(copy.deepcopy(bp))
    for name in ['global_costmap','local_costmap']:
        a=rnorm[name][name]['ros__parameters'];b=bnorm[name][name]['ros__parameters']
        assert a['static_layer']['map_topic']=='/osmag/planning_window' and b['static_layer']['map_topic']=='/map'
        a['static_layer']['map_topic']=b['static_layer']['map_topic']='MAP_INPUT'
    assert rnorm==bnorm,'motion/costmap profile mismatch beyond source topic'
    return dict(scope='One trial per variant, matched query/profile/world/localization. Windowed OSM raster versus static PGM; rasterization and proxy dispatch differ. No statistical speed claim.',
        controlled_inputs_match=True,rolling=summarize(rr,rn),static=summarize(br,bn),
        retained_full_map_users=['AMCL','Python graph preprocessing/evaluator'],
        interpretation='Grid cell counts directly show bounded metric planning. RSS/CPU are exploratory component samples, not proof of building-size-independent total memory or Pi performance.')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--rolling',type=Path,required=True)
    p.add_argument('--rolling-prefix',default='rolling_nominal_03');p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--baseline-prefix',default='static_baseline_01');p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=compare(load(a.rolling,a.rolling_prefix),load(a.baseline,a.baseline_prefix))
    result['archive_sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [a.rolling,a.baseline]}
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
