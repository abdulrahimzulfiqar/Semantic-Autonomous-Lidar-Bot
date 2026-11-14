#!/usr/bin/env python3
"""Fresh Phase 07 simulation workspace from a frozen Phase 06 baseline.

Never launches physics. AMCL/route preprocessing retain the full map; Nav2
costmaps consume only the window topic in the rolling variant.
"""
import argparse
import hashlib
import json
from pathlib import Path
import tempfile
import yaml
from prepare_gcu_segmented_crossing import prepare as prepare_crossing


def prepare(baseline,assets,fixture_path,destination):
    fixture=json.loads(fixture_path.read_text());input_fixture=dict(fixture)
    already_fused="'imu0': '/imu_broadcaster/imu'" in (baseline/'gcu_amcl.launch.py').read_text()
    if already_fused:input_fixture['fuse_imu_yaw_rate']=False
    # The shared preparer adds IMU fusion only to an unfused launch template.
    # A previously fused accepted baseline is preserved, not patched twice.
    with tempfile.TemporaryDirectory() as folder:
        path=Path(folder)/'fixture.json';path.write_text(json.dumps(input_fixture))
        prepare_crossing(baseline,assets,destination,path)
    (destination/'assets/mission_fixture.json').write_text(json.dumps(fixture,indent=2)+'\n')
    if fixture.get('fuse_imu_yaw_rate') and "'imu0': '/imu_broadcaster/imu'" not in (destination/'gcu_amcl.launch.py').read_text():
        raise ValueError('required_imu_fusion_missing')
    params=yaml.safe_load((destination/'assets/nav2_sensor_constraints.yaml').read_text())
    for name in ['global_costmap','local_costmap']:
        costmap=params[name][name]['ros__parameters']
        # Hold this frequency fixed for the static baseline as well.
        costmap.update(update_frequency=5.,publish_frequency=5.)
        if fixture.get('rolling_window'):
            costmap['static_layer']['map_topic']='/osmag/planning_window'
    class Dumper(yaml.SafeDumper):
        def ignore_aliases(self,data):return True
    (destination/'assets/nav2_sensor_constraints.yaml').write_text(yaml.dump(params,Dumper=Dumper,sort_keys=False))
    files={str(p.relative_to(destination)):hashlib.sha256(p.read_bytes()).hexdigest()
           for p in destination.rglob('*') if p.is_file() and p.name!='crossing_manifest.json'}
    (destination/'crossing_manifest.json').write_text(json.dumps(dict(scope='Phase 07 admitted simulation; explicit execute required',
        baseline=str(baseline),already_fused_baseline=already_fused,spawn=fixture['spawn'],goal=fixture['goal'],
        files_sha256=files),indent=2)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--source-assets',type=Path,required=True);p.add_argument('--fixture',type=Path,required=True)
    p.add_argument('--destination',type=Path,required=True);a=p.parse_args()
    prepare(a.baseline,a.source_assets,a.fixture,a.destination)
