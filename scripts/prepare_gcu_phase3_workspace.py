#!/usr/bin/env python3
"""Copy the checked-in Phase 03 inputs to a NEW isolated workspace on Ubuntu.

Standard library only. Existing destinations are refused. Does not install ROS,
modify robot workspaces, start processes, or issue commands to a robot.
"""
import argparse
import hashlib
import html
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


def prepare(destination):
    root=Path(__file__).resolve().parents[1]
    target=Path(destination).resolve()
    target.mkdir(parents=True,exist_ok=False)
    assets=target/'assets';assets.mkdir()
    profile=target/'robot_profile';profile.mkdir()
    (target/'results').mkdir()
    source_assets=root/'simulation/gcu_ground_floor'
    for name in ['gcu_ground_floor.sdf','gcu_ground_floor.yaml','gcu_ground_floor.pgm','amcl.yaml','readiness_report.json']:
        shutil.copy2(source_assets/name,assets/name)
    sdf=ET.parse(assets/'gcu_ground_floor.sdf')
    engines=sdf.findall('.//plugin/render_engine')
    if len(engines)!=1:raise ValueError('Expected one Sensors render engine')
    engines[0].text='ogre';sdf.write(assets/'gcu_ground_floor_ogre.sdf',encoding='utf-8',xml_declaration=True)
    gz=root/'lidarbot_ws/src/lidarbot/lidarbot_gz'
    shutil.copy2(gz/'config/controllers_sim.yaml',assets/'controllers_sim.yaml')
    for name in ['lidarbot_gz_core.xacro','lidarbot_gz.urdf.xacro','sensors.xacro','gcu_evaluation.urdf.xacro']:
        text=(gz/'urdf'/name).read_text()
        text=text.replace('$(find lidarbot_gz)/config/controllers_sim.yaml',html.escape(str(assets/'controllers_sim.yaml'),quote=False))
        (profile/name).write_text(text)
    for name in ['gcu_amcl.launch.py','gcu_amcl_initial_pose.py','check_gcu_stationary.py','check_gcu_amcl.py','check_gcu_motion.py','check_gcu_recovery_watchdog.py','run_gcu_phase3_suite.py']:
        shutil.copy2(root/'scripts'/name,target/name)
    manifest={'purpose':'isolated GCU Phase 03 acceptance workspace','destination':str(target),
        'files_sha256':{str(f.relative_to(target)):hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(target.rglob('*')) if f.is_file()},
        'prerequisites':'Existing compatible ROS 2 Humble / Fortress robot installation, Ubuntu desktop session, NumPy, Pillow, PyYAML'}
    (target/'workspace_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--destination',type=Path,required=True)
    args=parser.parse_args()
    result=prepare(args.destination)
    print('Prepared '+result['destination'])
