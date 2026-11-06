#!/usr/bin/env python3
"""Create a separate preview-only workspace; no installed robot files are modified."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess


def prepare(destination):
    root=Path(__file__).resolve().parents[1];target=Path(destination)
    target.mkdir(parents=True,exist_ok=False);assets=target/'assets';assets.mkdir()
    scripts=['gcu_osmag_route_preview.py','gcu_osmag_preview.launch.py','osmag_hierarchy.py',
             'osmag_paper_planner.py','osmag_route_refinement.py','audit_gcu_interfaces.py','gcu_plan_validation.py',
             'gcu_process_audit.py','check_gcu_osmag_preview.py','run_gcu_osmag_preview_test.py']
    for name in scripts:shutil.copy2(root/'scripts'/name,target/name)
    shutil.copytree(root/'simulation/gcu_ground_floor/support/gcu_tf_probe',target/'probe_src')
    for name in ['gcu_ground_floor.osm','gcu_ground_floor.yaml','gcu_ground_floor.pgm','regions_pixels.geojson','readiness_report.json']:
        shutil.copy2(root/'simulation/gcu_ground_floor'/name,assets/name)
    manifest={'backend':'paper_based_reference_python_not_author_runtime','files_sha256':{
        str(p.relative_to(target)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(target.rglob('*')) if p.is_file()}}
    (target/'preview_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--destination',type=Path,required=True)
    parser.add_argument('--build-probe',action='store_true')
    args=parser.parse_args();prepare(args.destination)
    if args.build_probe:
        for command in [['cmake','-S',str(args.destination/'probe_src'),'-B',str(args.destination/'probe_build'),
                         '-DCMAKE_INSTALL_PREFIX='+str(args.destination/'probe_install')],
                        ['cmake','--build',str(args.destination/'probe_build'),'--parallel','1'],
                        ['cmake','--install',str(args.destination/'probe_build')]]:subprocess.run(command,check=True)
