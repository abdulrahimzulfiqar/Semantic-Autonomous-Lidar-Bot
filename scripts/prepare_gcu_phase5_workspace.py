#!/usr/bin/env python3
"""Prepare a new isolated Phase 05 workspace; keep Phase 04 evidence/runtime intact."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys


def prepare(destination):
    source = Path(__file__).resolve().parent
    subprocess.run([sys.executable, str(source/'prepare_gcu_phase4_workspace.py'),
                    '--destination', str(destination)], check=True)
    target = Path(destination).resolve()
    shutil.copy2(source.parent/'simulation/gcu_ground_floor/nav2_handoff.yaml',
                 target/'assets/nav2_autonomous.yaml')
    shutil.copy2(source/'run_gcu_phase5_navigation_test.py', target/'run_gcu_navigation_test.py')
    for name in ['gcu_process_audit.py','check_gcu_truth_bridge.py','run_gcu_truth_comparison.py']:
        shutil.copy2(source/name,target/name)
    shutil.copy2(source/'gcu_phase5_sim.launch.py',target/'gcu_phase5_sim.launch.py')
    shutil.copy2(source/'run_gcu_phase5_navigation_suite.py',target/'run_gcu_navigation_suite.py')
    shutil.copytree(source.parent/'simulation/gcu_ground_floor/support/gcu_truth_bridge',target/'support_src')
    manifest = {'scope': 'Phase 05 candidate; upstream rotation shim and ordered bridge shutdown',
                'files_sha256': {str(p.relative_to(target)): hashlib.sha256(p.read_bytes()).hexdigest()
                                 for p in sorted(target.rglob('*')) if p.is_file()}}
    (target/'phase5_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return manifest


def build_support(workspace):
    target=Path(workspace).resolve()
    commands=[['cmake','-S',str(target/'support_src'),'-B',str(target/'support_build'),
               '-DCMAKE_BUILD_TYPE=Release','-DCMAKE_INSTALL_PREFIX='+str(target/'support_install')],
              ['cmake','--build',str(target/'support_build'),'--parallel','2'],
              ['cmake','--install',str(target/'support_build')]]
    for command in commands:subprocess.run(command,check=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--destination', required=True, type=Path)
    parser.add_argument('--build-support',action='store_true',help='Build evaluation bridge in an already sourced Humble/Fortress environment')
    args=parser.parse_args();prepare(args.destination)
    if args.build_support:build_support(args.destination)
