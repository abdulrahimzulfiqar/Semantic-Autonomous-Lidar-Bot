#!/usr/bin/env python3
"""Prepare a NEW GCU planning and autonomous-test workspace from the repository, on Ubuntu.

Requires the existing compatible robot installation and rmw_cyclonedds_cpp.
Does not start processes.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
from prepare_gcu_phase3_workspace import prepare


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--destination',type=Path,required=True);args=parser.parse_args()
    prepare(args.destination)
    root=Path(__file__).resolve().parents[1];target=args.destination.resolve()
    for name in ['nav2_plan_only.yaml','nav2_autonomous.yaml','navigation_acceptance.json','navigate_strict.xml','navigate_through_strict.xml']:
        shutil.copy2(root/'simulation/gcu_ground_floor'/name,target/'assets'/name)
    for name in ['gcu_plan_only.launch.py','gcu_goal_preview.py','gcu_plan_validation.py','gcu_navigation_metrics.py','check_gcu_plan_only.py','run_gcu_plan_only_test.py','wait_gcu_localization.py','gcu_navigation.launch.py','gcu_navigation_goal.py','gcu_contact_sim.launch.py','prepare_gcu_contact_assets.py','prepare_gcu_navigation_fixtures.py','check_gcu_navigation.py','run_gcu_navigation_test.py','run_gcu_navigation_suite.py']:
        shutil.copy2(root/'scripts'/name,target/name)
    manifest={'purpose':'GCU metric preview plus explicitly enabled simulation-only autonomous tests',
        'files_sha256':{str(p.relative_to(target)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(target.rglob('*')) if p.is_file()}}
    (target/'phase4_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('Prepared '+str(target))


if __name__=='__main__':main()
