#!/usr/bin/env python3
"""Copy 06B1 into a new isolated mock-only workspace; starts no processes."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def prepare(destination):
    root=Path(__file__).resolve().parents[1];target=Path(destination)
    target.mkdir(parents=True,exist_ok=False);(target/'assets').mkdir()
    for name in ['gcu_mission_mock_adapter.py','check_gcu_mission_ros.py','run_gcu_mission_mock_test.py',
                 'osmag_mission_contract.py','osmag_mission_state.py','osmag_paper_planner.py',
                 'osmag_hierarchy.py','osmag_route_refinement.py','gcu_plan_validation.py',
                 'gcu_navigation_metrics.py','audit_gcu_interfaces.py','gcu_process_audit.py']:
        shutil.copy2(root/'scripts'/name,target/name)
    for name in ['gcu_ground_floor.osm','gcu_ground_floor.pgm','gcu_ground_floor.yaml',
                 'regions_pixels.geojson','readiness_report.json']:
        shutil.copy2(root/'simulation/gcu_ground_floor'/name,target/'assets'/name)
    data={'scope':'06B1 mock-only ROS; real navigation action and controllers unavailable',
          'files_sha256':{str(f.relative_to(target)):hashlib.sha256(f.read_bytes()).hexdigest()
                          for f in sorted(target.rglob('*')) if f.is_file()}}
    (target/'manifest.json').write_text(json.dumps(data,indent=2)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--destination',type=Path,required=True)
    prepare(p.parse_args().destination)
