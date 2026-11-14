#!/usr/bin/env python3
"""Freeze an isolated GCU rolling-map plan-only workspace; no motion runner."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np
import yaml
from osmag_paper_planner import PaperPlanner


def prepare(assets,destination):
    destination.mkdir(parents=True,exist_ok=False)
    planner=PaperPlanner(assets,execution_padding=.02)
    start=[-.5,-12.960313240266368];goal=[10.38303756422381,-5.388606412064207]
    route=planner.plan(start,goal)
    fixture=dict(start=start,goal=goal,route=route,resolution=float(np.float32(planner.resolution)),
        anchor=planner.origin,shape=[128,128],window_distances=[0.,2.5,5.],
        closure_passage='-93',scope='Synthetic TF poses; no controller, physics or motion')
    (destination/'fixture.json').write_text(json.dumps(fixture,indent=2)+'\n')
    shutil.copy2(assets/'gcu_ground_floor.osm',destination/'geometry.osm')
    for name in ['osmag_rolling_raster.py','osmag_window_goal.py','osmag_window_admission.py','osmag_constraint_mask.py',
                 'gcu_plan_validation.py','gcu_process_audit.py','check_gcu_rolling_native.py','run_gcu_rolling_native.py']:
        shutil.copy2(Path(__file__).parent/name,destination/name)
    common=dict(use_sim_time=False,global_frame='map',robot_base_frame='base_footprint',
        robot_radius=.18,footprint_padding=.02,resolution=fixture['resolution'],
        transform_tolerance=.3,update_frequency=10.,publish_frequency=10.,
        always_send_full_costmap=True,track_unknown_space=True,
        plugins=['static_layer','inflation_layer'],
        static_layer=dict(plugin='nav2_costmap_2d::StaticLayer',map_topic='/osmag/planning_window',
                          map_subscribe_transient_local=True),
        inflation_layer=dict(plugin='nav2_costmap_2d::InflationLayer',inflation_radius=.35,cost_scaling_factor=5.),
        filters=['mission_keepout','constraint_inflation'],
        mission_keepout=dict(plugin='nav2_costmap_2d::KeepoutFilter',enabled=True,filter_info_topic='/osmag/window_filter_info'),
        constraint_inflation=dict(plugin='nav2_costmap_2d::InflationLayer',inflation_radius=.35,cost_scaling_factor=5.))
    local=copy.deepcopy(common);local.update(rolling_window=True,width=6,height=6)
    planner=dict(use_sim_time=False,planner_plugins=['GridBased'],
        GridBased=dict(plugin='nav2_navfn_planner/NavfnPlanner',tolerance=0.,use_astar=True,allow_unknown=False))
    # A planner owns the local-style costmap thread/lifecycle. The standalone
    # costmap executable failed teardown in retained trials; no controller or
    # velocity publisher is needed for this costmap-consumption fixture.
    cfg=dict(planner_server={'ros__parameters':planner},
        global_costmap={'global_costmap':{'ros__parameters':common}})
    cfg['/window_local/local_planner']={'ros__parameters':copy.deepcopy(planner)}
    cfg['/window_local/global_costmap/global_costmap']={'ros__parameters':local}
    class Dumper(yaml.SafeDumper):
        def ignore_aliases(self,data):return True
    (destination/'params.yaml').write_text(yaml.dump(cfg,Dumper=Dumper,sort_keys=False))
    files={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in destination.iterdir() if p.is_file()}
    (destination/'manifest.json').write_text(json.dumps(dict(scope=fixture['scope'],files_sha256=files),indent=2)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--assets',type=Path,default=Path('simulation/gcu_ground_floor'))
    p.add_argument('--destination',type=Path,required=True);a=p.parse_args();prepare(a.assets,a.destination)
