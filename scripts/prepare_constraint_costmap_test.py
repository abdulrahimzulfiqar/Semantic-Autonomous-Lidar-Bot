#!/usr/bin/env python3
"""Prepare a synthetic native KeepoutFilter test, with no controller launch."""
import argparse,json,hashlib,shutil
from pathlib import Path


def prepare(destination):
    target=Path(destination);target.mkdir(parents=True,exist_ok=False);root=Path(__file__).resolve().parents[1]
    for name in ['check_constraint_costmaps_ros.py','osmag_constraint_mask.py','gcu_plan_validation.py',
                 'run_constraint_costmap_test.py','gcu_process_audit.py']:
        shutil.copy2(root/'scripts'/name,target/name)
    import yaml
    common={'use_sim_time':False,'global_frame':'map','robot_base_frame':'base_footprint','robot_radius':.18,
        'footprint_padding':.02,'resolution':.05,'transform_tolerance':.3,'update_frequency':10.,'publish_frequency':10.,
        'always_send_full_costmap':True,'track_unknown_space':True,'plugins':['static_layer','inflation_layer'],
        'static_layer':{'plugin':'nav2_costmap_2d::StaticLayer','map_topic':'/map','map_subscribe_transient_local':True},
        'inflation_layer':{'plugin':'nav2_costmap_2d::InflationLayer','inflation_radius':.35,'cost_scaling_factor':5.},
        'filters':['mission_keepout'],'mission_keepout':{'plugin':'nav2_costmap_2d::KeepoutFilter','enabled':True,'filter_info_topic':'/constraint_filter_info'}}
    cfg={'planner_server':{'ros__parameters':{'use_sim_time':False,'planner_plugins':['GridBased'],
        'GridBased':{'plugin':'nav2_navfn_planner/NavfnPlanner','tolerance':0.,'use_astar':True,'allow_unknown':False}}},
        'global_costmap':{'global_costmap':{'ros__parameters':common}},
        'costmap':{'costmap':{'ros__parameters':{**common,'global_frame':'odom','rolling_window':True,'width':6,'height':6}}}}
    class PlainDumper(yaml.SafeDumper):
        def ignore_aliases(self,data):return True
    # Humble's rcl YAML parser rejects aliases, even though PyYAML accepts them.
    (target/'params.yaml').write_text(yaml.dump(cfg,Dumper=PlainDumper,sort_keys=False))
    (target/'manifest.json').write_text(json.dumps({'scope':'Synthetic native plan-only filters; no controller',
        'files_sha256':{f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(target.iterdir()) if f.is_file()}},indent=2)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--destination',type=Path,required=True);prepare(p.parse_args().destination)
