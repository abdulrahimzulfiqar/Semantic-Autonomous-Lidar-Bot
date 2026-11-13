#!/usr/bin/env python3
"""Admit one GCU simulation query only after fresh topology and padded-mask checks.

No ROS imports, simulator launch or motion. Existing fixtures are never overwritten.
"""
import argparse,hashlib,json
from pathlib import Path
from gcu_mission_fixture import load_fixture,validate_current_contract
from osmag_paper_planner import PaperPlanner,length
from osmag_mission_contract import make_contract
from osmag_constraint_mask import build_mask


def admit(assets,name,spawn,goal,minimum_passages=1,simulation_timeout_s=150,checker_wall_timeout_s=700,local_costmap_frame='odom',constraint_aware_route=False):
    planner=PaperPlanner(assets,execution_padding=.02 if constraint_aware_route else None)
    contract=make_contract(planner,spawn[:2],goal[:2],goal[2],mission_id=name,now=1.,pose_stamp=1.)
    if len(contract.passage_ids)<minimum_passages:raise ValueError('insufficient_distinct_passages')
    incidence={n:(v['osmAG:from'],v['osmAG:to']) for n,v in planner.passages.items()}
    snapshot=build_mask(planner.grid,planner.resolution,planner.origin,planner.leaves,planner.passage_segments,incidence,
        allowed_areas=contract.area_ids,open_passages=contract.passage_ids,disabled_passages=[],robot_pose=spawn[:2],mission_id=name)
    validation=validate_current_contract(contract,snapshot)
    source=Path(__file__).parent
    pinned=['gcu_segmented_executor.py','osmag_mission_contract.py','osmag_mission_state.py','check_gcu_segmented_crossing.py',
        'gcu_mission_fixture.py','gcu_costmap_observation.py','osmag_constraint_mask.py','gcu_plan_validation.py','osmag_paper_planner.py','osmag_execution_route.py','gcu_cancel_metrics.py','gcu_closure.py','gcu_physical_obstacle.py','gcu_metric_baseline.py','gcu_user_mission.py']
    return dict(name=name,spawn=spawn,goal=goal,expected_passages=contract.passage_ids,expected_areas=contract.area_ids,
        route_length_m=length(contract.path),simulation_timeout_s=simulation_timeout_s,checker_wall_timeout_s=checker_wall_timeout_s,
        mask_validation=validation,route_contract=contract.report(),input_sha256=planner.fingerprints(),inflate_constraints=True,
        check_handoff_rest=True,local_costmap_frame=local_costmap_frame,constraint_aware_route=constraint_aware_route,execution_source_sha256={n:hashlib.sha256((source/n).read_bytes()).hexdigest() for n in pinned},
        scope='Offline admitted simulation fixture; runtime acceptance requires a separate scored trial')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--assets',type=Path,required=True);p.add_argument('--name',required=True)
    p.add_argument('--spawn',type=float,nargs=3,required=True);p.add_argument('--goal',type=float,nargs=3,required=True)
    p.add_argument('--minimum-passages',type=int,default=1);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--local-costmap-frame',choices=['map','odom'],default='odom')
    p.add_argument('--constrain-route',action='store_true');p.add_argument('--imu-yaw-rate',action='store_true');p.add_argument('--align-local-grid',action='store_true')
    p.add_argument('--obstacle-box',type=float,nargs=5,metavar=('X','Y','SX','SY','SZ'))
    a=p.parse_args()
    if a.minimum_passages<1:p.error('--minimum-passages must be positive')
    if a.output.exists():p.error('output already exists')
    result=admit(a.assets,a.name,a.spawn,a.goal,a.minimum_passages,local_costmap_frame=a.local_costmap_frame,constraint_aware_route=a.constrain_route)
    if a.imu_yaw_rate:result['fuse_imu_yaw_rate']=True
    if a.align_local_grid:result['align_local_grid']=True
    if a.imu_yaw_rate or a.align_local_grid:result['local_costmap_publish_hz']=10.
    if a.obstacle_box:
        from gcu_physical_obstacle import overlay_box
        from gcu_plan_validation import validate_path
        box={'center':a.obstacle_box[:2],'size':a.obstacle_box[2:]};planner=PaperPlanner(a.assets)
        overlay=overlay_box(planner.grid,planner.resolution,planner.origin,box)
        for point in [a.spawn[:2],a.goal[:2]]:
            if not validate_path(overlay,planner.resolution,planner.origin,[point],.20)['valid']:raise ValueError('obstacle_blocks_endpoint')
        if validate_path(overlay,planner.resolution,planner.origin,result['route_contract']['path'],.20)['valid']:raise ValueError('obstacle_does_not_require_detour')
        result['physical_obstacle']=box
    with a.output.open('x') as out:out.write(json.dumps(result,indent=2)+'\n')
    load_fixture(a.output)
    print(json.dumps({k:result[k] for k in ['name','expected_passages','route_length_m','mask_validation']},indent=2))


if __name__=='__main__':main()
