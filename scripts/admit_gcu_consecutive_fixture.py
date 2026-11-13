#!/usr/bin/env python3
"""Offline admission of two consecutive destinations; the second spawn is a proof only.

Runtime replans from live TF without moving/respawning the robot to that pose.
"""
import argparse,json
from pathlib import Path
from admit_gcu_mission_fixture import admit
from gcu_mission_fixture import load_fixture


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--assets',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--name',required=True)
    for name in ['spawn','via','goal']:p.add_argument('--'+name,nargs=3,type=float,required=True)
    p.add_argument('--sigma-hit',type=float)
    p.add_argument('--imu-yaw-rate',action='store_true')
    p.add_argument('--align-local-grid',action='store_true')
    a=p.parse_args()
    if a.output.exists():p.error('output already exists')
    first=admit(a.assets,a.name+'_out',a.spawn,a.via,simulation_timeout_s=80,checker_wall_timeout_s=500,local_costmap_frame='map')
    second=admit(a.assets,a.name+'_back',a.via,a.goal,simulation_timeout_s=80,checker_wall_timeout_s=500,local_costmap_frame='map')
    first['local_costmap_publish_hz']=10.;second['local_costmap_publish_hz']=10.
    if a.sigma_hit is not None:first['amcl_sigma_hit']=a.sigma_hit;second['amcl_sigma_hit']=a.sigma_hit
    if a.imu_yaw_rate:first['fuse_imu_yaw_rate']=True;second['fuse_imu_yaw_rate']=True
    if a.align_local_grid:first['align_local_grid']=True;second['align_local_grid']=True
    first['next_mission']=second;first['scope']='Offline admitted two-destination session; live success not yet established'
    with a.output.open('x') as out:out.write(json.dumps(first,indent=2)+'\n')
    load_fixture(a.output)
    print(json.dumps({'names':[first['name'],second['name']],'passages':[first['expected_passages'],second['expected_passages']]},indent=2))


if __name__=='__main__':main()
