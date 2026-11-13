#!/usr/bin/env python3
"""Freeze a stationary GCU sensor fixture locally, or prepare an isolated VM copy.

Never changes the accepted workspace. No navigator or mission execution starts.
"""
import argparse,hashlib,json,shutil
from pathlib import Path


def fixture(assets,output):
    import numpy as np
    from osmag_paper_planner import PaperPlanner
    from osmag_constraint_mask import build_mask
    output.mkdir(parents=True,exist_ok=False);planner=PaperPlanner(assets)
    start=[-.05864395131342248,-12.960313240266368];goal=[start[0]+1.5,start[1]]
    route=planner.plan(start,goal)
    incidence={n:(t['osmAG:from'],t['osmAG:to']) for n,t in planner.passages.items()}
    snap=build_mask(planner.grid,planner.resolution,planner.origin,planner.leaves,planner.passage_segments,incidence,
        allowed_areas=route['area_ids'],open_passages=route['passage_ids'],disabled_passages=[],robot_pose=start,mission_id='06b2a_stationary')
    # A declared test-only exclusion, away from the reference route, makes
    # local-filter verification non-vacuous within the 6 m rolling window.
    from gcu_plan_validation import validate_path
    from shapely.geometry import LineString,Point
    patch=None
    for dx,dy in [(1.,-1.),(1.,1.),(-1.,-1.),(-1.,1.)]:
        center=[start[0]+dx,start[1]+dy]
        if validate_path(snap.combined,snap.resolution,snap.origin,[center],.4)['valid'] and LineString(route['path']).distance(Point(center))>.6:
            patch=center;break
    if patch is None:raise RuntimeError('no_valid_stationary_probe_patch')
    rows,cols=np.mgrid[:snap.mask.shape[0],:snap.mask.shape[1]]
    x=snap.origin[0]+(cols+.5)*snap.resolution;y=snap.origin[1]+(rows+.5)*snap.resolution
    selected=(abs(x-patch[0])<=.25)&(abs(y-patch[1])<=.25)
    updated=snap.mask.copy();updated[selected]=100
    combined=np.where((planner.grid==0)&(updated==0),0,np.where(planner.grid<0,-1,100))
    if not validate_path(combined,snap.resolution,snap.origin,route['path'],.20)['valid']:raise RuntimeError('probe_patch_blocks_reference_route')
    np.savez_compressed(output/'fixture.npz',mask=snap.mask,mask_after_refresh=updated,physical=planner.grid,combined=snap.combined,combined_after_refresh=combined)
    (output/'fixture.json').write_text(json.dumps({'snapshot':snap.report(),'start':start,'goal':goal,
        'route':route,'inputs':planner.fingerprints(),'test_keepout':{'center':patch,'half_width':.25,'newly_forbidden_cells':int(np.sum(selected&(snap.mask==0))),
        'scope':'Stationary fixture-only off-route logical exclusion, not a CAD wall or inferred door closure.'}},indent=2)+'\n')


def prepare(baseline,fixture_dir,destination):
    import yaml
    destination.mkdir(parents=True,exist_ok=False)
    for directory in ['assets','support_install']:
        shutil.copytree(baseline/directory,destination/directory)
    for name in ['gcu_phase5_sim.launch.py','gcu_amcl.launch.py','gcu_amcl_initial_pose.py',
                 'wait_gcu_localization.py','gcu_process_audit.py']:
        shutil.copy2(baseline/name,destination/name)
    for path in (destination/'assets').rglob('*'):
        if path.is_file() and path.suffix in ['.sdf','.urdf','.yaml']:
            path.write_text(path.read_text().replace(str(baseline),str(destination)))
    for name in ['check_gcu_sensor_constraints.py','run_gcu_sensor_constraints.py','gcu_costmap_observation.py',
                 'osmag_constraint_mask.py','gcu_plan_validation.py']:
        shutil.copy2(Path(__file__).parent/name,destination/name)
    shutil.copytree(fixture_dir,destination/'fixture')
    cfg=yaml.safe_load((baseline/'assets/nav2_autonomous.yaml').read_text())
    for name in ['global_costmap','local_costmap']:
        params=cfg[name][name]['ros__parameters'];params['filters']=['mission_keepout']
        params['mission_keepout']={'plugin':'nav2_costmap_2d::KeepoutFilter','enabled':True,'filter_info_topic':'/osmag/constraint_filter_info'}
        params['publish_frequency']=5.0
        params['obstacle_layer']['scan']['expected_update_rate']=.5
    class PlainDumper(yaml.SafeDumper):
        def ignore_aliases(self,data):return True
    (destination/'assets/nav2_sensor_constraints.yaml').write_text(yaml.dump(cfg,Dumper=PlainDumper,sort_keys=False))
    files={str(p.relative_to(destination)):hashlib.sha256(p.read_bytes()).hexdigest() for p in destination.rglob('*') if p.is_file()}
    (destination/'sensor_manifest.json').write_text(json.dumps({'baseline':str(baseline),'scope':'Stationary real sensors/costmaps only; no navigator', 'files_sha256':files},indent=2)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='command',required=True)
    f=sub.add_parser('fixture');f.add_argument('--assets',type=Path,required=True);f.add_argument('--output',type=Path,required=True)
    b=sub.add_parser('prepare');b.add_argument('--baseline',type=Path,required=True);b.add_argument('--fixture',type=Path,required=True);b.add_argument('--destination',type=Path,required=True)
    a=p.parse_args()
    if a.command=='fixture':fixture(a.assets,a.output)
    else:prepare(a.baseline,a.fixture,a.destination)
