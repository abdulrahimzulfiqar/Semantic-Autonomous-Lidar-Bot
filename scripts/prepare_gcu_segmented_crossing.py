#!/usr/bin/env python3
"""Prepare a fresh 06B2B VM workspace; never start or modify installed robots."""
import argparse,hashlib,json,shutil
import xml.etree.ElementTree as ET
import copy
from pathlib import Path


def prepare(baseline,source_assets,destination,fixture_path=None):
    import yaml
    source=Path(__file__).parent
    fixture=None
    if fixture_path:
        from gcu_mission_fixture import load_fixture,verify_runtime_sources
        fixture,_=load_fixture(fixture_path)
    shutil.copytree(baseline,destination,ignore=shutil.ignore_patterns('results','*manifest*.json','__pycache__'))
    (destination/'results').mkdir()
    for path in (destination/'assets').rglob('*'):
        if path.is_file() and path.suffix in ['.sdf','.urdf','.yaml']:path.write_text(path.read_text().replace(str(baseline),str(destination)))
    for name in ['gcu_segmented_executor.py','check_gcu_segmented_crossing.py','run_gcu_segmented_crossing.py',
                 'osmag_mission_contract.py','osmag_mission_state.py','osmag_paper_planner.py','osmag_hierarchy.py',
                 'osmag_route_refinement.py','audit_gcu_interfaces.py','gcu_navigation_metrics.py','gcu_quiescent_shutdown.py','gcu_mission_fixture.py',
                 'gcu_costmap_observation.py','osmag_constraint_mask.py','gcu_plan_validation.py','osmag_execution_route.py','gcu_cancel_metrics.py','gcu_closure.py','gcu_physical_obstacle.py','gcu_metric_baseline.py','gcu_user_mission.py','check_gcu_review_interface.py']:
        shutil.copy2(source/name,destination/name)
    if fixture:verify_runtime_sources(fixture,destination)
    if fixture and fixture.get('fuse_imu_yaw_rate'):
        # Keep wheel translation, replace encoder yaw rate with measured IMU
        # angular velocity. Absolute IMU orientation is deliberately unused.
        overrides={'odom0_config':[False]*6+[True,True,False,False,False,False]+[False]*3,
            'imu0':'/imu_broadcaster/imu','imu0_config':[False]*11+[True]+[False]*3,
            'imu0_differential':False,'imu0_relative':False,'imu0_queue_size':20}
        launch=destination/'gcu_amcl.launch.py';code=launch.read_text();needle="False, False, False]}], output='screen'),"
        if code.count(needle)!=1:raise ValueError('unexpected_ekf_launch_template')
        code=code.replace(needle,"False, False, False]}, "+repr(overrides)+"], output='screen'),")
        import ast
        ast.parse(code);launch.write_text(code)
        # Fortress requires an IMU system in the world as well as a sensor
        # declaration and ROS bridge. Sensors alone does not publish IMU data.
        world_path=destination/'assets/contact_v2/world.sdf';world_tree=ET.parse(world_path);world_node=world_tree.getroot().find('world')
        if not any('imu-system' in p.get('filename','') for p in world_node.findall('plugin')):
            ET.SubElement(world_node,'plugin',filename='ignition-gazebo-imu-system',name='ignition::gazebo::systems::Imu')
            world_tree.write(world_path,encoding='unicode')
    for name in ['gcu_ground_floor.osm','regions_pixels.geojson']:
        shutil.copy2(source_assets/name,destination/'assets'/name)
    contact=destination/'assets/contact_v2';robot=ET.parse(contact/'robot.sdf')
    if fixture and fixture.get('physical_obstacle'):
        box=fixture['physical_obstacle'];x,y=box['center'];sx,sy,sz=box['size']
        world_path=contact/'world.sdf';tree=ET.parse(world_path);node=ET.SubElement(tree.getroot().find('world'),'model',name='test_obstacle_detour')
        ET.SubElement(node,'static').text='true';ET.SubElement(node,'pose').text=f'{x} {y} {sz/2} 0 0 0'
        link=ET.SubElement(node,'link',name='link')
        for tag in ['collision','visual']:
            entry=ET.SubElement(link,tag,name=tag);ET.SubElement(ET.SubElement(ET.SubElement(entry,'geometry'),'box'),'size').text=f'{sx} {sy} {sz}'
        tree.write(world_path,encoding='unicode')
    collisions=[]
    for link in robot.findall('./model/link'):
        for sensor in link.findall('sensor'):
            if sensor.get('type')!='contact':continue
            collision=sensor.findtext('contact/collision');topic=sensor.findtext('topic')
            if collision not in [c.get('name') for c in link.findall('collision')] or not topic or not topic.startswith('/gcu/contacts/'):
                raise ValueError('invalid_instrumented_collision')
            collisions.append({'link':link.get('name'),'collision':collision,'topic':topic})
    if len(collisions)!=13 or len({v['topic'] for v in collisions})!=13:raise ValueError('contact_contract_changed')
    (contact/'contact_manifest.json').write_text(json.dumps({'collisions':collisions,'source':'Existing instrumented SDF, unchanged collisions'},indent=2)+'\n')
    spawn=fixture['spawn'] if fixture else [-.5,-12.960313240266368,0.]
    sim=(destination/'gcu_phase5_sim.launch.py').read_text()
    for before,after in [("'-x', '-0.05864395131342248'", "'-x', "+repr(str(spawn[0]))),
                         ("'-y', '-12.960313240266368'", "'-y', "+repr(str(spawn[1]))),("'-Y', '0'", "'-Y', "+repr(str(spawn[2])))]:
        if sim.count(before)!=1:raise ValueError('unexpected_spawn_template')
        sim=sim.replace(before,after)
    (destination/'gcu_phase6_sim.launch.py').write_text(sim)
    class PlainDumper(yaml.SafeDumper):
        def ignore_aliases(self,data):return True
    amcl=yaml.safe_load((destination/'assets/amcl.yaml').read_text());prior=amcl['amcl']['ros__parameters']['initial_pose'];prior.update(x=spawn[0],y=spawn[1],yaw=spawn[2])
    if fixture and 'amcl_sigma_hit' in fixture:amcl['amcl']['ros__parameters']['sigma_hit']=fixture['amcl_sigma_hit']
    (destination/'assets/amcl.yaml').write_text(yaml.dump(amcl,Dumper=PlainDumper,sort_keys=False))
    params=yaml.safe_load((destination/'assets/nav2_sensor_constraints.yaml').read_text())
    if fixture and fixture.get('align_local_grid'):
        import numpy as np
        raster=yaml.safe_load((destination/'assets/gcu_ground_floor.yaml').read_text())
        local=params['local_costmap']['local_costmap']['ros__parameters']
        local.update(resolution=float(np.float32(raster['resolution'])),origin_x=raster['origin'][0],origin_y=raster['origin'][1])
    if fixture and 'local_costmap_frame' in fixture:
        params['local_costmap']['local_costmap']['ros__parameters']['global_frame']=fixture['local_costmap_frame']
    if fixture and 'local_costmap_publish_hz' in fixture:
        params['local_costmap']['local_costmap']['ros__parameters']['publish_frequency']=fixture['local_costmap_publish_hz']
    if fixture and fixture.get('inflate_constraints',False):
        for name in ['global_costmap','local_costmap']:
            costmap=params[name][name]['ros__parameters']
            costmap['filters']=['mission_keepout','constraint_inflation']
            costmap['constraint_inflation']=copy.deepcopy(costmap['inflation_layer'])
    bt=params['bt_navigator']['ros__parameters'];bt['default_nav_to_pose_bt_xml']=str(destination/'assets/navigate_strict.xml');bt['default_nav_through_poses_bt_xml']=str(destination/'assets/navigate_through_strict.xml')
    (destination/'assets/nav2_sensor_constraints.yaml').write_text(yaml.dump(params,Dumper=PlainDumper,sort_keys=False))
    # This full stack has more participants than CycloneDDS's default automatic
    # index range. Keep loopback-only discovery, with enough explicit indices.
    (destination/'assets/cyclonedds_local.xml').write_text('''<CycloneDDS><Domain id="any"><General><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>\n''')
    if fixture_path:shutil.copy2(fixture_path,destination/'assets/mission_fixture.json')
    files={str(p.relative_to(destination)):hashlib.sha256(p.read_bytes()).hexdigest() for p in destination.rglob('*') if p.is_file()}
    (destination/'crossing_manifest.json').write_text(json.dumps({'scope':'06B2B single simulation crossing; explicit execute required','baseline':str(baseline),
        'spawn':spawn,'goal':fixture['goal'] if fixture else [1.4413560486865775,-12.960313240266368,0.],
        'simulation_timeout_s':fixture['simulation_timeout_s'] if fixture else 80,'checker_wall_timeout_s':fixture['checker_wall_timeout_s'] if fixture else 420,'global_grid_age_s':.75,'local_grid_age_s':.5,
        'files_sha256':files},indent=2)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--baseline',type=Path,required=True);p.add_argument('--source-assets',type=Path,required=True);p.add_argument('--destination',type=Path,required=True);p.add_argument('--fixture',type=Path);a=p.parse_args()
    prepare(a.baseline,a.source_assets,a.destination,a.fixture)
