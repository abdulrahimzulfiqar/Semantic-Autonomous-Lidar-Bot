#!/usr/bin/env python3
"""Prepare isolated Pi/simulator staging trees without changing accepted sources."""
import argparse
import ast
import hashlib
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

if __package__:
    from .localizer_input_relay import RELAY_IMU, RELAY_ODOM
else:
    from localizer_input_relay import RELAY_IMU, RELAY_ODOM


def derive_executor(source):
    tree=ast.parse(source)
    original=[n for n in tree.body if not isinstance(n,ast.FunctionDef) or n.name!='simulation_guard']
    guards=[i for i,n in enumerate(tree.body) if isinstance(n,ast.FunctionDef) and n.name=='simulation_guard']
    if len(guards)!=1:raise ValueError('unexpected_executor_guard')
    tree.body[guards[0]]=ast.ImportFrom(module='profile_phase9',names=[ast.alias(name='simulation_guard')],level=0)
    derived=ast.unparse(ast.fix_missing_locations(tree))+'\n'
    checked=[n for n in ast.parse(derived).body if not isinstance(n,ast.ImportFrom) or n.module!='profile_phase9']
    if ast.dump(ast.Module(body=original,type_ignores=[]))!=ast.dump(ast.Module(body=checked,type_ignores=[])):
        raise ValueError('executor_behavior_changed_beyond_guard')
    return derived


def prepare(baseline, destination, pi_root, sim_root, pi_ip, sim_ip, session,
            clock_transport_hz=0., physics_real_time_factor=1., timing_trace=False,
            separate_timer=False, reliable_scan_link=False, dedicated_tf=False,
            capture_inputs=False, *, dedicated_mission_timer=False, dedicated_actor_clock=False,
            mask_on_revision=False, pipeline_trace=False, reliable_localizer_inputs=False,
            global_costmap_hz=None):
    if dedicated_actor_clock and not dedicated_tf:
        raise ValueError('dedicated_actor_clock_requires_dedicated_tf')
    timing_trace = timing_trace or pipeline_trace
    import yaml
    if not math.isfinite(physics_real_time_factor) or not 0<physics_real_time_factor<=1:
        raise ValueError('invalid_physics_real_time_factor')
    if not math.isfinite(clock_transport_hz) or not 0<=clock_transport_hz<=1000:
        raise ValueError('invalid_clock_transport_rate')
    if global_costmap_hz is not None and (not math.isfinite(global_costmap_hz) or global_costmap_hz <= 0):
        raise ValueError('invalid_global_costmap_rate')
    destination.mkdir(exist_ok=False)
    sources=Path(__file__).resolve().parent
    scripts=sources.parents[1]/'scripts'
    pi=destination/'pi';sim=destination/'simulator'
    pi.mkdir();(pi/'assets').mkdir()
    shutil.copytree(baseline,sim)
    # Baseline paths in accepted SDF/YAML are rewritten only in this derived copy.
    old='/home/ubuntu/edgeosmag_phase8_amcl_baseline_ws_01'
    for path in sim.rglob('*'):
        if path.is_file() and path.suffix in ['.py','.yaml','.sdf','.urdf','.xml']:
            path.write_text(path.read_text().replace(old,str(sim_root)))
    for name in ['gcu_ground_floor.osm','gcu_ground_floor.pgm','gcu_ground_floor.yaml',
                 'amcl.yaml','nav2_sensor_constraints.yaml','navigate_strict.xml',
                 'navigate_through_strict.xml','regions_pixels.geojson']:
        shutil.copy2(sim/'assets'/name,pi/'assets'/name)
    for name in ['gcu_amcl.launch.py','gcu_amcl_initial_pose.py']:
        shutil.copy2(baseline/name,pi/name)
    if reliable_localizer_inputs:
        launch_text = (pi/'gcu_amcl.launch.py').read_text()
        for before, after in [('/diff_controller/odom', RELAY_ODOM), ('/imu_broadcaster/imu', RELAY_IMU)]:
            if launch_text.count(repr(before)) != 1:
                raise ValueError('unexpected_localizer_input:'+before)
            launch_text = launch_text.replace(repr(before), repr(after))
        (pi/'gcu_amcl_reliable_inputs.launch.py').write_text(launch_text)
        for name in ['arrival_probe.py', 'localizer_input_relay.py']:
            shutil.copy2(sources/name, pi/name)
    shutil.copy2(scripts/'wait_gcu_localization.py',pi/'wait_gcu_localization.py')
    params=yaml.safe_load((pi/'assets/nav2_sensor_constraints.yaml').read_text())
    bt=params['bt_navigator']['ros__parameters']
    bt['default_nav_to_pose_bt_xml']=str(pi_root/'assets/navigate_strict.xml')
    bt['default_nav_through_poses_bt_xml']=str(pi_root/'assets/navigate_through_strict.xml')
    if global_costmap_hz is not None:
        global_costmap = params['global_costmap']['global_costmap']['ros__parameters']
        global_costmap['update_frequency'] = float(global_costmap_hz)
        global_costmap['publish_frequency'] = float(global_costmap_hz)
    (pi/'assets/nav2_sensor_constraints.yaml').write_text(yaml.safe_dump(params,sort_keys=False))
    files={'worker.py':'worker_phase9.py','resources.py':'resources_phase9.py',
           'simulation_profile.py':'profile_phase9.py','run_role.py':'run_role.py',
           'dispatch_admission.py':'dispatch_admission.py'}
    for before,after in files.items():shutil.copy2(sources/before,pi/after)
    if timing_trace:
        for name in ['arrival_probe.py','callback_trace.py']:shutil.copy2(sources/name,pi/name)
    if pipeline_trace:
        shutil.copy2(sources/'arrival_probe.py', sim/'arrival_probe.py')
    if separate_timer or dedicated_tf or dedicated_mission_timer:
        shutil.copy2(sources/'executor_groups.py',pi/'executor_groups.py')
    if capture_inputs:shutil.copy2(sources/'sensor_recording.py',sim/'sensor_recording.py')
    if reliable_scan_link:
        for folder in [pi,sim]:shutil.copy2(sources/'scan_link.py',folder/'scan_link.py')
    # Copy the transitive local imports of the existing execution components.
    pending=['osmag_paper_planner','gcu_segmented_executor','gcu_rolling_window_manager']
    seen=set()
    while pending:
        name=pending.pop()
        if name in seen:continue
        seen.add(name);text=(scripts/(name+'.py')).read_text()
        shutil.copy2(scripts/(name+'.py'),pi/(name+'.py'))
        for node in ast.walk(ast.parse(text)):
            if isinstance(node,ast.ImportFrom) and node.module and (scripts/(node.module+'.py')).is_file():pending.append(node.module)
    original=(pi/'gcu_segmented_executor.py').read_text()
    (pi/'gcu_segmented_executor.py').write_text(derive_executor(original))
    derivation=dict(original_sha256=hashlib.sha256(original.encode()).hexdigest(),
        derived_sha256=hashlib.sha256((pi/'gcu_segmented_executor.py').read_bytes()).hexdigest(),
        scope='Only simulation_guard definition replaced by explicit Phase 09 profile import; remaining AST identical')
    (pi/'executor_derivation.json').write_text(json.dumps(derivation,indent=2)+'\n')
    shutil.copy2(sources/'observer.py',sim/'observer_phase9.py')
    shutil.copy2(sources/'run_role.py',sim/'run_role.py')
    if clock_transport_hz:
        if not 0<clock_transport_hz<=1000:raise ValueError('invalid_clock_transport_rate')
        shutil.copy2(sources/'clock_transport.py',sim/'clock_transport.py')
    for name in ['gcu_process_audit.py','gcu_quiescent_shutdown.py']:
        shutil.copy2(scripts/name,sim/name)
    launch=(sim/'gcu_phase6_sim.launch.py').read_text()
    needle="'topics.navigation.topic':'cmd_vel'"
    if launch.count(needle)!=1:raise ValueError('unexpected_command_relay')
    launch=launch.replace(needle,"'topics.navigation.topic':'/phase9/sim_cmd_vel'")
    launch=launch.replace("        Node(package='ros_gz_image', executable='image_bridge', arguments=['/camera/image_raw']),\n",'')
    if clock_transport_hz:
        needle="'--ros-args','-p','config_file:='+str(bridge)"
        if launch.count(needle)!=1:raise ValueError('unexpected_clock_bridge_template')
        launch=launch.replace(needle,"'--ros-args','-r','/clock:=/phase9/raw_clock','-p','config_file:='+str(bridge)")
    if reliable_scan_link:
        needle="'-p','config_file:='+str(bridge)"
        if launch.count(needle)!=1:raise ValueError('unexpected_scan_bridge_template')
        launch=launch.replace(needle,"'-r','/scan:=/phase9/source_scan','-p','config_file:='+str(bridge)")
    (sim/'gcu_phase9_sim.launch.py').write_text(launch)
    robot=sim/'assets/contact_v2/robot.sdf';tree=ET.parse(robot)
    camera_removed=0
    for link in tree.findall('./model/link'):
        for sensor in list(link.findall('sensor')):
            if sensor.get('type')=='camera':link.remove(sensor);camera_removed+=1
    tree.write(robot,encoding='unicode')
    world=sim/'assets/contact_v2/world.sdf';world_tree=ET.parse(world)
    factor=world_tree.getroot().find('./world/physics/real_time_factor')
    if factor is None:raise ValueError('missing_physics_time_factor')
    factor.text=str(physics_real_time_factor)
    world_tree.write(world,encoding='unicode')
    for role,folder,ip in [('navigation_pi',pi,pi_ip),('simulator',sim,sim_ip)]:
        root=ET.Element('CycloneDDS');domain=ET.SubElement(root,'Domain',id='any')
        general=ET.SubElement(domain,'General');interfaces=ET.SubElement(general,'Interfaces')
        ET.SubElement(interfaces,'NetworkInterface',address=ip);ET.SubElement(general,'AllowMulticast').text='false'
        discovery=ET.SubElement(domain,'Discovery');ET.SubElement(discovery,'ParticipantIndex').text='auto'
        ET.SubElement(discovery,'MaxAutoParticipantIndex').text='32'
        peers=ET.SubElement(discovery,'Peers')
        for peer in [pi_ip,sim_ip]:ET.SubElement(peers,'Peer',Address=peer)
        ET.indent(root);(folder/'dds.xml').write_text(ET.tostring(root,encoding='unicode')+'\n')
        profile=dict(scope='gcu_phase9_processor_in_loop',role=role,domain=218,
            partition='gcu_phase9_'+session,pi_ip=pi_ip,simulator_ip=sim_ip,
            command_topic='/phase9/sim_cmd_vel',camera_sensors_removed=camera_removed,
            clock_transport_hz=clock_transport_hz,
            physics_real_time_factor=physics_real_time_factor,
            timing_trace=bool(timing_trace),
            pipeline_trace=bool(pipeline_trace),
            separate_mission_timer=bool(separate_timer),
            dedicated_mission_timer=bool(dedicated_mission_timer),
            dedicated_actor_clock=bool(dedicated_actor_clock),
            mask_on_revision=bool(mask_on_revision),
            reliable_scan_link=bool(reliable_scan_link),
            reliable_localizer_inputs=bool(reliable_localizer_inputs),
            dedicated_tf=bool(dedicated_tf),capture_inputs=bool(capture_inputs),
            dds_sha256=hashlib.sha256((folder/'dds.xml').read_bytes()).hexdigest())
        if global_costmap_hz is not None:
            profile['global_costmap_hz'] = global_costmap_hz
        (folder/'profile.json').write_text(json.dumps(profile,indent=2)+'\n')
        manifest_path = folder/'phase9_manifest.json'
        manifest={str(p.relative_to(folder)):hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in folder.rglob('*') if p.is_file() and p != manifest_path}
        manifest_path.write_text(json.dumps(dict(files_sha256=manifest),indent=2)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['baseline','destination','pi-root','sim-root']:p.add_argument('--'+name,type=Path,required=True)
    for name in ['pi-ip','sim-ip','session']:p.add_argument('--'+name,required=True)
    p.add_argument('--clock-transport-hz',type=float,default=0.)
    p.add_argument('--physics-real-time-factor',type=float,default=1.)
    p.add_argument('--global-costmap-hz', type=float, help='Override only the Pi global costmap update and publish rates')
    p.add_argument('--timing-trace',action='store_true')
    p.add_argument('--pipeline-trace',action='store_true')
    p.add_argument('--separate-mission-timer',action='store_true')
    p.add_argument('--dedicated-mission-timer',action='store_true')
    p.add_argument('--dedicated-actor-clock',action='store_true')
    p.add_argument('--mask-on-revision',action='store_true')
    p.add_argument('--reliable-scan-link',action='store_true')
    p.add_argument('--reliable-localizer-inputs',action='store_true')
    p.add_argument('--dedicated-tf',action='store_true')
    p.add_argument('--capture-inputs',action='store_true')
    a = p.parse_args()
    prepare(a.baseline, a.destination, a.pi_root, a.sim_root, a.pi_ip, a.sim_ip, a.session,
            clock_transport_hz=a.clock_transport_hz, physics_real_time_factor=a.physics_real_time_factor,
            timing_trace=a.timing_trace, separate_timer=a.separate_mission_timer,
            reliable_scan_link=a.reliable_scan_link, dedicated_tf=a.dedicated_tf,
            capture_inputs=a.capture_inputs, dedicated_mission_timer=a.dedicated_mission_timer,
            dedicated_actor_clock=a.dedicated_actor_clock, mask_on_revision=a.mask_on_revision,
            pipeline_trace=a.pipeline_trace, reliable_localizer_inputs=a.reliable_localizer_inputs,
            global_costmap_hz=a.global_costmap_hz)
