#!/usr/bin/env python3
"""One explicitly enabled, independently scored simulation crossing (06B2B)."""
import argparse,json,math,time
from dataclasses import replace
from pathlib import Path
import numpy as np
from gcu_plan_validation import validate_path
from gcu_navigation_metrics import rest_window,coverage
from gcu_costmap_observation import inspect
from osmag_paper_planner import PaperPlanner
from osmag_mission_contract import make_contract,CrossingMonitor
from osmag_constraint_mask import build_mask
from gcu_segmented_executor import create_executor,simulation_guard


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--workspace',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--execute',action='store_true');p.add_argument('--fixture',type=Path)
    p.add_argument('--diagnose-stationary',action='store_true',help='Capture consumption for at most 30 wall seconds; never dispatch a navigation action')
    p.add_argument('--cancel-near-handoff',action='store_true',help='One scored cancel-topic test before the first passage handoff')
    p.add_argument('--close-upcoming-passage',action='store_true',help='Close the first upcoming passage, stop, and require explicit no route')
    p.add_argument('--drop-required-imu',action='store_true',help='Stop forwarding real IMU samples to the executor; EKF input remains unchanged')
    p.add_argument('--metric-baseline',action='store_true',help='Send only the final Nav2 goal, with the same fixed scenario mask/profile')
    p.add_argument('--drop-window-source',action='store_true',help='Phase 07 raster heartbeat loss while moving')
    p.add_argument('--interactive',action='store_true');p.add_argument('--session-wall-timeout',type=float,default=900.)
    a=p.parse_args()
    fixture=None;fixture_sha=None
    if a.fixture:
        from gcu_mission_fixture import load_fixture,verify_runtime_sources
        fixture,fixture_sha=load_fixture(a.fixture)
        verify_runtime_sources(fixture,Path(__file__).parent)
        if fixture.get('next_mission') and a.diagnose_stationary:p.error('stationary diagnosis requires a single fixture')
    if a.cancel_near_handoff and (a.diagnose_stationary or not fixture or fixture.get('next_mission')):p.error('cancellation requires a single admitted motion fixture')
    if a.close_upcoming_passage and (a.cancel_near_handoff or a.diagnose_stationary or not fixture or fixture.get('next_mission')):p.error('closure requires one admitted motion fixture and no other scenario')
    if a.drop_required_imu and (a.cancel_near_handoff or a.close_upcoming_passage or a.diagnose_stationary or not fixture or fixture.get('next_mission') or not fixture.get('fuse_imu_yaw_rate')):p.error('IMU loss requires one admitted IMU-enabled motion fixture')
    if a.metric_baseline and (a.drop_required_imu or a.cancel_near_handoff or a.close_upcoming_passage or a.diagnose_stationary or not fixture):p.error('baseline requires an admitted normal-motion fixture')
    if a.interactive and (a.metric_baseline or a.drop_required_imu or a.cancel_near_handoff or a.close_upcoming_passage or a.diagnose_stationary or not fixture or fixture.get('next_mission') or not 0<a.session_wall_timeout<=1200):p.error('interactive mode requires one seed profile and a bounded session')
    rolling=bool(fixture and fixture.get('rolling_window'))
    if rolling and (a.metric_baseline or a.close_upcoming_passage or a.drop_required_imu or a.cancel_near_handoff or a.diagnose_stationary):p.error('rolling fixtures use their own window/fault mode')
    if a.drop_window_source and (not rolling or a.interactive or fixture.get('next_mission')):p.error('window loss requires one noninteractive rolling fixture')
    import os
    simulation_guard(a.execute,os.environ)
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
    from nav_msgs.msg import OccupancyGrid,Path as NavPath
    from nav2_msgs.msg import CostmapFilterInfo
    from nav2_msgs.action import ComputePathToPose
    from lifecycle_msgs.srv import ChangeState
    from rclpy.action import ActionClient
    from tf2_msgs.msg import TFMessage
    from ros_gz_interfaces.msg import Contacts
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import LaserScan,Imu
    from std_msgs.msg import String,Empty
    from geometry_msgs.msg import PoseStamped
    rclpy.init();planner=PaperPlanner(a.workspace/'assets',execution_padding=.02 if fixture and fixture.get('constraint_aware_route') else None);adapter=create_executor(planner,a.execute,require_imu=bool(fixture and fixture.get('fuse_imu_yaw_rate')),imu_topic='/gcu_fault/required_imu' if a.drop_required_imu else '/imu_broadcaster/imu')
    if fixture and fixture['input_sha256']!=planner.fingerprints():raise RuntimeError('fixture_map_changed')
    evaluator=Node('gcu_segmented_evaluator',parameter_overrides=[Parameter('use_sim_time',value=True)])
    # Contact evidence must not starve mission/scan callbacks. Each node retains
    # its mutually-exclusive group; TF has its own listener group.
    executor=MultiThreadedExecutor(num_threads=3);executor.add_node(adapter);executor.add_node(evaluator)
    grids={};truth=[];matched=[];commands=[];contacts=[];counts={};plans=[];subs=[];recording=False;begin=time.monotonic();last_match=-1.;rolling_manager=None
    recent_scans=[]
    forwarding_imu=True;imu_forwarding=[]
    if a.drop_required_imu:
        imu_relay=evaluator.create_publisher(Imu,'/gcu_fault/required_imu',qos_profile_sensor_data)
        def relay_imu(m):
            imu_forwarding.append({'stamp':m.header.stamp.sec+m.header.stamp.nanosec/1e9,'received_sim':adapter.sim(),'forwarded':forwarding_imu})
            if forwarding_imu:imu_relay.publish(m)
        subs.append(evaluator.create_subscription(Imu,'/imu_broadcaster/imu',relay_imu,qos_profile_sensor_data))
    def remember_scan(m):
        recent_scans.append(m);del recent_scans[:-10]
    subs.append(evaluator.create_subscription(LaserScan,'/scan',remember_scan,qos_profile_sensor_data))
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def stamp(s):return s.sec+s.nanosec/1e9
    def angle(q):return math.atan2(2*q.w*q.z,1-2*q.z*q.z)
    def truth_cb(m):
        if not recording:return
        for t in m.transforms:
            if t.child_frame_id.strip('/').split('::')[-1]!='lidarbot':continue
            s=stamp(t.header.stamp);v=t.transform
            if not truth or s>truth[-1]['sim_time']:truth.append({'sim_time':s,'truth':[v.translation.x,v.translation.y,angle(v.rotation)]})
    def contact_cb(m,topic):
        counts[topic]=counts.get(topic,0)+1
        if not recording:return
        for c in m.contacts:
            names=[c.collision1.name,c.collision2.name]
            if any('cad_walls' in n or 'obstacle' in n for n in names) or not all(names) or not (any(n.startswith('ground::') for n in names) or all(n.startswith('lidarbot::') for n in names)):
                contacts.append({'sim_time':stamp(m.header.stamp),'names':names,'topic':topic})
    for key,topic in [('global','/global_costmap/costmap'),('local','/local_costmap/costmap')]:
        subs.append(evaluator.create_subscription(OccupancyGrid,topic,lambda m,k=key:grids.update({k:m}),qos))
    sensor_queue=QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT)
    subs.append(evaluator.create_subscription(TFMessage,'/gcu_ground_truth',truth_cb,sensor_queue))
    subs.append(evaluator.create_subscription(Twist,'/cmd_vel',lambda m:commands.append({'sim_time':adapter.sim(),'v':m.linear.x,'w':m.angular.z}),100))
    subs.append(evaluator.create_subscription(NavPath,'/plan',lambda m:plans.append({'sim_time':adapter.sim(),'frame':m.header.frame_id,'points':[[p.pose.position.x,p.pose.position.y] for p in m.poses]}),10))
    if rolling:
        def followed_path(m):
            if rolling_manager is None or not m.poses:return
            try:
                v=adapter.buffer.lookup_transform('map',m.header.frame_id,Time.from_msg(m.header.stamp)).transform
                yaw=angle(v.rotation);c=math.cos(yaw);s=math.sin(yaw)
                rolling_manager.last_native_plan=[[v.translation.x+c*p.pose.position.x-s*p.pose.position.y,
                    v.translation.y+s*p.pose.position.x+c*p.pose.position.y] for p in m.poses]
                rolling_manager.last_native_plan_stamp=stamp(m.header.stamp)
            except Exception:pass
        subs.append(evaluator.create_subscription(NavPath,'/received_global_plan',followed_path,10))
    collisions=json.loads((a.workspace/'assets/contact_v2/contact_manifest.json').read_text())['collisions']
    for item in collisions:
        topic=item['topic'];subs.append(evaluator.create_subscription(Contacts,topic,lambda m,t=topic:contact_cb(m,t),qos_profile_sensor_data))
    mask_pub=evaluator.create_publisher(OccupancyGrid,'/osmag/constraint_mask',qos);info_pub=evaluator.create_publisher(CostmapFilterInfo,'/osmag/constraint_filter_info',qos)
    report={'passed':False,'scope':'06B2B single real-Nav2 simulation crossing; truth evaluates only','goal':[1.4413560486865775,-12.960313240266368,0.], 'spawn':[-.5,-12.960313240266368,0.], 'preflight':{},'constraint_receipts':[]};started=[];snap=None;contract=None;published=None;last_grids={};observations={};last_errors={}
    if fixture:report.update(scope='06D nominal frozen-fixture simulation mission; truth evaluates only',fixture=fixture,fixture_sha256=fixture_sha,goal=fixture['goal'],spawn=fixture['spawn'])
    session_fixture=fixture;sequence=bool(fixture and fixture.get('next_mission'));missions=[];msg=None;capture_prefix=''
    sim_timeout=fixture['simulation_timeout_s'] if fixture else 80
    wall_timeout=fixture['checker_wall_timeout_s'] if fixture else 420
    stop_requested=False;review_pending=None;review_busy=False;review_rejections=[];review_missions=[];review_good=True
    if a.interactive:
        import signal,threading
        review_lock=threading.Lock();wall_timeout=a.session_wall_timeout
        session_pub=evaluator.create_publisher(String,'/osmag/session_status',qos)
        def session_status(state,**fields):session_pub.publish(String(data=json.dumps({'state':state,**fields})))
        def stop_session(*_):
            nonlocal stop_requested
            stop_requested=True
        def user_goal(message):
            nonlocal review_pending
            from gcu_user_mission import parse_goal
            with review_lock:
                try:
                    if review_busy or review_pending is not None:raise RuntimeError('busy_cancel_before_new_goal')
                    v=message.pose.position;q=message.pose.orientation
                    review_pending=parse_goal(message.header.frame_id,[v.x,v.y],[q.x,q.y,q.z,q.w])
                except Exception as error:
                    review_rejections.append({'reason':str(error),'sim':adapter.sim(),'event_count':len(adapter.events)})
                    session_status('rejected',message='Goal rejected; check the map or wait for the active mission.',reason=str(error))
        subs.append(evaluator.create_subscription(PoseStamped,'/goal_pose',user_goal,10))
        subs.append(evaluator.create_subscription(PoseStamped,'/osmag/mission_goal',user_goal,10))
        subs.append(evaluator.create_subscription(Empty,'/osmag/session_stop',stop_session,10))
        signal.signal(signal.SIGINT,stop_session);signal.signal(signal.SIGTERM,stop_session)
    def step():
        nonlocal last_match
        executor.spin_once(timeout_sec=.015)
        if a.interactive and stop_requested and adapter.enabled and adapter.model.state in ['dispatching','executing','waiting_handoff','waiting_proxy','verifying']:adapter.cancel('user_cancel')
        for item in truth[-200:]:
            if item['sim_time']<=last_match:continue
            try:
                t=adapter.buffer.lookup_transform('map','base_footprint',Time(seconds=item['sim_time'])).transform
                pose=[t.translation.x,t.translation.y,angle(t.rotation)];actual=item['truth']
                matched.append(dict(item,estimated=pose,position_error_m=math.dist(pose[:2],actual[:2]),yaw_error_deg=math.degrees(abs(math.atan2(math.sin(pose[2]-actual[2]),math.cos(pose[2]-actual[2]))))));last_match=item['sim_time']
            except Exception:pass
        if time.monotonic()-begin>wall_timeout:raise RuntimeError('crossing_wall_timeout')
        if rolling_manager is not None:rolling_manager.tick()
    def wait(predicate,seconds=30):
        end=time.monotonic()+seconds
        while not predicate():
            if time.monotonic()>end:raise RuntimeError('preflight_timeout')
            step()
    def fresh_localized_pose():
        result=[]
        def available():
            from osmag_mission_contract import MissionError
            try:result[:]=adapter.pose();return True
            except MissionError as error:
                if str(error)!='stale_robot_tf':raise
                return False
        # Setup at rest can race independently delivered /clock and TF. Wait
        # for a genuinely fresh sample; retain the same .15 s age requirement.
        wait(available,2)
        return result
    def transition(name,code):
        if code==4:
            from lifecycle_msgs.srv import GetState
            state_client=evaluator.create_client(GetState,'/'+name+'/get_state');wait(state_client.service_is_ready,10)
            state_future=state_client.call_async(GetState.Request());wait(state_future.done,10)
            current_state=state_future.result().current_state.id;evaluator.destroy_client(state_client)
            if current_state in [1,2]:return
        c=evaluator.create_client(ChangeState,'/'+name+'/change_state');wait(c.service_is_ready,15)
        req=ChangeState.Request();req.transition.id=code;f=c.call_async(req);wait(f.done,15)
        if not f.result().success:raise RuntimeError('lifecycle_failed:'+name+':'+str(code))
    def capture_native(label):
        """Diagnostic snapshots only: no truth/scan feedback to the executor."""
        capture={'label':label,'observed_at':adapter.sim(),'state':adapter.model.report(),'grids':{},'scans':[]}
        try:capture['pose'],capture['pose_stamp']=adapter.pose()
        except Exception as e:capture['pose_error']=str(e)
        if contract is not None:
            capture['goal']=list(contract.goal);capture['segment']=list(contract.segments[max(0,adapter.model.index)].point)
        for key,m in list(grids.items()):
            name=f'{label}_{key}.npz';np.savez_compressed(a.output.parent/name,data=np.asarray(m.data,dtype=np.int16).reshape(m.info.height,m.info.width))
            capture['grids'][key]={'file':name,'stamp':stamp(m.header.stamp),'frame':m.header.frame_id,'resolution':m.info.resolution,'origin':[m.info.origin.position.x,m.info.origin.position.y]}
        from nav2_msgs.srv import GetCostmap
        for key in ['global','local']:
            try:
                client=evaluator.create_client(GetCostmap,'/'+key+'_costmap/get_costmap');wait(client.service_is_ready,3)
                future=client.call_async(GetCostmap.Request());wait(future.done,3);m=future.result().map
                name=f'{label}_{key}_raw.npz';np.savez_compressed(a.output.parent/name,data=np.asarray(m.data,dtype=np.uint8).reshape(m.metadata.size_y,m.metadata.size_x))
                capture['grids'][key+'_raw']={'file':name,'stamp':stamp(m.header.stamp),'frame':m.header.frame_id,'resolution':m.metadata.resolution,'origin':[m.metadata.origin.position.x,m.metadata.origin.position.y]}
                evaluator.destroy_client(client)
            except Exception as e:capture['grids'][key+'_raw_error']=str(e)
        for i,m in enumerate(list(recent_scans)):
            name=f'{label}_scan_{i}.npz';np.savez_compressed(a.output.parent/name,ranges=np.asarray(m.ranges))
            row={'file':name,'stamp':stamp(m.header.stamp),'frame':m.header.frame_id,'angle_min':m.angle_min,'angle_increment':m.angle_increment,'range_min':m.range_min,'range_max':m.range_max}
            try:
                tf=adapter.buffer.lookup_transform('map',m.header.frame_id,Time.from_msg(m.header.stamp)).transform
                row['map_from_scan']=[tf.translation.x,tf.translation.y,angle(tf.rotation)]
            except Exception as e:row['transform_error']=str(e)
            capture['scans'].append(row)
        report.setdefault('native_captures',[]).append(capture)
    def consumed():
        if rolling_manager is not None:return rolling_manager.ready()
        current=adapter.sim()
        for key,m in grids.items():
            s=stamp(m.header.stamp)
            # A costmap can arrive ahead of this node's /clock callback by a
            # few milliseconds. Keep the last validated observation until the
            # clock catches up; never replace it with a future-dated ACK.
            if s>current:continue
            if last_grids.get(key)==s:continue
            try:
                if s<published:raise RuntimeError('costmap_predates_mask')
                tf=[0.,0.,0.]
                transform_stamp=None
                if m.header.frame_id!='map':
                    if m.header.frame_id!='odom':raise RuntimeError('unexpected_costmap_frame')
                    # Humble KeepoutFilter::process uses TimePointZero, not the
                    # costmap publication timestamp. AMCL post-dates map/odom
                    # by its transform tolerance; interpolating at the grid
                    # stamp checks a different raster boundary during motion.
                    native_tf=adapter.buffer.lookup_transform('map','odom',Time())
                    transform_stamp=stamp(native_tf.header.stamp)
                    v=native_tf.transform;tf=[v.translation.x,v.translation.y,angle(v.rotation)]
                if abs(m.info.origin.orientation.x)+abs(m.info.origin.orientation.y)+abs(m.info.origin.orientation.z)>1e-8:raise RuntimeError('rotated_grid_origin')
                data=np.asarray(m.data).reshape(m.info.height,m.info.width)
                if key=='local' and fixture and fixture.get('align_local_grid'):
                    from gcu_costmap_observation import lattice_phase
                    alignment=lattice_phase(m.info.resolution,(m.info.origin.position.x,m.info.origin.position.y),snap.resolution,snap.origin)
                    if m.header.frame_id!='map' or not alignment['aligned']:raise RuntimeError('local_grid_lattice_not_aligned')
                    report['local_grid_alignment']=alignment
                probes=[contract.start,contract.goal[:2]] if key=='global' else [adapter.pose()[0][:2]]
                observed=inspect(snap,data,m.info.resolution,(m.info.origin.position.x,m.info.origin.position.y),tf,probes,full=key=='global')
                observations[key]=dict(observed,stamp=s,transform_stamp=transform_stamp,transform_policy='latest_native_keepout');last_grids[key]=s;last_errors.pop(key,None)
                report['constraint_receipts'].append(dict(source=key,observed_at=current,**observations[key]))
            except Exception as e:
                last_errors[key]=str(e)
                if 'native_mask_cells_missing' in str(e) and len(report.setdefault('native_grid_failures',[]))<8 and not any(v['source']==key and v['stamp']==s for v in report['native_grid_failures']):
                    ident=len(report['native_grid_failures'])+1
                    np.savez_compressed(a.output.parent/f'{capture_prefix}native_grid_failure_{ident}.npz',native=np.asarray(m.data,dtype=np.int16).reshape(m.info.height,m.info.width),mask=snap.mask)
                    latest=None;stamped=tf
                    if m.header.frame_id=='odom':
                        latest_tf=adapter.buffer.lookup_transform('map','odom',Time())
                        v=latest_tf.transform
                        latest={'stamp':stamp(latest_tf.header.stamp),'transform':[v.translation.x,v.translation.y,angle(v.rotation)]}
                        v=adapter.buffer.lookup_transform('map','odom',Time.from_msg(m.header.stamp)).transform
                        stamped=[v.translation.x,v.translation.y,angle(v.rotation)]
                    report['native_grid_failures'].append({'source':key,'error':str(e),'stamp':s,'observed_at':current,
                        'frame':m.header.frame_id,'resolution':m.info.resolution,'origin':[m.info.origin.position.x,m.info.origin.position.y],
                        'map_from_grid_at_stamp':stamped,'map_from_grid_latest':latest,'map_from_grid_checked':tf,
                        'mask_resolution_wire':float(np.float32(msg.info.resolution)),'mask_origin':list(snap.origin),
                        'scope':'Diagnostic capture; does not relax or acknowledge the failed grid.'})
        ready=set(observations)=={'global','local'} and all(0<=current-v['stamp']<=(.75 if k=='global' else .5) for k,v in observations.items())
        report['last_readiness']={'clock':current,'ages':{k:current-v['stamp'] for k,v in observations.items()},'errors':dict(last_errors),'ready':ready}
        return ready
    def execute_leg(leg,number):
        nonlocal report,fixture,contract,snap,published,recording,last_grids,observations,last_errors,msg,capture_prefix,sim_timeout,forwarding_imu,rolling_manager
        fixture=leg
        if number or a.interactive:
            report={'passed':False,'scope':'06D consecutive mission; same live executor and native servers','fixture':leg,'goal':leg['goal'],'spawn':leg['spawn'],'preflight':{},'constraint_receipts':[]}
        adapter.enabled=False
        from gcu_segmented_executor import require_completed_previous
        require_completed_previous(adapter.model,adapter.handles)
        truth_begin=len(truth);match_begin=len(matched);event_begin=len(adapter.events);command_begin=len(commands);pose_begin=len(adapter.poses);tf_begin=len(adapter.tf_timing);plan_begin=len(plans)
        observations={};last_grids={};last_errors={};capture_prefix=('user_mission_'+str(number).zfill(2)+'_') if a.interactive else f'leg_{number+1}_' if sequence else ''
        sim_timeout=leg['simulation_timeout_s'] if leg else 80
        start,tfstamp=fresh_localized_pose();now=time.monotonic()
        contract=make_contract(planner,start[:2],report['goal'][:2],report['goal'][2],mission_id=fixture['name'] if fixture else '06b2b_short_01',now=now,pose_stamp=now-(adapter.sim()-tfstamp),route_revision=number+1,closure_revision=adapter.closure['revision'],disabled_passages=adapter.closure['disabled_passages'])
        expected=tuple(fixture['expected_passages']) if fixture else ('-93',)
        if contract.passage_ids!=expected or (fixture and contract.area_ids!=tuple(fixture['expected_areas'])):raise RuntimeError('fixture_topology_changed')
        if a.metric_baseline:
            from gcu_metric_baseline import metric_contract
            report['planned_contract']=contract.report();contract=metric_contract(contract);report['execution_mode']='metric_final_goal'
        incidence={n:(v['osmAG:from'],v['osmAG:to']) for n,v in planner.passages.items()}
        snap=build_mask(planner.grid,planner.resolution,planner.origin,planner.leaves,planner.passage_segments,incidence,
            allowed_areas=contract.area_ids,open_passages=contract.passage_ids,disabled_passages=contract.disabled_passages,robot_pose=start[:2],mission_id=contract.mission_id,route_revision=number+1,mask_revision=number+1,closure_revision=contract.closure_revision)
        from gcu_mission_fixture import validate_current_contract
        report['current_contract_mask_validation']=validate_current_contract(contract,snap)
        report['snapshot']=snap.report();report['contract']=contract.report()
        constraint_file=capture_prefix+'constraint.npz' if a.interactive else f'constraint_{number+1}.npz' if sequence or rolling else 'constraint.npz'
        report['constraint_file']=constraint_file
        np.savez_compressed(a.output.parent/constraint_file,mask=snap.mask,combined=snap.combined,physical=planner.grid)
        if rolling:
            from gcu_rolling_window_manager import RollingWindowManager
            if rolling_manager is not None:rolling_manager.close()
            rolling_manager=RollingWindowManager(evaluator,adapter,planner,contract,grids,mask_pub,info_pub,a.output.parent,wait,
                shape=tuple(fixture['rolling_window']['shape']),shift_m=fixture['rolling_window']['shift_m'],
                file_prefix=f'rolling_mission_{number+1}_')
            rolling_manager.publish_window(start);adapter.window_provider=rolling_manager
            published=rolling_manager.pending['published_sim']
        else:
            msg=OccupancyGrid();msg.header.frame_id='map';msg.header.stamp=adapter.get_clock().now().to_msg();published=stamp(msg.header.stamp)
            msg.info.resolution=snap.resolution;msg.info.width=snap.mask.shape[1];msg.info.height=snap.mask.shape[0];msg.info.origin.position.x=snap.origin[0];msg.info.origin.position.y=snap.origin[1];msg.info.origin.orientation.w=1.;msg.data=snap.mask.ravel().astype(int).tolist();mask_pub.publish(msg)
            info=CostmapFilterInfo();info.type=0;info.base=0.;info.multiplier=1.;info.filter_mask_topic='/osmag/constraint_mask';info_pub.publish(info)
        if not started:
            for name in ['planner_server','controller_server','bt_navigator']:
                transition(name,1);transition(name,3);started.append(name)
        elif number:
            # The actor has already certified terminal actions and final rest.
            # Clear native bounds under that hold before acknowledging a new mask.
            from nav2_msgs.srv import ClearEntireCostmap
            for service in ['/global_costmap/clear_entirely_global_costmap','/local_costmap/clear_entirely_local_costmap']:
                client=evaluator.create_client(ClearEntireCostmap,service);wait(client.service_is_ready,10)
                future=client.call_async(ClearEntireCostmap.Request());wait(future.done,10)
            report['reset_under_rest']=True
        if a.diagnose_stationary:
            report['scope']='06D stationary consumption diagnosis; no navigation goals dispatched'
            if adapter.require_imu:
                from gcu_segmented_executor import fresh_scan_at
                wait(lambda:adapter.count_publishers('/imu_broadcaster/imu')==1 and fresh_scan_at(adapter.sim(),adapter.imu_observations,.15),10)
                if not adapter.buffer.can_transform('base_footprint','imu_link',Time()):raise RuntimeError('missing_imu_static_transform')
            imu_begin=len(adapter.imu_samples)
            recording=True;end=time.monotonic()+30;initial=adapter.sim()
            while time.monotonic()<end and adapter.sim()-initial<8:
                step();consumed()
            displacement=max((math.dist(t['truth'][:2],truth[0]['truth'][:2]) for t in truth),default=float('inf'))
            report['stationary']={'duration_sim_s':adapter.sim()-initial,'displacement_m':displacement,'mask_ready_at_end':consumed(),
                'nonzero_commands':sum(abs(v['v'])+abs(v['w'])>1e-8 for v in commands),'navigation_sends':len(adapter.events)}
            report['diagnostic_completed']=True
            report['passed']=bool(truth and displacement<.01 and not report['stationary']['nonzero_commands'] and not adapter.events)
            if adapter.require_imu:
                samples=adapter.imu_samples[imu_begin:]
                report['imu_diagnostic']={'publishers':adapter.count_publishers('/imu_broadcaster/imu'),'samples':samples,
                    'fresh':fresh_scan_at(adapter.sim(),adapter.imu_observations,.15)}
                report['passed']=bool(report['passed'] and report['imu_diagnostic']['fresh'] and len(samples)>=200 and max(abs(v['yaw_rate']) for v in samples)<.02)
            return 0 if report['passed'] else 1
        wait(lambda:consumed() and adapter.scan_stamp>=published and 0<=adapter.sim()-adapter.scan_stamp<=.3 and adapter.client.server_is_ready(),40)
        c=ActionClient(evaluator,ComputePathToPose,'/compute_path_to_pose');wait(c.server_is_ready)
        if rolling:rolling_manager.prepare()
        for segment in ([] if rolling else contract.segments):
            from geometry_msgs.msg import PoseStamped
            req=ComputePathToPose.Goal();req.planner_id='GridBased';req.use_start=False;req.goal=PoseStamped();req.goal.header.frame_id='map';req.goal.pose.position.x,req.goal.pose.position.y=map(float,segment.point);req.goal.pose.orientation.w=1.
            f=c.send_goal_async(req);wait(f.done);h=f.result()
            if not h.accepted:raise RuntimeError('segment_preflight_rejected')
            f=h.get_result_async();wait(f.done);r=f.result();points=[[p.pose.position.x,p.pose.position.y] for p in r.result.path.poses]
            check=validate_path(snap.combined,snap.resolution,snap.origin,points,.20)
            report['preflight'][str(segment.index)]={'status':r.status,'points':points,'validation':check}
            if r.status!=4 or not check['valid']:raise RuntimeError('segment_preflight_not_feasible')
        if any(abs(v['v'])+abs(v['w'])>1e-8 for v in commands[command_begin:]):raise RuntimeError('motion_before_dispatch')
        report['ownership']={'cmd_vel':[i.node_name for i in evaluator.get_publishers_info_by_topic('/cmd_vel')],
            'navigate_status':evaluator.count_publishers('/navigate_to_pose/_action/status'),'direct_goal_publishers':evaluator.count_publishers('/phase6_disabled_goal_pose'),
            'old_bridge_subscribers':evaluator.count_subscribers('/gcu/navigation_goal')}
        if report['ownership']!={'cmd_vel':['controller_server'],'navigate_status':1,'direct_goal_publishers':0,'old_bridge_subscribers':0}:raise RuntimeError('unexpected_execution_owner')
        from gcu_process_resources import sample as resource_sample
        report['resource_at_dispatch']=resource_sample()
        report['native_grid_shapes_at_dispatch']={key:[m.info.height,m.info.width] for key,m in grids.items()}
        report['dispatch_sim']=adapter.sim();recording=True;adapter.begin(contract,consumed);report['contract']=adapter.model.contract.report()
        if a.drop_window_source and number==0:
            from gcu_window_loss_metrics import score_window_loss
            wait(lambda:math.dist(adapter.pose()[0][:2],contract.start)>=.65,120)
            report['window_loss_request']={'sim':adapter.sim(),'pose':adapter.pose()[0],'wall':time.monotonic()}
            rolling_manager.freeze()
            wait(lambda:adapter.model.stop_confirmed and not adapter.handles,20)
            hold=rest_window(truth[truth_begin:],earliest=report['window_loss_request']['sim'])
            if hold is None:wait(lambda:rest_window(truth[truth_begin:],earliest=report['window_loss_request']['sim']) is not None,10)
            hold=rest_window(truth[truth_begin:],earliest=report['window_loss_request']['sim'])
            wait(lambda:truth and truth[-1]['sim_time']>=hold['confirmed_sim']+1.,15)
            report.update(events=adapter.events[event_begin:],state=adapter.model.report(),truth=truth[truth_begin:],
                localization=matched[match_begin:],commands=commands[command_begin:])
            report['window_loss']=score_window_loss(report)
            report['rolling']=rolling_manager.report();report['expected_window_source_loss']=True
            report['resource_at_stop']=resource_sample()
            report['passed']=bool(report['window_loss']['passed'] and not contacts)
            return
        if sequence:capture_native(f'leg_{number+1}_dispatch')
        if a.cancel_near_handoff or a.close_upcoming_passage or a.drop_required_imu:
            from std_msgs.msg import Empty,String
            from gcu_cancel_metrics import score_cancellation
            closing=a.close_upcoming_passage;imu_loss=a.drop_required_imu;request_key='imu_loss_request' if imu_loss else 'closure_request' if closing else 'cancel_request'
            closure_messages=[]
            if closing:subs.append(evaluator.create_subscription(String,'/osmag/closure_status',lambda m:closure_messages.append({'sim':adapter.sim(),'payload':json.loads(m.data)}),qos))
            if not imu_loss:
                cancel_pub=evaluator.create_publisher(String if closing else Empty,'/osmag/passage_closures' if closing else '/osmag/mission_cancel',10)
                wait(lambda:cancel_pub.get_subscription_count()==1,5)
            report['scope']='06D/E required executor IMU topic loss; real EKF IMU remains live' if imu_loss else '06D/E live passage-closure fault test; truth evaluates only' if closing else '06D/E live cancel-topic fault test; truth evaluates only'
            while request_key not in report:
                step()
                if contacts:raise RuntimeError('wall_or_unclassified_contact')
                if adapter.fault_reason or adapter.model.state not in ['dispatching','executing','waiting_handoff']:raise RuntimeError('fault_before_cancel_trigger')
                point,_=adapter.pose();distance=math.dist(point[:2],contract.segments[0].point)
                if adapter.model.index!=0 or distance<=(.60 if closing or imu_loss else .30):raise RuntimeError('cancel_trigger_missed_handoff')
                if adapter.model.state=='executing' and distance<=(.73 if closing or imu_loss else .43):
                    report[request_key]={'wall':time.monotonic(),'sim':adapter.sim(),'pose':point,'segment_distance_m':distance}
                    if imu_loss:forwarding_imu=False
                    elif closing:
                        update={'revision':1,'disabled_passages':[contract.passage_ids[0]]};report[request_key]['update']=update
                        cancel_pub.publish(String(data=json.dumps(update)))
                    else:cancel_pub.publish(Empty())
                if adapter.sim()-report['dispatch_sim']>sim_timeout:raise RuntimeError('cancel_trigger_timeout')
            wait(lambda:adapter.model.state==('failed' if closing or imu_loss else 'cancelled') and adapter.model.stop_confirmed and not adapter.handles,15)
            if closing:
                wait(lambda:bool(closure_messages) and adapter.closure_result is not None,15)
                report['closure_resolution']=adapter.closure_result;report['closure_status_messages']=closure_messages
                if adapter.closure_result['status']!='no_route':raise RuntimeError('fixture_requires_no_route_after_closure')
                pose,_=adapter.pose();snap=build_mask(planner.grid,planner.resolution,planner.origin,planner.leaves,planner.passage_segments,incidence,
                    allowed_areas=contract.area_ids,open_passages=[],disabled_passages=[contract.passage_ids[0]],robot_pose=pose[:2],mission_id=contract.mission_id,
                    closure_revision=1,mask_revision=2)
                report['closure_snapshot']=snap.report();report['closure_mask_pose']=pose
                np.savez_compressed(a.output.parent/'closure_constraint.npz',mask=snap.mask,combined=snap.combined,physical=planner.grid)
                msg.header.stamp=adapter.get_clock().now().to_msg();published=stamp(msg.header.stamp);msg.data=snap.mask.ravel().astype(int).tolist()
                observations={};last_grids={};last_errors={};mask_pub.publish(msg);report['closure_mask_published_sim']=published
                wait(consumed,20);capture_native('closed')
                req=ComputePathToPose.Goal();req.planner_id='GridBased';req.use_start=False;req.goal=PoseStamped();req.goal.header.frame_id='map'
                req.goal.pose.position.x,req.goal.pose.position.y=map(float,contract.goal[:2]);req.goal.pose.orientation.w=1.
                f=c.send_goal_async(req);wait(f.done);h=f.result()
                if not h.accepted:raise RuntimeError('closed_plan_query_rejected')
                f=h.get_result_async();wait(f.done);result=f.result()
                report['closed_native_plan']={'status':result.status,'points':[[p.pose.position.x,p.pose.position.y] for p in result.result.path.poses]}
                if result.status!=6 or report['closed_native_plan']['points']:raise RuntimeError('native_plan_bypassed_closed_door')
            wait(lambda:rest_window(truth[truth_begin:],earliest=report[request_key]['sim']) is not None,10)
            hold=rest_window(truth[truth_begin:],earliest=report[request_key]['sim'])['confirmed_sim']+1.
            wait(lambda:truth and truth[-1]['sim_time']>=hold,10)
            report.update(events=adapter.events[event_begin:],state=adapter.model.report(),truth=truth[truth_begin:],localization=matched[match_begin:],commands=commands[command_begin:])
            report['cancellation']=score_cancellation(report,reason='watchdog:stale_or_unowned_imu' if imu_loss else 'passage_closed' if closing else 'user_cancel',final_state='failed' if closing or imu_loss else 'cancelled',band=(.60,.75) if closing or imu_loss else (.30,.45),request_key=request_key)
            if imu_loss:
                report['imu_forwarding']=imu_forwarding;report['required_imu_samples']=list(adapter.imu_samples)
                report['raw_imu_publishers']=evaluator.count_publishers('/imu_broadcaster/imu');report['required_imu_publishers']=evaluator.count_publishers('/gcu_fault/required_imu')
            report['swept_truth']=validate_path(snap.combined,snap.resolution,snap.origin,[v['truth'][:2] for v in report['truth']],.18)
            report['localization_p95_m']=float(np.percentile([v['position_error_m'] for v in report['localization']],95))
            report['localization_p95_deg']=float(np.percentile([v['yaw_error_deg'] for v in report['localization']],95))
            report['passed']=bool(report['swept_truth']['valid'] and not contacts and report['localization_p95_m']<=.25 and report['localization_p95_deg']<=5 and max(abs(v['v']) for v in report['commands'])<=.10001)
            return
        while adapter.model.state!='succeeded':
            step()
            if adapter.fault_reason:raise RuntimeError(adapter.fault_reason)
            if adapter.model.state in ['cancelling','cancelled','failed']:
                raise RuntimeError('mission_action_failure:'+str(adapter.model.reason))
            if contacts:raise RuntimeError('wall_or_unclassified_contact')
            if adapter.sim()-report['dispatch_sim']>sim_timeout:raise RuntimeError('crossing_sim_timeout')
        # Score actual truth only after the executor's own action/pose/odom-rest
        # decision. It never receives this evaluator's samples.
        wait(lambda:len(truth[truth_begin:])>10 and rest_window(truth[truth_begin:],earliest=adapter.final_action_sim) is not None,10)
        leg_truth=truth[truth_begin:];leg_matched=[x for x in matched if leg_truth[0]['sim_time']<=x['sim_time']<=leg_truth[-1]['sim_time']];leg_events=adapter.events[event_begin:];leg_commands=commands[command_begin:]
        final=leg_truth[-1]['truth'];report['position_error_m']=math.dist(final[:2],report['goal'][:2]);delta=final[2]-report['goal'][2];report['heading_error_deg']=math.degrees(abs(math.atan2(math.sin(delta),math.cos(delta))))
        report['truth_rest']=rest_window(leg_truth,earliest=adapter.final_action_sim);report['coverage']=coverage(leg_truth,leg_matched)
        report['swept_truth']=validate_path(snap.combined,snap.resolution,snap.origin,[v['truth'][:2] for v in leg_truth],.18)
        if fixture and fixture.get('physical_obstacle'):
            from gcu_physical_obstacle import overlay_box
            evaluation=overlay_box(snap.combined,snap.resolution,snap.origin,fixture['physical_obstacle'])
            np.savez_compressed(a.output.parent/'obstacle_evaluation.npz',combined=evaluation)
            report['obstacle_sweep']=validate_path(evaluation,snap.resolution,snap.origin,[v['truth'][:2] for v in leg_truth],.18)
            # These plan-only queries are never sent to the controller. An
            # unseen box corner is an oracle diagnostic, not a requirement that
            # navigation know the complete unknown geometry before observing it.
            report['obstacle_preflight_oracle']={k:validate_path(evaluation,snap.resolution,snap.origin,v['points'],.20) for k,v in report['preflight'].items()}
            report['passed_obstacle']=bool(report['obstacle_sweep']['valid'])
        truth_contract=replace(adapter.model.contract,start=tuple(leg_truth[0]['truth'][:2]),pose_stamp=leg_truth[0]['sim_time'])
        monitor=CrossingMonitor(truth_contract,planner.leaves,planner.passage_segments)
        for item in leg_truth[1:]:monitor.observe(item['truth'][:2],item['sim_time'])
        report['truth_crossings']=monitor.events
        sends=[e for e in leg_events if e['event']=='send'];results=[e for e in leg_events if e['event']=='result']
        first_original=next(e for e in sends if e['segment']==0 and not e.get('proxy'))
        first_handle=next(e['uuid'] for e in leg_events if e['event']=='accepted' and e['token']==first_original['token'])
        first_result=next((e for e in results if e['uuid']==first_handle),None)
        next_original=next((e for e in sends if e['segment']==1),None)
        report['handoff']=None if a.metric_baseline or not contract.passage_ids else {'next_goal_sim':next_original['sim'],'first_result':first_result,'before_intermediate_terminal':first_result is None or next_original['wall']<first_result['wall'],
            'confirmed_at_next_goal':next_original['confirmed']}
        report['handoffs']=[]
        for index in range(0 if a.metric_baseline else len(contract.passage_ids)):
            send=next(e for e in sends if e['segment']==index and not e.get('proxy'));next_send=next(e for e in sends if e['segment']==index+1)
            handle=next(e['uuid'] for e in leg_events if e['event']=='accepted' and e['token']==send['token'])
            terminal=next((e for e in results if e['uuid']==handle),None)
            report['handoffs'].append({'segment':index,'passage_id':contract.passage_ids[index],'next_goal_sim':next_send['sim'],'before_intermediate_terminal':terminal is None or next_send['wall']<terminal['wall']})
        report['localization_p95_m']=float(np.percentile([v['position_error_m'] for v in leg_matched],95));report['localization_p95_deg']=float(np.percentile([v['yaw_error_deg'] for v in leg_matched],95))
        report['passed']=bool(monitor.confirmed==len(contract.passage_ids) and all(h['before_intermediate_terminal'] for h in report['handoffs']) and report['position_error_m']<=.20 and report['heading_error_deg']<=10 and report['swept_truth']['valid'] and report['coverage']['matched_fraction']>=.9 and report['coverage']['max_truth_gap_sim_s']<=.15 and report['localization_p95_m']<=.25 and report['localization_p95_deg']<=5 and max(abs(v['v']) for v in leg_commands)<=.10001)
        report.update(events=leg_events,state=adapter.model.report(),poses=adapter.poses[pose_begin:],tf_timing=adapter.tf_timing[tf_begin:],scan_observations=list(adapter.scan_observations),truth=leg_truth,localization=leg_matched,commands=leg_commands,plans=plans[plan_begin:],contacts=list(contacts),contact_counts=dict(counts),last_consumption_errors=dict(last_errors),imu_samples=[v for v in adapter.imu_samples if leg_truth[0]['sim_time']<=v['sim_time']<=leg_truth[-1]['sim_time']])
        report['contact_publishers']={item['topic']:evaluator.count_publishers(item['topic']) for item in collisions}
        report['resource_at_stop']=resource_sample()
        report['native_grid_shapes_at_stop']={key:[m.info.height,m.info.width] for key,m in grids.items()}
        report['passed']=bool(report['passed'] and not contacts and all(v==1 for v in report['contact_publishers'].values()))
        if rolling:
            report['rolling']=rolling_manager.report()
            report['planning_map_subscribers']=[{'name':i.node_name,'namespace':i.node_namespace} for i in evaluator.get_subscriptions_info_by_topic('/map')]
            if any('costmap' in i['name'] for i in report['planning_map_subscribers']):raise RuntimeError('full_floor_map_still_used_by_costmap')
            if not a.interactive and not fixture.get('allow_short_window_mission'):
                report['passed']=bool(report['passed'] and len(report['rolling']['windows'])>=3
                    and any(e.get('proxy') for e in leg_events if e['event']=='send'))
        if fixture and fixture.get('physical_obstacle'):report['passed']=bool(report['passed'] and report['passed_obstacle'])
        missions.append(json.loads(json.dumps(report)))
        if sequence:(a.output.parent/f'mission_{number+1}.json').write_text(json.dumps(report,indent=2)+'\n')
        adapter.enabled=False
        if not report['passed']:raise RuntimeError('mission_acceptance_failed')
    try:
        wait(lambda:adapter.sim()>0 and adapter.scan_stamp is not None and adapter.buffer.can_transform('map','base_footprint',Time()),50)
        if not a.interactive:
            execute_leg(session_fixture,0)
            if a.drop_window_source and session_fixture.get('recover_window_source'):
                fault_report=json.loads(json.dumps(report));fault_report['contacts']=list(contacts)
                fault_report['contact_publishers']={item['topic']:evaluator.count_publishers(item['topic']) for item in collisions}
                (a.output.parent/'source_loss_mission.json').write_text(json.dumps(fault_report,indent=2)+'\n')
                restored=adapter.sim();rolling_manager.resume()
                wait(lambda:consumed() and all(v['stamp']>=restored for v in rolling_manager.committed['gate'].observations.values()),15)
                def recovery_ready():
                    from osmag_mission_contract import MissionError
                    try:adapter.recover();return True
                    except MissionError as error:
                        if str(error)!='stale_robot_tf':raise
                        return False
                wait(recovery_ready,2)
                point,tfstamp=fresh_localized_pose();now=time.monotonic()
                goal=list(session_fixture['spawn']);goal[2]=math.pi
                preview=make_contract(planner,point[:2],goal[:2],goal[2],mission_id=session_fixture['name']+'_recovery',
                    now=now,pose_stamp=now-(adapter.sim()-tfstamp),route_revision=2,closure_revision=adapter.closure['revision'])
                recovery_leg=dict(session_fixture,name=preview.mission_id,spawn=point,goal=goal,
                    expected_passages=list(preview.passage_ids),expected_areas=list(preview.area_ids),
                    route_contract=preview.report(),allow_short_window_mission=True)
                execute_leg(recovery_leg,1)
                (a.output.parent/'recovery_mission.json').write_text(json.dumps(report,indent=2)+'\n')
                report['source_loss_and_recovery']=dict(source_loss_file='source_loss_mission.json',
                    recovery_file='recovery_mission.json',source_restored_sim=restored,same_runtime_processes=True)
                report['passed']=bool(report['passed'] and fault_report['passed'])
            if sequence:execute_leg(session_fixture['next_mission'],1)
        else:
            session_status('ready',message='Click a goal in the map.')
            number=0
            while not stop_requested and time.monotonic()-begin<a.session_wall_timeout:
                step()
                with review_lock:
                    target=review_pending;review_pending=None
                    if target is not None:review_busy=True
                if target is None:continue
                try:
                    fresh=[]
                    def fresh_pose():
                        try:fresh[:]=adapter.pose();return True
                        except Exception:return False
                    wait(fresh_pose,2)
                    point,stamp0=fresh;now=time.monotonic()
                    preview=make_contract(planner,point[:2],target[:2],target[2],mission_id='review_'+str(number+1),now=now,pose_stamp=now-(adapter.sim()-stamp0),route_revision=number+1,closure_revision=adapter.closure['revision'],disabled_passages=adapter.closure['disabled_passages'])
                    if adapter.model.state in ['failed','cancelled']:adapter.recover()
                    leg=dict(session_fixture,name=preview.mission_id,spawn=point,goal=target,expected_passages=list(preview.passage_ids),expected_areas=list(preview.area_ids),route_contract=preview.report(),scope='Runtime-validated user query; seed supplies maps/profile only')
                    # Preview is display-only. Native feasibility/readiness and
                    # a fresh contract are still required before begin sends.
                    path=NavPath();path.header.frame_id='map';path.header.stamp=adapter.get_clock().now().to_msg()
                    for x,y in preview.path:
                        pose=PoseStamped();pose.header=path.header;pose.pose.position.x=x;pose.pose.position.y=y;pose.pose.orientation.w=1.;path.poses.append(pose)
                    adapter.path_pub.publish(path);session_status('running',goal=target)
                    number+=1
                    try:execute_leg(leg,number)
                    except Exception as error:
                        report['error']=str(error);report['passed']=False
                        if adapter.enabled:
                            if adapter.fault_reason is None and adapter.model.reason is None:adapter.cancel('test_failure')
                            wall_timeout=max(wall_timeout,time.monotonic()-begin+20)
                            wait(lambda:adapter.model.stop_confirmed and not adapter.handles,15)
                        after=report.get('dispatch_sim',adapter.sim());created=report['contract']['created_at']
                        report.update(events=[e for e in adapter.events if e['wall']>=created],state=adapter.model.report(),truth=[v for v in truth if v['sim_time']>=after],localization=[v for v in matched if v['sim_time']>=after],commands=[v for v in commands if v['sim_time']>=after],contacts=list(contacts))
                    finally:adapter.enabled=False
                    filename='user_mission_'+str(number).zfill(2)+'.json';(a.output.parent/filename).write_text(json.dumps(report,indent=2)+'\n')
                    active=adapter.model.contract is not None and adapter.model.contract.mission_id==leg['name']
                    outcome='succeeded' if report['passed'] else 'cancelled' if active and adapter.model.state=='cancelled' else 'failed'
                    reason=adapter.model.reason or report.get('error')
                    review_missions.append({'file':filename,'goal':target,'outcome':outcome,'reason':reason})
                    session_status(outcome,goal=target,reason=reason)
                except Exception as error:
                    review_rejections.append({'reason':str(error),'sim':adapter.sim(),'event_count':len(adapter.events)})
                    session_status('rejected',message='No safe mission is available for that goal.',reason=str(error))
                    if adapter.enabled or contacts:review_good=False;stop_requested=True
                finally:
                    with review_lock:review_busy=False
            if adapter.model.contract is not None and adapter.model.state not in ['succeeded','cancelled','failed']:raise RuntimeError('session_has_unsettled_mission')
            report['passed']=bool(review_good and not contacts and (adapter.model.contract is None or adapter.model.stop_confirmed))
    except Exception as e:
        import traceback
        report['error_traceback']=traceback.format_exc()
        report['error']=repr(e)
        adapter.cancel('test_failure')
        try:capture_native('failure')
        except Exception as capture_error:report['capture_error']=str(capture_error)
        try:
            end=time.monotonic()+15
            while adapter.model.contract is not None and not adapter.model.stop_confirmed and time.monotonic()<end:
                executor.spin_once(timeout_sec=.015)
        except Exception:pass
    finally:
        adapter.enabled=False
        if rolling_manager is not None:
            report['rolling']=rolling_manager.report();rolling_manager.close()
        report['lifecycle_shutdown']={}
        for name in reversed(started):
            try:transition(name,4);report['lifecycle_shutdown'][name]=True
            except Exception as e:report['lifecycle_shutdown'][name]=str(e);report['passed']=False
        report.update(events=adapter.events,state=adapter.model.report(),poses=adapter.poses,tf_timing=adapter.tf_timing,scan_observations=adapter.scan_observations,truth=truth,localization=matched,commands=commands,plans=plans,contacts=contacts,contact_counts=counts,last_consumption_errors=last_errors,wall_elapsed_s=time.monotonic()-begin)
        report['contact_publishers']={c['topic']:evaluator.count_publishers(c['topic']) for c in collisions}
        report['passed']=bool(report['passed'] and all(v==1 for v in report['contact_publishers'].values()) and not contacts)
        payload=report
        if a.interactive:
            payload={'passed':report['passed'],'scope':'Bounded interactive simulation review session; not Phase 06 completion','seed_fixture':session_fixture,'missions':review_missions,'rejections':review_rejections,
                'events':adapter.events,'truth':truth,'localization':matched,'commands':commands,'contacts':contacts,'final_state':adapter.model.report(),
                'lifecycle_shutdown':report['lifecycle_shutdown'],'contact_publishers':report['contact_publishers'],'wall_elapsed_s':time.monotonic()-begin}
        if sequence:
            payload={'passed':bool(report['passed'] and len(missions)==2 and all(m['passed'] for m in missions)),
                'scope':'06D two consecutive destinations; one simulator, localization, native servers and executor',
                'fixture':session_fixture,'missions':missions,'error':report.get('error'),'lifecycle_shutdown':report['lifecycle_shutdown'],
                'truth':truth,'localization':matched,'commands':commands,'events':adapter.events,'contacts':contacts,'imu_samples':adapter.imu_samples,
                'final_state':adapter.model.report(),'contact_publishers':report['contact_publishers'],'wall_elapsed_s':time.monotonic()-begin}
            if len(missions)<2:payload['failed_mission']=report
        executor.shutdown();a.output.write_text(json.dumps(payload,indent=2)+'\n');adapter.destroy_node();evaluator.destroy_node();rclpy.shutdown()
        report=payload
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
