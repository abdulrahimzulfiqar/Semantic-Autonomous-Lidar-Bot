#!/usr/bin/env python3
"""Score SIMULATION-ONLY NavigateToPose. Truth never selects successful stopping.

Requires --execute for action requests; positive-control mode sends no commands.
Emergency cancellation is logged as failure. Every attempt writes its evidence.
"""
import argparse
import json
import math
from pathlib import Path
import time
import numpy as np
from gcu_plan_validation import validate_path
from gcu_navigation_metrics import rest_window,coverage


def wrap(x):return math.atan2(math.sin(x),math.cos(x))
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def stamp(s):return s.sec+s.nanosec*1e-9


def main():
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.time import Time
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from tf2_ros import Buffer,TransformListener,TransformException
    from tf2_msgs.msg import TFMessage
    from rosgraph_msgs.msg import Clock
    from ros_gz_interfaces.msg import Contacts
    from geometry_msgs.msg import Twist,PoseStamped
    from std_msgs.msg import String,Empty as EmptyMessage
    from types import SimpleNamespace
    from nav_msgs.msg import OccupancyGrid,Odometry,Path as NavPath
    from nav2_msgs.action import NavigateToPose,ComputePathToPose
    from std_srvs.srv import Empty
    from lifecycle_msgs.srv import GetState
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--via-click',action='store_true');p.add_argument('--execute',action='store_true');p.add_argument('--positive-control',action='store_true')
    p.add_argument('--goal',type=float,nargs=3,default=[1.4413560486865775,-12.960313240266368,0.])
    p.add_argument('--expected',choices=['success','rejected','cancelled'],default='success')
    p.add_argument('--sim-timeout',type=float,default=80.);p.add_argument('--wall-timeout',type=float,default=400.)
    p.add_argument('--score-map',type=Path);p.add_argument('--assets',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if not (a.execute or a.positive_control):p.error('--execute required for simulated navigation')
    if a.output.exists():p.error('Refusing to overwrite existing evidence')
    rclpy.init(args=[]);node=rclpy.create_node('gcu_autonomous_evaluator',parameter_overrides=[Parameter('use_sim_time',value=True)]);buf=Buffer();listener=TransformListener(buf,node)
    sensor=QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT)
    latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    state={'sim':0.,'truth':None,'clock_wall':0.,'truth_wall':0.,'pending':[],'map':None,'scan_wall':0.,'odom_wall':0.,'bridge':None}
    truth_samples=[];samples=[];contacts=[];contact_event_count=0;commands=[];paths=[];contact_counts={};bridge_statuses=[];display_paths=[];subs=[]
    started=time.monotonic();last_progress=started;recording=False;last_match=-1.;last_truth=-1.;handle=None
    manifest=json.loads((a.assets/'contact_manifest.json').read_text())
    def truth(m):
        nonlocal last_truth
        for t in m.transforms:
            if t.child_frame_id.strip('/').split('::')[-1]!='lidarbot':continue
            state['truth']=t;state['truth_wall']=time.monotonic();s=stamp(t.header.stamp)
            if s<=last_truth:continue
            last_truth=s;v=t.transform
            item={'sim_time':s,'truth':[v.translation.x,v.translation.y,yaw(v.rotation)]}
            state['pending'].append(item);state['pending']=state['pending'][-200:]
            if recording:truth_samples.append(item)
    def clock(m):
        v=stamp(m.clock)
        if v!=state['sim']:state['clock_wall']=time.monotonic()
        state['sim']=v
    def contact(m,topic):
        nonlocal contact_event_count
        contact_counts[topic]=contact_counts.get(topic,0)+1
        if not recording:return
        for c in m.contacts:
            names=[c.collision1.name,c.collision2.name]
            # Ground support and robot self contact are not wall contacts.
            obstacle=any('cad_walls' in n or 'validation_obstacle' in n or 'test_obstacle' in n for n in names)
            ground=any(n.startswith('ground::') for n in names)
            self_contact=all(n.startswith('lidarbot::') for n in names)
            unclassified=not all(names) or not (ground or self_contact or obstacle)
            if obstacle or unclassified:
                contact_event_count+=1
                if len(contacts)>=200:continue
                contacts.append({'sim_time':stamp(m.header.stamp),'names':names,'topic':topic,
                    'unclassified':unclassified,'max_depth_m':max(c.depths,default=0.)})
    def bridge_status(m):
        item=json.loads(m.data);state['bridge']=item
        if recording:bridge_statuses.append(dict(item,sim_time=state['sim']))
    def display_path(m):
        if recording:display_paths.append({'sim_time':state['sim'],'points':[[v.pose.position.x,v.pose.position.y] for v in m.poses]})
    def grid(m):state['map']=m
    def command(m):
        if recording:commands.append({'sim_time':state['sim'],'v':m.linear.x,'w':m.angular.z})
    def path(m):
        if recording and m.poses:paths.append({'sim_time':state['sim'],'frame':m.header.frame_id,
            'points':[[v.pose.position.x,v.pose.position.y] for v in m.poses]})
    from sensor_msgs.msg import LaserScan
    def scan(m):state['scan_wall']=time.monotonic()
    def odom(m):state['odom_wall']=time.monotonic()
    subs.extend([node.create_subscription(TFMessage,'/gcu_ground_truth',truth,sensor),
        node.create_subscription(Clock,'/clock',clock,sensor),node.create_subscription(OccupancyGrid,'/map',grid,latched),
        node.create_subscription(Twist,'/cmd_vel',command,100),node.create_subscription(NavPath,'/plan',path,10),
        node.create_subscription(LaserScan,'/scan',scan,sensor),node.create_subscription(Odometry,'/odometry/filtered',odom,sensor)])
    for c in manifest['collisions']:
        topic=c['topic'];subs.append(node.create_subscription(Contacts,topic,lambda m,t=topic:contact(m,t),sensor))
    click_pub=node.create_publisher(PoseStamped,'/gcu/navigation_goal',10)
    cancel_pub=node.create_publisher(EmptyMessage,'/gcu/navigation_cancel',10)
    subs.extend([node.create_subscription(String,'/gcu/navigation_status',bridge_status,latched),node.create_subscription(NavPath,'/gcu/navigation_path',display_path,latched)])
    nav=ActionClient(node,NavigateToPose,'/navigate_to_pose');planner=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
    nomotion=node.create_client(Empty,'/request_nomotion_update')
    navstate=node.create_client(GetState,'/bt_navigator/get_state')
    report={'simulation_only':True,'goal':a.goal,'expected':a.expected,'ground_truth_controls_successful_stop':False,
        'sim_timeout_s':a.sim_timeout,'wall_timeout_s':a.wall_timeout,'positive_control':a.positive_control,'via_click_bridge':a.via_click,
        'gates':{'final_position_m':.20,'final_heading_deg':10.,'localization_p95_m':.25,'localization_p95_deg':5.,
            'rest_s':1.,'rest_speed_mps':.01,'rest_yaw_rate_radps':.02,'swept_radius_m':.18}}
    def step():
        nonlocal last_match,last_progress
        rclpy.spin_once(node,timeout_sec=.03)
        for item in state['pending']:
            if item['sim_time']<=last_match:continue
            try:
                t=buf.lookup_transform('map','base_footprint',Time(seconds=item['sim_time']))
            except TransformException:continue
            v=t.transform;actual=item['truth'];est=[v.translation.x,v.translation.y,yaw(v.rotation)]
            if recording:samples.append(dict(item,estimated=est,position_error_m=math.hypot(est[0]-actual[0],est[1]-actual[1]),yaw_error_deg=math.degrees(abs(wrap(est[2]-actual[2])))))
            last_match=item['sim_time']
        if time.monotonic()-last_progress>5:
            progress={'wall_elapsed_s':time.monotonic()-started,'sim_time':state['sim'],'truth':(truth_samples[-1] if truth_samples else None),'last_command':(commands[-1] if commands else None),'contact_events':contact_event_count}
            print(json.dumps(progress),flush=True);last_progress=time.monotonic()
        if time.monotonic()-started>a.wall_timeout:raise RuntimeError('wall_timeout')
    def wait(f,timeout=20.):
        end=time.monotonic()+timeout
        while not f.done():
            if time.monotonic()>end:raise RuntimeError('action/service wall timeout')
            step()
        return f.result()
    def pose():
        v=state['truth'].transform;return [v.translation.x,v.translation.y,yaw(v.rotation)]
    def rest():
        recent=[v for v in truth_samples if v['sim_time']>=state['sim']-1.3]
        return rest_window(recent)
    try:
        ready_end=time.monotonic()+60
        while True:
            step()
            if state['truth'] is not None and state['sim']>1 and state['scan_wall']>0:
                if a.positive_control:break
                if state['map'] is not None and state['odom_wall']>0 and nav.server_is_ready() and planner.server_is_ready() and buf.can_transform('map','base_footprint',Time()):
                    if navstate.service_is_ready() and wait(navstate.call_async(GetState.Request())).current_state.id==3:break
            if time.monotonic()>ready_end:raise RuntimeError('simulation/navigation readiness failed')
        if not a.positive_control:
            for i in range(6):
                if nomotion.service_is_ready():wait(nomotion.call_async(Empty.Request()))
                begin=state['sim']
                while state['sim']-begin<.3:step()
        recording=True;begin=state['sim'];report['start_truth']=pose();report['start_sim']=begin
        if a.positive_control:
            while state['sim']-begin<5.:step()
            report['passed']=any(any('validation_obstacle' in n for n in c['names']) for c in contacts)
        else:
            request=NavigateToPose.Goal();request.pose.header.frame_id='map';request.pose.header.stamp=node.get_clock().now().to_msg()
            request.pose.pose.position.x=a.goal[0];request.pose.pose.position.y=a.goal[1]
            request.pose.pose.orientation.z=math.sin(a.goal[2]/2);request.pose.pose.orientation.w=math.cos(a.goal[2]/2)
            # Preflight full static footprint; rejection cases deliberately exercise native action failure.
            m=state['map'];g=np.asarray(m.data,dtype=np.int16).reshape(m.info.height,m.info.width)
            origin=[m.info.origin.position.x,m.info.origin.position.y];r=m.info.resolution
            goal_check=validate_path(g,r,origin,[a.goal[:2]])
            report['goal_footprint']=goal_check
            if a.expected in ['success','cancelled']:
                pr=ComputePathToPose.Goal();pr.goal=request.pose;pr.planner_id='GridBased';pr.use_start=False
                ph=wait(planner.send_goal_async(pr))
                if not ph.accepted:raise RuntimeError('preflight planner rejected goal')
                result=wait(ph.get_result_async());pts=[[v.pose.position.x,v.pose.position.y] for v in result.result.path.poses]
                report['preflight']={'status':result.status,'path':pts,'validation':validate_path(g,r,origin,pts)}
                if result.status!=4 or not report['preflight']['validation']['valid']:raise RuntimeError('preflight swept footprint blocked')
            if a.via_click:
                if click_pub.get_subscription_count()!=1 or state['bridge'] is None:raise RuntimeError('click bridge not ready')
                bridge_request=state['bridge']['request']+1;click_pub.publish(request.pose)
                def finished():
                    b=state['bridge'];return b is not None and b['request']==bridge_request and b['state'] in ['succeeded','cancelled','failed','rejected']
            else:
                handle=wait(nav.send_goal_async(request));report['goal_accepted']=handle.accepted
                if not handle.accepted:raise RuntimeError('NavigateToPose goal not accepted')
                future=handle.get_result_async()
                def finished():return future.done()
            cancelled=False
            while not finished():
                step()
                if contacts:raise RuntimeError('wall/obstacle or unclassified contact observed')
                if time.monotonic()-state['truth_wall']>2 or time.monotonic()-state['clock_wall']>2 or time.monotonic()-state['scan_wall']>2 or time.monotonic()-state['odom_wall']>2:raise RuntimeError('stale sensor/clock/truth')
                if state['sim']-begin>a.sim_timeout:raise RuntimeError('sim_timeout')
                if a.expected=='cancelled' and not cancelled and math.hypot(pose()[0]-report['start_truth'][0],pose()[1]-report['start_truth'][1])>.25:
                    report['cancel_request_sim']=state['sim']
                    if a.via_click:cancel_pub.publish(EmptyMessage())
                    else:
                        response=wait(handle.cancel_goal_async());report['cancel_ack_count']=len(response.goals_canceling)
                    cancelled=True
            response=(SimpleNamespace(status={'succeeded':4,'cancelled':5,'failed':6,'rejected':6}[state['bridge']['state']]) if a.via_click else future.result())
            report['action_status']=(None if state['bridge']['state']=='rejected' else state['bridge'].get('action_status')) if a.via_click else response.status
            if a.via_click:
                report['bridge_outcome']=state['bridge']['state']
                report['goal_accepted']=any(b['state']=='executing' for b in bridge_statuses)
            report['action_end_sim']=state['sim']
            deadline=state['sim']+8
            while not rest():
                step()
                if state['sim']>deadline:raise RuntimeError('failed to rest after action result')
            report['rest_confirmed_sim']=state['sim']
            final=pose();report['final_truth']=final;report['final_position_error_m']=math.hypot(final[0]-a.goal[0],final[1]-a.goal[1])
            report['final_heading_error_deg']=math.degrees(abs(wrap(final[2]-a.goal[2])))
            # Score every observed truth segment; the sampled-radius test also covers gaps.
            pts=[v['truth'][:2] for v in truth_samples]
            score_grid=g
            if a.score_map:
                import yaml
                from PIL import Image
                cfg=yaml.safe_load(a.score_map.read_text())
                if abs(cfg['resolution']-r)>1e-8 or not np.allclose(cfg['origin'][:2],origin,atol=1e-8,rtol=0):raise RuntimeError('score map alignment mismatch')
                image=np.flipud(np.asarray(Image.open(a.score_map.parent/cfg['image'])))
                score_grid=np.where(image==254,0,np.where(image==0,100,-1))
            report['swept_truth']=validate_path(score_grid,r,origin,pts)
            pe=[v['position_error_m'] for v in samples];ye=[v['yaw_error_deg'] for v in samples]
            report['localization_pairs']=len(samples)
            report['localization_p95_position_m']=float(np.percentile(pe,95)) if pe else None
            report['localization_p95_heading_deg']=float(np.percentile(ye,95)) if ye else None
            report['max_command_mps']=max((abs(c['v']) for c in commands),default=0.)
            result_ok=(response.status==4 and report['final_position_error_m']<=.20 and report['final_heading_error_deg']<=10) if a.expected=='success' else (response.status in [5] if a.expected=='cancelled' else response.status==6)
            if a.via_click:
                report['bridge_preview_valid']=any(p['points'] and math.dist(p['points'][-1],a.goal[:2])<=r and validate_path(g,r,origin,p['points'])['valid'] for p in display_paths)
                if a.expected=='success':result_ok=result_ok and report['bridge_preview_valid']
                if a.expected=='rejected':result_ok=result_ok and bool(display_paths) and not display_paths[-1]['points']
            if a.expected=='rejected':result_ok=result_ok and not any(abs(c['v'])+abs(c['w'])>1e-5 for c in commands)
            if a.expected=='cancelled':
                stopped=rest_window(truth_samples,earliest=report['cancel_request_sim'])
                report['cancel_stop_delay_sim_s']=stopped['start_sim']-report['cancel_request_sim'] if stopped else None
                result_ok=result_ok and report['cancel_stop_delay_sim_s'] is not None and report['cancel_stop_delay_sim_s']<=1.
            report['sampling_coverage']=coverage(truth_samples,samples)
            report['passed']=bool(result_ok and not contacts and report['swept_truth']['valid'] and len(samples)>=10 and
                report['localization_p95_position_m']<=.25 and report['localization_p95_heading_deg']<=5 and report['max_command_mps']<=.10001 and report['sampling_coverage']['matched_fraction']>=.90 and report['sampling_coverage']['max_truth_gap_sim_s']<=.15)
    except Exception as e:
        report.update(passed=False,error=str(e))
        if a.via_click:cancel_pub.publish(EmptyMessage())
        if handle is not None and handle.accepted:
            try:wait(handle.cancel_goal_async(),5.)
            except Exception:pass
    finally:
        report.update(wall_elapsed_s=time.monotonic()-started,end_sim=state['sim'],contact_counts=contact_counts,
            bridge_statuses=bridge_statuses,display_paths=display_paths,contacts=contacts,contact_event_count=contact_event_count,truth=truth_samples,localization=samples,commands=commands,paths=paths)
        report['sim_elapsed_s']=state['sim']-report.get('start_sim',state['sim'])
        report['contact_publishers']={c['topic']:node.count_publishers(c['topic']) for c in manifest['collisions']}
        report['velocity_publishers']=[{'name':p.node_name,'namespace':p.node_namespace} for p in node.get_publishers_info_by_topic('/cmd_vel')]
        if not a.positive_control:report['passed']=bool(report.get('passed') and len(report['velocity_publishers'])==1 and report['velocity_publishers'][0]['name']=='controller_server')
        report['passed']=bool(report.get('passed') and all(v==1 for v in report['contact_publishers'].values()))
        a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ['truth','localization','commands','paths','contacts','display_paths','bridge_statuses']},indent=2),flush=True)
        node.destroy_node();rclpy.shutdown()
    return 0 if report.get('passed') else 1


if __name__=='__main__':raise SystemExit(main())
