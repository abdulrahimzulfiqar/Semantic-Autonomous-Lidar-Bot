#!/usr/bin/env python3
"""Exercise live motion-free preview, frame guards, closures and TF ownership."""
import argparse
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--assets',type=Path,required=True);args=parser.parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rcl_interfaces.srv import SetParameters
    from rclpy.time import Time
    from rclpy.duration import Duration
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    from geometry_msgs.msg import PoseStamped,TransformStamped,Twist
    from nav_msgs.msg import Path as NavPath
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer,TransformListener
    from gcu_plan_validation import validate_path
    import numpy as np
    from PIL import Image
    import yaml
    cfg=yaml.safe_load((args.assets/'gcu_ground_floor.yaml').read_text())
    image=np.flipud(np.asarray(Image.open(args.assets/cfg['image'])))
    grid=np.where(image==254,0,np.where(image==0,100,-1))
    rclpy.init();node=Node('gcu_osmag_preview_evaluator',parameter_overrides=[Parameter('use_sim_time',value=True)])
    qos=QoSProfile(depth=20,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sensor=QoSProfile(depth=200,reliability=ReliabilityPolicy.BEST_EFFORT)
    statuses=[];paths=[];graph_paths=[];commands=[];truth=[];ownership={}
    def path(msg):paths.append({'frame':msg.header.frame_id,'points':[[p.pose.position.x,p.pose.position.y] for p in msg.poses]})
    def ownership_report(msg):ownership.update(json.loads(msg.data))
    def ground(msg):
        for t in msg.transforms:
            if t.child_frame_id=='lidarbot':truth.append([t.transform.translation.x,t.transform.translation.y])
    subs=[node.create_subscription(String,'/osmag/route_status',lambda m:statuses.append(json.loads(m.data)),qos),
          node.create_subscription(NavPath,'/osmag/global_path',path,qos),
          node.create_subscription(NavPath,'/osmag/graph_path',lambda m:graph_paths.append(
              {'frame':m.header.frame_id,'points':[[p.pose.position.x,p.pose.position.y] for p in m.poses]}),qos),
          node.create_subscription(Twist,'/cmd_vel',lambda m:commands.append([m.linear.x,m.angular.z]),sensor),
          node.create_subscription(String,'/gcu/tf_ownership_report',ownership_report,qos),
          node.create_subscription(TFMessage,'/gcu_ground_truth',ground,sensor)]
    pub=node.create_publisher(PoseStamped,'/osmag/preview_goal',10)
    fake_tf=node.create_publisher(TFMessage,'/tf',10)
    duplicate_tf=node.create_publisher(TFMessage,'/tf',10)
    params=node.create_client(SetParameters,'/gcu_osmag_route_preview/set_parameters')
    buffer=Buffer();listener=TransformListener(buffer,node)
    result={'passed':False,'cases':[],'motion_commands_sent_by_evaluator':False}
    def wait(predicate,seconds=30):
        end=time.monotonic()+seconds
        while not predicate():
            if time.monotonic()>end:raise RuntimeError('preview_evaluation_timeout')
            rclpy.spin_once(node,timeout_sec=.03)
    def set_parameter(name,value):
        request=SetParameters.Request();request.parameters=[Parameter(name,value=value).to_parameter_msg()]
        future=params.call_async(request);wait(future.done,5)
        assert all(v.successful for v in future.result().results)
    def request(name,point,expected,reason=None,frame='map',orientation=True):
        previous=max((s['request'] for s in statuses),default=0);index=len(paths);graph_index=len(graph_paths)
        msg=PoseStamped();msg.header.frame_id=frame;msg.pose.position.x=float(point[0]);msg.pose.position.y=float(point[1]);msg.pose.orientation.w=1. if orientation else 0.
        pub.publish(msg)
        wait(lambda:any(s['request']>previous and s['state'] in ['ready','rejected','failed'] for s in statuses))
        status=next(s for s in reversed(statuses) if s['request']>previous)
        if expected=='ready':wait(lambda:len(paths)>index and bool(paths[-1]['points']) and math.dist(paths[-1]['points'][-1],point)<1e-6,5)
        else:wait(lambda:len(paths)>index and not paths[-1]['points'],5)
        wait(lambda:len(graph_paths)>graph_index and bool(graph_paths[-1]['points'])==(expected=='ready'),5)
        ok=status['state']==expected and (reason is None or status.get('reason')==reason)
        validation=None
        if expected=='ready':
            validation=validate_path(grid,cfg['resolution'],cfg['origin'][:2],paths[-1]['points'])
            ok=ok and paths[-1]['frame']=='AGmap' and validation['valid'] and not status['motion_enabled']
            distance=lambda points:sum(math.dist(a,b) for a,b in zip(points,points[1:]))
            ok=ok and abs(distance(paths[-1]['points'])-status['refined_length_m'])<1e-7
            ok=ok and abs(distance(graph_paths[-1]['points'])-status['graph_cost_m'])<1e-7
            ok=ok and status['refined_length_m']<=status['graph_cost_m']+1e-7
            ok=ok and status['refinement']['topology']['passage_ids']==status['passage_ids']
            if name in ['short_goal','diagonal_goal','downward_goal']:
                ok=ok and abs(status['refined_length_m']-math.dist(status['start'],point))<1e-6
        result['cases'].append({'name':name,'passed':bool(ok),'status':status,'path':paths[-1],
                                'graph_path':graph_paths[-1], 'independent_footprint':validation})
        if not ok:raise RuntimeError('preview_case_failed:'+name)
    try:
        wait(lambda:bool(statuses) and statuses[-1].get('ready',False) and bool(ownership)
             and params.service_is_ready() and buffer.can_transform('map','base_footprint',Time()),90)
        for _ in range(20):rclpy.spin_once(node,timeout_sec=.03)
        t=buffer.lookup_transform('map','base_footprint',Time()).transform.translation
        request('same_sector',[t.x,t.y],'ready')
        request('short_goal',[1.4413560486865775,-12.960313240266368],'ready')
        request('diagonal_goal',[5.,-8.],'ready')
        request('downward_goal',[.5,-16.],'ready')
        request('room_route',[-10.,-7.5],'ready')
        request('courtyard',[0.,0.],'rejected','unknown_goal')
        request('occupied',[-1.4074548315221413,-16.244374513818027],'rejected','occupied_goal')
        request('outside',[200.,200.],'rejected','outside_goal')
        request('unsupported_frame',[1.,-12.],'rejected','goal_frame_must_be_map',frame='odom')
        request('invalid_orientation',[1.,-12.],'rejected','invalid_goal_orientation',orientation=False)
        set_parameter('base_frame','missing_robot_pose_fixture')
        request('missing_tf',[1.,-12.],'rejected','robot_tf_unavailable')
        old=TransformStamped();old.header.frame_id='map';old.child_frame_id='stale_robot_pose_fixture';old.header.stamp=(node.get_clock().now()-Duration(seconds=5.)).to_msg();old.transform.rotation.w=1.
        fake_tf.publish(TFMessage(transforms=[old]));set_parameter('base_frame','stale_robot_pose_fixture')
        for _ in range(20):rclpy.spin_once(node,timeout_sec=.02)
        request('stale_tf',[1.,-12.],'rejected','robot_tf_stale')
        set_parameter('base_frame','base_footprint')
        osm=ET.parse(args.assets/'gcu_ground_floor.osm').getroot()
        all_passages=[w.get('id') for w in osm.findall('way') if any(t.get('k')=='osmAG:type' and t.get('v')=='passage' for t in w.findall('tag'))]
        set_parameter('disabled_passages',all_passages)
        request('disconnected_goal',[-10.,-7.5],'rejected','no_feasible_graph_route')
        set_parameter('disabled_passages',['-93'])
        request('selected_passage_closed',[1.4413560486865775,-12.960313240266368],'ready')
        assert '-93' not in result['cases'][-1]['status']['passage_ids']
        set_parameter('disabled_passages',[''])
        request('repeated_goal',[-10.,-7.5],'ready')
        fixture=TransformStamped();fixture.header.frame_id='gcu_probe_parent';fixture.child_frame_id='gcu_probe_child'
        fixture.header.stamp=node.get_clock().now().to_msg();fixture.transform.rotation.w=1.
        for _ in range(5):
            fake_tf.publish(TFMessage(transforms=[fixture]));duplicate_tf.publish(TFMessage(transforms=[fixture]))
            rclpy.spin_once(node,timeout_sec=.03)
        wait(lambda:ownership.get('edge_publishers',{}).get('gcu_probe_parent->gcu_probe_child')==2,5)
        result['duplicate_tf_fixture_detected']=True
        result['velocity_publishers']=node.count_publishers('/cmd_vel')
        result['navigation_action_publishers']={n:node.count_publishers(n+'/_action/status') for n in ['/navigate_to_pose','/follow_path']}
        result['tf_edge_publishers']={n:v for n,v in ownership['edge_publishers'].items() if n!='gcu_probe_parent->gcu_probe_child'}
        result['command_messages']=commands
        result['truth_samples']=len(truth)
        result['maximum_truth_drift_m']=max((math.dist(truth[0],p) for p in truth),default=None)
        result['passed']=bool(all(r['passed'] for r in result['cases']) and result['velocity_publishers']==0 and not commands
            and all(v==0 for v in result['navigation_action_publishers'].values())
            and result['tf_edge_publishers'].get('map->odom')==1 and result['tf_edge_publishers'].get('odom->base_footprint')==1
            and result['tf_edge_publishers'].get('AGmap->map')==1
            and result['maximum_truth_drift_m'] is not None and result['maximum_truth_drift_m']<.01)
    except Exception as error:result['error']=repr(error)
    finally:
        result.update(statuses=statuses,paths=paths)
        args.output.write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k not in ['cases','paths','statuses']}))
        node.destroy_node();rclpy.shutdown()
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
