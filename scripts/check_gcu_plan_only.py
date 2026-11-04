#!/usr/bin/env python3
"""Stationary costmap and preview acceptance checks. Publishes preview goals only."""
import argparse
import json
import math
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped,Twist,PolygonStamped
from nav_msgs.msg import Path as NavPath,OccupancyGrid
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from lifecycle_msgs.srv import GetState
from std_srvs.srv import Empty
from tf2_ros import Buffer,TransformListener
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from gcu_plan_validation import cell_at,validate_path


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    rclpy.init();node=Node('gcu_plan_only_check',parameter_overrides=[Parameter('use_sim_time',value=True)])
    buffer=Buffer();listener=TransformListener(buffer,node)
    latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    state={'map':None,'global':None,'local':None,'path':None,'status':None,'nonzero_commands':0,'truth':None,'footprint':None}
    subscriptions=[]
    for key,topic in [('map','/map'),('global','/global_costmap/costmap'),('local','/costmap/costmap')]:
        subscriptions.append(node.create_subscription(OccupancyGrid,topic,lambda msg,k=key:state.__setitem__(k,msg),latched))
    subscriptions.append(node.create_subscription(NavPath,'/gcu/preview_path',lambda msg:state.__setitem__('path',msg),latched))
    subscriptions.append(node.create_subscription(String,'/gcu/preview_status',lambda msg:state.__setitem__('status',json.loads(msg.data)),latched))
    subscriptions.append(node.create_subscription(PolygonStamped,'/costmap/published_footprint',lambda msg:state.__setitem__('footprint',msg),10))
    def command(msg):
        state['nonzero_commands']+=int(abs(msg.linear.x)+abs(msg.linear.y)+abs(msg.angular.z)>1e-6)
    for topic in ['/cmd_vel','/diff_controller/cmd_vel_unstamped']:
        subscriptions.append(node.create_subscription(Twist,topic,command,10))
    def truth(msg):
        for t in msg.transforms:
            if t.child_frame_id.strip('/').split('::')[-1]=='lidarbot':state['truth']=[t.transform.translation.x,t.transform.translation.y]
    subscriptions.append(node.create_subscription(TFMessage,'/gcu_ground_truth',truth,10))
    publisher=node.create_publisher(PoseStamped,'/gcu/preview_goal',10)
    action=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
    update=node.create_client(Empty,'/request_nomotion_update')
    def spin_for(seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
    start=time.monotonic();report={'plan_only':True,'velocity_commands_sent':0,'cases':[]};asked=False
    try:
        while time.monotonic()-start<60:
            rclpy.spin_once(node,timeout_sec=.1)
            if not asked and time.monotonic()-start>4 and update.service_is_ready():update.call_async(Empty.Request());asked=True
            if all(state[k] is not None for k in ['map','global','local','status','truth','footprint']) and action.server_is_ready() and buffer.can_transform('map','base_footprint',Time()):break
        else:raise RuntimeError('Costmap/planner/AMCL/preview readiness timeout')
        initial=list(state['truth']);m=state['map'];grid=np.asarray(m.data).reshape(m.info.height,m.info.width);res=m.info.resolution;origin=(m.info.origin.position.x,m.info.origin.position.y)
        occupied=np.argwhere(grid>0);best=min(occupied,key=lambda rc:math.hypot(origin[0]+(rc[1]+.5)*res-initial[0],origin[1]+(rc[0]+.5)*res-initial[1]));row,col=map(int,best)
        wall=(origin[0]+(col+.5)*res,origin[1]+(row+.5)*res)
        near=None
        for dr,dc in [(0,1),(0,-1),(1,0),(-1,0)]:
            if 0<=row+dr<grid.shape[0] and 0<=col+dc<grid.shape[1] and grid[row+dr,col+dc]==0:
                near=(wall[0]+dc*res,wall[1]+dr*res);break
        cases=[('short_corridor',(initial[0]+2,initial[1]),'map','ready'),('around_courtyard',(0.,20.),'map','ready'),
            ('room_preview',(-12.,0.),'map','ready'),('unknown_courtyard',(0.,0.),'map','rejected'),
            ('occupied_wall',wall,'map','rejected'),('wrong_frame',(initial[0]+2,initial[1]),'odom','rejected')]
        if near is not None:cases.append(('insufficient_goal_clearance',near,'map','rejected'))
        for name,point,frame,expected in cases:
            old=state['status']['request']
            goal=PoseStamped();goal.header.frame_id=frame;goal.header.stamp=node.get_clock().now().to_msg();goal.pose.position.x=float(point[0]);goal.pose.position.y=float(point[1]);goal.pose.orientation.w=1.
            publisher.publish(goal);begin=time.monotonic()
            while time.monotonic()-begin<20:
                rclpy.spin_once(node,timeout_sec=.05)
                status=state['status']
                if status and status['request']>old and status['state'] in ['ready','rejected']:break
            else:raise RuntimeError('Preview response timeout for '+name)
            spin_for(.3);status=dict(state['status']);path=state['path'];points=[(p.pose.position.x,p.pose.position.y) for p in path.poses] if path else []
            item={'name':name,'goal':list(point),'frame':frame,'expected':expected,'status':status,'path_poses':len(points),'latency_wall_s':time.monotonic()-begin}
            if expected=='ready':
                validation=validate_path(grid,res,origin,points);item['independent_static_validation']=validation
                item['path_xy']=points
                item['passed']=bool(status['state']=='ready' and len(points)>1 and validation['valid'] and math.hypot(points[-1][0]-point[0],points[-1][1]-point[1])<=res)
            else:item['passed']=status['state']=='rejected' and not points
            report['cases'].append(item)
        lifecycle={}
        for name in ['planner_server','costmap/costmap']:
            client=node.create_client(GetState,'/'+name+'/get_state')
            if client.wait_for_service(timeout_sec=2):
                future=client.call_async(GetState.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=3)
                lifecycle[name]=future.result().current_state.label if future.done() and future.result() else None
        g=state['global'];ggrid=np.asarray(g.data).reshape(g.info.height,g.info.width)
        report['global_courtyard_value']=cell_at(ggrid,g.info.resolution,(g.info.origin.position.x,g.info.origin.position.y),0.,0.)
        report['costmaps']={'global':{'width':g.info.width,'height':g.info.height,'frame':g.header.frame_id},'local':{'width':state['local'].info.width,'height':state['local'].info.height,'frame':state['local'].header.frame_id}}
        polygon=state['footprint'].polygon.points;cx=sum(p.x for p in polygon)/len(polygon);cy=sum(p.y for p in polygon)/len(polygon)
        report['published_footprint_max_radius_m']=max(math.hypot(p.x-cx,p.y-cy) for p in polygon)
        report['lifecycle']=lifecycle;report['nonzero_commands_observed']=state['nonzero_commands']
        report['truth_drift_m']=math.hypot(state['truth'][0]-initial[0],state['truth'][1]-initial[1])
        report['navigation_controller_present']='controller_server' in node.get_node_names()
        report['passed']=bool(all(c['passed'] for c in report['cases']) and all(v=='active' for v in lifecycle.values()) and len(lifecycle)==2 and report['global_courtyard_value']==-1 and report['published_footprint_max_radius_m']>=.18 and state['nonzero_commands']==0 and report['truth_drift_m']<.01 and not report['navigation_controller_present'])
    except Exception as error:report.update(passed=False,error=str(error))
    finally:
        report['wall_seconds']=time.monotonic()-start
        report['limitations']='Stationary costmaps and static swept-circle preview only; no NavigateToPose, contacts, autonomous execution or portal-wide feasibility acceptance.'
        args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k!='cases'},indent=2),flush=True)
        node.destroy_node();rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
