#!/usr/bin/env python3
"""Click-to-preview metric paths. Never publishes velocity or navigation goals.

Publish map-frame PoseStamped to /gcu/preview_goal. Validated full paths and JSON
status appear on /gcu/preview_path and /gcu/preview_status (latched).
"""
import json
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy
from rclpy.clock import Clock,ClockType
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path,OccupancyGrid
from nav2_msgs.action import ComputePathToPose
from std_msgs.msg import String
from gcu_plan_validation import cell_at,validate_path


class Preview(Node):
    def __init__(self,prefix='/gcu/preview',plan_only=True):
        super().__init__('gcu_goal_preview',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.plan_only=plan_only
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub=self.create_publisher(Path,prefix+'_path',latched)
        self.status_pub=self.create_publisher(String,prefix+'_status',latched)
        self.target_pub=self.create_publisher(PoseStamped,prefix+'_target',latched)
        self.map_sub=self.create_subscription(OccupancyGrid,'/map',self.map_callback,latched)
        self.goal_sub=self.create_subscription(PoseStamped,prefix+'_goal',self.goal_callback,10)
        self.client=ActionClient(self,ComputePathToPose,'/compute_path_to_pose')
        self.map=None;self.current_goal=None;self.generation=0;self.goal_handle=None;self.pending=False;self.deadline=0
        self.timer=self.create_timer(.25,self.check_timeout,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.status('idle',detail='Plan-only preview' if plan_only else 'Validated click-to-navigate enabled')

    def map_callback(self,msg):
        self.map=msg

    def status(self,state,**fields):
        self.status_pub.publish(String(data=json.dumps({'state':state,'request':self.generation,'plan_only':self.plan_only,**fields})))

    def clear(self):
        path=Path();path.header.frame_id='map';path.header.stamp=self.get_clock().now().to_msg();self.path_pub.publish(path)

    def goal_callback(self,goal):
        self.generation+=1;request=self.generation
        if self.goal_handle is not None:self.goal_handle.cancel_goal_async()
        self.goal_handle=None;self.pending=False;self.clear()
        q=goal.pose.orientation;p=goal.pose.position
        if goal.header.frame_id!='map':self.status('rejected',reason='goal_frame_must_be_map');return
        values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
        if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in [q.x,q.y,q.z,q.w])-1)>.01:
            self.status('rejected',reason='invalid_goal_pose');return
        if self.map is None:self.status('rejected',reason='map_not_ready');return
        m=self.map;rotation=m.info.origin.orientation
        if abs(rotation.x)+abs(rotation.y)+abs(rotation.z)>1e-8:self.status('rejected',reason='rotated_map_unsupported');return
        self.target_pub.publish(goal)
        grid=np.asarray(m.data).reshape(m.info.height,m.info.width);origin=(m.info.origin.position.x,m.info.origin.position.y)
        value=cell_at(grid,m.info.resolution,origin,p.x,p.y)
        if value is None or value!=0:
            self.status('rejected',reason='outside_map' if value is None else 'unknown_goal' if value<0 else 'occupied_goal');return
        check=validate_path(grid,m.info.resolution,origin,[(p.x,p.y)])
        if not check['valid']:self.status('rejected',reason='goal_footprint_blocked',validation=check);return
        if not self.client.server_is_ready():self.status('rejected',reason='planner_not_ready');return
        self.current_goal=goal;self.status('planning',goal={'x':p.x,'y':p.y})
        action=ComputePathToPose.Goal();action.goal=goal;action.planner_id='GridBased';action.use_start=False
        self.pending=True;self.deadline=time.monotonic()+15
        future=self.client.send_goal_async(action)
        future.add_done_callback(lambda f:self.accepted(f,request))

    def accepted(self,future,request):
        try:handle=future.result()
        except Exception as error:
            if request==self.generation:self.pending=False;self.status('rejected',reason='planner_request_error',detail=str(error))
            return
        if request!=self.generation:
            if handle.accepted:handle.cancel_goal_async()
            return
        if not handle.accepted:self.pending=False;self.status('rejected',reason='planner_rejected_request');return
        self.goal_handle=handle
        handle.get_result_async().add_done_callback(lambda f:self.finished(f,request))

    def finished(self,future,request):
        if request!=self.generation:return
        self.pending=False;self.goal_handle=None
        try:response=future.result()
        except Exception as error:self.clear();self.status('rejected',reason='planner_result_error',detail=str(error));return
        path=response.result.path
        if response.status!=4 or not path.poses:
            self.clear();self.status('rejected',reason='no_valid_plan',action_status=response.status);return
        if path.header.frame_id!='map':self.clear();self.status('rejected',reason='unexpected_path_frame');return
        m=self.map;grid=np.asarray(m.data).reshape(m.info.height,m.info.width)
        points=[(p.pose.position.x,p.pose.position.y) for p in path.poses]
        endpoint_error=math.hypot(points[-1][0]-self.current_goal.pose.position.x,points[-1][1]-self.current_goal.pose.position.y)
        if endpoint_error>m.info.resolution:
            self.clear();self.status('rejected',reason='path_does_not_reach_goal',endpoint_error_m=endpoint_error);return
        check=validate_path(grid,m.info.resolution,(m.info.origin.position.x,m.info.origin.position.y),points)
        if not check['valid']:self.clear();self.status('rejected',reason='path_footprint_blocked',validation=check);return
        length=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(points,points[1:]))
        self.validated(path,{'poses':len(points),'length_m':length,'endpoint_goal_error_m':endpoint_error,'validation':check})

    def validated(self,path,fields):
        self.path_pub.publish(path);self.status('ready',**fields)

    def check_timeout(self):
        if self.pending and time.monotonic()>self.deadline:
            self.generation+=1;self.pending=False
            if self.goal_handle is not None:self.goal_handle.cancel_goal_async()
            self.goal_handle=None;self.clear();self.status('rejected',reason='planning_wall_timeout')


def main():
    rclpy.init();node=Preview()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
