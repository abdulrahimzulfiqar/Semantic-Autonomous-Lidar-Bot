#!/usr/bin/env python3
"""Wait for active AMCL, simulation clock and map-to-base TF.

Read-only by default. --request-updates allows up to six stationary AMCL sensor
updates after the prior/odometry are ready, without publishing pose or motion.
"""
import argparse
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from lifecycle_msgs.srv import GetState
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer,TransformListener


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--timeout',type=float,default=45)
    parser.add_argument('--request-updates',action='store_true');args=parser.parse_args()
    rclpy.init(args=[]);node=Node('gcu_wait_localization',parameter_overrides=[Parameter('use_sim_time',value=True)])
    buffer=Buffer();listener=TransformListener(buffer,node);client=node.create_client(GetState,'/amcl/get_state')
    update=node.create_client(Empty,'/request_nomotion_update')
    scan_state={'count':0,'stamp':None}
    def scan(m):scan_state.update(count=scan_state['count']+1,stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9)
    sub=node.create_subscription(LaserScan,'/scan',scan,qos_profile_sensor_data)
    started=time.monotonic();future=None;active=False;ready=False;last_state=0.;last_update=0.;updates=0
    while time.monotonic()-started<args.timeout:
        rclpy.spin_once(node,timeout_sec=.05);now=time.monotonic()
        if future is None and client.service_is_ready() and now-last_state>.5:
            future=client.call_async(GetState.Request());last_state=now
        if future is not None and future.done():
            active=future.result() is not None and future.result().current_state.label=='active';future=None
        clock=node.get_clock().now().nanoseconds/1e9
        if active and clock>0 and buffer.can_transform('map','base_footprint',Time()):ready=True;break
        if args.request_updates and active and clock>0 and scan_state['count']>0 and updates<6 and now-last_update>.5:
            if update.service_is_ready() and buffer.can_transform('odom','base_footprint',Time()):
                update.call_async(Empty.Request());updates+=1;last_update=now
    print(json.dumps({'ready':ready,'amcl_active':active,'wall_seconds':time.monotonic()-started,
        'clock_sim_s':node.get_clock().now().nanoseconds/1e9,'scan':scan_state,'nomotion_requests':updates,
        'odom_to_base':buffer.can_transform('odom','base_footprint',Time()),
        'base_to_lidar':buffer.can_transform('base_footprint','lidar_link',Time()),'frames':buffer.all_frames_as_yaml()}),flush=True)
    node.destroy_node();rclpy.shutdown();return 0 if ready else 1


if __name__=='__main__':raise SystemExit(main())
