#!/usr/bin/env python3
"""Independent Pi process: read-only clock/sensor/TF callback receipt trace."""
import argparse
from collections import defaultdict,deque
import json
import os
from pathlib import Path
import signal
import struct
import time


def raw_header_stamp(payload):
    """Read the first field of a ROS 2 message whose first member is Header."""
    order = {b'\x00\x00': '>', b'\x00\x01': '<'}.get(payload[:2])
    if order is None or len(payload) < 12:
        raise ValueError('unsupported_stamp_encoding')
    seconds, nanos = struct.unpack_from(order+'iI', payload, 4)
    if nanos >= 1_000_000_000:
        raise ValueError('invalid_header_stamp')
    return seconds * 1_000_000_000 + nanos


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--pipeline', choices=['pi', 'simulator'])
    a=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='218':p.error('phase9_domain_required')
    import rclpy
    from rclpy.qos import DurabilityPolicy,QoSProfile,ReliabilityPolicy,qos_profile_sensor_data
    from rclpy.serialization import serialize_message
    from nav_msgs.msg import Odometry,OccupancyGrid
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan,Imu
    from tf2_msgs.msg import TFMessage
    node_name = 'phase9_arrival_'+a.pipeline if a.pipeline else 'phase9_independent_arrival_probe'
    rclpy.init(args=[]);node=rclpy.create_node(node_name)
    rows={};counts=defaultdict(int);running=True
    def append(kind,stamp,**extra):
        rows.setdefault(kind,deque(maxlen=40000)).append(dict(wall=time.monotonic(),stamp=stamp,**extra));counts[kind]+=1
    def stamp(t):return t.sec+t.nanosec/1e9

    def pipeline_row(kind, stamp_ns, info, **extra):
        info = info or {}
        append(kind, stamp_ns/1e9, stamp_ns=stamp_ns, unix_ns=time.time_ns(),
               dds_source_ns=info.get('source_timestamp', 0),
               dds_received_ns=info.get('received_timestamp', 0), **extra)

    def probe_header(kind, topic, message_type, qos):
        sample = message_type()
        sample.header.stamp.sec = 123
        sample.header.stamp.nanosec = 456_789
        if raw_header_stamp(serialize_message(sample)) != 123_000_456_789:
            raise RuntimeError('unexpected_header_encoding:'+topic)

        def receive(payload, info=None):
            pipeline_row(kind, raw_header_stamp(payload), info)

        return node.create_subscription(message_type, topic, receive, qos, raw=True)

    def transforms(m):
        for t in m.transforms:
            if (t.header.frame_id,t.child_frame_id) in [('odom','base_footprint'),('map','odom')]:
                append('tf',stamp(t.header.stamp),parent=t.header.frame_id,child=t.child_frame_id)
    subscriptions = []
    if a.pipeline:
        subscriptions.append(probe_header('wheel_odom', '/diff_controller/odom', Odometry, qos_profile_sensor_data))
        subscriptions.append(probe_header('imu_input', '/imu_broadcaster/imu', Imu, qos_profile_sensor_data))
    if a.pipeline == 'pi':
        subscriptions.append(probe_header('filtered_odom', '/odometry/filtered', Odometry, qos_profile_sensor_data))
        grid_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.BEST_EFFORT)
        for kind in ('global', 'local'):
            subscriptions.append(probe_header('grid_'+kind, '/'+kind+'_costmap/costmap', OccupancyGrid, grid_qos))

        def pipeline_tf(message, info=None):
            for transform in message.transforms:
                parent, child = transform.header.frame_id, transform.child_frame_id
                if (parent, child) in [('odom', 'base_footprint'), ('map', 'odom')]:
                    t = transform.header.stamp
                    pipeline_row('tf', t.sec*1_000_000_000+t.nanosec, info, parent=parent, child=child)

        subscriptions.append(node.create_subscription(TFMessage, '/tf', pipeline_tf,
                             QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)))
    if a.pipeline != 'simulator':
        subscriptions.append(node.create_subscription(Clock,'/clock',lambda m:append('clock',stamp(m.clock)),QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)))
        subscriptions.append(node.create_subscription(LaserScan,'/scan',lambda m:append('scan',stamp(m.header.stamp)),qos_profile_sensor_data))
    if not a.pipeline:
        subscriptions.append(node.create_subscription(Imu,'/imu_broadcaster/imu',lambda m:append('imu',stamp(m.header.stamp)),qos_profile_sensor_data))
        subscriptions.append(node.create_subscription(TFMessage,'/tf',transforms,QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT)))
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    began=time.monotonic()
    try:
        while running and rclpy.ok() and time.monotonic()-began<360:rclpy.spin_once(node,timeout_sec=.01)
    finally:
        a.output.write_text(json.dumps(dict(scope='Independent callback receipts and available DDS metadata. Monotonic times belong to the local host.',
            pipeline=a.pipeline,
            elapsed_wall_s=time.monotonic()-began,total_calls=dict(counts),maximum_rows_per_kind=40000,rows={k:list(v) for k,v in rows.items()}),indent=2)+'\n')
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
