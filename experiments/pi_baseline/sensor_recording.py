#!/usr/bin/env python3
"""Capture original ROS simulator inputs for a portable loopback replay."""
import argparse
import base64
import gzip
import json
import os
from pathlib import Path
import signal
import time


def message_types(scan_topic='/scan'):
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, LaserScan
    from tf2_msgs.msg import TFMessage
    return {
        '/scan': (scan_topic, LaserScan),
        '/imu_broadcaster/imu': ('/imu_broadcaster/imu', Imu),
        '/diff_controller/odom': ('/diff_controller/odom', Odometry),
        '/tf_static': ('/tf_static', TFMessage),
    }


def capture(path, scan_topic):
    if os.environ.get('ROS_DOMAIN_ID') != '218':
        raise RuntimeError('phase9_domain_required')
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from rclpy.serialization import serialize_message

    running = True
    def stop(*_):
        nonlocal running
        running = False

    rclpy.init(args=[])
    node = rclpy.create_node('phase9_sensor_capture')
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    rows = []

    def record(key, message):
        stamp = getattr(getattr(message, 'header', None), 'stamp', None)
        stamp_ns = stamp.sec * 10**9 + stamp.nanosec if stamp is not None else 0
        rows.append((key, stamp_ns, base64.b64encode(serialize_message(message)).decode('ascii')))

    for key, (topic, kind) in message_types(scan_topic).items():
        qos = (QoSProfile(depth=64, durability=DurabilityPolicy.TRANSIENT_LOCAL)
               if key == '/tf_static' else qos_profile_sensor_data)
        node.create_subscription(kind, topic, lambda message, key=key: record(key, message), qos)
    try:
        deadline = time.monotonic() + 340
        while running and rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.025)
    finally:
        with gzip.open(path, 'xt', compresslevel=3) as stream:
            json.dump({'format': 1, 'source_scan_topic': scan_topic, 'rows': rows}, stream)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('role', choices=['capture'])
    parser.add_argument('--recording', type=Path, required=True)
    parser.add_argument('--scan-topic', default='/scan')
    args = parser.parse_args()
    capture(args.recording, args.scan_topic)
