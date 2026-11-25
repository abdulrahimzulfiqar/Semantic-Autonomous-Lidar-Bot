#!/usr/bin/env python3
"""Request reliable simulator odometry and IMU, forwarding original ROS bytes."""
import argparse
from collections import defaultdict
import json
import os
from pathlib import Path
import signal
import time

if __package__:
    from .arrival_probe import raw_header_stamp
else:
    from arrival_probe import raw_header_stamp


RELAY_ODOM = '/phase9/localizer_odom'
RELAY_IMU = '/phase9/localizer_imu'


def run(output):
    path = Path(os.environ['EDGEOSMAG_PHASE9_PROFILE'])
    profile = json.loads(path.read_text())
    if (os.environ.get('ROS_DOMAIN_ID') != '218' or profile.get('role') != 'navigation_pi'
            or profile.get('scope') != 'gcu_phase9_processor_in_loop'
            or not profile.get('reliable_localizer_inputs')):
        raise RuntimeError('explicit_phase9_reliable_input_profile_required')

    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from rclpy.qos import QoSProfile, qos_profile_sensor_data

    rclpy.init(args=[])
    node = rclpy.create_node('phase9_localizer_input_relay')
    rows = defaultdict(list)
    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    def receive_callback(kind, publisher):
        def receive(payload):
            stamp_ns = raw_header_stamp(payload)
            received_wall = time.monotonic()
            publisher.publish(payload)
            rows[kind].append(dict(stamp_ns=stamp_ns, wall=received_wall, forwarded_wall=time.monotonic()))
        return receive

    configurations = [
        ('wheel_odom', '/diff_controller/odom', RELAY_ODOM, Odometry, 10),
        ('imu', '/imu_broadcaster/imu', RELAY_IMU, Imu, 20),
    ]
    for kind, original, target, msg_type, depth in configurations:
        publisher = node.create_publisher(msg_type, target, qos_profile_sensor_data)
        node.create_subscription(msg_type, original, receive_callback(kind, publisher), QoSProfile(depth=depth), raw=True)

    began = time.monotonic()
    try:
        while running and rclpy.ok() and time.monotonic()-began < 360:
            rclpy.spin_once(node, timeout_sec=.025)
    finally:
        output.write_text(json.dumps(dict(scope='Pi callbacks of reliable DDS input and verbatim local forwarding',
            rows=rows, elapsed_wall_s=time.monotonic()-began), indent=2)+'\n')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    run(parser.parse_args().output)
