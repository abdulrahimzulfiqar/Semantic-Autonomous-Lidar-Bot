#!/usr/bin/env python3
"""Replay sensor-sized traffic over ROS DDS without starting any navigation node."""
import argparse
import base64
from collections import defaultdict
import json
import os
from pathlib import Path
import re
import struct
import time

if __package__:
    from .arrival_probe import raw_header_stamp
    from .replay_stack import load_clip
    from .sensor_recording import message_types
else:
    from arrival_probe import raw_header_stamp
    from replay_stack import load_clip
    from sensor_recording import message_types


CLIP_START_NS = 10_000_000_000
CLIP_END_NS = 15_000_000_000
STREAM_NAMES = {'/scan': 'scan', '/imu_broadcaster/imu': 'imu', '/diff_controller/odom': 'wheel_odom'}


def with_stamp(payload, stamp_ns):
    raw_header_stamp(payload)
    stamp_struct = '<iI' if payload[:2] == b'\x00\x01' else '>iI'
    return payload[:4] + struct.pack(stamp_struct, *divmod(stamp_ns, 1_000_000_000)) + payload[12:]


def run(args):
    if os.environ.get('ROS_DOMAIN_ID') != '217' or os.environ.get('ROS_LOCALHOST_ONLY') != '0':
        raise RuntimeError('isolated_phase9_link_domain_required')
    import rclpy
    from rclpy.qos import QoSProfile, qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock

    rclpy.init(args=[])
    node = rclpy.create_node(f'phase9_link_{args.session}_{args.role}')
    prefix = f'/phase9_link_load/{args.session}/'
    recorded_types = message_types()
    types = {}
    for topic, name in STREAM_NAMES.items():
        qos = qos_profile_sensor_data
        if name == 'scan':
            qos = QoSProfile(depth=5)
        elif args.reliable_inputs:
            qos = QoSProfile(depth=32)
        types[name] = recorded_types[topic][1], qos
    types['clock'] = Clock, qos_profile_sensor_data
    rows = defaultdict(list)

    def record(kind, stamp_ns, payload_bytes, **extra):
        rows[kind].append(dict(stamp_ns=stamp_ns, wall=time.monotonic(), payload_bytes=payload_bytes, **extra))

    # The native ROS types and QoS are used on namespaced diagnostic topics.
    # No robot topic, action or velocity output is created here.
    try:
        if args.role == 'source':
            publishers = {name: node.create_publisher(kind, prefix+name, qos)
                          for name, (kind, qos) in types.items()}
            clip, _, _ = load_clip(args.recording, CLIP_START_NS, CLIP_END_NS)
            samples = [(stamp-CLIP_START_NS, STREAM_NAMES[topic], base64.b64decode(payload))
                       for (topic, stamp), payload in clip]
            clock = Clock()
            for offset in range(0, CLIP_END_NS-CLIP_START_NS, 10_000_000):
                samples.append((offset, 'clock', None))
            samples.sort(key=lambda sample: sample[0])
            discovery_deadline = time.monotonic()+15
            while not all(p.get_subscription_count() for p in publishers.values()):
                if time.monotonic() >= discovery_deadline:
                    raise RuntimeError('sensor_link_peer_discovery_timeout')
                time.sleep(.025)
            began = time.monotonic_ns()
            for cycle in range(args.duration//5):
                for offset, kind, payload in samples:
                    due = cycle*(CLIP_END_NS-CLIP_START_NS)+offset
                    stamp_ns = 3_000_000_000+due
                    remaining = began+due-time.monotonic_ns()
                    if remaining > 0:
                        time.sleep(remaining/1e9)
                    lateness_ns = time.monotonic_ns()-began-due
                    if lateness_ns > 2_000_000_000:
                        raise RuntimeError('sensor_link_source_stalled')
                    if kind == 'clock':
                        clock.clock.sec, clock.clock.nanosec = divmod(stamp_ns, 1_000_000_000)
                        publishers[kind].publish(clock)
                        size = 12
                    else:
                        publishers[kind].publish(with_stamp(payload, stamp_ns))
                        size = len(payload)
                    record(kind, stamp_ns, size, lateness_ns=lateness_ns)
            time.sleep(1)
        else:
            def make_receiver(kind):
                def receive(payload):
                    record(kind, raw_header_stamp(payload), len(payload))
                return receive

            for name, (msg_type, qos) in types.items():
                node.create_subscription(msg_type, prefix+name, make_receiver(name), qos, raw=True)
            first_receipt_deadline = time.monotonic()+25
            while not rows and time.monotonic()<first_receipt_deadline:
                rclpy.spin_once(node, timeout_sec=.05)
            if not rows:
                raise RuntimeError('sensor_link_no_receipts')
            end = time.monotonic()+args.duration+4
            while time.monotonic()<end:
                rclpy.spin_once(node, timeout_sec=.05)
    finally:
        args.output.write_text(json.dumps(dict(scope='Namespaced no-motion ROS sensor traffic probe',
            role=args.role, session=args.session, duration_s=args.duration,
            reliable_inputs=args.reliable_inputs, rows=rows), indent=2)+'\n')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--role', choices=['source', 'receiver'], required=True)
    parser.add_argument('--session', required=True)
    parser.add_argument('--duration', type=int, choices=range(5, 31, 5), default=20)
    parser.add_argument('--recording', type=Path)
    parser.add_argument('--reliable-inputs', action='store_true', help='use reliable wheel odometry and IMU delivery')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not re.fullmatch('[a-zA-Z0-9_]{1,24}', args.session):
        parser.error('invalid_session')
    if args.role == 'source' and args.recording is None:
        parser.error('--recording required for source')
    run(args)
