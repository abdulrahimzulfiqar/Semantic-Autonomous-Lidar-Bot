"""Feed recorded stationary simulator sensors to the Pi stack at native rates."""
import argparse
import base64
import gzip
import json
import os
from pathlib import Path
import statistics
import time

if __package__:
    from .sensor_recording import message_types
else:
    from sensor_recording import message_types


def load_clip(recording, start_ns, end_ns):
    with gzip.open(recording, 'rt') as stream:
        capture = json.load(stream)
    if capture['format'] != 1 or end_ns <= start_ns:
        raise ValueError('invalid_recording_or_interval')
    static = [item[2] for item in capture['rows'] if item[0] == '/tf_static']
    rows = sorted({(key, stamp): payload for key, stamp, payload in capture['rows']
                   if key != '/tf_static' and start_ns <= stamp < end_ns}.items(),
                  key=lambda item: item[0][1])
    expected = {'/scan': .1, '/imu_broadcaster/imu': .01, '/diff_controller/odom': .01}
    counts = {}
    for topic, period in expected.items():
        stamps = [stamp for (key, stamp), _ in rows if key == topic]
        gaps = [b - a for a, b in zip(stamps, stamps[1:])]
        if not gaps or max(gaps+[stamps[0]-start_ns,end_ns-stamps[-1]]) > period * 2.1e9:
            raise ValueError(f'recording_gap:{topic}')
        counts[topic] = {'samples': len(stamps), 'median_period_s': statistics.median(gaps) / 1e9}
    if not static:
        raise ValueError('missing_recorded_robot_extrinsics')
    return rows, static, counts


def run(recording, output, start_ns, end_ns):
    if os.environ.get('ROS_DOMAIN_ID') != '218' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('isolated_phase9_replay_required')
    import signal
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from rclpy.serialization import deserialize_message
    from rosgraph_msgs.msg import Clock
    from geometry_msgs.msg import Twist

    rows, static, counts = load_clip(recording, start_ns, end_ns)
    rclpy.init(args=[])
    node = rclpy.create_node('phase9_replay_inputs')
    types = message_types()
    publishers = {key: node.create_publisher(kind, key, QoSProfile(
        depth=64, durability=DurabilityPolicy.TRANSIENT_LOCAL) if key == '/tf_static'
        else QoSProfile(depth=10) if key == '/diff_controller/odom'
        else qos_profile_sensor_data) for key, (_, kind) in types.items()}
    clock = node.create_publisher(Clock, '/clock', qos_profile_sensor_data)
    commands = []
    node.create_subscription(Twist, '/phase9/replay_commands', lambda message:
        commands.append((time.monotonic(), message.linear.x, message.angular.z)), 10)

    def decode(payload, topic):
        return deserialize_message(base64.b64decode(payload), types[topic][1])

    for payload in static:
        publishers['/tf_static'].publish(decode(payload, '/tf_static'))
    messages = [(key, stamp-start_ns, decode(payload, key)) for (key, stamp), payload in rows]
    duration = end_ns-start_ns
    clock_message = Clock()
    epoch = 3_000_000_000
    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    begin = time.monotonic_ns()
    next_clock = 0
    loop, index = 0, 0
    lateness = {key: 0 for key in counts}
    published = dict.fromkeys(counts, 0)
    try:
        while running and rclpy.ok():
            elapsed = time.monotonic_ns()-begin
            if elapsed > 285_000_000_000:
                raise RuntimeError('stack_replay_timeout')
            while index < len(messages):
                key, relative, message = messages[index]
                due = loop * duration + relative
                if due > elapsed:
                    break
                stamp = epoch + due
                message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
                publishers[key].publish(message)
                lateness[key] = max(lateness[key], time.monotonic_ns()-begin-due)
                published[key] += 1
                index += 1
            if index == len(messages):
                loop, index = loop+1, 0
            if elapsed >= next_clock:
                clock_message.clock.sec, clock_message.clock.nanosec = divmod(epoch+elapsed, 10**9)
                clock.publish(clock_message)
                next_clock = (elapsed // 10_000_000 + 1)*10_000_000
            rclpy.spin_once(node, timeout_sec=0)
            time.sleep(.001)
    finally:
        output.write_text(json.dumps(dict(
            source_clip_s=[start_ns/1e9,end_ns/1e9], clip_inputs=counts,
            published=published, maximum_publication_lateness_wall_s={k:v/1e9 for k,v in lateness.items()},
            commands=commands, scope='Repeated recorded stationary clip at 1x with fixed per-repeat timestamp offset; recorded command sink has no feedback'),indent=2)+'\n')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--recording', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--start-ns', type=int, required=True)
    p.add_argument('--end-ns', type=int, required=True)
    a = p.parse_args()
    run(a.recording, a.output, a.start_ns, a.end_ns)
