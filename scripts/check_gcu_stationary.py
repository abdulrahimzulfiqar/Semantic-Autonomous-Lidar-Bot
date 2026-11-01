#!/usr/bin/env python3
"""Read-only ROS 2 scan/map diagnostic; never publishes robot commands.

Use after simulator + gcu_foxglove_alignment.launch.py in the same ROS domain.
This evaluates consistency with the raster-derived world, not real-building accuracy.
"""
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np


def raycast(grid, resolution, origin, position, angles, maximum):
    """First occupied cell along each ray, sampled at quarter-cell increments.

    Unknown cells are not treated as physical walls. Map origins must be unrotated.
    """
    distances = np.arange(resolution / 4, maximum + resolution / 4, resolution / 4)
    expected = []
    for angle in angles:
        xs = np.floor((position[0] + distances * math.cos(angle) - origin[0]) / resolution).astype(int)
        ys = np.floor((position[1] + distances * math.sin(angle) - origin[1]) / resolution).astype(int)
        inside = (xs >= 0) & (xs < grid.shape[1]) & (ys >= 0) & (ys < grid.shape[0])
        occupied = np.zeros(len(distances), dtype=bool)
        occupied[inside] = grid[ys[inside], xs[inside]] >= 65
        hits = np.flatnonzero(occupied)
        expected.append(float(distances[hits[0]]) if len(hits) else float('inf'))
    return np.asarray(expected)


def main():
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.time import Time
    from rclpy.duration import Duration
    from sensor_msgs.msg import LaserScan, Image
    from nav_msgs.msg import OccupancyGrid
    from rosgraph_msgs.msg import Clock
    from geometry_msgs.msg import Twist
    from tf2_ros import Buffer, TransformListener, TransformException

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=60, help='Wall-clock measurement duration after map/TF/scan readiness')
    parser.add_argument('--startup-timeout', type=float, default=45)
    parser.add_argument('--output', type=Path, default=Path('gcu_stationary_report.json'))
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('gcu_stationary_diagnostic')
    buffer = Buffer(cache_time=Duration(seconds=10))
    listener = TransformListener(buffer, node)
    sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    state = {'scan': None, 'map': None, 'sim': None, 'images': 0, 'nonzero_commands': 0}
    def scan(msg): state['scan'] = msg
    def map_message(msg): state['map'] = msg
    def clock(msg): state['sim'] = msg.clock.sec + msg.clock.nanosec / 1e9
    def image(msg):
        state['images'] += 1
        state['image_size'] = [msg.width, msg.height]
    def command(msg):
        if any(abs(v) > 1e-6 for v in (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z)):
            state['nonzero_commands'] += 1
    subscriptions = [node.create_subscription(LaserScan, '/scan', scan, sensor),
                     node.create_subscription(OccupancyGrid, '/map', map_message, latched),
                     node.create_subscription(Clock, '/clock', clock, sensor),
                     node.create_subscription(Image, '/camera/image_raw', image, sensor)]
    for topic in ['/cmd_vel', '/diff_controller/cmd_vel_unstamped']:
        subscriptions.append(node.create_subscription(Twist, topic, command, sensor))
    start = time.monotonic()
    ready = None
    sim_start = None
    scans = []
    poses = []
    last_stamp = None
    failures = 0
    latest = None
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic()
        msg = state['scan']
        if msg is not None and state['map'] is not None and state['sim'] is not None:
            stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            if stamp != last_stamp:
                try:
                    laser = buffer.lookup_transform('map', msg.header.frame_id, Time.from_msg(msg.header.stamp))
                    base = buffer.lookup_transform('map', 'base_footprint', Time.from_msg(msg.header.stamp))
                    q = laser.transform.rotation
                    yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
                    b = base.transform.rotation
                    base_yaw = math.atan2(2 * (b.w*b.z + b.x*b.y), 1 - 2 * (b.y*b.y + b.z*b.z))
                    if ready is None:
                        ready = now
                        sim_start = state['sim']
                        state['images'] = 0
                    scans.append(np.asarray(msg.ranges, dtype=float))
                    poses.append([base.transform.translation.x, base.transform.translation.y, base_yaw])
                    latest = (msg, laser, yaw)
                    last_stamp = stamp
                except TransformException:
                    failures += 1
        if ready is None and now-start >= args.startup_timeout:
            break
        if ready is not None and now-ready >= args.seconds:
            break
    result = {'purpose': 'stationary consistency against raster-derived simulation',
              'velocity_commands_sent': 0, 'nonzero_commands_observed': state['nonzero_commands'],
              'ready': ready is not None, 'requested_wall_seconds': args.seconds,
              'measurement_wall_seconds': time.monotonic()-ready if ready is not None else 0,
              'measurement_sim_seconds': state['sim']-sim_start if ready is not None else 0,
              'scans_with_timestamped_tf': len(scans), 'tf_lookup_retries': failures,
              'camera_images': state['images'], 'camera_size': state.get('image_size')}
    if latest is not None:
        msg, laser, yaw = latest
        m = state['map']
        q = m.info.origin.orientation
        if abs(q.x)+abs(q.y)+abs(q.z) > 1e-8:
            raise ValueError('Rotated occupancy origins are not supported')
        grid = np.asarray(m.data).reshape(m.info.height, m.info.width)
        angles = yaw + msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        expected = raycast(grid, m.info.resolution, (m.info.origin.position.x, m.info.origin.position.y),
                           (laser.transform.translation.x, laser.transform.translation.y), angles, msg.range_max)
        observed = scans[-1]
        finite = np.isfinite(observed)
        valid = finite & (observed >= msg.range_min) & (observed <= msg.range_max)
        compared = valid & np.isfinite(expected)
        errors = np.abs(observed[compared]-expected[compared])
        unexpected = valid & (~np.isfinite(expected) | (observed < expected - 3*m.info.resolution))
        missing = np.isposinf(observed) & np.isfinite(expected) & (expected < msg.range_max - 3*m.info.resolution)
        drift = np.asarray(poses)-poses[0]
        drift[:, 2] = np.arctan2(np.sin(drift[:, 2]), np.cos(drift[:, 2]))
        outliers = np.flatnonzero(unexpected)
        result.update({'resolution_m': m.info.resolution, 'scan_beams': len(observed),
                       'positive_infinite_beams': int(np.isposinf(observed).sum()),
                       'nan_beams': int(np.isnan(observed).sum()),
                       'negative_infinite_beams': int(np.isneginf(observed).sum()),
                       'finite_out_of_range_beams': int((finite & ~valid).sum()),
                       'compared_beams': int(compared.sum()),
                       'median_absolute_range_error_m': float(np.median(errors)) if len(errors) else None,
                       'p95_absolute_range_error_m': float(np.percentile(errors, 95)) if len(errors) else None,
                       'unexpected_short_return_beams': len(outliers),
                       'missing_expected_return_beams': int(missing.sum()),
                       'unexpected_return_samples': [{'index': int(i), 'angle_deg': math.degrees(msg.angle_min+i*msg.angle_increment),
                           'observed_m': float(observed[i]), 'expected_m': float(expected[i]) if np.isfinite(expected[i]) else None} for i in outliers[:40]],
                       'max_position_drift_m': float(np.max(np.linalg.norm(drift[:, :2], axis=1))),
                       'max_yaw_drift_deg': float(np.max(np.abs(drift[:, 2]))*180/math.pi),
                       'stationary_pose_is_odometry_based': True,
                       'scan_stability_median_std_m': float(np.median([np.std(a[np.isfinite(a)]) for a in np.asarray(scans).T if np.isfinite(a).sum() >= 2]))})
    result['stationary_check_passed'] = bool(ready is not None and len(scans) >= 10 and
        result['measurement_sim_seconds'] > 0 and state['nonzero_commands'] == 0 and
        result.get('max_position_drift_m', 1) <= .02 and result.get('max_yaw_drift_deg', 999) <= 1)
    result['scan_accuracy_gate_passed'] = bool(result.get('compared_beams', 0) >= 100 and
        result.get('p95_absolute_range_error_m', math.inf) <= 3*result.get('resolution_m', 0) and
        result.get('unexpected_short_return_beams', 1) == 0 and result.get('missing_expected_return_beams', 1) == 0)
    result['note'] = 'Pose stability is not ground truth or AMCL accuracy. Range comparison uses the final scan; thresholds are diagnostic gates.'
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(result, indent=2, allow_nan=False), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result['stationary_check_passed'] and result['scan_accuracy_gate_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
