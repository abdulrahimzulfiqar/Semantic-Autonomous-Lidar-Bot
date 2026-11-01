#!/usr/bin/env python3
"""Read-only stationary AMCL check, except explicit no-motion filter update requests.

Does not send initial poses, ground-truth data, goals or velocity commands.
Reference is known spawn; this is not a moving localization accuracy evaluation.
"""
import argparse
import json
import math
from pathlib import Path
import time


def main():
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.time import Time
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
    from sensor_msgs.msg import LaserScan, Image
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
    from std_srvs.srv import Empty
    from lifecycle_msgs.srv import GetState
    from tf2_ros import Buffer, TransformListener, TransformException
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=45)
    parser.add_argument('--output', type=Path, default=Path('gcu_amcl_report.json'))
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('gcu_amcl_stationary_check')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    latched = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    poses = []
    state = {'scans': 0, 'images': 0, 'nonzero_commands': 0, 'fixed_map_odom_messages': 0, 'dynamic_map_odom_messages': 0}
    def pose(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        poses.append({'x': p.x, 'y': p.y, 'yaw': math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
                      'variance_x': msg.pose.covariance[0], 'variance_y': msg.pose.covariance[7], 'variance_yaw': msg.pose.covariance[35]})
    def scan(msg): state['scans'] += 1
    def image(msg): state['images'] += 1
    def clock(msg):
        t = msg.clock.sec+msg.clock.nanosec/1e9
        state.setdefault('first_sim_time', t)
        state['last_sim_time'] = t
    def command(msg):
        if any(abs(v)>1e-6 for v in [msg.linear.x, msg.linear.y, msg.angular.z]): state['nonzero_commands'] += 1
    def tf_static(msg):
        state['fixed_map_odom_messages'] += sum(t.header.frame_id=='map' and t.child_frame_id=='odom' for t in msg.transforms)
    def tf_dynamic(msg):
        state['dynamic_map_odom_messages'] += sum(t.header.frame_id=='map' and t.child_frame_id=='odom' for t in msg.transforms)
    subscriptions = [node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', pose, 10),
        node.create_subscription(LaserScan, '/scan', scan, sensor), node.create_subscription(Image, '/camera/image_raw', image, sensor),
        node.create_subscription(Clock, '/clock', clock, sensor), node.create_subscription(TFMessage, '/tf_static', tf_static, latched),
        node.create_subscription(TFMessage, '/tf', tf_dynamic, 100)]
    for topic in ['/cmd_vel', '/diff_controller/cmd_vel_unstamped']:
        subscriptions.append(node.create_subscription(Twist, topic, command, sensor))
    updates = node.create_client(Empty, '/request_nomotion_update')
    lifecycle = node.create_client(GetState, '/amcl/get_state')
    future = None
    start = time.monotonic()
    requests = 0
    last_request = 0
    while time.monotonic()-start < args.seconds:
        rclpy.spin_once(node, timeout_sec=.05)
        elapsed = time.monotonic()-start
        if elapsed > 5 and requests < 3 and elapsed-last_request >= 8 and updates.service_is_ready():
            updates.call_async(Empty.Request())
            requests += 1
            last_request = elapsed
        if future is None and lifecycle.service_is_ready(): future = lifecycle.call_async(GetState.Request())
    state['lifecycle_state'] = future.result().current_state.label if future is not None and future.done() and future.result() is not None else None
    state['amcl_pose_messages'] = len(poses)
    state['nomotion_update_requests'] = requests
    state['wall_seconds'] = time.monotonic()-start
    state['sim_seconds'] = state.get('last_sim_time', 0)-state.get('first_sim_time', 0)
    state['velocity_commands_sent'] = 0
    state['reference'] = 'Known initial spawn; no moving ground-truth evaluation'
    if poses:
        state['last_pose'] = poses[-1]
        state['nondegenerate_covariance'] = all(math.isfinite(poses[-1][k]) and poses[-1][k] > 1e-10 for k in ['variance_x', 'variance_y', 'variance_yaw'])
        state['position_error_from_spawn_m'] = math.hypot(poses[-1]['x']+.05864395131342248, poses[-1]['y']+12.960313240266368)
        state['yaw_error_from_spawn_deg'] = abs(math.degrees(poses[-1]['yaw']))
        state['pose_history'] = poses
    try:
        t = buffer.lookup_transform('map', 'lidar_link', Time())
        state['map_to_lidar_available'] = True
    except TransformException:
        state['map_to_lidar_available'] = False
    state['passed'] = bool(state['lifecycle_state']=='active' and len(poses)>0 and state['map_to_lidar_available'] and
        state['fixed_map_odom_messages']==0 and state['dynamic_map_odom_messages']>0 and state['nonzero_commands']==0 and
        state.get('nondegenerate_covariance', False) and state['scans']>=10 and state['images']>0 and state['sim_seconds']>0 and
        state.get('position_error_from_spawn_m', 999)<=.25 and state.get('yaw_error_from_spawn_deg', 999)<=5)
    state['limitations'] = 'Single stationary known-initial-pose test with three explicit no-motion update requests; not recovery or navigation.'
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(state, indent=2)+'\n')
    print(json.dumps(state, indent=2), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if state['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
