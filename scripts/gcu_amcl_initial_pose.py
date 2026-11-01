#!/usr/bin/env python3
"""Publish one known-spawn AMCL prior with explicit nonzero uncertainty.

Waits for AMCL active and odometry TF. Publishes no robot movement commands.
"""
import argparse
import math
from pathlib import Path
import time
import yaml
import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--params', type=Path, required=True)
    args = parser.parse_args()
    prior = yaml.safe_load(args.params.read_text())['amcl']['ros__parameters']['initial_pose']
    rclpy.init(args=[])
    node = rclpy.create_node('gcu_amcl_initial_pose', parameter_overrides=[Parameter('use_sim_time', value=True)])
    publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    state = node.create_client(GetState, '/amcl/get_state')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    start = time.monotonic()
    success = False
    while time.monotonic()-start < 60:
        rclpy.spin_once(node, timeout_sec=.1)
        if not state.service_is_ready() or not publisher.get_subscription_count():
            continue
        future = state.call_async(GetState.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=.5)
        if not future.done() or future.result() is None or future.result().current_state.label != 'active':
            continue
        if not buffer.can_transform('odom', 'base_footprint', Time()) or node.get_clock().now().nanoseconds == 0:
            continue
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.pose.position.x = prior['x']
        msg.pose.pose.position.y = prior['y']
        msg.pose.pose.orientation.z = math.sin(prior['yaw']/2)
        msg.pose.pose.orientation.w = math.cos(prior['yaw']/2)
        msg.pose.covariance[0] = .25**2
        msg.pose.covariance[7] = .25**2
        msg.pose.covariance[35] = math.radians(10)**2
        publisher.publish(msg)
        node.get_logger().info('Published one prior: position stddev 0.25 m, heading stddev 10 deg')
        until = time.monotonic()+1
        while time.monotonic()<until:
            rclpy.spin_once(node, timeout_sec=.1)
        success = True
        break
    if not success:
        node.get_logger().error('Initial pose not sent: AMCL/odometry/clock not ready within 60 seconds')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
