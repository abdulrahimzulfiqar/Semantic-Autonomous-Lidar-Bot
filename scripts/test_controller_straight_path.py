#!/usr/bin/env python3
"""
Standalone Step 4 Verification Script: Nav2 Controller Server Straight-Line Test.

Purpose:
  Tests the Nav2 Controller Server (RegulatedPurePursuitController) and Local Costmap
  in total isolation from global topological planners (osmAG).

How it Works:
  1. Looks up the current robot pose in the 'map' frame using TF2.
  2. Generates a straight-line nav_msgs/Path extending forward by the requested distance (default: 2.0m).
  3. Sends the path as an Action Goal directly to the Nav2 /follow_path Action Server.
  4. Monitors distance_to_goal and speed feedback while the robot drives forward smoothly.
  5. Confirms arrival at destination when Nav2 completes the action.

Usage:
  python3 scripts/test_controller_straight_path.py --distance 2.0
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from action_msgs.msg import GoalStatus
import tf2_ros


class StraightPathTester(Node):
    """
    ROS 2 Node that commands Nav2 controller_server to follow a short straight path.
    """

    def __init__(self, distance: float = 2.0) -> None:
        super().__init__('controller_straight_path_tester')
        self.target_distance = distance

        # Set parameter use_sim_time to true
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.get_logger().info('Initializing Controller Server Straight Path Tester...')
        self.get_logger().info(f'Target Distance: {self.target_distance:.2f} meters')

        # Action Client for Nav2 FollowPath
        self._action_client = ActionClient(self, FollowPath, 'follow_path')

        # Path Publisher (so the green line appears in Foxglove / RViz)
        self._path_pub = self.create_publisher(Path, '/plan', 10)

        # TF Buffer to get current robot pose
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._goal_done = False
        self._goal_success = False

    def wait_for_services(self) -> bool:
        """Wait for the Nav2 follow_path action server to become available."""
        self.get_logger().info('Waiting for /follow_path action server from controller_server...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('/follow_path action server NOT found! Is controller_server running?')
            return False
        self.get_logger().info('✅ /follow_path action server is ready!')
        return True

    def get_current_pose(self) -> tuple[float, float, float]:
        """
        Query TF to get the current robot (x, y, yaw) in the 'map' frame.
        Falls back to 'odom' frame if 'map' is not yet connected.
        """
        target_frames = ['map', 'odom']
        for frame in target_frames:
            try:
                self.get_logger().info(f'Looking up robot pose: {frame} -> base_footprint...')
                trans = self._tf_buffer.lookup_transform(
                    frame, 'base_footprint', rclpy.time.Time(), timeout=Duration(seconds=3.0)
                )
                x = trans.transform.translation.x
                y = trans.transform.translation.y
                qx = trans.transform.rotation.x
                qy = trans.transform.rotation.y
                qz = trans.transform.rotation.z
                qw = trans.transform.rotation.w

                # Convert quaternion to yaw angle
                siny_cosp = 2.0 * (qw * qz + qx * qy)
                cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                yaw = math.atan2(siny_cosp, cosy_cosp)

                self.get_logger().info(f'Found robot pose in frame "{frame}": x={x:.3f}, y={y:.3f}, yaw={yaw:.3f} rad')
                return x, y, yaw
            except Exception as ex:
                self.get_logger().warn(f'Could not lookup {frame} -> base_footprint: {ex}')

        raise RuntimeError('Failed to get robot pose from TF!')

    def generate_straight_path(self, x0: float, y0: float, yaw: float) -> Path:
        """
        Generate a nav_msgs/Path with waypoints spaced 5 cm apart along the robot's heading.
        """
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        step_size = 0.05  # 5 cm resolution
        num_steps = int(math.ceil(self.target_distance / step_size))

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        # Quaternion for heading
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        for i in range(num_steps + 1):
            s = min(i * step_size, self.target_distance)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = path.header.stamp
            pose.pose.position.x = x0 + s * cos_yaw
            pose.pose.position.y = y0 + s * sin_yaw
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path.poses.append(pose)

        return path

    def feedback_callback(self, feedback_msg) -> None:
        """Process real-time feedback from the controller server."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'🚗 Nav2 Executing: Dist to goal = {feedback.distance_to_goal:.2f} m | Speed = {feedback.speed:.2f} m/s'
        )

    def goal_response_callback(self, future) -> None:
        """Callback when Nav2 accepts or rejects the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Nav2 controller_server REJECTED the path goal!')
            self._goal_done = True
            self._goal_success = False
            return

        self.get_logger().info('✅ Nav2 ACCEPTED the path goal! Robot is now moving...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future) -> None:
        """Callback when the goal completes."""
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('🎉 GOAL SUCCEEDED! Robot reached the 2-meter destination.')
            self._goal_success = True
        else:
            self.get_logger().warn(f'Goal finished with status code: {status}')
            self._goal_success = False
        self._goal_done = True

    def run(self) -> bool:
        """Execute the test sequence."""
        if not self.wait_for_services():
            return False

        # Sleep briefly to ensure TF buffer is filled
        time.sleep(1.0)
        rclpy.spin_once(self, timeout_sec=1.0)

        try:
            x0, y0, yaw = self.get_current_pose()
        except Exception as e:
            self.get_logger().error(f'TF Error: {e}')
            return False

        path = self.generate_straight_path(x0, y0, yaw)
        self.get_logger().info(f'Generated {len(path.poses)} waypoints for straight path.')

        # Publish path for Foxglove visualization
        self._path_pub.publish(path)

        # Build Action Goal
        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = 'FollowPath'

        self.get_logger().info('Sending FollowPath goal to controller_server...')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

        # Spin until the goal completes
        while rclpy.ok() and not self._goal_done:
            rclpy.spin_once(self, timeout_sec=0.2)

        return self._goal_success


def main():
    parser = argparse.ArgumentParser(description='Test Nav2 controller_server on a straight-line path.')
    parser.add_argument('--distance', type=float, default=2.0, help='Straight distance to drive forward in meters')
    args = parser.parse_args()

    rclpy.init()
    tester = StraightPathTester(distance=args.distance)
    success = tester.run()
    tester.destroy_node()
    rclpy.shutdown()

    if success:
        print('\n=== [STAGE 4 VERIFICATION PASSED] Controller Server Successfully Executed Path ===\n')
        sys.exit(0)
    else:
        print('\n=== [STAGE 4 VERIFICATION FAILED] ===\n')
        sys.exit(1)


if __name__ == '__main__':
    main()
