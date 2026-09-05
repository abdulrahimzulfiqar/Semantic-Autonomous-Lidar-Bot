#!/usr/bin/env python3
"""
Interactive Step 4 Verification Node: Nav2 Controller Server Goal Pose Navigator.

Purpose:
  Tests the Nav2 Controller Server (RegulatedPurePursuitController) and Local Costmap
  in isolation from osmAG by allowing you to click "2D Goal Pose" directly in Foxglove / RViz.

How it Works:
  1. Subscribes to '/goal_pose' (published whenever you click "Goal Pose" in Foxglove/RViz).
  2. Looks up the current robot pose in 'map' frame via TF2.
  3. Interpolates a smooth direct path of waypoints (spaced 5 cm apart) from the robot to the clicked goal.
  4. Publishes the path to '/plan' so Foxglove draws the green navigation line.
  5. Sends the path directly to the Nav2 '/follow_path' Action Server (controller_server).
  6. Nav2 drives the robot directly to the goal with active local obstacle avoidance!

Usage:
  # Interactive mode: click anywhere in Foxglove to drive there!
  python3 scripts/test_controller_straight_path.py

  # Or auto-drive forward 2 meters immediately:
  python3 scripts/test_controller_straight_path.py --auto 2.0
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


class ControllerGoalNavigator(Node):
    """
    ROS 2 Node that receives /goal_pose from Foxglove/RViz and commands
    Nav2 controller_server to follow the path via Regulated Pure Pursuit.
    """

    def __init__(self, auto_distance: float = 0.0) -> None:
        super().__init__('controller_goal_navigator')
        self.auto_distance = auto_distance

        # Use simulation clock (Gazebo)
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.get_logger().info('==================================================')
        self.get_logger().info('  Nav2 Controller Server Direct Goal Navigator   ')
        self.get_logger().info('==================================================')

        # Action Client for Nav2 FollowPath
        self._action_client = ActionClient(self, FollowPath, 'follow_path')

        # Path Publisher for Foxglove/RViz visualization
        self._path_pub = self.create_publisher(Path, '/plan', 10)

        # TF Buffer
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._active_goal_handle = None
        self._goal_in_progress = False

        # Subscribe to /goal_pose from Foxglove / RViz
        self._goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10
        )

        self.get_logger().info('Subscribed to /goal_pose. Ready for clicks from Foxglove!')

    def wait_for_controller_server(self) -> bool:
        """Wait for the Nav2 follow_path action server to become available."""
        self.get_logger().info('Connecting to /follow_path action server from controller_server...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('❌ /follow_path action server NOT found! Is controller_server running?')
            return False
        self.get_logger().info('✅ /follow_path action server is ready!')
        return True

    def get_current_pose(self) -> tuple[float, float, float]:
        """Query TF to get current robot (x, y, yaw) in the 'map' frame."""
        for frame in ['map', 'odom']:
            try:
                trans = self._tf_buffer.lookup_transform(
                    frame, 'base_footprint', rclpy.time.Time(), timeout=Duration(seconds=2.0)
                )
                x = trans.transform.translation.x
                y = trans.transform.translation.y
                qx = trans.transform.rotation.x
                qy = trans.transform.rotation.y
                qz = trans.transform.rotation.z
                qw = trans.transform.rotation.w

                siny_cosp = 2.0 * (qw * qz + qx * qy)
                cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                yaw = math.atan2(siny_cosp, cosy_cosp)
                return x, y, yaw
            except Exception:
                continue

        raise RuntimeError('Could not resolve robot transform from map/odom to base_footprint!')

    def interpolate_path(self, x0: float, y0: float, x1: float, y1: float, goal_qz: float, goal_qw: float) -> Path:
        """Create a smooth path with waypoints spaced 5 cm apart from (x0, y0) to (x1, y1)."""
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        dx = x1 - x0
        dy = y1 - y0
        total_dist = math.hypot(dx, dy)
        travel_yaw = math.atan2(dy, dx)

        travel_qz = math.sin(travel_yaw / 2.0)
        travel_qw = math.cos(travel_yaw / 2.0)

        step_size = 0.05  # 5 cm waypoint spacing
        num_steps = max(1, int(math.ceil(total_dist / step_size)))

        for i in range(num_steps + 1):
            ratio = min(1.0, (i * step_size) / total_dist) if total_dist > 0 else 1.0
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = path.header.stamp
            pose.pose.position.x = x0 + ratio * dx
            pose.pose.position.y = y0 + ratio * dy
            pose.pose.position.z = 0.0

            # Last waypoint gets final target orientation, intermediate points orient forward
            if i == num_steps:
                pose.pose.orientation.z = goal_qz
                pose.pose.orientation.w = goal_qw
            else:
                pose.pose.orientation.z = travel_qz
                pose.pose.orientation.w = travel_qw

            path.poses.append(pose)

        return path

    def goal_pose_callback(self, msg: PoseStamped) -> None:
        """Triggered whenever a 2D Goal Pose is clicked in Foxglove or RViz."""
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        gqz = msg.pose.orientation.z
        gqw = msg.pose.orientation.w

        self.get_logger().info('--------------------------------------------------')
        self.get_logger().info(f'📍 RECEIVED GOAL POSE FROM FOXGLOVE: ({gx:.2f}, {gy:.2f})')
        self.get_logger().info('--------------------------------------------------')

        # Cancel previous goal if still running
        if self._active_goal_handle is not None and self._goal_in_progress:
            self.get_logger().info('Cancelling previous navigation goal...')
            self._active_goal_handle.cancel_goal_async()

        try:
            x0, y0, yaw0 = self.get_current_pose()
        except Exception as ex:
            self.get_logger().error(f'Cannot get robot pose: {ex}')
            return

        dist = math.hypot(gx - x0, gy - y0)
        self.get_logger().info(f'  Robot at: ({x0:.2f}, {y0:.2f}) | Distance to goal: {dist:.2f} meters')

        path = self.interpolate_path(x0, y0, gx, gy, gqz, gqw)
        self._path_pub.publish(path)

        self.send_path_to_controller(path)

    def send_path_to_controller(self, path: Path) -> None:
        """Send path to Nav2 FollowPath action server."""
        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = 'FollowPath'

        self.get_logger().info(f'Sending {len(path.poses)}-waypoint path to Nav2 controller_server...')
        future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg) -> None:
        """Log real-time controller progress."""
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'🚗 Nav2 Driving: Dist = {fb.distance_to_goal:.2f} m | Speed = {fb.speed:.2f} m/s'
        )

    def goal_response_callback(self, future) -> None:
        """Handle goal acceptance/rejection by controller_server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Nav2 controller_server REJECTED the path goal!')
            self._goal_in_progress = False
            return

        self._active_goal_handle = goal_handle
        self._goal_in_progress = True
        self.get_logger().info('✅ Nav2 ACCEPTED the goal! Robot is now moving...')

        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future) -> None:
        """Handle goal arrival."""
        status = future.result().status
        self._goal_in_progress = False
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('🎉 GOAL SUCCEEDED! Robot reached the destination.')
        else:
            self.get_logger().warn(f'Goal finished with status: {status}')

    def trigger_auto_distance(self) -> None:
        """Trigger an automatic straight line forward."""
        try:
            x0, y0, yaw0 = self.get_current_pose()
        except Exception as ex:
            self.get_logger().error(f'Cannot get robot pose: {ex}')
            return

        gx = x0 + self.auto_distance * math.cos(yaw0)
        gy = y0 + self.auto_distance * math.sin(yaw0)
        gqz = math.sin(yaw0 / 2.0)
        gqw = math.cos(yaw0 / 2.0)

        self.get_logger().info(f'Auto-driving forward {self.auto_distance:.2f}m to ({gx:.2f}, {gy:.2f})...')
        path = self.interpolate_path(x0, y0, gx, gy, gqz, gqw)
        self._path_pub.publish(path)
        self.send_path_to_controller(path)


def main():
    parser = argparse.ArgumentParser(description='Interactive /goal_pose Navigator for Nav2 controller_server.')
    parser.add_argument('--auto', type=float, default=0.0, help='Auto-drive straight forward by N meters on startup')
    args = parser.parse_args()

    rclpy.init()
    node = ControllerGoalNavigator(auto_distance=args.auto)

    if not node.wait_for_controller_server():
        rclpy.shutdown()
        sys.exit(1)

    if args.auto > 0.0:
        time.sleep(1.0)
        rclpy.spin_once(node, timeout_sec=0.5)
        node.trigger_auto_distance()

    print('\n>>> READY: In Foxglove Studio, click the "2D Goal Pose" tool anywhere on the map! <<<\n')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
