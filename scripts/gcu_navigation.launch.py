"""GCU metric NavigateToPose stack. Requires simulator and active GCU AMCL.

Do not run together with plan-only or the legacy osmAG execution node.
"""
from pathlib import Path
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    assets = Path(__file__).resolve().parents[1] / 'simulation/gcu_ground_floor'
    params = LaunchConfiguration('params')
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=str(assets / 'nav2_autonomous.yaml')),
        DeclareLaunchArgument('tree', default_value=str(assets / 'navigate_strict.xml')),
        DeclareLaunchArgument('through_tree', default_value=str(assets / 'navigate_through_strict.xml')),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=[params], output='screen'),
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             parameters=[params], output='screen'),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             parameters=[params, {'default_nav_to_pose_bt_xml': LaunchConfiguration('tree'),
                                  'default_nav_through_poses_bt_xml': LaunchConfiguration('through_tree')}],
             output='screen'),
        ExecuteProcess(cmd=[sys.executable,str(Path(__file__).resolve().parent/'gcu_navigation_goal.py'),'--execute'],output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='gcu_navigation_lifecycle',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': ['planner_server', 'controller_server', 'bt_navigator']}],
             output='screen'),
    ])
