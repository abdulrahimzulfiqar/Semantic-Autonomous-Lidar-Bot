"""Stationary AMCL bringup. Stop the fixed-transform alignment launch first.

The simulator must use use_robot_localization:=false; this launch owns its EKF.
No robot commands, navigation servers or fixed map->odom transform are started.
"""
from pathlib import Path
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_map = str(Path(__file__).resolve().parents[1] / 'simulation/gcu_ground_floor/gcu_ground_floor.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('params', default_value=str(Path(__file__).resolve().parents[1] / 'simulation/gcu_ground_floor/amcl.yaml')),
        Node(package='nav2_map_server', executable='map_server', name='gcu_map_server',
             parameters=[{'yaml_filename': LaunchConfiguration('map'), 'use_sim_time': True}], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='gcu_map_lifecycle',
             parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['gcu_map_server', 'amcl']}], output='screen'),
        Node(package='robot_localization', executable='ekf_node', name='gcu_alignment_ekf',
             parameters=[{'use_sim_time': True, 'frequency': 30.0, 'sensor_timeout': 0.1,
                          'two_d_mode': True, 'publish_tf': True,
                          'map_frame': 'map', 'odom_frame': 'odom',
                          'base_link_frame': 'base_footprint', 'world_frame': 'odom',
                          'odom0': '/diff_controller/odom',
                          'odom0_config': [False, False, False, False, False, False,
                                           True, True, False, False, False, True,
                                           False, False, False]}], output='screen'),
        ExecuteProcess(cmd=[sys.executable, str(Path(__file__).resolve().parent / 'gcu_amcl_initial_pose.py'),
                            '--params', LaunchConfiguration('params')], output='screen'),
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             parameters=[LaunchConfiguration('params'), {'use_sim_time': True}], output='screen'),
    ])
