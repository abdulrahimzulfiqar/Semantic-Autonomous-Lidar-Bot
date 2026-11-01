"""Stationary GCU alignment check. Run with simulator localization disabled.

The fixed map->odom transform is a known-spawn diagnostic, NOT localization.
Stop this launch before starting AMCL or any other map->odom publisher.
"""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_map = str(Path(__file__).resolve().parents[1] / 'simulation/gcu_ground_floor/gcu_ground_floor.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        Node(package='nav2_map_server', executable='map_server', name='gcu_map_server',
             parameters=[{'yaml_filename': LaunchConfiguration('map'), 'use_sim_time': True}], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='gcu_map_lifecycle',
             parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['gcu_map_server']}], output='screen'),
        Node(package='robot_localization', executable='ekf_node', name='gcu_alignment_ekf',
             parameters=[{'use_sim_time': True, 'frequency': 30.0, 'sensor_timeout': 0.1,
                          'two_d_mode': True, 'publish_tf': True,
                          'map_frame': 'map', 'odom_frame': 'odom',
                          'base_link_frame': 'base_footprint', 'world_frame': 'odom',
                          'odom0': '/diff_controller/odom',
                          'odom0_config': [False, False, False, False, False, False,
                                           True, True, False, False, False, True,
                                           False, False, False]}], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher', name='gcu_known_spawn',
             arguments=['--x', '-0.05864395131342248', '--y', '-12.960313240266368', '--z', '0',
                        '--yaw', '0', '--pitch', '0', '--roll', '0',
                        '--frame-id', 'map', '--child-frame-id', 'odom'],
             parameters=[{'use_sim_time': True}], output='screen'),
    ])
