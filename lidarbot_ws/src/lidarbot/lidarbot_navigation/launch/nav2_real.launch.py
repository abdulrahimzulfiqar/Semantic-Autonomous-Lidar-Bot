import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('lidarbot_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Paths
    params_file = os.path.join(pkg_share, 'config', 'nav2_params_real.yaml')
    map_dir = LaunchConfiguration('map', default=os.path.expanduser('~/my_home_map.yaml'))

    # Launch Arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.expanduser('~/my_home_map.yaml'),
        description='Full path to map file to load'
    )

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Full path to the ROS2 parameters file to use'
    )

    # Include the standard Nav2 bringup launch file
    # This automatically launches map_server, amcl, planner, controller, recoveries, etc.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_dir,
            'use_sim_time': 'False',
            'params_file': params_file,
            'autostart': 'True'
        }.items()
    )

    # Rviz (Optional - usually run on Laptop, but good to have entry)
    # rviz_config_dir = os.path.join(pkg_share, 'rviz', 'nav2_default_view.rviz')
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_dir],
    #     parameters=[{'use_sim_time': False}],
    #     output='screen'
    # )

    return LaunchDescription([
        map_arg,
        params_arg,
        nav2_launch
        # rviz_node
    ])
