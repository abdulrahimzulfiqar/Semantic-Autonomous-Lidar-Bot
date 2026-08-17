import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('lidarbot_gazebo')
    pkg_nav = get_package_share_directory('lidarbot_navigation')
    pkg_osmag = get_package_share_directory('lidarbot_osmag')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    osm_file = LaunchConfiguration('osm_file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    default_map = os.path.join(pkg_nav, 'maps', 'big_map_7.yaml')
    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Path to 2D grid map YAML'
    )

    default_osm = os.path.join(pkg_osmag, 'data', 'big_map_7.osm')
    declare_osm_cmd = DeclareLaunchArgument(
        'osm_file',
        default_value=default_osm,
        description='Path to osmAG XML map'
    )

    # 1. Launch Gazebo Simulation with big_map_7 world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo_big_map_7_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_ros2_control': 'true',
        }.items()
    )

    # 2. Launch Navigation & osmAG Planner
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'navigation_osmag_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'osm_file': osm_file,
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_map_cmd,
        declare_osm_cmd,
        gazebo_launch,
        navigation_launch,
    ])
