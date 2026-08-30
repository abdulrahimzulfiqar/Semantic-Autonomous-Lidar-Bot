import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_gz = get_package_share_directory('lidarbot_gz')
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

    world_file = LaunchConfiguration('world')

    default_world = os.path.join(pkg_gz, 'worlds', 'big_map_7_world.sdf')
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Full path or name of Gazebo SDF world file'
    )

    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')

    yaw_pose = LaunchConfiguration('yaw_pose')

    declare_x_pose_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='-1.0',
        description='Initial robot spawn x position'
    )
    declare_y_pose_cmd = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Initial robot spawn y position'
    )
    declare_z_pose_cmd = DeclareLaunchArgument(
        'z_pose',
        default_value='0.1',
        description='Initial robot spawn z position'
    )
    declare_yaw_pose_cmd = DeclareLaunchArgument(
        'yaw_pose',
        default_value='0.0',
        description='Initial robot spawn yaw angle in radians'
    )

    # 1. Launch Gazebo Sim (Fortress / Ignition)
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz, 'launch', 'gz_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_ros2_control': 'true',
            'world': world_file,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': z_pose,
            'yaw_pose': yaw_pose,
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
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw_pose': yaw_pose,
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_world_cmd,
        declare_map_cmd,
        declare_osm_cmd,
        declare_x_pose_cmd,
        declare_y_pose_cmd,
        declare_z_pose_cmd,
        declare_yaw_pose_cmd,
        gz_launch,
        navigation_launch,
    ])
