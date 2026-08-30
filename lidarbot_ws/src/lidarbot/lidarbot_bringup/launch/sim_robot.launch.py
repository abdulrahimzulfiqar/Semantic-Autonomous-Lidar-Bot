import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_gz = get_package_share_directory('lidarbot_gz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw_pose = LaunchConfiguration('yaw_pose')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    default_world = os.path.join(pkg_gz, 'worlds', 'shanghaitech_merge_v1_world.sdf')
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Full path to Gazebo SDF world file'
    )

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
        default_value='-0.108',
        description='Initial robot spawn yaw angle in radians'
    )

    # Launch Gazebo Simulation with Robot Description, Bridges & EKF
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

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_world_cmd,
        declare_x_pose_cmd,
        declare_y_pose_cmd,
        declare_z_pose_cmd,
        declare_yaw_pose_cmd,
        gz_launch,
    ])
