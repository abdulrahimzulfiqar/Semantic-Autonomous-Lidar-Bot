import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_nav = get_package_share_directory('lidarbot_navigation')

    default_map_path = os.path.join(pkg_nav, 'maps', 'shanghaitech_merge_v1.yaml')
    default_params_file = os.path.join(pkg_nav, 'config', 'nav2_osmag_params.yaml')

    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw_pose = LaunchConfiguration('yaw_pose')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to 2D occupancy grid map yaml file'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file'
    )
    declare_x_pose_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='-1.0',
        description='Initial robot spawn x position for AMCL'
    )
    declare_y_pose_cmd = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Initial robot spawn y position for AMCL'
    )
    declare_yaw_pose_cmd = DeclareLaunchArgument(
        'yaw_pose',
        default_value='-0.108',
        description='Initial robot spawn yaw angle for AMCL'
    )

    lifecycle_nodes = ['map_server', 'amcl']

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file,
        'set_initial_pose': 'true',
        'initial_pose.x': x_pose,
        'initial_pose.y': y_pose,
        'initial_pose.z': '0.0',
        'initial_pose.yaw': yaw_pose
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    load_nodes = GroupAction(
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[configured_params, {'yaml_filename': map_yaml_file}],
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[
                    configured_params,
                    {
                        'initial_pose.x': x_pose,
                        'initial_pose.y': y_pose,
                        'initial_pose.yaw': yaw_pose,
                    }
                ],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'node_names': lifecycle_nodes
                }]
            )
        ]
    )

    return LaunchDescription([
        declare_map_yaml_cmd,
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        declare_params_file_cmd,
        declare_x_pose_cmd,
        declare_y_pose_cmd,
        declare_yaw_pose_cmd,
        load_nodes
    ])