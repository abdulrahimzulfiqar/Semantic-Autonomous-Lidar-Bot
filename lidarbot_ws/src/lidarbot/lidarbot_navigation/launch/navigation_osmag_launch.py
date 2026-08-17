import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_nav = get_package_share_directory('lidarbot_navigation')
    pkg_osmag = get_package_share_directory('lidarbot_osmag')

    # Default file paths
    default_map_path = os.path.join(pkg_nav, 'maps', 'big_map_7.yaml')
    default_params_file = os.path.join(pkg_nav, 'config', 'nav2_osmag_params.yaml')
    default_osm_file = os.path.join(pkg_osmag, 'data', 'big_map_7.osm')

    # Launch Configurations
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    osm_file = LaunchConfiguration('osm_file')

    # Declare Arguments
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
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    declare_osm_file_cmd = DeclareLaunchArgument(
        'osm_file',
        default_value=default_osm_file,
        description='Full path to the osmAG XML topometric map file'
    )

    lifecycle_nodes = ['map_server', 'amcl', 'controller_server']

    # Rewritten yaml for substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # 1. Map Server Node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params, {'yaml_filename': map_yaml_file}]
    )

    # 2. AMCL Localization Node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[configured_params]
    )

    # 3. Nav2 Controller Server (Local Planner for osmAG paths)
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[configured_params]
    )

    # 4. osmAG Semantic Hierarchical Global Planner Node
    osmag_node = Node(
        package='lidarbot_osmag',
        executable='osmag_planner_node',
        name='osmag_planner_node',
        output='screen',
        parameters=[{
            'osm_file_path': osm_file,
            'map_frame': 'map',
            'base_frame': 'base_link',
            'resolution': 0.05
        }]
    )

    # 5. Nav2 Lifecycle Manager
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes
        }]
    )

    return LaunchDescription([
        declare_map_yaml_cmd,
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        declare_params_file_cmd,
        declare_osm_file_cmd,
        map_server_node,
        amcl_node,
        controller_server_node,
        osmag_node,
        lifecycle_manager_node
    ])
