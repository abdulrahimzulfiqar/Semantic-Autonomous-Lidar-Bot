import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('lidarbot_osmag')
    config_file = os.path.join(pkg_share, 'config', 'osmag_params.yaml')

    osmag_node = Node(
        package='lidarbot_osmag',
        executable='osmag_planner_node',
        name='osmag_planner_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        osmag_node
    ])
