import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('lidarbot_bringup')
    desc_pkg_share = get_package_share_directory('lidarbot_description')
    
    # 1. Robot Model (URDF)
    xacro_file = os.path.join(desc_pkg_share, 'urdf', 'lidarbot.urdf.xacro')
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', xacro_file])}]
    )

    # 2. RPLiDAR
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB_LIDAR',
            'serial_baudrate': 115200,
            'frame_id': 'lidar_link',
            'inverted': False,
            'angle_compensate': True
        }]
    )

    # 3. Arduino Bridge
    bridge_script = os.path.join(pkg_share, 'script', 'arduino_bridge.py')
    arduino_bridge = ExecuteProcess(
        cmd=['python3', bridge_script],
        output='screen'
    )

    # 4. Extended Kalman Filter (EKF) - DISABLED FOR DIRECT TEST
    # ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    # ekf_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[ekf_config]
    #     # We don't remap odom0 here because config uses /odom_raw
    # )

    return LaunchDescription([
        robot_state_publisher,
        rplidar_node,
        arduino_bridge,
        # ekf_node
    ])
