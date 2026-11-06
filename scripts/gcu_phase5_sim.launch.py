"""Phase 05 headless simulator with controlled evaluation-bridge teardown.

Prepare robot.sdf/robot.urdf/contact_bridge.yaml with prepare_gcu_contact_assets.py.
The ROS description retains fixed joints; only the spawned SDF is instrumented.
"""
from pathlib import Path
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, AppendEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def launch_nodes(context):
    assets = Path(LaunchConfiguration('assets').perform(context))
    world = LaunchConfiguration('world').perform(context)
    spawn = Node(package='ros_gz_sim', executable='create', output='screen', arguments=[
        '-file', str(assets/'robot.sdf'), '-name', 'lidarbot',
        '-x', '-0.05864395131342248', '-y', '-12.960313240266368', '-z', '0.15', '-Y', '0'])
    bridge = Path(get_package_share_directory('lidarbot_gz'))/'config/lidarbot_bridge.yaml'
    return [
        AppendEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
                                  str(Path(get_package_prefix('ign_ros2_control'))/'lib')),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'use_sim_time': True, 'robot_description': (assets/'robot.urdf').read_text()}]),
        ExecuteProcess(cmd=['ign', 'gazebo', '-r', '-s', '-v', '3', world], output='screen'),
        ExecuteProcess(cmd=[str(assets.parent.parent/'support_install/lib/gcu_truth_bridge/gcu_ordered_bridge'),
                            '--ros-args','-p','config_file:='+str(bridge)],output='screen'),
        ExecuteProcess(cmd=[str(assets.parent.parent/'support_install/lib/gcu_truth_bridge/gcu_ordered_bridge'),
                            '--ros-args','-p','config_file:='+str(assets/'contact_bridge.yaml')],output='screen'),
        ExecuteProcess(cmd=[str(assets.parent.parent/'support_install/lib/gcu_truth_bridge/gcu_truth_bridge'),
                            '--ros-args','-p','source_topic:=/model/lidarbot/pose'],output='screen'),
        Node(package='ros_gz_image', executable='image_bridge', arguments=['/camera/image_raw']),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[
            Node(package='controller_manager', executable='spawner', arguments=[
                'joint_broadcaster', '--controller-manager-timeout', '40']),
            Node(package='controller_manager', executable='spawner', arguments=[
                'diff_controller', '--controller-manager-timeout', '40'])])),
        # Direct sole controller command path, no joystick or competing mux input.
        # Nav2 publishes /cmd_vel; this relay is the only receiver forwarding it.
        Node(package='twist_mux', executable='twist_mux', parameters=[{
            'use_sim_time':True, 'topics.navigation.topic':'cmd_vel',
            'topics.navigation.timeout':0.5, 'topics.navigation.priority':10}],
             remappings=[('/cmd_vel_out','/diff_controller/cmd_vel_unstamped')]),
        spawn,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('assets'), DeclareLaunchArgument('world'),
        OpaqueFunction(function=launch_nodes),
    ])
