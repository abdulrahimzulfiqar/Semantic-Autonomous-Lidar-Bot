"""Add only metric planner, global/local costmaps and click-to-preview display.

Requires simulator and gcu_amcl.launch.py. Starts no navigation controller, BT
navigator, osmAG execution node or velocity publisher.
"""
from pathlib import Path
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params',default_value=str(Path(__file__).resolve().parents[1]/'simulation/gcu_ground_floor/nav2_plan_only.yaml')),
        Node(package='nav2_planner',executable='planner_server',name='planner_server',
             parameters=[LaunchConfiguration('params'),{'use_sim_time':True}],output='screen'),
        Node(package='nav2_costmap_2d',executable='nav2_costmap_2d',
             parameters=[LaunchConfiguration('params'),{'use_sim_time':True}],output='screen'),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='gcu_planning_lifecycle',
             parameters=[{'use_sim_time':True,'autostart':True,'node_names':['planner_server','costmap/costmap']}],output='screen'),
        ExecuteProcess(cmd=[sys.executable,str(Path(__file__).resolve().parent/'gcu_goal_preview.py')],output='screen'),
    ])
