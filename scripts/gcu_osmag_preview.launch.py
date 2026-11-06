"""Launch only the motion-free paper-based route preview; simulation/AMCL are separate."""
from pathlib import Path
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('assets'),
        ExecuteProcess(cmd=[sys.executable,str(Path(__file__).parent/'gcu_osmag_route_preview.py'),
                            '--assets',LaunchConfiguration('assets')],output='screen'),
        ExecuteProcess(cmd=[str(Path(__file__).parent/'probe_install/lib/gcu_tf_probe/gcu_tf_probe')],output='screen'),
    ])
