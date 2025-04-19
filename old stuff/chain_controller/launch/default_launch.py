import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='chain_controller',
            namespace='',
            executable='chain_controller',
            name='chain_controller',
            output="screen",
            emulate_tty=True,
            parameters=[{
            }],
            arguments=['--ros-args', '--log-level', 'info']
        ),
    ])