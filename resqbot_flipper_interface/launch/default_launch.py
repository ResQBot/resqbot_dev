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
            package='resqbot_flipper_interface',
            namespace='',
            executable='flipper_interface',
            name='flipper_interface',
            output="screen",
            emulate_tty=True,
            parameters=[{
                "update_rate_hz": 10.0,
                "serial_timeout_sec": 0.1,
                "serial_name": "/dev/ttyACM1",
                "serial_baudrate": 115200,
            }],
            arguments=['--ros-args', '--log-level', 'info']
        ),
    ])