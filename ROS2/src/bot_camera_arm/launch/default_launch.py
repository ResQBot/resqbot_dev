from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='bot_camera_arm',
            namespace='',
            executable='arm_interface',
            name='arm_interface',
            output="screen",
            emulate_tty=True,
            parameters=[{
                "update_rate_hz": 10.0,
                "serial_timeout_sec": 0.1,
                "serial_baudrate": 115200,
            }],
            arguments=['--ros-args', '--log-level', 'info']
        ),
    ])