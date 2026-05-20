from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    device_arg = LaunchConfiguration('device')
    camera_name_arg = LaunchConfiguration('camera_name')

    return LaunchDescription([
        # Declare the arguments (with safe defaults if we forget to pass them)
        DeclareLaunchArgument(
            'device',
            default_value='/dev/video0',
            description='Path to the video device port'
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value='camera',
            description='Namespace to prevent topic collisions'
        ),

        Node(
            package='webcam_publisher',
            executable='ffmpeg_camera_node',
            namespace=camera_name_arg,  
            name='ffmpeg_camera',
            output='screen',
            parameters=[{
                'device': device_arg,  
                'width': 640,
                'height': 480,
                'fps': 30
            }]
        )
    ])