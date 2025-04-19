import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration 
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from ament_index_python import get_package_share_directory

def generate_launch_description():

    #get directories
    chain_package = get_package_share_directory ('chain_controller')

    joy_node = Node(
        package='joy',
        namespace='Controller',
        executable='joy_node',
        name='Controller'
    )

    chain_controller_node = IncludeLaunchDescription(
        os.path.join(chain_package, 'launch', 'default_launch.py'),
    )
    
    uncompress_cam1 = Node(
        package='image_transport',
        executable='republish',
        arguments= [
            'compressed',
            'raw',
        ],
        remappings=[
            ('in/compressed', 'usb_cam_1/image_raw/compressed'),
            ('out', 'usb_cam_1/image_raw/uncompressed')
        ]
    )

    uncompress_cam2 = Node(
        package='image_transport',
        executable='republish',
        arguments= [
            'compressed',
            'raw',
        ],
        remappings=[
            ('in/compressed', 'usb_cam_2/image_raw/compressed'),
            ('out', 'usb_cam_2/image_raw/uncompressed')
        ]
    )
    
    web_video_server_node= Node(
        package='web_video_server',
        namespace='web_video_server',
        executable='web_video_server',
        name='Video_Server'
    )
 
    return LaunchDescription([
        joy_node,
        chain_controller_node,
        uncompress_cam1,
        uncompress_cam2,
        web_video_server_node
    ])