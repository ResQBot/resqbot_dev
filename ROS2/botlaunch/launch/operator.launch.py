import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration 
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from ament_index_python import get_package_share_directory

def generate_launch_description():

    #get directories
    tele_op_package = get_package_share_directory ('tele_op')

    joy_node = Node(
        package='joy',
        namespace='Controller',
        executable='joy_node',
        name='Controller'
    )

    chain_controller_node = IncludeLaunchDescription(
        os.path.join(tele_op_package, 'launch', 'default_launch.py'),
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
    
    uncompress_cam3 = Node(
        package='image_transport',
        executable='republish',
        arguments= [
            'compressed',
            'raw',
        ],
        remappings=[
            ('in/compressed', 'usb_cam_3/image_raw/compressed'),
            ('out', 'usb_cam_3/image_raw/uncompressed')
        ]
    )

 
    return LaunchDescription([
        joy_node,
        chain_controller_node,
        uncompress_cam1,
        uncompress_cam2,
        uncompress_cam3
    ])