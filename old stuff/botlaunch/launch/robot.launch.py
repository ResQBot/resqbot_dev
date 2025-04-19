import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration 
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from ament_index_python import get_package_share_directory

def generate_launch_description():

    #get directories
    drive_package = get_package_share_directory('resqbot_drive_interface')
    flipper_package = get_package_share_directory('resqbot_flipper_interface')

#    usb_cam_1_config_file_path = LaunchConfiguration('usb_cam_1_config_file_path')
#    declare_usb_cam_1_config_file_path = DeclareLaunchArgument(
#        'usb_cam_1_config_file_path',
#        default_value='~/dev_ws/src/web_video_server/cam1.yaml',
#    )

    drive_interface_node = IncludeLaunchDescription(
        os.path.join(drive_package, 'launch', 'default_launch.py'),
    )

    flipper_interface_node = IncludeLaunchDescription(
        os.path.join(flipper_package, 'launch', 'default_launch.py'),
    )
    
    usb_cam_1_node = Node(
        package='usb_cam',
        namespace='usb_cam_1',
        executable='usb_cam_node_exe',
        name='usb_cam_1',
        output= 'screen',
        parameters=[{
                'video_device': "/dev/video0",
                'framerate': 10.0,
                'io_method': "mmap",
                'frame_id': "cam1",
                'pixel_format': "yuyv2rgb",
                'color_format': "yuyv2rgb",
                'image_width': 320,
                'image_height': 240,
                'camera_name': "cam1"
        }]
    )
    
    usb_cam_2_node = Node(
        package='usb_cam',
        namespace='usb_cam_2',
        executable='usb_cam_node_exe',
        name='usb_cam_2',
        output= 'screen',
        parameters=[{
                'video_device': "/dev/video2",
                'framerate': 10.0,
                'io_method': "mmap",
                'frame_id': "cam2",
                'pixel_format': "yuyv2rgb",
                'color_format': "yuyv2rgb",
                'image_width': 320,
                'image_height': 240,
                'camera_name': "cam2"
        }]
    )
    
    return LaunchDescription([
        drive_interface_node,
        flipper_interface_node,
        usb_cam_1_node,
        usb_cam_2_node
    ])