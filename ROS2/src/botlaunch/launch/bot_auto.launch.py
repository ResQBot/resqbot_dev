import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python import get_package_share_directory

def generate_launch_description():
    #get directories
    cam_arm_package = get_package_share_directory('bot_camera_arm')
    drive_package = get_package_share_directory('resqbot_drive_interface')
    flipper_package = get_package_share_directory('resqbot_flipper_interface')
    tele_op_package = get_package_share_directory ('tele_op')
    
    joy_node = Node(
        package='joy',
        namespace='Controller',
        executable='joy_node',
        name='Controller'
    )

    tele_op_node = IncludeLaunchDescription(
        os.path.join(tele_op_package, 'launch', 'default_launch.py'),
    )

    cam_arm_node = IncludeLaunchDescription(
        os.path.join(cam_arm_package, 'launch', 'default_launch.py'),
    )

    drive_interface_node = IncludeLaunchDescription(
        os.path.join(drive_package, 'launch', 'default_launch.py'),
    )

    flipper_interface_node = IncludeLaunchDescription(
        os.path.join(flipper_package, 'launch', 'default_launch.py'),
    )
    
    return LaunchDescription([
        joy_node,
        tele_op_node,
        cam_arm_node,
        drive_interface_node,
        flipper_interface_node,
    ])