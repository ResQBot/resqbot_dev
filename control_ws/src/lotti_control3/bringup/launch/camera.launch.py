from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# --- to use this launch file, expected command would be:
# ros2 launch lotti_control3 single_camera.launch.py camera_name:=camera_Front port:=/dev/video0
def generate_launch_description():
    
    #Declare arguments to pass variables from the terminal
    camera_name_arg = DeclareLaunchArgument(
        'camera_name', 
        default_value='camera_1',
        description='Namespace for the camera (e.g., camera_front)'
    )
    
    port_arg = DeclareLaunchArgument(
        'port', 
        default_value='/dev/video0',
        description='The hardware port of the camera (e.g., /dev/video0)'
    )

    camera_name = LaunchConfiguration('camera_name')
    port = LaunchConfiguration('port')

    # Define the camera node with optimized settings
    cam_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera_node',
        namespace=camera_name,
        parameters=[
            {'port': port},
            {'width': 640},
            {'height': 480},
            {'format': 'mjpeg'}
        ],
        output='screen'
    )

    return LaunchDescription([
        camera_name_arg,
        port_arg,
        cam_node
    ])