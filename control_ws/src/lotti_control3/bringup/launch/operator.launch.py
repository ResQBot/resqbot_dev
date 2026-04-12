import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def load_servo_params(package_name, file_path, overrides=None):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    with open(absolute_file_path, "r", encoding="utf-8") as file:
        raw_config = yaml.safe_load(file) or {}

    servo_config = dict(raw_config.get("moveit_servo", raw_config))
    if overrides:
        servo_config.update(overrides)

    return {"moveit_servo": servo_config}


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically on the operator station.",
        ),
    ]

    gui = LaunchConfiguration("gui")
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("lotti_control3"),
        "config",
        "view_lotti.rviz",
    ])

    moveit_config = (
        MoveItConfigsBuilder("lotti3")
        .robot_description(file_path="config/Lotti3.urdf.xacro")
        .to_moveit_configs()
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            load_servo_params("lotti_control3", "config/lotti_servo_config.yaml"),
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        output="screen",
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("lotti_teleop"),
                "launch",
                "teleop_launch.py",
            )
        )
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    nodes = [
        rviz_node,
        servo_node,
        joy_node,
        teleop_launch,
    ]

    return LaunchDescription(declared_arguments + nodes)
