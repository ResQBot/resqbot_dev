import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
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
            description="Start RViz2 automatically in full debug mode.",
        ),
        DeclareLaunchArgument(
            "enable_stub_body_controllers",
            default_value="true",
            description=(
                "Start the currently stubbed drive and flipper controllers. "
                "Full launch is intended for single-machine debug and keeps these enabled by default."
            ),
        ),
    ]

    gui = LaunchConfiguration("gui")
    enable_stub_body_controllers = LaunchConfiguration("enable_stub_body_controllers")

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("lotti_control3"),
            "description",
            "urdf",
            "Lotti.urdf.xacro",
        ]),
    ])
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution([
        FindPackageShare("lotti_control3"),
        "config",
        "lotti_controllers.yaml",
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("lotti_control3"),
        "config",
        "view_lotti.rviz",
    ])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["drive_controller", "-c", "/controller_manager"],
        condition=IfCondition(enable_stub_body_controllers),
    )

    flipper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["flipper3_controller", "-c", "/controller_manager"],
        condition=IfCondition(enable_stub_body_controllers),
    )

    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_drive_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[drive_controller_spawner],
        )
    )

    delay_flipper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[flipper_controller_spawner],
        )
    )

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

    delay_servo_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[servo_node],
        )
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
        control_node,
        robot_state_pub_node,
        arm_controller_spawner,
        delay_joint_state_broadcaster,
        delay_drive_controller,
        delay_flipper_controller,
        delay_servo_node,
        joy_node,
        teleop_launch,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
