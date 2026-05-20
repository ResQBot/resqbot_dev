import os
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
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
            description="Open the Gazebo GUI if the selected Gazebo launch supports it.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2 automatically.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([
                FindPackageShare("lotti_control3"),
                "description",
                "worlds",
                "lotti_world.world",
            ]),
            description="Path to the Gazebo world file.",
        ),
    ]

    use_rviz = LaunchConfiguration("use_rviz")
    world = LaunchConfiguration("world")

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("lotti_control3"),
            "description",
            "urdf",
            "Lotti.urdf.xacro",
        ]),
        " use_sim:=true",
    ])
    robot_description = {"robot_description": robot_description_content}

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            ])
        ]),
        launch_arguments={
            "gz_args": [world, " -r"],
            "on_exit_shutdown": "true",
        }.items(),
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "lotti3",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.15",
        ],
        output="screen",
    )

    delay_spawn = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_pub_node,
            on_start=[spawn_entity_node],
        )
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )

    drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["drive_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )

    flipper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["flipper3_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )

    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[arm_controller_spawner],
        )
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
            load_servo_params(
                "lotti_control3",
                "config/lotti_servo_config.yaml",
                overrides={"use_gazebo": True},
            ),
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
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

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("lotti_control3"),
        "config",
        "view_lotti.rviz",
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_rviz),
        parameters=[{"use_sim_time": True}],
    )

    nodes = [
        gz_sim,
        robot_state_pub_node,
        clock_bridge,
        delay_spawn,
        delay_arm_controller,
        delay_joint_state_broadcaster,
        delay_drive_controller,
        delay_flipper_controller,
        delay_servo_node,
        joy_node,
        teleop_launch,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
