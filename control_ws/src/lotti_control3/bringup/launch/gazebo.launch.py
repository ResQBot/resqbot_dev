# Copyright 2023 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    RegisterEventHandler,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    # --------------------------------------------------------------------------
    # Arguments
    # --------------------------------------------------------------------------
    declared_arguments = [
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Open the Gazebo GUI (gzclient).",
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
                "description/worlds",
                "lotti_world.world",
            ]),
            description="Path to the Gazebo world file.",
        ),
    ]

    gui = LaunchConfiguration("gui")
    use_rviz = LaunchConfiguration("use_rviz")
    world = LaunchConfiguration("world")

    # --------------------------------------------------------------------------
    # URDF — built with use_sim:=true so Gazebo plugins are included
    # --------------------------------------------------------------------------
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("lotti_control3"),
            "description/urdf",
            "Lotti.urdf.xacro",
        ]),
        " use_sim:=true",
    ])
    robot_description = {"robot_description": robot_description_content}

    # --------------------------------------------------------------------------
    # Gazebo server + client
    # --------------------------------------------------------------------------
    gzserver_node = Node(
        package="gazebo_ros",
        executable="gzserver",
        arguments=["--verbose", "-s", "libgazebo_ros_init.so",
                   "-s", "libgazebo_ros_factory.so", world],
        output="screen",
    )

    gzclient_node = Node(
        package="gazebo_ros",
        executable="gzclient",
        condition=IfCondition(gui),
        output="screen",
    )

    # --------------------------------------------------------------------------
    # Robot state publisher
    # --------------------------------------------------------------------------
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # --------------------------------------------------------------------------
    # Spawn robot into Gazebo (delayed so gzserver is ready)
    # --------------------------------------------------------------------------
    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "lotti3",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.15",
        ],
        output="screen",
    )

    delayed_spawn = TimerAction(period=3.0, actions=[spawn_entity_node])

    # --------------------------------------------------------------------------
    # Controller spawners — sequenced after spawn completes
    # --------------------------------------------------------------------------
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

    chain_controller_spawner = Node(
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

    # Spawn arm_controller first (after robot is in Gazebo)
    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[arm_controller_spawner],
        )
    )

    # After arm_controller is up, start joint_state_broadcaster, drive, and flipper
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_chain_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[chain_controller_spawner],
        )
    )

    delay_flipper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[flipper_controller_spawner],
        )
    )

    # --------------------------------------------------------------------------
    # MoveIt Servo (use_gazebo overridden to True)
    # --------------------------------------------------------------------------
    servo_yaml = load_yaml("lotti_control3", "config/lotti_servo_config.yaml")
    servo_params = {"moveit_servo": {**servo_yaml["moveit_servo"], "use_gazebo": True}}

    moveit_config = (
        MoveItConfigsBuilder("lotti3")
        .robot_description(file_path="config/Lotti3.urdf.xacro")
        .to_moveit_configs()
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
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

    # --------------------------------------------------------------------------
    # Teleop
    # --------------------------------------------------------------------------
    teleop_package = get_package_share_directory("lotti_teleop")

    teleop_node = IncludeLaunchDescription(
        os.path.join(teleop_package, "launch", "teleop_launch.py"),
    )

    delay_teleop = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=servo_node,
            on_exit=[teleop_node],
        )
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
    )

    # --------------------------------------------------------------------------
    # RViz
    # --------------------------------------------------------------------------
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("lotti_control3"), "config", "view_lotti.rviz"
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

    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # --------------------------------------------------------------------------
    # Launch description
    # --------------------------------------------------------------------------
    nodes = [
        gzserver_node,
        gzclient_node,
        robot_state_pub_node,
        delayed_spawn,
        delay_arm_controller,
        delay_joint_state_broadcaster,
        delay_chain_controller,
        delay_flipper_controller,
        delay_servo_node,
        joy_node,
        delay_teleop,
        delay_rviz,
    ]

    return LaunchDescription(declared_arguments + nodes)
