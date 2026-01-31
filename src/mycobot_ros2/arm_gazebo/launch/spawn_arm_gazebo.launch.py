#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command


def generate_launch_description():

    # -------------------------------
    # Paths
    # -------------------------------
    arm_gazebo_pkg = FindPackageShare("arm_gazebo").find("arm_gazebo")
    urdf_path = os.path.join(
        arm_gazebo_pkg,
        "urdf",
        "my_arm_gazebo.urdf.xacro"
    )

    ros_gz_sim_pkg = FindPackageShare("ros_gz_sim").find("ros_gz_sim")
    gz_launch_path = os.path.join(
        ros_gz_sim_pkg,
        "launch",
        "gz_sim.launch.py"
    )

    # -------------------------------
    # Gazebo (Fortress)
    # -------------------------------
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            "gz_args": "-r"
        }.items()
    )

    # -------------------------------
    # Robot State Publisher
    # -------------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": Command(["xacro ", urdf_path]),
            "use_sim_time": True
        }]
    )

    # -------------------------------
    # Spawn robot into Gazebo
    # -------------------------------
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name", "my_arm"
        ]
    )

    # -------------------------------
    # Controllers (DO NOT start ros2_control_node)
    # -------------------------------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager"
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager"
        ],
    )

    # -------------------------------
    # Event sequencing
    # -------------------------------
    delay_jsb = RegisterEventHandler(
        OnProcessStart(
            target_action=spawn_robot,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    delay_arm_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[arm_controller_spawner],
        )
    )

    # -------------------------------
    # LaunchDescription
    # -------------------------------
    return LaunchDescription([
        start_gazebo,
        robot_state_publisher,
        spawn_robot,
        delay_jsb,
        delay_arm_controller,
    ])
