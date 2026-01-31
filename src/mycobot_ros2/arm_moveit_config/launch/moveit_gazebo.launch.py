from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ----------------------------------------------------
    # MoveIt configuration (THIS is the core)
    # ----------------------------------------------------
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="mycobot_280",
            package_name="arm_moveit_config"
        )
        .robot_description(file_path="config/mycobot_280.urdf.xacro")
        .robot_description_semantic(file_path="config/mycobot_280.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )

    # ----------------------------------------------------
    # RViz configuration
    # ----------------------------------------------------
    rviz_config_path = os.path.join(
        get_package_share_directory("arm_moveit_config"),
        "config",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    # ----------------------------------------------------
    # Move Group (THIS talks to Gazebo controllers)
    # ----------------------------------------------------
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # ----------------------------------------------------
    # Launch
    # ----------------------------------------------------
    return LaunchDescription([
        move_group_node,
        rviz_node,
    ])
