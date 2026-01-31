#!/usr/bin/env python3
"""
Launch MoveIt 2 for the myCobot robotic arm with ROS 2 controllers.

This script creates a ROS 2 launch file that starts the necessary nodes and services
for controlling a myCobot robotic arm using MoveIt 2. It loads configuration files,
starts the move_group node, and optionally launches RViz for visualization.

:author: Addison Sears-Collins
:date: December 13, 2024
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, OpaqueFunction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """
    Generate a launch description for MoveIt 2 with myCobot robot.

    This function sets up the necessary configuration and nodes to launch MoveIt 2
    for controlling a myCobot robotic arm. It includes setting up paths to config files,
    declaring launch arguments, configuring the move_group node, and optionally starting RViz.

    Returns:
        LaunchDescription: A complete launch description for the MoveIt 2 system
    """
    # Constants for paths to different files and folders
    package_name_moveit_config = 'mycobot_moveit_config'

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    rviz_config_package = LaunchConfiguration('rviz_config_package')
    robot_name = LaunchConfiguration('robot_name')
    use_gazebo = LaunchConfiguration('use_gazebo', default='false')

    # Get the package share directory
    pkg_share_moveit_config_temp = FindPackageShare(package=package_name_moveit_config)

    # Declare the launch arguments
    declare_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value='mycobot_280',
        description='Name of the robot to use')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RViz')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value='move_group.rviz',
        description='RViz configuration file')

    declare_rviz_config_package_cmd = DeclareLaunchArgument(
        name='rviz_config_package',
        default_value=package_name_moveit_config,
        description='Package containing the RViz configuration file')
    
    declare_use_gazebo_cmd = DeclareLaunchArgument(
        name='use_gazebo',
        default_value='false',
        description='Whether to use Gazebo simulation (affects hardware interface)')

    def configure_setup(context):
        """Configure MoveIt and create nodes with proper string conversions."""
        # Get the robot name as a string for use in MoveItConfigsBuilder
        robot_name_str = LaunchConfiguration('robot_name').perform(context)
        use_gazebo_str = LaunchConfiguration('use_gazebo').perform(context)

        # Get package path
        pkg_share_moveit_config = pkg_share_moveit_config_temp.find(package_name_moveit_config)

        # Construct file paths using robot name string
        config_path = os.path.join(pkg_share_moveit_config, 'config', robot_name_str)

        # Define all config file paths
        initial_positions_file_path = os.path.join(config_path, 'initial_positions.yaml')
        joint_limits_file_path = os.path.join(config_path, 'joint_limits.yaml')
        kinematics_file_path = os.path.join(config_path, 'kinematics.yaml')
        moveit_controllers_file_path = os.path.join(config_path, 'moveit_controllers.yaml')
        srdf_model_path = os.path.join(config_path, f'{robot_name_str}.srdf')
        pilz_cartesian_limits_file_path = os.path.join(config_path, 'pilz_cartesian_limits.yaml')
        robofilepath = os.path.join(
            os.path.expanduser("~"),
            "ros2_ws",
            "src",
            "mycobot_ros2",
            "mycobot_description",
            "urdf",
            "robots",
            "mycobot_280.urdf.xacro"
        )
        
        # ROS 2 controllers configuration file
        ros2_controllers_file_path = os.path.join(config_path, 'ros2_controllers.yaml')

        # Create MoveIt configuration
        moveit_config = (
            MoveItConfigsBuilder(robot_name_str, package_name=package_name_moveit_config)
            .trajectory_execution(file_path=moveit_controllers_file_path)
            .robot_description_semantic(file_path=srdf_model_path)
            .joint_limits(file_path=joint_limits_file_path)
            .robot_description_kinematics(file_path=kinematics_file_path)
            .planning_pipelines(
                pipelines=["ompl"],
                default_planning_pipeline="ompl"
            )
            .planning_scene_monitor(
                publish_robot_description=False,
                publish_robot_description_semantic=True,
                publish_planning_scene=True,
            )
            .to_moveit_configs()
        )

        # Load ROS 2 controllers YAML
        from yaml import safe_load
        
        with open(ros2_controllers_file_path, 'r') as f:
            ros2_controllers_yaml = safe_load(f)
        
        # Create robot description with correct hardware interface based on use_gazebo
        robot_description_content = f"""
        <?xml version="1.0"?>
        <robot name="{robot_name_str}">
            <xacro:include filename="$(find mycobot_description)/urdf/mycobot_280.xacro" />
            <xacro:mycobot_280 
                robot_name="{robot_name_str}"
                prefix=""
                use_gazebo="{use_gazebo_str}"
                use_gripper="true"
                use_camera="false"
                add_world="true"
                base_type="g_shape"
                gripper_type="adaptive_gripper" />
        </robot>
        """
        
        # Create robot description parameter for controller_manager
        robot_description_param = {
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }
        
        # Merge controller parameters with robot description
        controller_manager_params = {**robot_description_param, **ros2_controllers_yaml}

        # MoveIt capabilities
        move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}

        # ========== CONTROLLER NODES (as shown in video) ==========
        
        # 1. Controller Manager Node (CRITICAL - as shown at ~30:00 in video)
        start_controller_manager_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controller_manager_params],
            output='screen',
            condition=IfCondition(use_sim_time),  # Only start if using simulation
        )
        
        # 2. Joint State Broadcaster Spawner
        joint_state_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
            condition=IfCondition(use_sim_time),
        )
        
        # 3. Arm Controller Spawner (as shown in video - exact names must match YAML)
        arm_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller', '--controller-manager', '/controller_manager'],
            output='screen',
            condition=IfCondition(use_sim_time),
        )
        
        # 4. Gripper Controller Spawner
        gripper_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_action_controller', '--controller-manager', '/controller_manager'],
            output='screen',
            condition=IfCondition(use_sim_time),
        )

        # ========== MOVEIT NODES ==========
        
        # Create move_group node
        start_move_group_node_cmd = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {'use_sim_time': use_sim_time},
                {'start_state': {'content': initial_positions_file_path}},
                move_group_capabilities,
            ],
        )

        # Create RViz node
        start_rviz_node_cmd = Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            arguments=[
                "-d",
                [FindPackageShare(rviz_config_package), "/rviz/", rviz_config_file]
            ],
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                {'use_sim_time': use_sim_time}
            ],
        )
        
        # Robot State Publisher (for TF)
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description_content},
                {'use_sim_time': use_sim_time}
            ],
        )

        # RViz exit handler
        exit_event_handler = RegisterEventHandler(
            condition=IfCondition(use_rviz),
            event_handler=OnProcessExit(
                target_action=start_rviz_node_cmd,
                on_exit=EmitEvent(event=Shutdown(reason='rviz exited')),
            ),
        )

        return [
            # Controller nodes first (as shown in video - controllers must start before MoveIt)
            start_controller_manager_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
            
            # Then robot state publisher
            robot_state_publisher_node,
            
            # Then MoveIt nodes
            start_move_group_node_cmd,
            start_rviz_node_cmd,
            
            # Event handlers
            exit_event_handler,
        ]

    # Create the launch description
    ld = LaunchDescription()

    # Add the launch arguments
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_rviz_config_package_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_gazebo_cmd)

    # Add the setup and node creation
    ld.add_action(OpaqueFunction(function=configure_setup))

    return ld