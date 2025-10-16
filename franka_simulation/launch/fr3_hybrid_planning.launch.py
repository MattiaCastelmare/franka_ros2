#!/usr/bin/env python3
"""
FR3 Hybrid Planning Launch
===========================

Launch completo per Hybrid Planning con MoveIt 2:
- Global Planner (OMPL) via move_group
- Local Planner (MoveIt Servo)
- Planning Scene Monitor
- Gazebo + Controllers + RViz

Pattern: riusa move_group.launch.py e aggiunge Servo + Hybrid config
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable


def load_yaml(package_name, file_path):
    """Load YAML file helper"""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    """Generate launch description for Hybrid Planning"""
    
    # Launch arguments (same as move_group.launch.py)
    load_gripper_arg = DeclareLaunchArgument("load_gripper", default_value="true")
    franka_hand_arg = DeclareLaunchArgument("franka_hand", default_value="franka_hand")
    arm_id_arg = DeclareLaunchArgument("arm_id", default_value="fr3")
    enable_moveit_arg = DeclareLaunchArgument(
        "enable_moveit", default_value="true",
        description="Enable MoveIt integration (must be true for hybrid)"
    )
    spawn_obstacles_arg = DeclareLaunchArgument(
        'spawn_obstacles', default_value='true',
        description='Spawn collision obstacles in simulation'
    )

    # Include base move_group launch (has Gazebo + MoveGroup + Controllers)
    pkg_franka_sim = get_package_share_directory("franka_simulation")
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_franka_sim, "launch", "move_group.launch.py")
        ),
        launch_arguments={
            'load_gripper': LaunchConfiguration('load_gripper'),
            'franka_hand': LaunchConfiguration('franka_hand'),
            'arm_id': LaunchConfiguration('arm_id'),
            'enable_moveit': 'true',  # Force MoveIt ON for hybrid
            'spawn_obstacles': LaunchConfiguration('spawn_obstacles'),
        }.items(),
    )

    # Load hybrid planning configuration
    hybrid_config = load_yaml('franka_simulation', 'config/hybrid_planning.yaml')
    servo_config = load_yaml('franka_simulation', 'config/servo.yaml')

    # MoveIt Servo Node (Local Planner)
    arm_id = LaunchConfiguration('arm_id')
    load_gripper = LaunchConfiguration('load_gripper')

    # Percorsi ai file XACRO e SRDF (senza .perform)
    xacro_file = os.path.join(
        get_package_share_directory("franka_description"),
        "robots", "fr3", "fr3.urdf.xacro"
    )

    semantic_xacro_file = os.path.join(
        get_package_share_directory("franka_description"),
        "robots", "fr3", "fr3.srdf.xacro"
    )

    # Genera robot_description (URDF)
    robot_description = {'robot_description': ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' ros2_control:=false',
            ' hand:=', LaunchConfiguration('load_gripper'),
            ' arm_id:=', LaunchConfiguration('arm_id')
        ]),
        value_type=str
    )}

    # Genera robot_description_semantic (SRDF)
    robot_description_semantic = {'robot_description_semantic': ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', semantic_xacro_file,
            ' hand:=', LaunchConfiguration('load_gripper')
        ]),
        value_type=str
    )}


    # Nodo Servo
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        namespace='moveit_servo',
        output='screen',
        parameters=[
            servo_config,
            robot_description,
            robot_description_semantic,
            {'use_sim_time': True,
            'move_group_name': 'fr3_arm',
            'command_out_topic': '/fr3_arm_controller/joint_trajectory'}
        ],
        remappings=[
            ('/servo_node/delta_twist_cmds', '/servo_server/delta_twist_cmds'),
            ('/servo_node/delta_joint_cmds', '/servo_server/delta_joint_cmds'),
            ('/servo_node/status', '/servo_server/status'),
        ]
    )


    # Placeholder for Hybrid Planning Coordinator (Step 2)
    # Will be replaced with actual coordinator node
    coordinator_placeholder = Node(
        package='franka_simulation',
        executable='hybrid_planning_coordinator_placeholder',
        name='hybrid_planning_coordinator',
        output='screen',
        parameters=[hybrid_config, {'use_sim_time': True}],
        condition=lambda context: False  # Disabled for Step 1
    )

    # Delay Servo start after MoveGroup is ready
    delayed_servo = TimerAction(
        period=10.0,  # Wait for move_group + controllers
        actions=[servo_node]
    )

    return LaunchDescription([
        # Arguments
        load_gripper_arg,
        franka_hand_arg,
        arm_id_arg,
        enable_moveit_arg,
        spawn_obstacles_arg,
        
        # Base system (Gazebo + MoveIt + Controllers)
        move_group_launch,
        
        # Hybrid Planning components
        delayed_servo,
        # coordinator_placeholder,  # Disabled for Step 1
    ])