#!/usr/bin/env python3
"""
Framework Test Launch - STEP 1
===============================

Launch file per testare solo il Nodo A (Goal Provider).
Integrazione minimale con l'architettura esistente di franka_simulation.

Test: Verificare che il Goal Provider riceva e memorizzi correttamente 
start e goal pose, e possa triggerare richieste di planning.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    declare_launch_base_system_arg = DeclareLaunchArgument(
        'launch_base_system',
        default_value='true',
        description='Launch Gazebo + MoveIt base system'
    )
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true',
        description='Use simulation time'
    )
    
    # Launch configurations
    launch_base_system = LaunchConfiguration('launch_base_system')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Include franka_simulation move_group launch se richiesto
    base_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('franka_simulation'),
                'launch',
                'move_group.launch.py'
            ])
        ]),
        launch_arguments={
            'arm_id': 'fr3',
            'load_gripper': 'true',
            'use_sim_time': use_sim_time,
            'launch_gazebo': 'true',
            'launch_controllers': 'true', 
            'enable_moveit': 'true',
            'enable_rviz': 'true',
        }.items(),
        condition=IfCondition(launch_base_system)
    )
    
    # Goal Provider Node (Nodo A)
    goal_provider_node = Node(
        package='franka_simulation',
        executable='goal_provider_node',
        name='goal_provider',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        # Remappings per integrazione con sistema esistente
        remappings=[
            # Il current_pose sarà pubblicato da move_group o robot_state_publisher
            ('/current_pose', '/move_group/monitored_planning_scene'), # Placeholder
        ]
    )
    
    # Delayed start per assicurarsi che il base system sia pronto
    delayed_goal_provider = TimerAction(
        period=10.0,  # Attesa per Gazebo + MoveIt startup
        actions=[goal_provider_node]
    )
    
    return LaunchDescription([
        # Arguments
        declare_launch_base_system_arg,
        declare_use_sim_time_arg,
        
        # Launch base system se richiesto
        base_system_launch,
        
        # Framework nodes
        delayed_goal_provider,
    ])