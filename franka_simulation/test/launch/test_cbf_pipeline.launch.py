#!/usr/bin/env python3
"""Test launch: CBF (avoidance) pipeline validation.

Starts the full CBF simulation pipeline (move_group.launch.py) and
optionally starts the avoidance test node to verify end-to-end operation.

Pipeline under test:
    franka_motion_server (MoveIt)
        └→ /velocity_blender/trajectory
    obstacle_synchronizer
        └→ /obstacle_scene
    online_avoidance_controller (Pinocchio + capsules)
        └→ /avoidance/min_distance
        └→ /avoidance/closest_constraint
        └→ /robot_capsules_markers (RViz)
    velocity_control_blender (CBF-QP)
        └→ /fr3_velocity_controller/commands
    fr3_velocity_controller → Gazebo

What to expect:
    - Gazebo opens with FR3 robot + obstacle scene
    - RViz opens with MoveIt panel and robot capsule markers (yellow cylinders)
    - /avoidance/min_distance topic publishes (> 0 when no collision)
    - /fr3_velocity_controller/commands publishes at ~100 Hz
    - Send trajectory via MoveIt panel or avoidance_test node to see CBF in action

CBF capsule markers in RViz:
    Topic: /robot_capsules_markers (MarkerArray)
    Display type: MarkerArray
    Expected: yellow/green cylinders overlapping robot links

Checking CBF is active:
    ros2 topic echo /avoidance/min_distance --once
    ros2 topic hz /fr3_velocity_controller/commands

Launch arguments forwarded to move_group.launch.py:
    spawn_obstacles:=true/false  — add obstacle scene to Gazebo
    enable_moveit:=true/false    — start MoveIt move_group
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('franka_simulation')

    spawn_obstacles_arg = DeclareLaunchArgument(
        'spawn_obstacles', default_value='true',
        description='Spawn obstacle boxes in Gazebo for CBF avoidance testing')
    enable_moveit_arg = DeclareLaunchArgument(
        'enable_moveit', default_value='true',
        description='Start MoveIt move_group (needed for trajectory planning)')

    spawn_obstacles = LaunchConfiguration('spawn_obstacles')
    enable_moveit   = LaunchConfiguration('enable_moveit')

    cbf_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={
            'spawn_obstacles': spawn_obstacles,
            'enable_moveit':   enable_moveit,
        }.items(),
    )

    return LaunchDescription([
        spawn_obstacles_arg,
        enable_moveit_arg,
        cbf_pipeline,
    ])
