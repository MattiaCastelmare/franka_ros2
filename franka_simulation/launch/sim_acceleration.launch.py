#!/usr/bin/env python3
"""Acceleration Simulation Pipeline.

Builds on the velocity pipeline by adding an integration bridge node that
converts joint acceleration commands into velocity setpoints.

Topic graph:
    external_node → /sim_acceleration_bridge/accel_cmd (Float64MultiArray, q̈)
    sim_acceleration_bridge → /fr3_velocity_controller/commands (q̇)

The bridge integrates:
    q̇(t+1) = clamp(q̇(t) + q̈·dt,  -qdot_max,  +qdot_max)

and reads current q̇ from /joint_states to seed each step.

No MoveIt, no CBF — minimal clean acceleration pipeline for external commanders.
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_rsp(context: LaunchContext, arm_id, load_gripper, franka_hand):
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)

    xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', arm_id_str, f'{arm_id_str}.urdf.xacro',
    )
    urdf_xml = xacro.process_file(
        xacro_file,
        mappings={
            'arm_id': arm_id_str,
            'hand': load_gripper_str,
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': franka_hand_str,
        },
    ).toxml()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': urdf_xml, 'use_sim_time': True}],
    )
    return [rsp]


def generate_launch_description():
    load_gripper_arg = DeclareLaunchArgument('load_gripper', default_value='true')
    franka_hand_arg  = DeclareLaunchArgument('franka_hand',  default_value='franka_hand')
    arm_id_arg       = DeclareLaunchArgument('arm_id',       default_value='fr3')

    load_gripper = LaunchConfiguration('load_gripper')
    franka_hand  = LaunchConfiguration('franka_hand')
    arm_id       = LaunchConfiguration('arm_id')

    rsp = OpaqueFunction(function=_make_rsp, args=[arm_id, load_gripper, franka_hand])

    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(
        get_package_share_directory('franka_description')
    )
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items(),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    controllers_yaml = os.path.join(
        get_package_share_directory('franka_simulation'), 'config', 'controllers.yaml'
    )

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawn_joint_state_broadcaster',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--param-file', controllers_yaml,
        ],
        output='screen',
    )

    vel_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawn_fr3_velocity_controller',
        arguments=[
            'fr3_velocity_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', controllers_yaml,
        ],
        output='screen',
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    accel_bridge = Node(
        package='franka_simulation',
        executable='sim_acceleration_bridge',
        name='sim_acceleration_bridge',
        output='screen',
    )

    rviz_file = os.path.join(
        get_package_share_directory('franka_description'), 'rviz', 'visualize_franka.rviz'
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['--display-config', rviz_file, '-f', 'fr3_link0'],
        output='screen',
    )

    # --- Trajectory node -------------------------------------------------
    # Change executable= to switch trajectory. Available options:
    #   acceleration_smooth_trajectory — ramp-up / steady / ramp-down profile
    trajectory_node = Node(
        package='franka_simulation',
        executable='cartesian_acceleration_mapper',
        output='screen',
    )
    # ---------------------------------------------------------------------

    return LaunchDescription([
        load_gripper_arg,
        franka_hand_arg,
        arm_id_arg,
        gz,
        rsp,
        clock_bridge,
        TimerAction(period=3.0,  actions=[spawn]),
        TimerAction(period=5.0,  actions=[jsb_spawner]),
        TimerAction(period=7.0,  actions=[vel_spawner]),
        TimerAction(period=7.5,  actions=[accel_bridge]),
        TimerAction(period=8.5,  actions=[rviz]),
        TimerAction(period=10.0, actions=[trajectory_node]),
    ])
