#!/usr/bin/env python3
"""Test launch: Safe-RL (ONNX policy) accel pipeline with fake hardware.

Thin wrapper around the CANONICAL ``torque_control_stack.launch.py`` — it only
pins the fake-hardware / no-perception arguments, so there is exactly one
definition of the pipeline and this file can never drift from it.

Pipeline under test (franka_sim_to_real_roadmap.md, Step 3):

    rl_policy_commander   (ONNX policy, obs rebuilt from /NS_1/joint_states)
        → /NS_1/qddot_nom
        → cbf_safety_filter          (HOCBF QP — the safety certificate)
        → /NS_1/qddot_safe
        → qddot_to_torque            (τ = M·q̈ + C·q̇)
        → /NS_1/torque_cmd
        → rt_torque_controller       → fake Franka hardware

The camera / real_time_distance / move_group nodes are OFF: with no perception
the commander runs against a parked synthetic obstacle (its documented
"perception never started" path) and the CBF filter keeps only its hard state
and workspace rows.  That is the point of this test — it validates the command
chain, not obstacle avoidance.

Usage::

    ros2 launch franka_experiments test_rl_fake.launch.py \\
        rl_onnx_model:=/ros2_ws/src/franka_sim/models/<exp>/best_model.onnx

Topics to monitor::

    /NS_1/joint_states   JointState             fake feedback
    /NS_1/qddot_nom      Float64MultiArray(7)   policy output   ~100 Hz
    /NS_1/qddot_safe     Float64MultiArray(7)   CBF output      ~100 Hz
    /NS_1/torque_cmd     Float64MultiArray(7)   torque command  ~100 Hz
    /NS_1/rl_status      Float64MultiArray(6)   [infer_ms, tick_ms, d_min,
                                                 dist, target_idx, gate]

Controllers that must be ACTIVE: joint_state_broadcaster, rt_torque_controller.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

_RL_ARGS = ('rl_onnx_model', 'rl_sim_config', 'rl_target_xyz',
            'rl_target_sequence', 'rl_action_scale')


def generate_launch_description():
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('franka_experiments'), 'launch',
            'torque_control_stack.launch.py',
        ])),
        launch_arguments={
            'use_fake_hardware':        'true',
            'fake_sensor_commands':     'true',
            'motion_source':            'rl',
            'enable_camera':            'false',
            'start_real_time_distance': 'false',
            'start_move_group':         'false',
            'start_experiment_logger':  'false',
            **{a: LaunchConfiguration(a) for a in _RL_ARGS},
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rl_onnx_model', default_value='',
            description='Path to the exported .onnx policy (required)'),
        DeclareLaunchArgument('rl_sim_config', default_value=''),
        DeclareLaunchArgument('rl_target_xyz', default_value='[0.45, 0.0, 0.45]'),
        DeclareLaunchArgument('rl_target_sequence', default_value=''),
        DeclareLaunchArgument('rl_action_scale', default_value='1.0'),
        stack,
    ])
