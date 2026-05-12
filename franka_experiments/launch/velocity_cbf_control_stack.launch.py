"""Velocity CBF control stack — FR3 robot.

Complete launch for the joint-velocity control pipeline with CBF obstacle
avoidance.  Nodes are started in the following logical order:

  [Perception]         RealSense camera driver
  [Distance est.]      real_time_distance  →  /human_robot/multi_distance
  [Safety layer]       cbf_velocity_filter
                         ← /NS_1/tracking_qdot   (nominal from commander)
                         ← /human_robot/multi_distance
                         →  /NS_1/qdot_cmd        (safe velocity command)
  [Motion generation]  ee_pentagon_velocity_commander
                         →  /NS_1/tracking_qdot
  [Execution]          rt_velocity_executor_controller
                         ← /NS_1/qdot_cmd  →  hardware

Two-phase operation
-------------------
Phase 1  (bypass_cbf:=true, default)
  cbf_velocity_filter passes tracking_qdot → qdot_cmd unmodified.
  Camera and real_time_distance are NOT started.
  Use this phase to verify smooth trajectory execution first.

Phase 2  (bypass_cbf:=false)
  cbf_velocity_filter solves a CBF QP at each distance update, modifying
  joint velocities only when the robot approaches an obstacle.
  Camera + real_time_distance are started automatically.

Startup sequence (delays relative to launch time)
--------------------------------------------------
  t = 0                    franka bringup  (robot driver + joint_state_broadcaster)
  t = 1 s                  world → fr3_link0 static TF (identity)
  [Phase 2] t = 0 s        RealSense camera driver
  [Phase 2] t = 3 s        image republisher
  [Phase 2] t = 1 s        camera extrinsics static TF
  t = control_delay        rt_velocity_executor_controller spawner
  t = control_delay + 4 s  cbf_velocity_filter
  [Phase 2] t = control_delay + 6 s  real_time_distance (after controller active)
  t = control_delay + 8 s  ee_pentagon_velocity_commander

Examples
--------
.. code-block:: bash

    # Phase 1 — bypass CBF, no camera required:
    ros2 launch franka_experiments velocity_cbf_control_stack.launch.py

    # Phase 2 — full CBF with obstacle avoidance:
    ros2 launch franka_experiments velocity_cbf_control_stack.launch.py bypass_cbf:=false

    # Fake hardware (simulation):
    ros2 launch franka_experiments velocity_cbf_control_stack.launch.py use_fake_hardware:=true
"""

import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from franka_experiments.utils.ros import (
    declare_robot_args,
    declare_rt_blender_args,
    load_franka_config_defaults,
    load_launch_defaults,
    pick_controllers_yaml,
    resolve_controller_manager_name,
)

# ── Defaults (single source of truth) ────────────────────────────────────────
_LAUNCH_DEFAULTS, _ = load_launch_defaults()
_BRINGUP_DEFAULTS, _ = load_franka_config_defaults()
_DEFAULTS = {**_LAUNCH_DEFAULTS, **_BRINGUP_DEFAULTS}

_ALL_PARAMS = [
    'namespace', 'use_fake_hardware', 'robot_ip', 'arm_id',
    'fake_sensor_commands', 'load_gripper', 'controllers_yaml',
    'qdot_max', 'max_accel', 'timeout_threshold_s', 'timeout_ramp_s',
    'gazebo', 'enable_interpolation', 'command_topic',
    'control_spawner_delay_s',
    'enable_camera', 'camera_extrinsics_yaml', 'camera_delay_s',
    'start_real_time_distance', 'start_experiment_logger',
    'experiment_logger_delay_s',
    'bypass_cbf',
]


def _as_bool(x: str) -> bool:
    return str(x).strip().lower() in ('1', 'true', 'yes', 'y', 'on')


def _launch_all(context):
    p = {k: LaunchConfiguration(k).perform(context) for k in _ALL_PARAMS}

    bypass_cbf = _as_bool(p['bypass_cbf'])
    use_fake   = _as_bool(p['use_fake_hardware'])

    # In Phase 2 the camera is always needed (for real_time_distance)
    start_camera = (not bypass_cbf) or _as_bool(p['enable_camera'])
    start_rtd    = (not bypass_cbf) or _as_bool(p['start_real_time_distance'])

    control_delay    = float(p['control_spawner_delay_s'])
    cbf_node_delay   = control_delay + 4.0   # CBF filter starts after controller
    rtd_delay        = control_delay + 6.0   # real_time_distance starts after CBF
    commander_delay  = control_delay + 8.0   # commander starts last

    # ── Generate controller YAML ──────────────────────────────────────────────
    rt_params = dict(
        is_real=not use_fake, arm_id=p['arm_id'],
        qdot_max=p['qdot_max'],
        command_topic=p['command_topic'],        # qdot_cmd — CBF filter output
        max_accel=p['max_accel'],
        timeout_threshold_s=p['timeout_threshold_s'],
        timeout_ramp_s=p['timeout_ramp_s'],
        gazebo=p['gazebo'],
        enable_interpolation=p['enable_interpolation'],
    )
    controllers_yaml = pick_controllers_yaml(p['controllers_yaml'], use_fake, rt_params)
    cm_name          = resolve_controller_manager_name(p['namespace'])

    phase_str = 'Phase 1 — BYPASS (no CBF, no camera)' if bypass_cbf \
                else 'Phase 2 — CBF QP active'

    # ── [Execution] franka bringup + rt_velocity_executor_controller ──────────
    franka_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py',
        ]).perform(context)),
        launch_arguments={
            'arm_id':               p['arm_id'],
            'robot_ip':             p['robot_ip'],
            'namespace':            p['namespace'],
            'use_fake_hardware':    p['use_fake_hardware'],
            'fake_sensor_commands': p['fake_sensor_commands'],
            'load_gripper':         p['load_gripper'],
            'controllers_yaml':     controllers_yaml,
        }.items(),
    )

    controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['rt_velocity_executor_controller',
                   '--controller-manager', cm_name,
                   '--controller-manager-timeout', '30'],
        output='screen',
    )

    # world → fr3_link0 static TF (always)
    world_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_robot_base_tf',
        output='log',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'world',
            '--child-frame-id', 'fr3_link0',
        ],
    )

    actions = [
        LogInfo(msg=[
            '[vel_cbf_stack] ', phase_str,
            '  arm=', p['arm_id'],
            '  ip=', p['robot_ip'],
            '  fake=', p['use_fake_hardware'],
        ]),
        franka_launch,
        TimerAction(period=1.0, actions=[world_tf_node]),
        TimerAction(period=control_delay, actions=[controller_spawner]),
    ]

    # ── [Perception] RealSense camera (Phase 2 only) ──────────────────────────
    if start_camera:
        cam_delay = float(p['camera_delay_s'])

        realsense_driver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py',
            ]).perform(context)),
        )
        actions.append(TimerAction(period=cam_delay, actions=[realsense_driver]))

        image_republisher = Node(
            package='franka_simulation',
            executable='image_publisher',
            name='image_republisher',
            output='log',
        )
        actions.append(TimerAction(period=cam_delay + 3.0, actions=[image_republisher]))

        # Camera extrinsics static TF
        extrinsics_path = p['camera_extrinsics_yaml']
        with open(extrinsics_path, 'r') as f:
            ext = yaml.safe_load(f)
        t_ext = ext['translation']
        r_ext = ext['rotation']
        camera_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_extrinsics_tf',
            output='log',
            arguments=[
                '--x',  str(t_ext['x']), '--y',  str(t_ext['y']), '--z',  str(t_ext['z']),
                '--qx', str(r_ext['x']), '--qy', str(r_ext['y']),
                '--qz', str(r_ext['z']), '--qw', str(r_ext['w']),
                '--frame-id', ext['parent_frame'],
                '--child-frame-id', ext['child_frame'],
            ],
        )
        actions.append(TimerAction(period=1.0, actions=[camera_tf_node]))
        actions.append(LogInfo(msg=['[vel_cbf_stack] [Perception]      camera ENABLED '
                                    '(delay=', str(cam_delay), 's)']))
    else:
        actions.append(LogInfo(msg='[vel_cbf_stack] [Perception]      camera DISABLED '
                                   '(bypass_cbf:=true)'))

    # ── [Distance estimation] real_time_distance (Phase 2 only) ──────────────
    if start_rtd:
        rtd_config = PathJoinSubstitution([
            FindPackageShare('franka_experiments'),
            'config', 'fr3_complete.yaml',
        ]).perform(context)
        real_time_distance_node = Node(
            package='franka_experiments',
            executable='real_time_distance',
            name='real_time_distance',
            output='log',
            parameters=[{
                'robot_config_path':      rtd_config,
                'camera_extrinsics_path': p['camera_extrinsics_yaml'],
            }],
        )
        actions.append(TimerAction(period=rtd_delay, actions=[real_time_distance_node]))
        actions.append(LogInfo(msg=['[vel_cbf_stack] [Distance est.]   real_time_distance ENABLED '
                                    '(delay=', str(rtd_delay), 's)']))
    else:
        actions.append(LogInfo(msg='[vel_cbf_stack] [Distance est.]   real_time_distance DISABLED '
                                   '(bypass_cbf:=true)'))

    # ── [Safety layer] cbf_velocity_filter ────────────────────────────────────
    cbf_node = Node(
        package='franka_experiments',
        executable='cbf_velocity_filter',
        name='cbf_velocity_filter',
        output='screen',
        parameters=[{'bypass_cbf': bypass_cbf}],
    )
    actions.append(TimerAction(period=cbf_node_delay, actions=[cbf_node]))
    actions.append(LogInfo(msg=['[vel_cbf_stack] [Safety layer]    cbf_velocity_filter '
                                '(delay=', str(cbf_node_delay), 's  bypass=',
                                str(bypass_cbf), ')']))

    # ── [Motion generation] ee_pentagon_velocity_commander ────────────────────
    commander_node = Node(
        package='franka_experiments',
        executable='ee_pentagon_velocity_commander',
        name='ee_pentagon_velocity_commander',
        output='screen',
    )
    actions.append(TimerAction(period=commander_delay, actions=[commander_node]))
    actions.append(LogInfo(msg=['[vel_cbf_stack] [Motion gen.]     ee_pentagon_velocity_commander '
                                '(delay=', str(commander_delay), 's)']))

    # ── Experiment logger ─────────────────────────────────────────────────────
    if _as_bool(p['start_experiment_logger']):
        experiment_logger_node = Node(
            package='franka_experiments',
            executable='experiment_logger',
            name='experiment_logger',
            output='screen',
        )
        actions.append(TimerAction(
            period=float(p['experiment_logger_delay_s']),
            actions=[experiment_logger_node],
        ))
        actions.append(LogInfo(msg='[vel_cbf_stack] [Logging]         experiment_logger ENABLED'))
    else:
        actions.append(LogInfo(msg='[vel_cbf_stack] [Logging]         experiment_logger DISABLED'))

    return actions


def generate_launch_description():
    return LaunchDescription(
        declare_robot_args(_DEFAULTS)
        + declare_rt_blender_args(_DEFAULTS)
        + [
            DeclareLaunchArgument(
                'control_spawner_delay_s',
                default_value=str(_DEFAULTS.get('control_spawner_delay_s', '10.0')),
                description='Seconds before spawning rt_velocity_executor_controller'),
            DeclareLaunchArgument(
                'enable_camera',
                default_value='false',
                description='Force-enable camera even in bypass mode'),
            DeclareLaunchArgument(
                'camera_extrinsics_yaml',
                default_value=PathJoinSubstitution([
                    FindPackageShare('franka_experiments'),
                    'config', 'camera_extrinsics.yaml',
                ]),
                description='Path to camera_extrinsics.yaml'),
            DeclareLaunchArgument(
                'camera_delay_s',
                default_value=str(_DEFAULTS.get('camera_delay_s', '0.0')),
                description='Seconds before launching camera pipeline'),
            DeclareLaunchArgument(
                'start_real_time_distance',
                default_value='false',
                description='Force-enable real_time_distance even in bypass mode'),
            DeclareLaunchArgument(
                'start_experiment_logger',
                default_value='true',
                description='Start experiment logger automatically'),
            DeclareLaunchArgument(
                'experiment_logger_delay_s',
                default_value='2.0',
                description='Seconds before launching experiment_logger'),
            DeclareLaunchArgument(
                'bypass_cbf',
                default_value='false',
                description='Phase 1: bypass CBF QP — tracking_qdot passes directly to qdot_cmd. '
                            'Set false for Phase 2 with full CBF obstacle avoidance.'),
            OpaqueFunction(function=_launch_all),
        ]
    )
