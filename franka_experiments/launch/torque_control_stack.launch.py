"""Torque control stack — FR3 robot (acceleration-space pipeline).

Complete launch for the joint-torque control pipeline.

Pipeline:
  [Perception]     RealSense camera driver
  [Distance est.]  real_time_distance       →  /cbf/per_link_distances
  [Motion gen.]    pentagon_qddot_commander →  /NS_1/qddot_nom   (q̈_nom)
  [CBF filter]     cbf_safety_filter        →  /NS_1/qddot_safe  (safe q̈)
  [Dynamics conv.] qddot_to_torque          →  /NS_1/torque_cmd  (τ = M·q̈ + C·q̇)
  [Execution]      rt_torque_controller      ←  /NS_1/torque_cmd  →  hardware  (adds g(q))

Startup sequence (delays relative to launch time)
--------------------------------------------------
  t = 0                      franka bringup  (robot driver + joint_state_broadcaster)
  t = 1 s                    world → fr3_link0 static TF (identity)
  t = camera_delay_s         RealSense camera driver  (if enable_camera)
  t = 1 s                    camera extrinsics static TF  (if enable_camera)
  t = camera_delay_s + 3 s   image republisher  (if enable_camera)
  t = control_delay          rt_torque_controller spawner
  t = control_delay + 4 s    cbf_safety_filter + qddot_to_torque
  t = control_delay + 6 s    real_time_distance  (if start_real_time_distance)
  t = control_delay + 8 s    pentagon_qddot_commander  (motion generator)

Examples
--------
.. code-block:: bash

    # Full stack (camera + distance estimation, default):
    ros2 launch franka_experiments torque_control_stack.launch.py

    # Without camera / distance (minimal, trajectory only):
    ros2 launch franka_experiments torque_control_stack.launch.py \\
        enable_camera:=false start_real_time_distance:=false

    # Fake hardware (simulation):
    ros2 launch franka_experiments torque_control_stack.launch.py use_fake_hardware:=true
"""

import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from franka_experiments.utils.ros import (
    declare_robot_args,
    declare_rt_torque_args,
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
    'gazebo', 'lpf_alpha', 'tau_max_scale',
    'control_spawner_delay_s',
    'enable_camera', 'camera_extrinsics_yaml', 'camera_delay_s',
    'start_real_time_distance',
    'start_experiment_logger', 'experiment_logger_delay_s',
]


def _as_bool(x: str) -> bool:
    return str(x).strip().lower() in ('1', 'true', 'yes', 'y', 'on')


def _launch_all(context):
    p = {k: LaunchConfiguration(k).perform(context) for k in _ALL_PARAMS}

    use_fake     = _as_bool(p['use_fake_hardware'])
    start_camera = _as_bool(p['enable_camera'])
    start_rtd    = _as_bool(p['start_real_time_distance'])

    control_delay   = float(p['control_spawner_delay_s'])
    dynamics_delay  = control_delay + 4.0   # qddot_to_torque starts after controller
    rtd_delay       = control_delay + 6.0   # real_time_distance starts after dynamics node
    commander_delay = control_delay + 8.0   # commander starts last

    # ── Build controller YAML for rt_torque_controller ────────────────────────
    # The controller listens on torque_cmd — the direct output of qddot_to_torque.
    rt_params = dict(
        is_real=not use_fake,
        arm_id=p['arm_id'],
        controller_type='torque',
        torque_command_topic='torque_cmd',
        gazebo=p['gazebo'],
        lpf_alpha=float(p['lpf_alpha']),
        tau_max_scale=float(p['tau_max_scale']),
    )
    controllers_yaml = pick_controllers_yaml(p['controllers_yaml'], use_fake, rt_params)
    cm_name          = resolve_controller_manager_name(p['namespace'])

    # ── [Execution] franka bringup + rt_torque_controller ────────────────────
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
        arguments=['rt_torque_controller',
                   '--controller-manager', cm_name,
                   '--controller-manager-timeout', '30'],
        output='screen',
    )

    # world → fr3_link0 static TF
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
            '[torque_stack] accel-space pipeline'
            '  arm=', p['arm_id'],
            '  ip=', p['robot_ip'],
            '  fake=', p['use_fake_hardware'],
        ]),
        franka_launch,
        TimerAction(period=1.0, actions=[world_tf_node]),
        TimerAction(period=control_delay, actions=[controller_spawner]),
    ]

    # ── [Perception] RealSense camera ─────────────────────────────────────────
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
        actions.append(LogInfo(msg=['[torque_stack] [Perception]      camera ENABLED '
                                    '(delay=', str(cam_delay), 's)']))
    else:
        actions.append(LogInfo(msg='[torque_stack] [Perception]      camera DISABLED'))

    # ── [Distance estimation] real_time_distance ──────────────────────────────
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
        actions.append(LogInfo(msg=['[torque_stack] [Distance est.]   real_time_distance ENABLED '
                                    '(delay=', str(rtd_delay), 's)']))
    else:
        actions.append(LogInfo(msg='[torque_stack] [Distance est.]   real_time_distance DISABLED'))

    # ── [CBF safety filter] cbf_safety_filter ────────────────────────────────
    # Reads /NS_1/qddot_nom, applies acceleration-space CBF QP, publishes
    # /NS_1/qddot_safe.  qddot_to_torque converts qddot_safe → torque_cmd.
    cbf_node = Node(
        package='franka_experiments',
        executable='cbf_safety_filter',
        name='cbf_safety_filter',
        output='screen',
    )
    # qddot_to_torque subscribes to qddot_nom by default; remap to qddot_safe
    # so it converts the CBF-filtered acceleration to torque.
    qddot_to_torque_node = Node(
        package='franka_experiments',
        executable='qddot_to_torque',
        name='qddot_to_torque',
        output='screen',
        remappings=[('/NS_1/qddot_nom', '/NS_1/qddot_safe')],
    )
    actions.append(TimerAction(period=dynamics_delay,
                               actions=[cbf_node, qddot_to_torque_node]))
    actions.append(LogInfo(msg=['[torque_stack] [CBF filter]      cbf_safety_filter + qddot_to_torque'
                                ' (delay=', str(dynamics_delay), 's)']))

    # ── [Motion generation] pentagon_qddot_commander ──────────────────────────
    commander_node = Node(
        package='franka_experiments',
        executable='pentagon_qddot_commander',
        name='pentagon_qddot_commander',
        output='screen',
    )
    actions.append(TimerAction(period=commander_delay, actions=[commander_node]))
    actions.append(LogInfo(msg=['[torque_stack] [Motion gen.]     pentagon_qddot_commander '
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
        actions.append(LogInfo(msg='[torque_stack] [Logging]         experiment_logger ENABLED'))
    else:
        actions.append(LogInfo(msg='[torque_stack] [Logging]         experiment_logger DISABLED'))

    return actions


def generate_launch_description():
    return LaunchDescription(
        declare_robot_args(_DEFAULTS)
        + declare_rt_torque_args(_DEFAULTS)
        + [
            DeclareLaunchArgument(
                'control_spawner_delay_s',
                default_value=str(_DEFAULTS.get('control_spawner_delay_s', '10.0')),
                description='Seconds before spawning rt_torque_controller'),
            DeclareLaunchArgument(
                'enable_camera',
                default_value=_DEFAULTS.get('enable_camera', 'true'),
                description='Start RealSense camera driver and image republisher'),
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
                default_value=_DEFAULTS.get('start_real_time_distance', 'true'),
                description='Start real_time_distance node'),
            DeclareLaunchArgument(
                'start_experiment_logger',
                default_value='true',
                description='Start experiment logger automatically'),
            DeclareLaunchArgument(
                'experiment_logger_delay_s',
                default_value='2.0',
                description='Seconds before launching experiment_logger'),
            OpaqueFunction(function=_launch_all),
        ]
    )
