"""Wrapper: bringup + velocity controller spawner (RT or legacy).

**RT mode** (``use_rt_blender:=true``, DEFAULT):
    Spawns ``rt_velocity_blender_controller`` — blending in C++ at 1 kHz.

**Legacy mode** (``use_rt_blender:=false``):
    Spawns ``fr3_forward_velocity_controller`` + optional Python blender.

Defaults are loaded from ``franka_experiments/config/launch_defaults.yaml``.
Robot-specific overrides come from ``franka_bringup/config/franka.config.yaml``.
Every argument is still overridable from the CLI.
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
    build_wrapper_log_actions,
    declare_legacy_blender_args,
    declare_robot_args,
    declare_rt_blender_args,
    load_franka_config_defaults,
    load_launch_defaults,
    pick_controllers_yaml,
    resolve_controller_manager_name,
)

# Load launch_defaults.yaml (user-editable, single source of truth)
_LAUNCH_DEFAULTS, _LAUNCH_DEFAULTS_PATH = load_launch_defaults()

# Merge robot-specific overrides from franka_bringup/config/franka.config.yaml
# (robot_ip, namespace, etc.) — these win over launch_defaults.yaml
_BRINGUP_DEFAULTS, _CONFIG_PATH = load_franka_config_defaults()
_DEFAULTS = {**_LAUNCH_DEFAULTS, **_BRINGUP_DEFAULTS}

# All LaunchConfiguration names resolved inside _launch_all
_ALL_PARAMS = [
    'use_rt_blender', 'namespace', 'use_fake_hardware', 'robot_ip', 'arm_id',
    'fake_sensor_commands', 'load_gripper', 'start_blender', 'controllers_yaml',
    'qdot_max', 'alpha', 'max_accel', 'timeout_threshold_s', 'timeout_ramp_s',
    'gazebo', 'enable_interpolation', 'alpha_topic', 'tracking_topic',
    'avoidance_topic', 'output_command_topic', 'blender_rate_hz',
    'blender_qdot_max', 'blender_watchdog_s',
    'enable_camera',
    'do_calibration',
]


def _launch_all(context):
    """Resolve all parameters, include bringup, spawn controller."""
    # ── Resolve every param once ──────────────────────────────────────
    p = {k: LaunchConfiguration(k).perform(context) for k in _ALL_PARAMS}
    use_rt = p['use_rt_blender'].lower() == 'true'
    use_fake = p['use_fake_hardware'].lower() == 'true'

    # ── Build RT param dict for YAML generation ───────────────────────
    rt_params = None
    if use_rt:
        trk_raw, avd_raw = p['tracking_topic'], p['avoidance_topic']
        rt_params = dict(
            is_real=not use_fake, arm_id=p['arm_id'],
            qdot_max=p['qdot_max'], alpha=p['alpha'],
            trk_topic='tracking_qdot' if trk_raw == '__auto__' else trk_raw,
            avd_topic='avoidance_qdot' if avd_raw == '__auto__' else avd_raw,
            alpha_topic=p['alpha_topic'], max_accel=p['max_accel'],
            timeout_threshold_s=p['timeout_threshold_s'],
            timeout_ramp_s=p['timeout_ramp_s'],
            gazebo=p['gazebo'],
            enable_interpolation=p['enable_interpolation'],
        )

    # ── Select YAML & controller ──────────────────────────────────────
    controllers_yaml = pick_controllers_yaml(
        p['controllers_yaml'], use_fake, use_rt, rt_params)
    controller_name = ('rt_velocity_blender_controller' if use_rt
                       else 'fr3_forward_velocity_controller')
    cm_name = resolve_controller_manager_name(p['namespace'])

    # ── Include franka bringup ────────────────────────────────────────
    franka_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py',
        ]).perform(context)),
        launch_arguments={
            'arm_id': p['arm_id'], 'robot_ip': p['robot_ip'],
            'namespace': p['namespace'],
            'use_fake_hardware': p['use_fake_hardware'],
            'fake_sensor_commands': p['fake_sensor_commands'],
            'load_gripper': p['load_gripper'],
            'controllers_yaml': controllers_yaml,
        }.items(),
    )

    # ── Controller spawner ────────────────────────────────────────────
    controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=[controller_name, '--controller-manager', cm_name,
                   '--controller-manager-timeout', '30'],
        output='screen',
    )

    # ── Assemble: logs + bringup + delayed spawner ────────────────────
    actions = build_wrapper_log_actions(
        p, use_rt, _CONFIG_PATH, controllers_yaml, controller_name, cm_name)
    actions += [
        franka_launch,
        TimerAction(period=10.0, actions=[controller_spawner]),
    ]

    # ── Legacy mode: optional Python velocity_blender ─────────────────
    if not use_rt:
        if p['start_blender'].lower() == 'true':
            prefix = ('/' + p['namespace']) if p['namespace'] else ''
            trk = (prefix + '/tracking_qdot'
                   if p['tracking_topic'] == '__auto__'
                   else p['tracking_topic'])
            avd = (prefix + '/avoidance_qdot'
                   if p['avoidance_topic'] == '__auto__'
                   else p['avoidance_topic'])
            cmd = (prefix + '/fr3_forward_velocity_controller/commands'
                   if p['output_command_topic'] == '__auto__'
                   else p['output_command_topic'])

            blender_node = Node(
                package='franka_experiments', executable='velocity_blender',
                name='velocity_blender', output='screen',
                parameters=[{
                    'command_topic': cmd, 'tracking_topic': trk,
                    'avoidance_topic': avd,
                    'rate_hz': float(p['blender_rate_hz']),
                    'qdot_max': float(p['blender_qdot_max']),
                    'watchdog_s': float(p['blender_watchdog_s']),
                }],
            )

            actions += [
                LogInfo(msg=['[wrapper] velocity_blender  : ENABLED']),
                LogInfo(msg=['[wrapper]   tracking  <- ', trk]),
                LogInfo(msg=['[wrapper]   avoidance <- ', avd]),
                LogInfo(msg=['[wrapper]   command   -> ', cmd]),
                LogInfo(msg=['[wrapper]   rate_hz=', p['blender_rate_hz'],
                             '  qdot_max=', p['blender_qdot_max'],
                             '  watchdog_s=', p['blender_watchdog_s']]),
                RegisterEventHandler(OnProcessExit(
                    target_action=controller_spawner,
                    on_exit=[
                        LogInfo(msg=['[wrapper] Controller spawner done '
                                     '— starting velocity_blender']),
                        blender_node,
                    ],
                )),
            ]
        else:
            actions.append(
                LogInfo(msg=['[wrapper] velocity_blender  : DISABLED '
                             '(start_blender:=false)']))

    # ── RViz2 (minimal, no MoveIt) ────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-f', 'fr3_link0'],
        output='screen',
    )
    actions.append(TimerAction(period=5.0, actions=[rviz_node]))

    # ── Camera pipeline (optional) ────────────────────────────────────
    if p['enable_camera'].lower() == 'true':
        # RealSense driver
        realsense_driver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch', 'rs_launch.py',
            ]).perform(context)),
        )
        actions.append(realsense_driver)

        # Image republisher (delayed so the driver is up)
        image_republisher = Node(
            package='franka_simulation',
            executable='image_publisher',
            name='image_republisher',
            output='screen',
        )
        actions.append(TimerAction(period=3.0, actions=[image_republisher]))

        # Human pose node — gated on /my_camera/image being published
        human_pose_node = Node(
            package='franka_simulation',
            executable='human_pose_node',
            name='human_pose_node',
            output='screen',
        )

        # One-shot process: exits as soon as one message arrives on the topic
        wait_for_my_camera_image = ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '--once', '/my_camera/image'],
            output='screen',
        )

        # Start human_pose_node only after the wait process succeeds
        start_human_pose_when_image_ready = RegisterEventHandler(
            OnProcessExit(
                target_action=wait_for_my_camera_image,
                on_exit=[
                    LogInfo(msg=['[wrapper] /my_camera/image detected '
                                 '— starting human_pose_node']),
                    human_pose_node,
                ],
            )
        )

        # Launch the wait probe after the image republisher, then register the gate
        actions.append(TimerAction(period=4.0,
                                   actions=[wait_for_my_camera_image]))
        actions.append(start_human_pose_when_image_ready)

        actions.append(
            LogInfo(msg=['[wrapper] Camera pipeline     : ENABLED']))
    else:
        actions.append(
            LogInfo(msg=['[wrapper] Camera pipeline     : DISABLED '
                         '(enable_camera:=false)']))

    # ── Camera extrinsics static TF (from YAML) ──────────────────────
    if p['enable_camera'].lower() == 'true' and p['do_calibration'].lower() != 'true':
        extrinsics_path = PathJoinSubstitution([
            FindPackageShare('franka_experiments'),
            'config', 'camera_extrinsics.yaml',
        ]).perform(context)
        with open(extrinsics_path, 'r') as f:
            ext = yaml.safe_load(f)
        t = ext['translation']
        r = ext['rotation']
        camera_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_extrinsics_tf',
            output='screen',
            arguments=[
                '--x',  str(t['x']),
                '--y',  str(t['y']),
                '--z',  str(t['z']),
                '--qx', str(r['x']),
                '--qy', str(r['y']),
                '--qz', str(r['z']),
                '--qw', str(r['w']),
                '--frame-id',       ext['parent_frame'],
                '--child-frame-id', ext['child_frame'],
            ],
        )
        actions.append(camera_tf_node)
        actions.append(
            LogInfo(msg=['[wrapper] Camera extrinsics TF : ENABLED '
                         '(', ext['parent_frame'], ' -> ', ext['child_frame'], ')']))
    elif p['enable_camera'].lower() == 'true':
        actions.append(
            LogInfo(msg=['[wrapper] Camera extrinsics TF : DISABLED '
                         '(do_calibration:=true)']))

    return actions


def generate_launch_description():
    return LaunchDescription(
        declare_robot_args(_DEFAULTS)
        + declare_rt_blender_args(_DEFAULTS)
        + declare_legacy_blender_args(_DEFAULTS)
        + [
            DeclareLaunchArgument(
                'enable_camera',
                default_value=_DEFAULTS.get('enable_camera', 'true'),
                description='Enable RealSense driver + image republisher'),
            DeclareLaunchArgument(
                'do_calibration',
                default_value=_DEFAULTS.get('do_calibration', 'false'),
                description='Calibration mode: skip YAML static TF so '
                            'calibration tools can publish their own'),
            OpaqueFunction(function=_launch_all),
        ]
    )
