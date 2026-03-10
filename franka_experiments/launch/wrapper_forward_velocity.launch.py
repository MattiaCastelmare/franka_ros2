"""Wrapper: bringup + RT velocity blender controller spawner.

Spawns ``rt_velocity_blender_controller`` — blending in C++ at 1 kHz.

Defaults are loaded from ``franka_experiments/config/launch_defaults.yaml``.
Robot-specific overrides come from ``franka_bringup/config/franka.config.yaml``.
Every argument is still overridable from the CLI.
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
    build_wrapper_log_actions,
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
    'namespace', 'use_fake_hardware', 'robot_ip', 'arm_id',
    'fake_sensor_commands', 'load_gripper', 'controllers_yaml',
    'qdot_max', 'alpha', 'max_accel', 'timeout_threshold_s', 'timeout_ramp_s',
    'gazebo', 'enable_interpolation', 'alpha_topic', 'tracking_topic',
    'avoidance_topic',
    'enable_camera',
    'do_calibration',
    'control_spawner_delay_s', 'start_rviz', 'rviz_delay_s',
    'camera_delay_s', 'start_human_pose', 'human_pose_delay_s',
]


def _as_bool(x: str) -> bool:
    """Interpret a launch-arg string as boolean (tolerant)."""
    return str(x).strip().lower() in ('1', 'true', 'yes', 'y', 'on')


def _launch_all(context):
    """Resolve all parameters, include bringup, spawn RT controller."""
    # ── Resolve every param once ──────────────────────────────────────
    p = {k: LaunchConfiguration(k).perform(context) for k in _ALL_PARAMS}
    use_fake = _as_bool(p['use_fake_hardware'])

    # ── Build RT param dict for YAML generation ───────────────────────
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
        p['controllers_yaml'], use_fake, rt_params)
    controller_name = 'rt_velocity_blender_controller'
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
        p, _CONFIG_PATH, controllers_yaml, controller_name, cm_name)
    actions += [
        franka_launch,
        TimerAction(period=float(p['control_spawner_delay_s']),
                    actions=[controller_spawner]),
    ]

    # ── RViz2 (minimal, no MoveIt) ────────────────────────────────────
    if _as_bool(p['start_rviz']):
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-f', 'fr3_link0'],
            output='screen',
        )
        actions.append(TimerAction(period=float(p['rviz_delay_s']),
                                   actions=[rviz_node]))

    # ── Camera pipeline (optional) ────────────────────────────────────
    if _as_bool(p['enable_camera']):
        cam_delay = float(p['camera_delay_s'])

        # RealSense driver
        realsense_driver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch', 'rs_launch.py',
            ]).perform(context)),
        )
        actions.append(TimerAction(period=cam_delay,
                                   actions=[realsense_driver]))

        # Image republisher (delayed so the driver is up)
        image_republisher = Node(
            package='franka_simulation',
            executable='image_publisher',
            name='image_republisher',
            output='screen',
        )
        actions.append(TimerAction(period=cam_delay + 3.0,
                                   actions=[image_republisher]))

        # Human pose node (timer-based, no process gating)
        if _as_bool(p['start_human_pose']):
            human_pose_node = Node(
                package='franka_simulation',
                executable='human_pose_node',
                name='human_pose_node',
                output='screen',
            )
            hp_delay = cam_delay + 4.0 + float(p['human_pose_delay_s'])
            actions.append(TimerAction(period=hp_delay,
                                       actions=[human_pose_node]))
            actions.append(
                LogInfo(msg=['[wrapper] human_pose_node    : ENABLED '
                             '(delay=', str(hp_delay), 's)']))
        else:
            actions.append(
                LogInfo(msg=['[wrapper] human_pose_node    : DISABLED '
                             '(start_human_pose:=false)']))

        actions.append(
            LogInfo(msg=['[wrapper] Camera pipeline     : ENABLED '
                         '(delay=', str(cam_delay), 's)']))
    else:
        actions.append(
            LogInfo(msg=['[wrapper] Camera pipeline     : DISABLED '
                         '(enable_camera:=false)']))

    # ── Camera extrinsics static TF (from YAML) ──────────────────────
    if _as_bool(p['enable_camera']) and not _as_bool(p['do_calibration']):
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
    elif _as_bool(p['enable_camera']):
        actions.append(
            LogInfo(msg=['[wrapper] Camera extrinsics TF : DISABLED '
                         '(do_calibration:=true)']))

    return actions


def generate_launch_description():
    return LaunchDescription(
        declare_robot_args(_DEFAULTS)
        + declare_rt_blender_args(_DEFAULTS)
        + [
            DeclareLaunchArgument(
                'enable_camera',
                default_value=str(_DEFAULTS.get('enable_camera', 'true')),
                description='Enable RealSense driver + image republisher'),
            DeclareLaunchArgument(
                'do_calibration',
                default_value=str(_DEFAULTS.get('do_calibration', 'false')),
                description='Calibration mode: skip YAML static TF so '
                            'calibration tools can publish their own'),
            DeclareLaunchArgument(
                'control_spawner_delay_s',
                default_value=str(_DEFAULTS.get('control_spawner_delay_s', '10.0')),
                description='Seconds before spawning the controller'),
            DeclareLaunchArgument(
                'start_rviz',
                default_value=str(_DEFAULTS.get('start_rviz', 'true')),
                description='Launch RViz2'),
            DeclareLaunchArgument(
                'rviz_delay_s',
                default_value=str(_DEFAULTS.get('rviz_delay_s', '5.0')),
                description='Seconds before launching RViz2'),
            DeclareLaunchArgument(
                'camera_delay_s',
                default_value=str(_DEFAULTS.get('camera_delay_s', '0.0')),
                description='Seconds before launching camera pipeline'),
            DeclareLaunchArgument(
                'start_human_pose',
                default_value=str(_DEFAULTS.get('start_human_pose', 'true')),
                description='Launch human_pose_node'),
            DeclareLaunchArgument(
                'human_pose_delay_s',
                default_value=str(_DEFAULTS.get('human_pose_delay_s', '0.0')),
                description='Extra seconds before human_pose_node '
                            '(added to camera_delay_s + 4)'),
            OpaqueFunction(function=_launch_all),
        ]
    )
