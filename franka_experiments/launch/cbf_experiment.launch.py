"""CBF pipeline bringup — two-phase architecture.

Pipeline (both phases):
  pentagon_qddot_commander  →  /NS_1/qddot_nom
  cbf_safety_filter         →  /NS_1/torque_cmd  (τ = M·q̈ + C·q̇)
  rt_torque_controller      →  hardware          (adds gravity g(q))

Phase 1  (bypass_cbf:=true, default)
  cbf_safety_filter runs in BYPASS mode: no QP, no distance required.
  Useful to verify smooth trajectory execution before enabling CBF.

Phase 2  (bypass_cbf:=false)
  cbf_safety_filter runs the full HOCBF QP at 200 Hz.
  real_time_distance must be available → camera + enable_camera:=true.

Startup sequence (delays relative to launch time):
  t=0                franka bringup (joint_state_broadcaster)
  t=control_delay    rt_torque_controller spawner
  t=control_delay+4  cbf_safety_filter
  t=control_delay+6  pentagon_qddot_commander
  (Phase 2 only: real_time_distance starts once rt_torque_controller is active)
"""

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
    load_franka_config_defaults,
    load_launch_defaults,
    resolve_controller_manager_name,
)

_LAUNCH_DEFAULTS, _ = load_launch_defaults()
_BRINGUP_DEFAULTS, _ = load_franka_config_defaults()
_DEFAULTS = {**_LAUNCH_DEFAULTS, **_BRINGUP_DEFAULTS}

_CBF_PARAMS = [
    'arm_id', 'robot_ip', 'namespace', 'use_fake_hardware',
    'fake_sensor_commands', 'load_gripper',
    'enable_camera', 'camera_extrinsics_yaml',
    'control_spawner_delay_s', 'bypass_cbf',
]


def _as_bool(x: str) -> bool:
    return str(x).strip().lower() in ('1', 'true', 'yes', 'y', 'on')


def _launch_cbf(context):
    p = {k: LaunchConfiguration(k).perform(context) for k in _CBF_PARAMS}

    cbf_delay        = float(p['control_spawner_delay_s'])
    cbf_filter_delay = cbf_delay + 6.0   # filter after controller is active
    pentagon_delay   = cbf_delay + 10.0  # commander after filter + extra margin
    bypass_cbf       = _as_bool(p['bypass_cbf'])
    cm_name          = resolve_controller_manager_name(p['namespace'])

    # Phase 2 needs real_time_distance; Phase 1 does not
    start_rtd = 'false' if bypass_cbf else 'true'

    # ── Include minimal.launch.py — spawns rt_torque_controller ──────────────
    minimal_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('franka_experiments'), 'launch', 'minimal.launch.py',
        ]).perform(context)),
        launch_arguments={
            'arm_id':                   p['arm_id'],
            'robot_ip':                 p['robot_ip'],
            'namespace':                p['namespace'],
            'use_fake_hardware':        p['use_fake_hardware'],
            'fake_sensor_commands':     p['fake_sensor_commands'],
            'load_gripper':             p['load_gripper'],
            'enable_camera':            p['enable_camera'],
            'camera_extrinsics_yaml':   p['camera_extrinsics_yaml'],
            'control_spawner_delay_s':  p['control_spawner_delay_s'],
            'use_torque_controller':    'true',
            'torque_command_topic':     'torque_cmd',
            'start_real_time_distance': start_rtd,
        }.items(),
    )

    # ── cbf_safety_filter ─────────────────────────────────────────────────────
    cbf_filter = Node(
        package='franka_experiments',
        executable='cbf_safety_filter',
        name='cbf_safety_filter',
        output='screen',
        parameters=[{
            'bypass_cbf':       bypass_cbf,
            'torque_out_topic': '/NS_1/torque_cmd',
        }],
    )

    # ── pentagon_qddot_commander ──────────────────────────────────────────────
    # qddot_safe_topic is repurposed here to route nominal accelerations to the
    # CBF filter input (/NS_1/qddot_nom) instead of the old cbf_torque_controller.
    pentagon = Node(
        package='franka_experiments',
        executable='pentagon_qddot_commander',
        name='pentagon_qddot_commander',
        output='screen',
        parameters=[{
            'qddot_safe_topic':  '/NS_1/qddot_nom',
            'q_des_topic':       '/NS_1/q_des_state',
            'joint_state_topic': '/NS_1/joint_states',
        }],
    )

    phase_str = 'Phase 1 — BYPASS (no CBF, no distances)' if bypass_cbf \
        else 'Phase 2 — CBF QP active (distances required)'
    return [
        LogInfo(msg=['[cbf_exp] ', phase_str]),
        LogInfo(msg=['[cbf_exp] arm=', p['arm_id'],
                     '  ip=', p['robot_ip'],
                     '  fake=', p['use_fake_hardware'],
                     '  ns=', p['namespace'] or '<none>']),
        LogInfo(msg='[cbf_exp] pipeline: pentagon → qddot_nom → cbf_filter'
                    ' → torque_cmd → rt_torque_controller → hardware'),
        minimal_launch,
        LogInfo(msg=['[cbf_exp] cbf_safety_filter  delay=', str(cbf_filter_delay), 's']),
        TimerAction(period=cbf_filter_delay, actions=[cbf_filter]),
        LogInfo(msg=['[cbf_exp] pentagon            delay=', str(pentagon_delay), 's']),
        TimerAction(period=pentagon_delay, actions=[pentagon]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'arm_id',
            default_value=_DEFAULTS.get('arm_id', 'fr3'),
            description='Robot arm model identifier'),
        DeclareLaunchArgument(
            'robot_ip',
            default_value=_DEFAULTS.get('robot_ip', '192.168.2.10'),
            description='IP address of the robot'),
        DeclareLaunchArgument(
            'namespace',
            default_value=_DEFAULTS.get('namespace', ''),
            description='Namespace for the robot'),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value=_DEFAULTS.get('use_fake_hardware', 'false'),
            description='Use fake (mock) hardware'),
        DeclareLaunchArgument(
            'fake_sensor_commands',
            default_value=_DEFAULTS.get('fake_sensor_commands', 'false'),
            description='Fake sensor commands'),
        DeclareLaunchArgument(
            'load_gripper',
            default_value=_DEFAULTS.get('load_gripper', 'false'),
            description='Load Franka Gripper'),
        DeclareLaunchArgument(
            'enable_camera',
            default_value=_DEFAULTS.get('enable_camera', 'true'),
            description='Enable RealSense driver (needed for Phase 2)'),
        DeclareLaunchArgument(
            'camera_extrinsics_yaml',
            default_value=PathJoinSubstitution([
                FindPackageShare('franka_experiments'),
                'config', 'camera_extrinsics.yaml',
            ]),
            description='Path to camera_extrinsics.yaml'),
        DeclareLaunchArgument(
            'control_spawner_delay_s',
            default_value=_DEFAULTS.get('control_spawner_delay_s', '10.0'),
            description='Seconds before spawning rt_torque_controller'),
        DeclareLaunchArgument(
            'bypass_cbf',
            default_value='true',
            description='Phase 1: bypass CBF QP (no distances needed). '
                        'Set false for Phase 2 with full CBF safety filter.'),
        OpaqueFunction(function=_launch_cbf),
    ])
