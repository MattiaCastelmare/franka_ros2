"""Minimal bringup: robot driver + RT velocity executor controller.

Lightweight launch for debugging — no RViz, no human_pose.
Optional camera pipeline and real_time_distance node are off by default.

* ``franka_bringup/franka.launch.py`` (robot driver + joint_state_broadcaster)
* ``rt_velocity_executor_controller`` spawner (delayed)
* Static TF  ``world → fr3_link0``  (identity)
* Camera pipeline (enable_camera:=true)
* real_time_distance node (start_real_time_distance:=true)

Defaults are loaded from ``franka_experiments/config/launch_defaults.yaml``
and robot-specific overrides from ``franka_bringup/config/franka.config.yaml``.

Examples
--------
.. code-block:: bash

    # Fake hardware (no physical robot required):
    ros2 launch franka_experiments minimal.launch.py use_fake_hardware:=true

    # Real robot (IP and arm_id taken from franka.config.yaml):
    ros2 launch franka_experiments minimal.launch.py

    # With camera and real-time distance:
    ros2 launch franka_experiments minimal.launch.py enable_camera:=true start_real_time_distance:=true
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

# ── Load defaults (single source of truth) ───────────────────────────────────
_LAUNCH_DEFAULTS, _LAUNCH_DEFAULTS_PATH = load_launch_defaults()
_BRINGUP_DEFAULTS, _CONFIG_PATH = load_franka_config_defaults()
_DEFAULTS = {**_LAUNCH_DEFAULTS, **_BRINGUP_DEFAULTS}

_ALL_PARAMS = [
    'namespace', 'use_fake_hardware', 'robot_ip', 'arm_id',
    'fake_sensor_commands', 'load_gripper', 'controllers_yaml',
    'qdot_max', 'max_accel', 'timeout_threshold_s', 'timeout_ramp_s',
    'gazebo', 'enable_interpolation', 'command_topic',
    'use_torque_controller', 'torque_command_topic', 'lpf_alpha', 'tau_max_scale',
    'control_spawner_delay_s',
    'enable_camera', 'camera_extrinsics_yaml', 'camera_delay_s',
    'start_real_time_distance', 'real_time_distance_delay_s',
    'start_experiment_logger',
    'experiment_logger_delay_s',
]


def _as_bool(x: str) -> bool:
    """Interpret a launch-arg string as boolean (tolerant)."""
    return str(x).strip().lower() in ('1', 'true', 'yes', 'y', 'on')


def _launch_all(context):
    """Resolve all parameters, include bringup, spawn RT controller."""
    p = {k: LaunchConfiguration(k).perform(context) for k in _ALL_PARAMS}
    use_fake = _as_bool(p['use_fake_hardware'])

    # ── Build RT param dict for YAML generation ───────────────────────────
    use_torque_val = p['use_torque_controller'].strip().lower()
    use_torque   = use_torque_val in ('1', 'true', 'yes', 'y', 'on')
    cbf_pipeline = (use_torque_val == 'cbf')

    if cbf_pipeline:
        controller_name = 'cbf_torque_controller'
        rt_params = dict(
            is_real=not use_fake, arm_id=p['arm_id'],
            qdot_max='0.0', command_topic='none',
            max_accel='0.0', timeout_threshold_s='0.0', timeout_ramp_s='0.0',
            gazebo=p['gazebo'], enable_interpolation='false',
            controller_type='cbf',
        )
    elif use_torque:
        controller_name = 'rt_torque_controller'
        rt_params = dict(
            is_real=not use_fake, arm_id=p['arm_id'],
            qdot_max=p['qdot_max'],
            command_topic=p['command_topic'],
            max_accel=p['max_accel'],
            timeout_threshold_s=p['timeout_threshold_s'],
            timeout_ramp_s=p['timeout_ramp_s'],
            gazebo=p['gazebo'],
            enable_interpolation=p['enable_interpolation'],
            controller_type='torque',
            torque_command_topic=p['torque_command_topic'],
            lpf_alpha=p['lpf_alpha'],
            tau_max_scale=p['tau_max_scale'],
        )
    else:
        controller_name = 'rt_velocity_executor_controller'
        rt_params = dict(
            is_real=not use_fake, arm_id=p['arm_id'],
            qdot_max=p['qdot_max'],
            command_topic=p['command_topic'],
            max_accel=p['max_accel'],
            timeout_threshold_s=p['timeout_threshold_s'],
            timeout_ramp_s=p['timeout_ramp_s'],
            gazebo=p['gazebo'],
            enable_interpolation=p['enable_interpolation'],
        )

    controllers_yaml = pick_controllers_yaml(
        p['controllers_yaml'], use_fake, rt_params)
    cm_name = resolve_controller_manager_name(p['namespace'])

    # ── Include franka bringup ────────────────────────────────────────────
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

    # ── world → fr3_link0 static TF (identity) ───────────────────────────
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

    # ── Single startup log (essentials only) ─────────────────────────────
    ns_display = p['namespace'] or '<none>'
    startup_log = LogInfo(msg=[
        '[minimal] arm_id=', p['arm_id'],
        '  ip=', p['robot_ip'],
        '  fake=', p['use_fake_hardware'],
        '  ns=', ns_display,
        '  yaml=', controllers_yaml,
    ])

    actions = [
        startup_log,
        franka_launch,
        TimerAction(period=1.0, actions=[world_tf_node]),
    ]
    if controller_name is not None:
        controller_spawner = Node(
            package='controller_manager', executable='spawner',
            arguments=[controller_name, '--controller-manager', cm_name,
                       '--controller-manager-timeout', '30'],
            output='screen',
        )
        actions.append(TimerAction(period=float(p['control_spawner_delay_s']),
                                   actions=[controller_spawner]))

    # ── Camera pipeline (optional) ────────────────────────────────────────
    if _as_bool(p['enable_camera']):
        cam_delay = float(p['camera_delay_s'])

        realsense_driver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch', 'rs_launch.py',
            ]).perform(context)),
        )
        actions.append(TimerAction(period=cam_delay,
                                   actions=[realsense_driver]))

        image_republisher = Node(
            package='franka_simulation',
            executable='image_publisher',
            name='image_republisher',
            output='log',
        )
        actions.append(TimerAction(period=cam_delay + 3.0,
                                   actions=[image_republisher]))

        extrinsics_path = p['camera_extrinsics_yaml']
        with open(extrinsics_path, 'r') as f:
            ext = yaml.safe_load(f)
        t = ext['translation']
        r = ext['rotation']
        camera_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_extrinsics_tf',
            output='log',
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
        actions.append(TimerAction(period=1.0, actions=[camera_tf_node]))
        actions.append(
            LogInfo(msg=['[minimal] Camera pipeline     : ENABLED '
                         '(delay=', str(cam_delay), 's)']))
    else:
        actions.append(
            LogInfo(msg='[minimal] Camera pipeline     : DISABLED '
                        '(enable_camera:=false)'))

    # ── Real-time distance node (optional) ───────────────────────────────
    if _as_bool(p['start_real_time_distance']):
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
                'robot_config_path': rtd_config,
                'camera_extrinsics_path': p['camera_extrinsics_yaml'],
            }],
        )

        if controller_name is not None:
            # Wait until the controller reports 'active' before starting rtd.
            poll_proc = ExecuteProcess(
                name='wait_controller_active',
                cmd=[
                    'bash', '-c',
                    f'until ros2 control list_controllers'
                    f' --controller-manager {cm_name} 2>/dev/null'
                    f' | grep -q "{controller_name}.*active";'
                    f' do sleep 0.5; done',
                ],
                output='log',
            )
            actions.append(poll_proc)
            actions.append(RegisterEventHandler(
                OnProcessExit(target_action=poll_proc,
                              on_exit=[real_time_distance_node])))
            actions.append(
                LogInfo(msg=['[minimal] real_time_distance  : ENABLED '
                             '(waits for ', controller_name, ' active)']))
        else:
            # No controller to wait for — fall back to fixed delay.
            _cam_last = (float(p['camera_delay_s']) + 3.0
                         if _as_bool(p['enable_camera']) else 0.0)
            rtd_delay = max(float(p['control_spawner_delay_s']), _cam_last) + 2.0
            actions.append(TimerAction(period=rtd_delay,
                                       actions=[real_time_distance_node]))
            actions.append(
                LogInfo(msg=['[minimal] real_time_distance  : ENABLED '
                             '(delay=', str(rtd_delay), 's — no controller)']))
    else:
        actions.append(
            LogInfo(msg='[minimal] real_time_distance  : DISABLED '
                        '(start_real_time_distance:=false)'))
    # ── Experiment logger ───────────────────────────────────────────────
    if _as_bool(p['start_experiment_logger']):
        experiment_logger_node = Node(
            package='franka_experiments',
            executable='experiment_logger',
            name='experiment_logger',
            output='screen',
        )

        actions.append(TimerAction(
            period=float(p['experiment_logger_delay_s']),
            actions=[experiment_logger_node]
        ))

        actions.append(LogInfo(msg='[minimal] experiment_logger : ENABLED'))
    else:
        actions.append(LogInfo(msg='[minimal] experiment_logger : DISABLED'))

    return actions


def generate_launch_description():
    return LaunchDescription(
        declare_robot_args(_DEFAULTS)
        + declare_rt_blender_args(_DEFAULTS)
        + [
            DeclareLaunchArgument(
                'use_torque_controller',
                default_value=str(_DEFAULTS.get('use_torque_controller', 'true')),
                description='Use rt_torque_controller instead of rt_velocity_executor_controller'),
            DeclareLaunchArgument(
                'torque_command_topic',
                default_value=str(_DEFAULTS.get('torque_command_topic', 'torque_cmd')),
                description='[torque] Topic for torque commands (Float64MultiArray, size=7)'),
            DeclareLaunchArgument(
                'lpf_alpha',
                default_value=str(_DEFAULTS.get('lpf_alpha', '1.0')),
                description='[torque] Low-pass filter alpha for tau_cmd. 1.0 = off'),
            DeclareLaunchArgument(
                'tau_max_scale',
                default_value=str(_DEFAULTS.get('tau_max_scale', '1.0')),
                description='[torque] Scale factor applied to per-joint torque limits'),
            DeclareLaunchArgument(
                'control_spawner_delay_s',
                default_value=str(_DEFAULTS.get('control_spawner_delay_s', '10.0')),
                description='Seconds before spawning the RT controller'),
            DeclareLaunchArgument(
                'enable_camera',
                default_value=str(_DEFAULTS.get('enable_camera', 'false')),
                description='Enable RealSense driver + image republisher'),
            DeclareLaunchArgument(
                'camera_extrinsics_yaml',
                default_value=PathJoinSubstitution([
                    FindPackageShare('franka_experiments'),
                    'config', 'camera_extrinsics.yaml',
                ]),
                description='Path to camera_extrinsics.yaml '
                            '(parent_frame, child_frame, translation, rotation)'),
            DeclareLaunchArgument(
                'camera_delay_s',
                default_value=str(_DEFAULTS.get('camera_delay_s', '0.0')),
                description='Seconds before launching camera pipeline'),
            DeclareLaunchArgument(
                'start_real_time_distance',
                default_value=str(_DEFAULTS.get('start_real_time_distance', 'false')),
                description='Launch real_time_distance node'),
            DeclareLaunchArgument(
                'real_time_distance_delay_s',
                default_value=str(_DEFAULTS.get('real_time_distance_delay_s', '8.0')),
                description='Seconds before launching real_time_distance node'),
            DeclareLaunchArgument(
                'start_experiment_logger',
                default_value='true',
                description='Start experiment logger automatically'
            ),
            DeclareLaunchArgument(
                'experiment_logger_delay_s',
                default_value='2.0',
                description='Seconds before launching experiment_logger'
            ),
            OpaqueFunction(function=_launch_all),
        ]
    )
