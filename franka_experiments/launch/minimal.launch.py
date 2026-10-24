"""Minimal bringup: robot driver + RT velocity executor controller.

Lightweight launch for debugging — no RViz, no human_pose.
Optional camera pipeline and real_time_distance node are off by default.

* ``franka_bringup/franka.launch.py`` (robot driver + joint_state_broadcaster)
* ``rt_velocity_executor_controller`` spawner (delayed)
* Static TF  ``world → fr3_link0``  (identity)
* Scene camera D455 pipeline (enable_camera:=true)
* Wrist camera D405 + rqt_image_view (start_wrist_camera:=true)
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

    # Only the wrist D405, live in rqt_image_view, no robot:
    ros2 launch franka_experiments minimal.launch.py use_fake_hardware:=true \\
        enable_camera:=false start_wrist_camera:=true
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
    'enable_camera', 'camera_extrinsics_yaml', 'camera_delay_s', 'camera_align_depth',
    'camera_serial',
    'start_wrist_camera', 'wrist_camera_serial', 'wrist_camera_name',
    'wrist_color_profile', 'wrist_depth_profile', 'show_wrist_image',
    'wrist_enable_depth',
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

        # Driver default is align_depth.enable:=false — without this,
        # /camera/camera/aligned_depth_to_color/* never gets published.
        align_depth = _as_bool(p['camera_align_depth'])
        cam_args = {
            'camera_namespace': 'camera',
            'camera_name': 'camera',
            'align_depth.enable': 'true' if align_depth else 'false',
        }
        # serial_no is NOT optional with the wrist D405 also plugged in: a
        # driver launched without it opens whichever device librealsense
        # enumerates first, so /camera/camera/* silently becomes the D405.
        if str(p['camera_serial']).strip():
            cam_args['serial_no'] = str(p['camera_serial']).strip()
        realsense_driver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch', 'rs_launch.py',
            ]).perform(context)),
            launch_arguments=cam_args.items(),
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
                         '(delay=', str(cam_delay), 's, align_depth=',
                         str(align_depth), ')']))
    else:
        actions.append(
            LogInfo(msg='[minimal] Camera pipeline     : DISABLED '
                        '(enable_camera:=false)'))

    # ── Wrist camera (D405, eye-in-hand) + live view ─────────────────────
    # Independent of enable_camera: that one is the scene D455. Own namespace
    # AND name, so the topics are /<name>/<name>/... exactly like the scene
    # camera and like velocity_cbf_control_stack.launch.py already does.
    #
    # The D405 carries its colour stream on the DEPTH module, hence
    # depth_module.color_profile and not rgb_camera.color_profile.
    #
    # If the topics appear but stay at 0 Hz the camera is WEDGED, not slow: the
    # driver advertises every topic and then never publishes, which is exactly
    # how a bag ends up with /d405/d405/* recorded and message_count 0 (see
    # rosbag/handover_rosbag2). Neither initial_reset nor a USB unbind/rebind
    # clears it — both leave the camera powered. What clears it is REMOVING
    # POWER: unplug and replug, or, if the camera hangs off a hub with
    # per-port power switching,
    #
    #     uhubctl -l <hub> -p <port> -a cycle
    #
    # The USB link speed is a red herring on its own: this camera streams
    # colour+depth at 640x480x30 over a 480 Mbps link once it is unwedged.
    # What it will NOT do on 2.1 is 848x480x30 (the driver falls back to x10)
    # or three streams at once — keep infra off and depth modest there.
    if _as_bool(p['start_wrist_camera']):
        wrist_name = str(p['wrist_camera_name']).strip() or 'd405'
        wrist_depth = _as_bool(p['wrist_enable_depth'])
        wrist_args = {
            'camera_namespace':   wrist_name,
            'camera_name':        wrist_name,
            'enable_color':       'true',
            'enable_depth':       'true' if wrist_depth else 'false',
            'enable_infra1':      'false',
            'enable_infra2':      'false',
            'align_depth.enable': 'true' if wrist_depth else 'false',
        }
        if str(p['wrist_camera_serial']).strip():
            wrist_args['serial_no'] = str(p['wrist_camera_serial']).strip()
        if str(p['wrist_color_profile']).strip():
            wrist_args['depth_module.color_profile'] = \
                str(p['wrist_color_profile']).strip()
        if str(p['wrist_depth_profile']).strip():
            wrist_args['depth_module.depth_profile'] = \
                str(p['wrist_depth_profile']).strip()

        wrist_driver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch', 'rs_launch.py',
            ]).perform(context)),
            launch_arguments=wrist_args.items(),
        )
        actions.append(TimerAction(period=float(p['camera_delay_s']),
                                   actions=[wrist_driver]))
        actions.append(
            LogInfo(msg=['[minimal] Wrist camera        : ENABLED (',
                         wrist_name, '  serial=',
                         str(p['wrist_camera_serial']) or '<any>', ')']))

        if _as_bool(p['show_wrist_image']):
            wrist_topic = f'/{wrist_name}/{wrist_name}/color/image_raw'
            wrist_image_view = Node(
                package='rqt_image_view',
                executable='rqt_image_view',
                name='wrist_image_view',
                output='log',
                arguments=[wrist_topic],
            )
            # The driver needs a few seconds to open the sensor; rqt_image_view
            # resolves its topic once at startup, so starting it too early
            # leaves an empty combo box.
            actions.append(TimerAction(period=float(p['camera_delay_s']) + 5.0,
                                       actions=[wrist_image_view]))
            actions.append(
                LogInfo(msg=['[minimal] Wrist image view    : ENABLED on ',
                             wrist_topic]))
        else:
            actions.append(
                LogInfo(msg='[minimal] Wrist image view    : DISABLED '
                            '(show_wrist_image:=false)'))
    else:
        actions.append(
            LogInfo(msg='[minimal] Wrist camera        : DISABLED '
                        '(start_wrist_camera:=false)'))

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
                'camera_align_depth',
                default_value=str(_DEFAULTS.get('camera_align_depth', 'true')),
                description='RealSense align_depth.enable. Driver default is false, which '
                            'means /camera/camera/aligned_depth_to_color/* is never published'),
            DeclareLaunchArgument(
                'camera_serial',
                default_value=str(_DEFAULTS.get('camera_serial', '')),
                description='Scene camera (D455) serial_no. Keep the leading underscore, '
                            'or it is parsed as an int and librealsense answers "device '
                            'NOT found". Empty = whichever device enumerates first, which '
                            'is a coin flip with the D405 plugged in'),
            DeclareLaunchArgument(
                'start_wrist_camera',
                default_value=str(_DEFAULTS.get('start_wrist_camera', 'false')),
                description='Also start the eye-in-hand D405 (independent of enable_camera)'),
            DeclareLaunchArgument(
                'wrist_camera_serial',
                default_value=str(_DEFAULTS.get('wrist_camera_serial', '')),
                description='Wrist camera serial_no (leading underscore required)'),
            DeclareLaunchArgument(
                'wrist_camera_name',
                default_value=str(_DEFAULTS.get('wrist_camera_name', 'd405')),
                description='Wrist camera camera_namespace AND camera_name: topics become '
                            '/<name>/<name>/color/image_raw'),
            DeclareLaunchArgument(
                'wrist_color_profile',
                default_value=str(_DEFAULTS.get('wrist_color_profile', '')),
                description='Wrist camera depth_module.color_profile (the D405 colour '
                            'stream lives on the depth module). 848x480x30 needs a '
                            'SuperSpeed link; on USB 2.1 the driver falls back to x10'),
            DeclareLaunchArgument(
                'wrist_depth_profile',
                default_value=str(_DEFAULTS.get('wrist_depth_profile', '')),
                description='Wrist camera depth_module.depth_profile'),
            DeclareLaunchArgument(
                'wrist_enable_depth',
                default_value=str(_DEFAULTS.get('wrist_enable_depth', 'true')),
                description='Wrist camera depth + aligned depth. Turn OFF to stream colour '
                            'alone when the D405 sits on a USB 2 link, where three '
                            'simultaneous streams do not fit'),
            DeclareLaunchArgument(
                'show_wrist_image',
                default_value=str(_DEFAULTS.get('show_wrist_image', 'true')),
                description='Open rqt_image_view on the wrist colour stream. Only has an '
                            'effect with start_wrist_camera:=true'),
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
