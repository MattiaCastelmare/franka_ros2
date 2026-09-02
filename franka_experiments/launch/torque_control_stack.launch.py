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
  t = 2 s                    cbf_safety_filter + qddot_to_torque  (pre-init before RT loop)
  t = 2 s                    real_time_distance  (pre-init: trimesh loading before RT loop)
  t = control_delay          rt_torque_controller spawner  (RT 1 kHz loop starts here)
  t = control_delay + 2 s    pentagon_qddot_commander  (motion generator)

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
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from franka_experiments.utils.config import (
    load_franka_config_defaults,
    load_launch_defaults,
)
from franka_experiments.utils.launch_support import (
    declare_robot_args,
    declare_rt_torque_args,
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
    'enable_camera', 'camera_extrinsics_yaml', 'camera_link_extrinsics_yaml', 'camera_delay_s',
    'start_real_time_distance',
    'start_experiment_logger', 'experiment_logger_delay_s',
    'start_move_group',
    'robot_config_yaml', 'torque_command_topic', 'controller_spawner_timeout_s',
    'torque_dynamics_delay_s', 'torque_rtd_delay_s', 'torque_commander_extra_delay_s',
    'torque_world_tf_delay_s', 'torque_camera_tf_delay_s',
    'torque_image_republisher_extra_delay_s',
    'torque_finger_pub_delay_s', 'torque_finger_pub_rate_hz',
]


def _as_bool(x: str) -> bool:
    return str(x).strip().lower() in ('1', 'true', 'yes', 'y', 'on')


def _launch_all(context):
    p = {k: LaunchConfiguration(k).perform(context) for k in _ALL_PARAMS}

    use_fake     = _as_bool(p['use_fake_hardware'])
    start_camera = _as_bool(p['enable_camera'])
    start_rtd    = _as_bool(p['start_real_time_distance'])

    control_delay   = float(p['control_spawner_delay_s'])
    dynamics_delay  = float(p['torque_dynamics_delay_s'])  # cbf + qddot_to_torque pre-init (before RT loop)
    rtd_delay       = float(p['torque_rtd_delay_s'])       # real_time_distance pre-init (trimesh before RT loop)
    commander_delay = control_delay + float(p['torque_commander_extra_delay_s'])

    # ── Build controller YAML for rt_torque_controller ────────────────────────
    # The controller listens on torque_cmd — the direct output of qddot_to_torque.
    rt_params = dict(
        is_real=not use_fake,
        arm_id=p['arm_id'],
        controller_type='torque',
        torque_command_topic=p['torque_command_topic'],
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
                   '--controller-manager-timeout',
                   str(int(float(p['controller_spawner_timeout_s'])))],
        output='screen',
    )

    # world → base static TF (identity).  robot_state_publisher already
    # publishes base → fr3_link0 (identity); publishing world → fr3_link0
    # here would give fr3_link0 two parents, orphaning 'base' and splitting
    # the TF tree — which breaks move_group's frame transforms (FK error -21).
    # world → base → fr3_link0 keeps a single connected tree.
    world_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_robot_base_tf',
        output='log',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'world',
            '--child-frame-id', 'base',
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
        TimerAction(period=float(p['torque_world_tf_delay_s']),
                    actions=[world_tf_node]),
        TimerAction(period=control_delay, actions=[controller_spawner]),
    ]

    # ── [MoveIt] move_group — planning services for the commander ────────────
    # pentagon_qddot_commander generates its pentagon via MoveIt's compute_fk /
    # compute_cartesian_path services, served by move_group.  It must run in
    # the robot namespace: the bringup's robot_state_publisher publishes TF on
    # /<ns>/tf and joint states on /<ns>/joint_states, and move_group needs
    # both (FK answers in fr3_link0 require the base→fr3_link0 transform).
    if _as_bool(p['start_move_group']):
        move_group_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('franka_fr3_moveit_config'), 'launch',
                'move_group.launch.py',
            ]).perform(context)),
            launch_arguments={
                'robot_ip':             p['robot_ip'],
                'namespace':            p['namespace'] or '/',
                'use_fake_hardware':    p['use_fake_hardware'],
                'fake_sensor_commands': p['fake_sensor_commands'],
                # hand:=true always: the commander's EE frame is fr3_hand_tcp,
                # same convention as the qddot_to_torque dynamics model.
                'load_gripper':         'true',
            }.items(),
        )
        actions.append(move_group_launch)
        actions.append(LogInfo(msg='[torque_stack] [MoveIt]          move_group ENABLED'))
    else:
        actions.append(LogInfo(msg='[torque_stack] [MoveIt]          move_group DISABLED — '
                                   'pentagon_qddot_commander will publish zeros until '
                                   'move_group is started manually'))

    # ── [MoveIt] Finger joint state publisher ─────────────────────────────────
    # move_group is launched with load_gripper:=true so its URDF includes
    # fr3_finger_joint1/2, but joint_state_broadcaster only publishes the 7 arm
    # joints. PlanningSceneMonitor warns "Missing fr3_finger_joint1" in a loop.
    # MoveIt's CurrentStateMonitor merges per-joint updates from multiple
    # messages on the same topic, so publishing the two finger joints separately
    # at zero is sufficient — no need to merge into a single 9-joint message.
    js_topic = f'/{p["namespace"]}/joint_states' if p['namespace'] else '/joint_states'
    finger_state_publisher = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', js_topic,
            'sensor_msgs/msg/JointState',
            '{name: [fr3_finger_joint1, fr3_finger_joint2],'
            ' position: [0.0, 0.0], velocity: [0.0, 0.0], effort: [0.0, 0.0]}',
            '--rate', str(float(p['torque_finger_pub_rate_hz'])),
        ],
        output='log',
        name='finger_state_publisher',
    )
    actions.append(TimerAction(period=float(p['torque_finger_pub_delay_s']),
                               actions=[finger_state_publisher]))
    actions.append(LogInfo(msg='[torque_stack] [MoveIt]          finger_state_publisher ENABLED'))

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
        actions.append(TimerAction(
            period=cam_delay + float(p['torque_image_republisher_extra_delay_s']),
            actions=[image_republisher]))

        # TF publisher: base → camera_link (connects the RealSense TF sub-tree
        # to the robot tree). Uses camera_link_extrinsics.yaml, NOT the same
        # file as real_time_distance — those serve different purposes:
        #   camera_extrinsics.yaml     → base→camera_color_optical_frame (calibration
        #                                 used by real_time_distance for depth projection)
        #   camera_link_extrinsics.yaml → base→camera_link (used here for TF tree
        #                                 so MoveIt can transform from any camera frame)
        link_ext_path = p['camera_link_extrinsics_yaml']
        with open(link_ext_path, 'r') as f:
            link_ext = yaml.safe_load(f)
        t_link = link_ext['translation']
        r_link = link_ext['rotation']
        camera_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_extrinsics_tf',
            output='log',
            arguments=[
                '--x',  str(t_link['x']), '--y',  str(t_link['y']), '--z',  str(t_link['z']),
                '--qx', str(r_link['x']), '--qy', str(r_link['y']),
                '--qz', str(r_link['z']), '--qw', str(r_link['w']),
                '--frame-id', link_ext['parent_frame'],
                '--child-frame-id', link_ext['child_frame'],
            ],
        )
        actions.append(TimerAction(period=float(p['torque_camera_tf_delay_s']),
                                   actions=[camera_tf_node]))
        actions.append(LogInfo(msg=['[torque_stack] [Perception]      camera ENABLED '
                                    '(delay=', str(cam_delay), 's)']))
    else:
        actions.append(LogInfo(msg='[torque_stack] [Perception]      camera DISABLED'))

    # ── [Distance estimation] real_time_distance ──────────────────────────────
    if start_rtd:
        rtd_config = p['robot_config_yaml']
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
    # qddot_to_torque subscribes directly to qddot_safe (the CBF-filtered
    # acceleration) and converts it to torque — no remap needed.
    qddot_to_torque_node = Node(
        package='franka_experiments',
        executable='qddot_to_torque',
        name='qddot_to_torque',
        output='screen',
    )
    actions.append(TimerAction(period=dynamics_delay,
                               actions=[cbf_node, qddot_to_torque_node]))
    actions.append(LogInfo(msg=['[torque_stack] [CBF filter]      cbf_safety_filter + qddot_to_torque'
                                ' (delay=', str(dynamics_delay), 's)']))

    # ── [Motion generation] pentagon_qddot_commander ──────────────────────────
    # Runs in the robot namespace so its relative MoveIt service clients
    # (compute_fk, compute_cartesian_path) resolve to move_group above.
    # Its topics are absolute (/NS_1/…) and unaffected by the namespace.
    commander_node = Node(
        package='franka_experiments',
        executable='pentagon_qddot_commander',
        name='pentagon_qddot_commander',
        namespace=p['namespace'],
        output='screen',
        # Path geometry (centre / shape / radius) is NOT set here: the node
        # reads it from config/fr3_control.yaml (params: path_center_xyz,
        # path_type, path_radius) as its declare_parameter defaults. Launch
        # files carry wiring, not tunables.

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
                description='Path to camera_extrinsics.yaml (base→camera_color_optical_frame, used by real_time_distance)'),
            DeclareLaunchArgument(
                'camera_link_extrinsics_yaml',
                default_value=PathJoinSubstitution([
                    FindPackageShare('franka_experiments'),
                    'config', 'camera_link_extrinsics.yaml',
                ]),
                description='Path to camera_link_extrinsics.yaml (base→camera_link, used by static TF publisher)'),
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
                default_value=str(_DEFAULTS.get('start_experiment_logger', 'true')),
                description='Start experiment logger automatically'),
            DeclareLaunchArgument(
                'experiment_logger_delay_s',
                default_value=str(_DEFAULTS.get('experiment_logger_delay_s', '2.0')),
                description='Seconds before launching experiment_logger'),
            DeclareLaunchArgument(
                'start_move_group',
                default_value=str(_DEFAULTS.get('start_move_group', 'true')),
                description='Start move_group (MoveIt planning services used by '
                            'pentagon_qddot_commander)'),

            # ── Wiring / sequencing (defaults in config/launch_defaults.yaml) ──
            DeclareLaunchArgument(
                'robot_config_yaml',
                default_value=PathJoinSubstitution([
                    FindPackageShare('franka_experiments'),
                    'config', 'fr3_complete.yaml',
                ]),
                description='Path to fr3_complete.yaml (robot/mesh/distance config '
                            'loaded by real_time_distance)'),
            DeclareLaunchArgument(
                'torque_command_topic',
                default_value=str(_DEFAULTS.get('torque_command_topic', 'torque_cmd')),
                description='Topic rt_torque_controller reads tau from (relative to '
                            'the controller_manager namespace)'),
            DeclareLaunchArgument(
                'controller_spawner_timeout_s',
                default_value=str(_DEFAULTS.get('controller_spawner_timeout_s', '30.0')),
                description='[s] controller_manager spawner timeout'),
            DeclareLaunchArgument(
                'torque_dynamics_delay_s',
                default_value=str(_DEFAULTS.get('torque_dynamics_delay_s', '2.0')),
                description='[s] delay before cbf_safety_filter + qddot_to_torque'),
            DeclareLaunchArgument(
                'torque_rtd_delay_s',
                default_value=str(_DEFAULTS.get('torque_rtd_delay_s', '2.0')),
                description='[s] delay before real_time_distance'),
            DeclareLaunchArgument(
                'torque_commander_extra_delay_s',
                default_value=str(_DEFAULTS.get('torque_commander_extra_delay_s', '2.0')),
                description='[s] added to control_spawner_delay_s before the commander'),
            DeclareLaunchArgument(
                'torque_world_tf_delay_s',
                default_value=str(_DEFAULTS.get('torque_world_tf_delay_s', '1.0')),
                description='[s] delay before the world -> base static TF'),
            DeclareLaunchArgument(
                'torque_camera_tf_delay_s',
                default_value=str(_DEFAULTS.get('torque_camera_tf_delay_s', '1.0')),
                description='[s] delay before the base -> camera_link static TF'),
            DeclareLaunchArgument(
                'torque_image_republisher_extra_delay_s',
                default_value=str(_DEFAULTS.get(
                    'torque_image_republisher_extra_delay_s', '3.0')),
                description='[s] added to camera_delay_s before the image republisher'),
            DeclareLaunchArgument(
                'torque_finger_pub_delay_s',
                default_value=str(_DEFAULTS.get('torque_finger_pub_delay_s', '2.0')),
                description='[s] delay before the MoveIt finger joint-state publisher'),
            DeclareLaunchArgument(
                'torque_finger_pub_rate_hz',
                default_value=str(_DEFAULTS.get('torque_finger_pub_rate_hz', '10.0')),
                description='[Hz] MoveIt finger joint-state publisher rate'),

            OpaqueFunction(function=_launch_all),
        ]
    )
