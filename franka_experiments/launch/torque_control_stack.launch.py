"""Torque control stack — FR3 robot (acceleration-space pipeline).

Complete launch for the joint-torque control pipeline.

Pipeline:
  [Perception]     RealSense camera driver
  [Distance est.]  real_time_distance       →  /cbf/per_link_distances
  [Motion gen.]    pentagon_qddot_commander →  /NS_1/qddot_nom   (q̈_nom)
                   or rl_policy_commander   →  /NS_1/qddot_nom   (motion_source:=rl)
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
  t = control_delay + 2 s    motion generator (pentagon_qddot_commander | rl_policy_commander)

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

    # Safe-RL policy instead of the pentagon path (see franka_sim_to_real_roadmap.md).
    # A cautious first run on real hardware: derate the policy to 30% authority.
    ros2 launch franka_experiments torque_control_stack.launch.py \\
        motion_source:=rl start_move_group:=false rl_action_scale:=0.3

    # ── Obstacle PREDICTION: avoid proportionally to the obstacle's speed ──
    # Tracking publishes the 3D velocity; the CBF consumes it instead of the
    # scalar residual. Each switch alone is a no-op, so both are needed.
    ros2 launch franka_experiments torque_control_stack.launch.py \\
        obstacle_tracking:=true obstacle_velocity_source:=tracker

    # ── Everything on: prediction + uncertainty margin + lateral evasion ──
    # With lateral_evasion the arm steps SIDEWAYS out of the swept volume when
    # its own acceleration box says it cannot null the closing rate in time.
    ros2 launch franka_experiments torque_control_stack.launch.py \\
        obstacle_tracking:=true obstacle_velocity_source:=tracker \\
        lateral_evasion:=true uncertainty_margin:=true

    # ── END-TO-END SIMULATION, no robot and no camera ──────────────────────
    # Fake hardware, depth replayed from a bag, and a synthetic sphere shuttled
    # through the workspace so the whole avoidance chain actually fires. The
    # arm moves away from something that is not there -- never run this with a
    # person nearby.
    ros2 launch franka_experiments torque_control_stack.launch.py \\
        use_fake_hardware:=true enable_camera:=false \\
        depth_bag:=$(ros2 pkg prefix franka_experiments)/../../src/franka_experiments/rosbag/arm_complex \\
        sim_obstacle:=true obstacle_tracking:=true \\
        obstacle_velocity_source:=tracker lateral_evasion:=true
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

# NumPy ships its own OpenBLAS, whose worker pool BUSY-WAITS instead of
# sleeping: on this 24-core box pentagon_qddot_commander was measured at 588%
# CPU with six threads pegged at ~95% while the node's own thread used 10.7%.
# Every matrix in this stack is 7x7 or smaller, where a thread pool is pure
# overhead, and the spinners sit on the P-cores next to the SCHED_FIFO thread
# of ros2_control_node -- cache thrash and thermal throttling on the one core
# that must not miss a 1 ms deadline.  One thread per node is both faster here
# and quieter for the RT loop.
_SINGLE_THREAD_BLAS = {
    'OPENBLAS_NUM_THREADS': '1',
    'OMP_NUM_THREADS': '1',
    'MKL_NUM_THREADS': '1',
    'NUMEXPR_NUM_THREADS': '1',
}

_ALL_PARAMS = [
    'namespace', 'use_fake_hardware', 'robot_ip', 'arm_id',
    'fake_sensor_commands', 'load_gripper', 'controllers_yaml',
    'gazebo', 'lpf_alpha', 'tau_max_scale',
    'control_spawner_delay_s', 'rt_pin_cpu',
    'enable_camera', 'camera_extrinsics_yaml', 'camera_link_extrinsics_yaml', 'camera_delay_s',
    'camera_depth_profile',
    'start_real_time_distance',
    'obstacle_tracking', 'obstacle_velocity_source', 'lateral_evasion', 'outrun_evasion',
    'livelock_escape', 'latency_compensation',
    'zone_ladder', 'obstacle_velocity_normal_guard', 'obstacle_identity_guard',
    'uncertainty_margin', 'sim_obstacle', 'depth_bag',
    'multi_obstacle_k', 'vobs_in_hdot', 'velocity_standoff',
    'start_experiment_logger', 'experiment_logger_delay_s',
    'start_move_group',
    'motion_source', 'rl_onnx_model', 'rl_sim_config', 'rl_target_xyz',
    'rl_target_sequence', 'rl_action_scale',
    'robot_config_yaml', 'torque_command_topic', 'controller_spawner_timeout_s',
    'torque_dynamics_delay_s', 'torque_rtd_delay_s', 'torque_commander_extra_delay_s',
    'torque_world_tf_delay_s', 'torque_camera_tf_delay_s',
    'torque_image_republisher_extra_delay_s',
    'torque_finger_pub_delay_s', 'torque_finger_pub_rate_hz',
]


def _as_bool(x: str) -> bool:
    return str(x).strip().lower() in ('1', 'true', 'yes', 'y', 'on')


def _as_float_list(x: str):
    """Parse ``'[0.4, 0.0, 0.45]'`` / ``'0.4,0.0,0.45'`` → list[float] ([] if empty)."""
    s = str(x).strip().strip('[]')
    if not s:
        return []
    return [float(v) for v in s.replace(';', ',').split(',') if v.strip()]


def _rtd_config_with_overrides(path: str, *, tracking: bool,
                               sim_obstacle: bool) -> str:
    """Return ``path``, or a copy of it with the two perception switches forced.

    ``real_time_distance`` reads its perception configuration from a YAML rather
    than from ROS parameters — the blocks are nested dictionaries that
    ``utils.params`` cannot express. That is the right shape for the file and
    the wrong shape for a launch argument, so this bridges the two.

    Returns the ORIGINAL path when neither switch is set, so the common case
    touches no filesystem and the node reads exactly the installed file.
    """
    if not (tracking or sim_obstacle):
        return path
    import os
    import tempfile
    with open(path) as f:
        cfg = yaml.safe_load(f)
    if tracking:
        cfg.setdefault('tracking', {})['enabled'] = True
    if sim_obstacle:
        cfg.setdefault('sim_obstacle', {})['enabled'] = True
    out = os.path.join(tempfile.gettempdir(),
                       f'fr3_complete_launch_{os.getpid()}.yaml')
    with open(out, 'w') as f:
        yaml.safe_dump(cfg, f, sort_keys=False)
    return out


def _depth_bag_player(bag: str, cfg_path: str):
    """``ros2 bag play`` restricted to the DEPTH stream, or ``None``.

    Only the depth image and its camera_info are played, and both are remapped
    onto the topics the config names. TF and joint states are deliberately NOT
    replayed: they must come from the robot that is actually running, or the
    mask would be built for one pose while the arm is in another — which is not
    a degraded measurement, it is a wrong one, and it is wrong in the direction
    of thinking the workspace is emptier than it is.

    ``--loop`` because the point is to keep the depth stream alive for as long
    as the stack runs, not to reproduce one recording end to end.
    """
    if not bag:
        return None
    with open(cfg_path) as f:
        topics = (yaml.safe_load(f) or {}).get('topics', {}) or {}
    depth = topics.get('depth_image', '/camera/camera/depth/image_rect_raw')
    info = topics.get('depth_camera_info', '/camera/camera/depth/camera_info')
    # The bags in this repo carry the ALIGNED depth stream under its own name.
    src_depth = '/camera/camera/aligned_depth_to_color/image_raw'
    src_info = '/camera/camera/aligned_depth_to_color/camera_info'
    return ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag, '--loop',
             '--topics', src_depth, src_info,
             '--remap', f'{src_depth}:={depth}', f'{src_info}:={info}'],
        output='screen')


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

    # ── [RT] pin the control loop onto the isolated core ─────────────────────
    # isolcpus=2,3 only EMPTIES those cores; nothing lands on them until asked.
    # Without this the SCHED_FIFO thread of ros2_control_node was measured on
    # CPU5, sharing a P-core with the OpenBLAS spinners and free to migrate onto
    # CPU4, whose iwlwifi/nvme IRQs produce 6 ms stalls -- six missed FCI
    # deadlines in a row, i.e. communication_constraints_violation.
    # Only the one FF thread is moved; see scripts/pin_rt_thread.sh for why.
    # Set rt_pin_cpu:='' to disable (e.g. on a machine without isolcpus).
    rt_pin_cpu = str(p['rt_pin_cpu']).strip()
    pin_rt_thread = ExecuteProcess(
        # 'bash <script>' rather than executing it directly: setuptools
        # data_files does not preserve the executable bit on install.
        cmd=['bash', PathJoinSubstitution([
            FindPackageShare('franka_experiments'), 'scripts', 'pin_rt_thread.sh',
        ]), rt_pin_cpu, '60'],
        output='screen',
        shell=False,
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
    if rt_pin_cpu:
        # Start alongside the spawner, not after it: the script polls for the
        # FF thread anyway (up to 60 s), so starting early costs nothing. An
        # added delay here left the RT thread unpinned for 1+ s right as it
        # begins the 1 kHz FCI loop -- exactly the window that produced
        # communication_constraints_violation (thread free to land on a busy
        # core before ever being pinned).
        actions.append(TimerAction(period=control_delay,
                                   actions=[pin_rt_thread]))
        actions.append(LogInfo(msg=['[torque_stack] [RT pinning]     '
                                    'ros2_control_node FF thread -> CPU',
                                    rt_pin_cpu]))
    else:
        actions.append(LogInfo(
            msg='[torque_stack] [RT pinning]     DISABLED (rt_pin_cpu empty)'))

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
        if str(p['motion_source']).strip().lower() == 'rl':
            # Not auto-disabled: start_move_group is an explicit user argument
            # and silently ignoring it would be worse than a wasted node.
            actions.append(LogInfo(
                msg='[torque_stack] [MoveIt]          NOTE: motion_source:=rl '
                    'does not use move_group — pass start_move_group:=false to '
                    'save the startup cost'))
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

        # Depth profile PINNED. With no profile rs_launch.py lets librealsense
        # pick, and on this rig it picked 15 fps: the July bags and today's
        # live tracker lines both show dt=66.7 ms. That is one frame period of
        # extra sampling wait and a 62 ms camera hop (measured, scripts/
        # latency_budget.py) against 13 ms at 30 fps — the single largest item
        # in the blind time a fast obstacle can exploit. Empty = leave the
        # driver's choice alone.
        cam_args = {}
        profile = str(p['camera_depth_profile']).strip()
        if profile:
            cam_args['depth_module.depth_profile'] = profile
        realsense_driver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py',
            ]).perform(context)),
            launch_arguments=cam_args.items(),
        )
        actions.append(TimerAction(period=cam_delay, actions=[realsense_driver]))
        actions.append(LogInfo(msg=f'[torque_stack] [Perception]      RealSense depth profile: '
                                   f'{profile or "driver default"}'))

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
        # The tracking / sim switches live in the ROBOT CONFIG (one file for
        # every perception knob), but a launch argument has to be able to flip
        # them without editing an installed YAML. So when either is asked for,
        # the config is rewritten ONCE into the log directory with those two
        # keys overridden and the node is pointed at the copy.
        #
        # A copy rather than an in-place edit, and only when a switch is
        # actually set: the shipped file must stay the thing that describes the
        # default behaviour, and `git diff` after a launch must be empty.
        rtd_config = _rtd_config_with_overrides(
            p['robot_config_yaml'],
            tracking=_as_bool(p['obstacle_tracking']),
            sim_obstacle=_as_bool(p['sim_obstacle']),
        )
        real_time_distance_node = Node(
            package='franka_experiments',
            executable='real_time_distance',
            name='real_time_distance',
            output='log',
            additional_env=_SINGLE_THREAD_BLAS,
            parameters=[{
                'robot_config_path':      rtd_config,
                'camera_extrinsics_path': p['camera_extrinsics_yaml'],
                # Obstacle rows per control point, one per cluster; overrides
                # perception.multi_obstacle_k in fr3_control.yaml (0 = YAML).
                'multi_obstacle_k':       int(p['multi_obstacle_k']),
            }],
        )
        actions.append(TimerAction(period=rtd_delay, actions=[real_time_distance_node]))
        bag_player = _depth_bag_player(p['depth_bag'], p['robot_config_yaml'])
        if bag_player is not None:
            # After the node, so no frame is published into the void while
            # trimesh is still loading.
            actions.append(TimerAction(period=rtd_delay + 3.0,
                                       actions=[bag_player]))
            actions.append(LogInfo(msg=['[torque_stack] [Perception]      '
                                        'DEPTH FROM BAG: ', p['depth_bag']]))
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
        additional_env=_SINGLE_THREAD_BLAS,
        # These three are ordinary ROS parameters, so they override the YAML
        # without rewriting it — declare_from_spec reads the parameter back
        # after declaring it with the YAML value as the default.
        parameters=[{
            'obstacle_velocity_source': p['obstacle_velocity_source'],
            'enable_lateral_evasion':   _as_bool(p['lateral_evasion']),
            'enable_outrun_evasion':    _as_bool(p['outrun_evasion']),
            'enable_livelock_escape':   _as_bool(p['livelock_escape']),
            'enable_latency_compensation': _as_bool(p['latency_compensation']),
            'enable_uncertainty_margin': _as_bool(p['uncertainty_margin']),
            'enable_zone_ladder':       _as_bool(p['zone_ladder']),
            'enable_vobs_in_hdot':      _as_bool(p['vobs_in_hdot']),
            'enable_velocity_standoff': _as_bool(p['velocity_standoff']),
            # A launch BOOL onto a threshold parameter: the guard's "off" state
            # is 0.0 rad, and exposing the angle on the command line would
            # invite tuning a number whose right value is a property of the
            # depth sensor, not of the run. 0.15 rad is derived in
            # fr3_control.yaml; change it there if the hardware says so.
            'obstacle_velocity_normal_rot_max':
                (0.15 if _as_bool(p['obstacle_velocity_normal_guard']) else 0.0),
            # Same bool-onto-a-threshold shape, and for the same reason: the
            # guard's "off" state is 0.0 m, and the right value of the jump
            # floor is a property of the depth sensor's argmin noise, not of
            # the run. 0.10 m is derived in fr3_control.yaml.
            'obstacle_velocity_identity_jump':
                (0.10 if _as_bool(p['obstacle_identity_guard']) else 0.0),
        }],
    )
    # qddot_to_torque subscribes directly to qddot_safe (the CBF-filtered
    # acceleration) and converts it to torque — no remap needed.
    qddot_to_torque_node = Node(
        package='franka_experiments',
        executable='qddot_to_torque',
        name='qddot_to_torque',
        output='screen',
        additional_env=_SINGLE_THREAD_BLAS,
    )
    actions.append(TimerAction(period=dynamics_delay,
                               actions=[cbf_node, qddot_to_torque_node]))
    actions.append(LogInfo(msg=['[torque_stack] [CBF filter]      cbf_safety_filter + qddot_to_torque'
                                ' (delay=', str(dynamics_delay), 's)']))

    # ── [Motion generation] one q̈_nom source — never two ─────────────────────
    # Both sources publish /NS_1/qddot_nom and would fight for the topic, so
    # motion_source selects exactly one:
    #   'pentagon' — analytic path + avoidance-first shaping (default)
    #   'rl'       — ONNX Safe-RL policy trained in franka_sim against this same
    #                CBF filter (franka_sim_to_real_roadmap.md, Step 3)
    # The downstream chain (cbf_safety_filter → qddot_to_torque → controller) is
    # identical in both cases: the safety certificate does not depend on who
    # generates the nominal acceleration.
    motion_source = str(p['motion_source']).strip().lower()
    if motion_source not in ('pentagon', 'rl'):
        raise RuntimeError(
            f'motion_source="{motion_source}" — expected "pentagon" or "rl"')

    if motion_source == 'rl':
        # Only non-empty overrides are passed: every one of these has a
        # declare_parameter default in the node (model/config auto-discovered
        # from the franka_sim checkout), and forwarding '' would override a
        # working default with an invalid path.
        rl_params = {
            'action_scale': float(p['rl_action_scale']),
            'target_xyz':   _as_float_list(p['rl_target_xyz']) or [0.45, 0.0, 0.45],
        }
        if p['rl_onnx_model']:
            rl_params['onnx_model'] = p['rl_onnx_model']
        if p['rl_sim_config']:
            rl_params['sim_config'] = p['rl_sim_config']
        seq = _as_float_list(p['rl_target_sequence'])
        if seq:
            rl_params['target_sequence'] = seq
        commander_node = Node(
            package='franka_experiments',
            executable='rl_policy_commander',
            name='rl_policy_commander',
            namespace=p['namespace'],
            output='screen',
            additional_env=_SINGLE_THREAD_BLAS,
            parameters=[rl_params],
        )
        commander_label = 'rl_policy_commander (ONNX Safe-RL policy)'
    else:
        # Runs in the robot namespace so its relative MoveIt service clients
        # (compute_fk, compute_cartesian_path) resolve to move_group above.
        # Its topics are absolute (/NS_1/…) and unaffected by the namespace.
        commander_node = Node(
            package='franka_experiments',
            executable='pentagon_qddot_commander',
            name='pentagon_qddot_commander',
            namespace=p['namespace'],
            output='screen',
            additional_env=_SINGLE_THREAD_BLAS,
            # Path geometry (centre / shape / radius) is NOT set here: the node
            # reads it from config/fr3_control.yaml (params: path_center_xyz,
            # path_type, path_radius) as its declare_parameter defaults. Launch
            # files carry wiring, not tunables.

        )
        commander_label = 'pentagon_qddot_commander'
    actions.append(TimerAction(period=commander_delay, actions=[commander_node]))
    actions.append(LogInfo(msg=['[torque_stack] [Motion gen.]     ', commander_label,
                                ' (delay=', str(commander_delay), 's)']))

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
                'camera_depth_profile',
                default_value=str(_DEFAULTS.get('camera_depth_profile', '')),
                description='RealSense depth_module.depth_profile, e.g. 848x480x30. '
                            'Empty = driver default (measured to fall back to 15 fps)'),
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
                            'pentagon_qddot_commander; not needed for '
                            'motion_source:=rl)'),

            # ── Motion source: pentagon (default) | rl ─────────────────────────
            DeclareLaunchArgument(
                'motion_source',
                default_value=_DEFAULTS.get('motion_source', 'pentagon'),
                description="Which node publishes q̈_nom: 'pentagon' "
                            "(analytic path) or 'rl' (ONNX Safe-RL policy)"),
            DeclareLaunchArgument(
                'rl_onnx_model',
                default_value=_DEFAULTS.get('rl_onnx_model', ''),
                description='Path to the exported .onnx actor. Empty = '
                            'auto-discover the newest model in franka_sim/models'),
            DeclareLaunchArgument(
                'rl_sim_config',
                default_value=_DEFAULTS.get('rl_sim_config', ''),
                description='Path to the franka_sim config.yaml the policy was '
                            'trained with. Empty = the config frozen next to the model'),
            DeclareLaunchArgument(
                'rl_target_xyz',
                default_value=str(_DEFAULTS.get('rl_target_xyz', '[0.45, 0.0, 0.45]')),
                description='[m] reach target in fr3_link0, as "x,y,z"'),
            DeclareLaunchArgument(
                'rl_target_sequence',
                default_value=str(_DEFAULTS.get('rl_target_sequence', '')),
                description='Flat "x,y,z, x,y,z, …" list of targets visited in '
                            'order (overrides rl_target_xyz)'),
            DeclareLaunchArgument(
                'rl_action_scale',
                default_value=str(_DEFAULTS.get('rl_action_scale', '1.0')),
                description='Derate in (0,1] applied to the policy output: '
                            'q̈_nom = a·q̈_max·action_scale. Use 0.3 for a first run'),

            # ── Wiring / sequencing (defaults in config/launch_defaults.yaml) ──
            DeclareLaunchArgument(
                'robot_config_yaml',
                default_value=PathJoinSubstitution([
                    FindPackageShare('franka_experiments'),
                    'config', 'fr3_complete.yaml',
                ]),
                description='Path to fr3_complete.yaml (robot/mesh/distance config '
                            'loaded by real_time_distance)'),
            # ── Obstacle tracking / prediction / evasion ──────────
            # One switch each, all defaulting to today's behaviour, so a run
            # that names none of them is byte-for-byte the pre-tracker stack.
            DeclareLaunchArgument(
                'obstacle_tracking',
                default_value=str(_DEFAULTS.get('obstacle_tracking', 'false')),
                description='Cluster + Kalman-track obstacles in '
                            'real_time_distance and publish their 3D velocity '
                            'on LinkDistance. On its own it only adds fields '
                            'nobody reads - pair it with '
                            'obstacle_velocity_source:=tracker'),
            DeclareLaunchArgument(
                'obstacle_velocity_source',
                default_value=str(_DEFAULTS.get('obstacle_velocity_source',
                                                'residual')),
                description='Where the CBF v_obs comes from: residual (the '
                            'scalar a^T qdot - ddot, EMA at 0.7) or tracker '
                            '(n_hat^T v from the Kalman track). tracker '
                            'REQUIRES obstacle_tracking:=true'),
            DeclareLaunchArgument(
                'lateral_evasion',
                default_value=str(_DEFAULTS.get('lateral_evasion', 'false')),
                description='Step SIDEWAYS out of the swept volume when the '
                            'acceleration box says the closing rate cannot be '
                            'nulled before the gap reaches zero. Needs a '
                            'tracked velocity to have a direction at all'),
            DeclareLaunchArgument(
                'outrun_evasion',
                default_value=str(_DEFAULTS.get('outrun_evasion', 'false')),
                description='Step aside toward the fastest direction orthogonal '
                            'to v_obs when the joint velocity box says the '
                            'obstacle cannot be outrun along the normal '
                            '(cbf_safety_filter enable_outrun_evasion)'),
            DeclareLaunchArgument(
                'livelock_escape',
                default_value=str(_DEFAULTS.get('livelock_escape', 'false')),
                description='When the QP has been bending the nominal while the '
                            'arm stands still for livelock_stall_s, nudge it '
                            'tangentially in the barrier nullspace (bounded, '
                            'logged; cbf_safety_filter enable_livelock_escape)'),
            DeclareLaunchArgument(
                'zone_ladder',
                default_value=str(_DEFAULTS.get('zone_ladder', 'false')),
                description='Four rungs on the obstacle gap (notice/active/'
                            'priority/hold) scheduling the HOCBF gains, the '
                            'slack priority and whether the trajectory runs at '
                            'all; boundaries are multiples of d_safe '
                            '(zone_r_*). Below zone_r_hold*d_safe the TASK is suspended and '
                            'the nominal becomes a braking command; the barrier '
                            'rows keep full authority, so this is not a freeze '
                            '(cbf_safety_filter enable_zone_ladder)'),
            DeclareLaunchArgument(
                'multi_obstacle_k',
                default_value=str(_DEFAULTS.get('multi_obstacle_k', '0')),
                description='Obstacle rows per control point, one per cluster '
                            '(real_time_distance multi_obstacle_k). 1 = single '
                            'nearest point as before; 0 = use fr3_control.yaml'),
            DeclareLaunchArgument(
                'vobs_in_hdot',
                default_value=str(_DEFAULTS.get('vobs_in_hdot', 'false')),
                description='Tracked obstacle velocity inside hdot, signed, on '
                            'rows with a confirmed track (cbf_safety_filter '
                            'enable_vobs_in_hdot). REQUIRES obstacle_tracking:=true'),
            DeclareLaunchArgument(
                'velocity_standoff',
                default_value=str(_DEFAULTS.get('velocity_standoff', 'false')),
                description='Move the obstacle barrier out in proportion to the '
                            'estimated closing speed: d_safe + time_s * v_app '
                            '(cbf_safety_filter enable_velocity_standoff). '
                            'Watch hstd= in CBFDIAG'),
            DeclareLaunchArgument(
                'obstacle_velocity_normal_guard',
                default_value=str(_DEFAULTS.get(
                    'obstacle_velocity_normal_guard', 'false')),
                description='Discard a residual v_obs frame when the contact '
                            'normal rotated more than 0.15 rad between the two '
                            'frames it differences, i.e. the nearest obstacle '
                            'point hopped to another surface patch. Aimed at '
                            'the closing speed fabricated on a STATIC obstacle; '
                            'watch nrot= in CBFDIAG '
                            '(obstacle_velocity_normal_rot_max)'),
            DeclareLaunchArgument(
                'obstacle_identity_guard',
                default_value=str(_DEFAULTS.get(
                    'obstacle_identity_guard', 'false')),
                description='Discard a control point closing-speed state when '
                            'its nearest obstacle changes identity (different '
                            'track_id, or closest_point_human jumped further '
                            'than the fastest admitted obstacle could travel). '
                            'Stops the residual estimator differencing the '
                            'distance across two different bodies when one '
                            'static and one moving obstacle swap places; the '
                            'tracker estimate is unaffected. Watch nid= in '
                            'CBFDIAG (obstacle_velocity_identity_jump)'),
            DeclareLaunchArgument(
                'latency_compensation',
                default_value=str(_DEFAULTS.get('latency_compensation', 'false')),
                description='Move each tracked obstacle forward by the measured '
                            'blind time and tighten by the propagated position '
                            'uncertainty (cbf_safety_filter '
                            'enable_latency_compensation). OFF by default.'),
            DeclareLaunchArgument(
                'uncertainty_margin',
                default_value=str(_DEFAULTS.get('uncertainty_margin', 'false')),
                description='Tighten the barrier by the tracker own admitted '
                            'velocity uncertainty. Inert without a track'),
            DeclareLaunchArgument(
                'sim_obstacle',
                default_value=str(_DEFAULTS.get('sim_obstacle', 'false')),
                description='DANGER: render a synthetic sphere into the depth '
                            'stream so the whole avoidance chain can be '
                            'exercised end to end. The arm WILL move away from '
                            'something that is not there - never with a person '
                            'nearby'),
            DeclareLaunchArgument(
                'depth_bag',
                default_value=str(_DEFAULTS.get('depth_bag', '')),
                description='Path to a rosbag to replay as the DEPTH SOURCE '
                            'instead of a live camera (only the depth image and '
                            'camera_info are played, so TF and joint states '
                            'still come from the running robot). Empty = off'),
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

            DeclareLaunchArgument(
                'rt_pin_cpu',
                default_value=str(_DEFAULTS.get('rt_pin_cpu', '3')),
                description="CPU for the ros2_control_node SCHED_FIFO thread; "
                            "must be one of the isolcpus cores. '' disables pinning"),

            OpaqueFunction(function=_launch_all),
        ]
    )
