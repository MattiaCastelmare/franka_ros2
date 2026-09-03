"""Composing a ROS 2 launch description for this package.

OWNS
----
Everything that runs at *launch composition* time, before any node exists:

* ``declare_robot_args`` / ``declare_rt_blender_args`` / ``declare_rt_torque_args``
  — the shared ``DeclareLaunchArgument`` sets
* ``generate_rt_controllers_yaml`` / ``pick_controllers_yaml`` /
  ``write_temp_yaml`` — generating the ros2_control controllers YAML
* :func:`resolve_controller_manager_name`
* :func:`build_wrapper_log_actions`

DOES NOT OWN
------------
* Anything a running node does — that is ``utils.node_runtime``.
* Reading configuration files — that is ``utils.config``.
* Node parameter declaration — that is ``utils.params``.

Hot-path note: none of this runs at robot rates; it executes once, in the
``ros2 launch`` process, before the controller manager starts.

Moved out of ``utils.ros`` in Phase 2; the bodies are byte-identical relocations
and every symbol is still importable from ``utils.ros``.
"""

from __future__ import annotations

import os

from typing import Any, Optional, Sequence

import yaml

from .config import load_launch_defaults  # noqa: F401


def resolve_controller_manager_name(namespace: str) -> str:
    """Build the absolute ``controller_manager`` node path from *namespace*."""
    if namespace:
        return '/' + namespace + '/controller_manager'
    return '/controller_manager'

def _generate_cbf_base_yaml(is_real: bool, arm_id: str) -> str:
    """Minimal YAML for CBF pipeline: only JSB + CBFTorqueController type registration."""
    lines = [
        '# Auto-generated for CBF pipeline — DO NOT EDIT',
        '',
        '/**:',
        '  controller_manager:',
        '    ros__parameters:',
        f'      update_rate: {1000 if is_real else 100}',
    ]
    if is_real:
        lines.append('      thread_priority: 85')
    lines += [
        '',
        '      joint_state_broadcaster:',
        '        type: joint_state_broadcaster/JointStateBroadcaster',
    ]
    if is_real:
        lines += [
            '',
            '      franka_robot_state_broadcaster:',
            '        type: franka_robot_state_broadcaster/FrankaRobotStateBroadcaster',
        ]
    lines += [
        '',
        '      cbf_torque_controller:',
        '        type: franka_rt_controllers/CBFTorqueController',
    ]
    if is_real:
        lines += [
            '',
            '/**:',
            '  franka_robot_state_broadcaster:',
            '    ros__parameters:',
            '      lock_try_count: 200',
            '      lock_sleep_interval: 50',
            '      lock_log_error: false',
            '      lock_update_success: true',
        ]
    lines += [
        '',
        '/**:',
        '  joint_state_broadcaster:',
        '    ros__parameters:',
        f'      arm_id: "{arm_id}"',
        '',
        '/**:',
        '  cbf_torque_controller:',
        '    ros__parameters:',
        f'      arm_id: {arm_id}',
        '      qddot_safe_topic: /NS_1/qddot_safe',
        '      qddot_timeout_s: 0.1',
        '      kd: [20.0, 20.0, 20.0, 20.0, 10.0, 10.0, 10.0]',
        '',
    ]
    return '\n'.join(lines)


# Public alias so launch files can call it directly
generate_cbf_base_yaml = _generate_cbf_base_yaml

def generate_rt_controllers_yaml(
    is_real: bool,
    arm_id: str,
    qdot_max: float = 0.0,
    command_topic: str = 'qdot_cmd',
    max_accel: float = 0.0,
    timeout_threshold_s: float = 0.0,
    timeout_ramp_s: float = 0.0,
    gazebo: str = 'false',
    enable_interpolation: str = 'true',
    *,
    controller_type: str = 'velocity',
    torque_command_topic: str = 'torque_cmd',
    lpf_alpha: float = 1.0,
    tau_max_scale: float = 1.0,
    d_gains: Optional[Sequence[float]] = None,
    e_max: float = 1.0,
    accel_topic: str = '/NS_1/qddot_safe',
    qdot_margin: float = 0.95,
) -> str:
    """Build a complete, self-contained controllers YAML.

    ``controller_type='velocity'`` (default) generates YAML for
    ``rt_velocity_executor_controller``.  ``controller_type='torque'``
    generates YAML for ``rt_torque_controller``.

    Returns the YAML content as a string.  All parameters are fully resolved
    — no ``REPLACE_ME`` placeholders.
    """
    if controller_type == 'cbf':
        return _generate_cbf_base_yaml(is_real=is_real, arm_id=arm_id)

    if controller_type == 'torque':
        return _generate_torque_yaml(
            is_real=is_real,
            arm_id=arm_id,
            command_topic=torque_command_topic,
            gazebo=gazebo,
            lpf_alpha=lpf_alpha,
            tau_max_scale=tau_max_scale,
            d_gains=d_gains,
            e_max=e_max,
            accel_topic=accel_topic,
            qdot_margin=qdot_margin,
        )

    lines = [
        '# Auto-generated at launch time by the launch system',
        '# Controller: rt_velocity_executor_controller  —  DO NOT EDIT (regenerated every launch)',
        '',
        '/**:',
        '  controller_manager:',
        '    ros__parameters:',
        f'      update_rate: {1000 if is_real else 100}',
    ]
    if is_real:
        lines.append('      thread_priority: 85')
    lines += [
        '',
        '      joint_state_broadcaster:',
        '        type: joint_state_broadcaster/JointStateBroadcaster',
    ]
    if is_real:
        lines += [
            '',
            '      franka_robot_state_broadcaster:',
            '        type: franka_robot_state_broadcaster/FrankaRobotStateBroadcaster',
        ]
    lines += [
        '',
        '      rt_velocity_executor_controller:',
        '        type: franka_rt_controllers/RtVelocityExecutorController',
    ]
    if is_real:
        lines += [
            '',
            '/**:',
            '  franka_robot_state_broadcaster:',
            '    ros__parameters:',
            '      lock_try_count: 200',
            '      lock_sleep_interval: 50',
            '      lock_log_error: false',
            '      lock_update_success: true',
        ]
    lines += [
        '',
        '/**:',
        '  joint_state_broadcaster:',
        '    ros__parameters:',
        f'      arm_id: "{arm_id}"',
        '',
        '/**:',
        '  rt_velocity_executor_controller:',
        '    ros__parameters:',
        f'      arm_id: {arm_id}',
        '      joints:',
    ]
    for i in range(1, 8):
        lines.append(f'        - {arm_id}_joint{i}')
    lines += [
        f'      command_topic: {command_topic}',
        f'      qdot_max: {qdot_max}',
        f'      max_accel: {max_accel}',
        f'      timeout_threshold_s: {timeout_threshold_s}',
        f'      timeout_ramp_s: {timeout_ramp_s}',
        f'      gazebo: {gazebo}',
        f'      enable_interpolation: {enable_interpolation}',
        '',
    ]
    return '\n'.join(lines)

def _ensure_urdf_cached(arm_id: str) -> str:
    """Return path to a cached FR3 URDF (with hand), generating it if absent."""
    import subprocess
    cache_path = f'/tmp/franka_rt_torque_{arm_id}.urdf'
    if not os.path.exists(cache_path):
        from ament_index_python.packages import get_package_share_directory
        desc_share = get_package_share_directory('franka_description')
        xacro_file = os.path.join(desc_share, 'robots', 'fr3', 'fr3.urdf.xacro')
        result = subprocess.run(
            ['xacro', xacro_file, 'hand:=true', 'ee_id:=franka_hand', '-o', cache_path],
            capture_output=True, text=True, timeout=60,
        )
        if result.returncode != 0:
            raise RuntimeError(f'xacro failed for {arm_id}: {result.stderr}')
    return cache_path

def _generate_torque_yaml(is_real, arm_id, command_topic, gazebo, lpf_alpha,
                          tau_max_scale, d_gains=None, e_max=1.0,
                          accel_topic='/NS_1/qddot_safe', qdot_margin=0.95):
    """Build controllers YAML for rt_torque_controller.

    d_gains / e_max / accel_topic parametrizzano il feedback di velocità a 1 kHz
    (τ = τ_ff + Kd·(q̇_des − q̇)). VALORI INIZIALI da validare in hardware.

    qdot_margin è il tetto di velocità sul riferimento integrato q̇_des, come
    frazione dell'inviluppo di velocità del firmware FR3 (il controller porta
    i limiti di franka_description come default, quindi qui basta la frazione).
    0 disattiva il tetto.
    """
    if d_gains is None:
        # Per-giunto, decrescente verso il polso (inerzie minori, attrito
        # relativo maggiore → si tara indipendentemente). INITIAL estimate.
        d_gains = [30.0, 30.0, 30.0, 25.0, 10.0, 10.0, 5.0]
    if not is_real:
        # Fake hardware: il mock non simula la dinamica di coppia (un comando di
        # effort non viene integrato in velocità), quindi il feedback Kd·(q̇_des−q̇)
        # sarebbe privo di significato fisico. Lo si disattiva via configurazione
        # (gain a zero → il controller resta puro pass-through del feedforward),
        # NON con un ramo condizionale nel C++.
        d_gains = [0.0] * 7
    lines = [
        '# Auto-generated at launch time by the launch system',
        '# Controller: rt_torque_controller  —  DO NOT EDIT (regenerated every launch)',
        '',
        '/**:',
        '  controller_manager:',
        '    ros__parameters:',
        f'      update_rate: {1000 if is_real else 100}',
    ]
    if is_real:
        lines.append('      thread_priority: 85')
    lines += [
        '',
        '      joint_state_broadcaster:',
        '        type: joint_state_broadcaster/JointStateBroadcaster',
    ]
    if is_real:
        lines += [
            '',
            '      franka_robot_state_broadcaster:',
            '        type: franka_robot_state_broadcaster/FrankaRobotStateBroadcaster',
        ]
    lines += [
        '',
        '      rt_torque_controller:',
        '        type: franka_rt_controllers/RtTorqueController',
    ]
    if is_real:
        lines += [
            '',
            '/**:',
            '  franka_robot_state_broadcaster:',
            '    ros__parameters:',
            '      lock_try_count: 200',
            '      lock_sleep_interval: 50',
            '      lock_log_error: false',
            '      lock_update_success: true',
        ]
    lines += [
        '',
        '/**:',
        '  joint_state_broadcaster:',
        '    ros__parameters:',
        f'      arm_id: "{arm_id}"',
        '',
        '/**:',
        '  rt_torque_controller:',
        '    ros__parameters:',
        f'      arm_id: {arm_id}',
        '      joints:',
    ]
    for i in range(1, 8):
        lines.append(f'        - {arm_id}_joint{i}')
    urdf_path = _ensure_urdf_cached(arm_id)
    d_gains_str = '[' + ', '.join(f'{g}' for g in d_gains) + ']'
    lines += [
        f'      command_topic: {command_topic}',
        f'      accel_topic: {accel_topic}',
        # Feedback di velocità a 1 kHz (vedi rt_torque_controller). INITIAL —
        # tarare in hardware. Su fake hardware il controller gira a 100 Hz e il
        # mock non simula la dinamica di coppia: il feedback resta limitato dal
        # clamp ma è privo di significato fisico (vedi note di design).
        f'      d_gains: {d_gains_str}',
        f'      e_max: {e_max}',
        # Backstop a 1 kHz contro `joint_velocity_violation`: q̇_des è un
        # integratore libero di q̈_safe e il box di velocità del CBF vive a
        # 100 Hz sul solo COMANDO q̈, quindi senza questo tetto nessuno fra i
        # due conosce q̇_max. 0.95 lascia il reflex del firmware come ultima
        # rete, non come prima.
        f'      qdot_margin: {qdot_margin}',
        f'      lpf_alpha: {lpf_alpha}',
        f'      tau_max_scale: {tau_max_scale}',
        f'      gazebo: {gazebo}',
        f'      urdf_path: {urdf_path}',
        '',
    ]
    return '\n'.join(lines)

def write_temp_yaml(content: str, prefix: str = 'franka_rt_blender_') -> str:
    """Write *content* to a temporary YAML file and return its path."""
    import tempfile
    fd, path = tempfile.mkstemp(prefix=prefix, suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        f.write(content)
    return path

def pick_controllers_yaml(
    explicit: str,
    use_fake: bool,
    rt_params: dict,
) -> str:
    """Select (or generate) the correct controllers YAML path.

    Parameters
    ----------
    explicit : str
        CLI override value (``'__auto__'`` means auto-select).
    use_fake : bool
        Whether fake hardware is being used.
    rt_params : dict
        Keyword arguments for :func:`generate_rt_controllers_yaml`.
    """
    if explicit != '__auto__':
        return explicit
    content = generate_rt_controllers_yaml(**rt_params)
    return write_temp_yaml(content)

def declare_robot_args(defaults: Optional[dict] = None) -> list:
    """Return common robot/hardware ``DeclareLaunchArgument`` list.

    These arguments are shared across multiple launch files in
    ``franka_experiments``.

    Parameters
    ----------
    defaults : dict or None
        Override default values.  Typically the dict returned by
        :func:`load_launch_defaults` (or :func:`load_franka_config_defaults`).
    """
    from launch.actions import DeclareLaunchArgument
    d = defaults or {}
    return [
        DeclareLaunchArgument(
            'arm_id', default_value=d.get('arm_id', 'fr3'),
            description='Robot arm model identifier'),
        DeclareLaunchArgument(
            'robot_ip', default_value=d.get('robot_ip', '192.168.2.10'),
            description='IP address of the robot'),
        DeclareLaunchArgument(
            'namespace', default_value=d.get('namespace', ''),
            description='Namespace for the robot (empty = no namespace)'),
        DeclareLaunchArgument(
            'use_fake_hardware', default_value=d.get('use_fake_hardware', 'false'),
            description='Use fake (mock) hardware'),
        DeclareLaunchArgument(
            'fake_sensor_commands', default_value=d.get('fake_sensor_commands', 'false'),
            description='Fake sensor commands'),
        DeclareLaunchArgument(
            'load_gripper', default_value=d.get('load_gripper', 'false'),
            description='Load Franka Gripper'),
        DeclareLaunchArgument(
            'controllers_yaml', default_value=d.get('controllers_yaml', '__auto__'),
            description='Path to controllers YAML (__auto__ = auto-select)'),
    ]

def declare_rt_blender_args(defaults: Optional[dict] = None) -> list:
    """Return RT-controller ``DeclareLaunchArgument`` list.

    Parameters
    ----------
    defaults : dict or None
        Override default values (from :func:`load_launch_defaults`).
    """
    from launch.actions import DeclareLaunchArgument
    d = defaults or {}
    return [
        DeclareLaunchArgument(
            'qdot_max', default_value=d.get('qdot_max', '0.2'),
            description='[RT] Per-joint velocity clamp [rad/s]. 0.0 = disabled.'),
        DeclareLaunchArgument(
            'command_topic', default_value=d.get('command_topic', 'qdot_cmd'),
            description='[RT] Topic for velocity commands (Float64MultiArray, size=7).'),
        DeclareLaunchArgument(
            'max_accel', default_value=d.get('max_accel', '0.0'),
            description='[RT] Max joint acceleration [rad/s²]. 0.0 = disabled.'),
        DeclareLaunchArgument(
            'timeout_threshold_s', default_value=d.get('timeout_threshold_s', '0.0'),
            description='[RT] Seconds before smooth timeout ramp. 0.0 = disabled.'),
        DeclareLaunchArgument(
            'timeout_ramp_s', default_value=d.get('timeout_ramp_s', '0.0'),
            description='[RT] Linear ramp to zero duration [s]. 0.0 = disabled.'),
        DeclareLaunchArgument(
            'gazebo', default_value=d.get('gazebo', 'false'),
            description='[RT] Set true when running in Gazebo.'),
        DeclareLaunchArgument(
            'enable_interpolation', default_value=d.get('enable_interpolation', 'true'),
            description='[RT] Linear interpolation between low-rate samples.'),
    ]

def declare_rt_torque_args(defaults: Optional[dict] = None) -> list:
    """Return torque-controller ``DeclareLaunchArgument`` list.

    Counterpart of :func:`declare_rt_blender_args` for the
    ``rt_torque_controller`` pipeline.  Declares only the parameters that
    are specific to torque control; hardware/robot args come from
    :func:`declare_robot_args`.

    Parameters
    ----------
    defaults : dict or None
        Override default values (from :func:`load_launch_defaults`).
    """
    from launch.actions import DeclareLaunchArgument
    d = defaults or {}
    return [
        DeclareLaunchArgument(
            'gazebo', default_value=d.get('gazebo', 'false'),
            description='[RT] Set true when running in Gazebo.'),
        DeclareLaunchArgument(
            'lpf_alpha', default_value=str(d.get('lpf_alpha', '1.0')),
            description='[torque] Low-pass filter alpha for tau_cmd. 1.0 = disabled.'),
        DeclareLaunchArgument(
            'tau_max_scale', default_value=str(d.get('tau_max_scale', '1.0')),
            description='[torque] Scale factor applied to per-joint torque limits.'),
    ]

def build_wrapper_log_actions(
    params: dict,
    config_path: str,
    controllers_yaml: str,
    controller_name: str,
    cm_name: str,
) -> list:
    """Build ``LogInfo`` actions for ``wrapper_forward_velocity`` launch.

    Parameters
    ----------
    params : dict
        All resolved launch parameters (string values).
    config_path : str
        Path to the ``franka.config.yaml`` that was loaded.
    controllers_yaml : str
        Resolved controllers YAML path.
    controller_name : str
        Name of the controller being spawned.
    cm_name : str
        Absolute controller_manager node path.
    """
    from launch.actions import LogInfo

    ns_display = params['namespace'] or '<none>'
    p = params

    actions = [
        LogInfo(msg=['╔══ wrapper_forward_velocity ══════════════════════════╗']),
        LogInfo(msg=['[wrapper] mode               : RT (rt_velocity_executor_controller)']),
        LogInfo(msg=['[wrapper] franka.config.yaml : ', config_path]),
        LogInfo(msg=['[wrapper] arm_id             : ', params['arm_id']]),
        LogInfo(msg=['[wrapper] robot_ip           : ', params['robot_ip']]),
        LogInfo(msg=['[wrapper] namespace          : ', ns_display]),
        LogInfo(msg=['[wrapper] use_fake_hardware  : ',
                     params['use_fake_hardware']]),
        LogInfo(msg=['[wrapper] controllers_yaml   : ', controllers_yaml]),
        LogInfo(msg=['[wrapper] controller_to_spawn: ', controller_name]),
        LogInfo(msg=['[wrapper] controller-manager : ', cm_name]),
        LogInfo(msg=['[wrapper] ── RT controller parameters ──']),
        LogInfo(msg=['[wrapper]   command_topic    : ', p['command_topic']]),
        LogInfo(msg=['[wrapper]   qdot_max         : ', p['qdot_max'],
                     ' rad/s (scalar, same for all 7 joints)']),
        LogInfo(msg=['[wrapper]   max_accel        : ', p['max_accel'],
                     ' rad/s²',
                     '  [ACTIVE]' if float(p['max_accel']) > 0
                     else '  [DISABLED]']),
        LogInfo(msg=['[wrapper]   timeout_threshold: ',
                     p['timeout_threshold_s'], ' s',
                     '  [ACTIVE]' if (float(p['timeout_threshold_s']) > 0
                                      and float(p['timeout_ramp_s']) > 0)
                     else '  [DISABLED]']),
        LogInfo(msg=['[wrapper]   timeout_ramp     : ',
                     p['timeout_ramp_s'], ' s']),
        LogInfo(msg=['[wrapper]   gazebo           : ', p['gazebo']]),
        LogInfo(msg=['[wrapper]   interpolation    : ',
                     p['enable_interpolation'],
                     '  [ACTIVE]' if p['enable_interpolation'] == 'true'
                     else '  [DISABLED]']),
        LogInfo(msg=['╚═════════════════════════════════════════════════════╝']),
    ]

    return actions
