"""Easy torque — pick & place on the rt_torque_controller pipeline.

Pipeline (identical in both modes):
  pick_place_qddot_commander → /NS_1/qddot_nom → qddot_to_torque → /NS_1/torque_cmd
  → rt_torque_controller → hardware (real FR3) | Gazebo (simulation)

control_mode:=cbf inserts the acceleration-level CBF between the two:
  /NS_1/qddot_nom → cbf_safety_filter → /NS_1/qddot_safe → qddot_to_torque
  and rt_torque_controller integrates /NS_1/qddot_safe (the same q̈ that becomes τ_ff).

human:=true adds the human-arm perception (tracker, distance, visualizer, logger);
human_distance then publishes on /cbf/per_link_distances, the filter's input.

real:=true  (default)  franka bringup (driver + broadcasters) + RT pinning + RViz
                       (+ RealSense driver with human:=true)
real:=false            Gazebo + robot_state_publisher + joint_state_publisher +
                       clock bridge + RViz, all under the same namespace as the
                       real robot so every node sees the same topics.
                       (+ camera topics of human_bag replayed on Gazebo's clock)

Examples
    ros2 launch franka_experiments easy_torque.launch.py              # real robot
    ros2 launch franka_experiments easy_torque.launch.py real:=false  # Gazebo
    ros2 launch franka_experiments easy_torque.launch.py control_mode:=cbf human:=true
    ros2 launch franka_experiments easy_torque.launch.py real:=false \\
        control_mode:=cbf human:=true human_bag:=/bags/varied
"""

import os
import time
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from franka_experiments.utils.config import (
    load_franka_config_defaults,
    load_launch_defaults,
)
from franka_experiments.utils.distance_utils import load_robot_config
from franka_experiments.utils.launch_support import (
    declare_robot_args,
    declare_rt_torque_args,
    pick_controllers_yaml,
    resolve_controller_manager_name,
    rviz_config_for_namespace,
)

_LAUNCH_DEFAULTS, _ = load_launch_defaults()
_BRINGUP_DEFAULTS, _ = load_franka_config_defaults()
_DEFAULTS = {**_LAUNCH_DEFAULTS, **_BRINGUP_DEFAULTS}
_DEFAULTS['load_gripper'] = 'true'
_QDDOT_NOM_TOPIC = '/NS_1/qddot_nom'
_QDDOT_SAFE_TOPIC = '/NS_1/qddot_safe'
_FAST_JOINT_STATES = '/NS_1/franka/joint_states'
_CONTROL_MODES = ('nominal', 'cbf')

# human_distance publishes where cbf_safety_filter reads (topics.per_link_distances)
_PER_LINK_REMAP = ('/human/per_link_distances', '/cbf/per_link_distances')

# Camera topics of a recorded bag; its robot topics (/NS_1/joint_states, /tf) would
# fight with the simulated robot, so only these are replayed
_CAMERA_TOPICS = [
    '/camera/camera/color/image_raw',
    '/camera/camera/color/camera_info',
    '/camera/camera/aligned_depth_to_color/image_raw',
    '/camera/camera/aligned_depth_to_color/camera_info',
]

# One BLAS thread per numpy node, as in torque_control_stack.launch.py
_SINGLE_THREAD_BLAS = {
    'OPENBLAS_NUM_THREADS': '1',
    'OMP_NUM_THREADS': '1',
    'MKL_NUM_THREADS': '1',
    'NUMEXPR_NUM_THREADS': '1',
}

# Kd di rt_torque_controller in Gazebo
_SIM_D_GAINS = [30.0, 30.0, 30.0, 25.0, 10.0, 10.0, 1.0]

def _as_bool(value) -> bool:
    return str(value).strip().lower() in ('1', 'true', 'yes', 'y', 'on')


def _gazebo_robot_description(p, controllers_yaml):
    """URDF for Gazebo with the ros2_control plugin retargeted at this stack.
    The plugin is configured with the same controllers.yaml as the real robot,
    so the rt_torque_controller sees the same parameters (Kd, LPF, etc.)
    """
    xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', p['arm_id'], f"{p['arm_id']}.urdf.xacro",
    )
    doc = xacro.process_file(
        xacro_file,
        mappings={
            'arm_id': p['arm_id'],
            'hand': 'true',
            'ee_id': 'franka_hand',
            'ros2_control': 'true',
            'gazebo': 'true',
            'gazebo_effort': 'true',   # expose the effort command interface
        },
    )
    for joint in doc.getElementsByTagName('joint'):
        if joint.getAttribute('type') != 'prismatic':
            continue
        if not joint.getAttribute('name').startswith(f"{p['arm_id']}_finger_joint"):
            continue
        joint.setAttribute('type', 'fixed')
        for mimic in joint.getElementsByTagName('mimic'):
            joint.removeChild(mimic)
    for plugin in doc.getElementsByTagName('plugin'):
        if plugin.getAttribute('filename') != 'franka_ign_ros2_control-system':
            continue
        for params in plugin.getElementsByTagName('parameters'):
            params.firstChild.data = controllers_yaml
        ros = doc.createElement('ros')
        if p['namespace']:
            ns = doc.createElement('namespace')
            ns.appendChild(doc.createTextNode('/' + p['namespace'].strip('/')))
            ros.appendChild(ns)
        remap = doc.createElement('remapping')
        remap.appendChild(doc.createTextNode('joint_states:=franka/joint_states'))
        ros.appendChild(remap)
        plugin.appendChild(ros)
    return doc.toxml()


def _real_robot_actions(p, controllers_yaml, cm_name, use_fake):
    actions = []

    # ── Franka Bringup (Driver + state broadcaster) ─────────────────────────
    franka_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('franka_bringup'), 'launch', 'franka.launch.py',
        )),
        launch_arguments={
            'arm_id': p['arm_id'],
            'robot_ip': p['robot_ip'],
            'namespace': p['namespace'],
            'use_fake_hardware': p['use_fake_hardware'],
            'fake_sensor_commands': p['fake_sensor_commands'],
            'load_gripper': p['load_gripper'],
            'controllers_yaml': controllers_yaml,
        }.items(),
    )
    actions.append(franka_launch)

    # ── Torque Controller Spawner ───────────────────────────────────────────
    controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['rt_torque_controller', '--controller-manager', cm_name],
        output='screen',
    )
    actions.append(TimerAction(period=2.0, actions=[controller_spawner]))

    # ── Pin del thread RT sul core isolato ─────────────────────────────────
    # Stesso meccanismo di torque_control_stack.launch.py (vedi
    # tools/rt-tuning/README.md): senza pin il thread SCHED_FIFO di
    # ros2_control_node può migrare su un core con IRQ (wifi/nvme) che lo
    # stallano per millisecondi → FCI deadline mancate →
    # communication_constraints_violation. Parte insieme allo spawner: lo
    # script aspetta da sé che il thread FF compaia. rt_pin_cpu:='' disattiva.
    rt_pin_cpu = str(p['rt_pin_cpu']).strip()
    if rt_pin_cpu and not use_fake:
        pin_rt_thread = ExecuteProcess(
            cmd=['bash', os.path.join(
                get_package_share_directory('franka_experiments'),
                'scripts', 'pin_rt_thread.sh',
            ), rt_pin_cpu, '60'],
            output='screen',
        )
        actions.append(TimerAction(period=2.0, actions=[pin_rt_thread]))

    actions.append(TimerAction(period=2.5, actions=_filter_and_dynamics_nodes(p, False)))
    actions.append(TimerAction(period=3.0, actions=[_rviz_node(p, False)]))
    actions.append(TimerAction(period=4.0, actions=[_commander_node(p, False)]))

    if _as_bool(p['human']):
        # RealSense driver with depth aligned to color, as human.launch.py
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py',
            )),
            # publish_tf off: the driver's camera_color_frame -> camera_color_optical_frame
            launch_arguments={'align_depth.enable': 'true', 'publish_tf': 'false'}.items(),
        ))
        actions.extend(_human_nodes(p, False))
    return actions


def _simulation_actions(p, controllers_yaml, cm_name):
    actions = []
    ns = p['namespace']

    # ── Gazebo (GUI + physics) ──────────────────────────────────────────────
    actions.append(SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.dirname(get_package_share_directory('franka_description')),
    ))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py',
        )),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items(),
    ))
    actions.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    ))

    # ── Robot description / TF ──────────────────────────────────────────────
    # Same layout as franka.launch.py: JSB → franka/joint_states (1 kHz) →
    # joint_state_publisher → joint_states (30 Hz) → robot_state_publisher.
    actions.append(Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace=ns,
        parameters=[{
            'robot_description': _gazebo_robot_description(p, controllers_yaml),
            'use_sim_time': True,
        }],
        output='screen',
    ))
    actions.append(Node(
        package='joint_state_publisher', executable='joint_state_publisher',
        name='joint_state_publisher', namespace=ns,
        parameters=[{
            'source_list': ['franka/joint_states'],
            'rate': 30,
            'use_robot_description': False,
            'use_sim_time': True,
        }],
        output='screen',
    ))

    # ── Spawn in Gazebo → JSB → rt_torque_controller → commander ───────────
    # Chained on process exit rather than on timers: how long Gazebo takes to
    # come up varies a lot, and a spawner that starts before the in-process
    # controller_manager exists just times out.
    robot_description_topic = ('/' + ns.strip('/') if ns else '') + '/robot_description'
    spawn_entity = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', robot_description_topic, '-name', p['arm_id']],
        output='screen',
    )
    jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', cm_name,
                   '--controller-manager-timeout', '30'],
        output='screen',
    )
    torque_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['rt_torque_controller', '--controller-manager', cm_name,
                   '--controller-manager-timeout', '30'],
        output='screen',
    )
    actions.append(TimerAction(period=2.0, actions=[spawn_entity]))
    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=spawn_entity, on_exit=[jsb_spawner])))
    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=jsb_spawner,
        on_exit=[torque_spawner, *_filter_and_dynamics_nodes(p, True)])))
    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=torque_spawner,
        on_exit=[TimerAction(period=2.0, actions=[_commander_node(p, True)])])))

    actions.append(TimerAction(period=3.0, actions=[_rviz_node(p, True)]))

    if _as_bool(p['human']):
        actions.extend(_human_nodes(p, True))
        if p['human_bag']:
            # Camera topics only and NO --clock: Gazebo owns /clock. The image stamps stay
            # those of the recording; the tracker only uses their differences, and
            # cbf_safety_filter falls back to the receipt time for an implausible capture age.
            actions.append(TimerAction(period=5.0, actions=[ExecuteProcess(
                cmd=['ros2', 'bag', 'play', os.path.expanduser(p['human_bag']), '--loop',
                     '--read-ahead-queue-size', '1000', '--topics', *_CAMERA_TOPICS],
                output='screen',
            )]))
    return actions


def _cbf_enabled(p) -> bool:
    return p['control_mode'] == 'cbf'


def _filter_and_dynamics_nodes(p, use_sim_time):
    """qddot_to_torque, preceded by cbf_safety_filter when control_mode:=cbf."""
    # ── Convertitore Dinamico (qddot_to_torque) ──────────────────────────────
    # Fondamentale: Converte le accelerazioni (q̈) in coppie (τ) usando Pinocchio.
    # Its input is /NS_1/qddot_safe: the CBF output, or the nominal q̈ remapped onto it.
    nodes = [Node(
        package='franka_experiments',
        executable='qddot_to_torque',
        name='qddot_to_torque',
        output='screen',
        additional_env=_SINGLE_THREAD_BLAS,
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[] if _cbf_enabled(p) else [(_QDDOT_SAFE_TOPIC, _QDDOT_NOM_TOPIC)],
    )]
    if _cbf_enabled(p):
        # ── CBF safety filter: /NS_1/qddot_nom → /NS_1/qddot_safe ───────────
        # Parameters not set here come from fr3_control.yaml. 'tracker' reads the
        # obstacle velocity human_distance computes from the Kalman filter (the
        # filter takes max(tracker, residual), so it is never less cautious than
        # 'residual'); vobs_in_hdot puts its signed component along the normal into ḣ.
        nodes.append(Node(
            package='franka_experiments',
            executable='cbf_safety_filter',
            name='cbf_safety_filter',
            output='both',
            additional_env=_SINGLE_THREAD_BLAS,
            parameters=[{
                'use_sim_time': use_sim_time,
                'obstacle_velocity_source': p['obstacle_velocity_source'],
                'enable_vobs_in_hdot': _as_bool(p['vobs_in_hdot']),
            }],
        ))
    return nodes


def _human_nodes(p, use_sim_time):
    """Human-arm perception feeding the CBF (same nodes as human.launch.py)."""
    pkg_share = get_package_share_directory('franka_experiments')
    extrinsics = load_robot_config(os.path.join(pkg_share, 'config', 'camera_extrinsics.yaml'))
    tr, rot = extrinsics['translation'], extrinsics['rotation']
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fr3_to_camera_link',
        arguments=[
            '--x', str(tr['x']), '--y', str(tr['y']), '--z', str(tr['z']),
            '--qx', str(rot['x']), '--qy', str(rot['y']), '--qz', str(rot['z']), '--qw', str(rot['w']),
            '--frame-id', 'fr3_link0', '--child-frame-id', 'camera_color_optical_frame',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='log',
    )

    def human_node(executable, name, extra_params=None):
        return Node(
            package='franka_experiments',
            executable=executable,
            name=name,
            output='screen',
            additional_env=_SINGLE_THREAD_BLAS,
            parameters=[{'use_sim_time': use_sim_time, **(extra_params or {})}],
            remappings=[_PER_LINK_REMAP],
        )

    return [
        LogInfo(msg='[easy_torque] human perception: tracker, distance → /cbf/per_link_distances, '
                    'visualizer, logger'),
        camera_tf,
        human_node('human_tracker', 'human_tracker'),
        human_node('human_distance', 'human_distance', {'joint_state_topic': _FAST_JOINT_STATES}),
        human_node('human_visualizer', 'human_visualizer'),
        human_node('human_logging', 'human_logger', {
            'run_name': p['run_name'],
            'robot_state_topic': _FAST_JOINT_STATES,
        }),
    ]


def _commander_node(p, use_sim_time):
    # ── Pick & Place Qddot Commander ────
    return Node(
        package='franka_experiments',
        executable='pick_place_qddot_commander',
        name='pick_place_qddot_commander',
        namespace=p['namespace'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )


def _rviz_node(p, use_sim_time):
    # trajectory_rviz.rviz carries the RobotModel (/NS_1/robot_description) and
    # TF displays, retargeted at the bringup namespace.
    return Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=p['namespace'],
        arguments=['-d', rviz_config_for_namespace(p['namespace'])],
        parameters=[{'use_sim_time': use_sim_time}],
        output='log',
    )


def _launch_all(context):
    p = {
        'real': LaunchConfiguration('real').perform(context),
        'namespace': LaunchConfiguration('namespace').perform(context),
        'use_fake_hardware': LaunchConfiguration('use_fake_hardware').perform(context),
        'robot_ip': LaunchConfiguration('robot_ip').perform(context),
        'arm_id': LaunchConfiguration('arm_id').perform(context),
        'fake_sensor_commands': LaunchConfiguration('fake_sensor_commands').perform(context),
        'load_gripper': LaunchConfiguration('load_gripper').perform(context),
        'controllers_yaml': LaunchConfiguration('controllers_yaml').perform(context),
        'gazebo': LaunchConfiguration('gazebo').perform(context),
        'lpf_alpha': LaunchConfiguration('lpf_alpha').perform(context),
        'tau_max_scale': LaunchConfiguration('tau_max_scale').perform(context),
        'torque_command_topic': LaunchConfiguration('torque_command_topic').perform(context),
        'rt_pin_cpu': LaunchConfiguration('rt_pin_cpu').perform(context),
        'control_mode': LaunchConfiguration('control_mode').perform(context).strip().lower(),
        'human': LaunchConfiguration('human').perform(context),
        'human_bag': LaunchConfiguration('human_bag').perform(context).strip(),
        'obstacle_velocity_source': LaunchConfiguration('obstacle_velocity_source').perform(context),
        'vobs_in_hdot': LaunchConfiguration('vobs_in_hdot').perform(context),
        'run_name': LaunchConfiguration('run_name').perform(context),
    }
    if p['control_mode'] not in _CONTROL_MODES:
        raise RuntimeError(f"control_mode must be one of {_CONTROL_MODES}, got '{p['control_mode']}'")

    real = _as_bool(p['real'])
    use_fake = real and _as_bool(p['use_fake_hardware'])
    if not real:
        # Simulation: the controller must skip the Franka-only service calls.
        p['gazebo'] = 'true'

    # ── Configurazione rt_torque_controller ─────────────────────────────────
    # is_real governa update_rate (1 kHz) e i gain Kp/Kd: vengono azzerati solo
    # per il mock hardware, che non integra la dinamica. Gazebo la integra (e
    # compensa g(q) come il firmware), quindi lì il feedback serve come sul
    # robot reale.
    rt_params = dict(
        is_real=not use_fake,
        arm_id=p['arm_id'],
        controller_type='torque',
        torque_command_topic=p['torque_command_topic'],
        gazebo=p['gazebo'],
        lpf_alpha=float(p['lpf_alpha']),
        tau_max_scale=float(p['tau_max_scale']),
        # The q̈ the PD integrates must be the one qddot_to_torque turns into τ_ff:
        # the CBF output with the filter, the nominal commander without it
        accel_topic=_QDDOT_SAFE_TOPIC if _cbf_enabled(p) else _QDDOT_NOM_TOPIC,
    )
    if not real:
        rt_params['d_gains'] = _SIM_D_GAINS
    controllers_yaml = pick_controllers_yaml(p['controllers_yaml'], use_fake, rt_params)
    cm_name = resolve_controller_manager_name(p['namespace'])

    if real:
        return _real_robot_actions(p, controllers_yaml, cm_name, use_fake)
    return _simulation_actions(p, controllers_yaml, cm_name)


def generate_launch_description():
    return LaunchDescription(
        declare_robot_args(_DEFAULTS)
        + declare_rt_torque_args(_DEFAULTS)
        + [
            DeclareLaunchArgument(
                'real',
                default_value='true',
                description='true = real FR3 (franka bringup); '
                            'false = Gazebo simulation'
            ),
            DeclareLaunchArgument(
                'torque_command_topic',
                default_value=str(_DEFAULTS.get('torque_command_topic', 'torque_cmd')),
                description='Topic for rt_torque_controller'
            ),
            DeclareLaunchArgument(
                'rt_pin_cpu',
                default_value=str(_DEFAULTS.get('rt_pin_cpu', '3')),
                description="Isolated CPU for the ros2_control RT thread ('' = no pinning)"
            ),
            DeclareLaunchArgument(
                'control_mode',
                default_value='nominal',
                description='nominal = commander q̈ straight to the torque chain; '
                            'cbf = through cbf_safety_filter'
            ),
            DeclareLaunchArgument(
                'human',
                default_value='false',
                description='true = human-arm perception (tracker, distance, visualizer, logger)'
            ),
            DeclareLaunchArgument(
                'human_bag',
                default_value='',
                description='real:=false only: bag whose camera topics are replayed for the tracker'
            ),
            DeclareLaunchArgument(
                'obstacle_velocity_source',
                default_value='tracker',
                description='cbf_safety_filter: tracker (Kalman velocity from human_distance) | residual'
            ),
            DeclareLaunchArgument(
                'vobs_in_hdot',
                default_value='false',
                description='cbf_safety_filter: signed tracked velocity inside ḣ. Off by default as in '
                            'fr3_control.yaml: the raw Kalman velocity is noisy on a still person '
                            '(~0.16 m/s median at sigma_a = 3) and a "receding" sample relaxes the row'
            ),
            DeclareLaunchArgument(
                'run_name',
                default_value=time.strftime('%Y%m%d_%H%M%S'),
                description='human_logging output folder experiment_logs/<run_name>'
            ),
            OpaqueFunction(function=_launch_all)
        ]
    )