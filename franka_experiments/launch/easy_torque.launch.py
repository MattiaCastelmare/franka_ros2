"""Easy torque — pick & place on the rt_torque_controller pipeline.

Pipeline (identical in both modes):
  pick_place_qddot_commander → /NS_1/qddot_nom → qddot_to_torque → /NS_1/torque_cmd
  → rt_torque_controller → hardware (real FR3) | Gazebo (simulation)

real:=true  (default)  franka bringup (driver + broadcasters) + RT pinning + RViz
real:=false            Gazebo + robot_state_publisher + joint_state_publisher +
                       clock bridge + RViz, all under the same namespace as the
                       real robot so every node sees the same topics.

Examples
    ros2 launch franka_experiments easy_torque.launch.py              # real robot
    ros2 launch franka_experiments easy_torque.launch.py real:=false  # Gazebo
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
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
_QDDOT_NOM_TOPIC = '/NS_1/qddot_nom'

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

    actions.append(TimerAction(period=2.5, actions=[_qddot_to_torque_node(False)]))
    actions.append(TimerAction(period=3.0, actions=[_rviz_node(p, False)]))
    actions.append(TimerAction(period=4.0, actions=[_commander_node(p, False)]))
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
        on_exit=[torque_spawner, _qddot_to_torque_node(True)])))
    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=torque_spawner,
        on_exit=[TimerAction(period=2.0, actions=[_commander_node(p, True)])])))

    actions.append(TimerAction(period=3.0, actions=[_rviz_node(p, True)]))
    return actions


def _qddot_to_torque_node(use_sim_time):
    # ── Convertitore Dinamico (qddot_to_torque) ──────────────────────────────
    # Fondamentale: Converte le accelerazioni nominali (q̈) in coppie (τ) usando Pinocchio
    return Node(
        package='franka_experiments',
        executable='qddot_to_torque',
        name='qddot_to_torque',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/NS_1/qddot_safe', _QDDOT_NOM_TOPIC)
        ]
    )


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
    }

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
        # Without CBF the safe qddot is the nominal commander
        accel_topic=_QDDOT_NOM_TOPIC,
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
            OpaqueFunction(function=_launch_all)
        ]
    )