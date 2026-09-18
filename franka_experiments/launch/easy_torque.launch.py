import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
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

_LAUNCH_DEFAULTS, _ = load_launch_defaults()
_BRINGUP_DEFAULTS, _ = load_franka_config_defaults()
_DEFAULTS = {**_LAUNCH_DEFAULTS, **_BRINGUP_DEFAULTS}

def _launch_all(context):
    p = {
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
        'enable_camera': LaunchConfiguration('enable_camera').perform(context),
    }

    use_fake = str(p['use_fake_hardware']).strip().lower() in ('1', 'true', 'yes', 'y', 'on')
    start_camera = str(p['enable_camera']).strip().lower() in ('1', 'true', 'yes', 'y', 'on')

    # ── 1. Configurazione rt_torque_controller ─────────────────────────────────
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
    cm_name = resolve_controller_manager_name(p['namespace'])

    actions = []

    # ── 2. Franka Bringup (Driver + state broadcaster) ─────────────────────────
    franka_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py',
        ]).perform(context)),
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

    # ── 3. Percezione: Telecamera e TF ─────────────────────────────────────────
    if start_camera:
        realsense_driver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py',
            ]).perform(context)),
        )
        actions.append(realsense_driver)
        
        link_ext_path = PathJoinSubstitution([
            FindPackageShare('franka_experiments'), 'config', 'camera_link_extrinsics.yaml'
        ]).perform(context)
        
        try:
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
            actions.append(TimerAction(period=1.0, actions=[camera_tf_node]))
        except FileNotFoundError:
            pass

    # ── 4. Torque Controller Spawner ───────────────────────────────────────────
    controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['rt_torque_controller', '--controller-manager', cm_name],
        output='screen',
    )
    actions.append(TimerAction(period=2.0, actions=[controller_spawner]))

    # ── 5. Convertitore Dinamico (qddot_to_torque) ──────────────────────────────
    # Fondamentale: Converte le accelerazioni nominali (q̈) in coppie (τ) usando Pinocchio
    qddot_to_torque_node = Node(
        package='franka_experiments',
        executable='qddot_to_torque',
        name='qddot_to_torque',
        output='screen',
        remappings=[
            ('/NS_1/qddot_safe', '/NS_1/qddot_nom')
        ]
    )
    actions.append(TimerAction(period=2.5, actions=[qddot_to_torque_node]))

    # ── 6. Pick & Place Qddot Commander ────
    commander_node = Node(
        package='franka_experiments',
        executable='pick_place_qddot_commander',
        name='pick_place_qddot_commander',
        namespace=p['namespace'],
        output='screen',
    )
    actions.append(TimerAction(period=4.0, actions=[commander_node]))

    return actions

def generate_launch_description():
    return LaunchDescription(
        declare_robot_args(_DEFAULTS)
        + declare_rt_torque_args(_DEFAULTS)
        + [
            DeclareLaunchArgument(
                'torque_command_topic',
                default_value=str(_DEFAULTS.get('torque_command_topic', 'torque_cmd')),
                description='Topic for rt_torque_controller'
            ),
            DeclareLaunchArgument(
                'enable_camera',
                default_value='true',
                description='Start RealSense camera driver and TF'
            ),
            OpaqueFunction(function=_launch_all)
        ]
    )