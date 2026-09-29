#!/usr/bin/env python3

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
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from franka_experiments.utils.ros import (
    load_franka_config_defaults,
    load_launch_defaults,
    pick_controllers_yaml,
    resolve_controller_manager_name,
)


# =============================================================================
# DEFAULTS
# =============================================================================

_LAUNCH_DEFAULTS, _ = load_launch_defaults()
_BRINGUP_DEFAULTS, _ = load_franka_config_defaults()

_DEFAULTS = {
    **_LAUNCH_DEFAULTS,
    **_BRINGUP_DEFAULTS,
}


# =============================================================================
# CONTROL TOPICS
# =============================================================================

_QDDOT_NOM_TOPIC = '/NS_1/qddot_nom'
_TORQUE_CONTROLLER = 'rt_torque_controller'


def _as_bool(value) -> bool:
    return str(value).strip().lower() in (
        '1', 'true', 'yes', 'y', 'on'
    )


def _poll(name: str, test: str, timeout_s: str) -> ExecuteProcess:
    """
    Wait until a ROS condition becomes true.

    Same startup philosophy used by the already working
    workspace_gripper_cycle launch.
    """
    return ExecuteProcess(
        name=name,
        cmd=[
            'timeout',
            str(timeout_s),
            'bash',
            '-c',
            f'until {test}; do sleep 1; done',
        ],
        output='screen',
    )


# =============================================================================
# MAIN SETUP
# =============================================================================

def _launch_setup(context):

    def p(key):
        return LaunchConfiguration(key).perform(context)

    ns = p('namespace').strip('/')
    cm = resolve_controller_manager_name(ns)

    use_fake = _as_bool(p('use_fake_hardware'))
    timeout_s = p('timeout_s')

    start_rviz = _as_bool(p('start_rviz'))
    start_logger = _as_bool(p('start_logger'))


    # =========================================================================
    # 1. RT TORQUE CONTROLLER CONFIGURATION
    # =========================================================================
    #
    # IMPORTANT:
    #
    # NO CBF in handover mode.
    #
    # The same nominal acceleration is used by:
    #
    #   qddot_to_torque
    #   rt_torque_controller watchdog
    #
    # Therefore:
    #
    #   accel_topic = /NS_1/qddot_nom
    #

    rt_params = dict(
        is_real=not use_fake,
        arm_id=p('arm_id'),

        controller_type='torque',

        torque_command_topic=p('torque_command_topic'),

        gazebo=p('gazebo'),

        lpf_alpha=float(p('lpf_alpha')),
        tau_max_scale=float(p('tau_max_scale')),

        # CRITICAL for handover without CBF
        accel_topic=_QDDOT_NOM_TOPIC,
    )

    controllers_yaml = pick_controllers_yaml(
        p('controllers_yaml'),
        use_fake,
        rt_params,
    )


    # =========================================================================
    # 2. REAL FR3 BRINGUP
    # =========================================================================

    franka = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('franka_bringup'),
                'launch',
                'franka.launch.py',
            ]).perform(context)
        ),
        launch_arguments={
            'arm_id': p('arm_id'),
            'robot_ip': p('robot_ip'),
            'namespace': ns,

            'use_fake_hardware': p('use_fake_hardware'),
            'fake_sensor_commands': p('fake_sensor_commands'),

            'load_gripper': p('load_gripper'),

            # YAML generated above with:
            # accel_topic=/NS_1/qddot_nom
            'controllers_yaml': controllers_yaml,
        }.items(),
    )


    # =========================================================================
    # 3. REALSENSE
    # =========================================================================

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py',
            ]).perform(context)
        ),
        launch_arguments={
            'align_depth.enable': 'true',
            # Needed with the D405 also plugged in, otherwise the driver may
            # open the D405 here instead of the D455.
            'serial_no': '_318122300288',
        }.items(),
    )

    # Wrist D405 on a USB 2 extension: 640x480x30 colour+depth fits,
    # 848x480x30 and infra streams do not. Same setup as minimal.launch.py.
    realsense_wrist = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py',
            ]).perform(context)
        ),
        launch_arguments={
            'camera_namespace': 'd405',
            'camera_name': 'd405',
            'serial_no': '_126122270738',
            'align_depth.enable': 'true',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'depth_module.color_profile': '640x480x30',
            'depth_module.depth_profile': '640x480x30',
        }.items(),
    )


    # =========================================================================
    # 4. BASE FRAME ALIAS
    # =========================================================================

    base_alias_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fr3_link0_to_hand_base_tf',
        output='log',

        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',

            '--qx', '0',
            '--qy', '0',
            '--qz', '0',
            '--qw', '1',

            '--frame-id', 'fr3_link0',
            '--child-frame-id', 'base',
        ],
    )


    # =========================================================================
    # 5. HAND TRACKER
    # =========================================================================

    tracker = Node(
        package='franka_experiments',
        executable='human_hand_tracker',
        output='screen',

        parameters=[{
            'use_sim_time': False,

            'publish_debug_image': True,
            'show_selected_landmarks': True,

            'model_complexity': 0,
            'static_image_mode': False,

            'min_tracking_confidence': 0.5,
            'min_detection_confidence': 0.4,
        }],
    )


    # =========================================================================
    # 6. LANDMARK KALMAN
    # =========================================================================

    kalman = Node(
        package='franka_experiments',
        executable='kalman_hand',
        output='screen',

        parameters=[{
            'use_sim_time': False,
        }],
    )


    # =========================================================================
    # 7. HAND STATE ESTIMATOR
    # =========================================================================

    estimator = Node(
        package='franka_experiments',
        executable='hand_state_estimator',
        output='screen',

        parameters=[{
            'use_sim_time': False,
            'velocity_mode': 'w75',
        }],
    )


    # =========================================================================
    # 8. END-EFFECTOR STATE (FK + J(q)qdot)
    # =========================================================================

    end_effector_state = Node(
        package='franka_experiments',
        executable='end_effector_state',
        output='screen',

        parameters=[{
            'use_sim_time': False,
        }],
    )


    # =========================================================================
    # 9. HAND <-> ROBOT RELATIVE KINEMATICS
    # =========================================================================

    handover_distance = Node(
        package='franka_experiments',
        executable='distance_handover_estimator',
        name='distance_handover_estimator',
        output='screen',

        parameters=[{
            'use_sim_time': False,
        }],
    )


    # =========================================================================
    # 9. HANDOVER OBSERVER
    # =========================================================================

    handover_observer = Node(
        package='franka_experiments',
        executable='handover_observer',
        output='screen',

        parameters=[{
            'use_sim_time': False,
        }],
    )


    # =========================================================================
    # 10. VISUAL DEBUG
    # =========================================================================

    compare_visualizer = Node(
        package='franka_experiments',
        executable='hand_compare_visualizer',
        output='screen',

        condition=IfCondition(
            LaunchConfiguration('start_rviz')
        ),

        parameters=[{
            'use_sim_time': False,
        }],
    )


    # =========================================================================
    # 11. LOGGER
    # =========================================================================

    logger = Node(
        package='franka_experiments',
        executable='hand_logger',
        output='screen',

        condition=IfCondition(
            LaunchConfiguration('start_logger')
        ),

        parameters=[{
            'use_sim_time': False,
        }],
    )


    # =========================================================================
    # 12. RVIZ
    # =========================================================================

    rviz_config = PathJoinSubstitution([
        FindPackageShare('franka_experiments'),
        'config',
        'hand_tracker.rviz',
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='handover_rviz',
        output='screen',

        condition=IfCondition(
            LaunchConfiguration('start_rviz')
        ),

        arguments=[
            '-d',
            rviz_config,
        ],

        parameters=[{
            'use_sim_time': False,
        }],

        additional_env={
            '__NV_PRIME_RENDER_OFFLOAD': '1',
            '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
        },
    )


    # =========================================================================
    # 13. PERCEPTION PIPELINE START
    # =========================================================================

    perception_pipeline = TimerAction(
        period=2.0,

        actions=[
            tracker,
            kalman,
            estimator,
            end_effector_state,

            handover_distance,
            handover_observer,

            compare_visualizer,
            logger,
            rviz,
        ],
    )


    # =========================================================================
    # 14. QDDOT -> TORQUE
    # =========================================================================
    #
    # Existing node expects:
    #
    #   /NS_1/qddot_safe
    #
    # For handover there is NO CBF, therefore:
    #
    #   /NS_1/qddot_nom -> qddot_to_torque
    #

    qddot_to_torque = Node(
        package='franka_experiments',
        executable='qddot_to_torque',

        name='qddot_to_torque',
        namespace=ns or None,

        output='screen',

        remappings=[
            (
                '/NS_1/qddot_safe',
                _QDDOT_NOM_TOPIC,
            ),
        ],
    )


    # =========================================================================
    # 15. RT TORQUE CONTROLLER SPAWNER
    # =========================================================================

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',

        arguments=[
            _TORQUE_CONTROLLER,

            '--controller-manager',
            cm,

            '--controller-manager-timeout',
            timeout_s,
        ],

        output='screen',
    )


    # =========================================================================
    # 16. RT THREAD PINNING
    # =========================================================================

    rt_pin_cpu = str(p('rt_pin_cpu')).strip()

    pin_rt_thread = None

    if rt_pin_cpu and not use_fake:

        pin_rt_thread = ExecuteProcess(
            name='pin_rt_torque_thread',

            cmd=[
                'bash',

                PathJoinSubstitution([
                    FindPackageShare('franka_experiments'),
                    'scripts',
                    'pin_rt_thread.sh',
                ]),

                rt_pin_cpu,
                '60',
            ],

            output='screen',
        )


    # =========================================================================
    # 17. HANDOVER COMMANDER
    # =========================================================================
    #
    # For now:
    #
    #   follow_hand = false
    #
    # therefore the robot starts in hold/test-offset mode.
    #

    handover_commander = ExecuteProcess(
        name='handover_qddot_commander_process',

        cmd=[
            'python3',

            '/ros2_ws/src/franka_experiments/scripts/'
            'handover_qddot_commander.py',

            '--ros-args',

            # The Python class inherits the Pentagon commander,
            # therefore we explicitly rename the ROS node here.
            '-r',
            '__node:=handover_qddot_commander',
        ],

        output='screen',
    )


    # =========================================================================
    # 18. STARTUP GATES
    # =========================================================================

    wait_cm = _poll(
        'wait_controller_manager',

        (
            f'ros2 service list 2>/dev/null '
            f'| grep -q "^{cm}/list_controllers$"'
        ),

        timeout_s,
    )


    wait_torque = _poll(
        'wait_rt_torque_controller',

        (
            f'ros2 control list_controllers '
            f'--controller-manager {cm} 2>/dev/null '
            f'| grep -q "{_TORQUE_CONTROLLER}.*active"'
        ),

        timeout_s,
    )


    # =========================================================================
    # 19. ACTION ORDER
    # =========================================================================

    actions = [

        LogInfo(
            msg=[
                '[handover] namespace=', ns or '<none>',
                '  robot_ip=', p('robot_ip'),
                '  qddot=', _QDDOT_NOM_TOPIC,
                '  torque_topic=', p('torque_command_topic'),
                '  CBF=DISABLED',
            ]
        ),

        # Hardware
        franka,

        # Cameras
        realsense,
        realsense_wrist,

        # TF compatibility
        base_alias_tf,

        # Perception
        perception_pipeline,

        # Wait until ros2_control is alive
        wait_cm,
    ]


    # Once controller_manager exists:
    #
    # - start qddot_to_torque
    # - spawn rt_torque_controller
    # - start RT pinning
    #

    after_cm = [
        qddot_to_torque,
        controller_spawner,
    ]

    if pin_rt_thread is not None:
        after_cm.append(pin_rt_thread)

    actions.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_cm,
                on_exit=after_cm,
            )
        )
    )


    # Once spawner finishes, wait for controller to actually be ACTIVE.
    actions.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=controller_spawner,
                on_exit=[
                    wait_torque,
                ],
            )
        )
    )


    # Only when torque controller is ACTIVE:
    #
    # start the handover commander.
    actions.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_torque,
                on_exit=[
                    handover_commander,
                ],
            )
        )
    )


    return actions


# =============================================================================
# LAUNCH DESCRIPTION
# =============================================================================

def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace',
            default_value=_DEFAULTS.get(
                'namespace',
                'NS_1',
            ),
        ),

        DeclareLaunchArgument(
            'arm_id',
            default_value=_DEFAULTS.get(
                'arm_id',
                'fr3',
            ),
        ),

        DeclareLaunchArgument(
            'robot_ip',
            default_value=_DEFAULTS.get(
                'robot_ip',
                '192.168.2.10',
            ),
        ),

        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value=_DEFAULTS.get(
                'use_fake_hardware',
                'false',
            ),
        ),

        DeclareLaunchArgument(
            'fake_sensor_commands',
            default_value=_DEFAULTS.get(
                'fake_sensor_commands',
                'false',
            ),
        ),

        DeclareLaunchArgument(
            'load_gripper',
            default_value='false',
        ),

        DeclareLaunchArgument(
            'controllers_yaml',
            default_value=_DEFAULTS.get(
                'controllers_yaml',
                '',
            ),
        ),

        DeclareLaunchArgument(
            'gazebo',
            default_value=_DEFAULTS.get(
                'gazebo',
                'false',
            ),
        ),

        DeclareLaunchArgument(
            'torque_command_topic',
            default_value=_DEFAULTS.get(
                'torque_command_topic',
                'torque_cmd',
            ),
        ),

        DeclareLaunchArgument(
            'lpf_alpha',
            default_value=str(
                _DEFAULTS.get(
                    'lpf_alpha',
                    '1.0',
                )
            ),
        ),

        DeclareLaunchArgument(
            'tau_max_scale',
            default_value=str(
                _DEFAULTS.get(
                    'tau_max_scale',
                    '1.0',
                )
            ),
        ),

        DeclareLaunchArgument(
            'rt_pin_cpu',
            default_value=str(
                _DEFAULTS.get(
                    'rt_pin_cpu',
                    '3',
                )
            ),
        ),

        DeclareLaunchArgument(
            'timeout_s',
            default_value='60',
        ),

        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
        ),

        DeclareLaunchArgument(
            'start_logger',
            default_value='true',
        ),

        OpaqueFunction(
            function=_launch_setup
        ),
    ])


if __name__ == '__main__':
    generate_launch_description()
