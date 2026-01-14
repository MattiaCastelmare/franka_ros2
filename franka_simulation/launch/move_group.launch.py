#!/usr/bin/env python3

import os
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch_ros.substitutions import FindPackageShare

from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessStart
from launch.events.process import ProcessStarted

from launch.conditions import IfCondition

def load_yaml(package_name, file_path):
    """Utility per caricare file YAML - basata su franka_fr3_moveit_config"""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def make_rsp_and_moveit_nodes(context: LaunchContext, arm_id, load_gripper, franka_hand, enable_moveit):
    """Crea robot_description e nodi MoveIt - basato su franka_fr3_moveit_config/launch/moveit.launch.py"""
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)
    enable_moveit_str = context.perform_substitution(enable_moveit)

    # Path del file XACRO - come in franka_fr3_moveit_config
    xacro_file = os.path.join(
        get_package_share_directory("franka_description"),
        "robots",
        arm_id_str,
        f"{arm_id_str}.urdf.xacro",
    )

    # URDF per Gazebo (con ros2_control)
    urdf_xml = xacro.process_file(
        xacro_file,
        mappings={
            "arm_id": arm_id_str,
            "hand": load_gripper_str,
            "ros2_control": "true",
            "gazebo": "true",
            "ee_id": franka_hand_str,
        },
    ).toxml()

    robot_description = {"robot_description": urdf_xml}

    # Robot State Publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description, {'use_sim_time': True}],
    )

    nodes = [rsp]

    # Se MoveIt è abilitato, aggiungi i nodi MoveIt - pattern da franka_fr3_moveit_config
    if enable_moveit_str.lower() == 'true':
        
        # URDF per MoveIt (senza ros2_control) - pattern da franka_fr3_moveit_config/launch/move_group.launch.py
        robot_description_command = Command(
            [
                FindExecutable(name='xacro'),
                ' ',
                xacro_file,
                ' ros2_control:=false',  # Come in move_group.launch.py linea 68
                ' hand:=',
                load_gripper_str,
                ' arm_id:=',
                arm_id_str,
            ]
        )
        
        robot_description_moveit = {'robot_description': ParameterValue(
            robot_description_command, value_type=str)}

        # SRDF semantico - pattern esatto da franka_fr3_moveit_config/launch/moveit.launch.py
        franka_semantic_xacro_file = os.path.join(
            get_package_share_directory('franka_description'),
            'robots', arm_id_str, f'{arm_id_str}.srdf.xacro'
        )

        robot_description_semantic_command = Command(
            [FindExecutable(name='xacro'), ' ',
             franka_semantic_xacro_file, ' hand:=', load_gripper_str]
        )

        robot_description_semantic = {'robot_description_semantic': ParameterValue(
            robot_description_semantic_command, value_type=str)}

        # Configurazioni MoveIt - usando franka_fr3_moveit_config come riferimento
        kinematics_yaml = load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')
        
        # Planning pipeline OMPL - pattern esatto da franka_fr3_moveit_config/launch/moveit.launch.py
        ompl_planning_pipeline_config = {
            'move_group': {
                'planning_plugin': 'ompl_interface/OMPLPlanner',
                'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                    'default_planner_request_adapters/ResolveConstraintFrames '
                                    'default_planner_request_adapters/FixWorkspaceBounds '
                                    'default_planner_request_adapters/FixStartStateBounds '
                                    'default_planner_request_adapters/FixStartStateCollision '
                                    'default_planner_request_adapters/FixStartStatePathConstraints',
                'start_state_max_bounds_error': 0.1,
            }
        }
        ompl_planning_yaml = load_yaml('franka_fr3_moveit_config', 'config/ompl_planning.yaml')
        if ompl_planning_yaml:
            ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

        # Controller per MoveIt - usando franka_fr3_moveit_config
        moveit_simple_controllers_yaml = load_yaml('franka_fr3_moveit_config', 'config/fr3_controllers.yaml')
        moveit_controllers = {
            'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
            'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        }

        # Trajectory execution - pattern da franka_fr3_moveit_config/launch/moveit.launch.py
        trajectory_execution = {
            'moveit_manage_controllers': False,
            'trajectory_execution.allowed_execution_duration_scaling': 1.2,
            'trajectory_execution.allowed_goal_duration_margin': 0.5,
            'trajectory_execution.allowed_start_tolerance': 0.01,
        }

        # Planning scene monitor - pattern da franka_fr3_moveit_config/launch/moveit.launch.py
        planning_scene_monitor_parameters = {
            'publish_planning_scene': True,
            'publish_geometry_updates': True,
            'publish_state_updates': True,
            'publish_transforms_updates': True,
            'publish_octomap': False,  # Disabilita (no 3D sensor disponibile)
            'default_robot_padding': 0.04,     
            'default_object_padding': 0.1,    
            'default_attached_padding': 0.02
        }

        # MoveIt move_group node - pattern esatto da franka_fr3_moveit_config/launch/moveit.launch.py
        move_group_node = Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                robot_description_moveit,
                robot_description_semantic,
                kinematics_yaml,
                ompl_planning_pipeline_config,
                trajectory_execution,
                moveit_controllers,
                planning_scene_monitor_parameters,
                {'use_sim_time': True},
            ],
        )
        
        nodes.append(move_group_node)

    return nodes


def make_rviz_node(context: LaunchContext, arm_id, load_gripper, enable_moveit):
    """Crea nodo RViz con parametri MoveIt se abilitato"""
    enable_moveit_value = context.perform_substitution(enable_moveit)
    
    # Usa la configurazione RViz di MoveIt se abilitata
    if enable_moveit_value.lower() == 'true':
        rviz_file = os.path.join(
            get_package_share_directory("franka_fr3_moveit_config"), "rviz", "moveit.rviz"
        )
    else:
        # Fallback alla configurazione di simulazione se disponibile
        try:
            rviz_file = os.path.join(
                get_package_share_directory("franka_simulation"), "rviz", "franka_simulation.rviz"
            )
        except:
            # Default minimale
            rviz_file = None
    
    rviz_params = []
    if enable_moveit_value.lower() == 'true':
        # Aggiungi parametri MoveIt per RViz - stesso pattern del move_group
        arm_id_str = context.perform_substitution(arm_id)
        load_gripper_str = context.perform_substitution(load_gripper)
        
        xacro_file = os.path.join(
            get_package_share_directory("franka_description"),
            "robots", arm_id_str, f"{arm_id_str}.urdf.xacro",
        )
        
        robot_description_command = Command([
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' ros2_control:=false', ' hand:=', load_gripper_str, ' arm_id:=', arm_id_str,
        ])
        
        franka_semantic_xacro_file = os.path.join(
            get_package_share_directory('franka_description'),
            'robots', arm_id_str, f'{arm_id_str}.srdf.xacro'
        )
        
        robot_description_semantic_command = Command([
            FindExecutable(name='xacro'), ' ', franka_semantic_xacro_file, 
            ' hand:=', load_gripper_str
        ])
        
        # Configurazioni MoveIt per RViz
        kinematics_yaml = load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')
        ompl_planning_yaml = load_yaml('franka_fr3_moveit_config', 'config/ompl_planning.yaml')
        
        ompl_planning_pipeline_config = {
            'move_group': {
                'planning_plugin': 'ompl_interface/OMPLPlanner',
                'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                    'default_planner_request_adapters/ResolveConstraintFrames '
                                    'default_planner_request_adapters/FixWorkspaceBounds '
                                    'default_planner_request_adapters/FixStartStateBounds '
                                    'default_planner_request_adapters/FixStartStateCollision '
                                    'default_planner_request_adapters/FixStartStatePathConstraints',
                'start_state_max_bounds_error': 0.1,
            }
        }
        if ompl_planning_yaml:
            ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)
        
        rviz_params = [
            {'robot_description': ParameterValue(robot_description_command, value_type=str)},
            {'robot_description_semantic': ParameterValue(robot_description_semantic_command, value_type=str)},
            kinematics_yaml,
            ompl_planning_pipeline_config,
            {'use_sim_time': True},

        ]
    
    if rviz_file:
        return [Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["--display-config", rviz_file, "-f", "fr3_link0"],
            parameters=rviz_params,
            output="screen",
        )]
    else:
        # RViz senza configurazione specifica
        return [Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-f", "fr3_link0"],
            parameters=rviz_params + [{'use_sim_time': True}],
            output="screen",
        )]


def generate_launch_description():
    # Argomenti - MoveIt abilitato di default
    load_gripper_arg = DeclareLaunchArgument("load_gripper", default_value="true")
    franka_hand_arg = DeclareLaunchArgument("franka_hand", default_value="franka_hand")
    arm_id_arg = DeclareLaunchArgument("arm_id", default_value="fr3")
    enable_moveit_arg = DeclareLaunchArgument("enable_moveit", default_value="true", 
                                              description="Enable MoveIt integration")
    spawn_obstacles_arg = DeclareLaunchArgument('spawn_obstacles',default_value='true',
                                              description='Spawn collision obstacles in simulation')
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Enable RealSense + image pipeline (realsense2_camera -> image_publisher -> human_pose_node)',
    )
    spawn_obstacles = LaunchConfiguration("spawn_obstacles")

    enable_camera = LaunchConfiguration('enable_camera')


    load_gripper = LaunchConfiguration("load_gripper")
    franka_hand = LaunchConfiguration("franka_hand")
    arm_id = LaunchConfiguration("arm_id")
    enable_moveit = LaunchConfiguration("enable_moveit")

    # RealSense camera driver (optional)
    # NOTE: Use FindPackageShare/PathJoinSubstitution so the launch file can still be used
    # even if realsense2_camera is not installed, as long as enable_camera:=false.
    realsense_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py',
            ])
        ),
        condition=IfCondition(enable_camera),
    )

    # Image pipeline nodes (optional)
    # Scripts are installed by CMake into lib/franka_simulation with names without .py.
    image_republisher = Node(
        package='franka_simulation',
        executable='image_publisher',
        name='image_republisher',
        output='screen',
        condition=IfCondition(enable_camera),
    )

    human_pose_node = Node(
        package='franka_simulation',
        executable='human_pose_node',
        name='human_pose_node',
        output='screen',
        condition=IfCondition(enable_camera),
    )

    # Ordered startup: driver -> republisher -> pose
    delayed_image_republisher = TimerAction(
        period=2.0,
        actions=[image_republisher],
        condition=IfCondition(enable_camera),
    )
    delayed_human_pose = TimerAction(
        period=3.0,
        actions=[human_pose_node],
        condition=IfCondition(enable_camera),
    )

    # RSP + MoveIt nodes (ritardati per evitare warning TF)
    robot_nodes_delayed = TimerAction(period=2.0, actions=[
        OpaqueFunction(
            function=make_rsp_and_moveit_nodes, 
            args=[arm_id, load_gripper, franka_hand, enable_moveit]
        )
    ])
    obstacle_sync = Node(
        package='franka_simulation',
        executable='obstacle_synchronizer',
        name='obstacle_synchronizer',
        output='screen',
        parameters=[{
            'obstacles_namespace': '/obstacle',
            'update_rate': 2.0,
            'reference_frame': 'base',
            # ✅ percorso del file .xacro da cui leggere automaticamente
            'urdf_xacro_path': os.path.join(
                get_package_share_directory('franka_simulation'),
                'urdf', 'obstacles', 'multi_obstacle_scene.urdf.xacro'
            ),
        }],
        condition=IfCondition(spawn_obstacles)
    )
    delayed_obstacle_sync = TimerAction(
        period=3.0,  # o quanto vuoi dopo l'avvio del move_group
        actions=[obstacle_sync],
        condition=IfCondition(spawn_obstacles)
    )
        # Spawn dell'ostacolo in Gazebo
    spawn_obstacle = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_obstacle',
        arguments=[
            '-name', 'collision_obstacle',
            '-topic', '/obstacle/robot_description',
        ],
        output='screen'
    )
    delayed_spawn_obstacle = TimerAction(
        period=4.5,
        actions=[spawn_obstacle],
        condition=IfCondition(spawn_obstacles)
    )


    # Gazebo (Ignition/GZ) mondo vuoto
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.path.dirname(
        get_package_share_directory("franka_description")
    )
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "empty.sdf -r"}.items(),
    )

    # Spawn del robot in Gazebo dal topic /robot_description
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "/robot_description"],
        output="screen",
    )

    obstacle_urdf_path = os.path.join(
        get_package_share_directory('franka_simulation'),
        'urdf', 'obstacles', 'multi_obstacle_scene.urdf.xacro'
    )

    obstacle_robot_description = Command([FindExecutable(name='xacro'), ' ', obstacle_urdf_path])

    # Robot State Publisher per l'ostacolo (pubblica i TF)
    obstacle_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='obstacle_state_publisher',
        namespace='obstacle',
        parameters=[{
            'robot_description': ParameterValue(obstacle_robot_description, value_type=str),
            'publish_frequency': 30.0,
            'frame_prefix': 'obstacle/',
        }],
        output='screen'
    )
    
    
    # Timer per spawnare l'ostacolo dopo il robot
    delayed_obstacle_rsp = TimerAction(
        period=4.0,
        actions=[obstacle_rsp],
        condition=IfCondition(spawn_obstacles)
    )
   

    # Spawner dei controller (parlano a /controller_manager dentro Gazebo)
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_joint_state_broadcaster",
        arguments=[
            "joint_state_broadcaster", 
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_fr3_arm_controller",
        arguments=[
            "fr3_arm_controller",
            "--inactive",
            "--controller-manager", "/controller_manager",
            "--param-file",
            os.path.join(
                get_package_share_directory("franka_simulation"),
                "config", "fr3_arm_controller.yaml"
            ),
        ],
        output="screen",
    )

    vel_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_fr3_velocity_controller",
        arguments=[
            "fr3_velocity_controller",
            "--controller-manager", "/controller_manager",
            "--param-file",
            os.path.join(
                get_package_share_directory("franka_simulation"),
                "config",
                "fr3_velocity_controller.yaml"
            ),
        ],
        output="screen",
    )



    # Static TF publisher: world -> base (new-style args)
    static_tf_world_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base_broadcaster",
        arguments=["--x", "0", "--y", "0", "--z", "0", 
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "world", "--child-frame-id", "base"]
    )

    # Static TF publisher: base -> fr3_link0 (new-style args)
    static_tf_base_link0 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_link0_broadcaster",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "base", "--child-frame-id", "fr3_link0"]
    )
    
    # Static TF publisher: world -> obstacle/world (new-style args)
    static_tf_obstacle_world = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_obstacle_world_broadcaster",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "world", "--child-frame-id", "obstacle/world"],
        condition=IfCondition(spawn_obstacles)
    )
    clock_bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    name="clock_bridge",
    arguments=["/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock"],
    output="screen",
    )

    # RViz con parametri MoveIt se abilitato
    rviz_node = OpaqueFunction(
        function=make_rviz_node,
        args=[arm_id, load_gripper, enable_moveit]
    )


    motion_server = Node(
        package='franka_simulation',
        executable='franka_motion_server',
        name='franka_motion_server',
        output='screen',
        parameters=[{
            'move_group_name': 'fr3_arm',
            'base_link_name': 'fr3_link0',
            'end_effector_name': 'fr3_hand_tcp',
            'planner_id': 'RRTstar',
            'allowed_planning_time': 5.0,
            'max_velocity_scaling_factor': 0.1,
            'max_acceleration_scaling_factor': 0.1,
            'max_motion_retries': 3,
            'max_ik_retries': 5,
            'ik_timeout': 10.0
        }]
    )

    # Parametri da config/avoidance_params.yaml
    avoidance_params_file = os.path.join(
        get_package_share_directory('franka_simulation'),
        'config', 'avoidance_params.yaml'
    )
    
    online_avoidance = Node(
        package='franka_simulation',
        executable='online_avoidance_controller',
        name='online_avoidance_controller',
        output='screen',
        parameters=[avoidance_params_file]
    )

    velocity_control_blender = Node(
        package='franka_simulation',
        executable='velocity_control_blender',
        name='velocity_control_blender',
        output='screen'
    )

    # Sequenza temporizzata: spawn -> JSB -> ARM -> RViz (dopo move_group)
    delayed_spawn = TimerAction(period=3.0, actions=[spawn])
    delayed_jsb = TimerAction(period=5.0, actions=[jsb_spawner])
    delayed_arm = TimerAction(period=7.0, actions=[arm_spawner])
    delayed_vel = TimerAction(period=8.0, actions=[vel_spawner])

    delayed_rviz = TimerAction(period=10.0, actions=[rviz_node])  # Ritardato per dare tempo a move_group
    delayed_avoidance = TimerAction(period=11.0, actions=[online_avoidance])
    delayed_motion_server = TimerAction(period=30.0, actions=[motion_server])
    motion_server_action = OpaqueFunction(function=lambda context: [motion_server])

    # File dei parametri del velocity blender
    velocity_blender_params_file = os.path.join(
        get_package_share_directory('franka_simulation'),
        'config',
        'velocity_blender_params.yaml'
    )

    delayed_blender = TimerAction(
        period=13.0,
        actions=[
            Node(
                package='franka_simulation',
                executable='velocity_control_blender',
                name='velocity_control_blender',
                output='screen',
                parameters=[velocity_blender_params_file]
            )
        ]
    )




    return LaunchDescription(
        [
            load_gripper_arg,
            franka_hand_arg,
            arm_id_arg,
            spawn_obstacles_arg,
            enable_moveit_arg,
            enable_camera_arg,
            static_tf_world_base,
            static_tf_base_link0,
            static_tf_obstacle_world,
            realsense_driver,
            gz,
            robot_nodes_delayed,
            delayed_spawn,
            delayed_jsb,
            #delayed_arm,
            delayed_vel,
            delayed_obstacle_rsp,
            delayed_spawn_obstacle,
            delayed_rviz,
            clock_bridge,
            delayed_obstacle_sync,
            delayed_motion_server,
            #motion_server_action,
            delayed_avoidance,
            delayed_blender,
            delayed_image_republisher,
            delayed_human_pose,
        ]
    )