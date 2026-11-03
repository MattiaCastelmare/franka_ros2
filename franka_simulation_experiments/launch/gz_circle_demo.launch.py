"""SIM. The FR3 tracing a circle in the y–z plane, with both paths drawn in Gazebo.

The smallest chain that answers "is the arm following the commanded circle?",
and nothing else: no perception, no CBF, no obstacles, no logging.

    ros2 launch franka_simulation_experiments gz_circle_demo.launch.py

  gz_plant.launch.py (Gazebo + FR3 + RSP + JSB + rt_torque_controller, gui on)
        │
        ├── pentagon_qddot_commander ── /NS_1/qddot_safe ──┬── qddot_to_torque ──
        │     path_type:=circle, plane:=yz                 │     /NS_1/torque_cmd
        │                                                  │          │
        │                                                  └──────────┴──> rt_torque_controller
        │                                                      (τ_ff + Kp·e + Kd·ė)
        └── /NS_1/ee_desired + /NS_1/ee_actual ── gz_trajectory_markers ──> Gazebo /marker

WHY THE COMMANDER PUBLISHES ON ``qddot_safe``
---------------------------------------------
That is the commander's own documented no-CBF wiring (``qddot_safe_topic``):
with no obstacles there is nothing for ``cbf_safety_filter`` to do, and leaving
it out is not a shortcut but the point — it is the node measured at ~18 Hz, and
a starved filter would show up here as tracking error that is not the
commander's. ``qddot_to_torque`` and ``rt_torque_controller`` then read their
default topics, unremapped.

Both are needed: the controller takes τ_ff from ``torque_cmd`` and integrates
its velocity reference from the acceleration topic. Drop ``qddot_to_torque`` and
the controller declares ``torque_cmd`` stale and holds position instead.

WHAT YOU SEE
------------
A **red** line, the Cartesian point the commander is asking for, and a **blue**
line, where the end effector actually is. Both expire after ``trail_seconds``
(2 s, as on the real robot), so they read as two short comets chasing each other
rather than two ever-growing scribbles; pass ``trail_seconds:=0`` to keep the
whole path instead. They are drawn by Gazebo's own MarkerManager, which is a GUI
plugin — hence ``gui:=true`` by default, and nothing is drawn in a headless run.

The commander publishes zeros for ``warmup_s``, then roots the path at the
measured EE position and leads into the circle over ``approach_time``, so the
first red segment is that lead-in and not a chord of the circle.
"""
import os
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_ARGS = {
    'run_dir': ('', 'Run directory (generated world/controller YAML). Empty = a new one'),
    'scene': ('diag', 'rig | bare | diag | diag_bare (simexp.sim_config.SCENES); '
                      'default = the DIAG lab'),
    'gui': ('true', 'Ignition GUI. REQUIRED for the markers: MarkerManager is a GUI plugin'),
    'initial_q': ('0.0, -0.70, 0.0, -2.35, 0.0, 1.57, 0.70', 'Start pose [rad]'),
    'center_xyz': ('0.4, 0.0, 0.45', 'Circle centre in fr3_link0 [m]'),
    'radius': ('0.25', 'Circle radius [m]'),
    'cycle_time': ('6.0', 'Seconds per lap'),
    'plane': ('yz', 'Circle plane: yz | xy | xz | front'),
    'warmup_s': ('2.0', 'Zeros before the trajectory is rooted at the measured EE pose'),
    'approach_time': ('2.5', 'Lead-in from the start pose onto the circle [s]'),
    'trail_seconds': ('2.0', 'How long a trail vertex lives before it expires, as on '
                              'the real robot. 0 disables expiry (keep the whole path)'),
    'min_spacing_m': ('0.002', 'Marker: keep a point only this far from the last kept one'),
    'redraw_hz': ('5.0', 'Marker: polyline resend rate (a call costs ~230 ms, detached)'),
    'node_start_delay_s': ('8.0', 'Wait before the commander starts: the plant spawns the '
                                  'model and activates the controller first'),
}

#: fr3_link0 is the Gazebo world origin (``ros_gz_sim create`` with no -x/-y/-z,
#: and the FR3 URDF roots at ``base``), so the markers need no frame offset.
_FRAME_OFFSET = [0.0, 0.0, 0.0]


def _simexp():
    lib = os.path.join(get_package_prefix('franka_simulation_experiments'), 'lib',
                       'franka_simulation_experiments')
    if lib not in sys.path:
        sys.path.insert(0, lib)
    from simexp import sim_config
    return sim_config


def _floats(text: str) -> list:
    return [float(v) for v in text.replace(',', ' ').split()]


def _launch(context):
    p = {k: LaunchConfiguration(k).perform(context) for k in _ARGS}
    sc = _simexp()
    run_dir = p['run_dir'] or sc.make_run_dir(name='circle_demo')
    os.makedirs(run_dir, exist_ok=True)

    # One partition for the server and for the marker calls: gz_fr3_plant would
    # otherwise invent its own from its pid and `ign service` would not find it.
    partition = os.environ.get('IGN_PARTITION') or f'fr3cbf_{os.getpid()}'
    os.environ['IGN_PARTITION'] = partition

    plant = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('franka_simulation_experiments'),
            'launch', 'gz_plant.launch.py')),
        launch_arguments={'run_dir': run_dir, 'scene': p['scene'], 'gui': p['gui'],
                          'initial_q': p['initial_q'],
                          'spawn_rt_controller': 'true'}.items())

    common = {'use_sim_time': True}
    commander = Node(
        package='franka_experiments', executable='pentagon_qddot_commander',
        name='pentagon_qddot_commander', namespace='NS_1', output='screen',
        parameters=[dict(common,
                         qddot_safe_topic='/NS_1/qddot_safe',
                         path_type='circle',
                         plane=p['plane'],
                         center_xyz=_floats(p['center_xyz']),
                         radius=float(p['radius']),
                         cycle_time=float(p['cycle_time']),
                         warmup_s=float(p['warmup_s']),
                         approach_time=float(p['approach_time']))])
    torque = Node(
        package='franka_experiments', executable='qddot_to_torque',
        name='qddot_to_torque', namespace='NS_1', output='screen',
        parameters=[dict(common)])
    markers = Node(
        package='franka_simulation_experiments', executable='gz_trajectory_markers',
        name='gz_trajectory_markers', output='screen',
        additional_env={'IGN_PARTITION': partition},
        parameters=[dict(common,
                         trail_seconds=float(p['trail_seconds']),
                         min_spacing_m=float(p['min_spacing_m']),
                         redraw_hz=float(p['redraw_hz']),
                         frame_offset_xyz=_FRAME_OFFSET)])

    return [plant, TimerAction(period=float(p['node_start_delay_s']),
                               actions=[commander, torque, markers])]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in _ARGS.items()]
        + [OpaqueFunction(function=_launch)])
