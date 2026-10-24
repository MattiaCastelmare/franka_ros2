"""Replay a recorded run through the SHIPPED perception + CBF, with no robot.

What it is for: changing a perception or CBF parameter and seeing, on the SAME
recorded ball throws, whether the ball now gets a track, whether its velocity
is trusted sooner, and how much earlier the filter acts — before trying it on
the arm. Score the output with scripts/ball_throw_eval.py.

What runs
---------
  ros2 bag play --clock   raw depth + camera_info, /tf, /tf_static,
                          joint states, /NS_1/qddot_nom       (from the bag)
  real_time_distance      →  /cbf/per_link_distances          (sim time)
  cbf_safety_filter       →  /NS_1/qddot_safe, /NS_1/cbf_status (sim time)
  ros2 bag record         the four outputs above + qddot_nom → out:=

Both nodes get exactly the parameters torque_control_stack gives them (the
same helpers build them), so a replay differs from the live stack only in
where its inputs come from.

OPEN LOOP. q and q̇ are the recorded ones: the arm in the replay never moves
out of the way, and nothing downstream of q̈_safe runs. What a replay measures
is WHEN the filter starts to act and how hard, not whether the ball would have
been dodged.

The bag must hold the RAW depth stream (/camera/camera/depth/image_rect_raw,
recorded since 2026-09-30); older bags carrying only the aligned stream are
replayed from it with a warning, which is a different resolution and rate than
the robot sees.

Usage (inside the container, after colcon build)
------------------------------------------------
    ros2 launch franka_experiments bag_replay.launch.py \\
        bag:=/ros2_ws/src/franka_experiments/rosbag/ball_throws_2 \\
        out:=/ros2_ws/src/franka_experiments/rosbag/replay_baseline
    python3 scripts/ball_throw_eval.py score rosbag/replay_baseline \\
        --truth rosbag/ball_throws_2_truth.npz
"""

import importlib.util
import os
import time

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, OpaqueFunction,
    RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The live stack's own helpers, so the two cannot drift apart.
_spec = importlib.util.spec_from_file_location(
    'torque_control_stack',
    os.path.join(os.path.dirname(os.path.abspath(__file__)), 'torque_control_stack.launch.py'))
_tcs = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_tcs)

_OWN = {
    'bag': ('', 'rosbag2 directory to replay (required)'),
    'out': ('', 'output bag directory (default: ./replay_<time>)'),
    'rate': ('1.0', 'playback rate; keep 1.0 — both nodes run in real time'),
    'start_offset': ('0.0', 'seconds into the bag to start'),
    'play_delay_s': ('8.0', 'wait for the nodes (trimesh, Pinocchio) before playing'),
    'perception_overrides': ('', 'YAML merged (nested) into the perception config for this replay'),
    'cbf_overrides': ('', 'flat YAML {param: value} passed to cbf_safety_filter as ROS parameters'),
}
_PASSED = ['robot_config_yaml', 'camera_extrinsics_yaml', 'camera_depth_profile',
           'obstacle_tracking', 'multi_obstacle_k', 'obstacle_velocity_source',
           'lateral_evasion', 'outrun_evasion', 'livelock_escape', 'latency_compensation',
           'uncertainty_margin', 'zone_ladder', 'vobs_in_hdot', 'velocity_standoff',
           'iso_enabled', 'iso_mode', 'iso_ssm_speed_rows', 'iso_monitor_enabled',
           'link_speed_max', 'retreat_cap_max_speed',
           'obstacle_velocity_normal_guard', 'obstacle_identity_guard']


def _deep_merge(dst, src):
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            _deep_merge(dst[k], v)
        else:
            dst[k] = v
    return dst


def _with_perception_overrides(path, overrides_path):
    """A copy of the perception config with an override YAML merged in."""
    import tempfile
    with open(path) as f:
        cfg = yaml.safe_load(f)
    with open(overrides_path) as f:
        _deep_merge(cfg, yaml.safe_load(f) or {})
    out = os.path.join(tempfile.gettempdir(), f'fr3_complete_replay_{os.getpid()}.yaml')
    with open(out, 'w') as f:
        yaml.safe_dump(cfg, f, sort_keys=False)
    return out


def _control_topics():
    from franka_experiments.utils.config import load_package_yaml
    return load_package_yaml('franka_experiments', 'config/fr3_control.yaml').get('topics', {})


def _replay(context):
    p = {k: LaunchConfiguration(k).perform(context) for k in list(_OWN) + _PASSED}
    bag = p['bag'].rstrip('/')
    if not bag or not os.path.isdir(bag):
        raise RuntimeError(f'bag:= must be a rosbag2 directory, got {bag!r}')
    out = p['out'] or os.path.abspath(time.strftime('replay_%Y%m%d_%H%M%S'))
    if os.path.exists(out):
        raise RuntimeError(f'out:={out} already exists — ros2 bag record would refuse it')
    in_bag = _tcs._bag_topics(bag)

    with open(p['robot_config_yaml']) as f:
        ptopics = (yaml.safe_load(f) or {}).get('topics', {}) or {}
    depth = ptopics.get('depth_image', '/camera/camera/depth/image_rect_raw')
    info = ptopics.get('depth_camera_info', '/camera/camera/depth/camera_info')
    src_depth, src_info = _tcs._depth_source_topics(bag, depth, info)

    ctopics = _control_topics()
    js_fast = ctopics.get('joint_states_fast', ctopics.get('joint_states_topic'))
    js_slow = ctopics.get('joint_states_topic', '/NS_1/joint_states')
    js_src = js_fast if js_fast in in_bag else js_slow
    nom = ctopics.get('qddot_nom', '/NS_1/qddot_nom')
    missing = [t for t in (src_depth, src_info, '/tf', '/tf_static', js_src, nom) if t not in in_bag]

    remaps = []
    if (src_depth, src_info) != (depth, info):
        remaps += [f'{src_depth}:={depth}', f'{src_info}:={info}']
    if js_src != js_fast:
        remaps.append(f'{js_src}:={js_fast}')

    actions = [LogInfo(msg=f'[bag_replay] {bag} -> {out}')]
    if src_depth != depth:
        actions.append(LogInfo(msg='[bag_replay] WARNING: no raw depth in the bag, replaying the '
                                   'ALIGNED stream (colour resolution and rate, not what the robot sees)'))
    if js_src != js_fast:
        actions.append(LogInfo(msg=f'[bag_replay] joint states from {js_src} (the fast topic '
                                   f'{js_fast} was not recorded) — lower rate than live'))
    if missing:
        actions.append(LogInfo(msg=f'[bag_replay] WARNING: not in the bag: {missing}'))

    rtd_config = _tcs._rtd_config_with_overrides(
        p['robot_config_yaml'], tracking=_tcs._as_bool(p['obstacle_tracking']),
        sim_obstacle=False, depth_rate_hz=_tcs._profile_fps(p['camera_depth_profile']),
        visualize=False)
    if p['perception_overrides']:
        rtd_config = _with_perception_overrides(rtd_config, p['perception_overrides'])
        actions.append(LogInfo(msg=f'[bag_replay] perception overrides: {p["perception_overrides"]}'))
    cbf_extra = []
    if p['cbf_overrides']:
        with open(p['cbf_overrides']) as f:
            cbf_extra = [yaml.safe_load(f) or {}]
        actions.append(LogInfo(msg=f'[bag_replay] cbf overrides: {cbf_extra[0]}'))
    sim = {'use_sim_time': True}
    rtd = Node(package='franka_experiments', executable='real_time_distance',
               name='real_time_distance', output='log',
               additional_env=_tcs._SINGLE_THREAD_BLAS,
               parameters=[{'robot_config_path': rtd_config,
                            'camera_extrinsics_path': p['camera_extrinsics_yaml'],
                            'multi_obstacle_k': int(p['multi_obstacle_k']),
                            'publish_overlay_image': False}, sim])
    cbf = Node(package='franka_experiments', executable='cbf_safety_filter',
               name='cbf_safety_filter', output='both',
               additional_env=_tcs._SINGLE_THREAD_BLAS,
               parameters=[*_tcs._cbf_parameters(p), *cbf_extra, sim])
    record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--use-sim-time', '-o', out,
             ptopics.get('per_link_distances', '/cbf/per_link_distances'),
             ctopics.get('qddot_safe', '/NS_1/qddot_safe'), nom,
             ctopics.get('cbf_status', '/NS_1/cbf_status')],
        output='screen')
    play_cmd = ['ros2', 'bag', 'play', bag, '--clock', '1000', '-r', p['rate'],
                '--start-offset', p['start_offset'], '--disable-keyboard-controls',
                '--topics', src_depth, src_info, '/tf', '/tf_static', js_src, nom]
    if remaps:
        play_cmd += ['--remap', *remaps]
    play = ExecuteProcess(cmd=play_cmd, output='screen')

    actions += [
        rtd, cbf, record,
        TimerAction(period=float(p['play_delay_s']), actions=[play]),
        # The recorder needs a moment to flush the last messages.
        RegisterEventHandler(OnProcessExit(target_action=play, on_exit=[
            LogInfo(msg=f'[bag_replay] playback done, output in {out}'),
            TimerAction(period=2.0, actions=[EmitEvent(event=Shutdown(reason='replay done'))])])),
    ]
    return actions


def generate_launch_description():
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory('franka_experiments')
    defaults = {**_tcs._DEFAULTS,
                'robot_config_yaml': os.path.join(share, 'config', 'fr3_complete.yaml'),
                'camera_extrinsics_yaml': os.path.join(share, 'config', 'camera_extrinsics.yaml')}
    args = [DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in _OWN.items()]
    args += [DeclareLaunchArgument(k, default_value=str(defaults.get(k, '')))
             for k in _PASSED]
    return LaunchDescription(args + [OpaqueFunction(function=_replay)])
