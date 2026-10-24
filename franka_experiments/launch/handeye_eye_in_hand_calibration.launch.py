"""Bringup: eye-in-hand calibration of the wrist D405 against a tag on the desk.

::

    # 1. place the tag: camera + RViz only, the robot is NOT started
    ros2 launch franka_experiments handeye_eye_in_hand_calibration.launch.py preview:=true

    # 2. calibrate
    ros2 launch franka_experiments handeye_eye_in_hand_calibration.launch.py

Put the arm where the D405 sees the AprilTag from roughly 0.2–0.4 m. RViz shows
the camera with the start check drawn on it (``tag_overlay_node``): the arm
does not move until the tag is inside the circle. The node then runs bootstrap
+ orbit by itself and writes ``config/camera_EE_extrinsic.yaml``
(``fr3_link8 → d405_color_optical_frame`` plus ``fr3_link8 → d405_link``) if
the held-out error passes.

Same structure as ``handeye_calibration_bringup.launch.py`` (eye-to-hand):

* ``minimal.launch.py`` — robot driver + bare ``rt_velocity_executor_controller``
  on ``tracking_qdot``; no CBF, no commander, no scene camera.
* RealSense driver for the D405 ONLY, selected by serial, depth off.
* ``image_rectifier_node`` — the D405 publishes ``color/image_raw`` with
  plumb_bob distortion, and apriltag_ros assumes a rectified image.
* ``apriltag_node`` on the rectified stream; ``tag_overlay_node`` + RViz.
* ``handeye_eye_in_hand_node`` after ``calibration_delay``; when it exits the
  whole launch shuts down.

Real-time hygiene, taken from ``torque_control_stack.launch.py`` after the first
hardware run ended in ``joint_motion_generator_acceleration_discontinuity``:
the ros2_control SCHED_FIFO thread is pinned to the isolated core
(``rt_pin_cpu``) and every numpy/OpenCV node runs single-threaded BLAS. The
controller's rate limiter scales its step with the MEASURED cycle period, so a
loop stalled a few ms (unpinned thread next to busy cores) hands libfranka a
velocity jump inside one 1 ms tick.

Topics and frames were read from the live D405 (realsense2_camera 4.58.3):
``/<cam>/<cam>/color/image_raw``, ``/<cam>/<cam>/color/camera_info``, frames
``<cam>_link → <cam>_color_frame → <cam>_color_optical_frame``; apriltag_ros
3.4.0 publishes ``<cam>_color_optical_frame → tag<family>:<id>``.
"""

import tempfile

import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription,
    LogInfo, OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Same as torque_control_stack.launch.py: NumPy's OpenBLAS pool busy-waits on
# this 24-core box next to the SCHED_FIFO thread; every matrix here is ≤ 7x7.
_SINGLE_THREAD_BLAS = {
    'OPENBLAS_NUM_THREADS': '1',
    'OMP_NUM_THREADS': '1',
    'MKL_NUM_THREADS': '1',
    'NUMEXPR_NUM_THREADS': '1',
}


def _as_bool(x: str) -> bool:
    return str(x).strip().lower() in ('1', 'true', 'yes', 'y', 'on')


def _launch(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    cam = arg('camera_name')
    family = arg('apriltag_family')
    tag_id = int(arg('tag_id'))
    tag_size = float(arg('apriltag_size'))
    max_offaxis = float(arg('max_start_offaxis_deg'))
    tag_frame = f'tag{family}:{tag_id}'
    raw_ns = f'/{cam}/{cam}/color'
    rect_ns = f'/{cam}/{cam}/color_rect'
    preview = _as_bool(arg('preview'))

    actions = [LogInfo(
        msg=f'[handeye_eye_in_hand] {"PREVIEW (robot NOT started)" if preview else "CALIBRATION"} '
            f'camera={cam} serial={arg("camera_serial")} image={raw_ns}/image_raw → '
            f'{rect_ns}/image_rect  tag={tag_frame} size={tag_size} m  '
            f'RViz topic={rect_ns}/image_overlay')]

    # ── Camera pipeline (both modes) ─────────────────────────────────────────
    cam_args = {
        'serial_no':        arg('camera_serial'),
        'camera_namespace': cam,
        'camera_name':      cam,
        'enable_color':     'true',
        'enable_depth':     'false',
    }
    if arg('color_profile').strip():
        # The D405's colour comes from its depth module, hence this key.
        cam_args['depth_module.color_profile'] = arg('color_profile').strip()
    exposure = arg('exposure').strip().lower()
    if exposure != 'auto':
        # MANUAL exposure. Measured with the tag on the white desk: auto
        # exposure clipped half the image and the tag's black read 77/255 —
        # 0 % detections, even with the AE ROI on the image centre. Manual
        # 6000 → 100 %, decision margin 65, nothing clipped (1000 → 88 %,
        # 600 → 22 %). rs_launch.py only forwards its own whitelisted
        # arguments, but passes every key of config_file to the node.
        cam_config = {
            'depth_module.enable_auto_exposure': False,
            'depth_module.exposure': int(float(exposure)),
            'depth_module.gain': int(float(arg('gain'))),
        }
        with tempfile.NamedTemporaryFile('w', prefix='handeye_d405_', suffix='.yaml',
                                         delete=False) as fh:
            yaml.safe_dump(cam_config, fh)
        cam_args['config_file'] = fh.name
        actions.append(LogInfo(msg=f'[handeye_eye_in_hand] D405 manual exposure '
                                   f'{cam_config["depth_module.exposure"]} gain '
                                   f'{cam_config["depth_module.gain"]} ({fh.name})'))
    # camera_delay_s, for consistency with the other launch files. It does NOT
    # avoid the "Device or resource busy" / "device has been disconnected"
    # cycle the D405/D455 always logs on its first open on this rig — that is
    # an internal driver self-heal (reset + re-enumeration) that reproduces at
    # the same offset from the node's own construction regardless of when this
    # action fires (see launch_defaults.yaml: camera_delay_s). It only pushes
    # when the camera driver itself starts relative to other nodes.
    realsense_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py',
        ]).perform(context)),
        launch_arguments=cam_args.items(),
    )
    actions.append(TimerAction(period=float(arg('camera_delay_s')),
                               actions=[realsense_driver]))

    rectifier = Node(
        package='franka_experiments',
        executable='image_rectifier_node',
        name='d405_image_rectifier',
        remappings=[
            ('image_raw',        f'{raw_ns}/image_raw'),
            ('camera_info',      f'{raw_ns}/camera_info'),
            ('image_rect',       f'{rect_ns}/image_rect'),
            ('camera_info_rect', f'{rect_ns}/camera_info'),
        ],
        additional_env=_SINGLE_THREAD_BLAS,
        output='log',
    )
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        remappings=[
            ('image_rect',  f'{rect_ns}/image_rect'),
            ('camera_info', f'{rect_ns}/camera_info'),
        ],
        parameters=[{
            'family': family,
            'size': tag_size,
            'image_transport': 'raw',
            # Default 2.0 halves the image before quad detection; at 10 Hz the
            # full resolution costs nothing and sharpens the corners of a tag
            # that spans only ~100 px.
            'detector.decimate': 1.0,
            'detector.threads': 1,
        }],
        output='log',
    )
    overlay = Node(
        package='franka_experiments',
        executable='tag_overlay_node',
        name='tag_overlay',
        remappings=[
            ('image',         f'{rect_ns}/image_rect'),
            ('camera_info',   f'{rect_ns}/camera_info'),
            ('image_overlay', f'{rect_ns}/image_overlay'),
        ],
        parameters=[{
            'tag_id': tag_id,
            'tag_size': tag_size,
            'max_offaxis_deg': max_offaxis,
        }],
        additional_env=_SINGLE_THREAD_BLAS,
        output='log',
    )
    cam_delay = float(arg('camera_delay_s'))
    actions.append(TimerAction(period=cam_delay + 3.0,
                               actions=[rectifier, apriltag_node, overlay]))

    if _as_bool(arg('use_rviz')):
        rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_handeye_eye_in_hand',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('franka_experiments'), 'config',
                'handeye_eye_in_hand_rviz.rviz']).perform(context)],
            output='log',
        )
        actions.append(TimerAction(period=cam_delay + 4.0, actions=[rviz]))

    if preview:
        actions.append(LogInfo(
            msg='[handeye_eye_in_hand] PREVIEW: move the tag until its outline is GREEN '
                'inside the circle in RViz, Ctrl-C, then launch without preview:=true.'))
        return actions

    # ── Robot + calibration ──────────────────────────────────────────────────
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('franka_experiments'), 'launch', 'minimal.launch.py',
        ]).perform(context)),
        launch_arguments={
            'command_topic':            'tracking_qdot',
            'use_torque_controller':    'false',
            'max_accel':                '0.5',
            'enable_camera':            'false',
            'start_real_time_distance': 'false',
            'start_experiment_logger':  'false',
        }.items(),
    ))

    rt_pin_cpu = arg('rt_pin_cpu').strip()
    if rt_pin_cpu:
        # Start with the controller spawner, like the torque stack: the script
        # polls for the FIFO thread, and a later start leaves the 1 kHz loop
        # unpinned right when it begins.
        spawner_delay = float(LaunchConfiguration(
            'control_spawner_delay_s', default='10.0').perform(context))
        actions.append(TimerAction(period=spawner_delay, actions=[ExecuteProcess(
            cmd=['bash', PathJoinSubstitution([
                FindPackageShare('franka_experiments'), 'scripts', 'pin_rt_thread.sh',
            ]).perform(context), rt_pin_cpu, '60'],
            output='screen',
        )]))
        actions.append(LogInfo(msg=f'[handeye_eye_in_hand] RT pinning: ros2_control_node '
                                   f'FIFO thread → CPU {rt_pin_cpu}'))
    else:
        actions.append(LogInfo(msg='[handeye_eye_in_hand] RT pinning DISABLED (rt_pin_cpu empty)'))

    handeye_node = Node(
        package='franka_experiments',
        executable='handeye_eye_in_hand_node',
        name='handeye_eye_in_hand_node',
        parameters=[{
            'camera_frame':          f'{cam}_color_optical_frame',
            'camera_link_frame':     f'{cam}_link',
            'tag_frame':             tag_frame,
            'tag_id':                tag_id,
            'orbit_samples':         int(arg('orbit_samples')),
            'output_file':           arg('output_file'),
            'output_dir':            arg('output_dir'),
            'max_start_offaxis_deg': max_offaxis,
            'overlay_topic':         f'{rect_ns}/image_overlay',
        }],
        additional_env=_SINGLE_THREAD_BLAS,
        output='screen',
    )
    actions.append(TimerAction(period=float(arg('calibration_delay')), actions=[handeye_node]))
    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=handeye_node,
        on_exit=[EmitEvent(event=Shutdown(
            reason='handeye_eye_in_hand_node finished — shutting down.'))],
    )))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'preview', default_value='false',
            description='true = camera + tag overlay + RViz only, to place the tag; '
                        'the robot is not started.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Open RViz with the camera view and the start check.'),
        DeclareLaunchArgument(
            'camera_name', default_value='d405',
            description='RealSense camera_namespace AND camera_name: frames become '
                        '<name>_link / <name>_color_optical_frame.'),
        DeclareLaunchArgument(
            'camera_serial', default_value='_126122270738',
            description='D405 serial. Keep the leading underscore, or the launch '
                        'system turns it into an integer and the device is not found.'),
        DeclareLaunchArgument(
            'color_profile', default_value='',
            description='depth_module.color_profile, e.g. 848x480x10. Empty = driver '
                        'default (848x480x10 measured on a USB 2.1 port).'),
        DeclareLaunchArgument(
            'camera_delay_s', default_value='0.0',
            description='Seconds before the RealSense driver node is launched. Does not '
                        'avoid the EBUSY/disconnect cycle logged on the D405 first open — '
                        'that is a self-healing driver quirk, not a launch-timing race.'),
        DeclareLaunchArgument(
            'exposure', default_value='6000',
            description="D405 manual exposure (depth_module.exposure). 'auto' re-enables "
                        'auto exposure, which blows out the tag over a white desk. Lower it '
                        'if RViz reports OVEREXPOSED; raise it if the decision margin drops.'),
        DeclareLaunchArgument(
            'gain', default_value='16',
            description='D405 gain (depth_module.gain) used with manual exposure.'),
        DeclareLaunchArgument(
            'apriltag_family', default_value='36h11',
            description='AprilTag family.'),
        DeclareLaunchArgument(
            'apriltag_size', default_value='0.10',
            description='Edge of the tag black square [m].'),
        DeclareLaunchArgument(
            'tag_id', default_value='0',
            description='ID of the tag on the desk.'),
        DeclareLaunchArgument(
            'max_start_offaxis_deg', default_value='12.0',
            description='The arm starts only once the tag is within this angle of the '
                        'optical axis (the circle in RViz).'),
        DeclareLaunchArgument(
            'orbit_samples', default_value='25',
            description='Look-at poses visited in the orbit phase.'),
        DeclareLaunchArgument(
            'output_file', default_value='camera_EE_extrinsic.yaml',
            description='Result file name.'),
        DeclareLaunchArgument(
            'output_dir', default_value='',
            description='Directory for the result and the dataset. Empty = '
                        'franka_experiments/config of the source tree and of the install.'),
        DeclareLaunchArgument(
            'rt_pin_cpu', default_value='3',
            description="Isolated CPU for ros2_control_node's SCHED_FIFO thread "
                        "(scripts/pin_rt_thread.sh). '' disables."),
        DeclareLaunchArgument(
            'calibration_delay', default_value='15.0',
            description='Seconds before the calibration node starts; must exceed '
                        'control_spawner_delay_s (~10 s) + controller activation.'),
        OpaqueFunction(function=_launch),
    ])
