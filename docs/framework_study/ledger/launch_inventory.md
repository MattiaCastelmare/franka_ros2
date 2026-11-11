# Launch inventory — extracted, NOT yet verified

> `Node(...)`, `ExecuteProcess(...)`, `IncludeLaunchDescription(...)` and `DeclareLaunchArgument(...)` call sites per launch file. Values are shown as written; which nodes actually start depends on conditions and `OpaqueFunction` logic resolved in Stage 1.

## `franka_experiments/launch/bag_replay.launch.py` (201 lines)

| Line | Package | Executable | Name | Conditional |
|---:|---|---|---|---|
| 157 | `'franka_experiments'` | `'real_time_distance'` | `'real_time_distance'` |  |
| 164 | `'franka_experiments'` | `'cbf_safety_filter'` | `'cbf_safety_filter'` |  |
| 168 | `(ExecuteProcess)` | `['ros2', 'bag', 'record', '--use-sim-time', '-o', out, ptop…` | `` |  |
| 179 | `(ExecuteProcess)` | `play_cmd` | `` |  |

**Launch arguments (2):** `k`, `k`

## `franka_experiments/launch/handeye_calibration_bringup.launch.py` (166 lines)

| Line | Package | Executable | Name | Conditional |
|---:|---|---|---|---|
| 104 | `'apriltag_ros'` | `'apriltag_node'` | `'apriltag_node'` |  |
| 125 | `'rviz2'` | `'rviz2'` | `'rviz2'` |  |
| 135 | `'franka_experiments'` | `'handeye_calibration_node'` | `'handeye_calibration_node'` |  |

**Includes:**

- L85: `PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('franka_experiments'), 'launch', 'minima…`

**Launch arguments (3):** `apriltag_family`, `apriltag_size`, `calibration_delay`

## `franka_experiments/launch/handeye_eye_in_hand_calibration.launch.py` (335 lines)

| Line | Package | Executable | Name | Conditional |
|---:|---|---|---|---|
| 137 | `'franka_experiments'` | `'image_rectifier_node'` | `'d405_image_rectifier'` |  |
| 150 | `'apriltag_ros'` | `'apriltag_node'` | `'apriltag_node'` |  |
| 170 | `'franka_experiments'` | `'tag_overlay_node'` | `'tag_overlay'` |  |
| 192 | `'rviz2'` | `'rviz2'` | `'rviz2_handeye_eye_in_hand'` |  |
| 231 | `(ExecuteProcess)` | `['bash', PathJoinSubstitution([FindPackageShare('franka_exp…` | `` |  |
| 242 | `'franka_experiments'` | `'handeye_eye_in_hand_node'` | `'handeye_eye_in_hand_node'` |  |

**Includes:**

- L128: `PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_laun…`
- L210: `PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('franka_experiments'), 'launch', 'minima…`

**Launch arguments (17):** `preview`, `use_rviz`, `camera_name`, `camera_serial`, `color_profile`, `camera_delay_s`, `exposure`, `gain`, `apriltag_family`, `apriltag_size`, `tag_id`, `max_start_offaxis_deg`, `orbit_samples`, `output_file`, `output_dir`, `rt_pin_cpu`, `calibration_delay`

## `franka_experiments/launch/minimal.launch.py` (526 lines)

| Line | Package | Executable | Name | Conditional |
|---:|---|---|---|---|
| 158 | `'tf2_ros'` | `'static_transform_publisher'` | `'world_to_robot_base_tf'` |  |
| 187 | `'controller_manager'` | `'spawner'` | `''` |  |
| 223 | `'franka_simulation'` | `'image_publisher'` | `'image_republisher'` |  |
| 237 | `'tf2_ros'` | `'static_transform_publisher'` | `'camera_extrinsics_tf'` |  |
| 323 | `'rqt_image_view'` | `'rqt_image_view'` | `'wrist_image_view'` |  |
| 353 | `'franka_experiments'` | `'real_time_distance'` | `'real_time_distance'` |  |
| 366 | `(ExecuteProcess)` | `['bash', '-c', f'until ros2 control list_controllers --cont…` | `` |  |
| 400 | `'franka_experiments'` | `'experiment_logger'` | `'experiment_logger'` |  |

**Includes:**

- L142: `PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('franka_bringup'), 'launch', 'franka.lau…`
- L213: `PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_laun…`
- L307: `PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_laun…`

**Launch arguments (21):** `use_torque_controller`, `torque_command_topic`, `lpf_alpha`, `tau_max_scale`, `control_spawner_delay_s`, `enable_camera`, `camera_extrinsics_yaml`, `camera_delay_s`, `camera_align_depth`, `camera_serial`, `start_wrist_camera`, `wrist_camera_serial`, `wrist_camera_name`, `wrist_color_profile`, `wrist_depth_profile`, `wrist_enable_depth`, `show_wrist_image`, `start_real_time_distance`, `real_time_distance_delay_s`, `start_experiment_logger`, `experiment_logger_delay_s`

## `franka_experiments/launch/thales.launch.py` (371 lines)

| Line | Package | Executable | Name | Conditional |
|---:|---|---|---|---|
| 142 | `'controller_manager'` | `'spawner'` | `''` |  |
| 150 | `'tf2_ros'` | `'static_transform_publisher'` | `'world_to_robot_base_tf'` |  |
| 194 | `'franka_simulation'` | `'image_publisher'` | `'image_republisher'` |  |
| 208 | `'tf2_ros'` | `'static_transform_publisher'` | `'camera_extrinsics_tf'` |  |
| 256 | `'franka_experiments'` | `'real_time_distance'` | `'real_time_distance'` |  |
| 278 | `'franka_experiments'` | `'experiment_logger'` | `'experiment_logger'` |  |
| 301 | `(ExecuteProcess)` | `['ros2', 'bag', 'record', '--output', bag_dir] + ROSBAG_TOP…` | `` |  |

**Includes:**

- L126: `PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('franka_bringup'), 'launch', 'franka.lau…`
- L185: `PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_laun…`

**Launch arguments (12):** `control_spawner_delay_s`, `enable_camera`, `camera_extrinsics_yaml`, `camera_delay_s`, `start_real_time_distance`, `real_time_distance_delay_s`, `start_experiment_logger`, `experiment_logger_delay_s`, `experiment_name`, `start_rosbag`, `rosbag_output_dir`, `rosbag_delay_s`

## `franka_experiments/launch/torque_control_stack.launch.py` (1527 lines)

| Line | Package | Executable | Name | Conditional |
|---:|---|---|---|---|
| 364 | `(ExecuteProcess)` | `cmd` | `` |  |
| 462 | `'controller_manager'` | `'spawner'` | `''` |  |
| 480 | `(ExecuteProcess)` | `['bash', PathJoinSubstitution([FindPackageShare('franka_exp…` | `` |  |
| 495 | `'tf2_ros'` | `'static_transform_publisher'` | `'world_to_robot_base_tf'` |  |
| 540 | `(ExecuteProcess)` | `['python3', PathJoinSubstitution([FindPackageShare('franka_…` | `` |  |
| 621 | `(ExecuteProcess)` | `['ros2', 'topic', 'pub', js_topic, 'sensor_msgs/msg/JointSt…` | `` |  |
| 665 | `'franka_simulation'` | `'image_publisher'` | `'image_republisher'` |  |
| 687 | `'tf2_ros'` | `'static_transform_publisher'` | `'camera_extrinsics_tf'` |  |
| 724 | `'franka_experiments'` | `'real_time_distance'` | `'real_time_distance'` |  |
| 760 | `'franka_experiments'` | `'cbf_safety_filter'` | `'cbf_safety_filter'` |  |
| 776 | `'franka_experiments'` | `'qddot_to_torque'` | `'qddot_to_torque'` |  |
| 805 | `'franka_experiments'` | `'iso_safety_monitor'` | `'iso_safety_monitor'` |  |
| 834 | `'franka_experiments'` | `'iso_evidence_logger'` | `'iso_evidence_logger'` |  |
| 890 | `'franka_experiments'` | `'rl_policy_commander'` | `'rl_policy_commander'` |  |
| 904 | `'franka_experiments'` | `'pentagon_qddot_commander'` | `'pentagon_qddot_commander'` |  |
| 943 | `'franka_experiments'` | `'experiment_logger'` | `'experiment_logger'` |  |
| 984 | `'franka_experiments'` | `'trajectory_overlay_node'` | `'trajectory_overlay'` |  |
| 1011 | `'franka_experiments'` | `'trajectory_visualization_node'` | `'trajectory_visualization'` |  |
| 1026 | `'rviz2'` | `'rviz2'` | `'rviz2'` |  |
| 1073 | `(ExecuteProcess)` | `['ros2', 'bag', 'record', '-o', out, '--compression-mode', …` | `` |  |

**Includes:**

- L447: `PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('franka_bringup'), 'launch', 'franka.lau…`
- L584: `PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('franka_fr3_moveit_config'), 'launch', '…`
- L655: `PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_laun…`

**Launch arguments (68):** `control_spawner_delay_s`, `enable_camera`, `camera_extrinsics_yaml`, `camera_link_extrinsics_yaml`, `camera_delay_s`, `camera_depth_profile`, `camera_align_depth`, `start_real_time_distance`, `publish_overlay_image`, `start_experiment_logger`, `experiment_logger_delay_s`, `start_trajectory_viz`, `trajectory_overlay_window`, `trajectory_trail_seconds`, `start_trajectory_markers`, `start_rviz`, `trajectory_viz_delay_s`, `start_move_group`, `motion_source`, `external_plant`, `cbf_param_overrides`, `rl_onnx_model`, `rl_sim_config`, `rl_target_xyz`, `rl_target_sequence`, `rl_action_scale`, `isolation_test`, `robot_config_yaml`, `obstacle_tracking`, `obstacle_velocity_source`, `lateral_evasion`, `outrun_evasion`, `livelock_escape`, `zone_ladder`, `multi_obstacle_k`, `vobs_in_hdot`, `velocity_standoff`, `iso_enabled`, `iso_mode`, `iso_ssm_speed_rows`, `iso_monitor_enabled`, `torque_iso_monitor_delay_s`, `start_iso_evidence_logger`, `torque_iso_evidence_delay_s`, `iso_evidence_dir`, `iso_evidence_run_name`, `link_speed_max`, `retreat_cap_max_speed`, `obstacle_velocity_normal_guard`, `obstacle_identity_guard`, `latency_compensation`, `uncertainty_margin`, `sim_obstacle`, `depth_bag`, `torque_command_topic`, `controller_spawner_timeout_s`, `torque_dynamics_delay_s`, `torque_rtd_delay_s`, `torque_commander_extra_delay_s`, `torque_world_tf_delay_s`, `torque_camera_tf_delay_s`, `torque_image_republisher_extra_delay_s`, `torque_finger_pub_delay_s`, `torque_finger_pub_rate_hz`, `start_rosbag`, `rosbag_output_dir`, `rosbag_delay_s`, `rt_pin_cpu`

## `franka_experiments/launch/velocity_cbf_control_stack.launch.py` (441 lines)

| Line | Package | Executable | Name | Conditional |
|---:|---|---|---|---|
| 155 | `'controller_manager'` | `'spawner'` | `''` |  |
| 166 | `'tf2_ros'` | `'static_transform_publisher'` | `'world_to_robot_base_tf'` |  |
| 218 | `'franka_simulation'` | `'image_publisher'` | `'image_republisher'` |  |
| 232 | `'tf2_ros'` | `'static_transform_publisher'` | `'camera_extrinsics_tf'` |  |
| 299 | `'franka_experiments'` | `'real_time_distance'` | `'real_time_distance'` |  |
| 317 | `'franka_experiments'` | `'cbf_velocity_filter'` | `'cbf_velocity_filter'` |  |
| 330 | `'franka_experiments'` | `'ee_pentagon_velocity_commander'` | `'ee_pentagon_velocity_commander'` |  |
| 342 | `'franka_experiments'` | `'experiment_logger'` | `'experiment_logger'` |  |

**Includes:**

- L140: `PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('franka_bringup'), 'launch', 'franka.lau…`
- L210: `PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_laun…`
- L278: `PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_laun…`

**Launch arguments (16):** `control_spawner_delay_s`, `enable_camera`, `camera_extrinsics_yaml`, `camera_delay_s`, `camera_serial`, `camera_align_depth`, `camera_depth_profile`, `start_wrist_camera`, `wrist_camera_serial`, `wrist_camera_name`, `wrist_color_profile`, `wrist_depth_profile`, `start_real_time_distance`, `start_experiment_logger`, `experiment_logger_delay_s`, `bypass_cbf`

## `franka_experiments/test/launch/test_rl_fake.launch.py` (78 lines)


**Includes:**

- L52: `PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('franka_experiments'), 'launch', 'torque…`

**Launch arguments (5):** `rl_onnx_model`, `rl_sim_config`, `rl_target_xyz`, `rl_target_sequence`, `rl_action_scale`

## `franka_experiments/test/launch/test_torque_fake.launch.py` (108 lines)

| Line | Package | Executable | Name | Conditional |
|---:|---|---|---|---|
| 82 | `'franka_experiments'` | `'pentagon_torque_commander'` | `'pentagon_torque_commander'` |  |

**Includes:**

- L68: `PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('franka_experiments'), 'launch', 'minima…`

**Launch arguments (2):** `namespace`, `control_spawner_delay_s`

## `franka_experiments/test/launch/test_velocity_fake.launch.py` (99 lines)

| Line | Package | Executable | Name | Conditional |
|---:|---|---|---|---|
| 74 | `'franka_experiments'` | `'ee_pentagon_velocity_commander'` | `'ee_pentagon_velocity_commander'` |  |

**Includes:**

- L61: `PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('franka_experiments'), 'launch', 'minima…`

**Launch arguments (2):** `namespace`, `control_spawner_delay_s`

## `franka_simulation_experiments/launch/gz_cbf_stack.launch.py` (273 lines)

| Line | Package | Executable | Name | Conditional |
|---:|---|---|---|---|
| 213 | `'franka_simulation_experiments'` | `'reflex_monitor'` | `'reflex_monitor'` |  |
| 217 | `'franka_simulation_experiments'` | `'episode_supervisor'` | `'episode_supervisor'` |  |
| 224 | `'franka_simulation_experiments'` | `'scenario_commander'` | `'scenario_commander'` |  |
| 230 | `'franka_simulation_experiments'` | `'emulated_perception'` | `'emulated_perception'` |  |
| 240 | `'franka_simulation_experiments'` | `'ground_truth_node'` | `'ground_truth'` |  |
| 254 | `(ExecuteProcess)` | `[os.path.join(get_package_prefix('franka_simulation_experim…` | `` |  |
| 260 | `(ExecuteProcess)` | `['ros2', 'bag', 'record', '--use-sim-time', '-o', os.path.j…` | `` |  |

**Includes:**

- L199: `PythonLaunchDescriptionSource(os.path.join(fs_share, 'launch', 'gz_fr3_plant.launch.py'))`
- L208: `PythonLaunchDescriptionSource(os.path.join(fe_share, 'launch', 'torque_control_stack.launch.py'))`

**Launch arguments (1):** `k`

## `franka_simulation_experiments/launch/gz_circle_demo.launch.py` (145 lines)

| Line | Package | Executable | Name | Conditional |
|---:|---|---|---|---|
| 112 | `'franka_experiments'` | `'pentagon_qddot_commander'` | `'pentagon_qddot_commander'` |  |
| 124 | `'franka_experiments'` | `'qddot_to_torque'` | `'qddot_to_torque'` |  |
| 128 | `'franka_simulation_experiments'` | `'gz_trajectory_markers'` | `'gz_trajectory_markers'` |  |

**Includes:**

- L103: `PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('franka_simulation_experiments'), 'lau…`

**Launch arguments (1):** `k`

## `franka_simulation_experiments/launch/gz_plant.launch.py` (66 lines)


**Includes:**

- L55: `PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('franka_simulation'), 'launch', 'gz_fr…`

**Launch arguments (1):** `k`
