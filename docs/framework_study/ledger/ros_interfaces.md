# ROS interfaces — extracted, NOT yet verified

> Every call site of `create_publisher/create_subscription/create_service/create_client/create_timer/declare_parameter(s)`, action clients/servers, TF listeners/broadcasters/lookups, executors and callback groups, found by `ast` in every Python file in scope. Arguments are shown **as written** (often a variable or `self.<attr>`, not the resolved topic name); resolving them to concrete names, types, rates and QoS is Stage 1 work (chapter 02). Helpers that create interfaces on behalf of a node (e.g. in `utils/ros*.py`, `utils/node_runtime.py`) are listed under their own file.

## Summary (counts per file)

| File | pub | sub | srv | cli | action_cli | action_srv | timer | param | params | tf_listen | tf_lookup | tf_bcast | tf_static | tf_send | executor | cbgroup | rate |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `franka_experiments/nodes/capsule_overlay_node.py` | 1 | 2 |  |  |  |  |  | 3 |  |  |  |  |  |  |  |  |  |
| `franka_experiments/nodes/cbf_OSCBF_filter.py` | 1 | 3 |  |  |  |  | 1 |  |  |  |  |  |  |  | 1 |  |  |
| `franka_experiments/nodes/cbf_safety_filter.py` | 2 | 4 |  |  |  |  | 2 |  |  |  |  |  |  |  | 1 | 3 |  |
| `franka_experiments/nodes/cbf_velocity_filter.py` | 1 | 3 |  |  |  |  | 1 | 1 |  |  |  |  |  |  |  |  |  |
| `franka_experiments/nodes/distance_visualization_node.py` | 1 | 3 |  |  |  |  |  | 3 |  | 1 |  |  |  |  |  |  |  |
| `franka_experiments/nodes/ee_circle_velocity_commander.py` | 1 |  |  |  |  |  | 1 | 17 |  |  |  |  |  |  |  |  |  |
| `franka_experiments/nodes/ee_pentagon_velocity_commander.py` | 1 |  |  |  |  |  | 1 | 17 |  |  |  |  |  |  |  |  |  |
| `franka_experiments/nodes/ee_random_waypoints_velocity_commander.py` | 1 |  |  |  |  |  | 1 | 17 |  |  |  |  |  |  |  |  |  |
| `franka_experiments/nodes/experiment_logger.py` |  | 9 |  |  |  |  | 1 | 17 |  |  |  |  |  |  |  |  |  |
| `franka_experiments/nodes/frame_grabber.py` |  | 2 |  |  |  |  | 1 | 6 |  |  |  |  |  |  |  |  |  |
| `franka_experiments/nodes/handeye_calibration_node.py` | 1 | 1 |  |  |  |  | 1 | 52 |  | 1 | 6 |  | 1 | 1 |  |  |  |
| `franka_experiments/nodes/handeye_eye_in_hand_node.py` | 1 | 1 |  |  |  |  | 1 |  |  | 1 | 2 |  |  |  |  |  |  |
| `franka_experiments/nodes/image_rectifier_node.py` | 2 | 2 |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
| `franka_experiments/nodes/iso_evidence_logger.py` |  | 3 |  |  |  |  | 1 |  |  |  |  |  |  |  | 1 | 2 |  |
| `franka_experiments/nodes/iso_safety_monitor.py` | 1 | 3 | 1 |  |  |  | 1 | 1 |  |  |  |  |  |  | 1 | 2 |  |
| `franka_experiments/nodes/pentagon_qddot_commander.py` | 4 | 4 |  |  |  |  | 1 | 67 |  |  |  |  |  |  |  |  |  |
| `franka_experiments/nodes/pentagon_torque_commander.py` | 1 | 1 |  |  |  |  | 1 | 18 |  |  |  |  |  |  |  |  |  |
| `franka_experiments/nodes/qddot_to_torque.py` | 2 | 2 |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
| `franka_experiments/nodes/real_time_distance.py` | 4 | 2 |  |  |  |  | 1 | 2 |  | 1 |  |  |  |  |  |  |  |
| `franka_experiments/nodes/rl_policy_commander.py` | 2 | 4 |  |  |  |  | 1 | 25 |  |  |  |  |  |  |  |  |  |
| `franka_experiments/nodes/tag_overlay_node.py` | 1 | 3 |  |  |  |  |  | 6 |  |  |  |  |  |  |  |  |  |
| `franka_experiments/nodes/trajectory_overlay_node.py` | 1 | 4 |  |  |  |  | 1 |  |  |  |  |  |  |  |  |  |  |
| `franka_experiments/nodes/trajectory_visualization_node.py` | 1 | 2 |  |  |  |  | 1 |  |  |  |  |  |  |  |  |  |  |
| `franka_experiments/utils/kinematics.py` |  | 2 |  |  |  |  | 1 |  |  |  |  |  |  |  |  |  |  |
| `franka_experiments/utils/params.py` |  |  |  |  |  |  |  | 5 |  |  |  |  |  |  |  |  |  |
| `franka_experiments/utils/ros_setup.py` |  |  |  | 1 |  |  |  |  |  |  |  |  |  |  |  |  |  |
| `franka_experiments/utils/tf_manager.py` |  |  |  |  |  |  |  |  |  |  | 2 |  |  |  |  |  |  |
| `scripts/cbf_phase4_check.py` |  | 2 |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
| `scripts/iso_constants_measure.py` | 1 | 4 |  |  |  |  |  |  |  | 1 |  |  |  |  |  |  |  |
| `scripts/iso_preflight_check.py` |  | 1 |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
| `scripts/latency_budget.py` | 3 | 5 |  |  |  |  | 2 |  |  |  |  |  |  |  |  |  |  |
| `scripts/pipeline_rate_monitor.py` |  | 1 |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
| `scripts/range_noise_calibration.py` |  | 2 |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
| `scripts/watch_obstacle_tracks.py` |  | 1 |  |  |  |  | 1 | 4 |  |  |  |  |  |  |  |  |  |
| `scripts/emulated_perception.py` | 1 | 3 |  |  |  |  | 1 |  |  |  |  |  |  |  |  |  |  |
| `scripts/episode_supervisor.py` | 2 | 2 |  | 1 |  |  | 1 |  |  |  |  |  |  |  |  |  |  |
| `scripts/ground_truth_node.py` | 2 | 3 |  |  |  |  | 2 |  |  |  |  |  |  |  |  |  |  |
| `scripts/gz_trajectory_markers.py` |  | 2 |  |  |  |  | 1 |  |  |  |  |  |  |  |  |  |  |
| `scripts/reflex_monitor.py` | 1 | 2 |  |  |  |  | 2 | 3 |  |  |  |  |  |  |  |  |  |
| `scripts/scenario_commander.py` | 3 | 2 |  |  |  |  | 1 | 3 |  |  |  |  |  |  |  |  |  |
| `scripts/simexp/graph_contract.py` |  | 1 |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |

## Details

### `franka_experiments/franka_experiments/nodes/capsule_overlay_node.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| sub | 113 | `CapsuleOverlayNode.__init__` | `JointState, self._joint_state_topic, self._joint_state_cb` |
| sub | 117 | `CapsuleOverlayNode.__init__` | `Image, self._image_topic, self._image_cb` |
| pub | 123 | `CapsuleOverlayNode.__init__` | `Image, self._output_topic, 10` |

**declare_parameter** ×3: `'joint_state_topic'`, `'image_topic'`, `'output_topic'`


### `franka_experiments/franka_experiments/nodes/cbf_OSCBF_filter.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| sub | 727 | `OSCBFFilterNode._create_ros_interfaces` | `JointState, js_topic, self._on_joint_state` |
| sub | 729 | `OSCBFFilterNode._create_ros_interfaces` | `Float64MultiArray, self._tau_nom_topic, self._on_tau_nom` |
| sub | 733 | `OSCBFFilterNode._create_ros_interfaces` | `MultiLinkDistance, '/cbf/per_link_distances', self._on_distances` |
| pub | 737 | `OSCBFFilterNode._create_ros_interfaces` | `Float64MultiArray, self._tau_safe_topic, 10` |
| timer | 740 | `OSCBFFilterNode._create_ros_interfaces` | `self._dt, self._tick` |
| executor | 961 | `main` | `` |

### `franka_experiments/franka_experiments/nodes/cbf_safety_filter.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| cbgroup | 421 | `CBFSafetyFilter.__init__` | `` |
| cbgroup | 422 | `CBFSafetyFilter.__init__` | `` |
| cbgroup | 423 | `CBFSafetyFilter.__init__` | `` |
| sub | 432 | `CBFSafetyFilter.__init__` | `JointState, js_topic, self._on_joint_state, callback_group=grp_io` |
| sub | 435 | `CBFSafetyFilter.__init__` | `Float64MultiArray, topics['qddot_nom'], self._on_qddot_nom, callback_group=grp_io` |
| sub | 438 | `CBFSafetyFilter.__init__` | `MultiLinkDistance, topics['per_link_distances'], self._on_distances, callback_group=grp_io` |
| pub | 443 | `CBFSafetyFilter.__init__` | `Float64MultiArray, topics['qddot_safe'], 10` |
| pub | 471 | `CBFSafetyFilter.__init__` | `Float64MultiArray, topics.get('cbf_status', '/NS_1/cbf_status'), 10` |
| sub | 510 | `CBFSafetyFilter.__init__` | `Float64MultiArray, topics.get('iso_safety', '/NS_1/iso_safety'), self._on_iso_safety, callback_group=grp_io` |
| timer | 525 | `CBFSafetyFilter.__init__` | `1.0 / P.cbf_update_rate_hz, self._update_constraints, callback_group=grp_perc` |
| timer | 527 | `CBFSafetyFilter.__init__` | `1.0 / P.qp_rate_hz, self._qp_tick, callback_group=grp_ctrl` |
| executor | 2075 | `main` | `` |

### `franka_experiments/franka_experiments/nodes/cbf_velocity_filter.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| pub | 165 | `CbfVelocityFilter.__init__` | `Float64MultiArray, self.topics_ctr['velocity_topic'], 10` |
| sub | 172 | `CbfVelocityFilter.__init__` | `JointState, self.topics_ctr['joint_states_topic'], make_joint_state_callback(controller=self, joint_names=self.robot_cfg…` |
| sub | 183 | `CbfVelocityFilter.__init__` | `MultiLinkDistance, self.topics_ctr['per_link_distances'], self._multi_distance_callback` |
| sub | 190 | `CbfVelocityFilter.__init__` | `Float64MultiArray, self.topics_ctr['qdot_nom'], self._nominal_velocity_callback` |
| timer | 198 | `CbfVelocityFilter.__init__` | `self._dt, self._control_loop` |

**declare_parameter** ×1: `'bypass_cbf'`


### `franka_experiments/franka_experiments/nodes/distance_visualization_node.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| tf_listen | 127 | `DistanceVisualization.__init__` | `self.tf_buffer, self` |
| sub | 166 | `DistanceVisualization.__init__` | `Image, depth_topic, self.depth_callback` |
| sub | 167 | `DistanceVisualization.__init__` | `CameraInfo, info_topic, self.camera_info_callback` |
| sub | 168 | `DistanceVisualization.__init__` | `MultiLinkDistance, '/cbf/per_link_distances', self.distance_callback` |
| pub | 177 | `DistanceVisualization.__init__` | `Image, overlay_topic, 10` |

**declare_parameter** ×3: `'robot_config_path'`, `'camera_extrinsics_path'`, `'publish_overlay_image'`


### `franka_experiments/franka_experiments/nodes/ee_circle_velocity_commander.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| pub | 218 | `EECircleVelocityCommander.__init__` | `Float64MultiArray, self.tracking_topic, 10` |
| timer | 221 | `EECircleVelocityCommander.__init__` | `period, self._timer_cb` |

**declare_parameter** ×17: `'tracking_topic'`, `'command_topic'`, `'joint_state_topic'`, `'ee_frame'`, `'rate_hz'`, `'warmup_s'`, `'ramp_s'`, `'center_xyz'`, `'radius'`, `'plane'`, `'plane_frame'`, `'cycle_time'`, `'kp_cart'`, `'damping'`, `'qdot_max'`, `'lpf_alpha'`, `'max_accel_local'`


### `franka_experiments/franka_experiments/nodes/ee_pentagon_velocity_commander.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| pub | 181 | `EEPentagonVelocityCommander.__init__` | `Float64MultiArray, self.tracking_topic, 10` |
| timer | 184 | `EEPentagonVelocityCommander.__init__` | `period, self._timer_cb` |

**declare_parameter** ×17: `'tracking_topic'`, `'command_topic'`, `'joint_state_topic'`, `'ee_frame'`, `'rate_hz'`, `'warmup_s'`, `'ramp_s'`, `'center_xyz'`, `'radius'`, `'plane'`, `'plane_frame'`, `'cycle_time'`, `'kp_cart'`, `'damping'`, `'qdot_max'`, `'lpf_alpha'`, `'max_accel_local'`


### `franka_experiments/franka_experiments/nodes/ee_random_waypoints_velocity_commander.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| pub | 188 | `EERandomWaypointsVelocityCommander.__init__` | `Float64MultiArray, self.tracking_topic, 10` |
| timer | 191 | `EERandomWaypointsVelocityCommander.__init__` | `period, self._timer_cb` |

**declare_parameter** ×17: `'tracking_topic'`, `'command_topic'`, `'joint_state_topic'`, `'ee_frame'`, `'rate_hz'`, `'warmup_s'`, `'ramp_s'`, `'num_waypoints'`, `'workspace_bounds'`, `'min_distance'`, `'segment_duration_s'`, `'hold_time_s'`, `'loop'`, `'kp_cart'`, `'damping'`, `'qdot_max'`, `'lpf_alpha'`


### `franka_experiments/franka_experiments/nodes/experiment_logger.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| sub | 247 | `ExperimentLogger.__init__` | `JointState, str(self.get_parameter('joint_state_topic').value), self.joint_state_cb` |
| sub | 253 | `ExperimentLogger.__init__` | `MultiDistance, str(self.get_parameter('multi_distance_topic').value), self.multi_distance_cb` |
| sub | 259 | `ExperimentLogger.__init__` | `Float64MultiArray, str(self.get_parameter('qdot_nom_topic').value), self.qdot_nom_cb` |
| sub | 265 | `ExperimentLogger.__init__` | `Float64MultiArray, str(self.get_parameter('qdot_cmd_topic').value), self.qdot_cmd_cb` |
| sub | 271 | `ExperimentLogger.__init__` | `Float64MultiArray, str(self.get_parameter('torque_cmd_topic').value), self.torque_cmd_cb` |
| sub | 282 | `ExperimentLogger.__init__` | `FrankaRobotState, _rs_topic, self.robot_state_cb` |
| sub | 293 | `ExperimentLogger.__init__` | `MultiLinkDistance, _pl, self.per_link_cb` |
| sub | 298 | `ExperimentLogger.__init__` | `Float64MultiArray, self.truth_topic, self.truth_cb` |
| sub | 307 | `ExperimentLogger.__init__` | `Float64MultiArray, topic, cb` |
| timer | 309 | `ExperimentLogger.__init__` | `1.0 / self.sample_rate_hz, self.sample` |

**declare_parameter** ×17: `'output_dir'`, `'run_dir_override'`, `'experiment_name'`, `'joint_names'`, `'joint_state_topic'`, `'multi_distance_topic'`, `'qdot_nom_topic'`, `'qdot_cmd_topic'`, `'torque_cmd_topic'`, `'robot_state_topic'`, `'cbf_status_topic'`, `'iso_safety_topic'`, `'torque_saturation_topic'`, `'qddot_nom_topic'`, `'qddot_safe_topic'`, `'per_link_distances_topic'`, `'truth_topic'`


### `franka_experiments/franka_experiments/nodes/frame_grabber.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| sub | 79 | `FrameGrabber.__init__` | `Image, image_topic, self._img_cb` |
| sub | 80 | `FrameGrabber.__init__` | `Float64MultiArray, cbf_topic, self._cbf_cb` |
| timer | 83 | `FrameGrabber.__init__` | `period, self._save_tick` |

**declare_parameter** ×6: `'image_topic'`, `'cbf_status_topic'`, `'output_dir'`, `'save_rate_hz'`, `'only_when_cbf_active'`, `'annotate'`


### `franka_experiments/franka_experiments/nodes/handeye_calibration_node.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| tf_listen | 461 | `HandEyeCalibrationNode.__init__` | `self._tf_buffer, self` |
| tf_static | 466 | `HandEyeCalibrationNode.__init__` | `self` |
| sub | 472 | `HandEyeCalibrationNode.__init__` | `AprilTagDetectionArray, '/detections', self._detections_cb` |
| tf_lookup | 581 | `HandEyeCalibrationNode._startup_tf_check` | `self._base_frame, self._tool_frame, Time()` |
| tf_lookup | 609 | `HandEyeCalibrationNode._startup_tf_check` | `self._cam_frame, self._tag_frame, Time()` |
| pub | 924 | `HandEyeCalibrationNode._init_velocity_auto_motion` | `Float64MultiArray, self._auto_tracking_topic, 10` |
| timer | 931 | `HandEyeCalibrationNode._init_velocity_auto_motion` | `period, self._auto_timer_cb` |
| tf_lookup | 1431 | `HandEyeCalibrationNode._wait_tag_stable` | `self._cam_frame, self._tag_frame, Time()` |
| tf_lookup | 1670 | `HandEyeCalibrationNode._capture_sample` | `self._cam_frame, self._tag_frame, Time()` |
| tf_lookup | 1685 | `HandEyeCalibrationNode._capture_sample` | `self._base_frame, self._tool_frame, tag_time` |
| tf_lookup | 1695 | `HandEyeCalibrationNode._capture_sample` | `self._base_frame, self._tool_frame, Time()` |
| tf_send | 2525 | `HandEyeCalibrationNode._publish_calibration_tf` | `msgs` |

**declare_parameter** ×52: `'robot_base_frame'`, `'robot_tool_frame'`, `'camera_frame'`, `'tag_frame'`, `'output_parent'`, `'output_child'`, `'tf_timeout_s'`, `'visualize'`, `'yaml_style'`, `'auto_motion'`, `'dataset_file'`, `'num_samples'`, `'settle_time_s'`, `'tag_stability_samples'`, `'tag_stability_max_spread_m'`, `'tag_stability_max_spread_deg'`, `'validation_ratio'`, `'rotation_weight'`, `'translation_weight'`, `'refine'`, `'velocity_scaling'`, `'workspace_center_x'`, `'workspace_center_y'`, `'workspace_center_z'`, `'workspace_extent'`, `'max_tilt_deg'`, `'tracking_topic'`, `'joint_state_topic'`, `'ee_frame'`, `'rate_hz'`, `'workspace_bounds'`, `'num_waypoints'`, `'min_distance'`, `'segment_duration_s'`, `'hold_time_s'`, `'kp_cart'`, `'damping'`, `'qdot_max'`, `'lpf_alpha'`, `'qdot_acc_max'`, `'orient_slerp_dur_s'`, `'warmup_s'`, `'ramp_s'`, `'capture_pos_tol_m'`, `'min_safe_z_m'`, `'orientation_excitation_enabled'`, `'orientation_kp'`, `'capture_rot_tol_deg'`, `'orientation_roll_deg_max'`, `'orientation_pitch_deg_max'`, `'orientation_yaw_deg_max'`, `'orientation_confidence_threshold'`


### `franka_experiments/franka_experiments/nodes/handeye_eye_in_hand_node.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| tf_listen | 210 | `HandEyeEyeInHandNode.__init__` | `self._tf_buffer, self` |
| pub | 254 | `HandEyeEyeInHandNode._init_motion` | `Float64MultiArray, topic, 10` |
| sub | 255 | `HandEyeEyeInHandNode._init_motion` | `AprilTagDetectionArray, '/detections', self._detections_cb` |
| timer | 294 | `HandEyeEyeInHandNode._init_motion` | `self._dt, self._tick` |
| tf_lookup | 314 | `HandEyeEyeInHandNode._lookup_cam_tag` | `self._p['camera_frame'], self._p['tag_frame'], Time()` |
| tf_lookup | 329 | `HandEyeEyeInHandNode._try_link_optical` | `self._p['camera_link_frame'], self._p['camera_frame'], Time()` |

### `franka_experiments/franka_experiments/nodes/image_rectifier_node.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| sub | 43 | `ImageRectifier.__init__` | `CameraInfo, 'camera_info', self._info_cb` |
| sub | 45 | `ImageRectifier.__init__` | `Image, 'image_raw', self._image_cb` |
| pub | 47 | `ImageRectifier.__init__` | `Image, 'image_rect', 5` |
| pub | 48 | `ImageRectifier.__init__` | `CameraInfo, 'camera_info_rect', 5` |

### `franka_experiments/franka_experiments/nodes/iso_evidence_logger.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| cbgroup | 214 | `ISOEvidenceLogger.__init__` | `` |
| cbgroup | 215 | `ISOEvidenceLogger.__init__` | `` |
| sub | 217 | `ISOEvidenceLogger.__init__` | `JointState, topics.get('joint_states_fast', topics['joint_states_topic']), self._on_js, callback_group=grp_io` |
| sub | 221 | `ISOEvidenceLogger.__init__` | `MultiLinkDistance, topics['per_link_distances'], self._on_dist, callback_group=grp_io` |
| sub | 230 | `ISOEvidenceLogger.__init__` | `Float64MultiArray, topics.get(key, default), cb, callback_group=grp_io` |
| timer | 234 | `ISOEvidenceLogger.__init__` | `1.0 / self._rate, self._tick, callback_group=grp_tick` |
| executor | 607 | `main` | `` |

### `franka_experiments/franka_experiments/nodes/iso_safety_monitor.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| cbgroup | 158 | `ISOSafetyMonitor.__init__` | `` |
| cbgroup | 159 | `ISOSafetyMonitor.__init__` | `` |
| sub | 160 | `ISOSafetyMonitor.__init__` | `JointState, topics.get('joint_states_fast', topics['joint_states_topic']), self._on_js, callback_group=grp_io` |
| sub | 165 | `ISOSafetyMonitor.__init__` | `MultiLinkDistance, topics['per_link_distances'], self._on_dist, callback_group=grp_io` |
| sub | 169 | `ISOSafetyMonitor.__init__` | `Float64MultiArray, topics.get('cbf_status', '/NS_1/cbf_status'), self._on_status, callback_group=grp_io` |
| pub | 173 | `ISOSafetyMonitor.__init__` | `Float64MultiArray, topics.get('iso_safety', '/NS_1/iso_safety'), 10` |
| srv | 177 | `ISOSafetyMonitor.__init__` | `Trigger, topics.get('safety_reset', '/NS_1/safety_reset'), self._on_reset, callback_group=grp_io` |
| timer | 181 | `ISOSafetyMonitor.__init__` | `1.0 / P.qp_rate_hz, self._tick, callback_group=grp_tick` |
| executor | 402 | `main` | `` |

**declare_parameter** ×1: `'iso_tcp_link'`


### `franka_experiments/franka_experiments/nodes/pentagon_qddot_commander.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| sub | 633 | `PentagonQddotCommander.__init__` | `JointState, js_topic, self._js_cb` |
| sub | 634 | `PentagonQddotCommander.__init__` | `Float64MultiArray, str(self.get_parameter('cbf_status_topic').value), self._cbf_status_cb` |
| sub | 638 | `PentagonQddotCommander.__init__` | `Float64MultiArray, str(self.get_parameter('iso_safety_topic').value), self._iso_safety_cb` |
| sub | 644 | `PentagonQddotCommander.__init__` | `JointState, effort_topic, self._effort_cb` |
| pub | 648 | `PentagonQddotCommander.__init__` | `Float64MultiArray, qddot_topic, 10` |
| pub | 649 | `PentagonQddotCommander.__init__` | `SensorJointState, q_des_topic, 10` |
| pub | 653 | `PentagonQddotCommander.__init__` | `PointStamped, ee_des_topic, 1` |
| pub | 654 | `PentagonQddotCommander.__init__` | `PointStamped, ee_act_topic, 1` |
| timer | 664 | `PentagonQddotCommander.__init__` | `self._dt, self._tick` |

**declare_parameter** ×67: `'qddot_safe_topic'`, `'q_des_topic'`, `'ee_desired_topic'`, `'ee_actual_topic'`, `'ee_track_frame'`, `'reset_thr_m'`, `'joint_state_topic'`, `'ee_frame'`, `'rate_hz'`, `'warmup_s'`, `'approach_time'`, `'timing_law'`, `'center_xyz'`, `'radius'`, `'path_type'`, `'plane'`, `'plane_frame'`, `'cycle_time'`, `'smoothness'`, `'lissajous_a'`, `'lissajous_b'`, `'kp_cart'`, `'kd_cart'`, `'kp_rot'`, `'kd_rot'`, `'log_dir'`, `'joint_effort_topic'`, `'k_sync_pos'`, `'k_sync_vel'`, `'soft_reset_thr'`, `'hard_reset_thr'`, `'avoid_near_m'`, `'soft_reset_alpha'`, `'k_null'`, `'d_null'`, `'lambda_sq_min'`, `'lambda_sq_max'`, `'manip_thr'`, `'governor_enabled'`, `'governor_r_full'`, `'governor_r_stop'`, `'governor_sigma_min'`, `'governor_timeout'`, `'cbf_status_topic'`, `'iso_safety_topic'`, `'cart_err_max'`, `'q_des_max_error'`, `'dq_des_max'`, `'dq_filter_alpha'`, `'isolation_test'`, `'isolation_amp'`, `'isolation_seg_s'`, `'isolation_freq_hz'`, `'isolation_kp'`, `'isolation_kd'`, `'singularity_recovery_enabled'`, `'recovery_sigma_thr'`, `'recovery_progress_eps'`, `'recovery_err_thr'`, `'recovery_stall_s'`, `'recovery_vmax'`, `'recovery_amax'`, `'recovery_min_s'`, `'recovery_settle_s'`, `'recovery_cooldown_s'`, `'recovery_kp'`, `'recovery_kd'`


### `franka_experiments/franka_experiments/nodes/pentagon_torque_commander.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| sub | 207 | `PentagonTorqueCommander.__init__` | `JointState, js_topic, self._joint_state_cb` |
| pub | 225 | `PentagonTorqueCommander.__init__` | `Float64MultiArray, pub_topic, 10` |
| timer | 226 | `PentagonTorqueCommander.__init__` | `1.0 / self.rate_hz, self._timer_cb` |

**declare_parameter** ×18: `'cbf_mode'`, `'qddot_nom_topic'`, `'torque_topic'`, `'joint_state_topic'`, `'ee_frame'`, `'rate_hz'`, `'warmup_s'`, `'ramp_s'`, `'center_xyz'`, `'radius'`, `'plane'`, `'plane_frame'`, `'cycle_time'`, `'kp_cart'`, `'kd_cart'`, `'kp_rot'`, `'kd_rot'`, `'smoothness'`


### `franka_experiments/franka_experiments/nodes/qddot_to_torque.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| pub | 103 | `QddotToTorqueNode.__init__` | `Float64MultiArray, topics.get('torque_saturation', '/NS_1/torque_saturation'), 10` |
| pub | 140 | `QddotToTorqueNode.__init__` | `Float64MultiArray, torque_out_topic, 10` |
| sub | 150 | `QddotToTorqueNode.__init__` | `JointState, topics.get('joint_states_fast', topics['joint_states_topic']), self._on_joint_state` |
| sub | 156 | `QddotToTorqueNode.__init__` | `Float64MultiArray, topics['qddot_safe'], self._on_qddot_nom` |

### `franka_experiments/franka_experiments/nodes/real_time_distance.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| tf_listen | 175 | `RealTimeDistance.__init__` | `self.tf_buffer, self` |
| sub | 493 | `RealTimeDistance.__init__` | `Image, depth_topic, self.depth_callback` |
| sub | 494 | `RealTimeDistance.__init__` | `CameraInfo, info_topic, self.camera_info_callback` |
| pub | 498 | `RealTimeDistance.__init__` | `MultiDistance, topics_cfg.get('multi_distance', '/human_robot/multi_distance'), 10` |
| pub | 501 | `RealTimeDistance.__init__` | `HumanRobotDistance, topics_cfg.get('distance', '/human_robot/distance'), 10` |
| pub | 504 | `RealTimeDistance.__init__` | `MultiLinkDistance, topics_cfg.get('per_link_distances', '/cbf/per_link_distances'), _be_qos` |
| pub | 526 | `RealTimeDistance.__init__` | `Image, overlay_topic, 10` |
| timer | 528 | `RealTimeDistance.__init__` | `_vis_period, self.visualize` |

**declare_parameter** ×2: `'control_config_path'`, `'multi_obstacle_k'`


### `franka_experiments/franka_experiments/nodes/rl_policy_commander.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| sub | 350 | `RLPolicyCommander.__init__` | `JointState, js_topic, self._js_cb` |
| sub | 352 | `RLPolicyCommander.__init__` | `MultiLinkDistance, dist_topic, self._obs_cb` |
| sub | 355 | `RLPolicyCommander.__init__` | `Float64MultiArray, self.get_parameter('cbf_status_topic').value, self._cbf_status_cb` |
| sub | 358 | `RLPolicyCommander.__init__` | `PointStamped, target_topic, self._target_cb` |
| pub | 361 | `RLPolicyCommander.__init__` | `Float64MultiArray, qddot_topic, 10` |
| pub | 362 | `RLPolicyCommander.__init__` | `Float64MultiArray, status_topic, 10` |
| timer | 369 | `RLPolicyCommander.__init__` | `self._dt, self._tick` |

**declare_parameter** ×25: `'onnx_model'`, `'sim_root'`, `'sim_config'`, `'qddot_nom_topic'`, `'joint_state_topic'`, `'per_link_distances_topic'`, `'cbf_status_topic'`, `'status_topic'`, `'target_topic'`, `'ee_frame'`, `'rate_hz'`, `'warmup_s'`, `'joint_state_timeout'`, `'distance_timeout'`, `'target_xyz'`, `'target_sequence'`, `'target_tol'`, `'dwell_s'`, `'stop_on_success'`, `'obstacle_radius'`, `'no_obstacle_xyz'`, `'distance_links'`, `'action_scale'`, `'log_csv'`, `'log_dir'`


### `franka_experiments/franka_experiments/nodes/tag_overlay_node.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| sub | 66 | `TagOverlay.__init__` | `CameraInfo, 'camera_info', self._info_cb` |
| sub | 67 | `TagOverlay.__init__` | `AprilTagDetectionArray, '/detections', self._det_cb` |
| sub | 68 | `TagOverlay.__init__` | `Image, 'image', self._image_cb` |
| pub | 69 | `TagOverlay.__init__` | `Image, 'image_overlay', 5` |

**declare_parameter** ×6: `'tag_id'`, `'tag_size'`, `'max_offaxis_deg'`, `'min_decision_margin'`, `'distance_min'`, `'distance_max'`


### `franka_experiments/franka_experiments/nodes/trajectory_overlay_node.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| pub | 202 | `TrajectoryOverlayNode.__init__` | `Image, out_topic, 1` |
| sub | 204 | `TrajectoryOverlayNode.__init__` | `CameraInfo, info_topic, self._on_info` |
| sub | 206 | `TrajectoryOverlayNode.__init__` | `Image, img_topic, self._on_image` |
| sub | 208 | `TrajectoryOverlayNode.__init__` | `PointStamped, des_topic, self._on_desired` |
| sub | 209 | `TrajectoryOverlayNode.__init__` | `PointStamped, act_topic, self._on_actual` |
| timer | 212 | `TrajectoryOverlayNode.__init__` | `5.0, self._watchdog` |

### `franka_experiments/franka_experiments/nodes/trajectory_visualization_node.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| pub | 108 | `TrajectoryVisualizationNode.__init__` | `MarkerArray, mrk_topic, 1` |
| sub | 109 | `TrajectoryVisualizationNode.__init__` | `PointStamped, des_topic, self._on_desired` |
| sub | 111 | `TrajectoryVisualizationNode.__init__` | `PointStamped, act_topic, self._on_actual` |
| timer | 113 | `TrajectoryVisualizationNode.__init__` | `1.0 / rate, self._draw` |

### `franka_experiments/franka_experiments/utils/kinematics.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| timer | 323 | `JointStateManager.__init__` | `1.0, self._discover_topic` |
| sub | 330 | `JointStateManager.__init__` | `JointState, topic_param, self._joint_state_cb` |
| sub | 406 | `JointStateManager._subscribe` | `JointState, topic, self._joint_state_cb` |

### `franka_experiments/franka_experiments/utils/params.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|

**declare_parameter** ×5: `name`, `name`, `name`, `name`, `name`


### `franka_experiments/franka_experiments/utils/ros_setup.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| cli | 34 | `fetch_robot_description` | `GetParameters, service_name` |

### `franka_experiments/franka_experiments/utils/tf_manager.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| tf_lookup | 164 | `TFManager._lookup_one` | `self._base, link_name, tf_time` |
| tf_lookup | 174 | `TFManager._lookup_one` | `self._base, link_name, RclpyTime()` |

### `franka_experiments/scripts/cbf_phase4_check.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| sub | 88 | `Phase4Check.__init__` | `MultiLinkDistance, args.distance_topic, self._on_dist` |
| sub | 91 | `Phase4Check.__init__` | `Float64MultiArray, args.status_topic, self._on_status` |

### `franka_experiments/scripts/iso_constants_measure.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| sub | 207 | `cmd_stop._Runner.__init__` | `JointState, args.joint_states, self._js` |
| sub | 209 | `cmd_stop._Runner.__init__` | `Float64MultiArray, args.qddot_safe, self._safe` |
| pub | 212 | `cmd_stop._Runner.__init__` | `Float64MultiArray, args.qddot_nom, 10` |
| tf_listen | 466 | `cmd_uncertainty._Probe.__init__` | `self.buf, self` |
| sub | 473 | `cmd_uncertainty._Probe.__init__` | `CameraInfo, args.info_topic, self._info` |
| sub | 474 | `cmd_uncertainty._Probe.__init__` | `Image, args.depth_topic, self._depth` |

### `franka_experiments/scripts/iso_preflight_check.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| sub | 201 | `check_topic` | `Float64MultiArray, topic, lambda _m: seen.__setitem__('n', seen['n'] + 1)` |

### `franka_experiments/scripts/latency_budget.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| sub | 157 | `cmd_cam.Probe.__init__` | `Image, t, lambda m, t=t: self.cb(m, t)` |
| sub | 160 | `cmd_cam.Probe.__init__` | `Metadata, args.metadata, self.mcb` |
| pub | 586 | `cmd_dds.Pub.__init__` | `Image, '/lb/depth', 10` |
| pub | 587 | `cmd_dds.Pub.__init__` | `MultiLinkDistance, '/lb/per_link', be` |
| pub | 588 | `cmd_dds.Pub.__init__` | `Float64MultiArray, '/lb/qddot', 10` |
| timer | 593 | `cmd_dds.Pub.__init__` | `1.0 / 30.0, self.slow` |
| timer | 594 | `cmd_dds.Pub.__init__` | `1.0 / 100.0, self.fast` |
| sub | 620 | `cmd_dds.Sub.__init__` | `Image, '/lb/depth', self.on_img` |
| sub | 621 | `cmd_dds.Sub.__init__` | `MultiLinkDistance, '/lb/per_link', self.on_mld` |
| sub | 622 | `cmd_dds.Sub.__init__` | `Float64MultiArray, '/lb/qddot', self.on_qdd` |

### `franka_experiments/scripts/pipeline_rate_monitor.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| sub | 203 | `PipelineRateMonitor.rescan` | `msg_type, topic, lambda _m, p=probe: p.tick(time.perf_counter())` |

### `franka_experiments/scripts/range_noise_calibration.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| sub | 188 | `cmd_capture.Probe.__init__` | `Image, args.depth_topic, self._on_depth` |
| sub | 189 | `cmd_capture.Probe.__init__` | `CameraInfo, args.info_topic, self._on_info` |

### `franka_experiments/scripts/watch_obstacle_tracks.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| sub | 66 | `WatchObstacleTracks.__init__` | `MultiLinkDistance, topic, self._on_msg` |
| timer | 69 | `WatchObstacleTracks.__init__` | `1.0 / max(0.1, float(self.get_parameter('rate_hz').value)), self._print` |

**declare_parameter** ×4: `'topic'`, `'rate_hz'`, `'only_tracked'`, `'gap_max_m'`


### `franka_simulation_experiments/scripts/emulated_perception.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| pub | 60 | `EmulatedPerception.__init__` | `MultiLinkDistance, '/cbf/per_link_distances', QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)` |
| sub | 63 | `EmulatedPerception.__init__` | `JointState, '/NS_1/franka/joint_states', self._js` |
| sub | 64 | `EmulatedPerception.__init__` | `PoseArray, '/sim/truth/obstacle_poses', self._poses` |
| sub | 65 | `EmulatedPerception.__init__` | `PoseArray, '/sim/truth/obstacle_poses_vel', self._vels` |
| timer | 66 | `EmulatedPerception.__init__` | `1.0 / self.fps, self._tick` |

### `franka_simulation_experiments/scripts/episode_supervisor.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| cli | 72 | `EpisodeSupervisor.__init__` | `ListControllers, dp('controller_manager', '/NS_1/controller_manager').value + '/list_c…` |
| pub | 84 | `EpisodeSupervisor.__init__` | `Float64MultiArray, '/sim/episode', QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)` |
| pub | 88 | `EpisodeSupervisor.__init__` | `Float64, '/sim/episode_start', 10` |
| sub | 91 | `EpisodeSupervisor.__init__` | `Float64MultiArray, topic, lambda m, k=key: self._seen(k)` |
| sub | 95 | `EpisodeSupervisor.__init__` | `MultiLinkDistance, '/cbf/per_link_distances', lambda m: self._seen('per_link_distances')` |
| timer | 99 | `EpisodeSupervisor.__init__` | `0.02, self._tick` |

### `franka_simulation_experiments/scripts/ground_truth_node.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| pub | 54 | `GroundTruth.__init__` | `Float64MultiArray, '/sim/truth/obstacles', 10` |
| pub | 55 | `GroundTruth.__init__` | `Float64MultiArray, '/sim/truth/cp_gaps', 10` |
| sub | 56 | `GroundTruth.__init__` | `PoseArray, '/sim/truth/obstacle_poses', self._poses` |
| sub | 57 | `GroundTruth.__init__` | `PoseArray, '/sim/truth/obstacle_poses_vel', self._vels` |
| sub | 58 | `GroundTruth.__init__` | `JointState, '/NS_1/franka/joint_states', self._js` |
| timer | 59 | `GroundTruth.__init__` | `0.01, self._publish` |
| timer | 60 | `GroundTruth.__init__` | `5.0, self.save` |

### `franka_simulation_experiments/scripts/gz_trajectory_markers.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| sub | 175 | `GzTrajectoryMarkers.__init__` | `PointStamped, des_topic, self._on_desired` |
| sub | 176 | `GzTrajectoryMarkers.__init__` | `PointStamped, act_topic, self._on_actual` |
| timer | 177 | `GzTrajectoryMarkers.__init__` | `1.0 / max(redraw_hz, 0.1), self._redraw` |

### `franka_simulation_experiments/scripts/reflex_monitor.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| pub | 62 | `ReflexMonitor.__init__` | `Float64MultiArray, '/sim/reflex', 10` |
| sub | 63 | `ReflexMonitor.__init__` | `JointState, '/NS_1/franka/joint_states', self._js` |
| sub | 64 | `ReflexMonitor.__init__` | `Clock, '/clock', self._on_clock` |
| timer | 65 | `ReflexMonitor.__init__` | `0.1, self._publish` |
| timer | 66 | `ReflexMonitor.__init__` | `2.0, self.write_summary` |

**declare_parameter** ×3: `'run_dir'`, `'urdf_path'`, `'gravity'`


### `franka_simulation_experiments/scripts/scenario_commander.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| pub | 41 | `ScenarioCommander.__init__` | `Float64MultiArray, '/NS_1/qddot_nom', 10` |
| pub | 42 | `ScenarioCommander.__init__` | `PointStamped, '/NS_1/ee_desired', 10` |
| pub | 43 | `ScenarioCommander.__init__` | `PointStamped, '/NS_1/ee_actual', 10` |
| sub | 44 | `ScenarioCommander.__init__` | `JointState, '/NS_1/franka/joint_states', self._js` |
| sub | 45 | `ScenarioCommander.__init__` | `Float64MultiArray, '/sim/episode', self._ep` |
| timer | 46 | `ScenarioCommander.__init__` | `1.0 / rate, self._tick` |

**declare_parameter** ×3: `'nominal_json'`, `'q0'`, `'rate_hz'`


### `franka_simulation_experiments/scripts/simexp/graph_contract.py`

| Kind | Line | Enclosing def | Arguments (as written) |
|---|---:|---|---|
| sub | 99 | `capture_tf` | `TFMessage, topic, cb_for(topic, static)` |
