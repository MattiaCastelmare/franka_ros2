# Module definitions — top-level classes and functions

> Top-level `class`/`def` per module (methods counted, not listed). Used as the checklist of symbols each chapter must account for.

**Total top-level symbols: 791.**

### `franka_experiments/franka_experiments/nodes/capsule_overlay_node.py`

Classes: `CapsuleOverlayNode` (L75, 13 methods)

Functions: `main` (L776)

### `franka_experiments/franka_experiments/nodes/cbf_OSCBF_filter.py`

Classes: `_RobotDynamics` (L106, 2 methods), `_OSCBFQPBuilder` (L191, 3 methods), `OSCBFFilterNode` (L539, 13 methods)

Functions: `_load_control_yaml` (L78), `_build_urdf_no_hand` (L86), `_build_joint_position_cbf` (L307), `_build_joint_velocity_cbf` (L353), `_build_ee_workspace_cbf` (L384), `_build_obstacle_cbf` (L431), `main` (L958)

### `franka_experiments/franka_experiments/nodes/cbf_safety_filter.py`

Classes: `CBFSafetyFilter` (L136, 26 methods)

Functions: `main` (L2070)

### `franka_experiments/franka_experiments/nodes/cbf_velocity_filter.py`

Classes: `CbfVelocityFilter` (L60, 9 methods)

Functions: `main` (L612)

### `franka_experiments/franka_experiments/nodes/distance_visualization_node.py`

Classes: `_VizCP` (L64, 0 methods), `DistanceVisualization` (L71, 9 methods)

Functions: `main` (L390)

### `franka_experiments/franka_experiments/nodes/ee_circle_velocity_commander.py`

Classes: `EECircleVelocityCommander` (L96, 4 methods)

Functions: `quintic_ramp` (L68), `quintic_ramp_integral` (L78), `main` (L467)

### `franka_experiments/franka_experiments/nodes/ee_pentagon_velocity_commander.py`

Classes: `EEPentagonVelocityCommander` (L67, 3 methods)

Functions: `main` (L355)

### `franka_experiments/franka_experiments/nodes/ee_random_waypoints_velocity_commander.py`

Classes: `EERandomWaypointsVelocityCommander` (L72, 7 methods)

Functions: `main` (L502)

### `franka_experiments/franka_experiments/nodes/experiment_logger.py`

Classes: `ExperimentLogger` (L107, 35 methods)

Functions: `main` (L1110)

### `franka_experiments/franka_experiments/nodes/frame_grabber.py`

Classes: `FrameGrabber` (L41, 5 methods)

Functions: `main` (L150)

### `franka_experiments/franka_experiments/nodes/handeye_calibration_node.py`

Classes: `HandEyeCalibrationNode` (L333, 30 methods)

Functions: `transform_to_matrix` (L157), `_quat_to_rot` (L167), `_rot_to_quat` (L182), `matrix_to_translation_quaternion` (L214), `invert_matrix` (L221), `rotation_error_deg` (L231), `_mat_to_rvec_tvec` (L239), `_rvec_tvec_to_mat` (L246), `_axis_angle_rotation` (L255), `_so3_log` (L265), `_slerp_so3` (L303), `main` (L2555)

### `franka_experiments/franka_experiments/nodes/handeye_eye_in_hand_node.py`

Classes: `HandEyeEyeInHandNode` (L122, 31 methods)

Functions: `_quat_dict` (L89), `_transform_dict` (L95), `_tf_to_matrix` (L104), `_offaxis` (L110), `main` (L957)

### `franka_experiments/franka_experiments/nodes/image_rectifier_node.py`

Classes: `ImageRectifier` (L34, 3 methods)

Functions: `main` (L89)

### `franka_experiments/franka_experiments/nodes/iso_evidence_logger.py`

Classes: `ISOEvidenceLogger` (L136, 17 methods)

Functions: `_git_describe` (L591), `main` (L604)

### `franka_experiments/franka_experiments/nodes/iso_safety_monitor.py`

Classes: `ISOSafetyMonitor` (L124, 9 methods)

Functions: `trip_ticks` (L115), `main` (L399)

### `franka_experiments/franka_experiments/nodes/pentagon_qddot_commander.py`

Classes: `PentagonQddotCommander` (L84, 20 methods)

Functions: `main` (L1683)

### `franka_experiments/franka_experiments/nodes/pentagon_torque_commander.py`

Classes: `PentagonTorqueCommander` (L57, 4 methods)

Functions: `main` (L491)

### `franka_experiments/franka_experiments/nodes/qddot_to_torque.py`

Classes: `QddotToTorqueNode` (L57, 6 methods)

Functions: `main` (L312)

### `franka_experiments/franka_experiments/nodes/real_time_distance.py`

Classes: `RealTimeDistance` (L94, 19 methods)

Functions: `main` (L1245)

### `franka_experiments/franka_experiments/nodes/rl_policy_commander.py`

Classes: `RLPolicyCommander` (L106, 13 methods)

Functions: `main` (L748)

### `franka_experiments/franka_experiments/nodes/tag_overlay_node.py`

Classes: `TagOverlay` (L49, 5 methods)

Functions: `_text` (L40), `main` (L142)

### `franka_experiments/franka_experiments/nodes/trajectory_overlay_node.py`

Classes: `TrajectoryOverlayNode` (L107, 15 methods)

Functions: `_text` (L96), `_share_dir` (L439), `main` (L444)

### `franka_experiments/franka_experiments/nodes/trajectory_visualization_node.py`

Classes: `TrajectoryVisualizationNode` (L60, 6 methods)

Functions: `main` (L166)

### `franka_experiments/franka_experiments/utils/avoidance.py`


Functions: `influence_weight` (L39), `tangential_redirect` (L53), `feasibility_beta_target` (L111), `rate_limited` (L143), `_unit_ramp` (L157), `_clamp01` (L164)

### `franka_experiments/franka_experiments/utils/avoidance_math.py`


Functions: `clip_aabb` (L30), `outward_normal_aabb` (L37), `closest_points_segment_aabb` (L54), `closest_points_on_segments` (L93), `tangential_dir` (L129), `capsule_segment_distance_to_obb` (L148), `quat_from_z_axis_to_direction` (L254), `enforce_halfspace_with_box` (L278), `check_halfspace_feasible` (L354), `check_all_halfspaces_feasible` (L375), `capsule_risk_weight` (L400), `solve_cbf_qp_1constraint_box` (L437), `pocs_project_halfspaces_with_box` (L559), `point_jacobian_world` (L627), `compute_closest_constraint_jacobian` (L654), `build_reduced_pinocchio_model_from_urdf` (L719), `build_capsules_for_link_pairs` (L759), `build_joint_to_joint_capsules` (L815), `ordered_joint_positions_from_joint_state` (L939), `filtered_collision_objects_from_planning_scene` (L953), `quat_to_rot_matrix` (L977), `stable_sign_from_id` (L1002), `smooth_alpha` (L1011), `smoothstep01` (L1031), `ramp_smooth` (L1037), `distance_capsule_to_collision_object_boxes` (L1062), `make_capsule_markers` (L1166), `make_distance_markers` (L1236)

### `franka_experiments/franka_experiments/utils/ballistic_prediction.py`

Classes: `PredictionConfig` (L71, 1 methods)

Functions: `_is_fast` (L91), `_ballistic` (L104), `predict_trajectory` (L111), `add_predicted_hits` (L122)

### `franka_experiments/franka_experiments/utils/calibration_check.py`

Classes: `CalibrationResidual` (L83, 2 methods)

Functions: `calibration_residual` (L153)

### `franka_experiments/franka_experiments/utils/cbf_constraints.py`

Classes: `CBFParams` (L107, 0 methods)

Functions: `build_simple_accel_cbf_constraints` (L20), `build_hocbf_constraints` (L118), `predict_state` (L203)

### `franka_experiments/franka_experiments/utils/cbf_diagnostics.py`


Functions: `family_violations` (L45)

### `franka_experiments/franka_experiments/utils/cbf_evasion.py`


Functions: `normal_brake_authority` (L86), `evasion_urgency` (L123), `escape_direction` (L179), `evasion_bias` (L259)

### `franka_experiments/franka_experiments/utils/cbf_hard_limits.py`


Functions: `fr3_velocity_envelope` (L106), `hard_accel_box` (L129), `position_velocity_accel_box` (L232), `apply_slew_limit` (L296), `workspace_face_rows` (L355), `velocity_accel_box` (L388)

### `franka_experiments/franka_experiments/utils/cbf_qp.py`

Classes: `CBFQP` (L7, 2 methods)

### `franka_experiments/franka_experiments/utils/cbf_qp_assembly.py`


Functions: `build_osqp_A` (L38), `build_qp_objective` (L86), `build_osqp_bounds` (L117), `build_row_rhs` (L151), `pad_rows_to_block` (L266), `_tb_diag` (L321), `tangential_bias` (L340), `accept_iterate` (L541), `box_only_solve` (L566), `braking_command` (L592)

### `franka_experiments/franka_experiments/utils/cbf_singularity.py`

Classes: `SingularityRowBuilder` (L64, 5 methods)

### `franka_experiments/franka_experiments/utils/cbf_state_rows.py`

Classes: `SelfCollisionRowBuilder` (L951, 7 methods), `Shadow` (L1175, 0 methods), `ShadowRowBuilder` (L1193, 3 methods), `JointSnap` (L1768, 0 methods), `NomSnap` (L1774, 0 methods), `Obstacle` (L1779, 0 methods), `ObstacleSnap` (L1820, 0 methods), `ConstraintSnap` (L1842, 0 methods), `ConstraintBuilder` (L2104, 13 methods)

Functions: `joint_limit_rows` (L59), `retreat_cap_speed` (L152), `retreat_cap_rhs` (L244), `link_speed_cap` (L266), `obstacle_link_speed_cap` (L292), `link_speed_row` (L311), `retreat_speed_available` (L369), `retreat_accel_available` (L397), `retreat_speed_reachable` (L410), `retreat_displacement_reachable` (L441), `velocity_feedforward_terms` (L498), `velocity_standoff` (L537), `uncertainty_margin` (L563), `latency_compensation_terms` (L646), `sensor_range_uncertainty` (L740), `joint_limit_risk_margin` (L867), `risk_slack_weight` (L909), `build_self_collision_builder` (L1369), `_approach_weight` (L1496), `row_slack_slots` (L1531), `evidence_class` (L1568), `evidence_margin` (L1588), `fitted_rate` (L1602), `measured_curvature_radius` (L1629), `segment_point_distance` (L1656), `in_self_mask` (L1665), `in_self_shadow` (L1710), `memory_surface_gap` (L1741), `build_optional_row_builders` (L1994)

### `franka_experiments/franka_experiments/utils/cbf_utils.py`


Functions: `select_gamma` (L15)

### `franka_experiments/franka_experiments/utils/cbf_zones.py`

Classes: `ZoneGains` (L141, 0 methods), `ZoneLadder` (L150, 8 methods)

Functions: `smoothstep` (L120), `ladder_from_params` (L318)

### `franka_experiments/franka_experiments/utils/config.py`


Functions: `resolve_log_dir` (L44), `load_package_config` (L93), `load_config_file` (L112), `load_extrinsics` (L128), `load_camera_info_yaml` (L160), `load_franka_config_defaults` (L187), `load_launch_defaults` (L237), `load_package_yaml` (L272), `load_franka_joint_limits` (L295), `load_cbf_config` (L773)

### `franka_experiments/franka_experiments/utils/distance_engine.py`

Classes: `ObstacleHit` (L58, 0 methods), `ControlPointResult` (L73, 1 methods), `ObstacleCloud` (L128, 0 methods), `DistanceEngine` (L173, 15 methods)

Functions: `_subset_cloud` (L1000)

### `franka_experiments/franka_experiments/utils/distance_utils.py`


Functions: `point_to_segment_distance_with_projection` (L23), `define_robot_segments` (L34), `compute_closest_distance_from_segments` (L90), `get_robot_segments_from_transforms` (L165), `get_rotation_from_quaternion` (L188), `compute_direction_vector` (L198), `cam_to_base` (L209), `base_to_cam_z` (L218), `compute_roi` (L230), `define_control_points` (L255)

### `franka_experiments/franka_experiments/utils/evasion_direction.py`


Functions: `outrun_ratio` (L61), `blend_weight` (L88), `_plane_basis` (L107), `lateral_direction` (L117), `escape_direction` (L176), `evasion_bias` (L228)

### `franka_experiments/franka_experiments/utils/go_around.py`

Classes: `GoAroundState` (L64, 0 methods)

Functions: `_smoothstep` (L52), `_dls` (L57), `_pick_side` (L72), `go_around_step` (L86)

### `franka_experiments/franka_experiments/utils/handeye_solver.py`

Classes: `HandEyeResult` (L234, 1 methods)

Functions: `make_T` (L39), `inv_T` (L46), `rotvec` (L54), `from_rotvec` (L59), `angle_deg` (L63), `rotation_distance_deg` (L67), `_stack` (L71), `loop_closure_errors` (L77), `_relative_motions` (L102), `initial_guess` (L119), `refine` (L161), `mad_inlier_mask` (L196), `rotation_axis_sigmas` (L212), `solve` (L253), `look_at` (L296), `bootstrap_ee_poses` (L312), `_pose_distance` (L335), `orbit_camera_poses` (L341), `sample_orbit_candidates` (L369), `select_diverse_poses` (L402)

### `franka_experiments/franka_experiments/utils/iso_ssm.py`


Functions: `stopping_distance` (L51), `protective_separation` (L75), `ssm_speed_cap` (L112), `pfl_speed` (L162)

### `franka_experiments/franka_experiments/utils/kinematics.py`

Classes: `JointStateManager` (L273, 9 methods), `CBFKinematics` (L451, 9 methods)

Functions: `generate_urdf_from_xacro` (L38), `load_pinocchio_model` (L56), `load_pinocchio_model_from_urdf_path` (L74), `resolve_frame_id` (L90), `resolve_arm_joint_ids` (L103), `compute_ee_fk` (L120), `compute_arm_jacobian` (L130), `transform_ee_to_frame` (L146), `so3_log` (L182), `dls_solve` (L230), `configure_control_points_from_node` (L591), `build_urdf_with_sc` (L603), `build_urdf_no_hand` (L640)

### `franka_experiments/franka_experiments/utils/launch_support.py`


Functions: `resolve_controller_manager_name` (L38), `_generate_cbf_base_yaml` (L44), `generate_rt_controllers_yaml` (L105), `_ensure_urdf_cached` (L222), `_generate_torque_yaml` (L238), `write_temp_yaml` (L362), `rviz_config_for_namespace` (L377), `pick_controllers_yaml` (L407), `declare_robot_args` (L428), `declare_rt_blender_args` (L466), `declare_rt_torque_args` (L500), `build_wrapper_log_actions` (L527)

### `franka_experiments/franka_experiments/utils/livelock.py`

Classes: `ProgressWindow` (L61, 3 methods), `LivelockDetector` (L85, 5 methods)

### `franka_experiments/franka_experiments/utils/logging_utils.py`

Classes: `ThrottledLogger` (L24, 7 methods), `PerfTimer` (L81, 4 methods), `_TimerCtx` (L111, 3 methods)

Functions: `vec_to_str` (L69), `format_velocity_summary` (L126), `_zone_field` (L155), `_iso_num` (L179), `_gov_field` (L190), `format_cbf_diag` (L211)

### `franka_experiments/franka_experiments/utils/mask_builder.py`

Classes: `MaskBuilder` (L59, 5 methods)

Functions: `_kernel` (L302), `_dilate` (L308)

### `franka_experiments/franka_experiments/utils/math_utils.py`


Functions: `min_jerk` (L11), `cosine_ramp` (L36), `clamp_joints` (L49), `lpf` (L54), `rate_limit` (L59), `skew` (L66), `_safe_float` (L82)

### `franka_experiments/franka_experiments/utils/node_runtime.py`


Functions: `get_namespace_from_config` (L41), `build_namespaced_topic` (L59), `resolve_tracking_topic` (L69), `resolve_avoidance_topic` (L73), `resolve_topic_with_deprecated_alias` (L77), `run_node_main` (L98), `teardown` (L133), `_now_sec` (L156), `_stamp_to_sec` (L160), `_as_list7` (L165)

### `franka_experiments/franka_experiments/utils/obstacle_clusters.py`

Classes: `Cluster` (L113, 1 methods)

Functions: `voxel_downsample` (L158), `_neighbour_edges` (L185), `label_points` (L206), `cluster_obstacles` (L242)

### `franka_experiments/franka_experiments/utils/obstacle_sim.py`

Classes: `InjectedSphere` (L59, 5 methods)

### `franka_experiments/franka_experiments/utils/obstacle_track_pipeline.py`

Classes: `TrackInfo` (L59, 0 methods), `ObstacleTrackPipeline` (L75, 10 methods)

### `franka_experiments/franka_experiments/utils/obstacle_tracker.py`

Classes: `KalmanTrack` (L122, 14 methods), `IMMTrack` (L325, 16 methods), `TrackManager` (L682, 11 methods)

Functions: `transition` (L98), `process_noise` (L106), `_measurement` (L676)

### `franka_experiments/franka_experiments/utils/params.py`


Functions: `_fail` (L41), `declare_float` (L58), `declare_int` (L99), `declare_bool` (L137), `declare_str` (L154), `declare_vec` (L190), `declare_from_spec` (L226)

### `franka_experiments/franka_experiments/utils/perception_msgs.py`


Functions: `find_pt_confidence` (L57), `get_safety_zone` (L66), `build_cp_messages` (L93), `_cp_rows` (L300), `labelled_links` (L316), `annotate_track_fields` (L370), `no_obs_warn` (L452)

### `franka_experiments/franka_experiments/utils/rate_scaling.py`

Classes: `FrameRateEstimator` (L96, 7 methods)

Functions: `frames_for` (L46), `ema_alpha_for` (L61), `tau_for_alpha` (L81)

### `franka_experiments/franka_experiments/utils/rl_policy.py`

Classes: `ObsSpec` (L96, 4 methods)

Functions: `obs_spec_from_config` (L147), `build_observation` (L174), `control_point_geometry` (L236), `obstacle_velocity` (L277), `action_to_qddot` (L296), `obstacle_centre` (L319), `nearest_obstacle` (L326), `synthetic_obstacle` (L352), `qddot_max_from_limits` (L370), `joint_limits_mismatch` (L379), `load_yaml` (L404), `find_sim_root` (L409), `resolve_model_path` (L431), `find_latest_model` (L450), `resolve_sim_config_path` (L481)

### `franka_experiments/franka_experiments/utils/ros.py`


Functions: `selfcheck_run` (L93)

### `franka_experiments/franka_experiments/utils/ros_setup.py`


Functions: `fetch_robot_description` (L27), `init_pinocchio_and_capsules` (L62), `init_pinocchio_only` (L143), `make_joint_state_callback` (L159), `make_planning_scene_callback` (L188)

### `franka_experiments/franka_experiments/utils/rtd_debug.py`

Classes: `StageStats` (L44, 6 methods), `RtdProfiler` (L77, 7 methods), `_StageCM` (L147, 3 methods), `FrameTrace` (L165, 4 methods), `ThreadHeartbeat` (L193, 4 methods), `FreezeDetector` (L219, 2 methods), `MemoryTracker` (L285, 4 methods), `QueueMonitor` (L316, 8 methods)

Functions: `rtd_tag` (L26), `dump_all_threads` (L254), `dump_faulthandler` (L272), `install_faulthandler` (L278)

### `franka_experiments/franka_experiments/utils/run_metadata.py`


Functions: `write_runtime_metadata` (L12)

### `franka_experiments/franka_experiments/utils/self_collision.py`

Classes: `Capsule` (L37, 0 methods)

Functions: `rpy_to_matrix` (L47), `_origin_of` (L59), `parse_sc_capsules` (L72), `body_chain_index` (L107), `_adjacent` (L121), `build_capsule_pairs` (L136), `segment_segment_closest` (L180)

### `franka_experiments/franka_experiments/utils/self_detection.py`

Classes: `SelfDetectionMonitor` (L77, 7 methods)

### `franka_experiments/franka_experiments/utils/self_shadow.py`

Classes: `ShadowCloud` (L223, 2 methods), `ShadowRow` (L505, 0 methods), `VoidSphere` (L608, 0 methods)

Functions: `classify_masked_rays` (L150), `empty_cloud` (L262), `unproject` (L272), `reject_model_explained` (L287), `cluster_support` (L380), `extract_shadow_candidates` (L428), `nearest_candidate_per_cp` (L468), `shadow_barrier` (L509), `shadow_row` (L535), `void_spheres_from_rays` (L629), `_spread` (L681)

### `franka_experiments/franka_experiments/utils/simulation_imports.py`


Functions: `_ensure_simulation_utils_on_path` (L26)

### `franka_experiments/franka_experiments/utils/singularity_recovery.py`

Classes: `SingularityRecovery` (L103, 8 methods)

Functions: `quintic_coeffs` (L49), `quintic_eval` (L64), `plan_duration` (L76)

### `franka_experiments/franka_experiments/utils/state_governor.py`

Classes: `GovernorState` (L81, 0 methods), `StateGovernor` (L90, 6 methods)

Functions: `governor_from_params` (L223), `directional_fade` (L246)

### `franka_experiments/franka_experiments/utils/static_scene.py`

Classes: `_Track` (L89, 1 methods), `StaticSceneHold` (L106, 9 methods), `_Cell` (L237, 1 methods), `StaticSceneVoxels` (L250, 10 methods)

Functions: `hold_from_params` (L466), `voxels_from_params` (L484)

### `franka_experiments/franka_experiments/utils/surface_velocity.py`

Classes: `_Win` (L137, 1 methods), `SurfaceNormalSpeed` (L148, 6 methods)

Functions: `surface_speed_from_params` (L261)

### `franka_experiments/franka_experiments/utils/tf_manager.py`

Classes: `TFManager` (L31, 6 methods)

Functions: `_extract_Rt` (L218)

### `franka_experiments/franka_experiments/utils/timing_law.py`

Classes: `TimingLaw` (L26, 4 methods), `LinearTimingLaw` (L51, 3 methods), `ExponentialTimingLaw` (L91, 2 methods), `TrapezoidalTimingLaw` (L113, 2 methods)

### `franka_experiments/franka_experiments/utils/trajectory.py`

Classes: `PentagonTrajectory` (L47, 3 methods), `CircularTrajectory` (L326, 3 methods), `GeometricPath` (L435, 7 methods), `PentagonPath` (L488, 3 methods), `LissajousPath` (L521, 4 methods), `CircularPath` (L662, 4 methods), `LinearSegmentPath` (L777, 2 methods), `CompositePath` (L805, 3 methods)

Functions: `sample_single_waypoint` (L849), `sample_waypoints` (L873)

### `franka_experiments/franka_experiments/utils/trajectory_overlay.py`

Classes: `PixelBounds` (L159, 4 methods)

Functions: `project_base_to_pixels` (L51), `_distort` (L114), `trail_alphas` (L130), `fading_runs` (L202), `_flush` (L249), `draw_fading_polyline` (L260), `_clip_run` (L292), `composite` (L310)

### `franka_experiments/franka_experiments/utils/trajectory_trace.py`

Classes: `TraceBuffer` (L39, 8 methods)

Functions: `_points` (L165), `_strip` (L175), `trace_markers` (L193)

### `franka_experiments/franka_experiments/utils/visualization.py`

Classes: `VisFrame` (L22, 0 methods)

Functions: `draw_overlay` (L49), `_project` (L131), `_zone_color` (L147)

### `franka_simulation_experiments/scripts/simexp/analysis.py`


Functions: `parse_cbfdiag` (L27), `_num` (L39), `qdd_ratio` (L46), `slack_max` (L69), `find_files` (L82), `pentagon_tracking` (L96), `load_json` (L112), `read_bag_topics` (L120), `qdd_projection` (L138), `per_link_entries` (L180), `load_truth` (L204), `ros_log_lines` (L212)

### `franka_simulation_experiments/scripts/simexp/attribution.py`


Functions: `truth_files` (L36), `score` (L59), `causes` (L70), `residual_causes` (L97), `attribute_run` (L119)

### `franka_simulation_experiments/scripts/simexp/batch.py`


Functions: `cell_name` (L28), `emulated30_grid` (L38), `phase5_grid` (L45), `flags_grid` (L73), `cell_ok` (L81), `_flat` (L99), `run_one` (L151), `sim_config_manifest` (L188), `_json_default` (L193), `run_grid` (L205), `score_cell` (L247), `_rescore_one` (L259), `rescore` (L277), `write_csv` (L296)

### `franka_simulation_experiments/scripts/simexp/closed_loop.py`


Functions: `rate_verdict` (L63), `_wall` (L83), `log_window` (L88), `_cumulative_delta` (L98), `fallback_counts` (L120), `slew_conflict_counts` (L127), `cbfdiag_capage` (L139), `cbfdiag_slack` (L163), `summarize_run` (L195)

### `franka_simulation_experiments/scripts/simexp/conditions.py`


Functions: `default_path` (L48), `load` (L58), `_s` (L63), `params_arg` (L69), `resolve` (L79)

### `franka_simulation_experiments/scripts/simexp/control_regression.py`


Functions: `cells` (L14), `summarize_diagnostics` (L18), `tracking_from_stamped_points` (L40), `check_obstacle_motion` (L59), `score` (L85), `write_summary` (L121)

### `franka_simulation_experiments/scripts/simexp/emulation.py`

Classes: `TrackEmulator` (L76, 3 methods)

Functions: `_cbf_scenarios` (L37), `nearest` (L50), `frame_results` (L61), `fill_track_fields` (L97)

### `franka_simulation_experiments/scripts/simexp/estimator_metrics.py`


Functions: `fe_scripts` (L52), `_cv` (L64), `control_params` (L70), `trust_frames` (L81), `_stamp` (L90), `read_run` (L94), `_at` (L125), `make_residual` (L134), `build_series` (L143), `_frame_dt` (L207), `estimator_scores` (L212), `track_scores` (L255), `score_run` (L287)

### `franka_simulation_experiments/scripts/simexp/graph_contract.py`


Functions: `_fq` (L33), `_qos` (L37), `capture_graph` (L42), `capture_tf` (L80), `reference_dir` (L108), `load_reference` (L118), `_endpoints` (L127), `diff` (L132)

### `franka_simulation_experiments/scripts/simexp/obstacle_laws.py`


Functions: `_axis` (L18), `_phase` (L33), `_wander` (L50), `position` (L63), `velocity` (L86), `pursue` (L104)

### `franka_simulation_experiments/scripts/simexp/offline.py`


Functions: `overrides` (L36), `run` (L57)

### `franka_simulation_experiments/scripts/simexp/procs.py`

Classes: `Launch` (L34, 6 methods)

Functions: `_pids_with_env` (L19), `_group_alive` (L93), `leftovers` (L103)

### `franka_simulation_experiments/scripts/simexp/provenance.py`


Functions: `snapshot` (L11)

### `franka_simulation_experiments/scripts/simexp/reflex_checks.py`

Classes: `ReflexChecker` (L61, 2 methods), `RtfMeter` (L120, 3 methods)

Functions: `robot_limits` (L46), `envelope` (L55)

### `franka_simulation_experiments/scripts/simexp/report.py`


Functions: `_val` (L29), `load_rows` (L40), `select` (L49), `_nums` (L53), `med` (L58), `count` (L68), `total` (L73), `causes` (L78), `table` (L92), `offline_column` (L108), `t_quality` (L129), `t_estimator` (L152), `t_tracks` (L170), `t_closed` (L185), `t_three` (L203), `t_pentagon` (L235), `t_throws` (L249), `t_flags` (L262), `build` (L283), `default_report_path` (L342), `write` (L351)

### `franka_simulation_experiments/scripts/simexp/runner.py`


Functions: `experiments_src` (L24), `run_cell` (L33), `_run_script` (L75), `digest_cell` (L82), `_verdicts` (L119)

### `franka_simulation_experiments/scripts/simexp/scenario_catalog.py`


Functions: `default_path` (L32), `load` (L42), `_fmt_v` (L47), `throw_names` (L51), `all_names` (L63), `_throw` (L69), `resolve` (L104), `_spawn` (L163), `check_start` (L180), `enforce_start_clearance` (L190)

### `franka_simulation_experiments/scripts/simexp/scenario_laws.py`

Classes: `FlangeKinematics` (L44, 2 methods), `NominalLaw` (L60, 3 methods)

Functions: `joint_pd` (L27), `cartesian` (L32)

### `franka_simulation_experiments/scripts/simexp/sim_config.py`


Functions: `share` (L29), `franka_simulation_utils` (L34), `default_paths` (L45), `scene_paths` (L84), `sim_runs_root` (L101), `make_run_dir` (L120), `rt_controllers_yaml` (L127), `world_sdf` (L143), `deep_merge` (L162), `perception_yaml` (L171), `git_provenance` (L186), `depth_adapter_yaml` (L207)

### `franka_simulation_experiments/scripts/simexp/truth_geometry.py`

Classes: `TruthGeometry` (L35, 4 methods)

Functions: `load_perception_config` (L26), `cp_label` (L62), `surface_gaps` (L67), `closest_point_visible` (L77)

### `franka_experiments/scripts/analyze_isolation_ab.py`


Functions: `load_csv` (L74), `load_pentagon_csv` (L97), `pentagon_tracking_stats` (L123), `segment` (L152), `per_joint_stats` (L167), `_print_single` (L212), `main` (L236)

### `franka_experiments/scripts/approach_envelope.py`


Functions: `_fifo_run_scenario` (L22), `cell` (L60), `main` (L80)

### `franka_experiments/scripts/approach_probe.py`


Functions: `_pct` (L34), `_stamps` (L39), `cmd_rates` (L59), `_ball_at` (L98), `_passes_of` (L109), `cmd_depth` (L121), `cmd_assoc` (L301), `cmd_causes` (L358), `main` (L394)

### `franka_experiments/scripts/bag_to_mp4.py`


Functions: `image_msg_to_cv2` (L16), `main` (L51)

### `franka_experiments/scripts/ball_closed_loop.py`

Classes: `Recording` (L62, 3 methods), `CircleRef` (L112, 4 methods), `Commander` (L162, 4 methods), `_Sink` (L227, 2 methods), `Clock` (L289, 0 methods)

Functions: `make_filter` (L238), `_use_source_share` (L278), `run_pass` (L293), `_oracle` (L458), `_threat_shift` (L510), `_fmt` (L528), `_ros_time` (L534), `_ball_at` (L542), `_reproject` (L553), `_metrics` (L610), `find_passes` (L664), `find_events` (L674), `check_nominal` (L694), `main` (L711)

### `franka_experiments/scripts/ball_throw_eval.py`


Functions: `read_bag` (L73), `_stamp` (L100), `ball_mask` (L108), `cmd_truth` (L113), `_trust_frames` (L198), `passes` (L209), `truth_velocity` (L241), `cmd_score` (L255), `main` (L376)

### `franka_experiments/scripts/cbf_phase4_check.py`

Classes: `Phase4Check` (L53, 4 methods)

Functions: `main` (L258)

### `franka_experiments/scripts/cbf_scenarios.py`

Classes: `_Log` (L69, 7 methods), `Sphere` (L122, 2 methods), `Filter` (L146, 3 methods)

Functions: `shipped_params` (L83), `camera_velocity_noise` (L104), `run_scenario` (L276), `_oscillation` (L422), `summarize` (L448), `print_report` (L477), `scenarios` (L498), `main` (L560), `bag_replay` (L605)

### `franka_experiments/scripts/check_control_derivatives.py`


Functions: `main` (L20)

### `franka_experiments/scripts/compare_range_noise.py`


Functions: `_noisy_single_pixel_depth` (L103), `synthetic_noise_sweep` (L136), `cmd_synthetic` (L178), `_sphere_ray_z` (L232), `cmd_bag` (L250), `_add_disparity_noise` (L404), `main` (L421)

### `franka_experiments/scripts/compare_vobs.py`

Classes: `_Log` (L64, 7 methods), `JointTrace` (L146, 4 methods), `_PShim` (L200, 1 methods)

Functions: `read_bag` (L81), `build_tf_buffer` (L113), `make_residual_estimator` (L208), `cross_correlation_lag` (L227), `static_windows` (L266), `rms` (L286), `replay` (L294), `report` (L538), `_plot` (L649), `main` (L680)

### `franka_experiments/scripts/experiment_summary.py`


Functions: `load` (L31), `col` (L48), `fin` (L58), `peak` (L62), `vec_peak` (L74), `covered` (L88), `sat_count` (L92), `fmt` (L118), `line` (L122), `main` (L131)

### `franka_experiments/scripts/gen_zero_gain_controllers_yaml.py`


Functions: `main` (L45)

### `franka_experiments/scripts/iso_constants_measure.py`


Functions: `_load` (L73), `_store` (L80), `_pct` (L94), `cmd_reaction` (L103), `cmd_stop` (L167), `c_from_detection_capability` (L343), `cmd_detection` (L360), `cmd_uncertainty` (L426), `cmd_yaml` (L550), `main` (L586)

### `franka_experiments/scripts/iso_evidence_report.py`

Classes: `Result` (L56, 2 methods)

Functions: `load` (L70), `col` (L89), `finite` (L101), `coverage` (L105), `worst` (L109), `exceed_stats` (L121), `_period` (L131), `ev_episodes` (L138), `_d1_active` (L157), `check_workspace_feasibility` (L174), `check_integrity` (L211), `check_tcp_speed` (L248), `check_reduced_speed` (L273), `check_separation` (L295), `check_speed_cap` (L344), `check_floor` (L364), `check_joint_speed` (L399), `check_stops` (L421), `check_braking_authority` (L459), `check_saturation` (L482), `check_chain_faults` (L504), `check_two_channels` (L529), `main` (L582)

### `franka_experiments/scripts/iso_pfl_speed.py`


Functions: `main` (L76)

### `franka_experiments/scripts/iso_preflight_check.py`

Classes: `Check` (L61, 4 methods)

Functions: `check_config` (L89), `_detection_record` (L177), `check_topic` (L187), `main` (L223)

### `franka_experiments/scripts/latency_budget.py`

Classes: `_Log` (L76, 7 methods)

Functions: `stats` (L59), `line` (L68), `_hdr_stamp` (L95), `cmd_bag` (L100), `cmd_cam` (L141), `cmd_perception` (L212), `_cbf_params` (L362), `cmd_cbf` (L373), `cmd_dds` (L569), `main` (L652)

### `franka_experiments/scripts/pipeline_rate_monitor.py`

Classes: `RateProbe` (L116, 3 methods), `PipelineRateMonitor` (L154, 2 methods), `CpuSampler` (L216, 2 methods)

Functions: `_short_cmd` (L251), `_fit` (L263), `render` (L272), `write_plot` (L312), `main` (L339)

### `franka_experiments/scripts/plot_rl_timing.py`


Functions: `_default_csv` (L28), `load` (L38), `stats` (L52), `main` (L58)

### `franka_experiments/scripts/range_noise_calibration.py`


Functions: `fit_sigma_d` (L68), `cmd_fit` (L115), `cmd_capture` (L174), `main` (L253)

### `franka_experiments/scripts/smoothness_report.py`


Functions: `load` (L33), `pct` (L52), `verdict` (L56), `report` (L64)

### `franka_experiments/scripts/throw_clearance.py`


Functions: `fk` (L27)

### `franka_experiments/scripts/throw_lead.py`


Functions: `main` (L18)

### `franka_experiments/scripts/watch_obstacle_tracks.py`

Classes: `WatchObstacleTracks` (L47, 4 methods)

Functions: `main` (L133)

### `franka_simulation_experiments/scripts/emulated_perception.py`

Classes: `EmulatedPerception` (L39, 9 methods)

Functions: `main` (L135)

### `franka_simulation_experiments/scripts/episode_supervisor.py`

Classes: `EpisodeSupervisor` (L55, 7 methods)

Functions: `hardware_log_dir` (L46), `main` (L207)

### `franka_simulation_experiments/scripts/ground_truth_node.py`

Classes: `GroundTruth` (L36, 7 methods)

Functions: `main` (L126)

### `franka_simulation_experiments/scripts/gz_trajectory_markers.py`

Classes: `GzTrajectoryMarkers` (L151, 7 methods)

Functions: `marker_request` (L96), `delete_request` (L125), `trace_signature` (L137), `main` (L242)

### `franka_simulation_experiments/scripts/pin_process.py`


Functions: `_environ` (L22), `_cmdline` (L30), `find` (L38), `main` (L44)

### `franka_simulation_experiments/scripts/plot_control_regression.py`


Functions: `main` (L10)

### `franka_simulation_experiments/scripts/reflex_monitor.py`

Classes: `ReflexMonitor` (L39, 6 methods)

Functions: `main` (L139)

### `franka_simulation_experiments/scripts/run_gz_scenarios.py`


Functions: `main` (L30)

### `franka_simulation_experiments/scripts/scenario_commander.py`

Classes: `ScenarioCommander` (L31, 4 methods)

Functions: `main` (L81)
