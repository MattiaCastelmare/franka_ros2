# ARCHITECTURE.md — `franka_experiments`

Map of the **active** pipeline: everything reachable from
`launch/torque_control_stack.launch.py`. Anything not on this map is either a
support module or flagged in [LEGACY.md](LEGACY.md).

State: post-refactor (Phases 1–5). Verified statically only — see
*Verification* at the end.

---

## 1. Launch graph

Namespace defaults to `NS_1` (from `franka_bringup/config/franka.config.yaml`,
which overrides `config/launch_defaults.yaml`'s empty default on merge).

```
torque_control_stack.launch.py
│  defaults ← config/launch_defaults.yaml         (utils.config.load_launch_defaults)
│           ← franka_bringup/config/franka.config.yaml  (…load_franka_config_defaults) [WINS]
│  controllers YAML generated at composition time (utils.launch_support)
│
├─ t=0                     INCLUDE franka_bringup/launch/franka.launch.py
│                            ├── controller_manager (+ generated controllers.yaml)
│                            ├── joint_state_broadcaster
│                            ├── franka_robot_state_broadcaster   [real HW only]
│                            └── robot_state_publisher
├─ torque_world_tf_delay_s  tf2_ros/static_transform_publisher  world → base
├─ (start_move_group)       INCLUDE franka_fr3_moveit_config/launch/move_group.launch.py
├─ torque_finger_pub_delay_s  ExecuteProcess  finger_state_publisher
│                            (rate = torque_finger_pub_rate_hz)
├─ camera_delay_s           INCLUDE realsense2_camera/launch/rs_launch.py   [enable_camera]
├─ torque_camera_tf_delay_s tf2_ros/static_transform_publisher  base → camera_link
│                            ← config/camera_link_extrinsics.yaml (read by the launch file)
├─ camera_delay_s + torque_image_republisher_extra_delay_s
│                           franka_simulation/image_publisher
├─ torque_rtd_delay_s       franka_experiments/real_time_distance   [start_real_time_distance]
│                            params: robot_config_path      ← robot_config_yaml arg
│                                    camera_extrinsics_path ← camera_extrinsics_yaml arg
├─ torque_dynamics_delay_s  franka_experiments/cbf_safety_filter
│                            (self-loads config/fr3_control.yaml)
├─ torque_dynamics_delay_s  franka_experiments/qddot_to_torque
│                            (self-loads config/fr3_control.yaml)
├─ control_spawner_delay_s  controller_manager/spawner → rt_torque_controller
│                            (timeout = controller_spawner_timeout_s)
├─ control_spawner_delay_s + torque_commander_extra_delay_s
│                           franka_experiments/pentagon_qddot_commander  [ns=NS_1]
└─ experiment_logger_delay_s  franka_experiments/experiment_logger  [start_experiment_logger]
```

## 2. Runtime data flow

```
 depth ──► real_time_distance ──► /cbf/per_link_distances ──► cbf_safety_filter
   TF ──►      (~30 Hz)              (BEST_EFFORT, depth 1)          │
                                                                     │
 pentagon_qddot_commander ──► /NS_1/qddot_nom ───────────────────────┤
        (100 Hz)                                                     ▼
                                                            /NS_1/qddot_safe
                                                              │           │
                                          ┌───────────────────┘           └──► rt_torque_controller
                                          ▼                                       accel_sub_ (Kd term)
                                   qddot_to_torque                                        ▲
                                          │                                               │
                                          └──► /NS_1/torque_cmd ─────────────────────────┘
                                                                                    command_sub_ (tau_ff)
                                                                                          │
                                                                                    Franka FCI (adds g(q))

 cbf_safety_filter ──► /NS_1/cbf_status   (no subscriber in the active graph)
 experiment_logger  ◄── joint_states, multi_distance, tracking_qdot, qdot_cmd, torque_cmd
```

| Node | Subscribes | Publishes | QoS notes |
|---|---|---|---|
| `pentagon_qddot_commander` | `/NS_1/joint_states` ×2 | `/NS_1/qddot_nom`, `/NS_1/q_des_state` | default RELIABLE, depth 10 |
| `cbf_safety_filter` | `/NS_1/joint_states`, `/NS_1/qddot_nom` (depth 1), `/cbf/per_link_distances` (depth 1, BEST_EFFORT) | `/NS_1/qddot_safe`, `/NS_1/cbf_status` | inputs depth 1 by design |
| `qddot_to_torque` | `/NS_1/joint_states`, **`/NS_1/qddot_safe`** | `/NS_1/torque_cmd` | depth 10 |
| `real_time_distance` | depth image, camera_info | `multi_distance`, `distance`, `/cbf/per_link_distances` (BEST_EFFORT) | consumes TF `fr3_link0→fr3_link{0..8}` |
| `rt_torque_controller` (C++) | `torque_cmd`, `/NS_1/qddot_safe` | joint effort interfaces | `SensorDataQoS` = BEST_EFFORT |

No QoS incompatibility: every RELIABLE publisher is read by RELIABLE or
BEST_EFFORT subscribers; the one BEST_EFFORT publisher has only BEST_EFFORT
subscribers. All profiles VOLATILE.

## 3. Parameter map

| Node | Parameter source | Main groups of knobs |
|---|---|---|
| `cbf_safety_filter` | `config/fr3_control.yaml` `params:` (self-loaded; every key declared as a validated ROS parameter) | rates (`qp_rate_hz`, `cbf_update_rate_hz`); barrier (`d_safe`, `k0_cbf`, `k1_cbf`, `cbf_obstacle_horizon`, `cbf_min_leverage`); QP (`rho_slack`, `qp_solver`, `osqp_max_iter`); staleness (`distance_timeout`, `nom_timeout`, `joint_state_timeout`, `k_brake`); state box (`velocity_box_margin`); diagnostics (`diag_*`, `tick_gap_warn_factor`) |
| `pentagon_qddot_commander` | own `declare_parameter` defaults; path geometry from `fr3_control.yaml` `params: path_*` | path (`path_center_xyz`, `path_type`, `path_radius`, `plane`, `cycle_time`); Cartesian PD (`kp_cart`, `kd_cart`, `kp_rot`, `kd_rot`); reference integrator (`k_sync_*`, `q_des_max_error`, `dq_des_max`); anti-windup (`soft_reset_thr`, `hard_reset_thr`, `soft_reset_alpha`); DLS (`lambda_sq_*`, `manip_thr`); isolation test (`isolation_*`) |
| `qddot_to_torque` | `fr3_control.yaml` `topics:` | `torque_out_topic` |
| `real_time_distance` | `config/fr3_complete.yaml` (path passed as a ROS parameter) | `distance:` (thresholds, `lpf_*`, ROI, `publish_empty_per_link`); `mask:`; `zones:`; `booleans:`; `robot:`/`meshes:` |
| `experiment_logger` | own `declare_parameter` defaults; `d_safe` seeded from `fr3_control.yaml` | output paths, `sample_rate_hz`, topic names, plotting |
| launch file | `config/launch_defaults.yaml` (flat, 1:1 with launch arguments) | robot/hardware, camera, RT controller, `torque_*` startup sequencing |

**Convention:** `snake_case` throughout; plain grouped dicts, *not*
`node_name: ros__parameters:` (only the generated controllers YAML uses that,
as ros2_control requires); subsystem prefixes on flat keys (`k*_cbf`, `hard_*`,
`lpf_*`, `path_*`, `torque_*`); units and rationale in inline comments.

## 4. Utils map

| Module | Owns |
|---|---|
| `config.py` | Reading configuration off disk (package configs, launch defaults, extrinsics, CameraInfo) |
| `params.py` | Declaring + range-validating ROS parameters; fails loudly at construction |
| `node_runtime.py` | Node lifecycle, topic-name resolution, ROS time/message marshalling |
| `launch_support.py` | Launch-description composition: argument sets, controllers-YAML generation |
| `perception_msgs.py` | Building/interpreting the distance messages |
| `cbf_qp_assembly.py` | Turning CBF rows into the sparse matrices OSQP expects |
| `cbf_hard_limits.py` | Per-tick acceleration box from joint state limits |
| `kinematics.py` | The FR3 robot model: URDF generation, Pinocchio model/data, frames, point Jacobians |
| `distance_engine.py` | Depth-space per-control-point distances + the conservative LPF |
| `distance_utils.py` | Control-point geometry, ROI, TF-derived segments |
| `trajectory.py` / `timing_law.py` | Path geometry / phase timing |
| `mask_builder.py`, `tf_manager.py`, `visualization.py` | Perception support |
| `logging_utils.py` | Throttled logging, formatting, `PerfTimer` |
| `math_utils.py`, `constants.py` | Frame-independent math; shared constants |

Compatibility facades kept so no import breaks: `ros.py`, `node_utils.py`,
`camera_yaml.py`, `cbf_kinematics.py`. All four are flagged in
[LEGACY.md](LEGACY.md).

## 5. Legacy

See **[LEGACY.md](LEGACY.md)** — 38 `TODO[LEGACY]` markers, generated from the
markers actually present in the tree. Nothing has been deleted.
