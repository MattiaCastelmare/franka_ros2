# franka_experiments

CBF safety filter and experiment nodes for the Franka FR3 manipulator (ROS2).

---

## Overview

`franka_experiments` is a Python ROS2 package (`ament_python`) that implements
two end-to-end control stacks for the Franka FR3 arm, both centred on
**Control Barrier Function (CBF)** safety filters that formally guarantee
collision avoidance at run time.

The **torque stack** operates in acceleration space.  A Cartesian motion
generator (`pentagon_qddot_commander`) produces a nominal joint-acceleration
command q̈_nom.  A HOCBF QP filter (`cbf_safety_filter`) projects it onto the
safe set and outputs q̈_safe.  A dynamics converter (`qddot_to_torque`) applies
τ = M(q)·q̈ + C(q,q̇)·q̇ via Pinocchio and forwards the torque to
`rt_torque_controller` (gravity is added by the firmware).  An alternative
torque path uses `pentagon_torque_commander` (6D Cartesian PD + damped-LS
Jacobian) and `cbf_oscbf_filter`, which implements the Operational Space CBF
from Morton & Pavone (arXiv:2503.06736, 2025) — a torque-level QP with
separate null-space and task-space cost terms.

The **velocity stack** operates at the kinematic level.  A velocity commander
(`ee_pentagon_velocity_commander`) outputs a nominal joint-velocity q̇_nom.  A
velocity-level CBF QP filter (`cbf_velocity_filter`) solves for the nearest
safe q̇ and forwards it to `rt_velocity_executor_controller`.  This stack has a
**two-phase** design: Phase 1 bypasses the QP (useful for trajectory
verification without a camera), and Phase 2 enables the full CBF obstacle
avoidance pipeline.

Shared infrastructure includes `real_time_distance` — a depth-camera
human-robot distance estimator (Flacco depth-space method) that publishes
per-link distances on `/cbf/per_link_distances` (type `MultiLinkDistance`).
Additional nodes cover experiment CSV/plot logging (`experiment_logger`), RViz
capsule visualization (`capsule_overlay_node`), and hand-eye calibration
(`handeye_calibration_node`).

---

## Package layout

```
franka_experiments/
├── config/
│   ├── fr3_control.yaml          # controller + CBF tuning (torque stack)
│   ├── fr3_complete.yaml         # robot geometry + distance estimation
│   ├── fr3_distance.yaml         # per-link distance thresholds
│   ├── launch_defaults.yaml      # shared launch argument defaults
│   ├── oscbf_params.yaml         # OSCBF QP weights and tuning
│   └── ...                       # camera intrinsics/extrinsics, RViz config
├── franka_experiments/
│   ├── nodes/                    # all ROS2 node entry-points
│   └── utils/                    # shared kinematics, logging, ROS helpers
├── launch/
│   ├── torque_control_stack.launch.py       # accel-space pipeline
│   ├── velocity_cbf_control_stack.launch.py # velocity pipeline
│   ├── thales.launch.py                     # production velocity pipeline + rosbag
│   ├── handeye_calibration_bringup.launch.py
│   └── minimal.launch.py
├── test/
└── setup.py
```

---

## Node reference

| Node executable | Source file | Pipeline | Subscribes | Publishes | Description |
|---|---|---|---|---|---|
| `pentagon_qddot_commander` | `nodes/pentagon_qddot_commander.py` | Torque (accel) | `/NS_1/joint_states`, move_group services (`compute_fk`, `compute_cartesian_path`) | `/NS_1/qddot_nom` (Float64MultiArray, 7), `/NS_1/q_des_state` (JointState), `~/planned_trajectory` (JointTrajectory, debug) | MoveIt-based pentagon reference generator: Cartesian path planned by move_group (group `fr3_arm`, `fr3_link0` → `fr3_hand_tcp`), sampled at `rate_hz`; q̈ output = feedforward + Cartesian-space tracking correction `J†(q)·(kp_cart·clip(x_d−x,±cart_err_clamp) + kd_cart·(ẋ_d−ẋ))` (applied before the CBF filter). Pinocchio (FR3 + Franka hand URDF) provides FK and the 3×7 linear Jacobian at `fr3_hand_tcp`; Cartesian references are pre-computed during _ingest (offline). Requires a running `move_group` (`ros2 launch franka_fr3_moveit_config move_group.launch.py`); publishes zeros until it is available |
| `rl_policy_commander` | `nodes/rl_policy_commander.py` | Torque (accel) | `/NS_1/joint_states`, `/cbf/per_link_distances`, `/NS_1/cbf_status` (diagnostics), optional target `PointStamped` | `/NS_1/qddot_nom` (Float64MultiArray, 7), `/NS_1/rl_status` (Float64MultiArray, 6) | Sim-to-real deployment of the Safe-RL policy trained in `franka_sim/` against this same CBF filter. Rebuilds the 24-dim training observation `[q, q̇, ee, target, obstacle, d_min]` (Pinocchio FK at `ee_frame`, obstacle slot reconstructed from the point-cloud surface distance — see `utils/rl_policy.py`), runs the exported ONNX actor with `onnxruntime` (single-threaded, ~0.1 ms) and publishes `q̈_nom = a·q̈_max·action_scale`. Alternative to `pentagon_qddot_commander` on the same topic — select with `motion_source:=rl`, never both. Publishes zeros during warm-up, on stale joint state and on a perception fault |
| `cbf_safety_filter` | `nodes/cbf_safety_filter.py` | Torque (accel) | `/NS_1/qddot_nom`, `/cbf/per_link_distances`, `/NS_1/joint_states` | `/NS_1/qddot_safe` (Float64MultiArray, 7) | HOCBF QP filter: min ‖q̈ − q̈_nom‖² s.t. CBF constraints |
| `qddot_to_torque` | `nodes/qddot_to_torque.py` | Torque (accel) | `/NS_1/qddot_safe` (remapped from `qddot_nom`), `/NS_1/joint_states` | `/NS_1/torque_cmd` (Float64MultiArray, 7) | Dynamics converter: τ = M(q)·q̈ + C(q,q̇)·q̇ via Pinocchio |
| `pentagon_torque_commander` | `nodes/pentagon_torque_commander.py` | Torque (OSCBF) | `/NS_1/joint_states` | `/NS_1/torque_cmd` (Float64MultiArray, 7) | 6D Cartesian PD + damped-LS Jacobian torque commander |
| `cbf_oscbf_filter` | `nodes/cbf_OSCBF_filter.py` | Torque (OSCBF) | `/NS_1/torque_cmd`, `/NS_1/joint_states`, `/cbf/per_link_distances` | `/NS_1/torque_safe` (Float64MultiArray, 7) | Operational Space CBF (Morton & Pavone 2025): torque-level QP with null-space + task-space cost |
| `ee_pentagon_velocity_commander` | `nodes/ee_pentagon_velocity_commander.py` | Velocity | `/NS_1/joint_states` | `/NS_1/tracking_qdot` (Float64MultiArray, 7) | Pentagon EE trajectory in velocity space |
| `cbf_velocity_filter` | `nodes/cbf_velocity_filter.py` | Velocity | `/NS_1/tracking_qdot`, `/human_robot/multi_distance`, `/NS_1/joint_states` | `/NS_1/qdot_cmd` (Float64MultiArray, 7) | Velocity-level CBF QP; param `bypass_cbf` for Phase 1 pass-through |
| `real_time_distance` | `nodes/real_time_distance.py` | Shared | Depth image, `/NS_1/joint_states`, TF tree | `/cbf/per_link_distances` (MultiLinkDistance), `/human_robot/multi_distance` | Flacco depth-space human-robot distance estimator; multithreaded compute/visualize |
| `experiment_logger` | `nodes/experiment_logger.py` | Shared | `/NS_1/joint_states`, `/NS_1/torque_cmd`, CBF topics | — | CSV + plot logger for joint states, torques, CBF values |
| `capsule_overlay_node` | `nodes/capsule_overlay_node.py` | Shared | TF tree, robot model | Marker array (RViz) | Publishes capsule geometry for robot-body visualisation in RViz |
| `handeye_calibration_node` | `nodes/handeye_calibration_node.py` | Shared | TF tree, camera images | — | Interactive hand-eye calibration tool |

---

## Launch files

### 5.1 Torque control stack (`torque_control_stack.launch.py`)

**Pipeline:**

```
[Camera]  RealSense driver
    │
    ▼
real_time_distance  ──►  /cbf/per_link_distances
                                  │
pentagon_qddot_commander          │
  (or rl_policy_commander,        │
   motion_source:=rl)             │
    │                             │
    ▼                             ▼
/NS_1/qddot_nom  ──►  cbf_safety_filter  ──►  /NS_1/qddot_safe
                                                      │
                                               qddot_to_torque
                                                      │
                                                      ▼
                                              /NS_1/torque_cmd
                                                      │
                                            rt_torque_controller  ──►  HW
```

**Startup sequence:**

| Time | Action |
|---|---|
| t = 0 s | franka bringup (robot driver + joint_state_broadcaster) |
| t = 1 s | world → fr3_link0 static TF (identity) |
| t = `camera_delay_s` | RealSense driver (if `enable_camera:=true`) |
| t = 1 s | camera extrinsics static TF (if `enable_camera:=true`) |
| t = 2 s | `cbf_safety_filter` + `qddot_to_torque` (pre-init before RT loop) |
| t = 2 s | `real_time_distance` (if `start_real_time_distance:=true`) |
| t = `control_spawner_delay_s` | `rt_torque_controller` spawner (1 kHz RT loop starts) |
| t = `control_spawner_delay_s` + 2 s | `pentagon_qddot_commander`, or `rl_policy_commander` when `motion_source:=rl` |

> **Note** — `motion_source` selects the single q̈_nom generator: `pentagon`
> (default, analytic path + avoidance-first shaping) or `rl` (the ONNX Safe-RL
> policy from `franka_sim/`, see `../franka_sim_to_real_roadmap.md`).  Both
> publish `/NS_1/qddot_nom`, so exactly one runs at a time; everything
> downstream (CBF filter → torque → controller) is identical:
>
> ```bash
> ros2 launch franka_experiments torque_control_stack.launch.py \
>     motion_source:=rl rl_onnx_model:=/path/to/best_model.onnx
> ```
>
> `move_group` is only needed by `pentagon_qddot_commander`; pass
> `start_move_group:=false` with `motion_source:=rl`.

> **Note** — `pentagon_qddot_commander` now generates its trajectory via
> MoveIt.  The stack starts `move_group` automatically (in the robot
> namespace, so it sees `/NS_1/tf` and `/NS_1/joint_states`); disable with
> `start_move_group:=false`.  Until move_group and a joint state are
> available the commander publishes zero accelerations (the robot holds).

**Invocations:**

```bash
# Full stack — camera + CBF + distance estimation (default):
ros2 launch franka_experiments torque_control_stack.launch.py robot_ip:=192.168.2.10

# Without camera / distance (trajectory only):
ros2 launch franka_experiments torque_control_stack.launch.py \
    enable_camera:=false start_real_time_distance:=false

# Fake hardware (no robot required):
ros2 launch franka_experiments torque_control_stack.launch.py use_fake_hardware:=true

# With namespace:
ros2 launch franka_experiments torque_control_stack.launch.py \
    namespace:=NS_1 robot_ip:=192.168.2.10
```

**Key launch arguments:**

| Argument | Default | Description |
|---|---|---|
| `namespace` | `""` | ROS2 namespace for all topics |
| `robot_ip` | `192.168.1.10` | Robot IP address |
| `use_fake_hardware` | `false` | Simulate without physical robot |
| `control_spawner_delay_s` | `10.0` | Seconds before spawning `rt_torque_controller` |
| `enable_camera` | `true` | Start RealSense driver |
| `start_real_time_distance` | `true` | Start distance estimator |
| `start_experiment_logger` | `true` | Start CSV logger |
| `lpf_alpha` | `0.3` | Low-pass filter coefficient in `rt_torque_controller` |
| `tau_max_scale` | `1.0` | Torque saturation scale factor |

---

### 5.2 Velocity CBF stack (`velocity_cbf_control_stack.launch.py`)

**Pipeline (Phase 2):**

```
[Camera]  RealSense driver
    │
    ▼
real_time_distance  ──►  /human_robot/multi_distance
                                  │
ee_pentagon_velocity_commander    │
    │                             │
    ▼                             ▼
/NS_1/tracking_qdot  ──►  cbf_velocity_filter  ──►  /NS_1/qdot_cmd
                                                            │
                                               rt_velocity_executor_controller  ──►  HW
```

**Two-phase operation:**

| Phase | `bypass_cbf` | Behaviour |
|---|---|---|
| **Phase 1** (default) | `true` | `cbf_velocity_filter` passes `tracking_qdot` → `qdot_cmd` unmodified. Camera and distance estimator are **not** started. Use to verify trajectory first. |
| **Phase 2** | `false` | Full CBF QP active. Camera + `real_time_distance` are started automatically. |

**Startup sequence (Phase 2, `control_spawner_delay_s` = D):**

| Time | Action |
|---|---|
| t = 0 s | franka bringup |
| t = 1 s | world → fr3_link0 static TF |
| t = 0 s [Phase 2] | RealSense driver |
| t = 1 s [Phase 2] | camera extrinsics static TF |
| t = 3 s [Phase 2] | image republisher |
| t = D | `rt_velocity_executor_controller` spawner |
| t = D + 4 s | `cbf_velocity_filter` |
| t = D + 6 s [Phase 2] | `real_time_distance` |
| t = D + 8 s | `ee_pentagon_velocity_commander` |

**Invocations:**

```bash
# Phase 1 — bypass CBF, trajectory only (default):
ros2 launch franka_experiments velocity_cbf_control_stack.launch.py bypass_cbf:=true

# Phase 2 — full CBF with obstacle avoidance:
ros2 launch franka_experiments velocity_cbf_control_stack.launch.py bypass_cbf:=false

# Fake hardware:
ros2 launch franka_experiments velocity_cbf_control_stack.launch.py use_fake_hardware:=true

# With namespace:
ros2 launch franka_experiments velocity_cbf_control_stack.launch.py \
    namespace:=NS_1 robot_ip:=192.168.2.10 bypass_cbf:=false
```

---

### 5.3 Production velocity stack with rosbag (`thales.launch.py`)

`thales.launch.py` is the production launch file for the velocity pipeline.
It extends `velocity_cbf_control_stack` with optional rosbag recording
(`start_rosbag:=true`) that co-locates CSV logs and bag files under a shared
timestamped directory `~/ros2_experiments/<timestamp>_<name>/`.
The OSCBF torque pipeline (`pentagon_torque_commander` → `cbf_oscbf_filter`,
Morton & Pavone 2025 — torque-level QP with operational-space task and
null-space cost terms → `rt_torque_controller`) is configured via
`oscbf_params.yaml`; see that file for per-weight tuning.

```bash
# Fake hardware (no robot required):
ros2 launch franka_experiments thales.launch.py use_fake_hardware:=true

# Real robot with rosbag recording:
ros2 launch franka_experiments thales.launch.py start_rosbag:=true

# With camera and real-time distance:
ros2 launch franka_experiments thales.launch.py \
    enable_camera:=true start_real_time_distance:=true
```

---

## Configuration files

| File | Purpose | Key parameters |
|---|---|---|
| `fr3_control.yaml` | CBF tuning for the torque and velocity stacks | `d_safe` (min obstacle distance, m), `k0_cbf` / `k1_cbf` (HOCBF class-K gains), `gamma` (velocity-CBF class-K), `rho_slack` (QP slack penalty), `distance_ema_alpha`, `max_qddot_delta`, `max_tau_delta`, `k_brake` |
| `oscbf_params.yaml` | OSCBF QP weights and CBF gains | `w_j` / `w_o` (null-space / task-space cost weights), `alpha1` / `alpha2` (HOCBF decay rates), `tau_max[7]`, `joint_limit_margin`, `enable_vel_cbf`, `enable_ws_cbf`, `enable_obstacle_cbf` |
| `fr3_complete.yaml` | Robot geometry for `real_time_distance` | Control points per link, mesh paths, distance thresholds, TF frame names |
| `fr3_distance.yaml` | Per-link distance thresholds | Link-specific `d_safe` overrides used by the velocity CBF filter |
| `launch_defaults.yaml` | Shared defaults for all launch files | `robot_ip`, `namespace`, `use_fake_hardware`, `lpf_alpha`, `tau_max_scale`, `control_spawner_delay_s`, `qdot_max`, `enable_camera`, `start_real_time_distance` |
| `camera_extrinsics.yaml` | Camera-to-robot extrinsic calibration | `parent_frame`, `child_frame`, `translation`, `rotation` (quaternion) |
| `camera_intrinsics.yaml` / `depth_intrinsics.yaml` | Sensor intrinsics | Focal lengths, principal point, distortion coefficients |

---

## Dependencies

### ROS2 packages

- `franka_msgs` — custom message types including `MultiLinkDistance`, `HumanRobotDistance`
- `franka_description` — URDF and mesh assets for the FR3
- `franka_bringup` — robot driver launch infrastructure
- `franka_simulation` — image republisher for depth-camera pipeline
- `ros2_control`, `controller_manager` — hardware abstraction and controller lifecycle
- `tf2_ros` — TF tree for kinematics and camera extrinsics
- `cv_bridge` — ROS ↔ OpenCV image conversion
- `visualization_msgs` — RViz marker publishing
- `realsense2_camera` — Intel RealSense depth driver

The package also depends on `franka_rt_controllers` (C++ package, not bundled
here) which provides the two real-time controllers:
`rt_torque_controller` (1 kHz joint-torque loop with gravity compensation and
LPF) and `rt_velocity_executor_controller` (joint-velocity loop with
interpolation and timeout ramp).

### Python packages (install via pip)

```bash
pip install pinocchio "qpsolvers[osqp]" numpy scipy opencv-python pyyaml trimesh matplotlib
```

---

## Build & install

```bash
cd ~/Git/franka_ros2
colcon build --symlink-install --packages-select franka_experiments
source install/setup.bash

# Verify entry-points are registered:
ros2 run franka_experiments --help 2>&1 | head
```

---

## Debug commands

```bash
# Verify controllers are active:
ros2 control list_controllers -c /NS_1/controller_manager

# List hardware interfaces:
ros2 control list_hardware_interfaces -c /NS_1/controller_manager

# Monitor CBF filter output (accel-space):
ros2 topic echo /NS_1/qddot_safe
ros2 topic echo /NS_1/torque_cmd

# Monitor safe velocity command:
ros2 topic echo /NS_1/qdot_cmd

# Monitor per-link distances:
ros2 topic echo /cbf/per_link_distances

# Check topic frequencies:
ros2 topic hz /NS_1/qddot_safe
ros2 topic hz /cbf/per_link_distances

# Watch joint states:
ros2 topic echo /NS_1/joint_states

# Live CBF barrier values (if experiment_logger is running):
tail -f ~/ros2_experiments/*/cbf_log.csv
```

---

## References

- Ferraguti et al., "A Control Barrier Function Approach for Maximizing
  Performance While Fulfilling to ISO/TS 15066 Regulations", *RA-L* 2020.
- Ferraguti et al., "Safety and Efficiency in Robotics: The Control Barrier
  Functions Approach", *RAM* 2022.
- Morton & Pavone, "Safe, Task-Consistent Manipulation with Operational Space
  Control Barrier Functions", *arXiv:2503.06736*, 2025.
- Flacco et al., "A depth space approach to human-robot collision avoidance",
  *ICRA* 2012.
