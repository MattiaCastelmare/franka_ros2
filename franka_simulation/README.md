# franka_simulation

ROS 2 Humble package for simulating the Franka FR3 7-DOF robot arm in Gazebo Ignition.
Provides four independent control pipelines (velocity, acceleration, torque, CBF/avoidance),
a numpy-only kinematics library, trajectory nodes, test infrastructure, and a MoveIt-based
motion server with capsule-model obstacle avoidance.

**Build type**: `ament_cmake`  
**Simulator**: Gazebo Ignition (`ros_gz_sim`, `ros_gz_bridge`)  
**Robot model**: `franka_description` FR3 URDF/xacro  
**Python scripts**: installed via `install(PROGRAMS … RENAME)` — no `.py` extension at runtime

---

## Table of contents

1. [Package structure](#1-package-structure)
2. [Controllers (ros2_control)](#2-controllers-ros2_control)
3. [Pipeline 1 — Velocity](#3-pipeline-1--velocity)
4. [Pipeline 2 — Acceleration](#4-pipeline-2--acceleration)
5. [Pipeline 3 — Torque](#5-pipeline-3--torque)
6. [Pipeline 4 — CBF / Avoidance](#6-pipeline-4--cbf--avoidance)
7. [Trajectory nodes](#7-trajectory-nodes)
8. [Kinematics library (fr3_kinematics.py)](#8-kinematics-library-fr3_kinematicspy)
9. [Custom interfaces](#9-custom-interfaces)
10. [Test infrastructure](#10-test-infrastructure)
11. [Topic reference](#11-topic-reference)
12. [Critical notes](#12-critical-notes)

---

## 1. Package structure

```
franka_simulation/
├── launch/
│   ├── sim_velocity.launch.py        # Velocity pipeline
│   ├── sim_acceleration.launch.py    # Acceleration pipeline (+ bridge node)
│   ├── sim_torque.launch.py          # Torque pipeline (gazebo_effort:=true)
│   ├── sim_position.launch.py        # Position pipeline (fr3_arm_controller)
│   └── move_group.launch.py          # CBF/Avoidance pipeline (MoveIt + blender)
│
├── config/
│   ├── controllers.yaml              # All controller type + parameter declarations
│   ├── fr3_arm_controller.yaml       # JointTrajectoryController config
│   ├── fr3_velocity_controller.yaml  # Velocity controller joints
│   ├── velocity_blender_params.yaml  # CBF blender parameters
│   └── avoidance_params.yaml         # Online avoidance controller parameters
│
├── scripts/
│   ├── sim_acceleration_bridge.py    # q̈→q̇ integration bridge (standalone node)
│   ├── velocity_control_blender.py   # CBF-QP blender (MoveIt traj + avoidance)
│   ├── online_avoidance_controller.py # Pinocchio + capsule distance/CBF
│   ├── franka_motion_server.py       # MoveIt action server (MoveToPose etc.)
│   ├── obstacle_synchronizer.py      # URDF xacro → MoveIt planning scene
│   ├── human_pose_node.py            # MediaPipe 2D skeleton → HumanPose2D
│   ├── image_publisher.py            # RealSense image republisher
│   │
│   ├── trajectories/
│   │   ├── cartesian_velocity_mapper.py    # Cartesian circle → qdot (DLS)
│   │   ├── cartesian_acceleration_mapper.py # Cartesian circle → qddot (closed-loop PD)
│   │   ├── cartesian_torque_mapper.py      # G(q) + joint PD + Jᵀ·F → torque
│   │   ├── velocity_circle_trajectory.py   # Joint-space sinusoidal qdot
│   │   ├── velocity_figure8_trajectory.py  # Joint-space Lissajous qdot
│   │   ├── acceleration_smooth_trajectory.py # Joint-space sinusoidal qddot
│   │   ├── torque_sine_trajectory.py       # Joint-space PD + sinusoidal perturbation
│   │   └── trajectory_plotter.py           # /joint_states recorder + CSV + matplotlib
│   │
│   └── utils/
│       ├── fr3_kinematics.py         # URDF-based FK, Jacobian, DLS, gravity (numpy only)
│       ├── avoidance_core.py         # Capsule geometry (ROS-agnostic)
│       ├── avoidance_math.py         # CBF math, closest points, DLS (ROS-agnostic)
│       ├── velocity_blender_core.py  # Blender math (polyline projection, POCS)
│       └── …                         # Additional blender helpers, logging, params
│
├── action/
│   ├── MoveToPose.action
│   ├── MoveToJoint.action
│   └── PlanGlobalPath.action
│
├── msg/
│   └── HumanPose2D.msg               # MediaPipe 2D skeleton landmarks
│
├── test/
│   ├── launch/                       # Test launch files (one per pipeline)
│   ├── scripts/                      # Test publisher nodes + check_pipeline.sh
│   └── config/test_publishers.yaml   # Default test parameters
│
└── urdf/obstacles/                   # Obstacle URDF xacro (box scene)
```

---

## 2. Controllers (ros2_control)

All controllers are declared in `config/controllers.yaml`. The `controller_manager` runs
inside Gazebo at `update_rate: 1000 Hz`.

> **Note**: controller types must also be declared in
> `franka_gazebo_bringup/config/franka_gazebo_controllers.yaml` (loaded by the Gazebo
> plugin at startup). Without that entry, the `controller_manager` inside Gazebo does
> not know the controller type even if the spawner provides a `--param-file`.

### joint_state_broadcaster

```yaml
type: joint_state_broadcaster/JointStateBroadcaster
```

- No command interface
- State interfaces: all joints (position, velocity, effort)
- Publishes: `/joint_states` (`sensor_msgs/JointState`)
- Rate: inherited from physics update (up to 1000 Hz in Gazebo)
- Active in all four pipelines

### fr3_arm_controller (position pipeline)

```yaml
type: joint_trajectory_controller/JointTrajectoryController
joints: [fr3_joint1 … fr3_joint7]
command_interfaces: [position]
state_interfaces:   [position, velocity, effort]
open_loop_control: true
allow_integration_in_goal_trajectories: true
```

- Receives `trajectory_msgs/JointTrajectory` on `/fr3_arm_controller/joint_trajectory`
- Used by MoveIt via `moveit_simple_controller_manager`
- Spawned `--inactive` in `move_group.launch.py` (MoveIt activates it when needed)

### fr3_velocity_controller (velocity and acceleration pipelines)

```yaml
type: velocity_controllers/JointGroupVelocityController
joints: [fr3_joint1 … fr3_joint7]
command_interfaces: [velocity]
state_interfaces:   [position, velocity]
```

- Receives `std_msgs/Float64MultiArray` (7 floats, [rad/s]) on
  `/fr3_velocity_controller/commands`
- Used directly in velocity pipeline; used via bridge in acceleration pipeline;
  used by velocity_control_blender in CBF pipeline

### fr3_effort_controller (torque pipeline)

```yaml
type: effort_controllers/JointGroupEffortController
joints: [fr3_joint1 … fr3_joint7]
command_interfaces: [effort]
state_interfaces:   [position, velocity]
```

- Receives `std_msgs/Float64MultiArray` (7 floats, [Nm]) on
  `/fr3_effort_controller/commands`
- Requires `gazebo_effort:=true` in the xacro mapping (exposes effort command interface)
- **Only** active in `sim_torque.launch.py`

### fr3_gripper

```yaml
type: position_controllers/GripperActionController
joint: fr3_finger_joint1
max_effort: 100.0
```

- Loaded in all pipelines (gripper loaded by default: `load_gripper:=true`)

---

## 3. Pipeline 1 — Velocity

**Launch**: `sim_velocity.launch.py`  
**xacro mapping**: `gazebo_effort:=false` (velocity interface only)

### Startup sequence

| Delay | Action |
|-------|--------|
| 0 s | Gazebo Ignition (`empty.sdf -r`) + RSP |
| 0 s | `/clock` bridge (`rosgraph_msgs/Clock ↔ gz.msgs.Clock`) |
| 3 s | `ros_gz_sim/create` — spawn FR3 from `/robot_description` |
| 5 s | `spawner joint_state_broadcaster` |
| 7 s | `spawner fr3_velocity_controller` |
| 8.5 s | RViz2 (`visualize_franka.rviz`, fixed frame `fr3_link0`) |
| 10 s | Trajectory node (`cartesian_velocity_mapper` by default) |

### Data flow

```
cartesian_velocity_mapper
        │  /fr3_velocity_controller/commands  (Float64MultiArray, 7×[rad/s])
        ▼
fr3_velocity_controller  ──────────────────────────►  Gazebo physics
        ▲
/joint_states  (sensor_msgs/JointState)
```

### Default trajectory node: `cartesian_velocity_mapper`

Generates a horizontal circle in the EE XY plane. At first step, FK is
called on the current `q` to obtain `x_cur`, then the circle centre is set to
`x_cur − [R, 0, 0]` so the EE lies exactly on the circle at `t=0` (no initial
position error).

Control law (100 Hz):

```
xdot_cmd = ramp(t) · (xdot_des + KP_CART · (x_des − x_cur))
qdot     = J†(q) · xdot_cmd          # DLS pseudoinverse, λ variable
qdot     = clip(qdot, −0.6·qdot_max, +0.6·qdot_max)
```

Parameters (hardcoded in script):

| Name | Value | Description |
|------|-------|-------------|
| `RADIUS` | 0.05 m | Circle radius |
| `OMEGA` | 0.30 rad/s | Angular speed (~21 s period) |
| `KP_CART` | 1.5 1/s | Corrective position gain |
| `DLS_LAM` | 0.05 | DLS base damping |
| `RAMP_S` | 3.0 s | Cosine startup ramp |

### Alternative trajectory nodes (change `executable=` in launch)

| Executable | Description |
|------------|-------------|
| `velocity_circle_trajectory` | Joint-space sinusoidal qdot, per-joint phases |
| `velocity_figure8_trajectory` | Joint-space Lissajous (ω and 2ω alternating joints) |
| `cartesian_velocity_mapper` | Cartesian circle, Jacobian-based (default) |

---

## 4. Pipeline 2 — Acceleration

**Launch**: `sim_acceleration.launch.py`  
**xacro mapping**: same as velocity (`gazebo_effort:=false`)

### Startup sequence

| Delay | Action |
|-------|--------|
| 0 s | Gazebo + RSP + clock bridge |
| 3 s | spawn FR3 |
| 5 s | `spawner joint_state_broadcaster` |
| 7 s | `spawner fr3_velocity_controller` |
| 7.5 s | `sim_acceleration_bridge` node |
| 8.5 s | RViz2 |
| 10 s | Trajectory node (`cartesian_acceleration_mapper` by default) |

### sim_acceleration_bridge

**Node**: `sim_acceleration_bridge` (script: `scripts/sim_acceleration_bridge.py`)  
**Role**: integrates external joint acceleration commands into velocity setpoints

```
/sim_acceleration_bridge/accel_cmd  (Float64MultiArray, 7×[rad/s²])
             │
    Low-pass filter:  qddot_filt[k] = (1−α)·qddot_filt[k-1] + α·qddot_raw[k]
             │        α = 0.2  (τ ≈ 40 ms at 100 Hz)
             │
    Euler integration (100 Hz):
    qdot_cmd = clip(qdot_actual + qddot_filt · dt,  −qdot_max,  +qdot_max)
             │
             ▼
/fr3_velocity_controller/commands  (Float64MultiArray, 7×[rad/s])
```

- `qdot_actual` is read from `/joint_states` (BEST_EFFORT QoS, depth 1)
- `qdot_max` defaults to FR3 limits `[2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26]` rad/s
- Configurable via ROS parameters (`rate_hz`, `qdot_max`, `accel_cmd_topic`, `vel_cmd_topic`)

### Data flow

```
cartesian_acceleration_mapper
        │  /sim_acceleration_bridge/accel_cmd  (Float64MultiArray, 7×[rad/s²])
        ▼
sim_acceleration_bridge  (100 Hz integrator + LP filter)
        │  /fr3_velocity_controller/commands  (Float64MultiArray, 7×[rad/s])
        ▼
fr3_velocity_controller  ──────────────────────────►  Gazebo physics
        ▲
/joint_states  (sensor_msgs/JointState)
```

### Default trajectory node: `cartesian_acceleration_mapper`

Generates a horizontal circle in the EE XY plane. Circle centre is set so the
EE is on the circle at `t=0` (same zero-initial-error trick as velocity mapper:
`center = x_cur − [R, 0, 0]`).

Control law (100 Hz) — closed-loop Cartesian PD with feedforward, no J̇ term:

```
x_des    = center + [R·cos(ω·t),  R·sin(ω·t),  0]
xdot_des = [−R·ω·sin(ω·t),  R·ω·cos(ω·t),  0]
xddot_des = [−R·ω²·cos(ω·t), −R·ω²·sin(ω·t), 0]     # centripetal feedforward

xdot_cur = J(q) · qdot_actual                          # estimated EE velocity

xddot_cmd = xddot_des + KP_C·(x_des − x_cur) + KD_C·(xdot_des − xdot_cur)
qddot     = J†(q) · xddot_cmd                         # DLS pseudoinverse
qddot     = clip(ramp(t) · qddot,  −qddot_max,  +qddot_max)
```

The J̇·q̇ term (present in the full operational-space acceleration formula) is
intentionally omitted to avoid numerical differentiation noise.

Parameters:

| Name | Value | Description |
|------|-------|-------------|
| `RADIUS` | 0.05 m | Circle radius |
| `OMEGA` | 0.25 rad/s | Angular speed |
| `KP_C` | 20.0 1/s² | Cartesian position gain |
| `KD_C` | 8.0 1/s | Cartesian velocity gain |
| `DLS_LAM` | 0.05 | DLS base damping |
| `RAMP_S` | 4.0 s | Cosine startup ramp |
| `QDDOT_MAX` | [5,5,5,5,8,8,8] rad/s² | Per-joint clamp |

### Alternative trajectory nodes

| Executable | Description |
|------------|-------------|
| `acceleration_smooth_trajectory` | Joint-space sinusoidal qddot, zero-mean (bounded velocity) |
| `cartesian_acceleration_mapper` | Cartesian circle, closed-loop PD (default) |

---

## 5. Pipeline 3 — Torque

**Launch**: `sim_torque.launch.py`  
**xacro mapping**: `gazebo_effort:=true` — this flag exposes the effort command interface
in the URDF; without it the `controller_manager` inside Gazebo never registers
`fr3_joint{1..7}/effort [available]`.

### Startup sequence

| Delay | Action |
|-------|--------|
| 0 s | Gazebo + RSP (`use_sim_time: true`) + clock bridge |
| 3 s | spawn FR3 |
| 5 s | `spawner joint_state_broadcaster` |
| 7 s | `spawner fr3_effort_controller` |
| 8.5 s | RViz2 |
| 10 s | Trajectory node (`cartesian_torque_mapper` by default) |

### Data flow

```
cartesian_torque_mapper
        │  /fr3_effort_controller/commands  (Float64MultiArray, 7×[Nm])
        ▼
fr3_effort_controller  ──────────────────────────►  Gazebo physics (effort actuators)
        ▲
/joint_states  (sensor_msgs/JointState)
```

### Gravity semantics in Gazebo (critical)

The Gazebo physics engine simulates gravity on link masses independently of
the ros2_control effort controller. The `fr3_effort_controller` injects
commanded torques into the joint actuators as additional forces on top of
whatever the physics engine already computes.

In practice this means:
- Publishing `τ = 0` → zero actuator torque; Gazebo physics handles the arm
  weight but it is uncertain whether the arm holds position or falls depending
  on the gz_ros2_control version and whether the physics constraint solver
  provides static equilibrium.
- The `cartesian_torque_mapper` adds analytic `G(q)` explicitly.
  **If Gazebo already provides gravity equilibrium at τ=0, this results in
  double gravity compensation** and the arm will slowly drift upward.
  Use `torque_sine_trajectory` (joint-space PD without explicit G(q)) for a
  safer conservative approach.

### Default trajectory node: `cartesian_torque_mapper`

Control law (100 Hz):

```
G         = gravity_torques(q)              # analytic, from URDF link masses
F_cart(t) = [F_AMP·sin(ω·t),  0,  0]       # slow sinusoidal force in world X
τ         = G  +  ramp(t) · (KP·(q_home − q)  −  KD·qdot  +  Jᵀ(q)·F_cart)
τ         = clip(τ, −TAU_MAX, +TAU_MAX)
```

`G(q)` from `fr3_kinematics.py::gravity_torques()` is analytic (no Pinocchio).
`KP`, `KD` are per-joint arrays (higher for proximal joints).

Parameters:

| Name | Value | Description |
|------|-------|-------------|
| `F_AMP` | 3.0 N | Cartesian force amplitude |
| `OMEGA` | 0.20 rad/s | Oscillation frequency |
| `KP` | [40,55,35,55,20,15,10] Nm/rad | Joint stiffness |
| `KD` | [6,8,5,8,3,2,1.5] Nm·s/rad | Joint damping |
| `RAMP_S` | 5.0 s | Cosine startup ramp |
| `TAU_MAX` | [87,87,87,87,12,12,12] Nm | Joint torque clamp |

### Alternative trajectory nodes

| Executable | Description |
|------------|-------------|
| `torque_sine_trajectory` | Joint-space PD (implicit gravity) + sinusoidal perturbation |
| `cartesian_torque_mapper` | Analytic G(q) + joint PD + Jᵀ·F (default) |

---

## 6. Pipeline 4 — CBF / Avoidance

**Launch**: `move_group.launch.py`  
**xacro mapping**: `gazebo_effort:=false` (velocity interface)

This is the most complex pipeline. It combines:
- MoveIt `move_group` for trajectory planning
- `online_avoidance_controller` for real-time capsule-based distance computation
- `velocity_control_blender` for CBF-QP blending of trajectory tracking and avoidance
- `obstacle_synchronizer` to bridge Gazebo obstacles into MoveIt planning scene

### Startup sequence (from `move_group.launch.py`)

| Delay | Action |
|-------|--------|
| 0 s | Gazebo + clock bridge + static TF (`world→base`, `base→fr3_link0`) |
| 0 s | Optional: RealSense camera driver |
| 2 s | RSP + MoveIt `move_group` (if `enable_moveit:=true`) |
| 3 s | spawn FR3, obstacle RSP |
| 4 s | Optional: image_publisher, human_pose_node |
| 4.5 s | Optional: spawn obstacle in Gazebo |
| 5 s | `spawner joint_state_broadcaster` |
| 7 s | `spawner fr3_arm_controller --inactive` |
| 8 s | `spawner fr3_velocity_controller` |
| 3 s | `obstacle_synchronizer` (if `spawn_obstacles:=true`) |
| 10 s | RViz2 (MoveIt config if `enable_moveit:=true`) |
| 11 s | `online_avoidance_controller` |
| 13 s | `velocity_control_blender` |
| 14 s | `franka_motion_server` |
| 18 s | Optional: `safe_avoidance_test` |

### Data flow

```
franka_motion_server  (MoveIt2, pymoveit2)
        │  /velocity_blender/trajectory  (trajectory_msgs/JointTrajectory)
        ▼
velocity_control_blender  ◄──────────────────────────────────────────────
        │                                                                  │
        │   reads /avoidance/closest_constraint (Float64MultiArray, 7)    │
        │   reads /avoidance/min_distance       (Float64MultiArray)       │
        │   reads /joint_states                 (JointState)              │
        │                                                                  │
        │  /fr3_velocity_controller/commands  (Float64MultiArray, 7×[rad/s])
        ▼
fr3_velocity_controller  ──────────────────────────►  Gazebo physics
        ▲
/joint_states  (sensor_msgs/JointState)

online_avoidance_controller  (Pinocchio + capsule geometry)
        │  subscribes /joint_states
        │  subscribes /obstacle_scene  (from obstacle_synchronizer)
        ├→ /avoidance/min_distance      (Float64MultiArray)
        ├→ /avoidance/closest_constraint (Float64MultiArray, 7 — distance Jacobian)
        └→ /robot_capsules_markers      (visualization_msgs/MarkerArray)
```

### online_avoidance_controller

- Uses **Pinocchio** for FK and Jacobian computation (`pin.computeAllTerms`)
- Models robot body as 8 capsules (base + 6 joints + flange), configurable radii
- Computes minimum distance to external obstacles (box shapes from planning scene)
- Publishes closest constraint Jacobian `∂d/∂q` (translational part, 7-vector)
- Parameters in `config/avoidance_params.yaml`

### velocity_control_blender (CBF-QP)

Blends trajectory tracking velocity with constraint-based avoidance:

1. **Trajectory tracking**: nearest point on polyline projection + lookahead;
   tracking velocity `v_track = kp · (q_lookahead − q)`, filtered with LPF
2. **CBF constraint**: `ḋ ≥ −κ · (d − d_min)` projected onto `∂d/∂q`
3. **POCS projection**: iterative projection onto halfspace constraints
4. **Emergency mode**: hard override when `d < 0.08 m`
5. **Null-space tangential escape**: anti-stall when avoidance collapses blended cmd
6. Parameters: `config/velocity_blender_params.yaml` (≈ 40 parameters)

### franka_motion_server

- Action servers: `MoveToPose`, `MoveToJoint`, `PlanGlobalPath`
- Uses `pymoveit2.MoveIt2` for planning + execution
- Collision avoidance **disabled** in IK (`avoid_collisions=False`):
  avoidance is entirely delegated to `online_avoidance_controller`
- Publishes planned trajectory to `/velocity_blender/trajectory`

### obstacle_synchronizer

- Reads `urdf/obstacles/multi_obstacle_scene.urdf.xacro` (or configurable path)
- Publishes `moveit_msgs/PlanningScene` to `/obstacle_scene`
- Spawns corresponding object in Gazebo via `/obstacle/robot_description` + RSP

---

## 7. Trajectory nodes

All trajectory nodes are installed as ROS 2 executables (no `.py` suffix) via
`install(PROGRAMS … RENAME)` in `CMakeLists.txt`. Python path includes `utils/`
(installed adjacent to executables).

### Summary table

| Executable | Pipeline | Control space | Publishes to | Notes |
|------------|----------|---------------|--------------|-------|
| `cartesian_velocity_mapper` | velocity | Cartesian → joint via DLS | `/fr3_velocity_controller/commands` | Closed-loop PD, FK feedback |
| `velocity_circle_trajectory` | velocity | Joint | `/fr3_velocity_controller/commands` | Sinusoidal per joint |
| `velocity_figure8_trajectory` | velocity | Joint | `/fr3_velocity_controller/commands` | Lissajous (ω + 2ω) |
| `cartesian_acceleration_mapper` | acceleration | Cartesian → joint via DLS | `/sim_acceleration_bridge/accel_cmd` | Closed-loop PD, no J̇ |
| `acceleration_smooth_trajectory` | acceleration | Joint | `/sim_acceleration_bridge/accel_cmd` | Zero-mean sinusoid |
| `cartesian_torque_mapper` | torque | Cartesian + joint | `/fr3_effort_controller/commands` | G(q) + PD + Jᵀ·F |
| `torque_sine_trajectory` | torque | Joint | `/fr3_effort_controller/commands` | PD spring + sinusoidal perturbation |
| `trajectory_plotter` | any | — | — | Records `/joint_states` → CSV + matplotlib |

### Startup ramp

All trajectory nodes apply a raised-cosine envelope to avoid impulse commands:

```python
ramp(t) = 0.5 · (1 − cos(π · min(t, RAMP_S) / RAMP_S))
```

`RAMP_S` ranges from 2 s (velocity) to 5 s (torque).

### trajectory_plotter

Standalone logging tool. Usage:

```bash
ros2 run franka_simulation trajectory_plotter
```

On Ctrl-C: saves `/tmp/trajectory_log_<timestamp>.csv` with columns
`[time_s, q1..q7, dq1..dq7, tau1..tau7]` and optionally shows a matplotlib
3-panel plot (position / velocity / effort). Controlled by `PLOT = True`
at top of script.

---

## 8. Kinematics library (fr3_kinematics.py)

**Location**: `scripts/utils/fr3_kinematics.py`  
**Dependencies**: numpy, math only (no Pinocchio, no KDL)

Used by all trajectory nodes (velocity, acceleration, torque mappers).
The CBF pipeline uses Pinocchio instead (different module).

### Forward kinematics

Transform convention: each joint contributes

```
T_i(q_i) = Trans(xyz_i) @ Rx(rpy_i[0]) @ Rz(q_i)
```

where all 7 FR3 revolute joints rotate about their local Z-axis.
Constant offsets from `franka_description/robots/fr3/fr3.urdf` (NOT DH parameters):

| Joint | xyz | rpy[0] |
|-------|-----|--------|
| 1 | [0, 0, 0.333] | 0 |
| 2 | [0, 0, 0] | −π/2 |
| 3 | [0, −0.316, 0] | +π/2 |
| 4 | [0.0825, 0, 0] | +π/2 |
| 5 | [−0.0825, 0.384, 0] | −π/2 |
| 6 | [0, 0, 0] | +π/2 |
| 7 | [0.088, 0, 0] | +π/2 |
| flange | [0, 0, 0.107] | — (fixed) |

`fk(q7)` returns `(pos, T_4x4)` where `pos` is the flange centre in world frame.

Verified at `Q_HOME = [0, −π/4, 0, −3π/4, 0, π/2, π/4]`:
EE at approximately `[0.307, 0.000, 0.590] m`.

### Geometric Jacobian

Translational 3×7 Jacobian:

```
J[:, i] = z_i × (p_EE − p_i)
```

where `z_i` is the Z-axis of the frame **before** joint `i` applies its rotation,
and `p_i` is the origin of that frame in world coordinates.

### Damped-least-squares pseudoinverse

```
qdot = Jᵀ (J Jᵀ + λ² I)⁻¹ xdot
```

with variable damping: `λ` increases when manipulability
`w = √det(J Jᵀ) < 0.04` to prevent velocity spikes near singularities.

### Gravity torques (analytic)

```
G_i(q) = Σ_{j≥i}  m_j · [0,0,g] · (z_i × (p_com_j − p_i))
```

Link masses and CoM positions from `fr3.urdf` inertial tags (7 links, values
in code comments). No Pinocchio required.

---

## 9. Custom interfaces

### Actions

**`MoveToPose.action`**
- Goal: `geometry_msgs/PoseStamped`, cartesian_motion bool, velocity/acceleration scaling,
  tolerance, planner_id, planning_time
- Result: `moveit_msgs/MoveItErrorCodes`, final_pose, execution_time, planning_time,
  planning_attempts
- Feedback: current_state string, progress [0,1], current_pose, current_attempt, status_message

**`MoveToJoint.action`**
- Goal: target joint configuration + motion parameters
- Result: MoveIt error code + timing
- Feedback: progress + state string

**`PlanGlobalPath.action`**
- Goal: goal pose + planner parameters
- Result: `nav_msgs/Path` + `moveit_msgs/RobotTrajectory`
- Used for planning-only (no execution)

### Messages

**`HumanPose2D.msg`**
```
std_msgs/Header header
uint32   image_width
uint32   image_height
uint32[]  ids          # MediaPipe landmark indices (0–32)
float32[] u            # pixel x
float32[] v            # pixel y
float32[] visibility   # MediaPipe score [0,1]
```

Published by `human_pose_node.py` (MediaPipe Pose model on RealSense RGB stream).

---

## 10. Test infrastructure

```
test/
├── launch/
│   ├── test_velocity_pipeline.launch.py      # sim_velocity + test_velocity_publisher
│   ├── test_acceleration_pipeline.launch.py  # sim_acceleration + test_acceleration_publisher
│   ├── test_torque_pipeline.launch.py        # sim_torque + test_torque_publisher
│   └── test_cbf_pipeline.launch.py           # move_group (CBF pipeline)
├── scripts/
│   ├── test_velocity_publisher.py            # Sinusoidal qdot on 1 joint, parameterizable
│   ├── test_acceleration_publisher.py        # Sinusoidal qddot on 1 joint
│   ├── test_torque_publisher.py              # Phase 0: zeros; Phase 1: sinusoidal torque
│   └── check_pipeline.sh                     # Bash validator (ros2 control + topic hz)
└── config/
    └── test_publishers.yaml                  # Default parameters for all test publishers
```

### Single-command pipeline tests

```bash
ros2 launch franka_simulation test_velocity_pipeline.launch.py
ros2 launch franka_simulation test_acceleration_pipeline.launch.py
ros2 launch franka_simulation test_torque_pipeline.launch.py
ros2 launch franka_simulation test_cbf_pipeline.launch.py
```

### Standalone test publishers

All three publishers are installed as `ros2 run`-able executables and accept ROS parameters:

```bash
# Velocity publisher: 30 s of sinusoidal qdot on joint 1
ros2 run franka_simulation test_velocity_publisher \
  --ros-args -p rate_hz:=50.0 -p amplitude:=0.15 -p duration_s:=30.0 -p active_joint:=0

# Acceleration publisher: 30 s of sinusoidal qddot on joint 1
ros2 run franka_simulation test_acceleration_publisher \
  --ros-args -p amplitude:=0.3 -p frequency:=0.1 -p active_joint:=0

# Torque publisher: 10 s zeros → 20 s sinusoidal torque
ros2 run franka_simulation test_torque_publisher \
  --ros-args -p amplitude:=2.0 -p zeros_duration_s:=10.0 -p active_duration_s:=20.0
```

### check_pipeline.sh

Validates controllers and topics for a given pipeline:

```bash
./test/scripts/check_pipeline.sh velocity
./test/scripts/check_pipeline.sh acceleration
./test/scripts/check_pipeline.sh torque
./test/scripts/check_pipeline.sh cbf
```

Checks: `controller_manager` reachable, `/joint_states` active at ≥50 Hz,
pipeline-specific controller state (`active`/`inactive`), and command topic activity.

---

## 11. Topic reference

### Always active (all pipelines)

| Topic | Type | Publisher | Notes |
|-------|------|-----------|-------|
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_broadcaster` | pos+vel+eff, up to 1000 Hz |
| `/robot_description` | `std_msgs/String` | `robot_state_publisher` | URDF XML string |
| `/clock` | `rosgraph_msgs/Clock` | `ros_gz_bridge` (clock_bridge) | Gazebo sim time bridge |

### Velocity pipeline

| Topic | Type | Publisher → Subscriber | Notes |
|-------|------|------------------------|-------|
| `/fr3_velocity_controller/commands` | `std_msgs/Float64MultiArray` | traj_node → `fr3_velocity_controller` | 7 floats [rad/s] |

### Acceleration pipeline (superset of velocity)

| Topic | Type | Publisher → Subscriber | Notes |
|-------|------|------------------------|-------|
| `/sim_acceleration_bridge/accel_cmd` | `std_msgs/Float64MultiArray` | traj_node → `sim_acceleration_bridge` | 7 floats [rad/s²]; private topic `~/accel_cmd` resolves here |
| `/fr3_velocity_controller/commands` | `std_msgs/Float64MultiArray` | `sim_acceleration_bridge` → controller | integrated+filtered |

### Torque pipeline

| Topic | Type | Publisher → Subscriber | Notes |
|-------|------|------------------------|-------|
| `/fr3_effort_controller/commands` | `std_msgs/Float64MultiArray` | traj_node → `fr3_effort_controller` | 7 floats [Nm] |

### CBF/Avoidance pipeline

| Topic | Type | Publisher → Subscriber |
|-------|------|------------------------|
| `/velocity_blender/trajectory` | `trajectory_msgs/JointTrajectory` | `franka_motion_server` → `velocity_control_blender` |
| `/avoidance/min_distance` | `std_msgs/Float64MultiArray` | `online_avoidance_controller` → `velocity_control_blender` |
| `/avoidance/closest_constraint` | `std_msgs/Float64MultiArray` | `online_avoidance_controller` → `velocity_control_blender` |
| `/robot_capsules_markers` | `visualization_msgs/MarkerArray` | `online_avoidance_controller` → RViz |
| `/fr3_velocity_controller/commands` | `std_msgs/Float64MultiArray` | `velocity_control_blender` → controller |
| `/obstacle_scene` | `moveit_msgs/PlanningScene` | `obstacle_synchronizer` → `online_avoidance_controller` |
| `/obstacle/robot_description` | `std_msgs/String` | `obstacle_synchronizer/RSP` → Gazebo spawn |

### Optional (camera + human pose)

| Topic | Type | Notes |
|-------|------|-------|
| `/color/image_raw` | `sensor_msgs/Image` | RealSense driver |
| `/image_republished` | `sensor_msgs/Image` | `image_publisher` (bridge) |
| `/human_pose_2d` | `franka_simulation/HumanPose2D` | `human_pose_node` |

---

## 12. Critical notes

### Controller type registration (dual YAML requirement)

The `controller_manager` inside Gazebo loads its plugin configuration from
`franka_gazebo_bringup/config/franka_gazebo_controllers.yaml` at startup.
The `--param-file` argument to `spawner` is insufficient for type registration.
Both `fr3_velocity_controller` and `fr3_effort_controller` must appear in that
file with their `type:` declaration, otherwise spawning fails with
`"type not defined for <controller_name>"`.

### Gazebo effort interface activation

`fr3_effort_controller` requires `gazebo_effort:=true` in the xacro mapping
passed to `xacro.process_file()`. This flag controls whether gz_ros2_control
exposes the effort command interface. `sim_torque.launch.py` sets this;
`sim_velocity.launch.py` and `sim_acceleration.launch.py` do not.

Verify with:

```bash
ros2 control list_hardware_interfaces | grep effort
```

Expected output (torque pipeline only):
```
fr3_joint1/effort [available] [claimed]
…
```

### Gravity compensation semantics (torque pipeline)

Gazebo Ignition simulates link gravity via the physics engine.
The `fr3_effort_controller` injects commanded torques directly into the
actuator model on top of physics. There is an unresolved ambiguity in the
codebase comments:

- `sim_torque.launch.py` header states: "Publishing τ=0 → zero motor torque,
  gravity pulls the robot down."
- `test_torque_publisher.py` states: "Publishing zeros → robot at Gazebo
  equilibrium (arm does NOT fall)."

The actual behavior depends on the gz_ros2_control version and physics solver.
The `cartesian_torque_mapper` adds explicit `G(q)` from `gravity_torques()`.
If the physics engine already compensates gravity, this creates a double
compensation and the arm drifts upward. Use `torque_sine_trajectory` (no
explicit G(q)) as a conservative alternative.

### Kinematics convention (URDF, not DH)

`fr3_kinematics.py` builds transforms directly from URDF `<origin xyz rpy>` tags:

```
T_i(q_i) = Trans(xyz_i) @ Rx(rpy_i[0]) @ Rz(q_i)
```

Standard DH tables found online for the FR3/Panda are **not compatible** with
this implementation. Key difference: joint 3 has `xyz="0 -0.316 0"` (offset
in local Y after the RPY rotation), not a DH `d`-parameter offset along Z.
An implementation using DH parameters produces wrong FK (EE error > 20 cm at
home pose).

### Acceleration pipeline stability

The `sim_acceleration_bridge` seeds each integration step from the actual
joint velocity read from `/joint_states`, not from an internal integrated
state. This means if the commanded acceleration produces a velocity that the
`fr3_velocity_controller` cannot track immediately (control delay), the next
integration step starts from a lower-than-commanded velocity, which can cause
instability if the trajectory mapper also has a large initial position error.

Two mitigations are in place:
1. Circle centre offset: `center = x_cur − [R, 0, 0]` ensures zero initial
   Cartesian position error in the trajectory mappers.
2. Low-pass filter in the bridge (α=0.2, τ≈40 ms) smooths acceleration spikes.

### Pinocchio dependency (CBF pipeline only)

`online_avoidance_controller` and `velocity_control_blender` require
`pinocchio` (`import pinocchio as pin`). The trajectory scripts and
`fr3_kinematics.py` do **not** use Pinocchio. The two kinematics implementations
(`fr3_kinematics.py` vs Pinocchio) are independent and will produce slightly
different results near the flange due to frame conventions (URDF flange vs
`fr3_hand_tcp`).

### Script installation path

All Python scripts in `scripts/`, `scripts/trajectories/`, and
`test/scripts/` are installed to `lib/franka_simulation/<name>` (without
`.py`). The `utils/` directory is installed adjacent:
`lib/franka_simulation/utils/`. Scripts import utils with
`from utils.fr3_kinematics import …` which resolves because Python's working
directory at `ros2 run` is `lib/franka_simulation/`.

This path resolution **only works** when launched with `ros2 run` or from a
launch file `Node(package='franka_simulation', executable='…')`. Running the
`.py` files directly from their source location requires adding `scripts/` to
`PYTHONPATH`.
