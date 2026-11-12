# 02 — The ROS 2 graph and the data flow

> **Verified against** HEAD `ba68077`, 2026-11-11. Every table row cites the file that creates the
> interface.
> **Three kinds of evidence are kept apart in this chapter:**
> - **Static**: read in the source at HEAD.
> - **Runtime (recorded)**: from an artefact that a previous run wrote. Mainly the fake-hardware
>   graph capture in `franka_simulation/docs/sim_reference_contract/fake_hw/` (commit `ac528f1`,
>   21–22 Oct 2026, *older than HEAD*), and simulation `cell_result.json` files.
> - **Unverified**: neither of the above.
>
> Nothing was executed for this chapter. The raw call-site extraction is in
> [ledger/ros_interfaces.md](ledger/ros_interfaces.md).

---

## 1. Intuition — the graph as a set of contracts

Each arrow in the architecture diagram of [chapter 01](01_architecture.md) is a ROS 2 topic. A
topic is a contract with five clauses:
1. a **name**;
2. a **message type**;
3. a **rate**, or a trigger;
4. a **QoS policy**: reliability, history depth and durability;
5. a **frame**, the coordinate system that the numbers inside are expressed in.

Most integration bugs in a system like this break one clause while leaving the others intact. For
example:
- a subscriber with the right name but an incompatible QoS policy silently receives nothing;
- a message with the right type in the wrong frame is silently wrong.

This chapter writes down all five clauses for every topic on the control path.

## 2. Foundations

### 2.1 Name resolution, and where `NS_1` comes from

ROS 2 resolves a name like this:
- a name starting with `/` is **absolute** and used as written;
- otherwise it is **relative**, and the node's namespace is prepended.

Which of the two forms each node uses determines the whole naming scheme.

**The namespace `NS_1` is not set in this repository's launch files.** It comes from a chain:

1. `franka_bringup/config/franka.config.yaml` → `ROBOT1.namespace: "NS_1"`.
2. `utils/config.py::load_franka_config_defaults()` reads that file.
3. `torque_control_stack.launch.py` merges the defaults as
   `_DEFAULTS = {**_LAUNCH_DEFAULTS, **_BRINGUP_DEFAULTS}`, so **the bringup file wins** over
   `launch_defaults.yaml`, which has `namespace: ""`.
4. The `namespace` launch argument (from `launch_support.declare_robot_args`) is passed to
   `franka_bringup/launch/franka.launch.py`.
5. Inside `NS_1`, the bringup runs the hardware interface, `/NS_1/controller_manager`,
   `joint_state_broadcaster` (publishing `/NS_1/franka/joint_states`), `robot_state_publisher`, and
   a `joint_state_publisher` that republishes on `/NS_1/joint_states`.

The same mechanism overrides `robot_ip`: `launch_defaults.yaml` says `192.168.1.10`, and the
bringup file says `192.168.2.10`, which wins. The recorded launch log confirms this at runtime
(`[torque_stack] ... ip=192.168.2.10`, in `.../fake_hw/launch.log`).

Consequences:
- **Python nodes use absolute names**, read from `fr3_control.yaml → topics:` (all `/NS_1/...`)
  or from node defaults (also `/NS_1/...`). `cbf_safety_filter`, `qddot_to_torque` and
  `real_time_distance` are launched **without** a namespace. Their names, `/cbf_safety_filter`
  and so on, are global, which the recorded graph confirms. The commander is launched with
  `namespace=NS_1`, which changes only its node name (`/NS_1/pentagon_qddot_commander`), because
  its topics are absolute.
- **The C++ controllers use relative names** inside the controller manager's namespace.
  `rt_torque_controller.command_topic: torque_cmd` resolves to `/NS_1/torque_cmd`. Its second
  input, `accel_topic`, is written as the absolute literal `/NS_1/qddot_safe` by
  `launch_support.py::_generate_torque_yaml`.
- **`NS_1` is hard-coded in many places:** `fr3_control.yaml`, node defaults, `launch_support.py`,
  the simulation scripts (`scenario_commander`, `ground_truth_node`, `reflex_monitor`, …) and
  `gz_fr3_plant.launch.py` (`namespace` default `NS_1`, described as "the contract"). Changing
  `ROBOT1.namespace` alone **would not move** these topics (Derived).

### 2.2 Trigger models and the sample-and-hold delay

A node publishes either:
- **on a timer** at a fixed period `T`; or
- **on an event**, meaning inside the callback of a message it receives.

When a timer-driven node A reads the *latest* value of a stream produced at period `T_p`, two
waits can add up. The input sample can be up to `T_p` old when it arrives, and it is then held
until A's next tick, which is up to `T_A` away. The worst-case **sampling staleness** of a chain
of hold-and-sample stages is therefore

    Δ_max = Σ_k T_k        (the sum over every stage that samples a held value)

An event-driven stage adds no sampling wait, only its own computation time. This formula is
behind the latency example in chapter 01 §7, and it explains why the framework compensates for a
"blind time" explicitly (chapter 07).

### 2.3 QoS rules that matter here

- **Reliability compatibility.** A subscriber requesting `RELIABLE` does **not** match a publisher
  offering `BEST_EFFORT`. The other three combinations match (a `BEST_EFFORT` subscriber accepts
  either kind of publisher). When there is no match, no connection forms and no warning appears in
  the node's logic. This is a ROS 2 / DDS rule. It comes from general ROS 2 knowledge and was not verified in this
  repository or at runtime here.
- **`QoSProfile(depth=1)`** keeps only the newest message. With an integer depth such as
  `create_subscription(..., 10)`, rclpy uses the default profile: RELIABLE, VOLATILE, KEEP_LAST
  with that depth.
- **Effect in this framework.** The distance stream `/cbf/per_link_distances` is published
  `BEST_EFFORT`, depth 1, by `real_time_distance`. It is published the same way by
  `emulated_perception` in simulation. **Every subscriber must therefore be `BEST_EFFORT`.** At
  HEAD every subscriber is, with one exception: `cbf_OSCBF_filter` (§7.2, finding F-1-6).

### 2.4 Executors and callback groups

`cbf_safety_filter` is the only node on the control path with an explicit multithreaded design.
It is verified in `cbf_safety_filter.py::__init__` (L421–528) and `main` (L2075):

| Callback group (`MutuallyExclusiveCallbackGroup`) | Callbacks |
|---|---|
| `grp_io` | joint-state, `q̈_nom`, distance and `iso_safety` subscriptions |
| `grp_perc` | `_update_constraints`, the 50 Hz rebuild of the constraint geometry |
| `grp_ctrl` | `_qp_tick`, the QP at `qp_rate_hz` |

The executor is a `MultiThreadedExecutor(num_threads=3)`.

> **Note 2026-11-12 (R1, branch `refactor/cbf-executor-scheduling`, uncommitted; the documented
> baseline is unchanged).** The executor is now chosen by the parameter `executor_mode`
> (`fr3_control.yaml`, `CBF_PARAM_SPEC`). `multi` is the default and is the executor above;
> `single` is a `SingleThreadedExecutor` (`cbf_safety_filter.py::make_executor`, `main`).
> The callback groups are unchanged. [refactoring/R1_EXECUTOR_CONFIGURATION.md](refactoring/R1_EXECUTOR_CONFIGURATION.md).

These run in one Python interpreter, so they share the GIL. The QP thread's optional SCHED_FIFO
elevation (`qp_thread_fifo_prio`) has defaulted to **off since 2026-11-07**, with a comment
explaining that it caused a priority inversion through the GIL (`_elevate_thread_priority`).
Every other Python node on the path uses the default single-threaded executor.

## 3. Pipeline A: node-by-node interfaces (static)

Topic values below are resolved through `fr3_control.yaml → topics:` and
`fr3_complete.yaml → topics:`. "BE" means BEST_EFFORT; "R" means RELIABLE.

### 3.1 `real_time_distance` (perception and distance)

Source: `nodes/real_time_distance.py::__init__`. It is launched by `torque_control_stack` at
`t = torque_rtd_delay_s = 2 s`.

| Dir | Topic | Type | QoS | Trigger / rate |
|---|---|---|---|---|
| sub | `/camera/camera/depth/image_rect_raw` (`fr3_complete.topics.depth_image`) | `sensor_msgs/Image` | default (R, 10) | camera; the profile `848x480x90` gives 90 Hz |
| sub | `/camera/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | R, 10 | camera |
| pub | `/cbf/per_link_distances` | `franka_msgs/MultiLinkDistance` | **BE, depth 1** | one per processed depth frame |
| pub | `/human_robot/multi_distance` | `franka_msgs/MultiDistance` | R, 10 | per frame |
| pub | `/human_robot/distance` (node default; the YAML has no `distance` key) | `franka_msgs/HumanRobotDistance` | R, 10 | per frame; **no subscriber in the repository** |
| pub | `/real_time_distance/overlay_image` | `sensor_msgs/Image` | R, 10 | vis timer, `distance.vis_rate_hz = 10` Hz; only if `publish_overlay_image` |
| TF | lookups `fr3_link0 → fr3_link{1..8}` | — | — | per frame (`utils/tf_manager.py::lookup_all`) |

Notes:
- The QoS of the two camera subscriptions is the rclpy default (RELIABLE). Whether that matches
  the RealSense driver's publisher QoS is **Unverified**. The driver usually offers RELIABLE for
  images, which would match.
- **The camera pose does not come from TF.** It is read from the file `camera_extrinsics_path`
  (`config/camera_extrinsics.yaml`: `base → camera_color_optical_frame`) by
  `utils/config.py::load_extrinsics`, at L119 of the node.
- The node subscribes to the **depth** image, whose native frame is
  `camera_depth_optical_frame` (`fr3_complete.topics.depth_optical_frame`). Whether the code
  accounts for the difference between the depth and colour optical frames is an **open question
  for Stage 2**, recorded as Q-1-2.

### 3.2 The commander (`pentagon_qddot_commander`, default)

Source: `nodes/pentagon_qddot_commander.py`, parameters at L100–316 and interfaces at L633–664.
It is launched in namespace `NS_1` at `t = control_spawner_delay_s + 2 = 12 s`.

| Dir | Topic | Type | QoS | Rate |
|---|---|---|---|---|
| sub | `/NS_1/franka/joint_states` (`joint_state_topic = __auto__` resolves to `topics.joint_states_fast`) | `JointState` | R, 1 | 1 kHz on hardware |
| sub | `/NS_1/cbf_status` | `Float64MultiArray` | R, 10 | QP rate |
| sub | `/NS_1/iso_safety` | `Float64MultiArray` | R, 10 | only if the ISO monitor runs |
| sub | `/NS_1/joint_states` (`joint_effort_topic`) | `JointState` | R, 10 | 30 Hz republisher |
| pub | `/NS_1/qddot_nom` (`qddot_safe_topic` defaults to `topics.qddot_nom`) | `Float64MultiArray` (7) | R, 10 | timer, `rate_hz = 100` |
| pub | `/NS_1/q_des_state` | `JointState` | R, 10 | 100 Hz; **no consumer in any shipped launch** (only `CBFTorqueController` reads it, below) |
| pub | `/NS_1/ee_desired`, `/NS_1/ee_actual` | `geometry_msgs/PointStamped` | R, 1 | 100 Hz; frame `fr3_link0` |

- **No MoveIt services.** The module imports no `moveit_msgs` and calls no MoveIt service
  (Verified by grep over `franka_experiments/franka_experiments`). The simulation launch says so
  too ("the commander does not use it", in `gz_cbf_stack.launch.py`). Yet
  `torque_control_stack.launch.py` defaults `start_move_group:=true` and its comment claims the
  commander uses `compute_fk` / `compute_cartesian_path` (F-1-5). The recorded fake-hardware graph
  does contain `/NS_1/move_group`, so it is started, but nothing in Pipeline A calls it.
- **The node and module name say "pentagon", but the default path is a circle.** The shipped
  `fr3_control.yaml → params.path_type` is `circle`, the cycle time is 6 s, and the plane default
  is `yz`. The module docstring still says "Pentagon trajectory for cbf_torque_controller — no CBF
  filter", which is stale.

### 3.3 `rl_policy_commander` (with `motion_source:=rl`)

Source: `nodes/rl_policy_commander.py` L129–369.

| Dir | Topic | Type | QoS | Rate |
|---|---|---|---|---|
| sub | joint states (`__auto__`) | `JointState` | R, 1 | — |
| sub | `/cbf/per_link_distances` | `MultiLinkDistance` | **BE, 1** | perception rate |
| sub | `/NS_1/cbf_status` | `Float64MultiArray` | R, 10 | — |
| sub | `target_topic` (empty by default, so not created) | `PointStamped` | — | — |
| pub | `/NS_1/qddot_nom` | `Float64MultiArray` | R, 10 | timer, `rate_hz`, or else the policy's `control_rate_hz` (100) |
| pub | `/NS_1/rl_status` | `Float64MultiArray` | R, 10 | same; **no consumer in the repository** |

### 3.4 `cbf_safety_filter`

Source: `nodes/cbf_safety_filter.py::__init__` L425–528. It is launched without a namespace at
`t = torque_dynamics_delay_s = 2 s`.

| Dir | Topic | Type | QoS | Rate |
|---|---|---|---|---|
| sub | `/NS_1/franka/joint_states` (`topics.joint_states_fast`) | `JointState` | R, depth 1 | 1 kHz |
| sub | `/NS_1/qddot_nom` | `Float64MultiArray` | R, 1 | 100 Hz |
| sub | `/cbf/per_link_distances` | `MultiLinkDistance` | **BE, 1** | perception rate; parsed in `_on_distances` into the cached tuple `self._obs` |
| sub | `/NS_1/iso_safety` | `Float64MultiArray` | R, 1 | if the ISO monitor runs |
| pub | `/NS_1/qddot_safe` | `Float64MultiArray` (7) | R, 10 | `_qp_tick`, `qp_rate_hz = 300` |
| pub | `/NS_1/cbf_status` | `Float64MultiArray` (`len(STATUS_FIELDS)`, schema in `utils/cbf_diagnostics.py`) | R, 10 | QP rate |
| timer | `_update_constraints` | — | — | `cbf_update_rate_hz = 50` |
| timer | `_qp_tick` | — | — | `qp_rate_hz = 300` |

### 3.5 `qddot_to_torque`

Source: `nodes/qddot_to_torque.py` L68–165. There is no timer; the node is event-driven.

| Dir | Topic | Type | QoS | Rate |
|---|---|---|---|---|
| sub | `/NS_1/franka/joint_states` | `JointState` | R, depth 1 | 1 kHz; mapped **by joint name** (L177–183) |
| sub | `/NS_1/qddot_safe` | `Float64MultiArray` | R, 10 | QP rate; the callback is named `_on_qddot_nom` (a naming leftover) |
| pub | `/NS_1/torque_cmd` (parameter `torque_out_topic`, defaulting to `topics.torque_cmd`) | `Float64MultiArray` (7) | R, 10 | one per `q̈_safe` |
| pub | `/NS_1/torque_saturation` | `Float64MultiArray` | R, 10 | same |

The torque-rate limit `iso_tau_rate_max` is rescaled by `100 / qp_rate_hz` (L98). It is applied
only when `iso_enabled` is set (chapter 09).

### 3.6 `rt_torque_controller` (C++, `franka_rt_controllers`)

Source: `franka_rt_controllers/src/rt_torque_controller.cpp`. Parameters are declared at
L286–350 and subscriptions created at L473 and L501. It is spawned at
`t = control_spawner_delay_s = 10 s`.

| Dir | Interface | Notes |
|---|---|---|
| sub | `command_topic` = `torque_cmd`, relative, so `/NS_1/torque_cmd` | `τ_ff` |
| sub | `accel_topic` = `/NS_1/qddot_safe` | integrated into `q̇_des` and `q_des` (chapter 09) |
| command interfaces | `fr3_joint{1..7}/effort` | — |
| state interfaces | `fr3_joint{1..7}/position`, `/velocity` | — |
| service client | `service_server/set_full_collision_behavior` | only when `gazebo:=false` (L443) |
| rate | `controller_manager.update_rate`: 1000 (real or Gazebo) or 100 (fake) | `launch_support.py::_generate_torque_yaml` |

**Parameters that are generated but ignored.** The YAML generator writes `lpf_alpha`,
`tau_max_scale` and `urdf_path`, but the C++ never declares them (Verified: they appear only in a
comment at L295 of the `.cpp`). As a result:
- the launch arguments `lpf_alpha` and `tau_max_scale` have **no effect** on this controller
  (F-1-8);
- `accel_timeout` is declared with default 0.1, but the generator writes `command_timeout`, which
  is also declared. Both exist.

### 3.7 Other controllers in `franka_rt_controllers`

- `CBFTorqueController` (`cbf_torque_controller.cpp`):
  - subscribes `qddot_safe_topic = /NS_1/qddot_safe` and `q_des_state_topic = /NS_1/q_des_state`;
  - is spawned only by `minimal.launch.py use_torque_controller:=cbf`, with its YAML from
    `launch_support.generate_cbf_base_yaml`;
  - is the only consumer of `/NS_1/q_des_state`.

  Status: reachable only through that string value, so it **requires user confirmation**.
- `RtVelocityExecutorController` (in `rt_velocity_blender_controller.cpp`): the Pipeline B
  executor, §7.1.

### 3.8 Supporting nodes on the Pipeline A graph

| Node | Subscribes | Publishes | Rate | Source |
|---|---|---|---|---|
| `experiment_logger` | joint states, `/human_robot/multi_distance`, `/NS_1/tracking_qdot`, `/NS_1/qdot_cmd`, `/NS_1/torque_cmd`, `FrankaRobotState` (BE), `/cbf/per_link_distances` (BE), `/NS_1/cbf_status`, `/NS_1/iso_safety`, `/NS_1/torque_saturation`, `/NS_1/qddot_nom`, `/NS_1/qddot_safe`, optional truth topic | — (CSV under the log directory) | sampling timer `sample_rate_hz` | `nodes/experiment_logger.py` L68–309 |
| `iso_safety_monitor` (optional) | joint states (R, 1), `/cbf/per_link_distances` (BE, 1), `/NS_1/cbf_status` | `/NS_1/iso_safety`; service `/NS_1/safety_reset` (`std_srvs/Trigger`) | `qp_rate_hz` | `nodes/iso_safety_monitor.py` L160–181 |
| `iso_evidence_logger` (optional) | joint states, `/cbf/per_link_distances` (BE), several `Float64MultiArray` topics | — | `sample_rate_hz = 100` | `nodes/iso_evidence_logger.py` |
| `trajectory_overlay` | colour image + info (BE), `/NS_1/ee_desired`, `/NS_1/ee_actual` | `/NS_1/trajectory_overlay` | per image; 5 s watchdog | `nodes/trajectory_overlay_node.py` |
| `trajectory_visualization` (optional) | `ee_desired`, `ee_actual` | `/NS_1/trajectory_markers` | timer | `nodes/trajectory_visualization_node.py` |
| `finger_state_publisher` (`ros2 topic pub`) | — | `/NS_1/joint_states` with the 2 finger joints at 0 | 10 Hz | torque launch, for MoveIt |
| `world_to_robot_base_tf` | — | static `world → base` | latched | torque launch |
| `camera_extrinsics_tf` (camera on) | — | static `base → camera_link` (from `camera_link_extrinsics.yaml`) | latched | torque launch |
| `image_republisher` (`franka_simulation`) | camera | — | — | interface only |

`/NS_1/joint_states` has **two publishers** in Pipeline A: the bringup's 30 Hz republisher and
the 10 Hz finger publisher. Any consumer that expects a full arm state on every message must
handle finger-only messages. The recorded fake-hardware rate of 40.0 Hz on that topic is exactly
30 + 10 (Runtime (recorded), `rates.txt`; the explanation is Derived).

## 4. The topic table for Pipeline A

| Topic | Type | Producer(s) | Consumer(s) | QoS (pub) |
|---|---|---|---|---|
| `/camera/camera/depth/image_rect_raw` | Image | RealSense, or `depth_sim_adapter` (sim) | `real_time_distance` | driver's |
| `/cbf/per_link_distances` | MultiLinkDistance | `real_time_distance` / `emulated_perception` | `cbf_safety_filter`, `rl_policy_commander`, `experiment_logger`, `iso_*`, `episode_supervisor`; also `cbf_velocity_filter` (Pipeline B) | BE, 1 |
| `/NS_1/franka/joint_states` | JointState | `joint_state_broadcaster` | filter, `qddot_to_torque`, commander, `iso_*`; simulation nodes | (JSB) |
| `/NS_1/joint_states` | JointState | 30 Hz republisher + 10 Hz finger publisher | logger, commander (effort), `move_group` | R |
| `/NS_1/qddot_nom` | Float64MultiArray[7] | commander / `rl_policy_commander` / `scenario_commander` | `cbf_safety_filter`, logger | R, 10 |
| `/NS_1/qddot_safe` | Float64MultiArray[7] | `cbf_safety_filter` | `qddot_to_torque`, `rt_torque_controller`, logger | R, 10 |
| `/NS_1/torque_cmd` | Float64MultiArray[7] | `qddot_to_torque` | `rt_torque_controller`, logger | R, 10 |
| `/NS_1/cbf_status` | Float64MultiArray[N] | `cbf_safety_filter` | commander, `rl_policy_commander`, `iso_safety_monitor`, logger, `frame_grabber` | R, 10 |
| `/NS_1/iso_safety` | Float64MultiArray | `iso_safety_monitor` | `cbf_safety_filter`, commander, logger, evidence logger | R, 10 |
| `/NS_1/ee_desired`, `/NS_1/ee_actual` | PointStamped | commander / `scenario_commander` | overlays, `gz_trajectory_markers` | R, 1 |
| `/NS_1/q_des_state` | JointState | commander | (`CBFTorqueController` only) | R, 10 |
| `/NS_1/torque_saturation` | Float64MultiArray | `qddot_to_torque` | logger, evidence logger | R, 10 |
| `/human_robot/multi_distance` | MultiDistance | `real_time_distance` | logger | R, 10 |
| `/human_robot/distance` | HumanRobotDistance | `real_time_distance` | — | R, 10 |
| `/NS_1/rl_status` | Float64MultiArray | `rl_policy_commander` | — | R, 10 |

## 5. Coordinate frames and the TF tree

Two static transforms in the tree come from `torque_control_stack.launch.py`:
- `world → base` is identity, from node `world_to_robot_base_tf` (`_launch_all`).
- `base → camera_link` comes from `camera_link_extrinsics.yaml` (translation `[1.993, 0.032,
  1.080]` m), via node `camera_extrinsics_tf`, only with `enable_camera`.

```
world ──(static, identity; torque launch)──▶ base ──(static, identity; robot_state_publisher)──▶ fr3_link0 ──▶ fr3_link1 … fr3_link8 ──▶ fr3_hand ──▶ fr3_hand_tcp
                                               │
                                               └──(static; torque launch, camera on)──▶ camera_link ──(RealSense driver; Inferred)──▶ camera_*_frame / *_optical_frame
```

**Runtime (recorded).** `fake_hw/tf.json` contains `base` with parent `world` and `fr3_link0`
with parent `base`, both static identity. That matches the torque launch's comment: publishing
`world → fr3_link0` would give `fr3_link0` two parents.

**Frames used by the computation** (Static):
- **Robot base frame.** `fr3_link0` (`fr3_complete.robot.base_frame`) is the frame of every
  distance, every control point, and `ee_desired`/`ee_actual`. It is identical to `base`.
- **Camera.** `real_time_distance` takes `base → camera_color_optical_frame` from
  `camera_extrinsics.yaml` (translation `[1.754, 0.154, 1.059]` m, quaternion in `x, y, z, w`
  order) and does **not** take it from TF. So the TF-published `base → camera_link`
  (`camera_link_extrinsics.yaml`) and the file used for projection are **two independent
  calibrations of the same camera**. Nothing checks that they agree. The torque launch comment
  says this split is intentional (Q-1-3).

**Inconsistent TF set-ups in other launch files** (Static; the runtime effect is Inferred):

| Launch file | What it publishes | Problem |
|---|---|---|
| `minimal.launch.py`, `thales.launch.py` | `world → fr3_link0` (L158–167 / L150–159) | Exactly the two-parents situation the torque launch comment warns about (F-1-7) |
| `minimal.launch.py`, `thales.launch.py`, `velocity_cbf_control_stack.launch.py` | static `base → camera_color_optical_frame` (from `camera_extrinsics.yaml`) | If the RealSense driver also publishes its own `camera_link → … → camera_color_optical_frame` chain, which is its default behaviour (Inferred), that frame gets two parents (F-1-7) |

## 6. Rates: configured vs. observed

| Signal | Configured (Static) | Recorded on fake hardware, `ac528f1`, 100 Hz controller (Runtime) | Recorded in sim (Runtime) |
|---|---|---|---|
| `/NS_1/franka/joint_states` | `update_rate` 1000 (real), 100 (fake) | 100.0 Hz | — |
| `/NS_1/joint_states` | 30 Hz + 10 Hz finger publisher | 40.0 Hz | — |
| `/NS_1/qddot_nom` | 100 Hz | 99.99 Hz | — |
| `/NS_1/qddot_safe`, `/NS_1/cbf_status` | `qp_rate_hz` = 300 | 299.99 / 299.97 Hz | 99.2 Hz QP (target 100) in `ssb_poke__minimal_l1_ssb__gz__s3` |
| `/NS_1/torque_cmd` | = QP rate (event-driven) | 300.1 Hz | — |
| Constraint geometry rebuild | `cbf_update_rate_hz` = 50 | not in `rates.txt` | **35.1 Hz** "rows rate" in the same sim cell |
| `/cbf/per_link_distances` | camera rate (90 Hz) | not present (no camera on fake hardware) | — |

How to read this table:
- The fake-hardware capture shows that the configured rates are reached **when perception is
  idle** and the controller runs at 100 Hz.
- The simulation cell shows that the 50 Hz geometry rebuild **can fall below its target** under a
  full perception load.
- A project note (not verified in this stage) reports `qddot_safe` dropping to about 18 Hz in one
  Gazebo configuration.

The rates of the real robot with the live camera were not measured in any artefact consulted
here. **Unverified.**

## 7. The other graphs

### 7.1 Pipeline B (velocity)

| Node | Subscribes | Publishes | Rate | Source |
|---|---|---|---|---|
| `ee_pentagon_velocity_commander` | joint states (through `kinematics.JointStateManager`, auto-detected) | `/NS_1/tracking_qdot` (`resolve_tracking_topic()`: the namespace from `franka.config.yaml` + `tracking_qdot`) | 200 Hz | `nodes/ee_pentagon_velocity_commander.py` |
| `cbf_velocity_filter` | `/NS_1/joint_states` (`topics.joint_states_topic`), `/cbf/per_link_distances` (BE), `/NS_1/tracking_qdot` (`topics.qdot_nom`) | `/NS_1/qdot_cmd` (`topics.velocity_topic`) | `control_rate_hz = 100` | `nodes/cbf_velocity_filter.py` L66–198 |
| `rt_velocity_executor_controller` | `command_topic = qdot_cmd`, so `/NS_1/qdot_cmd` | `fr3_joint*/velocity` | 1 kHz | `rt_velocity_blender_controller.cpp` L191–327 |

`cbf_velocity_filter` loads `fr3_distance.yaml` and `fr3_control.yaml`, using
`load_robot_config('distance' | 'control')`, which resolves to `share/config/fr3_<name>.yaml`.

### 7.2 Pipeline C (OSCBF)

- `pentagon_torque_commander` publishes `/NS_1/torque_cmd` when `cbf_mode:=false`, or
  `/NS_1/qddot_nom` when `cbf_mode:=true`.
- `cbf_oscbf_filter`:
  - subscribes to joint states, to `tau_nom_topic = /NS_1/torque_cmd`, and, only if
    `enable_obstacle_cbf`, to `'/cbf/per_link_distances'` with **reliable depth 10** (L733–735;
    QoS-incompatible with the BE publisher, F-1-6);
  - publishes `tau_safe_topic = /NS_1/torque_safe` at `control_rate_hz = 100`;
  - takes its parameters from `load_params` (node defaults, L576–632). The commander and the
    filter both claim the name `torque_cmd`, which makes them mutually exclusive with
    `qddot_to_torque`.

### 7.3 Simulation-only graph (`gz_cbf_stack.launch.py`)

| Node | Subscribes | Publishes | Rate | Source |
|---|---|---|---|---|
| `gz_server` + `clock_bridge` | — | `/clock` | sim | `gz_fr3_plant.launch.py` |
| `robot_state_publisher`, `joint_state_publisher` (in `NS_1`) | `franka/joint_states` | `/tf`, `/tf_static`, `/NS_1/joint_states` (30 Hz) | — | same |
| `d455_bridge` + `depth_sim_adapter` (with `perception:=true`) | `/d455/depth/image` (gz) → `/sim/d455/depth/*` | the RealSense-equivalent `/camera/camera/depth/*` (Inferred from the plant docstring; verified in Stage 6) | 90 Hz | same |
| `scenario_bridge` | gz | `/sim/episode_start` (ROS→gz), `/sim/truth/obstacle_poses[_vel]` (gz→ROS) | — | same |
| `episode_supervisor` | `ListControllers` client, readiness topics, `/cbf/per_link_distances` (BE) | `/sim/episode` (R), `/sim/episode_start` | 50 Hz | `scripts/episode_supervisor.py` |
| `ground_truth` | `/sim/truth/obstacle_poses[_vel]`, `/NS_1/franka/joint_states` | `/sim/truth/obstacles`, `/sim/truth/cp_gaps` | 100 Hz | `scripts/ground_truth_node.py` |
| `reflex_monitor` | `/NS_1/franka/joint_states`, `/clock` | `/sim/reflex` | 10 Hz | `scripts/reflex_monitor.py` |
| `scenario_commander` (`motion:=scenario`) | `/NS_1/franka/joint_states`, `/sim/episode` | `/NS_1/qddot_nom`, `/NS_1/ee_desired`, `/NS_1/ee_actual` | timer | `scripts/scenario_commander.py` |
| `emulated_perception` (`perception:=emulated`) | joint states, truth poses | `/cbf/per_link_distances` (BE, 1) | `fps` (90) | `scripts/emulated_perception.py` |
| `sim_bag` | the 19 `BAG_TOPICS` | — | — | `gz_cbf_stack.launch.py` |

Every node runs with `use_sim_time:=true` (`SetParameter` at the top of the include).

### 7.4 Calibration graphs

These are summarised here and detailed in chapter 05:
- `apriltag_ros` detects tags in images, and the calibration nodes consume the detections and TF;
- `handeye_calibration_node` publishes a static TF and writes YAML;
- the eye-in-hand variant adds `image_rectifier_node` and `tag_overlay_node`.

## 8. Practical example — "where does `q̈_safe` go, and who notices if it stops?"

`/NS_1/qddot_safe` has **two control consumers**, a fact that is easy to miss:
1. `qddot_to_torque` turns each message into `τ_ff` on `/NS_1/torque_cmd`.
2. `rt_torque_controller` also subscribes to it directly, as `accel_topic`, to integrate its own
   reference velocity and position.

Suppose `cbf_safety_filter` dies:
- `qddot_to_torque` stops publishing, so `τ_ff` goes stale.
- The controller reads a stale `q̈_safe` too.
- The controller's own freshness checks then decide what happens: `command_timeout` and
  `accel_timeout`, both 0.1 s. According to the generator's comments, a stale `τ_ff` leads to a
  **position hold on the current pose**.

The graph has no other watchdog. In particular the commander keeps publishing `q̈_nom`, which
nobody consumes. Chapter 09 verifies the hold law. This example shows why the controller's
timeouts are part of the safety architecture and not a detail.

## 9. Consequences of modification

- **Changing the QoS of `/cbf/per_link_distances`.** Its eight or so subscribers are BE. Making
  the publisher RELIABLE keeps them working, because a BE subscriber accepts either. Making any
  subscriber RELIABLE breaks that subscriber **silently** (§2.3).
- **Renaming the namespace.** This needs a coordinated change in `franka.config.yaml`,
  `fr3_control.yaml → topics`, `launch_support.py` (`accel_topic`), node defaults, the simulation
  scripts, and `gz_fr3_plant.launch.py`.
- **Adding a second consumer to `/NS_1/joint_states`.** Remember the finger-only messages (§3.8).
- **Changing the filter's executor (`executor_mode`, R1, 2026-11-12).** The callbacks are
  unchanged, but the order and timing in which they see their inputs are not. With `single`, a
  tick that falls due during a rebuild runs after it, before the joint-state and `/clock` messages
  that arrived meanwhile: rclpy yields ready timers before subscriptions (`executors.py`
  L712–728). Measure input ages and the tick tail, not only the mean rate
  ([refactoring/R1_EXECUTOR_CONFIGURATION.md](refactoring/R1_EXECUTOR_CONFIGURATION.md)).
- **Changing `qp_rate_hz`:**
  - it changes the rate of `torque_cmd` and `cbf_status`;
  - it changes the effective torque-rate limit in `qddot_to_torque`, through the rescale
    `100 / qp_rate_hz`;
  - it changes the ISO monitor's tick.

  These coupled effects are why simulation conditions pin it explicitly.
- **Using the camera TF instead of the file.** Projecting through TF instead of
  `camera_extrinsics.yaml` would change which calibration perception uses (§5).

## 10. Findings and open questions from this chapter

Logged in [PROGRESS.md](PROGRESS.md):
- F-1-5: `start_move_group` and the comment claiming MoveIt use;
- F-1-6: the OSCBF QoS mismatch;
- F-1-7: TF parents in `minimal`, `thales` and the velocity stack;
- F-1-8: ignored controller parameters;
- F-1-9: topics without consumers (`/human_robot/distance`, `/NS_1/rl_status`, and
  `/NS_1/q_des_state` outside `cbf` mode);
- Q-1-2: the depth optical frame against the colour-frame extrinsics;
- Q-1-3: the two camera calibrations.
