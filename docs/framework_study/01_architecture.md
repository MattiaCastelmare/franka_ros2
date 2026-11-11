# 01 — Architecture: what runs, and how the pieces fit

> **Verified against** HEAD `ba68077` (branch `humble-mattia`), 2026-11-11.
> **Evidence base:** static reading of the launch files, the node constructors, the controller
> generator and the C++ controller interfaces. Nothing in this chapter was executed. Runtime
> statements are labelled **Runtime (recorded)** when they come from an existing simulation
> result file, and **Unverified** otherwise.
> **Companion chapters:** [02](02_ros_graph_and_dataflow.md) gives every topic, rate and QoS;
> [03](03_configuration.md) shows where every number comes from.

---

## 1. Intuition — what problem does this framework solve?

A Franka FR3 arm follows a task, such as tracing a circle with its hand, while an obstacle that
nobody scripted (a hand, a thrown ball, a box) moves through its workspace. A fixed depth camera
watches the scene. The robot must keep doing its task when it can, and must **never** let any part
of the arm get closer to the obstacle than a safety distance `d_safe`.

The framework splits this into two jobs that are deliberately kept apart:

1. **A nominal controller** decides what the arm *would like* to do in order to make progress on the
   task. It knows nothing about obstacles. Example: `pentagon_qddot_commander`.
2. **A safety filter** takes that wish and returns the *closest admissible* command, the one that
   keeps every safety condition true. Example: `cbf_safety_filter`.

This separation is the **safety-filter pattern**: the task can be anything (an analytic path, an
RL policy, a scripted scenario) and the safety guarantee lives in one place. The guarantee is
expressed with **Control Barrier Functions (CBFs)**. A CBF is a function `h(q, …)` that is
positive exactly when the robot is safe; the filter only permits commands that keep `h` from
crossing zero (chapter 07).

Two further jobs complete the loop:

3. **Perception and distance**: turn depth images into "how far is each part of the arm from the
   nearest obstacle point, and in which direction" (`real_time_distance`, chapter 05).
4. **Actuation**: turn the filtered command into joint torques that the robot (or Gazebo) executes
   at 1 kHz (`qddot_to_torque` plus the C++ `rt_torque_controller`, chapter 09).

Why the main pipeline works with **accelerations** (`q̈`) rather than velocities:
- A distance barrier `h = d − d_safe` depends on the configuration `q` only. Its first time
  derivative contains `q̇`, and only its **second** derivative contains `q̈`.
- So if the controlled input is `q̈`, the barrier has relative degree 2. That needs a
  *higher-order* CBF (HOCBF), which is what this framework implements (chapter 07).
- Commanding `q̈` and then converting it to torque through the robot model is also how the
  framework reaches the torque interface of the FR3.

## 2. Mathematical skeleton of the main pipeline

This section defines the mathematical role of each block, so the architecture can be read as
mathematics. Every block is derived in full in its own chapter; here we fix the notation and the
direction of the information flow.

**Notation.**
- `q ∈ ℝ⁷` [rad] and `q̇ ∈ ℝ⁷` [rad/s] are the joint positions and velocities.
- `q̈ ∈ ℝ⁷` [rad/s²] are joint accelerations.
- `τ ∈ ℝ⁷` [N·m] are joint torques.
- `p_i(q) ∈ ℝ³` [m] is the position of control point *i* on the arm, expressed in the robot base
  frame `fr3_link0` (identical to `base`, chapter 02 §5).
- `J_i(q) = ∂p_i/∂q` is its 3×7 translational Jacobian.
- `d_i` [m] is the measured distance from control point *i* to the nearest obstacle point, and
  `n̂_i` is the unit direction from the obstacle towards the arm.

**Block 1 — nominal motion** (chapter 06). A map from the measured state and time to a desired
acceleration:

    q̈_nom(t) = π(q, q̇, t)                                   [rad/s²]

Here `π` is a Cartesian PD law on a path `P(s(t))` with a joint-space null-space term
(`pentagon_qddot_commander`), or a neural policy (`rl_policy_commander`), or a scripted law
(`scenario_commander`, simulation only).

**Block 2 — safety filter** (chapters 07–08). In its simplest form, the "minimal HOCBF" that the
simulation conditions call `minimal`:

    q̈_safe = argmin_u  ‖u − q̈_nom‖²
             s.t.  ḧ_i(u) + k₁·ḣ_i + k₀·h_i ≥ 0     for every obstacle row i
                   (joint, velocity, slew, self-collision, … rows)

with `h_i = d_i − d_safe` and `ḣ_i = n̂_iᵀ (J_i q̇ − v_obs,i)`. In this notation the obstacle row
reads:

    ḧ_i = n̂_iᵀ (J_i u + J̇_i q̇) + (curvature and obstacle-acceleration terms)

So the constraint is affine in `u`, and the problem is a quadratic program (QP).

Shipped gains (`fr3_control.yaml`): `k₀ = 25.0 s⁻²`, `k₁ = 10.5 s⁻¹`, `d_safe = 0.10 m`.

The shipped "full" configuration adds many terms on top of this core (margins, gates, biases,
caps). Chapters 07–08 separate the core from the additions.

**Block 3 — inverse dynamics** (chapter 09). Verified in
`nodes/qddot_to_torque.py` (`data.nle − data.g`, around L299–302):

    τ_ff = M(q)·q̈_safe + C(q, q̇)·q̇                          [N·m]

**Gravity `g(q)` is deliberately absent.** On the real FR3 the Franka control box adds gravity
compensation itself. This is stated in `test/launch/test_torque_fake.launch.py` and is consistent
with `rt_torque_controller.cpp`, which contains no gravity term. The docstring of
`torque_control_stack.launch.py`, which says that `rt_torque_controller` "adds g(q)", is therefore
misleading (finding F-1-3). Who supplies gravity in Gazebo is an open question for chapters 09/12.

**Block 4 — 1 kHz joint controller** (chapter 09). From the controller-YAML generator comments
in `utils/launch_support.py::_generate_torque_yaml` and the parameters the C++ declares:

    τ = τ_ff + K_p·(q_des − q) + K_d·(q̇_des − q̇)

where `q_des` and `q̇_des` integrate `q̈_safe`. The applied correction is capped (`p_max`, `e_max`),
and a stale `τ_ff` falls back to holding the current position (`command_timeout`). The exact
discrete law is derived in chapter 09; at this stage only its *interface* is verified.

**Multi-rate structure.** Each block runs at its own rate, so the loop is a multi-rate sampled
system. The shipped hardware rates come from static configuration (chapter 02 §6):

| Block | Trigger | Shipped rate |
|---|---|---|
| Depth camera | driver | 90 Hz (`camera_depth_profile: 848x480x90`) |
| `real_time_distance` | each depth frame | ≤ 90 Hz |
| `cbf_safety_filter`, geometry rebuild | timer | 50 Hz (`cbf_update_rate_hz`) |
| `cbf_safety_filter`, QP | timer | 300 Hz (`qp_rate_hz`) on hardware. In simulation, 19 of 23 conditions pin it to 100 Hz through `cbf_params` (all except `residual`, `kf`, `imm`, `imm+pred`). |
| Commander | timer | 100 Hz (`rate_hz` default; `control_rate_hz` for RL) |
| `qddot_to_torque` | each `q̈_safe` message | = QP rate |
| `rt_torque_controller` | controller_manager | 1000 Hz (real or Gazebo), 100 Hz (fake hardware) |

## 3. Implementation — the two packages and their roles

### 3.1 `franka_experiments` (ament_python): the framework itself

| Layer | Contents (`franka_experiments/franka_experiments/…`) | Chapter |
|---|---|---|
| Nodes (`nodes/`) | 23 node modules; 22 are installed as console scripts (`setup.py`). `distance_visualization_node.py` has no entry point. | all |
| Library (`utils/`) | 59 modules: kinematics, perception, tracking, CBF rows, QP assembly, governors, ISO, config/launch helpers | 03–11 |
| Launch (`launch/`) | 7 launch files: 3 control stacks, 1 bringup-only, 1 replay, 2 calibration | 01, 02 |
| Config (`config/`) | the YAMLs that hold every tunable number (chapter 03) | 03 |
| Scripts (`scripts/`) | 27 offline and diagnostic tools, plus `pin_rt_thread.sh` | 13 |
| Tests (`test/`) | about 100 pytest modules plus fake-hardware launch tests | 14 |

### 3.2 `franka_simulation_experiments` (ament_cmake): the Gazebo experiment harness

It contains **no control code of its own.** Instead it:
- starts the Gazebo plant (Ignition Fortress, FR3, `ros2_control`) through
  `franka_simulation/launch/gz_fr3_plant.launch.py`;
- **includes `franka_experiments/launch/torque_control_stack.launch.py` unchanged**, with
  `external_plant:=true` so the hardware bringup is skipped;
- adds simulation-only nodes: ground truth, a reflex monitor that emulates the FR3 firmware trip
  limits, an episode supervisor, a scripted-motion commander, and an emulated-perception
  alternative;
- defines the experiment axes as data (`config/conditions.yaml` × `config/scenarios.yaml` ×
  `seed` × `perception` × `scene`), and runs and scores batches (`scripts/simexp/`).

This "include unchanged" design is the most important architectural fact about the simulation. A
simulation run exercises **the same launch composition and the same node parameters** as a
hardware run. The differences are injected as launch arguments, and the run manifest records each
one (`gz_cbf_stack.launch.py::_launch`, the `stack_args.update({...})` block).

### 3.3 Dependency direction

```
franka_simulation_experiments ──(Python import + launch include)──▶ franka_experiments
            │                                                           │
            ├──launch include──▶ franka_simulation (gz_fr3_plant)       ├──spawns──▶ franka_rt_controllers (C++)
            └──sys.path──▶ franka_simulation/scripts/utils              └──include──▶ franka_bringup (hardware)
```

`franka_experiments` never imports the simulation package. Details and the hidden `sys.path`
couplings are in [ledger/dependencies.md](ledger/dependencies.md).

## 4. The control pipelines

The repository contains four distinct command chains. Two different things decide which one runs:
the launch file chosen, and the launch arguments given to it.

### 4.1 Pipeline A: acceleration-space HOCBF, torque execution (the main pipeline)

```
            depth camera (RealSense D455, or Gazebo + depth_sim_adapter)
                 │ /camera/camera/depth/image_rect_raw  (+ camera_info)
                 ▼
         real_time_distance ──/cbf/per_link_distances (MultiLinkDistance, BEST_EFFORT)──┐
                                                                                        ▼
 commander ──/NS_1/qddot_nom──▶ cbf_safety_filter ──/NS_1/qddot_safe──▶ qddot_to_torque ──/NS_1/torque_cmd──┐
 (pentagon_qddot_commander |        │    ▲                     │                                            ▼
  rl_policy_commander |             │    └ /NS_1/franka/joint_states (1 kHz)                rt_torque_controller (1 kHz)
  scenario_commander)               └──/NS_1/cbf_status──▶ commander (phase governor)          ▲  reads τ_ff AND q̈_safe
                                                                                    /NS_1/qddot_safe ┘
                                                                                                    ▼
                                                                                  FR3 effort interfaces (or Gazebo)
```

- **Launch files:** `franka_experiments/launch/torque_control_stack.launch.py` on hardware or fake
  hardware; `franka_simulation_experiments/launch/gz_cbf_stack.launch.py` in Gazebo, which
  includes the former; `gz_circle_demo.launch.py`, which runs this chain *without* the filter.
- **Nodes:** listed in the table below. Optional additions: `iso_safety_monitor`,
  `iso_evidence_logger`, `trajectory_overlay_node`, `trajectory_visualization_node`, `move_group`.

| Role | Node | Source |
|---|---|---|
| Perception and distance | `real_time_distance` | `nodes/real_time_distance.py` |
| Commander | `pentagon_qddot_commander`, or `rl_policy_commander` (`motion_source:=rl`) | `nodes/pentagon_qddot_commander.py`, `nodes/rl_policy_commander.py` |
| Safety filter | `cbf_safety_filter` | `nodes/cbf_safety_filter.py` |
| Inverse dynamics | `qddot_to_torque` | `nodes/qddot_to_torque.py` |
| Joint controller | `rt_torque_controller` | `franka_rt_controllers/src/rt_torque_controller.cpp` |
| Logging | `experiment_logger` | `nodes/experiment_logger.py` |

- **Selection logic** (Verified, `torque_control_stack.launch.py::_launch_all`):
  - `motion_source ∈ {pentagon, rl, none}` picks the commander. `none` means the caller
    publishes `q̈_nom`; the simulation uses it for `scenario_commander`.
  - `external_plant:=true` drops the `franka_bringup` include. The spawner for
    `rt_torque_controller` still runs, against whichever controller manager provides `/NS_1`.
  - Most perception and CBF mechanisms are switched by launch flags whose defaults live in
    `config/launch_defaults.yaml` (chapter 03).
- **The commander's two output modes.** `pentagon_qddot_commander` publishes on its parameter
  `qddot_safe_topic`, which defaults to `topics.qddot_nom` (`/NS_1/qddot_nom`). Pointed at
  `/NS_1/qddot_safe`, it bypasses the filter entirely. That is how `gz_circle_demo.launch.py`
  wires it (Verified, docstring and node L104–105).

### 4.2 Pipeline B: velocity-space CBF, velocity execution

```
ee_pentagon_velocity_commander ──/NS_1/tracking_qdot──▶ cbf_velocity_filter ──/NS_1/qdot_cmd──▶ rt_velocity_executor_controller
                                                              ▲   (bypass_cbf:=true → pass-through)        (RtVelocityExecutorController,
                                     /cbf/per_link_distances ─┘                                              rt_velocity_blender_controller.cpp)
```

- **Launch files:**
  - `velocity_cbf_control_stack.launch.py`. ~~Phase 1, `bypass_cbf:=true`, is the default and is a
    pure pass-through. Phase 2 is `bypass_cbf:=false`.~~ **Corrected in Stage 5 (F-5-1):** the
    launch argument is declared with `default_value='false'` (L434–436), so **Phase 2, the CBF QP,
    is the default**. `bypass_cbf:=true` gives the pass-through. The launch docstring (L19) says the
    opposite, and the superseded sentence above repeated it. See 10 §2.1.
  - `minimal.launch.py` and `thales.launch.py` start the **velocity executor controller and
    perception only**. They start neither a commander nor a filter, so those must be started by
    hand.
- **Rates:**
  - `cbf_velocity_filter`: `control_rate_hz = 100` from `fr3_control.yaml`.
  - Commander: `rate_hz = 200` (node default).
  - Executor: 1 kHz on hardware.
- **Note:** the docstring of `velocity_cbf_control_stack.launch.py` says the filter reads
  `/human_robot/multi_distance`. The code subscribes to `topics.per_link_distances`
  (`MultiLinkDistance`) (`cbf_velocity_filter.py` L183). The docstring is stale (F-1-4).

### 4.3 Pipeline C: torque-space Dynamic-OSCBF

```
pentagon_torque_commander (cbf_mode:=false) ──/NS_1/torque_cmd──▶ cbf_oscbf_filter ──/NS_1/torque_safe──▶ ?
```

- **Launch files:** no production launch file starts either node. `pentagon_torque_commander` is
  started only by `test/launch/test_torque_fake.launch.py`, and that test launch wires it
  **directly** to `rt_torque_controller` on `/NS_1/torque_cmd`, without the OSCBF filter.
- **Missing consumer:** the filter publishes `/NS_1/torque_safe`, and no shipped launch
  configures a controller on that topic. `rt_torque_controller` reads `torque_command_topic`,
  default `torque_cmd`. The launch argument `torque_command_topic:=torque_safe` *could* close the
  loop, but no file in the repository does so.
- **Obstacle input:** the filter subscribes to `/cbf/per_link_distances` with *reliable* QoS
  (depth 10). Every publisher of that topic offers BEST_EFFORT. Under the ROS 2 QoS
  compatibility rules a RELIABLE subscriber does not match a BEST_EFFORT publisher, so the filter
  would receive no distances (**Inferred** from the QoS rules, **Unverified** at runtime; F-1-6).
  This path is also gated behind `enable_obstacle_cbf`, whose default is `False`
  (`cbf_OSCBF_filter.py::load_params`).
- **Status:** see §6. Its math is in chapter 10.

### 4.4 Pipeline D: no filter (bring-up, demonstration, diagnostics)

- `gz_circle_demo.launch.py`: commander → `/NS_1/qddot_safe` → `qddot_to_torque` →
  `rt_torque_controller`.
- `test_torque_fake.launch.py`: `pentagon_torque_commander` → `/NS_1/torque_cmd` →
  `rt_torque_controller`.
- `torque_control_stack.launch.py isolation_test:=true`: a joint-by-joint step-response
  diagnostic in the commander, which terminates itself after about 14 s.

### 4.5 Auxiliary, non-control graphs

- **Calibration:** `handeye_calibration_bringup.launch.py` (AprilTag + `handeye_calibration_node`)
  and `handeye_eye_in_hand_calibration.launch.py` (D405 wrist camera, `image_rectifier_node`,
  `tag_overlay_node`, `handeye_eye_in_hand_node`). Both include `minimal.launch.py`. They produce
  the extrinsics YAMLs that perception consumes (chapter 05).
- **Offline replay:** `bag_replay.launch.py` replays recorded depth, TF and joint states and
  `/NS_1/qddot_nom` through `real_time_distance` and `cbf_safety_filter`, in **open loop**. The
  arm never moves out of the way; a replay measures when and how hard the filter acts.

## 5. Integration — how a simulation run reuses the hardware stack

`gz_cbf_stack.launch.py::_launch` (Verified) performs these steps:

1. **Resolve the experiment axes.**
   - `simexp.conditions.resolve(condition, --set)` produces the launch arguments
     (`stack_args`), the perception overrides, and a flat record of the toggles.
   - `scenario_catalog.resolve(scenario, seed)` produces the obstacles, `q0`, the nominal law and
     the duration `T`.
   - `enforce_start_clearance` makes the start state lie outside the barrier.
2. **Write the run directory.** It holds the world SDF (from scene, camera extrinsics and depth
   intrinsics), the controller YAML (`sim_config.rt_controllers_yaml`, which calls
   `franka_experiments`' own generator with `is_real=True, gazebo='true'`, so the rate is 1000 Hz
   and the hardware gains are used), a perception YAML (a copy of `fr3_complete.yaml` with the
   condition merged in), and `run_manifest_sim.json`.
3. **Set `use_sim_time:=true` globally** with `SetParameter`, and include
   `gz_fr3_plant.launch.py`. The plant provides `/NS_1/controller_manager`,
   `robot_state_publisher`, the 30 Hz `/NS_1/joint_states` republisher, `joint_state_broadcaster`
   on `/NS_1/franka/joint_states`, `/clock`, and optionally the D455 depth bridge plus
   `depth_sim_adapter`.
4. **Include `torque_control_stack.launch.py`** with these overrides:
   - `external_plant:=true`, `enable_camera:=false`, `rt_pin_cpu:=''`, `start_move_group:=false`;
   - `motion_source` mapped from `motion` (`scenario` becomes `none`);
   - `robot_config_yaml` set to the run's perception YAML, and `camera_extrinsics_yaml` set to the
     scene's extrinsics.
5. **Add the simulation-only nodes** after 3 s: `reflex_monitor`, `episode_supervisor`, and
   optionally `scenario_commander`, `emulated_perception`, `ground_truth`.
6. **Record a bag** of the non-image topics, and **shut down** when `episode_supervisor` exits.

The `perception` argument selects one of three columns:

| `perception:=` | Source of the distance rows |
|---|---|
| `true` | The simulated camera chain: Gazebo depth → `depth_sim_adapter` → `real_time_distance` |
| `emulated` | `emulated_perception`, which computes rows from ground truth plus a noise model and replaces `real_time_distance` |
| `false` | None |

## 6. Status of each pipeline — evidence, not assumption

The categories used here and in chapter 15 are:
- **current**: a production launch file starts it with shipped defaults;
- **experimental**: reachable, but default-off or flag-gated;
- **legacy**: superseded, with evidence;
- **requires user confirmation**: the evidence is ambiguous.

| Pipeline | Evidence for | Evidence against | Classification |
|---|---|---|---|
| A (accel HOCBF → torque) | Default of the canonical hardware launch; included unchanged by every Gazebo run; most tests target it; latest commits touch it | — | **current** |
| B (velocity CBF) | Launch file with Phase 1/2 and its own controller in `franka_rt_controllers`; `minimal` and `thales` default to its controller; tests `test_cbf_velocity_filter.py` and `test_velocity_fake.launch.py` | Phase 1 (pass-through) is the default; no Gazebo integration; `experiment_logger`'s default topics are velocity-pipeline names (`tracking_qdot`, `qdot_cmd`) | **current but secondary**. Whether it is still run on hardware **requires user confirmation**. |
| C (OSCBF torque) | Entry point exists; its own math module; a numbered "Torque pipeline 2" in `setup.py` | No production launch; output topic `/NS_1/torque_safe` has no configured consumer; obstacle subscription QoS-incompatible (Inferred); `oscbf_params.yaml` is loaded by no code (chapter 03) | **experimental / dormant**. Whether it is abandoned or parked **requires user confirmation**. |
| D (no filter) | Demo and test launches | — | **current** as a diagnostic tool |
| RL commander inside A | Selected by `motion_source:=rl`; simulation `motion:=rl` | Default is `pentagon` | **experimental (flag-selected)** |

## 7. Practical example — one depth frame's journey, with numbers

Take the shipped hardware rates (§2) and suppose an obstacle approaches the forearm at
`v = 1.0 m/s`. How stale can the information reaching the motor be? We count the **sampling
delays only**; the computation times are measured by `scripts/latency_budget.py` (chapter 13).

1. **Camera.** A frame is exposed at `t₀`. At 90 Hz, the next frame is at most `1/90 = 11.1 ms`
   away.
2. **Distance.** `real_time_distance` processes the frame in its depth callback and publishes one
   `MultiLinkDistance` per frame.
3. **Geometry rebuild.** `cbf_safety_filter` caches the newest message (BEST_EFFORT, depth 1) and
   rebuilds the constraint geometry on its 50 Hz timer, which adds up to `1/50 = 20 ms` of
   sampling wait.
4. **QP.** The QP timer at 300 Hz picks up the new rows within `1/300 = 3.3 ms`.
5. **Torque conversion.** `qddot_to_torque` reacts to each `q̈_safe` message. No sampling wait.
6. **Joint controller.** `rt_torque_controller` reads the latest `τ_ff` and `q̈_safe` at 1 kHz,
   which adds up to 1 ms.

**Derived.** The worst-case sampling staleness is about 11.1 + 20 + 3.3 + 1 ≈ **35 ms**. At
1 m/s the obstacle travels about **35 mm** in that time, a third of `d_safe = 0.10 m`, before
any processing time is added. This is why the framework spends so much machinery on obstacle
**velocity** and latency compensation (chapter 07). It is also why the geometry rebuild rate
matters as much as the QP rate.

**Runtime (recorded), one simulation cell:**
`sim_runs/batches/ssb_p2/cells/ssb_poke__minimal_l1_ssb__gz__s3/cell_result.json` reports
`qp_rate_hz = 99.2` against a target of 100, but `rows_rate_hz = 35.1` against the configured
50 Hz. So in that run the geometry rebuild was the slower link. This is one cell, not a general
measurement; chapter 02 §6 discusses it.

## 8. Consequences of modification

- **Changing a topic name** in `fr3_control.yaml → topics:` changes it for every node that
  reads that map:
  - `cbf_safety_filter`, `qddot_to_torque`, `iso_*` and the commander's defaults read it;
  - the C++ controllers do not. `rt_torque_controller.accel_topic` is written by
    `launch_support.py` as the literal `/NS_1/qddot_safe`, and simulation scripts hard-code
    `/NS_1/...` names.

  The namespace `NS_1` is therefore a de-facto constant in several places (chapter 02 §2).
- **Changing `torque_control_stack.launch.py`** changes every Gazebo run, because
  `gz_cbf_stack` includes it unchanged. `test_torque_stack_external_plant_off_identical.py` pins
  the parameter list when `external_plant:=false`.
- **Adding a CBF parameter** requires a YAML key in `fr3_control.yaml` *and* a spec entry in
  `utils/config.py::CBF_PARAM_SPEC`. Otherwise the node refuses to start; see chapter 03 §4.2,
  which also covers the stale installed copy.
- **Changing the commander's output topic or mode** silently removes the safety filter from the
  loop. Pipeline D is exactly this configuration.
- **Reviving Pipeline C** requires at least:
  - a launch path;
  - a controller consuming `/NS_1/torque_safe` (for example `torque_command_topic:=torque_safe`);
  - a BEST_EFFORT subscription on `/cbf/per_link_distances`;
  - deciding whether `oscbf_params.yaml` should be read.

## 9. Findings from this chapter

These are logged in [PROGRESS.md](PROGRESS.md) §4:
- F-1-3: the torque-stack docstring's "adds g(q)" is misleading;
- F-1-4: the velocity-stack docstring names a stale distance topic;
- F-1-5: `start_move_group` defaults to `true`, but the commander uses no MoveIt service;
- F-1-6: Pipeline C has no consumer and a QoS mismatch.
