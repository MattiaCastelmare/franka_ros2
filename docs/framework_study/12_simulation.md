# 12 — Simulation: the Gazebo plant, the experiment harness and what its numbers mean

> **Source baseline:** `ba68077`. Written in Stage 6 (2026-11-11). Labels as in
> [00_INDEX.md](00_INDEX.md) §2. Line numbers refer to the baseline.
>
> **Reproducible checks.** [tools/stage6_verification/](tools/stage6_verification/README.md):
> 69 checks, all PASS on 2026-11-11 ([observed](tools/stage6_verification/results/observed_2026-11-11.txt)).
> They call the production laws, catalog, conditions, scorers, reflex checker, emulation, depth
> conversion and robot/world builders. **Nothing in this chapter comes from running Gazebo.** Every
> runtime statement is either Runtime (recorded), read from `sim_runs/` (produced by older code,
> F-6-1), or Unverified.
>
> **Companion.** [SIMULATION_VALIDATION_PROTOCOL.md](SIMULATION_VALIDATION_PROTOCOL.md) turns this
> chapter into the per-modification procedure of the refactor.
>
> **Depth of reading.**
> - **Read in full:**
>   - the two launch files of the run (`gz_cbf_stack`, `gz_fr3_plant`);
>   - the robot and world builders (`gz_robot_description`, `gz_world`);
>   - the C++ obstacle system (laws, timing, truth);
>   - every sim node (`episode_supervisor`, `ground_truth_node`, `reflex_monitor`,
>     `scenario_commander`, `emulated_perception`);
>   - the `simexp` modules `conditions`, `scenario_catalog`, `obstacle_laws`, `scenario_laws`,
>     `truth_geometry`, `sim_config`, `emulation`, `reflex_checks`, `runner`, `procs`, `batch`,
>     `closed_loop`, `analysis`, `provenance`;
>   - `depth_sim_adapter` + `depth_conversion`.
> - **Read in the parts that define a number:** `estimator_metrics`, `report`.
> - **Read at interface level** (docstring, functions, inputs and outputs): `attribution`,
>   `offline`, `graph_contract`, `control_regression`, `gz_trajectory_markers`, `pin_process`,
>   `check_self_occlusion`, `plot_control_regression`, `gz_plant`/`gz_circle_demo`, and the tests.
>   The ledger records which.

---

## 1. Intuition

The simulation exists to answer one question before a change reaches the robot: **does the
unchanged control stack still behave safely when the world moves?** It does that by replacing
exactly two things and keeping everything else.

1. **The plant.** The FR3 and its `franka_hardware` interface are replaced by Gazebo (Ignition
   Fortress) with the `franka_ign_ros2_control` plugin. The same `rt_torque_controller` runs inside
   it.
2. **The world.**
   - The D455 and the people are replaced by a rendered depth camera and by spheres whose motion
     is a closed-form law.
   - Because the law is known, the simulation also knows the **truth**: where every obstacle is,
     and how far every control point is from it.

Everything between them is the same code as on the rig (01 §3.2): `torque_control_stack.launch.py`
with commander, `cbf_safety_filter`, `qddot_to_torque` and `real_time_distance`. On top of that,
the harness adds:
- an **episode clock**, so that the obstacles and the nominal motion start together;
- a **reflex monitor**, which records where the real robot would have aborted;
- **conditions**: named sets of CBF flags, which make arms comparable;
- a **batch runner** that starts a fresh Gazebo per cell and scores it.

The harness has to keep four kinds of number apart, and the rest of this chapter is organised
around them:

| Kind | Example | Comes from |
|---|---|---|
| **True** (what physically happened in the sim) | true clearance `min_gap`, `contact` | `truth.npz` (ground_truth_node) |
| **Estimated** (what the filter believed) | `/cbf/per_link_distances`, track velocity | perception (real or emulated) |
| **Commanded** | q̈_nom, q̈_safe, τ_cmd | the stack's topics in the bag |
| **Actual** (what the plant did) | q, q̇, effort | `/NS_1/franka/joint_states` in the bag |
| **Reconstructed** | jerk, QP rate, latency, q̈ projection | computed after the run from bag receive times and log lines |

Confusing them is the main way a sim result misleads. For example, `neg_duration_s` is a *true*
quantity, but it measures time inside d_safe, not time in contact (§8.2). The "QP rate" is
*reconstructed* from what the recorder kept (§8.3).

---

## 2. The launch: from one command to a running episode (part A)

### 2.1 Entry points (Verified)

| Launch | Package | Use |
|---|---|---|
| `gz_cbf_stack.launch.py` | franka_simulation_experiments | **The** experiment run: plant + stack + sim nodes + recording; one episode, then shutdown |
| `gz_plant.launch.py` | franka_simulation_experiments | Phase 1: plant + `rt_torque_controller` YAML (+ optional spawn), no stack (read at docstring level) |
| `gz_circle_demo.launch.py` | franka_simulation_experiments | a demo (interface level only) |
| `gz_fr3_plant.launch.py` | franka_simulation | the plant itself, included by the two above |
| `run_gz_scenarios` | franka_simulation_experiments (program) | batch: one `gz_cbf_stack` per cell (§8.1) |

The arguments of `gz_cbf_stack` (`_ARGS`, L36–61) are:
- `condition` (required; its description still lists only residual/kf/imm/imm+pred, not the 23
  conditions);
- `set`, `motion` (pentagon | scenario | rl), `scenario`, `seed`;
- `perception` (true | emulated | false), `emulated_fps` (90), `scene`;
- `gui`, `duration_s`, `run_dir`, `record_bag` (true), `filter_cpu`, `rl_onnx_model`.

### 2.2 The resolution step — `gz_cbf_stack.launch.py::_launch` (Verified)

`_launch` runs inside an `OpaqueFunction`, so everything below happens in the launch process
**before any node starts**.

1. **Condition.** `simexp.conditions.resolve(condition, _parse_set(set))` gives:
   - `stack_args`: the launch arguments of `torque_control_stack`, plus `cbf_param_overrides`;
   - the perception overrides;
   - the toggles (§6).
2. **Scenario** (if one is given). `_resolve_scenario`:
   - `scenario_catalog.resolve(name, seed)` (§5);
   - then the RSP URDF built at q0, the control points from `TruthGeometry`, and d_safe from the
     installed `fr3_control.yaml`;
   - then `enforce_start_clearance` (12.21).
3. **Derived settings.**
   - `motion`: the argument, else the scenario's (`pentagon` or `scenario`).
   - `scene`: the argument, else the scenario's, else the default.
   - `duration`: `duration_s` if given, else the scenario's `T`, else 0 (= until stopped).
4. **Run directory.** `sim_config.make_run_dir` → `sim_runs/YYYYMMDD/HHMMSS_<name>`, or
   `$FRANKA_SIM_RUNS_DIR` (`sim_runs_root`).
5. **Generated files** (§2.3), and `run_manifest_sim.json` with:
   - git provenance;
   - the resolved condition and scenario;
   - the toggles and the file list.
6. **Environment for every node:**
   - `SetEnvironmentVariable FRANKA_LOGS_DIR=<run_dir>/logger`, which moves the logger's output
     out of the hardware directory;
   - `SetParameter use_sim_time=True` (global);
   - with a scenario, `truth_topic=/sim/truth/cp_gaps` and `truth_n_cp=11` for `experiment_logger`.
7. **Actions** (§2.5):
   - the plant include;
   - the `torque_control_stack` include, with `stack_args` plus the sim settings:
     - `external_plant:=true`, `use_fake_hardware:=false`;
     - `enable_camera:=false`, `start_real_time_distance:=<perception == true>`;
     - the scene's `camera_extrinsics_yaml`, `robot_config_yaml:=fr3_complete_sim.yaml`;
     - `camera_depth_profile:=848x480x90`, `rt_pin_cpu:=''`;
     - `start_move_group:=false`, `trajectory_overlay_window:=false`, `start_rosbag:=false`;
   - the sim nodes.

   The actions are deliberately **not** wrapped in a scoped `GroupAction`: the stack starts its
   nodes from `TimerAction`s that fire after a group scope has been popped, and they would lose
   `use_sim_time` and `FRANKA_LOGS_DIR` (comment at L266–269; Runtime (recorded) per that comment).

### 2.3 Files a run writes (Verified)

| File | Writer | Content |
|---|---|---|
| `rt_controllers_gazebo.yaml` | `sim_config.rt_controllers_yaml` → `launch_support.generate_rt_controllers_yaml(is_real=True, gazebo='true')` | hardware gains, 1000 Hz, `gazebo: true` |
| `world.sdf` | `sim_config.world_sdf` → `gz_world.configure_world` + `scenario_obstacles_plugin` + `obstacle_models` | the scene, camera, obstacles (§3.3) |
| `fr3_gazebo.urdf`, `fr3_rsp.urdf` | plant → `gz_robot_description.build_descriptions` | §3.1 |
| `fr3_complete_sim.yaml` | `sim_config.perception_yaml` | `fr3_complete.yaml` ⊕ `config/sim/perception_sim.yaml` ⊕ condition perception ⊕ scenario perception |
| `depth_adapter.yaml` | `sim_config.depth_adapter_yaml` | `config/sim/depth_adapter.yaml` ⊕ scenario `depth_adapter` |
| `run_manifest_sim.json` | launch, then supervisor, then reflex monitor | provenance, the resolved condition/scenario, episode times, RTF, reflex verdict |
| `logger/<stamp>/…` | `experiment_logger` (+ the commander CSV) | the run CSV, affected by F-5-21 (§8.6) |
| `truth.npz` | `ground_truth_node` | §7.1 |
| `reflex_trips.csv`, `reflex_summary.json` | `reflex_monitor` | §7.3 |
| `bag/` | `ros2 bag record --use-sim-time` | the 19 `BAG_TOPICS` (L65–70); no images, no `/rosout`, no `/NS_1/iso_safety` |
| `launch.log`, `ros_log/` | `simexp.runner` (batch only) | the CBFDIAG / START / DONE lines the scorer parses |
| `cell_result.json`, `digest.json`, `compare_vobs_sim.npz` | `batch` / `runner` / `estimator_metrics` | scores (§8) |

### 2.4 Installed, not source (Verified; F-6-3)

`sim_config.default_paths` (L45–60) builds every configuration path with
`get_package_share_directory`. These are the **installed** copies, and colcon copies `config/`
without `--symlink-install`. Every file below therefore comes from the last build, not from the
working tree:
- `fr3_control.yaml`, `fr3_complete.yaml`;
- `conditions.yaml`, `scenarios.yaml`;
- the sim overrides;
- the world template and the extrinsics.

The programs are the installed copies too.

On 2026-11-11, `check_installed.py` found:
- the installed `fr3_control.yaml`, `fr3_complete.yaml`, `conditions.yaml` (58 lines, against
  the 23-condition source) and `scenarios.yaml` **differ** from the baseline;
- the installed `fr3_control.yaml` has `d_safe: 0.15`, while the source has `0.10`, as do the 300
  recorded runs audited;
- the installed `cbf_safety_filter`, `real_time_distance` and `qddot_to_torque` differ too.

A simulation launched now would not run the documented code. This is a property of the container,
not of the code. It is the first thing the validation protocol checks (Phase 1).

### 2.5 The start-up timeline (Verified from the timers; times are launch-relative wall seconds)

| t [s] | Action | Source |
|---|---|---|
| 0 | `ign gazebo -s -r -v 1 --headless-rendering <world.sdf>` (server; GUI client only with `gui:=true`); NVIDIA EGL vendor file when present; `IGN_PARTITION` inherited or `fr3cbf_<pid>` | `gz_fr3_plant` |
| 0 | `clock_bridge` (`/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock`), `robot_state_publisher` (NS_1, RSP URDF), `joint_state_publisher` (sources `franka/joint_states` + `franka_gripper/joint_states`, 30 Hz → `/NS_1/joint_states`) | `gz_fr3_plant` L119–131 |
| 0 | `d455_bridge` + `depth_sim_adapter` (with the camera), `scenario_bridge` (`/sim/episode_start` ROS→gz Float64→Double; `/sim/truth/obstacle_poses[_vel]` gz→ROS Pose_V→PoseArray) | `gz_fr3_plant` L148–166 |
| 0 → | `torque_control_stack` with its own timers (02 §4: commander, filter, `qddot_to_torque`, `real_time_distance`, logger, the `rt_torque_controller` spawner at `control_spawner_delay_s`) | stack |
| 2 | `ros_gz_sim create -file fr3_gazebo.urdf -name fr3` (`spawn_delay_s`) | `gz_fr3_plant` L132 |
| 3 | `joint_state_broadcaster` spawner (timeout 60 s) | L136 |
| 3 | `reflex_monitor`, `episode_supervisor`, `ground_truth_node` (with a scenario), `scenario_commander` (motion `scenario`), `emulated_perception` (mode `emulated`) | `gz_cbf_stack` L222–251 |
| 4 | extra spawners (`gz_plant` with `spawn_rt_controller`), `pin_filter` (`filter_cpu`), the sim bag | L168, L254–263 |
| ≥ ready | **episode start** (12.20) | supervisor |
| t_start + T | supervisor finalises and exits → `Shutdown` | `OnProcessExit` L248–251 |

**The episode start (Verified, `episode_supervisor._ready/_tick`).** The supervisor declares the
loop *ready* when all of these hold:
- `/NS_1/qddot_nom`, `/NS_1/qddot_safe` and `/NS_1/torque_cmd` have each been received within
  the last 0.5 s, and so has `/cbf/per_link_distances` if `perception` is set;
- `ListControllers` reports `rt_torque_controller` **active**.

After `warmup_s` = 2 s of continuous readiness it announces

  t_start = t_now + start_lead_s + start_offset_s,  start_lead_s = 0.25 s, start_offset_s from the seed. (12.20)

It publishes t_start 50 times on `/sim/episode_start`. The C++ obstacle system latches the first
value it receives (§5.3). It also publishes `/sim/episode = [state, t_start, t_now]` at 50 Hz
(0 waiting, 1 running, 2 done). A lost readiness resets the warm-up. The lead exists so that the
obstacles, the scenario commander and the supervisor all switch at the same sim instant. The pentagon
commander does not listen (F-6-12).

### 2.6 Time (Verified + Inferred)

- **Sim time everywhere.** `use_sim_time` is set globally; `/clock` comes from Gazebo through the
  bridge. Every stamp, timer and episode time is sim time. The bag records with `--use-sim-time`,
  so bag *receive* times are sim times too.
- **Real-time factor.** The simulation runs as fast as the server can. The RTF is measured
  (12.19), not imposed. A slow run keeps the stack's timing in sim seconds, but the stack's
  computations still take wall time. When RTF < 1, a node that is CPU-bound in wall time looks
  faster in sim time; when RTF > 1, the opposite. The harness flags runs below 0.9. (Inferred: the
  direction of the bias. The flag itself is Verified.)
- **Physics step.** The truth period is 0.001 s sim (`truth_period`). The joint states arrive at
  1 kHz sim (the plugin's update rate; Runtime (recorded) per the node docstrings).

### 2.7 Stopping and cleaning up (Verified)

- **Normal end.** The supervisor exits after `finalise()`, and `OnProcessExit` emits `Shutdown`.
  Each node's `finally` then runs:
  - the reflex monitor writes its final summary and stamps the manifest (`reflex_final`);
  - ground truth saves `truth.npz`;
  - the bag gets `sigterm_timeout` 30 s.
- **Batch end.** `simexp.procs.Launch` starts `ros2 launch` in its own session (`setsid`) with its
  own `IGN_PARTITION`, and `stop()` sends SIGINT (20 s), SIGTERM (5 s), then SIGKILL (5 s) to the
  group. It then kills every process whose environment still carries that partition: orphaned
  servers would otherwise keep publishing `/clock` into the next run (lead in PROGRESS §7).
  `procs.leftovers` lists the survivors, and any survivor makes the cell not ok.
- **Stray output.** The supervisor moves anything that appeared in
  `franka_experiments/franka_logs` during the run into `<run_dir>/stray_logs` and records it.

### 2.8 Failure conditions of start-up (Verified from code; behaviour Unverified)

| Failure | What happens |
|---|---|
| Unknown condition or scenario, a misspelt flag, `tracking ≠ obstacle_tracking` | `resolve` raises in the launch process: nothing starts |
| A float/str parameter type the node rejects | the node raises in `declare_from_spec` → the stack is incomplete → the supervisor never becomes ready (F-6-2 is the known near miss) |
| The controller never activates | the supervisor waits forever (`duration_s` is measured from t_start); the batch cell times out (`timeout_s` 400 s) → not ok |
| Perception never publishes | same, when `perception` is on |
| The RSP/xacro build fails | the launch dies before the plant |
| A leftover `/clock` publisher on the same partition and domain | two clocks; cells time out (Runtime (recorded), memory lead) |

The supervisor has **no readiness timeout**. Only the batch runner's wall-clock timeout ends a run
that never starts.

---

## 3. The simulated robot and world (part A)

### 3.1 The robot description — `gz_robot_description.build_descriptions` (Verified)

The xacro output of `franka_description` (`process_xacro('fr3', hand=True)`, run with
`gazebo:=true`) goes through six transformations:

1. **`_insert_base`.** It renames `world_joint` to `fr3_base_joint` with parent `base`, and adds
   `world --(identity)--> base`. The `fr3_base_joint` origin is the identity too (checked), so
   **Gazebo world = `base` = `fr3_link0`**. The stack publishes `world → base` as the identity
   (`world_to_robot_base_tf`), and the perception and truth are rooted at `fr3_link0`. All frames
   agree.
2. **`_add_armature`.** Each arm joint gets a massless disk on its child link, with inertia I_a
   about the joint axis ẑ and I_a/2 across it. ARMATURE = (0.195 ×4, 0.074 ×3) kg·m². The disk
   is merged by (12.15):

   m = Σ m_i,  c = Σ m_i c_i / m,  I = Σ [I_i + m_i(‖c_i − c‖² 𝟙 − (c_i − c)(c_i − c)ᵀ)]   (12.15)

   With m_disk = 0, the mass and the COM are unchanged, so **g(q) is exactly the original**
   (checked to 1.8·10⁻¹⁵ N·m). M(q) changes: the joint's own diagonal gains I_a,j, and each
   upstream joint j gains

   ΔM_jj − I_a,j = Σ_{k>j} I_a,k (1 + (ẑ_j·ẑ_k)²)/2 ∈ [Σ_{k>j} I_a,k / 2, Σ_{k>j} I_a,k]   (12.16)

   because the transverse part turns with the link. Observed at five random q: up to +0.56 kg·m²
   on joint 1, with off-diagonal changes up to 0.46 kg·m². The docstring states this
   approximation. The armature exists for numerical stability: without it, the 1 kHz damping of
   joint 7 (Kd·T/I = 2.2) exceeds the discrete limit 2 (docstring; Runtime (recorded) there).
   **Consequence:** the sim's M(q) is not the model that `qddot_to_torque` inverts. This is the
   F-4-9 mismatch, now quantified.
3. **`_rigid_open_fingers`.** The fingers are fixed at 0.04 m, and their mass is lumped into
   `fr3_hand` with (12.15). The total mass is unchanged (18.604 kg).
4. **`_drop_finger_control`, `_set_initial_q`.** The ros2_control `initial_value` is q0.
5. **`_configure_plugin`.** It sets `franka_ign_ros2_control-system`, the extra `<parameters>`
   (the generated controller YAML), the namespace `/NS_1`, and the remap
   `joint_states:=franka/joint_states`.
6. **`_strip_world`.** It produces the RSP copy, which has no `world` link. `kdl_tip_link` must be
   `fr3_hand_tcp` (checked), so the KDL gravity chain of the controller covers the hand.

### 3.2 Joint-state topics (Verified)

| Topic | Rate (sim) | Content | Readers |
|---|---|---|---|
| `/NS_1/franka/joint_states` | 1 kHz (JSB in the plugin) | 7 arm joints, effort = τ_cmd + g(q) (reflex docstring; Runtime (recorded) Phase 1) | every safety-path node, ground truth, reflex monitor, emulated perception, scenario commander, the bag |
| `/NS_1/joint_states` | 30 Hz (`joint_state_publisher`) + the stack's 10 Hz finger publisher | arm + fingers | RSP, **`experiment_logger`** (F-5-21) |

The real FR3 publishes the arm joints in a scrambled order (memory lead, 04 §2.2). The sim nodes all
map by name.

### 3.3 The world — `gz_world` (Verified)

- **Scenes** (`sim_config.SCENES`):
  - `rig`: the lab with its tables; `bare`: floor, d455 and mount only;
  - `diag` and `diag_bare`: a different template with the "diag" furniture.

  Any other name raises. The catalog uses `bare` 68×, `diag` 14×, `rig` 10× and `diag_bare` 8×
  (check).
- **Camera.** The scene's `camera_extrinsics` gives the optical pose (R_opt, t_opt) of
  `camera_color_optical_frame` in `base`. Gazebo cameras look along the link's x, so

  R_link = R_opt · R_LINK_FROM_OPTICAL^T,  t_link = t_opt   (12.1)

  `hfov = 2·atan(W / 2f_x)`   (12.2)

  The width, f_x and the 90 Hz update rate come from the scene's D455 intrinsics. A
  `visibility_mask` hides the obstacles' halo visuals from the depth sensor.
- **Obstacles** (`sphere_model`):
  - each is a **visual-only** sphere: no `<collision>`, gravity off;
  - each is parked at `PARK_AT = (0, 0, −20)` until the episode starts;
  - each is moved kinematically by the system plugin (§5.3).
- **Consequence (F-6-7).** Gazebo's physics never sees robot–obstacle contact. The arm passes
  through a ball, and "contact" exists only in the truth computation (§7.1).

---

## 4. Simulated perception: three modes and what each one exercises (part C)

| Mode (`perception:=`) | Producer of `/cbf/per_link_distances` | Depth image | `real_time_distance` (mask, distance engine, tracker, IMM, prediction) | What the CBF sees |
|---|---|---|---|---|
| `true` | `real_time_distance` (unchanged) | Gazebo depth → `depth_sim_adapter` | **runs** with the condition's tracking/IMM/prediction | the real estimator on rendered depth |
| `emulated` | `emulated_perception` | none | **bypassed** | true gaps clamped at 0, truth velocity + noise, one track id |
| `false` | nobody | none | not started | no obstacle rows (the supervisor does not wait for rows) |

### 4.1 `true`: the full chain (Verified)

1. **Rendering.** Gazebo renders a 32FC1 depth image in metres at 90 Hz. It is bridged to
   `/sim/d455/depth/image` and its camera info.
2. **`depth_sim_adapter`** (`utils/depth_conversion`) re-encodes it as the RealSense stream:
   16UC1 millimetres on `/camera/camera/depth/image_rect_raw`, frame
   `camera_depth_optical_frame`, with the capture stamp kept. Non-finite, ≤ 0 and > 65.535 m all
   become 0 ("no return"). Three fidelity stages exist, each **off** in the shipped
   `depth_adapter.yaml`, with this order: delay, then dropout, then range noise:

   σ_z(z) = z² σ_d / (f_px b),  b = 0.095 m, σ_d = 0.20 px   (12.10)

   This is the same model that `sensor_range_uncertainty` prices (07). `wait_for_tf` holds frames
   until `base` and `fr3_link8` exist, because Gazebo renders before the robot is spawned.
3. **`real_time_distance`** runs unchanged. It uses the merged `fr3_complete_sim.yaml`; the
   `perception_sim.yaml` differences are:
   - `visualize: false`, because headless OpenCV aborts;
   - `sample_points_per_link` raised from 3000 to 12000, because the self-mask leaked arm pixels
     as obstacles at 3000 (comment, Runtime (recorded) in that comment).

**This is the only mode that tests perception.** It still does not test:
- calibration error (F-6-14): the camera is rendered from the same extrinsics file that the
  perception projects with;
- depth noise, dropout or latency, unless a scenario turns them on. `static_noisy`, `slow_noisy`,
  `ssb_poke_noisy` and `ssb_poke_dropout` set `depth_adapter` overrides; for example `range_noise`
  is on in two of them, and `dropout_p` 0.1 in `ssb_poke_dropout`.

The depth itself is noise-free and has no rig-like artefacts (no flying pixels or holes beyond
the renderer's).

### 4.2 `emulated`: perception replaced by its idealisation (Verified; F-6-8)

`emulated_perception` publishes a frame every 1/fps of sim time (fps 90 by default; 30 in the
`emulated30` grid). The frame is the one **captured** t_cam = 13 ms earlier: the newest buffered
joint state and obstacle pose stamped at or before t_cap, from 200-sample buffers. For each control
point (`simexp.emulation`):

- `distance = max(gap_i, 0)`, with gap_i from (12.11) to the nearest sphere: **penetration is
  reported as 0**;
- the normal is unit(cp − c), and the obstacle point is c + r·n̂;
- `track_id = 1` **for every obstacle**; `frames_seen` counts from each obstacle's first frame;
- the velocity is the truth (the law velocity, §5.3) plus N(0, 0.02²) per axis, or
  `cbf_scenarios.camera_velocity_noise` when `cam_noise`. The covariances are fixed: P_vv = 0.05²𝟙
  and P_pp = 0.01²𝟙;
- an obstacle still parked (z < −10 m) is not in the frame; with no CP within `max_thresh`, the
  empty heartbeat is sent.

The messages go through the real `build_cp_messages`, so the message *format* is the production
one. The *content* is an idealisation: there is no masking, field of view, occlusion, tracker
latency or identity confusion. This mode tests the **plant and the filter** against near-ideal
sensing. It is the middle column of the "three-column comparison" (offline `cbf_scenarios` /
emulated / full chain; `simexp.offline`, `report.t_three`).

### 4.3 What differs from hardware (Derived)

| Aspect | Rig | Sim `true` | Sim `emulated` |
|---|---|---|---|
| Depth source | D455 stereo | Gazebo raster, exact | none |
| Extrinsic error | real hand-eye residual (−6.1 cm noted in `perception_sim.yaml`) | none: rendered from the calibration file | none |
| Range noise | z²-law noise + artefacts | off unless the scenario enables it | none |
| Camera latency | ≈ 25.6 ms median (adapter comment) | 0 unless `delay_s` | 13 ms fixed |
| Frame loss | ≈ 16 % (adapter comment) | 0 unless `dropout_p` | 0 |
| Tracker | runs | runs | absent; identity 1 for all |
| Obstacle shape | people, arbitrary | spheres | spheres |
| Contact physics | real | none (visual-only spheres) | none |

**Rule for reading results.** A run with `perception:=emulated` or with ground-truth-like sensing
validates the filter and the plant, not perception. Only `perception:=true` runs say anything about
perception, and only about rendered, calibration-perfect depth.

---

## 5. Scenarios: what moves, when, and how often (part D)

### 5.1 The catalog — `config/scenarios.yaml` + `simexp.scenario_catalog` (Verified)

The 100 names (`all_names`) come from five sources:

| Source | Count | Motion | Nominal law | Scene | Obstacles |
|---|---|---|---|---|---|
| `catalog` (port of `cbf_scenarios.py`) | 11 | scenario | joint_pd (10), cartesian (1) | bare | 5 static, 6 moving |
| `pentagon.scenarios` | 26 | pentagon | the shipped commander | rig / diag / diag_bare | 16 static, 8 moving, 1 combined, 1 none |
| `throw_sweep` (generated) | 48 | scenario | joint_pd | bare | linear, 1 ball |
| `ballistic_throws` (generated) | 5 | scenario | joint_pd | bare | ballistic |
| `tests` | 10 | scenario | joint_pd | bare / diag_bare | 1 static, 7 moving, 1 combined, 1 none |

The only **combined** (static + moving) scenarios are `pentagon_multi_mixed` and `p4_laws`
(check). Positions come either as `offset` (from `flange_q0` = FK of Q0 =
(0.328018, 0, 0.566007) m, checked to 0.2 µm) or as an absolute `p0`.

`resolve(name, seed)` returns:
- `obstacles`: absolute specs with every law parameter carried, so that a dropped key cannot turn
  a moving ball into a static one;
- `nominal`, `T`, `scene`, `motion`;
- the perception and depth-adapter overrides;
- `start_offset_s` and `jitter_m`.

### 5.2 Motion sources and the episode clock (Verified; F-6-12)

- **`motion: scenario` → `scenario_commander`** (`simexp.scenario_laws.NominalLaw`, bit-equal to
  `cbf_scenarios.run_scenario` per its tests).
  - Before the start it **holds Q0** with the joint PD: q̈ = K_q(q0 − q) − 2√K_q q̇, K_q = 12. This
    also removes the spawn sag.
  - From t_start it runs the law on episode time t − t_start, at 300 Hz. That is its default
    `rate_hz`, which the launch does not set, even for 100 Hz conditions.
  - `joint_pd` drives to `goal_q` (default Q0).
  - `cartesian` uses KP 25, KD 10, damping λ 0.05 and null-space KN 6 (06).
- **`motion: pentagon` → the shipped `pentagon_qddot_commander`** (06).
  - **It has no subscription to `/sim/episode`** (no match in `franka_experiments`). It starts its
    path when its own node starts, so at the episode start it is at a boot-timing-dependent phase.
    The obstacles, by contrast, start exactly at t_start.
  - The scorer measures its tracking error from t_start (`pentagon_tracking(t_from)`).
- **`motion: rl`.** The RL policy commander (10), with `rl_onnx_model`.

### 5.3 Obstacle laws — `scenario_obstacles_system.cpp`, mirrored by `simexp.obstacle_laws` (Verified)

The law time is

  t = max(0, t_sim − t_start)   (12.7)

where t_sim is the time at the **end** of the step being computed: gz-sim advances `simTime` before `PreUpdate`, and the comment there says this was
measured. The laws are:

  static:  p = p0, v = 0;  linear: p = p0 + v0 t, v = v0   (12.3)

  ballistic:  p = p0 + v0 t + ½ g t²,  v = v0 + g t   (exact)   (12.4)

  oscillate:  p = p0 + â A sin(ωt + φ),  v = â A ω cos(ωt + φ),  ω = 2π/T_p   (12.5)

  wander:  p_i = p0_i + B_i sin(ω₀ r_i t + φ_i),  r = (1, φ_golden, 1 + √2),  ω₀ = 2π/T_p   (12.6)

  pursuit:  v_k = s · unit(x_target − p_k),  p_{k+1} = p_k + v_k · min(Δt, t)   (12.8)

The pursuit target is the flange: `fr3_link7` + 0.107 m along its z. Pursuit is explicit Euler with
no clamp at the target; at 1 ms it chatters by s·Δt ≤ 1 mm around a stationary target (Derived).

**Timing semantics** (Verified):
- **Before the start**, each obstacle is at `PARK_AT` (or at p0 when `park: false`), with v = 0.
- `running` is t_end ≥ t_start − 1 ns, so a start time on the step grid starts on that step, at
  p(0).
- The start time is **latched once** from `/sim/episode_start`; later messages are ignored.
- **Truth.** Every `truth_period` (1 ms) `PostUpdate` publishes:
  - the obstacle **positions** from `worldPose`, i.e. the simulated pose;
  - the **law velocities** `ob.v`, written into the `position` field of a second `Pose_V`.

  Both are stamped with the sim time. The velocity is therefore exact by construction: it is never
  a measured or differentiated quantity.

`check_obstacle_laws.py` verifies three things:
- velocity = d/dt position for every analytic law (to ≤ 10⁻⁸ m/s);
- the C++ formulas match the Python mirror (by text);
- the three copies of `WANDER_RATIOS` agree.

**Generated throws** (`_throw`).
- **Ballistic.** Release at distance d along û from the flange, aimed so that the ball reaches the
  flange at T_f = d/v:

    v0 = (x_fl − p0)/T_f − ½ g T_f   ⇒   p(T_f) = x_fl   (12.9)

  Checked to 10⁻¹⁶ m. The episode lasts T = min(T_f + 0.5, 6) s.
- **Linear sweeps.** `throw_a/b/c` are linear along −y at speed v from d0, with
  T = min(d0/v + 0.5, 6) s.

### 5.4 Seeds and repetition (Verified; F-6-9)

`resolve` uses `rng = default_rng(seed)`. For **seed 0**, `start_offset_s = 0` and there is no
jitter: this is the 1:1 port. For **seed k > 0**:
- `start_offset_s ~ U[0, 1/90 s)`, the phase of the start inside one camera frame;
- p0 is moved by U[−j, j]³ **only** if the scenario declares `jitter_m` = j (only
  `pentagon_wander` and `pentagon_wander_close`, j = 0.04 m).

The seed is also passed to the depth adapter (`depth_seed`) and to `emulated_perception`. It
matters only when noise or dropout is on. **So for 98 of the 100 scenarios, repeated seeds probe
timing sensitivity** (and any Gazebo non-determinism, which is Unverified), not geometric
variation.

### 5.5 Start clearance (Verified)

`enforce_start_clearance` checks every obstacle at its **spawn** p(0). This is not p0: an
oscillate with a phase, or a wander, is born elsewhere (`_spawn`). The check is against the true
control points at q0. If the minimum gap is below d_safe, the whole trajectory is translated by

  δ = (d_safe − gap_min + 0.001 m) · unit(spawn − cp_nearest)   (12.21)

and the shift is recorded in `start_shift` in the manifest.

At the baseline, d_safe = 0.10 m (source), and **no scenario needs a shift**: the worst start
margin is +2.5 cm (check). The docstring example (`wedge` 9 mm inside) refers to d_safe = 0.15,
which is the value of the currently **installed** YAML (F-6-3). A run launched from that install
would shift `wedge`.

### 5.6 Static, moving and combined runs (Derived)

| Kind | What it exercises | Typical scenarios |
|---|---|---|
| static | the steady-state barrier, the go-around and lateral evasion, the static phantom paths (memory lead: `static` fails on the furniture scene) | `static`, `around`, `wedge`, `pentagon_on_path`, `pentagon_pos_p*`, `pentagon_two_static` |
| moving | velocity estimation (tracker/residual), latency compensation, braking authority | `slow`, `fast`, `ball*`, throws, `pentagon_crossing/lateral/pursuer/sweep/wander*`, `ssb_*` |
| combined | identity separation, multi-obstacle `k` | `pentagon_multi_mixed`, `p4_laws` |
| none | the nominal loop alone (Phase 2) | `pentagon_clear`, `p3_robot_only` |

---

## 6. Conditions: the experiment arms (part D)

### 6.1 Resolution — `simexp.conditions.resolve` (Verified)

`conditions.yaml` has a global `cbf:` block (the 15 `CBF_ARGS` launch flags), an optional global
`cbf_params:` block, and 23 named conditions. Each condition has the `TRACK_ARGS`
(`obstacle_tracking`, `obstacle_velocity_source`), a `perception:` block, and optionally its own
`cbf:` and `cbf_params:`. The precedence is

  global `cbf` < condition `cbf` < `--set`;  global `cbf_params` < condition `cbf_params` < `--set`

with these checks:
- a condition flag that is not in the global block raises;
- **`--set` may only replace a key that is already there**: a `fr3_control.yaml` key that the
  condition does not list raises (`unknown overrides`);
- `tracker` without tracking raises;
- `perception.tracking.enabled` must equal `obstacle_tracking`.

`cbf_params` reach the node through `params_arg` → the `cbf_param_overrides` string →
`torque_control_stack._cbf_param_overrides`, which applies `yaml.safe_load` to each value and adds
the result as a parameter dict (03 §3.6).

**F-6-2 (Verified + Inferred).** `params_arg` writes `obstacle_row_osqp_eps_rel=1e-05`, because
`str(1.0e-5)` is `'1e-05'`. PyYAML follows YAML 1.1, under which a float needs a dot, so
`'1e-05'` loads as a **string**. launch_ros keeps it a string and dumps it unquoted into the
parameter file (`p: 1e-05`). rcl's C YAML parser then reads it as a double (Inferred), and
`declare_float` accepts it. A recorded `minimal_l1_ssb` run shows the filter running with 31 rows
(Runtime (recorded), at `9884c4e`). The chain works by accident. Sixteen conditions carry the key,
and Python consumers of the resolved dict see a str (`C3.effective_params`).

### 6.2 What each condition really changes (Verified, `C3.effective_params('sim:<c>')`)

| From → to | Effective differences |
|---|---|
| hardware default → `full` | `qp_rate_hz` 300 → 100, `enable_go_around` on → off. **Nothing else** |
| `full` → `minimal` | 19 parameters: the h-recovery, ḣ-filter and tangential gains → 0; latency compensation, lateral evasion, livelock escape, uncertainty margin, velocity standoff, zone ladder, link-speed rows, approach margin gate, fast trust, residual floor, retreat cap and state governor → off; the identity-jump and normal-rotation guards → 0; velocity median 5 → 1 and 0.3 s → 0 |
| `minimal` no-ops | 9 settings that `minimal` lists equal what `full` already has, e.g. `enable_velocity_feedforward`, `enable_sensor_range_uncertainty`, `rho_slack`, `obstacle_slack_max`, `qp_failure_policy` |
| `minimal` → `minimal_l1` | `obstacle_slack_mode` family → row_l1; `obstacle_row_osqp_eps_rel` 1e-5 → `'1e-05'` (F-6-2) |
| `minimal_l1` → `minimal_l1_ssb` | `ssb_rows_enabled` → true |
| `residual`, `kf`, `imm`, `imm+pred` | no `qp_rate_hz` → **300 Hz, go-around ON** (the stack default); differ among themselves in velocity source, tracking, IMM and prediction |
| `kf100` | `kf` at 100 Hz |

**F-6-4.** The four Phase 5 conditions (`batch.CONDITIONS`) therefore run at a different QP rate
and with go-around, unlike `full`/`minimal`. Their results cannot be compared with the
`full`/`minimal` family without accounting for that. In addition, their rate gate is always
UNKNOWN (§8.3).

### 6.3 Comparing control configurations (Derived)

A comparison of two arms is valid only when all of the following hold:
1. **Same scenario, scene and perception mode.** The cell name encodes them.
2. **Same QP rate, both nominal and achieved.** The slew box makes the jerk equal to
   `max_qddot_delta/Δt` (`batch.run_one` comment). The achieved rate depends on CPU: `full`
   measured 65 Hz against a 100 Hz target unpinned and 98 Hz pinned (comment, Runtime (recorded)).
   `--filter-cpu` pins `cbf_safety_filter` to one core in every cell.
3. **Several seeds**, knowing that a seed mostly varies timing (§5.4).
4. **The same code revision for every cell**, which `batch_manifest.json` records per invocation
   and each run manifest records per run (F-6-1, F-6-11).

---

## 7. Ground truth and the reflex monitor (parts D/E)

### 7.1 True clearance — `ground_truth_node` + `simexp.truth_geometry` (Verified)

On every `/NS_1/franka/joint_states` sample (1 kHz, queue 100), the node computes the control points
with **the perception's own function** `define_control_points` (05), applied to Pinocchio FK of the
**true** q on the run's RSP URDF. At Q0 these are 11 CPs (fr3_link4…fr3_link8), all with
r_cp = 0.05 m. For each CP i and obstacle j (last received pose, centre c_j, radius r_j):

  gap_ij = ‖cp_i − c_j‖ − r_j − r_cp,i   (signed; < 0 = the spheres overlap)   (12.11)

The truth is **not** clamped. It is the quantity that the engine's
`surface = max(dist − radius − margin, 0)` estimates from the visible points.

- **Published** at 100 Hz on `/sim/truth/cp_gaps`:
  [t, n_cp, min over obstacles per CP …, global min gap, h_min = min gap − d_safe]. Nothing is
  published while there is no obstacle pose yet (the gaps are ∞).
- **Stored** in `truth.npz` (every sample): t, q, flange, CP positions, gaps (n_cp × n_obs),
  obstacle p and v, the pose stamp, labels, radii, d_safe.

**What "true clearance" covers (F-6-7).** It is the clearance between **spheres**: the CP spheres
along the axes of links 4–8 against the obstacle sphere. It is not mesh-to-mesh clearance:
- the upper arm (links 1–3) has no CP;
- the hand's corners lie outside the fingertip CPs (Inferred from the CP placement; not measured).

It is exactly the geometry the barrier constrains, so it measures *barrier* violations, not
*collisions* of the real robot shape. Gazebo's own physics cannot provide the latter (§3.3).

**F-6-5: the series has holes.** The node keeps the whole history in Python lists, and every 5 s
it rewrites all of it with `np.savez_compressed` from the same single-threaded executor. The
joint-state queue holds 0.1 s. Recorded (39 recent runs):
- the median of each run's largest hole inside the episode is 518 ms, and the maximum 837 ms;
- 95 % of the holes > 50 ms lie within 0.5 s of a 5 s multiple (20 % expected if they were placed
  at random).

The save is therefore the probable cause (Inferred). The sample rate within the episode is
909–997 per sim second. This refines the earlier lead "448–688 Hz", which was measured under other
load. Every clearance metric (§8.2) is computed on this grid.

### 7.2 What the logger sees of the truth (Verified)

With a scenario, `experiment_logger` receives `truth_topic=/sim/truth/cp_gaps` and `truth_n_cp=11`
and writes `truth_*` columns at 100 Hz (memory lead: they were dropped by `e9ff7cd` and restored on
5 Nov 2026; `test_phase4_obstacles` pins them). They are a down-sampled copy of the node's
published minimum, not of the 1 kHz series.

### 7.3 Reflex monitor — `reflex_monitor` + `simexp.reflex_checks` (Verified)

Gazebo has no velocity, position or torque reflex, so a run could "pass" where the FR3 would have
stopped. On every 1 kHz arm sample, `ReflexChecker.check` records:

| Event | Condition | Counted as |
|---|---|---|
| velocity | q̇_i > up_i(q) + 10⁻³ or < lo_i(q) − 10⁻³, envelope `fr3_velocity_envelope(q, margin=1)` (libfranka's position-dependent curve) | **trip** |
| position | q_i outside [q_min, q_max] (franka_description) | **trip** |
| torque | \|τ_total,i\| > τ_max,i (87/12 N·m), where τ_total = JSB effort | **trip** |
| torque rate | \|Δτ_user/Δt\| > 999.999 N·m/s, τ_user = effort − g(q) (Pinocchio, g = 9.8 as the plugin), only for Δt ≤ 2.5 ms | **clip** (the FR3 has its rate limiter on, so it smooths rather than aborts), not a trip |

The verdict is `FAIL` if any trip occurred. Otherwise it is `PASS` only if the monitor saw
≥ 90 % of 1 kHz over a span > 0.5 s, and `NO_DATA` if not ("no data" is not "no trip"). The
collision reflexes and the communication reflexes have no sim counterpart. The monitor also measures
the **RTF** from `/clock` against monotonic wall time, per 1 s window:

  RTF = Δt_sim / Δt_wall   (12.19)

It flags runs with RTF < 0.9. The trips are counted from the monitor's own start, but the scorer
counts only those inside the episode window (§8.2).

---

## 8. Batch execution and scoring (part E)

### 8.1 Runner and batch (Verified)

`run_gz_scenarios`:
- uses `--grid phase5 | emulated30 | flags`, or explicit `--scenarios/--conditions/--seeds/--perception`;
- accepts `--set`, `--filter-cpu`, `--max-cells`, `--batch-dir`;
- `--rescore` re-runs the scorers without Gazebo, and `--report` writes the report.

`batch.run_grid` runs the cells **sequentially**:
- each cell is `runner.run_cell`: a fresh `gz_cbf_stack` in its own process group, partition, and
  `ROS_DOMAIN_ID=77`, with `ROS_LOG_DIR` in the cell. The wall timeout is 400 s (`run_one`);
- then `score_cell`;
- then `cell_result.json` is written and `cells.csv` is rebuilt from **every** cell in the batch
  directory.

A cell with a `cell_result.json` is **skipped** on resume, **whether it was ok or not**: a failed
cell is not retried automatically. Two batches started at once would share domain 77 (Inferred
risk; nothing prevents it).

`batch_manifest.json` appends `{git, n_cells, started, cells}` per invocation.

`phase5_grid`:
- seeds 0, 1, 2 × the 11 catalog scenarios × the four `CONDITIONS` with `perception:=true`;
- plus `residual`/`kf` emulated;
- for seeds 0 and 1, the 26 pentagon scenarios and 5 ballistic throws × the four conditions;
- for seed 0, the 8 `throw_a_*` × residual/kf/imm+pred.

The grid has 470 cells; `flags_grid` has 21 and `emulated30_grid` 22 (checked by calling the
functions).

### 8.2 Metrics and what they mean (Verified; `closed_loop.summarize_run` unless noted)

Window: [t_start, t_end] from the manifest (sim time). Log lines are windowed by the wall stamps of
the supervisor's START/DONE lines. With W the truth samples k inside the window, g_k the minimum of
(12.11) over CPs and obstacles at sample k, and Q_m the q̈_safe messages received at bag times τ_m:

  min_h = min_{k∈W} g_k − d_safe   (12.12)

  neg_duration_s = #{k∈W : g_k − d_safe < 0} · median_{k∈W}(t_{k+1} − t_k)   (12.13)

  contact = ∃ k∈W : g_k < 0   (12.14)

  jerk_m = ‖Q_{m+1} − Q_m‖ / median(τ_{m+1} − τ_m),  jerk_p99 = P99(jerk)   (12.17)

  qp_rate_hz = #{/cbf_status messages with τ ∈ W} / (t_end − t_start),  ok ⇔ |qp_rate_hz/target − 1| ≤ 0.05   (12.18)

  frac = (q̈_real · q̈_cmd) / ‖q̈_cmd‖²,  q̈_real = box₁₀ₘₛ(d q̇/dt), counted when ‖q̈_cmd‖ ≥ 0.5 rad/s²   (12.22)


| Metric | Formula | Kind | Caveat |
|---|---|---|---|
| `min_gap` | min over window, CPs and obstacles of (12.11) | **true** | on the holed grid (F-6-5); spheres, not meshes (F-6-7) |
| `min_h` | min_gap − d_safe   (12.12) | **true** | same |
| `contact` | ∃ sample with min gap < 0   (12.14) | **true** | a hole can hide it: synthetic 200 ms hole → a −3 cm penetration reported as +1.2 cm, `contact` False (check) |
| `neg_duration_s` | #(h < 0) · median(Δt)   (12.13) | **true** | time **inside d_safe**, not in contact; median spacing ignores the holes |
| `min_h_cp`, `t_min_h` | label and time of the minimum | true | |
| `task_final_err`, `task_done` | ‖q(t_end) − goal_q‖, < 0.05 rad | **actual** | `goal_q` defaults to Q0; pentagon runs use the EE error instead |
| `pentagon.p95_m` | \|ee − ee_des\| from the commander's CSV, t ≥ t_start | actual vs nominal | the commander's own FK |
| `jerk_p99` | ‖Δq̈_safe‖ / median Δt over bag messages   (12.17) | **commanded**, reconstructed | bag receive times |
| `qp_rate_hz`, `qp_rate_ok` | #/NS_1/cbf_status in window / (t_end − t_start); \|rate/target − 1\| ≤ 0.05   (12.18) | **reconstructed** | the target is only from `condition.cbf_params.qp_rate_hz` (F-6-4); a recorder drop lowers the rate (Inferred) |
| `slack_max_any`, `fault_ticks`, `n_rows_max` | from `/cbf_status` | commanded (the filter's own state) | |
| `slack_max` per family | max over CBFDIAG lines | reconstructed, **sampled** (throttled to `diag_period_s`) | parsed positionally; pinned by a test |
| `fallback_levels`, `slew_conflict` | cumulative counters on the filter's log line, end − start | reconstructed, 100-tick resolution | None = field absent, not 0 |
| `livelock_escapes` | LIVELOCK lines in window | reconstructed | |
| `rows_latency_ms` | bag receive − capture stamp of `/cbf/per_link_distances` | reconstructed | the perception hop only |
| `capage_ms` | CBFDIAG `capage` | estimated (the filter's sensing age) | sampled |
| `reflex_trips`, `rate_clips` | events of §7.3 inside the window | **actual** vs the FR3 limits | |
| `rtf`, `rtf_p10` | (12.19) | harness | |
| `qdd_projection.frac` | (q̈_real · q̈_cmd) / ‖q̈_cmd‖², q̈_real = d/dt of the reported q̇, smoothed 10 ms   (12.22) | **actual vs commanded** | `runner.digest_cell` only |
| estimator `lag/bias/rms` | v_est − n̂ᵀv_obs on on-obstacle samples (`estimator_metrics`, compare_vobs definitions) | **estimated vs true** | v_true is the law velocity, exact |
| tracks `confirm_ms`, `trust_ms`, `id_switches`, `sig_k*` | per obstacle | estimated | |
| attribution `causes` | `ball_throw_eval score` + `approach_probe` causes per pass | reconstructed | moving obstacles only |

### 8.3 Validity of a cell — `batch.cell_ok` (Verified; F-6-6)

  ok = ¬timed_out ∧ ¬leftovers ∧ (min_h exists) ∧ (qp_rate_ok ≠ False)

`ok` means **"a valid data point"**. It does not mean safe, it does not mean the robot would have
run, and it does not mean the controller was active. Not checked:
- `contact` or `min_h < 0`;
- the reflex verdict (FAIL or NO_DATA);
- `controller_active_at_start`;
- the QP fault ticks;
- the perception rate;
- stray hardware logs.

An UNKNOWN rate verdict (`None`) passes. Recorded: of 301 cells, 197 were ok. Of those, 75 had
contact, 13 had a reflex FAIL, and 160 had an UNKNOWN rate (all of the latter from the Phase 5
conditions, F-6-4).

### 8.4 Report, rescore and aggregation (Verified in the parts read)

- **`report.write(batch_dir)`** rebuilds only the text between `<!-- AUTO:BEGIN/END -->` in
  `franka_simulation/docs/gz_cbf_experiments_report.md`:
  - it reads `cells.csv` and aggregates over seeds as the **median** [min, max], with booleans
    as k/N;
  - `t_quality` counts `ok` per perception mode;
  - whether every table excludes non-ok cells was **not verified** (Q-6-2).
- **`--rescore` (F-6-10)** re-runs the scorers on recorded cells, with the **current**
  `conditions.yaml` and the current estimator code. It also writes `compare_vobs_sim.npz` into each
  cell.

### 8.5 Provenance (Verified; F-6-1, F-6-11)

- **Run manifests.** `run_manifest_sim.json` carries `git_provenance()` (SHA, branch, dirty flag).
  The logger manifests do not (13 §2).
- **No patch.** `simexp.provenance.snapshot`, which would copy the sources and `git diff HEAD`, is
  never called outside its test. A dirty run is therefore identified by its base SHA only, and its
  actual code is lost.
- **Recorded runs.** Of the 1227 recorded manifests, 1226 carry a SHA (11 distinct, all ancestors
  of `ba68077`, **none at it**), and 1195 come from a dirty tree.

### 8.6 F-5-21 in simulation (Verified; F-6-13)

The finger publisher corrupts the logger's q, q̇ and τ columns in **every** sim
`experiment_log.csv`; the recorded rate is 26.3–26.7 % of rows (13 §2.2). The scope is:

- **Affected:**
  - the logger CSV and its plots;
  - `experiment_summary.py` (run only by `runner.digest_cell`, which only
    `test_phase2_closed_loop` calls);
  - any hand analysis of those CSVs (e.g. `analyze_isolation_ab.py`, F-5-21).
- **Not affected:** `cells.csv` and the report. `score_cell` uses `truth.npz`, the bag (joint
  states mapped by name), `reflex_*` and `launch.log`, and no `simexp` scorer reads
  `experiment_log.csv`.

---

## 9. Recorded evidence: how to use it (Runtime (recorded))

| Question | Answer from `sim_runs/` (2026-11-11) |
|---|---|
| Which code produced it? | 11 ancestor SHAs; most runs at `2d87ab6` (443), `03ef58c` (329), `ce80f36` (210); 97 % dirty. None at the baseline (F-6-1) |
| Is the truth complete? | No: holes up to 0.84 s per run, aligned with the 5 s save (F-6-5) |
| Are "ok" cells safe? | No: ok is validity only (F-6-6) |
| Are rates comparable across arms? | Not for the Phase 5 conditions (300 Hz, UNKNOWN gate; F-6-4) |
| Is the logger CSV usable for q? | No (F-5-21); use the bag |

**Rule.** A recorded number is evidence about the revision in its manifest, under the scorer of
that time. Before it is used to justify a refactor decision, it must be re-produced on the
baseline under the validation protocol. A run whose truth has a hole at its closest approach
cannot support a "no contact" claim.

---

## 10. Component table (part B)

| Component | Launch (file) | Node / executable | Inputs | Outputs (type) | Effective parameters | Rate configured / measured | Starts after | Failure behaviour |
|---|---|---|---|---|---|---|---|---|
| Gazebo server | `gz_fr3_plant` | `ign gazebo -s -r` | world.sdf | `/clock`; sensors | physics 1 ms (truth_period) | RTF measured (12.19); recorded RTF per run in `reflex_summary` | — | headless render needs EGL; a wrong GPU vendor → CPU rendering (memory lead) |
| ros2_control | plugin in the server | `franka_ign_ros2_control-system` | `rt_controllers_gazebo.yaml` | `/NS_1/franka/joint_states` (JointState) | hardware gains, 1 kHz, `gazebo: true` | 1 kHz sim | spawn at 2 s | — |
| JSB / `rt_torque_controller` | plant / stack spawners | `spawner` | controller manager | active controllers | timeout 60 s | — | 3 s / `control_spawner_delay_s` | inactive → supervisor never ready |
| Commander | stack | `pentagon_qddot_commander` or `scenario_commander` | `/NS_1/franka/joint_states`; `/sim/episode` (scenario only) | `/NS_1/qddot_nom` (Float64MultiArray), `/NS_1/ee_desired`, `/NS_1/ee_actual` (PointStamped) | pentagon: 06; scenario: K_q = 12, 300 Hz | 300 Hz (scenario default); pentagon: 06 | 3 s (scenario) | — |
| CBF filter | stack | `cbf_safety_filter` | qddot_nom, joint states, `/cbf/per_link_distances` | `/NS_1/qddot_safe`, `/NS_1/cbf_status` | condition (§6) | 100 or 300 Hz nominal; achieved from the bag (12.18) | stack timer | QP failure → policy (08) |
| Torque realisation | stack | `qddot_to_torque` | qddot_safe, joint states | `/NS_1/torque_cmd` | 09 | 09 | stack timer | — |
| Perception (true) | stack + plant | `d455_bridge` → `depth_sim_adapter` → `real_time_distance` | Gazebo depth | `/cbf/per_link_distances` (MultiLinkDistance, BEST_EFFORT) | `fr3_complete_sim.yaml`, `depth_adapter.yaml` | 90 Hz render; recorded `rows_hz` per cell | TF `base`, `fr3_link8` | no frames → no rows → supervisor waits |
| Perception (emulated) | `gz_cbf_stack` | `emulated_perception` | joint states, truth poses/velocities | `/cbf/per_link_distances` | fps 90, t_cam 13 ms, seed, `cam_noise` | 90 Hz sim | 3 s | no js → no frames |
| Obstacles | world.sdf plugin | `scenario_obstacles_system` | `/sim/episode_start` (bridged Double) | poses, `/sim/truth/obstacle_poses[_vel]` (PoseArray) | laws (§5.3), `truth_period` 1 ms | 1 kHz sim | in the server | no start message → parked forever |
| Ground truth | `gz_cbf_stack` | `ground_truth_node` | joint states, truth poses | `/sim/truth/cp_gaps`, `/sim/truth/obstacles` (Float64MultiArray), `truth.npz` | d_safe, `fr3_complete.yaml` (installed, unmerged) | js-driven 1 kHz; publish 100 Hz; recorded 909–997 Hz with holes (F-6-5) | 3 s | no pose → no publication |
| Reflex monitor | `gz_cbf_stack` | `reflex_monitor` | joint states, `/clock` | `/sim/reflex` 10 Hz, `reflex_*` | limits from `franka_experiments` | js-driven | 3 s | starved → NO_DATA |
| Episode supervisor | `gz_cbf_stack` | `episode_supervisor` | qddot_nom/safe, torque_cmd, rows, ListControllers | `/sim/episode` 50 Hz, `/sim/episode_start` | warmup 2 s, lead 0.25 s, offset, duration | 50 Hz timer | 3 s | never ready → no end (batch timeout) |
| Logger | stack | `experiment_logger` | `/NS_1/joint_states` (default), truth topic, … | `logger/<stamp>/experiment_log.csv` | defaults; `truth_topic`, `truth_n_cp` (sim) | 100 Hz | stack timer | F-5-21 |
| Recorder | `gz_cbf_stack` | `ros2 bag record --use-sim-time` | 19 topics | `bag/` (sqlite3, uncompressed) | — | native | 4 s | messages before 4 s are not recorded |

---

## 11. Practical example: `slow`, condition `kf`, seed 1, `perception:=true` (Derived from the resolved values)

1. **Resolution.**
   - `scenario_catalog.resolve('slow', 1)`: T = 3 s, scene `bare`, motion `scenario`, nominal
     `joint_pd` to Q0.
   - One obstacle: radius 0.05 m, `pursuit` at 0.3 m/s from flange_q0 + (0, 0.55, 0) =
     (0.328, 0.55, 0.566) m. `start_offset_s` = 5.687 ms, no jitter.
   - `conditions.resolve('kf')`: tracking on, velocity source `tracker`, IMM and prediction off,
     the global block's flags (all on except `outrun_evasion`), no `cbf_params`. That gives **300 Hz with
     go-around** (F-6-4).
2. **Start.** At d_safe 0.10 m the start gap is ≈ 0.55 − 0.05 − 0.05 = 0.45 m at the flange CPs
   (Derived from the flange position; the CPs sit a few cm from it), so there is no shift.
3. **Episode.**
   - The supervisor waits for the commander, filter, torque, rows and the active controller, then
     2 s. It sets t_start = now + 0.25 + 0.0057 s.
   - The ball leaves `PARK_AT` at t_start and approaches the flange at 0.3 m/s. If the arm held
     still, it would reach h = 0 after ≈ (0.45 − 0.10)/0.3 ≈ 1.2 s and contact after ≈ 1.5 s.
   - The scenario commander keeps pulling to Q0, so the CBF must move the arm away against the
     nominal.
   - The episode ends at t_start + 3 s.
4. **Scoring.**
   - `min_h` and `contact` come from `truth.npz` over [t_start, t_start + 3]. A 3 s episode can
     still contain a 5 s save tick, depending on the node's start phase.
   - `task_done` asks whether q returned to within 0.05 rad of Q0 by the end.
   - `estimator.used = v_trk`; attribution runs, because the obstacle moves.
   - `qp_rate_ok` is None, because `kf` has no `qp_rate_hz`.
   - The cell is "ok" if it ended, left no processes and produced a `min_h`, whatever that value
     is.

---

## 12. Tests that pin this subsystem (Verified at docstring level; not run, since pytest needs approval)

| Test | Pins |
|---|---|
| `test_conditions.py` (9) | every toggle explicit, precedence, unknown keys |
| `test_scenario_catalog.py` (9), `test_scenario_laws.py` (5), `test_obstacle_law_wander.py` (10), `test_ssb_scenarios.py` (6) | the port of `cbf_scenarios`, the laws, the SSB geometry |
| `test_closed_loop.py` (10), `test_analysis.py` (2), `test_batch.py` (6), `test_estimator_metrics.py` (8), `test_offline_and_attribution.py` (6) | the scorers on synthetic runs; `FAMILIES` = `GROUP_NAMES`; `LEVELS` |
| `test_reflex_checks.py` (7) | the reflex checks |
| `test_emulation.py` (6) | emulation through the real message constructor |
| `test_sim_config.py` (11) | the sim files that must not drift from the hardware files |
| `test_run_provenance.py` (1), `test_control_regression.py` (3), `test_gz_trajectory_markers.py` (9) | provenance snapshot (uncalled in production), regression scoring, markers |
| `test/slow/test_phase1-5` | real Gazebo runs: plant, closed loop, perception, obstacles + `truth_*` columns, one batch cell |
| `test/launch/launch_test_graph_contract.py` | the sim graph reproduces the fake-hardware safety-path contract (`simexp.graph_contract`) |

---

## 13. Consequences of modification

- **Any change to the stack** (filter, perception, torque, controller) reaches the simulation only
  after a **rebuild**, because the sim reads installed configs and programs (F-6-3).
  `check_installed.py` must report `INSTALLED == SOURCE` before a run counts.
- **`fr3_control.yaml`** changes the filter, the reflex limits (via `franka_experiments`), d_safe
  for truth and start clearance, and `qp_rate_hz` for the rate gate. Changing d_safe changes which
  scenarios are shifted at start (§5.5).
- **`define_control_points` or the CP configuration** changes the perception *and* the truth
  together. A sim run then cannot detect a CP regression, because both sides move. Compare against
  a fixed reference geometry instead.
- **`GROUP_NAMES` / CBFDIAG format.** `closed_loop.FAMILIES` parses the slack line positionally (a
  test pins it). A new family must be added in both places.
- **`/cbf_status` layout** (13 §4) changes `slack_max_any`, `fault_ticks`, `n_rows_max` and
  `qp_rate_hz` (§8.2).
- **Obstacle laws.** The C++ system, `obstacle_laws.py` and the WANDER ratios in three files are
  one motion: change all of them together, and rerun `check_obstacle_laws.py`.
- **`scenarios.yaml`.** The flange offset is relative to Q0's FK; changing Q0 or the hand changes
  every `offset` scenario.
- **Conditions.** Use dotted float literals in `cbf_params` (F-6-2). Add `qp_rate_hz` to any
  condition meant to be rate-gated (F-6-4).
- **Ground truth.** Moving the save off the executor (or to exit only) changes every recorded
  clearance statistic. Old and new runs are not comparable on `min_h`/`contact` (F-6-5).
- **Hidden couplings:**
  - the `franka_sim` (MuJoCo) mirror shares laws and limits (memory lead; out of scope);
  - `emulation` imports `cbf_scenarios`;
  - `estimator_metrics` imports `compare_vobs` and `ball_throw_eval`;
  - `reflex_checks` imports `cbf_hard_limits` and `config`.

  A rename in `franka_experiments/scripts` breaks the sim scorers silently, at import time of a
  rescore.

---

## 14. Findings from this chapter

| ID | Short form | Section |
|---|---|---|
| F-6-1 | no recorded run at the baseline; 97 % dirty | §8.5, §9 |
| F-6-2 | `'1e-05'` string chain works by accident | §6.1 |
| F-6-3 | the installed workspace ≠ the source (configs, d_safe 0.15 vs 0.10, nodes) | §2.4 |
| F-6-4 | Phase 5 conditions at 300 Hz + go-around; rate gate UNKNOWN | §6.2, §8.3 |
| F-6-5 | truth holes up to 0.84 s at the 5 s save; clearance metrics biased | §7.1, §8.2 |
| F-6-6 | `ok` = valid data point, not safe | §8.3 |
| F-6-7 | contact = sphere–sphere; obstacles have no collision | §3.3, §7.1 |
| F-6-8 | emulated perception clamps at 0, one track id | §4.2 |
| F-6-9 | seeds vary timing, not geometry (98/100) | §5.4 |
| F-6-10 | rescore uses current code and writes into run dirs | §8.4 |
| F-6-11 | provenance snapshot never called; dirty patches lost | §8.5 |
| F-6-12 | pentagon commander ignores the episode clock | §5.2 |
| F-6-13 | F-5-21 scope: CSVs yes, batch scores no | §8.6 |
| F-6-14 | sim camera rendered from the calibration file: no extrinsic error | §4.1, §4.3 |

Open questions: Q-6-1 … Q-6-4 in PROGRESS §5.
