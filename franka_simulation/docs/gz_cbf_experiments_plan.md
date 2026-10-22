# Gazebo replica of the franka_experiments CBF + Kalman experiments — plan

Phase 0 deliverable of the spec for this work (kept outside the repo; "the spec" below). Developed
on `humble-mattia` from `94cd7b0`. Written 2026-10-21. **Phase 0 changed no
code**: it added this file and `docs/sim_reference_contract/`.

Everything here was checked against the code; where the spec is wrong, §3 says so and the code wins.
Every number labelled SIM later in this work comes from Gazebo, never from the robot.

---

## 0. What I need from you (STOP gate)

**Answered 2026-10-21: H1 = option B (thin package `franka_simulation_experiments`), layout OK as proposed.**

1. **H1, the dependency cycle: pick A, B or C** (§6). I recommend **B**, a thin package
   `franka_simulation_experiments`.
2. **OK on the layout** (§4). It includes three small, default-off edits to `franka_experiments`
   (§5.2). Each one gets a test proving the default path is unchanged:
   * `torque_control_stack.launch.py`: `external_plant:=false` (new). When `true`, the stack does not
     include `franka_bringup/franka.launch.py`; the caller (Gazebo) provides RSP, the controller_manager
     and the JSB. Every other action stays the same, including the `rt_torque_controller` spawner.
   * `torque_control_stack.launch.py`: `motion_source:=none` becomes a legal value. No commander is
     started, so the scenario commander can own `/NS_1/qddot_nom`.
   * `experiment_logger`: `truth_topic:=''` (new). When it is non-empty, ground-truth columns are added.
     When it is empty, the header and the rows are exactly today's (`test_experiment_log_columns.py`).

---

## 1. Reference contract (Phase 0 step 1)

`docs/sim_reference_contract/` (README there). Pipeline A on fake hardware: 17 nodes, 45 topics, 22 TF
frames, 11 param dumps. Captured by `capture_contract.py`, the same tool Phase 1's `launch_testing` diff
will use. Points that constrain the sim:

* **Safety path.** `/NS_1/franka/joint_states` comes from the JSB itself: RELIABLE, 1 kHz on hardware,
  100 Hz on fake HW. It is read by `cbf_safety_filter`, `qddot_to_torque` and the commander.
  `/cbf/per_link_distances` is BEST_EFFORT on both ends. `RtTorqueController` subscribes BEST_EFFORT
  depth 1 to `torque_cmd` and `qddot_safe`. Measured on fake HW: QP, `qddot_safe`, `torque_cmd` and
  `cbf_status` at 300 Hz; `qddot_nom` at 100 Hz.
* **TF.** `world →(static) base →(static, RSP) fr3_link0 → … → fr3_link8`, on the absolute `/tf`. No hand
  frames, because the bringup resolves `load_gripper` to `"false"`.

---

## 2. Hazards H1–H13: verification

Verdicts: **✔** confirmed as written, **◐** confirmed with a correction, **✘** wrong.

| # | Verdict | Evidence (file:line @ 94cd7b0) | Consequence for the implementation |
|---|---|---|---|
| H1 | ✔ | `franka_experiments/package.xml:30` `<exec_depend>franka_simulation</exec_depend>`. Reverse uses: `utils/simulation_imports.py:27-60` (only importer: `nodes/capsule_overlay_node.py:47`); `image_publisher` started by `torque_control_stack.launch.py:624`, `minimal.launch.py:224`, `thales.launch.py:195`, `velocity_cbf_control_stack.launch.py:219`. **Measured**: I copied both `package.xml`s into a scratch workspace and added the reverse `exec_depend`. Then `colcon list`, `colcon graph` and `colcon build` all fail with `Unable to order packages topologically: franka_experiments: ['franka_simulation'] / franka_simulation: ['franka_experiments']`. This is a hard error for the **whole workspace**, not a warning. | Your choice, §6. |
| H2 | ✔ | `utils/distance_engine.py:145` `_DEPTH_TO_M = np.float32(0.001)`; `:401-402` `Z = depth*0.001; valid = (Z >= min_depth) & (Z <= max_depth)`; `nodes/real_time_distance.py:633-634` `imgmsg_to_cv2(..., 'passthrough')`. **How 0 is treated:** `min_depth_m: 0.15` (`fr3_complete.yaml:136`), so 0 is dropped as invalid. The mask depth gate (`distance_engine.py:376-377`) requires `depth_here > 0` to reopen a pixel, so 0 is never "in front of the arm". 0 already means "no return", which is the RealSense convention. A raw 32FC1 frame would be scaled by 0.001 and land below `min_depth`: the engine would see nothing, silently. | `depth_sim_adapter`: 32FC1 m → 16UC1 mm, rounded to 1 mm (the D455 depth unit). NaN/±inf/≤0 → 0; values above 65.535 m clip to 0. Engine untouched. |
| H3 | ◐ | Plugin load confirmed on the processed xacro (`fr3.urdf.xacro ros2_control:=true gazebo:=true gazebo_effort:=true hand:=true`): `<plugin>franka_ign_ros2_control/IgnitionSystem</plugin>` (`franka_arm.ros2_control.xacro:26-28`) and `<plugin filename="franka_ign_ros2_control-system">` with `franka_gazebo_controllers.yaml` (`:104-109`). Gravity: `ign_system.cpp:653-657` computes `kdl_model_.gravity(q, {0,0,-9.8})` and `:714` adds it to every **EFFORT-mode** joint's force command. `rt_torque_controller.cpp:120-122` adds none. **Correction 1:** the KDL chain is not "the arm". `ign_ros2_control_plugin.cpp:417-420` builds it from the URDF root to `findTipLink()` (`:249-277`), which is the **last leaf in BFS order**. With the hand that is `fr3_hand_tcp`: urdfdom orders children by joint name, and `fr3_hand_tcp_joint` sorts after `fr3_finger_joint{1,2}`. The fingers (2 × 0.0291 kg, `end_effectors/franka_hand/inertials.yaml`) sit on side branches and are **not compensated**. **Correction 2:** KDL uses g = 9.8, so the world must also use exactly 9.8. **Correction 3:** the README's "Publishing τ = 0 → … uncertain" is resolved by the code: τ = 0 means gravity-compensated float, like the firmware. | World gravity `0 0 -9.8`. The sim URDF makes the fingers rigid at fully open and lumps their mass into `fr3_hand`. KDL then covers the whole EE (franka_sim did the same: "fingers rigid, parked fully open"). Phase 1 test 1 proves it with τ_cmd = 0. |
| H4 | ✔ | `franka_gazebo_bringup/config/franka_gazebo_controllers.yaml` lists 6 types, no `RtTorqueController`. `rt_torque_controller.cpp:443` `if (!is_gazebo_)` gates `SetFullCollisionBehavior`. The plugin reads `update_rate` from its `<parameters>` files (`ign_ros2_control_plugin.cpp:495-507`) and accepts **several** `<parameters>` elements (`:326-334`). | No edit to `franka_gazebo_controllers.yaml`. The sim URDF gets a **second** `<parameters>`: the YAML that `launch_support.generate_rt_controllers_yaml(controller_type='torque', is_real=True, gazebo='true')` already writes. That gives hardware gains and 1000 Hz, with no copy of them. |
| H5 | ◐ | `utils/cbf_hard_limits.py:106` `fr3_velocity_envelope(q, margin)`; `franka_sim/envs/cbf_filter.py:98` the same curve; `libfranka/include/franka/rate_limiting.h:43-46` `kMaxTorqueRate = 1000 N·m/s` (applied as a limiter, `control_loop.cpp:140`); `franka_description/robots/fr3/joint_limits.yaml` effort 87/87/87/87/12/12/12. | `reflex_monitor`: velocity vs envelope (margin 1.0), position limits, \|τ\| vs effort, all at the 1 kHz JSB rate; any of those = FAIL. **Correction (Phase 2):** torque rate is NOT a reflex on this robot. franka_hardware keeps libfranka's limiter on (`torque_command_rate_limiter_active_{true}`, `franka_hardware/include/franka_hardware/robot.hpp:318`, applied `src/robot.cpp:163-166`), so a Δτ/Δt > 1000 N·m/s is CLIPPED, not aborted. It is counted as `torque_rate_clipped` and reported, not failed. Collision thresholds: not emulated (§3.3). |
| H6 | ◐ | `grep use_sim_time nodes/` → 0 hits (✔). **Correction:** the CBF's staleness logic does **not** run on wall time. `cbf_safety_filter._now()` is `self.get_clock().now()` (`:1635-1636`). Every freshness check in `_qp_tick` and `_on_distances` compares that clock with header stamps or receipt stamps from the same clock (`:661,669,682,770-781,844,900`). With `use_sim_time:=true` all of it becomes consistently sim time. Audit of every `time.perf_counter/monotonic/time` in pipeline A: see §2.1. | `use_sim_time:=true` for **every** node through `launch_ros.actions.SetParameter` around the include. No `franka_experiments` change: no semantic use needs a fix at RTF ≥ 0.9. |
| H7 | ✔ | `config/fr3_control.yaml:1-27`; `cbf_safety_filter.py:339-344`, `qddot_to_torque.py:142-152` and `pentagon_qddot_commander.py:619-628` all resolve to `joint_states_fast` = `/NS_1/franka/joint_states`. All of them map joints by name (`cbf_safety_filter.py:644`, `qddot_to_torque.py:177`, `pentagon_qddot_commander.py:686`). | The plugin gets `<ros><namespace>/NS_1</namespace><remapping>joint_states:=franka/joint_states</remapping>`, the same remap `franka.launch.py` applies to `ros2_control_node`. The sim also runs the bringup's 30 Hz `joint_state_publisher`, so `/NS_1/joint_states` exists. |
| H8 | ✔ | `real_time_distance.py:103,116` `load_extrinsics(camera_extrinsics_path)` = `base → camera_color_optical_frame` (`config/camera_extrinsics.yaml`). Intrinsics: `config/depth_intrinsics.yaml` 848×480, fx = fy = 428.844, cx = 426.06, cy = 235.17. HFOV = 2·atan(424/428.844) = 1.5594 rad (89.3°). | `config/sim/camera_extrinsics_sim.yaml`: by default the same numbers, plus a test that it still equals the real file. The SDF sensor pose is generated from it: optical→link rotation, z-forward vs x-forward. Gazebo has square pixels and a centred principal point. `real_time_distance` reads K from `camera_info`, so it stays self-consistent. The 2.1/4.8 px principal-point offset is a documented SIM difference. |
| H9 | ✔ | Mask from **collision STLs** (`fr3_complete.yaml:90-100`, hand → `fr3_link8`); Gazebo renders the **visual DAEs**. Leak counter exists (`real_time_distance.py:914-918`, commit `ce99f50`/`48c284e`). The fingers are not in the mask mesh set, on hardware either. | H9 test: robot-only scene, zero obstacle rows required, leak share reported. |
| H10 | ✔ | `fr3_complete.yaml:412` `cluster_min_points: 5` (lowered from 10, comment `:398-411`); `pixel_step: 6` (`:146`). The comments at `:394` and `:810` still say 10, which is stale. Samples on the grid = π(f·D/2Z)²/36 with f = 428.8. A 5 cm ball gives **2.5** samples at 2 m and **4.5** at 1.5 m: never a cluster. An 8 cm ball gives 6.4 at 2 m and 11.4 at 1.5 m. Gazebo returns 100 % of the disc; the D455 returned ~40 % on the real throws (`approach_speed_envelope.md` H1). So sim is **optimistic** here. Also: the `obstacle_sim.py:78-83` docstring numbers assume `pixel_step` 10. | Reported as a finding. No threshold changed. |
| H11 | ✔ | `approach_envelope.py:5-8` fps 76, t_cam 0.048 s; `range_noise_calibration.py:7-11` σ_z = z²/(f·b)·σ_d with `fr3_control.yaml:2127-2143` (f 428.8, b 0.095, σ_d 0.20); order in `franka_sim/envs/randomization.py:143-188`: delay → dropout (hold last good) → noise → (engine LPF, clamp: those run for real here). Depth-vs-colour frame: `fr3_complete.yaml:102-110` (5.9 cm, uncompensated on the rig). | `depth_sim_adapter` stages, each with its own switch, all off by default: `delay_s` (keeps the capture stamp), `dropout_p` (frame not published; `real_time_distance` then holds, which is what "hold the last good frame" amounts to), `range_noise` (σ_z(z) per pixel, constants read from `fr3_control.yaml`). |
| H12 | ✔ | `fr3_complete.yaml:380` `tracking.enabled: false`, `:648` `imm_enabled: true  # TEMPORARY`, `:523` `prediction.enabled: false`; `fr3_control.yaml:1245` `obstacle_velocity_source: residual`; `launch_defaults.yaml:202-203` turns both to tracker. | `config/conditions.yaml` sets every toggle per condition. IMM and prediction are not launch arguments, so they go into the per-run perception config passed as `robot_config_yaml`. `run_manifest.json` records all of them. |
| H13 | — (risk) | Probe: a headless depth camera at 848×480×90 renders on the RTX 5090 at RTF 1.0, **but only after** pointing `__EGL_VENDOR_LIBRARY_FILENAMES` at the NVIDIA ICD. The container only registers Mesa's (`/usr/share/glvnd/egl_vendor.d/50_mesa.json`), and Mesa fails with `failed to create dri2 screen` and **no frame at all** (§3.2). | RTF logged per run from `/clock` against wall time; runs below 0.9 are flagged. End-to-end capture→CBF latency is measured from the sim bag the way `latency_budget.py bag` does it. |

### 2.1 H6 audit: every wall-clock read in pipeline A

| Site | Use | Class | Action |
|---|---|---|---|
| `cbf_safety_filter.py:584,638,1372-1374` | rebuild / solve timing in ms | diagnostic | none |
| `cbf_safety_filter.py:1292-1297` `_watch_tick_gap` | warns if a QP tick came > `tick_gap_warn_factor` periods late | diagnostic; **gives spurious warnings when RTF < 1** (wall gap vs sim period) | none; documented |
| `real_time_distance.py:632-639,689-694` | `cvt=`, `wait=` stage timers | diagnostic | none |
| `real_time_distance.py:868`, `utils/perception_msgs.py:454` | log throttle | diagnostic | none |
| `real_time_distance.py:996` | calibration-check period (`calibration_check.period_s`) | diagnostic | none |
| `utils/tf_manager.py:160,190-196` | age of the level-3 stale-cache fallback vs `max_age` | semantic, self-consistent (both ends wall): at RTF < 1 it expires *sooner* in sim seconds, i.e. fails closed | none |
| `utils/tf_manager.py:211` | warn throttle | diagnostic | none |
| `utils/kinematics.py:322,391` | startup discovery timeout | diagnostic | none |
| `utils/logging_utils.py:119-123` | `_perf` stage timers | diagnostic | none |
| `pentagon_qddot_commander.py:738,904` | phase-governor freshness of `cbf_status` vs `governor_timeout` | semantic, self-consistent (receipt and check both monotonic). It only bites if `cbf_status` stops for > timeout of wall time. While Gazebo is paused the commander's ROS timer does not fire either. | none |
| `pentagon_qddot_commander.py:724,788` | `request_stop` zero-command duration (shutdown only) | semantic, harmless | none |
| `rl_policy_commander.py:368,434,460,472,486,492` | input staleness (receipt-side monotonic) | semantic, self-consistent; matters only below RTF ≈ 0.2 | none (Phase 6 smoke only) |
| `rl_policy_commander.py:399,411,611-613` | warm-up / inference timing | diagnostic | none |
| `qddot_to_torque.py`, `experiment_logger.py` | none (ROS clock: `_now_sec`) | — | — |
| `RtTorqueController` | `get_node()->now()` vs `update(time)`. The gz plugin forces `use_sim_time` on its CM (`ign_ros2_control_plugin.cpp:509-511`); Humble's CM copies it into every controller it loads. | to verify in Phase 1 (log line) | — |

`ConstraintBuilder`, `obstacle_tracker`, `obstacle_track_pipeline`, `distance_engine`, `self_detection`,
`mask_builder`, `state_governor`, `livelock` and `cbf_evasion` read no wall clock (grep). The tracker's dt
comes from capture stamps.

---

## 3. Things the spec does not list (found while verifying)

### 3.1 The Gazebo URDF is not the bringup URDF

What `xacro … gazebo:=true gazebo_effort:=true hand:=true` produces, and what the sim does about it. All
of it is an adapter on the sim side; `franka_description` is not touched:

| # | Fact (processed URDF / plugin code) | Sim-side handling |
|---|---|---|
| H14 | `gazebo:=true` replaces `base` with a `world` link and `world_joint → fr3_link0` (`franka_robot.xacro:103-115`, `franka_arm.xacro:19-21`). The hardware tree is `world →(static TF) base →(RSP) fr3_link0`. | Two strings from one xacro run. Gazebo's copy gets `world → base → fr3_link0`. RSP's copy gets `base → fr3_link0`, so the stack's own `world_to_robot_base_tf` stays the only publisher of `world → base`, as on hardware. |
| H15 | **Every** joint with an `effort` command interface starts in EFFORT mode at init (`ign_system.cpp:397-399`), and that includes `fr3_finger_joint1`. `write()` then reads `joint_efforts[i]` for i = 7 out of a `std::array<double,7>` (`:647-714`): undefined behaviour on the finger force. | Fingers rigid (fixed joints at 0.04 m open) and removed from `<ros2_control>`. This also fixes H3 Correction 1. |
| H16 | The xacro's initial joint values are `[0, -π/4, 0, -3π/4, 0, π/2, π/4]`. The plugin turns `initial_value` into `JointPositionReset` (`ign_system.cpp:410-415`). | `initial_value` set to the scenario start pose (Q0 = `[0, -0.70, 0, -2.35, 0, 1.57, 0.70]`, `cbf_scenarios.py:62`). |
| H17 | The plugin's CM is `/controller_manager` unless `<ros><namespace>` is given (`ign_ros2_control_plugin.cpp:344-356`). Its `<remapping>` goes into the global rcl context. | `<ros><namespace>/NS_1</namespace><remapping>joint_states:=franka/joint_states</remapping></ros>`. To verify in Phase 1: whether the un-wildcarded `controller_manager:` key of `franka_gazebo_controllers.yaml` still applies under `/NS_1`. The generated YAML uses `/**:`, so we do not depend on it. |
| H18 | `JointTransmittedWrench` → the JSB `effort` field (`ign_system.cpp:544-568`). If the joint constraint transmits the commanded force along the axis, that field is τ_cmd + g(q). | Phase 1 measures it. `reflex_monitor` uses it if it holds; otherwise it uses `torque_cmd` plus the RT law's bound. |
| H19 | **No joint armature** (reflected rotor inertia) in franka_description's URDF; URDF/SDF cannot express it. Joint 7 is then 0.0023 kg·m² (measured: 0.02 N·m → 8.7 rad/s²). The shipped 1 kHz `d_gains[6] = 5` gives Kd·T/I = 2.2, past the discrete stability limit of 2. Phase 2 measured q̇7 flipping sign every tick, Kd saturating at 4.8 N·m, and j7 running 4 rad into its stop (EE error 27-49 cm). | Armature from the project's own MuJoCo FR3 (`franka_sim/assets/franka_fr3/fr3.xml`, Menagerie: 0.195 on j1-4, 0.074 on j5-7), added as a mass-less rotor disk on each child link. g(q) is unchanged; M_77 grows by exactly I_a. After it: Phase 1 j7 step tracked to 1.4 % rms, and Phase 2 passes. |
| H20 | The container's OpenCV bundles Qt with the `xcb` platform only (no offscreen plugin, no Xvfb, no sudo). `cv2.imshow` with `QT_QPA_PLATFORM=offscreen` ABORTS `real_time_distance` (exit -6 on its first frame). The Phase 0 capture never hit this because no frame arrived. | `config/sim/perception_sim.yaml`: `booleans.visualize: false` for headless runs (the overlay is still drawn and published); `gui:=true` turns it back on. |
| H21 | Gazebo renders depth ~1 s before the robot is spawned and its TF exists, so `real_time_distance` masks an arm it cannot place. Measured: a phantom 0.146 m from `fr3_link5` that the filter acted on before the episode. A smaller version remains in rtd's first frames while its own TF buffer fills. That happens before `rt_torque_controller` is spawned (t = 10 s), so nothing executes it, as on the rig. | `depth_sim_adapter` holds frames until `base → fr3_link8` resolves (`wait_for_tf`). Scoring uses the episode window only. |

### 3.2 Environment

* **Headless rendering needs the NVIDIA EGL ICD.** Without it, Mesa: `libEGL warning: egl: failed to
  create dri2 screen`, and the depth topic never publishes a frame. With
  `__EGL_VENDOR_LIBRARY_FILENAMES` pointing at a one-line JSON (`{"ICD":{"library_path":"libEGL_nvidia.so.0"}}`):
  frames arrive and `/stats` reports RTF 1. The JSON ships as `franka_simulation/config/sim/nvidia_egl_vendor.json`
  and the launch sets the variable.
* Fortress is `ign gazebo` 6.18.0 (`ign`, `IGN_GAZEBO_RESOURCE_PATH`). `sim_torque.launch.py` sets
  `GZ_SIM_RESOURCE_PATH`, the Garden+ name. I will check in Phase 1 whether 6.18 honours it, and set both.
* `ros2 param dump` and `ros2 control` can hang on a stale ros2 daemon (`xmlrpc Fault … !rclpy.ok()`, seen
  here). The capture tools use `--no-daemon` and stop the daemon.
* Every `ros2 launch` gets `ROS_DOMAIN_ID=77`, because `franka_labs009` shares the host network.

### 3.3 Firmware features the sim does not reproduce

H5 covers velocity and torque limits. Two more:
* **No collision reflex**: nothing estimates τ_ext against the `SetFullCollisionBehavior` thresholds
  (`rt_torque_controller.cpp:447-454`). Obstacles cannot collide (by design), so the only contacts are the
  arm with the floor or table, and those are caught geometrically.
* **Link-side joint friction**: the URDF puts Coulomb `friction 0.2` and `damping 0.003` on every joint
  (`fr3/dynamics.yaml`), and DART applies them. On hardware the link-side torque loop largely hides
  friction. So the sim plant has 0.2 N·m of stiction the real arm does not show. The Phase 2 qdd_cmd vs
  qdd_real ratio will include it.

### 3.4 Findings of the shipped stack (reported, not changed: out of scope)

* **Torque-rate steps at 300 Hz.** `qddot_to_torque` publishes τ_ff once per q̈_safe (300 Hz), and
  `RtTorqueController` applies each step within one 1 kHz tick. With the slew box at
  `max_qddot_delta` 1.5 rad/s² per QP tick (sized when the QP ran at 100 Hz:
  `ball_throw_closed_loop.md` §3 quotes 150 rad/s³, which is 450 rad/s³ at 300 Hz), M·Δq̈ on joint 2
  reaches 1.2-1.5 N·m per ms. That is above libfranka's 1 N·m/ms, so on the robot the limiter clips it.
  Measured in Phase 2: 6 ticks at the pentagon's first deceleration, peak 1494 N·m/s. Proposal only:
  interpolate τ_ff in the RT loop, or bound the slew in N·m/s.

* **The reported surface gap is biased low by the mask dilation, by design.** `distance_engine`
  subtracts the exclusion-mask margin, converted to metres at each obstacle pixel's depth
  (`distance_engine.py:453-466`), from EVERY obstacle pixel, not only from those near the arm. The
  effective margin is 12 px for the body and 24 px for the EE band (`mask_builder.py:134-136,260-263`).
  The config values are full-resolution pixels integer-divided by `mask_downsample` 4:
  `robot_mask_dilate_px: 2` becomes `max(1, 2//4) = 1` downsampled pixel, i.e. 4 px, and the extra 8 px
  add 8. Measured in Phase 3 (static spheres, true gaps 0.44-0.56 m): raw bias −4.1 to −5.9 cm.
  The residual against `true − 12·Z/fx` has median 1.7 mm and p95 7.3 mm. So the CBF sees obstacles
  ~5 cm closer than they are at 1.6-2.1 m, about a third of `d_safe`. `cbf_scenarios`' emulated
  perception has no such term (relevant to the Phase 5 comparison).
* **Receding obstacle read as approaching at the end of a pass.** A 0.3 m/s sphere crossing in front of
  the arm keeps one confirmed track for the whole pass (velocity error median 2.2 cm/s, speed ratio
  0.94). In the last ~0.5 s, as it moves out of view, the estimate loses half its speed and gains
  −x/−z components: receding samples agree in n̂ᵀv sign only 76 % of the time (approaching: 100 %).
  The consumer takes max(n̂ᵀv, 0), so the error can only tighten the barrier.

* **gz-sim time convention (fixed in the plugin, recorded for anyone writing a system):** in Fortress,
  `UpdateInfo.simTime` in PreUpdate is already the END-of-step time (SimulationRunner advances it
  before calling the systems). Evaluating a law at `simTime + dt` put every obstacle one step
  ahead (measured: 1 ms × v on every sample). A start time on the step grid also needs a 1 ns
  tolerance: 3.53 s arrives as 3.5300000000000002.

* **`cbf_scenarios.py` delivers no perception frame above 1/t_cam ≈ 77 fps.** `run_scenario` keeps ONE
  frame in flight: a frame captured while the previous one is undelivered replaces it and moves the
  delivery time `t_cap + t_cam` out again (`cbf_scenarios.py:307-322`). With `t_cam` = 13 ms and a
  frame period below 13 ms, `last_msg` stays `None`, so no constraint is ever built. Measured on `slow`:
  448 builds at 30 and 60 fps, **0** at 77 and 90 fps (min h −0.25 m, every scenario in contact). The
  offline column is therefore run at 30 fps (shipped) and 60 fps, not at the simulated chain's 90.
* **`compare_vobs.make_residual_estimator` no longer runs.** Its `__new__` shim gives
  `ConstraintBuilder` three attributes, and the current `_obstacle_speed` reads seven
  (`AttributeError: '_vres_dt_min'`, `cbf_state_rows.py:3242`). So `compare_vobs.py`'s replay crashes on
  its residual path. No test covers it. The Phase 5 scoring calls `_obstacle_speed` on a REAL builder
  (`cbf_scenarios.Filter(P).builder`, the run's flags) with the arguments of its call site
  (`cbf_state_rows.py:2048`, n̂ included).
* **`approach_probe.py causes` silently drops a pass with a negative lead.** Its table regex reads leads
  as `(\w+)ms`, and `-40ms` does not match. `ball_throw_eval.py score` prints a negative `cbf_lead`
  when the CBF first acted after the closest approach (its window runs to +0.1 s), and such a pass
  vanishes from the cause counts. The harness rewrites negative leads to `0ms` before parsing. That
  does not change any pass's cause: a lead counts only through `isfinite` and `< need_ms`.

### 3.6 Harness findings (fixed in the harness)

* **The episode could start before the RT controller was active.** The supervisor waited for q̈_nom,
  q̈_safe and τ_cmd. `qddot_to_torque` publishes τ_cmd whether or not anything consumes it, and the
  stack's spawner fires at `control_spawner_delay_s` = 10 s of WALL time. Found in Phase 5 when
  emulated rows made the stack "ready" early: a 4 s episode started and ended with the arm floating
  on the plugin's gravity term. The supervisor now also waits for `rt_torque_controller` to be
  `active` (`controller_manager/list_controllers`) and records `controller_active_at_start`.
  The earlier runs also started before activation: Phase 3 (all), Phase 4 (all), from the launch
  logs. Their conclusions stand, because each needed a still arm, and the plugin's g(q) holds q0
  exactly with a zero command (Phase 1: drift 0). Phase 2's pentagon commander is spawned after
  the controller, so its episode was not affected.
* **The QP loop runs at 220–290 Hz of sim time, not 300.** The median spacing of `/NS_1/qddot_safe` is
  4 ms. The filter's 300 Hz timer runs on the 1 kHz `/clock`, so ticks are quantised, and a late tick is
  dropped, not caught up. CPU contention on the shared host (load 12–24 from another user's jobs) lowers
  it further. There is no shipped knob for it: `rt_pin_cpu` pins only `ros2_control_node`'s RT thread,
  which does not exist with `external_plant`. The batch reports the achieved rate per cell.
* **The run manifest held a stale reflex snapshot.** The supervisor copies `reflex_summary.json`
  when the episode ends, but the monitor rewrites it every 2 s and again at exit, after the supervisor.
  Across the batch the copies held 12458 torque-rate clips, where the trip files hold 21003 inside the
  episodes. No trip was missed:
  every event in every `reflex_trips.csv` is a clip, and all 345 final verdicts are PASS. Fixed twice:
  the monitor puts its final numbers into the manifest at exit (`reflex_final: true`), and the scorer
  counts events from `reflex_trips.csv` inside the episode window.
* **The graph-contract launch test measured rates in wall time.** It assumed RTF ≈ 1 and read 196 Hz
  for the QP at RTF ~0.9 on a loaded host. It now counts over a sim-time window. The QP loop still
  reached only 228–235 Hz unpinned under host load ~20. Pinning the filter to idle isolated core 3 in
  the same running stack gave 300 Hz (245 → 300 Hz), so the cause is CPU contention.
  `gz_cbf_stack filter_cpu:=N` (`scripts/pin_process.py`, `taskset -a`, only this run's partition)
  is now an option, default unpinned as in the batch. The launch test uses core 2 (3 is the robot's RT
  core) and passes with 297.5–297.8 Hz, 0 contract problems, threshold unchanged.
* **`emulated_perception` first used a 1 kHz publish timer**, which cost CPU the QP loop needed. It now
  publishes on one frame-rate timer, building each frame from a 200 ms truth history at `now − t_cam`.

### 3.5 Stale documentation found on the way (not fixed: out of scope)

* `ARCHITECTURE.md` §6.8: `outrun_evasion` default is now `false` (`launch_defaults.yaml:222`), not `true` at `:218`.
* `torque_control_stack.launch.py:536-540`: TF is on `/tf`, not `/<ns>/tf`.
* `fr3_complete.yaml:394,431,810`: comments still assume `pixel_step` 10.

---

## 4. Proposed file layout

Written for option **B** of §6; the A/C deltas are in §6. Two rules decide where a file goes:
* `franka_simulation` keeps **plant** assets with no `franka_experiments` dependency.
* The new package holds the **experiment harness**, which needs both.

```
franka_simulation/                              (ament_cmake, existing)
├── worlds/fr3_cbf_lab.sdf                      world template: 1 kHz physics, g = -9.8, floor, base
│                                               pedestal / table as on the rig, camera mount
├── src/scenario_obstacles_system.cpp           ignition-gazebo6 System (C++) — §5.4
├── config/sim/                                 sim-only overrides (rule 2), one comment per key
│   ├── camera_extrinsics_sim.yaml              = config/camera_extrinsics.yaml by default (test-pinned)
│   ├── perception_sim.yaml                     fr3_complete.yaml overrides: booleans.visualize false
│   │                                           (headless), and nothing else unless physically different
│   ├── depth_adapter.yaml                      encoding + the three H11 stages (all off)
│   └── nvidia_egl_vendor.json                  §3.2
├── scripts/depth_sim_adapter.py                node: 32FC1 → 16UC1 (+ optional stages)
├── scripts/utils/depth_conversion.py           pure logic of the adapter (pytest)
├── scripts/utils/gz_robot_description.py       xacro → (gazebo URDF, RSP URDF), §3.1 post-processing
├── test/sim/                                   pytest for the above (no ROS graph needed)
└── docs/ (this plan, contract, report)

franka_simulation_experiments/                  (NEW, ament_cmake; exec_depends both)
├── launch/gz_cbf_stack.launch.py               the one-command experiment (DoD), §5
├── launch/gz_plant.launch.py                   Gazebo + FR3 + controllers only (Phase 1 tests)
├── config/
│   ├── scenarios.yaml                          1:1 port of cbf_scenarios.scenarios() + pentagon_* +
│   │                                           throw sweep (test-pinned to the Python)
│   └── conditions.yaml                         residual | kf | imm | imm+pred, every toggle explicit
├── scripts/                                    (install(PROGRAMS … RENAME), no .py)
│   ├── scenario_commander.py                   run_scenario's nominal laws → /NS_1/qddot_nom
│   ├── ground_truth_node.py                    true obstacle states + true per-CP gaps → /sim/truth/*
│   ├── reflex_monitor.py                       H5 + RTF → reflex_trips.csv, summary
│   ├── episode_supervisor.py                   waits for the stack, starts the episode, ends the run,
│   │                                           finalises run_manifest.json (plant, RTF, git)
│   ├── run_gz_scenarios.py                     batch runner, CSV + report
│   └── utils/  scenario_catalog.py  scenario_laws.py  truth_geometry.py  reflex_checks.py
│               sim_config.py  scoring.py  run_manifest.py               (pure, pytest-reachable)
└── test/
    ├── test_*.py                               pure logic (default suite)
    ├── launch/test_graph_contract.launch_test.py   launch_testing: sim graph vs Phase 0 contract
    └── slow/                                   @pytest.mark.slow Gazebo runs (not in the default suite)
```

Outputs go to `franka_simulation_experiments/sim_runs/<date>/<cell>/` (git-ignored), never to
`franka_experiments/franka_logs/` or `rosbag/`. Each run directory holds `run_manifest.json` with
`"plant": "gazebo"`, RTF and git sha, plus the logger CSV, a small sim bag (`--use-sim-time`, no images),
truth, reflex trips and the launch log.

---

## 5. How the Gazebo launch reuses pipeline A (no copied node wiring)

### 5.1 Shape

`gz_cbf_stack.launch.py` is an `OpaqueFunction` that:

1. Resolves `scenario`, `condition`, `seed` and `motion` (`pentagon` | `scenario` | `rl`) into one explicit
   toggle dict (rule 3), using `sim_config.py`, and writes the run directory.
2. Generates the per-run files: both URDF strings (§3.1), the world SDF (template + camera pose from
   `camera_extrinsics_sim.yaml` + obstacles + plugin config), and the merged perception YAML (`fr3_complete.yaml` ⊕
   `config/sim/perception_sim.yaml` ⊕ the condition's `tracking.enabled` / `imm_enabled` / `prediction.enabled`).
   It reuses `torque_control_stack._rtd_config_with_overrides`' merge semantics, not a copy of them.
3. Emits `SetEnvironmentVariable` (`FRANKA_LOGS_DIR` = run dir, which `utils/config.resolve_log_dir`
   already honours; resource paths; EGL; `QT_QPA_PLATFORM=offscreen`) and `SetParameter(use_sim_time, True)`.
   Then the **plant**: `ign gazebo -s -r --headless-rendering` (GUI optional), `/clock` + depth +
   `camera_info` + truth bridges, `ros_gz_sim create`, RSP and `joint_state_publisher` in `NS_1` (the
   bringup's two nodes, which a Gazebo plant has to provide itself), and the JSB spawner.
4. **Includes `franka_experiments/launch/torque_control_stack.launch.py`** with `external_plant:=true`,
   `enable_camera:=false`, `rt_pin_cpu:=''`, `start_move_group:=false` (the commander does not use it,
   ARCHITECTURE §12.3), `motion_source:=pentagon|none|rl`, `camera_extrinsics_yaml:=<sim>`,
   `robot_config_yaml:=<merged>`, `camera_depth_profile:=848x480x90`, every CBF flag spelled out, and
   `trajectory_overlay_window:=false`. The stack's own spawner starts `rt_torque_controller` in Gazebo's
   `/NS_1/controller_manager`, unchanged.
5. Adds the sim-only nodes: `depth_sim_adapter`, `scenario_commander` (only when `motion:=scenario`),
   `ground_truth_node`, `reflex_monitor`, `episode_supervisor`, and a `ros2 bag record --use-sim-time` of
   the non-image topics that `smoothness_report.py` and `latency_budget.py bag` read.

`launch_support.py` is reused as is: `generate_rt_controllers_yaml(controller_type='torque', is_real=True,
gazebo='true', …)` produces the plugin's second `<parameters>` file (H4).

### 5.2 The three default-off edits to `franka_experiments`

| Edit | Diff size | Default-path proof |
|---|---|---|
| `external_plant` arg; skip `franka_launch` (`torque_control_stack.launch.py:413-426,481`) when true | ~10 lines | `test/test_torque_stack_external_plant_off_identical.py`: the expanded action tree is serialised for the defaults and for `external_plant:=false`. Normalised (temp paths), it must equal a golden snapshot taken from the file at `94cd7b0`. `external_plant:=true` may differ by exactly the bringup include. |
| `motion_source:=none` (`:817-887`) | ~5 lines | same test: `pentagon` and `rl` trees unchanged; `none` = `pentagon` minus the commander node |
| `experiment_logger` `truth_topic` (default `''`) | ~40 lines | `test_experiment_log_columns.py` unchanged and green, plus a test that the header is identical with the parameter empty and gains only `truth_*` columns when it is set |

Why the launch needs an edit at all: the stack includes the robot bringup unconditionally, and there is
no external way to drop one included action. Fake hardware does not help either: it would start a second
`/NS_1/controller_manager` next to Gazebo's.

### 5.3 Scenario commander

The two nominal laws of `run_scenario` are inline at `cbf_scenarios.py:355-374`, so they cannot be
imported. `scenario_laws.py` restates them:
* joint PD `-12(q-goal) - 2√12 q̇`;
* Cartesian `25e + 10(v_ref-v)` through `J⁺ = Jᵀ(JJᵀ+0.05²I)⁻¹`, null-space `-6(I-J⁺J)q̇`, on `fr3_link8` with the
  same Pinocchio no-hand model (`kinematics.build_urdf_no_hand`, `CBFKinematics.point_jacobian`).

The test does **source-level pinning**, the way `approach_envelope.py` patches `run_scenario`: it asserts that
the expressions in `cbf_scenarios.run_scenario` are still the ones restated. It also checks the module
numerically against those expressions evaluated on random states.

### 5.4 Obstacles (Fortress mechanism) and ground truth

* **`ScenarioObstacles` system plugin** (C++). In every `PreUpdate` it writes `WorldPoseCmd` for each
  obstacle model from **one** law function `p(τ)` evaluated at `simTime + dt`. Laws: static, constant
  velocity, pursuit of the **true** `fr3_link8` world pose read from the ECM in the same step (= `Sphere.step`),
  analytic ballistic `p0 + v0τ + ½gτ²`, and wedge = two statics.
  The models are visual-only: no `<collision>`, gravity off. The depth camera sees them; the arm cannot touch
  them. This holds the law at the physics rate with no per-tick service call.
* Why not the alternatives: actors can only follow a fixed script, so no pursuit. `VelocityControl` and
  `TrajectoryFollower` are force- or command-topic driven, so not exact. Fortress has no Python systems.
* **Episode discipline:** obstacles are parked out of view until `episode_supervisor` sends the start
  trigger (gz topic). They then appear at `p(0)`: a birth, not a teleport, and the first step moves them by
  `p(dt)-p(0)`. This matches `cbf_scenarios`, where every track is born at t = 0. Each start pose is checked
  to be outside `d_safe` of every control point before the episode is allowed (`_sample_episode` rule).
  Seeds drive: the start offset within one camera frame period, optional position jitter (off for the 1:1
  port), and the adapter's noise RNG. Everything is written into the run manifest, so a run can be replayed.
* **Ground truth:** poses bridged at 1 kHz → `ground_truth_node`. Truth velocities come from a central
  difference, exact for constant velocity. True CP positions use FK of the true `/NS_1/franka/joint_states`,
  which is exact in sim, through `distance_utils.define_control_points` with `fr3_complete.yaml`'s
  segments, radii and `ee_tip_offset`. True gap = `‖cp − c‖ − r_obs − r_cp`. Published on
  `/sim/truth/obstacles` and `/sim/truth/cp_gaps`, and logged by `experiment_logger` through `truth_topic`.

### 5.5 Phase 5 measurement plan (sketch, fixed in Phase 4)

* Estimator vs truth: `compare_vobs.cross_correlation_lag`, `static_windows` and `rms` are imported, not
  copied. σ calibration uses `sqrt(n̂ᵀP_vv n̂)` exactly as `watch_obstacle_tracks.py:121`.
* Closed loop: `summarize()`'s definitions applied to logs. h from truth gaps − d_safe; slack per family,
  fallback rungs and livelock escapes from CBFDIAG / `cbf_status`.
* Three columns: `cbf_scenarios` offline is run as is. The "Gazebo + emulated tracker" column means
  replacing `real_time_distance` with a truth-fed `MultiLinkDistance` publisher. It is built only if
  `perception_msgs` construction can be reused cheaply; otherwise it is skipped, as the spec allows.

---

## 6. H1 options (pick one)

| | What | Cost | Risk to the hardware path | Spec deviation |
|---|---|---|---|---|
| **A** break the old edge | Drop `franka_experiments → franka_simulation`. Move `image_publisher.py` into `franka_experiments` and repoint the 4 launch files (`capsule_overlay_node`'s import bridge stays an undeclared runtime use). Then `franka_simulation` can declare `exec_depend franka_experiments`. | ~1–2 h | **Yes**: edits 4 hardware launch files and the package manifest. `colcon build --packages-up-to franka_experiments`, the documented recipe, stops building `franka_simulation`. | none on file locations |
| **B** thin package *(recommended)* | New `franka_simulation_experiments`, exec_depending on both. Plant assets stay in `franka_simulation` (no new dependency there); the harness goes in the new package (§4). | ~30 min scaffolding, one more package to build | **None**: no existing edge or launch file changes | harness files live in the new package instead of `franka_simulation`; `ros2 launch franka_simulation_experiments gz_cbf_stack.launch.py …` instead of `franka_simulation <…>` |
| **C** undeclared runtime dependency | Everything in `franka_simulation`; it uses `franka_experiments` only at runtime (FindPackageShare, imports from its install) and declares nothing. | 0 | none | none on locations, but it repeats the hidden-dependency smell ARCHITECTURE §12.6 complains about. Build order is correct only by accident, and `--packages-up-to franka_simulation` does not build `franka_experiments`. |

Every option keeps the three default-off edits of §5.2. Under A or C, everything in §4 lives in
`franka_simulation` (`launch/`, `config/sim/`, `scripts/`, `test/`), and the DoD command is the spec's.

---

## 7. Corrections to the spec (summary)

1. H3: the KDL chain ends at `fr3_hand_tcp` and leaves out the fingers; KDL uses g = 9.8. "Unresolved" in
   the README is resolved by the code: τ = 0 floats.
2. H6: `cbf_safety_filter`'s staleness checks run on the node's ROS clock, not on wall time. The wall-clock
   reads that remain are diagnostics, or receipt-age checks that are consistent on both ends.
3. H7 holds, but the TF topics are the absolute `/tf` and `/tf_static`; the launch comment saying `/<ns>/tf` is wrong.
4. H2 cites `real_time_distance.py:633`; the code is `:633-634`.
5. New hazards H14–H18 (§3.1). Without them, the plant would not produce the contract: no `base` frame,
   finger UB, wrong start pose, wrong namespace.

---

## 8. Task list (one item per phase step; kept current)

| Phase | Step | Status |
|---|---|---|
| 0 | 0.1 reference contract on fake HW | done |
| 0 | 0.2 verify H1–H13 | done (§2, §3) |
| 0 | 0.3 this plan; **STOP** for H1 + layout | done: B, layout OK |
| 1 | 1.1 world `fr3_cbf_lab.sdf` (1 kHz, g -9.8, floor, tables measured from `irim1_raw`, camera mount) | done |
| 1 | 1.2 sim URDF post-processing (§3.1) + plugin params (H4) + `gz_plant.launch.py` | done |
| 1 | 1.3 test: τ_cmd = 0 hold 10 s at Q0 (H3) | done: drift 0 rad, effort = g(q) |
| 1 | 1.4 test: q̈ step via `/NS_1/qddot_safe` tracked | done (with H19): rms j1 1.6 %, j6 0.85 %, j7 1.4 %; velocity integrity 0.2 % |
| 1 | 1.5 test: launch_testing graph diff vs contract (safety path) | done: 0 problems |
| 1 | 1.6 `external_plant` + `motion_source:=none` edits + identical-default test; suite 1196 green | done |
| 2 | 2.1 `gz_cbf_stack.launch.py` (pentagon; perception ON, see H21/§5) | done |
| 2 | 2.2 `reflex_monitor` (H5) + RTF + `episode_supervisor` (manifests SIM-labelled) | done |
| 2 | 2.3 run: digests unchanged (exit 0, PASS); EE err p95 4.3 mm; 0 trips (6 rate clips); projection 1.006 | done |
| 3 | 3.1 camera sensor at H8 pose; `depth_sim_adapter` (H2, H11 stages off) | done (in Phase 2) |
| 3 | 3.2 test: static spheres, per-CP gap vs truth | done: residual median 1.7 mm, p95 7.3 mm (after the 12 px margin) |
| 3 | 3.3 test: robot-only scene (H9), leak share | done: 0 rows, leak 0 px |
| 3 | 3.4 test: constant-velocity sphere → confirmed track, v_obs sign/magnitude | done: err 2.2 cm/s, approach sign 100 % |
| 4 | 4.1 `scenarios.yaml` + pytest vs `cbf_scenarios.scenarios()` / `approach_envelope.cell` | done |
| 4 | 4.2 `scenario_commander` + law tests (bit-equal to the extracted source lines) | done |
| 4 | 4.3 `ScenarioObstacles` plugin + truth-vs-law test (analytic ≤ 3e-14 m, pursuit 2e-7 m) | done |
| 4 | 4.4 `ground_truth_node` + `experiment_logger` `truth_topic` edit + golden-header test | done |
| 4 | 4.5 episode discipline (birth at p0, start clearance: wedge +10.4 mm, seeded replay) | done |
| 5 | 5.1 conditions × scenarios runs; estimator vs truth tables | done: 345 cells (302 grid + 22 emul@30 + 21 flag-off), all ok, 0 reflex trips |
| 5 | 5.2 closed-loop tables; three-column comparison + cause attribution | done (emulated column built, offline at 30/60 fps: §3.4) |
| 5 | 5.3 tables into the report (STOP gate → continue, proposals only) | done: `gz_cbf_experiments_report.md` |
| 6 | 6.1 `run_gz_scenarios` (headless, fresh Gazebo per cell, resumable, `--rescore`, `--report`), CSV + report | done |
| 6 | 6.2 docs: franka_simulation README §13, ARCHITECTURE §10, status P1.6 | done |
| 6 | 6.3 optional: `motion_source:=rl` smoke | done: auto-selected ens_e3_all12.onnx, q̈_nom 100 Hz, flange 0.21 m toward the target, reflex PASS, no contact |

---

## 9. Deviations from the spec (complete list, with the reason)

| # | Spec | What was done | Why |
|---|---|---|---|
| D1 | `ros2 launch franka_simulation <sim_stack>.launch.py scenario:=… condition:=…` | `ros2 launch franka_simulation_experiments gz_cbf_stack.launch.py scenario:=… condition:=…` | H1, option B (your choice): `franka_simulation` cannot depend on `franka_experiments` without a cycle, so the harness lives in the thin package. |
| D2 | scenario catalog in `config/sim/scenarios.yaml`; new scripts installed by `franka_simulation/CMakeLists.txt`, pure logic under `scripts/utils/` | `franka_simulation_experiments/config/scenarios.yaml` and `conditions.yaml`; scripts installed by `franka_simulation_experiments/CMakeLists.txt` (`install(PROGRAMS … RENAME)`); pure logic in `scripts/simexp/` | Same reason. The plant-side overrides (rule 2) are in `franka_simulation/config/sim/*.yaml` as the spec says. `simexp`, not `utils`, so one interpreter can import both packages. |
| D3 | `gz sim -s` | `ign gazebo -s -r --headless-rendering` | Fortress 6.18's CLI is `ign gazebo`. |
| D4 | — | `handeye_eye_in_hand_node.py:200`: `self._parameters` → `self.get_parameters_by_prefix('')`, behaviour-identical | The suite was red before any change (`test_node_self_references`); "green" needed it. Separate commit. |
| D5 | Phase 3 moving sphere: "v_obs sign / magnitude" | Acceptance on the APPROACHING samples (sign 100 %); receding samples (76 %) reported as a finding (§3.4) | The barrier consumes max(n̂ᵀv, 0). A receding obstacle read as approaching only tightens it, so the safety-relevant sign is the approaching one. Both numbers are written to `phase3_results.json`. |
| D6 | Three-column comparison at a common rate | offline column at 30 fps (shipped) and 60 fps; emulated Gazebo at 30 and 90 fps; full chain 90 fps | `cbf_scenarios` delivers no frame above ≈77 fps (§3.4). emul@30 vs off@30 isolates the plant; emul@90 vs full isolates the perception. |
| D7 | "reuse compare_vobs' functions" | `cross_correlation_lag`, `static_windows`, `rms` imported. The residual estimator is called through a real `ConstraintBuilder` (`cbf_scenarios.Filter`), not through `make_residual_estimator` | The shim no longer runs (§3.4). |
| D8 | "attribute with approach_probe.py causes" | `ball_throw_eval.py score` and `approach_probe.cmd_causes` unchanged, after rewriting negative leads to `0ms` | The regex drops negative leads (§3.4). The rewrite preserves every cause, and a unit test proves it. |
| D9 | "Gazebo with the tracker emulated, if cheap, else skip" | Built: `emulated_perception` (rows from truth through `perception_msgs.build_cp_messages`, track fields as `cbf_scenarios`) | Cheap. It uses the real CP set and radii, not `cbf_scenarios`' R_CAP set. The velocity noise is drawn per entry per frame, not per 50 Hz rebuild. Both are listed as part of the column-1 → column-2 gap. |
| D10 | Contact = obstacle touches the arm | contact = TRUE surface gap < 0 for the real control-point capsules (`define_control_points` radii) | Obstacles are visual-only (no collision), as the spec requires. The geometric definition is `cbf_scenarios`' (h < −d_safe), applied to the real CP set. |
| D11 | — | `wedge` start moved 10.4 mm radially; `pentagon_on_path` obstacle at [0.4, −0.25, 0.45], r 0.05 | "Never start inside the barrier": with the real CP set, `wedge`'s 1:1 start is 9.4 mm inside d_safe. The shift is recorded per run (`start_shift`). |
| D12 | Reflex "trips" | torque RATE above 1000 N·m/s counted as `torque_rate_clipped`, not as a trip | H5 correction: libfranka's limiter clips (`robot.hpp:318`), it does not abort. |
| D13 | optional cross with the CBF flags that consume the track | kf × {vobs_in_hdot, uncertainty_margin, latency_compensation} off, one at a time, 7 scenarios, seed 0 | Kept small. It is optional, and each cell is a fresh Gazebo. |
| D14 | Phase 6 `run_gz_scenarios.py` | Written in Phase 5 (the Phase 5 grid needed it), finished in Phase 6 | Order only. |
