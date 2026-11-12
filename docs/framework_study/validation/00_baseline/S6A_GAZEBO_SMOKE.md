# 00_baseline — S6A: one Gazebo smoke test (result)

> Approved by the user on 2026-11-12 (S6A only: one complete Gazebo smoke test). **One cell was run,
> once.** No Tier 1/Tier 2 run, no other scenario, and no retry. No source, launch, configuration or
> test file was changed.
>
> Evidence: `~/franka_baseline_evidence/baseline_f708991/04_gazebo_smoke/` (90 files, 28 MB,
> `SHA256SUMS`, write-protected; SHA-256 of `SHA256SUMS`
> `2f2bef587c491ea1a3a7d29cf7f7d3bc418e2240565e449ba21baf790c370517`).
>
> Labels: **Runtime (recorded)** = measured in this run; **Derived** = computed from those
> measurements by the analysis named; **Inferred** = an explanation not tested.

## 0. Verdict

| | |
|---|---|
| **Pipeline smoke test** (the 11 stages of the approval, §5) | **PASS** (Runtime): every stage carried data and the arm executed the commanded motion to its goal |
| **Harness validity of the cell** (`batch.cell_ok`) | **FAIL**: QP rate 73.6 Hz against the condition's 100 Hz (±5 %, `QP_RATE_TOL`); `ok = False`, RATE_FAIL. The rate requirement is not met at the baseline, unpinned (F-8-5) |
| **Safety in this cell** | no contact (truth, valid near the closest approach), `min_h` +0.068 m, reflex PASS, 0 faults, 0 fallbacks |
| **What this cell does NOT show** | an obstacle-driven intervention. 22–29 obstacle rows were present every tick, but **none was active** (`n_act = 0` on every CBFDIAG line). The only command modification was the joint acceleration box (9.6 → 6.0 rad/s² on joint 1) |

> **Correction (S6A.1, 2026-11-12; F-8-11).** The last row overstates what the recording shows. `n_act` counts rows with h̄ < 0
> (`cbf_safety_filter.py` L1145), not rows binding in the QP. The per-tick slew bound also shaped the
> command (11 ticks at the bound). The obstacle-gap-dependent **link-speed cap** was emitted in
> 7.43–7.70 s (`CBFDIAG vlink` 0.30–0.41 / 0.564 m/s). Box + slew leave a residual up to
> 1.17 rad/s² on 61 ticks. Whether an obstacle-dependent row changed the command is therefore **not
> established** (it is no longer "no"). See [S6A1_PERFORMANCE_ANALYSIS.md](S6A1_PERFORMANCE_ANALYSIS.md) §5.

## 1. Cell selection (why `static / full / seed 0 / perception true`)

**`static`** is the smallest existing scenario with a static obstacle and a measurable end
condition:
- port of `cbf_scenarios.py`, `config/scenarios.yaml` L28–32;
- **T = 4 s**, scene `bare`, `motion: scenario`;
- nominal `joint_pd` from q0 = [0, −0.7, 0, −2.35, 0, 1.57, 0.7] to goal_q = [**0.8**, −0.7, 0, −2.35, 0, 1.57, 0.7];
- one static sphere, r = 0.08 m, at (0.328, 0.18, 0.966) m: 0.40 m above the flange, offset 0.18 m in y;
- end condition: `task_done` (final joint error < 0.05 rad, the `cbf_scenarios.summarize` definition).

The alternatives were rejected:
- `pentagon_on_path` and the other `rig` cells: the scenario file states that the `rig` structure is
  perceived at 0 m and "starves [the QP] to ~20–36 Hz" (L106–108).
- `p3_static`, `around`, `wedge`: 2.5–3× longer.

The commander is the sim's `scenario_commander`. The shipped `pentagon_qddot_commander` is **not**
exercised by this cell. Everything downstream of `/NS_1/qddot_nom` is the production stack.

**`full`** is the protocol's reference condition (§6.1):
- it declares `qp_rate_hz` 100, so the rate verdict is known (unlike F-6-4);
- `enable_go_around` false;
- everything else is as on hardware.

**`perception:=true`** runs the full chain: depth render → `depth_sim_adapter` → `real_time_distance`.

**Seed 0** is the 1:1 port: no start offset, no jitter.

## 2. Preflight (all passed before the launch)

| Check | Result |
|---|---|
| Branch / HEAD / baseline | `humble-mattia` / `f7089911decf` / `ba68077`; 0 non-doc paths changed since the baseline; `git status` outside the docs empty; 3 stashes |
| Install = source | `sha256sum -c` of the 1158 S3 files OK; `check_installed.py` → **`INSTALLED == SOURCE`** (protocol 1.4) |
| Environment resolves to the rebuilt install | `ros2 pkg prefix` of the six → `/ros2_ws/install/<pkg>`; `simexp` imported from the install |
| Earlier evidence | `00`–`03` `SHA256SUMS` verify |
| No Gazebo/ROS running | host and container: only `/bin/bash` |
| **Simulation only** | see "Simulation only" below |
| Resources | 28 CPUs (i7-14700HX: P-cores 0–15 = 8 × 2 SMT, E-cores 16–27); load 0.4; 22 GiB RAM available; 285 GB disk free; RTX 4070 (8 GB, idle); governor `powersave` (intel_pstate, EPP performance); **no isolated CPU** |
| Expected names | supervisor readiness: `/NS_1/qddot_nom`, `/NS_1/qddot_safe`, `/NS_1/torque_cmd`, `/cbf/per_link_distances`, controller `rt_torque_controller` via `/NS_1/controller_manager/list_controllers`; the 19 `BAG_TOPICS` |
| Timeout / termination | the supervisor ends the episode at t_start + T; `procs.Launch` stops the process group (SIGINT 20 s → SIGTERM 5 s → SIGKILL 5 s) and kills by `IGN_PARTITION`; batch timeout 400 s; the supervisor has no readiness timeout (12 §2.8) |
| Rendering | the plant points glvnd at the package's `nvidia_egl_vendor.json`; `libEGL_nvidia` present; NVIDIA GPU visible in the container |

**Simulation only.** Two layers keep commands away from a real robot.

The process path (Verified in the generated files and the log):
- the stack runs with `external_plant:=true` and `use_fake_hardware:=false`;
- `launch.log` L6: "franka bringup SKIPPED (external_plant:=true: the caller provides the robot)";
- the only `ros2_control` hardware is `franka_ign_ros2_control/IgnitionSystem` (`fr3_gazebo.urdf`),
  loaded inside the Gazebo server;
- no `ros2_control_node` and no `franka_hardware` process existed;
- `launch.log` L5 prints `ip=192.168.2.10`, but nothing consumes it.

The network:
- the host's wired interface `eno1` was DOWN; Wi-Fi only (10.131.56.0/24);
- DDS domain 77 (set by `runner.run_cell`) plus `ROS_LOCALHOST_ONLY=1`;
- caveat (Inferred): the container's FastDDS profile `fastdds_no_shm.xml` declares its own unrestricted
  UDPv4 transport, so localhost-only may not be enforced at the transport level. The isolation from
  a LAN ROS graph therefore rests on domain 77.

**`--filter-cpu`: not used (unpinned).** No CPU choice is supported by the existing configuration
and evidence:
- the batch and launch default is unpinned;
- the documented pinning target is an *idle isolated* core (`pin_process.py`, README §6), and this
  host has none;
- recorded uses are inconsistent: CPU 20, an E-core, in `smoke3_20261102`; CPU 2 in the launch
  test; CPU 3 is the robot's RT core;
- the only recorded pinned run of this very cell (`smoke3_20261102`, CPU 20, older dirty revision
  `2d87ab6`) **timed out after 403 s**.

I reported this before launching.

## 3. Command and provenance

```
docker exec -w /tmp -e PYTHONDONTWRITEBYTECODE=1 -e ROS_LOCALHOST_ONLY=1 franka_ros2 bash -c \
  'source /ros2_ws/install/setup.bash; ros2 run franka_simulation_experiments run_gz_scenarios \
   --scenarios static --conditions full --seeds 0 --perception true \
   --batch-dir /home/user/s6a_20261112/batch'
```

- The launch it produced: `ros2 launch franka_simulation_experiments gz_cbf_stack.launch.py
  condition:=full scenario:=static seed:=0 perception:=true run_dir:=…`, with `ROS_DOMAIN_ID=77`,
  `ROS_LOG_DIR=<run>/ros_log` and its own `IGN_PARTITION`.
- Launched 12:08:08 UTC; runner finished after 26 s (cell wall 24.0 s); runner exit 0.
- **Provenance.** `run_manifest_sim.json` records `git.sha` `f7089911…`, branch `humble-mattia`,
  `dirty: true`.
- **F-6-11:** the manifest keeps no patch, so the patch was saved by hand before the launch:
  `preflight/working_tree.patch`.
  - Its non-documentation diffstat is **empty**: the dirty state is the uncommitted
    `docs/framework_study/` only.
  - The code that ran is therefore exactly `f708991`, equal to `ba68077` (install == source).
- **Effective configuration.** All of it is preserved in the run directory:
  - `fr3_complete_sim.yaml`, `rt_controllers_gazebo.yaml`, `depth_adapter.yaml`, `world.sdf`,
    `fr3_gazebo.urdf`, `fr3_rsp.urdf`;
  - the resolved condition and scenario in the manifest and in `preflight/resolved_cell_full_static_s0.json`:
    - `cbf_param_overrides = enable_go_around=false;qp_rate_hz=100.0`;
    - the 17 `stack_args` toggles of `full`;
    - tracker velocity source, no IMM, no prediction;
  - the `fr3_control.yaml` installed copy (== source; `d_safe` 0.10).
- A read-only graph probe confirmed the controllers live. Its parameter dumps started as the run
  ended and failed. Its 1 s node discovery was incomplete (3 nodes), so the node sequence comes from
  `launch.log`.

## 4. Startup and run timeline (`launch.log`; sim times from the manifest and the bag)

| Step | Evidence |
|---|---|
| Gazebo server | `ign gazebo -s -r -v 1 --headless-rendering …/world.sdf` started (pid 10294); `/clock` from 0.198 s sim |
| Plant nodes | `parameter_bridge` ×3, `robot_state_publisher`, `joint_state_publisher`, `depth_sim_adapter`, `static_transform_publisher` |
| FR3 spawn | `create` (ros_gz_sim) started; joint states from 0.222 s sim; q at the first sample = q0 within 0.5 mrad |
| Stack nodes | `real_time_distance`, `cbf_safety_filter`, `qddot_to_torque`, `experiment_logger`, `trajectory_overlay_node` |
| Sim nodes | `scenario_commander`, `ground_truth_node`, `reflex_monitor`, `episode_supervisor`, `sim_bag` |
| Controllers | `joint_state_broadcaster` "Configured and activated"; `rt_torque_controller` "Loaded" and "Configured and activated" (`franka_rt_controllers/RtTorqueController`). The probe's `list_controllers` shows both **active**; manifest `controller_active_at_start: true` |
| Episode | START announced at 6.920 s, **t_start = 7.170 s**, **t_end = 11.180 s** sim (T = 4.0 s); DONE logged; the supervisor exited cleanly → Shutdown |
| Sim time | 11.317 s sim in 16.40 s wall: **RTF 0.69** (p10 window 0.61; every window below the 0.9 flag; one window at 0.008, i.e. a near stall) |
| Termination | `timed_out: false`, `leftovers: []`, runner exit 0. **Teardown was not clean** (F-8-6), after DONE: `gz_server` aborted (exit −6: `std::runtime_error: Node needs to be associated with an executor` in `~IgnitionROS2ControlPlugin`); `cbf_safety_filter` `RCLError: rcl_shutdown already called`; `qddot_to_torque` and `joint_state_publisher` `RuntimeError` on SIGINT. Bag ("Recording stopped"), `truth.npz`, reflex summary and logger output were all written |

Warnings before DONE:
- 54 `cbf_safety_filter` "`_qp_tick gap`";
- `real_time_distance` "approach-jump" on CPs 3–7, 0.47 s wall after START, when the sphere jumped
  from its park position (0, 0, −20) m to p0 (Derived, `truth.npz` `p`);
- 2 `trajectory_overlay` "no frames on /camera/camera/color/image_raw" (no RGB camera in the sim);
- 1 rosbag QoS notice.

There are no errors before DONE.

## 5. The 11 pipeline stages

| # | Stage | Evidence (episode window unless stated) | Result |
|---|---|---|---|
| 1 | Gazebo startup | server up; `/clock` 1000 Hz sim; RTF 0.69 | PASS (RTF flagged < 0.9) |
| 2 | FR3 spawned | `create`; 1 kHz `/NS_1/franka/joint_states` (median Δt 1.0 ms, max 9 ms) | PASS |
| 3 | Controllers active | `joint_state_broadcaster`, `rt_torque_controller` active; `controller_active_at_start: true` | PASS |
| 4 | Nominal commander | `/NS_1/qddot_nom` at 299.8 Hz; joint 1 peak 9.6 rad/s² (= K_q · 0.8) | PASS |
| 5 | Obstacle distances | `/cbf/per_link_distances` 123 messages at **30.7 Hz** (camera 90 Hz; F-8-7), 0–11 valid links; perceived minimum 0.148–0.189 m; perceived − true gap at capture: median **−13.5 mm** (p5 −23, p95 −8.9), i.e. conservative (Derived) | PASS (rate F-8-7) |
| 6 | CBF filter | `/NS_1/cbf_status` every tick (41 fields, schema v3); 693 ticks before START and 295 in the episode, all `fallback_level` 0 (full QP) | PASS |
| 7 | QP execution | n_rows 0 / 22 / 29 (min / median / max); solve wall p50 3.0 ms, p95 9.9, max 17.1; build wall p50 16.1 ms, p95 26.7; `fault_braking` 0; `n_violated` 0; slack max 3.5·10⁻⁴; **rate 73.6 Hz** vs 100 (tick gap p50 10.5 ms, p95 34.4, max 91.0 ms sim) | executes; **rate FAIL** (F-8-5) |
| 8 | Safe acceleration | `/NS_1/qddot_safe` 73.6 Hz; joint 1 peak 6.0 rad/s² = `qddot_accel_limits[0]` (`fr3_control.yaml` L1204); \|safe − latest nominal\| median 0.08, p95 1.31, max 9.05 rad/s²; 60 % of ticks differ by > 0.05 | PASS |
| 9 | Acceleration → torque | `/NS_1/torque_cmd` 73.6 Hz (one per QP tick); peak \|τ\| 3.7 N·m (joints 1, 3) | PASS |
| 10 | C++ torque controller | `RtTorqueController` active. Measured effort ≈ g(q) + `torque_cmd`: residual RMS per joint 0.94, 0.05, 0.54, 0.05, 0.03, 0.01, 0.10 N·m (pinocchio 4.0.0 gravity on `fr3_gazebo.urdf`); at t_end the effort equals g(q) within 0.03 N·m. The residual is the controller's own PD term plus sampling (Inferred) | PASS; **Q-1-1 confirmed at runtime** (the plugin adds g(q)) |
| 11 | Robot motion | see §6 | PASS |

## 6. Measured motion (Runtime + Derived, `analysis/analysis.json`)

Joint states are mapped by **name**. The message order is `[j1, j3, j2, j4, j5, j6, j7]`.

| | joint 1 | joints 2–7 |
|---|---|---|
| q(t_start) | −2.6·10⁻⁷ | q0 within 0.32 mrad (spawn sag removed by the hold) |
| q(t_end) | **0.79934** | unchanged within 0.58 mrad |
| Δq over the episode | **+0.7993 rad** | ≤ 5.8·10⁻⁴ rad |
| peak \|q̇\| | **1.19 rad/s** | ≤ 0.043 rad/s |
| error to goal at t_end | 6.6·10⁻⁴ rad | ≤ 5.6·10⁻⁴ rad |

- **Goal.** ‖q(t_end) − goal‖ = 9.1·10⁻⁴ rad (`task_final_err`); **`task_done` = true** (< 0.05 rad).
- **Trajectory against the intended one.** The reference is the ideal `joint_pd` law (K_q = 12,
  critically damped) from rest at q(t_start), a Derived double-integrator reference.
- **Joint 1 trails the reference early.** Values below are q actual / q reference [rad]:

  | t − t_start | 0.25 s | 0.5 s | 1.0 s | 2.0 s | 4.0 s |
  |---|---|---|---|---|---|
  | q₁ actual / reference | 0.095 / 0.173 | 0.373 / 0.414 | 0.630 / 0.689 | 0.773 / 0.794 | 0.7993 / 0.79999 |

  RMS deviation 0.034 rad, max 0.077 rad.
- **Cause of the lag (Derived):** the QP's acceleration box of 6.0 rad/s² against a nominal peak of
  9.6. **It is not an obstacle effect:** `n_act = 0` throughout.

  > **Correction (S6A.1, 2026-11-12; F-8-11).** Superseded in part. The box explains the plateau, but not every tick:
  > the slew bound and, in 7.43–7.70 s, probably the gap-dependent link-speed cap also acted
  > (S6A.1 §5). `n_act = 0` means h̄ ≥ 0, not "no row bound".
- **Commanded against realised acceleration.**
  - The realised q̈ (10 ms forward difference of q̇) projected on `qddot_safe`: median 0.77
    (p10 0.34, p90 1.19), over 155 ticks.
  - Joint-1 correlation 0.98.
- **Verdict.** The arm follows the safe command and reaches the intended goal; tracking is limited
  by the configured acceleration box.

## 7. Safety metrics (12 §8.2; `cell_result.json → closed_loop`)

| Metric | Value | Note |
|---|---|---|
| `contact` | **false** | sphere–sphere model, not meshes (F-6-7) |
| `min_h` / `min_gap` | **+0.0677 / 0.1677 m** at t − t_start = 0.814 s, CP `fr3_link7#1` | d_safe 0.10 |
| `neg_duration_s` | 0.0 | time inside d_safe |
| **Truth grid (F-6-5)** | 3936 of 4010 expected samples in the episode; median Δt 1.0 ms; **largest hole in the episode 38 ms; within ±0.5 s of the closest approach 3 ms**; the only hole > 50 ms is at t = 0.029 s (101 ms, before START) | the "no contact" claim is supported by the grid **for this cell** |
| reflex | **PASS**, 0 trips; 2 `torque_rate_clipped` events (libfranka would clip, not abort) | not part of `ok` (F-6-6), checked explicitly |
| fallbacks / faults | full 404 / box-only 0 / brake 0 (harness count, 100-tick resolution); `fault_ticks` 0 | |
| slack | `slack_max_any` 3.5·10⁻⁴; every per-family `slack_max` 0.0 | |
| livelock escapes / slew conflicts | 0 / 0 | |
| `jerk_p99` | 69.9 rad/s³ (max 167.6) | |
| perception age at the filter | `distance_capture_age` p50 112 ms, p95 154, max 176 (sim); harness `rows_latency_ms` p50 77, p95 99 | F-8-7 |
| QP rate | **73.6 Hz vs 100: `qp_rate_ok = False`** | F-8-5 |
| `ok` | **False** (rate only) | `ok` alone is not acceptance (F-6-6) |

A recorded run of the same cell on `2d87ab6` (dirty, F-6-1) gave 64.6 Hz, RTF 0.77,
`min_h` +0.074, `task_done`, PASS. The character is the same, but it is not a baseline.

On F-6-5 itself, nothing is superseded: this run is short (11 s), so the 5 s saves of
`ground_truth_node` are cheap. The holes of up to 0.84 s were recorded in longer runs. They must
still be checked per cell.

## 8. Failures and anomalies, analysed

**1. QP rate miss (F-8-5). Component: control (filter compute); environment-sensitive.**
- **Evidence.**
  - 73.6 Hz average against 100 Hz.
  - The median tick gap is on target (10.5 ms), but the tail is long: p95 34 ms, max 91 ms, and
    54 `_qp_tick gap` warnings.
  - Wall time per tick: constraint build p50 16 ms, p95 27 ms; solve p50 3 ms, p95 10 ms.
  - At RTF 0.69, 10 ms of sim time is 14.5 ms of wall time.
- **Cause (Derived):** the filter is compute-bound in wall time, mostly in the constraint build.

  > **Correction (S6A.1, 2026-11-12; F-8-11).** The budget line above adds a per-rebuild cost (50 Hz nominal, 36
  > per sim s achieved) to a per-tick cost, and uses the whole-run RTF. The episode RTF was 0.62–0.64.
  > Both the timer and the gate are in **sim** time (not a clock artefact). Before START the same
  > timer held 99.6 ticks per sim s, and in the episode late ticks follow a fresh rebuild 51 % of the time
  > against 2 % otherwise. Leading hypothesis: in-process contention (GIL / executor), Inferred.
  > "Compute-bound" is not established (the wall times are elapsed, with no CPU sampling).
  > S6A.1 §1.
- **Not established:** whether this comes from GIL contention between the build and tick callbacks,
  or would improve by pinning (Inferred; untested). No retry was made.
- **Classification:** a baseline performance defect of the `full` configuration on this host, made
  worse by an RTF below 1. Host load was about 0.4, so it is not caused by other tenants.

**2. Perception throughput (F-8-7). Component: perception.**
- **Evidence.**
  - Rows arrive at 30.7 Hz of sim time against a 90 Hz rendered camera.
  - `real_time_distance` logs about 57 ms of wall time per frame (n = 4 status lines), with the
    mask step taking 28 ms.
  - The filter sees distances 112 ms old (median).
- **Cause (Derived):** frames are dropped because processing is slower than the frame period.
- **Classification:** a characteristic of the baseline sim. Hardware behaviour is not established.

**3. Unclean shutdown (F-8-6). Component: launch/teardown.**
- It occurs after DONE and has no effect on the recorded data: all artifacts were written, and there
  were no leftovers.
- `gz_server` SIGABRT in the `ign_ros2_control` plugin destructor.
- Python nodes raise on SIGINT. `cbf_safety_filter` calls `rclpy.shutdown` twice.

**4. RTF 0.69.** Every RTF window was below the 0.9 flag. The acceptance bar is not decided
(protocol 3.2: REQUIRES USER APPROVAL).

None of these is an install or provenance problem. The run used the verified baseline.

## 9. Known limitations of this smoke test

- **Correction (S6A.1, 2026-11-12; F-8-11).** "none bound" below is withdrawn: binding was not recorded, and a
  gap-dependent cap row was emitted (S6A.1 §5). What stands is that the barrier's effect on motion is
  **not demonstrated** by this cell.
- **No obstacle-driven intervention was exercised.** The rows were built, but none bound; the
  closest true gap is 6.8 cm outside d_safe. The barrier's effect on motion is **not** demonstrated
  by this cell.
- **The shipped nominal commander was not exercised.** `pentagon_qddot_commander` is untested in
  Gazebo at the baseline.
- **One seed, one run.** No statistics. The memory lead σ = 2.27 cm applies to clearance claims.
- **Contact is the sphere–sphere control-point model** (F-6-7). Gazebo has no collision geometry on
  the obstacle.
- **The perception error is against the CP-sphere truth model, not the robot mesh**, so the −13.5 mm
  figure mixes geometric definitions (Derived).
- **Unpinned.** Timing results depend on host state (powersave governor, desktop session).
- **Not executed:** `digest_cell`, i.e. `experiment_summary` and `smoothness_report` (the batch does
  not call it). The equivalent projection was recomputed in `analysis/`.

## 10. Readiness for a static-obstacle experiment (assessment, not a decision)

- **Functionally ready (Runtime).** The complete acceleration-level pipeline runs in Gazebo at the
  verified baseline, end to end, with valid perception and a correctly actuated plant.
- **Not ready for a comparative or acceptance experiment until the user decides:**
  1. **The QP-rate problem (F-8-5).** Every `full` cell will be `ok = False` unpinned on this host.
     The options are:
     - accept a lower nominal rate;
     - investigate the build cost;
     - choose and validate a `--filter-cpu` core;
     - fix the governor and RTF.

     Each is a decision or a modification for the protocol.
  2. **The RTF bar** and the other REQUIRES-USER-APPROVAL thresholds (Q-6-3).
  3. **A cell where the barrier actually binds** (e.g. `wedge`, `around`, or a `diag_bare` pentagon
     cell). Otherwise the static-obstacle behaviour stays untested.
  4. **The seed count** (single-seed claims are unreliable).

## 11. Follow-up actions (each needs approval)

1. Decide the handling of F-8-5:
   - (a) diagnose the constraint-build cost with existing instrumentation;
   - (b) a pinning experiment on one P-core;
   - (c) the governor set to `performance`.
2. Choose a binding static cell for the first static-obstacle experiment.
3. Decide thresholds (Q-6-3) and seeds.
4. F-8-6 (teardown) and F-8-7 (perception rate) as candidate modifications.
5. Keep saving `working_tree.patch` by hand until F-6-11 is fixed.
