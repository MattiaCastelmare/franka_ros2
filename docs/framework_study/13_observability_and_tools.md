# 13 — Observability and tools: what a run leaves behind, and how to read it

> **Source baseline:** `ba68077`. Written in Stage 5 (2026-11-11). Labels as in
> [00_INDEX.md](00_INDEX.md) §2. Line numbers refer to the baseline.
>
> **Reproducible checks.** [tools/stage5_verification/](tools/stage5_verification/README.md):
> `check_observability.py` drives the production `ExperimentLogger` callbacks with real
> `sensor_msgs`/`franka_msgs` messages and checks the recording wiring by source match.
> `check_iso.py` 9 covers the `/cbf_status` S_p field. Checks 5–11 of `check_observability.py` come
> from a parallel session's draft, merged at the reconciliation (PROGRESS §4 F-5-24…F-5-27; C-035).
>
> **Depth of reading.** The data-producing nodes (`experiment_logger`, `iso_evidence_logger`,
> `cbf_diagnostics`, the rosbag block, `read_bag`) were read in the parts that decide **what a
> number means**. The 25 analysis scripts and the five visualisation nodes were read at interface
> level: docstring, inputs, outputs, and whether they move the arm. The inventory in §7 says which.

---

## 1. Intuition

A safety filter that misbehaves for 30 ms leaves no trace unless something wrote it down, and a
refactor is only as safe as the evidence that the behaviour did not change. Four layers of
evidence exist:

| Layer | Producer | Rate | Survives the run as |
|---|---|---|---|
| The run CSV | `experiment_logger` | 100 Hz | `experiment_log.csv`, `run_manifest.json`, plots |
| The filter's own status | `cbf_safety_filter` → `/NS_1/cbf_status` (41 fields), plus the CBFDIAG log line | 300 Hz / throttled | only if recorded (bag) or logged |
| The ISO evidence | `iso_evidence_logger` (11 §6.2) | 100 Hz + 1 kHz running maxima | `iso_manifest.json`, `iso_evidence.csv`, `iso_events.csv` |
| The raw streams | `ros2 bag record` (`start_rosbag`) | native | a zstd-compressed rosbag2 |

Each layer answers a different question. The most common mistake when reading them is to take a
column's **name** for its **meaning**. The meanings, with their caveats, are in §2.4 and §9.

---

## 2. The run CSV: `experiment_logger` (Verified)

### 2.1 What it writes

- `<output_dir>/<stamp>_<experiment_name>/experiment_log.csv`, at `sample_rate_hz` = 100 Hz.
  - `output_dir` defaults to `resolve_log_dir()` (the repository's `franka_logs/`, or
    `$FRANKA_LOGS_DIR`).
  - `experiment_name` defaults to `bag_02_dynamic_test`. `torque_control_stack` starts the logger
    with **no parameters** (`Node(...)` at L943–948), so every run directory carries that name (F-5-23 g).
- `run_manifest.json`: schema `experiment_log/2`, topics, `d_safe`, TCP link, and
  `git {sha, branch, dirty}`. The git fields come from `git` run in the source directory or the
  working directory. In the container install neither is a checkout, and the recorded manifests
  carry `sha: null` (F-5-23 e).
- `utils/run_metadata.write_runtime_metadata` (git SHA, dirty flag and environment, into
  `FRANKA_RUN_METADATA_DIR`) would close that gap, but no current node calls it and nothing sets
  the variable. 297 archived node copies under `sim_runs/*/provenance` do call it; they come from
  other code versions (Inferred: not on this branch's history) (F-5-27, `check_observability.py` 4, 8).
- At shutdown: PNG plots of q, q̇, q̈, τ_effort, τ_cmd, min h, per-link h, min distance, velocity
  filter effect, norms and communication health.

The header is a contract, pinned by `test_experiment_log_columns` and by a golden header
(`test/data/experiment_log_header_94cd7b0.json`, `test_experiment_logger_truth_off_identical`).
Its column groups are:
- joints, nine blocks of seven: `q`, `qdot`, `qddot`, `tau_effort`, `tau_cmd`, `qdot_nom`,
  `qdot_cmd`, `qddot_nom`, `qddot_safe`;
- norms;
- the Cartesian block (`tcp_*`, `tcp_speed`, `tcp_omega`);
- per control point (`cp_*`, from `/cbf/per_link_distances`);
- legacy per link (`min_h…`, `cbf{k}_*`, from `/human_robot/multi_distance`);
- the filter status (`cbf_*`), the monitor (`iso_*`), `tau_sat_*`;
- optional simulation truth (`truth_*`).

### 2.2 How the numbers are formed

Joint state comes from `joint_state_topic` = `/NS_1/joint_states`, the **30 Hz** republisher plus
the **10 Hz finger publisher** (02 §3.8). Vectors are mapped by name when every arm joint is present
(`_vector_from_joint_state`). The logged acceleration is a smoothed finite difference of that
30 Hz velocity:

  (13.1)  q̈_k = α (q̇_k − q̇_{k−1})/(t_k − t_{k−1}) + (1 − α) q̈_{k−1},  α = `accel_lpf_alpha` = 0.35,

on header stamps (with receipt time as the fallback for a zero stamp). (13.1) is an EMA of a
first difference, so its bandwidth is far below the filter's 300 Hz. **It is not the realised
acceleration the filter or the ISO brake-fraction check use** (those come from the 1 kHz state,
F-5-23 b).

The TCP speed is computed from the Jacobian, not by differencing positions:

  (13.2)  v_TCP = ‖J_v(q) q̇‖,  ω_TCP = ‖J_ω(q) q̇‖  at `tcp_link` = `fr3_link8` (`_cartesian`).

**F-5-21: finger messages corrupt about a quarter of the rows (Verified + Runtime (recorded)).**
A finger-only message (`fr3_finger_joint1/2`, all zeros) fails the by-name mapping.
`_vector_from_joint_state` then falls back to **positional** copying, so the logger's
q = [0, 0, NaN, NaN, NaN, NaN, NaN]. q̇ and τ_effort are corrupted the same way, and the Cartesian
block becomes NaN. Each such row is overwritten only by the next 30 Hz arm message.
- `check_observability.py` 1 reproduces it on the production `joint_state_cb`; an arm message in
  the driver's scrambled order is mapped correctly by name.
- In four recorded Gazebo runs, 26.3–26.7 % of rows show exactly q₁ = q₂ = 0 with q₃ and
  `tcp_speed` NaN.
- Consumers that treat 0.0 as data are affected. Example: `analyze_isolation_ab.py` reads
  `qdot_1..7`, and joints 1–2 read **zero** velocity in those rows (Inferred effect).
- NaN-aware consumers (`experiment_summary.py`'s `finite`/`peak`) skip the rows but lose a quarter
  of the samples.
- The 06 commander, 07 filter and 11 monitor are not affected: they map by name and drop
  incomplete messages, or read the 1 kHz `franka/joint_states`.

### 2.3 Liveness semantics

Every `last_*` buffer starts as NaN, so "never received" reads as NaN. After that, `sample()` writes
the **last value with no age check** (F-5-23 a). A filter that dies mid-run leaves `qddot_safe_*`
frozen at its last command in every later row. The CSV cannot tell "constant" from "dead"; only the
`/cbf_status` ages (§4) can, and only if they were recorded. The communication-health
(`comm_success_min`) and ISO stop counters are updated **in the callbacks**, so events shorter
than 10 ms still reach the CSV.

### 2.4 Column meanings that differ from their names

| Column | What it actually is | Pitfall |
|---|---|---|
| `qddot_*` | (13.1) on 30 Hz data | not the realised q̈ |
| `qdot_nom_*`, `qdot_cmd_*` | Pipeline B topics | empty (NaN) in a torque run; the manifest says so |
| `qddot_nom_*` | the commander's **published** command on `/NS_1/qddot_nom` | the commander's own CSV uses `qddot_nom_*` for its ideal open-loop channel (F-3-7), and the commander's publish parameter is named `qddot_safe_topic` while defaulting to `/NS_1/qddot_nom` (F-5-25 b, `check_observability.py` 11) |
| `qddot_delta_norm` | ‖q̈_nom − q̈_safe‖ with the topic above | not against the filter's shaped nominal (F-5-25 c; parallel-session subagent lead, Unverified here) |
| `cbf_*` | `/cbf_status[0..10]` only | the 30 later fields (fallback level, solve time, tick gap, the four ages, the family slacks and violations) never reach the CSV; they survive only in a bag. The 300 Hz status is sampled at 100 Hz, so single-tick events can be missed (F-5-24, `check_observability.py` 5) |
| `min_h`, `cbf{k}_h`, `cbf_violation_count` | `MultiDistance.distance − d_safe` from `/human_robot/multi_distance` (the logger's own `d_safe`, YAML 0.10) | no margins, no zone ladder: **not** the filter's h̄. `cbf_n_violated` is the QP's count of h̄ < 0 (F-5-23 c) |
| `cp_v_obs_max` | ‖v_track‖, raw, over valid entries | unconditioned; `cbf_v_obs_cond` is what the barrier used |
| `cbf_S_p`, `cbf_v_cap_min`, `cbf_v_closing_max` | `/cbf_status[5..7]`: the monitor's values when its message is fresh, otherwise the row builder's | `cbf_S_p` from the builder equals a gap, not a demand (F-5-18) |
| `iso_*` | `/NS_1/iso_safety` | absent (NaN) unless the monitor runs |
| `tau_cmd_*` | `/NS_1/torque_cmd` | excludes gravity (09) |
| `q_*`, `qdot_*`, `tau_effort_*`, `tcp_*` | the last `/NS_1/joint_states` message | F-5-21 |

---

## 3. Bags: recording, replay and reading (Verified)

**Recording.** `torque_control_stack start_rosbag:=true` (default false) starts, at
`rtd_delay + rosbag_delay_s` (2 + 3 s), a recording of 19 topics: raw and aligned depth, colour,
camera info and extrinsics, `/tf`, `/tf_static`, both joint-state topics, `qddot_nom`,
`qddot_safe`, `torque_cmd`, `cbf_status`, `ee_actual`, `ee_desired` and `/cbf/per_link_distances`.
- Files are zstd-compressed per file.
- `sigterm_timeout` is 600 s, because compression runs at shutdown; the code records a truncated bag
  on 2026-09-30.
- **Not recorded (F-5-22):** `/rosout` (CBFDIAG, fallback and ISO messages), `/NS_1/iso_safety`,
  `/NS_1/torque_saturation`, `/NS_1/q_des_state`.

**Replay.** `bag_replay.launch.py` plays raw depth, TF, joint states and `qddot_nom` with `--clock`.
It runs the **shipped** `real_time_distance` and `cbf_safety_filter`, with the parameters built by
the torque stack's own helpers (the launch file is imported), and records their outputs. The replay
is **open loop**: q and q̇ are the recorded ones, so it measures when and how hard the filter acts,
not whether the obstacle would have been avoided (the file says so).

**Reading.** `ball_throw_eval.read_bag` opens `*.db3` through sqlite3, read-only, and deserialises
with `rclpy`. `throw_actuation`, `throw_clearance`, `pass_plot` and `smoothness_report` import it.
It cannot read the zstd files the recorder writes and prints the `zstd -d` command instead
(F-5-22). `compare_vobs.py` and `pipeline_rate_monitor.py` have their own bag paths.

---

## 4. The filter's diagnostic contract: `/NS_1/cbf_status` and CBFDIAG (Verified)

`utils/cbf_diagnostics.STATUS_FIELDS` is the **append-only** schema (v3, 41 fields). Its order is:
1. the historical eleven: `n_rows`, `slack_max`, `fault_braking`, `n_violated`,
   `d_obstacle_min`, the ISO tail `ssm_sp_min` … `iso_stop_latched`, `v_obs_conditioned`,
   `v_obs_hdot`;
2. `fallback_level`, `slew_conflict_joints`, `solve_wall_ms`, `tick_gap_ros_s`;
3. **four ages**: `joint_receipt_age_s`, `distance_capture_age_s`, `distance_receipt_age_s`,
   `constraint_snapshot_age_s`;
4. per-family `slack_*`, `violation_without_slack_*` and `violation_with_slack_*` for the six
   historical families (`obs`, `sc`, `qlim`, `sing`, `cap`, `spd`);
5. `constraint_build_wall_ms`;
6. schema v3: the same three fields for the self-shadow family `ssb`, appended last so that a v2 bag
   still zips against the first 38 names.

NaN means "not measured", not a measured zero. The residuals are evaluated **after** the command is
clipped, each in its own family's units, so they must not be compared across families (module
docstring). A comment on `FAMILIES` forbids adding to it, because the interleaved triples would
shift every `violation_*` column and silently re-interpret every recorded bag. The positional
consumers (`frame_grabber`, `rl_policy_commander`, `experiment_logger`,
`pentagon_qddot_commander`) guard every index with `len(msg.data)` (Verified). `test_iso_status_contract` pins the first eleven.

The four ages are the only recorded measure of how stale the geometry was, and the run CSV does not
hold them (§2.4, F-5-24). The capture age and
the snapshot age differ by the rebuild hold: 50 Hz configured, 35.1 Hz recorded in one sim cell
(08 §4, 09 §4).

**CBFDIAG** (`logging_utils.format_cbf_diag`, called from `_report`) is one CSV-like log line per
`diag_period_s`, emitted only when n_c > 0. It carries the binding row's label, ḣ filtered and raw
("filtered == raw for many ticks means q̇ is FROZEN", which the docstring says diagnosed two
hardware aborts), h_qp, the margins, `qdd_cmd`/`qdd_real`, governor fields and the ISO fields. It
goes to `/rosout` only, which the run bag does not record (F-5-22).

**Brake fraction.** The ISO brake-authority diagnostic and the evidence logger both use the
projection

  (13.3)  f = q̈_realᵀ q̈_cmd / ‖q̈_cmd‖²,  judged only when ‖q̈_cmd‖ > 0.5 rad/s²,

that is, the fraction of the commanded acceleration that was realised along its own direction.
The code chooses the projection over the norm ratio on purpose, and `test_iso_torque_limits` pins
that choice.

---

## 5. ISO evidence

See 11 §6.2–6.3: `iso_evidence_logger` (manifest + 100 Hz CSV + events), `iso_evidence_report`
(13 checks, coverage ≥ 50 % for PASS, D1 = STRUCTURAL). One observability caveat belongs here:
`tau_max_abs` is max|τ| of `/NS_1/torque_cmd`, which excludes gravity, so it is not the joint
torque judged against the effort limits (F-5-23 d). Saturation is reported separately by
`qddot_to_torque` on `/NS_1/torque_saturation`, per joint, even with the ISO flag off
(`test_iso_torque_limits::test_with_the_flag_off_the_saturation_topic_still_reports`).

---

## 6. Visualisation (Verified at interface level)

| Node / module | Shows | Inputs → output | Launched |
|---|---|---|---|
| `trajectory_visualization_node` + `utils/trajectory_trace.py` | red P(s) vs blue actual EE, as RViz markers with the gap in mm | `/NS_1/ee_desired`, `/NS_1/ee_actual` (both from the commander, same tick and FK) → MarkerArray | `start_trajectory_markers` (default **false**), plus `start_rviz` (false) for `rviz2` with `config/trajectory_rviz.rviz` via `launch_support.rviz_config_for_namespace` |
| `trajectory_overlay_node` + `utils/trajectory_overlay.py` | the same two traces, fading, on the scene camera image | the same + RGB image, camera info → Image | `start_trajectory_viz` (default **true**), with a window (`trajectory_overlay_window` true); 3 s delay |
| `capsule_overlay_node` | the FR3 capsules (geometry from `franka_simulation`) projected on the image | `/NS_1/joint_states` (by name), `/human_pose/image` → `/human_pose/capsule_overlay` | unwired (F-0-2) |
| `distance_visualization_node` + `utils/visualization.py` | depth overlay with per-control-point distances and zones | depth, camera info, `/cbf/per_link_distances` (BE) → `/real_time_distance/overlay_image` | no entry point (F-0-4); `visualization.py` is also used inside `real_time_distance` |
| `tag_overlay_node` | the hand-eye start check (tag outline, off-axis, distance) | rectified image, `/detections` → `image_overlay` | `handeye_eye_in_hand_calibration.launch.py` (05) |
| `frame_grabber` | PNG frames, optionally only while "the CBF is active" | colour image, `/NS_1/cbf_status` | unwired (F-0-2) |
| `docs/make_ros_node_graph.py` | hand-laid SVG of the torque-stack graph | none | offline |

**The trajectory overlay doubles as a calibration check** (Verified intent, its docstring). It
projects with `camera_extrinsics.yaml`, the same calibration `real_time_distance` uses for the
robot mask. A blue trace that does not sit on the gripper therefore shows the safety pipeline's
calibration error (cf. F-2-7, Q-1-2).

`frame_grabber`'s "CBF active" is `cbf_status[0] > 0` (`n_rows`) or `fault > 0`. That means "rows
exist", not "a barrier binds" (`[3]`, F-5-23 f).

---

## 7. Inventory of analysis tools

Kinds:
- **live**: subscribes to a running graph;
- **bag**: reads a rosbag2;
- **CSV**: reads `experiment_log.csv`;
- **model**: runs shipped code offline on a simulated arm.

"Moves arm" is from the code. All rows are Verified at interface level (docstring, imports,
I/O calls).

| Script | Kind | Purpose | Writes | Moves arm |
|---|---|---|---|---|
| `experiment_summary.py` | CSV | one-screen digest: moved?, closest gap, barrier effort, saturation, faults; NaN-aware | stdout | no |
| `analyze_isolation_ab.py` | CSV | isolation-test runs, A/B compare of commanded vs realised per joint | stdout | no (affected by F-5-21) |
| `smoothness_report.py` | bag | did the arm move, and smoothly, inside its limits | stdout | no |
| `pass_plot.py`, `throw_actuation.py`, `throw_clearance.py`, `throw_lead.py` | bag (+ truth `.npz`) | per-ball-pass plots: actuation lag and gain, clearance, perception lead time | PNG / `.npz` / stdout | no |
| `ball_throw_eval.py` | bag | `truth` (colour-threshold ball → 3-D, independent of the pipeline) and `score` (row, track, trust, action timing per pass) | `.npz`, stdout | no |
| `compare_vobs.py` | bag | the two obstacle-velocity estimators head to head | `.npz`, PNG | no |
| `bag_to_mp4.py` | bag | video export | MP4 | no |
| `pipeline_rate_monitor.py` | live / bag | actual vs nominal rate and jitter per pipeline stage | CSV, PNG | no |
| `watch_obstacle_tracks.py` | live | per-control-point tracked velocity readout | stdout | no |
| `latency_budget.py` | live + offline | hop-by-hop latency of depth → distance → track → CBF → torque; publishes only probe topics `/lb/*` | stdout | no |
| `approach_probe.py` | live | measurements behind `approach_speed_envelope.md`; read-only per its docstring | stdout | no |
| `range_noise_calibration.py`, `compare_range_noise.py` | live / offline | range-noise constant σ_d and the quadratic noise law | JSON / stdout | no |
| `plot_rl_timing.py` | CSV | `rl_policy_commander` timing figure | PNG | no |
| `cbf_scenarios.py` | model | the shipped `ConstraintBuilder` and QP helpers, closed loop on a double-integrator FR3 with emulated perception | stdout | no |
| `approach_envelope.py`, `ball_closed_loop.py`, `cbf_phase4_check.py` | model (`cbf_phase4_check` also live/bag) | hit/miss envelopes; a recorded throw against the shipped filter; Phase 4 acceptance | JSON / stdout | no |
| `gen_zero_gain_controllers_yaml.py` | config | a hardware controllers YAML with zero PD gains | YAML file | no (its output, if used, changes the controller) |
| `iso_constants_measure.py` | live | T_r, a_s, C, Z_d (11 §6.5) | JSON | **`stop` only**, behind `--i-am-supervising` |
| `iso_preflight_check.py`, `iso_evidence_report.py`, `iso_pfl_speed.py` | config / CSV / calc | 11 §6 | stdout / JSON | no |

**Stale defaults (F-5-26, Verified, `check_observability.py` 10).** Four tools ship defaults that no
longer match the stack, so they must be given arguments:
- `watch_obstacle_tracks` listens to `/cbf/per_link_distances_tracked`, which no module publishes;
- `throw_clearance` loads `<pkg>/rosbag/<BAG>_truth.npz`, but there is no `rosbag/` directory;
- `pipeline_rate_monitor` expects `/NS_1/qddot_safe` and `/NS_1/cbf_status` at 100 Hz, but
  `qp_rate_hz` is 300 (distances at 30 Hz against the 90 Hz camera);
- `iso_constants_measure uncertainty` defaults to `/cams/d455/...`, but the shipped topics are
  `/camera/camera/...`.

**Hidden couplings for a refactor (Verified where stated):**
- `cbf_scenarios.py` re-implements the filter tick around the shipped helpers. Its docstring says
  "the QP assembled exactly as `cbf_safety_filter._qp_tick` assembles it", which makes it a
  **mirror** that a refactor of `_qp_tick` must keep in step (Inferred: not compared line by line).
- `franka_simulation_experiments` imports `cbf_scenarios.py` and `compare_vobs.py` by relative
  source path (F-0-5).
- The scripts are installed to `share/franka_experiments/scripts` (`setup.py` `data_files`), and
  the preflight is run from there; an edit needs a rebuild (lead in PROGRESS §7).

---

## 8. Timing and performance measurement

| Quantity | Where it is measured | Where it is recorded |
|---|---|---|
| per-stage publish rate and jitter | `pipeline_rate_monitor.py` | its CSV |
| per-hop latency | `latency_budget.py` (stamps where they exist, shipped code for the compute hops) | stdout |
| QP solve time, tick gap, geometry ages | `cbf_safety_filter` | `/cbf_status[13..18]` (bag only) |
| constraint build time | `cbf_safety_filter` | `/cbf_status` last v2 field |
| solve-time line | `_report` every 100 ticks | `/rosout` |
| communication success (FCI) | `experiment_logger.robot_state_cb` at 1 kHz | `comm_success_rate`, `comm_success_min` |

**Rate-induced waits** (`latency_budget` adds them analytically, Derived). Two distinct
quantities are involved when a consumer with period T_c polls a producer with period T_p, the two
being asynchronous:

  (13.4)  event → consumption:   wait ~ U[0, T_c],  E = T_c/2,  max = T_c;
          age of the sample in use at a random consumer tick:  ~ U[0, T_p] (plus the producer's
          own latency).

For example, a new distance frame waits up to 20 ms (mean 10 ms) for the 50 Hz rebuild, and the
new snapshot waits up to 3.3 ms for the 300 Hz QP tick: up to 23.3 ms added between perception and
the command. Meanwhile, the snapshot the QP is using is up to 20 ms old (the rebuild period). The
`constraint_snapshot_age_s` field in §4 measures that second quantity directly.

Python `rclpy` timers have no deadline guarantees. `tick_gap_ros_s` is the evidence when the
300 Hz loop slipped. Under simulation time it is measured on the simulation clock (schema
docstring).

---

## 9. Missing or misleading signals: what will bite during debugging and refactoring

| # | Signal | Problem | Evidence | Consequence |
|---|---|---|---|---|
| 1 | logger q, q̇, τ, TCP | finger rows: [0, 0, NaN…] in ~26 % of rows | F-5-21 | per-joint statistics on joints 1–2 biased; a quarter of TCP samples missing |
| 2 | logger values after a channel dies | held, no age | F-5-23 a | a dead filter looks like a constant command |
| 3 | `qddot_*` | 30 Hz EMA difference | F-5-23 b | not comparable with `qdd_real` (CBFDIAG) or the brake fraction |
| 4 | `min_h` / `cbf_violation_count` vs `cbf_n_violated` | two h's, two violation counts | F-5-23 c | a "violation" in one is not a violation in the other |
| 5 | `/cbf_status[5]` S_p | monitor's or builder's depending on freshness; the builder's is a gap | F-5-18 | ISO margins computed from it are wrong when the monitor is off |
| 6 | run bag | no `/rosout`, `/iso_safety`, `/torque_saturation`, `/q_des_state` | F-5-22 | stops, fallbacks and saturation not reconstructable from the bag |
| 7 | bag format | zstd vs `read_bag`'s `.db3` | F-5-22 | manual decompression before every analysis |
| 8 | provenance | `git.sha` null in container runs; `run_metadata` unused | F-5-23 e | a run cannot be tied to a commit |
| 9 | run naming | every run named `bag_02_dynamic_test` | F-5-23 g | runs distinguishable only by timestamp |
| 10 | `tau_max_abs`, `tau_cmd_*` | gravity excluded | F-5-23 d | not the joint torque |
| 11 | `frame_grabber` "active" | `n_rows > 0` | F-5-23 f | not "a barrier binds" |
| 12 | velocity stack wrapper log | would print the timeouts `[ACTIVE]` (dormant) | F-5-16 | a log line contradicting F-5-7 if ever wired |
| 13 | monitor ↔ filter fault flag | no timestamp; latch loop | F-5-17, F-5-19 | an ISO stop cannot be cleared |
| 14 | CSV `cbf_*` | only `/cbf_status[0..10]` of 41 | F-5-24 | fallbacks, solve times and ages need a bag |
| 15 | `qddot_nom_*` naming | the same name means two different signals; a parameter named `qddot_safe_topic` publishes the nominal | F-5-25 | the wrong channel compared across CSVs |
| 16 | tool defaults | four tools point at topics or paths that do not exist | F-5-26 | a silent empty readout |

---

## 10. Practical example: reading a recorded Gazebo run

Take `sim_runs/leakfix_20261106/logger/20261106_093800_bag_02_dynamic_test/`
(Runtime (recorded); read-only, with `check_observability.py` 1).
1. `experiment_log.csv` has 3903 rows at 100 Hz, about 39 s.
2. In 1030 of them (26.4 %), q₁ = q₂ = 0, q₃…q₇ = NaN and `tcp_speed` = NaN: these are the finger
   rows. Drop rows where `q_3` is NaN **before** any per-joint statistic. Dropping only NaN cells
   would keep the zeros of joints 1–2.
3. `run_manifest.json` says `joint_state_topic: /NS_1/joint_states` and `git.sha: null`, so the
   code revision must be taken from the run's own provenance (12, Stage 6) or from the date.
4. For what the barrier did, read `cbf_n_rows`, `cbf_n_violated`, `cbf_slack` and `cbf_fault`, not
   `min_h`.
5. For the solve times and the geometry ages you need the bag. These are not in the CSV.

---

## 11. Consequences of modification

| Change | Effect |
|---|---|
| Logger: subscribe to `/NS_1/franka/joint_states`, or skip incomplete messages | fixes F-5-21 and turns (13.1) into a 1 kHz difference (then re-tune α); the golden header is unaffected |
| Logger: per-channel age columns or NaN after a timeout | F-5-23 a; changes the header, so regenerate the golden (`test_experiment_logger_truth_off_identical`, `test_experiment_log_columns`) |
| `/cbf_status` | append only. A new field goes at the end and updates `STATUS_FIELDS`; never insert into `FAMILIES` (it would re-interpret every bag). Positional consumers: `frame_grabber`, `rl_policy_commander`, `experiment_logger`, the commander. Pinned by `test_iso_status_contract` |
| Bag topic list | adding `/rosout` and the ISO topics closes F-5-22; the replay and `read_bag` need no change |
| `_qp_tick` refactor | `cbf_scenarios.py` (and through it `franka_simulation_experiments`) mirrors it |
| Overlay / calibration | the trajectory overlay shares `camera_extrinsics.yaml` with perception; changing one without the other hides a calibration error |
| Tests pinning this chapter | `test_experiment_log_columns`, `test_experiment_logger_truth_off_identical` (golden header at 94cd7b0), `test_iso_status_contract`, `test_run_metadata`; `test_control_point_derivatives` and `test_kinematics_fix_regression` test ch. 04/05 geometry and are listed there |

## 12. Findings from this chapter

| ID | Short title | Section | Reproduced by | Still needs |
|---|---|---|---|---|
| F-5-21 | finger messages corrupt ~26 % of logger rows | §2.2 | `check_observability.py` 1 (+ recorded CSVs) | — |
| F-5-22 | run bag lacks `/rosout` and the ISO topics; zstd vs `.db3` | §3 | `check_observability.py` 3 | — |
| F-5-23 | misleading logger diagnostics (a–g) | §2–§6 | `check_observability.py` 2, 4 | — |
| F-5-18 | two meanings of `/cbf_status[5]` | §4 | `check_iso.py` 9 | — |
| F-5-24 | the CSV holds only `/cbf_status[0..10]` | §2.4, §4 | `check_observability.py` 5, 9 | — |
| F-5-25 | misleading logger columns (`min_h`, `qddot_nom_*`, `qddot_delta_norm`, run name) | §2.4 | `check_observability.py` 6, 7, 11 | (c) unverified |
| F-5-26 | stale tool defaults | §7 | `check_observability.py` 10 | — |
| F-5-27 | `write_runtime_metadata` uncalled; archived copies call it | §2.1 | `check_observability.py` 4, 8 | origin of the copies |
