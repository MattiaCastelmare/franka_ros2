# LEGACY.md — flagged-for-review inventory

**Nothing listed here has been deleted.** This is the input to a later deletion
decision. "Legacy" means **unreachable from `torque_control_stack.launch.py`**
(the stack that runs on the robot), not necessarily "known dead".

Markers in the source use this format:

```
# TODO[LEGACY]: <reason> | confidence: <high|medium|low> | superseded-by: <what|none> | flagged: <YYYY-MM-DD>
```

Regenerate the marker list with:

```bash
grep -rn "TODO\[LEGACY\]" --include=*.py --include=*.yaml .
```

Re-audited **2026-09-10** against HEAD `2debdff` (importers mapped with grep plus
an AST pass over every `from franka_experiments.X import Y`: 0 broken imports).
The first version of this file (2026-09-01, "38 markers") had about a dozen
wrong rows; they are corrected below, and §1 says which.

---

## 1. Source markers — 35, with their verified status

"Status" is the re-audit's verdict, which does not always agree with the marker's
own text. Where it disagrees, the marker is **stale** and should be fixed or
removed before anyone acts on it.

### Nodes

| Path:line | Item | Marker says | Status (2026-09-10) |
|---|---|---|---|
| `nodes/cbf_velocity_filter.py:41` | node | velocity-space mode | **LEGACY** — only `velocity_cbf_control_stack.launch.py` starts it |
| `nodes/ee_pentagon_velocity_commander.py:24` | node | velocity-space mode | **LEGACY** — velocity stack + `test/launch/test_velocity_fake` |
| `nodes/ee_circle_velocity_commander.py:24` | node | velocity-space mode | **DEAD** — referenced only by `setup.py` |
| `nodes/ee_random_waypoints_velocity_commander.py:25` | node | velocity-space mode | **DEAD** — referenced only by `setup.py` |
| `nodes/cbf_OSCBF_filter.py:35` | node | pipeline 2, no launch | **DEAD/LEGACY** — no launch; `test_oscbf_fake.launch.py` never existed |
| `nodes/pentagon_torque_commander.py:28` | node | pipeline 2, no launch | **LEGACY** — started by `test/launch/test_torque_fake.launch.py` only |
| `nodes/experiment_logger.py:77` | `cfg_topics` | built and never read | **Correct** — dead block; its only input besides YAML is `config/fr3_distance.yaml` |
| `nodes/pentagon_qddot_commander.py:106` | `reset_thr_m` | read into `self.reset_thr`, never used | **Correct** |
| `nodes/qddot_to_torque.py:159` | `_on_qddot_nom` | misnomer (carries qddot_SAFE) | **Correct** — naming only, not dead code |

### Utils

| Path:line | Item | Marker says | Status (2026-09-10) |
|---|---|---|---|
| `utils/node_utils.py:11` | module | shim over perception_msgs/logging_utils | **DEAD** — no importer. (The old "would break logging_utils" was backwards: `node_utils` imports `logging_utils`) |
| `utils/cbf_qp.py:2` | module | no importer | **DEAD** — correct |
| `utils/rtd_debug.py:8` | module | no importer | **DEAD** — correct (370 lines of profilers) |
| `utils/cbf_constraints.py:2` | module | no importer | **DEAD** — `cbf_OSCBF_filter` only mentions it in a docstring |
| `utils/cbf_kinematics.py:7` | module | shim → `kinematics.py` | **LEGACY** — importers are `cbf_OSCBF_filter` and `cbf_constraints`, both dead |
| `utils/camera_yaml.py:11` | module | shim → `config.py` | **Correct** — sole importer `capsule_overlay_node` |
| `utils/ros.py:14` | module | facade over node_runtime/launch_support/config | **LEGACY but still needed** — the torque stack no longer uses it; `minimal`, `thales`, velocity stack, 2 test launches, `frame_grabber`, handeye and `rl_policy_commander:88` still do (~13 import sites to repoint first) |
| `utils/simulation_imports.py:16` | module | shim, single consumer | **Correct** — loads *franka_simulation's* `utils.*`, for `capsule_overlay_node` |
| `utils/avoidance.py:32` | module | test-only; governor reverted | **Correct** — only `test/test_avoidance.py` imports it |
| `utils/avoidance_math.py:15` | module | only importer is `simulation_imports` | **STALE MARKER** — `simulation_imports` loads franka_simulation's copy; this package's file is imported only by `utils/ros_setup.py` (itself legacy). Near-copy of `franka_simulation/scripts/utils/avoidance_math.py` (17 differing lines) |
| `utils/self_collision.py:26` | module | imported only by a test | **STALE MARKER — ACTIVE**: imported by `utils/cbf_state_rows.py` (self-collision rows). Remove the marker |
| `utils/cbf_hard_limits.py:198` | `workspace_face_rows` | test-only | **Correct** |
| `utils/config.py:221` | `load_package_yaml` | duplicates `load_package_config` | **Low value** — also used by `pentagon_qddot_commander`, `config.load_cbf_config` and `scripts/latency_budget.py`, not only `cbf_safety_filter` |

### Config files and keys

| Path:line | Item | Status (2026-09-10) |
|---|---|---|
| `config/oscbf_params.yaml:1` | file | **DEAD** — loaded by no code (only READMEs mention it) |
| `config/fake_hw_controller_params.yaml:1` | file | **DEAD** — loaded by no code |
| `config/fr3_distance.yaml:1` | file | **Effectively dead** — loaded by `experiment_logger` (result discarded) and `cbf_velocity_filter` (legacy) |
| `config/fr3_control.yaml:51` | `d_safe_default` | **LEGACY** — read only by `utils/cbf_constraints.py` (dead) |
| `config/fr3_control.yaml:83` | `control_rate_hz` | **LEGACY** — read only by `cbf_OSCBF_filter`, `cbf_velocity_filter` (`rl_policy_commander` reads the *franka_sim* key of the same name) |
| `config/fr3_control.yaml:116` | `ema_alpha` | **DEAD** |
| `config/fr3_control.yaml:126` | `gamma` | **DEAD** (franka_sim `rl.gamma` is unrelated) |
| `config/fr3_control.yaml:321` | `cbf_activation_margin` | **LEGACY** — `cbf_velocity_filter` only |
| `config/fr3_control.yaml:323` | `cbf_hysteresis` | **LEGACY** — `cbf_velocity_filter` only |
| `config/fr3_control.yaml:386` | `qp_smooth_weight` | **DEAD** |
| `config/fr3_control.yaml:418` | `max_tau_delta` | **DEAD** — and its comment is wrong: the QP has no torque row |
| `config/fr3_control.yaml:1720` | `distance_ema_alpha` | **DEAD** — DistanceEngine reads `fr3_complete.yaml` `distance.lpf_alpha` |

### Rows removed from the 2026-09-01 version (they were wrong)

- `max_qddot_delta` "read by no active node" — **live**: `cbf_safety_filter`
  calls `apply_slew_limit(..., P.max_qddot_delta)` when `slew_box_enabled`.
- `cbf_hard_limits.hard_accel_box` / `apply_slew_limit` "test-only" — **both
  live**. The test-only one is **`velocity_accel_box`** (its own docstring says
  SUPERSEDED; only `test_cbf_multi_cp_qp` and `test_cbf_hard_constraints` use it).

---

## 2. Not marked in the source, but legacy or dead

| Item | Status | Evidence |
|---|---|---|
| `nodes/distance_visualization_node.py` | **DEAD** | No `setup.py` entry point, no launch, no test; cannot be started with `ros2 run` |
| `nodes/capsule_overlay_node.py` | LEGACY/OPTIONAL | Entry point exists, no launch; needs franka_simulation built (via `simulation_imports`) |
| `utils/ros_setup.py` | LEGACY | Sole importer `cbf_velocity_filter`; near-copy of franka_simulation's `ros_setup.py` |
| `cbf_utils.select_gamma`, `cbf_utils.skew` re-export | LEGACY | Used only by `cbf_velocity_filter` |
| `cbf_hard_limits.velocity_accel_box` | test-only | See §1 |
| `distance_utils`: `point_to_segment_distance_with_projection`, `compute_closest_distance_from_segments`, `get_robot_segments_from_transforms`, `compute_direction_vector`, `cam_to_base` | **DEAD** | No callers; `define_robot_segments` used only by the dead `distance_visualization_node` |
| `ros.py:selfcheck_run` | **DEAD** | No callers |
| `launch/velocity_cbf_control_stack.launch.py` | LEGACY | Only launcher of `cbf_velocity_filter` + `ee_pentagon_velocity_commander` |
| `launch/thales.launch.py` | LEGACY | rt_velocity_executor bringup + logger + rosbag, no CBF; included by nothing; only user of `start_rosbag`/`rosbag_*` |
| `test/launch/test_velocity_fake.launch.py`, `test_torque_fake.launch.py` | LEGACY | Exercise the legacy commanders |
| `test/scripts/check_topics.sh` branches `velocity`, `torque`, `oscbf`, `oscbf_obstacle` | LEGACY | `oscbf` targets a launch file that never existed |
| `config/camera_intrinsics.yaml` | DEAD | No reader; same content as `rgb_intrinsics.yaml` |
| `test/config/test_defaults.yaml` | DEAD | No reader, although its header says the test launches read it |
| `launch_defaults.yaml`: `start_rviz`, `rviz_delay_s`, `start_human_pose`, `human_pose_delay_s` | DEAD | No launch file references them |
| `setup.py` entry points of every LEGACY/DEAD node above | LEGACY | Go with their nodes |

---

## 3. Deletion units

Things that must go **together** (deleting half leaves broken imports, launches
or tests).

| Unit | Contents |
|---|---|
| **A. Dead, standalone** | `distance_visualization_node.py`; `ee_circle_velocity_commander.py`, `ee_random_waypoints_velocity_commander.py` (+ entry points); `utils/node_utils.py`, `cbf_qp.py`, `rtd_debug.py`; the 5 dead `distance_utils` functions; `ros.py:selfcheck_run`; `oscbf_params.yaml`, `fake_hw_controller_params.yaml`, `camera_intrinsics.yaml`, `test/config/test_defaults.yaml`; keys `ema_alpha`, `gamma`, `qp_smooth_weight`, `max_tau_delta`, `distance_ema_alpha`; the 4 unused `launch_defaults` keys |
| **B. Velocity pipeline** | `cbf_velocity_filter.py`, `ee_pentagon_velocity_commander.py`, `velocity_cbf_control_stack.launch.py`, `test_velocity_fake.launch.py`, `utils/ros_setup.py`, `utils/avoidance_math.py` (1308 lines), `cbf_utils.select_gamma`/`skew`, `test/test_cbf_velocity_filter.py`, `check_topics.sh` `velocity` branch, keys `cbf_activation_margin`, `cbf_hysteresis`, `control_rate_hz` (last user of it), entry points |
| **C. OSCBF / pipeline 2** | `cbf_OSCBF_filter.py`, `pentagon_torque_commander.py`, `utils/cbf_kinematics.py`, `utils/cbf_constraints.py` (+ key `d_safe_default`), `test_torque_fake.launch.py`, `check_topics.sh` `torque`/`oscbf*` branches, "Pipeline 2/3" sections of `test/README.md`, entry points |
| **D. Governor** | `utils/avoidance.py` + `test/test_avoidance.py` |
| **E. Hard-limits leftovers** | `velocity_accel_box`, `workspace_face_rows` + their cases in `test_cbf_hard_constraints.py`; repoint `test_cbf_multi_cp_qp.py` to `position_velocity_accel_box` |
| **F. Logger leftovers** | `experiment_logger` `cfg_topics` block + `config/fr3_distance.yaml` (after B) |
| **G. Commander leftover** | `pentagon_qddot_commander` `reset_thr_m` param (+ YAML/launch entry if any) |
| **H. Capsule overlay** | `capsule_overlay_node.py` + `simulation_imports.py` + `camera_yaml.py` (if the node is dropped) |
| **I. `ros.py` facade** | After B and `thales.launch.py`: repoint `minimal.launch.py`, handeye, `frame_grabber`, `rl_policy_commander`, remaining test launches to `node_runtime`/`launch_support`/`config`, then delete |

**Keep:** every node the torque stack starts (`real_time_distance`,
`cbf_safety_filter`, `qddot_to_torque`, `pentagon_qddot_commander`,
`experiment_logger`), `rl_policy_commander`, `handeye_calibration_node`,
`frame_grabber`, `minimal.launch.py`, `handeye_calibration_bringup.launch.py`,
`test/launch/test_rl_fake.launch.py`, `self_collision.py`,
`hard_accel_box`/`apply_slew_limit`/`position_velocity_accel_box`,
`max_qddot_delta`, `load_package_yaml`, all `scripts/` (all runnable against
the current API).

**Suggested order:** A → B → C → D/E/F/G → I → H (optional) → then fix the
stale markers in §1 and the docs in §5.

---

## 4. Duplicated sources of truth (not legacy, but a risk)

- **Feature flags OFF in `fr3_control.yaml`, ON in `launch_defaults.yaml`:**
  `uncertainty_margin`, `lateral_evasion`, `outrun_evasion`, `livelock_escape`,
  `zone_ladder`, `vobs_in_hdot`, `velocity_standoff`, `obstacle_velocity_source`
  (residual vs tracker), `multi_obstacle_k` (1 vs 2), `obstacle_tracking`
  (`fr3_complete.yaml` `tracking.enabled` false). The two guards
  (`obstacle_velocity_normal_rot_max` 0.15, `obstacle_velocity_identity_jump`
  0.10) are hard-coded in `torque_control_stack.launch.py` — a third source —
  and the launch file's `_DEFAULTS.get(..., 'false')` fallbacks are a fourth.
  Intentional (flags-off builds stay bit-identical), but a run's actual config
  can only be read from the launch, not the YAML.
- **`joint_limits` in 3 copies** (identical today): `fr3_control.yaml`,
  `franka_sim/config.yaml`, `franka_description/robots/fr3/joint_limits.yaml`.
  `cbf_safety_filter` already reads the franka_description one;
  `pentagon_qddot_commander` and `rl_policy_commander` still read `fr3_control`.
- **`d_safe`:** RESOLVED 2026-09-17 — the robot was raised 0.10 → 0.15 to match
  `franka_sim/config.yaml`, and `test_rl_policy::test_real_configs_are_in_sync`
  passes again. NOT yet validated on hardware, and it moved every zone-ladder
  rung with it (`zone_r_*` are multiples of `d_safe`).
  (`experiment_logger` still falls back to 0.20 in code.)
- **Controller gains now have FIVE copies.** `rt_torque_controller.cpp`'s
  `d_gains`/`p_gains` defaults are mirrored into `franka_sim/config.yaml` when
  the actuation-fidelity item lands (backlog P1). Same risk as `joint_limits`.
- **`lpf_alpha`** names two different things: the torque LPF in
  `launch_defaults` (0.3) and the distance LPF in `fr3_complete.yaml` (0.5).
- **Same-name helpers:** `cbf_utils.load_robot_config` = `config.load_package_config`,
  but `distance_utils.load_robot_config` = `config.load_config_file`.

---

## 5. Stale documentation (reported, not edited)

| Doc | Problem |
|---|---|
| `franka_experiments/README.md` | Presents the OSCBF and velocity pipelines as first-class; lists `oscbf_params.yaml` and `fr3_distance.yaml` (the latter described wrongly); says `thales` extends the velocity stack; nothing on tracker / zone ladder / evasion |
| `franka_experiments/test/README.md` | "68 tests" (the suite has ~650); documents the non-existent `test_oscbf_fake.launch.py` |
| Root `README.md` | Config table still lists `oscbf_params.yaml` and `fr3_distance.yaml` as active |
| `config/camera_link_extrinsics.yaml` | Points to `FIX_GUIDE_franka_experiments.md`, which does not exist |

**Resolved 2026-09-14.** Six point-in-time documents were deleted rather than
archived: `CBF_PIPELINE_AUDIT.md` (its findings B1 and B2 are fixed in the
code), `SAFE_RL_CBF_HANDOVER.md`, `franka_sim_to_real_roadmap.md`,
`franka_simulation/refactoring_code.md`, `franka_simulation/implementation_log.md`
and `franka_experiments/ARCHITECTURE.md` (stale as described above, and
regenerable from `torque_control_stack.launch.py`). The handover's gotchas and
backlog were folded into `franka_sim_to_real_implementation_status.md` §8–§9, and
the gravity analysis of `refactoring_code.md §10.2` into
`franka_experiments/test/README.md`. `docs/reactivity_evasion_report.md` is now
linked from the root README documentation map.
