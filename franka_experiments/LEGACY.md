# LEGACY.md — flagged-for-review inventory

**Nothing in this table has been deleted.** Every row corresponds to a
`TODO[LEGACY]` marker present in the source, in this exact format:

```
# TODO[LEGACY]: <reason> | confidence: <high|medium|low> | superseded-by: <what|none> | flagged: <YYYY-MM-DD>
```

Regenerate the list with:

```bash
grep -rn "TODO\[LEGACY\]" --include=*.py --include=*.yaml .
```

"Legacy" here means **unreachable from `torque_control_stack.launch.py`**, not
"known dead". This document is the input to a later deletion decision.

Flagged **2026-09-01** · **38** markers.

| Path | Line | Scope | Item | Reason | Conf. | Superseded by | Would break |
|---|---|---|---|---|---|---|---|
| `config/fr3_control.yaml` | 15 | symbol | `d_safe_default` | read by no active node | **high** | d_safe | `franka_experiments/utils/cbf_constraints.py` |
| `config/fr3_control.yaml` | 21 | symbol | `control_rate_hz` | read by no active node | **high** | qp_rate_hz | `config/oscbf_params.yaml`, `franka_experiments/nodes/cbf_OSCBF_filter.py`, `franka_experiments/nodes/cbf_velocity_filter.py` +1 more |
| `config/fr3_control.yaml` | 34 | symbol | `ema_alpha` | read by no active node; post-QP smoothing was removed | **high** | — | nothing |
| `config/fr3_control.yaml` | 40 | symbol | `gamma` | read by no active node; velocity-era ZCBF gain | **high** | k0_cbf + k1_cbf | `franka_experiments/nodes/cbf_velocity_filter.py`, `franka_experiments/utils/avoidance.py`, `franka_experiments/utils/cbf_constraints.py` +2 more |
| `config/fr3_control.yaml` | 57 | symbol | `cbf_activation_margin` | read only by cbf_velocity_filter, itself flagged legacy | **high** | continuous HOCBF activation + cbf_obstacle_horizon | `franka_experiments/nodes/cbf_safety_filter.py`, `franka_experiments/nodes/cbf_velocity_filter.py`, `test/test_cbf_velocity_filter.py` |
| `config/fr3_control.yaml` | 59 | symbol | `cbf_hysteresis` | read only by cbf_velocity_filter, itself flagged legacy | **high** | continuous HOCBF activation | `franka_experiments/nodes/cbf_velocity_filter.py` |
| `config/fr3_control.yaml` | 79 | symbol | `qp_smooth_weight` | read by no active node | **high** | — | nothing |
| `config/fr3_control.yaml` | 83 | symbol | `max_qddot_delta` | read by no active node — the restored cbf_safety_filter has no slew-rate box (that was a WIP feature reverted in 4d4d450) | **high** | — | `test/test_rl_policy.py` |
| `config/fr3_control.yaml` | 87 | symbol | `max_tau_delta` | read by NO code — the comment above claims it is a QP constraint; the QP has no torque row | **high** | — | nothing |
| `config/fr3_control.yaml` | 120 | symbol | `distance_ema_alpha` | read by no code; DistanceEngine reads lpf_alpha from fr3_complete.yaml instead | **high** | fr3_complete.yaml distance.lpf_alpha | nothing |
| `config/oscbf_params.yaml` | 1 | file | `oscbf_params` | loaded by no code; cbf_OSCBF_filter reads fr3_control.yaml instead | **high** | config/fr3_control.yaml | nothing |
| `franka_experiments/nodes/cbf_velocity_filter.py` | 41 | symbol | `?` | velocity-space control mode; the stack is torque/acceleration-based | **high** | nodes/cbf_safety_filter.py | nothing |
| `franka_experiments/nodes/ee_circle_velocity_commander.py` | 24 | symbol | `?` | velocity-space control mode; the stack is torque/acceleration-based | **high** | nodes/pentagon_qddot_commander.py | nothing |
| `franka_experiments/nodes/ee_pentagon_velocity_commander.py` | 24 | symbol | `?` | velocity-space control mode; the stack is torque/acceleration-based | **high** | nodes/pentagon_qddot_commander.py | nothing |
| `franka_experiments/nodes/ee_random_waypoints_velocity_commander.py` | 25 | symbol | `?` | velocity-space control mode; the stack is torque/acceleration-based | **high** | nodes/pentagon_qddot_commander.py | nothing |
| `franka_experiments/nodes/experiment_logger.py` | 70 | symbol | `cfg_topics` | cfg_topics is built from two YAML files and never read; its keys (joint_states_topic) would not match the parameter names (joint_state_topic) even if wired | **high** | — | `config/fr3_distance.yaml` |
| `franka_experiments/nodes/pentagon_qddot_commander.py` | 105 | symbol | `self` | reset_thr_m is declared and read into self.reset_thr, which is then referenced nowhere; superseded by the two-level soft_reset_thr/hard_reset_thr | **high** | soft_reset_thr + hard_reset_thr | `config/fr3_control.yaml`, `franka_experiments/nodes/capsule_overlay_node.py`, `franka_experiments/nodes/cbf_OSCBF_filter.py` +31 more |
| `franka_experiments/nodes/qddot_to_torque.py` | 151 | symbol | `_on_qddot_nom` | name is now a misnomer — this callback carries qddot_SAFE (the CBF-filtered acceleration), not qddot_nom. Not renamed: ground rule 3 forbids renaming | **high** | — | `franka_experiments/nodes/cbf_safety_filter.py` |
| `franka_experiments/utils/camera_yaml.py` | 11 | file | `camera_yaml` | compatibility shim: load_camera_info_yaml now lives in utils/config.py | **high** | utils/config.py | `franka_experiments/nodes/capsule_overlay_node.py` |
| `franka_experiments/utils/cbf_constraints.py` | 2 | file | `cbf_constraints` | no importer; velocity-era ZCBF rows (b = -gamma*h), not the HOCBF the stack uses | **high** | cbf_safety_filter HOCBF rows | `franka_experiments/nodes/cbf_OSCBF_filter.py` |
| `franka_experiments/utils/cbf_kinematics.py` | 7 | file | `cbf_kinematics` | compatibility shim: CBFKinematics now lives in utils/kinematics.py | **high** | utils/kinematics.py | `franka_experiments/nodes/cbf_OSCBF_filter.py`, `franka_experiments/utils/cbf_constraints.py`, `franka_experiments/utils/kinematics.py` |
| `franka_experiments/utils/cbf_qp.py` | 2 | file | `cbf_qp` | no importer; the CBF filter builds and solves its OSQP problem inline | **high** | cbf_safety_filter inline OSQP + utils/cbf_qp_assembly.py | nothing |
| `franka_experiments/utils/node_utils.py` | 11 | file | `node_utils` | compatibility shim: contents split into utils/perception_msgs.py and utils/logging_utils.py | **high** | utils/perception_msgs.py + utils/logging_utils.py | `franka_experiments/utils/logging_utils.py` |
| `franka_experiments/utils/ros.py` | 14 | symbol | `?` | compatibility facade: split into utils/node_runtime.py, utils/launch_support.py, utils/config.py | **high** | utils/node_runtime.py + utils/launch_support.py + utils/config.py | nothing |
| `franka_experiments/utils/rtd_debug.py` | 8 | file | `rtd_debug` | no importer anywhere in the package | **high** | — | nothing |
| `config/fake_hw_controller_params.yaml` | 1 | file | `fake_hw_controller_params` | not referenced by the active launch graph | **medium** | generated controllers YAML (utils/launch_support.py) | nothing |
| `config/fr3_distance.yaml` | 1 | file | `fr3_distance` | loaded by experiment_logger only, which merges its topics into cfg_topics and never reads the result | **medium** | config/fr3_complete.yaml | nothing |
| `franka_experiments/nodes/cbf_OSCBF_filter.py` | 35 | symbol | `from` | torque-level "pipeline 2"; no launch file starts it | **medium** | nodes/cbf_safety_filter.py | `config/camera_link_extrinsics.yaml`, `config/fr3_complete.yaml`, `config/fr3_control.yaml` +66 more |
| `franka_experiments/nodes/pentagon_torque_commander.py` | 28 | symbol | `?` | torque-level "pipeline 2"; no launch file starts it | **medium** | nodes/pentagon_qddot_commander.py | nothing |
| `franka_experiments/nodes/rl_policy_commander.py` | 53 | symbol | `?` | no launch file starts it since the RL branch was reverted in 4d4d450 | **medium** | none (RL work to KEEP — needs a launch entry point restored) | nothing |
| `franka_experiments/utils/avoidance.py` | 32 | symbol | `?` | imported only by test/test_avoidance.py; the restored commander has no governor | **medium** | none (was pentagon_qddot_commander governor, reverted in 4d4d450) | nothing |
| `franka_experiments/utils/avoidance_math.py` | 15 | symbol | `?` | only importer is utils/simulation_imports.py, itself an import shim | **medium** | — | nothing |
| `franka_experiments/utils/cbf_hard_limits.py` | 39 | symbol | `hard_accel_box` | used only by test/test_cbf_hard_constraints.py; the restored filter uses velocity_accel_box | **medium** | velocity_accel_box (same module) | `test/test_cbf_hard_constraints.py` |
| `franka_experiments/utils/cbf_hard_limits.py` | 82 | symbol | `apply_slew_limit` | used only by test/test_cbf_hard_constraints.py; no slew box in the restored filter | **medium** | — | `test/test_cbf_hard_constraints.py` |
| `franka_experiments/utils/cbf_hard_limits.py` | 109 | symbol | `workspace_face_rows` | used only by test/test_cbf_hard_constraints.py; workspace rows reverted in 4d4d450 | **medium** | — | `test/test_cbf_hard_constraints.py` |
| `franka_experiments/utils/self_collision.py` | 26 | symbol | `?` | imported only by test/test_cbf_hard_constraints.py | **medium** | — | nothing |
| `franka_experiments/utils/simulation_imports.py` | 16 | symbol | `?` | import shim with a single consumer (capsule_overlay_node) | **medium** | — | nothing |
| `franka_experiments/utils/config.py` | 221 | symbol | `load_package_yaml` | duplicates load_package_config for the fr3_*.yaml files (same resolved path) | **low** | load_package_config (same module) | `franka_experiments/nodes/cbf_safety_filter.py` |

## Notes

* **`nodes/rl_policy_commander.py`** is flagged only because commit `4d4d450`
  (the restore to `7c8677d`) removed the `motion_source:=rl` branch from
  `torque_control_stack.launch.py`. The node, `utils/rl_policy.py` and everything
  under `franka_sim/` are intact. This is a **missing launch entry point**, not
  abandoned code — restore the launch branch rather than deleting anything.

* **`utils/ros.py`, `utils/node_utils.py`, `utils/camera_yaml.py`,
  `utils/cbf_kinematics.py`** are Phase-2 compatibility shims, not rotting code.
  They become removable once every import site points at the owning module; the
  active pipeline already does.

* **`utils/cbf_hard_limits.py` is NOT legacy** — Phase 3 gave it a live importer
  (`velocity_accel_box`). Only its three orphaned functions are marked.

* **`params: max_tau_delta`** is the sharpest documentation error found: the
  comment above it claims it is "encoded as QP linear constraint". The QP has no
  torque row at all.

* **`config/fr3_distance.yaml`** is loaded at `experiment_logger.py:92`, but the
  merged result (`cfg_topics`) is never read — so the file has no effect today.

* The dead YAML keys under `params:` have **"Would break: nothing"** because they
  are read by no code. The files listed for other rows are importers of the
  module, i.e. what a deletion would have to be reconciled with.
