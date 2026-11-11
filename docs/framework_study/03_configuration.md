# 03 — Configuration: where every number comes from

> **Verified against** HEAD `ba68077`, 2026-11-11. This chapter is based on static analysis. The
> cross-checks in §5 were computed with read-only Python over the YAML files and the source.
> Nothing was launched.
> **What you get from it.** Given any parameter of any node, you should be able to say which file
> supplied its value in a given run, which mechanism can override it, and what else reads the
> same key.

---

## 1. Intuition — why configuration deserves a chapter

A safety filter is only as correct as its constants: `d_safe`, the HOCBF gains, the rates, the
limits. In this repository a single run's numbers come from:
- **six** YAML files;
- **two** installed-versus-source copies of each;
- launch-argument defaults that come from **two** more YAMLs;
- runtime rewrites of the perception config;
- generated controller files;
- in simulation, a further layer of experiment conditions.

The framework's own design rule is stated in `utils/params.py::declare_from_spec`: **"the YAML
is the ONLY source of truth"**. Node-side defaults were removed after they had drifted from the
YAML: `qp_rate_hz` read 200 in the node against 100 in the YAML, and `d_safe` read 0.2 against
0.15. This chapter checks how far that rule actually holds, and finds that the YAML is the
source of truth **only for keys that no launch file overrides** (§5.1).

## 2. Foundations — a precedence model

Each value a node ends up using is a function of several layers. Write `v(k)` for the effective
value of key `k`, and list the layers from lowest to highest priority. The **last layer that
defines `k` wins.**

**Launch-argument defaults** (all hardware launch files; `torque_control_stack.launch.py` L104–106):

    v_arg(k) = CLI `k:=…`  ▷  franka_bringup/config/franka.config.yaml [ROBOT1]  ▷  launch_defaults.yaml  ▷  hard-coded default in DeclareLaunchArgument

Here `a ▷ b` means "a if defined, else b". The bringup file sits **above**
`launch_defaults.yaml` because of `{**_LAUNCH_DEFAULTS, **_BRINGUP_DEFAULTS}`. It affects six
keys: `arm_id`, `robot_ip`, `use_fake_hardware`, `namespace`, `fake_sensor_commands` and
`load_gripper` (`utils/config.py::load_franka_config_defaults`).

**A `cbf_safety_filter` parameter** (`utils/config.py::load_cbf_config` →
`utils/params.py::declare_from_spec`):

    v(k) = ROS parameter override from the launch  ▷  installed share/franka_experiments/config/fr3_control.yaml [params][k]
    and v(k) must satisfy the bounds in CBF_PARAM_SPEC[k], or the node raises at construction.

- A key that is in `CBF_PARAM_SPEC` but missing from the YAML is a **startup failure** (a
  `KeyError` that names the key).
- An override for a key the node never declares is **silently ignored**. That is standard
  rclpy behaviour without `allow_undeclared_parameters` (Inferred; F-1-12).

**In simulation** (`simexp/conditions.py` docstring and `resolve`), the launch arguments
themselves are produced by:

    stack_arg(k) = --set override  ▷  condition's own `cbf:` block  ▷  global `cbf:` block in conditions.yaml

Keys without a launch argument go through `cbf_params:` → the single launch argument
`cbf_param_overrides` → ROS parameters on `cbf_safety_filter`.

**Installed-copy rule.** Every node reads `get_package_share_directory('franka_experiments')/config/…`,
the **installed** copy. `setup.py` installs `config/*.yaml` through `data_files`, which copies
the files unless the package is built with `--symlink-install`. So an edit to the source YAML
has no effect until the package is rebuilt, and an edit that adds a key the node requires makes
the node die at startup until then. The error message in `declare_from_spec` says exactly this.

## 3. Implementation — the files and their readers

### 3.1 Files in `franka_experiments/config/`

| File | Size | Read by (Verified) | Role |
|---|---:|---|---|
| `fr3_control.yaml` | 2,672 lines | `cbf_safety_filter` (via `load_cbf_config`), `qddot_to_torque`, `pentagon_qddot_commander`, `rl_policy_commander`, `cbf_velocity_filter`, `cbf_OSCBF_filter`, `trajectory_*_node`, `iso_*`, `real_time_distance` (`perception:` only), offline scripts and `simexp` | **Topics**, the **CBF parameter block** (280 keys under `params:`), `perception:` (2 keys), `joint_limits:` (7 rows) |
| `fr3_complete.yaml` | 930 lines | `real_time_distance` (`robot_config_path`), `utils/kinematics.py`, simulation truth geometry, scripts | Perception: robot segments, meshes, depth topics, distance thresholds, mask, zones, tracking, `sim_obstacle` |
| `launch_defaults.yaml` | 332 lines | `utils/config.py::load_launch_defaults` → every hardware launch file; also `scripts/cbf_scenarios.py` and `simexp/sim_config.py` | **Defaults of the launch arguments**: 89 keys |
| `fr3_distance.yaml` | 23 lines | `cbf_velocity_filter` only (`load_robot_config('distance')`) | joint and segment names, `multi_distance` topic |
| `camera_extrinsics.yaml` | 11 lines | `real_time_distance` (projection), overlays, launch files (static TF in `minimal`, `thales` and the velocity stack), scripts | `base → camera_color_optical_frame` |
| `camera_link_extrinsics.yaml` | 16 lines | `torque_control_stack.launch.py` only (static TF) | `base → camera_link` |
| `camera_EE_extrinsic.yaml`, `dataset_handeye_eye_in_hand.yaml`, `ee_tag_extrinsics.yaml` | — | hand-eye calibration nodes and launch files | calibration inputs and outputs (chapter 05) |
| `rgb_intrinsics.yaml` | — | `capsule_overlay_node` | colour intrinsics |
| `depth_intrinsics.yaml` | — | `simexp/sim_config.py` (simulated camera) | depth intrinsics |
| `*.rviz` | — | launch files (`trajectory_rviz.rviz` through `launch_support.rviz_config_for_namespace`) | visualisation |
| `oscbf_params.yaml`, `fake_hw_controller_params.yaml`, `camera_intrinsics.yaml` | 63 / 7 / 56 lines | **no reader** (Verified: matched only by a generic pattern in `utils/config.py`) | dead; consistent with `LEGACY.md` |
| `safety/watchman_profile.md` | — | documentation | chapter 11 |

### 3.2 Loaders (`utils/config.py`)

| Function | What it reads | Fallback behaviour |
|---|---|---|
| `load_launch_defaults()` | `share/config/launch_defaults.yaml`; values turned into strings (bool → `'true'`/`'false'`) | **never raises**: prints a WARN and returns `{}` |
| `load_franka_config_defaults('ROBOT1')` | `franka_bringup/config/franka.config.yaml` | never raises; hard-coded defaults |
| `load_package_config(name)`, alias `cbf_utils.load_robot_config` | `share/config/fr3_<name>.yaml` | raises |
| `load_package_yaml(pkg, rel)` | any share-relative YAML (duplicates the above for `fr3_*`; `TODO[LEGACY]`) | raises |
| `load_config_file(path)`, alias `distance_utils.load_robot_config` | an explicit path (used by `real_time_distance`) | raises |
| `load_cbf_config(node)` | `fr3_control.yaml` → `declare_from_spec(cfg['params'], CBF_PARAM_SPEC)`; attaches `CBF_RAW_KEYS` and `config_path` | raises naming the missing keys |
| `load_franka_joint_limits()` | `franka_description/robots/fr3/joint_limits.yaml` | — |
| `load_extrinsics(path)` | an extrinsics YAML → `(R, t)` | — |

**Two names, two different functions.** `load_robot_config` means two different things depending
on the module it is imported from:
- in `cbf_utils` it takes a **name**;
- in `distance_utils` it takes a **path**.

### 3.3 The CBF parameter block, as data

- `utils/config.py::CBF_PARAM_SPEC` (L385–766) maps **260 keys** to `(kind, bounds)`. The values
  live in `fr3_control.yaml → params:`, which has **280** keys.
- Three keys are read raw, without being declared as ROS parameters: `CBF_RAW_KEYS =
  (self_collision_safety_distance, self_collision_exclude_pairs, qddot_accel_limits)`.
- Bounds sit in the spec and not in the YAML on purpose: "a launch file must be able to retune a
  gain, not to widen the range a gain is allowed to take".
- Every key becomes a real ROS parameter, so `ros2 param get /cbf_safety_filter d_safe` works and
  the logger can freeze them.

### 3.4 Launch-time rewriting of the perception configuration

`torque_control_stack.launch.py::_rtd_config_with_overrides` **copies** `fr3_complete.yaml` to
`$TMPDIR/fr3_complete_launch_<pid>.yaml` whenever any of the following holds, and points
`real_time_distance` at the copy:
- `obstacle_tracking` is true, which forces `tracking.enabled = true`;
- `sim_obstacle` is true;
- a depth rate can be parsed from `camera_depth_profile`, which sets `distance.depth_rate_hz`;
- `visualize` is forced.

With the shipped defaults (`camera_depth_profile: 848x480x90`, `obstacle_tracking: true`) **the
copy is always made**. The node never reads the installed file directly in a default hardware
run. The inline comment says the copy goes "into the log directory"; the code writes to
`tempfile.gettempdir()` (a minor documentation mismatch).

In simulation, `gz_cbf_stack` does the equivalent itself (`sim_config.perception_yaml`):
`fr3_complete.yaml` deep-merged with the condition's `perception:` block and the scenario's, and
written into the run directory.

### 3.5 Generated controller configuration

- **Hardware.** `pick_controllers_yaml('__auto__', use_fake, rt_params)` →
  `generate_rt_controllers_yaml(...)` → a temporary file passed to `franka_bringup` as
  `controllers_yaml`.
- **Simulation.** `sim_config.rt_controllers_yaml(run_dir)` calls the same generator with
  `is_real=True, gazebo='true'`. The file is given to the Gazebo plugin.
- **Contents** (Verified in `_generate_torque_yaml`):
  - `update_rate` 1000 (real) or 100 (fake);
  - `thread_priority: 85` (real);
  - `d_gains = [30, 30, 30, 25, 10, 10, 5]` and `p_gains = [120, 120, 120, 100, 40, 40, 20]`,
    **both zeroed on fake hardware**;
  - `e_max: 1.0`, `p_max: 0.15`, `command_timeout: 0.1`, `qdot_margin: 0.95`;
  - `accel_topic: /NS_1/qddot_safe`;
  - plus `lpf_alpha`, `tau_max_scale` and `urdf_path`, which the C++ does not declare and
    therefore ignores (F-1-8).
- The generator's comments mark these gains as "INITIAL — to be validated on hardware".

### 3.6 Simulation configuration (`franka_simulation_experiments/config/`)

| File | Content | Reader |
|---|---|---|
| `conditions.yaml` | global `cbf:` block (15 flags, pinned equal to `launch_defaults.yaml` by `test/test_conditions.py`), `cbf_yaml_only:` (`[enable_sensor_range_uncertainty]`), and **23 conditions** | `simexp/conditions.py::resolve` |
| `conditions.d/mppi_it08.yaml` | one extra condition with a `shaper:` block | **no reader at HEAD** (Verified: `conditions.load` reads `conditions.yaml` only; `shaper` and `mppi` appear nowhere in code) (F-1-13) |
| `scenarios.yaml` | `q0`, `flange_q0`, `catalog`, `pentagon`, `throw_sweep` (75 scenario names per the README; counted in Stage 6) | `simexp/scenario_catalog.py` |
| `control_regression_scenarios.yaml` | regression set | read by `test/test_control_regression.py` (no reader among the `simexp` modules was found) |

The 23 conditions fall into these groups:
- **Perception and estimator axis:** `residual`, `kf`, `kf100`, `imm`, `imm+pred`.
- **Ablation ladder:** `full`, `minimal`, and `minimal_l1*` (14 variants).
- **Other:** `imm_pred_l1`, `imm_pred_l1_evid`.

19 of them pin `qp_rate_hz: 100` (chapter 01 §2).

## 4. Integration — the configuration of one hardware run, end to end

The sequence for `ros2 launch franka_experiments torque_control_stack.launch.py` with no
arguments (Static):

1. **Launch composition** (in the `ros2 launch` process).
   - Read `launch_defaults.yaml` and `franka.config.yaml`.
   - Declare the launch arguments: the 78 names in `_ALL_PARAMS`, declared partly in the file
     and partly by `declare_robot_args` and `declare_rt_torque_args`.
   - The ISO preflight runs only if `iso_enabled` is set.
2. **Controller YAML.** A temporary file from `_generate_torque_yaml` goes to `franka_bringup`.
3. **`real_time_distance`.** Parameters `robot_config_path` (the temporary copy of
   `fr3_complete.yaml`, §3.4), `camera_extrinsics_path`, `multi_obstacle_k = 2` (from
   `launch_defaults`; the YAML's 1 is overridden) and `publish_overlay_image`. The node also reads
   `fr3_control.yaml → perception:` itself (`control_config_path`, default the share copy).
4. **`cbf_safety_filter`.**
   - The base layer is the installed `fr3_control.yaml → params` (260 declared keys).
   - Then come the ROS overrides from `_cbf_parameters(p)`:
     - `obstacle_velocity_source`, the eight `enable_*` flags, the four `iso_*` keys and
       `obstacle_input_rate_hz = 90` (parsed from the profile);
     - `obstacle_velocity_normal_rot_max` (0.15 or 0.0) and `obstacle_velocity_identity_jump`
       (0.10 or 0.0), which turn a launch **bool** into a threshold;
     - optionally `link_speed_max`, `retreat_cap_max_speed` and `cbf_param_overrides`;
     - `robot_config_path`, which the node **never declares**, so it is ignored (F-1-12).
5. **`qddot_to_torque`.** No launch parameters. It reads `fr3_control.yaml` itself.
6. **Commander.** One launch parameter (`isolation_test`). It reads `fr3_control.yaml` for:
   - topics and `joint_limits`;
   - `params.path_*`, which set the path type, size and cycle time;
   - `params.d_safe`, read **directly from the installed YAML** (L357–358) to scale its phase
     governor (`governor_r_full`, `governor_r_stop`).

## 5. Cross-checks: where the "single source of truth" does not hold

### 5.1 The shipped YAML describes a different filter from the one the launch runs

Comparing `fr3_control.yaml → params` with the values `torque_control_stack` injects by default
(from `launch_defaults.yaml`) gives:

| Key in `fr3_control.yaml` | YAML value | Value in a default launch | Source of the override |
|---|---|---|---|
| `obstacle_velocity_source` | `residual` | **`tracker`** | `launch_defaults.obstacle_velocity_source` |
| `enable_lateral_evasion` | false | **true** | `lateral_evasion` |
| `enable_livelock_escape` | false | **true** | `livelock_escape` |
| `enable_uncertainty_margin` | false | **true** | `uncertainty_margin` |
| `enable_zone_ladder` | false | **true** | `zone_ladder` |
| `enable_vobs_in_hdot` | false | **true** | `vobs_in_hdot` |
| `enable_velocity_standoff` | false | **true** | `velocity_standoff` |
| `obstacle_velocity_normal_rot_max` | 0.0 | **0.15** | `obstacle_velocity_normal_guard: true` |
| `obstacle_velocity_identity_jump` | 0.0 | **0.10** | `obstacle_identity_guard: true` |
| `enable_latency_compensation`, `enable_outrun_evasion`, `iso_*` | equal | equal | — |
| `fr3_complete.tracking.enabled` | false | **true** | `obstacle_tracking: true` (rewritten copy) |
| `fr3_control.perception.multi_obstacle_k` | 1 | **2** | `multi_obstacle_k: 2` |

**Derived consequence.** Reading `fr3_control.yaml` alone gives a wrong picture of the shipped
behaviour: **nine filter mechanisms that the YAML marks as off are on in every default launch.**
The same holds for anything that reads the YAML without applying `launch_defaults.yaml`. The
tests built on `test/_cbf_builder_harness.py` and the offline scripts need to be checked one by
one in later stages. `scripts/cbf_scenarios.py` and `simexp` do read `launch_defaults.yaml`
(F-1-11).

### 5.2 Keys in `fr3_control.yaml → params` that no code reads

- 20 of the 280 keys are not in `CBF_PARAM_SPEC`.
- 14 of those have another reader:
  - path and governor keys → commander;
  - `cbf_activation_margin`, `cbf_hysteresis` → velocity filter;
  - `control_rate_hz` → velocity filter, OSCBF and RL;
  - the 3 `CBF_RAW_KEYS` → the filter.
- **6 have no reader in `franka_experiments`:** `d_safe_default` (documented as an "alias kept for
  backward compatibility"; only a *field* of the same name exists in `utils/cbf_constraints.py`),
  `distance_ema_alpha`, `ema_alpha`, `gamma`, `max_tau_delta` and `qp_smooth_weight`. This agrees
  with `LEGACY.md` (which lists five of them).

### 5.3 Launch defaults with no consumer

Of the 89 keys of `launch_defaults.yaml`, two are referenced by no launch file, launch helper,
test launch file or `simexp` module: `start_human_pose` and `human_pose_delay_s`.

### 5.4 Two sources of joint limits

| Consumer | Source |
|---|---|
| `cbf_safety_filter` | `franka_description/robots/fr3/joint_limits.yaml` (`load_franka_joint_limits`), with accelerations capped by `params.qddot_max_abs` |
| commander, RL commander, velocity filter, OSCBF filter | `fr3_control.yaml → joint_limits:` (columns `[q_min, q_max, qdot_max, deceleration_limit, effort]`) |

The source comment in the filter says this split is intentional ("the two can drift"). Whether
they currently agree is checked in Stage 4.

### 5.5 Parameters that are set but have no effect

| Parameter | Set by | Why it is inert |
|---|---|---|
| `lpf_alpha`, `tau_max_scale`, `urdf_path` | controller YAML generator and launch arguments | not declared by `RtTorqueController` (F-1-8) |
| `robot_config_path` | the torque launch, passed to `cbf_safety_filter` | not declared by the node (F-1-12) |
| `start_move_group:=true` (default) | torque launch | starts `move_group`, which no Pipeline A node uses (F-1-5) |
| `d_safe` set through `cbf_param_overrides` | simulation conditions or the CLI | reaches the filter, but **not** the commander's governor, which re-reads `d_safe` from the installed YAML (Derived; F-1-10) |

## 6. Practical examples

### 6.1 Trace `d_safe` (the safety distance)

1. **Value.** `fr3_control.yaml → params.d_safe: 0.10`, measured from the capsule surface
   according to the inline comment (chapter 05 verifies this).
2. **Validation.** `CBF_PARAM_SPEC['d_safe'] = ('float', minimum 0.0, maximum 2.0)`.
3. **Declaration.** `declare_from_spec` → `declare_float(node, 'd_safe', 0.10, …)`, which becomes
   the live ROS parameter `/cbf_safety_filter:d_safe`.
4. **Possible overrides:**
   - none from the hardware launch, because `d_safe` has no launch argument;
   - `cbf_param_overrides:="d_safe=0.12"` from the CLI or a simulation condition;
   - with `iso_enabled`, `_iso_configure(P)` can raise it to the floor `C + Z_d + Z_r` before
     anything reads it (chapter 11).
5. **Other readers of the same key:**
   - the commander's phase governor (installed YAML only);
   - `simexp`'s start-clearance check (`gz_cbf_stack.launch.py::_resolve_scenario` reads the
     installed YAML);
   - `ground_truth_node`, which receives it as a parameter;
   - `cbf_velocity_filter`.

   A run that overrides `d_safe` therefore changes the filter, but not the governor, the start
   clearance or the truth labelling.

### 6.2 Trace `obstacle_tracking:=true` (one switch, three effects)

This argument is the default in `launch_defaults.yaml`. It has three effects:
- the launch rewrites the `fr3_complete` copy with `tracking.enabled: true`, so
  `real_time_distance` runs clustering and the Kalman/IMM tracker;
- on its own, that only adds velocity fields to `LinkDistance`;
- the launch help text states that the filter uses them only when
  `obstacle_velocity_source:=tracker`, which is also the default.

So two keys in two files, both overridden by the launch, decide whether the filter sees a
tracked obstacle velocity. In simulation the condition sets both explicitly, for example
`kf: {obstacle_tracking: true, obstacle_velocity_source: tracker, perception.tracking.enabled: true}`.

### 6.3 Reproduce a simulation arm by hand

The condition `minimal_l1` shows how an ablation arm is built:
- its `cbf:` block turns off eight launch-level mechanisms, keeping `vobs_in_hdot: true`;
- its `cbf_params:` sets 33 YAML-level keys, for example
  `obstacle_slack_mode: row_l1, qp_rate_hz: 100, enable_go_around: false,
  state_governor_enabled: false`;
- `resolve` emits these as `cbf_param_overrides="k=v;…"`.

The installed `fr3_control.yaml` is **not** edited, and the next run is unaffected.

## 7. Consequences of modification

- **Adding a filter knob** requires all three of:
  1. a YAML key under `params:`;
  2. a `CBF_PARAM_SPEC` entry;
  3. a rebuild.

  Optionally add a launch argument and a `launch_defaults` key (and then the `conditions.yaml`
  global `cbf:` block too, because `test_conditions.py` pins them equal).
- **Changing a default behaviour.** Decide **which layer** is authoritative. For the nine keys in
  §5.1, editing `fr3_control.yaml` changes nothing in launched runs. Edit `launch_defaults.yaml`
  (and `conditions.yaml`) instead.
- **Changing `d_safe`.** Change it in the YAML, not through overrides, if the governor, the start
  clearance and the truth labelling must follow it (§6.1).
- **Removing a "dead" key.** The dead keys of §5.2 can be deleted from the YAML without breaking
  `declare_from_spec`, which ignores extra keys. Check first:
  - the `franka_rl`/`franka_sim` mirror, which reads `fr3_control.yaml` according to
    `utils/rl_policy.py` (interface only, not verified here);
  - the scripts.
- **Editing extrinsics.** `camera_extrinsics.yaml` and `camera_link_extrinsics.yaml` describe the
  same camera independently (chapter 02 §5). Update both, or neither.

## 8. Findings from this chapter

Logged in [PROGRESS.md](PROGRESS.md):
- F-1-10: `d_safe` is read in several places;
- F-1-11: the YAML and the launch defaults disagree on nine mechanisms;
- F-1-12: the undeclared `robot_config_path`;
- F-1-13: the orphan `conditions.d/`;
- plus the dead keys and files listed in §3.1, §5.2 and §5.3.

## 9. Stage 7 coverage addendum (2026-11-11, C-049): `utils/params.py` read in full

Every function of `utils/params.py` was read at the baseline (`_fail`, `declare_float`,
`declare_int`, `declare_bool`, `declare_str`, `declare_vec`, `declare_from_spec`). (Verified)

- Each helper declares the parameter with its default, reads it back, validates it **once**, and
  returns the value; a failure logs at ERROR and raises `ValueError`, so the node dies in
  `__init__`. `declare_float` rejects NaN/±inf, `positive`, and `[minimum, maximum]`;
  `declare_int` the same without the finiteness test; `declare_bool` does no validation;
  `declare_str` rejects empty strings (unless `allow_empty`) and values outside `choices`;
  `declare_vec` checks the length and finiteness.
- `declare_from_spec` raises `KeyError` naming every spec key missing from the YAML block (the
  message points at the **installed** file, F-7-4), then calls the helper with
  `cast[kind]` applied to `values[name]`, so the YAML value is the default and the spec holds only bounds.
- **F-7-10:** the `description` argument of every helper is never used: no `ParameterDescriptor`
  is passed to `declare_parameter`. The module docstring calls the parameters "self-documenting",
  and the helper docstrings say the text is "shown by `ros2 param describe`"; it is not, and the
  bounds are not registered with ROS either. Validation therefore happens at declaration only.
  Whether a later `ros2 param set` is validated depends on per-node callbacks, which were not
  examined (Unverified).
- Consequence for the refactor: moving the bounds into ROS descriptors would change the declared
  parameter types and ranges seen by `ros2 param describe` and by launch-time overrides; the
  identity tests (`test_torque_stack_external_plant_off_identical`) pin the launch parameter list,
  not the descriptors.
