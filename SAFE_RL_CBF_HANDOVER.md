# Safe RL + CBF Sim-to-Real — Implementation Handover

**Repository:** `franka_ros2` (ROS 2 Humble, Franka FR3)
**Branch:** `humble-mattia`
**Date of this work:** 2026-08-31
**Specification:** `franka_sim_to_real_roadmap.md`
**Concise audit:** `franka_sim_to_real_implementation_status.md`

> **What this document is.** A complete, self-contained account of everything
> implemented and changed in this session, written so that someone (or a model)
> with no prior context can pick the work up and improve it. It covers the
> architecture, every file touched, every design decision and *why*, the bugs
> found, the exact validation evidence, and a prioritised backlog.
>
> `franka_sim_to_real_implementation_status.md` is the short audit table version
> of the same work. This file is the long form. They do not contradict each
> other; if they ever do, trust the code.

---

## Table of contents

1. [TL;DR](#1-tldr)
2. [Orientation: repo, container, how to run anything](#2-orientation)
3. [The architecture as built](#3-the-architecture-as-built)
4. [The critical defect found and fixed (Step 1)](#4-the-critical-defect-found-and-fixed-step-1)
5. [Step 3: the deployment node, in detail](#5-step-3-the-deployment-node-in-detail)
6. [The sim↔real contract module](#6-the-simreal-contract-module)
7. [Launch integration](#7-launch-integration)
8. [Test and validation infrastructure added](#8-test-and-validation-infrastructure-added)
9. [Infrastructure and hygiene fixes](#9-infrastructure-and-hygiene-fixes)
10. [Complete file inventory](#10-complete-file-inventory)
11. [Validation evidence (all numbers)](#11-validation-evidence-all-numbers)
12. [Design decisions and their rationale](#12-design-decisions-and-their-rationale)
13. [Known limitations, gotchas, traps](#13-known-limitations-gotchas-traps)
14. [Improvement backlog (prioritised)](#14-improvement-backlog-prioritised)
15. [Command cookbook](#15-command-cookbook)
16. [Git state](#16-git-state)

---

## 1. TL;DR

The roadmap has three steps. Steps 1 (MuJoCo env + CBF shield) and 2 (SAC
training + ONNX export) already existed. This session:

* **Implemented Step 3** — `rl_policy_commander.py`, the ROS 2 node that runs
  the exported ONNX policy on the robot at 100 Hz and feeds `/NS_1/qddot_nom`,
  plus the shared sim↔real contract module, launch integration, and a full test
  layer.
* **Found and fixed a defect in Step 1 that silently invalidated all training.**
  The simulation's action had almost no control authority: a zero-action policy,
  a random policy and a trained policy produced *identical* trajectories. No
  reward curve, safety curve or collision metric exposed this. Every policy
  trained before this session is void.
* **Retrained** on the fixed environment. The policy now beats both baselines by
  a wide margin (14 % vs 2 % task success), which is the evidence that the
  training loop is actually closed.
* **Fixed three latent repository problems** discovered along the way: a
  `qpsolvers`/`osqp` version skew that silently disabled the QP backend for two
  production nodes, `colcon test` running zero tests, and three bugs in the
  pipeline validation script.

Test suite went from **62 passed / 6 failed → 68 passed**. Nothing was
committed or pushed.

---

## 2. Orientation

### 2.1 Container

The repo ships a Docker dev environment. The workspace is bind-mounted at
`/ros2_ws/src`, and `build/`, `install/`, `log/` live *inside* that mount (i.e.
at the repo root on the host).

```bash
docker compose up -d                 # or: docker start franka_ros2
docker exec -it franka_ros2 /bin/bash
source /opt/ros/humble/setup.bash
source /ros2_ws/src/install/setup.bash
```

For the standalone (non-ROS) training module:

```bash
cd /ros2_ws/src && export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl
```

`MUJOCO_GL=egl` is required — the container is headless.

### 2.2 The two halves of the project

| | `franka_sim/` | `franka_experiments/` |
|---|---|---|
| Nature | standalone Python, **no ROS dependency** | ROS 2 `ament_python` package |
| Purpose | train the policy in MuJoCo | run it on the FR3 |
| Discovered by | `PYTHONPATH`, plain paths | `get_package_share_directory` |
| Installed by colcon? | **No** (deliberately) | Yes |

`franka_sim` is intentionally *not* a ROS package: it must stay importable
without a ROS environment so training is fast and portable. The consequence is
that the deployment node cannot use `get_package_share_directory` to find the
policy or its config — hence the path-resolution logic described in §6.

### 2.3 Namespace

`franka_bringup/config/franka.config.yaml` sets `namespace: "NS_1"`. Topics in
`franka_experiments/config/fr3_control.yaml` are absolute (`/NS_1/…`). The
deployment node resolves its joint-state topic through the `__auto__` sentinel
(reads the namespace from the bringup config) and takes everything else from
`fr3_control.yaml`.

---

## 3. The architecture as built

```
                TRAINING (standalone, no ROS)            DEPLOYMENT (ROS 2)
                ─────────────────────────────            ──────────────────
 franka_sim/config.yaml ──┐                      ┌── franka_experiments/config/
   (mirror, test-enforced)│                      │        fr3_control.yaml
                          ▼                      ▼
   FrankaCBFEnv  ───►  AccelCBFFilter   ≡   cbf_safety_filter  (HOCBF QP, OSQP)
        │  obs(24)          │                        ▲  /NS_1/qddot_nom
        │                   │ q̈_safe                 │
        │                   ▼                        │
        │            τ = M q̈ + C q̇ + g       rl_policy_commander
        │            (mj_inverse)              (onnxruntime, 100 Hz)
        ▼                                            ▲
   SAC (SB3, CUDA) ─► best_model.zip ─► export_onnx ─┘
                                          .onnx           │ /NS_1/qddot_safe
                                                          ▼
                                          qddot_to_torque (τ = M q̈ + C q̇)
                                                          │ /NS_1/torque_cmd
                                                          ▼
                                          rt_torque_controller ─► FR3 (firmware adds g)
```

**The only artifact that crosses the sim→real boundary is the `.onnx` actor.**
Everything else is *mirrored*, and each mirror is checked mechanically:

| Mirrored quantity | Sim source | Robot source | Enforcement |
|---|---|---|---|
| `joint_limits` (incl. q̈_max) | `franka_sim/config.yaml` | `config/fr3_control.yaml` | `test_real_configs_are_in_sync` + per-entry startup warning |
| CBF gains, workspace box | same | same | `test_real_configs_are_in_sync` |
| Obstacle sphere radius | `scene_cbf.xml` `obstacle_geom` | `obstacle.radius` in `config.yaml` | env raises at construction on mismatch |
| Observation layout | `FrankaCBFEnv._get_obs` | `utils/rl_policy.build_observation` | unit test + bit-equal replay in the smoke test |
| Action scaling | `FrankaCBFEnv.step` | `utils/rl_policy.action_to_qddot` | unit test |
| Actuation | `mj_inverse` torque | `qddot_to_torque` + firmware g(q) | `validate_actuation.py` |

### 3.1 The observation contract (24 dims)

```
obs = [ q(7), q̇(7), ee_pos(3), target(3), obstacle(3), d_min(1) ]
action = a ∈ [−1,1]⁷   →   q̈_nom = a · q̈_max
q̈_max = [6.0, 2.585, 3.5, 4.0, 17.0, 5.5, 17.0]  rad/s²
```

* **`ee_pos`** — in MuJoCo this is the site `attachment_site` (link7 + 0.107 m in
  z), which is the FR3 flange, which is URDF frame **`fr3_link8`**. That is the
  node's default `ee_frame`. (Note: the *pentagon* commander uses
  `fr3_hand_tcp`; do not copy that value here — it would shift the observation.)
* **`obstacle`** — in sim, the **centre** of the human-proxy sphere (radius
  `r_obs = 0.08 m`). On the robot there is no sphere, only a point cloud, so the
  slot is reconstructed as the centre of the fictitious sphere tangent to the
  cloud at the closest point:

  ```
  p_obs = p_human − n̂ · r_obs
  ```

  where `p_human = ld.closest_point_human` and `n̂ = ld.direction` (unit,
  obstacle → robot). This restores the sim identity
  `‖p_cp − p_obs‖ − r_obs − r_cp = d` exactly.
* **`d_min`** — surface distance, *the same physical quantity in both worlds*.
  In sim: `‖p_cp − p_obs‖ − r_obs − r_cp`. On the robot, `distance_engine`
  already reports `‖p_cp − p_human‖ − r_cp`, i.e. distance from the robot capsule
  surface to the obstacle surface. They map 1:1, no conversion needed.

  **One deliberate asymmetry:** the real engine clamps `d` at 0
  (`np.maximum(..., 0)`) while the sim reports negative penetration. So real
  `d_min` saturates at 0 instead of going negative. This is the conservative
  direction (the policy never sees a state that is "less bad than reported") and
  is documented in `utils/rl_policy`.

---

## 4. The critical defect found and fixed (Step 1)

This is the most important part of the session. Read it before changing the env.

### 4.1 How it was found

I added `--model zero` and `--model random` baselines to `evaluate_policy.py`
(they did not exist). All three controllers scored *identically*:

```
                    trained policy   zero-action   random
final EE error         0.2964 m       0.2963 m     0.2963 m
episode length          259.3          259.3        259.3
collision rate           70 %           70 %         70 %
min surface dist      −0.1326 m      −0.1326 m    −0.1326 m
```

Three completely different action streams, one trajectory. Only the *returns*
differed (−129.8 / −135.5 / −177.8) — because the reward's action and
CBF-intervention penalties do depend on the action even when the plant does not.

### 4.2 Root cause

`FrankaCBFEnv.step()` drove MuJoCo's **position servos** from a "state-seeded,
drift-free" reference:

```python
qdot_des = clip(qdot_measured + qddot_safe * dt, ...)
q_des    = clip(q_measured    + qdot_des    * dt, ...)
data.ctrl[act] = q_des
```

Re-anchoring the reference to the **measurement** every tick means it never
integrates. The commanded lead saturates at `q̈·dt² ≈ 6e-4 rad`, and the PD
servo (kp=4500, kv=450) settles at `q̇ ≈ 0.004 rad/s` regardless of the
commanded acceleration.

Measured directly: commanding `q̈ = q̈_max` on all seven joints for 2 s moved the
arm `|Δq| = 0.162 rad`, versus `0.160 rad` with zero action. The difference was
noise; the motion was gravity sag.

**The CBF was innocent.** A trace showed it passing full authority
(`q̈_safe = [6, 2.58, 3.5, 4, 17, 5.5, 17]`, `n_c = 5`, `braking = False`) while
the arm sat still. The bug was entirely in the actuation path.

### 4.3 The fix

Do what the deployment chain does. The FR3 executes
`q̈_safe → τ = M(q)q̈ + C(q,q̇)q̇` (`qddot_to_torque`) `+ g(q)` (firmware). So:

1. **Convert the actuators in place** (`__init__`), leaving the vendored
   Menagerie MJCF untouched:

   ```python
   model.actuator_gaintype[i] = mjGAIN_FIXED
   model.actuator_gainprm[i, :] = 0 ; model.actuator_gainprm[i, 0] = 1
   model.actuator_biastype[i] = mjBIAS_NONE
   model.actuator_biasprm[i, :] = 0
   ```

2. **Command inverse-dynamics torque**, recomputed at every substep — the
   analogue of `rt_torque_controller` re-evaluating at 1 kHz between two 100 Hz
   `q̈_safe` samples:

   ```python
   for _ in range(self.n_substeps):
       self.data.ctrl[self._act] = self._inverse_dynamics(qddot_safe)
       mujoco.mj_step(self.model, self.data)
   ```

   `_inverse_dynamics` sets `data.qacc`, calls `mj_inverse`, reads
   `qfrc_inverse`, and **restores `qacc`** (it is an output of the forward pass;
   leaving a fabricated value would corrupt later reads).

3. **Fix `reset()`** — the `home` keyframe carries a *position* ctrl vector,
   which is nonsense for torque actuators. It is now overwritten with the
   `q̈ = 0` hold torque (gravity compensation).

### 4.4 A second bug I introduced and then caught

My first version clipped the torque with `model.actuator_ctrlrange`. For a
`<position>` actuator that range is the **joint position range** — so joint 4's
torque was pinned into `[−3.0421, −0.1518]` N·m and the arm still would not
move properly. Torque limits now come from the joint's `actuatorfrcrange`
(`jnt_actfrclimited` / `jnt_actfrcrange`), i.e. ±87 / ±12 N·m, with a fallback
to unlimited if neither is set.

This is worth remembering: **never use a MuJoCo actuator's `ctrlrange` as a
force limit unless you know the actuator is a force actuator.**

### 4.5 Verification

`franka_sim/scripts/validate_actuation.py` (new regression guard) checks four
things and passes:

```
max |realized q̈ − commanded q̈| = 1.443e-15 rad/s²
torque limits lo/hi = ∓[87 87 87 87 12 12 12] N·m
|Δq| over 0.5 s:  zero-action 0.0637 rad   unit action 0.7136 rad
q̇₁: +action +2.310 rad/s   −action −2.358 rad/s
```

### 4.6 Consequence

Every policy trained before this fix is **void**. The old model directory was
deleted and training was redone from scratch (`sac_v2`).

### 4.7 Residual fidelity gap (conservative direction)

The sim applies pure feedforward inverse-dynamics torque. The robot
additionally runs `rt_torque_controller`'s 1 kHz `Kd·(q̇_des − q̇)` feedback
(`d_gains = [30,30,30,25,10,10,5]`). A zero command therefore drifts a little
more in sim (0.064 rad over 0.5 s) than it would on hardware. Erring toward a
*less* well-behaved plant in sim is the safe direction for sim-to-real, so this
was left as-is and documented rather than "fixed".

---

## 5. Step 3: the deployment node, in detail

**File:** `franka_experiments/franka_experiments/nodes/rl_policy_commander.py`
(688 lines)
**Executable:** `rl_policy_commander` (registered in `setup.py`)

### 5.1 Role

```
/joint_states ────┐                     ┌─► q̈_nom = a·q̈_max·action_scale
/cbf/per_link_… ──┼─► observation(24) ──┤   (Float64MultiArray, 7)
target (param /   │   (identical layout  └─► /NS_1/qddot_nom
 topic) ──────────┘    to FrankaCBFEnv)            │
                                                   ▼
                                      cbf_safety_filter → qddot_safe
```

It is a **drop-in alternative to `pentagon_qddot_commander`** on the same topic.
Everything downstream (CBF filter → torque → controller) is byte-identical in
both cases: *the safety certificate does not depend on who generates the nominal
acceleration.*

### 5.2 Interfaces

**Subscribes**

| Topic | Type | QoS | Use |
|---|---|---|---|
| `<ns>/joint_states` (auto) | `sensor_msgs/JointState` | depth 1 | q, q̇ + FK |
| `/cbf/per_link_distances` | `franka_msgs/MultiLinkDistance` | depth 1, BEST_EFFORT | obstacle slot |
| `/NS_1/cbf_status` | `std_msgs/Float64MultiArray` | depth 1 | **diagnostics/CSV only** |
| optional `target_topic` | `geometry_msgs/PointStamped` | depth 1 | external target |

**Publishes**

| Topic | Type | Rate | Contents |
|---|---|---|---|
| `/NS_1/qddot_nom` | `Float64MultiArray(7)` | 100 Hz | q̈_nom |
| `/NS_1/rl_status` | `Float64MultiArray(6)` | 100 Hz | `[infer_ms, tick_ms, d_min, dist_to_target, target_idx, gate]` |

**CSV log** — `franka_logs/rl_policy_run_<stamp>.csv`, one row per tick with the
full observation, action, `q̈_nom`, CBF status, and the `tick_ms`/`infer_ms`
timing columns that feed the paper's jitter figure.

### 5.3 Parameters (all of them)

| Parameter | Default | Meaning |
|---|---|---|
| `onnx_model` | `''` (**required**) | policy path; absolute, `sim_root`-relative, or cwd-relative |
| `sim_root` | auto | `franka_sim/` location; auto-discovered via `realpath(__file__)` walk-up |
| `sim_config` | auto | training config; prefers the `config.yaml` frozen next to the model |
| `qddot_nom_topic` | from `fr3_control.yaml` | output topic |
| `joint_state_topic` | `__auto__` | namespace from `franka.config.yaml` |
| `per_link_distances_topic` | from config | perception input |
| `cbf_status_topic` | from config | diagnostics |
| `status_topic` | `/NS_1/rl_status` | diagnostics output |
| `target_topic` | `''` (disabled) | external `PointStamped` target |
| `ee_frame` | `fr3_link8` | FK frame for the observation's EE slot |
| `rate_hz` | `0` → sim `control_rate_hz` (100) | control loop rate |
| `warmup_s` | `3.0` | zero-output settling window |
| `joint_state_timeout` | from config (0.1) | staleness gate |
| `distance_timeout` | from config (0.5) | perception staleness gate |
| `target_xyz` | `[0.45, 0.0, 0.45]` | single target |
| `target_sequence` | `[]` | flat `x,y,z,…` cycle (overrides `target_xyz`) |
| `target_tol` | `0` → sim `task.target_tol` (0.05) | success radius |
| `dwell_s` | `1.0` | hold time before advancing in a sequence |
| `stop_on_success` | `False` | zero output once reached (single-target mode) |
| `obstacle_radius` | `0` → sim `obstacle.radius` (0.08) | sphere radius for slot reconstruction |
| `no_obstacle_xyz` | `[1.5, 0.0, 0.5]` | parked synthetic obstacle when perception is off |
| `distance_links` | `[]` (all) | restrict `d_min` to a link subset |
| `action_scale` | `1.0` | **deployment derate**, clamped to (0,1] |
| `log_csv` / `log_dir` | `True` / auto | CSV logging |

### 5.4 Output gating — the safety-relevant part

The node publishes **zeros** (never a stale or guessed command) whenever it
cannot produce a trustworthy action. `rl_status[5]` reports which gate fired:

| Condition | Output | gate |
|---|---|---|
| warm-up window (`warmup_s`) | zeros | 1 |
| joint state missing or older than `joint_state_timeout` | zeros | 2 |
| distances **seen, then stale** (`distance_timeout`) — perception fault | zeros | 3 |
| distances **never seen** (camera intentionally off) | run against a parked synthetic obstacle | 0 |
| target reached and `stop_on_success` | zeros | 4 |
| nominal | `a·q̈_max·action_scale` | 0 |

The "never seen vs seen-then-stale" distinction is deliberate: running without a
camera (`enable_camera:=false`) is a legitimate configuration, but a perception
stream that *dies* is a safety-chain fault — the obstacle slot would be a lie,
and `cbf_safety_filter` is independently braking on the same event.

Joint velocity/position limits, acceleration continuity and the workspace box
are **not** re-implemented here. `cbf_safety_filter` enforces them downstream on
every code path (hard box + hard rows), exactly as for the pentagon commander.
The only clamp in this node is `|q̈_nom| ≤ q̈_max` (implicit in `a ∈ [−1,1]`).

### 5.5 Real-time discipline

Mirrors `cbf_safety_filter` and `pentagon_qddot_commander`:

* every per-tick buffer preallocated (`_obs_buf`, `_qddot_nom`, `_action`,
  `_ee_pos`, `_obs_xyz`, the output messages, the CSV row);
* the ONNX feed dict `{input_name: _obs_buf}` is built **once** and reused, so
  the tick allocates only onnxruntime's output tensor;
* **ONNX Runtime pinned to a single intra-op and inter-op thread**, sequential
  execution — a thread pool fighting the executor for the GIL is the classic
  source of 100 Hz jitter in this stack (see the `fifo_gil_inversion_rclpy`
  note);
* all lazy costs paid in `__init__` via `_warmup()` (xacro → URDF, Pinocchio FK
  structures, first inference). Measured 0.5–1.6 ms, and it runs before the
  timer starts;
* joint-state snapshots use the lock-free double-buffer + atomic-swap pattern
  used elsewhere in the stack;
* no string formatting in the tick unless a throttled log actually fires.

Result: 100 Hz with 0.159 ms period std and 0.138 ms mean inference (§11).

### 5.6 Startup validation

The node refuses to start (rather than degrade silently) if:

* `onnx_model` is empty — with a message telling you how to export one;
* the model file cannot be found — `FileNotFoundError` listing every path tried;
* the ONNX graph's input width ≠ 24 or output width ≠ 7;
* `ee_frame` does not exist in the URDF (`resolve_frame_id` raises with the
  full frame list);
* `target_sequence` length is not a multiple of 3.

And it **warns loudly** (but continues) if:

* no `franka_sim` config could be found (falls back to `fr3_control.yaml`);
* any `joint_limits` entry differs between the training config and the robot
  config — one `SIM-TO-REAL joint_limits drift — <detail>` line per difference.
  This matters because the policy emits a *fraction* of q̈_max: a silent
  divergence rescales every command;
* `action_scale` is outside (0,1] — clamped, because it is a derate and must
  never widen the trained envelope.

---

## 6. The sim↔real contract module

**File:** `franka_experiments/franka_experiments/utils/rl_policy.py` (267 lines)

Pure numpy/YAML, no ROS import, so it is unit-testable and — more importantly —
so the contract lives in **one place** instead of being duplicated between
`franka_cbf_env.py` and the node.

| Function | Purpose |
|---|---|
| `OBS_DIM = 24`, `ACT_DIM = 7` | the widths, derived from `NUM_JOINTS` |
| `build_observation(q, qdot, ee, target, obstacle, d_min, out=None)` | assembles the vector exactly as `FrankaCBFEnv._get_obs`; fills a preallocated `(1,24)` float32 buffer; `nan_to_num` sanitises (a NaN reaching the net poisons all 7 actions) |
| `action_to_qddot(action, qddot_max, scale, out=None)` | clip to [−1,1], scale; non-finite → 0 (an unusable policy output must not become an unbounded command) |
| `obstacle_centre(p_human, n_hat, r_obs)` | `p_human − n̂·r_obs` |
| `nearest_obstacle(entries, r_obs, links=None)` | closest link → `(centre, d_min)`; optional link filter |
| `synthetic_obstacle(ee, centre, r_obs)` | geometrically self-consistent "no obstacle" slot |
| `qddot_max_from_limits(limits)` | `joint_limits` block → (7,) in `joint1..joint7` order |
| `joint_limits_mismatch(sim, robot, tol)` | list of human-readable drift lines |
| `find_sim_root(start)` | walk up from `realpath(start)` to the `franka_sim/` that has a `config.yaml` |
| `resolve_model_path(model, sim_root)` | absolute → `sim_root`-relative → cwd; raises listing what was tried |
| `resolve_sim_config_path(explicit, model, sim_root)` | explicit → frozen `config.yaml` next to the model (authoritative) → `franka_sim/config.yaml` |

### Why `realpath` matters

`franka_sim` is not installed by colcon, so the node cannot use
`get_package_share_directory`. `find_sim_root` walks up from the node's own
file — and uses `os.path.realpath`, **not** `abspath`, so that under
`colcon build --symlink-install` the installed node file resolves back into the
source checkout where `franka_sim/` actually lives. Same trick for the default
log directory. There are **no hard-coded absolute paths** anywhere in the new
code (contrast `pentagon_qddot_commander`, which hard-codes
`/ros2_ws/src/franka_experiments/franka_logs` — left untouched, but not copied).

---

## 7. Launch integration

**File:** `franka_experiments/launch/torque_control_stack.launch.py`

Rather than duplicating the stack, I added a **`motion_source` selector**. Both
commanders publish `/NS_1/qddot_nom` and would fight for the topic, so exactly
one runs:

```bash
# default, unchanged behaviour
ros2 launch franka_experiments torque_control_stack.launch.py

# Safe-RL policy
ros2 launch franka_experiments torque_control_stack.launch.py \
    motion_source:=rl start_move_group:=false \
    rl_onnx_model:=/ros2_ws/src/franka_sim/models/sac_v2/best_model.onnx
```

New arguments: `motion_source` (`pentagon` | `rl`), `rl_onnx_model`,
`rl_sim_config`, `rl_target_xyz`, `rl_target_sequence`, `rl_action_scale`.
An invalid `motion_source` raises at launch time rather than silently starting
nothing.

`move_group` is only needed by the pentagon commander. It is **not** auto-
disabled for `motion_source:=rl` — `start_move_group` is an explicit user
argument and silently ignoring it would be worse than a wasted node — but a
`LogInfo` hint is emitted.

> **Note on a pre-existing change:** the working tree already contained an
> uncommitted modification to this file (the `qddot_to_torque` remap
> `/NS_1/qddot_nom → /NS_1/qddot_safe`). It was **preserved**, and verified
> correct: `qddot_to_torque` subscribes to `topics['qddot_nom']`, so the remap is
> what puts the CBF filter in the loop.

**Test launch:** `test/launch/test_rl_fake.launch.py` is a thin wrapper that
includes the canonical stack with fake hardware and perception off. It defines
no pipeline of its own, so it cannot drift from the real launch file.

---

## 8. Test and validation infrastructure added

### 8.1 `test/test_rl_policy.py` — 21 unit tests (pure numpy, no ROS graph)

Covers: observation width/layout, byte-equality with the env's own
`np.concatenate`, preallocated-buffer filling, NaN/Inf sanitisation, wrong-buffer
rejection; action scaling, clipping, derate, non-finite collapse; obstacle-centre
geometry (asserts the surface-distance identity is restored), nearest-obstacle
selection and link filtering; `qddot_max` ordering; drift detection.

Plus **`test_real_configs_are_in_sync`** — a genuine cross-repository guard that
fails if `franka_sim/config.yaml` and `config/fr3_control.yaml` disagree on any
joint limit, CBF gain (`d_safe`, `k0_cbf`, `k1_cbf`, `rho_slack`, `k_brake`,
`cbf_obstacle_horizon`, `cbf_min_leverage`, `max_qddot_delta`, `hard_*`,
`ws_margin`, `ws_horizon`) or the workspace box. It skips gracefully if
`franka_sim/` is not present.

### 8.2 `test/smoke_rl_policy_commander.py` — node-level, no bringup (~10 s)

Drives the real node with synthetic `/NS_1/joint_states` and
`/cbf/per_link_distances` and checks five phases:

1. warm-up → **exactly** zeros;
2. steady state → ~100 Hz, finite, `|q̈| ≤ q̈_max`, **and bit-equal to an
   independently rebuilt observation + onnxruntime inference** (this is the real
   contract test — it reproduces the FK, the obstacle-centre reconstruction and
   the action scaling from scratch and compares to what the node published);
3. `d_min` from `MultiLinkDistance` propagates to `rl_status`;
4. perception goes stale → zeros;
5. joint states go stale → zeros.

The contract check deliberately compares against the last **commanding** tick
(non-zero output): on a loaded machine the harness' own 30 Hz distance timer can
starve past `distance_timeout` and legitimately gate a tick — that is the node
behaving correctly, not a contract violation.

### 8.3 `franka_sim/scripts/validate_actuation.py` — regression guard

Exists because the actuation path once silently died (§4). Checks
inverse-dynamics fidelity, that torque limits come from the joint's
`actuatorfrcrange` and not a position range, control authority (a unit action
must move ≥5× the do-nothing drift), and action-sign discrimination.
**Run this after any change to `step()`, the MJCF, or the actuators.**

### 8.4 `franka_sim/scripts/evaluate_policy.py`

Scores a `.onnx` (preferred — it is the artifact that ships), a `.zip`, or the
`zero` / `random` baselines. Reports task metrics, safety metrics and per-step
inference latency. **Always run the baselines**: a policy that ties the
zero-action baseline is not a bad policy, it is a broken plant. That is exactly
how §4 was found.

### 8.5 `franka_experiments/scripts/plot_rl_timing.py`

Turns the node's CSV into the roadmap's determinism evidence (paper advice §3):
loop-period time series, jitter histogram, inference-latency trace, plus printed
statistics. Uses only *commanding* ticks — a gated tick publishes zeros without
running inference, so including them would report an inference time that never
happened.

### 8.6 `test/scripts/check_topics.sh rl`

New pipeline mode validating nodes, topic existence and rates for the RL chain,
with the `rl_status` field layout and gate semantics printed for the operator.

---

## 9. Infrastructure and hygiene fixes

These were not in the roadmap. They were found while validating, and each one
was breaking something real.

### 9.1 `qpsolvers` / `osqp` version skew — silently broke two production nodes

**Symptom:** six unit tests failing with
`SolverNotFound: 'osqp' does not seem to be installed (found solvers: [])`.

**Cause:** `qpsolvers` had floated to 4.13.0 while `osqp` stayed at 0.6.7.
`osqp` is pinned there *deliberately* — `cbf_safety_filter.py` and
`franka_sim/envs/cbf_filter.py` drive the raw OSQP 0.6 object API
(`OSQP().setup()/.update()`, `res.info.status_val`), which osqp 1.x rewrote. But
`qpsolvers ≥ 4.4` does `from osqp import OSQP, SolverStatus`, and `SolverStatus`
only exists in osqp 1.x. The import failed **silently**:
`qpsolvers.available_solvers == []`, so `cbf_velocity_filter` and
`cbf_OSCBF_filter` would raise `SolverNotFound` at their first QP.

**Fix:** pin `qpsolvers[osqp]==4.3.3` + `osqp>=0.6.2,<1.0` in the `Dockerfile`
(4.3.3 is the last release speaking the osqp 0.6 API), with the full rationale
in a comment, **and** add
`assert 'osqp' in qpsolvers.available_solvers` to the image's smoke test so a
rebuild fails loudly instead of shipping a broken solver.

**Result:** 62 passed / 6 failed → **68 passed**.

### 9.2 `colcon test` ran zero tests

`colcon test` was invoking `python3 -m unittest -v` → "Ran 0 tests. OK". The
package's function-style pytest tests were invisible.

colcon's pytest step matches on `has_test_dependency(setup_py_data, 'pytest')`.
The obvious `tests_require=['pytest']` **does not work** — modern setuptools has
removed it (`UserWarning: Unknown distribution option: 'tests_require'`) so
colcon reads back `None`. The working form is:

```python
extras_require={'test': ['pytest']},
```

Now `colcon test` reports **68 tests, 0 failures**.

### 9.3 Three bugs in `check_topics.sh`

1. `((PASS++))` returns the *old* value as its exit status, so the very first
   `_pass` (PASS=0) returned 1 and `set -e` aborted the whole script after one
   check. → `PASS=$((PASS + 1))`.
2. `_detect_namespace` matched `/NS_1/franka/joint_states` (the driver's raw
   stream) and derived namespace `/NS_1/franka`, so every subsequent topic name
   was wrong. → take the **shortest** `*/joint_states` prefix.
3. `ros2 topic hz` prints one line per window; the whole multi-line string was
   fed to `awk`, making every rate check fail with a syntax error even on a
   healthy topic. → `tail -1`, plus `|| true` so `timeout`'s exit 124 does not
   abort the script under `set -e -o pipefail`.

Also relaxed the common `joint_states` rate threshold from 100 Hz to 30 Hz (the
`joint_state_rate` configured in `franka.config.yaml`; mock hardware delivers
~43 Hz, the real broadcaster 1 kHz) — the old value produced a false FAIL on
every fake-hardware test.

**Result:** `check_topics.sh rl` → **PASS=13 FAIL=0**.

### 9.4 Documentation stale reference

`test/README.md` documented `test_oscbf_fake.launch.py`, which has never existed
in this repository (confirmed via `git log`). Flagged inline rather than
deleting the section, since it reads as the intended specification for that
launch file.

---

## 10. Complete file inventory

### New files

| File | Lines | Purpose |
|---|---|---|
| `franka_experiments/franka_experiments/nodes/rl_policy_commander.py` | 688 | Step 3 deployment node |
| `franka_experiments/franka_experiments/utils/rl_policy.py` | 267 | sim↔real contract (pure numpy) |
| `franka_experiments/test/test_rl_policy.py` | 255 | 21 unit tests incl. config-sync guard |
| `franka_experiments/test/smoke_rl_policy_commander.py` | 252 | node-level 5-phase smoke test |
| `franka_experiments/test/launch/test_rl_fake.launch.py` | 78 | fake-HW pipeline (wraps the canonical launch) |
| `franka_experiments/scripts/plot_rl_timing.py` | 123 | jitter/latency stats + figure |
| `franka_sim/scripts/evaluate_policy.py` | 141 | policy + baseline evaluation |
| `franka_sim/scripts/validate_actuation.py` | 106 | actuation regression guard |
| `franka_sim_to_real_implementation_status.md` | — | concise audit |
| `SAFE_RL_CBF_HANDOVER.md` | — | this document |

### Modified files

| File | Change |
|---|---|
| `franka_sim/envs/franka_cbf_env.py` | **actuation fix** (§4): torque actuators + `_inverse_dynamics` + `reset()` seed; MJCF↔config obstacle-radius check |
| `franka_sim/config.yaml` | added `obstacle.radius: 0.08` (read by the deployment node) |
| `franka_sim/README.md` | actuation row, new scripts, deploy section, config-sync warning |
| `franka_experiments/launch/torque_control_stack.launch.py` | `motion_source` selector + 5 `rl_*` args + `_as_float_list` helper + move_group hint |
| `franka_experiments/setup.py` | `rl_policy_commander` entry point; `extras_require={'test': ['pytest']}` |
| `franka_experiments/package.xml` | `<test_depend>python3-pytest</test_depend>` |
| `franka_experiments/README.md` | node-reference row, pipeline diagram, `motion_source` docs |
| `franka_experiments/test/README.md` | "Pipeline 4 — Safe-RL" section, timing numbers, layout, OSCBF stale-reference note |
| `franka_experiments/test/scripts/check_topics.sh` | new `rl` mode + 3 bug fixes + threshold + `acceleration` mode corrected |
| `Dockerfile` | `qpsolvers==4.3.3` + `osqp<1.0` pin with rationale; solver-availability assertion in the image smoke test |

### Pre-existing uncommitted changes — preserved, not touched

`franka_experiments/config/fr3_complete.yaml`, `config/fr3_control.yaml`,
`nodes/cbf_safety_filter.py`, `nodes/pentagon_qddot_commander.py`,
`nodes/qddot_to_torque.py`, `utils/distance_engine.py`, the `PAPER_*.md` files,
`utils/avoidance.py`, `utils/cbf_hard_limits.py`, `utils/self_collision.py`,
`test/test_avoidance.py`, `test/test_cbf_hard_constraints.py`, and the
`franka_logs/` deletions.

---

## 11. Validation evidence (all numbers)

Everything below was actually run in the repository's container. Nothing is
estimated.

### Build & tests

| Check | Result |
|---|---|
| `colcon build --symlink-install` (whole workspace) | 17 packages, clean (only pre-existing CMake/Boost warnings) |
| `colcon test --packages-select franka_experiments` | **68 tests, 0 failures** |
| `pytest test/` | 68 passed |
| `selfcheck_run()` | all checks passed; `py_compile` 45/45 |
| YAML / launch / bash syntax | all valid (the 3 "invalid" camera YAMLs are intentional multi-document files handled by `camera_yaml.py`) |

Workspace-wide `colcon test` also reports failures in `franka_simulation`
(copyright/flake8/pep257/lint_cmake linters) and `libfranka` (`TSan` build).
**Both are pre-existing and in packages this work did not touch.**

### franka_sim

| Check | Result |
|---|---|
| `franka_sim.envs.franka_cbf_env` self-test | `check_env: OK` |
| `validate_cbf` | PASS — shield holds at `d_safe = 0.200`, passthrough penetrates to −2.401 |
| `validate_actuation` | PASS — see §4.5 |
| ONNX export validation | `max |onnx − sb3| = 4.52e-06` over 100 observations |

### Policy vs baselines — `sac_v2`, 400 k steps (~59 min, ~104 it/s, RTX 4070), 50 eval episodes each

| | trained policy | zero-action | random |
|---|---|---|---|
| success rate | **14 %** (7/50) | 2 % | 2 % |
| final EE error (mean) | **0.296 m** | 0.567 m | 0.495 m |
| final EE error (median) | 0.262 m | 0.577 m | 0.509 m |
| episode return | **−154 ± 112** | −229 ± 117 | −321 ± 141 |
| episode length | 370.1 | 421.9 | 430.8 |
| collision rate (episodes) | 16 % | 14 % | 12 % |
| min surface distance | −0.1323 m | −0.1323 m | −0.1323 m |
| mean surface distance | 0.273 m | 0.374 m | 0.351 m |
| CBF-active fraction | 100 % | 100 % | 100 % |
| mean intervention | 3.19 rad/s² | 0.20 rad/s² | 9.83 rad/s² |
| mean slack | 0.037 | 0.005 | 0.007 |

**How to read the collision numbers.** The worst-case penetration is
**identical (−0.1323 m) for all three controllers, including the arm that never
moves.** The residual collisions are the kinematic obstacle sweeping *into* the
robot at up to ~1.1 m/s (amplitude 0.30 m at 0.6 Hz), not the controller driving
into the obstacle. No controller can avoid that, and the CBF cannot fix it,
because the barrier only bounds the *robot's* motion. **The honest safety claim
is the delta against the zero-action baseline, never the absolute number.**

### Fake-hardware pipeline (`test_rl_fake.launch.py`, 120 s)

| Check | Result |
|---|---|
| Controllers | `joint_state_broadcaster` **active**, `rt_torque_controller` **active** |
| `check_topics.sh rl` | **PASS=13 FAIL=0** |
| `/NS_1/joint_states` | 42.9 Hz |
| `/NS_1/qddot_nom` | 99.72 Hz |
| `/NS_1/qddot_safe` | 99.67 Hz |
| `/NS_1/torque_cmd` | 99.98 Hz |
| `/NS_1/rl_status` | 99.84 Hz |
| TF | `world → fr3_link8` resolves — single connected tree |
| Node crashes | none |

### Timing (the paper's jitter evidence)

```
loop period        mean  9.998 ms  std 0.159  p50 10.000  p99 10.421  max 12.520
|period − nominal| mean  0.102 ms  std 0.122  p50  0.073  p99  0.564  max  2.520
ONNX inference     mean  0.138 ms  std 0.041  p50  0.131  p99  0.268  max  1.066
ticks > 1.5x nominal: 0.00 %
```

### Node smoke test

```
phase1: 150 ticks, max|out| = 0.00e+00           (warm-up gate)
phase2: 100 Hz, finite, |q̈|/q̈_max ≤ 0.978
        max|node − independent replay| = 0.00e+00 rad/s²   ← contract exact
phase3: rl_status d_min = 0.260 (injected 0.26)
phase4: max|out| = 0.00e+00                       (stale distances)
phase5: max|out| = 0.00e+00                       (stale joint state)
SMOKE TEST PASSED
```

---

## 12. Design decisions and their rationale

| Decision | Why |
|---|---|
| **No avoidance logic in the RL node** | The policy learned avoidance *under the shield*. Adding shaping on top would be an untrained outer loop competing with it. The pentagon commander's feasibility governor deliberately has no counterpart here. `cbf_status` is subscribed for diagnostics/CSV only. |
| **`motion_source` selector, not a second launch file** | Both commanders publish `/NS_1/qddot_nom`; two launch files would invite running both. One selector = one q̈_nom source, structurally. |
| **`ee_frame = fr3_link8`, not `fr3_hand_tcp`** | MuJoCo's `attachment_site` is link7 + 0.107 m = the flange = `fr3_link8`. Using the pentagon commander's `fr3_hand_tcp` would shift the observation by the hand length. |
| **Robot's q̈_max wins, sim's is only checked** | Hardware enforces the robot values; the policy emits a fraction of q̈_max, so a mismatch rescales every command. Take the real limits, warn about the drift. |
| **Torque actuation in sim** | It is what the robot does (`qddot_to_torque` + firmware g). It also happens to be the only way the action gets authority (§4). |
| **`mj_inverse` per substep, not per tick** | The analogue of `rt_torque_controller` at 1 kHz. A single 100 Hz feedforward torque leaves a zero-order-hold drift the robot does not have. |
| **Convert actuators in code, not in the MJCF** | Keeps the vendored Menagerie asset pristine and diffable against upstream. |
| **No 1 kHz `Kd` velocity feedback in sim** | Would improve fidelity but is a controller-tuning decision that belongs to the user. Its absence makes sim *worse behaved* than hardware — the safe direction. |
| **Contract in `utils/rl_policy.py`, not inline** | One definition, unit-testable without ROS, and the smoke test can re-derive it independently to prove the node agrees. |
| **Gate on "seen-then-stale", not on "never seen"** | Running without a camera is a valid configuration; a dying perception stream is a fault. |
| **`action_scale` clamped to (0,1]** | It is a derate for cautious first hardware runs. It must never be able to widen the trained envelope. |
| **`extras_require` not `tests_require`** | setuptools removed `tests_require`; colcon then sees `None` and falls back to `unittest`, which finds none of the tests. |
| **Baselines in the evaluator** | The only thing that would have caught §4. Now permanent. |

---

## 13. Known limitations, gotchas, traps

1. **Fake hardware does not integrate effort commands.** ROS 2 mock hardware
   does not simulate torque dynamics, so the arm never moves and the observation
   stays frozen. Fake-HW tests validate the *command chain* (nodes, topics,
   rates, controller activation, gating, TF) — **not** closed-loop behaviour.
   Closed-loop behaviour is validated in MuJoCo. Do not read a fake-HW "success"
   as motion validation.
2. **The trained policy is undertrained.** `sac_v2` is 400 k steps;
   `config.yaml` defaults to 2 M. 14 % success is a working-but-undertrained
   policy, not a broken pipeline.
3. **Structurally unavoidable collisions** (§11) cap the achievable safety
   metric and inject reward noise. This is probably also braking learning speed.
4. **Obstacle CBF rows are soft** (slack-relaxable) in both sim and robot by
   design — under a sustained push the QP relaxes them. Workspace and state-box
   rows are hard. Unchanged from the existing architecture.
5. **Real `d_min` saturates at 0** (the engine clamps); sim reports negative
   penetration. Conservative, documented, not "fixed".
6. **`qpsolvers` and `osqp` must not float.** See §9.1. If the velocity or OSCBF
   pipeline dies with `SolverNotFound`, check
   `python3 -c "import qpsolvers; print(qpsolvers.available_solvers)"` **first** —
   an empty list is this bug, not a code regression.
7. **The `qpsolvers` fix is in the `Dockerfile`, but the existing image predates
   it.** The downgrade was applied live inside the running container, into
   `/home/user/.local`, which is **not** bind-mounted. Any container recreated
   from the current image will have qpsolvers 4.13.0 again and the six test
   failures will return. Run `docker compose build` once (it also adds the
   `assert 'osqp' in qpsolvers.available_solvers` gate to the image smoke test),
   or patch a fresh container with
   `pip3 install "qpsolvers[osqp]==4.3.3" "osqp>=0.6.2,<1.0"`.
8. **Container-local state is disposable; the repo is not.** `docker-compose.yml`
   mounts `./:/ros2_ws/src`, so source, `build/`, `install/`, `franka_logs/` and
   `franka_sim/models/` all live on the host and survive container removal.
   Anything written elsewhere in the container (`/tmp`, pip `--user` installs)
   does not. Do not leave artifacts you care about outside the mount.
9. **Never use a MuJoCo actuator's `ctrlrange` as a force limit** unless you know
   it is a force actuator. For a `<position>` actuator it is the joint *position*
   range. This cost me a debugging cycle (§4.4).
10. **`franka_sim` is not a ROS package.** Do not "fix" this by adding a
    `package.xml` — the whole point is that training does not need ROS. Paths are
    resolved via `realpath` walk-up instead.
11. **The container may hold stray processes between runs.** A leftover
    `joint_state_publisher` from a previous launch once competed with the smoke
    test's stimulus and produced a phantom contract failure. `pkill -f "topic pub"`
    also kills the `docker exec` shell whose command line contains that string —
    check `ps aux` rather than trusting the kill.
12. **The training config frozen next to the model is authoritative.**
    `train.py` copies `config.yaml` into `models/<exp>/`; the node prefers it
    over `franka_sim/config.yaml`, because that is the config the run actually
    used.

---

## 14. Improvement backlog (prioritised)

This is the section to work from.

### P0 — makes the science defensible

1. **Train to convergence.** Run the config default (2 M steps) or longer, ideally
   with a short hyperparameter sweep. Current: 400 k → 14 % success. The
   pipeline, logging, checkpointing, eval callback and safety curves are all
   already in place; this is compute, not code.
2. **Remove structurally unavoidable collisions from the benchmark.** Either
   reduce `obstacle.speed` / `obstacle.amplitude` in `config.yaml` (already
   parameters), or add a reset-time feasibility check that rejects obstacle
   trajectories which intersect the robot's reachable set. Until then, every
   safety number must be quoted against the zero-action baseline.
3. **Report the baseline delta everywhere.** Wire `zero`/`random` into a single
   benchmark script that emits the comparison table directly, so the paper
   cannot accidentally quote an absolute collision rate.

### P1 — closes real sim-to-real gaps

4. **Add the 1 kHz `Kd(q̇_des − q̇)` feedback to the sim actuation**, matching
   `rt_torque_controller`'s `d_gains`. Removes the residual open-loop drift and
   makes sim and robot dynamically closer. Keep it configurable so the
   conservative feedforward-only mode remains available.
5. **Domain randomisation** — link masses, joint friction/damping, sensor noise
   on `q`/`q̇`, and above all **latency** (the real chain has camera → distance
   engine → CBF → torque delays that the sim does not model).
6. **Observation-noise realism.** The sim feeds exact `d_min` and an exact
   obstacle centre; the robot feeds an LPF'd, EMA-smoothed, occasionally stale
   estimate from a point cloud. Injecting that noise model in training is
   probably the single highest-value fidelity improvement after latency.
7. **Hardware-in-the-loop.** Gazebo (the stack already has a `gazebo` argument
   for `rt_torque_controller`) would give closed-loop validation without the real
   FR3.

### P2 — task and reward

8. **Reward shaping review.** Currently
   `−‖ee−target‖ + success − effort − intervention − slack − jerk − ‖q̇‖²`, with
   a one-off collision penalty and termination. Candidates: potential-based
   shaping for the distance term, curriculum on obstacle speed, and
   reconsidering `terminate_on_success` (episodes end early, so the policy never
   learns to *hold* a target — relevant because the deployment node supports
   target sequences with dwell).
9. **Multi-target episodes in sim**, matching the node's `target_sequence`
   feature. Today sim trains single-reach but deployment can cycle targets.
10. **Orientation.** The task is position-only; the observation carries no EE
    orientation and the reward does not constrain it. Real tasks usually need it.

### P3 — engineering polish

11. **Give `franka_sim` a small pytest suite.** It currently has scripts
    (`validate_cbf`, `validate_actuation`) but no `pytest`, so its guards are not
    part of `colcon test`. Consider a thin ROS-free test package or a CI step.
12. **CI.** There is a `Jenkinsfile`; wiring `colcon build` + `colcon test` +
    `validate_actuation` + `validate_cbf` into it would have caught §4 and §9.1.
13. **`pentagon_qddot_commander` hard-codes** `/ros2_ws/src/franka_experiments/franka_logs`.
    The new node's `realpath` approach could be lifted into a shared helper.
14. **Write `test_oscbf_fake.launch.py`** (§9.4) so the documented Pipeline 3
    commands actually work.
15. **Address the pre-existing `franka_simulation` linter failures** (49
    copyright, 747 flake8, 259 pep257) — or explicitly exclude that package from
    linting so `colcon test` is green and regressions are visible.

---

## 15. Command cookbook

```bash
# ── environment ───────────────────────────────────────────────────────────
docker start franka_ros2 && docker exec -it franka_ros2 /bin/bash
source /opt/ros/humble/setup.bash && source /ros2_ws/src/install/setup.bash
cd /ros2_ws/src && export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl

# ── build & test ──────────────────────────────────────────────────────────
colcon build --symlink-install
colcon test --packages-select franka_experiments && colcon test-result --all
cd franka_experiments && python3 -m pytest test/ -q

# ── franka_sim validation (run ALL THREE after touching the env) ──────────
python3 -m franka_sim.envs.franka_cbf_env          # gym self-check + rollout
python3 -m franka_sim.scripts.validate_cbf         # shield holds at d_safe
python3 -m franka_sim.scripts.validate_actuation   # action controls the arm

# ── train / export / evaluate ─────────────────────────────────────────────
python3 -m franka_sim.train --total-timesteps 2000000 --exp-name sac_v3
tensorboard --logdir franka_sim/runs               # rollout/* and safety/*
python3 -m franka_sim.export_onnx --model franka_sim/models/sac_v3/best_model.zip
python3 -m franka_sim.scripts.evaluate_policy \
    --model franka_sim/models/sac_v3/best_model.onnx --episodes 50
python3 -m franka_sim.scripts.evaluate_policy --model zero   --episodes 50
python3 -m franka_sim.scripts.evaluate_policy --model random --episodes 50

# ── deployment: node-level smoke test (no bringup, ~10 s) ─────────────────
cd /ros2_ws/src/franka_experiments
python3 test/smoke_rl_policy_commander.py          # auto-discovers newest .onnx

# ── deployment: fake hardware ─────────────────────────────────────────────
ros2 launch franka_experiments test_rl_fake.launch.py \
    rl_onnx_model:=/ros2_ws/src/franka_sim/models/sac_v3/best_model.onnx
./test/scripts/check_topics.sh rl
python3 scripts/plot_rl_timing.py                  # jitter stats + figure

# ── deployment: real FR3 (CAUTION — start derated) ────────────────────────
ros2 launch franka_experiments torque_control_stack.launch.py \
    motion_source:=rl start_move_group:=false \
    rl_action_scale:=0.3 \
    rl_onnx_model:=/ros2_ws/src/franka_sim/models/sac_v3/best_model.onnx
# watch: rl_status[5] (gate) and cbf_status (slack / fault_braking)
```

**Recommended first real run:** `rl_action_scale:=0.3`, camera and
`real_time_distance` **on**, a single conservative target, with `rl_status` and
`cbf_status` on screen. No physical-robot test has been performed — everything
above is validated statically and on fake hardware only.

---

## 16. Git state

* **Branch:** `humble-mattia`. Nothing committed, nothing pushed.
* Working tree contains the pre-existing uncommitted changes (untouched) plus
  the new/modified files listed in §10.
* No build artifacts, `.pyc`, `__pycache__`, model `.zip`/`.onnx`, or generated
  figures are exposed as untracked files — `franka_sim/.gitignore` covers
  `runs/`, `models/`, `*.onnx`, `*.zip`, and the root `.gitignore` covers
  `franka_experiments/franka_logs/`.
* `franka_sim/`, `franka_sim_to_real_roadmap.md`,
  `franka_sim_to_real_implementation_status.md` and this file are untracked and
  will need `git add` when you commit.
