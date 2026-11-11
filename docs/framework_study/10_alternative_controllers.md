# 10 — Alternative controllers: velocity CBF, torque-level OSCBF, and the RL policy

> **Source baseline:** `ba68077`. Written in Stage 5 (2026-11-11). Labels as in
> [00_INDEX.md](00_INDEX.md) §2. Line numbers refer to the baseline.
>
> **Reproducible checks.** [tools/stage5_verification/](tools/stage5_verification/README.md):
> `check_velocity_cbf.py` (the production `_solve_cbf_qp` through the deployed qpsolvers/OSQP),
> `check_oscbf.py` (the production OSCBF builders on the real Pinocchio models) and
> `check_torque_commander.py` (a labelled transcription). None of them runs a node.

Chapters 06–09 documented **Pipeline A**: an acceleration commander, the acceleration-level HOCBF
filter, inverse dynamics and the 1 kHz torque controller. The repository contains three more ways
to move the arm. This chapter derives what each one computes, how it differs from Pipeline A, and
whether a shipped launch file actually runs it (01 §4 for the pipeline map).

| Pipeline | Command chain | Started by |
|---|---|---|
| **B** — velocity CBF | `ee_pentagon_velocity_commander` → `/NS_1/tracking_qdot` → `cbf_velocity_filter` → `/NS_1/qdot_cmd` → `RtVelocityExecutorController` → velocity interface | `velocity_cbf_control_stack.launch.py` |
| **C** — OSCBF (torque) | `pentagon_torque_commander` (or `qddot_to_torque`) → `/NS_1/torque_cmd` → `cbf_oscbf_filter` → `/NS_1/torque_safe` → **nothing** | no production launch (F-0-2, F-1-6) |
| **A with RL** | `rl_policy_commander` → `/NS_1/qddot_nom` → `cbf_safety_filter` (unchanged) | `torque_control_stack.launch.py motion_source:=rl` |

---

## 1. Intuition: what changes with the control variable

A barrier condition must be imposed on a quantity the controller can set **this instant**.

- **Pipeline A** sets q̈. The gap h has relative degree 2 in q̈, so the barrier needs the HOCBF
  chain, the drift term c and the obstacle velocity inside ḣ (07 §2).
- **Pipeline B** sets q̇. h has relative degree 1, so a first-order (zeroing) CBF suffices:
  ḣ ≥ −γh. It assumes the commanded velocity is realised immediately, which the velocity
  interface, its rate limiter and the arm's inertia only approximate.
- **Pipeline C** sets τ. The barrier is still relative degree 2, but now in τ through the dynamics
  q̈ = M⁻¹(τ − Cq̇). The cost can then be shaped by the operational-space decomposition: weight the
  task-space part and the null-space part of the torque change separately.

The lower the control variable (τ < q̈ < q̇), the more of the robot's physics the filter must
model, and the fewer assumptions it makes about the layers beneath it.

**One family, three relative degrees (Derived).** With h(q) the gap and the obstacle static, the
three filters impose the same idea at different orders:

  (10.1)  velocity:      ḣ + γh ≥ 0,                       ḣ = n̂ᵀJ_p q̇               (decision q̇)
          acceleration:  ḧ + (α₁+α₂)ḣ + α₁α₂h ≥ 0,         ḧ = n̂ᵀJ_p q̈ + c           (decision q̈)
          torque:        the same as acceleration, with q̈ = M⁻¹(τ − Cq̇), τ excluding g   (decision τ)

where c = n̂ᵀJ̇_p q̇ + ṅ̂ᵀJ_p q̇ is the drift (04 §3.4, 07 §2.4). The second line is the HOCBF chain
ψ₁ = ḣ + α₁h ≥ 0, ψ̇₁ + α₂ψ₁ ≥ 0. Pipeline A writes it with k₁ = α₁ + α₂ and k₀ = α₁α₂ (07 §2.5).
Each line is only as safe as the assumption that the layer below realises the decision variable
exactly. For q̇, that is the executor's rate limiter and the arm's inertia (§2.4). For q̈, it is
the inverse dynamics and the PD (09). For τ, it is the model's M and C and the gravity that the
filter leaves out (10.9).

---

## 2. Pipeline B — the velocity-level CBF

### 2.1 Launch and effective configuration (Verified)

`velocity_cbf_control_stack.launch.py` starts, with delays:
1. the bringup with the `rt_velocity_executor_controller`, configured by the generator from
   `launch_defaults`: `qdot_max` 1.5 rad/s, `max_accel` 1.5 rad/s², `enable_interpolation` true;
2. the camera and `real_time_distance` (unless bypassed);
3. `cbf_velocity_filter` with the single parameter `bypass_cbf`;
4. `ee_pentagon_velocity_commander`, with no parameters (06 §8);
5. the experiment logger.

**`bypass_cbf` defaults to `false`** (`DeclareLaunchArgument`, L434–436), so the CBF QP **is active
by default**. The launch docstring (L19) calls Phase 1 (bypass) the default, and ch. 01 §4.2
repeated that; both are wrong (**F-5-1**, ch. 01 corrected). With `bypass_cbf:=true` the filter
publishes clip(q̇_nom, ±q̇_max) and the launch skips the camera and perception unless they are asked
for. That is the "previously identified pass-through": it exists, but it is opt-in.

### 2.2 Mathematics (Verified against `cbf_velocity_filter.py`)

For each valid `LinkDistance` entry i whose link is **active**:
- the link activates when d < d_safe + 0.10 and deactivates when d > d_safe + 0.15
  (`cbf_activation_margin`, `cbf_hysteresis`, retired for Pipeline A);
- the activation state is keyed by `robot_link_name`, which several control points share, so the
  outcome depends on the order of the entries (F-5-2).

  (10.2)  h = d − d_safe (surface gap),  ḣ = n̂ᵀJ_p q̇  (static obstacle: no v_obs term),
          n̂ = the **perception** direction `ld.direction` (not p_r − p_h),
          J_p = Jv − [r]× Jω at `closest_point_robot` (LOCAL_WORLD_ALIGNED, `_compute_point_jacobian`).

The ZCBF condition ḣ ≥ −γh becomes, with one shared slack s ≥ 0:

  (10.3)  min_{q̇, s} ½‖q̇ − q̇_nom‖² + ½ρs²
          s.t. aᵢᵀq̇ + s ≥ −γᵢhᵢ (aᵢ = n̂ᵢᵀJ_p,i),  −q̇_max ≤ q̇ ≤ q̇_max,  0 ≤ s ≤ 1e6,

with ρ = `rho_slack` = 1000 and q̇_max = column 2 of `fr3_control.yaml joint_limits`
(2.62 … 5.26 rad/s). **There is no position limit and no acceleration limit in this QP.** It is
solved by `qpsolvers.solve_qp(..., solver='osqp')` at ~1e-3 accuracy. As in Pipeline A, the single
slack makes every row a penalty: the closed form s = (b − aᵀq̇_n)/(1 + ρ‖a‖²) is reproduced exactly
(`check_velocity_cbf.py` 1).

  (10.4)  γ = clip(conf, 0.5, 1)·(2 + 10·e^{−18·max(d − d_safe, 0)})  (`cbf_utils.select_gamma`),

so γ is 12 s⁻¹ at the barrier and 3.65 s⁻¹ 10 cm outside it.

### 2.3 Post-processing and timing (Verified)

The 100 Hz timer (`control_rate_hz`) applies, after the QP, a per-joint rate clamp:

  (10.5)  q̇_cmd = q̇_prev + clip(q̇_QP − q̇_prev, ±q̈_max·Δt),
          q̈_max = min(decel limit, jerk_max·1 ms·0.8/2) = (3, 1.5, 2, 2.5, 3, 4, 4) rad/s².

This **can undo the barrier**. Approaching at 0.3 m/s along n̂, the QP output met the row up to the
penalty residue (4e-4 m/s), but the clamped command violated it by **0.099 m/s** in that tick
(**F-5-3**, `check_velocity_cbf.py` 2).

The joint state comes from `topics.joint_states_topic`, the **30 Hz** `/NS_1/joint_states`
republisher, with no age check. `ros_setup.make_joint_state_callback` maps positions and velocities **by
name**, so the scrambled driver order does not matter. A message without velocities leaves the
previous q̇ in place silently. The Pinocchio model comes from `ros_setup.init_pinocchio_only`: the
reduced model built from `robot_description` (04 §2.3). Two more permissive branches (**F-5-2**):
- distances stale (> 0.5 s) **or never received** → q̇_cmd target = clip(q̇_nom) (pass-through);
- QP failure → 0.5·clip(q̇_nom).

There is no braking path.

### 2.4 The velocity executor (`rt_velocity_blender_controller.cpp`, Verified)

`RtVelocityExecutorController` (plugin name; file named "blender", F-0-7) claims
`fr3_joint*/velocity`, and subscribes to `/NS_1/qdot_cmd` with SensorDataQoS, keep last 1.
In `update()` at the controller-manager rate:

  (10.6)  q̇_cmd(t) = linear interpolation from the previous sample to the newest one over the
          measured inter-arrival time (one sample of delay),
          then |Δq̇| ≤ `max_accel`·Δt, then |q̇| ≤ `qdot_max`.

**There is no command watchdog**: `timeout_threshold_s` and `timeout_ramp_s` are read in
`on_configure` but marked "reserved – not used in RT path" (header L88–89). The last velocity is
held indefinitely if `qdot_cmd` stops (**F-5-7**). Because the filter publishes nothing until it has
a joint state, a filter that stalls leaves the arm at its last commanded velocity. On real hardware
it also sets the same collision thresholds as `RtTorqueController` (09 §3.2).

### 2.5 Velocity CBF vs Pipeline A's HOCBF

| Aspect | Pipeline A (07–08) | Pipeline B |
|---|---|---|
| Decision variable | q̈ (+ 7 family slacks) | q̇ (+ 1 slack) |
| Barrier | ḧ + k₁ḣ + k₀h ≥ 0 with c (and κ) | ḣ + γh ≥ 0 |
| Obstacle motion | v_obs in ḣ (tracker/residual), margins, latency | none (static) |
| Normal | p_r − p_h (from the message) | perception direction |
| Gains | k₀ = 25, k₁ = 10.5 (zone ladder) | γ(d, conf) ∈ [1, 12] |
| Joint limits | hard position/velocity/acceleration box + slew | velocity box only |
| Evidence gates, memory, identity guard | yes (07 §3–§4) | none |
| Stale perception | brake (−k_brake q̇) | pass-through |
| Post-QP | clip to the box (tolerance) | rate clamp that can violate the row |
| Rate / state | 300 Hz, 1 kHz joint state | 100 Hz, 30 Hz joint state |

---

## 3. Pipeline C — the torque-level Dynamic-OSCBF

### 3.1 The nominal: `pentagon_torque_commander` (Verified)

A 6-D Cartesian PD (default K_p = 80, K_d = 15 for position; 20, 5 for orientation), mapped by DLS
(λ² = 1e-6) and inverse dynamics:

  (10.7)  ẍ = [K_p(p_d − p) + K_d(v_d − J_v q̇) ;  K_pR e_R − K_dR J_ω q̇],
           q̈ = J⁺(ẍ − J̇q̇),  τ = M q̈ + C q̇  (gravity excluded), at 100 Hz.

With `cbf_mode` it publishes q̈ on `/NS_1/qddot_nom` instead.

**F-5-6 (Verified + numerically illustrated).** e_R = ½vee(R_dᵀR − RᵀR_d) ≈ **+θ** for R =
R_d·exp([θ]×). That is the rotation from desired to current, in R_d's frame, and it is fed back
with **+K_pR**. The orientation loop is therefore θ̈ = +K_pR θ − K_dR θ̇, which is unstable with root
2.62 s⁻¹. A 0.011 rad perturbation grows to 1.02 rad in 2 s under ideal tracking, while the form
`pentagon_qddot_commander` adopted (−R_d·log(R_dᵀR), 06 §4.1) decays it to 8e-5 rad
(`check_torque_commander.py`). There is also no a_d feedforward. The node is started only by
`test/launch/test_torque_fake.launch.py`.

### 3.2 The filter: cost (Verified, `_OSCBFQPBuilder.build_cost`)

Decision x = [τ (7); s (1)]. With J the 6×7 end-effector Jacobian, Λ = (JM⁻¹Jᵀ + 1e-6 I)⁻¹, and
the dynamically consistent null-space projector Nᵀ = I − JᵀΛJM⁻¹:

  (10.8)  min ½‖w_j M⁻¹Nᵀ(τ − τ_n)‖² + ½‖w_o J M⁻¹(τ − τ_n)‖² + ½ρs²  (w_j = w_o = 1, ρ = 1000).

- J M⁻¹Δτ is the task-space acceleration change, and M⁻¹NᵀΔτ the null-space part: the cost
  penalises the **accelerations** the torque change causes, split by operational-space
  decomposition (Morton & Pavone 2025, eq. 37, per the docstring).
- Verified numerically: P_arm = A_jᵀA_j + A_oᵀA_o is positive definite, the unconstrained optimum
  is τ_n, and the quadratic form equals (10.8).
- P_arm is badly conditioned, with eigenvalues 0.06 … 2.7·10⁵ at q_home (**F-5-15**).

### 3.3 The filter: constraints (Verified)

All rows have the form Aτ + s ≥ b with the dynamics

  (10.9)  q̈ = M⁻¹(τ − Cq̇)   (the τ published excludes g; something downstream must add it).

- **Joint position** (always on; α₁ = α₂ = 10; limits are the YAML limits ± 0.05 rad):

    (10.10)  Aτ ≥ b ⇔ q̈ᵢ ≥ −(α₁+α₂)q̇ᵢ − α₁α₂(qᵢ − q_min,i), and the mirrored upper row.

  This is the correct HOCBF chain (verified to 7e-15, `check_oscbf.py` 1).
- **Joint velocity** (`enable_vel_cbf`, off): q̈ᵢ ≥ −α_v(q̇ᵢ − q̇_min,i).
- **End-effector workspace box** (`enable_ws_cbf`, off): the HOCBF chain on p_ee.
- **Obstacle** (`enable_obstacle_cbf`, **off by default**), per valid entry with conf ≥ 0.2:

    (10.11)  h = ‖p_r − p_h‖ − 0.15,  ḣ = n̂ᵀJ_p q̇,  c = n̂ᵀJ̇_p q̇ + ṅ̂ᵀJ_p q̇,
             coded: n̂ᵀJ_pM⁻¹τ + s ≥ n̂ᵀJ_pM⁻¹Cq̇ − c − α₂(ḣ + α₁h)
             ⇔ ḧ ≥ −α₂ḣ − α₁α₂h.

  **F-5-4.** The HOCBF chain ψ₁ = ḣ + α₁h, ψ̇₁ + α₂ψ₁ ≥ 0 gives ḧ ≥ −(α₁+α₂)ḣ − α₁α₂h; the
  obstacle row drops α₁ḣ, which the joint and workspace rows keep.
  - The closed loop becomes s² + 10s + 100 (ζ = 0.5, 16.3 % impulse undershoot).
  - In a 1-D binding approach at 1 m/s from h = 0.20 m it reaches h = **−29.8 mm**, while the
    correct law stops at 0 (`check_oscbf.py` 2–3).

  **F-5-5.**
  - h is the **centre** distance minus a hard-coded 0.15 m: `LinkDistance.distance` is ignored, and
    so are the capsule radius and the mask margin that the main filter's comment calls a 6–16 cm
    optimism (07 §2.2).
  - The obstacle Jacobian uses the hand-less model at the frame named by `robot_link_name`
    (substring fallback).
  - It assumes a static obstacle.
- **Torque box:** |τ| ≤ (87, 87, 87, 87, 12, 12, 12), hard; slack in [0, 1e8].

### 3.4 Interfaces and failure behaviour (Verified)

| Item | Value |
|---|---|
| Input | `tau_nom_topic` = `/NS_1/torque_cmd` (depth 10) |
| Joint state | `topics.joint_states_topic` = `/NS_1/joint_states` (30 Hz); index map cached on the first message (F-2-5) |
| Distances | `/cbf/per_link_distances`, depth 10 = RELIABLE vs the BEST_EFFORT publisher → nothing is received (F-1-6) |
| Output | `/NS_1/torque_safe`: no consumer anywhere (F-1-6) |
| Rate | 100 Hz timer; qpsolvers `solve_qp` with warm start |
| Stale nominal (> 0.5 s) | τ_n = 0 |
| Stale distances | obstacle rows skipped, no fallback |
| QP failure | publish τ_n unfiltered |
| `bypass_filter` | publish τ_n |

**Is it executable?** The node is a console script (`cbf_oscbf_filter`), and its dependencies are
the same Pinocchio/qpsolvers stack. No launch file starts it, and even started by hand it would
filter nothing (obstacle rows off, no distance data under the QoS mismatch) and drive nothing (no
consumer of `/NS_1/torque_safe`). Its own header marks it legacy ("no launch file starts it"). These
are statements about **wiring**, not about the value of the method.

### 3.5 OSCBF vs Pipeline A

| Aspect | Pipeline A | Pipeline C |
|---|---|---|
| Decision | q̈ | τ |
| Model needed in the filter | kinematics only (J, J̇) | M, C (Pinocchio, hand model) and J |
| Cost | ½‖q̈ − q̈_nom‖² (W = I) | task/null-space-weighted acceleration change (10.8) |
| Barrier in τ | — (09 converts afterwards) | via q̈ = M⁻¹(τ − Cq̇) |
| Obstacle h | surface gap − d_safe − margins | centre distance − 0.15 |
| Obstacle chain | correct (k₁ = α₁ + α₂) | missing α₁ḣ (F-5-4) |
| Torque limits | none (09 §6) | hard τ box |

---

## 4. Reinforcement learning (Pipeline A with `motion_source:=rl`)

The deployed interface is documented and audited in **06 §7**, which covers the observation layout
(51-D), action = clip(a)·q̈_max·scale, F-3-10 and F-3-19. What this chapter adds:

| Topic | Deployment (this repository) | Training (`franka_rl`, read only where needed) |
|---|---|---|
| Command | q̈_nom on `/NS_1/qddot_nom`, through `cbf_safety_filter` unchanged (Verified) | the training environment applies its own shield; the config calls it the "sim↔robot shield parity register" (`franka_rl/config.yaml` L452–; not analysed) |
| Rate | `env.control_rate_hz` = 100 Hz (Verified, `franka_rl/config.yaml` L21) | the same value; the comment says the robot QP runs at 300 Hz "since 2026-10-07" |
| Obstacle | perception entries → the nearest centre p_h − r·n̂; v_obs = finite difference at the 100 Hz tick (sawtooth, F-3-10) | a sphere of radius 0.08 m (`obstacle.radius`); v_obs differentiated "exactly as on the robot", per the config comment (Unverified) |
| Control points | robot `end_link` names, so `fr3_link3` is never filled (F-3-19) | `body` origins with radii 0.07–0.13 m (`config.yaml` L431–446) |
| Missing data | warm-up zeros; stale joints or distances → zeros; no snapshot → a synthetic far sphere | — |
| Model discovery | `find_sim_root` walks up from the node file looking for `franka_sim/config.yaml`; `find_latest_model('')` returns `''`, so with the launch defaults (`rl_onnx_model: ""`, no `sim_root`) the constructor raises `RuntimeError` (`rl_policy_commander.__init__` L194–211; Derived: no `franka_sim/` directory exists on the host checkout or in the container, checked 2026-11-11; F-0-10) | the package is `franka_rl/` |

**Safety assumption.** The policy is not trusted: its output is a nominal like any other, and the
whole of 07–09 applies to it. Out-of-distribution observations (F-3-10's spike, F-3-19's empty
slot) affect only the nominal's quality, not the filter's constraints.

---

## 5. Comparison of the implemented controllers

| | Pipeline A (HOCBF) | Pipeline B (velocity CBF) | Pipeline C (OSCBF) | A + RL |
|---|---|---|---|---|
| Control variable | q̈ → τ (09) | q̇ | τ | q̈ (policy) |
| Formulation | HOCBF rel. deg. 2, v_obs, margins (07) | ZCBF rel. deg. 1, static obstacle | HOCBF in τ via dynamics (obstacle chain incomplete) | A |
| Optimisation | OSQP, 14 (46) vars, padded rows, 7 family slacks | qpsolvers/OSQP, 8 vars, 1 slack | qpsolvers/OSQP, 8 vars, 1 slack | A |
| Sensors | 1 kHz joint state, distances + tracks | 30 Hz joint state, distances (direction) | 30 Hz joint state, distances (unreceivable) | A + policy observations |
| Robot interface | effort via `RtTorqueController` (1 kHz) | velocity via `RtVelocityExecutorController` | none configured | A |
| Effective launch | `torque_control_stack` (default) and the Gazebo stack | `velocity_cbf_control_stack` (QP on) | none | `motion_source:=rl` (needs a model path, F-0-10) |
| On stale perception | brake | pass-through | rows skipped | A |
| Key defects found | F-4-1/2/3/5 | F-5-1/2/3/7 | F-5-4/5/6 | F-3-10/19, F-0-10 |
| Computational load | rebuild 50 Hz + QP 300 Hz (02 §6) | QP 100 Hz | dynamics + QP 100 Hz | A + ONNX inference |
| Status | production | launched, legacy-flagged ("TODO[LEGACY]") | legacy, unwired | experimental |

**Shared components:**
- perception and `MultiLinkDistance` (all);
- `PentagonTrajectory` / the path classes (B, C, A's legacy pentagon);
- Pinocchio models from `generate_urdf_from_xacro` (A's `qddot_to_torque`, C);
- the `fr3_control.yaml` joint-limit table (all);
- the collision thresholds set by the C++ controllers (A, B).

Status labels are from the code's own `TODO[LEGACY]` markers and the launch wiring. Q-1-4 asks the
user which of B and C are still meant to run.

### 5.1 Status of every alternative-controller component

The classes are defined from code evidence only:
- **active**: a shipped launch starts it with its default arguments;
- **opt-in**: a shipped launch starts it behind a non-default argument;
- **dormant**: it runs, but a default flag keeps its effect off;
- **unwired**: it is an executable, but no production launch starts it;
- **unused**: nothing outside tests imports or references it.

| Component | Class | Evidence (Verified) |
|---|---|---|
| `cbf_velocity_filter` (QP) | active in `velocity_cbf_control_stack` | `bypass_cbf` default false (F-5-1); `TODO[LEGACY]` header L41 |
| `cbf_velocity_filter` bypass | opt-in | `bypass_cbf:=true` |
| `ee_pentagon_velocity_commander` | active in the velocity stack | launch L330–335 (06 §8) |
| `ee_circle_velocity_commander`, `ee_random_waypoints_velocity_commander` | unwired | F-0-2 |
| `RtVelocityExecutorController` | active in the velocity stack | spawned from the generated YAML (`generate_rt_controllers_yaml`) |
| executor `timeout_threshold_s`, `timeout_ramp_s` | unused | read, never used (F-5-7) |
| `franka_rt_controllers/config/rt_velocity_blender.yaml` (`qdot_max` 0.4, `max_accel` 4.0, timeouts 0.3/0.8) | not the effective configuration | loaded only by `franka_rt_controllers/launch/rt_velocity_blender.launch.py`; the stack writes its own YAML with 1.5/1.5/0.5/1.0 from `launch_defaults.yaml` L93–97 (F-5-16) |
| `launch_support.build_wrapper_log_actions` | unused | re-exported by `utils/ros.py`, called by no launch file (grep); it would print the timeouts as `[ACTIVE]` (F-5-16) |
| `cbf_oscbf_filter` | unwired | F-0-2; header L35 "no launch file starts it" |
| OSCBF obstacle rows | dormant (`enable_obstacle_cbf` false), and inert even when on (QoS, F-5-5) | §3.4 |
| OSCBF velocity and workspace rows | dormant (`enable_vel_cbf`, `enable_ws_cbf` false) | §3.3 |
| `pentagon_torque_commander` | test-only | `test/launch/test_torque_fake.launch.py` |
| `rl_policy_commander` | opt-in (`motion_source:=rl`); fails at start-up without an explicit model path | §4 |
| `CBFTorqueController` | opt-in (`minimal.launch.py use_torque_controller:=cbf`) | F-4-8, 09 §3.3 |

---

## 6. Practical example: the same approach seen by the three barriers

Take an obstacle 2 cm outside the barrier, with the arm approaching it. Each line below is
reproduced by a script; nothing here is a closed-loop result.

**Pipeline B, one 100 Hz tick** (`check_velocity_cbf.py` 1–3; the production `_solve_cbf_qp` and
`select_gamma`):
1. h = 0.02 m and conf = 1, so (10.4) gives γ = 2 + 10·e^{−0.36} = 8.98 s⁻¹. The row demands
   ḣ ≥ −γh = −0.18 m/s: the arm may close on the obstacle at most 18 cm/s.
2. The nominal approaches faster than that. The QP returns q̇ = q̇_n + λa, and the row is short by
   the penalty residue (b − aᵀq̇_n)/(1 + ρ‖a‖²) = 4.2·10⁻⁴ m/s (exact to 5·10⁻¹⁷).
3. The OSQP answer through qpsolvers differs from the exact optimum by up to 1.1·10⁻³ rad/s.
4. With q̇_prev = q̇_n (the arm was already approaching at 0.3 m/s), the rate clamp (10.5) lets the
   command change by at most q̈_max·0.01 s per joint. The published q̇ violates the row by
   **0.099 m/s**, so in that tick the arm closes at about 0.28 m/s instead of 0.18 m/s. The clamp
   needs several ticks to bring the command onto the row, and during them h keeps shrinking (F-5-3).

**Pipeline C, a 1-D approach** (`check_oscbf.py` 2–3; the production `_build_obstacle_cbf` row at
q_home, and a 1-D integration of the binding row):
1. At the test configuration, the coded row asks ḧ ≥ −9.57 m/s². The HOCBF chain would ask
   ḧ ≥ −8.57 m/s². The difference is the missing α₁ḣ = −1.00 m/s² (F-5-4).
2. Integrated from h = 0.20 m at 1 m/s with the row binding, the coded law undershoots to
   h = −29.8 mm, while the corrected law
   (10.12)  ḧ ≥ −(α₁+α₂)ḣ − α₁α₂h
   stops at h = 0.
3. Even with that fixed, h is the centre distance minus 0.15 m (10.11). The surface gap that
   perception reports is ignored (F-5-5).

**Pipeline A** for comparison: the same approach goes through the HOCBF row of 07 §5 with
k₁ = 10.5, k₀ = 25, v_obs and the margins, and the barrier is soft only through the family slack
(F-4-1).

## 7. Consequences of modification

| Change | Effect |
|---|---|
| Fixing F-5-4 (add −α₁ḣ) | the OSCBF obstacle row becomes the true HOCBF (critically damped at α₁ = α₂) |
| Using `LinkDistance.distance` in the OSCBF (F-5-5) | the barrier moves in by the capsule radius plus the margin (6–16 cm) |
| Enabling the OSCBF obstacle rows | also needs BEST_EFFORT QoS on the subscription, and a consumer of `/NS_1/torque_safe` |
| Velocity filter: braking on stale data, a watchdog in the executor | closes F-5-2 / F-5-7 |
| Moving the rate clamp into the QP (as bounds) | removes F-5-3 |
| Fixing the torque commander's e_R | needed before any Pipeline C test with orientation control |
| Tests | `test_cbf_velocity_filter.py`: its `_solve` is a "minimal replica" of `_solve_cbf_qp` (docstring L22), so a change to the node's QP is **not** caught. `test_torque_stack_external_plant_off_identical.py`: pins the torque-stack launch expansion against a golden snapshot (commit 94cd7b0) for the default, fake-hardware and `motion_source:=rl` variants; any edit to `torque_control_stack.launch.py` must regenerate it. `test_rl_policy.py`: parity tests skip (F-0-10). No test covers `cbf_OSCBF_filter`, `pentagon_torque_commander` or the velocity executor. |

## 8. Findings from this chapter

| ID | Short title | Section | Reproduced by | Still needs |
|---|---|---|---|---|
| F-5-1 | `bypass_cbf` default is false (ch. 01 corrected) | §2.1 | `check_velocity_cbf.py` 4 | — |
| F-5-2 | the velocity filter fails open (pass-through on stale data) | §2.3 | `check_velocity_cbf.py` 4 | — |
| F-5-3 | the post-QP rate clamp violates the velocity barrier | §2.3 | `check_velocity_cbf.py` 2 | runtime |
| F-5-4 | the OSCBF obstacle row omits α₁ḣ | §3.3 | `check_oscbf.py` 2–3 | — |
| F-5-5 | the OSCBF obstacle path is inert or optimistic (centre distance, hard-coded 0.15, off, QoS) | §3.3–3.4 | `check_oscbf.py` 4, 6 | — |
| F-5-6 | `pentagon_torque_commander` orientation sign error | §3.1 | `check_torque_commander.py` | — |
| F-5-7 | the velocity executor has no watchdog | §2.4 | `check_velocity_cbf.py` 5 | hardware behaviour |
| F-5-15 | the OSCBF cost is badly conditioned | §3.2 | `check_oscbf.py` 5 | — |
| F-5-16 | stale velocity-executor configuration and diagnostics (unused YAML, dormant `[ACTIVE]` log, commander docstring) | §5.1 | `check_velocity_cbf.py` 6 | — |
