# 06 — Nominal motion: how the desired joint acceleration q̈_nom is generated and shaped

> **Source baseline:** `ba68077`. Written in Stage 3 (2026-11-11). Labels as in
> [00_INDEX.md](00_INDEX.md) §2. Line numbers refer to the baseline.
>
> **Reproducible checks.** The numbers marked *numerically checked* are recomputed by
> [tools/stage3_verification/](tools/stage3_verification/README.md):
> `check_timing_governor.py` (§3, §5), `check_nominal_shaping.py` (§4, §6, §7) and
> `effective_params.py` (which mechanisms are on). All of them run on the host and are read-only.
>
> **Provenance of §6.4–§6.7, §7 and §8.** These sections were first drafted from two read-only
> subagent reports. In the **Stage 3 audit** (checkpoints C-017 to C-019) every equation, parameter
> and implementation claim in them was re-read in the source by the main session, and the
> "Inferred (subagent reading)" labels were replaced by the result of that re-reading. The
> corrections, with the superseded wording, are listed in §13.

The safety filter of chapter 07 does not invent motion. It receives a **nominal** command and
changes it as little as the constraints allow. This chapter covers where that nominal comes from
(the commanders, §2–§5, §7, §8) and how the filter reshapes it *before* the QP, through objective
biases that are not constraints (§6).

Code map (package `franka_experiments/franka_experiments/`):

| Role | `file::function` |
|---|---|
| Pipeline A commander (default) | `nodes/pentagon_qddot_commander.py::_tick` L785–1136 |
| Paths | `utils/trajectory.py`: `CircularPath`, `LinearSegmentPath`, `CompositePath`, `LissajousPath`, `PentagonPath` |
| Timing laws | `utils/timing_law.py`: `LinearTimingLaw` (default), `TrapezoidalTimingLaw`, `ExponentialTimingLaw` |
| Phase governor | `pentagon_qddot_commander._governor_sigma` L751–783 |
| Singularity recovery | `pentagon_qddot_commander._tick_recovery` L1140–1203 |
| Shaping inside the filter | `nodes/cbf_safety_filter.py::_qp_tick` L1186–1433; `utils/state_governor.py`, `cbf_qp_assembly.tangential_bias`, `go_around.py`, `cbf_evasion.py`, `evasion_direction.py`, `livelock.py`, `cbf_zones.ZoneLadder.task_weight` |
| RL commander | `nodes/rl_policy_commander.py`, `utils/rl_policy.py` |
| Pipeline B commanders | `nodes/ee_pentagon_velocity_commander.py` (launched), `ee_circle_…`, `ee_random_waypoints_…` (not launched) |
| Test-only | `utils/avoidance.py` (F-3-18) |

---

## 1. Intuition

The arm should trace a task, by default a 25 cm circle in the y–z plane, while the filter keeps it
away from obstacles. The task is defined geometrically (a path P(s)) and in time (a timing law
s(t)). A Cartesian PD with acceleration feedforward turns tracking error into a desired end-effector
acceleration, and a damped inverse of the Jacobian turns that into joint accelerations. A posture
term in the Jacobian's null space keeps the redundant 7th degree of freedom near the start
configuration.

Two layers of "slowing down" sit on top:
1. **In the commander**, the phase governor slows the *reference* (s advances more slowly) when the
   filter reports an obstacle nearby, so the reference does not run away while the barrier holds
   the arm back.
2. **In the filter**, before the QP, the nominal is faded toward braking (zone ladder, state
   governor) and biased sideways (tangential bias, go-around, evasion, livelock). These shift the
   QP's objective and never loosen a constraint.

Without the first layer, a held-back arm accumulates a large tracking error, and the PD then demands
a large acceleration the moment the obstacle leaves. Without the second, the QP's minimal-change
solution tends to stall the arm against the obstacle (livelock).

---

## 2. Which commander runs

| Commander | Output (topic, type, unit) | Started by | Status |
|---|---|---|---|
| `pentagon_qddot_commander` | `/NS_1/qddot_nom`, `Float64MultiArray`, q̈ in rad/s², 100 Hz | `torque_control_stack.launch.py` (default `motion_source`) | **Pipeline A default** (Verified) |
| `rl_policy_commander` | `/NS_1/qddot_nom`, q̈ rad/s², `env.control_rate_hz` (100 Hz) | same launch with `motion_source:=rl` | Pipeline A alternative (Verified, launch L870–899) |
| `ee_pentagon_velocity_commander` | `/NS_1/tracking_qdot`, q̇ in rad/s, 200 Hz | `velocity_cbf_control_stack.launch.py` | Pipeline B (Verified) |
| `ee_circle_velocity_commander`, `ee_random_waypoints_velocity_commander` | q̇ | no production launch | unused (F-0-2) |
| `pentagon_torque_commander` | τ | `test/launch/test_torque_fake.launch.py` only | test-only (F-0-2) |

The name "pentagon" is historical. `path_type: circle` is the YAML default, and the code comment
calls the pentagon path legacy, because its corner blends step the feedforward acceleration
(Verified, declaration comment L152–155 of the parameter block).

---

## 3. Path and timing law

### 3.1 Mathematics

A geometric path P: [0, ∞) → ℝ³ (metres, frame `plane_frame` = `fr3_link0`) is parameterised by a
**phase** s (dimensionless; one lap = 1). A timing law s(t) gives ṡ and s̈. By the chain rule,

  (6.1)  p_d = P(s),
  (6.2)  v_d = P′(s)·ṡ,
  (6.3)  a_d = P″(s)·ṡ² + P′(s)·s̈.

`GeometricPath.velocity` and `.acceleration` implement exactly (6.2) and (6.3) (Verified).
With σ = 1 the code's v_d and a_d match central differences of p_d(t) to 5e-8 (**numerically
checked**).

**Circle** (`CircularPath._compute`). With orthonormal plane axes u, v (for `yz`: u = ŷ, v = ẑ):

  P(s) = c + r(cos 2πs·u + sin 2πs·v),  P′ = 2πr(−sin·u + cos·v),  P″ = −(2π)²r(cos·u + sin·v).

Shipped values (`fr3_control.yaml`): c = (0.40, 0, 0.45) m, r = 0.25 m, cycle 6 s, so the full-rate
ṡ is 1/6 s⁻¹. Tangential speed 2πr/T = **0.262 m/s**; centripetal acceleration (0.262)²/0.25 =
**0.274 m/s²**. The entry point P(0) = (0.40, 0.25, 0.45). (Verified + numerically checked.)

**Lead-in** (`_start_trajectory`). The node does not jump to the circle. It builds

  `CompositePath([(LinearSegmentPath(p₀, P_circle(0), span=S_a), S_a), (circle, ∞)])`,
  S_a = approach_time / cycle = 2.5/6 = 0.4167,

where p₀ is the measured end-effector position at the end of the warm-up (3 s). On the segment,
P′ = (entry − p₀)/S_a and P″ = 0.

**Timing law** (`LinearTimingLaw`, default). With x = t/T_r, T_r = approach_time = 2.5 s:

  ṡ = rate·g(x), g(x) = 10x³ − 15x⁴ + 6x⁵ (quintic smootherstep);  s̈ = rate·g′(x)/T_r;
  after T_r: ṡ = rate, s̈ = 0.

Since ∫₀¹g = ½, s(T_r) = rate·T_r/2 = **S_a/2**: the lead-in is half done when the ramp ends, and
the junction s = S_a is reached at t = 2.5 + 1.25 = **3.75 s** after the start (numerically
checked). `trapezoidal` and `exponential` laws exist; the latter is one-shot and logs a warning.

### 3.2 F-3-9: the junction is only C⁰ in velocity

At s = S_a, P′ jumps from (entry − p₀)/S_a to the circle's tangent 2πr·v. At full rate this is a
step in v_d of

  Δv_d = ‖2πr/T·t̂ − (entry − p₀)/approach_time‖ = **0.296 m/s** from the home TCP,

and a_d has a one-tick impulse that the discrete code does not represent. The PD's Kd·ė term turns
the velocity step into a 9 × 0.296 = 2.7 m/s² step in ẍ_cmd. It depends on p₀. **Derived;
magnitude numerically checked** (`check_timing_governor.py` item 5).

---

## 4. The task-space law (`_tick`)

### 4.1 Errors

Per tick, at `rate_hz` = 100 Hz, with Pinocchio quantities of the `fr3_hand_tcp` frame
(LOCAL_WORLD_ALIGNED; the linear rows rotated into `plane_frame`):

  (6.4)  e = sat_{0.08 m}(p_d − p),  ė = v_d − J_v q̇;
         e_R = −R_d·log(R_dᵀR),  ė_R = −J_ω q̇.

- `cart_err_max` = 0.08 m clips ‖e‖ (L955–956), not ė.
- R_d is the orientation **at the first tick** (`_orient_ok`), frozen: the task keeps the starting
  orientation.
- log(R_dᵀR) is the rotation from R_d to R in R_d's frame. Pre-multiplying by R_d expresses it in
  the world, and the minus sign turns it into the correction toward R_d. (Verified; the comment
  L170–175 says an earlier version had the frame and sign wrong.)

### 4.2 Desired task acceleration (PD + feedforward)

  (6.5)  ẍ_cmd = [ a_d + K_p e + K_d ė ;  K_pR e_R + K_dR ė_R ].

K_p = `path_kp_cart` = 20 s⁻², K_d = `path_kd_cart` = 9 s⁻¹, so ζ = 9/(2√20) = 1.006;
K_pR = 5 s⁻², K_dR = 6 s⁻¹ (node defaults). **F-3-8:** the YAML comments give K_p and K_d in N/m
and N·s/m. In an acceleration-level law they are s⁻² and s⁻¹.

**Derivation of the intended error dynamics.** If the joint command q̈ realises J q̈ + J̇ q̇ = ẍ_cmd
exactly (full rank, λ → 0, no clipping, no filter intervention), then ẍ = ẍ_cmd and, with
e = p_d − x,

  ë = p̈_d − ẍ = (p̈_d − a_d) − K_d ė − K_p e.

With a_d = p̈_d this is ë + K_d ė + K_p e = 0: critically damped, ω_n = √20 = 4.47 rad/s. Two
things break the a_d = p̈_d premise: the governor (F-3-3, §5) and the junction (F-3-9).

### 4.3 Damped inverse and null space

  (6.6)  q̈_task = J⁺(ẍ_cmd − J̇ q̇),
  (6.7)  λ² = λ²_min + (λ²_max − λ²_min)(1 − w/w₀)²  if w = √det(JJᵀ) < w₀, else λ²_min,
  (6.8)  J⁺ = Jᵀ(JJᵀ + λ²I₆)⁻¹  (coded as `solve(JJᵀ + λ²I, J)ᵀ`),
  (6.9)  q̈_null = N(−k_N(q − q_home) − d_N q̇),  N = I₇ − J⁺J,
  (6.11) q̈ = clip(q̈_task + q̈_null, ±q̈_max).

- Parameters: λ²_min = 1e-4, λ²_max = 5e-2, w₀ = `manip_thr` = 0.05, k_N = 5 s⁻², d_N = 2 s⁻¹.
- q_home is the configuration at trajectory start (`capture_home`).
- q̈_max = `fr3_control.yaml joint_limits` column 3 = (6.0, 2.585, 3.5, 4.0, 17.0, 5.5, 17.0) rad/s².
- (6.8) is numerically checked against the closed form to 2e-13.

**(6.10) F-3-14: N is not an exact projector.** With the damped J⁺,
J·N = J − JJᵀ(JJᵀ + λ²I)⁻¹J = λ²(JJᵀ + λ²I)⁻¹J, whose singular values are λ²σᵢ/(σᵢ² + λ²) ≤ λ/2.
The posture term leaks into the task by up to **0.005** (λ² = 1e-4) and **0.11** (λ² = 5e-2) m/s²
per rad/s² of null-space command. **Derived + numerically checked.** The leak is small away from
singularities and larger exactly where λ² is raised.

### 4.4 Outputs, reference integration and resets

- **Published:** `/NS_1/qddot_nom` ← q̈ of (6.11) (L1101–1103).
- **Reference state** (L1032–1046). q̇_d and q_d are integrated from q̈ with synchronisation gains
  (k_sync,vel = 5, k_sync,pos = 2) and clamps (|q̇_d| ≤ 2 rad/s, |q_d − q| ≤ 0.5 rad). They are
  published on `/NS_1/q_des_state`, which only `CBFTorqueController` consumes (F-1-9).
- **Resets** (L1075–1098). If ‖e‖ > `hard_reset_thr` = 0.05 m with no obstacle within
  `avoid_near_m` = 0.30 m, q_d and q̇_d snap to the measured state. If ‖e‖ > 0.02 m they are
  blended with α = 0.95.
- **CSV (F-3-7).** It logs the published q̈ as `qddot_des_*` and a separate **open-loop ideal**
  channel `q_nom/dq_nom/qddot_nom`, from `_step_nominal`: the same law on an integrated model that
  ignores the robot, with no `cart_err_max` clip. The ideal `qddot_nom` column shares its name with
  the topic `/NS_1/qddot_nom`, which carries `qddot_des`.
- **ISO stop latch:** q̈ = 0 and the phase is held (σ = 0) until `/NS_1/safety_reset`.
- **Singularity recovery** (`_tick_recovery`, ON). When σ_min(J) < 0.07 and the task error does not
  improve for 3 s, the node runs a joint-space PD (k_p = 40, k_d = 12) along a profiled return to
  q_home, settles, then restarts the trajectory from there.

---

## 5. The phase governor and F-3-3

**Mechanism (Verified).** `_cbf_status_cb` stores `msg.data[4]` = the filter's `d_obs_min`. Then

  σ(d) = 1 for d ≥ d_full; σ_min for d ≤ d_stop; σ_min + (1 − σ_min)(d − d_stop)/(d_full − d_stop)
  in between,

with d_full = 1.5·d_safe = 0.15 m, d_stop = 0.8·d_safe = 0.08 m and σ_min = 0.05. d_safe is read
from the **installed** YAML (F-1-10). A status older than 0.5 s gives σ = 1, full speed. The tick
then calls `_timing.step(actual_dt·σ)` and evaluates (6.1)–(6.3) with the returned (s, ṡ, s̈).

**F-3-3 (Derived, numerically reproduced).** `LinearTimingLaw.step(Δ)` advances s by ṡ·Δ but
returns ṡ = rate (and s̈ in timing-law time). In real time, then, ds/dt = σ·rate, while the code
uses ṡ = rate:

  v_d(code) = P′·rate = v_d(true)/σ,  a_d,centripetal(code) = P″·rate² = a_d(true)/σ².

At σ = 0.3 the commanded feedforward velocity is 3.33× the reference's real velocity and the
centripetal term 11.1×. At σ = 0.05 the factors are 20× and 400× (`check_timing_governor.py`).

**Consequence.** The K_d·ė term uses v_d − J_v q̇, so the PD drives the arm *ahead of* the slowed
reference. On an idealised task-space double integrator, the steady lead is
(K_d/K_p)(1 − σ)|v_full|: 0.059 m at σ = 0.5. At σ = 0.3 the 0.08 m error clip binds and the lead
grows to 0.126 m (illustrative, `check_timing_governor.py` item 4, not the robot). The governor
therefore slows p_d but not the velocity feedforward, which is the opposite of its intent near an
obstacle. The filter still bounds what is executed; the effect is a larger nominal pushing toward
the path.

**F-3-11.** `test/test_phase_governor.py` mirrors `_governor_sigma` with stale constants
(D_FULL 0.05, D_STOP 0.02 m instead of 0.15, 0.08). Its docstring asserts "v_d = P′(s)·ṡ" with σ
carried, and it never calls the timing law. (Verified.)

---

## 6. Shaping the nominal inside the filter (`_qp_tick`)

The filter receives q̈_nom (stale > `nom_timeout` → −k_brake q̇, k_brake = 3 s⁻¹) and, in this
order, modifies it before the QP `min ½‖q̈ − q̈_nom‖²` (chapter 08):

  q̈₁ = fade_gov(fade_zone(q̈_nom))                       (zone first, L1186; governor second, L1228)
  q̈₂ = q̈₁ + B_tan(q̈₁)                                     (L1303–1323)
  q̈₃ = q̈₂ + B_ga(q̈₂) + cut_ga(q̈₂)                        (L1330–1348)
  q̈₄ = q̈₃ + E_esc [+ E_out] + L_lock                       (L1366–1432)

Each bias is computed from the nominal as already modified by the previous steps.

Which terms are on in the hardware default (`effective_params.py`): the zone fade, the state
governor, the tangential bias (gain 0.5), go-around, lateral evasion and livelock. Outrun is off.
`minimal_l1` turns all of them off. The EMA factors are a = α^(Δt/0.01 s) at the 300 Hz tick
(`__init__` L332–336, Verified): a_tan = 0.9^{1/3} = 0.965 (τ = −Δt/ln a ≈ 95 ms) and
a_out = 0.8^{1/3} = 0.928 (τ ≈ 45 ms). The same rule gives the q̇_cbf filter
a_hdot = 0.6^{1/3} = 0.843 (`_smooth_qdot`).

### 6.1 Zone task weight (Verified)

w_task = `ZoneLadder.task_weight(d_obs_min, dt)`. Its target is
1 − (0.5·w_prio(d) + w_hold(d)), clipped to [0, 1]; it falls at once and recovers at 1/0.5 s⁻¹.
Then q̈_nom ← w_task q̈_nom + (1 − w_task)(−k_brake q̇). This is the "hold" rung of 07 §5: the
task is suspended, the barrier rows keep full authority.

### 6.2 State governor (`utils/state_governor.py`, Verified)

With S(x; 0, b) the C¹ smoothstep:
- m_v = minⱼ(bⱼ − |q̇ⱼ|), with bⱼ = 0.65·(the upper envelope ūⱼ(q) if q̇ⱼ ≥ 0, else −l̲ⱼ(q)),
  `fr3_velocity_envelope`; w_v = S(m_v; 0, 0.30);
- w_σ = S(σ_min − 0.05; 0, 0.04) (singularity), w_sc = S(d_sc − 0.02; 0, 0.04) (self-collision);
- w* = min(w_v, w_σ, w_sc). It falls at once and climbs at most 2 s⁻¹ (`governor_resume_s` 0.5).

If the binding term is `sing` or `sc` and `state_governor_directional` is set,
`directional_fade` removes only the component of q̈_nom (and of the velocity, braked by k_brake)
that points *into* each binding row. Otherwise q̈_nom ← w q̈_nom + (1 − w)(−k_brake q̇).

**F-3-15 (Verified + numerically checked).** At q̇ⱼ = 0 exactly, the margin uses the upper envelope.
For joint 4 near its upper reference (−0.1458 rad, which the filter also uses as its position
limit), a stopped joint scores lower than a joint moving at −1e-6 rad/s:

| q₄ − q_ref,up | w (q̇ = 0) | w (q̇₄ = −1e-6) |
|---|---|---|
| −0.10 | 1.000 | 1.000 |
| −0.05 (position-box edge) | 0.809 | 1.000 |
| −0.02 | 0.120 | 1.000 |
| 0 | 0.000 | 1.000 |

The docstring says a parked joint keeps full margin; the existing test covers the lower side only.

### 6.3 Tangential bias (`cbf_qp_assembly.tangential_bias`, Verified)

For each obstacle or self-collision row with h̄ < 0.30 m, take â = a/‖a‖. Then:
- q̈_⊥ = q̈_nom − (q̈_nomᵀâ)â is the joint-space part of the nominal orthogonal to the row;
- if that part is small, a fallback along the velocity's orthogonal part q̇_⊥ blends in through a
  smoothstep s;
- the row contributes w·(s q̈_⊥ + (1 − s) fallback), with w = clip((0.30 − h̄)/0.30).

The sum is multiplied by the gain 0.5, norm-capped, and EMA'd with a_tan. It amplifies the
nominal's sideways motion near a barrier, so the arm slides instead of stalling.

### 6.4 Lateral evasion (`cbf_evasion.py`, built at 50 Hz in `_escape_row`) — Verified

- **Authority** (`normal_brake_authority` L86–120): a_max = max(0, η Σⱼ max(aⱼ lbⱼ, aⱼ ubⱼ)),
  with η = `lateral_evasion_authority` = 0.5 and the **static** acceleration box ±q̈_cap (not the
  per-tick velocity-tightened box; the `_escape_row` docstring says so). Unit: m/s² along n̂.
- **Urgency** (`evasion_urgency` L123–176): ḣ = aᵀq̇ − v_o (raw q̇; v_o is the conditioned closing
  speed of 07 §3.4). w = 0 if ḣ ≥ 0, and w = 1 if a_max ≤ 1e-9. Otherwise the stopping distance
  s = ḣ²/(2a_max) and r = s/max(h̄, 0.01), where h̄ is the **tightened** barrier value of 07 §3.5.
  w = 0 for r ≤ 0.7, 1 for r ≥ 1, and smoothstep((r − 0.7)/0.3) between. Dimensions:
  (m/s)²/(m/s²) = m, so r is dimensionless.
- **Direction** (`escape_direction` L179–256), base frame. It needs ‖v_vec‖ ≥ `lateral_evasion_v_min`
  = 0.15 m/s; then v̂ = v_vec/‖v_vec‖ and d_⊥ = d − (dᵀv̂)v̂ with d = p_r − p_h.
  - If ‖d_⊥‖ ≥ 0.02 m: ê = d_⊥/‖d_⊥‖, i.e. away from the obstacle's line of travel, toward the
    control point.
  - Otherwise (head-on): build a basis (u₁, u₂) of the plane ⟂ v̂ and take ê = the top
    eigenvector of MMᵀ, with M = [u₁ᵀJ_p; u₂ᵀJ_p]. This is the 2-norm leverage direction. Its
    sign makes êᵀd_⊥ ≥ 0, with a deterministic tie-break.
- **Evidence (Derived).** The trigger w uses the conditioned v_o, but the direction uses the raw
  `v_vec`, which is **not** gated on `frames_seen` (only on ‖v_vec‖ ≥ 0.15). A young track's prior
  velocity can therefore set the direction while the residual floor supplies the trigger
  (F-3-20).
- **Joint bias** (`evasion_bias` L259–306): b = 1.5 Σₖ wₖ gₖ/‖gₖ‖, with gₖ = J_pᵀêₖ, rows with
  ‖gₖ‖ < 1e-3 skipped, and the sum norm-capped at 3 rad/s². The builder keeps it only if
  ‖b‖ > 0 (L3769–3777). **F-3-17:** a normalised **transpose** mapping. Its Cartesian effect
  J_pJ_pᵀê is not parallel to ê, unlike the DLS used by outrun and go-around.
- **F-3-16 (Verified, L1366–1371):** E_esc is EMA'd but added only while `con.esc_bias` is not
  None. When it disengages, the filtered bias (≤ 3 rad/s²) leaves q̈_nom in one tick.

### 6.5 Go-around (`go_around.py`, ON) — Verified

Geometry: (h̄, J_p, n̂) of the obstacle row with the smallest **tightened** h̄ (`around_best`,
builder L3107). With no row, h = ∞ and the step only decays the bias. Velocity:
v = J_p q̇_cbf (filtered q̇, L1336). Δt = 1/300 s. `go_around_step` L86–160:
- **Weight:** w* = smoothstep((m − h̄)/m), m = 0.15 m. w moves toward w* by at most 4·Δt per
  tick (`w_rate` = 4 s⁻¹, a hard-coded default the node does not pass).
- **Engagement hysteresis:** engaged when h̄ < 0.15 m; released, and the side forgotten, when
  h̄ > 1.5 × 0.15 = 0.225 m.
- **Side:** P = I − n̂n̂ᵀ; intent i = P(v + T·J_p q̈_nom), T = 0.3 s.
  - A kept side t̂ is re-projected, P t̂/‖P t̂‖, and turned toward i at β = Δt/1 s **only if**
    iᵀt̂ > 0; an opposite intent is ignored.
  - With no side: t̂ = i/‖i‖ if ‖i‖ ≥ 0.03 m/s (hard-coded `t_min`). Otherwise t̂ = ±(P·top left
    singular vector of P J_p), with the sign along P v.
- **Push:** a_t = w·clip(3(0.25 − t̂ᵀv), ±2.5) m/s². raw = J_pᵀ(J_pJ_pᵀ + 0.05²I)⁻¹(a_t t̂),
  norm-capped at 8 rad/s². The **returned bias** moves toward raw by at most 10·Δt = 0.033 rad/s²
  per tick, so the bias fades out instead of stepping.
- **Inward cut:** if a_in = n̂ᵀJ_p q̈_nom < 0, then cut = J_pᵀ(J_pJ_pᵀ + 0.05²I)⁻¹(−w·a_in·n̂).
  - It is **not** rate-limited, and it becomes zero in one tick when the engagement releases or
    the row disappears (**F-3-16**).
  - **Derived:** J_p·cut = M(M + λ²I)⁻¹(w|a_in|n̂), M = J_pJ_pᵀ. The inward component is reduced
    by less than w|a_in|, and if n̂ is not an eigenvector of M the cut also has a tangential part.
- **Derived (F-2-11 reach):** the geometry is the builder's row normal, so a held row
  (p_h = 0, 07 §8.1) that is the closest one gives go-around the base-radial normal too.

### 6.6 Outrun evasion (OFF everywhere) — Verified

`evasion_direction.py` + `ConstraintBuilder._outrun_bias` L3901–3961:
- **Retreat speed available:** v_avail(a) = max(0, Σⱼ max(aⱼ q̇minⱼ, aⱼ q̇maxⱼ)), the best
  separation speed along n̂ inside the velocity box. q̇max = 0.7·q̇_lim and q̇min = −q̇max.
- **Ratio:** r = v_c / (0.8·max(v_avail, 0.3)), where v_c is the conditioned v_o.
  **Blend:** w = smoothstep from r = 0.6 to r = 1.
- **Lateral direction:** the unit vector ⟂ v_obs that maximises v_avail(êᵀJ_p), searched over 72
  angles on a half circle. The half circle suffices because v_avail(−g) = v_avail(g) for a
  symmetric box. Its sign points toward d_⊥.
- **Escape:** ê = normalise((1 − w)n̂ + w ê_lat), so for 0 < w < 1 it still contains a +n̂
  component (Derived).
- **Bias:** gain·DLS_{0.05}(accel·m·w·gate·ê), with m = clip(v_c/0.5, 0, 1), accel = 1.5 m/s²,
  and gate = smoothstep((0.30 − h̄)/(0.33·0.30)) (zero above h̄ = 0.30 m). It is capped at
  3 rad/s² per row and again on the sum.

### 6.7 Livelock escape (ON in the default launch)

- **Detector** (`LivelockDetector.update`, Verified). "blocked" (L1399–1402) means all of:
  - n_c > 0;
  - a livelock direction exists (`con.livelock_dir`, built only for obstacle rows with
    h̄ < `livelock_engage_gap` = 0.25 m);
  - ‖q̈_nom,raw‖ > 1 rad/s², taking the command as received, after the stale-nominal substitution;
  - ‖q̈_safe,prev − q̈_nom,prev‖ > 1.5 rad/s². Both are from the previous tick, and q̈_nom,prev
    includes every bias, the livelock bias itself among them (L1433).

  "moving" means ‖q(t) − q(t₀)‖₂ > 0.05 rad, with t₀ the oldest sample in a 0.5 s window
  (`ProgressWindow.push`).

  States:
  - IDLE → STALLING when blocked and not moving.
  - STALLING → IDLE if moving or not blocked; STALLING → ESCAPING after 1 s.
  - ESCAPING: mag = min(t/0.3 s, 1). `moving` is ignored. It ends when not blocked or after 2 s,
    passing to RELEASING with mag decreasing at 1/0.3 s⁻¹.
  - RELEASING → COOLDOWN (2 s) → IDLE.
- **Direction** (`_livelock_direction`, Verified L3854–3899): ê ⟂ (v_obs, or n̂ when the obstacle
  is slow, ‖v_vec‖ < 0.15), the fastest in the velocity box (`lateral_direction`, v_min = 0);
  g = J_pᵀê projected onto the null space of the closest row and the ≤ 3 rows with h̄ < 5 cm,
  g ← g − Aᵀ(AAᵀ)⁺Ag, normalised. To first order the nudge therefore changes no binding barrier.
  **Derived:** for a static obstacle, d = p_r − p_h = d_ctr·n̂ is parallel to the axis n̂, so
  d_⊥ = 0 and the sign of ê comes from the "largest component positive" tie-break, not from the
  geometry.
- **Bias:** L ← a_tan L + (1 − a_tan)·2·mag·dir, added whenever n_c > 0, so it fades properly
  (Verified L1419–1432).

### 6.8 What reaches the QP

The QP's linear term is −q̈_nom after all of the above (L1440). The barrier rows of chapter 07 are
unaffected by any of these biases: they move the target, not the feasible set.

---

## 7. The RL commander (`rl_policy_commander`, `motion_source:=rl`)

*(All Verified in the audit, C-018, unless marked.)*

- **Observation** (`ObsSpec`, `build_observation` L174–233), float32, no normalisation in the
  node, NaN/±∞ → 0. Layout:
  - q (7), q̇ (7), TCP position (3, `fr3_hand_tcp`, FK), target (3), obstacle centre (3),
    min(d_min, clip) (1);
  - then v_obs (3) if `obs.obstacle_velocity`;
  - then (d, n̂) per policy control point, `CP_WIDTH` = 4, d clipped.

  dim = 2·7 + 10 + 3 + 4·n_cp. With `franka_rl/config.yaml` (`obstacle_velocity: true`, 6 control
  points) this gives 14 + 10 + 3 + 24 = **51**. clip = `obs.clip_distance` or the config's
  `cbf_obstacle_horizon`, default 1.2.
- **Obstacle decoding (Verified, `_obs_cb` L436–460):** entries are kept if valid, with finite d
  and ‖direction‖ ≥ 0.5; `p_h` is not validated. `nearest_obstacle` takes the minimum-d entry, and
  `obstacle_centre` = p_h − r_obs·n̂ (r_obs = 0.08).
- **F-3-10 (Verified + numerically checked):** for a held entry (F-2-11), p_h = 0, so the centre
  is −r_obs·n̂, a point 0.08 m from the base origin. v_obs = (c_k − c_{k−1})/Δt with Δt the
  **policy tick** (`now − _obs_xyz_prev_t`, monotonic clock, 100 Hz; `_tick` L570–579), not the
  message interval. The jump is ‖p_h,prev‖/0.01 s = **71 m/s** for ‖p_h‖ = 0.71 m, unclipped. The
  policy's response is Unverified.
  - **Derived, independent of the defect:** the tick samples a 90 Hz stream at 100 Hz, so about
    1 tick in 10 sees an unchanged snapshot and reports v_obs = 0. The other ticks report the
    motion of one message interval (≈ 11 ms) divided by 10 ms, ≈ 1.11× the true speed. v_obs is
    therefore a sawtooth.
- **Gates in `_tick`:**
  - zeros during `warmup_s` = 3 s;
  - zeros if the joint state is invalid or older than 0.1 s;
  - distances older than `distance_timeout` = 0.5 s → zeros, and the v_obs history is reset;
  - no snapshot yet, or no entry → a synthetic obstacle at (1.5, 0, 0.5);
  - a single target with `stop_on_success` → zeros on arrival.

  `_target_cb` ignores the `PointStamped` frame_id (L474–478).
- **Action → command** (`action_to_qddot` L296–314): q̈_nom = clip(a, −1, 1)·q̈_max·s, where:
  - q̈_max = `fr3_control.yaml joint_limits` column 3 (§4.3; joints 5 and 7 can be asked for
    17 rad/s², and the filter caps at `qddot_max_abs` = 10);
  - s = `action_scale`, clamped to [0, 1] with a warning.

  It is published as q̈ on `/NS_1/qddot_nom` at `env.control_rate_hz` = 100 Hz.
- **F-3-19 (Verified):** `ld.robot_link_name` is the segment's end link (`perception_msgs.py`
  L212–216), so the wire names are fr3_link4…fr3_link8. The policy control points are
  `c.get('robot_link', c['body'])` = fr3_link3, 4, 5, 6, 7, and fr3_hand → fr3_link8.
  - `cp:fr3_link3` never matches and stays at (clip, 0, 0, 0).
  - `cp:fr3_linkk` (k = 4…7) receives the minimum over the robot samples of the segment that
    *ends* at link k, i.e. between links k−1 and k, while the simulated sphere sits at the origin
    of link k. The config comment "fr3_link3 mirrors segment 3" assumes a naming that the wire
    does not use.
- **F-0-10 (Verified code, Derived outcome):** `find_sim_root` walks up from the node's file
  looking for `franka_sim/config.yaml`; the training package is `franka_rl/`. With the launch
  default `rl_onnx_model: ""` and no `sim_root`, `find_latest_model('')` returns `''` and
  `__init__` raises `RuntimeError` (L196–207), unless a `franka_sim/` directory exists above the
  installed file.

---

## 8. Pipeline B velocity commanders and the legacy avoidance helpers

*(Verified in the audit, C-018, except where marked.)*

- **`ee_pentagon_velocity_commander`** (`velocity_cbf_control_stack.launch.py` L330–335, Node with
  **no parameters**, so the node defaults apply). `_timer_cb` at 200 Hz, after a 2 s warm-up of
  zeros:
  - p and J_pos = R_refᵀJ_v of `fr3_hand_tcp`, in `plane_frame` = `fr3_link0`;
  - `PentagonTrajectory.evaluate(t − warmup)`: constant arc-length speed (s ∝ t) along straight
    sides joined by quadratic-Bézier corner blends;
  - v_cmd = v_d + K_p(p_d − p), K_p = 1.5 s⁻¹, position only, no orientation;
  - q̇_raw = J_posᵀ(J_posJ_posᵀ + λ²I₃)⁻¹v_cmd, with λ = max(0.05, 0.1·clip((cond(J_pos) − 50)/250,
    0, 1)) (`kinematics.dls_solve`: a **max**, not a sum);
  - then q̇ ← ½(1 − cos(π t/2 s))·q̇_raw (cosine start envelope), clipped to ±0.3 rad/s, EMA'd as
    0.85·prev + 0.15·new, and rate-limited at 3 rad/s² (0.015 rad/s per tick);
  - published on `/NS_1/tracking_qdot`;
  - no null space and no obstacle logic (the velocity CBF filter is chapter 10).
  - **F-3-18 (Verified):** the node docstring promises minimum-jerk C² sides; the trajectory is
    constant-speed with C¹ Bézier blends (`PentagonTrajectory` says C¹).
- `ee_circle_…` and `ee_random_waypoints_…`: both call `dls_solve` with a `kp_cart` law (Verified
  by grep), and **no** launch file starts either (Verified by grep). Their reference profiles and
  filter chains were **not** re-read in the audit; nothing in this chapter depends on them.
- **`utils/avoidance.py` (Verified, F-3-18)** is imported only by `test/test_avoidance.py`. Its
  `tangential_redirect` (a' = a + γ·approach·n̂ + γ·min(approach, r_max)·t̂) and
  `feasibility_beta_target` are not part of any running pipeline. The live "avoidance-first"
  behaviour is the set of filter-side biases in §6.

---

## 9. Integration

| Item | Value |
|---|---|
| Commander inputs | `/NS_1/joint_states` (AUTO; arm joints mapped once by name, F-2-5); `/NS_1/cbf_status` (`d_obs_min`, governor); `/NS_1/iso_safety`; `/NS_1/safety_reset` |
| Commander outputs | `/NS_1/qddot_nom` (100 Hz), `/NS_1/q_des_state`, `/NS_1/ee_desired`, `/NS_1/ee_actual`, CSV |
| Start-up | 3 s warm-up of zeros, then the lead-in from the measured TCP |
| Failure | joint state > 0.1 s old → q̈ = 0; the filter brakes on a stale nominal (> `nom_timeout`) |
| Coupling | the governor reads the filter's `d_obs_min` (which includes entries the filter ignores, 07 §3.1); the filter reshapes the nominal (§6); the QP enforces the rows (07) |

## 10. Practical example: the first 6 seconds from the home pose

Using `check_timing_governor.py`:
1. t = 0–3 s: warm-up, q̈ = 0.
2. t = 3 s: lead-in from the home TCP (0.307, 0, 0.487) toward (0.40, 0.25, 0.45), 0.269 m. ṡ ramps
   over 2.5 s; at full rate the lead-in moves at 0.108 m/s.
3. t = 6.75 s (3.75 s after the start): junction. v_d steps by 0.296 m/s (F-3-9), so K_d·Δv ≈
   2.7 m/s² appears in ẍ_cmd for one tick and then decays with the PD.
4. On the circle: 0.262 m/s, centripetal a_d = 0.274 m/s².
5. An obstacle at d_obs_min = 0.115 m gives σ = 0.525. p_d slows to 0.137 m/s, but v_d stays at
   0.262 m/s (F-3-3), so the PD pushes ahead by about (9/20)(1 − 0.525)·0.262 = 0.056 m.

## 11. Consequences of modification

| Change | What it affects |
|---|---|
| Fixing F-3-3 (scale ṡ, s̈ by σ, plus σ̇ terms) | v_d and a_d near obstacles; the lead and the push toward the path; `test_phase_governor` would need its constants updated (F-3-11). |
| Path or timing parameters | the cycle speed and centripetal load; the junction step (F-3-9) depends on approach_time and the start pose. |
| K_p, K_d, cart_err_max | the error dynamics (§4.2); how hard the nominal pushes against the barrier; the soft/hard reset thresholds. |
| λ² schedule | null-space leakage (F-3-14); behaviour near singularities; singularity recovery triggers. |
| `governor_r_*`, `d_safe` | σ(d) is read from the installed YAML (F-1-10); it interacts with the zone ladder's task weight, which uses the same `d_obs_min`. |
| Filter-side bias flags | the objective only; but F-3-16 means that disengagement steps reach the torque. |
| The RL observation layout | it must match `franka_rl` exactly (the mirror tests skip, F-0-10). F-3-10 and F-3-19 are input-side. |
| Tests | `test_phase_governor`, `test_phase_governor_staging`, `test_trajectory_trace`, `test_trajectory_overlay`, `test_state_governor`, `test_directional_fade`, `test_go_around`, `test_cbf_lateral_evasion`, `test_evasion_direction`, `test_cbf_outrun_evasion`, `test_livelock`, `test_rl_policy`, `test_avoidance` (names; contents read at interface level only). |

## 12. Findings from this chapter

| ID | Short title | Section | Reproduced by | Still needs |
|---|---|---|---|---|
| F-3-3 | the governor slows s but not ṡ in v_d, a_d | §5 | `check_timing_governor.py` | runtime |
| F-3-7 | the CSV `qddot_nom` (ideal) collides with the topic name | §4.4 | — | — |
| F-3-8 | Cartesian gains documented in N/m | §4.2 | — | — |
| F-3-9 | lead-in/circle junction steps v_d by 0.296 m/s | §3.2 | `check_timing_governor.py` | — |
| F-3-10 | the RL commander turns a held entry into a centre at the base and a v_obs spike (71 m/s at the 100 Hz tick) | §7 | `check_nominal_shaping.py` | runtime (policy response) |
| F-3-11 | `test_phase_governor` stale constants; timing law not exercised | §5 | — | — |
| F-3-14 | damped null-space projector leaks into the task | §4.3 | `check_nominal_shaping.py` | — |
| F-3-15 | state governor scores a stopped joint against its upper envelope | §6.2 | `check_nominal_shaping.py` | — |
| F-3-16 | evasion/outrun bias and go-around cut drop in one tick | §6.4–6.5 | code reading | runtime |
| F-3-17 | transpose vs DLS mappings across the biases | §6.4 | code reading | — |
| F-3-18 | `avoidance.py` test-only; velocity commander docstring wrong | §8 | grep | — |
| F-3-19 | RL `fr3_link3` control point never filled; the other slots shifted by one segment | §7 | config + code (audit) | — |
| F-3-20 | lateral-evasion direction from an ungated `v_vec` | §6.4 | code reading (audit) | — |

## 13. Audit record (Stage 3 audit, 2026-11-11, C-017 to C-019)

Every equation, parameter and implementation claim in §6.4–§6.7, §7 and §8 was re-read in the
source by the main session, not taken from a subagent report. §6 (intro), §6.1–§6.3 and the
references to ch. 07 were re-checked where they touch those sections. Superseded statements:

| Where | Superseded wording | Correction | Evidence |
|---|---|---|---|
| §6 intro | "q̈_nom ← fade_zone ∘ fade_gov (q̈_nom) + …" (composition order reversed; biases written as independent of each other) | the zone fade is applied first, then the governor; each bias is computed from the nominal as modified so far | `_qp_tick` L1186, L1228, L1303–1432 |
| §6 intro | EMA factors "Inferred, subagent reading of `__init__` L339–341" | Verified at L332–336; added a_hdot for q̇_cbf | source |
| §7, F-3-10 | "jumps by ‖p_h,prev‖/Δt (64 m/s at 90 Hz …)" | Δt is the 100 Hz policy tick: 71 m/s. A sawtooth exists even without the defect | `_tick` L570–579; `check_nominal_shaping.py` already printed both values |
| §7, F-3-19 | "(Config Verified.)", with the matching code not re-read | Verified end to end, including the shift of the other slots by one segment | `perception_msgs.py` L212–216; `obs_spec_from_config`; `control_point_geometry` |
| §6.4–§6.7, §7, §8 | "Inferred (subagent reading)" labels | replaced by Verified formulas, with details the drafts lacked: static vs tightened box, h̄ tightened, ungated v_vec (F-3-20), hard-coded `w_rate`/`t_min`, partial cut cancellation, F-2-11 reaching go-around, the livelock sign tie-break, the RL gates, `find_sim_root` outcome | source |

**Confirmed unchanged** from the drafts: F-3-15 (as already corrected in Stage 3), F-3-16, F-3-17
and F-3-18.

**Not re-read** (no claim in this chapter depends on them): the reference profiles of
`ee_circle_velocity_commander` and `ee_random_waypoints_velocity_commander`; `avoidance.py`
beyond `tangential_redirect` and its import graph; `avoidance_math.py`.
