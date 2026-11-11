# 07 — The obstacle barrier: from a `LinkDistance` entry to a linear row in q̈

> **Source baseline:** `ba68077`. Written in Stage 3 (2026-11-11). Labels as in
> [00_INDEX.md](00_INDEX.md) §2. Line numbers refer to the baseline.
>
> **Reproducible checks.** Every number labelled *numerically checked*, *reproduced* or *measured
> in the builder* is recomputed by a script in
> [tools/stage3_verification/](tools/stage3_verification/README.md). `run_all.sh` runs them all and
> is read-only. Two scripts drive the **real** `ConstraintBuilder` with the real Pinocchio
> kinematics inside the `franka_ros2` container. The others run on the host.

This chapter covers one family of rows in the safety filter's QP: the **obstacle rows** (group
`G_OBS`). Each valid entry of `/cbf/per_link_distances` becomes one linear inequality on the joint
acceleration q̈. The other families share the same form but are built from joint states: joint
limits, self-collision, singularity, the rate caps and the self-shadow barrier. They are chapter 08
(Stage 4). So are the QP itself, its slack and its fallbacks. What the filter does to the
*nominal* command before the QP (governor, tangential bias, go-around, evasion, livelock) is
chapter 06 §6.

Code map:

| Role | `file::function` (package `franka_experiments/franka_experiments/`) |
|---|---|
| Message → `Obstacle` tuples | `nodes/cbf_safety_filter.py::_on_distances` L820–926 |
| Row construction, 50 Hz | `utils/cbf_state_rows.py::ConstraintBuilder.build` L2335–3816; obstacle loop L2489–3158 |
| Per-control-point estimators | `ConstraintBuilder._obstacle_speed`, `_obstacle_speed_tracked`, `_obstacle_identity_changed`, `_fast_track`, `_retreat_cap` (L3963–4349) |
| Margin helpers | `cbf_state_rows.py` L152–946, L1496–1765 |
| Gain schedule | `utils/cbf_zones.py` (`ZoneLadder`, `ladder_from_params`) |
| Right-hand side per QP tick (300 Hz) | `utils/cbf_qp_assembly.py::build_row_rhs` L151–249 |
| Unused alternative | `utils/cbf_constraints.py` (§11, F-3-1) |

---

## 1. Intuition

The arm must keep each of its **control points** (11 spheres along the links, chapter 04 §4) at least
`d_safe` = 0.10 m from every obstacle surface the camera sees. A position constraint cannot be
enforced directly by choosing an acceleration: acceleration changes velocity, and only velocity
changes position. The barrier is therefore a **second-order** condition. It asks the arm to
decelerate its approach early enough that the gap never falls below `d_safe`, given how fast the
gap is shrinking now.

Three things make the real implementation larger than the textbook HOCBF:

1. **The obstacle moves and is measured badly.** The gap comes at 90 Hz from a depth camera, with
   latency, noise and occasional drop-outs. Its rate of change has to be estimated, by two
   estimators combined with `max`. Most of `build` is about how much to trust that estimate:
   gates, medians, deadbands, guards and margins.
2. **The measurement is a surface point, not an object.** `closest_point_human` is an argmin over
   pixels. It hops between surface patches and can belong to a different body from one frame to
   the next. The normal n̂ is rebuilt from it every frame.
3. **Everything that loosens must be earned.** The code's stated convention is that a term may
   tighten the row freely but loosen it only with evidence. A closing speed is clamped to its
   approaching half unless a confirmed track says otherwise; recovery of h is rate-limited; and κ
   is applied only to confirmed tracks. §8 shows two input defects (F-2-11, F-2-10) that break
   this convention from upstream.

Without the barrier the commander's nominal acceleration (chapter 06) would be applied unchanged.
Without the conditioning, the row would chatter at the camera rate. The comments record measured
2–3 rad/s² steps per tick from velocity noise on a static obstacle.

---

## 2. Mathematical foundations

### 2.1 Frames and symbols

All Cartesian quantities are in the robot base frame `fr3_link0` (= Pinocchio world; chapter 04
§2). Units: m, s, rad.

| Symbol | Meaning | Unit | Source |
|---|---|---|---|
| q, q̇, q̈ | arm joint position, velocity, acceleration (7) | rad, rad/s, rad/s² | `JointSnap` |
| p_r | control-point position (on the segment **axis**) | m | `LinkDistance.closest_point_robot` |
| p_h | closest observed obstacle point | m | `LinkDistance.closest_point_human` |
| d | **surface gap** published by perception | m | `LinkDistance.distance` |
| d_ctr = ‖p_r − p_h‖ | centre-to-point distance | m | computed in `build` |
| n̂ = (p_r − p_h)/d_ctr | unit normal, obstacle → control point | – | `build` L2509–2513 |
| J_p (3×7), J̇_p | control-point Jacobian and its time derivative (LOCAL_WORLD_ALIGNED) | m/rad, m/(rad·s) | `CBFKinematics.control_point_jacobian` |
| v_o, a_o | obstacle surface-point velocity and acceleration | m/s, m/s² | tracker / residual |
| v_n = n̂ᵀv_o | obstacle velocity along n̂ (> 0 = toward the control point) | m/s | §3.5 |
| d_safe | barrier offset | m | 0.10 (all configurations) |

### 2.2 The barrier function h

Perception publishes the **surface gap** (chapter 05 §3.2):

  (7.1)  d = d_ctr − r − m(o),  with r = 0.05 m (sphere radius) and m(o) the mask-dilation margin
  in metres at the obstacle pixel's depth (05 §3.4, F-2-8).

The barrier function is

  (7.3)  h = d − d_safe,  safe set C = {h ≥ 0}.

**Verified** (`build` L2586): `h_raw = ob.d − P.d_safe`. The code does **not** use d_ctr as the
barrier argument. The comment at L2540–2553 explains why: d_ctr omits r and the margin and would be
optimistic by 6–16 cm.

### 2.3 First derivative

Differentiate d_ctr = ‖p_r − p_h‖ (n̂ as defined in the §2.1 table):

  ḋ_ctr = n̂ᵀ(ṗ_r − ṗ_h) = n̂ᵀJ_p q̇ − n̂ᵀv_o.

With r constant, and treating m(o) as constant over the derivative (an assumption, see below):

  (7.4)  ḣ = aᵀq̇ − v_n,  a := J_pᵀn̂ ∈ ℝ⁷ (the row of A),  v_n := n̂ᵀv_o.

**Signs:** n̂ points from the obstacle to the robot. Robot motion along +n̂ increases h, so aᵀq̇ > 0
means separating. Obstacle motion along +n̂ (toward the robot) gives v_n > 0, closing.

**Implementation:** a = `n_w @ Jp` (L2526). In ḣ the code uses v_n = the row's `v_obs` (§3.5).
`_obstacle_speed_tracked`'s docstring derives the same sign: "plain projection, no sign flip".
**Verified.**

**Assumption, Derived:** m(o) depends on the depth Z of the obstacle pixel (it is a fixed pixel
count converted at Z, 05 §3.4). So ḋ = ḋ_ctr − (∂m/∂Z)Ż. The term is ignored everywhere. It is
nonzero only when the obstacle point moves in depth relative to the camera. Its sign depends on
whether the point moves toward or away from the camera, which is independent of whether it
approaches the robot. With the 05 §3.4 numbers, m ≈ 0.028·Z for body points (2.8 cm at 1 m), so
the error is about 3 % of the depth speed |Ż|: about 3 mm/s at |Ż| = 0.1 m/s. **Derived**
(magnitude) / **Unverified** (no measurement).

> *Audit (C-018):* an earlier version stated that "an approaching obstacle (Ż < 0 when the camera
> faces the robot) makes the published d shrink slightly more slowly". That sign does not follow,
> because approaching the robot does not fix the sign of Ż. Superseded.

### 2.4 Second derivative and the curvature term

Differentiate (7.4). The normal itself rotates. For a locally spherical obstacle of curvature radius
r_o with centre c, n̂ = (p_r − c)/‖p_r − c‖ and ‖p_r − c‖ = d_ctr + r_o =: ρ_c, so

  ṅ̂ = (I − n̂n̂ᵀ)v_rel / ρ_c,  v_rel := J_p q̇ − v_o,

and

  (7.5)  ḧ = aᵀq̈ + n̂ᵀJ̇_p q̇ + κ − n̂ᵀa_o,  κ := ṅ̂ᵀv_rel = ‖(I − n̂n̂ᵀ)v_rel‖² / ρ_c ≥ 0.

Write c := n̂ᵀJ̇_p q̇ (the code's `jdot_qdot`, L2958). κ is "the part of ḧ that tangential motion
produces for free": sliding around an obstacle bends the normal and raises ḧ.
**Derived;** the code comment L2960–2972 states the same formula.

**What the shipped rows use:** ḧ ≈ aᵀq̈ + c. κ is OFF in every configuration
(`enable_curvature_term: false`). The obstacle-acceleration term −n̂ᵀa_o is never included, by
choice (L2975–2978: a_o is the noisiest tracker output). **Verified** (`effective_params.py`).

### 2.5 Relative-degree-two HOCBF

h has relative degree 2 with respect to q̈. The exponential/high-order CBF construction chains two
class-K conditions with linear gains α₁, α₂ > 0:

  ψ₀ = h,  ψ₁ = ψ̇₀ + α₁ψ₀,  ψ₂ = ψ̇₁ + α₂ψ₁ ≥ 0
  ⇔  ḧ + (α₁ + α₂)ḣ + α₁α₂ h ≥ 0.

With k₁ := α₁ + α₂ and k₀ := α₁α₂:

  (7.6)  ḧ + k₁ḣ + k₀h ≥ 0,  s² + k₁s + k₀ = (s + α₁)(s + α₂).

The guarantee: if ψ₁(0) ≥ 0 and h(0) ≥ 0, then ψ₂ ≥ 0 gives ψ₁ ≥ 0 for all t (comparison lemma
on ψ̇₁ ≥ −α₂ψ₁), and then h ≥ 0 for all t (same lemma on ψ̇₀ ≥ −α₁ψ₀). **This requires real
α₁, α₂ > 0, i.e. k₁² ≥ 4k₀.**

Substituting (7.4) and (7.5), with h replaced by the tightened h̄ of §3.6, gives the row:

  (7.7)  aᵀq̈ ≥ −k₁(aᵀq̇ − v_n) − k₀h̄ − c − κ.

This is linear in q̈: one row of A, one entry of the bound vector. The QP stores it as
`G = [−A | −m]` and `G x ≤ h_qp`, i.e. −aᵀq̈ − m·s ≤ h_qp ⇔ aᵀq̈ + m·s ≥ −h_qp, with
`h_qp = k₁ḣ + k₀h̄ + c (+ κ + b_ff)`. Here s ≥ 0 is the family slack (chapter 08) and m its
multiplier (§5.3). **Verified** (`build` L3600–3726, `build_row_rhs` L221). The direction of the
inequality is set by `cbf_qp_assembly.build_osqp_bounds` L146–147 (lower −∞, upper h_qp on the CBF
rows). The objective is `build_qp_objective`: P = I on q̈ and `qvec = −q̈_nom` (`_qp_tick` L1440),
i.e. ½‖q̈ − q̈_nom‖² up to a constant, plus ½ρ s² per family slack (read in the audit, C-018).

**The shipped gains.** k₀ = 25 s⁻², k₁ = 10.5 s⁻¹: k₁² − 4k₀ = 10.25 > 0, so
α = 6.851 and 3.649 s⁻¹, and ζ = k₁/(2√k₀) = 1.05. The zone-ladder rungs (§5) are notice
(12, 7: α = 4, 3), active (25, 10.5) and priority/hold (50, 14). The last has **k₁² − 4k₀ = −4,
complex roots** (F-3-5). Reproduced in `check_hocbf_gains.py`.

**(7.8) Why real roots matter, and how much complex roots cost.** Write the closed loop of a binding
row as ḧ + k₁ḣ + k₀h = u(t) with u ≥ 0 (the slack of the inequality). The solution is the free
response plus ∫g(t − τ)u(τ)dτ, where g is the impulse response of 1/(s² + k₁s + k₀). With real
roots, g(t) = (e^{−α₂t} − e^{−α₁t})/(α₁ − α₂) ≥ 0, so a positive u can only raise h: that is the
comparison property. With complex roots, g(t) = e^{−ζωt} sin(ω_d t)/ω_d has negative lobes, so the
same positive u can later push h below its free response. The deepest relative undershoot is
exp(−ζπ/√(1 − ζ²)), which is **2.8 × 10⁻¹⁰** at ζ = 0.99. So the formal guarantee is lost on the
priority/hold rungs, but the violation it admits is numerically negligible. The blended gains enter
this region for priority weight w > 0.5044, i.e. only for gaps below 0.0499 m, more than 5 cm inside
d_safe. **Derived;** numerically checked (`check_hocbf_gains.py`, items 2–3).

**(7.9) Where the row starts to push.** For a static obstacle (v_n = 0) and ignoring c, the bound
−k₁aᵀq̇ − k₀h̄ becomes positive (it demands deceleration) once

  h̄ < (k₁/k₀)·|ḣ|,  k₁/k₀ = 0.42 s (shipped), 0.583 s (notice rung), 0.28 s (priority).

An arm walking in at 0.12 m/s is first resisted at d = d_safe + margins + 0.42 × 0.12 = 0.15 m plus
margins. `cbf_zones.py`'s docstring makes the same point and quotes a measured first slack at
d = 0.227 m. **Derived;** the ratios are checked in `check_hocbf_gains.py`.

**(7.11) Rest speed inside the barrier.** With h̄ < 0 and a binding row, the k₀ term keeps
accelerating the point away until k₁(v_n − v_sep) + k₀|h̄| = 0, i.e. it settles at
v_sep = v_n + (k₀/k₁)|h̄|. h̄ contains speed-proportional margins (§3.6), so this excess grew with
the approach speed. The code floors h̄ **inside the k₀ term only** at
h_pen = −Δv·k₁/k₀ (`retreat_excess_speed_max` Δv = 0.25 m/s; `build` L3662–3668,
`build_row_rhs` L214–220). This bounds the excess to Δv for every rung. **Derived +
numerically checked** (`check_hocbf_gains.py` item 7).

A 1-D closed loop (a point robot with a one-row QP and an identity cost) against an obstacle closing
at 0.3 or 1.0 m/s from h = 0.30 m keeps min h ≥ 0 with the shipped gains. The point ends up
retreating at exactly the obstacle's speed (`check_hocbf_gains.py` item 8, illustrative).

---

## 3. Implementation: the obstacle loop step by step

`build(js, obs, now)` runs at `cbf_update_rate_hz` = 50 Hz in its own timer
(`_update_constraints`). It returns `None`, so no rows exist, when the newest distance message is
older than `distance_timeout` = 0.5 s. It first calls `kin.update(q, q̇, with_jdot=True)`: one FK,
Jacobian and J̇ pass. For each `Obstacle` in `obs.items`, in message order:

### 3.1 Input and gates

`_on_distances` (L840–885) keeps the **valid** entries of the `MultiLinkDistance`, in their
`labelled_links` order. Each becomes `Obstacle(link, d, pr, ph, conf, v_vec, frames_seen, vel_cov,
track_id, cp_label, a_vec, pos_cov, pv_cov, range_m)`. **Verified facts that matter later:**

- `v_vec` and `vel_cov` are **always numpy arrays**, zero when no track is attached; never `None`.
  Every `ob.v_vec is not None` test in the builder therefore passes, and only the `frames_seen`
  gates stop an untracked entry.
- `ld.direction` (the perception normal) is **not read**: the filter rebuilds n̂ from `pr − ph`.
- `cp_label` = `'<end_link>#k'` (with `'.r'` for obstacle rank r > 0), positional in the message
  (05 §6.2). It keys every per-control-point state below.
- `t_cap` = the message header stamp (capture time), used to differentiate. The receipt time is
  used for staleness only.

Gates inside the loop (L2499–2538), in order:
1. `d_obs_min ← min(d)`, computed **before** any gate. It is published on `/cbf_status` for the
   commander's phase governor (06 §5) and used by the zone ladder's task weight (06 §6.1).
2. Skip if `d > cbf_obstacle_horizon` (1.2 m) or `conf < min_confidence` (0.2). The horizon is a
   computational cutoff. Perception publishes nothing valid beyond `max_thresh` = 0.7 m (05 §6.3),
   so it never binds in practice.
3. Skip if d_ctr < 1e-8 (degenerate normal).
4. Skip if the link has no frame (`_frame_id`; substring fallback, F-2-6).
5. Jacobian: `kin.control_point_jacobian(cp_label)` when a label exists. The rank suffix is
   stripped with `label.partition('.')[0]`, so the rank-r row of a control point uses that point's
   kinematics. Without a label: `point_jacobian(fid, pr)`.
6. **Leverage gate:** skip if ‖a‖ < `cbf_min_leverage` = 0.05 m/rad (it replaced an older
   cond(J) test).

### 3.2 Barrier value with asymmetric recovery

  h_raw = d − d_safe;  h = h_raw if h_raw ≤ h_prev (or first sight), else
  h = α_r h_prev + (1 − α_r) h_raw with α_r = `cbf_h_recovery_alpha` = 0.8 (L2598–2603).

A closer measurement is taken at once; a farther one is approached at 20 % per rebuild, i.e.
τ ≈ 90 ms at 50 Hz. The stored state is the **measured** h; every margin below is subtracted after
the store, so margins never compound. **Verified.** `minimal_l1` sets α_r = 0.

### 3.3 Identity guard (`_obstacle_identity_changed`, L4103–4211)

Every temporal filter assumes that a control point keeps seeing the same obstacle. The guard
declares a change when one of two conditions holds:
- both track ids are nonzero and they differ; or
- ‖p_h − p_h,prev‖ > max(`obstacle_velocity_identity_jump` = 0.10 m, v_max·min(Δt, 0.2 s)), with
  v_max = 2.0 m/s.

On a change it clears the residual anchor, frame counter, normal, rotation hold, median window and
uncertainty EMA. It does **not** clear `_h_smooth`, so the barrier keeps its conservative asymmetry
(§3.2). The YAML has the guard off (0.0); the default launch turns it on (0.10).
**Verified**, including the configuration values (`effective_params.py`).

### 3.4 Obstacle closing speed v_o (L2619–2753)

The hardware default uses `obstacle_velocity_source: tracker` with the residual floor on. The YAML
default is `residual`.

**(7.14) Tracker estimate** (`_obstacle_speed_tracked`). It is 0 unless the track is old enough,
`frames_seen ≥ _min_frames_eff`, or passes the fast-track test. Otherwise
v = clip(n̂ᵀv_vec, ±v_max), followed by a **deadband** of 0.15 m/s:
v ← sign(v)·max(|v| − 0.15, 0). Then v_o = max(v, 0).
- `_min_frames_eff` = `frames_for(obstacle_velocity_min_span_s, rate)` = round(0.265 × 90) =
  **24 measurements** (`_apply_input_rate`). It is re-derived when the measured stream rate moves
  (`_track_input_rate`).
- The fast-track test (`_fast_track`) needs frames_seen ≥ 3 and n̂ᵀv − 2σ_n ≥ 1.0 m/s, where
  σ_n² = n̂ᵀP_vv n̂. It lets a thrown object through early, up to a clip of 6 m/s.

**(7.13) Residual estimate** (`_obstacle_speed`). Using (7.4) backwards,

  v_raw = aᵀq̇ − ḋ_meas,  ḋ_meas = (d_k − d_anchor)/(t_cap,k − t_anchor),

clipped to ±2 m/s, then smoothed with an EMA of weight exp(−Δt/τ) on the history, τ = 0.075 s.
Details:
- The anchor is held until Δt > `obstacle_velocity_dt_min_s` = 0.033 s, so at 90 Hz the difference
  spans 3 frames (3/90 = 33.3 ms > 33 ms).
- A **rotation guard** discards the update and holds the previous v when n̂ turned by more than
  0.15 rad since the anchor. It holds for up to 0.10 s.
- It returns 0 on the first frame.

Note that this estimator makes the k₁ term see ḣ ≈ ḋ_meas whatever a is (§8.1 uses this).

**Combination.** In tracker mode, if `obstacle_velocity_residual_floor` is on and the smoothed h is
below `obstacle_velocity_residual_floor_gap` = 0.7 m, then v_o = max(v_track, v_res⁺). The comment
explains why: a person's cluster centroid barely moves while a limb closes. The frame count used by
the later gates is `n_seen = max(frames_seen, residual frames)`.

**Median.** When `obstacle_velocity_median` > 1 (5) with a window
`obstacle_velocity_median_s` = 0.3 s, v_o becomes the median of the last 0.3 s of per-frame values
(≈ 27 taps at 90 Hz, cap 64), keyed by capture time. A fast-trusted track bypasses the median on its
rising edge.

**What enters ḣ** (`rows_vobs`). By default this is v_o. With `enable_vobs_in_hdot` (ON in the
default launch) and a confirmed or fast track, it is instead the **signed** clipped n̂ᵀv_vec
(±2 m/s), so a receding obstacle relaxes the k₁ term. Only this one term uses the signed value; the
retreat cap and the evasion keep v_o ≥ 0. **Verified.**

### 3.5 Margins: from h to h̄ (L2794–2953)

  (7.12)  h̄ = h − h_brake − h_unc − h_lat − h_rng − h_std − h_ev.

| Term | Formula (helper) | Gate | Shipped (hw default) |
|---|---|---|---|
| h_brake (feedforward) | min(v_o²/(2·a_dec), 0.25); plus b_ff = −g·v_o on the RHS (`velocity_feedforward_terms`) | n_seen ≥ `_ff_min_frames_eff` (round(0.10 × 90) = 9) | **OFF** |
| h_unc (uncertainty) | min(k_σ·√(n̂ᵀP_vv n̂)·t_lat, 0.25), EMA α = 0.8, × w_gate (`uncertainty_margin`) | n_seen ≥ **8** | ON (k_σ = 2, t_lat = 0.2 s) |
| h_lat (latency) | min( max(n̂ᵀ(v t + ½a t²), 0) + k_σ√(n̂ᵀ(P_pp + t(P_pv + P_pvᵀ) + t²P_vv)n̂), 0.25 ) × w_gate (`latency_compensation_terms`) | n_seen ≥ **8** | ON (t = 0.085 s, k_σ = 2) |
| h_rng (sensor range) | min(k_σ·Z²σ_d/(f b), max) (`sensor_range_uncertainty`) | none (per pixel) | OFF |
| h_std (standoff) | min(0.25·v_o, 0.35); rises at once, decays by EMA α = 0.8 | n_seen ≥ **8** or fast | ON |
| h_ev (evidence margin) | per evidence class: fresh / young / coasting / untracked (`evidence_class`) | — | OFF |

The approach gate is w_gate = smoothstep(v_o/0.15) (`_approach_weight`), applied to h_unc and
h_lat when `margin_gate_on_approach` is on. It switches both off for a static obstacle, because the
covariance never reaches zero.

**Units check (Derived):** √(n̂ᵀP_vv n̂)·t_lat is (m/s)·s = m. √(n̂ᵀ(P_pp + tP_pv + t²P_vv)n̂) is
m (P_pv is m²/s). Z²σ_d/(f b) is m²·px/(px·m) = m. All the margins are metres, consistent with h.

**Note (F-3-6):** h_unc, h_lat, h_std and the evidence class compare n_seen with the **raw**
`obstacle_velocity_min_frames` = 8. The velocity itself (7.14), v_hdot and κ use the span-derived 24.
At 90 Hz the margins therefore engage after 89 ms of evidence, while the velocity they protect needs
267 ms. The YAML comment on the 8 frames dates from the 30 Hz era ("~265 ms at 30 Hz").

### 3.6 Drift and curvature terms, row append

- c = n̂ᵀ(J̇_p q̇) at the snapshot's q̇ (L2958). It is frozen for the 50 Hz period, while aᵀq̇ is
  refreshed at the QP rate.
- κ (OFF): computed with v_rel = J_p q̇ − v_vec, ρ_c = d_ctr + `curvature_obstacle_radius` (0.30 m).
  Optionally ρ_c = max(ρ_c, measured ‖v⊥‖/‖ṅ̂‖), where ‖ṅ̂‖ is fitted over 0.2 s of normals. It is
  clipped to [0, 2] m/s² and gated on a confirmed or fast track.
- **Finite guard:** a row is appended only if a, h̄ and c are finite. `rows_vobs`, `obs_bff`,
  `obs_kappa`, `obs_dz` (the measured d for the zone ladder) and `obs_w` are appended in lockstep,
  inside the guard. A comment (L2772–2779) records the earlier misalignment bug that this ordering
  fixed.
- **Retreat cap** (ON): one extra row per obstacle row, with `−a` so that it is an UPPER bound on
  the separation acceleration:

    (7.15)  v_cap = min(1.1, 0.05 + 1.0·v_o + 2.0·max(−h̄, 0)·min(v_o/0.3, 1) + 1.1·clip(h̄/0.15, 0, 1)),

  enforced as aᵀq̈ ≤ (v_cap − aᵀq̇)/0.15 s (`retreat_cap_speed`, `retreat_cap_rhs`). It prevents
  the barrier from flinging the arm away. It is a rate cap, chapter 08.
- Task-space speed points (`spd_pts`, ON) and the evasion inputs (`_escape_row`,
  `_outrun_bias`, livelock candidates) are collected here; chapter 06 §6.

### 3.7 After the loop: ladder, push floor, slack, scatter (L3556–3816)

- The obstacle rows come **first**. Memory rows (OFF) follow, then joint-limit, singularity,
  self-collision and SSB rows, and finally the rate caps as the trailing `n_cap` rows.
- **Zone ladder** (ON in the default launch, §5). It sets per-row `k0_row`, `k1_row`, a slack
  multiplier and a diagnostic rung index from each row's **measured** d. Non-obstacle rows keep the
  scalar gains.
- **Push floor** `h_pen_row` (7.11).
- **Slack column:** `G[i, NV + group] = −m_i`. Here m_i is 1, or w_max/w_i with weighted slack
  (OFF), times the zone multiplier. With `obstacle_slack_mode: row_l1` (only `minimal_l1`), each
  obstacle row gets its own column, up to 32.
- `b_ff` and κ are scattered onto the G_OBS rows **by family**, not by position. When their flags
  are off they are `None`, which keeps the QP expression bit-identical.

### 3.8 The right-hand side at the QP rate (`build_row_rhs`)

At every QP tick (`qp_rate_hz` = 300 Hz in the hardware default, 100 Hz in `minimal_l1`):

  (7.10)  h_qp = k₁(A q̇_cbf − v_obs) + k₀·max(h̄, h_pen) + c [+ b_ff] [+ κ]  (barrier rows),
          h_qp = (v_cap + aᵀq̇)/T  (rate-cap rows, raw q̇, T = 0.15 s or the speed horizon).

k₀ and k₁ are per row when the ladder is on. q̇_cbf is an EMA of q̇ (`_smooth_qdot`, weight
`cbf_hdot_filter_alpha` = 0.6 per 10 ms); it is used **only** in the k₁ term. A, h̄, c, v_obs and κ
are frozen at the last 50 Hz rebuild. **Verified;** reproduced on a synthetic snapshot to 1e-12
(`check_hocbf_gains.py` item 6).

**Effective q̈-row, summarised:**

  aᵢᵀq̈ + mᵢ s ≥ −k₁,ᵢ(aᵢᵀq̇_cbf − v_obs,ᵢ) − k₀,ᵢ max(h̄ᵢ, h_pen,ᵢ) − cᵢ.

---

## 4. Evidence conditioning: which estimate is trusted when

| Consumer | Evidence required | Count at 90 Hz |
|---|---|---|
| tracker v_o (7.14), v_hdot, κ | `frames_seen ≥ _min_frames_eff` or fast | 24 (span 0.265 s) |
| fast track | frames_seen ≥ 3 and n̂ᵀv − 2σ_n ≥ 1 m/s | 3 |
| residual v_o | none; 0 on first frame, then EMA | — |
| h_unc, h_lat, h_std, evidence class | n_seen ≥ `obstacle_velocity_min_frames` | **8** (F-3-6) |
| feedforward (OFF) | n_seen ≥ `_ff_min_frames_eff` | 9 |
| sensor range (OFF) | none | — |

n_seen is the larger of the tracker's `frames_seen` and the residual's per-label counter. The
residual counter restarts on an identity reset or a new label (§8). Conditions that set
`obstacle_velocity_min_frames: 8` through `cbf_params` (for example `minimal_l1`) do not change the
24-frame gate, because `obstacle_velocity_min_span_s` overrides the count. **Verified**
(`_apply_input_rate`; `effective_params.py`).

The evidence-margin classes (`evidence_class`, OFF) are: untracked (track_id 0), coasting (frozen
frames_seen for longer than `evidence_coast_grace_s` = 0.07 s), young (< 8), fresh.

---

## 5. Zone ladder (`utils/cbf_zones.py`)

### 5.1 Definition

Rung boundaries on the measured gap, as multiples of d_safe (`ladder_from_params`):
hold 0.25, priority 0.5, active 1.0, notice 1.5 → 0.025 / 0.05 / 0.10 / 0.15 m at d_safe = 0.10.
The blend width is `zone_blend_r`·d_safe = 0.02 m.

`weights(d)` builds three cumulative smoothsteps tⱼ across the boundaries **hold, priority,
active** and differences them:

  w_hold = 1 − t₀,  w_prio = t₀ − t₁,  w_active = t₁ − t₂,  w_notice = t₂.

The result is a partition of unity, so the blended (k₀, k₁, m) are convex combinations of the rung
values: k₀ = (50, 50, 25, 12), k₁ = (14, 14, 10.5, 7), m = (0.25, 0.5, 1, 1).

### 5.2 Findings

- **F-3-4 (Verified, reproduced):** `d_notice` is not among the boundaries `weights` uses, so the
  notice gains apply for every d > d_active + blend/2 = 0.11 m. They do not start at d_notice =
  0.15 m, as the module table and the class docstring imply; the YAML comment on `zone_r_active`
  does describe the code. Two ladders that differ only in d_notice give identical weights
  everywhere. `zone_r_notice` affects only `describe()` and the ordering check.
- **F-3-5 (Derived, numerically checked):** the priority and hold rungs have complex
  characteristic roots (§2.5, (7.8)). The effect is negligible, but "same damping as the shipped
  pair" in the YAML is ζ = 0.99 < 1, i.e. underdamped, and the HOCBF factorisation does not exist
  there.

### 5.3 What the rungs change

- Bandwidth and engagement distance: k₁/k₀ = 0.583 s on the notice rung, so the row starts to
  push earlier but more gently (7.9).
- The slack multiplier m < 1 in the inner rungs makes violation 4× (priority) or 16× (hold) more
  expensive under the ½ρs² cost (chapter 08).
- The **task weight** (`task_weight`, applied in `_qp_tick`, chapter 06 §6.1) fades the nominal
  toward −k_brake·q̇. It is computed from `d_obs_min`, not per row.

---

## 6. Multiple obstacles and multiple control points

- **One row per valid entry.** Perception publishes one entry per control point and per obstacle
  rank. With `multi_obstacle_k` = 2 (launch default) and a second cluster, each control point also
  gets a rank-1 entry (`#k.1`). Each becomes an independent row with its own normal, gap, estimator
  state and margins. Rows do not interact except through the QP.
- **No per-obstacle aggregation.** Two control points seeing the same obstacle produce two rows.
  `d_obs_min` is the minimum over all valid entries, including those that fail the horizon or
  confidence gates.
- **Duplicates.** Segment 5 has zero length (F-2-1), so `fr3_link6#0` and `#1` give identical rows
  (to 5.6e-17, `check_cp_label_rows.py`). The QP carries that constraint twice. In `family` slack
  mode this is harmless to feasibility. In `row_l1` mode the duplicate takes a second slack slot.
- **Memory rows** (OFF): a confirmed track that was close (d ≤ `memory_trigger_gap_m`) and then
  vanished keeps a row for up to `memory_hold_s` = 0.5 s. The row sits at the propagated point
  p + v·age, with the surface eroded by `memory_inflate_speed_mps`·age and v_obs = 0. An optional
  self-mask test can release it early.

---

## 7. Integration

| Item | Value | Label |
|---|---|---|
| Input | `/cbf/per_link_distances` (`franka_msgs/MultiLinkDistance`), ~90 Hz from `real_time_distance` (05) | Verified (topic), rate from the camera profile |
| Joint state | `JointSnap(q, q̇, stamp)` from the filter's joint-state callback | Verified |
| Rebuild | 50 Hz timer → `ConstraintSnap` (A, h̄, c, G, v_obs, gains, caps, biases, diagnostics) | Verified |
| Consumer | `_qp_tick` at 300 Hz: `build_row_rhs` → OSQP (chapter 08) | Verified |
| Published | `/NS_1/qddot_safe`; `/NS_1/cbf_status` carries `d_obs_min` (read by the commander's governor) | Verified |
| Staleness | distance > 0.5 s old → no rows; `_qp_tick` then brakes with −k_brake·q̇ (chapter 08) | Verified |
| Latency chain | capture → perception → message → 50 Hz rebuild (up to 20 ms of age) → 300 Hz QP. Only aᵀq̇ is fresh at the QP rate. | Derived |

---

## 8. The two upstream defects, traced through the barrier

### 8.1 F-2-11 → F-3-2, F-3-13: the held entry (answers Q-2-1)

**Upstream (05 §3.5).** When a control point loses its measurement, the distance engine holds the
last d and direction for up to `iso_distance_hold_max_s` = 0.10 s. The entry is published
`valid = True` with `closest_point_human = (0, 0, 0)` and no track fields.

**Nothing downstream rejects it.** Verified in code, and reproduced through the real builder
(`check_held_row.py`):
- it passes `valid`, the confidence gate (conf 0.80 ≥ 0.2) and the horizon;
- n̂ = p_r/‖p_r‖ (base-radial). For the home fingertip with an obstacle in front (+x), that is
  **122°** from the true normal;
- a = n̂ᵀJ_p uses that wrong normal, while h = held d − d_safe is the right number;
- the identity guard fires on entry (p_h jumps to 0) and again on exit, resetting the velocity
  state both times.

**Scenario A**, untracked static obstacle with the arm approaching at 0.10 m/s:

| Frames | true gap (m) | h (m) | v_obs (m/s) | ḣ in the k₁ term, aᵀq̇ − v_obs (m/s) |
|---|---|---|---|---|
| 44 (last normal) | 0.111 | 0.0111 | 0 | −0.100 |
| 45–49 (hold) | 0.110 → 0.106 | 0.0111 | 0 | **+0.053** |
| 50–53 (hold) | 0.104 → 0.101 | 0.0111 → 0.0064 | 0.010 → 0.019 | +0.044 → +0.034 |
| 54–55 (invalid) | 0.100 → 0.099 | no row | — | — |

The true approach −0.10 m/s reads as +0.053 m/s along the held normal, i.e. as separation. The
residual estimator then manufactures a small closing speed from the wrong a (7.13), which adds a
small standoff. Net effect: for ≤ 0.1 s the row does not resist the approach it was built for.

**Scenario B**, confirmed track closing at 0.5 m/s with the arm still (**F-3-13**):

| | last normal frame | held frames 45 → 53 |
|---|---|---|
| v_obs | 0.500 | **0** on every frame (frames_seen 0 → tracker 0; the residual sees a frozen d) |
| h̄ | −0.116 | −0.043, −0.024, … +0.039 |
| true gap | 0.156 | 0.150 → 0.106 |

On entering the hold, the latency margin vanishes at once (its n_seen gate), the uncertainty margin
too, and the standoff decays by its EMA. **From the last normal frame (44) to the last held frame
(53), the row's h̄ relaxes by 15.5 cm while the true gap shrinks by 5.0 cm**, and the k₁·v_obs term
(10.5 × 0.5 = 5.25 m/s² of demanded separation along n̂) disappears.

> *Audit (C-018):* this sentence used to read "relaxes by 15.5 cm while the obstacle closes 4.4 cm".
> 4.4 cm is the gap change between the first and last *held* frames (45 → 53), a different
> interval from the 15.5 cm (44 → 53). The script prints both: `check_held_row.py` item 6.
This is the opposite of the code's own "only tighten without evidence" convention. The cause is
upstream: the held entry is valid but carries no evidence. **Numerically reproduced with the real
builder; the closed-loop effect needs runtime (Unverified).**

**Mitigations (for the refactor; not applied):** publish held entries with the last p_h and the
track fields, or as invalid; or have the builder treat `ph == 0` with `track_id == 0` as
"hold the previous row".

### 8.2 F-2-10 → F-3-12: control-point labels

**Upstream (05 §6.2).** With a second cluster, the duplicate wrist point `fr3_link6#1` is labelled
`fr3_link6#0.2` (and its extra `#0.3`); with one cluster it is `#1` again.

**Downstream** (`check_cp_label_rows.py`, receding obstacle):
- The geometry is right: `#0.2` → `partition('.')` → `#0`, whose kinematics equal `#1`'s.
- At the flip to the new key `#0.2`, `_h_smooth` has no entry, so h_raw is accepted at once: an
  **11.9 mm loosening in one frame** against the stable twin `#0`. The residual anchor, median and
  margins also start empty.
- At the flip back, the STALE `#1` state from before the flip is resumed. The EMA restarts from
  0.068, the value left 11 frames earlier, and here gives h 25 mm *below* the twin. The direction
  depends on the scene.
- The self-detection keys (05 §6.2) use `#1` and never match the published `#0.2`.

---

## 9. Practical example: the home-pose fingertip

Configuration q_home. Control point `fr3_link8#2` (the fingertip, p_r = (0.307, 0, 0.490) m).
Obstacle 0.21 m in front along +x, so d = 0.16 m, n̂ = (−1, 0, 0) and h = 0.06 m. Measured in the
builder (`check_held_row.py`) and derived:

1. a = n̂ᵀJ_p ≈ (0, ·, 0, ·, 0, ·, 0) (only joints 2, 4 and 6 move the tip in x at this pose).
2. Arm approaching at 0.10 m/s (static obstacle, residual path): v_obs = 0, ḣ = −0.10 m/s.
   Barrier bound for scalar gains: −k₁ḣ − k₀h̄ − c = 1.05 − 25·h̄ − c. It demands deceleration once
   h̄ < 0.042 m, i.e. d < 0.142 m plus margins (7.9). With the ladder on at d > 0.11 m, the notice
   gains apply: 7·0.10 − 12·h̄ → d < 0.158 m.
3. At d = 0.111 (frame 44 in §8.1), h̄ = 0.011 m. Scalar bound: 1.05 − 0.28 = **0.77 m/s²** of
   required separation acceleration along n̂. Notice rung: 0.70 − 0.13 = 0.57 m/s². The QP must
   produce aᵀq̈ ≥ that bound minus c.

---

## 10. Implemented but not shipped

All OFF in the YAML, the default launch and the simulation conditions (`effective_params.py`):
velocity feedforward (h_brake, b_ff), curvature κ (with measured ρ_c), evidence margin, obstacle
memory, sensor-range margin, weighted slack, SSB rows and the ISO speed rows. Their formulas are
in §3.5–3.6 and §6. Every one is wired so that "off" means `None` in the snapshot, and
`test_cbf_*` assert bit-identity with the flag off.

## 11. `utils/cbf_constraints.py` is unused (F-3-1)

`build_simple_accel_cbf_constraints`, `build_hocbf_constraints`, `predict_state` and `CBFParams`
are imported by no node, script or test. Their HOCBF differs from the live rows: cᵢ is clipped to
±5, ṅ̂ is divided by max(d, 0.05), rows are skipped at cond(J) > 1e5, and there are no margins.
Do not read it as documentation of the live barrier.

---

## 12. Consequences of modification

| Change | What it affects |
|---|---|
| `d_safe` | h; the zone boundaries (ratios); the commander's governor radii, which are read from the **installed** YAML (F-1-10). |
| k₀, k₁, zone gains | engagement distance (7.9), rest speed (7.11), the real-root condition (F-3-5), and the noise gain of v_obs through k₁. Keep k₁² ≥ 4k₀. |
| `obstacle_velocity_*` (source, floor, median, deadband, spans) | v_obs, and therefore the k₁ term, retreat cap, standoff, evasion urgency and outrun ratio. The 8-vs-24 gate split (F-3-6). |
| A margin flag | h̄ only; the `_h_smooth` store must stay before the margins, or they compound. |
| The order of row builders | protected by the by-family scatter (b_ff, κ, zones, weights). The trailing rate caps are addressed by slice, so they must stay last. |
| `cp_label` construction (05 §6.2) | every per-control-point state key (§8.2). Fixing F-2-10 changes which history each row inherits. |
| `_lpf_pass` hold behaviour (05 §3.5) | F-3-2 and F-3-13 close if held entries carry p_h and track fields, or are invalid. |
| `cbf_update_rate_hz` | the age of A, c and v_obs at the QP. The commander, ladder and governor read `d_obs_min` at that rate. |
| Tests pinning this | `test_cbf_state_rows`, `test_cbf_zone_ladder`, `test_cbf_tracker_source`, `test_cbf_vobs_rate_independent`, `test_cbf_normal_rotation_guard`, `test_cbf_uncertainty_margin`, `test_cbf_latency_compensation`, `test_cbf_velocity_standoff`, `test_vobs_in_hdot`, `test_cbf_curvature_term`, `test_cbf_evidence_trust`, `test_cbf_retreat_cap`, `test_cbf_fast_track` (names only; the tests themselves were read at interface level). |

## 13. Findings from this chapter

| ID | Short title | Section | Reproduced by ([tools/stage3_verification/](tools/stage3_verification/README.md)) | Still needs |
|---|---|---|---|---|
| F-3-1 | `cbf_constraints.py` unused, and different from the live HOCBF | §11 | grep | — |
| F-3-2 | held entry (ph = 0) builds a base-radial row; Q-2-1 answered NO | §8.1 | `check_held_row.py` | runtime |
| F-3-4 | `d_notice` inert in the zone ladder | §5.2 | `check_hocbf_gains.py` | — |
| F-3-5 | priority/hold rung gains have complex roots | §2.5, §5.2 | `check_hocbf_gains.py` | — |
| F-3-6 | margins gated on 8 frames, velocity on 24 | §3.5, §4 | `effective_params.py` (values) | author's intent |
| F-3-12 | label flips: duplicate rows; new key loosens, old key resumes stale state | §8.2 | `check_cp_label_rows.py` | — |
| F-3-13 | held row loosens by 15.5 cm while a tracked obstacle closes 5.0 cm (same 9 frames) | §8.1 | `check_held_row.py` | runtime |
| (assumption) | m(o) treated as constant in ḣ | §2.3 | — | measurement |

## 14. Audit record (Stage 3 audit, 2026-11-11, C-017 to C-019)

A targeted consistency review of the core equations and of F-3-2, F-3-3, F-3-12 and F-3-13.

**Re-verified in the source:**
- (7.3) h = d − d_safe, `build` L2586.
- (7.4) a = n̂ᵀJ_p, L2526; the n̂ sign is obstacle → robot, L2509–2513; ḣ = aᵀq̇_cbf − v_obs,
  `build_row_rhs` L221.
- (7.7) and (7.10): h_qp = k₁(Aq̇_cbf − v_obs) + k₀ max(h̄, h_pen) + c (+ b_ff + κ), with
  G = [−A | −m] (L3600–3726) and −∞ ≤ Gx ≤ h_qp (`build_osqp_bounds` L146–147). Together these
  are aᵀq̈ + m s ≥ −h_qp, the sign of (7.7).
- The QP objective ½‖q̈ − q̈_nom‖² + ½ρs² (`build_qp_objective`; `qvec[:NV] = −q̈_nom`).

**Reproduced again at HEAD `61260f6`:** `check_held_row.py` 21/21 (F-3-2, F-3-13),
`check_cp_label_rows.py` 8/8 (F-3-12), `check_timing_governor.py` 16/16 (F-3-3, ch. 06 §5).

**What is reproducible vs what still needs closed-loop validation:**

| Finding | Reproducible numerically (open loop, real code) | Needs closed-loop / runtime validation |
|---|---|---|
| F-3-2 | the held entry passes every gate; the row normal is base-radial, 122° from the true one; the approach reads as +0.053 m/s along it (scenario A) | how often holds happen on hardware; whether the arm actually closes the gap during a 0.1 s hold |
| F-3-13 | v_obs 0.5 → 0, h̄ −0.116 → +0.039 m while the imposed gap shrinks 5.0 cm (scenario B) | the resulting minimum distance with the QP, the dynamics and the 300 Hz loop in the loop |
| F-3-12 | duplicate rows (5.6e-17); 11.9 mm one-frame loosening on the new key; stale-state resume on the old key | how often a second cluster appears near the wrist; any effect on clearance |
| F-3-3 | v_d ×1/σ and centripetal ×1/σ² (exact); a lead of 0.059 m at σ = 0.5 on an *idealised* double integrator | the lead on the real arm, where the filter and the dynamics intervene |

**Corrections:** §2.3 (the unsupported sign of the m(o) term) and §8.1 / F-3-13 (mismatched
intervals). Both are recorded in place with the old wording. **No evidence label was raised or
lowered** except where the audit re-read the source, as listed above.
