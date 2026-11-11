# Appendix C — Glossary and notation

> Stage 7, 2026-11-11. Baseline `ba68077`. One entry per symbol or term, with the chapter that
> defines it. The chapter is authoritative; this appendix only gathers the definitions and flags
> the symbols that mean different things in different chapters (§3). Equation numbers point to
> [B_equation_index.md](B_equation_index.md).

## 1. Symbols

Units: SI throughout (m, s, rad, N·m). Vectors are columns. "Row" means one inequality of the QP.
Topic names: the full name carries the robot namespace (`/NS_1/qddot_safe`, from `franka_bringup`,
F-1-1); a short form such as `/qddot_safe`, `/cbf_status[5]` or `/q_des_state` is shorthand for the
same `/NS_1/…` topic (index in brackets = array element). `/cbf/per_link_distances` and
`/camera/...` are not namespaced.

| Symbol | Meaning | Defined in |
|---|---|---|
| q, q̇, q̈ ∈ ℝ⁷ | arm joint position, velocity, acceleration (fingers excluded) | [04](04_math_foundations.md) §1 |
| q̇_cbf | lightly smoothed q̇, used only in the k₁ term of barrier rows | [07](07_cbf_obstacle_barrier.md) (7.10), [08](08_constraints_and_qp.md) |
| q̈_nom | nominal acceleration from the commander (`/NS_1/qddot_nom`) | [06](06_nominal_motion.md) |
| q̈_safe, u | the QP's acceleration solution (`/NS_1/qddot_safe`) | [08](08_constraints_and_qp.md) §2 |
| x | QP decision vector [q̈ ; 7 family slacks ; K per-row slots] ∈ ℝ¹⁴ (`family`) or ℝ⁴⁶ (`row_l1`) | [08](08_constraints_and_qp.md) (8.2) |
| J_p, J̇_p | 3×7 Jacobian of a robot point (control point) and its time derivative | [04](04_math_foundations.md) (4.4)–(4.5) |
| J, J⁺ | 6×7 task Jacobian of the commander; damped pseudo-inverse | [06](06_nominal_motion.md) (6.6)–(6.8) |
| d | perceived surface gap from a control point to the obstacle (the wire distance) | [05](05_perception_and_distance.md), [07](07_cbf_obstacle_barrier.md) (7.1) |
| d_safe (d_s) | safety distance subtracted from d; **0.10 m** at the baseline (`fr3_control.yaml params.d_safe`) | [03](03_configuration.md), [07](07_cbf_obstacle_barrier.md) (7.3) |
| h | barrier value h = d − d_safe | [07](07_cbf_obstacle_barrier.md) (7.3) |
| h̄ | conditioned (tightened) barrier: h minus the margins h_brake, h_unc, h_lat, h_rng, h_std, h_ev | [07](07_cbf_obstacle_barrier.md) (7.12) |
| h_pen | penetration floor: the k₀ term uses max(h̄, h_pen) | [07](07_cbf_obstacle_barrier.md) (7.10)–(7.11) |
| h_qp | right-hand side of a row, computed at the QP rate by `build_row_rhs` | [07](07_cbf_obstacle_barrier.md) (7.10), [08](08_constraints_and_qp.md) |
| n̂ | unit normal, obstacle → robot | [07](07_cbf_obstacle_barrier.md) (7.4) |
| a | row coefficient a = J_pᵀn̂ (a row of A) | [07](07_cbf_obstacle_barrier.md) (7.4) |
| c | drift n̂ᵀJ̇_p q̇ (carried from the 50 Hz rebuild) | [07](07_cbf_obstacle_barrier.md) (7.5) |
| κ | curvature term ṅ̂ᵀv_rel of ḧ; optional, `enable_curvature_term` (off) | [07](07_cbf_obstacle_barrier.md) (7.5) |
| b_ff | Phase-1 feed-forward term of the row RHS (flag-gated) | [07](07_cbf_obstacle_barrier.md) (7.10) |
| v_o, v_n, v_obs | obstacle velocity; its normal component; the conditioned scalar the row uses | [07](07_cbf_obstacle_barrier.md) §4, (7.13)–(7.14) |
| k₀, k₁ | HOCBF gains (s⁻², s⁻¹); per-row under the zone ladder | [07](07_cbf_obstacle_barrier.md) (7.6), §5 |
| α₁, α₂ | roots of s² + k₁s + k₀ (real iff k₁² ≥ 4k₀, F-3-5) | [07](07_cbf_obstacle_barrier.md) (7.6), (7.8) |
| s_obs, s_sc, s_qlim, s_sing, s_cap, s_spd, s_ssb | the seven family slacks | [08](08_constraints_and_qp.md) (8.2) |
| σ_k | per-row obstacle slack slot (`row_l1` mode only) | [08](08_constraints_and_qp.md) (8.2), (8.14) |
| ρ_g, ρ_l1 | family slack penalties; the ℓ1 price of the per-row slots | [08](08_constraints_and_qp.md) (8.3) |
| m_i | slack coefficient of row i (zone and risk scaling) | [07](07_cbf_obstacle_barrier.md), [08](08_constraints_and_qp.md) |
| k_brake | braking gain of the fallback, 3 s⁻¹ | [08](08_constraints_and_qp.md) (8.16) |
| σ_min | smallest singular value of the (scaled) Jacobian; singularity rows and recovery | [08](08_constraints_and_qp.md) (8.11), [06](06_nominal_motion.md) §4.4 |
| σ (governor) | phase-governor time scale ∈ [0, 1] | [06](06_nominal_motion.md) §5 |
| β | state-governor scaling of the nominal | [06](06_nominal_motion.md) §6.2 |
| s, ṡ, s̈ | path parameter of the nominal trajectory and its derivatives | [06](06_nominal_motion.md) (6.1) |
| λ | damping of J⁺ (commander) | [06](06_nominal_motion.md) (6.7) |
| τ_ff, τ_cmd | model torque from `qddot_to_torque` (RNEA − g) | [09](09_torque_realization.md) (9.3) |
| K_p, K_d | joint PD gains of `RtTorqueController` | [09](09_torque_realization.md) §3 |
| s_j (ffScale) | per-joint feed-forward fade near the velocity envelope | [09](09_torque_realization.md) §3 |
| E⁻(q), E⁺(q) | position-dependent FR3 velocity envelope | [08](08_constraints_and_qp.md) §2.4, [09](09_torque_realization.md) (9.5) |
| γ | class-K gain of the velocity CBF (`select_gamma`) | [10](10_alternative_controllers.md) (10.1), F-7-3 |
| S_p, S_h, S_r, S_s | ISO protective separation and its human, reaction and stopping terms | [11](11_iso_safety_layer.md) (11.1)–(11.2) |
| C, Z_d, Z_r | intrusion distance and the two uncertainty allowances; C + Z_d + Z_r = 0.92 m shipped | [11](11_iso_safety_layer.md) (11.5) |
| T_r, a_s, v_h, v_r | reaction time, stopping deceleration, human and robot speeds | [11](11_iso_safety_layer.md) |
| v_PFL | power-and-force-limiting speed ceiling, 0.68 m/s shipped (hands) | [11](11_iso_safety_layer.md), F-5-11 |
| w_gate | approach weight smoothstep(v_o/0.15) gating h_unc and h_lat | [07](07_cbf_obstacle_barrier.md) §3.6 |
| P, F, Q | Kalman covariance, transition and process noise of a track | [05](05_perception_and_distance.md) (5.7) |
| μ_j | IMM mode probability | [05](05_perception_and_distance.md) (5.9)–(5.10) |

## 2. Terms

| Term | Meaning | Defined in |
|---|---|---|
| **Baseline** | the commit whose source the chapters describe: `ba68077`. Not HEAD. | [PROGRESS](PROGRESS.md) §8 |
| **Pipeline A** | accel HOCBF: commander → `cbf_safety_filter` (QP) → `qddot_to_torque` + `RtTorqueController`. The shipped pipeline. | [01](01_architecture.md) |
| **Pipeline B** | velocity CBF: `ee_pentagon_velocity_commander` → `cbf_velocity_filter` → `RtVelocityExecutorController` | [01](01_architecture.md), [10](10_alternative_controllers.md) |
| **Pipeline C** | OSCBF torque-level filter (`cbf_OSCBF_filter`); no production launch | [01](01_architecture.md), [10](10_alternative_controllers.md) |
| **Pipeline D** | no filter (`minimal`, `thales` launches) | [01](01_architecture.md) |
| **External plant** | the Gazebo harness reusing `torque_control_stack` with `external_plant:=true` | [12](12_simulation.md) |
| **Control point (cp)** | a sampled robot point with a label (`fr3_linkN#k`) at which the distance and the row are computed | [04](04_math_foundations.md), [05](05_perception_and_distance.md) |
| **Rebuild** | the 50 Hz constraint-geometry update (`_update_constraints` → `ConstraintBuilder.build`) | [08](08_constraints_and_qp.md) §1 |
| **QP tick** | the `qp_rate_hz` (300 Hz shipped) solve (`_qp_tick`) | [08](08_constraints_and_qp.md) §1 |
| **Row family** | the group of rows that shares one slack: obstacle, self-collision, joint limit, singularity, retreat cap, link speed, SSB | [08](08_constraints_and_qp.md) (8.2) |
| **`family` / `row_l1` slack mode** | one penalised slack per family (shipped) / one ℓ1-priced slot per obstacle row | [08](08_constraints_and_qp.md) (8.14), F-4-1, F-4-2 |
| **State box, slew window** | hard per-joint acceleration bounds from position/velocity limits; the jerk bound ±`max_qddot_delta` per tick. The state box wins when they are disjoint (`slew_yields_to_state_box`). | [08](08_constraints_and_qp.md) §2.4 |
| **Brake (fallback)** | q̈ = clip(−k_brake q̇, box ∩ slew), on QP failure (`qp_failure_policy: brake`) or stale input | [08](08_constraints_and_qp.md) §5 |
| **Zone ladder** | distance-scheduled gains and slack coefficients (notice / priority / hold) | [07](07_cbf_obstacle_barrier.md) §5 |
| **Margins** | the tightenings that turn h into h̄ (brake, uncertainty, latency, range, standoff, evidence) | [07](07_cbf_obstacle_barrier.md) §3 |
| **Evidence class** | fresh / young / coasting / untracked, used by the evidence margin (off) | [07](07_cbf_obstacle_barrier.md) §3.6 |
| **Track, coast** | a Kalman/IMM obstacle hypothesis; coasting = predicted without a measurement (≤ `max_coast_s` = 0.8 s) | [05](05_perception_and_distance.md) §7 |
| **Hold (held row)** | the bounded LPF hold of a vanished distance (≤ 0.10 s) that emits `ph = (0,0,0)` (F-2-11, F-3-2) | [05](05_perception_and_distance.md), [07](07_cbf_obstacle_barrier.md) |
| **SSB** | self-shadow barrier: rows for space hidden behind the robot itself; off | [05](05_perception_and_distance.md), [08](08_constraints_and_qp.md) |
| **Go-around, lateral evasion, outrun, livelock escape** | objective biases on q̈_target (never rows) | [06](06_nominal_motion.md), [07](07_cbf_obstacle_barrier.md) |
| **Singularity recovery** | the commander's return-to-home state machine on a stall (default on; F-7-1) | [06](06_nominal_motion.md) §4.4 |
| **ISO layer** | the SSM/PFL computations, the monitor and the evidence logger; non-safety-rated; off by default | [11](11_iso_safety_layer.md) |
| **Install tree** | `/ros2_ws/install` in the container; what a launch actually runs (F-7-4) | [14](14_build_launch_test.md) |
| **Condition, scenario, cell** | simulation axes: a parameter set, an obstacle/motion script, one (scenario, condition, seed) run | [12](12_simulation.md) |
| **CURRENT / ALTERNATIVE / EXPERIMENTAL / LEGACY / UNUSED CANDIDATE / UNKNOWN** | code classes | [15](15_code_status.md) §2 |
| **S1–S4, P0–P3** | severity and priority of a finding | [16](16_issues_and_discrepancies.md) Part A |
| **Verified / Derived / Inferred / Unverified / Runtime (recorded)** | evidence labels | [00_INDEX](00_INDEX.md) §2 |
| **NUM / REPRO / FACT** | kinds of check in the verification scripts | [PROGRESS](PROGRESS.md) §5 decisions |

## 3. Symbols with more than one meaning (read by chapter)

These are consistent within each chapter; they collide across chapters. A refactor that renames
code should not import the collision into identifiers.

| Symbol | Meanings | Where |
|---|---|---|
| σ | governor time scale (06); singular value σ_min (06, 08); per-row slack slot σ_k (08); a standard deviation, e.g. depth noise or the baseline spread of 2.27 cm (05, 07, 12) | named in each row above |
| s | slack variable (08); path parameter (06); Laplace variable in s² + k₁s + k₀ (07); ffScale s_j (09) | |
| α | HOCBF roots α₁, α₂ (07, 10); the EMA weight α = exp(−Δt/τ_f) (05) | |
| λ | J⁺ damping (06, 10) | |
| γ | velocity-CBF gain (10); the OSCBF and governor use other letters | |
| C | ISO intrusion distance (11); Coriolis matrix C(q, q̇) (04, 09) | |
