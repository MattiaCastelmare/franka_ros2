# 16 — Issues, bugs and discrepancies (consolidated)

> Stage 7, 2026-11-11. Baseline `ba68077`. This chapter consolidates every finding of Stages 0–7
> (F-0-1 … F-7-11, 125 findings) without renumbering. The **full original text** of each finding
> stays, unchanged, in [PROGRESS.md](PROGRESS.md) §4, which remains the authoritative log. This
> chapter adds the fields needed for a refactor: category, severity, priority, confidence,
> reproduction status, links and a corrective direction. It also holds the comparison with the old
> technical guide (Part E). **No fix is implemented in this stage.**

---

## Part A — How to read the tables

| Field | Values |
|---|---|
| **Sev** (severity, impact if the issue manifests) | **S1** can produce unsafe motion or hide a collision on hardware · **S2** a wrong result: control, estimate or evidence · **S3** functional or architectural defect with a workaround · **S4** cosmetic: comment, name, documentation |
| **Pri** (provisional priority) | **P0** investigate before hardware use · **P1** important correctness or reliability · **P2** functional and architectural improvement · **P3** cleanup, documentation, maintainability |
| **Conf** | confidence that the issue exists as stated: high / medium / low (the evidence label is in PROGRESS §4) |
| **Num** | numerically reproduced: the script name, or `static` (read in code only) or `—` |
| **CL** | closed-loop consequences tested: `no`, or `recorded` (seen in an older run, F-6-1), or `sim` |
| **Links** | other findings with the same root cause, or that refine this one |

Two rules from the stage prompt:
- a high severity was **not** given just because a numerical test shows unusual behaviour;
- a safety issue was **not** dismissed because Gazebo has not reproduced it.

Every P0 is "investigate", not "proven dangerous": most P0s are Verified in code and numerically
reproduced, but **none** has a closed-loop test at the baseline (the install problem, F-7-4, makes
every recorded run evidence about another revision).

---

## Part B — Findings by category

### B.1 Safety-critical control behaviour

| ID | Issue (files::functions) | Evidence / reproduction | Sev | Pri | Conf | Num | CL | Corrective direction | Links |
|---|---|---|---|---|---|---|---|---|---|
| F-7-1 | Singularity recovery: the quintic ends at q̇ = v₀; with \|v₀\| > 0.5 rad/s, `plan_duration` never converges (T ≈ 457 s) and the reference leaves the joint range by tens of rad. **Default ON** (`singularity_recovery.py::quintic_coeffs`, `plan_duration`; `pentagon_qddot_commander._tick_recovery`) | `check_unread_functions.py` 1–4; test covers v₀ = 0 only | S1 | **P0** | high | yes | no | true rest-to-rest quintic (c₂ = 0, solve with q̇(T) = 0); exclude τ = 0 from the bound or start from a braking phase; a test with v₀ ≠ 0 | F-4-5 (the box is the last defence) |
| F-4-12 | Before the first distance message, the QP has **no** rows (no joint-limit, self-collision or singularity rows) and no fault (`cbf_safety_filter._update_constraints`) | static (L958–961) | S1 | **P0** | high | static | no | build the joint-state-only families without `obs`; raise a fault until perception is alive | F-4-3 |
| F-4-5 | Stale-joint-state fallback is open-loop: −k·q̇_stale every tick, without rows, box or slew; the commanded velocity reverses and accelerates (`_qp_tick` L1099–1104) | `check_box_and_brake.py` 5 | S1 | **P0** | high | yes | no | brake against a time-integrated q̇ estimate, keep the box and slew, or hold zero acceleration and raise a fault | F-7-1 |
| F-4-8 | `CBFTorqueController` uses the commander's **nominal** q̈ while `/q_des_state` is fresh, bypassing `/qddot_safe` (opt-in, `minimal use_torque_controller:=cbf`) | static (`cbf_torque_controller.cpp`) | S1 | P1 (P0 if that option is used on hardware) | high | static | no | consume only `/qddot_safe`; or remove the option | F-1-9 |
| F-5-2 | The velocity filter fails open (stale or absent distances → pass-through; QP failure → 0.5·nominal); no joint-state age check (Pipeline B) | `check_velocity_cbf.py` | S1 | P1 (P0 if Pipeline B runs on hardware, Q-1) | high | yes | no | brake on stale distances; age-check the joint state | F-5-3, F-5-7 |
| F-5-3 | Pipeline B's post-QP rate clamp can undo the barrier (`cbf_velocity_filter`) | `check_velocity_cbf.py` | S1 | P1 (P0 if B on hardware) | high | yes | no | put the rate limit inside the QP as a box | F-5-2 |
| F-5-7 | `RtVelocityExecutorController` has no command watchdog; the last q̇ is held indefinitely | static (C++ header L88–89) | S1 | P1 (P0 if B on hardware) | high | static | no | implement the declared `timeout_threshold_s`/`timeout_ramp_s` | F-5-16 |
| F-5-6 | `pentagon_torque_commander` orientation-loop sign error → unstable (Pipeline C, test launch only) | `check_torque_commander.py` | S2 | P2 | high | yes | no | flip the sign, as `pentagon_qddot_commander` does | F-0-2 |
| F-3-16 | Lateral-evasion and outrun biases drop at once on disengagement (≤ 3 rad/s² step) | static | S2 | P1 | high | static | no | fade the filtered bias after disengagement | F-3-17 |
| F-3-15 | `StateGovernor.velocity_margin` scores a stopped joint against its upper envelope only | `check_nominal_shaping.py` | S2 | P2 | high | yes | no | use the binding side by position, not by sign(q̇) | — |
| F-5-17 | ISO stop-latch deadlock (dormant; needs `iso_enabled` + `iso_monitor_enabled`) | static | S2 | P2 | high | static | no | break the cycle between `fault_braking` and the monitor's STALE trip | F-5-19 |
| F-5-19 | ISO monitor liveness is one-sided (trips 30 ms after start; ignores a dead filter) | static | S2 | P2 | high | static | no | timestamped status; bidirectional heartbeat | F-5-17 |

### B.2 CBF mathematical formulation

| ID | Issue | Evidence | Sev | Pri | Conf | Num | CL | Direction | Links |
|---|---|---|---|---|---|---|---|---|---|
| F-3-5 | Priority/hold rungs (k0, k1) = (50, 14): k1² − 4k0 < 0, so no two-stage class-K decomposition exists for those rows | `check_hocbf_gains.py` | S2 | P1 | high | yes | no | choose gains with real roots (k1² ≥ 4k0), or justify complex roots by another proof | F-4-1 |
| F-3-4 | Zone ladder: `d_notice` has no effect on the gains; docstrings disagree (YAML comment agrees: C-014 correction) | static | S4 | P3 | high | static | no | align the docstrings | — |
| F-3-6 | Two evidence gates on one track (raw 8 frames vs span-derived 24) | static | S2 | P2 | high | static | no | one rate-scaled gate | F-3-13, `set_input_rate` |
| F-3-14 | Damped-pseudoinverse null space leaks into the task space (≤ λ/2) | Derived | S3 | P2 | high | yes | no | use the undamped projector or a weighted one | F-3-3 |
| F-3-17 | Lateral evasion maps with J_pᵀ, while outrun/go-around use J⁺ | static + Derived | S3 | P2 | high | static | no | a common mapping | F-3-16 |
| F-3-20 | Evasion trigger is gated, its direction is raw (young-track prior can set it) | static | S2 | P2 | high | static | no | gate the direction on `frames_seen` | F-3-6 |
| F-5-4 | OSCBF obstacle row omits α₁ḣ | `check_oscbf.py` | S2 | P2 | high | yes | no | the HOCBF chain form | F-5-5 |
| F-5-5 | OSCBF obstacle path inert or optimistic (centre distance, hard-coded d_safe, QoS mismatch) | `check_oscbf.py` | S2 | P2 | high | yes | no | use the surface gap; fix the QoS | F-1-6, F-0-2 |
| F-5-8 | ISO S_h uses the measured approach speed, not the normative human speed | static | S2 | P2 (P1 if ISO is enabled) | high | static | no | the normative v_h | F-5-14 |
| F-5-14 | ISO standoff puts the human speed in the robot's stopping-time slot | `check_iso.py` | S3 | P2 | high | yes | no | T_s = v_r/a_s | F-5-8 |
| F-5-11 | `iso_v_pfl` = 0.68 m/s is the hands value; other body regions bind lower | `iso_pfl_speed.py` | S2 | P2 | high | yes | no | the minimum over reachable regions | F-5-9 |
| F-5-9 | The ISO layer cannot be enabled in this cell (floor 0.92 m > reach) | test `test_iso_dsafe_floor` | S3 | P2 | high | yes | no | a research decision (detection capability) | F-5-13 |
| F-3-1 | `cbf_constraints.py`: an unused HOCBF variant that differs from the live one | static | S4 | P3 | high | static | no | delete after approval (ch. 15 §4.4) | F-0-4 |

### B.3 QP and slack handling

| ID | Issue | Evidence | Sev | Pri | Conf | Num | CL | Direction | Links |
|---|---|---|---|---|---|---|---|---|---|
| F-4-1 | **The shipped obstacle barrier is a penalty, not a constraint**: with family slack ½ρs², every binding row is violated by (b − aᵀq̈ₙ)/(1 + ρ‖a‖²) | `check_slack_and_priority.py`; S6B closed loop: obstacle slack on 46 % of ticks, true gap still ≥ 12.27 cm | S1 | **P0** | high | yes | no | quantify the violation in closed loop (first cell: S6B); consider `row_l1` (exact penalty) as the default | F-4-2, F-4-3, F-3-5 |
| F-4-2 | One slack per family couples the rows: an infeasible row relaxes all obstacle rows | `check_slack_and_priority.py` | S1 | **P0** | high | yes | no | per-row slack (`row_l1`) | F-4-1 |
| F-4-3 | The hard box (incl. the slew window 0.5 rad/s² per tick) has priority over every CBF row | `check_slack_and_priority.py`, `check_qp_assembly.py` | S2 | P1 | high | yes | no | document as a design priority, or widen the slew when a row binds | F-4-1 |
| F-4-4 | `accept_iterate` raises TypeError on OSQP's None array; the fallback is reached via the generic except | `check_slack_and_priority.py` | S3 | P2 | high | yes | no | check `res.info.status` first | F-4-10 |
| F-4-10 | OSQP tolerance 1e-3, polish off; residuals 7.8e-4, warm/cold differences 6.4e-3 rad/s² | `check_qp_assembly.py` | S3 | P2 | high | yes | no | tighter eps or polish for the relaxed rows | F-4-4 |
| F-4-11 | No workspace or torque rows in the running QP; the acceleration box is the only actuator constraint | static + numerical | S3 | P2 | high | yes | no | a torque-feasibility check | F-4-6 |
| F-5-15 | OSCBF cost Hessian condition number ~4.5·10⁶ | `check_oscbf.py` 5 | S3 | P3 | high | yes | no | rescale | F-4-10 |

### B.4 Robot dynamics and torque realization

| ID | Issue | Evidence | Sev | Pri | Conf | Num | CL | Direction | Links |
|---|---|---|---|---|---|---|---|---|---|
| F-4-6 | `qddot_to_torque`: effort clip and rate limit only under `iso_enabled` (shipped false); no joint-state staleness check | static | S1 | P1 | high | static | no | always clip; age-check q | F-4-7, F-7-6 |
| F-4-7 | `RtTorqueController`: no torque clamp and no LPF, contrary to its plugin description | static (C++ L116–154) | S2 | P1 | high | static | no | clamp, or correct the description | F-1-8, F-7-6 |
| F-7-6 | `qddot_to_torque` comment "rt_torque_controller ALREADY clips" is false | static | S4 (misleading) | P1 | high | static | no | correct the comment together with F-4-6 | F-4-7 |
| F-4-9 | Gazebo plant has reflected rotor inertia; the Pinocchio model has none (feedforward realizes 3–86 % of q̈) | `check_torque_realization.py` | S2 | P1 | high | yes | recorded | the same armature in both, or document as a deliberate mismatch | F-6-7 |
| F-2-3 | Mass-matrix symmetrization depends on the Pinocchio version (4.0.0 fills both triangles) | `check_kinematics_dynamics.py` [C] (Pinocchio 4.0.0 only) | S3 | P3 | high | yes | no | symmetrize explicitly; a version test | — |
| F-1-8 | `lpf_alpha`, `tau_max_scale`, `urdf_path` written into the controller YAML but never declared → inert launch arguments | static | S3 | P2 | high | static | no | declare or remove | F-4-7 |
| F-1-3 | Torque-stack docstring: "rt_torque_controller adds g(q)" (it does not; firmware/plugin does) | static | S4 | P3 | high | static | no | correct the docstring | F-4-13 |
| F-4-13 | The same claim in `qddot_to_torque`'s module docstring | static | S4 | P3 | high | static | no | as above | F-1-3 |

### B.5 Perception and tracking

| ID | Issue | Evidence | Sev | Pri | Conf | Num | CL | Direction | Links |
|---|---|---|---|---|---|---|---|---|---|
| F-2-11 | During the LPF hold, a held entry carries `closest_point_human` = (0,0,0) with `valid = True` | `check_held_entry.py` | S1 | **P0** | high | yes | no | invalidate or repair the held point | F-3-2, F-3-10, F-3-13, F-5-12 |
| F-3-2 | No stage rejects the (0,0,0) point; v = 0 and the row uses a base-radial normal; go-around can take its geometry from it (audit C-018) | `check_held_row.py` | S1 | **P0** | high | yes | no | as F-2-11 | F-2-11 |
| F-3-13 | A held row **loosens** while a tracked obstacle closes (evidence-gated margins drop) | `check_held_row.py` (C-018 correction on the intervals) | S1 | **P0** | high | yes | no | keep the margins through a hold | F-2-11, F-3-6 |
| F-3-10 | F-2-11 reaches the RL commander (spiking v_obs observation; C-018 correction on Δt) | static + Derived | S2 | P2 | medium | yes | no | as F-2-11 | F-2-11 |
| F-2-4 | 11 spheres, not capsules: the hand is not covered (94 % of its vertices outside, up to 6.84 cm); links 0–3 have no CP; 3.2 cm gaps on the forearm | `check_geometry_and_so3.py` | S1 | P1 | high | yes | no | capsules or more CPs; a hand CP | F-2-9 |
| F-2-9 | The hand mesh in the mask is rotated by 45° (posed with link8's TF, not the hand joint) | `check_hand_mesh.py` | S2 | P1 | high | yes | no | use the `fr3_hand` frame | F-2-4 |
| F-2-7 | TF `base → camera_link` is from an old calibration (0.276 m, 5.14° off); distances unaffected (the node reads the file); C-correction: 5 later calibrations, matching `e5fcdb2` | `check_camera_extrinsics.py` | S2 | P1 | high | yes | no | regenerate from the current calibration, or derive the TF from the file at launch | F-6-14 |
| F-2-8 | Mask-dilation margin subtracted from every obstacle pixel: distances biased low by 2.8–5.6 cm (conservative) | Derived; magnitudes in `check_perception_numbers.py` | S3 | P2 | high | yes | no | subtract at the mask edge only, or document as margin | — |
| F-2-1 | Segment 5 has zero length, so 2 of 11 CPs are duplicates | `check_geometry_and_so3.py` (200 random q) | S3 | P2 | high | yes | no | drop the duplicate CP | F-2-10, F-3-12 |
| F-2-10 | Duplicate CPs make labels flip (`fr3_link6#1` ↔ `#0.2`) with `multi_obstacle_k ≥ 2` | `check_cp_labels.py` | S2 | P1 | high | yes | no | label by CP index, not position | F-2-1, F-3-12 |
| F-3-12 | Per-label state resets on a flip (C-014 correction: half of the state) | `check_cp_label_rows.py` | S2 | P1 | high | yes | no | as F-2-10 | F-2-10 |
| F-3-19 | RL `cp:fr3_link3` never matches the robot's labels (audit C-018 confirmed) | static | S2 | P2 | high | static | no | align `franka_rl/config.yaml` | F-0-10 |
| F-2-5 | Five consumers cache the joint-state index map from the first message | static | S2 | P2 | high | static | no | map per message (see the scrambled-order lead) | — |
| F-2-6 | `resolve_frame_id` falls back to a substring match | static | S3 | P3 | high | static | no | exact match, or raise | — |
| F-2-12 | Hand-eye: docstring says LM, code uses TRF + Huber (f_scale 1, quadratic zone) | static | S4 | P3 | high | static | no | correct the docstring or set f_scale | — |
| F-6-14 | The Gazebo camera is rendered from the calibration file, so sim cannot reproduce a calibration error | static | S3 | P2 | high | static | no | an optional extrinsic perturbation in sim | F-2-7 |
| F-8-9 | The barrier's h̄ uses capture-time geometry (`pr − ph` from the message); the measured sim capture age (p50 112 / p95 154 ms) exceeds `latency_t_blind` 85 ms; ≤ 16/22 mm optimism at S6A's closing rate | S6A.1 (Runtime + code) | S3 | P2 | high | S6A.1 | sim | measure the age on hardware; derive `t_blind` from it; consider FK-refreshing `pr` at the QP rate | F-8-7, F-3-6 |

### B.6 Simulation fidelity

| ID | Issue | Evidence | Sev | Pri | Conf | Num | CL | Direction | Links |
|---|---|---|---|---|---|---|---|---|---|
| F-6-7 | Sim "contact" is sphere–sphere; obstacles have no collision geometry, so Gazebo never registers a hit | static | S2 | P1 | high | static | sim (by construction) | true robot–obstacle geometry in the metric | F-2-4 |
| F-6-8 | Emulated perception clamps the gap at 0 and uses one track ID | `check_scoring.py` | S2 | P1 | high | yes | no | do not clamp; per-obstacle IDs | — |
| F-6-9 | Seeds vary only the start timing (98 of 100 scenarios) | `check_obstacle_laws.py` | S2 | P2 | high | yes | no | geometric seed variation | — |
| F-6-12 | The pentagon ignores the episode clock: its phase at the start depends on boot timing | static | S2 | P2 | high | static | no | start the trajectory on `/sim/episode_start` | F-6-9 |
| F-6-4 | Phase 5 conditions run 300 Hz + go-around; their QP-rate gate is UNKNOWN | `check_conditions.py` 4 | S2 | P1 | high | yes | recorded | add `qp_rate_hz` to those conditions | F-1-16 |
| F-1-16 | One recorded cell: rebuild 35.1 Hz vs 50 Hz configured | runtime (recorded) | S3 | P2 | medium | — | recorded | measure at the baseline | F-6-4 |
| F-4-9 | (listed in B.4) plant armature ≠ model | | | | | | | | |

### B.7 Logging and experimental validity

| ID | Issue | Evidence | Sev | Pri | Conf | Num | CL | Direction | Links |
|---|---|---|---|---|---|---|---|---|---|
| F-6-5 | Ground-truth holes up to 0.84 s, aligned with the 5 s save; a 200 ms hole hid a 3 cm penetration in a synthetic test; S6B: holes grow 91 → 562 ms over 30 s in the bare scene, one at a pass minimum; S6B.1: filled from the bag's measured joints (exact match), pass 3 min 0.12313 m inside the hole | `check_scoring.py`, `check_sim_runs.py` | S2 | **P1** (before any sim evidence) | high | yes | recorded | save off the executor thread, or once at exit | — |
| F-6-1 | No recorded run is at the baseline (1226 manifests, 11 SHAs, 1195 dirty) | `check_sim_runs.py` | S2 | P1 | high | yes | recorded | the baseline runs of REFACTORING_READINESS §9 | F-6-11, F-7-4 |
| F-6-6 | Batch `ok` means "valid data point", not "safe" (75 ok cells with contact) | `check_scoring.py` | S2 | P1 | high | yes | recorded | separate validity from outcome in reports | Q-6-2 |
| F-6-10 | Rescore uses current code and writes into old run directories | static | S2 | P2 | high | static | no | write rescores elsewhere, with the code revision | F-6-11 |
| F-6-11 | The provenance snapshot is never called | static | S2 | P1 | high | static | no | call it per run | F-5-27, F-6-1 |
| F-6-13 | F-5-21 scope: logger CSVs are affected, batch scores are not | `check_scoring.py` | S3 | P2 | high | yes | no | — | F-5-21 |
| F-5-21 | `experiment_logger` records corrupted arm rows in ≈ 25 % of samples (the finger-only JointState publisher) | `check_observability.py` | S2 | P1 | high | yes | recorded | map by name; drop partial messages | F-1-15, F-6-13 |
| F-1-15 | `/NS_1/joint_states` has two publishers (30 Hz arm + 10 Hz fingers) | runtime (recorded) | S2 | P2 | high | — | recorded | separate topic for the fingers | F-5-21, F-2-5 |
| F-5-22 | The run bag lacks `/rosout`, `/NS_1/iso_safety`, `/NS_1/torque_saturation`, `/NS_1/q_des_state` | static | S3 | P2 | high | static | no | add them | — |
| F-5-23 | Logger holds stale values with no age check; q̈ is a finite difference of 30 Hz data | static | S2 | P2 | high | static | no | age columns | F-5-25 |
| F-5-24 | The experiment log lacks the QP's own diagnostics (`/cbf_status` 41 fields) | static | S3 | P2 | high | static | no | log `/cbf_status` | F-5-25 |
| F-5-25 | Misleading logger columns (`min_h` is the logger's own d − d_safe, …) | static | S2 | P2 | high | static | no | rename, or use the filter's h | F-5-24 |
| F-5-18 | Two meanings of "S_p" on `/cbf_status[5]` | static | S3 | P3 | high | static | no | split the field | — |
| F-5-27 | `run_metadata.write_runtime_metadata` has no caller | static | S3 | P3 | high | static | no | call it, or remove it | F-6-11 |
| F-3-7 | Commander CSV `qddot_nom` is the open-loop model (C-014: the published one is `qddot_des_*`) | static | S3 | P3 | high | static | no | rename the columns | — |
| F-7-5 | A test run from the repository root imports the installed copy | probe | S2 | **P0** (for any validation) | high | probe | no | run from the package dir; a guard in every run log | F-7-4 |
| F-8-8 | `_qp_tick gap` warning: wall-clock gap vs a threshold from the sim period (3 × 10 ms); at RTF 0.63 it fires at ~1.9 periods (54 warnings vs 26 sim gaps > 30 ms) | S6A.1 (code + Runtime) | S4 | P3 | high | S6A.1 | sim | measure the gap on the node clock, or scale the threshold by the measured RTF | F-8-5 |
| F-8-10 | Sim-time timing metrics (QP rate, ages, `rows_latency`, gap counts) depend on RTF and host; perception is wall-bound, so low RTF makes it look faster; `rtf_run`/`rtf_min_window` include the startup window | S6A.1 (Runtime, Derived) | S3 | P2 | high | S6A.1 | sim | report RTF per phase; compare timing only at similar RTF; decide the RTF bar (Q-6-3) | F-8-5, F-8-7, Q-6-3 |
| F-8-12 | q̈_safe − q̈_nom in the bag is not evidence of QP intervention: the QP targets an unpublished shaped nominal (zone / governor / tangential bias …); far from the obstacle ‖Δ‖ > 0.2 on 727 / 1246 ticks with `dnorm` p95 0; no per-tick active set | S6B (code + Derived) | S3 | P2 | high | S6B | sim | log the shaped nominal and the binding rows per tick (needs approval); attribute only via slack / violation / CBFDIAG until then | F-8-11, F-4-1 |
| F-8-13 | `truth.npz` pairs each joint sample with the last received obstacle pose (queues 50 vs 100): after each save stall 49–50 ms ahead for 235–431 ms (7 % of S6B samples); no effect for a static obstacle, up to \|v\|·50 ms for a moving one | S6B.1 (code + Runtime) | S3 | P2 | high | S6B.1 | sim | pair by stamp (needs approval); until then check `t − t_pose` per sample | F-6-5 |

### B.8 ROS 2 integration

| ID | Issue | Evidence | Sev | Pri | Conf | Num | CL | Direction | Links |
|---|---|---|---|---|---|---|---|---|---|
| F-6-2 | `obstacle_row_osqp_eps_rel` reaches the node as the string `'1e-05'` (works by accident) | `check_conditions.py` 3 | S3 | P2 | high | yes | no | numeric literal; typed declaration | — |
| F-1-1 | `NS_1` hard-coded across files; it comes from `franka_bringup` | static | S3 | P3 | high | static | no | one namespace parameter | — |
| F-1-2 | `robot_ip`/`namespace` launch defaults always overridden by the bringup file | static + recorded | S3 | P3 | high | static | recorded | remove the dead defaults | — |
| F-1-5 | `start_move_group` defaults true; no Pipeline A node uses MoveIt | static | S3 | P3 | high | static | no | default false | — |
| F-1-6 | Pipeline C: no launch, `/NS_1/torque_safe` has no consumer, QoS mismatch | static | S3 | P2 | high | static | no | decide C's status (Q-1) | F-0-2, F-5-5 |
| F-1-7 | Conflicting static TFs (`world → fr3_link0`, `base → camera_color_optical_frame`) in minimal/thales/velocity stack | static | S2 | P2 | medium | static | no | one TF authority | F-2-7 |
| F-1-9 | Topics with no consumer (`/human_robot/distance`, `/NS_1/rl_status`, `q_des_state`) | static + recorded | S4 | P3 | high | static | recorded | remove or document | F-4-8 |
| F-1-12 | `robot_config_path` passed to `cbf_safety_filter` but never declared | static | S3 | P3 | high | static | no | remove or declare | — |
| F-5-10 | ISO wiring gaps: preflight sees only the YAML; overrides never preflight-checked (subagent report, spot-checked) | static | S2 | P2 (P1 if ISO is enabled) | medium | static | no | pass the effective parameters to the preflight | F-5-20 |
| F-5-20 | The preflight placeholder gate exists only in the preflight | static | S2 | P2 | high | static | no | the same gate in `_iso_configure` | F-5-10 |
| F-7-9 | `handeye_eye_in_hand_node` builds its parameter dict from the private `rclpy.node.Node._parameters` (L200); the old guide reports that `test_node_self_references` fails on it | static (E9); test not run | S4 | P3 | medium | static | no | use `get_parameters`/the declared list | — |
| F-7-10 | `params.py` helpers drop their `description` (no `ParameterDescriptor`); bounds are not registered with ROS; validation only at declaration | static (03 §9) | S4 | P3 | high | static | no | pass a descriptor with description and ranges | — |
| F-7-11 | The Stage 0 data-directory exclusion hides one tracked tool (`franka_logs/plot_franka_log.py`) from the ledger | `check_docs.py` 4 | S4 | P3 | high | static | no | none (recorded; ch. 15 §6.3 classifies it) | F-1-18 |

### B.9 Configuration inconsistencies

| ID | Issue | Evidence | Sev | Pri | Conf | Num | CL | Direction | Links |
|---|---|---|---|---|---|---|---|---|---|
| F-7-4 | **The install is a mixed tree, mostly `fr3-student-sim` (`42df80e`)**: neither the baseline nor any revision | `check_install_provenance.py` | S1 | **P0** | high | yes | no | clean, revision-stamped build (14 §5.3) | F-2-2, F-6-3, F-7-5 |
| F-6-3 | Installed workspace ≠ source (d_safe 0.15 vs 0.10; different nodes) | `check_installed.py` | S1 | **P0** | high | yes | no | as F-7-4 | F-7-4 |
| F-2-2 | The installed `franka_experiments` is stale (lacks `configure_control_points`) | probe | S1 | **P0** | high | probe | no | as F-7-4 | F-7-4 |
| F-1-10 | `d_safe` read from the installed YAML by the commander's governor and the sim start check: a ROS-parameter override reaches the filter only | Derived | S2 | P1 | high | static | no | one source for `d_safe` | F-6-3 |
| F-1-11 | `fr3_control.yaml` and the launch defaults disagree on 9 filter mechanisms + 2 keys | static (ch. 03 §5.1); `stage3_verification/effective_params.py` resolves the effective set | S2 | P1 | high | static | no | one effective-parameter table, generated | — |
| F-1-13 | `conditions.d/mppi_it08.yaml` has no reader | static | S4 | P3 | high | static | no | remove (after approval) | — |
| F-1-14 | Dead config: 3 files, 6 keys, 2 launch defaults | static | S4 | P3 | high | static | no | remove (after approval) | ch. 15 §6.2 |
| F-1-17 | Two joint-limit sources (`franka_description` vs `fr3_control.yaml`) | static | S2 | P2 | high | static | no | one source | — |
| F-5-12 | `iso_distance_hold_max_s` acts without `iso_enabled` (contrary to "ALL FALSE … adds no behaviour") | static | S3 | P2 | high | static | no | gate or document | F-2-11 |
| F-5-13 | `fr3_control.yaml` comment says d_safe 0.15; value is 0.10; tests repeat 0.15 | static | S4 | P3 | high | static | no | correct the comments | F-6-3, ch. 14 §7.4 |
| F-5-16 | Stale velocity-executor YAML (`rt_velocity_blender.yaml`) and `[ACTIVE]` diagnostics | static | S3 | P3 | high | static | no | remove the stale file, or wire it | F-5-7 |
| F-3-8 | `path_kp_cart` units in comments are N/m (actual s⁻²) | static | S4 | P3 | high | static | no | correct the comments | — |
| F-7-7 | `test_rate_scaling.py::test_the_shipped_config_states_durations` asserts `tracking.max_coast_s == 0.167`; the shipped `fr3_complete.yaml` says 0.8 (since 2026-11-01): the test must fail at the baseline | `check_guide_vs_code.py` E7 (static; not run) | S3 | P2 | high | static | no | update the test or the value, after Q-2-2 | Q-2-2, F-7-5 |

### B.10 Code structure and maintainability

| ID | Issue | Evidence | Sev | Pri | Conf | Num | CL | Direction | Links |
|---|---|---|---|---|---|---|---|---|---|
| F-7-2 | 4 of the 22 `TODO[LEGACY]` markers are false or misleading | `inventory.py`; ch. 15 §5 | S3 | P2 | high | static | no | rewrite the markers from the inventory | F-0-4 |
| F-0-2 | 6 of 22 console scripts are started by no production launch | static | S4 | P3 | high | static | no | ch. 15 classification | F-1-6 |
| F-0-3 | `static_scene.py`, `surface_velocity.py` imported only by each other and tests ("not yet wired") | static | S4 | P3 | high | static | no | user decision (Q-7-2) | F-0-4 |
| F-0-4 | Modules reached by no executable (candidates; Stage 7 confirmed against dynamic use, ch. 15 §4.4) | static, confirmed | S4 | P3 | high | static | no | ch. 15 §9 removal rules | F-3-1, F-3-18, F-7-2 |
| F-0-5 | `simexp` imports two `franka_experiments` scripts by relative source path | static | S3 | P2 | high | static | no | move them into the package | ch. 15 §6.3 |
| F-0-6 | `franka_rt_controllers` not declared in `franka_experiments/package.xml` | static | S3 | P3 | high | static | no | declare it | — |
| F-0-7 | `rt_velocity_executor_controller` is implemented in `rt_velocity_blender_controller.cpp` | static | S4 | P3 | high | static | no | rename (careful: plugin name) | — |
| F-0-8 | Parameters declared through helpers; the per-node surface is not visible | Extracted → resolved in ch. 03 | S4 | P3 | high | static | no | generated parameter docs | — |
| F-0-10 | `find_sim_root` looks for `franka_sim`; the directory is `franka_rl` (parity tests skip) | static (skip effect Unverified) | S3 | P2 | high | static | no | fix the name | F-3-19 |
| F-3-18 | `avoidance.py` docstring claims a user that does not exist; a velocity-commander docstring promises C² | static | S4 | P3 | high | static | no | correct the docstrings | F-0-4 |
| F-3-11 | `test_phase_governor.py` mirrors the node with stale defaults (a replica test) | static | S3 | P2 | high | static | no | test the node function itself | F-3-3, ch. 14 §7.4 |
| F-3-3 | Phase governor breaks the chain rule (v_d, a_d not the derivatives of p_d under σ) | `check_timing_governor.py` | S2 | P1 | high | yes | no | scale ṡ by σ and include σ̇ | F-3-11 |
| F-3-9 | `CompositePath` is only C⁰ in velocity at the junction | `check_timing_governor.py` | S2 | P2 | high | yes | no | a C¹/C² blend | — |
| F-0-1 | An untracked test file vanished before Stage 0 (not in any commit) | git | S4 | P3 | high | — | — | none (recorded only) | — |
| F-8-1 | The torque-stack launch golden (`97db307`) is stale: 8 failures, 2 leaves (`robot_config_path` of `real_time_distance` and `cbf_safety_filter`); the launch has no green regression guard | S5 run (Runtime) | S3 | P2 | high | S5 | no | the user decides: regenerate the golden after reviewing the two leaves | F-8-2 |
| F-8-2 | No green automated gate: pytest red by construction (35 known), `colcon test` exits 0 and skips the `franka_simulation_experiments` suite | S5 run (Runtime) | S3 | P1 | high | S5 | no | judge runs by the failing set; register the simexp suite; fix or mark the known failures | F-0-3, F-7-7, F-7-9, F-8-1 |
| F-8-3 | `smoke_cbf_construct.py`: stale `FakeKin` stub (no `configure_control_points`) and exit 0 on failure | S5 run (Runtime) | S4 | P3 | high | S5 | no | update the stub; exit non-zero on failure | F-8-2 |
| F-8-4 | `franka_simulation` fails its own `ament_lint_auto` (1155 checks); cppcheck self-skips in both C++ packages | S5 colcon test | S4 | P3 | high | S5 | no | clean up or narrow the linters | — |
| F-8-5 | `full` misses its QP rate in Gazebo at the baseline: 73.6 vs 100 Hz unpinned (sim-time timer and gate, so not a clock artefact; ≈ 46 per wall s; S6A.1: tied to constraint rebuilds, in-process contention Inferred; S6B: 88.4 per sim s, filter process ≈ 1.06 cores over 4 threads, GIL Inferred); every `full` cell is `ok = False` on this host | S6A run (Runtime) | S2 | P1 | high | S6A | sim | decide: nominal rate, build cost, a validated `--filter-cpu`, governor | F-6-4, F-8-2 |
| F-8-6 | Unclean Gazebo teardown after DONE: `gz_server` SIGABRT in the ign_ros2_control destructor; Python nodes raise on SIGINT (double `rclpy.shutdown`) | S6A `launch.log` | S4 | P3 | high | S6A | sim | clean shutdown handling; exclude teardown from the "no process died" check | — |
| F-8-7 | Sim perception delivers 30.7 Hz of a 90 Hz camera (about 57–65 ms wall per frame; frames dropped by design, wall-bound, S6A.1); distances reach the filter 112 ms old (median) | S6A run (Runtime) | S3 | P2 | high | S6A | sim | profile `real_time_distance`; decide the bound before moving-obstacle cells | F-6-8, F-6-14 |

### B.11 Documentation inconsistencies

| ID | Issue | Evidence | Sev | Pri | Conf | Num | CL | Direction | Links |
|---|---|---|---|---|---|---|---|---|---|
| F-0-9 | The technical guide predates 19 commits that touch the packages (20 by `git rev-list`, Part E.1) | git | S4 | P3 | high | — | — | Part E | F-7-8 |
| F-1-4 | Velocity-stack docstring names the wrong distance topic | static | S4 | P3 | high | static | no | correct the docstring | — |
| F-5-1 | `bypass_cbf` defaults to false (QP active); the launch docstring and ch. 01 said true (ch. 01 corrected) | `check_velocity_cbf.py` 4 | S3 | P3 | high | yes | no | correct the docstring | — |
| F-5-26 | Stale tool defaults (`watch_obstacle_tracks` listens to an unpublished topic, …) | static | S4 | P3 | high | static | no | update the defaults | — |
| F-7-3 | `select_gamma` docstring: γ ≈ 6.0 at δ = 0.1 m (code: 3.65) | `check_unread_functions.py` 5 | S4 | P3 | high | yes | no | correct the docstring | — |
| F-1-18 | Stage 0 extractor missed a file (fixed in the tool) | ledger | S4 | P3 | high | — | — | none (fixed) | — |
| F-7-8 | The old technical guide is stale (E.3: QP size, fallback, slew rule, FIFO, logger truth) and misses the 10 mechanisms of E.4; its config values still match but 51 keys are missing | `check_guide_vs_code.py` (21 checks) | S3 | P3 | high | yes | — | Part E; do not use the guide as reference | F-0-9 |
| F-8-11 | S6A record overstated "no obstacle row bound / only the box" (`n_act` = count h̄ < 0; slew bound and the gap-dependent link-speed cap also acted) and F-8-5's budget argument; corrected by dated notes | S6A.1 (code + Derived) | S4 | P3 | high | S6A.1 | — | none beyond the notes; read binding from per-row data in future runs | F-5-23, F-8-5 |

Total: 138 findings (B.6 repeats F-4-9 only as a pointer). `tools/stage7_verification/check_docs.py`
asserts that every ID of PROGRESS §4 has exactly one row here.

---

## Part C — Duplicates, shared causes and contradictions

### C.1 Groups with one root cause (fix the root, close the group)

| Root cause | Findings | Note |
|---|---|---|
| R1 the install is not the source | F-2-2 → F-6-3 → **F-7-4** (cause) → F-7-5 (tests) | Q-6-1 answered by F-7-4. One clean build closes F-2-2/F-6-3/F-7-4; F-7-5 needs a run rule |
| R2 the held distance entry | **F-2-11** → F-3-2, F-3-13 (loosening), F-3-10 (RL), F-5-12 (hold is not ISO-gated) | One fix in `distance_engine`/`perception_msgs` |
| R3 duplicate control points | **F-2-1** → F-2-10 → F-3-12 | Removing the duplicate CP closes the chain |
| R4 the soft barrier | **F-4-1**, F-4-2, F-4-3, F-4-10, F-3-5 | A design choice, not one bug: the safety claim must be restated or the slack mode changed |
| R5 gravity and torque-limit claims in comments | F-1-3, F-4-13, F-7-6 (comments) ↔ F-4-6, F-4-7, F-1-8 (behaviour) | The comments are wrong in the reassuring direction |
| R6 Pipeline C unwired | F-0-2, F-1-6, F-5-4, F-5-5, F-5-6, F-5-15 | Status decision (Q-1) before any fix |
| R7 logger validity | F-1-15 → F-5-21 → F-6-13; F-5-23, F-5-24, F-5-25 | Joint-state mapping by name closes F-5-21 |
| R8 provenance | F-5-27, F-6-11, F-6-1, F-6-10 | One provenance call per run closes the first two |
| R9 `d_safe` value drift (0.10 vs 0.15) | F-5-13, F-6-3, F-1-10, ch. 14 §7.4 (test docstrings) | The value is 0.10 in the source; 0.15 is in comments, tests and the install |
| R10 unused modules | F-0-3, F-0-4, F-3-1, F-3-18, F-7-2 | Ch. 15 §4.4 is the consolidated list |
| R11 ISO layer | F-5-8 … F-5-14, F-5-17 … F-5-20 | Dormant while `iso_enabled` is false |
| R12 phase governor | F-3-3, F-3-11 | The test mirrors the buggy function, so it cannot catch the fix or a regression |

### C.2 Corrected findings: the latest interpretation, with history kept

| Finding | Original claim | Corrected to | Where |
|---|---|---|---|
| F-2-7 | "as of `d4e4028`", "three recalibrations of September" | the matching calibration is `e5fcdb2`; **five** later calibrations; the 0.276 m / 5.14° conclusion stands | PROGRESS §4 (correction text) |
| F-3-4 | the YAML comment also wrong | the YAML comment agrees with the code; only the module and class docstrings are wrong (C-014) | PROGRESS §4 |
| F-3-7 | the CSV `qddot_nom` is mistaken for the published command | the CSV also logs `qddot_des_*`, the header calls the nominal "ideal"; the defect is naming (C-014) | PROGRESS §4 |
| F-3-10 | spike from the 90 Hz interval | Δt is the 100 Hz policy tick (C-018) | PROGRESS §4 |
| F-3-12 | "resets that state" | half right: only on a flip to a NEW key (C-014) | PROGRESS §4 |
| F-3-13 | "while the true gap shrinks 4.4 cm" | the two intervals were mixed; corrected numbers (C-018) | PROGRESS §4 |
| F-5-1 | ch. 01 said the velocity QP is bypassed by default | it is active by default; ch. 01 corrected, old wording kept | 01 §4.2, 10 §2.1 |
| F-5-24 … F-5-27 | recorded as F-5-16 … F-5-19 by a parallel session | renumbered at C-035; the original numbers stay with the earlier rows | PROGRESS §4 |
| Q-6-1 | "why does the install differ?" | answered by F-7-4 | PROGRESS §5 |
| ch. 06 singularity recovery | "a profiled return" | F-7-1 (6.R1) | 06 §4.4 note |

### C.3 Contradictory evidence, discussed

- **The `TODO[LEGACY]` markers versus the import graph (F-7-2).** The markers were written on
  2026-09-01. Later commits re-wired `self_collision.py` into the row builder and kept Pipeline B
  alive, so the code moved and the markers did not. The import graph at the baseline wins.
- **Torque clipping (F-4-7 versus the comments).** The plugin description and the
  `qddot_to_torque` comment say the controller clips; the C++ source says "nessuna saturazione SW"
  and has no clamp. The source wins. Whether `franka_hardware`/libfranka/firmware clip is outside
  this repository (Unverified).
- **The ground-truth rate (F-6-5 versus the lead "truth.npz 448–688 Hz").** The lead measured an
  average rate. F-6-5 shows the average is fine (909–997 Hz in 39 recent runs) and the defect is
  **holes**, aligned with the 5 s save. Both observations are compatible; the hole is what breaks
  derivatives and hides contacts.
- **`d_safe` 0.10 versus 0.15 (R9).** All evidence agrees once the install (F-7-4) is separated from
  the source: the source is 0.10, everything that says 0.15 is a comment, a test docstring, the
  installed `fr3-student-sim` copy, or a recorded run of an older revision.

---

## Part D — Risk-based prioritization (provisional)

### D.1 P0: investigate before any hardware use

| # | Findings | Why P0 | First investigation (proposal, not executed) |
|---|---|---|---|
| 1 | F-7-4, F-6-3, F-2-2, F-7-5 | the code that would run is not the documented code (`d_safe` 0.15, different filter); every later validation depends on this | clean revision-stamped build (14 §5.3) + `check_install_provenance.py` |
| 2 | F-4-1, F-4-2 (+ F-4-3, F-3-5) | the obstacle barrier is soft and coupled; every binding tick violates the HOCBF inequality | closed-loop measurement of the per-row violation in Gazebo (protocol Phase 4) |
| 3 | F-2-11, F-3-2, F-3-13 | the barrier loosens during a perception hold while an obstacle closes | a Gazebo scenario with a forced hold during an approach |
| 4 | F-4-12 | no joint-limit, self-collision or singularity rows, and no fault, until the first distance message | start-up test with perception delayed |
| 5 | F-4-5 | the stale-joint-state fallback can drive a joint backwards open-loop | fault injection: freeze `/NS_1/joint_states` |
| 6 | F-7-1 | the default-on singularity recovery requests a reference far outside the joint range | unit test with v₀ ≠ 0; a sim run that triggers it |
| — | conditional: F-4-8, F-5-2, F-5-3, F-5-7 | P0 **if** `use_torque_controller:=cbf` or Pipeline B is used on hardware | Q-7-1 |

### D.2 P1: important correctness or reliability

F-4-6, F-4-7, F-7-6, F-2-4, F-2-9, F-2-7, F-2-10, F-3-12, F-3-3, F-3-16, F-4-3, F-4-9, F-6-4, F-6-5,
F-6-6, F-6-7, F-6-8, F-6-1, F-6-11, F-5-21, F-1-10, F-1-11, F-3-5.

### D.3 P2: functional and architectural

F-2-1, F-2-5, F-2-8, F-3-6, F-3-9, F-3-10, F-3-11, F-3-14, F-3-15, F-3-17, F-3-19, F-3-20, F-4-4,
F-4-10, F-4-11, F-1-6, F-1-7, F-1-8, F-1-15, F-1-16, F-1-17, F-0-5, F-0-10, F-5-4, F-5-5, F-5-6,
F-5-8, F-5-9, F-5-10, F-5-11, F-5-12, F-5-14, F-5-17, F-5-19, F-5-20, F-5-22, F-5-23, F-5-24, F-5-25,
F-6-2, F-6-9, F-6-10, F-6-12, F-6-13, F-6-14, F-7-2, F-7-7.

### D.4 P3: cleanup and documentation

F-0-1, F-0-2, F-0-3, F-0-4, F-0-6, F-0-7, F-0-8, F-0-9, F-1-1, F-1-2, F-1-3, F-1-4, F-1-5, F-1-9,
F-1-12, F-1-13, F-1-14, F-1-18, F-2-3, F-2-6, F-2-12, F-3-1, F-3-4, F-3-7, F-3-8, F-3-18, F-4-13,
F-5-1, F-5-13, F-5-15, F-5-16, F-5-18, F-5-26, F-5-27, F-7-3, F-7-8, F-7-9, F-7-10, F-7-11.

---

## Part E — The old technical guide versus the code

> Stage 7, 2026-11-11 (checkpoint C-046). Subject: `franka_experiments/docs/FRAMEWORK_COMPLETE_TECHNICAL_GUIDE.md`
> (5,813 lines, last changed in `ce80f36`, 2026-11-04), compared with the **baseline** `ba68077`.
> The guide was used only as a source of leads (CLAUDE.md). Every row below cites the baseline
> source; nothing here trusts the guide. Reproduction:
> `python3 -I -B docs/framework_study/tools/stage7_verification/check_guide_vs_code.py` (host,
> reads `git show ba68077:<path>` only; 21 of 21 checks PASS, output in
> `tools/stage7_verification/results/observed_2026-11-11.txt`).

### E.1 Method, and what the guide's revision really is

- **The guide describes dirty working trees, not commits.** Its §15.6 and §15.11.5 say so, and the
  script confirms it. `torque_control_stack.launch.py` differs between the commit `ce80f36` and the
  baseline (the `cbf_param_overrides` hatch was added in between), but the SHA-256 the guide
  recorded for it **equals** the baseline blob (check E11). So "the guide's revision" is defined
  here by the guide's own digests, not by `ce80f36`. (Verified)
- **20 commits** touching `franka_experiments` or `franka_simulation_experiments` lie between `ce80f36`
  and the baseline (`git rev-list --count ce80f36..ba68077 -- <the two packages>`). F-0-9 says 19.
  The difference is a counting detail of Stage 0 (the baseline commit itself is included here). F-0-9
  stays as written; this count supersedes its number. (Verified)
- **Compared mechanically (script):** §15.2 configuration values, §15.3 node parameter index, §15.6 /
  §15.11.5 file digests, every relative link, every identifier in the "Implementation" column of
  §15.1 and the "Entry points" of §15.4, every `d_safe` statement, and eleven specific statements
  (E1–E11).
- **Compared by reading** (guide section against the baseline function and against chapters 04–13,
  which were themselves verified against the baseline): §1.1, §6.7, §7.3–7.4, §8.7, §9.1–9.4, §10.2–
  10.4, §12.7, and the 57-row findings table §13.1. The other guide sections (§2–§5, §11, §12.1–12.6,
  §14) were **scanned for the mechanisms that changed after the guide**, not re-derived line by line.
  Their mathematics is covered independently by chapters 04, 05, 10, 13 and was not compared
  equation by equation (coverage gap, E.8).

### E.2 Mechanical comparison (Verified, `check_guide_vs_code.py`)

| Section of the guide | Result at the baseline | Meaning |
|---|---|---|
| §15.2, 10 YAML blocks | **0 changed values**, 0 keys removed; **51 keys added** since the guide (45 in `fr3_control.yaml params`, 6 in `fr3_complete.yaml distance`). `launch_defaults.yaml`, `fr3_distance.yaml`, `oscbf_params.yaml`, the four extrinsics files and `fake_hw_controller_params.yaml` are identical. | The listed values are still right; the list is incomplete. Every added key belongs to a mechanism of E.4. |
| §15.3, 21 nodes, 384 parameter rows | 0 parameters gone; 40 rows at another line; 1 literal declaration not listed (`experiment_logger.truth_topic`, restored by `7da68ac`). | The node-local parameter index is still usable as an index; its line numbers are not. `cbf_safety_filter`'s spec-declared parameters are not in §15.3 by design (the guide sends the reader to §15.2). |
| §15.6 + §15.11.5 digests (267 paths) | 217 still equal the baseline blob; **19 differ**; 31 are absent from this repository (`franka_description/` is git-ignored here, `analysis/` is not tracked). | The 19 changed files are exactly where the post-guide mechanisms live: `cbf_safety_filter.py`, `cbf_state_rows.py`, `cbf_qp_assembly.py`, `cbf_hard_limits.py`, `config.py`, `experiment_logger.py`, `real_time_distance.py`, `distance_engine.py`, `mask_builder.py`, `perception_msgs.py`, `kinematics.py`, `logging_utils.py`, both YAMLs, five tests. |
| Relative links (248) | 23 targets absent at the baseline: `../../franka_description/...` (ignored), `../Papers/*.pdf`, `paper_draft/*.pdf`, `../../KINEMATICS_*.md`, `../../analysis/...`. | The guide's quantitative claims that rest on `analysis/` scripts and JSON (§15.11: the 4.458×10⁻¹⁴ N·m dynamics agreement, the DIAG-bag numbers) **cannot be reproduced from this repository**. They stay Unverified. |
| §15.1 + §15.4 identifiers (359) | all 359 are defined somewhere at the baseline. | No mechanism the guide names was renamed or deleted. What changed is **added** mechanisms and **changed defaults**. |

### E.3 Outdated equations and changed control laws

| # | Guide statement | Baseline | Evidence | Effect if a reader trusts the guide |
|---|---|---|---|---|
| E-1 | (1.1), §9.1: x = [u; s] ∈ ℝ¹³, **six** family slacks (obstacle, self-collision, joint, singularity, retreat, point speed). | x ∈ ℝ¹⁴ in the shipped `family` mode: **seven** families (+ `s_ssb`, the self-shadow barrier); ℝ^(14+K) = ℝ⁴⁶ in `row_l1` mode (K = `obstacle_row_slack_slots` = 32). | `cbf_state_rows.py` `N_SLACK = 7`, `NX = NV + N_SLACK` (L1513–1515); at `ce80f36` `N_SLACK = 6`; ch. 08 (8.2); check E1 | Wrong matrix sizes in any reimplementation or log parser (the status array, the slack columns). |
| E-2 | (9.2)–(9.4): one penalty ½ρ_g s_g² per family; "ten obstacle inequalities use one s_obs". | Still the **default** (`obstacle_slack_mode: family`), but `row_l1` exists: one slot per obstacle row with an ℓ1 + ℓ2 price (`rho_slack_l1`) and a tighter `obstacle_row_osqp_eps_rel`; rows beyond the slots fall back to the shared slack. | `fr3_control.yaml` L225–229; ch. 08 (8.14); F-4-1, F-4-2; check E3 | The guide's coupling analysis is right **for the default**, and silent on the mode that removes the coupling (Q-4-1). |
| E-3 | §9.4 (9.15)–(9.16): failed QP → **box-only QP with the same objective first** ("can continue toward the nominal target with no obstacle rows"), then the brake. | `qp_failure_policy: brake` is the shipped value: a failed QP goes **directly** to (8.16) q̈ = clip(−k_brake q̇, state box ∩ slew window). The box-only rung exists only under `qp_failure_policy: box_only`. The brake is slew-limited (`brake_slew_limited: true`). | `cbf_safety_filter._solve` L1624–1725 (`getattr(self.P, 'qp_failure_policy', 'brake') == 'box_only'`); `fr3_control.yaml` L2297, L693; commit `25be305`; ch. 08 §5.2; check E2 | **Changed control law.** The guide's §13.1 row "ordinary failed QP first drops barriers and continues toward target" no longer describes the default. Its risk is now behind a non-default flag. |
| E-4 | §8.7, §13.1: in a disjoint case the slew window wins and the final box leaves the state interval ("postponing required braking"). | `slew_yields_to_state_box: true` (shipped): for a joint where the slew window and the state box are disjoint, the **state box wins**. `slew_rate_compensation` and `slew_dt_ratio_max` scale the window with the measured tick. | `fr3_control.yaml` L679; ch. 08 §2.4, L184–185; commit `25be305` | The guide's warning is superseded by default. It is still the behaviour with `slew_yields_to_state_box: false`. |
| E-5 | (7.7)–(7.8): the drift c = n̂ᵀJ̇_p q̇ "omits both ṅᵀv_rel and −nᵀa_h". | Still true **by default**. The optional curvature term κ = ‖(I − n̂n̂ᵀ)v_rel‖²/ρ_c (`enable_curvature_term`, off) adds the ṅᵀv_rel part; −n̂ᵀa_o is still omitted. Row: h_qp = k₁(Aq̇_cbf − v_obs) + k₀ max(h̄, h_pen) + c [+ b_ff] [+ κ]. | `cbf_qp_assembly.build_row_rhs` docstring; ch. 07 (7.5), (7.10); commit `8394f89`; check E5 | Equation correct for the default; the guide does not know the term exists. |
| E-6 | §7.4: the margin stack h̄ = h_geom − (brake, uncertainty, latency, range, standoff margins). | Adds the **evidence margin** h_ev (per evidence class: fresh / young / coasting / untracked; `enable_evidence_margin`, off) and the **obstacle memory** (`enable_obstacle_memory`, off): (7.12) h̄ = h − h_brake − h_unc − h_lat − h_rng − h_std − h_ev. | `cbf_state_rows.evidence_margin` L1588, `build` L2440–2444; ch. 07 (7.12); commit `5aa81df`; check E5 | Missing mechanism (default off). |
| E-7 | No self-shadow barrier anywhere in the guide. | `ssb_rows_enabled` (filter) and `distance.ssb_enabled` (perception) exist, both **off**; own slack family `s_ssb` (always in x, see E-1). | `fr3_control.yaml` L240; `fr3_complete.yaml distance.ssb_*`; commit `ba68077`; ch. 05, 08 | Missing mechanism (default off), but it changes the QP size even when off (E-1). |
| E-8 | §10.3: "the main filter attempts a SCHED_FIFO priority of 50 for the QP thread". | `qp_thread_fifo_prio: 0`: no per-thread elevation (GIL inversion, see the PROGRESS §7 lead). | `fr3_control.yaml` L301–304; ch. 02 L123; check E4 | Changed default. Latency reasoning in the guide that assumes FIFO is outdated. |
| E-9 | §12.7, §13.1: "`ExperimentLogger` no longer declares `truth_topic`/`truth_n_cp`; the CSV truth columns are absent". | Both are declared again (`experiment_logger.py` L158–159). | commit `7da68ac`; check E6 | Superseded: the guide's "harness targets a deleted API" is no longer true. |
| E-10 | §6.7 (6.24): recovery quintic ends at q̇(T) = v₀; increasing T cannot repair it. | **Agreement.** `singularity_recovery.py` is byte-identical at `ce80f36` and the baseline. F-7-1 adds what the guide did not quantify: with \|v₀\| > `recovery_vmax` the duration loop never converges and the reference leaves the joint range by tens of rad. | check E10; F-7-1; `check_unread_functions.py` 1–4 | The guide classed it as a reference discontinuity; it is a P0 candidate (D.1 #6). |

### E.4 Mechanisms added after the guide (missing from it)

All of them are Verified at the baseline in the chapter named. "Default" is the shipped value
(`fr3_control.yaml` / `fr3_complete.yaml`, and the torque launch where it overrides).

| Mechanism | Commit | Default | Chapter |
|---|---|---|---|
| Per-row ℓ1 obstacle slack (`obstacle_slack_mode`, `obstacle_row_slack_slots`, `rho_slack_l1`, `obstacle_row_osqp_eps_rel`, `obstacle_row_slack_eps`, `obstacle_slack_max`) | `25be305` | `family` (off) | 08 (8.14) |
| QP failure policy and slew-limited brake (`qp_failure_policy`, `brake_slew_limited`) | `25be305` | **brake (on)** | 08 §5.2 |
| Slew box yields to the state box (`slew_yields_to_state_box`, `slew_rate_compensation`, `slew_dt_ratio_max`) | `25be305` | **on** | 08 §2.4 |
| Curvature term κ of ḧ (`enable_curvature_term`, `curvature_*`) | `8394f89` | off | 07 (7.5) |
| Evidence-conditioned margin and obstacle memory (`enable_evidence_margin`, `evidence_*`, `enable_obstacle_memory`, `memory_*`) | `5aa81df`, `9884c4e` | off | 07 §3.6 |
| Self-shadow barrier (`ssb_*` in both YAMLs) | `ba68077` | off | 05, 08 |
| QP-thread FIFO switch (`qp_thread_fifo_prio`) | (key added by the time of `ba68077`, `git log -S`) | **0 (off)** | 02 |
| Static-scene hold and surface-velocity estimator (`static_scene.py`, `surface_velocity.py`) | `2402e58` | not wired (F-0-3) | 15 |
| `cbf_param_overrides` launch hatch | (described by the guide §15.10.7, hashed equal) | empty | 03, 12 |
| Simulation: `wander` law, QP-rate gate per condition, mesh sampling, isolation arms, run freezing | `277d446`, `12e135f`, `4db4af2`, `9884c4e`, `bf20c5c` | — | 12 |

### E.5 Configuration and launch defaults

- **No configuration value the guide prints has changed** (0 of 263 + 102 + … compared keys). The
  guide's `d_safe` statements already use 0.10 m and record the 1 November change from 0.15
  (guide L1470, L2889, L5556). The baseline agrees: `params.d_safe: 0.1`, `d_safe_default: 0.15`
  (the second is dead configuration, F-1-14). (Verified)
- **Launch defaults are unchanged**: the guide's digests of `launch_defaults.yaml`,
  `torque_control_stack.launch.py`, `minimal.launch.py` and `velocity_cbf_control_stack.launch.py`
  equal the baseline (E11). The guide's statement that "the torque launch enables several CBF
  features whose raw YAML value is false" agrees with F-1-11. (Verified)
- The guide prints configuration **file values**. The value a running node uses is the **installed**
  file (F-1-10, F-6-3, F-7-4), which at the time of this study belongs to another branch. The guide
  never mentions this route. (Verified, ch. 14)

### E.6 The guide's own findings table (§13.1) at the baseline

Status: **holds** (still true at the baseline), **superseded** (a later commit changed it),
**not re-checked** (no Stage 0–7 reading covers it; a lead only).

| Guide §13.1 row (short) | Status at `ba68077` | Our finding / chapter |
|---|---|---|
| Rigid-offset derivative and six-point interpolation corrected in the main path | holds (corrections present) | ch. 04 (Verified, 126/126 checks) |
| Python dynamics velocity torque `nle − g` | holds | ch. 09 |
| Barrier is not the smooth distance the row differentiates | holds | ch. 07 §2, F-2-8 |
| HOCBF omits normal and obstacle-acceleration drift | holds by default; κ optional (E-5) | ch. 07 (7.5) |
| State envelope can lose to slew continuity | **superseded** by default (E-4) | ch. 08 §2.4 |
| Ordinary failed QP first drops barriers | **superseded** by default (E-3) | ch. 08 §5.2, F-4-4 |
| Accepted inaccurate / clipped iterate not residual-gated | holds | F-4-10, ch. 08 §5.2 |
| Invalid numeric inputs are not rejected at ingress (shape-only check of `qddot_nom`) | not re-checked | — |
| No first perception message also gates internal barriers | holds | **F-4-12** (P0) |
| Empty scenes and absent geometry are not verified free space | holds: the empty-frame fault exists only with `iso_enabled` | ch. 08 §5.1 row 5, ch. 11 §6.4 |
| Phase governor omits derivative rescaling | holds | **F-3-3** |
| Geometry frozen between rebuilds | holds | ch. 08 §1, F-1-16 |
| Point-speed rows depend on accepted obstacle rows | not re-checked | ch. 07 §4 (`spd_pts`) |
| Soft constraints allow physical violations ("six shared finite slacks") | holds, now **seven** families (E-1) | **F-4-1, F-4-2** |
| Negative barrier floor weakens recovery | holds | ch. 07 (7.11) |
| Limited self-collision coverage | holds | ch. 08 (8.10) |
| Singularity row can disappear near flat minima | holds (not quantified) | ch. 08 |
| Raw tracker velocity bypasses denoising | holds | ch. 07 (7.14), F-3-13 |
| Kalman covariance is not a coverage guarantee | holds | ch. 05 §7 |
| Velocity diagnostic assigned before the median | not re-checked | — |
| `find_sim_root` searches `franka_sim` | holds | **F-0-10** |
| A shipped-configuration test contradicts the shipped configuration | **holds** (statically, E7) | **F-7-7** (new) |
| Association gate no longer sized to the coast | holds | Q-2-2 |
| Go-around steers off rejected geometry; tangent to one row only | not re-checked as a finding (ch. 06 §5 reads `go_around_step`) | — |
| Approach-gate diagnostic covers one of two margins | not re-checked | — |
| The covariance margin is priced against closing speed (`margin_gate_on_approach`) | holds as a description: w_gate = smoothstep(v_o/0.15) on h_unc and h_lat | ch. 07 §3.6 L329 |
| Recovery quintic does not end at rest | **holds**, worse than stated | **F-7-1** |
| Older torque orientation error sign | holds | **F-5-6** |
| Runtime feedback changes the QP acceleration | holds | ch. 09, F-4-9 |
| Converter saturation log overstates downstream clipping | holds | **F-4-6, F-4-7, F-7-6** |
| Independent torque / acceleration ages | holds | ch. 09 §4 |
| Velocity post-filtering invalidates the CBF guarantee | holds | **F-5-3** |
| Velocity executor has no watchdog | holds | **F-5-7** |
| OSCBF upper-bound coefficient and missing α₁ḣ | α₁ḣ holds (**F-5-4**); upper-bound coefficient not re-checked | F-5-4 |
| OSCBF obstacle QoS mismatch | holds | **F-1-6** |
| `CBFTorqueController` setpoint bypasses the safe acceleration | holds | **F-4-8** |
| ISO mode inconsistent with defaults | holds | **F-5-9**, F-5-10 |
| Stop-reset feedback can sustain the fault | holds | **F-5-17** |
| Capture-timestamp fallback does not reject old data | not re-checked | — |
| Logging/visualisation are not synchronised metrology | holds | F-5-21, F-5-23 |
| Frame fallback is permissive | holds | **F-2-6** |
| Rectifier map cache omits R | not re-checked | — |
| `iso_constants_measure.py` broken paths | not re-checked (read at interface level) | ch. 11 |
| Perception display switches are not control switches | not re-checked | — |
| Simulation and converter use different inertia | holds | **F-4-9** |
| Emulated perception bypasses the estimator ablations | holds | **F-6-8** |
| Simulation logger targets a deleted API | **superseded** (E-9) | — |
| Simulation limit monitor is passive | holds | ch. 12 §6 |
| Start-up timers are not readiness evidence | holds | ch. 14 §8 |
| Calibration node reads private `self._parameters`; the self-reference test fails | code holds (E9); test outcome not re-run | **F-7-9** (new) |
| `motion_source:=none` undocumented, does not gate MoveIt | not re-checked | F-1-5 (related) |
| `fr3_control.yaml joint_limits` effort column 500 N·m | holds (E8) | **F-1-17** |
| The two ablation arms ran at different QP rates | holds for that recorded batch; the per-cell QP-rate gate was added later (`12e135f`) | F-6-4, F-6-1 |
| Two ablated terms already off in the baseline | holds (`enable_velocity_feedforward`, `enable_sensor_range_uncertainty` false, L1743, L2392) | ch. 07 |
| The new parameter route into the filter has no test | not re-checked | — |

**Count:** the guide table has 57 rows; E.6 has 54 (the three kinematics/dynamics rows are one, the two
go-around rows are one). 40 hold, 3 superseded, 11 not re-checked. Of those that hold, 1 was not yet
in our log (F-7-7), and 1 is new as a code fact (F-7-9). The guide anticipated F-7-1, F-4-12, F-3-3,
F-4-1/F-4-2, F-4-8 and F-5-4 independently, which raises the confidence in those findings.

### E.7 Incorrect or unsafe assumptions to avoid when reading the guide

1. **"Six slack families", "13 variables".** Wrong at the baseline (E-1).
2. **"A failed QP continues toward the target without barriers".** No longer the default (E-3).
   The opposite hazard now applies: the brake is open-loop in q̇ and slew-limited (ch. 08 §5.2,
   F-4-5 for the stale-state variant).
3. **"The state box can lose to the slew window".** Not with the shipped flag (E-4).
4. **"The hashes identify the current tree".** They identify dated dirty trees (E.1); 19 files moved.
5. **The guide's file values are what runs.** Only if the install matches the source (F-7-4).
6. **The guide's numeric evidence (§15.9–15.11) is reproducible.** Its scripts and JSON live in
   untracked `analysis/` directories and are not in this repository (E.2). Unverified.

### E.8 What Part E did not do

- The guide's §2–§5 (kinematics, calibration, depth geometry, tracking), §11 (alternatives), §12.1–
  12.6 and §14 (worked example) were not compared equation by equation. Their subject is covered by
  chapters 04, 05, 10, 13 from the baseline, so nothing in the refactor depends on the guide. A
  line-by-line audit of those sections would only measure the guide's quality.
- The 11 "not re-checked" rows of E.6 are leads, recorded here, not findings.
- No test was run (`pytest` needs approval): F-7-7's failure is derived from the assertion and the
  shipped YAML, not observed.

### E.9 Consequences for the refactor

- The guide must not be used as the reference for any change. The reference is this documentation
  at the baseline. The guide's §13.1 rows that **hold** are useful as an independent second opinion.
- Any later edit of the guide should start from E.3 (outdated statements) and E.4 (missing
  mechanisms). That is a documentation task, outside the read-only phase.
