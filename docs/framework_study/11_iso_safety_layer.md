# 11 — The ISO-inspired safety layer: speed and separation monitoring, the monitor, the evidence

> **Source baseline:** `ba68077`. Written in Stage 5 (2026-11-11). Labels as in
> [00_INDEX.md](00_INDEX.md) §2. Line numbers refer to the baseline.
>
> **Reproducible checks.** [tools/stage5_verification/](tools/stage5_verification/README.md),
> `check_iso.py` (28 checks). It drives the production `iso_ssm` functions,
> `CBFSafetyFilter._iso_configure`, the monitor's `_tick` / `_on_status` / `_on_reset` and the
> preflight script. None of them runs a node.
>
> **What this chapter does not claim.** Nothing here establishes conformity with ISO 10218-1/-2:2025,
> ISO 13855:2024 or ISO/TS 15066. Conformity needs a risk assessment, rated safety functions
> (PL d / SIL 2) and validation by a competent person. The code says the same of itself (`SAFETY.md`;
> the monitor's docstring: "NON-SAFETY-RATED"). This chapter documents what the code **computes**
> and what it **does** with the result. Statements about what the standards require are quoted
> from the code's own comments, tagged there [R]/[S]/[E]. They were not checked against the
> standards' text, which is not in the repository (**Unverified**).

---

## 1. Intuition

The CBF barrier of chapters 07–08 keeps a **distance** d ≥ d_safe + margins. It does not ask
whether the arm could still **stop** if a person kept approaching. Speed and separation monitoring
(SSM) asks exactly that. During the time it takes to notice and to brake, the person keeps
walking and the robot keeps moving; the separation must cover both, plus the sensor's blind
distance and uncertainties. If it does not, the robot must stop.

The repository implements this as an optional layer on top of Pipeline A, in four parts:

| Part | Role | Effect on motion |
|---|---|---|
| `utils/iso_ssm.py` | the four closed forms (S_p, stopping distance, speed cap, PFL speed) | none (pure functions) |
| `cbf_safety_filter` (`_iso_configure`, SSM speed rows, reduced-mode row, ISO-stop branch) | **shapes** the command: slack-relaxable speed rows; brakes when the monitor latches | yes, when `iso_enabled` (+ sub-flags) |
| `iso_safety_monitor` | a second channel that **trips** a latched, non-safety-rated stop | yes, through the filter and the commander, when `iso_monitor_enabled` |
| `iso_evidence_logger`, `iso_evidence_report`, `iso_preflight_check`, `iso_constants_measure`, `iso_pfl_speed` | record, judge, gate and measure | none, except `iso_constants_measure stop`, which moves the arm |

**Every switch ships off** (Verified: `fr3_control.yaml` L134–174; `test_iso_flags_off_identical`
pins this and the bit-identical rows). With the shipped constants the layer **cannot** be switched
on: it fails closed at construction (§4, F-5-9).

---

## 2. Mathematical foundations

### 2.1 Symbols

| Symbol | Meaning | Unit | Config key (shipped) |
|---|---|---|---|
| d | separation: the perception surface gap of a control point (`LinkDistance.distance`) | m | — |
| v_r | robot speed **toward** the person | m/s | — |
| v_h | person's approach speed | m/s | `iso_v_human` (2.0), but see §3.3 |
| T_r | reaction time (detection + processing + command) | s | `iso_t_reaction` (0.10, placeholder) |
| a_s | realised Cartesian deceleration, assumed constant | m/s² | `iso_a_stop` (1.0, placeholder) |
| T_s | stopping time, = v_r/a_s | s | — |
| C | intrusion distance (depends on the detection capability) | m | `iso_c_intrusion` (0.85) |
| Z_d, Z_r | sensor and robot position uncertainty | m | `iso_z_depth` (0.06), `iso_z_robot` (0.01) |

### 2.2 Protective separation distance

The code writes the standard's structure as (`iso_ssm.protective_separation` docstring, tagged [S]):

  (11.1)  S_p = S_h + S_r + S_s + C + Z_d + Z_r.

Assuming a constant v_h, a constant v_r during T_r, and then a constant deceleration a_s (tagged
[E] in the code):

  (11.2)  S_h = v_h (T_r + T_s),  S_r = v_r T_r,  S_s = v_r²/(2a_s),  T_s = v_r/a_s.

So the robot's own stopping distance is

  (11.3)  S_r + S_s = v_r T_r + v_r²/(2a_s)   (`iso_ssm.stopping_distance`),

and S_p is a quadratic in v_r. Both speeds are clamped at ≥ 0 (a receding party never shrinks S_p).

### 2.3 The speed cap: inverting S_p(v_r) = d (Derived)

Set S_p(v_r) = d and collect powers of v_r, with K = C + Z_d + Z_r:

  v_r²/(2a_s) + v_r (T_r + v_h/a_s) + (v_h T_r + K − d) = 0.

Multiplying by 2a_s and writing b = a_s T_r + v_h:

  v_r² + 2b v_r + 2a_s(v_h T_r + K − d) = 0,

whose non-negative root is

  (11.4)  v_cap(d, v_h) = √(b² + 2a_s(d − K − v_h T_r)) − b,  clipped to [0, v_max],

with v_cap = 0 when the discriminant is ≤ 0, and exactly 0 whenever d ≤ K, which the code tests
first so that a float epsilon is never read as "creep allowed". This matches
`iso_ssm.ssm_speed_cap` term by term. Numerically, S_p(v_cap(d)) = d to 1.3·10⁻¹⁵ over 1921 cases
with a positive cap (`check_iso.py` 1).

The **floor** K = C + Z_d + Z_r is the part of S_p needed even at zero speed:

  (11.5)  K = 0.85 + 0.06 + 0.01 = 0.92 m (shipped).

### 2.4 Power- and force-limited speed

From the code (`iso_ssm.pfl_speed`, Annex M per its docstring, [S]): with the reduced mass
μ = (1/m_H + 1/m_R)⁻¹ and m_R = M/2 + payload,

  (11.6)  v_PFL = F_max / √(μ k).

This is the energy balance ½μv² = F²/(2k) of a linear spring of stiffness k loaded up to F_max.
For the hands-and-fingers row (F = 140 N, k = 75 N/mm, m_H = 0.6 kg) and M = 17.8 kg, μ =
0.5620 kg and v_PFL = 0.682 m/s (`check_iso.py` 2).

---

## 3. Implementation

### 3.1 One implementation, three consumers (Verified)

`iso_ssm` is pure numpy, with no state. The row builder (`cbf_state_rows`), the monitor and the
evidence logger all import it, so the arithmetic cannot drift between them. What can differ is the
**inputs** each consumer feeds it (§3.3, §3.5).

### 3.2 Constants and their status (Verified, `fr3_control.yaml` L130–188)

| Key | Value | Code's tag | Status |
|---|---|---|---|
| `iso_t_reaction` | 0.10 s | [E] | **placeholder** (`iso_preflight_check.PLACEHOLDERS`) |
| `iso_a_stop` | 1.0 m/s² | [E] | **placeholder** |
| `iso_z_depth`, `iso_z_robot` | 0.06, 0.01 m | [E] | **placeholders** |
| `iso_c_intrusion` | 0.85 m | [R] | the body-detection value; lowering it needs a detection record |
| `iso_v_human` | 2.0 m/s | [S] | used only in the standoff (§3.3) |
| `iso_v_pfl` | 0.68 m/s | [E] from [S] | hands-and-fingers row (F-5-11) |
| `iso_tcp_reduced_speed` | 0.25 m/s | [S] value, [E] use | reduced mode only |
| `iso_speed_tol`, `iso_monitor_ticks`, `iso_stop_tau` | 0.05 m/s, 3, 0.05 s | [E] | monitor and stop profile |
| `iso_stop_requires_reset` | true | [E] | latch |
| `iso_distance_hold_max_s`, `iso_empty_frame_max_s` | 0.10, 0.20 s | [E] | perception fail-closed (§6.4) |
| `iso_brake_frac_min`, `iso_brake_frac_ticks` | 0.5, 10 | [E] | braking-authority diagnostic |
| `iso_tau_rate_max` | 40 N·m per 100 Hz tick | [E] | `qddot_to_torque` (09 §2, F-4-6) |

**F-5-11.** `iso_v_pfl` is the hands-and-fingers value. The same script's table gives chest 0.328,
thigh 0.349, abdomen 0.408, forearm 0.482 and upper arm 0.521 m/s, and the script itself says the
minimum over reachable regions binds (`check_iso.py` 2).

### 3.3 Which v_h enters S_h (Verified, F-5-8)

Equation (11.2) asks for the person's speed. The code feeds three different quantities:

| Consumer | v_h used | Source |
|---|---|---|
| monitor `_evaluate`, evidence logger `_evaluate` | v_app = max(n̂ᵀ v_track, 0): the **raw** tracker velocity in `LinkDistance.obstacle_velocity`, projected on the normal | perception (05 §7) |
| SSM speed rows (`spd_pts`) | v_o: the **conditioned** closing speed (median, deadband, clamp; 07 §4) | row builder |
| velocity standoff (`_iso_configure`) | `iso_v_human` = 2.0 m/s | configuration |

n̂ points from the obstacle to the robot, so n̂ᵀv_track > 0 means the obstacle approaches. An
untracked person has v_track = 0, hence v_app = 0, and the monitor is not gated on `frames_seen`.
At the shipped constants, using the measured speed instead of 2.0 m/s shrinks S_p by
v_h(T_r + T_s): by 1.20 m at v_r = 0.5 m/s. At d = 1.5 m the cap rises from 0.174 m/s
(v_h = 2.0) to 0.982 m/s (v_h = 0) (`check_iso.py` 3).

The normative v_h survives only as the **velocity standoff**. `_iso_configure` sets

  (11.7)  t_stand = T_r + v_h/a_s = 2.1 s,  standoff_max = min(v_h·t_stand, 2.0 m) = 2.0 m,

which the barrier applies as d_safe,eff = d_safe + min(t_stand·v_app, standoff_max)
(`cbf_state_rows`, L2484–2487; the formula is in the YAML comment at L1909). That standoff is
itself multiplied by the **measured** approach speed. **F-5-14:** (11.7) puts v_h in the slot where
the standard's T_s = v_r/a_s has the **robot** speed. The code comment says so ("the roadmap's [E]
parameterisation"). It is conservative only while v_r ≤ v_h, which holds when
`link_speed_max` ≤ v_PFL < v_h (Derived).

### 3.4 The SSM speed rows (Verified, `cbf_state_rows` L3478–3530)

When `iso_enabled` **and** `iso_ssm_speed_rows`, the obstacle term of each control point's
task-space speed row is replaced by the SSM cap:

  (11.8)  ‖ṗ_i‖ ≤ min( v_cap(d_i, v_o,i), link_speed_cap(d_sc,min) ),

linearised along the current travel direction v̂ = J_p q̇/‖J_p q̇‖ (`link_speed_row`). The row is
emitted only when ‖ṗ_i‖ ≥ `link_speed_activate_frac`·cap.
- It bounds the **total** point speed, not only its component toward the person, which is the
  conservative reading.
- The self-collision term keeps its own cap; min() combines the two.
- In `reduced` mode one extra row caps the TCP control point (or, failing that, the last reported
  point) at 0.25 m/s.
- The rows belong to the speed family G_SPD. **They are slack-relaxable:** with the SSM rows on,
  the family price rises from 500 to max(500, 2000) = 2000, above the obstacle family's 1000
  (`_ISO_RHO_SLACK_SPEED`, L133, L256–261). As in F-4-1, a price is not a hard bound. The code
  states this explicitly and delegates enforcement to the monitor.

### 3.5 The builder's S_p diagnostic (F-5-18, numerically reproduced)

The builder records, per rebuild, `diag_ssm_sp` = max_i `protective_separation(v_cap,i, v_o,i)`.
By (11.4), S_p(v_cap(d)) = d whenever the cap is not clipped by v_max, so the "largest S_p" is
simply the **largest unsaturated gap** (identity to 2.2·10⁻¹⁶, `check_iso.py` 9). The monitor's
S_p uses the **current closing speed**, which is the demand the scene actually makes.
`/cbf_status[5]` (`ssm_sp_min`) carries the monitor's value while its message is fresh, and the
builder's otherwise (`_publish_status` L1970–1978). So the same column changes meaning with the
monitor's liveness. Example: d = 1.2 m, closing at 0.2 m/s, v_app = 0 gives a builder value of
1.200 m and a monitor value of 0.960 m.

---

## 4. The configuration gate: `CBFSafetyFilter._iso_configure` (Verified, L543–659)

It runs once, in the constructor, before anything reads `P`. With `iso_enabled` false it
returns at once.

1. **Floor.** d_safe ≥ K = C + Z_d + Z_r, else `ValueError` with the three admissible resolutions
   (no bypass flag).
2. **Standoff.** It enables `enable_velocity_standoff` and sets (11.7); it warns when the declared
   ranges (5 s / 2 m) clip it.
3. **PFL ceiling.** `link_speed_max` ≤ `iso_v_pfl`, and `retreat_cap_max_speed` < `link_speed_max`,
   else `ValueError`. Shipped: 1.3 and 1.1 m/s. The launch's `_speed_ceiling_overrides` passes
   command-line `link_speed_max:=`/`retreat_cap_max_speed:=` to the filter and the monitor.
4. A WARN line listing what the flag changed.

**F-5-9 (Verified + reproduced).** With the shipped values, enabling the layer raises:
d_safe = 0.10 < 0.92 m. Passing needs d_safe ≥ 0.92 m, which exceeds the FR3 reach (0.855 m, the
evidence logger's manifest default), so the barrier would be larger than the workspace. This is
`SAFETY.md` deviation D1 and the "STRUCTURAL" verdict of `iso_evidence_report`. The only route the
code names is a demonstrated detection capability d ≤ 40 mm, giving C = 8(d − 14) mm
(`iso_constants_measure.c_from_detection_capability`).

**F-5-13.** Two comments around d_safe say 0.15 m (L91 "0.15 m, above"; L109 "IN SYNC WITH SIM …
carries 0.15 m and so does this file"), but the value is 0.10 m (L66). `test_iso_dsafe_floor`
repeats 0.15.

---

## 5. The independent monitor: `iso_safety_monitor` (Verified, whole file)

### 5.1 Inputs, rate, outputs

| Item | Value |
|---|---|
| Joint state | `joint_states_fast` = `/NS_1/franka/joint_states` (1 kHz), RELIABLE depth 1, mapped **by name** |
| Distances | `/cbf/per_link_distances`, BEST_EFFORT depth 1 (matches the publisher) |
| Chain status | `/NS_1/cbf_status`, depth 1: only `data[2]` (`fault_braking`) is read |
| Kinematics | its own `CBFKinematics` on the hand-less URDF, with the same control points |
| Rate | `qp_rate_hz` = 300 Hz timer |
| Output | `/NS_1/iso_safety` = [latched, reason, S_p, v_cap_min, v_closing_max], **every tick** |
| Service | `/NS_1/safety_reset` (`std_srvs/Trigger`) |

### 5.2 Trip conditions and the latch

Per valid control point i, with n̂ from the obstacle to the point:

  (11.9)  v_closing,i = max(−n̂ᵀ J_p,i q̇, 0),  cap_i = v_cap(d_i, v_app,i) (with v_max = `link_speed_max`).

In reduced mode the TCP link's cap is also min'd with 0.25 m/s. The live reason is:
1. SPEED, when v_closing,i > cap_i + `iso_speed_tol` for some i;
2. INSIDE, when d_i < K (checked first; no Jacobian needed);
3. FAULT, when the last `cbf_status[2]` ≥ 1;
4. STALE, when q, q̇ or the distances are missing or older than `joint_state_timeout` (0.1 s) or
   `distance_timeout` (0.5 s).

Each reason keeps its own streak, and a trip needs
`trip_ticks` = round(`iso_monitor_ticks`·qp_rate_hz/100) = **9** consecutive ticks (30 ms)
(`check_iso.py` 5). On a trip the latch is set. `_on_reset` refuses while any reason is live. With
`iso_stop_requires_reset: false`, the latch clears by itself once nothing is live.

**What the latch does** (Verified):
- **filter** (`_qp_tick` STEP 0, L1071–1088, ahead of every other branch): all rows are skipped,
  and it brakes on q̈ = clip(−q̇/`iso_stop_tau`, box) through `box_only_solve`, publishing
  `fault = 1`. It also brakes when the monitor's topic is stale or has never arrived for more than
  `distance_timeout`, measured from filter construction (`_iso_stop_active`).
- **commander** (`pentagon_qddot_commander._iso_safety_cb`): q̈_nom = 0 and the phase is held. It
  does not fail closed on a missing monitor; the filter does.

### 5.3 The latch cannot be released (F-5-17), and the liveness model (F-5-19)

Combine the two halves of §5.2.
1. The latch sets → the filter publishes `fault_braking = 1`.
2. The monitor sees FAULT live.
3. `_on_reset` refuses, and auto-resume needs nothing live.
4. The filter keeps braking because the latch is set.

The cycle is closed. In `check_iso.py` 8, with the production monitor methods and the filter's
verified reaction transcribed, the latch set at tick 9 is still set 10 s after the scene cleared,
and `/safety_reset` answers "REFUSED: safety-chain fault on cbf_status is still true". This holds
with `iso_stop_requires_reset` true **and** false. The same loop makes **any** filter fault lasting
9 ticks a permanent stop: stale perception, a frozen joint state, an empty-frame run, and even
`_brake_authority_fault`, which its docstring calls "Diagnostic only — no stop" but which sets
`fault = 1.0` (L1504). Only restarting a node clears it.

`test_iso_monitor` tests the monitor alone; `test_a_chain_fault_also_blocks_the_reset` pins the
refusal. No test closes the loop. Everything here is dormant as shipped (both flags off). Runtime
behaviour is **Unverified**.

**F-5-19.**
- (a) Missing inputs at start latch STALE after 30 ms (`check_iso.py` 10).
- (b) The FAULT input has no timestamp. A filter that stops publishing is invisible to the monitor,
  and the last flag is kept forever.
- (c) *Inferred:* the launch starts the filter at 2 s and the monitor at 3 s (`launch_defaults`
  L140, L293). The filter therefore brakes with fault = 1 before the first `/iso_safety` arrives.
  Whether the boot falls into F-5-17 depends on two things: whether the filter un-brakes on the
  monitor's first unlatched message within 9 monitor ticks, and whether perception is already
  publishing.

---

## 6. Gate, evidence and measurement tools

### 6.1 `scripts/iso_preflight_check.py` (Verified, whole file)

With `iso_enabled` true in the YAML it read, it fails on:
- a placeholder constant (the four in §3.2);
- C < 0.85 m without a reliable detection record whose d gives that C;
- d_safe < K;
- `link_speed_max` > v_PFL, or a broken ordering.

With `--wait` it also requires `/NS_1/iso_safety` on the wire. With `iso_enabled` false it prints
one line and exits 0.

**F-5-10 (a), F-5-20.** `torque_control_stack` runs it at t = 0, before the spawner, with only
`--config <share>/config/fr3_control.yaml` (L539–556). It never sees a command-line
`iso_enabled:=true` and exits 0 ("nothing to check", `check_iso.py` 6). It is never given
`--wait`. The filter's `_iso_configure` repeats the floor and PFL checks but **not** the
placeholder check, so a command-line-enabled layer runs on placeholder T_r, a_s, Z_d and Z_r.

### 6.2 `iso_evidence_logger` (Verified: constructor, `_tick`, `_evaluate`)

A passive observer, started by `start_iso_evidence_logger` (delay 3 s). It evaluates the ISO
criteria **with the YAML constants, whether or not the layer is on**, through a second copy of the
monitor's arithmetic. It writes `iso_manifest.json` (the whole `iso_*` block, limits, git SHA),
`iso_evidence.csv` (100 Hz: TCP speed and its running max from the 1 kHz joint state, joint-speed
ratio, d_min, margin d − S_p, caps, closing speed, the stack's `/cbf_status` and `/iso_safety`,
brake fraction, torque saturation) and the sparse `iso_events.csv`.
- A channel that never published stays NaN, so "absent" is distinguishable from "zero"; the code
  cites a measured false PASS this prevents.
- **F-5-10 (b):** the launch passes it no `iso_*` flags and no speed overrides, so its manifest and
  its reduced-mode and v_max choices reflect the YAML, not the run.
- With C = 0.85 m every point within 0.92 m counts as "inside the floor", which is every point in
  the FR3 workspace (D1).

### 6.3 `scripts/iso_evidence_report.py` (Verified: D1, separation, cross-check, check list)

Thirteen checks, each with the verdict PASS / FAIL / INCONCLUSIVE / NOT FROM LOGS, or STRUCTURAL
for D1 (floor > reach).
- No PASS is given on less than 50 % coverage.
- Under D1 the separation check also reports the margin "excluding C" (margin + C), which is the
  part the controller governs.
- The cross-check FAILs when the logger's d_min and the filter's `cbf_d_min` differ by ≥ 2 cm.
- The script states that no verdict means "compliant" (`test_no_verdict_word_means_compliant`).

### 6.4 Perception and torque hooks (Verified at the call sites; detail in 05 and 09)

- `iso_distance_hold_max_s` (0.10 s): a held per-point distance older than this is published
  invalid by `real_time_distance`. **F-5-12:** it is read with no `iso_enabled` check.
- `iso_empty_frame_max_s` (0.20 s): with `iso_enabled`, a run of empty frames after a close frame
  brakes with fault = 1 (`_empty_frame_fault`, L1134–1141).
- `iso_brake_frac_*`: with `iso_enabled` and an active obstacle row, a realised/commanded
  acceleration projection below 0.5 for 10 (×3 at 300 Hz) ticks sets fault = 1. It is called
  diagnostic only, but see F-5-17.
- `qddot_to_torque`: with `iso_enabled`, an effort clip and a 40 N·m-per-100 Hz-tick rate limit
  (09 §2, F-4-6). Off as shipped.

### 6.5 Measurement and calculation scripts (Verified: docstrings and named functions)

- `iso_pfl_speed.py` computes (11.6) from a convenience table (the docstring warns that the
  values are not a source).
- `iso_constants_measure.py` has five sub-commands: `reaction` (wraps `latency_budget.py`),
  `stop`, `detection`, `uncertainty`, `yaml`.
  - **`stop` moves the arm**, and refuses without `--i-am-supervising`. It drives each joint to a
    fraction of q̇_max, commands the stop profile clip(−q̇/τ, ±decel_max), and reports
    a_s = v_tcp/T_s per run. `iso_a_stop` is the **minimum** over the grid.
  - *Derived caveat:* with a_s = v/T_s, S_s = v²/(2a_s) = vT_s/2. That equals the measured travel
    only for a linear speed decay. It over-estimates the travel for the convex (exponential)
    profile commanded here, and under-estimates it when T_s includes a dead time at the start. The
    script prints the longest measured S_s for the cross-check that `iso_ssm`'s docstring asks for.

---

## 7. Practical example

Use the shipped constants (T_r 0.10 s, a_s 1.0 m/s², K 0.92 m). A control point has d = 1.2 m and
closes at 0.2 m/s on a person the tracker has not yet picked up (`check_iso.py` 11).

| Quantity | v_h = v_app = 0 (what the monitor uses) | v_h = 2.0 m/s (normative) |
|---|---|---|
| S_p at v_r = 0.2 m/s (11.1) | 0.02 + 0.02 + 0.92 = **0.960 m** | + 2.0·(0.1 + 0.2) = **1.560 m** |
| margin d − S_p | +0.240 m (no trip) | −0.360 m (separation breached) |
| v_cap(1.2) (11.4) | √0.57 − 0.1 = **0.655 m/s** | √4.57 − 2.1 = **0.038 m/s** |
| SPEED trip? (0.2 > cap + 0.05) | no | yes, after 9 ticks |

With the layer actually enabled, d_safe would have to be ≥ 0.92 m (§4), so the barrier itself
already keeps the arm away from this scene. The stopping distance at the PFL ceiling,
0.68·0.1 + 0.68²/2 = 0.299 m, is what a measured a_s would have to confirm.

---

## 8. Monitoring versus enforcement

| Mechanism | Kind | When active | Enforced how |
|---|---|---|---|
| CBF barrier (07–08) | shaping (soft rows, F-4-1) | always | QP, slack-relaxable |
| SSM speed rows, reduced-mode row | shaping | `iso_enabled` + `iso_ssm_speed_rows` / `iso_mode: reduced` | QP, slack at price 2000 |
| standoff (11.7) | shaping | `iso_enabled` | adds to the barrier's d_safe |
| PFL ceiling, d_safe floor | configuration gate | `iso_enabled` | `ValueError` at construction |
| monitor latch | **stop** (non-safety-rated) | `iso_enabled` + `iso_monitor_enabled` | filter brakes q̈ = −q̇/τ in the box; commander zeroes; F-5-17 |
| empty-frame fault, stale monitor | brake | `iso_enabled` (+ monitor) | filter fallback |
| brake authority, torque saturation | diagnostic (`/cbf_status`, `/torque_saturation`) | `iso_enabled` | none, except through F-5-17 |
| evidence logger and report | observation | `start_iso_evidence_logger` | none |
| preflight | launch gate | `iso_enabled` in the **YAML** | launch `Shutdown` |

The rated layer underneath (Franka's own safety functions, `config/safety/watchman_profile.md`) is
configured by a person in Desk; "nothing reads" that file. Its behaviour is **Unverified** here.

---

## 9. Consequences of modification

| Change | Effect |
|---|---|
| Any `iso_ssm` formula | all three consumers move together. Pinned by `test_iso_ssm`, `test_iso_speed_rows`, `test_iso_monitor`, `check_iso.py` 1, 11 |
| Feeding `iso_v_human` into S_h (F-5-8) | caps shrink sharply (§7); the arm may not be able to move at all near a person |
| Lowering C below 0.85 | needs a detection record (preflight); changes the floor, the gate and every report verdict |
| Breaking F-5-17 (e.g. excluding the ISO-stop branch's own fault from the monitor's FAULT, or a separate field) | the reset becomes possible. The status schema is append-only (`cbf_diagnostics`), so add a field rather than re-purpose `data[2]` |
| A timestamp on the monitor's `cbf_status` input | turns F-5-19 (b) into a STALE trip |
| Passing the run's flags to the preflight and the evidence logger | closes F-5-10 / F-5-20 |
| Flags | `test_iso_flags_off_identical` requires every switch to ship off and the rows to stay bit-identical with the flags off |
| Test gaps | no test imports `iso_evidence_logger` or `iso_preflight_check` (Verified, grep of `test/`): the evidence logger node and the launch wiring of the preflight (F-5-10) are untested. `watchman_profile.md` is referenced only by a YAML comment and a test comment, never read by code (Verified) |
| `/cbf_status[5..8]` | consumers index positionally (`frame_grabber`, `rl_policy_commander`, `experiment_logger`, the commander); F-5-18 means a consumer must also know whether the monitor was fresh |

## 10. Findings from this chapter

| ID | Short title | Section | Reproduced by | Still needs |
|---|---|---|---|---|
| F-5-8 | S_h uses the measured approach speed, not v_h | §3.3 | `check_iso.py` 3, 7 | — |
| F-5-9 | the layer cannot be enabled in this cell (floor > reach) | §4 | `check_iso.py` 4 | — |
| F-5-10 | preflight sees only the YAML; evidence logger gets no flags | §6.1–6.2 | `check_iso.py` 6 | — |
| F-5-11 | v_PFL is the hands-and-fingers value, not the binding region | §3.2 | `check_iso.py` 2 | — |
| F-5-12 | `iso_distance_hold_max_s` acts without `iso_enabled` | §6.4 | source | — |
| F-5-13 | comments say d_safe 0.15, value 0.10 | §4 | source | — |
| F-5-14 | v_h in the T_s slot of the standoff | §3.3 | `check_iso.py` 4 | — |
| F-5-17 | the monitor latch can never be released while the filter brakes | §5.3 | `check_iso.py` 8 | runtime |
| F-5-18 | two meanings of S_p on `/cbf_status[5]` | §3.5 | `check_iso.py` 9 | — |
| F-5-19 | monitor liveness: STALE at start, unstamped fault, boot race | §5.3 | `check_iso.py` 10 | runtime (c) |
| F-5-20 | placeholder gate only in the preflight | §6.1 | source | — |
