> **Preserved draft (not the chapter).** A second Claude session wrote this version of chapter 11
> in parallel and saved it over `11_iso_safety_layer.md` at 16:24:56 on 2026-11-11. On the user's
> instruction (reconciliation, C-035) the chapter was restored to the version that the findings
> F-5-17…F-5-20, `check_iso.py` and chapter 13 cite, and this text was kept here unchanged, apart
> from the relative link paths (one directory deeper). Its "F-5-16" means the finding renumbered
> **F-5-24** in PROGRESS §4. Its verified content is either in the chapter or listed in C-035.

# 11 — The ISO-inspired safety layer: speed and separation monitoring, PFL bounds, monitor and evidence

> **Source baseline:** `ba68077`. Written in Stage 5 (2026-11-11). Labels as in
> [00_INDEX.md](../00_INDEX.md) §2. Line numbers refer to the baseline.
>
> **What this chapter is not.** It describes code that implements selected *concepts* from
> ISO 10218-1/-2:2025 (whose Annexes L and M absorb ISO/TS 15066), ISO 13855:2024 and
> ISO 13849-1:2023, as the code itself cites them. Reading source code cannot establish
> conformity, and the repository explicitly disclaims it. `SAFETY.md`: "It is **not a compliant
> system**, it is **not certified**". `fr3_control.yaml`: "NOTHING HERE IS A CERTIFIED SAFETY
> FUNCTION." The monitor docstring: "a **non-safety-rated stop**". No statement below should be read
> as a compliance claim.
>
> **Reproducible checks.** [tools/stage5_verification/](../tools/stage5_verification/README.md)
> `check_iso.py` (the production `iso_ssm`, `_iso_configure`, `trip_ticks` and the preflight script).
> §5–§6 also use a read-only subagent inventory, whose load-bearing claims were spot-checked in the
> source and are labelled accordingly.

---

## 1. Intuition

ISO speed and separation monitoring (SSM) asks one question per robot–human pair: **if a stop were
triggered now, would the robot come to rest before the human, still approaching at their
assumed speed, reached it?** The answer is a minimum distance, the protective separation distance
S_p. Inside it the robot must stop, not merely slow down. Power and force limiting (PFL) asks a
different question: if contact does happen, is the transferred energy within body-region limits?
It yields a maximum speed v_PFL.

The repository wires these ideas around the CBF filter in three places:
1. **inside the QP**, as SSM speed caps on the task-space speed rows (soft);
2. **beside the QP**, as an independent monitor that latches a stop, executed by the filter
   (hard brake) and the commander (zero nominal, phase held);
3. **after the fact**, as an evidence logger and an offline report.

Everything is **off by default**, and with the shipped constants it cannot be switched on in this
cell (§4).

---

## 2. Mathematics (`utils/iso_ssm.py`, Verified and numerically checked)

Symbols (the YAML key in brackets, shipped value): v_r is the robot's closing speed toward the
human; v_h the human's approach speed; T_r the reaction time (`iso_t_reaction`, 0.10 s); a_s the
robot's stopping deceleration (`iso_a_stop`, 1.0 m/s²); C the intrusion distance (`iso_c_intrusion`,
0.85 m); Z_d the position uncertainty of the sensor (`iso_z_depth`, 0.06 m); Z_r the robot's
(`iso_z_robot`, 0.01 m). All values in m, s, m/s, m/s².

**Stopping distance** (constant v_r during T_r, then constant deceleration a_s):

  (11.1)  S_r + S_s = v_r T_r + v_r²/(2a_s),  T_s = v_r/a_s.

**Protective separation** (the Annex L structure S_p = S_h + S_r + S_s + C + Z_d + Z_r, with the
human moving at v_h during T_r + T_s):

  (11.2)  S_p(v_r, v_h) = v_h(T_r + v_r/a_s) + v_r T_r + v_r²/(2a_s) + C + Z_d + Z_r.

**Speed cap: the inversion in v_r.** Set S_p = d and solve the quadratic:

  v_r²/(2a_s) + v_r(T_r + v_h/a_s) + (v_h T_r + C + Z_d + Z_r − d) = 0
  ⇒ (11.3)  v_cap(d, v_h) = −b + √(b² + 2a_s(d − C − Z_d − Z_r − v_h T_r)),  b = a_s T_r + v_h,

clipped to [0, v_max], and 0 when d ≤ C + Z_d + Z_r. The positive root is the only physical one.
**Numerically verified:** S_p(v_cap(d, v_h), v_h) = d to 1.3e-15 m on 1921 random cases, and the cap is
0 at and inside the floor (`check_iso.py` 1).

**PFL speed** (transient contact energy ½μv² equal to the elastic energy F²/(2k)):

  (11.4)  v_PFL = F_max / √(μ k),  μ = (1/m_H + 1/m_R)⁻¹,

with m_R = M_robot/2 + payload in `iso_pfl_speed.py`. For hands and fingers (F = 140 N,
k = 75 N/mm, m_H = 0.6 kg, m_R = 8.9 kg), v_PFL = 0.682 m/s, which reproduces `iso_v_pfl` = 0.68
(`check_iso.py` 2).

**Verification status.** The code's own tags mark the structure of (11.2) as normative [S] and the
closed forms (constant speeds, constant deceleration) as engineering assumptions [E]. The repository
does not measure a_s, T_r or Z_d; the shipped values are placeholders that the preflight is meant to
reject (§5). So the numbers above are as good as their inputs, which are **unverified**.

---

## 3. Where the mathematics enters the control system

### 3.1 SSM speed rows in the QP (`iso_enabled` ∧ `iso_ssm_speed_rows`; both off)

`ConstraintBuilder.build` (L3505–3535) replaces the task-space speed cap of each obstacle control
point (08 (8.15)) with

  v_allow = min( v_cap(clear_i, v_app,i; v_max = `link_speed_max`), link_speed_cap(d_sc) ),

where clear_i is the point's measured gap and v_app,i the same conditioned closing speed v_o the
barrier uses (07 §3.4). The speed-family slack weight is raised to at least `_ISO_RHO_SLACK_SPEED`
= 2000 (above ρ_obs = 1000). The rows stay **soft** (08 §3). `iso_mode: reduced` adds one TCP row
capped at `iso_tcp_reduced_speed` = 0.25 m/s.

### 3.2 The independent monitor (`iso_monitor_enabled`, off)

`iso_safety_monitor` runs at `qp_rate_hz` with its own kinematics. For every valid entry it
computes:

  (11.5)  v_closing = max(−n̂ᵀJ_p q̇, 0),  v_app = max(n̂ᵀv_track, 0),
          v_cap = v_cap(d, v_app), S_p = S_p(v_closing, v_app),

with n̂ the perception direction (obstacle → robot). It **trips** when a condition holds for
`trip_ticks` = round(3·qp_rate_hz/100) = **9 ticks (30 ms)** in a row:
1. v_closing > v_cap + 0.05 m/s (speed);
2. d < C + Z_d + Z_r (inside the floor);
3. `cbf_status[2]` = 1 (a chain fault);
4. stale or missing inputs.

The latch is held until `/NS_1/safety_reset`, which is refused while a condition is still live
(`iso_stop_requires_reset: true`). The monitor publishes
`/NS_1/iso_safety` = [latched, reason, S_p of the most binding point, v_cap_min, v_closing_max] every
tick. **It sends no motion command.** (Verified.)

### 3.3 The filter's own ISO behaviour (`iso_enabled`, off)

`_iso_configure` (L543–659, Verified):
- refuses to start unless d_safe ≥ C + Z_d + Z_r;
- refuses unless `link_speed_max` ≤ `iso_v_pfl` and `retreat_cap_max_speed` < `link_speed_max`;
- forces the velocity standoff on with

    (11.6)  t_stand = T_r + v_h/a_s (capped at 5 s),  standoff_max = min(v_h·t_stand, 2 m).

  This is the only place `iso_v_human` is used.

At run time (the `_qp_tick` rows of 08 §5.1):
- **ISO stop latched** (requires `iso_monitor_enabled`): a box-only QP with nominal
  clip(−q̇/`iso_stop_tau`, box), τ = 0.05 s; the CBF rows are skipped; fault = 1. A monitor message
  older than 0.5 s also brakes.
- **Empty-frame fault:** if perception keeps heartbeating empty frames for more than 0.2 s after an
  obstacle was within the active zone, the filter brakes.
- **Braking-authority diagnostic:** realised/commanded acceleration < 0.5 for 10·(rate/100) ticks
  while rows are violated logs an error and raises the fault flag, **without** stopping.
- `qddot_to_torque` clips τ to effort_max and rate-limits it (09 §2, F-4-6).

### 3.4 Deviations from the standard's assumptions (Derived / Verified)

- **F-5-8: S_h uses the measured speed.** The monitor, the QP SSM rows and the evidence logger all
  pass v_app, the tracked approach speed (0 for an untracked person, and not gated on
  `frames_seen`), as v_h. The YAML comment explains that 2.0 m/s is "the applicable value here". At
  the shipped constants:
  - S_p(v_r = 0.5) is 1.095 m with v_h = 0 and 2.295 m with v_h = 2.0;
  - the cap at d = 1.5 m is 0.982 m/s with v_h = 0 and 0.174 m/s with v_h = 2.0
    (`check_iso.py` 3).
- **F-5-14: the standoff's stopping time.** (11.6) uses v_h/a_s where the standard's T_s is the
  robot's v_r/a_s. It is conservative while v_r ≤ v_h, which the speed gate guarantees
  (`link_speed_max` ≤ v_PFL = 0.68 < 2.0).
- **F-5-11: which PFL value.** 0.68 m/s is the hands-and-fingers value. In the same script's table
  the chest gives 0.328 m/s, the thigh 0.349, the abdomen 0.408, the forearm 0.482 and the upper arm
  0.521 (`check_iso.py` 2). The script itself says the minimum over reachable regions binds.

---

## 4. Can it be enabled? (F-5-9, Verified + numerically reproduced)

C = 0.85 m is the body-detection value (no detection-capability record exists), so the floor is
C + Z_d + Z_r = **0.92 m**, larger than the FR3's reach (0.855 m, the evidence-logger manifest).
With the shipped d_safe = 0.10 m, `iso_enabled:=true` makes `_iso_configure` raise `ValueError`:
the filter fails **closed**, by design ("there is no bypass flag"). With d_safe = 0.95,
`link_speed_max` 0.68 and `retreat_cap_max_speed` 0.60 it passes and sets t_stand = 2.1 s and
standoff_max = 2.0 m (`check_iso.py` 4). In other words, the gate can pass only with a barrier
wider than the arm's workspace. `iso_evidence_report` calls this deviation "D1, STRUCTURAL". The
YAML's resolution (c) keeps `iso_enabled` false. Its comment calls d_safe "0.15 m, above"; the
value is 0.10 (F-5-13).

---

## 5. Evidence, preflight and measurement tools

*(Subagent inventory, read-only; items marked V were re-checked in the source.)*

| Tool | Role | Default | Notes |
|---|---|---|---|
| `nodes/iso_evidence_logger.py` | passive recorder: per-tick TCP speed, joint-speed ratio, perception age, brake fraction, τ saturation, and its own SSM margin, cap and excess per control point; edge events; a manifest with git SHA, the ISO parameters, limits and a disclaimer | `start_iso_evidence_logger: false` | not passed `iso_enabled` / `iso_mode` / `iso_ssm_speed_rows` / `iso_monitor_enabled` (V, launch L833–850), so its manifest and reduced-mode checks reflect the YAML (**F-5-10b**) |
| `scripts/iso_preflight_check.py` | aborts the launch on placeholder constants, C < 0.85 without a detection record, d_safe < floor, or the speed gates | run by the torque launch when `iso_enabled` | given only `--config <share>/fr3_control.yaml` (V, L536–556), it sees the YAML's `iso_enabled: false` and exits 0 (V, `check_iso.py` 6). Command-line `iso_enabled:=true` and the speed overrides are never checked (**F-5-10a**); the filter's gate (§4) is what actually stops the run |
| `scripts/iso_constants_measure.py` | measures T_r (from `latency_budget`), a_s (a supervised stop test, Annex H grid), C (detection capability), Z_d (calibration residual p95); prints a YAML block | — | the `stop` subcommand moves the arm through the filter; the `uncertainty` topic default is `/cams/d455/...` (stale, F-5-18, V) |
| `scripts/iso_pfl_speed.py` | (11.4) with an Annex M table copy ("DO NOT CITE THIS DICT") | — | prints "THIS IS NOT PFL COMPLIANCE" |
| `scripts/iso_evidence_report.py` | verdicts PASS / FAIL / INCONCLUSIVE / NOT FROM LOGS / STRUCTURAL over an evidence run; a list of questions that logs cannot answer (PL d / SIL 2, PFL biomechanics, risk assessment) | — | "cannot tell you whether the cell is ISO compliant" |
| `config/safety/watchman_profile.md` | a planned profile for Franka's rated safety functions (SLS-J, SLD, stop categories), all blank | — | **read by no code** (V, grep) |

---

## 6. Classification

| Mechanism | Implemented | Enabled by default | Effect | Needs runtime / hardware validation |
|---|---|---|---|---|
| SSM math (`iso_ssm`) | yes | used only when ISO is on (also by the offline tools) | — | inputs (T_r, a_s, C, Z_d) unmeasured |
| SSM speed rows in the QP | yes | **no** | constrains (soft rows) | the effect of slack (F-4-1/2) |
| Reduced-speed TCP row | yes | no | constrains (soft) | — |
| `_iso_configure` gate + standoff | yes | no (fails closed if enabled as shipped) | blocks startup; tightens h̄ | — |
| Independent monitor + latch | yes | **no** | stop decision (non-safety-rated) | trip timing, DDS loss |
| Filter ISO stop (box-only brake) | yes | no (needs the monitor) | actively brakes | stopping distance a_s |
| Empty-frame fault | yes | no (`iso_enabled`) | actively brakes | — |
| Distance hold expiry (`iso_distance_hold_max_s`) | yes | **yes**, regardless of `iso_enabled` (**F-5-12**) | invalidates held entries after 0.1 s (and creates F-2-11 during it) | — |
| Braking-authority check | yes | no | fault flag and log only | — |
| τ clip / rate limit in `qddot_to_torque` | yes | no | changes τ | — |
| Evidence logger / report | yes | no | logs / offline only | — |
| Preflight | yes | runs only with `iso_enabled` | aborts the launch, but blind to command-line overrides | — |
| Watchman profile (rated functions) | document only | — | none in code | hardware configuration |

## 7. Interaction with the rest of the framework (Derived)

- **QP.** SSM rows are soft and priced at ρ_spd ≥ 2000. Under barrier pressure the QP can still
  exceed them (F-4-1/F-4-2), which is exactly why the monitor docstring argues for an independent
  stop channel.
- **Velocity box.** `link_speed_max` ≤ v_PFL is a Cartesian per-point speed bound on task-space
  rows. The joint velocity box (0.7·q̇_max, 08 §2.4.2) is separate and unchanged.
- **Torque.** With ISO on, `qddot_to_torque` clips at effort_max. `RtTorqueController` still has no
  clamp (F-4-7).
- **Emergency handling.** The ISO stop is a filter brake through the box (−q̇/0.05 s, clipped to the
  acceleration box), and the arm's actual deceleration is what `iso_a_stop` must bound. It is not a
  Category 0/1 stop of the robot's rated safety functions (watchman profile, unconfigured).
- **Status.** `/cbf_status`[5..8] carries the monitor's S_p, v_cap, v_closing and latch when fresh,
  else the filter's own values. The experiment logger reads those fields, but not the rest of the
  tail (F-5-16).

## 8. Worked example (`check_iso.py`)

Shipped constants, a control point 1.5 m from a person, the robot closing at 0.5 m/s:
- **Normative v_h = 2.0:** S_p = 2.0·(0.1 + 0.5) + 0.05 + 0.125 + 0.92 = **2.295 m** > 1.5 m, so a
  real SSM implementation would stop. The cap at 1.5 m is 0.174 m/s.
- **As implemented (untracked person, v_app = 0):** S_p = 1.095 m < 1.5 m: no trip. The cap is
  0.982 m/s.
- **In this cell, today:** nothing happens either way; the layer is off, and switching it on fails
  at startup (§4).

## 9. Consequences of modification

| Change | Effect |
|---|---|
| Measuring and setting T_r, a_s, C, Z_d | every S_p and cap; the preflight's placeholder check passes |
| Using `iso_v_human` in S_h (fixing F-5-8) | S_p grows by v_h(T_r + T_s) (~1.2 m at 0.5 m/s); with C = 0.85 the cell becomes unusable for SSM |
| Passing the launch flags to the preflight and the logger (F-5-10) | the preflight checks what actually runs; manifests become truthful |
| Lowering `iso_v_pfl` to the binding region | `link_speed_max` must follow (the gate) |
| Gating `iso_distance_hold_max_s` on `iso_enabled` | changes perception (F-2-11 exposure) with the layer off |
| Tests | `test_iso_ssm`, `test_iso_dsafe_floor`, `test_iso_flags_off_identical`, `test_iso_speed_rows`, `test_iso_monitor`, `test_iso_perception_failclosed`, `test_iso_pfl_ceiling`, `test_iso_status_contract`, `test_iso_torque_limits`, `test_iso_evidence_report` (contracts listed by the subagent; not run). No test covers the evidence logger node, the preflight wiring or the launch flags (subagent; Inferred). |

## 10. Findings from this chapter

| ID | Short title | Section | Reproduced by | Still needs |
|---|---|---|---|---|
| F-5-8 | S_h uses the measured approach speed instead of the normative v_h | §3.4 | `check_iso.py` 3, 7 | — |
| F-5-9 | the floor (0.92 m) exceeds the FR3 reach; ISO cannot be enabled as shipped | §4 | `check_iso.py` 4 | detection-capability measurement |
| F-5-10 | the preflight is blind to launch overrides; the logger lacks the flags | §5 | `check_iso.py` 6 + launch source | — |
| F-5-11 | `iso_v_pfl` is the hands value; the chest gives 0.328 m/s | §3.4 | `check_iso.py` 2 | risk assessment |
| F-5-12 | the distance hold is active without the ISO flag | §6 | code | — |
| F-5-13 | the YAML comment's d_safe 0.15 vs 0.10 | §4 | code | — |
| F-5-14 | the standoff uses v_h/a_s as the stopping time | §3.4 | `check_iso.py` 4 | — |
