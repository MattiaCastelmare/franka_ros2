# SAFETY.md — what this package claims, and what it does not

**Read this before turning `iso_enabled` on.**

This package implements a set of **ISO-alignment measures** on a research cell.
It is **not a compliant system**, it is **not certified**, and nothing in it is
a **safety function** in the sense of ISO 10218 / ISO 13849-1. Every claim below
carries a status word, and only three are used:

| status | meaning |
|---|---|
| **implemented** | the code exists, is unit-tested, and does what this table says |
| **validated on hardware** | the above, plus measured on the FR3 in this cell |
| **certified** | assessed by a notified body against a standard |

**Nothing in this package is `certified`.** Nothing is `validated on hardware`
yet either: the ISO layer ships with every flag `false` and has not been run on
the robot.

## Why this can never be a safety function, however good the code gets

- The CBF chain is **single-channel Python over best-effort DDS**. No redundancy,
  no diagnostic coverage, no proof test, no defined failure rate.
- ISO 10218-1:2025 (5.1.17, Annex C) makes the FR3 a **Class II** robot
  (manipulator mass 17.8 kg > 10 kg). The safety functions required of a Class II
  robot must reach **PL d / SIL 2** (ISO 13849-1:2023 / IEC 62061:2021) **[R]**.
  A Python node on a general-purpose kernel does not reach PL d, and adding tests
  does not change that.
- The FR3's own rated functions (PL d / Cat 3) are certified against
  **EN ISO 10218-1:2011**, not against the 2025 editions. Re-assessment is the
  integrator's responsibility and has not been done here.

Terminology follows the 2025 editions: **collaborative application** /
**collaborative task** (*collaborative robot* and *collaborative operation* are
deprecated), **protected space**, **separation distance**, **protective stop**,
**monitored standstill**, **quasi-static** / **transient contact**. The word
*protective stop* is reserved for a rated function and does **not** appear in
this package's logs, topics or documentation — what `iso_safety_monitor`
implements is a **non-safety-rated stop**.

Tags used throughout: **[R]** required by the standard, **[S]** value or formula
from the standard, **[E]** engineering assumption made here.

---

## 1. Requirement table

| Requirement (clause) | Tag | Where implemented | Status | Residual gap |
|---|---|---|---|---|
| SSM separation distance `S_p = S_h + S_r + S_s + C + Z_d + Z_r` (ISO 10218-2:2025 5.14.5, **Annex L**) | [S] structure, [E] closed form | `utils/iso_ssm.py::protective_separation` | implemented | Closed form assumes constant `v_h`, constant `v_r` during `T_r`, constant `a_s` during `T_s` — not the standard's integrals |
| `S ≥ S_p` enforced on the robot's speed | [R] | `utils/cbf_state_rows.py` speed rows via `iso_ssm.ssm_speed_cap`; `nodes/iso_safety_monitor.py` | implemented | Rows are **slack-relaxable**; enforcement rests on a non-rated monitor |
| Robot **stops** below `S_p` (ISO 10218-2:2025 5.14.5) | [R] | `nodes/iso_safety_monitor.py` → `cbf_safety_filter._qp_tick` STEP 0 + `pentagon_qddot_commander._tick` | implemented | Non-safety-rated stop, single channel, best-effort transport |
| Intrusion distance `C` from detection capability `d` (ISO 13855:2024) | [R] | `params.iso_c_intrusion`; derived by `scripts/iso_constants_measure.py detection` | implemented (measurement **not done**) | See deviation D1 — the value in use is the body-detection 0.85 m and no `d` has been demonstrated |
| Reaction time `T_r` | [E] | `params.iso_t_reaction`; `scripts/iso_constants_measure.py reaction` | implemented (**placeholder value**) | Not measured on this cell |
| Stopping time / distance, Annex H grid (ISO 10218-1:2025 5.5.6, 5.5.7, **Annex H**) | [S] procedure, [E] values | `scripts/iso_constants_measure.py stop`; `params.iso_a_stop` | implemented (**placeholder value**) | Only a Category 2 stop is reproducible in software; Cat 0/1 come from the product manual. Not a rated stopping-time-limiting function |
| Barrier offset = irreducible `C + Z_d + Z_r` | [E] split of an [R] requirement | `cbf_safety_filter._iso_configure` | implemented | See deviation D2 — the floor is unreachable in this cell |
| `S_h = v_h·(T_r + T_s)` | [S] structure, [E] parameterisation | existing `enable_velocity_standoff`, re-parameterised in `_iso_configure` | implemented | Clamped to `velocity_standoff_max` (2.0 m spec ceiling); see gap G4 |
| Human approach speed `v_h` (ISO 13855:2024) | [S] | `params.iso_v_human = 2.0` | implemented | 2.0 m/s is the reach-into value; lowering it needs a validated directed-speed measurement this cell does not have |
| PFL speed from biomechanical limits (ISO 10218-2:2025 5.14.6, **Annex M**) | [S] formula, [E] region choice | `utils/iso_ssm.py::pfl_speed`; `scripts/iso_pfl_speed.py`; `params.iso_v_pfl` | implemented | See deviation D3 — calculated, not measured with a PFMD |
| PFL ceiling actually enforced | [E] | `cbf_safety_filter._iso_configure` raises if `link_speed_max > iso_v_pfl` | implemented | Refuses to start rather than clamping; today's config trips it (see G2) |
| Reduced speed ≤ 250 mm/s (ISO 10218-1:2025 5.5.3 / -2:2025 5.5.6) | [S] value, [E] application | extra `G_SPD` TCP row in `cbf_state_rows.build`; same cap in `iso_safety_monitor._evaluate` | implemented | See deviation D4 — a cell-wide derate, not a rated speed-monitoring function |
| Start/restart interlock and reset (ISO 10218-1:2025 5.5.2) | [E] | `iso_safety_monitor._on_reset` (`/NS_1/safety_reset`) | implemented | Only the "condition must be gone first" half is enforceable in software; the deliberate-separate-action half is a service call, not a rated reset device |
| Fault behaviour of the perception channel (ISO 13849-1:2023) | [E] — [R] only for a rated function | `real_time_distance` contact-regime publish; `distance_engine` bounded hold; `perception_msgs` confidence; `cbf_safety_filter._empty_frame_fault` | implemented | Not a rated function; failing closed here is good practice, not compliance |
| Command feasibility: torque limits, braking authority | [E] | `qddot_to_torque._limit_and_report`; `cbf_safety_filter._brake_authority_fault` | implemented | Braking-authority check is DIAGNOSTIC ONLY (no stop) until its fault rate is known on hardware |
| Documentation of SSM/PFL parameters for the user (ISO 10218-2:2025 clause 7) | [R] | this file + `config/fr3_control.yaml` `iso_*` block + `config/safety/watchman_profile.md` | implemented | Parameters are documented; several are placeholders and say so |
| Rated envelope underneath the software (SLS-J / SLD / safe inputs) | [R] for a rated layer | `config/safety/watchman_profile.md` | **not implemented** — a specification to be applied by a person | Nothing in this repo configures Watchman; FCI/SLS-J compatibility is unverified |

---

## 2. Deviation register

Every place this cell departs from the standards, why, and what a risk
assessment would have to cover.

### D1 — `iso_c_intrusion`: no demonstrated detection capability

**Deviation.** ISO 13855:2024 derives the intrusion distance `C` from the
protective device's **detection capability** `d`: `C = 8·(d − 14)` mm for
`d ≤ 40` mm, `C = 850` mm otherwise. The depth pipeline
(`real_time_distance` + a RealSense D455) has **no demonstrated detection
capability** and is **not a rated protective device** — there is no
IEC/TS 61496-4-3 assessment of it and it was never designed to be one.

**Consequence, stated plainly.** The conformant value is therefore
`C = 0.85 m`. At `C = 0.85 m`:

```
d_floor = C + Z_d + Z_r = 0.85 + 0.06 + 0.01 = 0.92 m   >   FR3 reach 0.855 m
```

**ISO-conformant SSM is not achievable in this workspace.** The protective
separation distance exceeds the arm's own reach, so there is no configuration in
which the robot can both do its task and satisfy `S ≥ S_p` under a conformant
`C`. This is a property of the cell, not a bug and not a tuning failure.

**What is done instead.** Resolution **(c)** of the three: `d_safe` stays at its
tuned research value of **0.10 m** (`config/fr3_control.yaml`), `iso_enabled`
stays **false**, and **no ISO conformance is claimed**. The shortfall against
the floor is therefore **0.82 m**, not a tuning distance. `cbf_safety_filter` raises a
`ValueError` naming all three terms if anyone turns the flag on without moving
one of the numbers, and there is deliberately **no bypass flag**.

**What would change this.** Demonstrating `d ≤ 40 mm` with
`scripts/iso_constants_measure.py detection` — 100 % detection, at the far end
of the working range, over the whole field of view, on the worst-case surface
and reflectivity in the cell. That lowers `C` but still does not make the depth
pipeline a rated protective device.

**Risk-assessment reference:** _____________________ (to be filed)

### D2 — `d_safe` is a tuned value, not an ISO-derived one

**Deviation.** `d_safe = 0.10 m` was tuned against hardware. Under the ISO layer
it would have to be at least `C + Z_d + Z_r`. See D1 for why it is not.

**Not a stable number.** It has moved four times (0.20 → 0.15 → 0.10 → 0.15 →
0.10, most recently in `5569259`), and the barrier's meaning changed underneath
it once: `h` is now measured from the capsule **surface**, so the effective
standoff grew 6-16 cm at an unchanged numeric value. Any statement about
`d_safe` in this file is a statement about the value in `fr3_control.yaml` at
the time of writing — check it, do not quote it. It is also **out of sync with
`franka_sim/config.yaml` (0.15 m)**, which is a real divergence between the
policy's training envelope and the robot's, not only a failing test.

**Consequence.** The barrier offset carries no ISO meaning while `iso_enabled`
is false. The zone ladder, whose rungs are multiples of `d_safe`, inherits that.

**Risk-assessment reference:** _____________________ (to be filed)

### D3 — PFL limits are calculated, not measured

**Deviation.** `iso_v_pfl = 0.68 m/s` comes from `scripts/iso_pfl_speed.py`,
i.e. from Annex M's biomechanical table and the `m_R = M/2 + payload`
simplification. **[R]** Claiming PFL requires force and pressure **measurement**
with a PFMD per ISO 10218-2:2025 clause **6.3.3** and **Annex N**.

**Consequence.** The number supports a preliminary risk assessment and nothing
more. It is also computed for **one** body region (hands-and-fingers,
quasi-static) with **no payload and no gripper**; the binding ceiling is the
smallest `v_PFL` over every region a contact can reach, in the configuration the
robot is actually built in.

**Quasi-static was chosen over transient** (which is roughly twice as
permissive) because a **clamping** event cannot be excluded in this cell — a
benchtop arm with a table under it, and no clamping-hazard analysis on file.
That is an **[E]** choice and it is the conservative one.

**Risk-assessment reference:** _____________________ (to be filed)

### D4 — reduced speed applied as a cell-wide derate

**Deviation.** ISO 10218-1:2025 (5.5.3) / -2:2025 (5.5.6) require reduced speed
(≤ 250 mm/s) for **manual modes**, enforced by a **rated speed-monitoring
function**. Here, `iso_mode: reduced` adds a slack-relaxable QP row on the TCP
and the same cap inside the monitor.

**Consequence.** Two non-rated channels agreeing on 250 mm/s is not a rated
speed-monitoring function and must not be described as one. The **[S]** part is
the number; the **[E]** part is applying it during automatic operation and
enforcing it this way.

**Risk-assessment reference:** _____________________ (to be filed)

### D5 — manual reset latch instead of automatic resumption

**Deviation, recorded for transparency rather than as a shortfall.** SSM
**permits** motion to resume automatically once `S ≥ S_p`. This package latches
(`iso_stop_requires_reset: true`) and requires `/NS_1/safety_reset`.

**Why.** Supervised experiments: a stop that clears itself while the operator is
still working out why it fired is worse than one that waits. It is consistent
with the start/restart interlock and reset function (ISO 10218-1:2025, 5.5.2)
but is not required by it, and setting the flag `false` is **equally
standard-conformant**.

**Risk-assessment reference:** n/a — permitted either way.

---

## 3. Residual gaps no amount of software can close

**G1 — the chain cannot be a safety function.** Single-channel Python over
best-effort DDS cannot meet the **PL d / SIL 2** required of a Class II robot's
safety functions. No amount of testing, review or additional software layers
changes this. The only route to a rated layer is the robot's own functions
(`config/safety/watchman_profile.md`), and that file is a specification, not a
configuration.

**G2 — the FR3's rated functions are certified to the 2011 edition.** EN ISO
10218-1:**2011**, not the 2025 editions. Re-assessment is the integrator's
responsibility and is not claimed here. It is also **not verified** that SLS-J
or SLD can be active while FCI is controlling the robot — the datasheet excludes
SLP-C, SLS-C and SLP-J but is silent about the other two, and an absence of a
note is not a statement of compatibility. If SLS-J turns out to be incompatible
with FCI, the "rated backstop underneath the software" described throughout this
package **does not exist during operation**, and this section is where that has
to be written down.

Two further findings, recorded in `config/safety/watchman_profile.md` §1: at
today's `velocity_box_margin = 0.9` the "SLS-J 20 % above the software cap" rule
is **unachievable** — the target exceeds the FR3's own `qdot_max` on every joint,
so either the margin drops to ≤ 0.83 or SLS-J collapses onto the firmware limit;
and since commit `f5a59f8` the bound the filter enforces is the flat cap
**intersected with the position-based velocity envelope**, a curve a per-joint
scalar SLS-J cannot represent. Near an end stop — which is where the five
`joint_velocity_violation` aborts happened — the software envelope is the only
layer shaped like the hazard.

**G3 — the safe set is not forward-invariant.** Obstacle rows remain
slack-relaxable, and the QP will pay slack rather than return infeasible. The
independent monitor bounds the CONSEQUENCE of a violation; it does not restore
the guarantee. Making the rows hard is not the fix: a hard speed row plus the
hard state box can be infeasible exactly when the arm is already over the cap,
and an infeasible QP is a worse answer than a relaxed row.

**G4 — `S_h` is clamped.** `velocity_standoff_max` caps the speed-proportional
standoff at 2.0 m (the declared spec ceiling). With `v_h = 2.0 m/s` and
`T_r + T_s = 2.1 s`, the full `S_h` would be 4.2 m. The standoff therefore
carries **less than `S_h`** whenever the closing speed is high, and
`_iso_configure` logs a warning saying so. This is a consequence of the same
arithmetic as D1: the distances the standard asks for are larger than this cell.

**G5 — `a_s` is a minimum over a finite grid.** The Annex H grid is 33/66/100 %
of speed, payload and extension. It is not a worst case over all payloads,
configurations and directions, and `S_s = v_r²/(2·a_s)` is conservative only to
the extent that grid found the worst deceleration.

**G6 — the braking-authority check does not stop.** `iso_brake_frac_min` /
`iso_brake_frac_ticks` raise `fault_braking` and log; they do not brake. The
q̈ → τ chain is open-loop (`M q̈ + C q̇`, no PD, no friction model), so the check
would fire on model error as readily as on danger until its fault rate is known
on hardware.

**G7 — `franka_sim` mirrors the pre-ISO math.** `franka_sim/envs/cbf_filter.py`
is a deliberate duplicate of the filter's row math and was **not** updated by
this work. Anything trained or validated against it is validated against the
pre-ISO filter, not the deployed one.

**G8 — the ISO layer has not been run on the robot.** Every `iso_*` flag ships
`false`. Nothing in section 1 is `validated on hardware`.

---

## 4. Turning it on

The flags exist so the layer can be validated, not so it can be left on. Before
`iso_enabled:=true`:

1. Measure the constants:
   `scripts/iso_constants_measure.py reaction | stop | detection | uncertainty`,
   then paste its `yaml` output into the `iso_*` block of
   `config/fr3_control.yaml`.
2. Recompute the PFL ceiling for the robot **as it is currently built**:
   `scripts/iso_pfl_speed.py --payload <kg> --gripper <kg>`.
3. Resolve D1: either demonstrate a detection capability, or accept that this is
   a research deviation and say so in writing.
4. Run the preflight: `scripts/iso_preflight_check.py --wait 5`. The launch file
   runs it automatically before the controller spawner and aborts on a non-zero
   exit.
5. Apply and validate `config/safety/watchman_profile.md` on the robot, with a
   Safety Operator.
6. First run on fake hardware
   (`torque_control_stack.launch.py use_fake_hardware:=true iso_enabled:=true`),
   then supervised on the FR3 with the enabling device in hand.

With every flag `false` — the shipped state — the filter's numerical output is
identical to what it was before this layer existed, and no ISO claim is made.

---

## 5. "roadmap Step N" in the code comments

About twenty comments and docstrings across the package cite *roadmap Step N*.
The roadmap itself was a point-in-time plan, fully executed in commit
`51dab09` ("Add the ISO 10218 alignment layer, behind flags that ship off") and
deleted afterwards; it is recoverable with
`git show 51dab09:franka_experiments/ISO_COMPLIANCE_ROADMAP.md`, together with
the implementation report of the same commit
(`git show 51dab09:franka_experiments/ISO_IMPLEMENTATION_REPORT.md`).
This table is what the surviving references mean, so they stay readable without
it:

| Step | Subject | Principal location |
|---|---|---|
| 1 | Measure the ISO input constants (`T_r`, `a_s`, `d` → `C`, `Z_d`) | `scripts/iso_constants_measure.py` |
| 2 | The declared `iso_*` parameter block and its validation spec | `config/fr3_control.yaml`, `utils/config.py` |
| 3 | The SSM / PFL closed forms | `utils/iso_ssm.py` |
| 4 | `d_safe` floor = `C + Z_d + Z_r`; `S_h` through the velocity standoff | `cbf_safety_filter._iso_configure` |
| 5 | The SSM cap drives the task-space speed rows | `utils/cbf_state_rows.py`, `_ISO_RHO_SLACK_SPEED` |
| 6 | Independent monitor + non-safety-rated stop | `nodes/iso_safety_monitor.py` |
| 7 | Fail-closed perception (contact regime, bounded hold, confidence, empty frames) | `real_time_distance`, `distance_engine`, `perception_msgs`, `cbf_safety_filter._empty_frame_fault` |
| 8 | Command feasibility: torque clip / rate limit, braking authority | `qddot_to_torque._limit_and_report`, `cbf_safety_filter._brake_authority_fault` |
| 9 | Absolute ceilings: PFL and the 250 mm/s reduced-speed derate | `scripts/iso_pfl_speed.py`, `cbf_state_rows`, `_iso_configure` |
| 10 | Status contract, diagnostics, logging | `cbf_safety_filter._publish_status`, `logging_utils`, `experiment_logger` |
| 11 | Hardware safety layer and the preflight gate | `config/safety/watchman_profile.md`, `scripts/iso_preflight_check.py` |
| 12 | This file | `SAFETY.md` |

Two things were added after Step 12 and belong to no step:
`nodes/iso_evidence_logger.py` with `scripts/iso_evidence_report.py` (record a
run, then read it back as a verdict per check), and the Cartesian /
acceleration / avoidance columns of `experiment_logger` with
`scripts/experiment_summary.py`. Both are described in the package `README.md`.
