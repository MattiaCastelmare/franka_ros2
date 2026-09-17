# Watchman profile — the rated envelope under the software filter

**Status: TO BE APPLIED AND VALIDATED.** Nothing in this file is configured on
the robot by any code in this repository. Watchman settings are applied through
Franka's own Desk interface by a person, and validated by the Safety Operator.
This document is the specification of what to apply and the record of what was
found; it is not a configuration file and nothing reads it.

## Why this exists

`cbf_safety_filter` is single-channel Python over best-effort DDS. It cannot
reach a safety rating — not with better code, not with more tests. The robot's
own functions can, and they are underneath everything this package does. The
design intent throughout is therefore:

> **the software bites first, the rated layer is the backstop.**

If the rated layer is the thing that stops the robot during normal operation,
the software limits are set wrong, not the Watchman limits.

## Normative context

ISO 10218-1:2025 (5.1.17, Annex C) classifies robots as **Class I**
(manipulator mass ≤ 10 kg, maximum force ≤ 50 N, speed ≤ 250 mm/s) or
**Class II**. The FR3 manipulator is 17.8 kg, so it is **Class II**, and the
safety functions required of a Class II robot must reach **PL d / SIL 2** per
ISO 13849-1:2023 / IEC 62061:2021 **[R]**.

The FR3's own functions are rated **PL d / Cat 3** and are certified against
**EN ISO 10218-1:2011**. Re-assessment against the 2025 editions is the
integrator's responsibility. **This document does not claim it has been done.**

## What to configure

### 1. SLS-J — safe limited speed, per joint  **[E]**

Set each joint's safe speed limit **~20 % above** the software cap, i.e. above
`velocity_box_margin * qdot_max` from `config/fr3_control.yaml` and
`franka_description/robots/fr3/joint_limits.yaml`.

**At today's `velocity_box_margin = 0.9`, that rule has no room left.** The +20 %
target lands above the FR3's own flat `qdot_max` on **every joint**, and a safe
limit cannot be set above the limit the firmware already enforces:

| joint | `qdot_max` [rad/s] | software cap (×0.9) | +20 % target | admissible? |
|---|---|---|---|---|
| 1 | 2.62 | 2.36 | 2.83 | **no** — above `qdot_max` |
| 2 | 2.62 | 2.36 | 2.83 | **no** |
| 3 | 2.62 | 2.36 | 2.83 | **no** |
| 4 | 2.62 | 2.36 | 2.83 | **no** |
| 5 | 5.26 | 4.73 | 5.68 | **no** |
| 6 | 4.18 | 3.76 | 4.51 | **no** |
| 7 | 5.26 | 4.73 | 5.68 | **no** |

The headroom exists only for `velocity_box_margin <= 1/1.2 = 0.83`. Two
admissible resolutions, and they are a choice, not a formality:

- **(a)** lower `velocity_box_margin` to **0.83 or below** and derive SLS-J as
  `1.2 · margin · qdot_max`. At 0.8 that is 2.52 / 2.52 / 2.52 / 2.52 / 5.05 /
  4.01 / 5.05 rad/s, all inside `qdot_max`, and the design intent holds: the
  software bites first with a real margin.
- **(b)** keep 0.9 and set **SLS-J = `qdot_max`**. The rated layer then
  coincides with the flat firmware limit rather than sitting above the software
  cap, so the backstop is 11 % away from the software cap instead of 20 % — and
  SLS-J stops being an independent envelope, because the firmware enforces that
  same number anyway. **[E]** Acceptable only if the risk assessment says 11 %
  is enough.

**A second, larger caveat since commit `f5a59f8`.** The bound the filter
enforces is no longer flat: it is `velocity_box_margin · qdot_max`
**intersected** with the FR3's *position-based* velocity envelope
(`utils/cbf_hard_limits.fr3_velocity_envelope`, from
`position_based_velocity_limits` in `joint_limits.yaml`). Near an end stop the
firmware allows far less than the flat limit — five hardware aborts with
`joint_velocity_violation` sat at 0.85 of the flat limit or below. A per-joint
scalar SLS-J cannot represent that curve; it bounds only the flat part. In the
region where the violations actually happened, the software envelope, not SLS-J,
is the only thing shaped like the hazard.

Re-derive the middle column from the `velocity_box_margin` actually in the YAML
at the time of configuration — it has moved before (0.6 → 0.9).

Recorded value applied: _______________   date: __________  by: __________

### 2. SLD / SLP-J — the cell envelope  **[E]**

An **SLD** safe distance and/or an **SLP-J** joint-space envelope bounding the
arm to the physical cell. Sized so the arm cannot reach outside the area the
presence-sensing device covers.

Recorded value applied: _______________   date: __________  by: __________

### 3. Safe inputs  **[R] for a rated protective function**

- **X3.2 / X3.3** mapped to the cell's presence-sensing device and to SMSS
  (safe monitored standstill).
- **X3.1** emergency stop present, reachable from every working position.
- **Enabling device** present and reachable, for any manual-mode work.

Recorded wiring: _______________   date: __________  by: __________

### 4. Stopping times and distances  **[S] measured to Annex H**

Transcribe the Category 0, 1 and 2 stopping times and distances from the Franka
Research 3 product manual into the table below, and cross-check them against
what `scripts/iso_constants_measure.py stop` measures on this unit.

The manual's data is measured to **EN ISO 10218-1:2011 Annex B**, renumbered
**Annex H** in the 2025 edition — the same procedure, a different number.

**Keep the worse value of the two.** `iso_a_stop` in `fr3_control.yaml` must be
the minimum realized deceleration over both sources.

| category | manual: time [s] | manual: distance | measured: T_s [s] | measured: S_s [m] | kept |
|---|---|---|---|---|---|
| 0 | | | (not reproducible in software) | | |
| 1 | | | (not reproducible in software) | | |
| 2 | | | | | |

Manual revision cited: _______________   date: __________  by: __________

### 5. FCI compatibility — **VERIFY ON THE ROBOT, DO NOT ASSUME**

The FR3 datasheet states that FCI **cannot control the robot** while **SLP-C**,
**SLS-C** or **SLP-J** is active.

It does **not** list **SLS-J** or **SLD** among those exclusions. That is an
*absence of a note*, not a statement of compatibility. Before the cell design
depends on either being active during FCI operation:

1. configure it in Watchman on **this unit**,
2. start the FCI stack and confirm the robot accepts torque commands,
3. record the result here.

| function | active during FCI? | verified on | by |
|---|---|---|---|
| SLS-J | ☐ yes ☐ no ☐ not tested | | |
| SLD | ☐ yes ☐ no ☐ not tested | | |
| SLP-J | expected NO (datasheet) | | |
| SLS-C | expected NO (datasheet) | | |
| SLP-C | expected NO (datasheet) | | |

If SLS-J turns out to be incompatible with FCI, the "rated backstop" part of
this design does not exist, and `SAFETY.md`'s residual-gap list must say so
explicitly rather than continuing to describe a backstop that is switched off
whenever the software is running.

## Sign-off

| role | name | date | signature |
|---|---|---|---|
| Configured by | | | |
| Safety Operator (validated) | | | |
