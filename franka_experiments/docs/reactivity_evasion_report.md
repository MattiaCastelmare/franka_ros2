# Reactivity, evasion and never-stop rework — Phase 0–5 report (Sep 2026)

Branch `humble-mattia`, one commit per phase after `611d3c8`. Everything below
was measured with the tools committed alongside it:
`scripts/latency_budget.py` (Phase 0), `scripts/compare_vobs.py` (Phase 1),
`scripts/cbf_scenarios.py` (Phase 5), and the test files named per phase.
Suite: 444 passed → 531 passed, the same 10 pre-existing failures
(9 in `test_distance_surface_semantics`, 1 in `test_cbf_risk_weighted_slack`).

## Phase 0 — measured latency budget and retreat authority

Hop by hop, median / p95 in ms:

| Hop | med | p95 | Source |
|---|---|---|---|
| frame sampling wait (event → next capture) | 17 @30 fps, 33 @15 fps | 32 / 63 | uniform on one period |
| camera stamp → depth receipt, D455 as recorded (July bags, 15 fps) | 62.1 | 64.3 | bag receipt stamps |
| camera stamp → CBF receipt (`capage`), today's live 30 fps runs | 12–14 | 17–19 | container logs |
| real_time_distance compute (tf, mask, distance, cluster+track, message) | 3.75 | 4.10 | offline replay, 640×480 |
| DDS distances → CBF (best-effort, no shared memory) | 1.37 | 1.66 | dds probe |
| wait for the 50 Hz rebuild | 10 | 19 | uniform |
| ConstraintBuilder.build (22 rows, tracker+evasion+uncertainty on) | 0.84 | 0.88 | real FR3 model |
| wait for the 100 Hz tick | 5 | 9.5 | uniform |
| whole QP tick (solve 0.03, setup 0.35) | 0.32 | 0.34 | real FR3 model |
| DDS qddot_safe, DDS torque_cmd (each) | 0.12 | 1.35 | dds probe |
| qddot_to_torque dynamics | 0.01 | 0.01 | pinocchio |
| 1 kHz controller pickup + apply | 1.5 | 2.0 | structural |

**t_blind** (event → torque write): ~48 / 85 ms at 30 fps, ~118 / 168 ms at
15 fps. The D455 stream was down during the session, so the 30 fps camera hop
comes from the `capage` field of today's live logs, not a direct probe.
Every bag and today's live tracker lines show a 66.7 ms frame period: the
driver had been falling back to 15 fps; the launch now pins 848x480x30.

Structural gates that dominate a fast obstacle: the 0.7 m publish gate
(`max_thresh`), the tracker birth (velocity usable 2–4 frames after first
sight), the association ceiling (4.25 m/s @30 fps, 3.5 m/s @15 fps: faster
objects never get a track), the distance-engine spike gate above 2 m/s.

**Retreat authority** (closed form, `retreat_speed_available`, exact LP over
the joint velocity box at 0.9·qdot_max): median 2.3 m/s, flange 2.3–3.5 m/s,
link4 control points 0.10–0.20 m/s along their weak normal; a_avail median
3.9 m/s², min 0.15.

**0c.** From the 0.7 m gate, standing start: retreat outruns ≤ 1.5–2.2 m/s;
lateral evasion of a new object ≤ 1.2–2.0 m/s; already tracked and seen from
1.2 m: 2.7–4.3 m/s. A thrown ball at 3–6 m/s is not reachable with the
current camera and gates.

## Phase 1 — root cause of the reactivity regression

Two structural causes, both measured on the hand bags:

1. `cluster_max_radius_m: 0.6` dropped every person (a standing person is one
   0.76–1.06 m blob), so the tracker delivered a velocity on 2–5 % of the
   approaching samples inside 0.7 m (61–67 % at 1.5 m). Raised to 1.5;
   nothing in three bags exceeds 1.43 m and the 2 m room blobs no longer occur.
2. Even with the person tracked, the track is the velocity of the body
   centroid: a reaching hand read 0.22–0.27 m/s on the residual and 0.003 m/s
   on the track. New flag `obstacle_velocity_residual_floor` hands the QP
   max(v_track, v_residual); tightening only.

Live logs: v_obs reached the QP on 22 % of the approaching ticks in tracker
mode vs 72 % in residual mode. Not the cause: KF bandwidth, coasting,
feedforward (off before and after), the runtime source (it is `tracker`).
Caps retuned for linearity over 0.1–1.0 m/s: retreat 0.6 → 1.1, link speed
0.8 → 1.3. Closed loop on the real model: retreat 0.19 / 0.47 / 0.95 m/s for
0.2 / 0.5 / 1.0 m/s approaches, cap slack zero; static obstacle leaves the
task untouched.

## Phase 2 — outrun test and lateral escape

`utils/evasion_direction.py`: r = n̂ᵀv_obs / (margin·v_avail); escape = +n̂
while outrunnable, rotating with a smoothstep toward the fastest direction
⟂ v_obs above it; bias on the QP target through a damped pinv(Jp), norm
capped. Flag `enable_outrun_evasion` (on in launch defaults). Bit-identical
rows with the flag off, zero velocity, static or outrunnable obstacle.

## Phase 3 — never stop

Fallback ladder (full → box-only → braking, every rung finite and in-box,
levels logged), accepted iterates clipped into the hard box, and a real hole
closed: the box guard could demand 189 rad/s² on a 17 rad/s² joint
(`accel_box_clip_to_limits`). Livelock detector (blocked by dnorm, not
progressing by displacement over 0.5 s) with a bounded, logged nudge in the
nullspace of every nearly-binding obstacle row. Freeze audit as a property
test: k < 7 binding rows always leave a nullspace escape.

## Phase 4 — latency compensation (flag OFF)

LinkDistance carries acceleration, position and position-velocity covariance;
the CBF moves each tracked obstacle forward by 0.085 s and tightens by the
propagated 2σ, tighten only, clamped. Constant-velocity prediction matches
truth within the filter's own 2σ. Also repaired the launch argument block
broken in Phases 2–3.

## Phase 5 — scenarios (real FR3 model, real builder, OSQP, emulated 30 fps chain)

| Scenario | min h [m] | h < 0 | contact | v_obs / retreat peak | escape | slack max | fallback | livelock | task |
|---|---|---|---|---|---|---|---|---|---|
| static beside path | +0.032 | no | no | 0 / 0 | – | obs 0.005 | none | 0 | completed |
| around (obstacle on the arc) | +0.012 | no | no | 0 / 0.05 | – | cap 0.53 | none | 0 | completed |
| slow pursuer 0.3 m/s | +0.019 | no | no | 0.30 / 0.32 | – | obs 0.02 | none | 0 | n/a (pursuit) |
| fast pass 3 m/s, seen at 0.7 m | −0.172 | 0.20 s | **yes** | 3.0 / 0.97 | w=1, push ⟂ v (cos 0.10) | obs 12.3 | none | 0 | n/a |
| same, seen at 1.5 m | −0.093 | 0.16 s | no | 3.0 / 1.01 | w=1, cos 0.09 | obs 9.7 | none | 0 | n/a |
| ball 4 m/s from 2 m | −0.196 | 0.14 s | **yes** | 4.0 / 0.58 | w=1, cos 0.00 | obs 13.4 | none | 0 | n/a |
| ball 1.5 m/s from 2 m | +0.030 | no | no | 1.5 / 1.03 | w=1, cos 0.34 | obs 2.7 | none | 0 | n/a |
| wedge (goal inside the V) | −0.057 | whole run | no | 0 / 0.05 | – | cap 1.9 | none | 1 escape | not completable |

No scenario produced a QP failure, a fallback rung, a non-finite command, or
a command outside the joint box. The two contacts are the two cases 0c
predicts as unreachable from the 0.7 m gate; the same 3 m/s pass seen from
1.5 m is dodged, so the limit is the publish gate, not the filter. The wedge
goal is inside the V by construction: the escape moves the arm along the free
direction and the task PD pulls it back — no deadlock, no contact.

Bag replay (`--bag rosbag/arm_complex`): inconclusive as a scenario — the
current extrinsics applied to the July recording put the nearest "obstacle"
on the capsule surface in every frame (the arm seeing itself); the filter
still answered all 899 ticks at level 0.

Not done: MuJoCo dynamics (the headless loop integrates joints as a double
integrator; franka_sim's CBF is a separate, stale port), hardware validation,
and a direct D455 latency probe.

## B5, honestly

Not reachable with the current camera. Above 4.25 m/s the tracker never
associates a second frame; below that, the 0.7 m publish gate plus a 48–85 ms
blind time leaves less than the 0.23–0.28 s a control point needs to move
15 cm sideways. The levers are the publish gate (`max_thresh`, or decoupling
it from the self-detection guard), the tracker gates, and a faster camera —
not the QP.


---

# Follow-up: the arm was not smooth, and the camera was making it move

Reported after the phases above went in: motion no longer smooth even with a
STATIC obstacle, and camera noise producing strange velocities. Both were
mine. What follows is what was measured, what was wrong, and what changed.

## What the data said (and what it contradicted)

The first hypothesis — the residual estimator I put back under the tracker —
was wrong, and the measurement says so. On genuinely static frames of
rosbag/arm_complex and rosbag/arm_repeated (gap steady to 1 cm over 0.7 s):

| estimator | p99 on a static scene | above 0.15 m/s |
|---|---|---|
| residual (EMA, shipped) | 0.017-0.029 m/s | 0.0 % |
| **tracker** | **0.205-0.286 m/s** | **1.4-2.5 %** |

The tracker is the noisy one, with excursions past 1.6 m/s on a scene where
nothing moves. Its cause is not tuning: q_jerk and sigma_meas were swept over
three decades each, sigma_v0 and the frame gate too, and an innovation-gated
robust Kalman update was implemented and measured — the fabricated velocity
moved by less than a millimetre per second at every percentile. The cluster
centroid genuinely wanders as the visible surface of a blob changes, and the
filter differentiates it correctly. Two changes were written against the wrong
hypothesis and reverted: a 0.2 s windowed residual (measured WORSE than the
EMA it replaced) and the robust update.

Through k1 = 10.5 that wander was putting a p99 step of 2.95 rad/s² into every
obstacle row's right-hand side, every tick.

## What was actually wrong

1. **The tracked velocity reached the QP unconditioned.** Fixed at the
   consumer: a 3-frame median (measured free — a real approach reports
   0.30 / 0.60 / 1.01 m/s against 0.3 / 0.6 / 1.0 of truth) and a 0.15 m/s
   soft deadband (a static scene's p90 becomes exactly zero). The residual
   floor is deliberately NOT deadbanded: it is the quiet one and the reactive
   one, and max() of the two is quiet AND reactive. Step: 2.95 → 1.41 rad/s².
2. **The outrun evasion read the raw track vector, not the conditioned
   estimate.** So it swerved the arm at full urgency while v_obs reaching the
   QP was exactly zero. It now reads the same number as the barrier and the
   retreat cap; the vector is used only for the direction, which is the one
   thing a scalar cannot give.
3. **The outrun evasion had no distance gate.** It compares a speed against a
   speed, so it fired anywhere inside the 1.2 m horizon. Now gated at 0.30 m
   with a smoothstep fade over the outer third, so the gate cannot step.
4. **v_avail collapses with row leverage**, so on the link4 control points
   (0.10-0.20 m/s along their weak normal) 0.15 m/s of noise read as "cannot
   be outrun" on 2-4 % of directions and 0.30 m/s on a third of them. Floored
   at 0.30 m/s: a point that cannot retreat at 0.3 m/s cannot escape sideways
   at 0.3 m/s either.
5. **The livelock escape ended in one tick** — a step of the full gain into
   q̈_nom — and was **added raw**, with no EMA, while its direction is rebuilt
   at 50 Hz. Now released on a ramp, EMA'd like every other bias, and only
   added with rows in hand.
6. **The livelock fired on an arm parked at its goal**, because a violated
   barrier bends the nominal there too. It now also requires the task's own
   command to be asking for motion.
7. Gentler defaults where Phase 5 had raised them for an unreachable
   scenario: escape accel 3.0 → 1.5, bias cap 6.0 → 3.0, its EMA 0.7 → 0.8,
   livelock gain 3.0 → 2.0, cluster radius 1.5 → 1.2 (the measured person
   clusters have a 99th percentile of 1.06-1.21 m).

## Measured result

New scenario `static_noisy`: a static obstacle plus the tracker's measured
noise floor, fitted to the real sequences. New metric on every scenario: the
tick-to-tick change of the commanded acceleration.

| scenario | jerk p99 [rad/s³] | per-joint step max | task |
|---|---|---|---|
| static, noise, before | 1055 | — | drifted 0.037 rad |
| **static, noise, after** | **0.0** | **0.00** | completed |
| 0.3 m/s approach + noise, before | 1098 | — | evasion fired, cos 0.80 |
| **0.3 m/s approach + noise, after** | **78** | **1.09** | v_obs 0.30, retreat 0.32 |

The reactive path is untouched: the noisy 0.3 m/s approach reports exactly the
same v_obs and the same retreat as the noiseless one. Across all ten
scenarios the per-joint step never exceeds `max_qddot_delta` (5.0 rad/s² per
tick), the emergency cases saturating it as they should.

Suite: 542 passed, the same 10 pre-existing failures. The node constructs and
ticks (`test/smoke_cbf_construct.py`), and the launch file parses.
