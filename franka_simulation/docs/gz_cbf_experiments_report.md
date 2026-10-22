# Gazebo CBF experiments: report (SIM)

**Every number in this report is SIM**: pipeline A of `franka_experiments`, unchanged,
closed around a Gazebo (Ignition Fortress 6.18, DART 1 kHz) FR3 with a simulated D455 and
scripted obstacles. Nothing here is a hardware result. Plan, hazard audit and every
deviation from the spec: `gz_cbf_experiments_plan.md` (§2, §3, §9). How to run it:
`franka_simulation/README.md` §13.

## 0. How the numbers were produced

```bash
# inside the franka_ros2 container, ROS_DOMAIN_ID=77
ros2 run franka_simulation_experiments run_gz_scenarios --grid phase5      --batch-dir <B>
ros2 run franka_simulation_experiments run_gz_scenarios --grid emulated30  --batch-dir <B>
ros2 run franka_simulation_experiments run_gz_scenarios --grid flags       --batch-dir <B>
ros2 run franka_simulation_experiments run_gz_scenarios --rescore          --batch-dir <B>
ros2 run franka_simulation_experiments run_gz_scenarios --report           --batch-dir <B>
```

`<B>` = `franka_simulation_experiments/sim_runs/batches/phase5` (git-ignored; every cell
keeps its bag, truth, logger files and `cell_result.json`). One cell = one fresh headless
Gazebo with the whole stack, torn down by process group before the next. Each cell's
manifest records the git hash it ran on. The batch ran on the working tree of commit 24ea70e
plus the Phase 5 files. Those commits were later undone at the user's request; the files are
unchanged in the working tree.

**What varies.** Condition (`residual`, `kf`, `imm`, `imm+pred`; every toggle explicit,
`config/conditions.yaml`) × scenario (the 11 of `cbf_scenarios.py` ported 1:1, 5 around the
pentagon task, ballistic throws 2–6 m/s, the `approach_envelope` straight throws 0.5–6 m/s)
× seed (start phase within a camera frame, adapter noise) × perception:

* **full** — Gazebo depth → `depth_sim_adapter` → `real_time_distance` → tracker → CBF;
* **emul** — `emulated_perception`: the rows `cbf_scenarios.py` feeds its offline filter
  (true gaps, noisy true velocity, one track), built by the real `build_cp_messages`;
* **off** — `cbf_scenarios.py` itself, offline (double integrator, emulated perception).

## 1. Headline (SIM)

345 cells, all completed and scored, 0 reflex trips (velocity / position / torque). Full chain:
RTF median 0.84 (p10 0.73, 172 of 236 cells below 0.9), achieved QP rate median 264 Hz of a nominal
300, capture → rows 24 ms p50 / 33 ms p95 and CBF sensing age 25 / 34 ms (sim time). Emulated
perception: RTF 1.00, QP 279 Hz.

* **The tracker estimates obstacle velocity 4–5× better than the residual, with no lag.** Pooled
  v_obs RMS against the truth, full chain, median over cells: 0.3 m/s obstacles, residual 0.10 m/s vs
  kf 0.023 (imm 0.028, imm+pred 0.028); 0.5–1.5 m/s, 0.58 vs 0.17 (0.21 / 0.25); ≥ 2 m/s, 3.5 vs
  0.72 (0.80 / 0.65). Lag: the residual trails by 5–21 ms on the 0.3 m/s pursuers and by 65–125 ms
  on everything that crosses or is thrown (the 0.3 m/s `pentagon_crossing` included, −80 ms); the
  trackers stay within ±8 ms everywhere.
* **That accuracy buys margin only below ~1.5 m/s.** min h, median over cells: 0.3 m/s, residual
  +0.072 m vs kf +0.127 (imm +0.129, imm+pred +0.130); 0.5–1.5 m/s, −0.033 vs +0.023 (+0.008 /
  −0.076). Contact rate on the dynamic scenarios is the same for all four conditions (68–72 %),
  because it is set by the ≥ 2 m/s passes: contact in 89 of 91 such cells. Offline
  `cbf_scenarios.py` is in contact on `fast` and `ball` as well; on `fast_seen_early` it is not: the plant
  costs ~8 cm there (F1) and perception the rest, up to contact (F2). The throws have no offline counterpart.
* **≥ 2 m/s is warning-time-limited, not estimator-limited.** Every tracker-condition pass is
  either ok or "4 too-late"; none is not-in-rows, untracked or untrusted. At ≥ 2 m/s the ball first
  appears in the rows 220 ms (median) before closest approach — the 0.7 m publish gate divided by the
  speed — and the CBF acts 148 ms before it, against the 450 ms the classifier asks for.
* **IMM and ballistic prediction add nothing measurable here.** imm ≈ kf on every metric;
  imm+pred lowers the velocity error at ≥ 2 m/s (0.65 vs 0.72) with the same contact count (23/24),
  and is the worst condition at 0.5–1.5 m/s (`ball_slow` min h −0.127 m median over 3 seeds,
  [−0.154, −0.108], contact 1/3, vs kf +0.004).
* **The uncertainty margin trusts a σ that is too large for slow obstacles.** Share of trusted rows
  with |v_trk − v_true| ≤ 1σ: 1.00 at 0.3 m/s, 0.91 at 0.5–1.5 m/s, 0.77 at ≥ 2 m/s (Gaussian
  0.68); per scenario `fast` / `fast_seen_early` fall to 0.27–0.36, where the track splits.
* **Columns: the Gazebo plant reproduces `cbf_scenarios` at low speed and costs margin at speed;
  perception costs the most where detection range matters** (table D, §4).

## 2. H1–H13: verification outcomes

Full evidence (file:line) in the plan §2; corrections in plan §7.

| # | Verdict | Outcome |
|---|---|---|
| H1 | ✔ | Dependency cycle confirmed (colcon refuses to order the workspace). Option B: thin package `franka_simulation_experiments`. |
| H2 | ✔ | Depth must be 16UC1 mm with 0 = no return; a raw 32FC1 frame would be dropped silently. `depth_sim_adapter` converts. |
| H3 | ◐ | Plugin adds KDL g(q) with **g = 9.8** to EFFORT joints, chain to `fr3_hand_tcp` (fingers not compensated). World at 9.8, fingers rigid and lumped. τ = 0 floats (Phase 1: drift 0, effort − g ≈ 1e-12 N·m). |
| H4 | ✔ | `RtTorqueController` not in the plugin's YAML; added as a second `<parameters>` file written by the stack's own generator (hardware gains, 1 kHz, `gazebo: true`). |
| H5 | ◐ | Envelope, position and effort limits checked at 1 kHz by `reflex_monitor`. **Correction:** torque RATE is clipped by libfranka's limiter (`robot.hpp:318`), not a reflex: counted, not failed. |
| H6 | ◐ | No node sets `use_sim_time`. **Correction:** the CBF's staleness checks already run on the node clock (`cbf_safety_filter.py:1635`), so `SetParameter(use_sim_time)` suffices. |
| H7 | ✔ | All consumers read `/NS_1/franka/joint_states` by joint name; the plugin is namespaced and remapped like `ros2_control_node`. |
| H8 | ✔ | Camera pose and intrinsics generated from `camera_extrinsics.yaml` / `depth_intrinsics.yaml` (HFOV 1.5594 rad). Gazebo's centred principal point: a 2–5 px SIM difference. |
| H9 | ✔ | Mask from collision meshes vs rendered visuals: robot-only scene gives 0 obstacle rows, 0 leak px (Phase 3). |
| H10 | ✔ | `cluster_min_points 5` at `pixel_step 6`: a 5 cm ball is 2.5 samples at 2 m. Gazebo returns the whole disc, the D455 ~40 %: SIM is optimistic. |
| H11 | ✔ | Fidelity stages (delay, dropout, range noise) in `depth_sim_adapter`, each switchable, off by default; `range_noise` on in `static_noisy` / `slow_noisy`. |
| H12 | ✔ | The three configs disagree on tracking / IMM / prediction / source; every run sets all of them explicitly. |
| H13 | risk | Headless rendering needs the NVIDIA EGL ICD (Mesa renders nothing). RTF and achieved QP rate are logged per cell (table A). |

H14–H21 (not in the spec, found while verifying; plan §3.1): the `base` frame, finger
command UB in the plugin, start pose, plugin namespace, effort semantics, **missing rotor
inertia (armature)** — without it joint 7 ran away under the hardware gains —, headless
OpenCV, depth before TF.

## 3. Tables

<!-- AUTO:BEGIN -->
_Generated by `run_gz_scenarios --report` on 2026-10-21 17:21 from `batches/phase5` (324 cells, git `94cd7b0` dirty). **Every number below is SIM.**_

### A. Run quality (SIM)

| cells | N | ok | RTF median | RTF p10 | RTF min | QP Hz median | QP Hz p10 | reflex PASS | reflex trips | τ-rate clips | RTF < 0.9 | rows Hz | capture→rows p50 [ms] | p95 | CBF sensing age p50 [ms] | p95 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| full chain | 236 | 236 | 0.84 | 0.73 | 0.45 | 264 | 226 | 236 | 0 | 8754 | 172 | 91 | 24 | 33 | 25 | 34 |
| emulated perception | 88 | 88 | 1.00 | 0.87 | 0.76 | 279 | 210 | 88 | 0 | 11504 | 13 | 90 | 14 | 15 | 14 | 16 |

### B. Estimator vs truth, full chain (SIM, Phase 5.1)

Per scenario and condition, median over seeds. "CBF uses" is the estimator the condition feeds the barrier; RMS / bias / lag are of that one, against n̂ᵀv_true on the samples whose obstacle point lies on the sphere, per control point with >= 30 samples (compare_vobs); "pooled" = over the on-obstacle samples of every control point (a fast ball is in front of each one for a few frames only). Lag: NEGATIVE = the estimate trails. The last two columns score both estimators on the same run (the residual is recomputed from the run's own rows). Noise floor: compare_vobs static windows on a static obstacle.

| scenario | condition | CBF uses | RMS [m/s] | bias [m/s] | lag [ms] | noise floor [m/s] | pooled RMS | pooled bias | pooled n | RMS residual | RMS tracker |
|---|---|---|---|---|---|---|---|---|---|---|---|
| static | residual | residual | 0.015 | +0.000 | – | 0.000 | 0.015 | +0.000 | 3993 | 0.015 | – |
| static | kf | tracker | 0.005 | -0.000 | – | 0.000 | 0.005 | -0.000 | 3993 | 0.019 | 0.005 |
| static | imm | tracker | 0.012 | +0.001 | – | 0.001 | 0.012 | +0.001 | 3993 | 0.019 | 0.012 |
| static | imm+pred | tracker | 0.012 | +0.001 | – | 0.001 | 0.012 | +0.001 | 3971 | 0.019 | 0.012 |
| around | residual | residual | 0.046 | +0.002 | – | 0.002 | 0.046 | +0.002 | 9999 | 0.046 | – |
| around | kf | tracker | 0.005 | +0.000 | – | 0.000 | 0.005 | +0.000 | 9988 | 0.039 | 0.005 |
| around | imm | tracker | 0.009 | -0.001 | – | 0.000 | 0.009 | -0.001 | 9988 | 0.040 | 0.009 |
| around | imm+pred | tracker | 0.009 | -0.001 | – | 0.000 | 0.009 | -0.001 | 9977 | 0.039 | 0.009 |
| static_noisy | residual | residual | 0.027 | +0.000 | – | 0.007 | 0.027 | +0.000 | 5995 | 0.027 | – |
| static_noisy | kf | tracker | 0.008 | -0.000 | – | 0.003 | 0.008 | -0.000 | 5973 | 0.027 | 0.008 |
| static_noisy | imm | tracker | 0.011 | -0.001 | – | 0.009 | 0.011 | -0.001 | 5962 | 0.028 | 0.011 |
| static_noisy | imm+pred | tracker | 0.011 | -0.001 | – | 0.010 | 0.011 | -0.001 | 5951 | 0.026 | 0.011 |
| approach_hold | residual | residual | 0.089 | -0.020 | – | – | 0.089 | -0.020 | 5855 | 0.089 | – |
| approach_hold | kf | tracker | 0.007 | -0.000 | – | – | 0.007 | -0.000 | 5866 | 0.090 | 0.007 |
| approach_hold | imm | tracker | 0.011 | -0.001 | – | – | 0.011 | -0.001 | 5846 | 0.087 | 0.011 |
| approach_hold | imm+pred | tracker | 0.011 | -0.001 | – | – | 0.011 | -0.001 | 5872 | 0.095 | 0.011 |
| slow | residual | residual | 0.098 | +0.008 | -5 | – | 0.098 | +0.008 | 3003 | 0.098 | – |
| slow | kf | tracker | 0.018 | -0.004 | +0 | – | 0.018 | -0.004 | 2981 | 0.105 | 0.018 |
| slow | imm | tracker | 0.023 | -0.006 | -2 | – | 0.023 | -0.006 | 2981 | 0.101 | 0.023 |
| slow | imm+pred | tracker | 0.023 | -0.006 | -2 | – | 0.023 | -0.006 | 2959 | 0.092 | 0.023 |
| slow_noisy | residual | residual | 0.106 | +0.008 | -21 | – | 0.106 | +0.008 | 2992 | 0.106 | – |
| slow_noisy | kf | tracker | 0.023 | -0.005 | +0 | – | 0.023 | -0.005 | 2992 | 0.102 | 0.023 |
| slow_noisy | imm | tracker | 0.028 | -0.008 | -2 | – | 0.028 | -0.008 | 2970 | 0.106 | 0.028 |
| slow_noisy | imm+pred | tracker | 0.023 | -0.006 | -0 | – | 0.023 | -0.006 | 2981 | 0.104 | 0.023 |
| fast | residual | residual | 2.202 | +0.378 | -112 | – | 2.202 | +0.378 | 549 | 2.202 | – |
| fast | kf | tracker | 0.725 | -0.056 | +7 | – | 0.725 | -0.056 | 561 | 2.114 | 0.725 |
| fast | imm | tracker | 0.640 | -0.063 | +8 | – | 0.640 | -0.063 | 550 | 2.025 | 0.640 |
| fast | imm+pred | tracker | 0.542 | -0.058 | -2 | – | 0.542 | -0.058 | 523 | 1.992 | 0.542 |
| fast_seen_early | residual | residual | 2.182 | +0.158 | -117 | – | 2.182 | +0.158 | 550 | 2.182 | – |
| fast_seen_early | kf | tracker | 0.653 | -0.097 | +2 | – | 0.653 | -0.097 | 561 | 2.246 | 0.653 |
| fast_seen_early | imm | tracker | 0.706 | -0.089 | +8 | – | 0.706 | -0.089 | 582 | 2.033 | 0.706 |
| fast_seen_early | imm+pred | tracker | 0.636 | -0.081 | +2 | – | 0.636 | -0.081 | 552 | 2.056 | 0.636 |
| ball | residual | residual | 3.303 | +0.224 | -118 | – | 3.303 | +0.224 | 341 | 3.303 | – |
| ball | kf | tracker | 0.669 | -0.199 | +1 | – | 0.669 | -0.199 | 363 | 2.996 | 0.669 |
| ball | imm | tracker | 0.797 | -0.238 | +2 | – | 0.797 | -0.238 | 363 | 3.180 | 0.797 |
| ball | imm+pred | tracker | 0.570 | -0.090 | +1 | – | 0.710 | -0.144 | 338 | 3.234 | 0.570 |
| ball_slow | residual | residual | 0.587 | -0.008 | -88 | – | 0.587 | -0.008 | 1034 | 0.587 | – |
| ball_slow | kf | tracker | 0.185 | -0.030 | -2 | – | 0.185 | -0.030 | 1023 | 0.537 | 0.185 |
| ball_slow | imm | tracker | 0.209 | -0.043 | -3 | – | 0.209 | -0.043 | 1034 | 0.543 | 0.209 |
| ball_slow | imm+pred | tracker | 0.277 | -0.040 | -1 | – | 0.277 | -0.040 | 823 | 1.070 | 0.277 |
| wedge | residual | residual | 0.017 | -0.004 | – | 0.001 | 0.017 | -0.004 | 6360 | 0.017 | – |
| wedge | kf | tracker | 0.015 | +0.007 | – | 0.012 | 0.015 | +0.007 | 6360 | 0.027 | 0.015 |
| wedge | imm | tracker | 0.016 | +0.005 | – | 0.009 | 0.016 | +0.005 | 6330 | 0.029 | 0.016 |
| wedge | imm+pred | tracker | 0.017 | +0.007 | – | 0.032 | 0.017 | +0.007 | 6340 | 0.028 | 0.017 |
| pentagon_on_path | residual | residual | 0.057 | -0.004 | – | – | 0.057 | -0.004 | 21970 | 0.057 | – |
| pentagon_on_path | kf | tracker | 0.005 | -0.000 | – | – | 0.005 | -0.000 | 22140 | 0.038 | 0.005 |
| pentagon_on_path | imm | tracker | 0.008 | -0.001 | – | – | 0.008 | -0.001 | 22266 | 0.036 | 0.008 |
| pentagon_on_path | imm+pred | tracker | 0.008 | -0.001 | – | – | 0.008 | -0.001 | 22092 | 0.039 | 0.008 |
| pentagon_beside_path | residual | residual | 0.052 | -0.003 | – | – | 0.052 | -0.003 | 22771 | 0.052 | – |
| pentagon_beside_path | kf | tracker | 0.024 | +0.000 | – | – | 0.024 | +0.000 | 22148 | 0.052 | 0.024 |
| pentagon_beside_path | imm | tracker | 0.025 | -0.000 | – | – | 0.025 | -0.000 | 22050 | 0.053 | 0.025 |
| pentagon_beside_path | imm+pred | tracker | 0.025 | -0.000 | – | – | 0.025 | -0.000 | 21862 | 0.053 | 0.025 |
| pentagon_crossing | residual | residual | 0.089 | -0.022 | -80 | – | 0.089 | -0.022 | 2090 | 0.089 | – |
| pentagon_crossing | kf | tracker | 0.031 | -0.008 | -1 | – | 0.031 | -0.008 | 1800 | 0.098 | 0.031 |
| pentagon_crossing | imm | tracker | 0.032 | -0.009 | -2 | – | 0.032 | -0.009 | 2070 | 0.091 | 0.032 |
| pentagon_crossing | imm+pred | tracker | 0.032 | -0.009 | -2 | – | 0.032 | -0.009 | 2080 | 0.089 | 0.032 |
| pentagon_pursuer | residual | residual | 0.109 | -0.005 | -16 | – | 0.109 | -0.005 | 5808 | 0.109 | – |
| pentagon_pursuer | kf | tracker | 0.044 | -0.008 | +0 | – | 0.044 | -0.008 | 5106 | 0.101 | 0.044 |
| pentagon_pursuer | imm | tracker | 0.048 | -0.010 | -0 | – | 0.048 | -0.010 | 5041 | 0.096 | 0.048 |
| pentagon_pursuer | imm+pred | tracker | 0.064 | -0.018 | +0 | – | 0.064 | -0.018 | 5204 | 0.094 | 0.064 |
| throw_a_v0.5 | residual | residual | 0.135 | -0.016 | -65 | – | 0.135 | -0.016 | 2156 | 0.135 | – |
| throw_a_v0.5 | kf | tracker | 0.073 | -0.008 | -4 | – | 0.073 | -0.008 | 2156 | 0.117 | 0.073 |
| throw_a_v0.5 | imm+pred | tracker | 0.083 | -0.014 | -5 | – | 0.083 | -0.014 | 2156 | 0.126 | 0.083 |
| throw_a_v1 | residual | residual | 0.303 | -0.065 | -93 | – | 0.303 | -0.065 | 1331 | 0.303 | – |
| throw_a_v1 | kf | tracker | 0.113 | -0.023 | -4 | – | 0.113 | -0.023 | 1331 | 0.300 | 0.113 |
| throw_a_v1 | imm+pred | tracker | 0.119 | -0.029 | -4 | – | 0.119 | -0.029 | 1331 | 0.285 | 0.119 |
| throw_a_v1.5 | residual | residual | 0.576 | -0.002 | -83 | – | 0.576 | -0.002 | 1023 | 0.576 | – |
| throw_a_v1.5 | kf | tracker | 0.160 | -0.029 | -2 | – | 0.160 | -0.029 | 1045 | 0.528 | 0.160 |
| throw_a_v1.5 | imm+pred | tracker | 0.247 | -0.057 | +0 | – | 0.247 | -0.057 | 819 | 0.583 | 0.247 |
| throw_a_v2 | residual | residual | 1.529 | +0.381 | -125 | – | 1.529 | +0.381 | 704 | 1.529 | – |
| throw_a_v2 | kf | tracker | 0.341 | -0.047 | -0 | – | 0.341 | -0.047 | 748 | 1.323 | 0.341 |
| throw_a_v2 | imm+pred | tracker | 0.395 | -0.077 | +0 | – | 0.395 | -0.077 | 710 | 1.612 | 0.395 |
| throw_a_v3 | residual | residual | 2.514 | +0.443 | -120 | – | 2.514 | +0.443 | 462 | 2.514 | – |
| throw_a_v3 | kf | tracker | 0.475 | +0.018 | +3 | – | 0.475 | +0.018 | 484 | 2.206 | 0.475 |
| throw_a_v3 | imm+pred | tracker | 0.649 | -0.080 | +8 | – | 0.653 | -0.074 | 476 | 2.599 | 0.649 |
| throw_a_v4 | residual | residual | 3.331 | +0.180 | -118 | – | 3.331 | +0.180 | 330 | 3.331 | – |
| throw_a_v4 | kf | tracker | 0.677 | -0.182 | +2 | – | 0.677 | -0.182 | 341 | 2.875 | 0.677 |
| throw_a_v4 | imm+pred | tracker | – | – | – | – | 0.567 | -0.197 | 308 | – | – |
| throw_a_v5 | residual | residual | – | – | – | – | 3.889 | +0.056 | 253 | – | – |
| throw_a_v5 | kf | tracker | – | – | – | – | 1.037 | +0.067 | 264 | – | – |
| throw_a_v5 | imm+pred | tracker | – | – | – | – | 1.069 | +0.033 | 254 | – | – |
| throw_a_v6 | residual | residual | – | – | – | – | 4.226 | -0.423 | 209 | – | – |
| throw_a_v6 | kf | tracker | – | – | – | – | 0.651 | -0.220 | 187 | – | – |
| throw_a_v6 | imm+pred | tracker | – | – | – | – | 0.633 | -0.215 | 191 | – | – |
| throw_ballistic_v2 | residual | residual | – | – | – | – | 4.037 | +1.395 | 286 | – | – |
| throw_ballistic_v2 | kf | tracker | – | – | – | – | 1.508 | +0.307 | 280 | – | – |
| throw_ballistic_v2 | imm | tracker | – | – | – | – | 1.612 | +0.236 | 280 | – | – |
| throw_ballistic_v2 | imm+pred | tracker | – | – | – | – | 1.513 | +0.330 | 274 | – | – |
| throw_ballistic_v3 | residual | residual | 4.031 | +1.306 | -77 | – | 4.031 | +1.306 | 352 | 4.031 | – |
| throw_ballistic_v3 | kf | tracker | 0.916 | -0.075 | +5 | – | 0.916 | -0.075 | 380 | 4.020 | 0.916 |
| throw_ballistic_v3 | imm | tracker | 1.065 | +0.111 | +3 | – | 1.065 | +0.111 | 374 | 3.770 | 1.065 |
| throw_ballistic_v3 | imm+pred | tracker | 0.628 | -0.177 | +2 | – | 0.616 | -0.171 | 364 | 3.919 | 0.628 |
| throw_ballistic_v4 | residual | residual | – | – | – | – | 4.082 | +0.971 | 319 | – | – |
| throw_ballistic_v4 | kf | tracker | 0.754 | -0.197 | +2 | – | 0.754 | -0.197 | 330 | 3.654 | 0.754 |
| throw_ballistic_v4 | imm | tracker | 0.630 | -0.183 | +1 | – | 0.630 | -0.183 | 330 | 3.666 | 0.630 |
| throw_ballistic_v4 | imm+pred | tracker | 0.578 | -0.172 | +1 | – | 0.575 | -0.168 | 318 | 3.734 | 0.578 |
| throw_ballistic_v5 | residual | residual | – | – | – | – | 4.285 | +0.786 | 275 | – | – |
| throw_ballistic_v5 | kf | tracker | – | – | – | – | 0.991 | -0.258 | 275 | – | – |
| throw_ballistic_v5 | imm | tracker | – | – | – | – | 1.093 | -0.061 | 280 | – | – |
| throw_ballistic_v5 | imm+pred | tracker | – | – | – | – | 0.882 | -0.035 | 276 | – | – |
| throw_ballistic_v6 | residual | residual | – | – | – | – | 4.816 | +0.551 | 226 | – | – |
| throw_ballistic_v6 | kf | tracker | – | – | – | – | 1.688 | +0.073 | 226 | – | – |
| throw_ballistic_v6 | imm | tracker | – | – | – | – | 1.186 | -0.123 | 226 | – | – |
| throw_ballistic_v6 | imm+pred | tracker | – | – | – | – | 1.428 | -0.524 | 224 | – | – |

#### B.2 Tracks: confirmation, identity, σ calibration (SIM)

σ = sqrt(n̂ᵀP_vv n̂) from the message; share of |v_trk − v_true| ≤ kσ on rows the CBF trusts (Gaussian 0.68 / 0.95 / 1.00).

| scenario | condition | first row → track [ms] | first row → trusted [ms] | track ids | id switches (Σ) | ≤1σ | ≤2σ | ≤3σ |
|---|---|---|---|---|---|---|---|---|
| static | kf | 11 | 253 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| static | imm | 11 | 253 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| static | imm+pred | 11 | 253 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| around | kf | 11 | 253 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| around | imm | 11 | 253 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| around | imm+pred | 11 | 253 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| static_noisy | kf | 11 | 253 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| static_noisy | imm | 11 | 253 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| static_noisy | imm+pred | 11 | 253 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| approach_hold | kf | 11 | 253 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| approach_hold | imm | 11 | 253 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| approach_hold | imm+pred | 11 | 253 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| slow | kf | 11 | 253 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| slow | imm | 11 | 253 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| slow | imm+pred | 11 | 253 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| slow_noisy | kf | 11 | 253 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| slow_noisy | imm | 11 | 253 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| slow_noisy | imm+pred | 11 | 253 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| fast | kf | 11 | 594 | 2 | 3 | 0.36 | 0.77 | 0.95 |
| fast | imm | 11 | 594 | 2 | 2 | 0.33 | 0.61 | 0.85 |
| fast | imm+pred | 22 | 605 | 2 | 3 | 0.33 | 0.52 | 0.79 |
| fast_seen_early | kf | 33 | 627 | 2 | 3 | 0.33 | 0.61 | 0.88 |
| fast_seen_early | imm | 33 | 616 | 2 | 3 | 0.27 | 0.59 | 0.91 |
| fast_seen_early | imm+pred | 22 | 605 | 2 | 2 | 0.32 | 0.64 | 0.91 |
| ball | kf | 11 | 341 | 1 | 0 | 0.75 | 0.95 | 1.00 |
| ball | imm | 22 | 341 | 1 | 0 | 0.83 | 0.95 | 1.00 |
| ball | imm+pred | 11 | 330 | 1 | 0 | 0.75 | 0.95 | 1.00 |
| ball_slow | kf | 22 | 264 | 1 | 0 | 0.89 | 1.00 | 1.00 |
| ball_slow | imm | 33 | 275 | 1 | 0 | 0.90 | 1.00 | 1.00 |
| ball_slow | imm+pred | 22 | 264 | 2 | 3 | 0.76 | 1.00 | 1.00 |
| wedge | kf | 11 | 253 | 2 | 0 | 1.00 | 1.00 | 1.00 |
| wedge | imm | 11 | 253 | 2 | 0 | 1.00 | 1.00 | 1.00 |
| wedge | imm+pred | 11 | 253 | 2 | 0 | 1.00 | 1.00 | 1.00 |
| pentagon_on_path | kf | 22 | 264 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| pentagon_on_path | imm | 22 | 264 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| pentagon_on_path | imm+pred | 22 | 280 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| pentagon_beside_path | kf | 22 | 264 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| pentagon_beside_path | imm | 22 | 264 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| pentagon_beside_path | imm+pred | 22 | 280 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| pentagon_crossing | kf | 0 | 187 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| pentagon_crossing | imm | 0 | 220 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| pentagon_crossing | imm+pred | 0 | 226 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| pentagon_pursuer | kf | 22 | 264 | 1 | 0 | 0.99 | 1.00 | 1.00 |
| pentagon_pursuer | imm | 27 | 286 | 1 | 0 | 0.99 | 1.00 | 1.00 |
| pentagon_pursuer | imm+pred | 16 | 259 | 2 | 1 | 0.99 | 1.00 | 1.00 |
| throw_a_v0.5 | kf | 44 | 286 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| throw_a_v0.5 | imm+pred | 55 | 297 | 1 | 0 | 0.98 | 1.00 | 1.00 |
| throw_a_v1 | kf | 33 | 275 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| throw_a_v1 | imm+pred | 33 | 275 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| throw_a_v1.5 | kf | 22 | 264 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| throw_a_v1.5 | imm+pred | 22 | 264 | 2 | 1 | 0.98 | 1.00 | 1.00 |
| throw_a_v2 | kf | 22 | 264 | 2 | 1 | 0.98 | 1.00 | 1.00 |
| throw_a_v2 | imm+pred | 22 | 264 | 2 | 1 | 1.00 | 1.00 | 1.00 |
| throw_a_v3 | kf | 11 | 374 | 1 | 0 | 0.96 | 1.00 | 1.00 |
| throw_a_v3 | imm+pred | 22 | 385 | 1 | 0 | 0.95 | 1.00 | 1.00 |
| throw_a_v4 | kf | 11 | 341 | 1 | 0 | 0.72 | 1.00 | 1.00 |
| throw_a_v4 | imm+pred | 22 | 374 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| throw_a_v5 | kf | 11 | 319 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| throw_a_v5 | imm+pred | 11 | 319 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| throw_a_v6 | kf | 11 | – | 1 | 0 | – | – | – |
| throw_a_v6 | imm+pred | 11 | – | 1 | 0 | – | – | – |
| throw_ballistic_v2 | kf | 22 | 352 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| throw_ballistic_v2 | imm | 22 | – | 1 | 0 | – | – | – |
| throw_ballistic_v2 | imm+pred | 22 | 352 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| throw_ballistic_v3 | kf | 16 | 363 | 1 | 0 | 0.87 | 1.00 | 1.00 |
| throw_ballistic_v3 | imm | 11 | 352 | 1 | 0 | 0.96 | 1.00 | 1.00 |
| throw_ballistic_v3 | imm+pred | 22 | 368 | 1 | 0 | 0.97 | 1.00 | 1.00 |
| throw_ballistic_v4 | kf | 17 | 346 | 1 | 0 | 0.88 | 1.00 | 1.00 |
| throw_ballistic_v4 | imm | 22 | 341 | 1 | 0 | 0.56 | 1.00 | 1.00 |
| throw_ballistic_v4 | imm+pred | 16 | 341 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| throw_ballistic_v5 | kf | 16 | 330 | 1 | 0 | 0.45 | 1.00 | 1.00 |
| throw_ballistic_v5 | imm | 11 | 319 | 1 | 0 | 0.84 | 1.00 | 1.00 |
| throw_ballistic_v5 | imm+pred | 17 | 324 | 1 | 0 | 1.00 | 1.00 | 1.00 |
| throw_ballistic_v6 | kf | 11 | – | 1 | 0 | – | – | – |
| throw_ballistic_v6 | imm | 11 | – | 1 | 0 | – | – | – |
| throw_ballistic_v6 | imm+pred | 22 | – | 1 | 0 | – | – | – |

### C. Closed loop, full chain (SIM, Phase 5.2)

min h from the TRUE gaps of the real control-point set (h = gap − d_safe, contact = gap < 0); slack per family from the CBFDIAG lines (sampled at diag_period_s); fallback = QP ticks at box-only / brake in the episode; causes = approach_probe.py causes over ball_throw_eval.py score passes of the moving obstacles (residual runs: rows and CBF lead only, approach_probe's track causes do not apply with tracking off).

| scenario | condition | N | min h [m] | h<0 [s] | contact | task done | slack obs | slack cap | fallback box/brake | escapes | jerk p99 | reflex trips | RTF | QP Hz | causes (passes) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| static | residual | 3 | +0.033 [+0.032, +0.034] | 0.00 | 0/3 | 3/3 | 0.00 | 0.00 | 0/0 | 0 | 167 | 0 | 0.90 | 265 | – |
| static | kf | 3 | +0.059 [+0.059, +0.060] | 0.00 | 0/3 | 3/3 | 0.10 | 0.85 | 0/0 | 0 | 253 | 0 | 0.85 | 253 | – |
| static | imm | 3 | +0.060 [+0.058, +0.060] | 0.00 | 0/3 | 3/3 | 0.15 | 1.29 | 0/0 | 0 | 284 | 0 | 0.84 | 255 | – |
| static | imm+pred | 3 | +0.059 [+0.058, +0.059] | 0.00 | 0/3 | 3/3 | 0.15 | 1.28 | 0/0 | 0 | 279 | 0 | 0.82 | 263 | – |
| around | residual | 3 | +0.016 [+0.014, +0.017] | 0.00 | 0/3 | 0/3 | 2.03 | 0.54 | 0/0 | 6 | 239 | 0 | 0.90 | 259 | – |
| around | kf | 3 | +0.016 [+0.012, +0.017] | 0.00 | 0/3 | 0/3 | 2.03 | 2.07 | 0/0 | 6 | 289 | 0 | 0.75 | 267 | – |
| around | imm | 3 | +0.014 [+0.013, +0.016] | 0.00 | 0/3 | 0/3 | 2.07 | 2.27 | 0/0 | 6 | 290 | 0 | 0.83 | 261 | – |
| around | imm+pred | 3 | +0.014 [+0.013, +0.016] | 0.00 | 0/3 | 0/3 | 2.07 | 2.26 | 0/0 | 6 | 297 | 0 | 0.83 | 260 | – |
| static_noisy | residual | 3 | +0.102 [+0.102, +0.102] | 0.00 | 0/3 | 3/3 | 0.00 | 0.00 | 0/0 | 0 | 1 | 0 | 0.86 | 268 | – |
| static_noisy | kf | 3 | +0.102 [+0.102, +0.102] | 0.00 | 0/3 | 3/3 | 0.06 | 0.60 | 0/0 | 0 | 33 | 0 | 0.91 | 260 | – |
| static_noisy | imm | 3 | +0.102 [+0.102, +0.102] | 0.00 | 0/3 | 3/3 | 0.05 | 0.43 | 0/0 | 0 | 31 | 0 | 0.87 | 258 | – |
| static_noisy | imm+pred | 3 | +0.102 [+0.102, +0.102] | 0.00 | 0/3 | 3/3 | 0.05 | 0.44 | 0/0 | 0 | 33 | 0 | 0.91 | 255 | – |
| approach_hold | residual | 3 | +0.004 [+0.001, +0.010] | 0.00 | 0/3 | 0/3 | 10.80 | 2.74 | 0/0 | 0 | 440 | 0 | 0.76 | 255 | – |
| approach_hold | kf | 3 | +0.053 [+0.052, +0.054] | 0.00 | 0/3 | 0/3 | 15.82 | 1.79 | 0/0 | 0 | 440 | 0 | 0.91 | 255 | – |
| approach_hold | imm | 3 | +0.054 [+0.053, +0.056] | 0.00 | 0/3 | 0/3 | 4.04 | 1.59 | 0/0 | 0 | 437 | 0 | 0.70 | 254 | – |
| approach_hold | imm+pred | 3 | +0.055 [+0.054, +0.055] | 0.00 | 0/3 | 0/3 | 4.07 | 0.78 | 0/0 | 0 | 440 | 0 | 0.91 | 254 | – |
| slow | residual | 3 | +0.074 [+0.070, +0.088] | 0.00 | 0/3 | 0/3 | 0.57 | 0.66 | 0/0 | 0 | 401 | 0 | 0.69 | 260 | ok 3 |
| slow | kf | 3 | +0.127 [+0.126, +0.133] | 0.00 | 0/3 | 0/3 | 0.39 | 0.56 | 0/0 | 0 | 395 | 0 | 0.93 | 260 | ok 3 |
| slow | imm | 3 | +0.130 [+0.129, +0.131] | 0.00 | 0/3 | 0/3 | 0.23 | 0.58 | 0/0 | 0 | 400 | 0 | 0.70 | 244 | ok 3 |
| slow | imm+pred | 3 | +0.132 [+0.129, +0.133] | 0.00 | 0/3 | 0/3 | 0.44 | 0.88 | 0/0 | 0 | 389 | 0 | 0.94 | 258 | ok 3 |
| slow_noisy | residual | 3 | +0.100 [+0.091, +0.102] | 0.00 | 0/3 | 0/3 | 0.42 | 0.97 | 0/0 | 0 | 408 | 0 | 0.85 | 252 | ok 3 |
| slow_noisy | kf | 3 | +0.138 [+0.130, +0.150] | 0.00 | 0/3 | 0/3 | 0.32 | 0.33 | 0/0 | 0 | 402 | 0 | 0.92 | 256 | ok 3 |
| slow_noisy | imm | 3 | +0.136 [+0.134, +0.146] | 0.00 | 0/3 | 0/3 | 0.12 | 0.29 | 0/0 | 0 | 402 | 0 | 0.68 | 255 | ok 3 |
| slow_noisy | imm+pred | 3 | +0.137 [+0.131, +0.145] | 0.00 | 0/3 | 0/3 | 0.31 | 0.25 | 0/0 | 0 | 407 | 0 | 0.91 | 259 | ok 3 |
| fast | residual | 3 | -0.223 [-0.241, -0.216] | 0.21 | 3/3 | 1/3 | 17.79 | 3.21 | 0/0 | 0 | 440 | 0 | 0.82 | 281 | late 6 |
| fast | kf | 3 | -0.224 [-0.226, -0.213] | 0.23 | 3/3 | 0/3 | 35.78 | 1.92 | 0/0 | 0 | 440 | 0 | 0.85 | 281 | late 6 |
| fast | imm | 3 | -0.222 [-0.223, -0.211] | 0.23 | 3/3 | 0/3 | 32.41 | 1.87 | 0/0 | 0 | 441 | 0 | 0.86 | 266 | late 6 |
| fast | imm+pred | 3 | -0.223 [-0.234, -0.222] | 0.22 | 3/3 | 0/3 | 34.41 | 2.35 | 0/0 | 0 | 439 | 0 | 0.84 | 262 | late 7 |
| fast_seen_early | residual | 3 | -0.240 [-0.245, -0.230] | 0.20 | 3/3 | 0/3 | 14.92 | 2.53 | 0/0 | 0 | 440 | 0 | 0.85 | 280 | late 3 |
| fast_seen_early | kf | 3 | -0.219 [-0.222, -0.217] | 0.24 | 3/3 | 0/3 | 43.29 | 2.45 | 0/0 | 0 | 438 | 0 | 0.82 | 270 | late 3 |
| fast_seen_early | imm | 3 | -0.214 [-0.217, -0.213] | 0.24 | 3/3 | 0/3 | 40.02 | 3.12 | 0/0 | 0 | 440 | 0 | 0.83 | 270 | late 3 |
| fast_seen_early | imm+pred | 3 | -0.214 [-0.220, -0.213] | 0.23 | 3/3 | 0/3 | 45.19 | 3.42 | 0/0 | 0 | 438 | 0 | 0.82 | 255 | late 3 |
| ball | residual | 3 | -0.214 [-0.214, -0.214] | 0.12 | 3/3 | 3/3 | 1.08 | 0.99 | 0/0 | 0 | 404 | 0 | 0.84 | 286 | late 6 |
| ball | kf | 3 | -0.195 [-0.199, -0.193] | 0.14 | 3/3 | 0/3 | 56.97 | 2.28 | 0/0 | 0 | 441 | 0 | 0.83 | 288 | late 6 |
| ball | imm | 3 | -0.201 [-0.209, -0.197] | 0.13 | 3/3 | 0/3 | 39.46 | 1.76 | 0/0 | 0 | 441 | 0 | 0.82 | 284 | late 6 |
| ball | imm+pred | 3 | -0.209 [-0.209, -0.204] | 0.13 | 3/3 | 0/3 | 24.86 | 2.64 | 0/0 | 0 | 439 | 0 | 0.83 | 280 | late 6 |
| ball_slow | residual | 3 | -0.038 [-0.039, -0.028] | 0.20 | 0/3 | 0/3 | 7.22 | 4.37 | 0/0 | 0 | 438 | 0 | 0.83 | 276 | late 3 |
| ball_slow | kf | 3 | +0.004 [-0.014, +0.006] | 0.00 | 0/3 | 0/3 | 10.45 | 2.65 | 0/0 | 0 | 408 | 0 | 0.83 | 278 | ok 2 late 1 |
| ball_slow | imm | 3 | +0.008 [-0.002, +0.021] | 0.00 | 0/3 | 0/3 | 9.37 | 2.24 | 0/0 | 0 | 414 | 0 | 0.83 | 268 | ok 2 late 1 |
| ball_slow | imm+pred | 3 | -0.127 [-0.154, -0.108] | 0.34 | 1/3 | 0/3 | 19.54 | 3.38 | 0/0 | 0 | 435 | 0 | 0.81 | 272 | ok 5 late 1 |
| wedge | residual | 3 | -0.001 [-0.001, -0.000] | 0.08 | 0/3 | 0/3 | 1.20 | 0.36 | 0/0 | 4 | 236 | 0 | 0.81 | 238 | – |
| wedge | kf | 3 | -0.001 [-0.001, -0.001] | 0.08 | 0/3 | 0/3 | 1.55 | 1.68 | 0/0 | 6 | 232 | 0 | 0.81 | 226 | – |
| wedge | imm | 3 | -0.000 [-0.000, -0.000] | 0.07 | 0/3 | 0/3 | 1.48 | 1.62 | 0/0 | 6 | 235 | 0 | 0.81 | 238 | – |
| wedge | imm+pred | 3 | -0.001 [-0.002, -0.000] | 0.08 | 0/3 | 0/3 | 1.51 | 1.64 | 0/0 | 6 | 246 | 0 | 0.82 | 230 | – |

### D. Three columns: offline / Gazebo + emulated perception / Gazebo full chain (SIM, Phase 5.3)

off = cbf_scenarios.py (double integrator, emulated perception) at 30 fps as shipped and at 60 fps, the highest rate its one-frame-in-flight delivery admits with t_cam = 13 ms (at 90 fps it delivers nothing). emul = the Gazebo plant and the whole stack with cbf_scenarios' perception emulation (emulated_perception) at 30 fps (seed 0: same rate as off@30, so the gap is the plant) and at 90 fps (the simulated chain's rate, so emul@90 → full is the perception). full = simulated D455 → real_time_distance → tracker (kf for the tracker source). Counts are over seeds.

| scenario | v_obs source | min h off@30 | min h emul@30 | min h off@60 | min h emul@90 | min h full | h<0 off@30 | h<0 emul@30 | h<0 emul@90 | h<0 full | contact off@30 | contact emul@30 | contact emul@90 | contact full | task off@30 | task emul@30 | task emul@90 | task full | causes emul@90 | causes full |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| static | residual | +0.052 | +0.049 | +0.052 | +0.050 | +0.033 | 0.00 | 0.00 | 0.00 | 0.00 | 0/1 | 0/1 | 0/3 | 0/3 | 1/1 | 1/1 | 3/3 | 3/3 | – | – |
| static | tracker | +0.052 | +0.049 | +0.052 | +0.051 | +0.059 | 0.00 | 0.00 | 0.00 | 0.00 | 0/1 | 0/1 | 0/3 | 0/3 | 1/1 | 1/1 | 3/3 | 3/3 | – | – |
| around | residual | +0.015 | +0.008 | +0.015 | +0.015 | +0.016 | 0.00 | 0.00 | 0.00 | 0.00 | 0/1 | 0/1 | 0/3 | 0/3 | 0/1 | 0/1 | 0/3 | 0/3 | – | – |
| around | tracker | +0.015 | +0.014 | +0.015 | +0.017 | +0.016 | 0.00 | 0.00 | 0.00 | 0.00 | 0/1 | 0/1 | 0/3 | 0/3 | 0/1 | 0/1 | 0/3 | 0/3 | – | – |
| static_noisy | residual | +0.100 | +0.102 | +0.100 | +0.102 | +0.102 | 0.00 | 0.00 | 0.00 | 0.00 | 0/1 | 0/1 | 0/3 | 0/3 | 0/1 | 1/1 | 0/3 | 3/3 | – | – |
| static_noisy | tracker | +0.100 | +0.102 | +0.100 | +0.102 | +0.102 | 0.00 | 0.00 | 0.00 | 0.00 | 0/1 | 0/1 | 0/3 | 0/3 | 0/1 | 0/1 | 1/3 | 3/3 | – | – |
| approach_hold | residual | +0.043 | +0.037 | +0.043 | +0.036 | +0.004 | 0.00 | 0.00 | 0.00 | 0.00 | 0/1 | 0/1 | 0/3 | 0/3 | 0/1 | 0/1 | 0/3 | 0/3 | – | – |
| approach_hold | tracker | +0.043 | +0.036 | +0.043 | +0.036 | +0.053 | 0.00 | 0.00 | 0.00 | 0.00 | 0/1 | 0/1 | 0/3 | 0/3 | 0/1 | 0/1 | 0/3 | 0/3 | – | – |
| slow | residual | +0.136 | +0.127 | +0.137 | +0.130 | +0.074 | 0.00 | 0.00 | 0.00 | 0.00 | 0/1 | 0/1 | 0/3 | 0/3 | 0/1 | 0/1 | 0/3 | 0/3 | ok 3 | ok 3 |
| slow | tracker | +0.136 | +0.127 | +0.137 | +0.129 | +0.127 | 0.00 | 0.00 | 0.00 | 0.00 | 0/1 | 0/1 | 0/3 | 0/3 | 0/1 | 0/1 | 0/3 | 0/3 | ok 3 | ok 3 |
| slow_noisy | residual | +0.149 | +0.172 | +0.168 | +0.166 | +0.100 | 0.00 | 0.00 | 0.00 | 0.00 | 0/1 | 0/1 | 0/3 | 0/3 | 0/1 | 0/1 | 0/3 | 0/3 | ok 2 late 1 | ok 3 |
| slow_noisy | tracker | +0.155 | +0.176 | +0.170 | +0.169 | +0.138 | 0.00 | 0.00 | 0.00 | 0.00 | 0/1 | 0/1 | 0/3 | 0/3 | 0/1 | 0/1 | 0/3 | 0/3 | ok 3 | ok 3 |
| fast | residual | -0.163 | -0.232 | -0.167 | -0.225 | -0.223 | 0.21 | 0.20 | 0.25 | 0.21 | 1/1 | 1/1 | 3/3 | 3/3 | 0/1 | 0/1 | 0/3 | 1/3 | late 3 | late 6 |
| fast | tracker | -0.161 | -0.213 | -0.162 | -0.230 | -0.224 | 0.21 | 0.25 | 0.26 | 0.23 | 1/1 | 1/1 | 3/3 | 3/3 | 0/1 | 0/1 | 0/3 | 0/3 | late 3 | late 6 |
| fast_seen_early | residual | +0.022 | -0.200 | +0.032 | -0.059 | -0.240 | 0.00 | 0.23 | 0.13 | 0.20 | 0/1 | 1/1 | 0/3 | 3/3 | 0/1 | 0/1 | 0/3 | 0/3 | ok 2 late 1 | late 3 |
| fast_seen_early | tracker | +0.006 | -0.077 | +0.024 | -0.063 | -0.219 | 0.00 | 0.15 | 0.14 | 0.24 | 0/1 | 0/1 | 0/3 | 3/3 | 0/1 | 0/1 | 0/3 | 0/3 | ok 2 late 1 | late 3 |
| ball | residual | -0.202 | -0.208 | -0.197 | -0.198 | -0.214 | 0.15 | 0.13 | 0.14 | 0.12 | 1/1 | 1/1 | 3/3 | 3/3 | 0/1 | 0/1 | 0/3 | 3/3 | late 3 | late 6 |
| ball | tracker | -0.201 | -0.211 | -0.199 | -0.197 | -0.195 | 0.15 | 0.13 | 0.14 | 0.14 | 1/1 | 1/1 | 3/3 | 3/3 | 0/1 | 0/1 | 0/3 | 0/3 | late 3 | late 6 |
| ball_slow | residual | +0.135 | +0.015 | +0.134 | +0.053 | -0.038 | 0.00 | 0.00 | 0.00 | 0.20 | 0/1 | 0/1 | 0/3 | 0/3 | 0/1 | 0/1 | 0/3 | 0/3 | ok 3 | late 3 |
| ball_slow | tracker | +0.139 | -0.007 | +0.136 | +0.053 | +0.004 | 0.00 | 0.08 | 0.00 | 0.00 | 0/1 | 0/1 | 0/3 | 0/3 | 0/1 | 0/1 | 0/3 | 0/3 | ok 3 | ok 2 late 1 |
| wedge | residual | -0.013 | -0.038 | -0.014 | -0.037 | -0.001 | 4.46 | 5.75 | 5.98 | 0.08 | 0/1 | 0/1 | 0/3 | 0/3 | 0/1 | 0/1 | 0/3 | 0/3 | – | – |
| wedge | tracker | -0.013 | -0.022 | -0.013 | -0.016 | -0.001 | 3.59 | 2.52 | 3.37 | 0.08 | 0/1 | 0/1 | 0/3 | 0/3 | 0/1 | 0/1 | 0/3 | 0/3 | – | – |

### E. Pentagon task with obstacles (SIM)

| scenario | condition | N | EE err p95 [m] | min h [m] | contact | h<0 [s] | jerk p99 | escapes | reflex trips | RTF | causes |
|---|---|---|---|---|---|---|---|---|---|---|---|
| pentagon_clear | residual | 2 | 0.082 | – | 0/2 | 0.00 | 55 | 0 | 0 | 0.80 | – |
| pentagon_clear | kf | 2 | 0.101 | – | 0/2 | 0.00 | 59 | 0 | 0 | 0.79 | – |
| pentagon_clear | imm | 2 | 0.101 | – | 0/2 | 0.00 | 63 | 0 | 0 | 0.80 | – |
| pentagon_clear | imm+pred | 2 | 0.101 | – | 0/2 | 0.00 | 64 | 0 | 0 | 0.89 | – |
| pentagon_on_path | residual | 2 | 0.383 | -0.000 [-0.001, +0.001] | 0/2 | 0.05 | 245 | 0 | 0 | 0.78 | – |
| pentagon_on_path | kf | 2 | 0.434 | +0.071 [+0.071, +0.072] | 0/2 | 0.00 | 135 | 0 | 0 | 0.84 | – |
| pentagon_on_path | imm | 2 | 0.435 | +0.063 [+0.055, +0.071] | 0/2 | 0.00 | 103 | 0 | 0 | 0.79 | – |
| pentagon_on_path | imm+pred | 2 | 0.435 | +0.072 [+0.072, +0.073] | 0/2 | 0.00 | 173 | 0 | 0 | 0.81 | – |
| pentagon_beside_path | residual | 2 | 0.194 | -0.023 [-0.024, -0.023] | 0/2 | 0.95 | 257 | 0 | 0 | 0.87 | – |
| pentagon_beside_path | kf | 2 | 0.252 | -0.023 [-0.023, -0.023] | 0/2 | 0.30 | 204 | 0 | 0 | 0.77 | – |
| pentagon_beside_path | imm | 2 | 0.232 | -0.023 [-0.024, -0.023] | 0/2 | 0.29 | 228 | 0 | 0 | 0.82 | – |
| pentagon_beside_path | imm+pred | 2 | 0.233 | -0.024 [-0.024, -0.024] | 0/2 | 0.34 | 210 | 0 | 0 | 0.82 | – |
| pentagon_crossing | residual | 2 | 0.268 | -0.149 [-0.159, -0.140] | 1/2 | 1.22 | 242 | 0 | 0 | 0.80 | late 2 |
| pentagon_crossing | kf | 2 | 0.520 | -0.176 [-0.178, -0.173] | 2/2 | 1.43 | 271 | 0 | 0 | 0.88 | late 2 |
| pentagon_crossing | imm | 2 | 0.460 | -0.190 [-0.194, -0.187] | 2/2 | 1.61 | 271 | 0 | 0 | 0.88 | late 2 |
| pentagon_crossing | imm+pred | 2 | 0.491 | -0.190 [-0.193, -0.187] | 2/2 | 1.63 | 288 | 0 | 0 | 0.83 | late 2 |
| pentagon_pursuer | residual | 2 | 1.033 | -0.221 [-0.221, -0.220] | 2/2 | 3.48 | 356 | 0 | 0 | 0.86 | ok 2 |
| pentagon_pursuer | kf | 2 | 1.084 | -0.221 [-0.221, -0.220] | 2/2 | 3.53 | 326 | 0 | 0 | 0.85 | ok 2 |
| pentagon_pursuer | imm | 2 | 1.085 | -0.221 [-0.221, -0.220] | 2/2 | 3.54 | 297 | 0 | 0 | 0.85 | ok 2 |
| pentagon_pursuer | imm+pred | 2 | 1.050 | -0.217 [-0.220, -0.215] | 2/2 | 3.43 | 302 | 0 | 0 | 0.84 | ok 2 |

### F. Throws (SIM)

| scenario | condition | N | min h [m] | contact | row → track [ms] | row → trusted [ms] | v_obs RMS pooled [m/s] | causes (passes) |
|---|---|---|---|---|---|---|---|---|
| throw_a_v0.5 | residual | 1 | +0.194 | 0/1 | – | – | 0.13 | ok 1 |
| throw_a_v0.5 | kf | 1 | +0.242 | 0/1 | 44 | 286 | 0.07 | ok 1 late 1 |
| throw_a_v0.5 | imm+pred | 1 | +0.248 | 0/1 | 55 | 297 | 0.08 | ok 1 |
| throw_a_v1 | residual | 1 | +0.151 | 0/1 | – | – | 0.30 | ok 1 |
| throw_a_v1 | kf | 1 | +0.214 | 0/1 | 33 | 275 | 0.11 | ok 1 |
| throw_a_v1 | imm+pred | 1 | +0.225 | 0/1 | 33 | 275 | 0.12 | ok 1 |
| throw_a_v1.5 | residual | 1 | -0.052 | 0/1 | – | – | 0.58 | late 1 |
| throw_a_v1.5 | kf | 1 | +0.040 | 0/1 | 22 | 264 | 0.16 | ok 1 |
| throw_a_v1.5 | imm+pred | 1 | -0.043 | 0/1 | 22 | 264 | 0.25 | ok 1 late 1 |
| throw_a_v2 | residual | 1 | -0.181 | 1/1 | – | – | 1.53 | late 2 |
| throw_a_v2 | kf | 1 | -0.116 | 0/1 | 22 | 264 | 0.34 | ok 1 late 1 |
| throw_a_v2 | imm+pred | 1 | -0.131 | 0/1 | 22 | 264 | 0.40 | ok 1 late 1 |
| throw_a_v3 | residual | 1 | -0.191 | 1/1 | – | – | 2.51 | late 2 |
| throw_a_v3 | kf | 1 | -0.181 | 1/1 | 11 | 374 | 0.47 | late 2 |
| throw_a_v3 | imm+pred | 1 | -0.196 | 1/1 | 22 | 385 | 0.65 | late 2 |
| throw_a_v4 | residual | 1 | -0.199 | 1/1 | – | – | 3.33 | late 2 |
| throw_a_v4 | kf | 1 | -0.189 | 1/1 | 11 | 341 | 0.68 | late 2 |
| throw_a_v4 | imm+pred | 1 | -0.197 | 1/1 | 22 | 374 | 0.57 | late 2 |
| throw_a_v5 | residual | 1 | -0.200 | 1/1 | – | – | 3.89 | late 2 |
| throw_a_v5 | kf | 1 | -0.190 | 1/1 | 11 | 319 | 1.04 | late 2 |
| throw_a_v5 | imm+pred | 1 | -0.197 | 1/1 | 11 | 319 | 1.07 | late 2 |
| throw_a_v6 | residual | 1 | -0.200 | 1/1 | – | – | 4.23 | late 2 |
| throw_a_v6 | kf | 1 | -0.200 | 1/1 | 11 | – | 0.65 | late 2 |
| throw_a_v6 | imm+pred | 1 | -0.198 | 1/1 | 11 | – | 0.63 | late 2 |
| throw_ballistic_v2 | residual | 2 | -0.221 [-0.221, -0.220] | 2/2 | – | – | 4.04 | late 4 |
| throw_ballistic_v2 | kf | 2 | -0.215 [-0.217, -0.213] | 2/2 | 22 | 352 | 1.51 | late 4 |
| throw_ballistic_v2 | imm | 2 | -0.214 [-0.214, -0.214] | 2/2 | 22 | – | 1.61 | late 4 |
| throw_ballistic_v2 | imm+pred | 2 | -0.216 [-0.218, -0.214] | 2/2 | 22 | 352 | 1.51 | late 4 |
| throw_ballistic_v3 | residual | 2 | -0.209 [-0.212, -0.206] | 2/2 | – | – | 4.03 | late 4 |
| throw_ballistic_v3 | kf | 2 | -0.198 [-0.202, -0.194] | 2/2 | 16 | 363 | 0.92 | late 4 |
| throw_ballistic_v3 | imm | 2 | -0.200 [-0.201, -0.199] | 2/2 | 11 | 352 | 1.06 | late 4 |
| throw_ballistic_v3 | imm+pred | 2 | -0.202 [-0.203, -0.200] | 2/2 | 22 | 368 | 0.62 | late 4 |
| throw_ballistic_v4 | residual | 2 | -0.205 [-0.205, -0.205] | 2/2 | – | – | 4.08 | late 4 |
| throw_ballistic_v4 | kf | 2 | -0.206 [-0.207, -0.206] | 2/2 | 17 | 346 | 0.75 | late 4 |
| throw_ballistic_v4 | imm | 2 | -0.206 [-0.206, -0.205] | 2/2 | 22 | 341 | 0.63 | late 4 |
| throw_ballistic_v4 | imm+pred | 2 | -0.206 [-0.207, -0.205] | 2/2 | 16 | 341 | 0.57 | late 4 |
| throw_ballistic_v5 | residual | 2 | -0.203 [-0.203, -0.203] | 2/2 | – | – | 4.28 | late 4 |
| throw_ballistic_v5 | kf | 2 | -0.203 [-0.204, -0.203] | 2/2 | 16 | 330 | 0.99 | late 4 |
| throw_ballistic_v5 | imm | 2 | -0.203 [-0.203, -0.203] | 2/2 | 11 | 319 | 1.09 | late 4 |
| throw_ballistic_v5 | imm+pred | 2 | -0.203 [-0.203, -0.203] | 2/2 | 17 | 324 | 0.88 | late 4 |
| throw_ballistic_v6 | residual | 2 | -0.201 [-0.202, -0.201] | 2/2 | – | – | 4.82 | late 4 |
| throw_ballistic_v6 | kf | 2 | -0.202 [-0.202, -0.202] | 2/2 | 11 | – | 1.69 | late 4 |
| throw_ballistic_v6 | imm | 2 | -0.202 [-0.202, -0.201] | 2/2 | 11 | – | 1.19 | late 4 |
| throw_ballistic_v6 | imm+pred | 2 | -0.202 [-0.202, -0.201] | 2/2 | 22 | – | 1.43 | late 4 |

### G. kf with one track-consuming CBF flag off (SIM, seed 0, optional cross)

| flag | scenario | min h kf | min h flag | h<0 kf | h<0 flag | contact kf | contact flag | jerk p99 kf | jerk p99 flag | causes kf | causes flag |
|---|---|---|---|---|---|---|---|---|---|---|---|
| latency_compensation=False | ball | -0.193 | -0.200 | 0.14 | 0.13 | yes | yes | 441 | 440 | late 2 | late 2 |
| latency_compensation=False | ball_slow | -0.014 | +0.028 | 0.14 | 0.00 | no | no | 413 | 408 | ok 1 | ok 1 |
| latency_compensation=False | fast_seen_early | -0.219 | -0.221 | 0.25 | 0.24 | yes | yes | 408 | 440 | late 1 | late 1 |
| latency_compensation=False | pentagon_crossing | -0.178 | -0.204 | 1.68 | 1.37 | yes | yes | 238 | 229 | late 1 | late 1 |
| latency_compensation=False | slow | +0.133 | +0.092 | 0.00 | 0.00 | no | no | 292 | 371 | ok 1 | ok 1 |
| latency_compensation=False | slow_noisy | +0.130 | +0.104 | 0.00 | 0.00 | no | no | 413 | 404 | ok 1 | ok 1 |
| latency_compensation=False | throw_ballistic_v3 | -0.194 | -0.194 | 0.15 | 0.15 | yes | yes | 441 | 440 | late 2 | late 2 |
| uncertainty_margin=False | ball | -0.193 | -0.197 | 0.14 | 0.13 | yes | yes | 441 | 440 | late 2 | late 2 |
| uncertainty_margin=False | ball_slow | -0.014 | +0.034 | 0.14 | 0.00 | no | no | 413 | 408 | ok 1 | ok 1 |
| uncertainty_margin=False | fast_seen_early | -0.219 | -0.229 | 0.25 | 0.24 | yes | yes | 408 | 441 | late 1 | late 1 |
| uncertainty_margin=False | pentagon_crossing | -0.178 | -0.197 | 1.68 | 1.37 | yes | yes | 238 | 218 | late 1 | late 1 |
| uncertainty_margin=False | slow | +0.133 | +0.110 | 0.00 | 0.00 | no | no | 292 | 393 | ok 1 | ok 1 |
| uncertainty_margin=False | slow_noisy | +0.130 | +0.114 | 0.00 | 0.00 | no | no | 413 | 411 | ok 1 | ok 1 |
| uncertainty_margin=False | throw_ballistic_v3 | -0.194 | -0.203 | 0.15 | 0.15 | yes | yes | 441 | 440 | late 2 | late 2 |
| vobs_in_hdot=False | ball | -0.193 | -0.194 | 0.14 | 0.14 | yes | yes | 441 | 441 | late 2 | late 2 |
| vobs_in_hdot=False | ball_slow | -0.014 | +0.007 | 0.14 | 0.00 | no | no | 413 | 412 | ok 1 | late 1 |
| vobs_in_hdot=False | fast_seen_early | -0.219 | -0.236 | 0.25 | 0.23 | yes | yes | 408 | 434 | late 1 | late 1 |
| vobs_in_hdot=False | pentagon_crossing | -0.178 | -0.236 | 1.68 | 1.46 | yes | yes | 238 | 250 | late 1 | late 1 |
| vobs_in_hdot=False | slow | +0.133 | +0.125 | 0.00 | 0.00 | no | no | 292 | 390 | ok 1 | ok 1 |
| vobs_in_hdot=False | slow_noisy | +0.130 | +0.135 | 0.00 | 0.00 | no | no | 413 | 396 | ok 1 | ok 1 |
| vobs_in_hdot=False | throw_ballistic_v3 | -0.194 | -0.201 | 0.15 | 0.15 | yes | yes | 441 | 440 | late 2 | late 2 |

<!-- AUTO:END -->

## 4. Findings, with attribution (SIM)

**F1 — Plant fidelity (off@30 vs emul@30, same perception emulation and rate).** static, around,
static_noisy, approach_hold and slow agree within ~1 cm of min h, slow_noisy within 2.3 cm. The Gazebo plant costs
margin on the fast reactions: `fast` −0.16 → −0.21 / −0.23 m, `fast_seen_early` (tracker) +0.006 →
−0.077 m, `ball_slow` +0.139 → −0.007 m. What differs: real dynamics with torque and envelope limits
behind M·q̈ + C·q̇ (vs a double integrator), the achieved QP rate (264 vs 300 Hz), and the real
control-point set. These were not separated further.

**F2 — Perception (emul@90 vs full, same plant and rate).** For the tracker source the full chain
matches the emulation within ~2 cm on the static and slow scenarios (`slow` +0.129 vs +0.127), and
within 3 cm on `slow_noisy` (range noise on). For the
residual source it does not: `approach_hold` +0.036 → +0.004, `slow` +0.130 → +0.074, `slow_noisy`
+0.166 → +0.100. That is the residual estimator's lag and noise on real (simulated) depth: RMS 0.09–0.11
m/s vs 0.007–0.028 for the trackers on the same scenarios. The biggest perception gap is
**`fast_seen_early`: −0.063 → −0.219 m, contact 0/3 → 3/3.** The emulation sees the 8 cm ball from
the 1.5 m gate (rows 534–566 ms before closest approach). The full chain first has it in the rows only
301–332 ms before, and splits it into two tracks, which restarts the trust count (trusted ~630 ms after
the first row vs 267 ms). The CBF then acts 206–237 ms ahead instead of 445–458 ms.

**F3 — The dilation margin makes the full chain safer on static geometry.** `wedge`: h < 0 for
2.5–6 s offline and emulated, 0.08 s on the full chain (min h −0.001 vs −0.013 / −0.037). The engine
subtracts the mask margin (12 px body, 24 px EE) from every obstacle pixel, so the reported gap is ~5 cm
short at 1.6–2.1 m (plan §3.4), and the barrier holds the arm that much farther away.

**F4 — Pentagon: the per-lap ~10 cm excursion is the table, in sim.** `pentagon_clear` has no
obstacle and still has EE error p95 0.08–0.10 m (p50 4 mm). Over one lap the error rises only at the
bottom of the circle (desired z 0.20–0.25 m, held at 0.29–0.31 m). There the binding row is the
fingertip control point `fr3_link8#2` at a reported 0.20 m gap from the table top: the shipped config
keeps table and floor as obstacle pixels (`fr3_complete.yaml:436-446`). The singularity row binds at
the top of the lap but bends nothing (dnorm 0). Phase 2, in the bare scene, had p95 4.3 mm.
**Hypothesis for the open hardware item** (per-lap 10 cm excursion attributed to the singularity):
check the CBFDIAG `link=` at the excursion on the robot.

**F5 — Pentagon with obstacles.** `pentagon_on_path`: residual touches the barrier (min h 0.000),
trackers keep +0.06–0.07 m. `pentagon_crossing`: contact in 7 of 8 cells. The residual is no worse
(−0.149 vs −0.176 to −0.190), and every pass is "too late". `pentagon_pursuer` (0.3 m/s homing on the
flange for 12 s while the commander pulls back to the path): contact in all 8, h < 0 for ~3.5 s,
whatever the estimator. The task keeps the arm where the pursuer is going.

**F6 — Flags that consume the track (table G, one seed each: indicative).** Turning
`latency_compensation` off costs 4 cm on `slow` (+0.133 → +0.092) and 2.6 cm on `slow_noisy`, and
turning `uncertainty_margin` off costs 2 cm on both. `vobs_in_hdot` off changes little at 0.3 m/s and
costs 6 cm on `pentagon_crossing`. None changes a contact. `ball_slow` moves the other way for all three
(−0.014 → +0.007…+0.034), but its seed-to-seed spread is of the same order.

**F7 — Torque-rate clips are frequent and happen during evasions.** 8754 clips over 236 full-chain cells
and 11504 over 88 emulated cells, counted inside the episodes; 60 % on joint 2 (12520 of the 21003 in-episode clips of all 345
cells). libfranka would clip them, not
abort. The emulated column clips more because it reacts earlier and harder (exact gaps and velocities
from the first frame). Same root as plan §3.4: τ_ff steps at the QP rate.

**F8 — Fidelity of the run itself.** QP rate in sim time 220–290 Hz unpinned. Pinning the filter to an
isolated core gives 300 Hz (`filter_cpu`, measured 245 → 300 Hz in one running stack); the batch ran
unpinned. 73 % of full-chain cells (172 of 236) are below the plan's RTF 0.9 flag. Everything runs on sim time, so
a low RTF does not slow the stack, but CPU-bound stages run relatively faster than at RTF 1.

## 5. Parameter proposals (NOT applied)

None of these is applied; each needs a hardware check first.

1. **Publish gate vs speed.** At ≥ 2 m/s the 0.7 m `distance.thresholds.max_thresh` caps warning at
   0.7/v (≈ 230 ms at 3 m/s). A speed-aware gate (publish a tracked cluster's rows from
   v·t_needed when it is closing fast) is the lever; `fast_seen_early` shows what 1.5 m buys in
   emulation (no contact) and that the full chain does not get it yet (F2).
2. **Track continuity on fast passes.** The 8 cm ball at 3 m/s splits into two tracks in the full chain,
   and `frames_seen` restarts. Look at the association gate for fast clusters (the `ball_throw` bench
   tuning: stride, fast-track trust) before touching the CBF.
3. **σ calibration.** The velocity σ covers 100 % of the error at 1σ on slow obstacles. Fitting the
   tracker's process / measurement noise to the truth (this harness gives it per run) would shrink the
   uncertainty margin where it is over-conservative. The fast-pass overconfidence (0.27–0.36 at 1σ)
   goes with proposal 2.
4. **Table rows under the pentagon.** Either raise `path_center_xyz` z by ~0.1 m, or remove the table
   plane from the obstacle pixels (a perception change). Verify F4 on the robot first.
5. **τ_ff interpolation in the RT loop** (or a slew bound in N·m/s) for the torque-rate clips (F7,
   plan §3.4).
6. **For future sim batches:** run with `filter_cpu:=2` so the QP loop holds 300 Hz (F8).

## 6. What limits these numbers

* **Plant:** KDL g(q), DART friction 0.2 N·m per joint (the real arm's torque loop hides
  it), armature from the MuJoCo Menagerie FR3, no collision reflex, no torque-rate
  limiter (would-be clips are only counted), fingers rigid.
* **Timing:** everything runs on sim time. The QP loop reaches 220–290 Hz of a nominal 300
  (1 kHz `/clock` quantisation, dropped late ticks, a shared host); perception and QP run
  relatively FASTER when RTF < 1, which flatters CPU-bound stages.
* **Perception:** noise-free depth unless `range_noise`; the whole disc of a sphere is
  returned (H10); no D405, no colour, no people. The engine's dilation margin (12 px body,
  24 px EE) is real and biases reported gaps low by ~5 cm at 1.6–2.1 m (plan §3.4).
* **Geometry:** contact and h use the real control-point capsules (`define_control_points`),
  not `cbf_scenarios`' R_CAP set; obstacles are visual only.

## 7. Deviations from the spec

Listed with their reasons in the plan §9 (D1–D14).
