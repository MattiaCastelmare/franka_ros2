# Next steps — FR3 CBF simplification

*Updated 2026-11-04. The full analysis is in `../CBF_QP_SIMPLIFICATION_ANALYSIS.md` (section 8 has the roadmap).
Evidence is in `sim_runs/minimal_l1_slack_20261104/`, `sim_runs/ball_position_sweep_20261104/` and
`sim_runs/flacco_depth_space_20261104/` (see also `../FLACCO_DEPTH_SPACE_REPORT.md`).*

## Status of the roadmap

| # | Step | What it is | Status |
|---|---|---|---|
| 0 | **Fix perception** | Lost and optimistic detections, phantom obstacles near the fingertip, unmonitored links 0–3 (OPEN_ISSUES F4), stale normal (F5), track lost in the self-mask (S5), rig phantoms (S3) | **Not done.** Every experiment points here as the main problem. Flacco's depth-space gray area + finite pixel was tried and **rejected**: the robot became over-conservative and one run stalled. The code was removed |
| 1 | Use `minimal` as the base (Option A) | Start from the minimal HOCBF, not the full framework | ✓ done |
| 2 | Per-row ℓ1 obstacle slack | Each obstacle row gets its own linear-priced slack, so one bad row no longer loosens the others | ✓ done: condition `minimal_l1` (`obstacle_slack_mode: row_l1`). Helps with moving balls; no effect with fixed ones |
| 3 | Curvature term in ḧ | Add `‖v_rel,⊥‖²/ρ_c` to ḧ, so the arm is not pushed outward too much while sliding around an obstacle | not done |
| 4 | One constant perception margin | Replace the dynamic margins with one fixed margin sized from the measured perception error | not done |
| 5 | Task/null-space objective (from oscbf) | The QP corrects through redundant joint motion first, so the hand deviates less from its path | not done |
| 6 | Safe fallback | On QP failure, brake. Never drop the obstacle rows (today's box-only rung does) | not done |
| 7 | Gain tuning | Sweep (k₀, k₁) = (ω², 2ω), ω ∈ {5, 7, 10} | not done |

Steps 3–7 are added **one at a time**, each compared against the previous version.

## What the experiments showed (n = 1 run per cell)

**Margin violations with a fixed ball.** The arm gets 3.5–6 cm inside the 10 cm margin whenever the ball must be
avoided (positions P1, P2, P3, P5).
- The cause is **not** the slack: with ℓ1 the slack is 0 and the result is the same.
- Perception is optimistic by 1.4 cm on average, and by up to 22–30 cm when the ball is lost (P2, P3).
- The rest comes from delays: 30 Hz perception, 50 Hz geometry, and commanded vs realised acceleration.

**Unnecessary large detours.** With the ball truly 17–24 cm away (P6, P7), perception reported something within 10 cm
of the fingertip (`fr3_link8#2`) on 25–30 % of ticks. The arm deviated 35–40 cm from its path.

**Moving ball.** ℓ1 slack gave 15.5 cm clearance against 12.4 cm, with somewhat more path deviation.

## Recommended next step: the ground-truth test

About 5 minutes, no code changes. Re-run P1, P2, P6 and P7 with the **exact ball position from Gazebo** instead of the
camera (perception bypassed), CBF `minimal_l1`, DIAG lab, 100 Hz.

| If, with perfect perception… | then the next work is… |
|---|---|
| the arm respects 10 cm and stops making detours | **perception** (step 0): find why the ball is lost and where the phantoms come from |
| it still enters the margin or makes detours | **the controller**: steps 3 → 5 (curvature term, task/null-space objective) |

**After that:** repeat the chosen comparison with **12 seeds per scenario, interleaved**, so the results are rates and
not single outcomes.

## Running a simulation yourself

```bash
docker exec -it -e FR3_SAFETY_HALO=0.10 franka_ros2 bash
source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash
ros2 launch franka_simulation_experiments gz_cbf_stack.launch.py \
  condition:=minimal_l1 motion:=pentagon scenario:=pentagon_lateral \
  scene:=diag seed:=0 gui:=true record_bag:=false
```

**Scenarios:**
- moving: `pentagon_lateral`, `pentagon_sweep`, `pentagon_crossing`, `pentagon_multi_mixed`;
- fixed ball: `pentagon_pos_p1` … `pentagon_pos_p8`;
- occlusion: `pentagon_occl_o1`.

**Conditions:** use `condition:=minimal` for the old slack.

**Afterwards:** run `bash /ros2_ws/src/franka_simulation_experiments/sim_runs/sim_cleanup.sh`.
