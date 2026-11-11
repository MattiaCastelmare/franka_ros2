# Stage 3 verification scripts (chapters 06 and 07)

Read-only scripts that reproduce every numerical result of
[06_nominal_motion.md](../../06_nominal_motion.md) and
[07_cbf_obstacle_barrier.md](../../07_cbf_obstacle_barrier.md). They import the **baseline source**
from the repository (never an installed copy; F-2-2), write nothing into the repository, and
disable bytecode caching. Vocabulary as in [../stage2_verification/README.md](../stage2_verification/README.md):
`NUM` (identity verified numerically), `REPRO` (a number in a chapter reproduced), `FACT` (a code or
configuration fact evaluated). Nothing here runs ROS nodes, Gazebo or the robot.

```bash
docs/framework_study/tools/stage3_verification/run_all.sh             # host + container
docs/framework_study/tools/stage3_verification/run_all.sh --host-only
```

## Scripts

| Script | Where | What it checks | Chapter |
|---|---|---|---|
| `_common3.py` | — | Effective `cbf_safety_filter` parameters per configuration, computed with the launch file's own helper functions (extracted by `ast`), on top of `../stage2_verification/_common.py` | — |
| `_builder3.py` | container | Builds the real `ConstraintBuilder` + `CBFKinematics` as `cbf_safety_filter.__init__` does (obstacle rows only); transcribes the `_on_distances` message → `Obstacle` mapping and checks its keyword set by `ast` | — |
| `effective_params.py` | host | Which barrier and shaping mechanisms are ON in `yaml`, hardware default, `sim:kf`, `sim:minimal_l1` (table + 9 FACTs) | 06 §6, 07 §3–§4, §10 |
| `check_timing_governor.py` | host | (6.1)–(6.3) chain rule; soft start; **F-3-3** (v_d ×1/σ, centripetal ×1/σ²); the illustrative lead; **F-3-9** junction step; σ(d) | 06 §3, §5, §10 |
| `check_hocbf_gains.py` | host | (7.6) factorisation per rung; **F-3-5** crossing weight and impulse-response lobe (7.8); (7.9) engagement ratios; **F-3-4** d_notice inert; (7.10) `build_row_rhs`; (7.11) bounded push; 1-D closed loop | 07 §2, §3.8, §5 |
| `check_nominal_shaping.py` | host | (6.8) damped pseudoinverse; (6.10) **F-3-14** null-space leakage; **F-3-15** state governor at q̇ = 0; **F-3-10** RL held-entry centre and v_obs jump | 06 §4, §6.2, §7 |
| `check_held_row.py` | container | **F-2-11 → F-3-2, F-3-13** through the real builder: gates passed, base-radial normal (122°), identity resets, scenario A (arm approaching) and B (tracked obstacle closing) traces | 07 §8.1, §9 |
| `check_cp_label_rows.py` | container | **F-2-10 → F-3-12** through the real builder: labels with 1 and 2 clusters, duplicate rows, `_h_smooth` restart on a new key and stale resume on the old one | 07 §6, §8.2 |

## Observed results

`results/observed_2026-11-11.txt`: the full output of `run_all.sh` on 2026-11-11, at baseline
`ba68077`. **85 of 85 checks passed** (9 + 16 + 23 + 8 host; 21 + 8 container). Container:
`franka_ros2`, Pinocchio 4.0.0.

Corrections this rerun caused (recorded in PROGRESS §4 with the original wording kept):
- F-3-12: the flip back to the old label resumes **stale** state; it does not reset it.
- F-3-15: a subagent reported a latch at w = 0; the measured values are 0.81 at the position-box
  edge and 0 only at q_ref itself.
- F-3-4: the YAML comment on `zone_r_active` agrees with the code; only two docstrings disagree.

## Assumptions and limits

- `check_held_row.py` and `check_cp_label_rows.py` keep q fixed at q_home and impose q̇, so the
  builder sees a consistent (q, q̇) per call but the arm does not move between calls. The gap
  sequence is imposed, not simulated. They test the builder's arithmetic and state handling, not
  closed-loop behaviour.
- Only obstacle rows (`G_OBS`) are examined. The self-collision, singularity and SSB builders are
  not constructed.
- Message → `Obstacle` is a transcription of `_on_distances` (the node needs a ROS graph). `ast`
  confirms the keyword set, not the expressions.
- `check_timing_governor.py` item 4 is an idealised double integrator, not the robot.

## Still needs runtime validation (not done; requires approval)

- F-3-3 / F-3-13 / F-3-16: closed-loop effects on the real or simulated arm.
- F-3-10: the RL policy's response to the spike.
- The m(o) assumption in ḣ (07 §2.3).
