# Stage 4 verification scripts (chapters 08 and 09)

Read-only scripts that reproduce the numerical results of
[08_constraints_and_qp.md](../../08_constraints_and_qp.md) and
[09_torque_realization.md](../../09_torque_realization.md). They import the **baseline source** from
the repository (never an installed copy; F-2-2), write nothing into the repository, and disable
bytecode caching. Vocabulary as in [../stage2_verification/README.md](../stage2_verification/README.md):
`NUM` (identity verified numerically), `REPRO` (a reported number reproduced), `FACT` (a code or
configuration fact evaluated). Nothing here runs ROS nodes, Gazebo or the robot.

```bash
docs/framework_study/tools/stage4_verification/run_all.sh             # host + container
docs/framework_study/tools/stage4_verification/run_all.sh --host-only
```

Container settings: `FS_CONTAINER` (default `franka_ros2`), `FS_REPO_IN_CONTAINER`
(`/ros2_ws/src`), `FS_ROS_SETUP` (`/ros2_ws/install/setup.bash`), as for Stages 2 and 3.

## Dependencies

| Where | What | Observed version (2026-11-11) |
|---|---|---|
| container | Python 3, numpy, scipy, PyYAML, Pinocchio, xacro, franka_description, **osqp** (the deployed solver) | osqp wrapper 0.6.7.post3, core v0.6.3; scipy 1.15.3; Pinocchio 4.0.0; qpsolvers 4.3.3 |
| host | Python 3, numpy | — (the host's osqp 1.1.1 has a different API and is **not** used) |

## Scripts

| Script | Where | What it checks | Chapter |
|---|---|---|---|
| `_common4.py` | — | `QPHarness`: the filter's QP built only from production builders (`build_qp_objective`, `build_osqp_A`, `build_osqp_bounds`, `pad_rows_to_block`, `accept_iterate`), the OSQP set-up arguments of `_solve`, and its `try/except`; `static_limits` (the limits as `__init__` derives them); an independent SciPy SLSQP solve | — |
| `check_qp_assembly.py` | container | On a real `ConstraintBuilder` snapshot (obstacle, joint-limit, self-collision, cap rows): the objective identity (8.3); the dense value-independent A pattern; update == fresh setup; padding inert; OSQP == SLSQP (eps 1e-9); box exact after `accept_iterate`; production-tolerance spreads; which rows use slack (the worked example of 08 §7) | 08 §2–§4, §7 |
| `check_slack_and_priority.py` | container | (8.12) family slack closed form (**F-4-1**); shared-slack coupling (**F-4-2**); `row_l1` exact penalty (8.14); hard box/slew beats rows (**F-4-3**); obstacle vs retreat-cap split (8.13); 200 random snapshots all feasible; hard-row infeasibility and the `accept_iterate` TypeError (**F-4-4**) | 08 §3, §5 |
| `check_box_and_brake.py` | container | (8.5)–(8.7) against an independent implementation; box never empty; position protection; QP-failure brake stays inside q_max; stale-joint-state brake reversal and overshoot per joint (**F-4-5**); slew window | 08 §2.4, §5 |
| `check_torque_realization.py` | container | the production `QddotToTorqueNode._compute_tau` == RNEA − g; gravity excluded; M symmetric; `_limit_and_report` unclipped with `iso_enabled` false (**F-4-6**); box vertices vs effort_max (F-4-11); armature mismatch (**F-4-9**) | 09 §1, §2, §5 |
| `check_controller_sources.py` | host | Source facts: `RtTorqueController` writes no clamp, no LPF, no gravity, against its XML description (**F-4-7**); undeclared YAML keys (F-1-8); the generator's zero gains when not real and the Gazebo `is_real=True`; the Gazebo plugin adds KDL gravity (**Q-1-1**); `CBFTorqueController` uses the commander's nominal from `/q_des_state` (**F-4-8**); a transcribed 1 kHz loop (armature gap and PD recovery) | 09 §3–§5 |

## Observed results

`results/observed_2026-11-11.txt`: the output of `run_all.sh` on 2026-11-11 at baseline `ba68077`.
**61 of 61 checks passed** (host 11; container 14 + 15 + 14 + 7).

Tests that failed on their first run, and why. All were **test** errors; no production code was
changed:
- comparisons at OSQP's default tolerance (1e-3) were turned into tight-tolerance identities
  (eps 1e-9) plus reported production-tolerance spreads;
- a duplicated `max_iter` keyword in the harness;
- the harness lacked `_solve`'s `try/except`. Adding it revealed F-4-4 (a production-code
  behaviour, recorded, not fixed);
- the stale-brake test used a wrong v₀ (the envelope's margin was cancelled) and a wrong threshold.
  The correct condition is h < a/(2k²), with the envelope binding first on joint 4, which therefore
  does not overshoot;
- two text matches broken by line wrapping and comment lines.

## Assumptions and limits

- The node's `_qp_tick` / `_solve` call sequence is **transcribed** in `_common4.QPHarness`; every
  formula inside it is the production one. The ROS callbacks, the 50/300 Hz timing and the callback
  groups are not exercised.
- `check_box_and_brake.py` and the brake items assume ideal tracking (realised q̈ = commanded).
- The armature mismatch uses a diagonal approximation (M + diag(ARMATURE)). The plugin adds a disk
  inertia on the child link, which also couples other joints.
- `check_controller_sources.py` item 8 is a one-joint transcription of `RtTorqueController::update`,
  not the C++ code.
- The box-vertex torque check uses q̇ = 0 (no Coriolis) and excludes gravity, as the commanded τ does.

## Still needs runtime validation (not done; requires approval)

- F-4-1 / F-4-2 / F-4-3: the size and frequency of barrier violation through slack on the robot or
  in Gazebo (family slacks on `/cbf_status`).
- F-4-5: the closed-loop effect of a joint-state stall.
- F-4-7 / Q-4-3: where torque is saturated on hardware.
- F-4-9: tracking of q̈_safe in Gazebo (armature) and on the FR3.
- F-4-10: solve times and iteration counts at 300 Hz.
