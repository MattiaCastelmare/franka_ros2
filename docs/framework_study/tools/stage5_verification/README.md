# Stage 5 verification scripts (chapters 10, 11 and 13)

These read-only scripts reproduce the numerical results and code facts of three chapters:
[10_alternative_controllers.md](../../10_alternative_controllers.md),
[11_iso_safety_layer.md](../../11_iso_safety_layer.md) and
[13_observability_and_tools.md](../../13_observability_and_tools.md).

- They import the **baseline source** from the repository, never an installed copy (F-2-2).
- They write nothing into the repository and disable bytecode caching.
- The vocabulary is the one in [../stage2_verification/README.md](../stage2_verification/README.md):
  - `NUM`: an identity verified numerically;
  - `REPRO`: a reported number reproduced;
  - `FACT`: a code or configuration fact evaluated (by text match or by calling production code);
  - `INFO`: a reported value with no pass/fail; used here for the Runtime (recorded) CSV statistics.

**Nothing here starts ROS nodes, Gazebo or the robot.** Production node methods are called
*unbound* on a stub `self` (`_common5.stub`), with real ROS message objects. The ISO preflight
script is the one exception to in-process calls: it runs as a read-only subprocess on the source
YAML.

## How to run

```bash
docs/framework_study/tools/stage5_verification/run_all.sh             # host + container
docs/framework_study/tools/stage5_verification/run_all.sh --host-only
```

Container settings, as for Stages 2–4:
- `FS_CONTAINER` (default `franka_ros2`);
- `FS_REPO_IN_CONTAINER` (`/ros2_ws/src`, the repository mounted into the container);
- `FS_ROS_SETUP` (`/ros2_ws/install/setup.bash`).

Exit codes:
- 0: every script that ran passed;
- 1: at least one check failed;
- 2: the container checks were skipped.

A single script can be run with the same command line that `run_all.sh` uses, for example:

```bash
docker exec franka_ros2 bash -c "source /ros2_ws/install/setup.bash && cd /tmp && \
  PYTHONDONTWRITEBYTECODE=1 python3 -B /ros2_ws/src/docs/framework_study/tools/stage5_verification/check_iso.py"
```

## Dependencies

| Where | What | Observed version (2026-11-11) |
|---|---|---|
| container | Python 3.10, numpy, scipy, PyYAML, Pinocchio, xacro, `franka_description`, the built workspace (`sensor_msgs`, `franka_msgs`, `rclpy`), **qpsolvers + OSQP** (the deployed solver of the velocity filter and the OSCBF) | qpsolvers 4.3.3; osqp 0.6.7.post3; Pinocchio 4.0.0; numpy 1.26.4; scipy 1.15.3 |
| host | Python 3, numpy | numpy 2.4.2 |

`check_observability.py` also reads the recorded CSVs under
`franka_simulation_experiments/sim_runs/*/logger/*/experiment_log.csv` (the four most recent) when
they exist. They are not tracked by git; without them the Runtime (recorded) lines are replaced by
one INFO line, and no check fails.

## Scripts

| Script | Where | What it checks | Chapter |
|---|---|---|---|
| `_common5.py` | — | reuses `../stage4_verification/_common4.py` (paths, source-import guard, `Report`, `C3.effective_params`, `C4.static_limits`); adds `src(path, strip)` and `stub(**kw)` | — |
| `check_velocity_cbf.py` | container | (10.3) shared-slack QP closed form on the production `_solve_cbf_qp` (deployed OSQP); (10.5) the post-QP rate clamp can undo the barrier; `select_gamma`; `bypass_cbf` default vs the docstring; stale-input and QP-failure paths; executor without a watchdog; **F-5-1**, **F-5-2**, **F-5-3**, **F-5-7**, **F-5-16** (dormant 0.4/4.0 YAML and an uncalled wrapper log) | 10 §2, §6 |
| `check_oscbf.py` | container | (10.10) joint rows = the HOCBF chain; (10.11) obstacle row is missing α₁ḣ (**F-5-4**) and its undershoot; centre distance with hard-coded 0.15 (**F-5-5**); (10.8) cost identity, positive definiteness and conditioning (F-5-15); defaults and QoS wiring | 10 §3, §6 |
| `check_torque_commander.py` | host | `pentagon_torque_commander` orientation sign: e_rot points desired → current while it is added with +K_rot, so the loop is unstable (transcribed law, **F-5-6**); no acceleration feedforward | 10 §3.1 |
| `check_iso.py` | container | (11.4) cap = exact inverse of S_p (11.1); (11.6) PFL 0.68; v_h usage (F-5-8); the `_iso_configure` gate (F-5-9, F-5-13, F-5-14); trip debounce; preflight only on the YAML (F-5-10); monitor S_h on the raw track; **F-5-17** stop-latch deadlock on the production monitor; **F-5-18** builder S_p = gap; **F-5-19** STALE latch and no age check on the fault flag; the 11 §7 worked example | 11 |
| `check_observability.py` | container | **F-5-21** finger messages on the production `joint_state_cb` (and the recorded CSV pattern); held values with no age check; legacy `min_h`; **F-5-22** run-bag topic list and zstd vs `read_bag`; `tau_max_abs` from `torque_cmd`; `run_metadata` without a caller (F-5-23). Checks 5–11 (merged from a parallel session's host script): the CSV reads only `/cbf_status[0..10]` (**F-5-24**); the logger's own h and defaults; archived `run_metadata` callers (**F-5-27**); the bag list; four stale tool defaults (**F-5-26**); the commander's `qddot_safe_topic` parameter (**F-5-25**) | 13 §2–§4, §7, §10 |

## Expected results

Every check is `PASS`: the expected values are written inside each check, in the scripts' own
`R.close` / `R.fact` calls. The checks are deterministic, apart from:
- the seeded random sample in `check_iso.py` 1;
- the `INFO` lines on recorded data.

Expected totals: host 5; container 19 + 16 + 28 + 24, that is **92 checks**.

Several checks are written to **pass when the documented defect is present**: for example,
"the obstacle row lacks α₁ḣ" and "the reset is refused after the scene cleared". A `FAIL` on one of
them after a code change means the code changed under the finding. The finding must then be
re-assessed (marked superseded, never deleted), not the script "fixed".

## Observed results

`results/observed_2026-11-11.txt` holds the full output of `run_all.sh` on 2026-11-11 at baseline
`ba68077` (HEAD `a66a134`, documentation-only changes since the baseline). **92 of 92 checks
passed**: host 5, container 19 + 16 + 28 + 24. The Runtime (recorded) lines showed the finger
pattern in 26.3–26.7 % of the rows of the four most recent Gazebo runs.

These were **test errors** on a first run, all fixed in the scripts; no production code was changed:
- the docstring of `check_velocity_cbf.py` 2 described a different q̇_prev from the one the
  check uses (q̇_prev = q̇_n, the chapter's scenario); the docstring was corrected;
- `check_observability.py` used the wrong message type for `MultiDistance.distances`
  (`HumanRobotDistance`, not `LinkDistance`) and a too-wide function slice;
- the caller scan of `run_metadata` first matched a copy of a test under a data directory. Data
  directories are now excluded, and the scan prints what it found.

**Reconciliation (C-035).** A parallel session replaced `check_observability.py` with its own
seven host-only text checks. The two versions were merged: this session's checks keep numbers
1–4, and the parallel session's checks are 5–11, unchanged apart from their numbers. Some of them
overlap (for example 3 and 9 on the bag list, 4 and 8 on `run_metadata`); both are kept.

## Assumptions and limits

- **Transcribed, not executed.** Three pieces are re-implemented in the scripts, each checked
  against the source text:
  - the post-QP rate clamp of `cbf_velocity_filter._control_loop`;
  - the orientation law of `pentagon_torque_commander`;
  - the filter's reaction to a monitor latch in `check_iso.py` 8 (fault = 1.0 while latched, from
    the verified ISO-stop branch of `_qp_tick`).
- **No timing.** ROS callbacks, executors, QoS delivery and node start-up order are not exercised.
  The boot race of 11 §6 (monitor at 3 s, filter at 2 s, STALE trip after 9 ticks) is Inferred.
- `check_oscbf.py` 3 and `check_torque_commander.py` 3 are illustrative 1-D / 1-DOF closed loops,
  not robot simulations.
- The text-match `FACT`s are tied to the baseline wording; a reformat can break them without a
  behaviour change.
- The recorded CSVs are data written by older code (`experiment_logger` unchanged since 7da68ac).
  Their manifests carry `git.sha = null`, so the exact code revision of those runs is Inferred
  from the date.

## Still needs runtime validation (not done; requires approval)

- F-5-17 / F-5-19: the ISO stop latch and the boot-time STALE trip on a running graph (Gazebo).
- F-5-3: whether the rate clamp undoes the velocity barrier at runtime; F-5-2, the fail-open
  paths under real input loss.
- F-5-4: the obstacle-row undershoot of the OSCBF under a real approach.
- F-5-21: the effect on `analyze_isolation_ab.py` results computed from affected runs.
- F-5-22: a bag recorded with `/rosout` and the ISO topics, to confirm the stop is reconstructable.
