# Stage 6 verification scripts (chapter 12)

These read-only scripts reproduce the numerical results and code facts of
[12_simulation.md](../../12_simulation.md), and give the state checks that
[SIMULATION_VALIDATION_PROTOCOL.md](../../SIMULATION_VALIDATION_PROTOCOL.md) builds on.

- They import the **baseline source** from the repository (`franka_simulation_experiments/scripts`,
  `franka_simulation/scripts`, `franka_experiments`), never an installed copy (F-2-2). `_common6.py`
  refuses an installed `simexp`.
- They write nothing into the repository and disable bytecode caching. `check_scoring.py` builds its
  synthetic run directories with `tempfile.mkdtemp()`, outside the repository.
- The vocabulary is the one in [../stage2_verification/README.md](../stage2_verification/README.md):
  `NUM`, `REPRO`, `FACT`, `INFO`.

**Nothing here starts Gazebo, ROS nodes or the robot.** The checks call the production functions
(laws, catalog, conditions, scorers, reflex checker, emulation, depth conversion, robot description
and world builders) on constructed inputs. The C++ obstacle system cannot run without Gazebo, so its
formulas are matched by text against the Python mirror.

## How to run

```bash
docs/framework_study/tools/stage6_verification/run_all.sh             # host + container
docs/framework_study/tools/stage6_verification/run_all.sh --host-only
```

The container settings are the same as for Stages 2–5:
- `FS_CONTAINER` (default `franka_ros2`);
- `FS_REPO_IN_CONTAINER` (default `/ros2_ws/src`);
- `FS_ROS_SETUP` (default `/ros2_ws/install/setup.bash`).

`check_installed.py` also reads `FS_INSTALL` (default `/ros2_ws/install`).

Exit codes:
- 0: every script that ran passed;
- 1: at least one script failed;
- 2: the container checks were skipped.

## Dependencies

| Where | What | Observed (2026-11-11) |
|---|---|---|
| host | Python 3, numpy, PyYAML, git | numpy 2.4.2 |
| container | Python 3.10, numpy, scipy, PyYAML, Pinocchio, xacro + `franka_description`, the built workspace (`std_msgs`, `franka_msgs`, `geometry_msgs`) | Pinocchio 4.0.0; numpy 1.26.4; scipy 1.15.3 |

`check_sim_runs.py` reads the untracked `franka_simulation_experiments/sim_runs/` when present:
- every `run_manifest_sim.json`;
- every `batches/**/cell_result.json`;
- the `truth.npz` of the 40 most recent runs.

Without that directory, its lines become INFO and nothing fails.

## Scripts

| Script | Where | What it checks | Chapter |
|---|---|---|---|
| `_common6.py` | — | reuses `../stage5_verification/_common5.py`; adds the two sim source trees to `sys.path` (as `test/conftest.py` does) and the source-import guard for `simexp` | — |
| `check_conditions.py` | host | (1) `hw_default → sim:full` = qp 300→100 and go-around off; (2) `full → minimal` = 19 effective changes, 9 no-op settings; (3) **F-6-2**, the `'1e-05'` string chain; (4) **F-6-4**, the Phase 5 conditions run 300 Hz + go-around and their rate gate is UNKNOWN; (5) the precedence global < condition < `--set`, and `--set` cannot add a key | 12 §6, §8.3 |
| `check_obstacle_laws.py` | host | (12.3)–(12.7) velocity = d/dt position for every analytic law; the C++ formulas and timing semantics by text; (12.8) pursuit Euler; (12.9) ballistic throws hit the flange; seeds (**F-6-9**); catalog classification | 12 §5, §7 |
| `check_sim_runs.py` | host | **F-6-1**: the SHAs of 1226 recorded manifests (none at the baseline, all ancestors); the batch-cell audit (**F-6-4**, **F-6-6**); **F-6-5**: the truth holes and their 5 s period | 12 §9 |
| `check_scoring.py` | container | the reflex checker and verdict; `RtfMeter`; (12.12)–(12.14) `summarize_run` on synthetic truth, exact on a uniform grid and biased by a 200 ms hole (**F-6-5**); `cell_ok` semantics (**F-6-6**); **F-6-13** scope of F-5-21; emulation = (12.11) clamped at 0, one track id (**F-6-8**); depth encoding, (12.10) σ_z, delay and dropout; rescore and truth-save facts (**F-6-10**, F-6-5) | 12 §4, §8 |
| `check_sim_geometry.py` | container | RPY round trip; (12.1) camera link pose; (12.2) HFOV; the parallel-axis lumping; mass and g(q) unchanged by armature and rigid fingers, M(q) within the disk bounds (12.16); world → base → fr3_link0 = identity; 11 truth CPs; every scenario starts outside d_safe | 12 §3, §7 |
| `check_installed.py` | container | **F-6-3**: the installed configs, programs and stack nodes against the source, byte for byte. A STATE check of the container, not of the code | 12 §2.4; protocol Phase 1 |

## Expected results

Every check is `PASS`, with the expected values written in the checks themselves. Expected totals:
- host: 11 + 17 + 2 = 30;
- container: 23 + 15 + 1 = 39;
- **69 checks in all.**

Several checks are written to **pass while the documented defect is present**, for example:
- the `'1e-05'` string;
- the UNKNOWN rate gate;
- the clamped emulated gap;
- the hole that hides a contact.

A `FAIL` on one of them after a code change means the code changed under the finding. The finding
must then be re-assessed (marked superseded, never deleted); the script must not simply be "fixed".

`check_installed.py` passes whether or not the installed tree matches the source. Its INFO lines are
the result: on 2026-11-11 they read `INSTALLED != SOURCE`.

## Observed results

`results/observed_2026-11-11.txt` is the full output of `run_all.sh` on 2026-11-11, at baseline
`ba68077` (HEAD `bb71534`, with documentation-only changes since the baseline). **69 of 69 checks
passed.** The Runtime (recorded) lines report:
- 1227 manifests;
- 11 distinct SHAs, 1195 of them from a dirty tree;
- 301 batch cells, 197 of them ok, 264 with an UNKNOWN rate verdict;
- in 39 recent runs, a largest truth hole of median 518 ms and maximum 837 ms, with 95 % of the
  holes within 0.5 s of a 5 s multiple.

These were **test errors** on the first run, all fixed in the scripts. No production code was
changed.
- `check_scoring.py` 5 matched `experiment_log` inside the node name `/experiment_logger` in
  `graph_contract.py`. The pattern is now `experiment_log.csv`.
- `check_sim_geometry.py` 4 used an invented bound on the armature effect. It was replaced by the
  exact disk bounds:

  ΔM_jj − I_a,j = Σ_{k>j} I_a,k (1 + (ẑ_j·ẑ_k)²)/2, which lies in [Σ I_a,k/2, Σ I_a,k].
- `check_sim_runs.py` 3 first claimed that *every* run has a hole. Three of 39 runs do not. The check
  now tests where the holes fall.
- `check_conditions.py` 5 tried to `--set` a key the condition does not list. That raises by design,
  and it is now one of the checked facts.

## Assumptions and limits

- **No Gazebo, no timing.** Nothing checks:
  - that gz-sim applies `SetWorldPoseCmd` before `PostUpdate`;
  - the bridge latencies;
  - executor scheduling;
  - that the supervisor's readiness test is sufficient.

  These are Unverified, or Runtime (recorded) only.
- **Transcribed by text.** The C++ obstacle laws are text matches, not executions. A reformat can
  break them without any change in behaviour.
- **The F-6-5 mechanism is Inferred.** The coincidence with the 5 s save period is measured; the
  causal link (executor blocked while saving) is not.
- **Recorded data belongs to older code** (F-6-1). The statistics describe those revisions, not the
  baseline.
- `check_sim_geometry.py` runs xacro on the **installed** `franka_description`, as the launch does.
  That package is not part of the documented scope.

## Still needs runtime validation (not done; requires approval)

- **F-6-5:** a run with the truth saved off the executor thread (or saved once at exit), compared
  with the current run on the same cell.
- **F-6-2:** `ros2 param get /cbf_safety_filter obstacle_row_osqp_eps_rel` in a `minimal_l1` run, to
  confirm the type the node holds.
- **F-6-12:** the pentagon phase at the episode start in two runs of the same cell.
- **F-6-3:** rebuild, then `check_installed.py` → `INSTALLED == SOURCE`. Protocol Phase 1 covers
  this.
