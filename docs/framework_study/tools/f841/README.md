# F-8-41: moving-obstacle harness correction (scripts_h3, C-078; prepared, not run)

Repository copies of `~/franka_baseline_evidence/refactor_r2/06_f841/scripts_h3/`. They are
byte-identical at copy time, and the sealed copies are the ones of record. These are harness-only
changes. No production, launch or perception file is involved. The corrected copies keep the H2
file names; each change is marked `F-8-41 (Cn)`. With a static obstacle each reduces exactly to the
sealed H2 script; `test_f841.py` class C checks that on 10 sealed runs.

| File | Change |
|---|---|
| `h3_moving.py` | New. `speed_bound` (static 0, linear, oscillate, wander; ballistic and pursuit refused), `g_min`, `continuous_lower_bound` (pose-age term), `pose_pairing_excess` |
| `e3_h2_metrics.py` | C1: V4 g_min with v_obs,max (wander_close: 0.2991 m); the ball's law position for the closing speed; time to d_safe at v_ref + v_obs,max |
| `s6b1_supplementary.py` | C2: lower bound with the pose age and the relative speed; C3: `episode_min_gap_excess_over_pose_bound` |
| `h3_analyze.sh` | C3: R13 uses the excess metric; R23 (A1): bag poses within 2 mm of the analytic law; C6: root from `H3_ROOT` |
| `e3_h2_bag.py` | C4: the self-mask replay renders the ball at its law position at each probe; the static-only refusal is removed |
| `s6b1_reconstruct_clearance.py` | C5: obstacle diagnostics from the run's scenario; `placed_max_dev_from_law_m` |
| `h3_run.sh`, `h3_resolve.py` | C6: the scenario is a recorded argument (allow-list) |
| `test_f841.py` | Host tests A (units and properties), B (bug demonstrations), C (static equivalence on sealed runs), D (scenario numbers). Run: `python3 -I -B test_f841.py` |
| `test_f841_container.sh` | C4 and C5 static equivalence on the sealed cells inside the container |

Addendum (sealed): `refactor_r2/06_f841/ADDENDUM_F841.md`.
Report: [R2_GAZEBO_STATIC_AND_F841.md](../../refactoring/R2_GAZEBO_STATIC_AND_F841.md) §6.
