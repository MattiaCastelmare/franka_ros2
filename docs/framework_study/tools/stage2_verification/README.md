# Stage 2 verification scripts

These scripts reproduce every numerical result in [chapter 04](../../04_math_foundations.md) and
[chapter 05](../../05_perception_and_distance.md). They were moved here from the session scratchpad,
where Stage 2 created them, so the evidence survives across sessions and accounts.

All of them are **read-only**:
- they import the baseline source and read configuration, meshes and git history;
- they write no file into the repository (bytecode caching is disabled);
- `xacro` writes temporary URDFs only in the container's `/tmp`.

None of them runs a ROS node, Gazebo or the robot.

## 1. Three kinds of evidence, kept apart

Every result line carries one tag (defined in `_common.py`):

| Tag | Meaning | Example |
|---|---|---|
| `NUM` | **Numerical verification** of a mathematical identity, against an independent computation | an analytic Jacobian vs a central finite difference |
| `REPRO` | **Reproduction** of a number printed in chapter 04 or 05, within the stated tolerance | the 0.276 m calibration mismatch |
| `FACT` | A code or configuration fact, evaluated numerically | "fr3_link6#0 and #1 coincide" |
| `INFO` | Printed context; not a pass or fail | the git history table |

**Mathematical derivations** are not tested here, beyond evaluating them. They live in the chapters
with the label *Derived*.

**Runtime validation** (what the running system does) is not covered here at all. §6 lists what still
needs it.

## 2. What each script verifies

| Script | Where it runs | Verifies | Equations / findings | Chapter section |
|---|---|---|---|---|
| `check_kinematics_dynamics.py` | container | [A] the two Pinocchio models, base = `fr3_link0` = identity, masses, `fr3_hand` at rpy (0, 0, −π/4) and `fr3_hand_tcp` at z = 0.1034. [B] over 25 random states: control-point FK (kinematics = perception), J, J̇, the acceleration identity, rigid-point J_P and J̇_P, Pinocchio's LWA J̇. [C] over 10 states of the hand model: the `data.M` fill, `data.C` = 0, nle − g = C q̇, the code's torque vs RNEA | (4.3)–(4.8), (4.10); F-2-3; basis of F-2-9 | 04 §2, §3.4–3.6, §4.3, §4.5 |
| `check_geometry_and_so3.py` | container | the home-pose frame-origin table; the inter-origin distances, constant over 200 q (with the zero-length segments); the duplicate control points; `so3_log` vs `pin.log3`; in the URDF, `hand.stl` belongs to `fr3_hand` and `fr3_hand_joint` has rpy (0, 0, −π/4); the forearm sphere gap; **NEW**: how far the correctly placed hand reaches outside the sphere model | (4.1), (4.11); F-2-1, F-2-4, F-2-9 | 04 §3.1, §4.2, §4.6; 05 §3.2, §4.2 |
| `check_home_example.py` | container | the worked example: fingertip p, J (3×7), J q̇, J̇ q̇, and column 1 against the closed form of (4.3) | (4.3), (4.4), (4.8) | 04 §6 |
| `check_camera_extrinsics.py` | host | camera position, optical axis and elevation; `camera_link_extrinsics.yaml` recomposed with the "D435i" offset and compared with HEAD and with **every** historical `camera_extrinsics.yaml` in git | (5.2); F-2-7; Q-1-3 | 05 §2, §5.2 |
| `check_hand_mesh.py` | host | the `hand.stl` bounding box (hand half-width); vertex and surface-sample displacement from the missing Rz(−45°); a closed-form check of the maximum (2ρ sin 22.5°) | F-2-4, F-2-9 | 05 §3.2, §4.2 |
| `check_perception_numbers.py` | host | the (5.1)+(5.2) round trip; the worked pixel example; the 12 / 24 px margins from the `MaskBuilder` integer arithmetic and the uniform bias 12 Z / f_x; sampling density; α = exp(−Δt/τ) from the source; the tracker and self-detection frame counts at 90 and 30 Hz | (5.1)–(5.3), (5.6); F-2-8 | 05 §3.4, §3.5, §4, §7.4, §8.1, §11 |
| `check_tracker.py` | host | Q(Δt) against the exact integral; KF and IMM on a static target, on a new 1 m/s track, and on a throw; the coasting covariance against `gate_max_m` | (5.7)–(5.11); Q-2-2 | 05 §7.2–7.4 |
| `check_cp_labels.py` | container | `build_cp_messages` + `labelled_links` on the 11 home-pose control points, each with one extra obstacle rank: the duplicate wrist point loses its label. **NEW** control: shifting the duplicate by 1 mm restores the intended labels | F-2-10 | 05 §6.2 |
| `check_held_entry.py` | container | a held entry (`closest_obstacle_point=None`) is published `valid` with `closest_point_human` = (0, 0, 0); the normal the filter would compute is 122° from the published direction | F-2-11 (message side only) | 05 §3.5 |

Supporting files:
- `_common.py`: paths derived from the script location, the source-import guard, YAML, quaternion and
  STL helpers, and the reporting.
- `run_all.sh`: runs everything.
- `results/observed_2026-11-11.txt`: the complete output of the run recorded in §5.

## 3. How to run

From the repository root:

```bash
docs/framework_study/tools/stage2_verification/run_all.sh              # host + container
docs/framework_study/tools/stage2_verification/run_all.sh --host-only  # host scripts only
```

**Exit code:** 0 if everything that ran passed; 1 if a check failed; 2 if the container checks were
skipped.

**A single host script:**

```bash
python3 -I -B docs/framework_study/tools/stage2_verification/check_tracker.py
```

**A single container script** (the repository is mounted at `/ros2_ws/src` in the `franka_ros2`
container):

```bash
docker exec franka_ros2 bash -c 'source /ros2_ws/install/setup.bash && cd /tmp && \
  python3 -B /ros2_ws/src/docs/framework_study/tools/stage2_verification/check_cp_labels.py'
```

**Container settings.** If your container name, mount point or setup file differ, set `FS_CONTAINER`,
`FS_REPO_IN_CONTAINER` or `FS_ROS_SETUP` for `run_all.sh`.

## 4. Environment and dependencies

| Scripts | Needs |
|---|---|
| host (`check_camera_extrinsics`, `check_hand_mesh`, `check_perception_numbers`, `check_tracker`) | Python ≥ 3.8, numpy, PyYAML; SciPy for `check_tracker`; `git` for the history part of `check_camera_extrinsics` (skipped without it). No ROS, no Pinocchio. |
| container (`check_kinematics_dynamics`, `check_geometry_and_so3`, `check_home_example`, `check_cp_labels`, `check_held_entry`) | a sourced ROS 2 Humble workspace with `xacro`, `franka_description` (ament index), `franka_msgs` and `builtin_interfaces`; Pinocchio; numpy; PyYAML |

**Source, not install.** Every script imports `franka_experiments` from the **repository source**
(`_common.use_source_package`) and refuses to run against another copy. The container's installed
copy was stale at Stage 2 (F-2-2).

**Pinocchio version.** With the ROS setup sourced, the container provides Pinocchio **4.0.0**, the
version used in Stage 2. Without it, `python3` in the same container finds a pip Pinocchio 4.1.0.
`run_all.sh` always sources ROS.

**Environment of the recorded run:**
- host: Python 3.13, numpy 2.4.2, SciPy 1.17.1;
- container: Python 3.10, numpy 1.26.4, SciPy 1.15.3, Pinocchio 4.0.0.

**Random states** use fixed seeds and the **same draw order** as the original Stage 2 scripts, so the
error magnitudes reproduce digit for digit. The one exception is the hand surface samples (see §5).

## 5. Expected vs observed (run of 2026-11-11, HEAD `bd8a4e2`, source baseline `ba68077`)

Result: **9 scripts, 126 checks, 126 passed, 0 failed.** The full output is in
`results/observed_2026-11-11.txt`.

### Chapter 04

| Check | Pass criterion | Reported in chapter | Observed now |
|---|---|---|---|
| Control-point FK, kinematics vs perception | ≤ 1e-12 m | 1.1e-16 | 1.110e-16 |
| J vs central difference | ≤ 1e-8 | 1.6e-10 | 1.615e-10 |
| J̇ vs central difference along q̇ | ≤ 1e-8 | 2.3e-10 | 2.276e-10 |
| p̈ = J q̈ + J̇ q̇ (4.6), h = 1e-4 | ≤ 1e-5 | 1.9e-7 | 1.892e-7 |
| Rigid J_P (4.4) / J̇_P (4.5) | ≤ 1e-8 | 1.2e-10 / 1.8e-10 | 1.218e-10 / 1.772e-10 |
| LWA J̇ = d/dt J_LWA | ≤ 1e-8 | 2.9e-10 | 2.861e-10 |
| nle − g vs C(q, q̇) q̇ | ≤ 1e-10 | 1.5e-14 | 1.532e-14 |
| (4.10) code torque vs RNEA − g | ≤ 1e-10 N·m | 1.3e-14, torques up to 13.6 N·m | 1.332e-14, up to 13.6 N·m |
| `data.C` = 0; `data.M` lower triangle filled | fact | yes / yes | yes / yes |
| Masses (no hand / hand) | ±5e-4 kg | 17.891 / 18.604 | pass |
| Home-pose origins, distances between origins | ±6e-5 m | the 04 §3.1 table | pass, spread over q ≤ 1e-9 |
| F-2-1 duplicate distance over 200 q | ≤ 1e-12 m | ≤ 2e-16 | 2.229e-16 |
| `so3_log` vs `pin.log3`: random / near 0 and π | ≤ 1e-8 / ≤ 1e-5 | 1.4e-10 / 2.6e-6 | 1.405e-10 / 2.598e-6 |
| Home example: p, J, J q̇, J̇ q̇ | ±6e-5 (J ±6e-4) | 04 §6 | pass |

### Chapter 05

| Check | Reported in chapter | Observed now |
|---|---|---|
| Camera t_b; optical axis; elevation; distance to base | (1.754, 0.154, 1.059); (−0.954, −0.053, −0.295); −17.2°; 2.055 m | identical; −17.19°; 2.0548 m |
| F-2-7: HEAD vs recomposed TF file | 0.276 m, 5.14° | 0.2758 m, 5.138° |
| F-2-7: which calibration the TF file matches | "`d4e4028`"; "three recalibrations since" | **`e5fcdb2` (2026-05-21)**, the version current at `d4e4028`; **five** calibrations since (see §7) |
| `hand.stl` bounding box | ±0.104 m lateral | (−0.0316, −0.1040, −0.0259) … (0.0316, 0.1004, 0.0660) |
| F-2-9 vertex displacement, max / mean | 7.96 / 6.85 cm | 7.959 / 6.854 cm |
| F-2-9 over 3000 surface samples, max / mean | 7.94 / 4.42 cm | 7.934 / 4.460 cm (different random draws; see below) |
| Worked example (05 §11): pixel, quantisation error, published d | (397.2, 296.0); 0.5 mm; 0.1634 m | (397.16, 295.99); 0.52 mm; 0.16336 m |
| F-2-8 margins and low bias at Z = 1.0 / 1.5 / 2.0 m | 12 / 24 px; 2.8 / 4.2 / 5.6 cm | 12 / 24 px; 2.80 / 4.20 / 5.60 cm |
| LPF α at 90 / 30 Hz | 0.793 / 0.499 | 0.7934 / 0.4994 |
| Tracker frame counts at 90 / 30 Hz | 3-of-5, coast 72 / 2-of-3, coast 24 | identical |
| Q(Δt) vs exact integral | 3.5e-18 | 3.469e-18 |
| KF / IMM static noise floor, mean / p95 | 0.072 / 0.120 and 0.072 / 0.122 m/s | 0.0718 / 0.1203 and 0.0721 / 0.1215 m/s |
| IMM throw, μ_ballistic at 56 / 167 / 333 ms | 0.47 / 0.88 / 0.81 | 0.467 / 0.878 / 0.809 |
| Coast σ_p after 0.1 / 0.4 / 0.8 s; 4σ gate | 1.3 / 8.0 / 29.8 cm; 1.19 m at 0.8 s | 1.28 / 7.96 / 29.76 cm; 1.191 m; it exceeds 0.5 m at **0.52 s** |
| F-2-10 labels | `fr3_link6#0.2`, `#0.3` | identical, with both transform constructions |
| F-2-11 held entry: valid, ph, normal angle | valid, (0, 0, 0), 122° | identical (122.0°) |
| Forearm gap between spheres (F-2-4) | 3.2 cm | 3.24 cm |

**What differs from the Stage 2 runs, and why it does not change the tests:**
- `check_hand_mesh.py` reads the STL with numpy instead of trimesh. The vertex set is identical (102
  unique vertices).
- The 3000 surface samples are drawn with a seeded numpy sampler from the same area-weighted
  distribution. Stage 2 used trimesh's unseeded sampler, so only the sample *statistics* are
  comparable, and they agree within 0.4 mm.
- Two checks are **new** and were not part of Stage 2:
  - the hand-vs-sphere-model excess at the home pose (94 % of the hand vertices lie outside every
    sphere; the largest excess is **6.84 cm**). This turns the "several centimetres" of F-2-4 from an
    Inferred magnitude into a numerical one, for the home pose only;
  - the 1 mm control run in `check_cp_labels.py`, which shows the labelling is correct once the
    duplicate is removed.
- Checks that were only arithmetic in the chapter text are now explicit:
  - the margin-to-metres conversion;
  - the frame counts;
  - the coast time at which the gate saturates (0.52 s; the chapter said "about 0.5 s").

## 6. What remains unverified (and what kind of evidence it needs)

| Item | Status after these scripts | What would settle it |
|---|---|---|
| F-2-11: does the filter actually build the held row with the base-radial normal, or does something reject ph = 0? (Q-2-1) | only the message side is verified | Stage 3 code reading of `cbf_state_rows`, then a runtime log of a hold |
| F-2-9: does the rotated hand model cause self-leak at the end effector? (Q-2-4) | the geometry is verified; the effect is **Inferred** | runtime: overlay and `leak=` counter with joint 7 rotated, with and without the −45° correction |
| Q-1-2: is the hand-eye result effectively base → depth optical? (Q-2-3) | not testable offline | runtime or replay: mask coverage and leak with the extrinsic as is, composed with depth→colour, and with its inverse |
| F-2-4: do the sphere gaps matter for safety? | the geometry is quantified at the home pose; the risk is **Inferred** | Stage 3 (margins in the barrier), then runtime or simulation |
| F-2-8: is the uniform margin subtraction intended? | the arithmetic is verified | the author's intent; the over-reactivity analysis in Stage 3 |
| F-2-10: the effect on per-control-point state in the filter, ISO nodes and logger | labels reproduced; effect **Derived** | Stage 3 (filter), Stage 5 (ISO, logger) |
| F-2-10: the self-detection key mismatch | **Derived** from code; not executed | a unit-level run of `_self_detected` with the duplicate (needs the node class) |
| F-2-2 (stale installed copy in the container) | an environment observation; re-checked by hand on 2026-11-11: **still stale** (`False`) | `docker exec franka_ros2 bash -c 'source /ros2_ws/install/setup.bash; cd /tmp; PYTHONDONTWRITEBYTECODE=1 python3 -c "from franka_experiments.utils.kinematics import CBFKinematics as K; print(hasattr(K, \"configure_control_points\"))"'` (False means still stale) |
| F-2-3, F-2-5, F-2-6, F-2-12 | code reading (Verified or Derived); only F-2-3's premise (the M fill) is numerically checked | — (code facts) |
| The 2026-09-30 replay results quoted from `fr3_complete.yaml` (Q-1-2 context), and the YAML's own tracker measurements | quoted from the author's comments, **not rerun** | a replay of the named bags |
| §7.3 tracker numbers | single-seed illustrations, not benchmarks | a multi-seed study, if ever needed |

## 7. Correction found by rerunning

The full-history scan in `check_camera_extrinsics.py` showed that chapter 05 §5.2 named the matching
calibration and the number of later recalibrations incorrectly. The rest of the finding was right.

- `camera_link_extrinsics.yaml` was last written in `d4e4028` (2026-06-19 10:06). At that commit,
  `camera_extrinsics.yaml` held the calibration of **`e5fcdb2` (2026-05-21)**, and the TF file
  matches it exactly.
- The calibration file then changed **five** times: `8bfd96b` (06-19 15:28), `0c7e2b9` (06-24),
  `72d2753` (09-01), `08710eb` (09-09) and `fda4af7` (09-16). The chapter said three (September only).
- The conclusion of F-2-7 is unchanged: the TF file is stale by 0.276 m and 5.14°.

Chapter 05 §5.2 and F-2-7 in `PROGRESS.md` are corrected, with the superseded wording kept visible.
