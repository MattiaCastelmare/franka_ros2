# Checkpoint log (append-only)

This is the session history of the documentation project. Rules:
- Append a new entry at the **bottom**, and never edit or delete old entries. If an old entry was
  wrong, the new entry says so.
- After appending, overwrite the resume card in [PROGRESS.md](PROGRESS.md) §0 so that it agrees with
  the newest entry.
- See PROGRESS.md §6 for when to checkpoint.

Template:

```
## C-NNN — YYYY-MM-DD HH:MM — Stage N — <short title>
- Baseline / HEAD / check: <baseline sha12> / <HEAD sha12> / check_state exit <0|1|2> (<REVIEW paths, if any>)
1. Analysed: <subsystems, sections written>
2. Remaining in this stage: <list>
3. Files read (ledger status set): <path → status, …>, or "none"
4. Documentation files updated: <list>
5. Findings / uncertainties: <F-… IDs added; Q-… IDs added or closed; doubts not yet logged>
6. Exact next action: <file + section to write, first source file/function to read>
```

---

## C-001 — 2026-11-11 — Stage 1 — Architecture, ROS graph and configuration done

This entry was reconstructed afterwards, at C-002, from PROGRESS.md and the chapter files.

- HEAD / tree: `ba68077052e8`; the tree is clean except for the untracked `docs/framework_study/`.
1. **Analysed:**
   - the four pipelines (A: accel HOCBF → torque; B: velocity; C: OSCBF; D: no filter);
   - the Gazebo harness reusing `torque_control_stack.launch.py` with `external_plant:=true`;
   - the node interfaces, topics, QoS, TF tree and rates (configured, and recorded where available);
   - the configuration precedence model and the `CBF_PARAM_SPEC` validation.
2. **Remaining in this stage:** none.
3. **Files read:** the ledger records them (5 `read`, 47 `partial`, 4 `documented`). The main ones:
   - the launch files;
   - `utils/launch_support.py`, `utils/config.py`, `utils/params.py`;
   - the node constructors;
   - the C++ controller interfaces;
   - `gz_cbf_stack.launch.py`, `gz_fr3_plant.launch.py`, `conditions.py`, `sim_config.py`.
4. **Documentation files updated:**
   - `01_architecture.md`, `02_ros_graph_and_dataflow.md`, `03_configuration.md`;
   - `PROGRESS.md`, `00_INDEX.md`, `ledger/*`;
   - the new `tools/ledger_status.py`;
   - the extractor grouping fix in `tools/stage0_extract.py`.
5. **Findings and uncertainties:** F-1-1 … F-1-18 were added, as were Q-1-1 … Q-1-5. Q-1-4 and Q-1-5
   need the user.
6. **Exact next action:** stop for the user's review of Stage 1.

## C-002 — 2026-11-11 — between Stage 1 and Stage 2 — Persistent-memory system

- HEAD / tree: `ba68077052e8`. The only untracked entries are `docs/framework_study/` and the new
  root `CLAUDE.md`. No tracked file was modified.
1. **Analysed:** no framework code. The user asked for session continuity across sessions and
   accounts.
2. **Remaining:** Stages 2–7. Stage 2 is not approved yet.
3. **Files read:** none of the framework. The ledger is unchanged (pending 290, partial 47, read 5,
   documented 4).
4. **Documentation files updated:**
   - created `/CLAUDE.md`, `CHECKPOINTS.md`, `RESUME_PROMPT.md` and `tools/check_state.py`;
   - `PROGRESS.md`: added §0 (resume card), §6 (checkpoint protocol) and §7 (leads carried over from
     account memory), plus the post-Stage-1 decisions in §5; the Stage 1 row is marked as
     acknowledged;
   - `00_INDEX.md`: status line and artefact table.
5. **Findings and uncertainties:** no new F-IDs. Q-1-4 and Q-1-5 are still open for the user.
6. **Exact next action:** wait for the user's approval of Stage 2. Then:
   1. run `python3 -I docs/framework_study/tools/check_state.py`;
   2. append C-003 "Stage 2 started";
   3. read `franka_experiments/franka_experiments/utils/kinematics.py` for `04_math_foundations.md`
      §Mathematical foundations (frames, FK, control-point Jacobians J and J̇).

## C-003 — 2026-11-11 — between Stage 1 and Stage 2 — Baseline-aware consistency check

- Baseline / HEAD / check: `ba68077052e8` / `ba68077052e8` / check_state exit 0
1. **Analysed:** no framework code. The user asked for the memory system to survive commits that
   touch only the documentation, and to detect real source changes.
2. **Remaining:** Stages 2–7. Stage 2 is not approved yet.
3. **Files read:** none of the framework. The ledger is unchanged.
4. **Documentation files updated:**
   - `tools/check_state.py` now compares the working tree with the *documented source baseline*
     instead of HEAD. It classifies changes as doc, source, interface or other, and has exit codes
     0, 1 and 2;
   - `PROGRESS.md`: §0 "Documented source baseline" replaces "Pinned HEAD"; §3 step 1; the §1
     column renamed; new §8 (the source baseline, its re-check procedure, and the history row B-1);
   - `CLAUDE.md`, `00_INDEX.md` §1, `RESUME_PROMPT.md` step 4, and the template in this file.
5. **Findings and uncertainties:** no new F-IDs. Ten scenarios were tested in a throwaway clone in the
   scratchpad, since deleted:
   - commits that touch only the documentation pass (exit 0);
   - an uncommitted, committed or untracked source change gives REVIEW (exit 2) with its chapters;
   - interface and other paths give a NOTE;
   - a baseline that is not an ancestor of HEAD, and §0 disagreeing with §8, give a PROBLEM (exit 1);
   - re-baselining after review returns exit 0.
6. **Exact next action:** unchanged from C-002. Wait for the user's approval of Stage 2.

## C-004 — 2026-11-11 — Stage 2 — Stage 2 started

- Baseline / HEAD / check: `ba68077052e8` / `bd8a4e238141` (docs commit of Stages 0–1) / check_state exit 0
1. **Analysed:** nothing yet in this stage. The user explicitly approved Stage 2 (ONLY) on 2026-11-11.
2. **Remaining in this stage:**
   - ch. 04 (kinematics, frames, Jacobians, J̇, dynamics, Pinocchio);
   - ch. 05 (camera, depth, calibration, masking, distance engine, `MultiLinkDistance`, tracking,
     prediction);
   - Q-1-2 and Q-1-3.
3. **Files read:** none yet.
4. **Documentation files updated:** this entry and the PROGRESS.md resume card.
5. **Findings and uncertainties:** none yet.
6. **Exact next action:** read `franka_experiments/franka_experiments/utils/kinematics.py` in full.

## C-005 — 2026-11-11 — Stage 2 — Chapter 04 complete

- Baseline / HEAD / check: `ba68077052e8` / `bd8a4e238141` / check_state run after this entry
1. **Analysed:**
   - frames, joint ordering and the three Pinocchio models;
   - FK; LWA Jacobians; the rigid-point J and J̇; the interpolated control points; the classical
     acceleration;
   - dynamics (`nle − g`, M fill), `so3_log`, DLS, `JointStateManager`.

   Numerical checks were run in the `franka_ros2` container against the **source** (via
   `PYTHONPATH`), using the scratchpad scripts `stage2/check_kin.py`, `home.py` and `check2.py`.
   - All Jacobians and J̇ match finite differences (≤ 2.3e-10).
   - Kinematics and perception control-point positions are identical.
   - Pinocchio 4.0.0 fills the full M; `data.C` is zero; `nle − g` equals C q̇.
2. **Remaining in this stage:** all of ch. 05 (perception, distance, `MultiLinkDistance`, tracking,
   prediction, calibration, Q-1-2, Q-1-3, F-2-4); then the final verification and report.
3. **Files read (ledger set):**
   - documented: `utils/kinematics.py`, `constants.py`, `math_utils.py`, `cbf_kinematics.py`,
     `scripts/check_control_derivatives.py`;
   - read: `utils/distance_utils.py`.

   Also consulted: the `cbf_safety_filter` joint-state callback, `qddot_to_torque` (`_js_cb`,
   `_compute_tau`), the commander's `_js_cb` and `_compute_tau_des`, and the test names in
   `test_control_point_derivatives.py` and `test_kinematics_fix_regression.py`.
4. **Documentation files updated:** `04_math_foundations.md` (new); `PROGRESS.md` (§0, §2, Stage 2
   findings); the ledger.
5. **Findings and uncertainties:** F-2-1, F-2-2, F-2-3, F-2-5 and F-2-6 added. F-2-4 (hand lateral
   extent vs the 0.05 m capsule) is reserved and still to be confirmed in ch. 05.
6. **Exact next action:** read `franka_experiments/franka_experiments/nodes/real_time_distance.py`
   (the frame-processing path) and `utils/distance_engine.py` for ch. 05 §Implementation.

## C-006 — 2026-11-11 — Stage 2 — Chapter 05, perception core written

- Baseline / HEAD / check: `ba68077052e8` / `bd8a4e238141` / check_state run after this entry
1. **Analysed:**
   - the `real_time_distance` frame path (de-duplication, queue, compute thread, publish gates);
   - `DistanceEngine.compute`: back-projection, depth gate, margin, per-control-point spheres,
     multi-obstacle extras, LPF and hold;
   - `MaskBuilder` (silhouette, z-buffer, margins); `TFManager`;
   - `perception_msgs` (`build_cp_messages`, `labelled_links`, `annotate_track_fields`, confidence,
     zones); the `MultiLinkDistance` / `LinkDistance` fields;
   - the calibration files, with Q-1-2 and Q-1-3 resolved.

   Numerical work (scratchpad `stage2/labels.py` and inline scripts):
   - the extrinsic recomposition against the git history of `camera_extrinsics.yaml`;
   - the hand-mesh 45° displacement;
   - the label reproduction with duplicate control points;
   - the worked pixel example.
2. **Remaining in this stage:** ch. 05 §7 (tracking and prediction), §8 (self-detection and SSB),
   §9 (calibration nodes), §10 (integration), §12 (consequences); the final verification and
   report.
3. **Files read (ledger set):**
   - read: `utils/distance_engine.py`, `mask_builder.py`, `tf_manager.py`, `perception_msgs.py`,
     `camera_extrinsics.yaml`, `camera_link_extrinsics.yaml`, `depth_intrinsics.yaml`,
     `franka_msgs` `MultiLinkDistance` and `LinkDistance` (moved to ch. 05);
   - documented: `utils/camera_yaml.py`.

   Also read: `real_time_distance.py` L1–1060 (remaining partial: `visualize`, `_log_ssb`,
   `_check_calibration`, `_self_detected`); the `cbf_safety_filter._on_distances` head.
4. **Documentation files updated:** `05_perception_and_distance.md` (new, partial), `PROGRESS.md`
   (§0, §2, F-2-4 and F-2-7…F-2-10, Q-1-2 and Q-1-3 annotated), the ledger.
5. **Findings and uncertainties:**
   - F-2-4 and F-2-7…F-2-10 added;
   - Q-1-2 answered (a deliberate choice; root cause open; runtime test proposed);
   - Q-1-3 answered (inconsistent; F-2-7);
   - the exact form of `ema_alpha_for` is to be confirmed (`rate_scaling.py`, ch. 08).
6. **Exact next action:** read `franka_experiments/franka_experiments/utils/obstacle_track_pipeline.py`,
   then `obstacle_clusters.py` and `obstacle_tracker.py`, for ch. 05 §7.

## C-007 — 2026-11-11 — Stage 2 — Stage 2 complete

- Baseline / HEAD / check: `ba68077052e8` / `bd8a4e238141` / check_state run after this entry
1. **Analysed:**
   - tracking: `obstacle_clusters` (depth-aware connected components, scene guard),
     `obstacle_track_pipeline` (base-frame tracking; point → cluster → track lookup),
     `obstacle_tracker` (constant-acceleration KF with white-noise jerk, IMM, greedy gated
     association, M-of-N lifecycle, coast);
   - the effective configuration: tracking and IMM ON in Pipeline A via the launch override,
     prediction OFF, SSB OFF; `static_scene` / `surface_velocity` unused;
   - `self_detection` (the rigid-offset test); `self_shadow` (ray classes, interface level);
     `ballistic_prediction` (interface level);
   - `calibration_check`; the hand-eye node's residual and solver; `handeye_solver` (docstring);
     `image_rectifier_node` and `frame_grabber` (interface level); `rate_scaling.frames_for` and
     `ema_alpha_for`.

   Numerical work (scratchpad `stage2/kf.py`, `stage2/hold.py`):
   - Q(Δt) verified to equal the exact integral;
   - KF/IMM static noise floor, response to a new track, throw mode probability;
   - lifecycle counts at 90 / 30 Hz; coast–gate growth;
   - the held-entry message.
2. **Remaining in this stage:** none. `05_perception_and_distance.md` §7–§10, §10A, §12 and §13 were
   written. The final verification (links, ledger, check_state, git) follows this entry.
3. **Files read (ledger set):**
   - documented: `distance_engine`, `mask_builder`, `tf_manager`, `perception_msgs`, `distance_utils`,
     `obstacle_clusters`, `obstacle_track_pipeline`, `obstacle_tracker`, `self_detection`,
     `calibration_check`, `real_time_distance` (the visualisation and debug-image methods are left to
     ch. 13), the three camera config files, and the two `franka_msgs` messages;
   - partial: `ballistic_prediction`, `self_shadow`, `handeye_solver`, `static_scene`,
     `surface_velocity`, `obstacle_sim`, `handeye_calibration_node`, `image_rectifier_node`,
     `frame_grabber`, `test_cp_label_agreement`, `test_multi_obstacle_rows`.
4. **Documentation files updated:** `05_perception_and_distance.md` (complete); `PROGRESS.md` (§0,
   §1 Stage 2 done, §2, F-2-10 extended, F-2-11 and F-2-12 added, Q-2-1…Q-2-4); `00_INDEX.md`
   (status, chapter table); the ledger.
5. **Findings and uncertainties:**
   - F-2-11 (a held entry with ph = 0; the filter effect still to be confirmed) and F-2-12 (hand-eye
     solver documentation) added;
   - F-2-10 extended with the self-detection key mismatch;
   - Q-2-1 (Stage 3), Q-2-2 (user), Q-2-3 and Q-2-4 (runtime tests) opened.
6. **Exact next action:** STOP and report Stage 2. Wait for the user's review and explicit approval
   before starting Stage 3.

## C-008 — 2026-11-11 — after Stage 2 — Verification scripts preserved and rerun

- Baseline / HEAD / check: `ba68077052e8` / `bd8a4e238141` / check_state run after this entry
1. **Analysed:** no new framework code. At the user's request, the Stage 2 numerical evidence was
   moved from the session scratchpad into the repository and rerun.
   - Scratchpad sources: `check_kin.py`, `check2.py`, `home.py`, `labels.py`, `hold.py`, `kf.py`,
     plus three inline checks recovered from the session transcript: the camera recomposition, the
     hand-mesh bounding box and the hand rotation.
   - They are preserved as 9 scripts in `tools/stage2_verification/`, with paths derived from the
     script location, a source-import guard (F-2-2), no trimesh, bytecode caching off, and the same
     seeds and draw order.
2. **Rerun result:** 126 of 126 checks passed (`results/observed_2026-11-11.txt`).
   - Chapter 04 values reproduced to the printed digits.
   - Chapter 05 values reproduced. The only statistical exception is the hand surface samples:
     seeded numpy instead of unseeded trimesh, agreeing within 0.4 mm.
3. **Correction found by the rerun:** the full git-history scan in `check_camera_extrinsics.py`
   showed that `camera_link_extrinsics.yaml` matches calibration `e5fcdb2` (2026-05-21), the content
   at `d4e4028`, and that there were five later calibrations, not three.
   - 05 §5.2 is corrected, with the superseded wording kept.
   - F-2-7 is amended. Its conclusion (0.276 m, 5.14°) is unchanged.
4. **New numerical results** (marked NEW in the scripts):
   - F-2-4: at q_home, 94 % of the hand's vertices lie outside the sphere model, with a maximum
     excess of 6.84 cm;
   - a 1 mm control run for F-2-10;
   - the coast time at which the 4σ gate exceeds `gate_max_m`: 0.52 s.

   F-2-2 was re-checked by hand: the installed copy is still stale.
5. **Side effects:** the original Stage 2 container run had refreshed five git-ignored
   `__pycache__/*.cpython-310.pyc` files under `franka_experiments/franka_experiments/utils/`
   (12:28). They were left in place. The rerun wrote no file outside `docs/framework_study/`.
6. **Documentation files updated:**
   - new: `tools/stage2_verification/` (README, `_common.py`, 9 scripts, `run_all.sh`,
     `results/observed_2026-11-11.txt`);
   - `04_math_foundations.md` (header, §3.1, §4.3, §4.5, §6, §7 rewritten);
   - `05_perception_and_distance.md` (header, §3.2, §3.4, §3.5, §4.2, §5.2, §6.2, §7.2–7.4, §11, §13);
   - `PROGRESS.md` (§0, F-2-4, F-2-7, §5 decisions); `00_INDEX.md` (artefact table).
7. **Exact next action:** STOP. Wait for the user's review and explicit approval of Stage 3.
