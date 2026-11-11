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

## C-009 — 2026-11-11 — Stage 3 — Stage 3 started

- Baseline / HEAD / check: `ba68077052e8` / `a688768f8a80` (docs commit of Stage 2) / check_state exit 0
1. **Analysed:** nothing yet in this stage. The user explicitly approved Stage 3 (ONLY) on 2026-11-11.
2. **Remaining in this stage:**
   - ch. 06 (commanders, trajectories, timing law, PD and feedforward, IK and null space, governor,
     avoidance-first, evasion, go-around, livelock, RL commander);
   - ch. 07 (HOCBF derivation, obstacle rows in `cbf_state_rows`, `cbf_constraints`, `cbf_zones`,
     `cbf_evasion`, the filter's obstacle path);
   - F-2-11 downstream (Q-2-1) and F-2-10 downstream;
   - `tools/stage3_verification/`.
3. **Files read:** none yet.
4. **Documentation files updated:** this entry, the PROGRESS.md resume card and the §1 row.
5. **Findings and uncertainties:** none yet.
6. **Exact next action:** read `franka_experiments/franka_experiments/utils/cbf_state_rows.py`
   L2400–2700.

## C-010 — 2026-11-11 — Stage 3 — obstacle-row builder read

- Baseline / HEAD / check: `ba68077052e8` / `a688768f8a80` / check_state exit 0 (at C-009)
1. **Analysed:**
   - `utils/cbf_state_rows.py`:
     - `ConstraintBuilder.build` L2335–3816, the whole obstacle loop, memory rows and snapshot
       assembly;
     - the helpers L152–946 and L1496–1765;
     - `__init__` L2120–2333 and the private methods L3963–4354.
   - `utils/cbf_qp_assembly.py::build_row_rhs` (the HOCBF RHS);
   - `utils/cbf_zones.py` (all);
   - `utils/cbf_constraints.py` (all; imported by nothing);
   - `cbf_safety_filter._on_distances`, `_latency_fields`, `_range_field`;
   - the hold branch of `distance_engine._lpf_pass`, `perception_msgs.annotate_track_fields` and
     `find_pt_confidence`, `obstacle_track_pipeline.track_for_point`;
   - `torque_control_stack.launch.py::_cbf_parameters` and its helpers, `simexp/conditions.py::resolve`,
     and the conditions `minimal` and `minimal_l1`.
2. **Still to read for ch. 07:**
   - `_escape_row`, `_livelock_direction`, `_outrun_bias` (L3818–3962);
   - `cbf_evasion.py`;
   - `cbf_qp_assembly.tangential_bias`;
   - the parts of `cbf_safety_filter._qp_tick` that use the snapshot (`qdot_cbf`, the biases,
     go-around, livelock, zone task weight).

   Then ch. 06 (commanders) in full.
3. **Provisional findings (to log in PROGRESS §4 at the next checkpoint):**
   - F-2-11 downstream: held entry → `Obstacle.ph = 0`, track fields zero, confidence ≥ 0.2, so it
     passes the `min_confidence` 0.2 gate; n̂ = pr/‖pr‖; the identity guard sees a jump (resets the
     velocity state); in residual mode the speed is computed with the wrong normal. Numerical
     reproduction pending.
   - `cbf_constraints.py` is unused.
   - `ZoneLadder` ignores `d_notice` in the weights.
   - The margin gates count `obstacle_velocity_min_frames` = 8 frames (89 ms at 90 Hz), while the
     `v_obs` gate uses the span (24 frames).
4. **Documentation files updated:** this entry only.
5. **Exact next action:** read `cbf_state_rows.py` L3818–3962, then `cbf_evasion.py`, then grep
   `cbf_safety_filter._qp_tick` for the snapshot consumers.

## C-011 — 2026-11-11 — Stage 3 — resumed after an interrupted session

- Baseline / HEAD / check: `ba68077052e8` / `a688768f8a80` / check_state exit 0
- The user re-approved **Stage 3 only** in this new session (the previous one ran out of usage).
1. **Recovered state:**
   - After C-010 the previous session wrote F-3-1 … F-3-12 into PROGRESS §4 and closed Q-2-1 there,
     but wrote no checkpoint for them. Their chapter sections (06, 07) do not exist yet.
   - Three scripts that the findings cite were **never written**: `check_held_row.py` (F-3-2),
     `check_hocbf_gains.py` (F-3-5), `check_cp_label_rows.py` (F-3-12). Until they exist and pass,
     the "numerically reproduced / evaluated" part of those labels is unsupported.
   - Present in `tools/stage3_verification/`: `_common3.py`, `effective_params.py` (9/9 PASS),
     `check_timing_governor.py` (16/16 PASS). Rerun at C-011.
   - The C-010 reading list still stands. The commander-side findings (F-3-3, F-3-7, F-3-8, F-3-9,
     F-3-10, F-3-11) show that `pentagon_qddot_commander`, `timing_law`, `trajectory`,
     `rl_policy_commander` and `test_phase_governor` were read too, but no checkpoint records it.
2. **Policy for this session:** every F-3-* finding is re-checked against the source before a
   chapter states it; a finding that does not survive is marked superseded, never deleted.
3. **Files read:** none new yet. Ledger statuses not yet updated for Stage 3.
4. **Documentation files updated:** this entry and the PROGRESS resume card.
5. **Exact next action:** re-read `cbf_state_rows.ConstraintBuilder.build` (obstacle loop) and
   `cbf_qp_assembly.build_row_rhs`; then write `check_held_row.py`, `check_hocbf_gains.py`,
   `check_cp_label_rows.py`; then write `07_cbf_obstacle_barrier.md`.

## C-012 — 2026-11-11 — Stage 3 — obstacle barrier re-read and findings re-checked

- Baseline / HEAD / check: `ba68077052e8` / `a688768f8a80` / check_state exit 0
1. **Analysed (re-read this session, line numbers at the baseline):**
   - `cbf_state_rows.py`: `Obstacle`/`ObstacleSnap`/`ConstraintSnap` L1768–1991; `__init__`,
     `_apply_input_rate` L2120–2333; `build` obstacle loop L2489–3158, memory rows L3160–3302,
     assembly L3556–3816 (zone ladder, depth push floor, slack multipliers, b_ff, κ scatter,
     evasion/outrun/livelock); `_escape_row`, `_livelock_direction`, `_outrun_bias`,
     `_retreat_cap`, `_fast_track`, `_obstacle_speed_tracked`, `_obstacle_identity_changed`,
     `_obstacle_speed`; helpers L152–946, L1496–1765.
   - `cbf_qp_assembly.build_row_rhs`, `tangential_bias`; `cbf_zones.py` (all);
     `cbf_safety_filter._on_distances`, `_latency_fields`, `_range_field`, `_track_input_rate`,
     `_update_constraints`, `_qp_tick` (nominal shaping order); `kinematics.control_point_jacobian`
     (label → `partition('.')[0]`); `config.load_cbf_config` (P = the YAML `params` block).
2. **Re-check of the provisional findings:**
   - F-3-1, F-3-6: code facts confirmed.
   - F-3-2: code path confirmed. Extra fact: `_on_distances` always builds `v_vec` as an array,
     so every `v_vec is not None` gate passes; the held entry is stopped only by `frames_seen` = 0.
   - F-3-4: **needs rewording**. The YAML comment on `zone_r_active` ("Outside it the notice gains")
     agrees with the code; only the module table and the class docstring imply that the notice
     rung starts at d_notice. `zone_r_notice` is still inert for the gains.
   - F-3-5: k1² − 4k0 = −4 confirmed; crossing weight 0.5044 confirmed by hand.
   - F-3-12: label resolution confirmed (`#0.2` → Jacobian of `#0` = `#1`). New consequence to
     verify: on a label flip `_h_smooth` has no entry, so the recovery EMA is skipped and a
     larger h is accepted at once (a one-frame loosening).
3. **Files read:** see 1. Ledger statuses not yet updated.
4. **Documentation files updated:** this entry and the resume card.
5. **Exact next action:** write `tools/stage3_verification/check_hocbf_gains.py` (host),
   `check_held_row.py` and `check_cp_label_rows.py` (container: real `ConstraintBuilder` +
   `CBFKinematics`), run them, then write `07_cbf_obstacle_barrier.md`.

## C-013 — 2026-11-11 — Stage 3 — ch. 07 verification scripts written and passing

- Baseline / HEAD / check: `ba68077052e8` / `a688768f8a80` / check_state exit 0
1. **Done:** `tools/stage3_verification/`:
   - `check_hocbf_gains.py` (host, 23/23);
   - `_builder3.py` (container helper: real `ConstraintBuilder` + `CBFKinematics`, the
     `_on_distances` mapping transcribed and `ast`-checked);
   - `check_held_row.py` (container, 21/21);
   - `check_cp_label_rows.py` (container, 8/8).
2. **Results to carry into ch. 07:**
   - F-3-2 (held row), scenario A: the row normal is base-radial, 122° from the true normal. The
     true approach (−0.10 m/s) reads +0.053 m/s along it. The k1 term's ḣ goes from −0.100 to
     +0.034…+0.053. The residual floor manufactures v_obs ≈ 0.019 from the wrong a.
   - F-3-2, scenario B (tracked obstacle closing at 0.5 m/s): v_obs drops 0.5 → 0 for the whole
     hold. The latency margin vanishes at once (n_seen gate) and the standoff decays by its EMA,
     so the row's h goes −0.116 → +0.039 m while the true gap shrinks 4.4 cm. **Safety-relevant.**
   - F-3-12 corrected: a flip to a NEW key (#1 → #0.2) accepts h_raw at once (11.9 mm loosening
     in the test); the flip BACK resumes the STALE #1 state (direction scene-dependent). Rows are
     duplicates to 5.6e-17.
   - F-3-5: complex roots only for gaps < 0.0499 m (> 5 cm inside d_safe = 0.10). Negative
     impulse lobe 2.8e-10 of the peak.
3. **Subagent leads for ch. 06 (UNVERIFIED until checked; do not cite as fact):**
   - The state governor `velocity_margin` takes the upper envelope at q̇ = 0 exactly. A parked
     joint 4 above q_ref,up gives w = 0 (task suspended).
   - The lateral-evasion and outrun EMA biases are added only while the snapshot field is not
     None, so on disengagement they drop in one tick. I read `_qp_tick` L1366–1386 myself:
     the code does this (Verified).
   - The go-around inward cut is not jerk-limited.
   - Lateral evasion maps with J_pᵀ, while outrun and go-around use DLS.
   - `utils/avoidance.py` is test-only. The three ee_* velocity commanders have no avoidance and
     no null space; position-only DLS with a max-type singularity boost.
   - RL commander: q̈_nom = clip(a)·decel_limit·scale. For F-2-11: centre −r·n̂ at the base, and a
     finite-difference v_obs spike of ≈ 60 m/s (unclipped). The cp names are off by one link
     (fr3_link3 never filled).
4. **Exact next action:** verify the governor latch with a host script (`check_nominal_shaping.py`);
   read `nodes/pentagon_qddot_commander.py` (`_tick`, `_step_nominal`, timing and IK),
   `utils/timing_law.py`, `utils/trajectory.py` (Circular/Linear/Composite); then write ch. 07,
   then ch. 06.

## C-014 — 2026-11-11 — Stage 3 — findings consolidated; chapters next

- Baseline / HEAD / check: `ba68077052e8` / `a688768f8a80` / check_state exit 0
1. **Done:**
   - Read `pentagon_qddot_commander` (`_tick`, `_governor_sigma`, `_step_nominal`,
     `_build_timing_law`, `_start_trajectory`, the parameter declarations, the CSV header),
     `timing_law.LinearTimingLaw`, `state_governor` (margin/weight/params), and the `go_around`
     tail.
   - Confirmed: F-3-3, F-3-8, F-3-11; `avoidance.py` is test-only; the RL held-entry centre is −r·n̂.
   - New script `check_nominal_shaping.py` (host, 8/8).
   - PROGRESS §4: F-3-2, F-3-4, F-3-5, F-3-7 and F-3-12 amended (old wording kept); new
     F-3-13 … F-3-19.
2. **Remaining:** write `07_cbf_obstacle_barrier.md`, then `06_nominal_motion.md`; README +
   `run_all.sh` + recorded results for `stage3_verification/`; ledger statuses; 00_INDEX; final
   checks.
3. **Exact next action:** write `docs/framework_study/07_cbf_obstacle_barrier.md`. The equation
   numbers are fixed by the scripts: (7.6) factorisation, (7.8) impulse response, (7.9) engagement,
   (7.10) `build_row_rhs`, (7.11) bounded push; and (6.1)–(6.3), (6.8), (6.10) for ch. 06.

## C-015 — 2026-11-11 — Stage 3 — chapter 07 written

- Baseline / HEAD / check: `ba68077052e8` / `a688768f8a80` / check_state exit 0
1. **Done:** `07_cbf_obstacle_barrier.md`, all sections 1–13. Equations (7.1)–(7.15).
2. **Remaining:** `06_nominal_motion.md`; the `stage3_verification/` README, `run_all.sh` and
   recorded results; ledger statuses; 00_INDEX; final checks.
3. **Exact next action:** read `utils/trajectory.py` (CircularPath, LinearSegmentPath,
   CompositePath) and `pentagon_qddot_commander._tick_recovery`; then write `06_nominal_motion.md`
   from the C-013/C-014 notes and the subagent leads (all re-checked or labelled).

## C-016 — 2026-11-11 — Stage 3 — complete; stopped for review

- Baseline / HEAD / check: `ba68077052e8` / `a688768f8a80` / check_state exit 0 (81 links consistent)
1. **Analysed and written:**
   - `06_nominal_motion.md` (§1–12);
   - `07_cbf_obstacle_barrier.md` (§1–13);
   - `tools/stage3_verification/`: `run_all.sh`, README, `results/observed_2026-11-11.txt`.
     85/85 checks: host 9 + 16 + 23 + 8, container 21 + 8.
2. **Remaining in this stage:** none.
3. **Ledger:** documented +2 (`cbf_zones`, `cbf_constraints`); partial +16 (see PROGRESS §2).
   Totals: pending 244, partial 70, read 4, documented 28.
4. **Documentation files updated:** 06, 07, PROGRESS (§0, §1, §2, §4 F-3-1…F-3-19, §5 Q-3-1,
   Q-3-2, Q-2-1 answered), 00_INDEX (status, chapter rows, artefacts), `ledger/*`, this log.
5. **Verification:**
   - No file outside `docs/framework_study/` differs from the baseline (`git diff ba68077`, the
     working tree and untracked files); 3 stashes untouched.
   - Every equation number a script cites exists in a chapter.
6. **Exact next action:** STOP. Wait for the user's review of Stage 3 and an explicit approval of
   Stage 4.

## C-017 — 2026-11-11 — Stage 3 audit — started

- Baseline / HEAD / check: `ba68077052e8` / `61260f64b0e2` (Stage 3 committed and pushed by the
  user) / check_state exit 0
- The user asked for a targeted audit of Stage 3. This is NOT a new stage: no Stage 4 work.
  - Ch. 06 §6.4–§6.7, §7, §8: every claim is re-read in the source by this session; no subagent
    reading counts as verification.
  - Ch. 07: a consistency review of the core equations and of F-3-2, F-3-3, F-3-12, F-3-13.
- **Exact next action:** read `utils/cbf_evasion.py` in full, then `go_around.py`,
  `evasion_direction.py`, `livelock.py`, the `_qp_tick` bias block and `__init__` (EMA factors).

## C-018 — 2026-11-11 — Stage 3 audit — source re-read complete; corrections pending

- Baseline / HEAD / check: `ba68077052e8` / `61260f64b0e2` / check_state exit 0
1. **Re-read by this session** (no subagent output used):
   - `cbf_evasion.py` (all); `go_around.py` (all); `evasion_direction.py` (all);
     `livelock.py` (all);
   - `cbf_state_rows.retreat_speed_available`, `_escape_row`, `_livelock_direction`,
     `_outrun_bias`;
   - `cbf_safety_filter` `__init__` L325–336 (EMA factors), `_smooth_qdot`, `_qp_tick` L1326–1433;
   - `state_governor.directional_fade`; `cbf_qp_assembly.build_osqp_bounds` and
     `build_qp_objective`;
   - `rl_policy.py` (ObsSpec, obs_spec_from_config, build_observation, control_point_geometry,
     obstacle_velocity, action_to_qddot, qddot_max_from_limits, find_sim_root,
     find_latest_model, resolve_model_path); `rl_policy_commander` (`__init__` parameters and
     model resolution, `_obs_cb`, `_target_cb`, `_tick`);
   - `ee_pentagon_velocity_commander._timer_cb` and its parameters; `kinematics.dls_solve`;
     `math_utils.lpf`; `PentagonTrajectory.evaluate`; `avoidance.tangential_redirect`;
   - `perception_msgs.build_cp_messages` L196–232 (wire link names); `franka_rl/config.yaml` (obs,
     control_points); launch Node blocks of both commanders.
2. **Errors found:**
   - ch. 06 §6: the composition order is wrong (the zone fade comes BEFORE the governor); the EMA
     line refs should be L332–336, not L339–341.
   - ch. 06 §7 / F-3-10: the finite difference uses the 100 Hz policy tick, not the 90 Hz message
     rate. The spike is 71 m/s (not 64) for ‖p_h‖ = 0.71 m, and the observation is a sawtooth even
     without the defect.
   - ch. 07 §2.3: an unsupported sign claim about m(o).
   - ch. 07 §8.1 / F-3-13: the 15.5 cm relaxation is paired with 4.4 cm of gap from a different
     interval; over the same frames the gap shrinks 5.0 cm.
3. **Reruns:** `check_timing_governor` 16/16, `check_held_row` 21/21, `check_cp_label_rows` 8/8.
4. **Exact next action:** apply the corrections to 06, 07 and PROGRESS (keeping the old wording in
   an audit record), then the final report.

## C-019 — 2026-11-11 — Stage 3 audit — complete; stopped for review

- Baseline / HEAD / check: `ba68077052e8` / `61260f64b0e2` / check_state exit 0
1. **Corrections applied** (old wording kept in place or in the audit tables):
   - ch. 06: composition order and EMA refs (§6); §6.4–§6.7 rewritten from the source; §7 (F-3-10
     at 100 Hz, 71 m/s; sawtooth; gates; model resolution; F-3-19 verified end to end); §8
     (verified; two un-launched commanders explicitly marked as not re-read); new §13 audit record.
   - ch. 07: §2.3 m(o) sign claim withdrawn; §2.5 QP direction and objective now cite their
     source; §8.1 / F-3-13 intervals (5.0 cm); new §14 audit record with a reproducible vs
     closed-loop table.
2. **PROGRESS §4:**
   - amended F-3-2 (go-around reach), F-3-10, F-3-13, F-3-19 (label → Verified);
   - new F-3-20.
   - No other evidence label was changed.
3. **Ledger:** `cbf_evasion`, `go_around`, `evasion_direction`, `livelock` → documented. Totals:
   pending 244, partial 66, read 4, documented 32.
4. **Exact next action:** STOP. Wait for the user's review and an explicit approval of Stage 4.

## C-020 — 2026-11-11 — Stage 4 — started

- Baseline / HEAD / check: `ba68077052e8` / `fd1b95735a87` (Stage 3 audit committed by the user) /
  check_state exit 0
- The user explicitly approved **Stage 4 ONLY** (Constraints, QP and torque realization):
  `08_constraints_and_qp.md`, `09_torque_realization.md`, `tools/stage4_verification/`.
1. **Plan:**
   1. QP formulation: `cbf_safety_filter` `__init__` (QP set-up), `_solve`, `_warmup`;
      `cbf_qp_assembly` (`build_osqp_A`, `build_qp_objective`, `pad_rows_to_block`, `accept_iterate`,
      `box_only_solve`, `braking_command`).
   2. Constraint families: `cbf_state_rows` (`joint_limit_rows`, rate caps, link speed,
      `SelfCollisionRowBuilder`, `ShadowRowBuilder`); `cbf_singularity`; `self_collision`;
      `cbf_hard_limits` (the box, slew).
   3. Fallbacks: `_qp_tick` and `_solve`, `_brake_authority_fault`, `_empty_frame_fault`, ISO stop.
   4. Torque: `qddot_to_torque.py`, `franka_rt_controllers` (`rt_torque_controller.cpp`,
      `cbf_torque_controller.cpp`, the YAML and XML), Gazebo plant (Q-1-1).
2. **Exact next action:** read `cbf_safety_filter.__init__` from L241 (QP set-up) and `_solve`.

## C-021 — 2026-11-11 — Stage 4 — sources read (QP, constraints, fallback, torque)

- Baseline / HEAD / check: `ba68077052e8` / `fd1b95735a87` / check_state exit 0
1. **Read:**
   - `cbf_safety_filter` `__init__` L241–540 (objective, slack boxes, timers), `_warmup`, `_solve`,
     `_brake_box`, `_slew_delta`, `_joint_state_frozen`, `_brake_authority_fault`,
     `_empty_frame_fault`, `_iso_stop_active`, `_publish`, `_on_joint_state`, `_on_qddot_nom`,
     `_qp_tick` (whole, in Stage 3 and again here);
   - `cbf_qp_assembly`: `build_osqp_A`, `build_qp_objective`, `build_osqp_bounds`,
     `pad_rows_to_block`, `accept_iterate`, `box_only_solve`, `braking_command`;
   - `cbf_state_rows`: `joint_limit_rows`, the link-speed functions, the retreat functions,
     `SelfCollisionRowBuilder`, `build_self_collision_builder`, `build_optional_row_builders`, the
     rate-cap and speed-row block L3414–3556;
   - `cbf_singularity.SingularityRowBuilder`; `cbf_hard_limits` (`hard_accel_box`,
     `position_velocity_accel_box`, `apply_slew_limit`, `fr3_velocity_envelope`, constants);
   - `qddot_to_torque.py` (all); `rt_torque_controller.cpp` and `cbf_torque_controller.cpp` (all);
     CMakeLists and the two plugin XMLs;
   - `launch_support._generate_torque_yaml` / `_generate_cbf_base_yaml`; `minimal.launch.py`
     controller selection; `torque_control_stack` controller YAML;
   - `gz_fr3_plant.launch.py`; `gz_robot_description` docstring and ARMATURE;
     `sim_config.rt_controllers_yaml`; `franka_ign_ros2_control/src/ign_system.cpp::write`.
2. **Key facts (Verified) for ch. 08/09 (not yet written):**
   - Objective P = I on q̈ (no weighting), ½ρ_g s_g² per family (7 families); row_l1 adds
     ε s² + ρ_l1 s per slot. Slack box [0, 1e6]. Rows padded to blocks of 16; OSQP rebuilt
     only when the padded row count changes.
   - No torque rows and no workspace rows in the QP (`workspace_face_rows` is unused). The box
     (position, velocity, acceleration, slew) is HARD; every CBF row is soft.
   - Post-QP: only `accept_iterate`'s clip to the box.
   - `qddot_to_torque`: τ = M q̈ + (nle − g); clip and rate limit only if `iso_enabled`; no
     staleness check on the joint state.
   - `RtTorqueController`: τ = ffScale·τ_ff + K_p p + K_d e, with q̇_des/q_des integrated from
     `/qddot_safe` at 1 kHz; **no torque clip and no LPF**, although the XML description says so
     (new finding); hold on a stale τ (> 0.1 s).
   - Gazebo: **Q-1-1 answered**: `IgnitionSystem::write` adds the KDL g(q) to every effort command;
     the controller YAML uses `is_real=True` (same gains, 1 kHz); ARMATURE is added to the links.
   - `CBFTorqueController`: when `q_des_state` is fresh it computes τ from the COMMANDER's
     nominal q̈ (`effort` field), bypassing `/qddot_safe` (new finding; minimal launch with
     `use_torque_controller:=cbf` only).
3. **Exact next action:** write `tools/stage4_verification/` (QP assembly and solve against an
   independent solver; slack and slew priority; box and braking; inverse dynamics; realized vs
   commanded acceleration; effort margins), then ch. 08, then ch. 09.

## C-022 — 2026-11-11 — Stage 4 — verification scripts done; findings logged

- Baseline / HEAD / check: `ba68077052e8` / `fd1b95735a87` / check_state exit 0
1. **Done:** `tools/stage4_verification/`:
   - `_common4.py` (`QPHarness` from the production QP builders; `static_limits`; an
     independent SLSQP solve);
   - `check_qp_assembly.py` 14/14, `check_slack_and_priority.py` 15/15, `check_box_and_brake.py`
     14/14, `check_torque_realization.py` 7/7 (all container, osqp 0.6.7, Pinocchio 4.0.0);
   - `check_controller_sources.py` 11/11 (host);
   - `run_all.sh`; `results/observed_2026-11-11.txt` (61/61).
2. **Tests that failed first and why (test errors, all fixed; the implementation was not
   changed):**
   - default-tolerance comparisons were rewritten as tight-tolerance identities plus reported
     spreads;
   - a double `max_iter` in the harness;
   - the harness lacked `_solve`'s try/except; it revealed F-4-4;
   - wrong v0 and threshold in my brake derivation (now h < a/(2k²), with the envelope binding
     first on joint 4);
   - two regex/whitespace matches.
3. **PROGRESS §4:** F-4-1 … F-4-11 logged. Q-1-1 answered. New Q-4-1 … Q-4-3.
4. **Exact next action:** write `08_constraints_and_qp.md` (QP formulation, families,
   slack, OSQP, fallback, the command-shaping order), then `09_torque_realization.md`, then the
   README, ledger, 00_INDEX and final checks.

## C-023 — 2026-11-11 — Stage 4 — chapter 08 written

- Baseline / HEAD / check: `ba68077052e8` / `fd1b95735a87` / check_state exit 0
1. **Done:** `08_constraints_and_qp.md`, §1–9 (equations 8.1–8.16, Mermaid flow). New F-4-12 (no
   rows and no fault before the first distance message). OSQP 0.6 settings verified from the
   solver banner (eps 1e-3, polish off, time_limit off).
2. **Remaining:** `09_torque_realization.md`; the stage4 README; ledger; 00_INDEX; PROGRESS §0/§1/§2;
   final checks.
3. **Exact next action:** write `09_torque_realization.md` from C-021 and the
   `check_torque_realization` / `check_controller_sources` results.

## C-024 — 2026-11-11 — Stage 4 — chapter 09 written

- Baseline / HEAD / check: `ba68077052e8` / `fd1b95735a87` / check_state (to run at C-025)
1. **Done:**
   - `09_torque_realization.md` §1–9 (equations 9.1–9.6, two Mermaid diagrams), with Q-1-1
     answered;
   - `tools/stage4_verification/README.md`;
   - new F-4-13 (the `qddot_to_torque` docstring's false gravity claim).
2. **Remaining:** ledger statuses, 00_INDEX, PROGRESS §0/§1/§2, rerun `run_all.sh`, link check,
   source-unchanged check.
3. **Exact next action:** update the ledger with `tools/ledger_status.py` for the Stage 4 files.

## C-025 — 2026-11-11 — Stage 4 — complete; stopped for review

- Baseline / HEAD / check: `ba68077052e8` / `fd1b95735a87` / check_state exit 0 (92 links consistent)
1. **Delivered:**
   - `08_constraints_and_qp.md` §1–9 (8.1–8.16);
   - `09_torque_realization.md` §1–9 (9.1–9.6);
   - `tools/stage4_verification/` (5 scripts + `_common4.py`, `run_all.sh`, README,
     `results/observed_2026-11-11.txt`): **61/61** on the final rerun.
2. **Findings:** F-4-1 … F-4-13 in PROGRESS §4. Q-1-1 answered (Gazebo plugin adds the KDL g(q)).
   New Q-4-1 … Q-4-3.
3. **Ledger:** documented 42, partial 61, read 4, pending 239.
   - Not read: `self_collision.py`, `singularity_recovery.py`, `pin_rt_thread.sh`, the ch. 08 tests
     (names only).
4. **Verification:**
   - no file outside `docs/framework_study/` differs from `ba68077` (working tree and untracked
     files included); 3 stashes untouched; nothing committed;
   - every equation number cited by a script exists in 08/09;
   - every F-4 ID cited in a chapter exists in PROGRESS §4.
5. **Exact next action:** STOP. Wait for the user's review of Stage 4 and an explicit approval of
   Stage 5 (`10_alternative_controllers.md`, `11_iso_safety_layer.md`, `13_observability_and_tools.md`).
