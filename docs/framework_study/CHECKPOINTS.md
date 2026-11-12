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

## C-026 — 2026-11-11 — Stage 5 — started

- Baseline / HEAD / check: `ba68077052e8` / `a66a134` (Stage 4 committed by the user) / check_state exit 0
- The user explicitly approved **Stage 5 ONLY**: `10_alternative_controllers.md`,
  `11_iso_safety_layer.md`, `13_observability_and_tools.md`, `tools/stage5_verification/`.
1. **Plan:**
   - read myself:
     - ch. 10: `cbf_velocity_filter.py`, `ros_setup.py`, `rt_velocity_blender_controller.cpp`
       + YAML/XML, `ee_pentagon_velocity_commander` (already in 06 §8), `cbf_OSCBF_filter.py`,
       `pentagon_torque_commander.py`;
     - ch. 11: `iso_ssm.py`, `iso_safety_monitor.py` and the ISO hooks in `cbf_safety_filter`
       (`_iso_configure`, the ISO rows);
   - subagents (read-only; their output is labelled and spot-checked): the ch. 13 inventory, and
     the ISO evidence logger / preflight / scripts / tests.
   - RL: 06 §7 (audited) is reused; only the training-vs-deployment specifics are added.
2. **Exact next action:** read `nodes/cbf_velocity_filter.py`.

## C-027 — 2026-11-11 — Stage 5 — velocity pipeline (Pipeline B) read

- Baseline / HEAD / check: `ba68077052e8` / `a66a134` / check_state exit 0
1. **Read (all Verified):** `cbf_velocity_filter.py` (all), `cbf_utils.select_gamma`,
   `ros_setup.init_pinocchio_only` / `make_joint_state_callback`,
   `rt_velocity_blender_controller.cpp` (all) + its header field comments,
   `velocity_cbf_control_stack.launch.py` (filter node and `bypass_cbf` argument).
2. **Facts:**
   - ZCBF of relative degree 1, aᵀq̇ + s ≥ −γh, with a = n̂ᵀJ_p, n̂ = the PERCEPTION
     direction (not pr−ph), h = d − d_safe, γ = select_gamma (2…12 · conf) and no v_obs. One
     slack, ρ = rho_slack. Box: |q̇| ≤ q̇_max from the joint_limits table (no position limits).
     qpsolvers `solve_qp` with `qp_solver` (osqp).
   - Activation hysteresis by `robot_link_name` (d < d_safe + 0.10 on, d > d_safe + 0.15 off), shared
     across a link's control points (order-dependent).
   - Fallbacks: distances stale or never received → clip(q̇_nom) PASS-THROUGH; QP failure →
     0.5·clip(q̇_nom).
   - Post-QP: a per-joint rate clamp min(decel_col3, jerk·1 ms·0.8/2), which can undo the CBF row.
     100 Hz timer; joint state from the 30 Hz `/NS_1/joint_states`, with no staleness check.
   - Executor: linear interpolation between samples, rate limit `max_accel` (1.5), clamp
     `qdot_max` (1.5). **`timeout_threshold_s` / `timeout_ramp_s` are unused**: a stale command
     is held forever (header: "reserved – not used in RT path").
   - **ch. 01 error:** `bypass_cbf` defaults to **false** (`DeclareLaunchArgument`, L434–436), so
     the QP is active by default. The launch docstring and ch. 01 §4.2 (L226–227) said the opposite.
3. **Exact next action:** read `nodes/cbf_OSCBF_filter.py`, then `nodes/pentagon_torque_commander.py`.

## C-028 — 2026-11-11 — Stage 5 — OSCBF, torque commander, ISO core read

- Baseline / HEAD / check: `ba68077052e8` / `a66a134` / check_state exit 0
1. **Read (Verified):** `cbf_OSCBF_filter.py` (all functions), `pentagon_torque_commander.py` (law,
   L385–470), `iso_ssm.py` (all), `iso_safety_monitor.py` (`_tick`, `_evaluate`, `_on_reset`,
   `__init__`), `cbf_safety_filter._iso_configure`, `torque_control_stack._speed_ceiling_overrides`,
   the ISO block of `launch_defaults.yaml`.
2. **Facts and suspected bugs (to verify numerically):**
   - OSCBF: decision τ (7) + 1 slack; cost ½‖w_j M⁻¹Nᵀ(τ−τ_n)‖² + ½‖w_o J M⁻¹(τ−τ_n)‖²;
     q̈ = M⁻¹(τ − Cq̇) (gravity assumed added downstream).
   - **The OSCBF obstacle row omits α₁ḣ:** it enforces ḧ ≥ −α₂(ḣ + α₁h), where the HOCBF chain
     needs ḧ ≥ −(α₁+α₂)ḣ − α₁α₂h. The joint-position and workspace rows are correct. α = 10, 10.
   - The OSCBF obstacle h = ‖p_r − p_h‖ − 0.15 (centre distance, d_safe hard-coded);
     `enable_obstacle_cbf` defaults to false; RELIABLE subscription vs BEST_EFFORT publisher (F-1-6);
     joint state from the 30 Hz topic; stale distances → rows silently skipped; infeasible → τ_nom
     passed; stale nominal → τ = 0; `/NS_1/torque_safe` has no consumer.
   - **pentagon_torque_commander orientation sign:** e_rot = vee((R_dᵀR − RᵀR_d)/2) ≈ +θ (current
     − desired), fed with +K_p ⇒ θ̈ = +K_p θ − K_d θ̇ (unstable). No a_d feedforward. Test-only
     node.
   - ISO: S_p, speed cap and v_PFL closed forms match the standard formulas.
     - The monitor and the QP SSM rows use the MEASURED track approach speed v_app (0 when
       untracked; ungated by frames_seen) for S_h, not `iso_v_human` (2.0).
     - `iso_v_human` is used only for the standoff (t = T_r + v_h/a_s).
     - `iso_enabled` fails closed unless `link_speed_max` ≤ v_PFL and the cap < link max.
3. **Exact next action:** write `tools/stage5_verification/` (velocity CBF, OSCBF, torque
   commander, ISO); then await the subagent reports (ch. 13 inventory; ISO evidence/preflight).

## C-029 — 2026-11-11 — Stage 5 — ch. 10/11 scripts written; findings logged

- Baseline / HEAD / check: `ba68077052e8` / `a66a134` / check_state exit 0
1. **Done:** `tools/stage5_verification/`:
   - `_common5.py`;
   - `check_velocity_cbf.py` 16/16, `check_oscbf.py` 16/16, `check_iso.py` 14/14 (container);
   - `check_torque_commander.py` 5/5 (host).
2. **Test errors fixed (production untouched):** a tolerance mismatch with qpsolvers' ~1e-3
   default; a wrong rate-clamp scenario (q̇_prev = 0 helps; the realistic one is q̇_prev = q̇_n); a
   missing `P.config_path` in the stub.
3. **Findings:** F-5-1 … F-5-15 in PROGRESS §4. The ISO subagent's F1/F2/F3/F7 were spot-checked and
   are now F-5-10/F-5-13/F-5-12.
4. **Pending:** the ch. 13 subagent report; ch. 01 §4.2 correction (F-5-1); write 10, 11, 13;
   README/run_all; ledger; 00_INDEX.
5. **Exact next action:** apply the ch. 01 correction, then write `10_alternative_controllers.md`.

## C-030 — 2026-11-11 — Stage 5 — resumed in a new session (another account)

- Baseline / HEAD / check: `ba68077052e8` / `a66a134` / check_state exit 1 (only problem: the
  ch. 10 link to the not-yet-written `tools/stage5_verification/README.md`)
- The user re-authorised **Stage 5 ONLY** in this session (resume and complete; STOP before Stage 6).
1. **Recovered from the repository (nothing else was available):**
   - C-026 … C-029; F-5-1 … F-5-15 in PROGRESS §4;
   - the ch. 01 §4.2 correction (F-5-1), applied after C-029, with the old wording struck through;
   - `10_alternative_controllers.md` drafted (§1–§7, 308 lines);
   - `tools/stage5_verification/`: 4 scripts + `_common5.py`. Re-run now: velocity 16/16,
     OSCBF 16/16, ISO 14/14 (container), torque commander 5/5 (host).
2. **Lost:** the ch. 13 inventory subagent report was never persisted. Of the ISO
   evidence/preflight subagent, only the spot-checked items F-5-10/12/13 survive. Both are redone by
   reading the source in this session; no unpersisted claim is reused.
3. **Exact next action:** spot-check and finish ch. 10 (equation numbering, practical example,
   status classes, remaining ch. 10 files), then read the ISO files for ch. 11.

## C-031 — 2026-11-11 — Stage 5 — chapter 10 complete

- Baseline / HEAD / check: `ba68077052e8` / `a66a134` / (link to README still pending)
1. **Done:**
   - ch. 10 spot-checked against the source (velocity filter hysteresis/QoS/limits, OSCBF row
     constants L338 vs L526, RL model discovery incl. the container layout, executor YAML).
   - Equations renumbered (10.1)–(10.12); (10.1) unifies the three relative degrees.
   - Added §5.1 (status classes), §6 (worked example, numbers from the scripts) and
     §7 (test coverage).
   - `check_velocity_cbf.py` now 19/19 (new check 6 = F-5-16).
2. **Ledger:** documented: `cbf_velocity_filter.py`, `cbf_OSCBF_filter.py`,
   `rt_velocity_blender_controller.cpp`, `rt_velocity_blender.yaml`,
   `rt_velocity_blender_controller.xml`, `test_cbf_velocity_filter.py`,
   `test_torque_stack_external_plant_off_identical.py`. Partial: `ros_setup.py`
   (`init_pinocchio_and_capsules` and `make_planning_scene_callback` are not read; they are
   re-exported by `simulation_imports.py`), `pentagon_torque_commander.py` (only the law).
3. **New finding:** F-5-16 (stale executor YAML, dormant `[ACTIVE]` log, commander docstring).
4. **Exact next action:** ch. 11. Read `utils/iso_ssm.py` in full, then `iso_safety_monitor.py`,
   `iso_evidence_logger.py`, `scripts/iso_preflight_check.py`, `iso_evidence_report.py`,
   `iso_pfl_speed.py`, `iso_constants_measure.py`, the ISO rows of `cbf_safety_filter` and the ISO
   tests (names and asserted constants).

## C-032 — 2026-11-11 — Stage 5 — ISO sources read; ch. 11 scripts extended

- Baseline / HEAD / check: `ba68077052e8` / `a66a134` / (README link still pending)
1. **Read in this session (Verified):**
   - `iso_ssm.py` (all); `iso_safety_monitor.py` (all);
   - `iso_evidence_logger.py` (constructor, `_tick`, `_evaluate`, columns, events);
   - `iso_preflight_check.py` (all); `iso_evidence_report.py` (D1, separation, cross-check, the
     check list); `iso_pfl_speed.py` (docstring); `iso_constants_measure.py` (`cmd_stop`,
     `c_from_detection_capability`, sub-commands);
   - filter ISO hooks: `_iso_configure`, the ISO-stop branch of `_qp_tick`, `_iso_stop_active`,
     `_brake_authority_fault`, `_empty_frame_fault`, the `/cbf_status` ISO tail;
   - `cbf_state_rows` SSM speed rows (L3478–3570) and `link_speed_row`;
   - the torque-stack ISO wiring (monitor delay 3 s, preflight, evidence logger);
   - the commander's `_iso_safety_cb`;
   - ISO test names; `watchman_profile.md` and `SAFETY.md` headings (leads only).
2. **`check_iso.py` now 22/22:**
   - 8: stop-latch deadlock, reproduced with the production monitor methods;
   - 9: the builder's S_p diagnostic equals the gap;
   - 10: STALE latch at start, and no age on the cbf_status flag.
3. **Findings:** F-5-17 (deadlock), F-5-18 (two S_p meanings), F-5-19 (liveness, boot race
   Inferred), F-5-20 (placeholder gate preflight-only); F-5-13 extended.
4. **Exact next action:** write `11_iso_safety_layer.md` (this checkpoint's facts plus F-5-8 …
   F-5-14 and F-5-17 … F-5-20), then update the ledger for the ch. 11 files.

## C-033 — 2026-11-11 — Stage 5 — chapter 11 written

- Baseline / HEAD / check: `ba68077052e8` / `a66a134` / (README link still pending)
1. **Done:**
   - `11_iso_safety_layer.md` §1–§10, equations (11.1)–(11.9), with a worked example pinned by
     `check_iso.py` 11 (now 28/28) and an explicit non-claim of conformity;
   - the remaining parts of `iso_evidence_logger` read (`_write_manifest`, input callbacks).
2. **Ledger:**
   - documented: `iso_ssm.py`, `iso_safety_monitor.py`, `iso_evidence_logger.py`,
     `iso_preflight_check.py`;
   - partial: `iso_evidence_report.py`, `iso_constants_measure.py`, `iso_pfl_speed.py`,
     `watchman_profile.md`, the 10 ISO tests (names only).
3. **Noted for ch. 13:** the evidence logger's `tau_max_abs` comes from `/NS_1/torque_cmd`, which
   excludes gravity (09), so it is not the joint torque.
4. **Exact next action:** ch. 13 inventory. Read `experiment_logger.py`, then `logging_utils.py`
   and `cbf_diagnostics.py`, then the overlay and visualisation nodes, then the scripts (docstrings,
   inputs, outputs).

## C-034 — 2026-11-11 — Stage 5 — ch. 13 sources inventoried; observability checks written

- Baseline / HEAD / check: `ba68077052e8` / `a66a134` / (README link still pending)
1. **Read (Verified):**
   - `experiment_logger.py`: constructor, header, every callback, the start of `sample`,
     `_write_manifest`, `_build_kinematics`, the plot list;
   - `cbf_diagnostics.py`;
   - `logging_utils.py`: structure, `format_cbf_diag` docstring and call site;
   - `run_metadata.py`, `simulation_imports.py`;
   - the headers and interfaces of `trajectory_overlay.py`, `trajectory_trace.py`,
     `visualization.py`, `capsule_overlay_node`, `distance_visualization_node`, `tag_overlay_node`,
     `trajectory_overlay_node`, `trajectory_visualization_node`;
   - every ch. 13 script: summary, inputs (live/bag/CSV), outputs;
   - `ball_throw_eval.read_bag`, `latency_budget` (header, publishers `/lb/*`), `bag_replay.launch.py`
     (header), the torque-stack rosbag block;
   - the names of the ch. 13 tests; `make_ros_node_graph.py` and `trajectory_rviz.rviz` (interface).
2. **`check_observability.py` 11/11:**
   - F-5-21 reproduced on `ExperimentLogger.joint_state_cb`; Runtime (recorded) pattern in 26 % of
     the rows of four sim runs;
   - no age check; legacy h; the bag topic list; zstd vs `.db3`; `tau_max_abs`; `run_metadata`.
3. **Findings:** F-5-21 (finger rows corrupt the logger), F-5-22 (bag gaps), F-5-23 (misleading
   logger diagnostics).
4. **Exact next action:** write `13_observability_and_tools.md`; then README/`run_all.sh`/results
   for `tools/stage5_verification/`, the ledger, 00_INDEX, and the final checks.

## C-030 — 2026-11-11 — Stage 5 — chapter 10 written; ch. 13 inventory received

- Baseline / HEAD / check: `ba68077052e8` / `a66a134` / check_state (to run)
1. **Done:**
   - `10_alternative_controllers.md` (§1–7, equations 10.2–10.12);
   - ch. 01 §4.2 corrected (F-5-1, old wording kept).
2. **Ch. 13 subagent report received (UNVERIFIED until spot-checked).** Leads:
   - the logger reads only `/cbf_status` data[0..10] of 41;
   - the `cbf*`/`min_h` columns come from the legacy MultiDistance with the logger's own d_safe;
   - `write_runtime_metadata` has no production caller;
   - the TCS bag omits `/NS_1/iso_safety`, `/NS_1/torque_saturation` and `/rosout`;
   - the logger's `experiment_name` default is `"bag_02_dynamic_test"`;
   - `watch_obstacle_tracks` defaults to an unpublished topic;
   - `throw_clearance` hard-codes a nonexistent `rosbag/` dir;
   - `pipeline_rate_monitor` has stale nominal rates;
   - the `iso_constants_measure uncertainty` topic default is `/cams/d455/...`;
   - `distance_visualization_node` duplicates `real_time_distance`'s overlay.
3. **Exact next action:** spot-check those leads in a host script
   `tools/stage5_verification/check_observability.py`; then write `11_iso_safety_layer.md` and
   `13_observability_and_tools.md`.

## C-031 — 2026-11-11 — Stage 5 — observability facts verified

- Baseline / HEAD / check: `ba68077052e8` / `a66a134` / check_state (to run)
1. **Done:** `check_observability.py` (host) 13/13. It turned the subagent's main leads into Verified
   facts (two of my own slicing errors fixed; the `sim_runs` data directory is excluded from the
   caller scan).
2. **Findings:** F-5-16 … F-5-19 added to PROGRESS §4.
3. **Exact next action:** write `11_iso_safety_layer.md`, then `13_observability_and_tools.md`,
   then the stage5 README/run_all, ledger, 00_INDEX, final checks.

## C-035 — 2026-11-11 — Stage 5 — complete; parallel-session reconciliation; stopped for review

- Baseline / HEAD / check: `ba68077052e8` / `a66a134` / `check_state.py` consistent (exit 0)
1. **Parallel session (record).** A second Claude session (another terminal) worked on Stage 5 at
   the same time as this one. Its entries are the second C-030 and C-031 above, appended after
   C-034. They are left as written (append-only log): their numbers duplicate this session's
   C-030/C-031, and they are the parallel session's account, not a continuation of C-034.
   At 16:23–16:25 the parallel session:
   - replaced `tools/stage5_verification/check_observability.py` with its own seven host checks;
   - added four rows numbered F-5-16…F-5-19 to PROGRESS §4 (IDs already in use, with other
     meanings);
   - set the §0 "Last updated" to its C-031;
   - saved its own version of `11_iso_safety_layer.md` over this session's chapter.
2. **Reconciliation (approved by the user in this session; the other session was stopped first).**
   Nothing was deleted:
   - `11_iso_safety_layer.md`: this session's chapter was restored from the session record, because
     it is the version that F-5-17…F-5-20, `check_iso.py` 8–11, ch. 13 and the README cite. One
     verified row was added from the parallel draft (the test gaps). The parallel draft is kept
     verbatim in `drafts/11_iso_safety_layer.parallel_session.md`; only its relative links were
     changed, plus a header note.
   - `check_observability.py`: the two versions were merged. This session's checks are 1–4 and the
     parallel session's are 5–11, renumbered only; 24/24 pass.
   - PROGRESS §4: the four duplicate rows were renumbered **F-5-24…F-5-27**. Each row keeps its text
     and a note of its original ID; the check numbers now follow the merged script, and the section
     numbers follow the written ch. 13. The F-5-16…F-5-19 cited in the parallel C-031 and in the
     preserved draft mean F-5-24…F-5-27.
   - Ch. 13 integrates F-5-24…F-5-27: §2.1 (`run_metadata`), §2.4 (three columns), §4 (ages not
     in the CSV), §7 (stale tool defaults), §9 rows 14–16, §12.
3. **Stage 5 deliverables:**
   - `10_alternative_controllers.md`, `11_iso_safety_layer.md`, `13_observability_and_tools.md`;
   - `tools/stage5_verification/`: README, `run_all.sh`, `results/observed_2026-11-11.txt`;
     **92/92 pass** (host 5; container 19 + 16 + 28 + 24);
   - ledger: pending 178, partial 107, read 4, documented 57; 00_INDEX rows 10/11/13 written and a
     stage5 artefact row; PROGRESS §0/§1/§2/§5 (Q-5-1…Q-5-4).
4. **Final checks:**
   - every script was re-run after the last edit;
   - the equation references of the scripts and the README were checked against chapters 10/11/13
     (the old numbering in `check_oscbf.py` and `check_iso.py` was corrected);
   - cross-chapter `NN §x.y` references were checked against the headings;
   - `check_state.py` reports the relative links consistent;
   - `git diff --stat ba68077` on the five ROS packages is empty, and `git status` shows changes only
     under `docs/framework_study/`.
5. **Findings Stage 5:** F-5-1…F-5-27 (F-5-24…27 renumbered as above).
6. **Exact next action:** wait for the user's review of Stage 5. Stage 6 (`12_simulation.md`) is
   identified, **not started and not approved**.

## C-036 — 2026-11-11 — Stage 6 — started

- Baseline / HEAD / check: `ba68077052e8` / `bb71534` (Stage 5 committed and pushed by the user) /
  `check_state.py` consistent (exit 0)
1. **Approval:** the user approved ONLY Stage 6. Deliverables:
   - `12_simulation.md` and `SIMULATION_VALIDATION_PROTOCOL.md`;
   - `tools/stage6_verification/` (README, run instructions, expected and observed results,
     limitations).
   No Gazebo, no ROS launch, no source or configuration change, no commit. The protocol is
   documented, not executed.
2. **Single writer:** `ps` shows one Claude session (this one). The parallel session of C-035 is no
   longer running.
3. **Exact next action:** read the ch. 12 sources: launch files and plant, `sim_config`,
   `conditions`, `scenarios`, the sim nodes, then `simexp`, the batch path and the tests.

## C-037 — 2026-11-11 — Stage 6 — plant, world, obstacles and conditions read

- Baseline / HEAD / check: `ba68077052e8` / `bb71534` / consistent
1. **Read (Verified):**
   - the whole of `gz_cbf_stack.launch.py` and `franka_simulation/launch/gz_fr3_plant.launch.py`;
   - in `franka_simulation`: `utils/gz_robot_description.py` (all), `utils/gz_world.py` (all), and
     `src/scenario_obstacles_system.cpp` (laws, PreUpdate/PostUpdate, start, target);
   - in `simexp`: `obstacle_laws.py`, `scenario_laws.py`, `truth_geometry.py`, `sim_config.py`,
     `conditions.py` (all);
   - `conditions.yaml`: the `cbf:` block and the `full`/`minimal` blocks; every condition resolved
     with the production `resolve` and with `C3.effective_params('sim:<c>')`.
2. **Facts to carry:**
   - truth clearance is between the control-point spheres and the obstacle spheres;
   - the obstacles have no collision element;
   - `full` = `hw_default` minus go-around, at 100 Hz instead of 300 Hz;
   - `minimal` changes 19 effective parameters; its "no braking margin" and "no range uncertainty"
     flags are no-ops (`full` already has them off);
   - sim manifests carry a git SHA, unlike the logger manifests.
3. **Findings:** F-6-1 (no recorded run at the baseline), F-6-2 (`'1e-05'` string chain).
4. **Exact next action:** read `scenarios.yaml` + `scenario_catalog`, the sim nodes (supervisor,
   ground truth, reflex, scenario commander, emulated perception, markers), `depth_sim_adapter`
   and `config/sim/*`.

## C-038 — 2026-11-11 — Stage 6 — nodes, batch, scoring read; checks written

- Baseline / HEAD / check: `ba68077052e8` / `bb71534` / consistent
1. **Read (Verified):**
   - `scenario_catalog.py`, `scenarios.yaml` (all 100 names resolved);
   - the nodes `episode_supervisor`, `reflex_monitor` + `reflex_checks`, `ground_truth_node`,
     `scenario_commander`, `emulated_perception` + `emulation`;
   - batch and scoring: `run_gz_scenarios`, `runner`, `procs`, `batch`, `closed_loop`, `analysis`;
   - `estimator_metrics` (structure and `score_run`), the docstrings and function lists of
     `attribution`, `offline`, `provenance`, `graph_contract`, `control_regression` and `report`;
   - in `franka_simulation`: `depth_sim_adapter` + `utils/depth_conversion`, and
     `config/sim/{perception_sim,depth_adapter}.yaml`.
2. **Checks written:** `tools/stage6_verification/`:
   - `check_conditions.py` 11/11 (host);
   - `check_obstacle_laws.py` 17/17 (host);
   - `check_sim_runs.py` 2/2 (host);
   - `check_scoring.py` 23/23 (container);
   - `check_sim_geometry.py` 15/15 (container);
   - `check_installed.py` 1/1 (container).

   Test errors fixed: an over-broad text pattern, an invented armature bound (replaced by the exact
   disk bounds), and an over-strong "every run" claim.
3. **Findings:** F-6-3 … F-6-14 logged in PROGRESS §4.
4. **Exact next action:** write `run_all.sh`, the README and the observed results; then
   `12_simulation.md` and `SIMULATION_VALIDATION_PROTOCOL.md`; then ledger, index and progress
   updates.

## C-039 — 2026-11-11 — Stage 6 — complete; stopped for review

- Baseline / HEAD / check: `ba68077052e8` / `bb71534` (Stage 6 not committed) / consistent
1. **Deliverables:**
   - `12_simulation.md`: §1–§14, equations (12.1)–(12.22), all displayed; part A §2–§3, B §10,
     C §4, D §5–§7, E §8–§9; consequences §13; findings §14;
   - `SIMULATION_VALIDATION_PROTOCOL.md`: the ten-step rule; Phases 1–6; acceptance criteria with
     their sources, every other threshold marked REQUIRES USER APPROVAL; evidence preservation;
     revert. Documented, not executed;
   - `tools/stage6_verification/`: 6 scripts + `_common6.py`, `run_all.sh`, README,
     `results/observed_2026-11-11.txt`, **69/69 PASS** (rerun after the final edits).
2. **Updated:** PROGRESS §0, §1, §2, §4 (F-6-1…F-6-14), §5 (Q-6-1…Q-6-5), §7 (leads checked);
   00_INDEX (banner, ch. 12, protocol, stage 6 tools); ledger (ch. 12: 23 documented, 39 partial,
   1 pending; totals pending 130 · partial 134 · read 2 · documented 80).
3. **Verification:**
   - `check_state.py`: consistent (121 relative links in 32 files);
   - `git diff ba68077 -- franka_experiments franka_simulation_experiments franka_rt_controllers
     franka_msgs franka_simulation` is empty;
   - nothing launched: no Gazebo, no ROS graph, no pytest, no colcon.
4. **State warning:** the container's installed workspace ≠ the baseline source (F-6-3).
5. **Exact next action:** wait for the user's review. Stage 7 needs a separate, explicit approval.

## C-040 — 2026-11-11 — Stage 7 — started

- Baseline / HEAD / check: `ba68077052e8` / `476109b` (Stages 0–6 committed and pushed) / consistent
- Approval: the user's pasted prompt "execute ONLY Stage 7 — Final Synthesis, Code Classification,
  Coverage Audit and Documentation Consolidation". No refactoring, no Gazebo, no commits.
- Single writer: no other session is known to be writing to `docs/framework_study/`; tree clean at start.
- Done so far: `tools/stage7_verification/inventory.py` (static evidence per ledger file: definitions,
  static and string imports, entry points, launch references, test importers, chapter citations) →
  `results/inventory.{json,md}`.
- Exact next action: targeted reading of the unread safety-critical parts (`self_collision.py`,
  the uncited builders of `cbf_state_rows.py`, the uncited methods of `cbf_safety_filter.py`,
  `singularity_recovery.py`, the small pending utils), then the test traceability.

## C-041 — 2026-11-11 — Stage 7 — safety-critical reading pass 1; install provenance

- Baseline / HEAD / check: `ba68077052e8` / `476109b` / consistent (Stage 7 edits uncommitted)
1. **Read (in full):**
   - `utils/self_collision.py`;
   - `utils/singularity_recovery.py`;
   - `utils/cbf_utils.py`, `utils/node_utils.py`;
   - `scripts/pin_rt_thread.sh`;
   - both `package.xml`, `setup.py`, `setup.cfg`, `MANIFEST.in`, `CMakeLists.txt`, `pytest.ini`,
     the `.gitignore` files.
2. **Read (the functions no chapter cited):**
   - `cbf_state_rows.py`: retreat_*, joint_limit_risk_margin, risk_slack_weight, evidence_margin,
     fitted_rate, measured_curvature_radius, in_self_mask, in_self_shadow, memory_surface_gap,
     set_input_rate;
   - `cbf_safety_filter.py`: `_joint_state_frozen`, `_on_iso_safety`, `_iso_stop_active`;
   - `ros_setup`/`avoidance_math`: the velocity-stack model and joint ordering.
3. **Findings:**
   - F-7-1: the singularity recovery quintic ends at q̇ = v₀, and with \|v₀\| > 0.5 rad/s the plan
     diverges (457 s, tens of rad);
   - F-7-2: stale `TODO[LEGACY]` markers;
   - F-7-3: the `select_gamma` docstring;
   - F-7-4: the install is a mixed `fr3-student-sim` build (the cause of F-6-3; Q-6-1 answered).
   - Chapter 06 got a Stage 7 correction note (6.R1).
4. **Ledger:** documented `self_collision` (ch 08), `singularity_recovery` (ch 06), `cbf_state_rows`,
   `cbf_safety_filter`.
5. **Scripts:** `stage7_verification/inventory.py` and `check_install_provenance.py` (3/3 PASS).
6. **Exact next action:** write `check_unread_functions.py` (F-7-1, F-7-3, risk weight) and
   `check_legacy_markers.py`. Then the remaining pending non-test files (`rtd_debug`, hand-eye,
   calibration configs, `control_regression_scenarios.yaml`), then test traceability.

## C-042 — 2026-11-11 — Stage 7 — test traceability and build facts gathered

- Baseline / HEAD / check: `ba68077052e8` / `476109b` / consistent (Stage 7 edits uncommitted)
1. **Scripts:**
   - `check_unread_functions.py`: 9/9 PASS (F-7-1 reproduced 4 ways; F-7-3; the slack weight; the
     joint-limit risk margin; segment/segment vs brute force; retreat displacement = ∫ speed);
   - `test_traceability.py`: 128 test files, 1487 test functions, 0 stale imports. Infrastructure
     per file: pure 74, ros 22, config 25, textual 20, gazebo 5, data 3, pinocchio 2, ml 1.
     Replicas confirmed by hand: `test_phase_governor.py` (whole file mirrors the node),
     `test_cbf_velocity_filter.py::test_gamma_continuous` (inlines `select_gamma`), and one helper
     in `test_cbf_state_rows.py` (L732, mirrors `_obstacle_speed`). `test_cbf_multi_cp_qp` was a
     false positive of the regex.
2. **Facts for ch. 14:**
   - the Dockerfile (ros:humble-ros-base; robotpkg Pinocchio; pip pins numpy 1.26.4, qpsolvers
     4.3.3, osqp < 1.0, torch/mujoco/sb3/onnxruntime);
   - docker-compose (repo bind-mounted at `/ros2_ws/src`; `build`, `install` and `log` live in the
     container layer);
   - the entrypoint; the container versions (pytest 6.2.5, Pinocchio 4.0.0, scipy 1.15.3, osqp
     0.6.7.post3);
   - the colcon build history (mixed symlink and copy builds);
   - `franka_simulation` install facts added to F-7-4.
3. **Exact next action:** write `14_build_launch_test.md`. Then 15, 16 (consolidating F-0-1 …
   F-7-4), the guide comparison, A/B/C, `REFACTORING_READINESS.md`, `check_docs.py`.

## C-043 — 2026-11-11 — Stage 7 — chapter 14 written

- Baseline / HEAD / check: `ba68077052e8` / `476109b` / consistent (Stage 7 edits uncommitted)
1. **`14_build_launch_test.md`:**
   - workspace and packages; declared and undeclared dependencies; Docker;
   - per-package install mechanics; copy vs symlink;
   - where configuration is read from; the F-6-3/F-7-4 timeline and the detection commands;
   - a proposed revision-tied clean build (NOT EXECUTED); the config freshness loop; environment
     reproduction;
   - the launch-file table; test suites and how to run them; the F-7-5 cwd rule; replica and
     textual tests;
   - runtime diagnostics; the `d_safe` example; consequences of modification.
2. **F-7-5 logged:** the test cwd decides source vs install.
3. **Ledger:**
   - documented, ch 14: the 11 build-metadata files and `pin_rt_thread.sh`;
   - documented: `cbf_utils`, `node_utils`;
   - read: `control_regression_scenarios.yaml`;
   - partial (interface level): `rtd_debug`, `handeye_eye_in_hand_node`, the 2 hand-eye launches,
     `camera_EE_extrinsic.yaml`;
   - **every remaining `pending` test → `partial`**, meaning static traceability only (imports,
     infrastructure, test counts, docstrings, machine-extracted by `test_traceability.py`);
     assertions not read except those named in ch 14 §7.4.
4. **Exact next action:** `15_code_status.md` (classification), then 16, the guide comparison,
   A/B/C, REFACTORING_READINESS, `check_docs.py`.

## C-044 — 2026-11-11 — Stage 7 — chapter 15 written

- Baseline / HEAD / check: `ba68077052e8` / `476109b` / consistent (Stage 7 edits uncommitted)
1. **`15_code_status.md`:**
   - the classes and evidence routes;
   - classification tables for every production file of both packages: nodes, the Pipeline A core,
     perception, alternatives, no-consumer files, configuration, scripts, simulation;
   - the `TODO[LEGACY]` marker reconciliation (4 of 22 false or misleading);
   - the flag layer; the summary; removal rules.
2. **Corrections made while writing:**
   - the ISO nodes are not started by default (launch_defaults L292/L301);
   - `obstacle_sim`'s gate is `sim_obstacle.enabled`.
3. **Exact next action:** `16_issues_and_discrepancies.md`. Read all of PROGRESS §4 (F-0-1 …
   F-7-5); categorize, prioritize, cross-link; then the guide comparison.

## C-045 — 2026-11-11 — Stage 7 — chapter 16 Parts A–D written

- Baseline / HEAD / check: `ba68077052e8` / `476109b` / consistent (Stage 7 edits uncommitted)
1. **`16_issues_and_discrepancies.md`:** all 120 findings (F-0-1 … F-7-6) categorized into 11
   tables, with severity, priority, confidence, Num, CL, direction and links.
   - Part C: 12 root-cause groups, 10 corrections and 4 contradictions discussed.
   - Part D: the P0–P3 lists.
2. **Re-verified at the baseline** (independent of earlier stages):
   - F-4-1 (`obstacle_slack_mode: family`);
   - F-4-12 (`_update_constraints` needs both `_js` and `_obs`);
   - F-4-6 (the effort clip is under `iso_enabled`);
   - F-4-7 (`rt_torque_controller.cpp` L121: "nessuna saturazione SW").
3. **New: F-7-6** (a false comment about clipping).
4. **Script attributions in the Num column** were checked against the PROGRESS evidence and the
   stage READMEs; 7 corrected.
5. **Exact next action:** the guide comparison (Part E of ch. 16). Then A/B/C, REFACTORING_READINESS,
   `check_docs.py`, and the 00_INDEX rewrite.

## C-046 — 2026-11-11 — Stage 7 — chapter 16 Part E (guide vs code) written; resumed in a new account

- Baseline / HEAD / check: `ba68077052e8` / `476109b` / check_state exit 1 before this entry (expected
  mid-stage: PROGRESS §2 counts not refreshed since Stage 6; link to the not-yet-written
  `REFACTORING_READINESS.md` from ch. 14). Stage 7 edits uncommitted.
- **Resumption:** the previous account stopped after C-045 (usage limit). This session re-read
  CLAUDE.md, PROGRESS §0/§4, C-040…C-045, ch. 16, and the untracked Stage 7 files. Nothing had been
  saved after C-045: Part E was a 3-line stub; B, C, `REFACTORING_READINESS.md`, `check_docs.py`,
  and the Stage 7 README/run_all/results did not exist. The PROGRESS §0 resume card was still at C-040
  (the previous session did not overwrite it at C-041…C-045); corrected now.
1. **Analysed:** the old guide against the baseline, mechanically (`check_guide_vs_code.py`: §15.2,
   §15.3, §15.6/§15.11.5 digests, links, §15.1/§15.4 identifiers, `d_safe`, E1–E11) and by reading
   (§1.1, §6.7, §7.3–7.4, §8.7, §9.1–9.4, §10.2–10.4, §12.7, §13.1).
   - Key fact: the guide hashed **dirty trees**; identity is taken from its digests, not `ce80f36`.
2. **Remaining in this stage:** appendices A (regenerate), B (equation index), C (glossary);
   `REFACTORING_READINESS.md`; the coverage audit; the cross-chapter consistency pass;
   `check_docs.py` (ch. 16 already promises it), the Stage 7 README, `run_all.sh`, recorded results;
   00_INDEX final; final verification.
3. **Files read:** the guide (sections above); at the baseline: `cbf_safety_filter._solve`,
   `cbf_qp_assembly.build_row_rhs` (docstring), `test_rate_scaling.py` L240–360, `fr3_control.yaml`
   (the post-guide keys, `joint_limits`), `fr3_complete.yaml` (`tracking`), `experiment_logger`
   L150–160, `handeye_eye_in_hand_node` L200, `torque_control_stack` diff `ce80f36..ba68077`.
   No ledger status changed (all already `documented`/`partial` with their chapters).
4. **Documentation updated:** ch. 16 Part E (E.1–E.9), header (123 findings, "Part E"), B.8/B.9/B.11
   rows, D.3/D.4; PROGRESS §4 (F-7-7…F-7-9), §5 (Q-7-1…Q-7-4, which ch. 15/16 already cited but
   were never defined); new script `tools/stage7_verification/check_guide_vs_code.py` (21/21 PASS).
5. **Findings:** F-7-7 (shipped-config test contradicts the YAML; static), F-7-8 (the guide is stale;
   summary), F-7-9 (private rclpy attribute). F-0-9's "19 commits" is 20 by `git rev-list` (noted in
   E.1, F-0-9 not edited). Q-7-1…Q-7-4 added.
6. **Exact next action:** regenerate `A_file_index.md` from `ledger/ledger.json` (chapter, status,
   class from ch. 15); then write `B_equation_index.md` by collecting the numbered equations of
   chapters 04–13 with their `file::function` references.

## C-047 — 2026-11-11 — Stage 7 — appendices A, B, C written

- Baseline / HEAD / check: `ba68077052e8` / `476109b` / check_state exit 1 (only the missing
  `REFACTORING_READINESS.md` link remains; §2 counts refreshed below)
1. **Analysed / written:**
   - `A_file_index.md` regenerated by the new `tools/stage7_verification/gen_file_index.py` (stdout →
     file) from `ledger.json`, `inventory.json` and ch. 15: 346 files by primary chapter, with status,
     ch. 15 class, production importers, launch references, tests, citing chapters.
   - `B_equation_index.md` generated by the new `gen_equation_index.py`: 122 numbered equations of
     ch. 04–13; 106 with a code pointer (chapter tokens plus a `MANUAL` table whose every symbol was
     checked with `git grep` at the baseline); 16 without, listed (derivation steps and theory, e.g.
     (4.9)/(9.1) dynamics, (7.6)/(7.8) HOCBF theory).
   - `C_glossary.md` (hand-written): symbols, terms, and the cross-chapter symbol collisions
     (σ, s, α, λ, γ, C).
2. **Remaining:** `REFACTORING_READINESS.md`; coverage audit; consistency pass; `check_docs.py`; Stage 7
   README, `run_all.sh`, results; final `00_INDEX.md`; final verification.
3. **Ledger:** the old guide → `partial` (ch. 16, stage 7: the sections listed in 16 E.1). Counts now
   pending 18, partial 228, read 3, documented 97.
4. **Documentation updated:** A, B, C; PROGRESS §2.
5. **Findings / uncertainties:** none new. The ch. 15 class column in A is parsed from ch. 15 tables;
   grouped rows (e.g. `depth_intrinsics`/`rgb_intrinsics`) carry the group's combined class.
6. **Exact next action:** write `REFACTORING_READINESS.md` (10 sections of the stage prompt), using
   ch. 14 §5–§7 (build/tests), ch. 15 §8–§9 (classes, removal rules), ch. 16 Part D (priorities) and
   `SIMULATION_VALIDATION_PROTOCOL.md` (baseline runs).

## C-048 — 2026-11-11 — Stage 7 — REFACTORING_READINESS.md written

- Baseline / HEAD / check: `ba68077052e8` / `476109b` / (run after this entry)
1. **Written:** `REFACTORING_READINESS.md`, the 10 sections of the stage prompt: readiness verdict
   (plan yes, execute no); investigations I-1…I-6; removable candidates; do-not-remove list;
   regression tests by component (from `inventory.json`); missing tests (11 gaps); baseline
   simulations; user decisions; workstreams W0–W10; the safest sequence and the per-change gate.
   ch. 14 §7.4 link corrected to §6 (the gaps section).
2. **Remaining:** coverage audit (pending/partial production files), consistency pass, `check_docs.py`,
   Stage 7 README / `run_all.sh` / recorded results, final `00_INDEX.md`, final verification.
3. **Files read:** `inventory.json` (test importers of 32 safety-critical files), ch. 14 §7,
   ch. 15 §8–§9, protocol §0 and §10. No ledger change.
4. **Documentation updated:** `REFACTORING_READINESS.md` (new), ch. 14 L407.
5. **Findings:** none new. Test-importer counts are static (`inventory.py`), not executed coverage.
6. **Exact next action:** coverage audit: list `pending`/`partial` non-test production files from
   `ledger.json`; read the small safety-relevant ones not yet read; record the remaining gaps in
   PROGRESS §2 (Stage 7 block) and in 00_INDEX.

## C-049 — 2026-11-11 — Stage 7 — coverage audit

- Baseline / HEAD / check: `ba68077052e8` / `476109b` / consistent at C-048
1. **Analysed:** the `pending`/`partial` non-test files of the ledger, prioritising the shipped path.
   Read in full at the baseline: `utils/rate_scaling.py` (`FrameRateEstimator`, `tau_for_alpha`),
   `utils/timing_law.py` (`ExponentialTimingLaw`, `TrapezoidalTimingLaw`), `utils/params.py`.
   Added coverage addenda: ch. 03 §9, ch. 06 §14, ch. 08 §10.
2. **Remaining:** consistency pass; `check_docs.py`; Stage 7 README / `run_all.sh` / results; final
   `00_INDEX.md`; final verification.
3. **Ledger:** `params.py` → documented (ch 03), `timing_law.py` → documented (ch 06),
   `rate_scaling.py` → documented (ch 08). Totals: documented 100, read 3, partial 225, pending 18.
   PROGRESS §2 has the Stage 7 block with the full gap list (inventoried-only vs partially reviewed).
4. **Documentation updated:** ch. 03, 06, 08 addenda; PROGRESS §2, §4; ch. 16 (F-7-10 row, 124
   findings); REFACTORING_READINESS (count).
5. **Findings:** F-7-10 (`params.py` drops `description`; no descriptor; validation at declaration
   only). Runtime `ros2 param set` validation: Unverified.
6. **Exact next action:** write `tools/stage7_verification/check_docs.py` (findings ↔ ch. 16 rows,
   Q-IDs defined, ch. 15 names every non-test production file, equation references resolve, links,
   chapter template sections); run it; fix what it finds.

## C-050 — 2026-11-11 — Stage 7 — check_docs.py and the consistency pass

- Baseline / HEAD / check: `ba68077052e8` / `476109b` / consistent at C-049
1. **Written:** `tools/stage7_verification/check_docs.py` (11 documentation checks: findings ↔ ch. 16
   rows and total, cited F/Q IDs defined, ch. 15 names every non-test production file, equation
   references resolve, "Consequences" sections, links, generated appendices current, verification
   directories complete, resume card, no superseded QP-size statements). 13/14 PASS; the remaining
   FAIL is the missing Stage 7 README/run_all/results (next action).
2. **Consistency pass** (grep across all chapters): no chapter claims six slack families or ℝ¹³;
   every `d_safe` 0.15 mention is qualified (install, comment, docstring, or 1.5·d_safe); rates
   agree (perception 90 Hz, rebuild 50 Hz, QP 300 Hz, commander 100 Hz, joint states and RT 1 kHz);
   topic names agree, with the short forms noted as shorthand in C §1.
3. **Fixes made because of the checks:** ch. 15 §6.1 now names the two hand-eye launch files; ch. 15
   §6.3 gets a row for `franka_logs/plot_franka_log.py`; A regenerated.
4. **Findings:** F-7-11 (Stage 0's `franka_logs/` exclusion hides one tracked tool from the ledger;
   recorded, ledger not extended). 125 findings in total.
5. **Exact next action:** write `tools/stage7_verification/README.md` and `run_all.sh`; run all Stage 7
   host scripts; save `results/observed_2026-11-11.txt`; then the final `00_INDEX.md`.

## C-051 — 2026-11-11 — Stage 7 — complete; stopped for review

- Baseline / HEAD / check: `ba68077052e8` / `476109b` / check_state consistent (run after this entry)
1. **Done in this step:** `tools/stage7_verification/README.md` (the three kinds of check:
   defect reproduction, correct behaviour, static/documentation), `run_all.sh`,
   `results/observed_2026-11-11.txt` (47/47, exit 0; `inventory.py` and `test_traceability.py`
   refreshed their results first, then A was regenerated); final `00_INDEX.md` (status, chapter
   table with 14/15/16/R/A/B/C, reading paths, the Stage 7 tools row, the `franka_logs` exception).
2. **Final verification:**
   - Stages 2–6 `run_all.sh` re-run: 126 + 85 + 61 + 92 + 69 = 433/433 PASS (outputs not re-recorded;
     the stage `results/observed_2026-11-11.txt` files are unchanged);
   - Stage 7: 9 + 21 + 14 + 3 = 47/47 PASS (plus the two refresh scripts);
   - `check_docs.py`: findings ↔ ch. 16 (125), IDs, ch. 15 coverage, equation references, links,
     appendices current, resume card — 14/14;
   - `git diff --name-only ba68077 -- . ':!docs/framework_study' ':!CLAUDE.md'`: empty; no untracked
     file outside `docs/framework_study/`; 3 stashes untouched; branch `humble-mattia`.
3. **Remaining uncertainties:** every closed-loop consequence of the P0 findings (no baseline run
   exists, F-6-1, F-7-4); test outcomes at the baseline (not run, Q-7-3); the 11 "not re-checked"
   rows of 16 E.6; the coverage gaps of PROGRESS §2 (Stage 7 block).
4. **Exact next action:** wait for the user's review of Stage 7 and a decision on committing it. The
   refactor needs separate, per-change approval (REFACTORING_READINESS §10).

## C-052 — 2026-11-12 — Baseline preparation (no stage started) — workspace investigation and plan

- Baseline / HEAD / check: `ba68077052e8` / `f7089911decf` / check_state exit 0 (8 documentation-only
  commits; 3 stashes untouched; tree clean before this entry)
1. **Analysed (read-only, at the user's request; no stage, build, test or launch was run):** context
   recovery (CLAUDE.md, PROGRESS, CHECKPOINTS, 00_INDEX, REFACTORING_READINESS, the protocol, ch. 12,
   14, 15, 16); the container workspace. New tool `tools/baseline_prep/probe_workspace_state.py`
   (read-only; `docker cp … -` works on a stopped container), output in
   `tools/baseline_prep/results/observed_2026-11-12.txt`. Observations (to be logged as findings
   refining F-7-4 when the user approves the baseline-preparation phase and its ID prefix; not added
   to §4 now because `check_docs.py` pairs every finding with ch. 16):
   - O-1 the container `franka_ros2` is **stopped** (exit 137, finished 2026-11-11 17:14 UTC); image
     `3ab2ed057f0e` (2026-10-23). The only Dockerfile change since is comments/print text (`1139cdd`).
     No build since `build_2026-11-10_14-42-01`.
   - O-2 `franka_simulation_experiments` is itself mixed: `simexp/*.py` and the launch files are
     symlinks into the source (2026-10-29 symlink build), the 10 programs and 4 config YAMLs are
     copies. The 2026-11-10 copy build left the symlinks in place. Refines ch. 14 §4.3 ("the two
     target packages are copies"). Programs and `simexp` equal the baseline; `conditions.yaml`,
     `scenarios.yaml` equal `42df80e`; `run_control_regression` has no source in either revision.
   - O-3 `franka_experiments` (copy mode): 24 copied files equal `42df80e` and differ from the
     baseline (incl. `cbf_safety_filter`, `cbf_state_rows`, `real_time_distance`, `fr3_control.yaml`,
     `torque_control_stack.launch.py`), 1 differs from both (`self_shadow.py`), 6 leftovers equal
     the baseline, 2 stale `.orig`/`.bak_baseline`: F-7-4 unchanged.
   - O-4 `franka_msgs` (symlink build 2026-11-10 11:26 UTC, before `ba68077` added the six `shadow_*`
     fields): the generated IDL of all 10 messages equals the baseline `.msg` field names and order
     (built from the then-uncommitted tree). Types not compared.
   - O-5 `franka_simulation`: the 34 mapped copied files equal the baseline (incl.
     `config/sim/perception_sim.yaml`, copied 2026-11-06 09:37 UTC before its commit `4db4af2`);
     launch files and `scripts/utils` are symlinks into the source. `libfranka_scenario_obstacles.so`
     (2026-11-05 15:20 UTC, HEAD `03ef58c`, whose `.cpp` lacks the wander law the binary contains):
     equivalence with the baseline `.cpp` Unverified (F-7-4).
   - O-6 `libfranka_rt_controllers.so`, `libcbf_torque_controller.so`,
     `libfranka_ign_ros2_control-system.so`: built 2026-10-29 13:50 UTC with HEAD `976b997`, whose
     package sources equal the baseline (Derived; an uncommitted tree cannot be excluded).
   - O-7 the host power profile is `performance` (intel_pstate EPP `performance`); no `isolcpus`; no
     orphan Gazebo/ROS processes.
   - O-8 PROGRESS §0 still said Stage 7 was uncommitted at `476109b`; it is committed (`f708991`).
     Corrected in §0 by this checkpoint.
2. **Remaining:** everything in the baseline plan (reported to the user, awaiting approval): start
   the container, preserve the pre-rebuild state, clean rebuild, provenance check, tests, baseline
   Gazebo runs; the user decisions Q-6-3, Q-6-4, Q-7-3 and the evidence location (protocol §10).
3. **Files read (ledger status set):** none changed.
4. **Documentation files updated:** this entry; PROGRESS §0; new `tools/baseline_prep/`.
5. **Findings / uncertainties:** O-1…O-8 above (pending IDs). No F- or Q- ID added or closed.
6. **Exact next action:** wait for the user's decision on the baseline plan (step 1: `docker start
   franka_ros2` and the pre-rebuild preservation). Run nothing before that approval.

## C-053 — 2026-11-12 — Baseline preparation, step S1 (preservation) — done; stopped

- Baseline / HEAD / check: `ba68077052e8` / `f7089911decf` / check_state exit 0
1. **Done (S1 only, approved by the user on 2026-11-12):** the pre-rebuild state was preserved,
   read-only, from the **stopped** container (`docker cp … -`), into
   `~/franka_baseline_evidence/baseline_f708991/00_pre_rebuild/` (1.2 GB, 22 files, `SHA256SUMS`
   verified, write-protected): `/ros2_ws/install`, all of `/ros2_ws/build`, `/ros2_ws/log`; container
   and image inspection; apt and pip versions; the probe re-run (unchanged since C-052); Git, stash,
   reflog and host/CPU provenance; the documentation patch and copies of the untracked files.
   Lightweight manifest: `validation/00_baseline/PRESERVATION_MANIFEST.md`.
   User decisions recorded there: evidence outside Git under `~/franka_baseline_evidence/baseline_f708991/`;
   summaries under `docs/framework_study/validation/00_baseline/`; O-1…O-8 stay observations; F-6-5
   and F-6-11 are not fixed in the harness for now; six-package rebuild tentatively preferred, to be
   verified before S3.
2. **Remaining:** S2 (`docker start` + read-only dependency checks), S3, S4, S5, runs; `--filter-cpu`;
   Q-6-3, Q-6-4.
3. **Files read (ledger status set):** none changed.
4. **Documentation files updated:** this entry; PROGRESS §0; new `validation/00_baseline/`.
5. **Findings / uncertainties:** none added. Nothing was deleted, moved or overwritten (the one file
   removed was a wrong, seconds-old output of this step, rewritten correctly before checksumming).
6. **Exact next action:** wait for the user's approval of S2. Run nothing before it.

## C-054 — 2026-11-12 — Baseline preparation, step S2 (container start + dependency verification) — done; stopped

- Baseline / HEAD / check: `ba68077052e8` / `f7089911decf` / check_state exit 0
1. **Done (S2 only, approved by the user on 2026-11-12):** S1 evidence re-verified (22/22, read-only);
   `docker start franka_ros2` → running (entrypoint `vcs import --skip-existing` skipped both
   checkouts, wrote nothing); read-only dependency, workspace, plugin and import checks; the existing
   F-7-4 checks re-run (unchanged: `INSTALLED != SOURCE`); colcon dependency graph. Results:
   `validation/00_baseline/S2_DEPENDENCY_CHECK.md`. S3 proposal: `validation/00_baseline/S3_BUILD_PLAN.md`
   (six packages, copy mode, Release, per-package clean, underlay-only environment; S4 checks; risks).
   Its preconditions and the `console_cohesion` handler were checked; the build itself is untested.
2. **Remaining:** S3 (approval), S4, S5, runs; `--filter-cpu`; Q-6-3, Q-6-4.
3. **Files read (ledger status set):** none changed.
4. **Documentation files updated:** this entry; PROGRESS §0; `validation/00_baseline/S2_DEPENDENCY_CHECK.md`,
   `S3_BUILD_PLAN.md`.
5. **Findings / uncertainties:** observations O-9…O-13 (observations, not findings): PYTHONPATH trailing
   `:`; two Pinocchio installs (4.0.0 used, 4.1.0 unused); four non-rebuilt packages link `franka_msgs`
   (not loaded in sim); untracked files in the `franka_description` checkout; `tf_transformations`
   absent and unused. No missing dependency or version conflict for the in-scope code. A start-up
   "new untracked files" alarm was a misread of the `franka_description` checkout's status.
   The container is left **running** (idle `/bin/bash`).
6. **Exact next action:** wait for the user's approval of S3 (`S3_BUILD_PLAN.md` §3). Run nothing
   before it.

## C-055 — 2026-11-12 — Baseline preparation, step S3 (clean six-package build) — done; stopped

- Baseline / HEAD / check: `ba68077052e8` / `f7089911decf` / check_state exit 0
1. **Done (S3 only, approved by the user on 2026-11-12):** preconditions passed; S1 re-verified;
   the 12 package directories (`build/` + `install/` of `franka_msgs`, `franka_ign_ros2_control`,
   `franka_rt_controllers`, `franka_simulation`, `franka_experiments`, `franka_simulation_experiments`)
   were **moved** into `/ros2_ws/quarantine_s3_20261112/` (with `COLCON_IGNORE`; nothing deleted);
   `colcon build --base-paths src --packages-select <six> --cmake-args -DCMAKE_BUILD_TYPE=Release`
   from the ROS underlay only (contamination guard passed) → **6/6 finished, 34.9 s, exit 0**; no
   compiler/linker warning (only byte-compile-disabled notices and the eigenpy FindBoost message).
   Sanity: copies only (0 symlinks), plugins and executables present, leftovers gone, installed
   `d_safe` 0.10; Git status, stashes and the source tree unchanged; container changed only under
   build/install/log/quarantine. Record: `validation/00_baseline/S3_BUILD_RESULT.md`; evidence
   `~/franka_baseline_evidence/baseline_f708991/01_build/` (write-protected, `SHA256SUMS`).
   Deviations from `S3_BUILD_PLAN.md`: quarantine instead of `rm -rf` (user preference) and
   `--base-paths src` (discovery safety).
2. **Remaining:** S4 (approval): source/install equivalence, message ABI vs the S1 archive, plugin
   `ldd`/CMake cache/symbols, import guard; then F-6-3/F-7-4 re-assessment (documentation). Then S5,
   runs; `--filter-cpu`; Q-6-3, Q-6-4; the fate of the quarantine.
3. **Files read (ledger status set):** none changed.
4. **Documentation files updated:** this entry; PROGRESS §0; `validation/00_baseline/S3_BUILD_RESULT.md`.
5. **Findings / uncertainties:** none added. The install is **not yet validated** (S4). F-6-3/F-7-4 are
   not marked superseded until S4 confirms `INSTALLED == SOURCE`.
6. **Exact next action:** wait for the user's approval of S4 (`S3_BUILD_PLAN.md` §4). Run nothing before it.

## C-056 — 2026-11-12 — Baseline preparation, step S4 (installation and source consistency) — done; stopped

- Baseline / HEAD / check: `ba68077052e8` / `f7089911decf` / check_state exit 0
1. **Done (S4 only, approved by the user on 2026-11-12; quarantine kept by the user's decision):**
   read-only verification of the S3 install. `check_installed.py` → `INSTALLED == SOURCE`;
   `check_install_provenance.py` → 178/178 = `ba68077`, checks 2–3 fail **by design** (obsolete
   historical assertions, script unchanged); probe → copies only, all `=base`, plugins built by S3.
   New read-only `tools/baseline_prep/check_s4_install.py`: 24 PASS, 0 FAIL — all 1158 installed files
   classified by content (303 tracked-equal, 797 S3-generated, 32 ament templates, 26 copied caches);
   `d_safe` 0.10; `franka_msgs` generated sources (495) and library symbols identical to S1, the four
   non-rebuilt linkers resolve; plugins link correctly (Pinocchio 4.0.0, `franka_msgs` from the new
   install), registered in pluginlib, one Gazebo search-path match each; imports from ten contexts load
   only baseline files. Install unchanged since S3; quarantine identical to S3; no repository or
   container change. Record: `validation/00_baseline/S4_INSTALL_VERIFICATION.md`; evidence
   `~/franka_baseline_evidence/baseline_f708991/02_install_verification/` (write-protected).
2. **Remaining:** S5 (tests, approval), runs; `--filter-cpu`; Q-6-3, Q-6-4; the user's decision on
   O-14 (finding ID or not); the fate of the quarantine.
3. **Files read (ledger status set):** none changed.
4. **Documentation files updated:** this entry; PROGRESS §0; PROGRESS §4 F-6-3, F-7-4, F-7-5 and §5
   Q-7-3 (dated re-assessments appended, original text kept); `validation/00_baseline/S4_INSTALL_VERIFICATION.md`;
   `tools/baseline_prep/check_s4_install.py`.
5. **Findings / uncertainties:** observations, not findings: **O-14** `franka_simulation/CMakeLists.txt`
   L144 copies the gitignored `scripts/utils/__pycache__` into the install (26 files; 3 cpython-310
   caches would be loaded but equal a fresh compile, so no stale bytecode runs today; the install is
   not a pure function of the commit). **O-15** `fr3_student_sim`'s symlink install puts
   `/ros2_ws/build/fr3_student_sim` early on `sys.path` (namespace names `config`, `launch`, …);
   no in-scope import is shadowed. **O-9 refined** (the empty entry sits after the install entries,
   before the standard library). Unverified until a runtime step: pluginlib/Gazebo loading and DDS
   message compatibility.
6. **Exact next action:** wait for the user's approval of S5 (tests). Run nothing before it.

## C-057 — 2026-11-12 — Baseline, step S5 (automated test execution) — done; stopped

- Baseline / HEAD / check: `ba68077052e8` / `f7089911decf` / check_state exit 0
1. **Done (S5 only, approved by the user on 2026-11-12):** every test needing neither Gazebo nor
   hardware, run in `franka_ros2` against the S3/S4 install (`source install/setup.bash`,
   `PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1`, import guard per run).
   - Suite 1, `franka_experiments` pytest from the package directory: 1427 collected, 1383 passed,
     **35 failed**, 9 skipped (26.7 s).
   - Suite 2, `franka_simulation_experiments` (not slow): 121 passed, 22 deselected (Gazebo).
   - Suite 3, `franka_simulation/test/sim`: 39 passed.
   - Smoke scripts:
     - `smoke_cbf_safety_filter` PASSED (one process, 300 Hz, the row binds, both braking fallbacks);
     - `smoke_cbf_construct` FAILED, while exiting 0 (stale stub);
     - `smoke_rl_policy_commander` NOT RUN (no ONNX model).
   - `colcon test` of the six packages:
     - lint passes in `franka_msgs` and `franka_ign_ros2_control`;
     - `franka_simulation` fails 1155 style checks;
     - `franka_rt_controllers` and `franka_simulation_experiments` have no registered tests;
     - the `franka_experiments` re-run gives the identical set of 35 failures.

   The 35 failures fall into four causes, all pre-existing:
   - A: 25 tests written ahead of the unwired features, as announced in `2402e58`;
   - B: 1 (F-7-9, the private rclpy API);
   - C: 1 (F-7-7, `max_coast_s` 0.8 against 0.167);
   - D: 8 (stale launch golden, F-8-1).

   The 9 skips are all NOT RUN, the RL parity tests (F-0-10).

   No Gazebo, launch or ros2_control process was started. Git status and stashes are unchanged;
   install and quarantine are unchanged. Record: `validation/00_baseline/S5_TEST_RESULTS.md`;
   evidence `~/franka_baseline_evidence/baseline_f708991/03_tests/` (31 files, write-protected).
2. **Remaining:** the Gazebo smoke test (S6, needs approval).
   - The 22 slow Gazebo tests and the launch graph contract are unexecuted.
   - User decisions: F-8-1 (regenerate the golden), test fixes, an RL model for the smoke test,
     `--filter-cpu`, Q-6-3, Q-6-4, O-14, the quarantine.
3. **Files read (ledger status set):** none changed (test files and the stub were read for the
   diagnosis only).
4. **Documentation files updated:**
   - this entry;
   - `validation/00_baseline/S5_TEST_RESULTS.md` (new);
   - PROGRESS §0;
   - PROGRESS §4: new subsection with F-8-1…F-8-4; dated outcomes appended to F-0-3, F-0-10, F-7-7
     and F-7-9 (original text kept);
   - PROGRESS §5 Q-7-3 (status appended);
   - ch. 16 B.10: rows F-8-1…F-8-4, total 125 → 129.
5. **Findings / uncertainties:**
   - **F-8-1** stale launch golden;
   - **F-8-2** no green automated gate (pytest red by construction; `colcon test` exits 0 and skips
     the simexp suite);
   - **F-8-3** `smoke_cbf_construct.py` is stale and cannot fail;
   - **F-8-4** `franka_simulation` lint.

   Observations, not findings:
   - **O-16**: `resolve_log_dir`'s write probe touches the gitignored `franka_experiments/franka_logs/`
     (the directory mtime only; the earlier file count was not recorded);
   - **O-17**: production code leaves temporary URDF and YAML files in the container's `/tmp`;
   - **O-18**: pytest's basetemp rotation removed three pre-S5 `/tmp/pytest-of-user` directories.

   Procedural note: `smoke_cbf_construct` was first run from the package directory. The stubs assume
   the repository root, so it was then run from there. Both logs are kept; both fail.
6. **Exact next action:** wait for the user's approval of S6 (the Gazebo smoke test). Run nothing
   before it.

## C-058 — 2026-11-12 — Baseline, step S6A (one Gazebo smoke test) — done; stopped

- Baseline / HEAD / check: `ba68077052e8` / `f7089911decf` / check_state exit 0
1. **Done (S6A only, approved by the user on 2026-11-12):**
   - **Preflight passed:**
     - install == S3 checksums; `check_installed.py` `INSTALLED == SOURCE`; packages resolve to
       `/ros2_ws/install`;
     - no ROS or Gazebo process; resources free;
     - simulation only (`external_plant:=true` skips the franka bringup; the hardware is
       `franka_ign_ros2_control/IgnitionSystem`; `eno1` down; DDS domain 77);
     - `--filter-cpu` not used: no supported choice (no isolated core; the only recorded pinned run
       of this cell timed out). Reported before launching.
   - **One cell, once:** `run_gz_scenarios --scenarios static --conditions full --seeds 0
     --perception true --batch-dir /home/user/s6a_20261112/batch`. 24 s wall; RTF 0.69; episode
     7.170 → 11.180 s sim.
   - **Pipeline verified end to end:**
     - the controllers were active;
     - nominal 300 Hz, rows 30.7 Hz, QP / safe / torque 73.6 Hz;
     - joint 1 moved 0 → 0.7993 rad (goal 0.8; `task_done`, error 9.1e-4 rad);
     - effort = g(q) + `torque_cmd` (Q-1-1 confirmed at runtime).
   - **Safety:**
     - `min_h` +0.068 m, no contact; the truth grid is valid near the minimum (hole ≤ 3 ms;
       38 ms max in the episode);
     - reflex PASS; 0 faults and 0 fallbacks; slack ≤ 3.5e-4.
   - **Limits of what it shows:**
     - no obstacle row ever bound (`n_act = 0`); the only intervention was the 6 rad/s² acceleration
       box;
     - the pentagon commander was not exercised.
   - **Harness verdict:** `ok = False` because of the QP-rate miss (F-8-5).
   - **Cleanup:** only the processes of this run, plus the probe's `ros2` daemon, which I stopped.
   - Git, stashes, install, quarantine and earlier evidence are unchanged; no repository file was
     written.
   - Record: `validation/00_baseline/S6A_GAZEBO_SMOKE.md`; evidence
     `~/franka_baseline_evidence/baseline_f708991/04_gazebo_smoke/` (90 files, write-protected).
2. **Remaining (each needs approval):**
   - F-8-5 handling (rate, build cost, pinning, governor);
   - a static cell where the barrier binds;
   - thresholds (Q-6-3) and seeds;
   - Tier 1 / Tier 2;
   - F-8-1, F-8-6, F-8-7;
   - O-14 and the quarantine.
3. **Files read (ledger status set):** none changed.
4. **Documentation files updated:**
   - this entry;
   - `validation/00_baseline/S6A_GAZEBO_SMOKE.md` (new);
   - PROGRESS §0;
   - PROGRESS §4: F-8-5…F-8-7 added; dated notes on F-6-5 and F-6-11;
   - PROGRESS §5 Q-1-1 (runtime confirmation);
   - ch. 16 B.10: rows F-8-5…F-8-7, total 129 → 132.
5. **Findings / uncertainties:**
   - **F-8-5** the QP rate is missed in `full` (73.6 vs 100 Hz; compute-bound, Derived; pinning
     effect untested);
   - **F-8-6** unclean teardown (`gz_server` SIGABRT; Python SIGINT errors);
   - **F-8-7** perception runs at 30.7 of 90 Hz, distances 112 ms old.

   Inferred:
   - `ROS_LOCALHOST_ONLY` may be ineffective with the container's FastDDS XML profile; isolation rests
     on domain 77.
   - The −13.5 mm perception offset mixes geometric models.
6. **Exact next action:** wait for the user's decision. Run no further simulation, Tier 1, Tier 2 or
   refactor without explicit approval.

## C-059 — 2026-11-12 — Baseline, step S6A.1 (offline performance and timing analysis) — done; stopped

- Baseline / HEAD / check: `ba68077052e8` / `f7089911decf` / check_state run at the end of this step
1. **Done (S6A.1 only, approved by the user on 2026-11-12; no simulation, launch or build):**
   - Inputs: the S6A recording (`04_gazebo_smoke/`, hash-verified; the container copy hash-checked
     equal for the five main files), the code, chapters 07/08/09/12/13, the protocol.
   - **QP rate.**
     - The CBF timer (rclpy `create_timer` with `use_sim_time`), the bag (`--use-sim-time`), the
       episode window and the 100 Hz nominal are all sim time: 73.6 is per sim s, ≈ 46 per wall s.
       The user's 106.7 reading is refuted.
     - 106 of 401 periods missed; worst gap 91 ms (`accel_timeout` 100 ms).
     - Before START the same timer held 99.6 per sim s. In the episode, late ticks follow a fresh
       rebuild 51 % of the time against 2 % otherwise; rebuilds achieved 36.2 of 50 per sim s.
     - Gazebo slowdown is excluded as a direct cause. In-process contention (GIL / executor) is
       the leading hypothesis (Inferred); compute versus OS contention is Unverified.
   - **Perception.**
     - Captures lie on the 90 Hz sim grid; 123 of about 361 frames were processed, the rest
       dropped by design (`Queue(1)`).
     - The node is compute-saturated (57–65 ms wall, `wait` 0) and wall-bound (≈ 19 per wall s
       both before START and in the episode).
     - The capture age (p50 112 / p95 154 ms) exceeds `latency_t_blind` 85 ms; h̄ uses
       capture-time `pr`.
   - **RTF.** Whole run 0.69 (startup included); before START ≈ 0.82; episode 0.62–0.64. The
     contributors were not measured (Inferred list).
   - **S6A correction.** "No obstacle row bound / only the box" is withdrawn:
     - `n_act` = count(h̄ < 0);
     - the slew bound also acted;
     - the gap-dependent link-speed cap was emitted in 7.43–7.70 s;
     - a box + slew residual up to 1.17 rad/s² remains.

     C-058 item 1 "Limits" is superseded on this point (this entry; C-058 itself is unchanged).
   - **Next experiment proposed (not run):** `pentagon_static_baseline_bare / full / s0 /
     perception true`, resolved offline. The nominal gap is 5.0 cm against `d_safe` 10 cm (Derived;
     the lead from `2d87ab6` / `minimal` measured 5.00 cm). The acceptance criteria V/B/C/S need
     user approval.
   - Evidence `~/franka_baseline_evidence/baseline_f708991/05_offline_timing/` (12 files, SHA256SUMS
     hash `39821592…0127`, write-protected). Container leftovers kept:
     `/home/user/s6a1_20261112_analysis/`, `/home/user/s6a1_*.py`.
2. **Remaining (each needs approval):**
   - the proposed cell and its criteria;
   - the F-8-5 policy;
   - the S6A1 §1.7 measurements;
   - thresholds (Q-6-3) and seeds;
   - Tier 1 / Tier 2;
   - F-8-1, F-8-6, F-8-7;
   - O-14 and the quarantine.
3. **Files read (ledger status set):** none changed (all files consulted are already `documented`).
4. **Documentation files updated:**
   - this entry;
   - `validation/00_baseline/S6A1_PERFORMANCE_ANALYSIS.md` (new);
   - `validation/00_baseline/S6A_GAZEBO_SMOKE.md` (four dated correction notes, original text kept);
   - PROGRESS §0;
   - PROGRESS §4: F-8-8…F-8-11 added; dated notes on F-8-5, F-8-7 and F-1-16; the IDs note;
   - ch. 16: F-8-9 (B.5), F-8-8 and F-8-10 (B.7), F-8-11 (B.11); the F-8-5 and F-8-7 rows refined;
     total 132 → 136.
5. **Findings / uncertainties:**
   - **F-8-8** tick-gap warning mixes clocks;
   - **F-8-9** capture age > `latency_t_blind`, and capture-time robot geometry in h̄;
   - **F-8-10** timing metrics depend on RTF and host;
   - **F-8-11** S6A documentation error.

   Unverified:
   - the camera publication rate;
   - whether rendering stalls physics;
   - the compute versus contention split;
   - which constraint shaped the command after 7.70 s.

   Observation: the per-node `ros_log` file of `real_time_distance` lost 2 lines at teardown;
   `launch.log` is complete.
6. **Exact next action:** wait for the user's decision. Run no simulation, Tier 1, Tier 2 or
   refactoring, and in particular not the proposed cell, without explicit approval.

## C-060 — 2026-11-12 — Baseline, step S6B (one static-obstacle experiment) — done; stopped

- Baseline / HEAD / check: `ba68077052e8` / `f7089911decf` / check_state and check_docs run at the end of this step
1. **Done (S6B only, approved by the user on 2026-11-12; ONE run):**
   - Cell: `pentagon_static_baseline_bare / full / seed 0 / perception true`, unpinned, run exactly once.
     Runner: `ok=False min_h=+0.023 rtf=0.65 qp=88Hz!=100 RATE_FAIL wall=74s`, exit 0. `ok=False`
     comes only from the QP-rate gate.
   - Preflight: install == source (1158 files); working-tree patch saved; git status, stash and HEAD
     identical after; 0 repository files newer than the launch marker; docker-diff delta 97 entries,
     all run scratch, launch temp files and logs; no processes left.
   - Side monitors (no production change): a host `/proc` sampler (4 Hz, ≈ 6 % of one core) and a
     second, wall-clock bag of `/NS_1/cbf_status` and `/cbf/per_link_distances`. The sampler missed
     the wall recorder's own pid (README).
   - **Results:**
     - pentagon 4.00 laps, final error 3.8 mm;
     - true min gap **12.27 cm** (`fr3_link8#2`); 0 s below d_safe on the samples;
     - pass 3 minimum inside a 395 ms truth hole (Unverified);
     - obstacle slack on 1214 / 2654 ticks; obstacle rows with h_qp < 0 on 41 CBFDIAG lines, each
       with `cart_rad` > 0;
     - `dnorm` p50 3.95 near vs p95 0 far; EE −x deviation median 5.2 cm near vs +0.2 cm far;
     - link-speed cap emitted but not at its bound; qlim, sc and sing not binding;
     - pre-QP shaping (tangential bias, zone / state fade) and the commander governor (σ p50 0.71
       near) also acted.
   - **Performance:**
     - QP 88.4 per sim s, 54.7 per wall s, RTF 0.618; max sim gap 64 ms;
     - late ticks after a rebuild 33.4 % vs 0.68 %;
     - filter process 106 % of one core over 4 threads (GIL Inferred);
     - perception 36.5 per sim s, capture age at the filter p50 108 / p95 132 ms;
     - perceived − true gap median −2.26 cm, max +0.78 cm.
   - Fallbacks, faults, trips: none; 82 torque-rate clips.
   - Evidence `~/franka_baseline_evidence/baseline_f708991/06_static_obstacle/` (120 files, 95 MB,
     SHA256SUMS hash `28bbc883…c053`, write-protected). `04_gazebo_smoke` and `05_offline_timing`
     re-verified OK. Container scratch kept: `/home/user/s6b_20261112/`,
     `/home/user/s6b_20261112_analysis/`, `/home/user/s6b_analyze.py`, `/tmp/s6b_marker`,
     `/tmp/launch_params_*`.
2. **Remaining (each needs approval):**
   - accept or reject S6B;
   - the S6A1 §6.4 thresholds (Q-6-3);
   - the `pentagon_clear` reference and more seeds;
   - the S6B §10 items;
   - the F-8-5 policy;
   - Tier 1 / Tier 2;
   - F-8-1, F-8-6, F-8-7;
   - O-14 and the quarantine.
3. **Files read (ledger status set):** none changed (all files consulted are already `documented`).
4. **Documentation files updated:**
   - this entry;
   - `validation/00_baseline/S6B_STATIC_OBSTACLE.md` (new);
   - PROGRESS §0;
   - PROGRESS §4: F-8-12 added; dated notes on F-4-1, F-6-5, F-8-5, F-8-9 and F-8-11; the IDs note;
   - ch. 16: the F-8-12 row (B.7); the F-4-1, F-6-5 and F-8-5 rows refined; total 136 → 137.
5. **Findings / uncertainties:**
   - **F-8-12**: the bag's q̈_safe − q̈_nom is not evidence of QP intervention (the shaped nominal is
     unpublished, there is no per-tick active set).

   Unverified:
   - the pass-3 hole window;
   - mesh-level contact;
   - the per-centimetre split between the barrier, the shaping and the governor;
   - whether the box or slew bound the QP solution per tick;
   - the GIL mechanism;
   - the effect of perception latency on the path.
6. **Exact next action:** wait for the user's decision. No second seed, no reference run, no other
   scenario, no Tier 1 / Tier 2 or refactoring without explicit approval.

## C-061 — 2026-11-12 — Baseline, step S6B.1 (offline clearance reconstruction) — done; stopped

- Baseline / HEAD / check: `ba68077052e8` / `f7089911decf` / check_state and check_docs run at the end of this step
1. **Done (S6B.1 only, approved by the user on 2026-11-12; no simulation, launch or build):**
   - Inputs: the S6B cell (`06_static_obstacle/`, SHA256SUMS OK before and after; the container
     copy checked against the same hashes). The installed geometry code and configuration
     (`truth_geometry.py`, `distance_utils.py`, `fr3_complete.yaml`) have sha256 equal to the source.
   - **Gaps:** 7 truth holes > 2 ms in the episode (91, 174, 336, **395**, 484, 3 and 562 ms, at 15,
     20, 25, **30**, 35 and 40 s), plus isolated single-sample drops after them. 2047 samples are
     missing from the truth. Before the episode: a 172 ms hole, an 18 ms hole and 3 ms holes, with
     the obstacle parked.
   - **Measured joints:** `/NS_1/franka/joint_states` (the plant's `joint_state_broadcaster`, the
     truth node's own input). 29 996 samples in the episode, max spacing 2 ms (15 intervals), 1 ms
     inside every hole. Name order `j1, j3, j2, j4…j7`, mapped by name.
   - **Obstacle:** `/sim/truth/obstacle_poses` at 1 kHz, constant p0 (0.55, −0.25, 0.449),
     r 0.05 m.
   - **Reconstruction** (FK + `define_control_points` + `surface_gaps`, the truth node's own code):
     - 27 949 / 27 949 truth samples matched by stamp; Δq = 0 and Δgap = 0;
     - one exception at t = 10.550 (the truth used the parked pose);
     - method check (FK of the truth's own q): error 0.
   - **Result** (sphere model):
     - episode minimum 0.12272 m at 36.927 s (`fr3_link8#2`); continuous-time bound ≥ 0.12236 m;
     - per pass 0.12281 / 0.12284 / **0.12313 (pass 3, inside the hole at 30.041 s)** / 0.12272;
     - 0 samples below 0.10 m and below 0; no unverified interval.
   - Not claimed: mesh-level contact (links 1–3 have no CP; the hand extends beyond its spheres;
     the Gazebo obstacle has no collision element), other seeds, moving obstacles.
   - Scripts: `tools/baseline_validation/` (`s6b1_reconstruct_clearance.py`, `s6b1_supplementary.py`,
     README).
   - Evidence: `~/franka_baseline_evidence/baseline_f708991/07_clearance_audit/` (11 files, 11 MB,
     SHA256SUMS hash `102256fc…7b27`, write-protected). `04`, `05` and `06` re-verified OK.
   - Container scratch kept: `/home/user/s6b1_20261112/`, `/home/user/s6b1_reconstruct_clearance.py`,
     `/home/user/s6b1_container_sums.txt`.
2. **Remaining (each needs approval):**
   - accept the S6B clearance result;
   - adopt the bag reconstruction as a clearance check, or fix the truth logger (F-6-5, F-8-13);
   - the `pentagon_clear` reference and more seeds;
   - the S6B §10 items;
   - the F-8-5 policy;
   - Tier 1 / Tier 2;
   - F-8-1, F-8-6, F-8-7;
   - O-14 and the quarantine.
3. **Files read (ledger status set):** none changed. `ground_truth_node.py` and `truth_geometry.py`
   were consulted; both are already covered by chapter 12.
4. **Documentation files updated:**
   - this entry;
   - `validation/00_baseline/S6B1_CLEARANCE_AUDIT.md` (new);
   - `tools/baseline_validation/` (new: two scripts and a README);
   - `S6B_STATIC_OBSTACLE.md` (two dated notes, original text kept);
   - PROGRESS §0;
   - PROGRESS §4: F-8-13 added; a dated note on F-6-5; the IDs note;
   - ch. 16: the F-8-13 row (B.7); the F-6-5 row refined; total 137 → 138.
5. **Findings / uncertainties:**
   - **F-8-13**: the truth pairs joint samples with the last received pose; 49–50 ms ahead after each
     hole; it matters only for moving obstacles.

   Unverified:
   - mesh-level clearance;
   - the frame offset beyond the perception cross-check;
   - inter-sample speed above the finite-difference maximum.
6. **Exact next action:** wait for the user's decision. No simulation, second seed, reference run,
   Tier 1 / Tier 2 or refactoring without explicit approval.
