# 05 — Perception and distance: from a depth pixel to `MultiLinkDistance`

> **Source baseline:** `ba68077`. Written in Stage 2 (2026-11-11). Labels as in
> [00_INDEX.md](00_INDEX.md) §2. The numerical checks were run as in
> [04](04_math_foundations.md) §7: baseline source imported inside the `franka_ros2` container,
> nothing in the repository changed.
>
> **Reproducible checks.** Every number labelled *numerically checked*, *measured* or *reproduced* in
> this chapter is recomputed by a script in
> [tools/stage2_verification/](tools/stage2_verification/README.md). Run `run_all.sh` there; it is
> read-only. The script for each section is named where the result appears. The 2026-11-11 rerun
> reproduced all of them (126 of 126 checks) and corrected one statement in §5.2.

The perception node, `real_time_distance`, turns every depth frame into one `MultiLinkDistance`
message. That message carries one entry per **control point** (and per obstacle rank), giving the
gap to the nearest observed obstacle surface, its direction, and optionally a tracked obstacle
velocity. The safety filter (chapter 07) builds one barrier row from each valid entry.

This chapter follows a pixel through the whole chain: §1–§5 cover geometry and distance, §6 the
message contract, §7 tracking and prediction, §8 self-detection and the self-shadow barrier, and
§9 calibration.

---

## 1. Intuition

The safety filter needs to know how far each robot point is from an obstacle, and in which
direction. One fixed RGB-D camera (an Intel RealSense D455 in the scene, per the comments in
`fr3_complete.yaml` and `launch_defaults.yaml`) looks at the robot and its surroundings. The
difficulty is that **the robot is in its own picture**: a naive "nearest depth point to the arm"
would always be the arm itself, at distance zero.

The pipeline therefore:
1. predicts where the robot appears in the image, by projecting sampled link meshes posed with TF;
2. masks those pixels out, but not the ones clearly *in front of* the robot (the depth gate);
3. back-projects the remaining pixels to 3-D;
4. measures, for each control point, the gap to the nearest remaining point;
5. filters the result conservatively in time: it reacts to approach at once and relaxes slowly on
   retreat.

Everything here is a design trade-off between two failures:
- **missing a real obstacle** (an over-large mask, a hand hidden in the halo);
- **seeing the robot as an obstacle** (self-leak, which makes the arm brake from itself).

The code comments record several measured instances of each.

## 2. Frames and data

| Symbol | Meaning | Source (Verified) |
|---|---|---|
| D(u, v) | depth image, uint16 **millimetres** | topic `fr3_complete.yaml topics.depth_image` = `/camera/camera/depth/image_rect_raw` (the **raw depth stream**, not the aligned one); `distance_engine._DEPTH_TO_M = 0.001` |
| K | 3×3 intrinsics fx, fy, cx, cy | `camera_info_callback`, from `/camera/camera/depth/camera_info`, taken **once** (the first message with fx ≠ 0) |
| {C} | camera optical frame: x right, y down, z forward | the pixel geometry |
| (R_b, t_b) | camera → base: p_b = R_b p_C + t_b | `utils/config.py::load_extrinsics(camera_extrinsics_path)`, read **from the YAML file at start-up, not from TF** |
| {B} | `fr3_complete.yaml robot.base_frame` = `fr3_link0` | equals the Pinocchio world ([04](04_math_foundations.md) §2.1) |
| ⁰T_ℓ | pose of link ℓ in {B} | TF, `utils/tf_manager.py::TFManager.lookup_all`, at the depth frame's stamp |

**Shipped values (Verified).**
- `camera_extrinsics.yaml` places the camera at t_b = (1.754, 0.154, 1.059) m, 2.06 m from the robot
  base. The optical axis points along (−0.954, −0.053, −0.295), about 17° below horizontal, back
  toward the robot.
- The depth intrinsics dump `depth_intrinsics.yaml` (848×480, `camera_depth_optical_frame`) gives
  fx = fy = 428.84 px, cx = 426.06, cy = 235.17.
- The node does **not** read that file. It takes K from `camera_info`. The dump is used by the
  simulation's emulated camera (`franka_simulation_experiments/test/test_sim_config.py`) and by
  `capsule_overlay_node` (`rgb_intrinsics.yaml`).

## 3. Mathematical foundations

### 3.1 Pinhole back-projection

A pixel (u, v) with depth Z [m] corresponds to the camera-frame point

  p_C = Z · K⁻¹ [u, v, 1]ᵀ = ( (u − c_x) Z / f_x, (v − c_y) Z / f_y, Z ).  (5.1)

`distance_engine.compute` (Step 4) evaluates (5.1) in float32. Distortion is ignored: the
`image_rect_raw` stream is rectified, and the dumped D is all zeros. A point is moved to base
coordinates with

  p_B = R_b p_C + t_b,  and back with  p_C = R_bᵀ (p_B − t_b).  (5.2)

**The engine works in the camera frame.** It moves the 11 control points into {C} once per frame,
cp_C = R_bᵀ(cp_B − t_b) (Step 5), instead of moving every pixel into {B}. Distances are invariant
under the rigid map (5.2), so the result is the same (Derived). Only the winning point is mapped
back to {B} for the message.

### 3.2 The distance that is published

Let O be the set of obstacle points that survive masking and the depth range [0.15, 4.0] m, and let
c_i be control point i with radius r_i (0.05 m, `fr3_complete.yaml robot.segments[].radius`). The
engine computes

  d_i = max( min_{o ∈ O} [ ‖o − c_i‖ − r_i − m(o) ], 0 ),  (5.3)

  n̂_i = (c_i − o*) / ‖c_i − o*‖,  o* = argmin of (5.3)  (direction **from the obstacle to the
  robot**).  (5.4)

Here m(o) is the mask-margin correction of §3.4.

**What geometry this is (Derived from the code).** Equation (5.3) is the gap from o to a **sphere**
of radius r_i centred on the control point. It is **not** the gap to a capsule around the link
segment. `distance_utils.define_robot_segments` and `compute_closest_distance_from_segments` do
implement capsules, but `DistanceEngine.compute` uses neither. The YAML comment ("capsule radius
… subtracted") describes the radius correctly, but the shape is a sphere.

The robot is therefore represented, *for distance purposes*, by **11 spheres of radius 5 cm**
([04](04_math_foundations.md) §4.1), two of which coincide (F-2-1). Consequences (Derived,
numerically evaluated from the FR3 geometry of 04 §3.1):
- **Links 0–3 carry no control points**, so an obstacle near the base or shoulder produces no row.
  This is a configuration choice (`control_points: 0`).
- **Spheres do not overlap along the long forearm segment** (link4 → link5, 0.393 m, centres at
  1/3 and 2/3). The spacing is 0.131 m against a sphere diameter of 0.10 m. A point lying on the
  5 cm radius tube of that segment, halfway between two centres, is at
  √(0.0655² + 0.05²) − 0.05 = **0.032 m** from the nearest sphere. The published gap is therefore up
  to about 3 cm **larger** than the gap to a 5 cm capsule, before the margin of §3.4 is subtracted.
- **The hand is wider than its sphere.** The hand collision mesh extends ±0.10 m across the gripper
  (bounding box of `hand.stl`, numerically measured). The fingertip sphere has r = 0.05 m, centred
  0.10 m beyond the flange. An obstacle beside the outer edge of the hand can therefore read several
  centimetres farther than it is from the hand surface (F-2-4).

  **Numerically measured** (added in the reproducibility pass, `check_geometry_and_so3.py`): at
  q_home, with the hand mesh placed as the URDF places it, 94 % of the hand's vertices lie outside
  every control-point sphere. The farthest is **6.84 cm** from the sphere model. This value holds for
  the home pose only; the risk it implies is still Inferred.

  The forearm gap above is recomputed by the same script (3.24 cm). The bounding box is in
  `check_hand_mesh.py`.

Whether these gaps matter depends on the margins the barrier adds (`d_safe` = 0.10 m and the
uncertainty terms of chapter 07), and on the conservative bias of §3.4, which works in the opposite
direction.

### 3.3 The robot model and the depth gate

`MaskBuilder.rebuild` builds the masks in five steps:

1. **Sample the meshes.** It samples each link's collision mesh: `meshes.sample_points_per_link` =
   3000 surface points, drawn once at start-up by `trimesh.sample`.
2. **Pose and project them.** Each sample is posed with TF, ⁰T_ℓ s, moved into the camera frame
   with (5.2), and projected with K. The projection gives:
   - a binary silhouette per class, end effector (`ee_link` = `fr3_link8`) or body;
   - a z-buffer z_rob(u, v) = minimum sample depth per pixel (`np.minimum.at`).
3. **Downsample and dilate.** Everything runs at 1/4 resolution (`mask_downsample` = 4). The radii
   are scaled down accordingly: body `robot_mask_dilate_px` = 2 → 1 ds-px; EE `ee_mask_dilate_px` =
   18 → 4 ds-px; plus `search_exclusion_extra_px` = 8 → 2 ds-px.

   **Effective full-resolution margins (Derived):** body = (1 + 2)·4 = **12 px**,
   EE = (4 + 2)·4 = **24 px** (`dilation_margins_px`).
4. **Fill the z-buffer holes** with a min-filter (erosion) of the same radius, then upsample with
   nearest-neighbour interpolation.
5. **Gate on depth** (`DistanceEngine.compute` Step 2, `depth_gate_tol_m` = 0.10 > 0). A pixel
   inside the exclusion mask is still kept as an obstacle if it lies clearly in front of the
   robot's modelled surface:

     keep ⇔ ¬mask(u, v) ∨ [ 0 < Z(u, v) < (z_rob(u, v) − b) − τ ],  (5.5)

   with bias b = `depth_gate_bias_m` = 0.06 m and tolerance τ = 0.10 m.

The comments explain the split. b is the **median** of the model-minus-measurement residual (mesh
sampling makes the model surface read too far). τ is its **spread**. Only b + τ changes behaviour.
Pixels at or behind the robot surface stay excluded.

### 3.4 The mask-margin correction m(o)

Dilating the mask pushes the first visible obstacle pixel away from the true robot edge. To
compensate, the engine converts the pixel margin to metres **at the obstacle pixel's own depth** and
subtracts it (Step 4b):

  m(o) = M_px(o) · Z(o) / f_x,  M_px = 24 if the pixel's provenance is the EE dilation, else 12.  (5.6)

**Finding (Derived, F-2-8; arithmetic in `check_perception_numbers.py`).** `ee_source_mask` is non-zero only inside the exclusion mask. So for
every obstacle pixel outside the mask, which is the normal case, M_px = 12, and the subtraction
applies to **every** obstacle pixel wherever it is, not only to pixels touching the mask edge.

The published distance is therefore biased low by 12 Z / f_x:
- 2.8 cm at Z = 1.0 m;
- 4.2 cm at 1.5 m;
- 5.6 cm at 2.0 m (f_x = 428.8).

This bias is conservative, so it is safe in direction. Its effect is that the barrier engages at
about d_safe + 4–6 cm of real clearance across the working volume. The comment's stated purpose
(an obstacle touching the dilated border reads ≈ 0) holds. The uniform application is not stated
anywhere.

Whether it is intended cannot be decided from the code. It is listed for the user and for the
over-reactivity discussion in chapter 07.

### 3.5 Temporal filter (`DistanceEngine._lpf_pass`)

The filter is kept per control point, keyed by (seg_idx, cp_idx), with the real inter-frame
interval Δt from the capture stamps. It has three branches and a hold.

**EMA weight.** α = `utils/rate_scaling.py::ema_alpha_for(τ_f, Δt)` = exp(−Δt/τ_f) (Verified),
with τ_f = `lpf_tau_s` = 0.048 s. That gives α = 0.793 at 90 Hz and 0.499 at 30 Hz, so the legacy
30 Hz value 0.5 is reproduced. With `lpf_tau_s` = 0 the legacy constant `lpf_alpha` (0.5) applies.

| Case | Output d⁺ |
|---|---|
| retreat, d_raw ≥ d_prev | α d_prev + (1 − α) d_raw (smoothed, so the published value lags low: conservative) |
| approach, d_raw < d_prev, with implied speed (d_prev − d_raw)/Δt ≤ v_max = 2.0 m/s | d_raw (immediate) |
| approach faster than v_max | max(d_prev − v_max Δt, d_raw), and the pair (d_prev, d_raw) is held *pending* |
| next frame after pending | if d_raw ≥ d_pre − max(0.10 d_pre, 0.03): **outlier**, rewind to d_pre and continue normally; otherwise **confirmed**, d⁺ = d_raw |
| no measurement (d_raw = ∞) | hold d_prev with the previous direction. After `iso_distance_hold_max_s` = **0.10 s** (`fr3_control.yaml params`), the direction is set to `None`, which makes the entry `valid = False` (§6) |

The direction n̂ is EMA-smoothed and renormalised. The obstacle point o* is **not** smoothed. So a
published entry can combine a smoothed distance and direction with a raw point. Consumers that
recompute geometry from `closest_point_human` (chapter 07) see the raw point.

**Finding: the held entry carries no obstacle point (F-2-11, Verified and numerically
reproduced).**
1. During the bounded hold, `_lpf_pass` emits `closest_obstacle_point=None` with the previous
   direction, so the entry is still `valid = True`.
2. `build_cp_messages` writes `closest_point_human` only when the point is not `None`, so the field
   keeps its wire default **(0, 0, 0)**: the origin of `fr3_link0`.
3. `cbf_safety_filter._on_distances` reads it into `Obstacle.ph`, with no check.
4. `cbf_state_rows.py` (around L2509) builds the row normal as n̂ = (pr − ph)/‖pr − ph‖, not from the
   published `direction`.

Reproduced with the baseline `build_cp_messages` (`tools/stage2_verification/check_held_entry.py`): for the home-pose
fingertip with a held direction (−1, 0, 0), the normal the filter would compute is
(0.531, 0, 0.848), **122° away** from the published direction. That normal points radially away
from the robot base.

**Effect on the barrier (Inferred, to be confirmed in Stage 3):** for up to 0.10 s after the
obstacle leaves the image, or is masked out, every held row pushes along the wrong normal with the
held distance. Whether a later stage of the filter rejects ph = 0 has not been checked yet; it is
the first item for chapter 07.

**Derived consequence.** A real approach faster than 2 m/s (a thrown ball) is delayed by exactly one
frame: 11 ms at 90 Hz.

## 4. Implementation: the per-frame path

`nodes/real_time_distance.py`. The executor thread handles callbacks; a daemon thread named
`rtd_compute` does the work.

1. **`depth_callback`.**
   - Drops a frame whose capture stamp equals the previous one. The comment says the driver
     republishes about 2/3 of the frames, measured on 2026-09-21.
   - Converts the image with `cv_bridge`.
   - Pushes it into a `Queue(maxsize=frame_queue_depth=2)`, dropping the oldest frame when full.
2. **`_compute_loop`** blocks on the queue and calls `_process_depth_impl` once `fx` is known.
3. **`_process_depth_impl`:**
   - (optional) renders the injected test sphere (`sim_obstacle.enabled`, false);
   - on a change of image shape, resets every cache, the LPF and the tracks;
   - `tf_mgr.lookup_all(segment_links, stamp)`, with three-level fallback (§4.1);
   - `define_control_points(transforms, …)`, the perception copy of the control-point geometry
     ([04](04_math_foundations.md) §4.4);
   - `mask_builder.rebuild` (§3.3), then `compute_roi`: the bounding box of the exclusion mask plus
     `roi_pad_px` = 180, clamped to `image_margin_px` = 10;
   - `distance_engine.compute(…)` (§3.1–3.5) on the grid `pixel_step` = 6 inside the ROI;
   - `_check_calibration` (§9.3), `_retune_for_rate`, and, if tracking is on, `track_pipeline.update`
     and `add_predicted_hits` (§7);
   - the publish decision (§6.3), then `_publish_rows`: `build_cp_messages` followed by
     `annotate_track_fields`.

**Sampling density (Derived).** At Z = 1.3 m one pixel covers 3.1 mm, so the stride of 6 samples
the scene every 1.8 cm. A 7 cm ball at 2 m yields about 2–3 samples per axis. The YAML comment cites
this reason for lowering the stride from 10 to 6.

### 4.1 TF lookup (`utils/tf_manager.py`)

`_lookup_one` tries three levels in order:
1. the exact depth stamp;
2. the latest available transform, silently;
3. a cached transform, if younger than `tf_cache_max_age_s` = 0.5 s.

`lookup_all` rejects the frame if a critical link is missing (default: the EE link) or if fewer than
half of the links resolve. `last_pose_age_s` records the worst age of a pose relative to the frame.
The comment notes that a level-2 fallback at 15 Hz TF lags 67 ms, which is 33 mm of end-effector
motion at 0.5 m/s, and that this shows up downstream as a self-leak. The control points that are
published come from these TF poses, **not** from Pinocchio and the joint states. They can therefore
lag the filter's kinematics by one TF period ([02](02_ros_graph_and_dataflow.md) §6).

### 4.2 Finding: the hand mesh is posed in the wrong frame

`meshes.files.fr3_link8` is `robot_ee/franka_hand_white/collision/hand.stl`. `MaskBuilder.rebuild`
poses each mesh with **the TF of the key it is listed under**, so the hand samples are posed with
⁰T_{fr3_link8}.

In the URDF, however, `hand.stl` is the collision geometry of link `fr3_hand`. The fixed joint
`fr3_hand_joint` places `fr3_hand` at `fr3_link8` with **rpy = (0, 0, −π/4)**. This was verified on
the xacro output, and numerically with the Pinocchio model of 04 §2.1. The missing Rz(−45°) rotates
the modelled hand by 45° about the flange axis.

**Numerically measured on the mesh** (`check_hand_mesh.py`): vertex displacement up to **7.96 cm** (mean 6.9 cm; mean
4.4 cm over the 3000 sampled points). The long axis of the gripper is modelled along y₈ while the
real one lies along (0.707, 0.707, 0)₈.

**Effects (Inferred, not observed):**
- the hand's silhouette and z-buffer are rotated, so parts of the real hand can fall outside the
  mask (self-leak at the end effector) while empty space beside it is masked;
- the same samples feed `calibration_check` (§9.3) and the self-shadow model points (§8.2), so
  their residuals at the hand carry this error too.

The 24 px end-effector dilation (about 8 cm at 1.5 m) probably hides much of it in the silhouette.
**Runtime test to settle it:** compare the overlay or the leak counter (`leak=` in the status line)
with the hand rotated about joint 7, with and without the −45° correction. Logged as F-2-9.

## 5. Calibration questions from Stage 1

### 5.1 Q-1-2: depth frame vs colour frame

**Verified.**
- The node projects the **raw depth** image (`/camera/camera/depth/image_rect_raw`, frame
  `camera_depth_optical_frame`).
- It uses `camera_extrinsics.yaml`, whose header says `child_frame: camera_color_optical_frame`.
- The hand-eye node estimates base → **colour** optical frame from AprilTag detections in the colour
  image (`handeye_calibration_node.py` docstring; §9).
- On the D455 the depth and colour optical frames are 5.9 cm apart (the YAML comment quotes the
  driver's `extrinsics/depth_to_color` t = (−0.0591, 0, 0.0002)).

**What the code base says about it** (`fr3_complete.yaml`, block above `topics:`). Composing the
5.9 cm offset was tried on 2026-09-30 on a recorded bag (`ball_throws_2`) and made both independent
checks worse:
- the coverage of the measured arm by the mask fell from 91 % to 84 %;
- the distance rows moved away from the colour-tracked ball.

The decision recorded there is: "the calibration, as used, behaves as base → DEPTH optical, and no
offset is applied".

**Status: resolved as a known, deliberate choice; the root cause is still open.** The label in the
YAML header disagrees with how the transform is used, and the only evidence for the usage is that
2026-09-30 replay (Runtime (recorded) by the author, not re-run here). Two explanations fit and have
not been told apart:
1. the hand-eye result really is closer to the depth frame (for example, the calibration absorbed
   the offset);
2. the offset trial composed the transform in the wrong direction or with the wrong sign.

The `align_depth.enable: true` launch default (`launch_defaults.yaml camera_align_depth`) does not
affect this node, because it subscribes to the unaligned stream.

**Runtime test that would settle it:** with `calibration_check` (§9.3) on a static arm, compare the
residual median and spread for the transform as is, composed with depth→colour, and composed with
its inverse.

### 5.2 Q-1-3: the two calibration files

`camera_link_extrinsics.yaml` (base → `camera_link`, published as static TF by the torque launch)
says, in its own header, that it is **derived** from `camera_extrinsics.yaml` by composing the
inverse of a "D435i typical" `camera_link → camera_color_optical_frame` offset (t = (0, 0.015, 0),
q = (−0.5, 0.5, −0.5, 0.5)).

**Numerically checked, by recomposing as the header describes** (`check_camera_extrinsics.py`,
which scans every version in git):

| `camera_extrinsics.yaml` version | Disagreement with `camera_link_extrinsics.yaml` (HEAD) |
|---|---|
| HEAD (`fda4af7`, 2026-09-16) | **0.276 m, 5.14°** (0.2758 m, 5.138°) |
| `08710eb` (09-09) | 0.267 m, 11.1° |
| `72d2753` (09-01) | 0.200 m, 10.7° |
| `0c7e2b9` (06-24) | 0.107 m, 12.2° |
| `8bfd96b` (06-19 15:28) | 0.033 m, 10.1° |
| **`e5fcdb2` (05-21)**, still the current calibration when `d4e4028` (06-19 10:06) last wrote `camera_link_extrinsics.yaml` | **0.000 m, 0.000°** |
| older versions | 0.82–1.42 m |

**Answer (Verified + numerically checked; corrected 2026-11-11).** `camera_link_extrinsics.yaml`
was derived from the 2026-05-21 calibration (`e5fcdb2`). It was written in `d4e4028` and became stale
the same afternoon, when `8bfd96b` recalibrated. `camera_extrinsics.yaml` has changed **five** times
since (`8bfd96b`, `0c7e2b9`, `72d2753`, `08710eb`, `fda4af7`), and the derived TF file was never
regenerated. The camera pose in the **TF tree** is therefore 27.6 cm and 5.1° away from the pose
`real_time_distance` uses. Logged as F-2-7.

> *Superseded wording (Stage 2, before the rerun), kept for traceability:* "The two files were
> consistent on 2026-06-19. `camera_extrinsics.yaml` was recalibrated three times since then
> (`72d2753` 09-01, `08710eb` 09-09, `fda4af7` 09-16)". The table also labelled the matching row
> `d4e4028` and called `8bfd96b` "earlier". That row was the content of `camera_extrinsics.yaml` *at*
> `d4e4028`, i.e. the `e5fcdb2` calibration, and `8bfd96b` is five hours *later*. The 0.276 m / 5.14°
> conclusion is unaffected.

The header also uses a D435i offset for what other comments call a D455, a separate approximation.

**What this affects.** `real_time_distance` does not use TF for the camera, so the distances are
unaffected. Anything that places camera data through TF is affected: the RealSense point cloud in
RViz, overlays drawn in `camera_*` frames, and any node that would look up `camera_*` frames. The
torque launch comment (L676–682) states the intent that the two stay consistent.

### 5.3 How many calibration sources there are

| File | Content | Reader (Verified) |
|---|---|---|
| `camera_extrinsics.yaml` | base → colour optical (used as depth) | `real_time_distance`, `iso_*` via launch, `bag_replay`, the static TF in `minimal`, `thales` and the velocity stack ([02](02_ros_graph_and_dataflow.md) §5) |
| `camera_link_extrinsics.yaml` | base → `camera_link` | static TF in the torque launch only |
| `camera_EE_extrinsic.yaml` | the wrist camera relative to the end effector (from the eye-in-hand node) | §9; not read by Pipeline A |
| `ee_tag_extrinsics.yaml` | `fr3_link8 → tag36h11:0` | hand-eye calibration (§9) |
| `depth_intrinsics.yaml`, `rgb_intrinsics.yaml` | `CameraInfo` dumps | the simulation's emulated camera, `capsule_overlay_node`; **not** `real_time_distance` |
| `camera_intrinsics.yaml` | — | no reader (F-1-14) |

## 6. The `MultiLinkDistance` contract

### 6.1 Fields (`franka_msgs/msg/LinkDistance.msg`, Verified)

| Field | Meaning and units (frame `fr3_link0`, header stamp = **depth capture stamp**) |
|---|---|
| `robot_link_name` | end link of the control point's segment. It repeats across the control points of that link. |
| `closest_point_robot` | the control point c_i [m] (the sphere centre, not a surface point) |
| `closest_point_human` | the obstacle point o* [m] (raw, unsmoothed) |
| `direction` | n̂, a unit vector from the obstacle to the robot (smoothed) |
| `distance` | d_i from (5.3) after the LPF, ≥ 0 [m] |
| `valid` | `isfinite(d) ∧ d ≥ 0 ∧ direction ≠ None` |
| `confidence` | `find_pt_confidence(d, n_pts)` = clip(n_pts/500, 0.2, 1) · (1 if d < 2 else clip(1 − (d−2)/3, 0.3, 1)). Fixed at 0.5 when d < `min_thresh` (contact regime). |
| `zone` | `critical` ≤ 0.1 < `danger` ≤ 0.2 < `warning` ≤ 0.3 < `safe` (`fr3_complete.yaml zones`); `predicted` for ballistic rows (§7.4) |
| `track_id`, `frames_seen`, `obstacle_velocity`, `velocity_covariance`, `obstacle_acceleration`, `position_covariance`, `position_velocity_covariance` | tracker output (§7); all zeros means "no track" |
| `range_m` | raw camera-frame Z of o* (0 = not carried); input to the sensor-noise model in chapter 07 |
| `shadow_*` | self-shadow barrier (§8.2); `shadow_valid = false` when SSB is off |

### 6.2 Order and labels — the implicit join key

The message carries **no control-point index**. `build_cp_messages` orders entries by
`segment_links` (YAML order), then by (seg_idx, cp_idx), with each control point's extra obstacle
ranks right after it. `perception_msgs.labelled_links` reconstructs labels positionally:
`"<link>#<k>"`, where k counts the control points of that link. An entry with the **same link and
`closest_point_robot`** as the previous one, but a **different `closest_point_human`**, is read as
rank r ≥ 1 of the same control point and labelled `"<link>#<k>.<r>"`.

`cbf_safety_filter._on_distances`, `iso_safety_monitor`, `iso_evidence_logger`, `experiment_logger`,
`annotate_track_fields` and the simulation's `estimator_metrics` all use this one function. The
kinematics side resolves `label.partition('.')[0]` to the configured geometry
([04](04_math_foundations.md) §4.3). The tests `test_cp_label_agreement.py` and
`test_multi_obstacle_rows.py` pin this contract.

**Finding (numerically reproduced, F-2-10).** The rank rule tells control points apart by the
**position** of `closest_point_robot`. The two duplicate wrist points `fr3_link6#0` and
`fr3_link6#1` (F-2-1) have exactly equal positions. This was verified with Pinocchio forward
kinematics; a TF composition with a zero offset also yields exact equality.

With `multi_obstacle_k ≥ 2`, which is the **launch default** (`launch_defaults.yaml
multi_obstacle_k: 2`, [03](03_configuration.md)), and a second obstacle cluster within the horizon,
`labelled_links` produces:

```
… 'fr3_link6#0', 'fr3_link6#0.1', 'fr3_link6#0.2', 'fr3_link6#0.3', 'fr3_link7#0', …
```

instead of `'fr3_link6#1', 'fr3_link6#1.1'`. This was reproduced (`check_cp_labels.py`) by running `build_cp_messages`
and `labelled_links` from the baseline source on the 11 home-pose control points, each with one
extra cluster.

**Effects.**
- The geometry is unaffected, because `#0` and `#1` are the same point.
- The label `fr3_link6#1` disappears: its rows carry ranks 2 and 3 of `#0`.
- Every per-control-point state keyed on the label (in the filter, the ISO nodes and the logger) is
  merged or shifted for that control point.
- Any logic that treats rank ≥ 1 differently (for example, SSB fields written only on rank 0,
  `_cap_extra_rows` budgets, or extra-row handling in the filter) sees what is in fact a primary
  row as an extra.

With k = 1 the rule is not triggered. How the filter treats ranks is examined in Stage 3.

**A second key convention exists (Derived).** `real_time_distance._self_detected` builds its own
`link#k` keys. It counts, in arrival order, only the control points that have an obstacle point.
In normal operation this agrees with `labelled_links`, for two reasons:
- every control point searches the **same** pixel set (Step 6 has no per-control-point cutoff), so
  all of them have a point or none does;
- extras are not counted.

Under F-2-10, however, the self-detection key `fr3_link6#1` does not exist among the published
labels (it reads `fr3_link6#0.2`). A self-detection verdict on that control point would therefore
never suppress its track fields in `annotate_track_fields`. The two conventions would also diverge
if a per-control-point search radius were ever introduced (§12).

### 6.3 When the message is published (`_process_depth_impl`)

- `in_band` = entries with `min_thresh` (0.08) ≤ d ≤ `max_thresh` (0.7);
  `publishable` = entries with d ≤ 0.7 (`publish_contact_regime: true`).
- If nothing is publishable and there is no predicted or shadow row, the node publishes the legacy
  fallback and an **empty `MultiLinkDistance` heartbeat** (`publish_empty_per_link: true`). An empty
  message means "nothing near"; a *missing* message means "perception dead" (message header
  comment).
- In the contact regime (only entries < 0.08) the rows are published with confidence 0.5 and a
  `CONTACT REGIME` warning.
- Otherwise every control point's entries are published, including invalid ones. Their positions
  carry the labels of §6.2.

**Consequence.** Obstacles beyond 0.7 m from every sphere produce **no rows at all**. The filter's
own computational horizon (`cbf_obstacle_horizon` 1.2 m) never sees anything between 0.7 and 1.2 m
in Pipeline A (Derived).

## 7. Obstacle tracking and prediction

### 7.0 Status: what runs

| Feature | Code default | `fr3_complete.yaml` | Launch | Net state in Pipeline A (torque launch) |
|---|---|---|---|---|
| Tracking (clusters + KF) | off | `tracking.enabled: false` | `launch_defaults.yaml obstacle_tracking: true`; `torque_control_stack._rtd_config_with_overrides` writes a temporary copy of the YAML with `enabled: true` | **ON** (Verified) |
| IMM (generic + ballistic) | `imm_enabled=False` | `imm_enabled: true` ("TEMPORARY for hardware validation (Sep 2026)") | — | **ON** whenever tracking is on |
| Self-detection guard | `enabled: True` | `self_detection.enabled: true` | — | **ON** whenever tracking is on |
| Ballistic impact prediction | off | `prediction.enabled: false` | — | **OFF** (implemented, tested, experimental) |
| Calibration drift check | `enabled: True` | `calibration_check.enabled: true`, `baseline_m` empty | — | runs every 10 s, **reports only** |
| Self-shadow barrier (SSB) | off | `distance.ssb_enabled: false` | — | **OFF** (experimental) |
| `static_scene.py`, `surface_velocity.py` | — | — | — | **unused**: they import only each other, and nothing in either package imports them (Verified by grep). The account memory "surface-point velocity needs a fitted slope" refers to this dormant code. |
| `obstacle_sim.InjectedSphere` | off | `sim_obstacle.enabled: false` | `sim_obstacle:=true` | OFF (test fixture) |

`bag_replay.launch.py` uses the same override function. The Gazebo harness reuses the torque launch
([01](01_architecture.md)), so the simulation runs with tracking on unless a condition overrides
`obstacle_tracking` (Stage 6).

### 7.1 Clustering (`utils/obstacle_clusters.py`)

The input is the **same pixel set** the distance pass used: `ObstacleCloud` (p_cam, u, v, step,
labels), exported only when tracking is on.

1. **Depth-aware connected components** (`label_points`).
   - Pixels are re-indexed on the stride grid ((u − u_min)/step), so neighbouring samples are
     adjacent.
   - Two 8-connected neighbours are joined iff |Z₁ − Z₂| < `cluster_depth_jump_m` = 0.10 m.
   - The graph goes to `scipy.sparse.csgraph.connected_components`.
   - With `multi_obstacle_k > 1` the engine has already labelled the pixels with the same
     `depth_jump`, and those labels are reused. The row's `cluster_id` and `Cluster.label` therefore
     agree.
2. **Per component** (`cluster_obstacles`):
   - drop it if it has fewer than `cluster_min_points` = 5 raw pixels (speckle);
   - voxel-downsample at 2 cm;
   - centroid = mean of the voxel centroids; radius = max distance from the centroid, at least
     1 cm;
   - drop it if the radius exceeds `cluster_max_radius_m` = 1.2 m. This is the "scene guard": walls
     and the floor would otherwise fuse into a frustum-wide blob whose centroid hops by tens of
     centimetres.
   - Keep the 16 largest, sorted deterministically.

### 7.2 State model (`utils/obstacle_tracker.py::KalmanTrack`)

The state is x = [p; v; a] ∈ ℝ⁹ in the **base frame**: centroids are mapped with (5.2) *before*
filtering, so the velocity is a base-frame quantity. The model is constant acceleration with
white-noise jerk of PSD q:

  F(Δt) = [[1, Δt, Δt²/2], [0, 1, Δt], [0, 0, 1]] ⊗ I₃,  (5.7)

  Q(Δt) = q · [[Δt⁵/20, Δt⁴/8, Δt³/6], [Δt⁴/8, Δt³/3, Δt²/2], [Δt³/6, Δt²/2, Δt]] ⊗ I₃.  (5.8)

The measurement is z = H x + w, with H = [I₃ 0 0] and R = σ_m² I₃.

**Numerically verified** (`check_tracker.py`): (5.8) equals the exact integral ∫₀^Δt F(τ) G Gᵀ F(τ)ᵀ dτ · q with
G = [0 0 I]ᵀ. The maximum error is 3.5e-18 against entries of 2e-2, so it is not the
piecewise-constant approximation.

| Step | Equation | Code |
|---|---|---|
| Predict | x⁻ = F x, P⁻ = F P Fᵀ + Q; **skipped** unless 1e-6 < Δt < 1 s | `KalmanTrack.predict` |
| Gain | S = P_pp + R, K = P Hᵀ S⁻¹ | `update`, `innovation_cov` |
| Update (Joseph form) | x⁺ = x⁻ + K(z − p⁻), P⁺ = (I−KH) P⁻ (I−KH)ᵀ + K R Kᵀ, then symmetrised | `update` |
| Gate metric | d_M = √(yᵀ S⁻¹ y), y = z − p⁻ | `mahalanobis` |

**Shipped values:** q = `q_jerk` = 2.0 m²/s⁵, σ_m = `sigma_meas_m` = 0.01 m, σ_v0 = `sigma_v0` =
**2.0** m/s (YAML; the code default is 1.0), σ_a0 = 5.0 m/s².

Δt is the **capture-stamp** difference (`ObstacleTrackPipeline.update`). It is accepted when it lies
in (1e-4, 1) s; otherwise `default_dt` = 1/`depth_rate_hz` is used.

### 7.3 IMM (`IMMTrack`, ON by the shipped YAML)

Two models run on (5.7)–(5.8) with the same q:
- **generic**, with a₀ = 0 and σ_a0 = 5;
- **ballistic**, with a₀ = g = (0, 0, −9.81) m/s² and σ_a0 = 2.

The Markov transition matrix is Π = [[0.99, 0.01], [0.01, 0.99]], with μ₀ = (0.5, 0.5). Standard IMM
(Verified against `predict` and `update`):

  c_j = Σ_i Π_ij μ_i,  μ_{i|j} = Π_ij μ_i / c_j,  x⁰_j = Σ_i μ_{i|j} x_i,
  P⁰_j = Σ_i μ_{i|j} [P_i + (x_i − x⁰_j)(x_i − x⁰_j)ᵀ];  (5.9)

  per-model Joseph update; Λ_j = N(y_j; 0, S_j), computed in the log domain; μ_j ∝ c_j Λ_j;  (5.10)

  x = Σ μ_j x_j,  P = Σ μ_j [P_j + (x_j − x)(x_j − x)ᵀ].  (5.11)

The blend (5.11) is what consumers read: velocity, P_vv, P_pp and P_pv. The spread-of-means term can
only widen the covariance. `mode_prob` is diagnostic only: nothing downstream reads it.
`ballistic_prediction` reads it, but that module is off.

**Numerically measured** (`tools/stage2_verification/check_tracker.py`, 90 Hz, σ = 1 cm white noise, shipped tuning):

| Scenario | KalmanTrack | IMMTrack |
|---|---|---|
| Static target: spurious ‖v‖ mean / p95 | 0.072 / 0.120 m/s | 0.072 / 0.122 m/s |
| Static target: √P_vv,xx | 0.064 m/s | 0.064 m/s (μ stays at 0.50 / 0.50) |
| New track, true 1 m/s: v̂ₓ at 56 / 100 / 200 ms | 1.22 / 0.81 / 0.95 | 1.05 / 1.13 / 0.86 |
| Throw (−3.5, 0, 2) m/s under gravity: μ_ballistic at 56 / 167 / 333 ms | — | 0.47 / 0.88 / 0.81, with â_z → −10.0 |

One seed and noise only, so this is illustrative, not a benchmark.

The static spurious speed of about 7 cm/s is the noise floor the filter's velocity consumers have to
gate (chapter 07: `obstacle_velocity_*` trust parameters).

### 7.4 Association and lifecycle (`TrackManager`)

1. **Predict** every track.
2. **Candidate pairs** (track, cluster) need **both** ‖z − p̂‖ ≤ `gate_max_m` = 0.5 m and
   d_M ≤ `gate_mahalanobis` = **4.0** (YAML; the code default is 3.0).
3. **Greedy global nearest neighbour.** Pairs are sorted by d_M and taken best-first; each track
   and each cluster is used at most once. This is not Hungarian assignment.
4. **Births.** An unmatched cluster spawns a new track (an IMM track when `imm_enabled`). If
   `max_tracks` = 20 is reached, the stalest *tentative* track is evicted
   (`evict_stale_tentative: true`); otherwise the cluster is dropped and counted as `over_capacity`.
5. **Confirmation (M-of-N).** A track is confirmed once it has M hits within the last N frames. M
   and N are derived from durations by `real_time_distance._track_counts`:
   - N = `frames_for(confirm_window_s = 0.056)`; M = round(0.6 N), clamped to [2, N];
   - **90 Hz → 3-of-5; 30 Hz → 2-of-3** (computed);
   - the YAML history explains the shortening from 0.167 s on 2026-09-30, for thrown balls.

   Confirmation is sticky until the track dies.
6. **Death.** A track is reaped when it has missed more than `max_coast_s` = 0.8 s of frames:
   **72 frames at 90 Hz**, 24 at 30 Hz.

**What a control point receives.** Only **confirmed** tracks that were **associated in this frame**
are ever attached.
- `ObstacleTrackPipeline._entries` is rebuilt from this frame's clusters, so a coasting track is
  never published. The coast keeps an identity alive; it does not extrapolate an obstacle.
- **Rank-0 rows:** `annotate_track_fields` uses the row's `cluster_id`
  (`track_info_for_cluster`). It falls back to point-in-sphere lookup (`track_for_point`: the
  nearest centroid among clusters with ‖p − centroid‖ ≤ radius + 0.05 m).
- **Predicted rows** (cid = −2) and self-detected control points are skipped.
- An unmatched control point keeps the all-zero "no track" fields.

**Numerical check of the coast–gate interaction** (`kf.py` §6). A converged static track that
coasts has σ_p = 1.3 cm after 0.1 s, 8.0 cm after 0.4 s and **29.8 cm after 0.8 s**. The
4σ Mahalanobis radius therefore reaches 1.19 m before the track dies, so **after about 0.5 s (0.52 s, `check_tracker.py`) of
coast the 0.5 m Euclidean `gate_max_m` is the only effective gate**.

This quantifies the caveat the YAML already records next to `max_coast_s` ("gate_max_m should be
re-derived before this ships to hardware"). It is logged as an open question (Q-2-2), not as a bug.

### 7.5 Ballistic impact prediction (`utils/ballistic_prediction.py`, OFF)

Prediction applies to every confirmed track with ‖v‖ − k_σ σ ≥ v_min (k_σ = 2, v_min = 1 m/s,
at least 3 frames seen). It propagates

  p(τ) = p + v τ + ½ g τ²  (g applied when μ_ballistic ≥ 0.5, or when `assume_ballistic`)

over τ ∈ [0, 0.5] s in 10 ms steps. For each control point it then computes

  g(τ) = ‖p(τ) − c‖ − r − r_obj,  r_obj = 0.035 m.  (5.12)

If min g = g* < 0.30 m and τ* ≥ 0.03 s, it appends an **extra rank row** at p(τ*) with distance g*,
zone `predicted` and cluster id −2, so no velocity is attached.

`replace_current` (false) would replace the control point's current-position row of the same track
instead. The module docstring records the motivation: the 2026-09-30 throws, where the filter acted
only 100–120 ms before closest approach. The ranking of extras (`.r`) makes these rows subject to
F-2-10.

## 8. Self-detection and the self-shadow barrier

### 8.1 Self-detection guard (`utils/self_detection.py`, ON with tracking)

**The problem.** With a tracker, a mis-masked arm becomes a cluster with a real velocity. That
velocity tightens the barrier, which slows the arm, which changes the velocity: a feedback loop.

**The test is kinematic, not geometric.** For each control point key, over a window of W frames, the
guard computes:

  travel = ‖max_k p_r,k − min_k p_r,k‖ (per-axis range),
  spread = ‖max_k (p_h,k − p_r,k) − min_k (p_h,k − p_r,k)‖.  (5.13)

The frame counts as rigid iff travel ≥ 0.08 m ∧ spread ≤ 0.02 m.
- **Verdict:** C consecutive rigid frames set it; Rl consecutive non-rigid frames clear it.
- **Counts:** W, C and Rl come from durations 0.667 / 0.167 / 0.5 s, giving **60 / 15 / 45 frames
  at 90 Hz** (20 / 5 / 15 at 30 Hz).
- **Calibration-independent by construction:** a wrong extrinsic moves p_h and p_r together, so
  their offset stays constant.

**The action is deliberately narrow.** Only the **track fields** of the flagged label are suppressed
(`skip_keys`), never the distance. A wrong verdict therefore falls back to v_obs = 0, never to a
missing barrier.

See §6.2 for the key-convention caveat.

### 8.2 Self-shadow barrier (`utils/self_shadow.py`, OFF)

The SSB looks at what the depth gate (5.5) **deleted**. Each masked ray is classified by comparing
its measured Z with ẑ = z_rob − b (`classify_masked_rays`), with τ = 0.10 and guard γ =
`ssb_guard_m` = 0.04:

| Class | Condition | Meaning |
|---|---|---|
| KEPT | Z < ẑ − τ | already an obstacle pixel |
| CANDIDATE | ẑ − τ ≤ Z < ẑ − γ | a real return in front of the modelled arm that the tolerance swallowed: **the self-shadow** |
| SELF | \|Z − ẑ\| ≤ γ | explained by the arm |
| THROUGH | Z > ẑ + γ | the ray saw past the arm: observed free |
| VOID | no valid return | unknown |

The candidates are filtered twice more:
- `reject_model_explained` drops any candidate within `ssb_model_clearance_m` = 0.05 m of a mesh
  sample (every 8th sample). These samples inherit F-2-9 at the hand.
- `cluster_support` keeps only connected support of at least 3 pixels.

The nearest candidate per control point then goes into the `shadow_*` fields. The filter turns it
into rows only with `cbf.ssb_rows_enabled` (chapter 07). The module also contains
`void_spheres_from_rays`, which is called only by `test_self_shadow_geometry.py`: it is unused in
production (Verified by grep).

The account-memory lead ("N2 self-mask ≠ occlusion", Nov 2026) and the commit message of `ba68077`
("Add the self-shadow barrier, off by default, with what it can and cannot do") agree with this
status.

## 9. Calibration

### 9.1 Fixed camera (`nodes/handeye_calibration_node.py`, the source of `camera_extrinsics.yaml`)

**Set-up:** a fixed camera; AprilTag `tag36h11:0` on the flange. The unknowns are
X = ᴮT_C (base → `camera_color_optical_frame`) and Y = ᴱT_T (`fr3_link8` → tag). Each sample gives
A_i = ᴮT_E(i) (TF FK) and B_i = ᶜT_T(i) (TF from the AprilTag detector). The loop closure is

  X B_i = A_i Y,  E_i = (X B_i)⁻¹ (A_i Y) ≈ I.  (5.14)

- **Residual per sample:** [w_R · log(R_{E_i}) (Rodrigues); w_t · t_{E_i}] ∈ ℝ⁶, with
  w_R = `rotation_weight` = 1.0.
- **Solver:** `scipy.optimize.least_squares` over 12 parameters (rotation vector + translation for
  X and for Y), `method='trf'`, `loss='huber'` (Verified at L2152).
- **Pipeline (docstring):** MAD outlier filtering on the loop-closure error, held-out validation,
  output to `camera_extrinsics.yaml` (X) and `ee_tag_extrinsics.yaml` (Y).

**Two observations (Derived):**
1. The docstring says "Levenberg–Marquardt", but the code uses the trust-region-reflective method.
   The optimum is the same; only the documentation is inaccurate.
2. With the default Huber scale f_scale = 1, residuals of a few mrad or mm are all far inside the
   quadratic zone, so the robust loss is inactive and the fit is ordinary least squares. Outlier
   robustness comes from the MAD rejection step alone.

The rotation (rad) and translation (m) residuals are mixed with unit weights, so 1 mm counts as
much as 1 mrad.

**Frame (relevant to Q-1-2).** The tag pose is looked up in `camera_frame` =
`camera_color_optical_frame`, so X is base → **colour** optical by construction. §5.1 records how
the perception node nevertheless uses it as base → depth optical.

### 9.2 Eye-in-hand (`nodes/handeye_eye_in_hand_node.py` + `utils/handeye_solver.py`)

Here the camera is on the flange and the tag is on the desk:

  ᴮT_E(i) · X · ᶜT_T(i) = Y,  X = ᴱT_C,  Y = ᴮT_T.  (5.15)

`handeye_solver.solve` is pure numpy/scipy and tested (`test_handeye_solver.py`):
1. initial guess: Kabsch on the rotation axes of relative motions, then linear least squares on the
   translations;
2. 12-DOF refinement;
3. MAD inlier mask, refit on a train split, held-out residuals.

The pose generators (`bootstrap_ee_poses`, `orbit_camera_poses`, `select_diverse_poses`) keep the
tag in view. The output (`camera_EE_extrinsic.yaml`) is **not consumed** by Pipeline A, which uses a
fixed camera. This node was read at interface level only.

Both calibration launches run the robot on the velocity controller (`command_topic=tracking_qdot`),
with no CBF ([02](02_ros_graph_and_dataflow.md)).

### 9.3 Calibration drift check (`utils/calibration_check.py`)

Every `period_s` = 10 s, the model front surface z_model(u, v) (the minimum over the projected mesh
samples, every 4th) is compared with the measured depth over the covered pixels. The node reports
the median and MAD of Z − z_model, and the leak fraction (residual < −gate).

The module's docstring records that the **absolute** value depends on the mesh sampling density
(−0.9 / +1.4 / +6.0 cm at 300 / 3000 / 20000 samples per link). It also records that the three
historical extrinsics scored within 1 cm of each other. The check is therefore a **drift** detector,
active only once `baseline_m` is set; it is empty in the shipped config, so it reports only.

**Consequence for Q-1-2 (Derived).** The median residual is probably too insensitive to separate a
5.9 cm frame offset. The test proposed in §5.1 should use the mask **coverage** and **leak**
statistics, as the 2026-09-30 note did, rather than the median alone. The hand samples carry F-2-9.

### 9.4 Auxiliary nodes

| Node | What it does (Verified at interface level) | In a default pipeline? |
|---|---|---|
| `image_rectifier_node.py` (107 lines) | `image_raw` + `camera_info` → `image_rect` + `camera_info_rect` (OpenCV undistortion) | no ([02](02_ros_graph_and_dataflow.md)); the RealSense publishes rectified streams itself |
| `frame_grabber.py` (155 lines) | saves frames of an image topic, plus a CBF diagnostics topic, for offline figures | no (manual tool) |

## 10. Integration

**Inputs:**
- `/camera/camera/depth/image_rect_raw` and `…/depth/camera_info`;
- TF for the 5 segment links;
- two files: the extrinsic and the robot config (`fr3_complete.yaml` or its temporary override copy).

**Outputs:**
- `/cbf/per_link_distances` (`MultiLinkDistance`, BEST_EFFORT, depth 1) → `cbf_safety_filter`,
  `iso_safety_monitor`, `iso_evidence_logger`, `experiment_logger`;
- `/human_robot/multi_distance` and `/human_robot/distance` (legacy) → overlays and loggers
  ([02](02_ros_graph_and_dataflow.md) §3).

**Timing chain (Derived from the code and configuration; rates are in 02 §6):**
1. depth capture at 90 Hz (`distance.depth_rate_hz`);
2. the driver duplicates frames, and these are dropped;
3. the 2-deep queue drops the oldest frame when the compute thread is behind;
4. mask, distance, tracking and publish all run in one thread;
5. the header stamp is the **capture** stamp, which the filter uses to compute ages and Δt
   (`_on_distances` plausibility window).

Every frame-counted threshold (LPF α, track M-of-N, coast, self-detection counts) is re-derived when
the measured rate departs from the nominal (`_retune_for_rate`).

**Failure modes and how they surface:**

| Failure | Perception behaviour | Visible as |
|---|---|---|
| No depth frames | no messages at all | the filter's stale-input handling (ch. 07/08) |
| No obstacle nearby | empty `MultiLinkDistance` heartbeat | `links = []` |
| Obstacle disappears | hold up to 0.10 s (valid entry, ph = 0: **F-2-11**), then invalid | `hold expired` log |
| TF missing for the EE, or for more than half the links | frame skipped | warning; no message for that frame |
| Stale extrinsic or wrong mesh | self-leak: the arm reads as an obstacle | `leak=` counter, the calibration check, self-detection |
| Tracker exception | the frame is published without tracks | error log; the filter falls back to the residual estimator |
| Resolution change | all caches, the LPF and the tracks are reset | info log |

## 10A. Consumer-side contract summary

The filter assumes the following:
1. `distance` is a gap to a **sphere** around `closest_point_robot`, already reduced by the radius
   and the mask margin;
2. labels come from `labelled_links`;
3. zero track fields mean "no track";
4. the header stamp is the capture time;
5. `closest_point_human` is a real point whenever `valid` is set. F-2-11 breaks this during holds.

## 11. Practical example — one obstacle point through the chain

**Numerically computed** with the shipped extrinsic and the depth K (`check_perception_numbers.py`).

Take an obstacle 0.25 m in +x of the fingertip control point at the home pose, at
o_B = (0.5569, 0, 0.4903) m:
- **Into the camera (5.2):** o_C = (−0.0888, 0.1869, 1.3182) m. It projects to pixel (397.2, 296.0)
  with Z = 1.318 m.
- **Back out.** Rounding to pixel (397, 296) and to the millimetre gives o_C' = (−0.0893, 0.1870,
  1.318). Mapped back to the base that is (0.5571, −0.0005, 0.4904). The quantisation error is
  0.5 mm.
- **The gap (5.3):** ‖o − c‖ = 0.2502 m. Subtracting r = 0.05 gives 0.2002 m. Subtracting the margin
  m = 12 · 1.318 / 428.8 = 0.0369 m (§3.4) gives a published **d = 0.163 m**: 3.7 cm less than the
  true gap to the sphere.
- **Direction:** n̂ ≈ (−1, 0, 0), from the obstacle toward the fingertip.
- **Confidence:** 1.0 with 500 or more obstacle pixels.
- **Zone:** 0.163 m lies in `danger` (0.1–0.2).

If the obstacle then approached at 3 m/s, the 90 Hz LPF would let the distance fall by at most
2 m/s × 11 ms = 2.2 cm in the first frame and accept the full drop one frame later (§3.5).

## 12. Consequences of modification

| If you change … | … then also check |
|---|---|
| `robot.segments` (control points, radii) | it is read by both control-point builders ([04](04_math_foundations.md) §4.4), so the published labels change for six consumers (§6.2). Zero-length segments create duplicates (F-2-1, F-2-10). Sphere coverage changes (F-2-4). |
| The order of `segment_links`, or the order of `build_cp_messages` | labels are positional. Change publisher and consumers together; `test_cp_label_agreement.py` and `test_multi_obstacle_rows.py` pin the contract. Adding an explicit index field to `LinkDistance` would remove the positional coupling, and would also fix F-2-10. |
| Mask dilation (`robot_mask_dilate_px`, `ee_mask_dilate_px`, `search_exclusion_extra_px`, `mask_downsample`) | the margin (5.6) moves with them, so every published distance moves (F-2-8). So do the SSB band geometry and the self-leak rate. |
| `depth_gate_bias_m` / `depth_gate_tol_m` | only b + τ matters for the gate (5.5); the SSB band uses τ and γ separately. Re-measure with `calibration_check`. |
| Meshes, the URDF hand joint, or `sample_points_per_link` | mask, z-buffer, `calibration_check` absolute values (the sampling bias) and SSB model points. Fixing F-2-9 changes all four. |
| `camera_extrinsics.yaml` (recalibration) | regenerate `camera_link_extrinsics.yaml` (F-2-7), reset the `calibration_check` baseline, and re-check Q-1-2. |
| `lpf_tau_s`, `max_approach_speed_mps`, `iso_distance_hold_max_s` | the conservative asymmetry of §3.5 and the hold window. A longer hold widens the F-2-11 exposure. |
| The behaviour of `_lpf_pass` during a hold | if held entries carried the previous point, or were published invalid, F-2-11 would close. Check the consumers in `cbf_state_rows` (`pr − ph`) and the logger. |
| Tracker tuning (`q_jerk`, `sigma_meas_m`, gates, M-of-N, coast) | the static noise floor (≈7 cm/s) and the lag (§7.3) feed the velocity trust gates of chapter 07. The coast length interacts with `gate_max_m` (Q-2-2). IMM with a short coast was the worst configuration the YAML recorded. |
| `imm_enabled` | it is "TEMPORARY" in the YAML; the consumer-visible blend and covariance change. |
| `multi_obstacle_k` | 1 vs the launch default 2: the extra rows and F-2-10 exposure, and the `cluster_id` path used for track lookup. |
| A per-control-point search radius in Step 6 | the self-detection key convention diverges from `labelled_links` (§6.2). |
| Enabling `prediction` or SSB | new extra rows and fields. The consumer behaviour is documented in chapter 07; both depend on F-2-9 and F-2-10. |
| `publish_contact_regime`, `min_thresh` / `max_thresh` | rows below 0.08 m and beyond 0.7 m. The filter's 1.2 m horizon is unreachable beyond 0.7 m (§6.3). |

## 13. Findings from this chapter

| ID | Short title | Section | Reproduced by ([tools/stage2_verification/](tools/stage2_verification/README.md)) | Still needs |
|---|---|---|---|---|
| F-2-4 | sphere control-point model: gaps between spheres, links 0–3 uncovered, the hand wider than its sphere | §3.2 | `check_geometry_and_so3.py`, `check_hand_mesh.py` | Stage 3 + runtime (does it matter) |
| F-2-7 | `camera_link_extrinsics.yaml` stale against `camera_extrinsics.yaml` (0.276 m, 5.14°) | §5.2 | `check_camera_extrinsics.py` | — |
| F-2-8 | the mask margin is subtracted from every obstacle pixel (a uniform 3–6 cm low bias) | §3.4 | `check_perception_numbers.py` (arithmetic) | the author's intent |
| F-2-9 | the hand mesh is posed with `fr3_link8` instead of `fr3_hand` (45°, up to 7.96 cm) | §4.2 | `check_hand_mesh.py`; URDF side in `check_geometry_and_so3.py` | runtime (Q-2-4) |
| F-2-10 | duplicate control points break the multi-obstacle rank labels | §6.2 | `check_cp_labels.py` | Stage 3/5 (downstream effect) |
| F-2-11 | a held entry is valid but its `closest_point_human` is (0, 0, 0); the filter's normal uses `pr − ph` | §3.5 | `check_held_entry.py` (message side only) | Stage 3 (Q-2-1), runtime |
| F-2-12 | hand-eye node: the docstring says LM, the code uses TRF; Huber at f_scale 1 is inert | §9.1 | — (code reading) | — |

Questions:
- **Q-1-2** is answered: a deliberate choice, with the root cause open.
- **Q-1-3** is answered: inconsistent.
- **Q-2-1** (new): does any stage of `cbf_state_rows` reject ph = 0 (F-2-11)? Stage 3.
- **Q-2-2** (new): `gate_max_m` against the 0.8 s coast (§7.4).
