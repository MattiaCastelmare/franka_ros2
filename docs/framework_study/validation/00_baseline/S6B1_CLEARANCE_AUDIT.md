# S6B.1 — Offline reconstruction and independent clearance verification

> **Scope.** Offline analysis only, approved by the user on 2026-11-12. **No simulation, launch or
> build was run.** No production, test, launch or configuration file was modified. The ground-truth
> logger and the harness were not touched. The input is the S6B recording of one cell
> (`pentagon_static_baseline_bare / full / seed 0 / perception true`; [S6B_STATIC_OBSTACLE.md](S6B_STATIC_OBSTACLE.md)).
> Baseline `ba68077`, HEAD `f708991`.
>
> **Evidence.**
> - Outputs: `~/franka_baseline_evidence/baseline_f708991/07_clearance_audit/` (README_S6B1.txt,
>   11 files, `SHA256SUMS` hash `102256fc…7b27`, write-protected).
> - Scripts: [tools/baseline_validation/](../../tools/baseline_validation/README.md).
> - The S6B evidence (`06_static_obstacle/`) was hash-verified before and after and is unchanged.
>   The container copy that the scripts read was checked against the same hashes.
>
> **Labels.** Each claim is placed in one of four classes (with the 00_INDEX §2 label in brackets):
> - **Recorded truth**: a `truth.npz` sample [Runtime (recorded)];
> - **Reconstructed**: computed from the bag's measured joint samples [Derived];
> - **Derived**: computed from either;
> - **Unverified**: not established.
>
> Code facts are labelled Verified.

---

## 0. Answer in brief

| # | Question | Answer | Label |
|---|---|---|---|
| 1 | Could the missing ground truth be reconstructed? | **Yes.** The bag recorded the measured joint stream at 1 kHz with **no gap above 2 ms** in the whole episode. That includes 90–561 samples inside each truth hole. The obstacle pose stream was also recorded at 1 kHz. The reconstruction uses the ground-truth node's own geometry code, URDF and configuration. | Verified (code, hashes); Derived |
| 2 | Agreement with the recorded truth? | **Exact**: 27 949 of 27 949 episode truth samples were matched by header stamp; the joint vectors are bit-identical; the clearance difference is **0.0 m** (median and maximum). The single exception is the obstacle-placement instant (t = 10.550 s), where the truth used the parked pose; it is explained in §4.2 and irrelevant (gap 0.30 m). | Derived |
| 3 | Minimum clearance over the episode | **0.12272 m**, CP `fr3_link8#2`, t = 36.927 s sim (pass 4; also a recorded-truth sample). Continuous-time lower bound between samples: **0.12236 m**. | Derived |
| 4 | Was 10 cm maintained? | **Yes, under the sphere model**: 0 of 29 996 reconstructed samples below 0.10 m, and 0 below 0. No interval of the episode is left without a sample. | Derived |
| 5 | Was pass 3 verified? | **Yes.** Its true minimum lies **inside** the 395 ms hole: **0.12313 m** at t = 30.041 s (`fr3_link8#2`). That is 0.12 mm below the last recorded sample before the hole (0.12325 m), so the recorded truth alone over-stated pass 3's minimum. | Derived |
| 6 | Remaining limitations | (a) The sphere model only: no mesh-level statement. Links 1–3 have no CP; the hand and finger corners are outside the spheres. Gazebo's obstacle has no collision element. (b) The reconstruction is independent of the truth node's **runtime**, not of its **inputs**: same joint stream, same code, same model. (c) A single seed. (d) It covers this static-obstacle cell; for moving obstacles see F-8-13. | Verified; Inferred |
| 7 | Accept the S6B clearance result as baseline evidence? | **Yes for the sphere-model clearance of this one cell**: 12.27 cm ≥ 10 cm, complete coverage, method validated sample-for-sample. **Not** as mesh-level collision freedom, and not as statistical evidence. F-6-5 stays open for the harness: `truth.npz`, `min_h` and `contact` still have holes, and the bag-based reconstruction is a per-cell remedy. | Derived; acceptance is the user's decision |

---

## 1. The ground-truth gaps

### 1.1 Mechanism (Verified, `ground_truth_node.py`)

- `_js` runs on every `/NS_1/franka/joint_states` sample (subscription queue depth **100**, i.e.
  0.1 s at 1 kHz).
- It pairs each sample with the **last received** obstacle pose (`_poses`, queue depth **50**).
- A 5 s timer `save()` rewrites the whole history with `np.savez_compressed` on the same
  single-threaded executor. The history grows, so each save takes longer. Once a save outlasts
  0.1 s of sim time, joint samples are dropped from the queue (chapter 12 §7.1, F-6-5).

### 1.2 Every gap (recorded truth, episode 10.550–40.560 s sim)

Inside the episode the truth stores 27 949 samples, while the bag holds 29 996 measured joint
samples, so 2047 samples are missing from the truth. They fall in these spans
(`out/s6b1_supplementary.json` `reconstruction_only_spans`):

| # | Last truth sample before | First truth sample after | Hole (ms) | Bag samples inside | Max bag spacing inside | Recorded truth at the edges (m) | Reconstructed min inside (m), CP | Pass affected |
|---|---|---|---|---|---|---|---|---|
| 1 | 14.998 | 15.089 | 91 | 90 | 1 ms | 0.1469 / 0.1429 | 0.14291 at 15.088, `fr3_link8#2` | approach to pass 1 (minimum at 15.601, recorded) |
| 2 | 20.001 | 20.175 | 174 | 173 | 1 ms | 0.3691 / 0.3480 | 0.34810, `fr3_link8#2` | none |
| 3 | 24.996 | 25.332 | 336 | 335 | 1 ms | 0.2497 / 0.3052 | 0.24984 at 24.997, `fr3_link7#1` | none |
| 4 | 29.999 | 30.394 | **395** | 394 | 1 ms | 0.1232 / 0.1280 | **0.12313 at 30.041, `fr3_link8#2`** | **pass 3 minimum** |
| 5 | 34.997 | 35.481 | 484 | 483 | 1 ms | 0.2632 / 0.1832 | 0.18330 at 35.480, `fr3_link8#2` | approach to pass 4 (minimum at 36.927, recorded) |
| 6 | 35.481 | 35.484 | 3 | 2 | 1 ms | 0.1832 / 0.1829 | 0.18300 | — |
| 7 | 39.999 | 40.561 (after the episode) | 562 | 561 | 1 ms | 0.3852 / 0.4194 | 0.38531 at 40.000, `fr3_link8#1` | none (end-of-episode save) |

In addition, the truth lacks **isolated single samples** right after holes 2, 3, 4 and 6 (for
example 20.179, 20.181, 20.183, 20.185 and 30.395, 30.397, 30.404). These are 2 ms steps that the
">10 ms" hole statistic of S6B did not count (Recorded truth).

The holes are 5.0 s apart and grow (91 → 562 ms). Before the episode the truth also has a 172 ms
hole at 0.03 s, an 18 ms hole at 4.914 s and several 3 ms holes, all while the obstacle was parked at
z = −20 m (gap ≈ 20 m) and therefore irrelevant (Recorded truth).

### 1.3 Are measured joint states available inside the holes? (Derived)

| Check | Result |
|---|---|
| Topic | `/NS_1/franka/joint_states`, the **same** topic the truth node subscribes to (Verified, `ground_truth_node.py` L58) |
| Source | the Gazebo plant's `joint_state_broadcaster`, from the simulated joint **state** interfaces, remapped to `franka/joint_states` (Verified: `gz_fr3_plant.launch.py` L11 docstring; the run's `fr3_rsp.urdf` plugin `<remapping>joint_states:=franka/joint_states</remapping>`; `launch.log` loads `joint_state_broadcaster`). Not a commanded quantity. |
| Not used | `/NS_1/joint_states`, the aggregated topic (also in the bag), and the commander's `q_des` |
| Messages | 40 659 in the run, 29 996 in the episode (30 011 expected at 1 kHz; 15 intervals of 2 ms, none longer) |
| Header stamps | monotonic, 0 duplicates; bag receive − header: median 0, range −4 … +5 ms (the recorder's sim clock) |
| Joint ordering | **one** name order in every message: `fr3_joint1, fr3_joint3, fr3_joint2, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7` (not 1…7). The script maps **by name**, as the truth node does (`_js` L76–79). |
| Inside each hole | 1 sample per ms, maximum spacing 1 ms (table above) |

### 1.4 Is the actual obstacle pose recoverable? (Derived)

| Source | Value |
|---|---|
| Scenario (`obstacles_json` of the truth node, `world.sdf` plugin) | static, p0 = (0.55, −0.25, 0.449) m, r = 0.05 m, parked at (0, 0, −20) until START (Verified) |
| Bag `/sim/truth/obstacle_poses` (ScenarioObstacles plugin, `truth_period` 0.001) | 40 132 messages, 1 pose each; first placed stamp 10.550 s; afterwards **constant**, max deviation from p0 1.6e-15 m; 1 ms spacing in the episode, including inside the truth holes |
| `truth.npz` `p` | p0 on every episode sample except the first (t = 10.550, still parked, §4.2) |
| Radius used by truth / reconstruction | 0.05 m (`obstacle_radius`) |

---

## 2. Method (Verified: code read and hashes)

1. **Inputs.**
   - The S6B cell bag, `truth.npz`, `fr3_rsp.urdf` and `run_manifest_sim.json`. Container copy;
     sha256 equal to the sealed S6B evidence (`inputs/container_copy_check.txt`).
   - The installed `fr3_complete.yaml`, `simexp/truth_geometry.py` and
     `franka_experiments/utils/distance_utils.py`. Their sha256 equals the repository source at
     `f708991` (`inputs/*_geometry_sha256.txt`).
   - These are exactly the truth node's launch parameters (`urdf_path`, `perception_config`,
     `obstacles_json`, `d_safe` 0.1; S6B `container_tmp/tmp__launch_params_gloxv0uu`).
2. **Kinematics.** `TruthGeometry(urdf, cfg)`: Pinocchio `framesForwardKinematics` of the measured
   q. Control points come from `define_control_points`, the perception's own function. There are
   11 CPs, `fr3_link4#0` … `fr3_link8#2`, all with r_cp = 0.05 m. The labels and radii are identical
   to `truth.npz`.
3. **Clearance.** `surface_gaps(cps, c, r)` = ‖p_cp − c‖ − r_obs − r_cp, signed and not clamped.
   This **is** the production truth formula (`truth_geometry.py` L67–74). It is not the perception
   engine's clamped visible-surface distance.
4. **Obstacle pairing.** The pose with the latest header stamp ≤ t (age 0 ms on every episode
   sample). The obstacle is static, so pairing by stamp or by arrival gives the same centre, except
   at the placement instant (§4.2).
5. **No interpolation.**
   - A clearance exists only at a measured joint sample.
   - The continuous-time statement in §5 is a **bound** (Lipschitz in the CP position, the obstacle
     being static: |d gap/dt| ≤ |v_cp|), not an interpolated value.

---

## 3. Coordinate frames (Verified; one Derived cross-check)

- The CP positions are FK in `fr3_link0`. The run's URDF roots `fr3_link0` at `base` with an
  identity `fr3_base_joint`.
- The robot is spawned by `ros_gz_sim create -file … -name fr3` with no pose arguments
  (`gz_fr3_plant.launch.py` L133–134), i.e. at the world origin. So `fr3_link0` = Gazebo world,
  which is also the frame of the ScenarioObstacles poses and `p0`. The `truth_geometry` docstring
  states the same convention.
- **Cross-check (Derived, weak):** the perception chain (camera at its `world.sdf` pose → TF → world)
  agreed with the truth gap within −8.0 … +0.8 cm (S6B §6.3). A frame offset of more than a few
  centimetres would show there. Truth and reconstruction share the frame by construction, so their
  agreement does not test it.

---

## 4. Validation against the recorded truth

### 4.1 Sample-for-sample comparison (Derived, `out/s6b1_reconstruction.json` `check_A_same_q`, `check_B_independent`)

| Check | Result |
|---|---|
| A. FK of `truth.npz`'s own q → its CP positions and gaps | max error **0.0 m** (CP) and **0.0 m** (gap) on 27 949 samples: the offline pipeline reproduces the node's computation exactly |
| B. Bag measured q matched to truth by header stamp | **27 949 / 27 949** truth samples matched; 0 truth samples without a bag sample |
| B. Joint vectors at matched stamps | max \|Δq\| = **0.0 rad** (bit-identical) |
| B. Clearance, all 11 CPs, matched samples | median **0.0 m**; maximum 0.0 m except one instant (§4.2) |
| Sampling | truth 1 kHz with 7 holes > 2 ms plus isolated single drops; reconstruction 1 kHz with 15 intervals of 2 ms and none longer |
| Timestamp alignment | identical header stamps; obstacle pose age at the reconstruction 0 ms |
| Geometric model | identical (same code, configuration, URDF and radii); both are sphere models |

**Tolerance justification.** Identical inputs and identical code should give bit-identical results,
so the tolerance is 0, and it is met. Any nonzero difference would have signalled a different
joint sample, code path or obstacle pose.

### 4.2 The one disagreement (Derived + Verified)

At t = 10.550 s (the first episode sample) the truth gap is 20.36 m and the reconstruction 0.303 m.
The truth paired that joint sample with the **parked** pose (stamp 10.549). The placed pose, stamped
10.550, had not yet been received: the truth node pairs by arrival, not by stamp (`_js` uses
`self.pos`, the last received pose). The difference is 0 from the next sample on. It does not
affect any clearance statement (gap 0.30 m).

### 4.3 What the agreement does and does not validate (Inferred)

- **It validates:**
  - that the bag holds the same measured joint stream the truth node consumed;
  - that the offline pipeline is the node's geometry;
  - therefore, that the reconstruction inside the holes is what the node **would** have stored.
- **It does not validate:**
  - the sphere model against the real geometry;
  - the joint-state stream against the physics state (it is the plant's broadcast state, taken as
    exact in sim);
  - anything an independent sensor would add.

---

## 5. Re-evaluated safety result

| Quantity | Value | Class |
|---|---|---|
| Minimum clearance, episode | **0.12272 m** at **t = 36.927 s**, `fr3_link8#2` | Recorded truth = Reconstructed |
| Continuous-time lower bound | ≥ 0.12236 m. Max CP speed 0.362 m/s (finite differences) × max spacing 2 ms / 2 = 0.36 mm | Derived (bound; assumes the speed within an interval does not exceed the observed finite-difference maximum, Inferred) |
| Samples below 0.10 m / below 0 | 0 / 0 (of 29 996) | Reconstructed |
| Time below 0.10 m / penetration (sphere model) | 0 s / none | Reconstructed |
| Unverified intervals of the episode | **none** (sphere model); every ms has a measured sample | Derived |

**Per pass** (local minima of the reconstructed minimum gap; `passes`):

| Pass | t of minimum (s) | Reconstructed min (m) | CP | Recorded truth at that instant? | Recorded-truth min in ±1 s (m) | Truth holes in ±1 s | Samples < 0.10 m |
|---|---|---|---|---|---|---|---|
| 1 | 15.601 | 0.12281 | `fr3_link8#2` | yes | 0.12281 | 91 ms at 14.998 (outside the minimum) | 0 |
| 2 | 23.025 | 0.12284 | `fr3_link8#2` | yes | 0.12284 | none | 0 |
| 3 | **30.041** | **0.12313** | `fr3_link8#2` | **no (inside the 395 ms hole)** | 0.12325 (edge at 29.999) | 395 ms at 29.999 | 0 |
| 4 | 36.927 | **0.12272** | `fr3_link8#2` | yes | 0.12272 | none | 0 |

**Classification (the user's scheme, thresholds unchanged).** All four passes and the whole episode:
**clearance ≥ 0.10 m, requirement met** under the sphere model. The S6B "pass 3 Unverified" verdict
is resolved by reconstruction: met, with a minimum of 0.12313 m.

**What is still not claimed.**
- **Collision freedom of the robot mesh.** Links 1–3 have no CP. The hand and finger geometry
  extends beyond the CP spheres (chapter 12 §7.1, F-6-7). Gazebo cannot report a contact with the
  obstacle, because its model has a **visual** sphere and no collision element (`world.sdf`,
  Verified). The arm passed 12.3 cm from the ball on the sphere scale. The hand's real surface is
  closer than its CP sphere in some directions, by an amount not computed here.
- **Generality.** One seed and one phase (F-6-12). No statistical claim.

---

## 6. Impact on baseline acceptance

1. **F-6-5 for S6B: addressed for this cell.**
   - Every truth hole is covered by measured joint samples at 1 kHz.
   - The reconstruction reproduces every existing truth sample exactly.
   - The reconstructed clearance is therefore an admissible stand-in for the missing truth
     (Derived).
2. **F-6-5 in general: still open.**
   - The harness metrics (`min_h`, `contact`, `neg_duration`) are still computed from `truth.npz`
     with holes. In S6B the harness happened to be right, because the episode minimum (pass 4) was
     recorded.
   - Pass 3's true minimum was not recorded. The truth over-stated it by 0.12 mm, harmless here,
     but the same blind spot hid a 3 cm penetration in the synthetic case of F-6-5.
   - The remedy used here (a bag-based reconstruction) needs `/NS_1/franka/joint_states` and
     `/sim/truth/obstacle_poses` in the bag. Both are in `BAG_TOPICS` (`gz_cbf_stack.launch.py`
     L66–69, Verified). It also needs the bag itself to have no drops; it had none
     here.
3. **Reliable evidence for 0.10 m?** **Yes, for this cell under the sphere model.** The clearance
   margin is 2.27 cm above d_safe with no unverified interval. **No** for mesh-level contact, for
   other seeds, phases or scenes, and for moving obstacles (see F-8-13).
4. **The thresholds and the harness are unchanged.** Whether to adopt the bag reconstruction as the
   standard clearance check, or to fix the logger, is the user's decision. Either is a production or
   protocol change needing approval.

---

## 7. New finding

**F-8-13 (Verified code; Runtime (recorded)).** The truth node pairs each joint sample with the
**last received** obstacle pose, not the pose with the same stamp.
- The pose queue holds 50 ms (depth 50 at 1 kHz) and the joint queue 100 ms (depth 100). After each
  save stall the node drains the backlog of joint samples while the pose has already jumped to the
  newest one.
- In S6B, 1947 of 27 949 episode samples (7 %) were paired with a pose **49–50 ms in the future**,
  for 235–431 ms after each hole (`truth_pose_pairing_after_holes`). The first-sample mismatch at
  placement (§4.2) is the same mechanism.
- **Effect here:** none (static obstacle).
- **Effect with a moving obstacle (Inferred):** a gap error up to |v_obs| × 50 ms right after each
  hole, e.g. 2.5 cm at 0.5 m/s, in either direction.

---

## 8. Reproducibility

```
# container (rosbag2_py, pinocchio, installed packages; reads the S6B cell read-only)
docker exec -w /tmp -e PYTHONDONTWRITEBYTECODE=1 franka_ros2 bash -c 'source /ros2_ws/install/setup.bash;
  python3 -B /home/user/s6b1_reconstruct_clearance.py <cell_dir> <out_dir>'
# host or container (numpy)
python3 -I docs/framework_study/tools/baseline_validation/s6b1_supplementary.py <cell_dir> \
  <out_dir>/s6b1_reconstruction.npz <out_dir>/s6b1_supplementary.json
```

- Run time: about 10 s for the whole run.
- Container scratch left in place: `/home/user/s6b1_20261112/`,
  `/home/user/s6b1_reconstruct_clearance.py`, `/home/user/s6b1_container_sums.txt`.
