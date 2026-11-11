# 04 — Mathematical foundations: frames, FR3 kinematics, control-point Jacobians, dynamics

> **Source baseline:** `ba68077`. Written in Stage 2 (2026-11-11). Labels as in
> [00_INDEX.md](00_INDEX.md) §2. **Numerically verified** means the relation was evaluated with the
> baseline source code and Pinocchio 4.0.0, inside the user's `franka_ros2` container, by an
> isolated script that changes nothing (see §7). All other numbers come from reading the source.
>
> **Reproducible checks.** Every numerically verified statement here is recomputed by the
> scripts in [tools/stage2_verification/](tools/stage2_verification/README.md):
> `check_kinematics_dynamics.py`, `check_geometry_and_so3.py` and `check_home_example.py`. The
> 2026-11-11 rerun reproduced every value in this chapter to the printed digits.

This chapter covers the mathematics that every later chapter assumes: frames, joint ordering,
forward kinematics, the Jacobians of the robot's *control points*, their time derivatives, and the
rigid-body dynamics used to turn accelerations into torques. The barrier itself (chapter 07) only
needs three things from here:
- a point **p** on the robot;
- its Jacobian **J**;
- the drift term **J̇ q̇**.

This chapter shows where these come from and checks that the code computes them correctly.

---

## 1. Intuition

The safety filter reasons about **points on the robot** that might hit something, not about the
robot as a whole. For each such point it needs to know:
1. **where it is** (forward kinematics), to measure its distance to the nearest obstacle;
2. **how its velocity depends on the joint velocities** (the Jacobian), so that "do not approach
   faster than…" becomes a linear constraint on the joints;
3. **how its acceleration depends on the joint accelerations.** The QP decides q̈, not q̇ (chapter 07
   uses a second-order barrier), and p̈ = J q̈ + J̇ q̇. The term J̇ q̇ is the acceleration the point has
   even when q̈ = 0, such as the centripetal acceleration of a point on a spinning link. Leaving it
   out produces a barrier that is wrong by exactly that amount.

The torque pipeline then converts the chosen q̈ into joint torques through the rigid-body dynamics
M q̈ + C q̇ + g = τ (§5).

Two things would go wrong silently if any of this were inconsistent:
- **Mismatched geometry.** Perception measures the distance from obstacle points to one set of
  robot points, while the filter differentiates another. The constraint would then protect a point
  nobody measured. The code guards against this explicitly, and §4.4 checks that it holds.
- **Scrambled joint order.** If the joint-state vector arrives in a different order (it does, §2.2),
  every Jacobian column pairs with the wrong joint velocity, and the result looks plausible but is
  wrong.

## 2. Frames, units, joint ordering

### 2.1 Frames

| Symbol | Frame | Where it is defined | Notes |
|---|---|---|---|
| {W} | `world` | TF static `world → base` (identity), published by the torque launch ([02](02_ros_graph_and_dataflow.md) §5) | Fixed |
| {B} | `base` | URDF root link (`franka_description` `fr3.urdf.xacro`) | **Numerically verified:** in the Pinocchio model the frames `base` and `fr3_link0` both have identity placement relative to `universe` (joint 0). So for this code, Pinocchio's world frame = `base` = `fr3_link0`. |
| {0} | `fr3_link0` | URDF, joint `fr3_base_joint` (fixed, identity) | `fr3_complete.yaml robot.base_frame`. Perception expresses obstacle points here (chapter 05). |
| {i} | `fr3_link1` … `fr3_link7` | URDF revolute joints `fr3_joint1..7` | The origin of {i} lies on the axis of joint i |
| {8} | `fr3_link8` | URDF fixed joint `fr3_joint8` | The flange. In the hand-less model it is the last frame. |
| {H}, {T} | `fr3_hand`, `fr3_hand_tcp` | only in the model with the hand | **Numerically verified:** relative to {8}, {H} is at t = 0 and rotated −45° about z; {T} is at t = (0, 0, 0.1034) m with the same rotation |
| {C} | camera optical frames | chapter 05 | |

**Units:** metres, radians, seconds, kilograms, newton-metres. **Convention:** `oMf[i]` is the
pose of frame i in the world, a rotation `R` (whose columns are the axes of i written in world
coordinates) plus a translation `p`. A point with local coordinates r has world coordinates
p_w = R r + p (`SE3.act`).

**Verified** (`utils/kinematics.py::CBFKinematics`). Every control-point position, Jacobian and
J̇ is written in **world coordinates**. Because world ≡ `fr3_link0`, they are also in robot-base
coordinates, which is the frame chapter 05 expresses obstacle points in. This holds only while
`world → base → fr3_link0` stays the identity. If the robot base were ever mounted with a non-zero
`world → base`, the two would differ. That would be harmless only if perception kept using
`fr3_link0` as its frame (chapter 05 checks this).

### 2.2 Joint ordering

- **The model.** Pinocchio orders joints as in the URDF: `fr3_joint1` … `fr3_joint7` (**numerically
  verified** for the hand-less model, nq = nv = 7). The model with the hand adds `fr3_finger_joint1`
  and `fr3_finger_joint2` (nq = nv = 9). Its neutral configuration is all zeros, so the fingers sit
  at 0 m, closed. `utils/constants.py::FR3_JOINT_NAMES` holds the arm order as a list.
- **The message.** `sensor_msgs/JointState` carries `name[]`, `position[]` and `velocity[]`. ROS does
  not guarantee any order. A project note records `/NS_1/franka/joint_states` arriving as
  `[j1, j3, j6, j7, j2, j4, j5]` (`PROGRESS.md` §7: a lead, not re-observed here).
- **How consumers map names to indices (Verified):**

  | Consumer | Mapping | Robust to orderings that differ between messages? |
  |---|---|---|
  | `cbf_safety_filter::_on_joint_state` | `dict(zip(name, position))` per message | yes |
  | `qddot_to_torque` `_js_cb` (L172–188) | rebuilds `name → index` per message; skips messages that lack an arm joint | yes; the comment explains why |
  | `kinematics.py::JointStateManager._joint_state_cb`, `pentagon_qddot_commander._js_cb`, `rl_policy_commander`, `pentagon_torque_commander`, `cbf_OSCBF_filter` | caches the index map **from the first message** that contains all seven names; later messages are only length-checked | only if every publisher on the topic uses the same order |

  A finger-only message on the same topic is skipped safely by the cached-map consumers (its
  `position` is too short). Two **arm** publishers with different orders on one topic would be mapped
  silently wrong (Inferred risk, F-2-5). No such configuration was found in the launch files
  ([02](02_ros_graph_and_dataflow.md) §3.8 lists the publishers).

### 2.3 Two Pinocchio models

| Model | Built by | Contents | Used for |
|---|---|---|---|
| **No hand** | `kinematics.py::build_urdf_no_hand` (`xacro … hand:=false`) | 7 DoF, frames up to `fr3_link8`; total mass 17.891 kg (numerically verified) | `CBFKinematics` in `cbf_safety_filter`, `iso_safety_monitor`, `iso_evidence_logger`, `experiment_logger`, many scripts |
| **Hand** | `kinematics.py::generate_urdf_from_xacro` (`hand:=true ee_id:=franka_hand`) | 9 DoF, adds `fr3_hand`, the fingers and `fr3_hand_tcp`; total mass 18.604 kg | dynamics in `qddot_to_torque`, the commanders, `cbf_OSCBF_filter`, the hand-eye nodes |
| **Self-collision** | `kinematics.py::build_urdf_with_sc` (`with_sc:=true`, optional `safety_distance`) | adds the `*_sc` capsule links | self-collision rows (`cbf_state_rows.py` L2017–2028; chapter 08) |

The kinematic chain up to `fr3_link8` is identical in all three, because they come from the same
xacro (Derived). The hand-less model is enough for the barrier: the hand is represented by a
**rigid fingertip offset** (§4.3), not by its links.

## 3. Mathematical foundations

### 3.1 Forward kinematics

Each joint i contributes a homogeneous transform. With the fixed placement X_i of joint i in its
parent frame and the rotation about the joint axis z_i,

  ⁰T_i(q) = ⁰T_{i−1}(q) · X_i · Rot_z(q_i),  ⁰T_0 = I  (4.1)

and a frame f attached to joint i has ⁰T_f = ⁰T_i · X_f.

**Numerically verified FR3 geometry** (`check_geometry_and_so3.py`). At the home pose
q_home = (0, −π/4, 0, −3π/4, 0, π/2, π/4), the frame origins (m) are:

| Frame | p (world) | Constant distance to the next origin (over 200 random q) |
|---|---|---|
| `fr3_link0` | (0, 0, 0) | 0.333 to link1 |
| `fr3_link1` | (0, 0, 0.333) | **0** to link2 |
| `fr3_link2` | (0, 0, 0.333) | 0.316 to link3 |
| `fr3_link3` | (−0.2234, 0, 0.5564) | 0.0825 to link4 |
| `fr3_link4` | (−0.1651, 0, 0.6148) | 0.39276 to link5 |
| `fr3_link5` | (0.2189, 0, 0.6973) | **0** to link6 |
| `fr3_link6` | (0.2189, 0, 0.6973) | 0.088 to link7 |
| `fr3_link7` | (0.3069, 0, 0.6973) | 0.107 to link8 |
| `fr3_link8` | (0.3069, 0, 0.5903) | — |

These are the FR3 DH parameters in Craig convention: d₁ = 0.333, d₃ = 0.316, a₄ = 0.0825,
d₅ = 0.384 (√(0.384² + 0.0825²) = 0.39276), a₇ = 0.088, d_F = 0.107. The origins of links 1 and 2
**coincide for every q**, and so do the origins of links 5 and 6, because the consecutive joint axes
intersect there. This matters in §4.2.

### 3.2 The geometric Jacobian and Pinocchio's `LOCAL_WORLD_ALIGNED`

For a frame f with origin o_f, the six-dimensional velocity is

  [ v_f ; ω_f ] = J_f(q) q̇,  J_f = [ J_v ; J_ω ] ∈ ℝ^{6×n}.  (4.2)

Pinocchio can write the result in three ways:
- `WORLD`: the spatial velocity at the world origin;
- `LOCAL`: in the axes of f;
- `LOCAL_WORLD_ALIGNED` (LWA): the linear velocity **of the origin of f**, ṗ_{o_f} = J_v q̇, and the
  angular velocity ω_f, both written in **world axes**.

Every Jacobian in this code base is LWA. That makes J_v the plain derivative of the world
coordinates of o_f: J_v = ∂p_{o_f}/∂q.

**Revolute-joint column (Derived).** For a revolute joint j with unit axis a_j (world) through point
o_j, and o_f downstream of j:

  J_v[:, j] = a_j × (o_f − o_j),  J_ω[:, j] = a_j  (and 0 for joints not upstream of f).  (4.3)

### 3.3 The Jacobian of a point rigidly attached to a link

Take a point P fixed on the body of frame f, with lever arm r = p_P − p_{o_f} written in world
coordinates. Rigid-body kinematics give

  ṗ_P = v_f + ω_f × r = J_v q̇ − r × (J_ω q̇) = (J_v − [r]_× J_ω) q̇,

where [r]_× is the matrix with [r]_× w = r × w (`math_utils.py::skew`). Therefore

  **J_P = J_v − [r]_× J_ω.**  (4.4)

### 3.4 The time derivative J̇ and the acceleration of a point

Differentiate (4.4) along the motion. The lever arm r is fixed in the body but rotates with it, so
ṙ = ω_f × r:

  J̇_P = J̇_v − [ṙ]_× J_ω − [r]_× J̇_ω = **J̇_v − [r]_× J̇_ω − [ω_f × r]_× J_ω**.  (4.5)

Differentiating ṗ_P = J_P q̇ once more gives the **classical** (not spatial) acceleration of the
point:

  **p̈_P = J_P q̈ + J̇_P q̇.**  (4.6)

This is the relation the HOCBF of chapter 07 needs. Its relative-degree-2 condition is written in
q̈, and J̇ q̇ is the drift.

**A subtlety about Pinocchio's J̇.** `getFrameJacobianTimeVariation(…, LOCAL_WORLD_ALIGNED)` must be
the *time derivative of the LWA Jacobian matrix*, d/dt J_LWA(q(t)), for (4.5)–(4.6) to hold. It must
not be the derivative of a spatial Jacobian expressed elsewhere. **Numerically verified**
(Pinocchio 4.0.0): the 6×7 time variation of `fr3_link5` matches the central difference of J_LWA
along q̇ to 2.9·10⁻¹⁰ (25 random states).

### 3.5 Interpolated control points

A control point between two frame origins (§4.1) is defined as

  p(t) = (1 − t) p_{o_0}(q) + t p_{o_1}(q),  t ∈ (0, 1) fixed.  (4.7)

Equation (4.7) is linear in the two positions, so its derivatives are exactly

  J = (1 − t) J_v⁰ + t J_v¹,  J̇ = (1 − t) J̇_v⁰ + t J̇_v¹,  p̈ = J q̈ + J̇ q̇.  (4.8)

Nothing here requires the two frames to belong to the same rigid body. When they don't (link3 and
link4 are separated by joint 4), the point is **not a material point** of the robot: it slides along
the segment joining the two origins as the joint between them moves. (4.8) is still the exact
derivative of the point *as defined*. Whether that point represents the physical surface well is a
geometric-modelling question, taken up in chapter 05.

### 3.6 Rigid-body dynamics

For the n = 9 joint model with the hand,

  M(q) q̈ + C(q, q̇) q̇ + g(q) = τ + τ_ext,  (4.9)

where:
- M is the symmetric positive-definite joint-space inertia matrix;
- C(q, q̇) q̇ collects the Coriolis and centrifugal torques;
- g is the gravity torque.

Pinocchio writes `nle(q, q̇) = C(q, q̇) q̇ + g(q)` (the "non-linear effects"). The feed-forward torque
computed here **excludes gravity**, because the FR3 firmware compensates it
([01](01_architecture.md) §2, F-1-3):

  τ_ff = M_aa(q) q̈_a + [nle(q, q̇) − g(q)]_a,  (4.10)

where the subscript a selects the seven arm joints. The finger joints are held at q = 0, q̇ = 0,
q̈ = 0. Their inertia still enters M_aa, through the hand link carried by joint 7.

### 3.7 Orientation error on SO(3)

For R ∈ SO(3) with rotation angle θ = arccos((tr R − 1)/2) ∈ [0, π] and unit axis a,

  log(R)^∨ = θ a,  with a = vee(R − Rᵀ)/(2 sin θ) for 0 < θ < π.  (4.11)

As θ → 0 this tends to ½ vee(R − Rᵀ). At θ = π the vee term vanishes, and the axis comes from
R + I = 2 a aᵀ. The norm θ grows monotonically up to π, unlike ‖½ vee(R − Rᵀ)‖ = sin θ, which folds
back above 90°. **Conjugation identity (Derived):** log(Q A Qᵀ) = Q log(A). Therefore

  log(R_d R_cᵀ) = −R_d log(R_dᵀ R_c),  (4.12)

which is the world-frame orientation error that the commanders use (chapter 06).

### 3.8 Damped least squares

For a task Jacobian J ∈ ℝ^{m×n} and a desired task velocity v,

  q̇ = Jᵀ (J Jᵀ + λ² I)⁻¹ v  minimises ‖J q̇ − v‖² + λ² ‖q̇‖².  (4.13)

## 4. Implementation, equation by equation

All in `franka_experiments/franka_experiments/utils/kinematics.py` unless stated otherwise.

### 4.1 `CBFKinematics.configure_control_points(robot_cfg, distance_cfg)`

This reads `fr3_complete.yaml` `robot.segments` (sorted by `seg_idx`) and `distance.ee_tip_axis` /
`ee_tip_offset`. For a segment with n > 0 control points between `start_link` and `end_link`:
- if `end_link` ≠ `ee_link` (default `fr3_link8`): t_k = (k+1)/(n+1), k = 0…n−1, so the points are
  interior to the segment;
- if `end_link` == `ee_link`: t_k = (k+1)/n. The last point (t = 1) is **replaced** by a rigid offset
  δ e_axis in the frame of `end_link`. With the shipped configuration that is 0.10 m along z of
  `fr3_link8`.

Labels are `"{end_link}#{i}"`, with i counting the samples per end link. A suffix after `.` (an
obstacle rank) is ignored by `label.partition('.')`. An unknown label raises `KeyError`, by design.

**The shipped geometry (Verified, `fr3_complete.yaml`; numerically confirmed):**

| Segment | start → end | n | t values | Labels |
|---|---|---|---|---|
| 0–2 | link0→1, 1→2, 2→3 | 0 | — | (none: no control points on the shoulder) |
| 3 | link3 → link4 | 2 | 1/3, 2/3 | `fr3_link4#0`, `#1` |
| 4 | link4 → link5 | 2 | 1/3, 2/3 | `fr3_link5#0`, `#1` |
| 5 | link5 → link6 | 2 | 1/3, 2/3 | `fr3_link6#0`, `#1` |
| 6 | link6 → link7 | 2 | 1/3, 2/3 | `fr3_link7#0`, `#1` |
| 7 | link7 → link8 | 3 | 1/3, 2/3, tip | `fr3_link8#0`, `#1`, `#2` (= link8 origin + 0.10 m · z₈) |

All segments have radius 0.05 m. There are 11 control points.

### 4.2 Finding: two of the eleven control points are the same point

Segment 5 (`fr3_link5 → fr3_link6`) has length **0 for every q** (§3.1). Its two samples
`fr3_link6#0` and `fr3_link6#1` therefore coincide. Over 200 random configurations they differed by
at most 2·10⁻¹⁶ m, and their Jacobians and J̇ are identical. At the home pose both sit at
(0.2189, 0, 0.6973). Segment 1 is degenerate in the same way but carries no control points.

**Derived and numerically verified (F-2-1).** The configuration spends two of the 11 control points
on one location, the wrist intersection point. Any per-control-point quantity downstream (perception
distances, QP rows, slack terms) is duplicated for that point.

Whether this matters depends on how chapters 05 and 07 treat duplicate rows. A duplicated
inequality does not change the feasible set, but it can double that point's weight in a slack
penalty. It is not called a bug here: it may be deliberate, for example to weight the wrist. Stage 3
should examine the row-level effect.

### 4.3 `control_point_position(label)` and `control_point_jacobian(label, with_jdot)`

| Equation | Code |
|---|---|
| (4.7), interpolated sample | `p0, p1 = data.oMf[f0].translation, data.oMf[f1].translation`; `(1−t)p0 + t p1` |
| Rigid tip p = R₈ (δ e_z) + p₈ | `data.oMf[f1].act(offset)` |
| (4.8) J | `getFrameJacobian(f0, LWA)[:3]`, `getFrameJacobian(f1, LWA)[:3]`, then `(1−t)J0 + t J1` |
| (4.8) J̇ | `getFrameJacobianTimeVariation(f0/f1, LWA)[:3]`, interpolated the same way |
| (4.4) rigid tip J_P | `point_jacobian_pos` / `point_jacobian`: `J6[:3] − skew(r) @ J6[3:]` |
| (4.5) rigid tip J̇_P | `point_jacobian`: `Jvd − skew(r) @ Jwd − skew(cross(ω, r)) @ Jw`, with ω = `getFrameVelocity(f, LWA).angular` |

**Order of operations.** `update(q, qdot, with_jdot)` runs, in this order:
1. `forwardKinematics(q, qdot)`, which fills `data.v`, needed by `getFrameVelocity`;
2. `computeJointJacobians(q)`;
3. if `with_jdot`, `computeJointJacobiansTimeVariation(q, qdot)`;
4. `updateFramePlacements`.

`control_point_*` must be called after `update`. The `with_jdot=False` path returns (J, None) for
callers that only need velocities.

**Numerical verification (25 random states; `check_kinematics_dynamics.py` [B]).** q is uniform
within the joint limits, q̇ ~ N(0, 1) rad/s and q̈ ~ N(0, 3) rad/s²:

| Check | Max abs error | Verdict |
|---|---|---|
| `control_point_position` vs `distance_utils.define_control_points` (perception's own construction) | 1.1·10⁻¹⁶ m | identical geometry |
| J vs central difference of perception's positions in q | 1.6·10⁻¹⁰ | correct |
| J̇ vs central difference of J along q̇ | 2.3·10⁻¹⁰ | correct |
| p̈ = J q̈ + J̇ q̇ vs second difference of p along q(t) = q + q̇t + ½q̈t² | 1.9·10⁻⁷ m/s² (the size of the finite-difference error at h = 10⁻⁴) | correct |
| Rigid random offset on `fr3_link5`: J_P | 1.2·10⁻¹⁰ | correct |
| Rigid random offset: J̇_P | 1.8·10⁻¹⁰ | correct |
| LWA 6-D J̇ = d/dt J_LWA | 2.9·10⁻¹⁰ | Pinocchio convention as assumed |

Conclusion: the implemented Jacobians and their time derivatives are mathematically correct for the
points they define, and those points are the same ones perception measures from. The repository
pins this with tests and a script:
- `test/test_control_point_derivatives.py` (perception finite differences, J̇ as a directional
  derivative, the rotating rigid offset);
- `test/test_kinematics_fix_regression.py` (rigid acceleration against Pinocchio and finite
  differences, nonconsecutive interpolation, a single rigid tip);
- the offline script `scripts/check_control_derivatives.py`, which repeats the comparison on
  recorded joint trajectories with a tolerance of 10⁻⁶.

### 4.4 Perception's mirror of the geometry: `utils/distance_utils.py`

Perception never imports Pinocchio for this. It rebuilds the same points from TF transforms:
- `define_control_points(transforms, robot_cfg, distance_cfg)` uses (4.7) and the tip offset
  `p_ee + ee_tip_offset · R_ee[:, ee_tip_axis]`, producing `seg_idx`, `cp_idx` and `radius` per
  point;
- `define_robot_segments(...)` builds the capsules p0 → p1 and extends the last segment to the tip.
  The end of the last capsule coincides exactly with the tip control point (numerically verified,
  difference 0).

These are **two independent implementations of one definition**. One sorts the segments by
`seg_idx`, the other iterates them in YAML order. They agree today because the YAML is already
sorted. A later edit that reorders `segments` would change perception's `cp_idx` sequence but not
the kinematics labels. Chapter 05 checks whether labels or indices are what joins the two.

### 4.5 Dynamics

The common pattern in `nodes/qddot_to_torque.py::_compute_tau` (L280–305):
1. Build `q_full` and `qdot_full` from the neutral configuration plus the 7 arm values.
2. Call `pin.computeAllTerms(model, data, q_full, qdot_full)`.
3. Compute `M_arm = data.M[arm, arm]` and `Cq̇_arm = (data.nle − data.g)[arm]`.
4. Return `M_arm @ q̈ + Cq̇_arm`, which is (4.10).

The same construction appears in `pentagon_qddot_commander._compute_tau_des` (which keeps gravity:
`M q̈ + nle`, for logging), `pentagon_torque_commander` and `cbf_OSCBF_filter` (L133–160).

**Numerically verified (Pinocchio 4.0.0, model with hand, 10 random states; `check_kinematics_dynamics.py` [C]):**

| Fact | Result |
|---|---|
| `data.C` after `computeAllTerms` | identically 0. The code comments are right: `data.C @ q̇` would drop all Coriolis torque. |
| `nle − g` vs `computeCoriolisMatrix(q, q̇) @ q̇` | 1.5·10⁻¹⁴ |
| `data.M` lower triangle after `computeAllTerms` | **filled**, not zero |
| `M_arm q̈ + (nle − g)_arm` (what the code computes) vs `rnea(q, q̇, q̈) − g` | 1.3·10⁻¹⁴ N·m, with torques up to 13.6 N·m |

Consequence (F-2-3). `pentagon_qddot_commander` mirrors the upper triangle of M into the lower one
before multiplying ("CRBA only fills M's upper triangle"). With Pinocchio 4.0.0 that step is
redundant but harmless. `qddot_to_torque` uses `data.M` without mirroring, which is correct
**because** Pinocchio 4.0.0 fills both triangles. The two nodes therefore rely on different
assumptions about the Pinocchio version.
- Under a Pinocchio whose `computeAllTerms` leaves the lower triangle at zero, `qddot_to_torque`
  would compute M_upper q̈. That is a wrong torque, and no test in the repository asserts the version.
- The `qpsolvers` pin is a separate concern ([PROGRESS.md](PROGRESS.md) §7).
- The behaviour of other Pinocchio versions was not executed here (Unverified).

### 4.6 Other helpers

| Function | Equation | Notes (Verified) |
|---|---|---|
| `compute_ee_fk` | (4.1) | `forwardKinematics` + `updateFramePlacement`; mutates `data` |
| `compute_arm_jacobian` | (4.2) | LWA, selects the arm's `idx_v` columns, 6×7 |
| `transform_ee_to_frame` | p_ref = (⁰T_ref)⁻¹ p, J_ref = R_refᵀ J_v | Writes the velocity in the axes of `ref`. This equals d/dt of p_ref only if `ref` is fixed. The commanders pass a fixed plane or base frame (chapter 06). |
| `so3_log` | (4.11) | Numerically verified against `pin.log3`: 1.4·10⁻¹⁰ over 2,000 random rotations; ≤ 2.6·10⁻⁶ within 10⁻⁵ rad of 0 and π (sign ambiguity at π allowed) |
| `dls_solve` | (4.13) | λ = max(damping + boost, 0.1·clip((cond(J) − 50)/250, 0, 1)), a smooth ramp in the condition number. Falls back to `pinv`; returns None on non-finite output. Used by the velocity commanders and the hand-eye nodes. |
| `JointStateManager` | — | Topic auto-detection at 1 Hz, from candidates `/{ns}/franka/joint_states`, `/{ns}/joint_states`, `/franka/joint_states`, `/joint_states`; falls back to `/joint_states` after 2 s; name-mapped and cached (§2.2) |
| `resolve_frame_id` (module function) | — | Exact name or `RuntimeError` |
| `CBFKinematics.resolve_frame_id` | — | Exact name, **else the first frame whose name contains the string**. A typo can therefore silently select a different frame (for example `"link1"` matches `fr3_link1`, but `"fr3_link"` matches `fr3_link0`). Inferred risk; the callers are in chapter 07/08. |
| `CBFKinematics.prepare_frame_cache` | — | Empty body (`pass`), kept for interface compatibility |
| `utils/cbf_kinematics.py` | — | A deprecated shim that re-exports `CBFKinematics`; marked `TODO[LEGACY]` (2026-09-01) |
| `math_utils.py` | min-jerk s(τ) = 10τ³ − 15τ⁴ + 6τ⁵ and ṡ; cosine ramp; `lpf`; `rate_limit`; `skew` | Used by the commanders and filters (chapters 06 and 08) |

## 5. Integration

- **Who calls `CBFKinematics.update` and at what rate.** `cbf_safety_filter` calls it in its
  constraint rebuild, nominally at 50 Hz ([02](02_ros_graph_and_dataflow.md) §6), with q and q̇ from
  `joint_states_fast` (`/NS_1/franka/joint_states`, about 1 kHz). It also runs in the ISO nodes, the
  logger and the simulation `scenario_laws`.
- **What flows downstream.** For each control point the filter uses (p, J, J̇ q̇) together with the
  obstacle data from `MultiLinkDistance` (chapter 05). It forms ḣ = n̂ᵀ(J q̇ − v_obs) and the
  second-order condition in q̈ (chapter 07). `qddot_to_torque` applies (4.10) at the event rate of
  `qddot_safe` (chapter 09).
- **Failure behaviour.** An unknown label raises `KeyError` and does not substitute another point.
  A missing frame raises at configuration time. Joint-state messages without all seven arm joints
  are skipped by every consumer.

## 6. Practical example — the fingertip at home

Every number below is recomputed by `check_home_example.py`.

At q_home with q̇ = (0.3, 0, 0, 0.3, 0, 0, 0) rad/s (joints 1 and 4 moving), the hand-less model
gives:

- tip control point `fr3_link8#2`: p = (0.3069, 0, 0.4903) m. That is the flange (0.3069, 0, 0.5903)
  plus 0.10 m along z₈ = (0, 0, −1) at this pose.
- its Jacobian:

  ```
  J = [ 0      0.1573  0      0.1245  0      0.207   0 ]
      [ 0.3069 0       0.3282 0       0.207  0       0 ]
      [ 0     −0.3069  0      0.472   0      0.088   0 ]
  ```

  Column 1 is a₁ × (p − o₁) = ẑ × (0.3069, 0, 0.4903 − 0) = (0, 0.3069, 0), as (4.3) predicts. Column
  7 is zero because joint 7 spins the tip about its own axis.
- the velocity: v = J q̇ = (0.0373, 0.0921, 0.1416) m/s.
- the drift: J̇ q̇ = (−0.0701, 0.0224, 0.0112) m/s². This is the centripetal acceleration of the tip,
  about 0.075 m/s², present even with q̈ = 0.

In a barrier with k₁ = 10.5 and k₀ = 25 (chapter 07), leaving out a drift of this size shifts the
admissible ḧ by the same 0.075 m/s². That corresponds to roughly 3 mm of the k₀ h term
(0.075 / 25 ≈ 0.003 m). The error is small here, but it grows with q̇²: at 1.5 rad/s it would be about
25 times larger.

## 7. How the numerical checks were run (reproducible, read-only)

- **Scripts.** They are preserved in the repository under
  [tools/stage2_verification/](tools/stage2_verification/README.md). The README there gives, for each
  script, the equations it tests, the environment, the expected and observed values, and what remains
  unverified.

  | Script | Covers in this chapter |
  |---|---|
  | `check_kinematics_dynamics.py` | §2 (models, base frame, masses, hand frames), §4.3 table (FK, J, J̇, (4.6), (4.4), (4.5), LWA J̇), §4.5 (dynamics) |
  | `check_geometry_and_so3.py` | §3.1 (home origins, distances between origins), §4.2 (F-2-1), §4.6 (`so3_log`) |
  | `check_home_example.py` | §6 |

  Stage 2 first ran them from the session scratchpad (`check_kin.py`, `check2.py`, `home.py`). The
  preserved versions keep the same seeds and draw order, so the numbers reproduce exactly.
- **How to run.** `docs/framework_study/tools/stage2_verification/run_all.sh` from the repository root.
  It runs these scripts inside the `franka_ros2` container, with the ROS setup sourced and the
  repository mounted at `/ros2_ws/src`.
- **Where the code is imported from.** The scripts import `franka_experiments` from the **repository
  source** and refuse to run against another copy. The container's **installed** copy is stale: its
  `kinematics.py` (dated 2026-11-10 14:39) has no `configure_control_points` (F-2-2, re-checked on
  2026-11-11: still stale).
- **Side effects.**
  - `xacro` writes temporary URDF files to the container's `/tmp`.
  - The preserved scripts disable bytecode caching, so they write nothing into the repository.
  - The original Stage 2 run did not disable it. Python therefore refreshed five git-ignored
    `__pycache__/*.cpython-310.pyc` files under `franka_experiments/franka_experiments/utils/` at
    12:28 on 2026-11-11. These are caches, not source, and were left in place.
- **Repository tests.** The essential comparisons also exist as repository tests (named in §4.3).

## 8. Consequences of modification

| Change | What it affects |
|---|---|
| `fr3_complete.yaml robot.segments` (control-point counts, `seg_idx` order, links) | `CBFKinematics` labels, perception's `define_control_points` order, the number and labels of `MultiLinkDistance` entries, every per-control-point QP row, `iso_*` nodes, `scenario_laws`, and the tests in §4.3. Both builders must stay in step (§4.4). |
| `ee_tip_offset` / `ee_tip_axis` | The tip control point and the end of the last capsule, in both the perception and the kinematics copies. Self-masking (chapter 05) is separate. |
| Replacing the hand-less model with the hand model in `CBFKinematics` | Frame ids shift but labels resolve by name. The hand frames would become available. Mass is irrelevant to kinematics. |
| Pinocchio upgrade or downgrade | The fill pattern of `data.M` (F-2-3), the `data.C` behaviour, and the LWA J̇ convention (§3.4). Re-run the §7 checks. |
| Joint-state topic or publisher changes | The cached-map consumers (§2.2, F-2-5) |
| `world → base` becoming non-identity | The assumption that the world frame equals `fr3_link0`, shared by Pinocchio and perception (§2.1) |

## 9. Findings from this chapter

Logged in [PROGRESS.md](PROGRESS.md) §4:
- **F-2-1:** duplicate control points `fr3_link6#0` and `#1`, from the zero-length segment
  link5→link6.
- **F-2-2:** the container's installed `franka_experiments` is stale relative to the baseline
  source.
- **F-2-3:** `data.M` fill assumptions differ between nodes and depend on the Pinocchio version.
- **F-2-5:** cached joint-state index maps.
- **F-2-6:** `CBFKinematics.resolve_frame_id` falls back to matching a substring of the frame name.
