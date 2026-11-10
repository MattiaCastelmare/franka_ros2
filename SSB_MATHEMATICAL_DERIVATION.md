# The Self-Shadow Barrier: derivation, and the corrections the proposal needed

**Status.** Mechanism implemented and unit/numerically verified; **off by
default** (`distance.ssb_enabled`, `cbf.ssb_rows_enabled`). Simulation results
are reported separately in `SSB_VALIDATION_REPORT.md`; nothing in this document
is an experimental claim.

---

## 1. The problem, stated precisely

`real_time_distance` deletes the robot's own depth pixels before it looks for
obstacles. The deletion has two parts (`franka_experiments/config/fr3_complete.yaml`,
`mask:`):

| step | parameter | shipped value |
|---|---|---|
| dilate the rendered silhouette | `robot_mask_dilate_px` | 2 px |
| …more, for the end-effector | `ee_mask_dilate_px` | 18 px |
| extra halo for the obstacle search | `search_exclusion_extra_px` | 8 px |
| mask/z-buffer grid | `mask_downsample` | 4 |
| per-pixel depth gate, systematic offset | `depth_gate_bias_m` | 0.06 m |
| per-pixel depth gate, tolerance | `depth_gate_tol_m` | 0.10 m |

A masked pixel survives as an obstacle only if its measurement is **in front
of** the gate:

```
surf = z_model − bias                       (the debiased model surface)
in_front  ⇔  z_meas < surf − tol            → kept as an obstacle
```

Everything else inside the mask is dropped. Downstream, `distance_engine`
computes gaps from the pixels that remain, so a dropped pixel is not
"uncertain" — it contributes nothing, which is numerically identical to free
space. **That is the defect:** missing evidence reads as evidence of absence.

---

## 2. Three corrections to the proposed geometry

The task proposed a spherical-shell wedge

```
U_i(q) = { x : ∠(x − c, p_i − c) ≤ θ_i ,  | ‖x − c‖ − D_i | ≤ tol }
```

Each of its three clauses is wrong for this pipeline, in a way that matters.

### 2.1 The band is a z-slab, not a spherical shell

The gate compares `z_meas` against `z_model`, and both are **optical-axis
depth** — `p_cam[:, 2]`, not Euclidean range `‖p_cam‖`. The deleted set is
therefore a slab of constant `z`, not a shell of constant radius. The two
differ by `1/cos θ`: at the edge of the D455's 87° horizontal FOV that is a
factor of ≈ 1.44, i.e. a 16 cm band modelled as a shell would be placed up to
7 cm wrong. Two rays with the **same Euclidean range** but different angles
fall in **different classes**.

> Pinned by `test_self_shadow_geometry.py::test_the_band_is_a_z_slab_not_a_spherical_shell`,
> which constructs exactly that pair at 12° off-axis.

### 2.2 The band is one-sided, not `|·| ≤ tol`

A return **behind** the debiased surface (`z > surf`) is not unknown: the ray
went *through* where the model says the robot is, so either the model is wrong
there or the robot is not there. Either way the ray carries information, and
treating it as unknown would manufacture obstacles out of calibration error.
The implementation gives that case its own class (`RAY_THROUGH`) and never
turns it into a barrier.

### 2.3 The silhouette is a dilated mesh projection, not a per-control-point cone

The proposal's `θ_i` is a constant half-angle per control point. The real mask
is the **rendered mesh silhouette**, dilated by a constant number of **pixels**.
A constant pixel margin is not a constant angle: for a pinhole camera the
angular width of `m` pixels at radius `r` from the principal point is
`≈ m·f/(f² + r²)`, so the halo narrows towards the image edge. The
implementation reads the actual `search_exclusion_mask` the pipeline built,
rather than reconstructing a cone.

**Consequence of all three:** `self_shadow.py` works on the pipeline's own
boolean mask and its own `z_model` z-buffer, and classifies each ray. Nothing
is reconstructed from a parametric region.

---

## 3. Classification: five cases, exhaustive

`classify_masked_rays(z_meas, z_model, excluded, bias, tol, guard, z_min, z_max)`:

```
have = finite(z_meas) ∧ z_min ≤ z_meas ≤ z_max ∧ z_meas > 0
surf = z_model − bias
gate = surf − tol
```

| class | condition | reading |
|---|---|---|
| `RAY_KEPT` | ¬excluded, or (excluded ∧ have ∧ `z_meas < gate`) | an ordinary measurement; the existing pipeline owns it |
| `RAY_VOID` | excluded ∧ ¬have | **no return at all** — unknown, and the only class where "nothing" is the evidence |
| `RAY_CANDIDATE` | excluded ∧ have ∧ `z_meas < surf − guard` ∧ ¬kept | a return the self-filter deleted that is in FRONT of the robot's surface |
| `RAY_THROUGH` | excluded ∧ have ∧ `z_meas > surf + guard` | the ray passed through the model: §2.2 |
| `RAY_SELF` | excluded ∧ have, otherwise | the robot itself, within the gate |

The partition is exhaustive **by construction**: `RAY_SELF` is the default of
the masked-and-measured set and the two tails overwrite it. (An earlier form
with three independent comparisons left a floating-point hole at
`|z − surf| = guard`, where a ray fell into no class at all.)

This is what satisfies §4.2 of the task: observed-occupied (`KEPT`,
`CANDIDATE`), observed-free (`THROUGH`), and unknown (`VOID`) are distinct, and
**a missing measurement is never read as free space** — it becomes `VOID`,
counted in `unknown_fraction()`, and is reported rather than silently dropped.

### 3.1 Why a candidate is not just "the arm with a stale z_model"

`depth_gate_bias_m` is a **median** residual, so with `guard = 0` roughly half
the arm's own pixels sit in front of `surf` and would be called candidates.
Three independent filters stand between a candidate and a barrier row:

1. **the guard band** (`ssb_guard_m`, 0.04 m ≈ 2σ of the measured residual) —
   demonstrated numerically in
   `test_ssb_perception.py::test_the_guard_band_is_what_keeps_the_arm_out`:
   with `guard = 0`, >40 % of a jittered arm's pixels are candidates; with
   `guard = 2σ`, <5 % and <1/10 of the `guard = 0` count;
2. **a 3-D model test** (`ssb_model_clearance_m`): the unprojected candidate
   must be further than that from every sampled mesh point (`cKDTree`,
   `distance_upper_bound`). Its error is independent of the z-buffer's
   sampling bias, which is what makes it a second opinion rather than the same
   test twice;
3. **cluster support** (`ssb_min_support_px`): a connected blob, so single-pixel
   speckle cannot build a constraint.

---

## 4. The barrier

### 4.1 `dist(p, U) = 0` has no escape direction — so the barrier is signed

The proposal's `h = dist(p_j, U_i) − d_safe` is identically 0 for
`p_j ∈ U_i`, and its subdifferential there **contains 0**: the gradient gives
no direction out. The fix is structural, not a smoothing trick. A shadow is
reduced to a **point** `x` (the candidate's unprojected position, or a covering
ball's centre), and the barrier is the signed sphere-inflated distance

```
h(q) = ‖p_j(q) − x‖ − ρ − r_j − d_safe_ssb                              (1)
n̂    = (p_j − x) / ‖p_j − x‖ ,          L = ‖p_j − x‖
```

`h` is smooth and `∇h ≠ 0` **everywhere except exact coincidence** `L = 0`,
including strictly inside the region, where `h < 0` and `n̂` still points the
short way out. At `L = 0` the function is non-differentiable and
`shadow_row(...)` returns `None` rather than emitting a row with an arbitrary
normal.

> `test_self_shadow_geometry.py::test_the_signed_barrier_still_points_outward_from_inside`
> and `test_self_shadow_derivatives.py::test_exact_coincidence_is_refused`.

### 4.2 Acceleration-level HOCBF

With `p_j(q)` the control point and `x` possibly moving:

```
ḣ  = n̂ᵀ(J_j q̇ − J_x q̇)                                                 (2)
ḧ  = n̂ᵀ(J_j − J_x) q̈  +  n̂ᵀ(J̇_j − J̇_x) q̇  +  ‖(I − n̂n̂ᵀ)v_rel‖² / L     (3)
       └──── a ────┘      └────── ċ ──────┘    └────── κ ≥ 0 ──────┘
```

so `ḧ + k₁ḣ + k₀h ≥ 0` becomes the ordinary row the rest of the QP speaks:

```
aᵀ q̈  ≥  −k₁ ḣ − k₀ h − ċ − κ                                           (4)
```

κ ≥ 0 **relaxes** (4), so dropping it is conservative. The SSB rows drop it on
purpose: κ is the free relief that tangential motion produces, and claiming it
on the family whose evidence is weakest — against a surface whose *identity* is
the thing in doubt — would be buying a relaxation with the least reliable
measurement in the problem. `shadow_row` still computes κ so it can be logged.

### 4.3 §6.1: the region moves, and in two different ways

This is where the two regimes differ, and the distinction is not cosmetic.

| regime | what it is | `J_x` | why |
|---|---|---|---|
| **candidate** (`RAY_CANDIDATE`) | a depth **measurement** | `0` | a measurement is a statement about the world. A surface was observed at that place at that time; it does not move because the arm moved. |
| **void** (`RAY_VOID`) | a slab defined by **where the arm is** | the owning link's point Jacobian | nothing was observed. The unknown region is a property of the robot's own pose, so it is carried rigidly with it. |

The difference between the naive (frozen) and the correct (attached) row is
**exactly `n̂ᵀJ_x`** — asserted, not asserted-about, in
`test_self_shadow_derivatives.py::test_the_frozen_and_attached_rows_differ_by_exactly_n_hat_J_x`,
which also checks that the attached one matches the true `ḣ`.

Both regimes are the *same* code path: `shadow_row(..., J_x=None)` for frozen,
`J_x=<link Jacobian>` for attached.

**Only the candidate regime is currently wired into the QP.** The void regime is
implemented and tested (`void_spheres_from_rays`, a deterministic median-split
outer-ball covering) but builds no row, because a `VOID` slab is unbounded in
depth and reducing it to covering balls introduces a modelling choice that has
not been validated. `VOID` rays are counted and reported
(`shadow_unknown_frac`) and nothing more. That is a stated limitation, not an
oversight.

### 4.4 Non-smoothness between frames

The mask is rebuilt per depth frame, so the candidate set changes **discretely**
and `h` is not globally smooth in time. Smooth HOCBF theory is therefore not
applied silently. What is done instead:

* each row is built from the measurement of **one** frame, and the HOCBF
  condition is imposed on that frame's smooth barrier (`a`, `ḣ`, `h` are all
  consistent within a rebuild);
* a row that loses its measurement is **held** for `ssb_hold_s` at the frozen
  point, with the surface **eroded** by `ssb_inflate_speed_mps × age`, so the
  hold is monotonically tightening — holding longer is strictly more
  conservative and never a stale certainty;
* engage/release hysteresis (`ssb_row_horizon`, `ssb_row_release`) keeps the
  row set from churning, which also preserves OSQP's warm start;
* a clock that moves **backwards** releases the hold rather than carrying a
  negative age, which would *un-erode* the row.

### 4.5 §5: the `i ≠ j` exclusion is not sufficient — and why

The task asked whether excluding `i = j` prevents artificial self-collision
constraints. **It does not.** The measured failure:

control points are **interpolations of frame origins**, and a revolute joint's
child origin is **rigid in its parent frame**. So for a region owned by a
segment's own start link, `dh/dq ≡ 0` — not approximately, exactly
(`‖a‖ ≈ 7 × 10⁻¹⁷`). Such a row cannot be satisfied by any `q̈` and can only be
paid for in slack.

Adjacency exclusion is therefore **mandatory**, and in this implementation it
is enforced by the leverage gate `cbf_min_leverage` applied to `‖a‖`, which
covers the exact case and the near-degenerate ones together.

> `test_self_shadow_derivatives.py::test_a_region_owned_by_a_segments_own_start_link_has_no_leverage`
> checks every control point.

---

## 5. Numerical verification of the derivatives (§6.2)

`shadow_row`'s analytic `a`, `ḣ`, and the **complete predicted barrier
acceleration** `aᵀq̈ + ċ + κ` were compared against independent central
differences on random valid FR3 configurations and velocities (Pinocchio model,
`nq = nv = 7`), 40 samples per cell, at perturbation sizes
`ε ∈ {10⁻⁴, 10⁻⁵, 10⁻⁶}`. Table reproduced by `log/ssb_work/ssb_deriv_table.py`:

| regime | ε | ∇h max abs | ∇h max rel | ḣ max abs | ḣ max rel | full ḧ max abs | full ḧ max rel |
|---|---|---|---|---|---|---|---|
| frozen (`J_x = 0`) | 1e-5 | 1.105e-9 | 2.800e-9 | 1.701e-9 | 6.292e-9 | 1.798e-6 | 1.681e-6 |
| attached, non-adjacent owner | 1e-5 | 1.199e-11 | 1.513e-10 | 2.472e-11 | 1.776e-9 | 1.115e-7 | 5.043e-6 |
| frozen, near the surface (L ≈ 1–4 mm) | 1e-6 | 1.663e-8 | 4.213e-8 | 3.039e-8 | 6.205e-8 | 9.350e-3 | 7.002e-5 |

The third row is the honest one: at `L` of a few millimetres the curvature term
`κ = ‖v_⊥‖²/L` blows up and the second difference of `h` is itself badly
conditioned, so the absolute error on `ḧ` is large while the **relative** error
stays at 7 × 10⁻⁵. This is a property of the barrier near coincidence, not a
coding error, and it is the reason `shadow_row` refuses `L ≤ 10⁻⁹` outright.

---

## 6. What this is **not**

* **Not a formal safety guarantee.** (4) is a valid QP constraint. With a soft
  slack it is a *preference*, and even hard it guarantees forward invariance
  only under assumptions this system does not meet exactly: exact `h`, exact
  `ḣ`, no actuation limit binding, continuous time, and a correctly identified
  surface. A satisfied row is not a proof of collision avoidance.
* **Not a fix for the blind band.** At the shipped calibration the deleted band
  corresponds to surface gaps of `(bias + guard − 2r, bias + tol − 2r)` — for a
  4 cm-radius obstacle, `(0.02, 0.08)` m, which is **entirely inside**
  `d_safe = 0.10 m`. The self-filter only ever deletes an obstacle that is
  *already* inside the safety distance, so SSB cannot prevent the `d_safe`
  violation that deletion causes. It can only bound how much closer the
  obstacle gets while the ordinary pipeline is blind. This is derived in
  `franka_simulation_experiments/config/scenarios.yaml` (`ssb_poke`) and is the
  single most important limit on what the mechanism can claim.
* **Not a reason to loosen `depth_gate_tol_m`.** Lowering the tolerance would
  feed the same uncertain pixels into the **obstacle** family, where they share
  `d_safe`, `rho_slack` and the zone ladder with trusted measurements. SSB is a
  separate family precisely so that a false positive costs *conservatism* rather
  than crushing the obstacle rows' slack budget (cf. the 58-slack / 23-sample
  phantom failure recorded in `OPEN_ISSUES.md`).
