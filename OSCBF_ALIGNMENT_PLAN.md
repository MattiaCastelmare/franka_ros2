# OSCBF/cbfpy alignment plan — implementation brief for Claude Code

*Written 2026-11-04 for branch `humble-mattia` (working tree with uncommitted changes).
Audience: a Claude Code session that will implement these changes in `franka_experiments`.*

This brief is a work order. It says **what to change, where, with which equations, how to test, and when to stop**.
The analysis behind it is in `CBF_QP_SIMPLIFICATION_ANALYSIS.md` (repo root); the experiment status is in
`franka_simulation_experiments/next_step.md`. Read both before starting. Do not re-derive their conclusions.

**Path convention.** Unless a path names another root, it starts at `franka_experiments/franka_experiments/`.
Line numbers are approximate (the files are being edited); search for the quoted function or symbol name.

---

## 0. Goal and ground rules

### Goal

Make the live safety filter (`nodes/cbf_safety_filter.py`) solve the **same QP that `oscbf` solves in torque mode**,
keeping the things where our framework is already better than the references. Then make it fast enough that the
achieved QP rate is a constant, not a variable.

Target formulation (the "B" arm in the analysis), over $x=[\ddot q;\ s]$:

$$
\begin{aligned}
\min_{\ddot q,\ s\ge 0}\ & \tfrac12\,\Delta^\top\big(J^\top W_t J + N^\top W_j N + \epsilon I\big)\Delta
\;+\;\rho_{obs}\mathbf 1^\top s_{obs} + \rho_{sc}\mathbf 1^\top s_{sc} + \rho_{ql}\mathbf 1^\top s_{ql},
\qquad \Delta = \ddot q - \ddot q_{nom}\\
\text{s.t. }\ & a_i^\top \ddot q + s_i \;\ge\; -k_1\dot h_i - k_0 h_i - \hat n_i^\top \dot J_{p,i}\dot q - \kappa_i
\qquad\text{(one row per obstacle / self-collision / joint-limit barrier, own slack)}\\
& h_i = d_i - (d_{safe}+\delta_{perc}),\quad \dot h_i = a_i^\top\dot q - \hat n_i^\top \hat v_{o,i},\quad
\kappa_i = \frac{\|(I-\hat n_i\hat n_i^\top)\,v_{rel,i}\|^2}{\rho_{c,i}} \\
& lb(q,\dot q) \le \ddot q \le ub(q,\dot q)\qquad\text{(hard box, unchanged)}
\end{aligned}
$$

with $N = I - \bar J J$, $\bar J = M^{-1}J^\top(JM^{-1}J^\top)^{-1}$, one gain pair $(k_0,k_1)=(\omega^2, 2\omega)$, and
on QP failure **brake**, never pass the nominal through.

### What to copy from the references, and what not to

| Copy (from) | Why |
|---|---|
| Task/null-space objective metric (`oscbf/oscbf/core/oscbf_configs.py`, `OSCBFTorqueConfig._P`, ~L104-125) | Corrections go to redundant joint motion first, so the end effector stays on its path |
| Per-row ℓ1 elastic slack (`cbfpy/cbfpy/cbfs/cbf.py` `safety_filter`, ~L160-200; penalties in `cbfpy/cbfpy/config/cbf_config.py`, CBF 1e3 < limits 1e5) | Exact penalty: zero slack whenever the row is feasible; one bad row cannot loosen the others |
| Complete $\ddot h$ (cbfpy gets it by nested `jax.jvp`, `cbf.py` `h()`, ~L203-221) | The missing curvature term over-constrains tangential motion, i.e. exactly the "go around" motion |
| One gain pair, no scheduling (oscbf examples: `alpha = alpha_2 = 10 h`, double pole at −10) | One barrier, one gain pair; tunable by a single ω |
| A thin filter: no biases, no task fades, no margins inside $h$ | Every term in the QP explainable |

| Do **not** copy | Why |
|---|---|
| Soft actuator box (`u_min/u_max` relaxed at 1e5) | Our box is hard and state-dependent (velocity/position braking). Keep it hard |
| Ignoring the solver status (`cbf.py` returns `x_qp[:m]` unconditionally) | We check status and fall back. Keep that |
| All pairs always active (oscbf: 21 spheres × N obstacles) | We cull by distance; keep culling |
| Their moving-obstacle handling (`oscbf/examples/dynamic_obstacle.py`, a self-described hack ignoring $\partial h/\partial t$) | Ours puts signed $\hat n^\top \hat v_o$ in $\dot h$, which is more correct |
| Rewriting the filter in JAX/qpax | Not needed for the formulation. Only consider it in Phase 4 if the rate cannot be held |

### Ground rules (mandatory)

1. **Every new behaviour sits behind a parameter whose default reproduces today's behaviour bit-for-bit.** The repo
   already follows this pattern (see `obstacle_slack_mode: family|row_l1` in `config/fr3_control.yaml` and
   `build_qp_objective` in `utils/cbf_qp_assembly.py`). Exception: Task 1 (safe fallback) is a safety fix and becomes the
   default; say so in the summary.
2. Every new parameter goes in **three places**: `config/fr3_control.yaml` (documented, with units), the spec
   `CBF_PARAM_SPEC` in `utils/config.py` (type and bounds), and — if the simulation harness must toggle it — a
   condition in `franka_simulation_experiments/config/conditions.yaml`. `cbf_param_overrides` in
   `launch/torque_control_stack.launch.py` already lets a condition set any `CBF_PARAM_SPEC` key.
3. **One task at a time, in the order below.** After each task: run the tests, show the diff summary, and stop for
   the user's review. Do not commit unless the user asks.
4. **Tests run inside the Docker container `franka_ros2`** (the host has no `pinocchio`). Check that it is running
   (`docker ps`); if not, ask the user to start it. Typical command:
   ```bash
   docker exec franka_ros2 bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash \
     && cd /ros2_ws/src/franka_experiments && python3 -m pytest -q -p no:cacheprovider test/'
   ```
   Verify first that `/ros2_ws/src` is a bind mount of this repo; if it is not, rebuild with
   `colcon build --packages-select franka_experiments` before testing.
5. **Get a baseline first.** Run the whole `franka_experiments/test/` and `franka_simulation_experiments/test/` suites
   before touching anything and record which tests already fail. Several untracked tests were written ahead of their
   implementation (see §1), so some failures are expected and are not yours.
6. Do not delete the untracked, unwired modules `utils/static_scene.py` and `utils/surface_velocity.py`. They are
   perception-side work in progress and out of scope here, except where Task 2 uses the static/dynamic distinction.
7. Do not touch the `full` condition's mechanisms (zones, margins, biases, governor, caps). The new arm is built by
   **turning them off in a condition**, not by deleting code. Deletion is a later, separate decision.

---

## 1. Current state (verified 2026-11-04)

| Item | State |
|---|---|
| Baseline arm | `minimal` condition (`conditions.yaml` ~L121): plain HOCBF, identity metric, shared quadratic family slacks, `k0=25`, `k1=10.5` |
| Per-row ℓ1 obstacle slack | **Done**, uncommitted. `obstacle_slack_mode: row_l1`, `rho_slack_l1: 1000`, `obstacle_row_slack_eps: 1000`, `obstacle_row_osqp_eps_rel: 1e-5`, `obstacle_row_slack_slots: 32`. Code: `build_qp_objective` (`utils/cbf_qp_assembly.py`), `row_slack_slots` and slot assignment in `ConstraintBuilder.build` (`utils/cbf_state_rows.py` ~L2770), node `__init__` and `_solve`. Condition `minimal_l1`. Test `test/test_cbf_row_l1_slack.py` |
| Safe fallback | **Specified by a test, not implemented.** `test/test_safety_failure_policy.py` expects a failed QP to go straight to braking (level 2) using `_state_lb/_state_ub`, which do not exist in the node yet. `_solve` still uses `box_only_solve` as level 1 (`nodes/cbf_safety_filter.py` ~L1500) |
| Status schema | `utils/cbf_diagnostics.py` (`STATUS_FIELDS`, `family_violations`) exists but is not imported by the node |
| Curvature term, OSCBF metric, constant margin, gain sweep, single-rate geometry | Not done |
| Rate | Python + GIL; `full` achieves 30–91 Hz at a 100 Hz target, `minimal` ≈100 Hz (`sim_runs/exp_minimal_vs_full_20261102/COST_PROFILE.md`) |
| `cbf_OSCBF_filter.py` | An older, torque-level OSCBF port, flagged legacy, not launched. **Do not revive it.** Use it only as a reference for the $\Lambda$, $\bar J$, $N$ algebra (its docstring and `_P` build) |

Known perception problems (`next_step.md` step 0, `OPEN_ISSUES.md` S3/S5/F4/F5) are **out of scope** but dominate
the safety outcomes; the validation in §4 controls for them with ground-truth obstacles.

---

## 2. Tasks

Each task lists: goal · reference · math · where · parameters · tests · done-when.

### Task 1 — Safe fallback: brake on QP failure, never drop the barrier rows

**Goal.** Today, when the full QP fails, level 1 (`box_only_solve`) solves the same objective over the box alone, i.e.
it passes the nominal through, clipped, at the exact moment the safety QP failed. Replace that with braking.

**Reference.** Neither reference has a fallback (they ignore the status). This is ours; it makes the design strictly
safer than theirs.

**Spec** — the test `test/test_safety_failure_policy.py::test_failed_qp_brakes_even_when_nominal_and_slew_demand_acceleration`
is the contract. Read it fully. In short:
- on any failure (non-solved status, solver exception, non-finite iterate) `_solve` returns
  $\ddot q = \text{clip}(-k_{brake}\,\dot q,\ \text{state\_lb},\ \text{state\_ub})$ at level 2 (`OSQP_LEVEL_BRAKE`);
- the clip uses the **state box** (`_state_lb/_state_ub`: acceleration + velocity + position braking box *before* the
  slew limit), not the slew-tightened box, because the slew limit must never prevent braking;
- `_fallback_count[2]` increments, `_osqp_prob = None`, `_prev_nc = -1`, slack reported as 0;
- solver exceptions from `update()`/`solve()` are caught and treated as failures.

**Where.**
- `nodes/cbf_safety_filter.py` `_solve` (~L1448-1530): remove the level-1 rung; wrap update/solve in try/except.
- STEP 4 of `_qp_tick` (~L1325+): store the box after `position_velocity_accel_box` and before the slew limit as
  `self._state_lb/_state_ub` (the slew step is `_slew_delta` / `apply_slew_limit`).
- `utils/cbf_qp_assembly.py` `braking_command`: reuse it if its semantics match the test; otherwise adapt it.
- Keep `OSQP_LEVEL_BOX` as a constant (status consumers read it) but nothing should emit it any more. Grep
  `franka_simulation_experiments/scripts` for consumers of `fallback_level` and make sure a level that never appears
  breaks nothing.
- Leave the ISO-stop branch in `_qp_tick` (~L955) alone: it already brakes.

**Parameters.** Optional `qp_failure_policy: brake|box_only` (default `brake`) only if the user wants the old path for
A/B; otherwise none.

**Tests.** `test_safety_failure_policy.py` passes in full (it also covers `apply_slew_limit` and the status schema;
wire `cbf_diagnostics.STATUS_FIELDS` / `family_violations` into `_publish_status` only as far as that test requires).

**Done when.** The whole suite is at baseline or better, and no code path can emit the nominal command after a failed QP.

---

### Task 2 — Complete $\ddot h$: add the curvature term $\kappa$

**Goal.** Our row uses $\ddot h \approx a^\top\ddot q + \hat n^\top \dot J_p \dot q$. The true value also contains
$\dot{\hat n}^\top v_{rel} \ge 0$ (and $-\hat n^\top a_o$). Without it the filter brakes or pushes outward while the arm
slides *around* an obstacle. At $v_\perp = 0.3$ m/s and $\rho_c = 0.15$ m it is 0.6 m/s², larger than $k_0 h$ at
$h = 2$ cm. cbfpy/oscbf include it automatically through autodiff.

**Math.** For sphere–sphere geometry (centre distance $\rho_c$):
$$\dot{\hat n} = \frac{(I-\hat n\hat n^\top)\,v_{rel}}{\rho_c},\qquad
\dot{\hat n}^\top v_{rel} = \kappa = \frac{\|(I-\hat n\hat n^\top)v_{rel}\|^2}{\rho_c},\qquad
v_{rel} = J_p\dot q - \hat v_o .$$
HOCBF condition $\ddot h + k_1\dot h + k_0 h \ge 0$ gives the row
$$a^\top\ddot q \;\ge\; -k_1\dot h - k_0 h - \hat n^\top\dot J_p\dot q - \kappa .$$
So $\kappa$ enters **exactly like the existing `jdot_qdot` term ($c$)**: same sign, same place in the RHS.
The obstacle-acceleration term $-\hat n^\top a_o$ stays out (the references omit it too; mention it in a comment).

**Safety of the approximation (important).** $\kappa \ge 0$ *relaxes* the row, so **overestimating it is unsafe**.
$\kappa$ is overestimated when $\rho_c$ is underestimated. Hence:
- $\rho_{c} = d + r_{robot} + r_{obs}$ with $r_{obs}$ an **upper bound** on the obstacle's radius of curvature
  (parameter, default 0.30 m; the simulated ball's radius is known, use it in the sim condition);
- **flat or room-like surfaces have $\rho_c \to \infty$, so $\kappa = 0$.** Apply $\kappa$ only to rows of a confirmed,
  moving track (same gate as `enable_vobs_in_hdot`: `ob.v_vec is not None and frames_seen >= self._min_frames_eff`).
  Untracked rows, static-scene tracks and self-collision/joint-limit rows get $\kappa = 0$;
- clamp: $\kappa \le$ `curvature_kappa_max` (default 2.0 m/s²) and $\kappa \ge 0$; non-finite → 0;
- $r_{robot}$: the capsule/control-point radius from the robot config (`config/fr3_complete.yaml`, control points);
  `LinkDistance.distance` is already the surface gap (see the comments in `franka_msgs/msg/LinkDistance.msg`), so
  $d + r_{robot}$ is the distance from the robot primitive's axis/centre to the obstacle surface.

**Where.**
- `utils/cbf_state_rows.py`, `ConstraintBuilder.build`, obstacle-row loop (~L1930-2420): after `a`, `Jp`, `Jpd`, `n_w`,
  `v_hdot` are known, compute `kappa` per row. `v_rel` must use the **same** $\dot q$ and the **same** obstacle velocity as
  $\dot h$ (the clamped `v_vec` behind `v_hdot`), so the row is internally consistent.
- `ConstraintSnap` (~L1407): add a field `kappa: Optional[np.ndarray]` (default `None` = feature off, bit-identical).
- `utils/cbf_qp_assembly.py` `build_row_rhs` (~L151): add `kappa` next to `jdot_qdot` when present.
- Because $\kappa$ is quadratic in $\dot q$, a 50 Hz snapshot of it goes stale faster than $c$. Start with the value
  computed at the 50 Hz rebuild and note it as a limitation in the code comment; Task 5 moves it to the QP rate.
- Diagnostics: report `kappa_max` in CBFDIAG / status.

**Parameters.** `enable_curvature_term: false`, `curvature_obstacle_radius: 0.30`, `curvature_kappa_max: 2.0`.

**Tests** (new file `test/test_cbf_curvature_term.py`, use `test/_cbf_builder_harness.py` like the static-scene tests):
1. Flag off → snapshot and RHS bit-identical to today.
2. Analytic check: point moving tangentially past a sphere of known radius; $\kappa$ equals
   $v_\perp^2/(d+r_r+r_o)$ to 1e-9, and equals a finite-difference $\dot{\hat n}^\top v_{rel}$ to 1e-3.
3. Purely radial motion → $\kappa = 0$. Untracked obstacle → $\kappa = 0$. Clamp respected.
4. Closed-loop toy (double integrator in 3-D, no ROS): sliding tangentially past a sphere at constant speed, the
   version with $\kappa$ deviates less from the straight path and still keeps $h \ge -10^{-3}$.

**Done when.** Tests pass and a condition `minimal_l1_curv` exists (= `minimal_l1` + the flag).

---

### Task 3 — OSCBF task/null-space objective metric on $\ddot q$

**Goal.** Replace the identity metric $\tfrac12\|\Delta\ddot q\|^2$ with oscbf's metric, so the QP prefers corrections
in the null space of the end-effector task and the hand stays on its path.

**Derivation (why this is exactly oscbf).** oscbf's torque cost is
$\tfrac12\Delta\tau^\top\big(N_T^\top M^{-\top}W_jM^{-1}N_T + M^{-\top}J^\top W_t J M^{-1}\big)\Delta\tau$ with
$N_T = I - J^\top\bar J^\top$. Our command is $\tau = M\ddot q + C\dot q$ (`qddot_to_torque.py`), so
$\Delta\tau = M\Delta\ddot q$, $M^{-1}\Delta\tau = \Delta\ddot q$, and $M^{-1}N_T M = I - \bar J J = N$. Therefore in
$\ddot q$ coordinates:
$$P_m = J^\top W_t J + N^\top W_j N + \epsilon I,\qquad q_m = -P_m\,\ddot q_{nom},$$
i.e. the same matrix as oscbf's *velocity* config, with the **dynamically consistent** $N$. Use the dynamically
consistent $\bar J$, not the kinematic $J^+$ of the commander.

**Details.**
- $J$: 6×7 end-effector Jacobian, `LOCAL_WORLD_ALIGNED`, frame `ee_link` from the robot config (default `fr3_link8`;
  the filter's model is built by `build_urdf_no_hand()` in the node `__init__` ~L202, so check the frame exists).
- $M$: `pin.crba` (symmetrise; pinocchio fills only the upper triangle). $\Lambda^{-1} = JM^{-1}J^\top$; invert with a
  damped solve ($+\lambda^2 I$, $\lambda^2 = 10^{-4}$) so the metric stays defined near singularities.
- $W_t = \mathrm{diag}(w_{pos}^2 I_3,\ w_{rot}^2 I_3)$, $W_j = w_{joint}^2 I_7$; oscbf defaults are all 1.
- $\epsilon$ (default 1e-3) keeps $P_m$ positive definite for OSQP.
- $P$ now changes every tick. OSQP can update its values only with a **fixed sparsity pattern**: build `_P_csc` once
  with a dense upper-triangular 7×7 block for $\ddot q$ plus the slack diagonals (keep the `row_l1` linear prices in
  `q[NX:]`, which the tick never overwrites), then per tick call `update(Px=<triu values>, q=...)`. Also pass the new
  `Px` on the `setup` path when the row count changes. Update `box_only_solve`/warm-up callers consistently.
- The nominal target: today the tick writes `self._qvec[:NV] = -qddot_nom` (~L1325). With the metric it must be
  `self._qvec[:NV] = -(P_m @ qddot_nom)`.
- Slack prices: dual multipliers scale with $P_m$. Verify with a test that `rho_slack_l1 = 1e3` is still exact (slack
  0 on a feasible row next to an infeasible one); if not, raise it and document why.
- Compute $M$ and $J$ once per QP tick from the same `q` used for the box. Cost is ~tens of µs in pinocchio; measure it.

**Where.** `utils/cbf_qp_assembly.py` (new pure function `oscbf_metric(J, M, w_pos, w_rot, w_joint, eps, lam2) -> P_m`,
plus a variant of `build_qp_objective` that returns the fixed pattern and an index map for the $\ddot q$ block);
`nodes/cbf_safety_filter.py` `__init__` (pattern), `_qp_tick` STEP 4 (values), `_solve` (update with `Px`).

**Parameters.** `objective_metric: identity|oscbf` (default `identity`), `metric_w_pos: 1.0`, `metric_w_rot: 1.0`,
`metric_w_joint: 1.0`, `metric_eps: 1.0e-3`, `metric_lambda_sq: 1.0e-4`.

**Tests** (`test/test_cbf_oscbf_metric.py`):
1. `identity` → `_P_csc`/`_qvec` bit-identical to today.
2. `oscbf_metric` equals the oscbf formula computed independently in the test (port the 10 lines of
   `OSCBFTorqueConfig._P` to numpy, map with $M$) to 1e-9 on random SPD $M$ and random full-rank $J$.
3. With no active rows the QP returns $\ddot q_{nom}$ (to OSQP tolerance).
4. One active obstacle row on a redundant 7-DoF configuration: the end-effector acceleration error
   $\|J(\ddot q - \ddot q_{nom})\|$ is smaller with `oscbf` than with `identity`, and the row is satisfied.
5. OSQP `update(Px=...)` with a changing $P_m$ across ticks gives the same solution as a fresh `setup` (1e-6).

**Done when.** Tests pass; condition `minimal_l1_curv_metric` exists.

---

### Task 4 — Per-row ℓ1 slack for self-collision and joint-limit rows; cbfpy penalty ordering

**Goal.** cbfpy orders penalties CBF (1e3) < limits (1e5). Ours is inverted: joint-limit rows have the *lowest*
weight (`rho_slack_joint_limit: 200`, quadratic, shared). Extend the `row_l1` machinery to these families.

**Where.** `row_slack_slots` and the slot assignment in `ConstraintBuilder.build` (`utils/cbf_state_rows.py`
~L1324, ~L2770): generalise from `G_OBS` to a set of families, each with its own slot range and linear price. Keep
the overflow rule (rows beyond the slots fall back to the family slack, counted and logged).

**Parameters.** `self_collision_slack_mode: family|row_l1`, `joint_limit_slack_mode: family|row_l1`,
`rho_slack_l1_self_collision: 1.0e4`, `rho_slack_l1_joint_limit: 1.0e5`, slot counts (8 and 14 are enough:
`SelfCollisionRowBuilder` caps at 8 rows; joint limits are at most 14).

**Tests.** Extend `test/test_cbf_row_l1_slack.py`: layout width, bit-identity when off, exactness per family.

**Done when.** Tests pass; added to the B condition.

---

### Task 5 — Single-rate geometry, and the rate as a controlled variable

**Goal.** Today $A$, $\bar h$, $c$ are rebuilt at 50 Hz (`cbf_update_rate_hz`) while only the $\dot q$ terms refresh at
the QP rate, and the normal comes from capture time while $J$ is from `q_now` (OPEN_ISSUES F5). oscbf computes
everything from one state, in one compiled call. Make the B arm single-rate and fast enough that 100 Hz is held.

**Steps.**
1. **Measure first.** Add timing to `ConstraintBuilder.build` per section (obstacle loop, self-collision, joint limits,
   kinematics) and to the QP tick, using the existing CBFDIAG reporting. Record the profile of `minimal_l1` in the
   `diag` scene (procedure in `next_step.md`). No optimisation without this profile.
2. **Fast path when mechanisms are off.** In the B condition, zones, margins, caps, biases, evasion, governor and the
   $v_o$ conditioning are all off; make sure their code is *skipped*, not computed and multiplied by zero. Guard each
   block on its flag.
3. **Vectorise the obstacle loop**: stack $\hat n_i$, $J_{p,i}$, $\dot J_{p,i}$ for all rows and compute `A`, `c`, `kappa`
   with batched numpy (`einsum`) instead of per-row Python.
4. **Rebuild at the QP rate** (`cbf_update_rate_hz` = QP rate) for the B arm, once 1–3 make it affordable.
   Recompute the robot-side closest point and $\hat n$ at `q_now` (F5): FK of the control point at `q_now`, keep the
   obstacle surface point from perception. Gate this behind `geometry_at_qp_rate: false`.
5. **Rate as a pass/fail input.** The batch scorer must reject runs whose achieved QP rate is outside ±5 % of the
   target (see `franka_simulation_experiments/scripts/simexp/`). Report the achieved rate with every result.

**Optional (only if 1–4 cannot hold 100 Hz with ≥20 % headroom):** move the per-tick numeric kernel (rows + metric +
OSQP call) out of the GIL-bound executor, e.g. a dedicated thread with `osqp`'s C solve releasing the GIL, or a C++
ros2_control-side filter. Ask the user before starting this; it is the "Option C" rewrite in the analysis.

**Done when.** `minimal_l1` and the B arm both hold 100 Hz ±5 % in the `diag` scene for a full `pentagon_lateral` run,
with p99 tick time reported.

---

### Task 6 — One constant perception margin, then the gain sweep

These are **configuration-only** for the B arm.

**6a. Constant margin.** $h = d - (d_{safe} + \delta_{perc})$. Before adding a new knob, check whether the existing
`d_safe` plus a single constant offset already gives this (grep `d_safe` handling in `cbf_state_rows.py`); if so,
reuse it. All dynamic margins (`m_unc`, `m_lat`, `m_std`, approach gate, sensor-range, braking margins) and the
asymmetric EMAs on $h$ stay **off** in the B arm. Size $\delta_{perc}$ from measured perception error: 0.04–0.06 m per
the calibration residual; start at 0.05 and state it in the condition comment.

**6b. Gains.** One pair for every barrier family, $(k_0,k_1) = (\omega^2, 2\omega)$, zone ladder off. Conditions
`B_w5` (25, 10), `B_w7` (49, 14), `B_w10` (100, 20; oscbf's choice). Higher ω reacts later and harder: check in each
run how often the hard box and slew limit bind (`slew_conflict_joints`, box-bite diagnostics).

---

## 3. The B arm, as conditions

Add to `franka_simulation_experiments/config/conditions.yaml`, each one inheriting the previous (follow how
`minimal_l1` is defined from `minimal`; the comment above `minimal_l1` explains the "byte-for-byte except one knob"
convention — keep it):

| condition | adds |
|---|---|
| `minimal_l1` | existing (per-row ℓ1 obstacle slack) |
| `minimal_l1_curv` | Task 2 |
| `minimal_l1_curv_metric` | Task 3 |
| `B` | Task 4 + Task 6a + all `full` mechanisms explicitly off (`enable_go_around: false` etc., via `cbf_param_overrides`) |
| `B_w5`, `B_w7`, `B_w10` | Task 6b |

Task 1 applies to every condition (it is the new default). Task 5's `geometry_at_qp_rate` is added to `B` only after
it is measured.

Note that `config/fr3_control.yaml` has `enable_go_around: true`, so a bare node (not launched through a condition)
runs go-around. Do not change that default in this work; make the `B` condition set it explicitly.

---

## 4. Validation protocol

Applies to every comparison; follow `CBF_QP_SIMPLIFICATION_ANALYSIS.md` §9. Minimum:

1. **Ground-truth obstacles first** (perception bypassed; `next_step.md` "Recommended next step"), then the kf
   tracker. A gain that appears only with the tracker belongs to perception, not to the filter.
2. Scenarios: `pentagon_lateral`, `pentagon_sweep`, `pentagon_crossing`, `pentagon_multi_mixed`, `pentagon_pos_p1`,
   `p2`, `p6`, `p7`. Scene `diag`. 100 Hz, rate-gated (Task 5.5).
3. **≥12 seeds per cell, interleaved seed by seed**, not arm by arm. Report Clopper–Pearson intervals on contact rates.
4. Metrics: min true clearance; time and depth inside $d_{safe}$; contacts; EE deviation from the geometric path;
   lap time; jerk on a 100 Hz grid; slack (fraction of ticks > 0, max, integral) per family; fallback counts per
   level; rows per tick; solve and tick time p50/p99; achieved rate.
5. Order: `minimal` → `minimal_l1` → `+curv` → `+metric` → `B` → `B_w*` → `full`. Accept a step only if safety does not
   get worse and tracking or smoothness gets better, on the rate-matched, ≥12-seed comparison. No decisions on
   single runs or on visuals.

Write each campaign's results under `franka_simulation_experiments/sim_runs/<name>_<YYYYMMDD>/` with a README, like
the existing ones, and update the status table in `franka_simulation_experiments/next_step.md`.

---

## 5. Checklist for the implementing session

- [ ] Read `CBF_QP_SIMPLIFICATION_ANALYSIS.md`, `franka_simulation_experiments/next_step.md`, this file.
- [ ] Container running; baseline test run recorded (list of pre-existing failures).
- [ ] Task 1 — safe fallback (`test_safety_failure_policy.py` green). Stop for review.
- [ ] Task 2 — curvature term + tests + `minimal_l1_curv`. Stop for review.
- [ ] Task 3 — OSCBF metric + tests + `minimal_l1_curv_metric`. Stop for review.
- [ ] Task 4 — per-row ℓ1 for self-collision / joint limits. Stop for review.
- [ ] Task 5 — profile, fast path, vectorise, single-rate, rate gate. Stop for review.
- [ ] Task 6 — `B`, `B_w5/7/10` conditions.
- [ ] §4 campaign (only when the user asks to run simulations).
- [ ] Every new parameter in `fr3_control.yaml` + `CBF_PARAM_SPEC`; defaults bit-identical (except Task 1).
- [ ] Final summary: what changed, which tests were added, which pre-existing failures remain, what was not done.
