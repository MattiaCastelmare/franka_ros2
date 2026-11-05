# CBF/QP Simplification Analysis — FR3 collision avoidance

*Written 2026-11-04. Sources: the working tree on `humble-mattia`, including uncommitted files, plus `~/Git/oscbf` and `~/Git/cbfpy`. No source code was modified.*

**Scope.** The main live filter is `franka_experiments/franka_experiments/nodes/cbf_safety_filter.py`, together with
`utils/cbf_state_rows.py`, `utils/cbf_qp_assembly.py` and `utils/cbf_hard_limits.py`. The two arms compared are the
`full` and `minimal` conditions in `franka_simulation_experiments/config/conditions.yaml:98-170`.
`cbf_OSCBF_filter.py` and `cbf_velocity_filter.py` are left out because no stack launches them (see OPEN_ISSUES F1).

**Path convention.** Paths start at `franka_experiments/franka_experiments/` unless they name another root.

**How claims are tagged.**
- **[V]** — verified in the code.
- **[D]** — supported by data already recorded in `sim_runs/` or `OPEN_ISSUES.md`.
- **[H]** — a hypothesis that still needs a test.

---

## 1. Executive summary

1. **The core is already right.** Both of our arms solve the same problem as `oscbf` in torque mode:
   - a min-norm QP over the joint acceleration, with one linear HOCBF row per (robot point, obstacle) pair:
     $a_i^\top\ddot q \ge -k_1\dot h_i - k_0 h_i - \dot a_i^\top\dot q$;
   - a box on the decision variable underneath.

   That structure needs no replacing.
2. **`full` adds about 20 mechanisms around that core.**
   - Most of them alter $h$, the gains, or the QP target $\ddot q_{ref}$. None of them alters the barrier condition itself.
   - Several of them patch symptoms that other mechanisms create. The retreat-speed cap, for example, limits push-away caused by the margins.
3. **The most likely formulation-level causes of over-reaction are four, all shown mathematically:**
   - (i) Velocity- and covariance-dependent margins are subtracted from $h$. Their time derivative is missing from $\dot h$, and they are multiplied by $k_0 = 25$–$50$. A noisy margin step therefore becomes an instant repulsive-acceleration step. Asymmetric EMAs (rise instantly, decay slowly) turn it into a ratchet.
   - (ii) The task fade / zone hold and the state governor **stop the robot by design**.
   - (iii) The curvature term $\dot{\hat n}^\top v_{rel} \ge 0$ is missing from $\ddot h$. This over-constrains tangential motion, which is exactly the "go around" motion we want.
   - (iv) Four more additive biases on $\ddot q_{ref}$, each with its own trigger and EMA. The oscillation risk is a hypothesis.
4. **The largest causes of actual *safety failures* in the data are not the QP.**
   - Perception failures: a track lost in the self-mask (S5), phantom rig detections (S3), and unmonitored links 0–3 (F4).
   - An uncontrolled QP rate: a GIL-bound Python node runs at 30–120 Hz on identical configurations (S6, COST_PROFILE).
5. **The minimal arm leaks penetration in static scenes**: min_h −1.8 to −7.6 cm, h<0 for up to 9.7 s.
   - The structural reason is the **quadratic slack, shared by a whole family**. A soft row binding against a pushing nominal always gives in, by $s^\*=r/(1+\rho\|a\|^2)$ or by whatever the box forbids.
   - The steady-state penetration is then $h_{ss}=-s/k_0$.
   - `full` hides this leak behind margins. That is conservatism spent compensating a soft constraint.
6. **The references are not safer.**
   - They use static, perfectly known sphere obstacles, a perfect state and an ideal PyBullet actuator.
   - Their ℓ1 slacks and soft control limits never report a violation.
   - They have no fallback. Their dynamic-obstacle support is a hack that ignores $\partial h/\partial t$, which ours handles via $v_o$.
   - What is worth copying: per-row ℓ1 (exact-penalty) slack, the task/null-space objective metric, autodiff-complete $\ddot h$, compiled computation, and a single gain pair with no scheduling.
7. **Recommendation: Option B.** Start from `minimal` and make these changes:
   - per-row ℓ1 slack with a large penalty;
   - a complete $\ddot h$;
   - one constant perception margin instead of the dynamic margins;
   - the OSCBF task/null metric on $\ddot q$;
   - a fallback that never drops obstacle rows;
   - removal of the task fade and all objective biases.

   Fix perception (S3/S5/F4/F5) and the rate first, otherwise no comparison means anything.

---

## 2. Current QP formulation (as implemented)

### 2.1 Shared substrate (identical in `full` and `minimal`)

**Decision variables** [V] (`cbf_state_rows.py:1306-1308`):
- $x = [\ddot q\in\mathbb R^7;\ s\in\mathbb R^6_{\ge0}]$.
- There is **one slack per constraint family**: obs, sc, qlim, sing, cap, spd.

**Objective** [V] (`cbf_safety_filter.py:251-259`, `:1308`):

$$\min_{\ddot q,s}\ \tfrac12\|\ddot q-\ddot q_{ref}\|_2^2+\tfrac12\sum_g\rho_g s_g^2,$$

with $\rho = \{\text{obs }10^3,\ \text{sc }2\cdot10^4,\ \text{qlim }200,\ \text{sing }2\cdot10^3,\ \text{cap }100,\ \text{spd }500\}$.

- The joint-space metric is the identity. There is no mass or task weighting.
- The slack penalty is quadratic, not ℓ1.

**Generic barrier row** [V]. Built at `cbf_state_rows.py:2757`, right-hand side at `cbf_qp_assembly.py:187`:

$$a_i^\top\ddot q + m_i\,s_{g(i)} \;\ge\; -k_{1,i}\big(a_i^\top\dot q_f - v_i\big) - k_{0,i}\,\bar h_i - c_i .$$

**Row families inside the QP:**

| family | $h$ | $a$ | active when | where |
|---|---|---|---|---|
| obstacle | $d-d_{safe}$ (− margins in `full`) | $\hat n^\top J_p$, $c=\hat n^\top\dot J_p\dot q$ | $d<1.2$ m, conf ≥ 0.2, $\|a\|\ge0.05$ | `cbf_state_rows.py:1957-2340` |
| joint limit | $q_{max}-q$, $q-q_{min}$ | $\mp e_j$, $c=0$ | $h<0.30$ rad | `cbf_state_rows.py:55`, `:2426` |
| self-collision | capsule gap − 0.02 | from the closest points | gap < 0.08 + 0.35·$v_{close}$ (release ×1.6), ≤ 8 rows | `cbf_state_rows.py:1068-1168` |
| singularity | $\sigma_{min}(\tilde J)-0.05$ | finite-difference ∇σ | $h<0.08$ | `cbf_singularity.py:161-187` |

Notes on the obstacle row:
- **Control points.** There are 11: two each on links 4–7 and three on link 8 (`config/fr3_complete.yaml:19-61`). Links 0–3 have none.
- **Rows per control point.** Each one gets `multi_obstacle_k = 2` rows, so there are at most 22 obstacle rows.
- **Gains.** $k_0=25$, $k_1=10.5$ for every family. The poles are −3.65 and −6.85, roughly a critically damped $\omega\approx5$.
- **The $\ddot h$ actually used.** It is $a^\top\ddot q + \hat n^\top\dot J_p\dot q$. The curvature term $\dot{\hat n}^\top(\dot p_r-\dot p_o)\ge0$ and the obstacle acceleration are omitted.

**Hard box, not relaxable** [V] (`cbf_safety_filter.py:1309-1320`, `cbf_hard_limits.py:129-326`). This is $lb(q,\dot q)\le\ddot q\le ub$, built from:
- the static acceleration limits `[6, 2.585, 3.5, 4, 6, 5.5, 6]` rad/s²;
- a one-step velocity bound against 0.7 × the FR3 position-dependent velocity envelope;
- a position braking curve (margin 0.05 rad);
- **the slew box**, intersected with the above: $|\ddot q-\ddot q_{prev}|\le0.5\cdot\Delta t_{meas}/0.01$.
- If the boxes are disjoint, the result is pinned to the slew edge.

**Torque.** Torque limits are **not** in the QP. `qddot_to_torque.py:279` computes $\tau=M\ddot q+C\dot q$; gravity is added by the firmware. Effort and rate clipping happen only when `iso_enabled` is set (`qddot_to_torque.py:222`).

**Timing** [V]:
- $A$, $\bar h$ and $c$ are rebuilt at 50 Hz (`cbf_update_rate_hz`).
- Only the $\dot q$ terms of the right-hand side are refreshed at the QP rate. The target is 300 Hz; the ablation uses 100 Hz.
- The solver is OSQP, warm-started. Rows are padded to blocks of 16 to avoid re-setup.

**Outside the QP** [V] (`cbf_safety_filter.py:912-1020`, `:1431-1498`). These do not change the QP:
- ISO stop;
- braking ($-k_b\dot q$) on a stale or frozen joint state;
- braking on a stale nominal command;
- braking on empty distance frames;
- braking on stale distance data;
- the brake-authority fault.

**The fallback ladder.** It runs full QP → **box-only QP, which drops every CBF row** (`:1478`) → closed-form braking.
- The box-only rung passes the nominal command through, clipped only to the box, at exactly the moment the safety QP has failed.
- It is flagged as a fault, but it is not a safe action.

### 2.2 `minimal` (conditions.yaml:121-170)

$$
\begin{aligned}
\min_{\ddot q,s\ge0}\ & \tfrac12\|\ddot q-\ddot q_{nom}\|^2+\tfrac12\textstyle\sum_{g\in\{obs,sc,qlim,sing\}}\rho_g s_g^2\\
\text{s.t. }\ & \hat n_i^\top J_{p,i}\ddot q + s_{obs}\ \ge\ -k_1\big(\hat n_i^\top J_{p,i}\dot q-\hat n_i^\top\hat v_{o,i}\big)-k_0(d_i-d_{safe})-\hat n_i^\top\dot J_{p,i}\dot q\quad(\le 22\text{ rows})\\
& \text{joint-limit, self-collision and singularity rows (§2.1)};\quad lb\le\ddot q\le ub .
\end{aligned}
$$

- $\hat v_o$ is the Kalman-tracked obstacle velocity, signed (`vobs_in_hdot`, `cbf_state_rows.py:2176-2185`).
- There is no filtering on $h$ or on $\dot q$.
- The task enters unmodified: $\ddot q_{ref}=\ddot q_{nom}$.
- Measured rows: median 23, maximum 29 (COST_PROFILE.md).

### 2.3 `full` (the global `cbf:` block, go-around off)

`full` uses the same rows as `minimal`, plus the following.

**(a) Margins inside $h$** [V] (`cbf_state_rows.py:1998-2335`):

$$\bar h_i = \underbrace{\text{EMA}_\uparrow(d_i-d_{safe})}_{\text{recovery }\alpha=0.8} - w_{gate}(v_o)\,[m_{unc}+m_{lat}] - m_{std},$$

where:
- $m_{unc}=\min(2\sqrt{\hat n^\top\Sigma_v\hat n}\cdot0.2,\ 0.25)$, itself EMA'd;
- $m_{lat}=\min\big((\hat n^\top(v\,t_b+\tfrac12 a\,t_b^2))_+ + 2\sqrt{\hat n^\top(\Sigma_{pp}+2t_b\Sigma_{pv}+t_b^2\Sigma_{vv})\hat n},\ 0.25\big)$ with $t_b=0.085$;
- $m_{std}=\min(0.25\,v_o,\ 0.35)$, rising instantly and decaying through an EMA with α=0.8;
- $w_{gate}$ is a smoothstep of $v_o/0.15$.

The worst-case total is **0.85 m on top of $d_{safe}$**. **None of these margins appears differentiated in $\dot h$.**

**(b) The zone ladder** [V] (`cbf_zones.py:224-299`, `cbf_state_rows.py:2675-2717`). It schedules each obstacle row by its raw gap:

| zone | gap | $k_0$ / $k_1$ | slack multiplier $m$ |
|---|---|---|---|
| notice | $d>0.15$ | 12 / 7 | — |
| active | 0.05–0.10 | 25 / 10.5 | — |
| priority | $<0.05$ | 50 / 14 | 0.5 |
| hold | $<0.025$ | — | 0.25 |

The ladder adds two more pieces:
- a depth floor $\bar h\leftarrow\max(\bar h,-0.25k_1/k_0)$ inside the $k_0$ term;
- the **task fade** $w_{task}$, which falls instantly and rises over 0.5 s.

**(c) Changes to $v_o$.** The tracker estimate is combined with a residual floor (max), a 5-tap / 0.3 s median, a fast-track override and identity/normal guards. In addition, $k_1$ sees the EMA $\dot q_f$ (α=0.6).

**(d) Two more row families**, each adding one row per obstacle row:
- **Retreat-speed cap**, `G_CAP`: $-a^\top\dot q$ bounded via the one-step form $(v_{cap}-a^\top\dot q)/0.15$ (`cbf_state_rows.py:148`, `cbf_qp_assembly.py:189-205`).
- **Link-speed cap**, `G_SPD`: $\hat v^\top J_p\dot q\le v_{cap}(d)$, emitted when the speed exceeds half of $v_{cap}$.
- Measured rows: median 46, maximum 69 (COST_PROFILE.md).

**(e) The objective target** [V] (`cbf_safety_filter.py:1031-1303`):

$$\ddot q_{ref}= w_{zone}w_{gov}\,\ddot q_{nom} + (1-w)(-3\dot q) + b_{tan}+b_{esc}+b_{lock}\ (+b_{ga}+b_{outrun}).$$

- The governor fade is directional on sing/sc rows.
- Each bias $b$ is EMA'd separately.

**Configured but inactive in the ablation:**
- go-around (it is **`true` in `fr3_control.yaml`**, so a bare node runs it);
- outrun evasion, the velocity-feedforward braking margin, sensor-range margin, weighted slack, and the ISO speed rows and monitor;
- `cbf_activation_margin` and `cbf_hysteresis`, which are used only by the velocity filter;
- `gamma` and `qp_smooth_weight`, which are unused.

The three default sources disagree with each other (`conditions.yaml:3-9`, "H12"), so what runs depends on the entry point.

---

## 3. Reference implementations

### `oscbf` (torque and velocity modes; uses cbfpy's QP machinery)

**Barriers.**
- Collision: $h_{ij}=\|p_i(q)-o_j\|-r_i-r_j$ for 21 robot spheres × N static obstacle spheres (`oscbf/core/franka_collision_model.py:5-84`). Table, joint-limit, workspace and singularity barriers ($\prod\sigma_k-10^{-3}$) are optional.
- Torque mode makes every barrier HOCBF via cbfpy (`cbfpy/cbfs/cbf.py:214-221`), with $\alpha=\alpha_2=10h$ in all examples. The condition is $\ddot h+20\dot h+100h\ge0$, a double pole at −10.
- $\ddot h$ is obtained by autodiff (nested `jvp`), so the curvature terms are included exactly.

**Objective** (`oscbf/core/oscbf_configs.py:107-125`, `:195-209`):
- Velocity mode: $\tfrac12\Delta u^\top(N^\top W_jN+J^\top W_tJ)\Delta u$.
- Torque mode: the torque equivalent with $M^{-1}$ factors.
- $N$ is the dynamically consistent null-space projector of the end-effector Jacobian.
- The effect: deviation is penalised separately in task space and in the null space, which keeps the end-effector task intact when the null space can absorb the correction.

**Constraints and solver.**
- One row per barrier, and **all pairs are always active**: 546 rows in `cluttered_tabletop`. There is no culling.
- $|u|\le u_{max}$ is soft.
- Slack is **ℓ1 elastic, per row**: ρ=10³ for CBF rows, 10⁵ for limits (`cbfpy/config/cbf_config.py:98-101`).
- The solver is qpax (interior point), fully `jax.jit`-compiled. The solver status and slack values are discarded (`cbf.py:201`).

**What it lacks** [V]:
- Moving obstacles are a self-described hack (`examples/dynamic_obstacle.py:395-420`): only the last sphere, robot velocity set to zero, and $v_o$ inflating $h$ without any $\partial h/\partial t$.
- No perception, latency, uncertainty, rate jitter, fallback or monitoring.
- PyBullet at 240 Hz with ideal actuators and the same model as the plant.

### `cbfpy` (generic library)

- **Config.** You supply `f, g, h_1, h_2`. The HOCBF is $\psi=L_fh_2+\alpha_2(h_2)$, then $L_f\psi+L_g\psi u\ge-\alpha(\psi)$. Defaults are $\alpha=\alpha_2=\text{id}$ (poles at −1).
- **QP.** $\min\|u-u_{des}\|^2$ (default $P=2I$), same elastic ℓ1 slack, soft box.
- **Hard mode.** With `relax_qp=False` it ignores the `converged` flag.
- **Examples.** Low-dimensional: a 3-DoF velocity-controlled arm with joint limits only, and point robots or a drone with a single sphere obstacle (≤7 rows). The obstacle barrier is a "look-ahead" $h=\|\Delta p\|-(v_o-v)^\top\hat n\,T-r$ with $T=2$ s, a heuristic rather than a valid time-varying CBF.
- **What to take from it:** a clean min-norm structure, autodiff Lie derivatives, ℓ1 exact-penalty relaxation, and the documented penalty ordering CLF < CBF < limits.

**Neither reference solves a problem comparable to ours.** Ours has perceived, moving, non-spherical obstacles, a torque-controlled real arm, latency, and non-real-time compute. The references validate the *structure*, not robustness.

---

## 4. Comparison table

| | `full` | `minimal` | `oscbf` (torque) | `cbfpy` |
|---|---|---|---|---|
| Decision variable | $\ddot q$ + 6 family slacks | $\ddot q$ + 6 family slacks | $\tau$ (+ per-row slack) | $u$ (+ per-row slack) |
| CBF form | linear HOCBF, per-row scheduled gains 12/7 → 50/14 | linear HOCBF, 25 / 10.5 | linear HOCBF, 100 / 20 | linear HOCBF, 1 / 2 (default) |
| $\ddot h$ terms | $a^\top\ddot q+\hat n^\top\dot J\dot q$ (no curvature) | same | exact (autodiff) | exact (autodiff) |
| Time-varying obstacle | $v_o$ in $\dot h$ (≥0 or signed) + margins | signed $\hat n^\top\hat v_o$ in $\dot h$ | ignored (hack) | ignored |
| Objective | $\|\Delta\ddot q\|^2$, $\ddot q_{ref}$ faded + 3–5 biases | $\|\Delta\ddot q\|^2$ | task/null-space weighted $\|\Delta\tau\|_P^2$ | $\|\Delta u\|^2$ |
| Slack | quadratic, **shared per family**, $m_i$ scaling | quadratic, shared per family | **ℓ1 per row** | ℓ1 per row |
| Rows (typical) | 46 median / 69 max | 23 / 29 | 21·N_obs (+21), always on | ≤7 |
| Activation | 1.2 m horizon, leverage, zone, horizons on sc/qlim/sing | same horizons | none | none |
| Joint / actuator limits | hard $\ddot q$ box (accel, velocity, position braking, slew) + qlim rows; torque outside the QP | same | soft $\|\tau\|\le\tau_{max}$ | soft box |
| Margins | $d_{safe}$ + up to 0.85 m dynamic | $d_{safe}$ only | radii only | padding |
| Extra mechanisms | zone, governor, 4–5 biases, h/q̇ EMAs, $v_o$ conditioning, caps | none (beyond the shared rows) | none | none |
| Outside the QP | brake on staleness, ISO stop, fallback ladder | same | sim clip only | none |
| Solver / compute | OSQP, Python, GIL-bound; 30–91 Hz achieved at 100 Hz target | 98–100 Hz achieved | qpax IPM, jit | qpax IPM, jit |

**A. What they share.**
- A min-norm QP over the control input, with linear HOCBF rows $L_g\psi\,u\ge-L_f\psi-\alpha(\psi)$ that are affine in the input, plus a box.
- The geometry: distance-based $h$ between robot primitives and obstacle primitives.

**B. What ours does differently, and better.**
- An acceleration decision variable with exact $M\ddot q+C\dot q$ downstream, which is equivalent to torque for the row algebra.
- The obstacle velocity enters $\dot h$ as a time-varying term. That is more correct than both references.
- Hard state-dependent velocity and position braking boxes.
- Distance culling.
- Degraded-mode braking.

**C. What it adds.** See §5.

**D. What the references omit.**
- Reasonable omissions for them, but needed for us: moving obstacles, perception staleness handling, braking on lost inputs, a hard velocity/position box, and rate-aware computation.
- Questionable omissions in any setting: a status/slack check, and soft actuator limits.

---

## 5. Unnecessary complexity — mechanism by mechanism

Verdict key:
- **E** — essential;
- **U** — potentially useful, keep only with A/B evidence;
- **R** — redundant;
- **C** — unnecessarily complicated;
- **H** — potentially harmful.

| mechanism (where) | what it does | verdict | reason |
|---|---|---|---|
| obstacle HOCBF rows | the barrier | **E** | — |
| hard $\ddot q$ box, velocity / position braking | actuator and joint safety | **E** | not relaxable; recorded hardware failures motivate it |
| joint-limit rows | gradual joint-limit avoidance | **E** | header of `cbf_state_rows.py` records the 10.9 rad/s² blowup without them |
| self-collision rows | robot-internal | **E** | the firmware reflex is otherwise the only guard |
| staleness braking, fault flags | degraded modes | **E** | the references have none, and that is the references' weakness |
| slew box | trackability of the command | **U** | jerk p99 ≈ 100–120 in both arms suggests it bites often [D]; consider a torque-rate limit instead |
| singularity row | keeps σ_min ≥ 0.05 | **U** | no A/B evidence; the nominal already uses adaptive damped least squares |
| shared quadratic family slack | feasibility | **H** | see §6 R2; replace with per-row ℓ1 |
| fallback rung 1, box only | QP-failure answer | **H** | drops all barrier rows; go straight to braking |
| $m_{unc}$, $m_{lat}$, approach gate | perception robustness | **C/H** | not differentiated in $\dot h$ and driven by the noisiest signals; replace with one constant, measured margin |
| $m_{std}$ (velocity standoff) | early standoff for fast obstacles | **R/H** | duplicates the $k_1 v_o$ term already in $\dot h$ |
| asymmetric EMA on $h$ | noise rejection | **H** | ratchets toward conservatism (§6 R1) |
| EMA on $\dot q$ for $k_1$ | derivative noise | **U** | the measured fix was 159 % swing on hardware; but this $\dot q$ is from encoders, so check whether the noise was really $\dot q$ or $v_o$ |
| $v_o$ median, residual floor, guards, fast-trust | conditioning of the tracker | **C** | belongs in the tracker, not in the filter; five interacting rules |
| zone ladder gains | per-gap gain scheduling | **C** | breaks the "one barrier, one gain pair" invariant; no A/B evidence |
| zone task fade / hold | stop near obstacles | **H** for task performance | causes "stops" by design; the barrier already prevents approach |
| state governor | task fade near limits | **R** | rows and box already enforce the limits; a fade on top stops the task |
| retreat-speed cap rows | limit push-away speed | **R** | a patch for push-away produced by margins and gains; doubles rows |
| link-speed cap rows | clearance-dependent speed | **U (ISO)** | meaningful only as part of an SSM argument, which is off |
| tangential, lateral-evasion, livelock, go-around, outrun biases | steer around | **C/H** | 4–5 overlapping heuristics on $\ddot q_{ref}$; the task/null metric plus complete $\ddot h$ address the same need inside the QP |
| depth floor `h_pen` | caps recovery speed | **R** | another push-away patch |
| row padding, 50 Hz / QP-rate split | compute | **C** | symptoms of Python cost; a compiled single-rate loop removes the need |

---

## 6. Excessive reactivity — causes ranked by evidence

### Mathematically demonstrated

**R1 — Margins in $h$ without $\dot m$, amplified by $k_0$.**

Let $h_{eff}=d-d_{safe}-m(t)$. The row uses $\dot h=a^\top\dot q-\hat n^\top v_o$ and omits $-\dot m$. A margin step $\Delta m$ therefore appears instantly as a demanded repulsive acceleration of $k_0\Delta m$ along $\hat n$.

Example:
- A $v_o$ blip of 0.4 m/s raises $m_{std}$ by 0.1 m.
- That demands **2.5 m/s²** of repulsion at $k_0=25$, and **5 m/s²** in the priority zone.

The margins depend on $v_o$ and on covariances, which are the noisiest quantities in the stack. The asymmetry makes it worse:
- $h$ recovery (α=0.8 at 50 Hz, τ≈0.22 s) and the $m_{std}$ decay both rise instantly and decay slowly.
- The arm is pushed away *and held away*.
- The retreat cap and the depth floor exist to limit exactly this push-away. They are patches on R1.

**R2 — Quadratic slack shared per family.**

*Single row, box inactive.* Minimising $\tfrac12\|\Delta\|^2+\tfrac12\rho s^2$ subject to $a^\top\Delta+s\ge r$ gives

$$s^\*=\frac{r}{1+\rho\|a\|^2}.$$

So a soft row *always* gives in when the nominal pushes into it ($r>0$).

*When the box or slew limit binds,* the slack absorbs whatever the box forbids.

*Steady state for a static obstacle:* $h_{ss}=-s/k_0$.
- For $\|a\|=0.05$ (the leverage floor), $\rho\|a\|^2=2.5$, so 30 % of the push leaks through.
- Because one $s_{obs}$ relaxes *every* obstacle row, the worst row loosens all the others.

Data [D] (`exp_minimal_vs_full_20261102/cells.csv`):
- The minimal static cells show min_h −1.8 to −7.6 cm with h<0 for 2.0–9.7 s, alongside nonzero `slack_obs` of 0.3–1.9 m/s².
- At $k_0=25$ that slack corresponds to −1.2 to −7.6 cm, consistent with what was measured. (Perception error, S1, may add to it.)
- `full`'s margins mask this leak, so conservatism is used as a substitute for a hard-enough constraint.

ℓ1 per-row slack, as in the references, is an *exact penalty*: when $\rho$ exceeds the row's dual multiplier, $s=0$ whenever the row is feasible.

> **Update 2026-11-04 (measured; see `franka_simulation_experiments/sim_runs/minimal_l1_slack_20261104/`).**
> Per-row ℓ1 slack was implemented as the `minimal_l1` condition and run in the DIAG lab.
>
> - **Static case (`pentagon_static_baseline`): R2 does not explain the penetration.**
>   - The old shared slack peaked at only 0.026 m/s² (≈1 mm).
>   - With ℓ1 the slack is exactly 0, yet the true gap still reaches 6.6 cm, against 6.5 cm for `minimal`.
>   - Perception was optimistic by +1.4 cm on average (up to +5.5 cm) near the ball. The filter itself saw h < 0 on about 32 % of ticks while its rows were satisfiable.
>   - So in this scenario the leak comes from perception and the delay chain, not from slack.
>   - The cells cited above (`slack_obs` 0.3–1.9) were different scenarios, so R2 may still matter there.
> - **Moving case (`pentagon_lateral`): R2's coupling effect was visible.**
>   - The shared slack reached 1.56 m/s²; the per-row slack peaked at 0.09.
>   - Clearance was 15.5 cm vs 12.4 cm.
> - n = 1 per cell.

**R3 — Task fade (zone hold/priority) and governor.**
- $w_{task}\to0$ below a 2.5 cm gap; the cut is 50 % below 5 cm.
- The resume ramp is 0.5 s.
- The governor fades the task near the velocity envelope, σ, or self-collision.
- These are *intended* stops, and with noisy gaps (4 cm calibration spread) they trigger on noise. The code comment itself acknowledges this at `cbf_safety_filter.py:1047-1051`.

**R4 — Missing curvature term.**

The true second derivative is $\ddot h=a^\top\ddot q+\hat n^\top\dot J_p\dot q+\dot{\hat n}^\top v_{rel}-\hat n^\top a_o$. For point–sphere geometry, $\dot{\hat n}^\top v_{rel}=\|v_{rel,\perp}\|^2/\rho_c\ge0$, with $\rho_c\approx d+r_{obs}$.

Dropping it is conservative in sign (OPEN_ISSUES S1 confirmed ≥0 on 44 samples), but its size is not small:
- At $v_\perp=0.3$ m/s and $\rho_c=0.15$ m it is **0.6 m/s²**.
- That is larger than $k_0h$ at h = 2 cm (0.5 m/s²).

The filter therefore brakes or pushes outward precisely while the arm slides *around* the obstacle. Both references include this term automatically through autodiff.

**R5 — Gain choice sets *when* the robot reacts.**
- A row binds when $k_1v>k_0h$ (ignoring $c$), i.e. at $h<(k_1/k_0)v$:
  - oscbf: 0.20·v;
  - ours: 0.42·v;
  - `full` notice zone: 0.58·v.
- The allowed steady closing speed is $(k_0/k_1)h$: 5h for oscbf vs 2.4h (ours) and 1.7h (notice).
- So `full` starts deviating about 3× earlier than oscbf, and more gently.
- This is a tunable trade against acceleration authority, not a bug. It should be swept with $(k_0,k_1)=(\omega^2,2\omega)$.

### Supported by existing data

**D1 — Achieved rate, which is the uncontrolled variable.**
- `full` achieved 30–91 Hz at a 100 Hz target, against ~100 Hz for `minimal` (cells.csv).
- Asking for 300 Hz delivered *less* than asking for 100 (COST_PROFILE.md).
- The rate drives tracking and contact outcome (S6: 5/7 contacts at >80 Hz, 0/4 at ≤80 Hz).
- The cause is Python and the GIL, not OSQP (0.25 ms offline). It is an implementation issue, but it confounds every comparison.

**D2 — Perception produces most of the dangerous events.**
- S5: the track dies in the self-mask, perception reports +0.157 m optimistic, contact follows.
- S3: rig phantom obstacles cause 36–76 rows, slack up to 45.7, 71–303 reflex trips — the robot "reacts" to nothing.
- F4: links 0–3 are unmonitored.
- F5: a 1–25° normal error, from $\hat n$ taken at $t_{cap}$ while $J$ is evaluated at $q_{now}$.

Over-reaction to phantom or furniture tracks is perception, not the QP.

**D3 — Tracking deviation.** `full` has worse EE p95 than `minimal` in all six paired scenarios: 0.32–1.08 m vs 0.06–0.31 m. Caveats: n ≤ 2 seeds, and the comparison is rate-confounded (D1).

**D4 — Jerk p99 sits at 100–120 in both arms.** This is consistent with the slew box being active often, so the QP solution jumps faster than the arm is allowed to follow.

### Plausible, untested

- **H1 — Bias stacking causes oscillation and switching.** Tangential, evasion, livelock and go-around have different triggers and separate EMAs. The code comments record oscillations that were fixed by adding EMAs, which is evidence the biases fight each other or the rows.
- **H2 — Identity metric plus a Cartesian PD nominal compound the deviation.** The min-$\|\Delta\ddot q\|$ correction also changes the EE acceleration. The PD nominal ($k_p=20$, $k_d=9$) then sees a growing error and pushes harder into the row. That increases $r$ in R2, so more slack leaks and the response swings more.
- **H3 — Stale geometry against a fresh $\dot q$.** Using 50 Hz geometry ($\hat n$, $h$, $c$) with a $\dot q$ refreshed at the QP rate gives a right-hand side inconsistent within the 20 ms window.
- **H4 — Duplicate rows.** `multi_obstacle_k=2` gives near-duplicate rows per control point when two clusters are adjacent. That is harmless to feasibility but adds cost.

**Verdict.**
- The over-reaction comes mainly from what `full` does *around* the HOCBF: R1, R3, the biases, and the margins that compensate for R2.
- The HOCBF core is not the main cause, though R4 and R5 contribute.
- The *contacts* come mainly from perception and the rate (D1, D2).
- The minimal arm's static leak is R2.

---

## 7. Simplification proposals

### Option A — Minimal changes (keep the architecture)

**Retain:** the `minimal` condition as the new baseline: the shared rows, box, slew box, staleness braking and OSQP.

**Remove:**
- Delete or hard-disable everything §5 marks R/C/H.
- Turn off `enable_go_around` in `fr3_control.yaml`.
- Reconcile the three default sources so that a bare launch runs `minimal`.

**Modify (small code changes):**
1. Fallback rung 1: keep the obstacle rows (soft), or go directly to braking. Never pass the nominal through the box alone.
2. Fix perception S3/S5/F4/F5 and S4 (the status fields).
3. Pin the rate (S6) and report it with every result.

**Resulting QP:** the one in §2.2.

**Advantages:** about half the rows, rate held, interpretable behaviour, zero new code risk.

**Disadvantages:** the static leak (R2) and the curvature conservatism (R4) remain.

**Effort:** about 1–2 days, mostly configuration plus deleting dead paths.

### Option B — Simplified CBF/QP (recommended)

$$
\begin{aligned}
\min_{\ddot q,\ s\ge0}\ & \tfrac12\|J\,\Delta\ddot q\|^2_{W_t}+\tfrac12\|N\,\Delta\ddot q\|^2_{W_j}+\tfrac{\epsilon}{2}\|\Delta\ddot q\|^2+\rho\,\mathbf 1^\top s_{obs}+\rho_{sc}\mathbf 1^\top s_{sc}\qquad(\Delta\ddot q=\ddot q-\ddot q_{nom})\\
\text{s.t. }\ & a_i^\top\ddot q+s_i\ \ge\ -k_1\dot h_i-k_0 h_i-\hat n_i^\top\dot J_{p,i}\dot q-\kappa_i,\qquad h_i=d_i-(d_{safe}+\delta_{perc}),\ \dot h_i=a_i^\top\dot q-\hat n_i^\top\hat v_{o,i}\\
& \kappa_i=\|(I-\hat n_i\hat n_i^\top)v_{rel,i}\|^2/\rho_{c,i}\quad\text{(curvature term, }\ge0)\\
& \text{joint-limit HOCBF rows (hard, or }\rho\gg),\ \text{self-collision rows (per-row slack)},\ lb(q,\dot q)\le\ddot q\le ub .
\end{aligned}
$$

**Retain:**
- the obstacle HOCBF, using signed $\hat v_o$ in $\dot h$ (correct and better than the references);
- the hard box including position and velocity braking;
- the joint-limit and self-collision rows;
- the slew box, or a torque-rate limit;
- staleness braking;
- distance culling;
- one gain pair, swept per §9.

**Remove:**
- all dynamic margins, the h/q̇ EMAs, the zone ladder and task fade, the governor;
- all objective biases;
- the retreat and link-speed families;
- the singularity row, unless an A/B test shows it is needed;
- the $v_o$ conditioning in the filter (the tracker owns it).

**Modify:**
- **Per-row ℓ1 slack.** Use a linear cost on $s$ with large ρ (for example 10⁴), plus a tiny quadratic term for OSQP. This gives exact-penalty behaviour: zero slack whenever the row is feasible.
- **One constant margin.** $\delta_{perc}$, sized from measured perception error (4–6 cm per the calibration residual), replaces 0.85 m of dynamic margins. If $\hat v_o$ uncertainty must enter, put it in $\dot h$ as a bounded worst case ($\hat n^\top\hat v_o+k\sigma_v$), not in $h$.
- **The OSCBF metric on $\ddot q$.** Use $J$ = EE Jacobian and $N$ = dynamically consistent null-space projector. Corrections then go to the null space first, which keeps the EE on its path. Our nominal already uses $J^+$ and $N$, so they are available.
- **Single-rate geometry.** Rebuild $h$, $\hat n$ and $J$ on the QP tick, or at least use the same-$q$ normal (F5).

**Advantages:**
- removes the R1–R4 mechanisms at the formulation level;
- hard-enough obstacle rows with zero leak when feasible;
- about 20–35 rows;
- every term is explainable.

**Disadvantages:**
- With ℓ1 and large ρ, infeasible situations (head-on, unescapable) produce large, abrupt slack. This must be reported, with braking as the fallback.
- The metric needs $M$ and the EE $J$ each tick, which is cheap through Pinocchio.
- Robustness to perception now rests on $\delta_{perc}$ and the tracker, and that assumption must be stated explicitly.
- No formal input-constrained feasibility guarantee. None of the four implementations has one.

**Effort:** about 1 week, mostly in `cbf_state_rows.py` and `cbf_qp_assembly.py`. The pieces needed for the metric exist in `pentagon_qddot_commander.py:975-1010`.

### Option C — Reference-inspired redesign

Use the same QP as B, but restructure the code the way oscbf does:
- **One barrier abstraction.** $h(q,t)$ over robot primitives (keep our control points and capsules, or move to the oscbf 21-sphere model) × perceived obstacle primitives.
- **One loop.** $\dot h$ and $\ddot h$ terms from autodiff (JAX/CasADi) or analytic code, so curvature is exact; one compiled loop computing the rows, metric and QP.
- **Compiled execution.** A jitted Python kernel, or a C++ ros2_control plugin with OSQP or qpOASES, which removes the GIL and rate issues entirely.
- **Separation of concerns.** Perception (tracking, margins), the safety filter (rows only) and the independent monitor (ISO) are separate processes with explicit interfaces.

**Advantages:** deterministic rate, removes about 70 % of `cbf_state_rows.py` (3,359 lines), testable math.

**Disadvantages:** a rewrite, with requalification of everything the current code learned on hardware (staleness cases, frozen state, braking semantics).

**Effort:** 3–6 weeks.

---

## 8. Final recommendation

**Do A now, then B, and C only if the rate cannot be held in Python.** Order matters:
1. First make the comparison valid: fix perception S3/S5/F4/F5, the status fields (S4), and the rate (pin it, reject runs outside a band).
2. Then adopt `minimal` as the baseline (A).
3. Then add B's changes **one at a time**, in decreasing expected impact:
   - per-row ℓ1 slack (R2);
   - the curvature term (R4);
   - the constant $\delta_{perc}$ (it replaces the margins);
   - the OSCBF metric (H2);
   - the gain sweep (R5).

Delete each `full` mechanism permanently unless a matched A/B test shows a safety benefit.

### The answer to the main question

**Keep:**
- the acceleration-level linear HOCBF with $\hat n^\top J_p$ rows and signed obstacle velocity in $\dot h$;
- the hard state-dependent $\ddot q$ box (acceleration, one-step velocity, position braking);
- joint-limit and self-collision HOCBF rows;
- a jerk or slew limit;
- staleness and frozen-state braking with fault flags;
- distance culling;
- OSQP with a warm start;
- the downstream $\tau=M\ddot q+C\dot q$.

**Remove:**
- the uncertainty, latency, standoff and braking margins and the approach gate;
- the asymmetric EMAs on $h$ and on the margins;
- the zone ladder (gains, slack multipliers, task fade) and the state governor;
- the tangential, lateral-evasion, livelock, go-around and outrun biases;
- the retreat-cap and depth-floor rows (they are push-away patches);
- the link-speed rows, unless ISO SSM is pursued;
- the $v_o$ conditioning in the filter;
- the box-only fallback rung;
- the singularity row, pending an A/B test.

**Redesign:**
- shared quadratic family slack → **per-row ℓ1 exact-penalty slack** (from oscbf and cbfpy);
- identity metric → **task/null-space metric** (oscbf);
- incomplete $\ddot h$ → **include curvature** (what oscbf and cbfpy get from autodiff);
- dynamic margins → **one constant, measured perception margin**;
- 50 Hz / QP-rate split in GIL-bound Python → **single-rate compiled loop** (oscbf's jit; Option C if needed);
- perception owns obstacle velocity and uncertainty; the filter consumes them.

---

## 9. Validation plan (minimum set)

**Controls, applied to every comparison:**
- the same commander and trajectories, scenes, perception stack and seeds;
- **achieved QP rate pinned**: reject any run outside ±5 % of 100 Hz, and report the rate alongside each result;
- conditions interleaved seed by seed, never arm by arm (S6);
- the same $d_{safe}$ and the same box.

**Two perception modes:**
1. **Ground-truth obstacles**, fed from `truth`. This isolates the controller.
2. The **kf tracker**. This measures the controller and perception together.

A mechanism that only helps in mode 2 is a perception fix and belongs in perception.

**Arms:** `minimal` (A) → +ℓ1 per-row slack → +curvature → +$\delta_{perc}$ → +OSCBF metric (= B) → `full`. Optionally add a gain sweep $\omega\in\{5,7,10\}$ on B.

**Scenarios:**
- the existing six: `pentagon_on_path`, `pentagon_multi_static`, `approach_hold`, `pentagon_crossing`, `pentagon_pursuer`, `pentagon_multi_mixed`;
- plus `pentagon_lateral` (escapable, moving) and `pentagon_sweep`.
- Use **≥12 seeds** per cell. Contact rates around 5–10 % need that many to separate arms; report Clopper–Pearson intervals.

**Metrics:**

| category | metrics |
|---|---|
| Safety | minimum true clearance; time and depth with true gap < $d_{safe}$; contact events; reflex trips |
| Performance | EE deviation from the nominal *path* (geometric, not time-indexed) and from the time-indexed target; task progress or lap time; time with $w_{task}<1$ (should be zero for B) |
| Smoothness | jerk on a common 100 Hz grid; fraction of ticks where the slew box bites; sign changes of $a^\top\ddot q$ on active rows (oscillation) |
| Solver | solve time p50/p99; achieved rate; rows per tick; fallback-rung counts |
| Constraints | fraction of ticks with $s_i>0$, max and integrated slack per family; active-row count; time each row is active |

**Cheap pre-checks** (unit tests, no Gazebo):
1. Static-ball steady state: check $h_{ss}$ against $\rho$ for quadratic-shared vs ℓ1-per-row slack. This confirms or refutes R2 directly.
2. Tangential sliding past a sphere, with and without $\kappa$. This confirms R4.
3. A margin step response: inject a 0.1 m step in $m$ and measure the peak $\|\Delta\ddot q\|$ (R1).

**Rule for every call above.** No simplification is accepted or rejected on visuals or nominal rate. Either decision needs a paired, rate-matched, ≥12-seed comparison under ground-truth perception, in which safety does not get worse and tracking or smoothness gets better.
