# FR3 simulation: baseline and full-online HQP controllers

**Technical report - 18 November 2026**

## 1. Purpose and evidence

This report explains the circular and perpendicular baseline controllers and their full-online optimized counterparts, including the actual hierarchical quadratic programming (HQP) implementation. It compares saved simulation measurements, explains the gain choices, and documents why smooth ramps are used to reduce observer transients.

Figures reproduce the standard plots saved with the selected runs. Controller comparisons are presented sequentially, with the baseline first and the full-online controller second.

The selected logs all request a 1000 Hz controller and a real-time factor (RTF) of 0.20. The circular pair reaches 20 simulated seconds. The perpendicular pair completes its scheduled approach and endpoint hold after approximately 0.57 s; the configured 2 s launch duration is an upper bound, not an unfinished trial.

The first-impact results show **35.69 percent less entry mass in the circular case** and **32.02 percent less in the perpendicular case**. Entry speeds are closely matched. These measurements compare complete controller implementations: they do not isolate mass minimization from differences in redundancy resolution, damping and constraints.

There are two important provenance limits. The circular optimized run includes a return to the initial posture after each revolution, while its baseline runs continuously. The perpendicular pair comes from the same saved tuning study, with the active 4/12 secondary gains, but neither log records the newer first-RUN-sample time-origin convention. The report uses the parameters stored inside each NPZ, rather than replacing historical settings with today's YAML files.

## 2. Experimental setup and metric definitions

Both experiments use the seven-joint FR3 model, starting at

$$
q_0=[0,-0.1517,0,-2.6105,0,4.0295,0.785]^T\ \mathrm{rad}.
$$

The controlled fingertip starts near $(0.56,0,0.50)$ m. The wall is the plane $x=0.62$ m, with outward normal $n=[-1,0,0]^T$, stiffness 20000 N/m and damping 15 N s/m. The logged normal velocity is positive into the wall. The selected runs use nominal dynamics without measurement noise, mass error or joint friction.

The circular reference is a vertical x-z circle of radius 0.04 m and cruise period 2 s, with a 1 s angular startup ramp. One revolution from rest takes 2.5 s because of that ramp. The perpendicular reference approaches at 0.20 m/s after a 0.10 s speed ramp, requests 10 mm final overlap, brakes over the final 4 mm and holds the endpoint for 0.15 s. Reference overlap is not measured wall penetration.

The comparison reuses `first_impact` from the analysis code. A contact begins at the first sample with $F_n>0$. Its first compression-release cycle ends at detachment or at the interpolated start of recompression, whichever occurs first. Later bounces do not contribute to that cycle's force, impulse or penetration metrics.

Entry mass and velocity come from the sample strictly before contact. Entry normal kinetic energy and normal impulse are

$$
E_n^- = \frac12 m_n^- (v_n^-)^2,\qquad
I_n=\int_{t_c}^{t_e}F_n(t)\,dt.
$$

Impulse uses trapezoids at the actual saved timestamps. Tracking RMS is time-weighted; the tables use the precontact interval. Missing data are not replaced by a current-contact sample. All limit checks refer to saved samples, not to a continuous-time certificate.

## 3. Controller formulation

### 3.1 Shared Cartesian tracking law

Let $J=\partial p/\partial q$, $e=p_d-p$ and $\dot e=v_d-J\dot q$. All four controllers request the same Cartesian acceleration feedback:

$$
a_{\mathrm{trk}}=a_d+100e+20\dot e,\qquad
b=a_{\mathrm{trk}}-\dot J\dot q.
$$

The primary request is acceleration feedback with trajectory feedforward. No posture-dependent Cartesian inertia multiplies the 100/20 gains. Only fingertip position is controlled; orientation remains free.

### 3.2 Baseline controllers

The baseline uses the Euclidean pseudoinverse and its nullspace projector:

$$
J^\dagger=\operatorname{pinv}(J),\qquad N_E=I-J^\dagger J,
$$

$$
u_B=J^\dagger b+N_E\bigl[25(q_0-q)-10\dot q\bigr],\qquad
\tau_{\mathrm{cmd}}=M(q)u_B+c(q,\dot q).
$$

Here $c=C(q,\dot q)\dot q$. The posture attraction and damping remain active throughout motion. They regulate redundancy without directly compensating an external contact force. The baseline records directional mass but does not minimize it. Its torque is clipped by the runner, which also enforces its existing stop conditions; the baseline does not solve an HQP.

`baseline_perp.py` inherits this law and replaces the circular reference with the perpendicular approach. In this simulator gravity compensation is supplied separately, so the returned actuator command does not add the gravity vector $g$.

### 3.3 Full-online optimized controllers

The optimized controllers construct a dynamically consistent inverse and projector:

$$
\bar J=M^{-1}J^T(JM^{-1}J^T)^{-1},\qquad N=I-\bar JJ.
$$

Directional effective mass and the online mass-gradient torque preference are

$$
m_n(q)=\frac{1}{n^TJM^{-1}J^Tn},\qquad
\tau_m=-\rho(t)k_mN^T\nabla_qm_n(q).
$$

The gradient is evaluated analytically at the measured configuration, using model derivatives. It is neither normalized nor clipped. No offline low-mass posture is required. The projected torque satisfies $JM^{-1}\tau_m=0$ in the nominal full-rank model; the final feasible command is selected by the HQP.

During ordinary tracking the secondary acceleration preference is

$$
u_{\mathrm{pref}}=\bar Jb-k_dN\dot q+M^{-1}\tau_m.
$$

The perpendicular full-online controller uses this strategy throughout its approach and contact, with $k_m=4$ and $k_d=12$. It has no initial-posture attraction and no collision-triggered retreat. The observer influences disturbance-aware constraint predictions; its residual is not directly subtracted from the commanded torque.

The circular full-online controller uses $k_m=2$ and $k_d=24$ while tracking. After each revolution it enters RETURN: a 0.20 s braking reference reaches the initial Cartesian point at rest, the mass term fades out, and a nullspace attraction towards $q_0$ fades in. Return gains are 25 and 10. It restarts only when every joint error is at most 0.001 rad and every joint speed is at most 0.005 rad/s. A return exceeding 5 s stops the trial. There is no continuous $q_0$ attraction during the tracking phase.

## 4. How the HQP is used and solved

### 4.1 Decision variable and constraints

The optimization variable is actuator-induced joint acceleration,

$$
u=M^{-1}(\tau_{\mathrm{cmd}}-c)\in\mathbb R^7.
$$

The external disturbance estimate and its uncertainty enter the acceleration bounds. They do not redefine $u$ as exact plant acceleration. The implementation predicts through the configured 3 ms command allowance and tightens the bounds using the observer estimate and an empirical uncertainty reserve.

The hard feasible set is

$$
\mathcal H=\left\lbrace u:\ell_h\le u\le r_h,\quad
-T_{\mathrm{eff}}\le Mu+c_{\mathrm{eff}}\le T_{\mathrm{eff}}\right\rbrace,
$$

$$
c_{\mathrm{eff}}=c+\frac12g,\qquad
T_{\mathrm{eff}}=T-\frac12|g|.
$$

This torque intersection bounds both the gravity-compensated actuator command and the nominal motor torque including gravity. The mandatory position margin is 0.06 rad. Configured velocity and torque limits remain hard; torque limits are 87 N m for joints 1-4 and 12 N m for joints 5-7.

Three groups of soft acceleration bounds are also supplied: the 30 rad/s$^2$ acceleration preference, the preventive 0.15 rad position barrier and the velocity-rate barrier. They may be relaxed to maintain hard feasibility. In particular, 30 rad/s$^2$ is not a guaranteed acceleration ceiling during impact.

### 4.2 Level 1: minimize soft-constraint relaxation

With nonnegative physical slacks $s_L,s_U\in\mathbb R^{3\times7}$ and positive normalization scales $S_{gi}$, solve

$$
\min_{u,s_L,s_U}\quad \frac12\sum_{g,i}
\left[\left(\frac{s_{L,gi}}{S_{gi}}\right)^2+
\left(\frac{s_{U,gi}}{S_{gi}}\right)^2\right]
$$

$$
\mathrm{subject\ to}\quad u\in\mathcal H,\quad s_L,s_U\ge0,\quad
L_{gi}-s_{L,gi}\le u_i\le U_{gi}+s_{U,gi}.
$$

There is no acceleration penalty at this level: minimizing relaxation has priority over tracking and redundancy. Numerically this QP has 49 variables: seven scaled accelerations and 42 dimensionless lower/upper slacks. Torque rows are normalized as well.

The verified optimal slacks are frozen for the next levels. Up to a numerical allowance $\delta=2\times10^{-5}$ in the bound units, they define

$$
\ell_F=\max\bigl(\ell_h,\max_g[L_g-\hat s_{L,g}-\delta]\bigr),\qquad
r_F=\min\bigl(r_h,\min_g[U_g+\hat s_{U,g}+\delta]\bigr).
$$

Lower-priority tasks therefore cannot obtain better tracking by demanding arbitrarily larger soft-bound relaxation.

### 4.3 Level 2: Cartesian tracking

Within the hard torque constraints and the frozen interval $\ell_F\le u\le r_F$, solve

$$
u_2=\arg\min_u\left[\frac12\|Ju-b\|_2^2+
\frac{\epsilon}{2}\|u\|_2^2\right],\qquad \epsilon=10^{-8}.
$$

The Hessian is $J^TJ+\epsilon I$. If constraints prevent exact tracking, the achieved task acceleration is $Ju_2$, which need not equal $b$.

### 4.4 Level 3: online mass and redundant motion

Solve

$$
u_3=\arg\min_u\frac12\|u-u_{\mathrm{pref}}\|_2^2,
\qquad Ju=Ju_2,
$$

under the same bounds and torque constraints. The equality preserves the task acceleration actually attained at level 2. The secondary objective is Euclidean, with Hessian $I$; inertia is used in the preference and torque mapping, not as an extra weighting matrix in this QP. The selected acceleration produces $\tau_{\mathrm{cmd}}=Mu_3+c$.

This is a lexicographic sequence of convex QPs at the current sample. It does not directly optimize future impact mass or solve a predictive trajectory problem.

### 4.5 Numerical solution and fallback policy

Three persistent OSQP instances retain sparse matrix patterns and warm starts between updates. Current settings are absolute/relative tolerance $10^{-6}$, at most 1000 iterations, termination checks every ten iterations, adaptive rho enabled and polishing disabled. Matrix coefficients and bounds are updated from the current state.

An OSQP status of solved or solved-inaccurate is accepted only after independent finite-value and physical-unit feasibility checks. The feasibility tolerance is $2\times10^{-4}$ and the task-preservation tolerance is $2\times10^{-5}$. Solver status alone is insufficient.

If the secondary QP fails, the fresh verified level-2 solution is used. If tracking fails after level 1 succeeds, the fresh verified level-1 command is used. If level 1 cannot produce a verified hard-feasible command, the controller requests termination and applies bounded damping. No earlier update's command is reused as a successful solution.

## 5. Gain choices and ramps

### 5.1 Analytical interpretation of tracking and posture gains

Under ideal model cancellation, full task rank and inactive constraints, neglecting the small QP regularizer, the shared tracking law yields

$$
\ddot e+20\dot e+100e=0.
$$

Thus $K_p=100$ and $K_d=20$ give nominal critical damping with natural frequency $\omega_n=10$ rad/s. Using the same primary gains makes the requested tracking law comparable across families.

Likewise 25/10 corresponds to nominal critical damping at 5 rad/s in a scalar posture model. This motivates the baseline posture gains and the circular RETURN gains. The robot's time-varying projection and active constraints mean that these scalar interpretations are not global stability proofs.

### 5.2 Numerical choice of the online secondary gains

The mass weight and nullspace damping balance posture change against available preparation time. With zero posture stiffness, a critical-damping formula does not determine the online damping gain. The mass gain is in N m rad/kg and damping in 1/s. The following values are measured from available perpendicular tuning logs at RTF 0.20:

| Mass gain | Nullspace damping (1/s) | Entry mass (kg) | Min margin (rad) |
| ---: | ---: | ---: | ---: |
| 2 | 24 | 5.941938 | 0.250943 |
| 4 | 24 | 5.199561 | 0.151993 |
| 4 | 12 | 5.181674 | 0.151151 |

The active 4/12 choice lowers entry mass while retaining the nominal 0.15 rad preferred margin in this saved trial. Its advantage over 4/24 is small, so the data support a local mass-priority choice, not a universal optimum or a robustness guarantee. The 0.10 s startup leaves about 0.25 s at full activation before the approximately 0.35 s perpendicular contact.

The circular TRACK settings 2/24 are separate starting settings, not the result of an exhaustive circular tuning. Their 0.10 s mass ramp completes about 1.07 s before first contact. RETURN uses the analytically motivated 25/10 posture gains rather than continuing mass descent while repositioning.

### 5.3 Why smooth ramps matter for the observer

The generalized momentum observer uses measured $q,\dot q$ and the aligned, delayed actuator command. It does not read true wall force. Its gain is 30 s$^{-1}$ and a detection occurs when any joint residual exceeds 0.1 N m in magnitude. Under the ideal model it estimates filtered external torque; discrete integration, command alignment and model errors can also create residuals during sharp input changes.

The secondary startup uses the cubic smoothstep $\rho(u)=3u^2-2u^3$, clipped to $u\in[0,1]$. It changes objectives before the QPs, rather than scaling the final constrained torque. The perpendicular 0.10 s speed ramp and circular 1 s angular ramp are separate trajectory ramps.

In the earlier abrupt-return circular log, noncontact detections occurred at 6.694 and 19.286 s, 2 ms after RETURN entry. Force was zero and the fingertip was approximately 60 mm outside the wall. Reference speed dropped from 0.12566 m/s to zero, the mass-torque norm from 15.39 N m to zero, and the actuator command changed by approximately 6.96 N m. No local scheduling gap was found at those two events.

The current 0.20 s quintic braking reference matches position, velocity and acceleration at entry and reaches the initial point at rest. A simultaneous smoothstep fades mass down and blends posture attraction and damping. Damping also blends on the subsequent 0.10 s tracking restart. Observer history, thresholds and absolute time remain unchanged.

![Before the return ramp: the original circular full-online observer plot. Noncontact detection onsets occur at 6.694 and 19.286 s.](report_assets/circular_observer_before.png)

![After the 0.20 s return transition: the original circular full-online observer plot for the selected 20 s run.](report_assets/circular_observer_after.png)

The selected newer 20 s circular log contains five detection onsets, all during positive wall force, and none during its RETURN transitions. This is evidence for the observed nominal run, not a guarantee against all future false detections. The older perpendicular pair still contains off-contact observer onsets; its timing provenance and detector transients must not be hidden by the ramp explanation.

## 6. Circular experiment

### 6.1 First compression-release cycle

| Metric | Baseline | Full online | Change (percent) |
| --- | ---: | ---: | ---: |
| Contact onset (s) | 1.167 | 1.167 | -- |
| Entry mass (kg) | 7.189 | 4.623 | -35.69 |
| Entry speed (m/s) | 0.10893 | 0.10893 | -0.0026 |
| Entry energy (mJ) | 42.656 | 27.430 | -35.69 |
| Peak force (N) | 43.147 | 34.354 | -20.38 |
| First-cycle impulse (N s) | 2.1002 | 1.2581 | -40.09 |
| Peak penetration (mm) | 2.155 | 1.714 | -20.44 |
| First-cycle duration (ms) | 84.0 | 59.0 | -29.76 |
| Precontact RMS error (mm) | 0.0190 | 0.0231 | +21.40 |

The entry-speed difference is only 0.0026 percent, so the 35.69 percent entry-energy reduction closely follows the mass reduction. Peak force falls by 20.38 percent, first-cycle impulse by 40.09 percent and penetration by 20.44 percent. Peak force and impulse need not scale linearly with effective mass: closed-loop motion, release timing, damping and the wall model also matter.

![Circular baseline: original first-collision plot, including precontact mass and speed, force, penetration, impulse and energy.](report_assets/circular_baseline_collision.png)

![Circular full-online controller: original first-collision plot for the selected optimized run.](report_assets/circular_full_online_collision.png)

### 6.2 Repeated circles and return scheduling

The baseline records ten main wall approaches during continuous tracking in the 20 s horizon, with 20 distinct positive-force contact episodes including recontacts. The full-online run completes five revolutions and records ten episodes. Four returns complete in approximately 1.799 s each; the final return begins at 19.696 s and is interrupted by the configured duration limit. All completed restarts satisfy the configured joint-position and velocity tolerances.

![Circular baseline: original all-collisions plot over the complete 20 s run.](report_assets/circular_baseline_all_collisions.png)

![Circular full-online controller: original all-collisions plot. Returns between laps produce a different impact schedule from the continuous baseline.](report_assets/circular_full_online_all_collisions.png)

Consequently, whole-run force totals or cumulative impulse would mix different numbers of contacts and different time schedules. The first-impact comparison above remains informative, but `compare_runs` reports `full_criterion_met: false` because `same_scenario` is false. This is the only unmet check for this pair; the primary tracking law and gains match. The baseline also predates the explicit first-sample time-origin metadata, whereas the newer optimized run records it.

## 7. Perpendicular experiment

| Metric | Baseline | Full online | Change (percent) |
| --- | ---: | ---: | ---: |
| Contact onset (s) | 0.350 | 0.350 | -- |
| Entry mass (kg) | 7.622 | 5.182 | -32.02 |
| Entry speed (m/s) | 0.20003 | 0.19996 | -0.0332 |
| Entry energy (mJ) | 152.484 | 103.596 | -32.06 |
| Peak force (N) | 85.235 | 68.909 | -19.15 |
| First-cycle impulse (N s) | 3.5438 | 2.6406 | -25.49 |
| Peak penetration (mm) | 4.256 | 3.438 | -19.20 |
| First-cycle duration (ms) | 64.0 | 60.0 | -6.25 |
| Precontact RMS error (mm) | 0.1011 | 0.0861 | -14.81 |

Entry speeds differ by 0.0332 percent. Relative to the selected baseline, the full-online run reduces entry mass by 32.02 percent, peak force by 19.15 percent, impulse by 25.49 percent and penetration by 19.20 percent. The benefit is measurable in this full-online comparison, but it is smaller in force than in mass. This experiment is distinct from earlier standard-optimized comparisons with an offline posture target.

![Perpendicular baseline: standard first-collision plot generated from the selected same-study baseline log.](report_assets/perpendicular_baseline_collision.png)

![Perpendicular full-online controller: standard first-collision plot generated from the selected 4/12 log.](report_assets/perpendicular_full_online_collision.png)

Both trials finish their planned approximately 0.57 s schedule, with complete first impacts. The current comparison helper returns `full_criterion_met: true` for this saved pair. This is a sampled acceptance result for these nominal legacy logs, not a validation of every later runtime revision. Their time-origin metadata is absent, and their first saved samples are near 0.007 s.

## 8. Constraints, fallbacks and execution timing

| Optimized run | Safety / tracking / secondary failures | Min margin (rad) | Clipped samples |
| --- | ---: | ---: | ---: |
| Circular | 0 / 0 / 0 | 0.148845 | 0 |
| Perpendicular | 0 / 0 / 38 | 0.151151 | 0 |

The circular optimized run has no failed QP level. The perpendicular optimized run has 38 secondary failures; safety and primary tracking remain accepted. On those updates the implementation uses the fresh primary solution, so the run can complete without uninterrupted mass optimization. Baseline rows have no QP status because no QP is solved.

The logged normalized maximum slack exceeds $10^{-6}$ in approximately 1.95 percent of circular optimized updates and 17.73 percent of perpendicular optimized updates. The denominator is all controller updates, not only impact samples. Estimated acceleration reaches approximately 30.04 rad/s$^2$ in the circular optimized run and 49.55 rad/s$^2$ in the perpendicular optimized run; the corresponding baseline maxima are 23.82 and 43.91 rad/s$^2$. These are finite differences of logged velocities, not exact plant accelerations. The 30 rad/s$^2$ preference is therefore not always respected.

The circular minimum physical margin, 0.14885 rad, briefly lies inside the preferred 0.15 rad band while remaining outside the 0.06 rad hard margin. The perpendicular optimized minimum is 0.15115 rad. Both selected optimized logs respect sampled hard position/velocity limits and report no actuator clipping.

| Run | Update mean / p95 (ms) | Update max (ms) | Missed samples (percent) |
| --- | ---: | ---: | ---: |
| Circular baseline | 0.508 / 0.847 | 22.227 | 26 (0.130) |
| Circular online | 0.958 / 1.342 | 27.782 | 10 (0.050) |
| Perpendicular baseline | 0.416 / 0.618 | 9.761 | 1 (0.177) |
| Perpendicular online | 0.994 / 1.610 | 2.709 | 0 (0.000) |

At requested RTF 0.20, a simulated 1 ms control interval corresponds to a 5 ms wall-clock budget. Matching RTF does not eliminate missed samples or scheduling outliers. The optimized timing distributions include calls above 1 ms, so these simulations do not establish deadline-compliant 1 kHz execution on a physical robot.

The virtual wall applies force only at the fingertip. Its displayed Gazebo wall has no full-body collision geometry. The hand-depth diagnostic reaches approximately 5.36 mm in the circular optimized log and 9.25 mm in the perpendicular optimized log; it is a geometric proxy, not a force on the hand. Position-only tracking, joint limits and between-lap returns do not guarantee orientation or body clearance.

## 9. Conclusions and reproducibility

Both saved full-online comparisons exceed the 30 percent first-contact mass-reduction target at almost unchanged entry speed. Their impact reductions are directly reported rather than inferred from mass alone. The primary tracking request is shared, while the HQP and secondary strategy remain material differences from baseline.

The current circular return policy contains accumulated postural drift and its smooth transition removes the off-contact onsets seen at abrupt RETURN entries in the selected before/after logs. It also changes the complete trajectory schedule. The perpendicular 4/12 result is a useful same-study measurement, with explicit legacy timing provenance and postcontact secondary fallbacks.

No new simulations were required for this report. Measurements are recomputed from the following NPZ files using the repository's analysis helpers:

- **Circular baseline:** `runs/circle_baseline_20261116_095341.npz`
- **Circular online:** `runs/optimized_full_online_circular_20261117_192815.npz`
- **Perpendicular baseline:** `runs/full_online_tuning_20261116/baseline_reference_20261116_151231.npz`
- **Perpendicular online:** `runs/full_online_tuning_20261116/m4_d12_r010_20261116_151407.npz`
- **Perpendicular gain comparison 1:** `runs/full_online_tuning_20261116/original_m2_d24_r010_20261116_151310.npz`
- **Perpendicular gain comparison 2:** `runs/full_online_tuning_20261116/m4_d24_r010_20261116_151348.npz`
- **Perpendicular gain comparison 3:** `runs/full_online_tuning_20261116/m4_d12_r010_20261116_151407.npz`

The abrupt-transition evidence comes from `runs/optimized_full_online_circular_20261117_174953.npz`. Implementation sources are `fr3_sim/controllers/baseline.py`, `baseline_perp.py`, `optimized_hqp.py`, `optimized_hqp_full_online.py`, `optimized_full_online_circular.py`, and the methods `cartesian_tracking.py`, `mass_objective.py`, `joint_constraints.py` and `hierarchical_qp.py`.

The primary report source is `docs/report.md`; original per-run plots and machine-readable measurement summaries are kept beside it in `docs/report_assets/`. `plot_sources.json` records the source plot and checksum for each copied image. The two perpendicular collision plots were generated from their selected NPZ files using the standard plotter. Rebuild in an environment providing pdfLaTeX and its Markdown package:

```bash
python3 docs/build_report.py
```

The delivered `docs/report.pdf` was compiled and visually checked.
