# IRIM2: control corrections and reproducible SIM checks

This change addresses four findings from `experiments_irim2.zip`: inconsistent
control-point derivatives, inconsistent governed trajectory derivatives,
conflicting state/slew limits and nominal-preserving QP fallback, and missing
reproducible comparisons. It does not alter the hardware gains or run the robot.

## Geometry and derivatives

`distance_utils.control_point_specs` now defines the same CP geometry for
perception and filtering. For an interpolated point,
`p = (1-t)*p_start + t*p_end`, its Jacobian and time derivative are the same
weighted sums of the endpoint derivatives. A fingertip offset is differentiated
as a point fixed to the final frame. Multi-obstacle labels share their CP
Jacobian; frame derivatives are cached within each kinematic update. An
alternating CPU-time benchmark (100 repeats, 100 calls each) reduced the exact
point-derivative cost from 56.4 to 19.7 microseconds. This is a local cost check,
not a claim that the full ROS executor sustains 300 Hz.

The rigid-point derivative includes the formerly omitted term
`-skew(omega cross r) @ J_angular`. The ROS launch passes the perception geometry
configuration to the filter; CP labels and the distance message layout stay the
same. Synthetic rigid-point callers retain their existing explicit geometry.

The numerical replay of **48 recorded configurations and 11 CPs** gave:

| Check | Maximum absolute error |
|---|---:|
| Jacobian vs central differences of perception CP positions | 1.61e-10 m/rad |
| Jacobian time derivative vs directional differences | 2.15e-10 |
| Drift acceleration `Jdot*qdot` | 3.99e-10 m/s² |

The recovered pre-correction check found Jacobian column errors up to
0.0587 m/rad and an omitted drift term up to 0.3285 m/s². These are derivative
checks, not an attribution of every movement in the recorded experiment.

## Governed timing

`GovernedTimingLaw` integrates the virtual clock and converts derivatives with
`ds/dt = sigma * ds/du` and
`d²s/dt² = sigma² * d²s/du² + sigma_dot * ds/du`.
Sigma changes at at most `governor_rate_limit = 2.0 /s`; an ISO latch holds the
clock immediately, and release resumes through the bounded ramp. The underlying
linear soft start, exponential and trapezoidal timing laws now integrate their
phase analytically, making their returned derivatives consistent with position.
At half speed the 25 cm circle requests 0.13090 m/s, rather than 0.26180 m/s.
Finite-difference tests cover changing sigma, soft start, hold/release and
subdivision of time steps.

## State limits, slew and fallback

Priority is actuator/state box, then compatible command continuity, then the
QP objective with its priced soft barrier families. If a state interval and
slew interval are disjoint, the command is pinned to the nearest **state** edge;
continuity yields for that joint. The conflict count is recorded. This removes
the old case where a safety interval [-6,-4], previous command +4 and delta 0.5
produced +3.5 outside the state interval; it now produces -4.

If full OSQP setup/update/solve raises, returns a nonfinite iterate or reports
an unacceptable status, the node discards the nominal objective and immediately
uses `clip(-k_brake*qdot, state_lb, state_ub)`, bypassing slew. It resets the
failed warm start, raises the existing fault and records fallback level 2.
Historical level 1 and the generic `box_only_solve` helper remain for reading old
logs and for the independent ISO stop; level 1 is no longer a nominal-following
fallback in the production QP path. Fault-injection tests exercise the node's
actual `_solve` method.

The state box is bounded by available acceleration. From an already nonviable
state it cannot promise instantaneous recovery of velocity or position. Torque
rate limits and RT feedback also affect realised motion; neither a successful
QP nor zero firmware flags constitutes a separation guarantee.

## Recorded evidence

The first eleven `/NS_1/cbf_status` fields retain their indices. The appended
schema is defined in `utils/cbf_diagnostics.py` and copied into each SIM manifest:

| Indices | Meaning |
|---|---|
| 11 | fallback level: 0 full QP, 2 braking |
| 12 | joints whose state/slew boxes conflict |
| 13–18 | solve wall ms, ROS tick gap, state receipt age, used depth capture age, latest distance receipt age, snapshot age |
| 19–24 | slack per obs/sc/qlim/sing/cap/spd family |
| 25–30 | maximum positive row residual per family before slack |
| 31–36 | maximum positive row residual per family including slack |
| 37 | constraint build wall ms |

Capture and build timestamps travel with the immutable constraint snapshot, so
a newer queued perception frame cannot make the geometry actually used by the
QP look fresher. Residuals are evaluated on the emitted, clipped acceleration;
they have different units across families. Missing ages are NaN, not zero.
These timings measure through QP publication, not an instrumented RT actuation
timestamp. The bags retain torque commands and 1 kHz joint states for examining
subsequent execution; no unmeasured RT or physical sensor delay is called zero.

Each run saves the seed, resolved obstacle laws, control/perception/controller
configurations, actual imported Python source copies and SHA-256 hashes,
working-tree patch, bag, ground truth and reflex summary. Corrected nodes also
save their effective ROS parameters and CBF configuration in runtime JSON.
Tracking is scored from simultaneous `PointStamped.header.stamp` pairs; the
commander CSV uses a different node-relative clock and is retained only as a
legacy diagnostic.

## SIM protocol and results

The catalog `franka_simulation_experiments/config/control_regression_scenarios.yaml`
separates clear motion, a static obstacle, a 0.3 m/s hand proxy pursuing the
flange, and a 0.37 m stick proxy made of seven overlapping spheres translating
together. The latter is an approximation, not a measured human/stick replay.
All cells use the same DIAG camera, bare scene, Kalman-tracker condition and
circular task. Seeds 0, 1, 2 change frame/start phase. The saved adapter configuration has
noise, dropout and explicit sensor delay disabled; these trials do not reproduce
the physical camera acquisition delay or independent human motions.

The baseline controller is from commit `976b997`; the corrected controller is
frozen separately by source snapshot. Each cell starts a fresh Gazebo with its
own partition and ROS domain. Only incomplete startup failures are retried and
archived. Contacts, reflexes and poor timing in completed episodes are retained.
The recorded 240 Hz QP-rate check is an evidence-quality indicator, not a claim
that a lower rate is safe.

The final comparison contains **24 completed cells**, 12 baseline and 12
corrected. Each variant has exactly one source fingerprint across all cells.
The earlier exact-derivative candidate is retained separately in `paired/`;
`paired_final/` uses the cached final implementation and references the same
baseline recordings. All recorded static/oscillating obstacle-law checks pass
at less than 1 micrometre maximum position error.

| Scenario | Reflex runs, before → after | Minimum true CP gap [mm], before → after | Median TCP p95 [mm], before → after |
|---|---:|---:|---:|
| Clear | 0/3 → 0/3 | — → — | 19.7 → 36.2 |
| Static | 2/3 → 0/3 | 19.1 → 232.0 | 716.9 → 465.3 |
| Approach | 2/3 → 0/3 | -75.9 → -67.5 | 1040.7 → 1152.4 |
| Stick | 0/3 → 0/3 | 247.1 → 252.4 | 431.3 → 407.3 |

The corrected controller has **zero reflex-flagged runs out of 12**, against
four out of 12 in the baseline. The static cases preserve at least **232 mm**
of true CP clearance in this sample. The advancing hand proxy still intersects
the CP capsule model in **3/3 runs**, just as the baseline does: the separation
requirement is **not met** there. This is an intrusive pursuer that never yields;
zero joint/torque reflexes do not make those episodes collision-free.

None of the 24 cells meets the separately recorded 240 Hz QP quality check.
Corrected achieved rates span **43.4–216.8 Hz** of simulation time.
Across corrected runs, used-geometry capture-age p95 ranges from
**106 to 132 ms**, with worst observed age
**197 ms**. These scheduling and freshness limits prevent
attributing all changes in tracking to the mathematical corrections or making
a nominal-300-Hz safety claim. Clear-scene tracking also has a larger p95 in
this small sample; do not describe the change as a universal tracking gain.

The completed tests establish the derivative fixes, conflict policy and fault
response, plus the measured SIM behaviour. They do **not** establish hardware
acceptance, persistent separation against an approaching person, or a certified
safety function. No controller was run on hardware.

Artifacts (git-ignored, under `franka_simulation_experiments/sim_runs/irim2_control/`):

- `paired_final/summary.json` and `comparison.png`: full comparison and figure.
- `paired_final/{baseline,corrected}/control_*__s*/`: bags, truth, manifests,
  source/configuration snapshots, runtime parameters and scored results.
- `controllers/`: persistent baseline, initial-exact and corrected Python sources.
- `numerical/`: the recorded-state derivative replay and CPU benchmark.
- `integration/`: additional Gazebo integration tests.

Validation counts are populated after the integration suite completes.


## Reproduce

Inside the sourced ROS container, run each variant with its chosen controller
package directory on PYTHONPATH (the script propagates it to every child):

```bash
python3 franka_simulation_experiments/scripts/run_control_regression.py \
  --root franka_simulation_experiments/sim_runs/irim2_control/paired_final \
  --variant corrected \
  --controller-source franka_simulation_experiments/sim_runs/irim2_control/controllers/corrected \
  --domain 151
```

Use `--variant baseline` with the saved `controllers/baseline` package directory
for the saved pre-correction package. `--rescore` reads existing evidence without
starting Gazebo. An optional `--filter-cpu` selects the existing per-partition
CPU-pinning facility; keep it identical for both variants in a comparison.

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m pytest \
  franka_experiments/test franka_simulation_experiments/test \
  -m 'not slow' -q
python3 -m pytest franka_simulation/test/sim -q
python3 franka_experiments/scripts/check_control_derivatives.py \
  franka_simulation_experiments/sim_runs/irim2_control/numerical/kinematic_inputs.npz \
  --output franka_simulation_experiments/sim_runs/irim2_control/numerical/derivative_check.json
```
