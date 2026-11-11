# Impedance/HQP: measured development comparison

With the approved acceleration cap of **30 rad/s²**, the new controller
completes the requested 20 s nominal experiment and meets the measured
comparison criterion: **31.5573% lower effective mass before impact** at
practically identical normal speed, with no saved position, velocity,
acceleration-cap or torque-limit violations. All primary and secondary QPs
are accepted. These are results for this scenario and saved samples, not a
continuous-time safety certificate or a guarantee for other objects.

## Scenario and logs

Both experiments use the existing nominal Gazebo model and virtual wall:
initial joint state from the baseline YAML, 4 cm circle, 4 s period, 1 s
startup ramp, wall at x = 0.62 m, stiffness 20000 N/m and damping 15 N s/m.
Control is 1000 Hz, requested real-time factor 0.20, duration 20 s, GUI off,
observer enabled, no realism perturbations, no stop at detachment.
No wall position, stiffness, collision speed or initial configuration was
changed to improve the result.

- Baseline: `runs/circle_baseline_hqp_comparison_20261111_083032.npz`.
- Impedance/HQP, 30 rad/s²: `runs/circle_impedance_hqp_20261111_084724.npz`.
- Earlier 8 rad/s² trial: `runs/circle_impedance_hqp_20261111_083343.npz`.
- Machine-readable report: [impedance_hqp_comparison.json](impedance_hqp_comparison.json).

The candidate uses the previously computed optional low-mass posture and
online mass gain 0.1. The joint-position margin is 0.15 rad and the
acceleration cap is 30 rad/s² on all seven joints. Cartesian
stiffness is 600 N/m and damping 120 N s/m on all three axes. Nullspace
posture/damping gains are 25 s⁻² and 10 s⁻¹. The acceleration cap and the
residual uncertainty reserve are experiment parameters, not physical FR3
datasheet bounds or certified estimates of an arbitrary contact.

## Measurements

The pre-contact quantities use the last sample strictly before the first
recorded positive contact force. Tracking RMS is time weighted using the
saved timestamps. Both runs last 20 s. Contact-force metrics cover only the
first compression-release cycle, ending at the first recompression.

| Quantity | Baseline | Impedance/HQP |
|---|---:|---:|
| Recorded contact onset | 1.834 s | 1.834 s |
| Effective mass before contact | 7.187720 kg | 4.919467 kg |
| Normal velocity before contact | 0.05443797 m/s | 0.05443565 m/s |
| Normal incoming kinetic energy | 10.65038 mJ | 7.28878 mJ |
| First-cycle peak force | 22.09089 N | 18.78576 N |
| First-cycle normal impulse | 1.118939 N s | 0.850331 N s |
| First-cycle maximum penetration | 1.103042 mm | 0.937681 mm |
| First-cycle duration | 0.089822 s | 0.077699 s |
| Pre-contact position-error RMS | 0.008062 mm | 0.009997 mm |
| Full-run position-error RMS | 9.01034 mm | 8.11267 mm |
| Full-run maximum position error | 21.68653 mm | 19.44801 mm |
| Minimum physical joint margin in saved motion | 0.358029 rad | 0.158191 rad |
| Saved position/velocity limit violations | 0 / 0 | 0 / 0 |
| Runner torque-saturation samples | 0 | 0 |
| Recorded RUN duration | 20.000 s | 20.000 s |
| First compression-release cycle complete | Yes | Yes |

The measured pre-impact mass reduction is **31.5573%**, with an entry-speed
difference of **0.00426%**. First-cycle peak force decreases by **14.9615%**,
impulse by **24.0056%**, penetration by **14.9914%** and incoming normal
kinetic energy by **31.5632%**. The JSON reports both `target_metrics_met`
and `full_criterion_met` as true, with no unmet checks.

The candidate's maximum force across the whole 20 s run is approximately
19.04 N; it is distinct from the 18.79 N peak of the first selected cycle.
Contact entries recur at 1.834, 5.834, 9.834, 13.834 and 17.834 s, with
pre-contact mass approximately 4.91947 kg each time.

The candidate never crosses its 0.15 rad protected position margin in the
saved motion. Its nominal motor torque, including plugin gravity
compensation, remains below 41.69% of the configured limits. The largest
absolute joint-velocity acceleration estimate is 12.59395 rad/s², below the
30 rad/s² cap. This uses nonuniform finite differences of saved velocities;
it is not an exact instantaneous measurement during impact.

Both HQP levels report `solved` on all 19964 controller calls. The maximum
logged constraint/equality violation is 1.1331e-5, below the configured
2e-4 hard-constraint acceptance tolerance.

## Tracking during contact and recovery

Tracking remains active throughout the run, including contact. In the
first reference-wall passage, 1.834-3.166 s, the tool moves downward by
approximately 6.91 cm while normal motion yields to the wall. Normal
position error grows to approximately 19.45 mm because the desired circle
extends into the wall; this is the expected compliance error, not accurate
tracking through the obstacle. Tangential tracking remains active.

After the first detachment at approximately 2.982 s, the free-motion
interval 3.5-4.5 s has position-error RMS 0.05193 mm and maximum 0.18011 mm.
The same recovery recurs at 7.5-8.5 s. There is no stop latch, retreat
state or manual restart of the reference.

## Earlier 8 rad/s² trial

The earlier configuration stopped at 1.840 s. Its pre-impact mass was
4.919466 kg, but the first impact was incomplete, so it did not satisfy
the full criterion. At its final update, the error reserve for joint 6
reaches approximately 7.2495 rad/s². After tightening acceleration and
position constraints for this reserve, its actuator-acceleration bounds are
approximately:

\[
u_6 \ge -1.25579,\qquad u_6 \le -1.29567
\quad[\mathrm{rad/s^2}].
\]

Their intersection is empty. This is a contradiction in the requested
robust constraint set, rather than a solver iteration failure, a joint
position safety stop or missing state samples. The logged termination cause
identifies the joint and both bounds. Bounded damping at termination is a
fallback, not a valid HQP solution or a proof that acceleration caps remain
satisfied thereafter.

The uncertainty reserve uses the residual's observed rate and filter time
constant. It is deliberately not deconvolved into a contact-force
measurement. Its conservatism can cause infeasibility even before a large
measured acceleration occurs. Conversely, it cannot certify acceleration
limits during every unknown impact: the residual is delayed and filtered.

The user approved changing only the new controller's acceleration cap from
8 to 30 rad/s². The position margin, uncertainty model, Cartesian gains,
mass gain, wall and reference were retained. The larger cap allows the
verified QP solution during impact; it does not change the physical wall
or impose a favorable contact trajectory. The successful trial still has
a peak error reserve of approximately 16.42 rad/s² on joint 6, illustrating
why the filtered-estimate reserve can dominate the former 8 rad/s² cap.

## Timing

| Measurement | Baseline | Impedance/HQP |
|---|---:|---:|
| Controller wall time, mean | 0.326 ms | 0.788 ms |
| Controller wall time, p95 | 0.429 ms | 1.278 ms |
| Controller wall time, maximum | 2.562 ms | 20.935 ms |
| Calls over nominal 1 ms period at RTF = 1 | 95 / 19996 (0.475%) | 4182 / 19964 (20.948%) |
| Missing 1 kHz plant samples | 4 (0.020%) | 36 (0.180%) |

The 1 ms computation comparison assumes RTF = 1; these runs requested
RTF = 0.20. It is separate from missed simulated-time samples. The
candidate has isolated computation spikes and a maximum saved plant-sample
gap of 6 ms, below the configured 10 ms gap fault threshold. The first
impact interval contains a maximum 2 ms sample gap. Timing covers
`Controller.update`, not the full ROS callback. This controller does not
demonstrate reliable 1 kHz execution at RTF = 1.

An observer threshold event occurs at 0.003 s while true contact force is
zero: it is a startup transient. The controller excludes the residual from
constraint predictions until its 0.2 s validity warmup completes and does
not react to the detection event. This trial therefore does not establish
false-positive-free collision detection. The true contact signal is used
only for offline metrics, never as an input to this controller.

## Reproduction

The README and progress document contain launch, plot and comparison
commands. For these exact logs, after building and sourcing the package:

```bash
cd /ros2_ws/src/fr3_student_sim
ros2 run fr3_student_sim plot_run \
  runs/circle_impedance_hqp_20261111_084724.npz --no-show
ros2 run fr3_student_sim compare_runs \
  runs/circle_baseline_hqp_comparison_20261111_083032.npz \
  runs/circle_impedance_hqp_20261111_084724.npz \
  --output runs/impedance_hqp_comparison.json
```

The three candidate PNGs were generated. The logs and PNGs remain in the
ignored `runs/` directory; this report and its JSON retain the numerical
results if those local artifacts are removed. No permanent test files were
added. This comparison concerns the complete controllers; it does not
isolate the causal contribution of mass reduction from the change in
Cartesian impedance and constraint handling.

The natural-inertia impedance law follows the local Robotics 2 lecture 15,
slide 13. Slide 14's qualification about variable inertia is retained: no
general passivity proof is claimed for this sampled HQP implementation.
OSQP's matrix updates and warm starts follow its
[official Python interface](https://osqp.org/docs/interfaces/python.html).
