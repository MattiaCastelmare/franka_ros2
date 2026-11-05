# Flacco depth-space distance (gray area + finite pixel) — implementation and Gazebo A/B

*2026-11-04. Data: `franka_simulation_experiments/sim_runs/flacco_depth_space_20261104/`
(videos, `results.md`, `results.json`, `map.png`, `analyze.py`). n = 1 run per cell.*

## 1. What was implemented

**Source.** Flacco et al., *A Depth Space Approach for Evaluating Distance to Objects* (2015 preprint, §3.3: Eq. 7–9 and
Algorithm 1). The ICRA 2012 paper contains the gray-area rule only, without pixel size.

**Gray area.** Everything behind an observed pixel along its ray is occluded and counted as obstacle. If the obstacle is
in front of the control point (`d_o < d_p`), the paper uses `d_o = d_p`.

**Finite pixel (frustum).** The pixel is a square, not its centre. The sub-pixel point nearest to the control point's
projection is used: `ô = clamp(p, pixel edges)` (Eq. 8).

**Distance and normal.** The distance is `D = ‖O* − P‖` with `O* = ((ô_x−c_x)d_o/f_x, (ô_y−c_y)d_o/f_y, d_o)`, which is
Eq. 7 rearranged. The CBF normal is the unit vector from `O*` to the control point.

**Where the code lives.**
- `franka_experiments/utils/distance_engine.py`: the function `depth_space_obstacle_points()` and Step 6 of
  `DistanceEngine.compute`.
- Two flags in `config/fr3_complete.yaml` under `distance:`, `depth_space_gray_area` and `depth_space_finite_pixel`.
  Both are **`false` by default**.
- Simulation condition `minimal_l1_ds` = `minimal_l1` + both flags. The CBF, QP, tracking, masks, filtering and the
  cloud handed to the tracker are all unchanged.

**Two literal-paper choices.**
- The paper's pixel `[ō, ō+1]` (truncated coordinates) is written as `[u−½, u+½]`, because our camera model has pixel
  centres at integer `u`. The square is the same.
- Each decimated sample (`pixel_step: 6`) gets its own 1-pixel frustum. The skipped pixels are not covered: covering
  them would be an extra feature.

**Verification.**
- With the flags off, the engine is bit-identical to the committed version: 660/660 control-point results across 60
  random frames, including multi-cluster rows and filter state.
- `test/test_distance_depth_space.py`: 10 tests pass. They cover the paper's two cases, the clamp, flags-off identity,
  and a control point behind the camera.
- Full suite: 1266 passed / 43 failed. The same 43 failures existed before this change.

## 2. Consistency with the CBF's derivative assumption

The CBF uses `ḣ = nᵀ(J_p q̇ − v_o)`, i.e. it assumes `∂D/∂P = n`. I compared finite-difference gradients with the
published normal on 3000 random geometries:

| method | non-occluded pixels | **occluded pixels** (`d_o < d_p`) |
|---|---|---|
| original (pixel centre) | exact (0°) | exact (0°) |
| finite pixel only | exact (p95 0°; up to 27° when the control point projects inside the pixel) | p95 0°, max 18° |
| gray area (± finite pixel) | exact | **median 8°, p95 21°, max 36°** |

The paper's occluded distance is a lateral offset taken at constant depth, not perpendicular to the ray. So in the
gray area the normal fed to the CBF is not the true gradient: ḣ is mis-estimated by up to `D·|∇D − n|`. The size of
the error grows with the ray's off-axis angle.

## 3. Gazebo results

**Setup:** DIAG lab, camera + kf tracker, CBF `minimal_l1` in both arms, 100 Hz (achieved 94–100 Hz), seed 0, 30 s,
runs interleaved. All 14 runs started on the first attempt.

| scenario | arm | min true gap | time < 10 cm | contact | phantom ticks¹ | EE err p95 | `dist` ms p50 |
|---|---|---|---|---|---|---|---|
| P1 static, front right | original | 6.3 cm | 8.1 s | 0 | 6.8 % | 10 cm | 13.0 |
| | depth space | **24.6 cm** | **0** | 0 | 10.2 % | 41 cm | 13.7 |
| P2 static, front left | original | 3.9 cm | 7.9 s | 0 | 5.2 % | 15 cm | 10.4 |
| | depth space | 4.6 cm | **0.4 s** | 0 | 16.9 % | 35 cm | 12.8 |
| P5 static, on path | original | 5.3 cm | 6.6 s | 0 | 10.0 % | 25 cm | 9.4 |
| | depth space | **23.7 cm** | **0** | 0 | 27.2 % | 50 cm | 12.0 |
| lateral ball, 0.3 m/s | original | 15.6 cm | 0 | 0 | 1.1 % | 19 cm | 10.5 |
| | depth space | 28.0 cm | 0 | 0 | 21.1 % | **75 cm, task stalled²** | 14.1 |
| O1 occluder on camera ray | original | 12.8 cm | 0 | 0 | 1.7 % | 7 cm | 10.8 |
| | depth space | 24.6 cm | 0 | 0 | 19.8 % | 53 cm | 16.1 |
| P7 robot hides ball | original | 23.6 cm | 0 | 0 | 17.2 % | 40 cm | 56.2³ |
| | depth space | 24.7 cm | 0 | 0 | 19.3 % | 40 cm | 14.1 |
| static + moving occluder | original | 4.0 cm | 8.3 s | 0 | 20.1 % | 23 cm | 12.1 |
| | depth space | **15.7 cm** | **0** | 0 | 36.2 % | 61 cm | 12.9 |

Notes:
1. Ticks where the filter perceived < 10 cm while the true gap was > 15 cm.
2. The robot completed 0.56 laps, then stopped progressing; the original completed 3.9 laps.
3. Single-run timing outlier. Per-frame `dist` cost is otherwise +1 to +5 ms (about +10–40 %), well inside the 33 ms
   frame budget.

### What happened

1. **Margin violations almost disappear.**
   - Time inside 10 cm drops to 0 in 5 of 7 scenarios, and to 0.4 s in P2.
   - P2 still dips to 4.6 cm. Its perception error reaches +30 cm optimistic: the ball is *lost*, and neither feature
     can fix a missing detection.
2. **They disappear because the robot stays far away, not because it avoids more precisely.**
   - The minimum gap jumps to about 24 cm, path deviation p95 grows 2–4× (35–75 cm), and phantom readings rise in
     every scenario.
   - The gray area turns everything *behind* an obstacle (from the camera's view) into obstacle. Whenever the ball
     appears in front of the arm in the image, the arm is "inside" its shadow.
   - O1 shows it directly: no avoidance is needed (12.8 cm with the original), yet the depth-space arm deviates 53 cm.
3. **One failure mode is new and serious.** In the lateral run, the ball's shadow covered the elbow and forearm
   (`fr3_link4#0`) while the true gap was about 30 cm.
   - 12–18 constraint rows were "violated" at once, and the command deviated from the nominal by about 9 rad/s².
   - This drove the arm toward a near-singular pose. It stayed stuck there for the rest of the run: singularity row
     active, deviation about 25 rad/s², no task progress.
4. **When the robot occludes the ball (P7), there is no difference.** That is expected: robot pixels are removed before
   the distance is computed, so the paper's gray area does not cover what is hidden behind the robot.

## 4. Limitations

- **One seed per cell.** This shows behaviour, not rates.
- **Simulated sensor.** The depth camera has no reflections, and calibration error is modelled only as noise.
- **Untuned gains.** The CBF gains and d_safe were tuned for the original distance. The gray area adds its own
  conservatism on top of d_safe and was not retuned. The paper used it with a repulsive-vector controller, not a HOCBF.
- **Approximate normal.** The occluded-case normal is approximate (§2), so the CBF's ḣ is inexact exactly where the
  gray area acts.
- **Decimation.** Decimated samples leave 5 of every 6 pixels uncovered by any frustum, so the finite-pixel feature
  covers only the sampled pixels.
- **Not tested separately.** The two features were not run in Gazebo one at a time. The offline analysis suggests the
  finite pixel alone is small and benign, and that the gray area drives everything above.

## 5. Recommendation

**Do not enable the depth-space distance as a whole.**
- In this stack the gray area trades the original's margin violations for severe over-conservatism: 2–4× path
  deviation, many phantom readings, and one stalled task.
- It also gives the CBF an inexact normal (p95 21°) in exactly the occluded cases it is meant for.
- It does not address the largest perception failure observed (P2, ball lost: +30 cm optimistic).

**If you want to continue with it, test the two features separately before deciding:**
1. **Finite pixel only.** It is cheap, its normal is exact in almost all cases, and its effect is to shorten distances by
   at most half a pixel footprint (≈2 mm at 1.8 m). It is unlikely to matter much, but it is safe to evaluate.
2. **Gray area.** Only with a bounded occlusion depth, or only for control points actually hidden in the image. Both are
   changes to the paper's method, which is why they were not implemented here.

The violations seen with the original method come mainly from perception (lost and optimistic detections, phantoms) and
the delay chain (see `sim_runs/ball_position_sweep_20261104/README.md`). Those remain the place to look.
