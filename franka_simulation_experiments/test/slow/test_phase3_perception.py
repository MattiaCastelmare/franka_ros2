"""Phase 3: the simulated D455 through the UNCHANGED real_time_distance (plan §8, 3.2-3.4).

SLOW: three Gazebo runs (~25 s wall each), the arm held at Q0 by the scenario
commander, scene ``bare``. Data: the run's bag (/cbf/per_link_distances) and
truth.npz (true control points and gaps at 1 kHz, simexp.truth_geometry).

3.2 Static spheres: reported per-CP surface gap vs geometric truth.
    The engine deliberately reports LESS than the surface gap. It subtracts the
    exclusion-mask dilation, converted to metres at the pixel's depth
    (distance_engine.py:453-466): margin = m_px * Z / fx, with m_px the
    effective full-resolution margin (mask_builder.py:134-136, 260-263),
    body (max(1, 2//4) + 8//4) * 4 = 12 px. That is a known, documented term,
    so the test predicts it: expected = true_gap - 12 * Z / fx.
    Tolerance on the residual, |median| <= 1.0 cm and p95 <= 1.5 cm:
      * grid sampling: pixel_step 6 at Z ~ 1.6-2.1 m is 2.2-2.9 cm between
        samples, so the nearest sample lies <= 2 cm (half-diagonal) from the
        true closest point. On a 6 cm sphere at a 0.45 m gap that adds
        <= ~3 mm to the distance;
      * depth quantisation: 1 mm (16UC1 millimetres);
      * principal point and fx: Gazebo's camera_info drives both rendering
        and back-projection, so they cancel.
    The spheres are placed so that the point nearest to each control point
    faces the camera and is not occluded (scenarios.yaml tests.p3_static).
3.3 H9, robot only: zero obstacle rows; the mask leak (real_time_distance's
    own leak= counter) is reported.
3.4 A sphere at constant velocity gets a confirmed track (track_id != 0,
    frames_seen >= 3) whose v_obs has the right direction and magnitude.
"""
import json
import os
import sys

import numpy as np
import pytest
import yaml

pytestmark = pytest.mark.slow
RESULTS = {}


def _simexp():
    from ament_index_python.packages import get_package_prefix
    sys.path.insert(0, os.path.join(get_package_prefix('franka_simulation_experiments'), 'lib',
                                    'franka_simulation_experiments'))
    sys.path.insert(0, os.path.join(get_package_prefix('franka_simulation'), 'lib',
                                    'franka_simulation'))
    from simexp import analysis, runner, sim_config
    return analysis, runner, sim_config


def _run(name):
    A, runner, sc = _simexp()
    rd = sc.make_run_dir(name=f'phase3_{name}')
    res = runner.run_cell(rd, condition='kf', scenario=name, timeout_s=300)
    assert not res['timed_out'] and not res['leftovers'], res
    man = A.load_json(os.path.join(rd, 'run_manifest_sim.json'))
    ep = man['episode']
    # 2 s of settling inside the episode (LPF, track confirmation) before scoring
    E = A.per_link_entries(os.path.join(rd, 'bag'), ep['t_start_sim'] + 2.0, ep['t_end_sim'])
    return rd, man, E, A.load_truth(rd)


def _save(rd, key, val):
    RESULTS[key] = val
    with open(os.path.join(rd, 'phase3_results.json'), 'w') as f:
        json.dump(RESULTS, f, indent=1, default=float)


@pytest.fixture(scope='module')
def margin_px():
    from ament_index_python.packages import get_package_share_directory
    with open(os.path.join(get_package_share_directory('franka_experiments'), 'config',
                           'fr3_complete.yaml')) as f:
        m = yaml.safe_load(f)['mask']
    ds = max(1, int(m['mask_downsample']))
    return (max(1, int(m['robot_mask_dilate_px']) // ds) + int(m['search_exclusion_extra_px']) // ds) * ds


def _optical():
    from utils import gz_world
    from ament_index_python.packages import get_package_share_directory
    return gz_world.load_optical_extrinsics(os.path.join(
        get_package_share_directory('franka_simulation'), 'config', 'sim',
        'camera_extrinsics_sim.yaml'))


def test_static_spheres_gap_matches_truth(margin_px):
    rd, man, E, tr = _run('p3_static')
    assert margin_px == 12
    R, t_opt = _optical()
    fx = 428.8443603515625
    centres = tr['p'][-1]
    radii = tr['obstacle_radius']
    labels = list(tr['labels'])
    per = {}
    for e in E:
        if not e['valid'] or not np.isfinite(e['d']):
            continue
        dc = np.linalg.norm(centres - e['ph'], axis=1) - radii
        j = int(np.argmin(np.abs(dc)))
        if abs(dc[j]) > 0.02:
            continue                                  # not a point on a sphere
        k = int(np.clip(np.searchsorted(tr['t'], e['t']), 0, len(tr['t']) - 1))
        i = int(np.argmin(np.linalg.norm(tr['cp'][k] - e['pr'], axis=1)))
        assert np.linalg.norm(tr['cp'][k][i] - e['pr']) < 0.01, 'CP not matched to truth'
        z = float((R.T @ (e['ph'] - t_opt))[2])
        true = float(tr['gap'][k, i, j])
        resid = e['d'] - (true - margin_px * z / fx)
        per.setdefault((labels[i], j), []).append((resid, e['d'] - true, true, z))
    assert {j for _, j in per} == {0, 1, 2}, f'not every sphere was seen: {sorted(per)}'
    rows = {f'{lab}->sphere{j}': dict(n=len(v), true=float(np.median([x[2] for x in v])),
                                     raw_bias=float(np.median([x[1] for x in v])),
                                     residual_median=float(np.median([x[0] for x in v])),
                                     residual_p95=float(np.percentile(np.abs([x[0] for x in v]), 95)),
                                     Z=float(np.median([x[3] for x in v])))
            for (lab, j), v in per.items()}
    allr = np.array([x[0] for v in per.values() for x in v])
    out = dict(rows=rows, margin_px=margin_px, residual_median=float(np.median(allr)),
               residual_p95=float(np.percentile(np.abs(allr), 95)), n=int(allr.size))
    _save(rd, 'static', out)
    assert abs(out['residual_median']) <= 0.010, out
    assert out['residual_p95'] <= 0.015, out


def test_robot_only_scene_has_no_obstacle_rows():
    A, _, _ = _simexp()
    rd, man, E, tr = _run('p3_robot_only')
    rows = [e for e in E if e['valid'] and np.isfinite(e['d']) and e['d'] <= 0.7]
    leak = []
    for ln in A.ros_log_lines(rd, 'leak='):
        try:
            leak.append(int(ln.split('leak=')[1].split()[0]))
        except (ValueError, IndexError):
            pass
    out = dict(n_entries=len(E), n_rows=len(rows),
               leak_px_per_frame=dict(n_lines=len(leak), mean=float(np.mean(leak)) if leak else None,
                                      max=int(max(leak)) if leak else None))
    _save(rd, 'robot_only', out)
    assert not rows, f'obstacle rows in a robot-only scene: {rows[:5]}'


def test_moving_sphere_gets_a_confirmed_track_with_the_right_velocity():
    rd, man, E, tr = _run('p3_moving')
    good = []
    for e in E:
        if e['track_id'] == 0 or e['frames_seen'] < 3:
            continue
        k = int(np.clip(np.searchsorted(tr['t'], e['t']), 0, len(tr['t']) - 1))
        c, v_true = tr['p'][k][0], tr['v'][k][0]
        if abs(np.linalg.norm(e['ph'] - c) - tr['obstacle_radius'][0]) > 0.03:
            continue
        good.append((e['t'], e['v'], v_true, e['n']))
    assert good, 'the moving sphere never had a confirmed track'
    V = np.array([g[1] for g in good])
    VT = np.array([g[2] for g in good])
    err = np.linalg.norm(V - VT, axis=1)
    nv = np.array([g[3] @ g[1] for g in good])
    nvt = np.array([g[3] @ g[2] for g in good])
    # The barrier consumes max(n̂ᵀv, 0): the sign that matters is APPROACHING
    # (n̂ᵀv_true > 0) being reported as approaching. A receding obstacle read
    # as approaching only tightens the barrier. Both are reported; the
    # approaching one is the acceptance criterion.
    app, rec = nvt > 0.05, nvt < -0.05

    def agree(m):
        return float(np.mean(np.sign(nv[m]) == np.sign(nvt[m]))) if m.any() else None
    out = dict(n=len(good), first_confirmed_t=float(good[0][0] - man['episode']['t_start_sim']),
               v_err_median=float(np.median(err)), v_err_p90=float(np.percentile(err, 90)),
               speed_ratio_median=float(np.median(np.linalg.norm(V, axis=1) / np.linalg.norm(VT, axis=1))),
               sign_agreement_all=agree(app | rec), sign_agreement_approaching=agree(app),
               sign_agreement_receding=agree(rec), n_approaching=int(app.sum()),
               n_receding=int(rec.sum()))
    _save(rd, 'moving', out)
    assert out['v_err_median'] < 0.10, out
    assert 0.8 < out['speed_ratio_median'] < 1.2, out
    assert out['n_approaching'] > 50 and out['sign_agreement_approaching'] > 0.95, out
