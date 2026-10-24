"""Estimator vs truth on a Gazebo run (spec Phase 5.1). Same definitions as compare_vobs.py.

Per control point and per perception frame (/cbf/per_link_distances in the
run's bag, rank-0 entries, labels from ``perception_msgs.labelled_links``):

* ``v_true = n̂ᵀ v_obstacle`` of the obstacle the entry's closest_point_human
  lies on (within r + 0.05 m of its true centre, compare_vobs' ``on_sphere``),
  from the 1 kHz truth poses at the frame's capture stamp;
* ``v_trk = n̂ᵀ v_track`` when the entry carries a track (compare_vobs (b));
* ``v_res``: the shipped residual estimator, ``ConstraintBuilder._obstacle_speed``,
  called on its own code with the entry's label, gap, stamp, ``aᵀq̇`` from the
  true joint state and n̂, the arguments of its call site
  (cbf_state_rows.py:2048). The builder is a REAL one with the run's flags
  (cbf_scenarios.Filter(P).builder). compare_vobs' ``make_residual_estimator``
  cannot be used: its ``__new__`` shim sets three attributes and the current
  ``_obstacle_speed`` reads seven (AttributeError ``_vres_dt_min``);
* ``sigma = sqrt(n̂ᵀ P_vv n̂)`` from the entry's velocity covariance.

Metrics, each over the on-obstacle samples (``cross_correlation_lag``,
``static_windows`` and ``rms`` are compare_vobs' functions, imported):

* lag [s] (POSITIVE = the estimate LEADS the truth), bias, RMS: per CP with
  >= 30 samples, then the median over CPs (lag) and pooled (bias, RMS); plus
  ``pooled_*`` over the on-obstacle samples of EVERY CP (fast passes leave few
  samples per CP);
* noise floor: RMS of the estimate on compare_vobs' static windows (gap steady
  to 4 mm and ‖q̇‖ < 0.05 over 9 frames) where the obstacle is truly static;
* time from the first row on an obstacle to its first tracked row and to the
  first row the CBF trusts (frames_seen >= the CBF's gate, ball_throw_eval's
  ``_trust_frames``);
* identity: distinct track ids on one obstacle, and switches of the per-frame
  majority id;
* sigma calibration: share of |v_trk − v_true| <= k·sigma, k = 1, 2, 3, on
  trusted rows (Gaussian: 0.683 / 0.954 / 0.997).

No rclpy node; reading the bag needs the ROS Python message packages.
"""
from __future__ import annotations

import os
import sys
from collections import Counter, defaultdict

import numpy as np

ON_OBSTACLE_M = 0.05          # compare_vobs: radius + 0.05
MIN_SAMPLES = 30              # compare_vobs report(): n < 30 -> skipped
POOLED_MIN = 10               # pooled over all CPs: at least one frame per ~CP
GAUSS = {1: 0.683, 2: 0.954, 3: 0.997}


def fe_scripts():
    here = os.path.dirname(os.path.realpath(__file__))
    for cand in (os.path.join(here, '..', '..', '..', 'franka_experiments', 'scripts'),
                 '/ros2_ws/src/franka_experiments/scripts'):
        cand = os.path.normpath(cand)
        if os.path.isfile(os.path.join(cand, 'compare_vobs.py')):
            if cand not in sys.path:
                sys.path.insert(0, cand)
            return cand
    raise ImportError('franka_experiments/scripts not found')


def _cv():
    fe_scripts()
    import compare_vobs
    return compare_vobs


def control_params() -> dict:
    import yaml
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(fe_scripts(), '..', 'config', 'fr3_control.yaml')
    if not os.path.isfile(path):
        path = os.path.join(get_package_share_directory('franka_experiments'), 'config',
                            'fr3_control.yaml')
    with open(path) as f:
        return yaml.safe_load(f)['params']


def trust_frames(rate_hz: float) -> int:
    """The CBF's frames_seen gate at this frame rate (ball_throw_eval._trust_frames)."""
    fe_scripts()
    import ball_throw_eval
    return int(ball_throw_eval._trust_frames(rate_hz))


# ── the series ───────────────────────────────────────────────────────────────

def _stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def read_run(run_dir: str) -> dict:
    """Bag topics the metrics need, as arrays."""
    from franka_msgs.msg import MultiLinkDistance
    from geometry_msgs.msg import PoseArray
    from sensor_msgs.msg import JointState

    from . import analysis as A
    b = A.read_bag_topics(os.path.join(run_dir, 'bag'), {
        '/cbf/per_link_distances': MultiLinkDistance,
        '/sim/truth/obstacle_poses': PoseArray, '/sim/truth/obstacle_poses_vel': PoseArray,
        '/NS_1/franka/joint_states': JointState})

    def poses(msgs):
        t = np.array([_stamp(m.header) for _, m in msgs])
        P = np.array([[[p.position.x, p.position.y, p.position.z] for p in m.poses]
                      for _, m in msgs])
        return t, P
    tp, P = poses(b['/sim/truth/obstacle_poses'])
    tv, V = poses(b['/sim/truth/obstacle_poses_vel'])
    arm = [f'fr3_joint{i}' for i in range(1, 8)]
    js_t, js_q, js_v = [], [], []
    for _, m in b['/NS_1/franka/joint_states']:
        d = {n: i for i, n in enumerate(m.name)}
        if all(n in d for n in arm):
            js_t.append(_stamp(m.header))
            js_q.append([m.position[d[n]] for n in arm])
            js_v.append([m.velocity[d[n]] for n in arm])
    return dict(dist=[m for _, m in b['/cbf/per_link_distances']], tp=tp, P=P, tv=tv, V=V,
                js_t=np.array(js_t), js_q=np.array(js_q), js_v=np.array(js_v))


def _at(t_grid, t):
    i = int(np.searchsorted(t_grid, t))
    if i <= 0:
        return 0
    if i >= len(t_grid):
        return len(t_grid) - 1
    return i if abs(t_grid[i] - t) < abs(t_grid[i - 1] - t) else i - 1


def make_residual(condition: str = 'kf', sets: dict | None = None):
    """A real ConstraintBuilder with the condition's flags; its _obstacle_speed is the estimator."""
    from . import conditions, offline
    from .emulation import _cbf_scenarios
    C = _cbf_scenarios()
    over = offline.overrides(conditions.resolve(condition, sets), C.shipped_params())
    return C.Filter(C.shipped_params(**over)).builder


def build_series(data: dict, radii, *, t_from=-np.inf, t_to=np.inf, kin=None,
                 residual=None, condition: str = 'kf', sets: dict | None = None) -> dict:
    """compare_vobs-format series {label: {field: list}}, plus 'obs' (obstacle index)."""
    from franka_experiments.utils.perception_msgs import labelled_links
    if kin is None:
        import pinocchio as pin
        from franka_experiments.utils.kinematics import CBFKinematics, build_urdf_no_hand
        kin = CBFKinematics(pin.buildModelFromUrdf(build_urdf_no_hand()))
    if residual is None:
        residual = make_residual(condition, sets)
    radii = np.asarray(radii, float)
    series = defaultdict(lambda: defaultdict(list))
    for msg in data['dist']:
        t_cap = _stamp(msg.header)
        if not (t_from <= t_cap <= t_to) or not msg.links or not len(data['js_t']):
            continue
        j = _at(data['js_t'], t_cap)
        q, qdot = data['js_q'][j], data['js_v'][j]
        kin.update(q, qdot, with_jdot=False)
        ip, iv = _at(data['tp'], t_cap), _at(data['tv'], t_cap)
        centres, vels = data['P'][ip], data['V'][iv]
        for lbl, ld in labelled_links(msg):
            if '.' in lbl or not ld.valid:
                continue                                 # rank-0 entries only
            n = np.array([ld.direction.x, ld.direction.y, ld.direction.z])
            nn = float(np.linalg.norm(n))
            if nn < 1e-9:
                continue
            n /= nn
            pr = np.array([ld.closest_point_robot.x, ld.closest_point_robot.y,
                           ld.closest_point_robot.z])
            ph = np.array([ld.closest_point_human.x, ld.closest_point_human.y,
                           ld.closest_point_human.z])
            fid = kin.resolve_frame_id(ld.robot_link_name)
            if fid is None:
                continue
            adotq = float((n @ kin.point_jacobian_pos(fid, pr)) @ qdot)
            v_res = float(residual._obstacle_speed(lbl, float(ld.distance), t_cap, adotq, n))
            dc = np.linalg.norm(centres - ph, axis=1) - radii
            k = int(np.argmin(dc))
            on = bool(dc[k] < ON_OBSTACLE_M)
            v_true = float(n @ vels[k])
            tid = int(ld.track_id)
            v = np.array([ld.obstacle_velocity.x, ld.obstacle_velocity.y, ld.obstacle_velocity.z])
            C = np.asarray(ld.velocity_covariance, float).reshape(3, 3)
            s = series[lbl]
            s['t'].append(t_cap)
            s['d'].append(float(ld.distance))
            s['adotq'].append(adotq)
            s['qdot'].append(float(np.linalg.norm(qdot)))
            s['v_res'].append(v_res)
            s['v_trk'].append(float(n @ v) if tid else np.nan)
            s['sigma'].append(float(np.sqrt(max(n @ C @ n, 0.0))) if tid else np.nan)
            s['track_id'].append(tid)
            s['frames_seen'].append(int(ld.frames_seen))
            s['v_true'].append(v_true)
            s['v_true_norm'].append(float(np.linalg.norm(vels[k])))
            s['on_sphere'].append(1.0 if on else 0.0)
            s['obs'].append(k)
    return {lbl: {k: np.asarray(v) for k, v in s.items()} for lbl, s in series.items()}


# ── the metrics ──────────────────────────────────────────────────────────────

def _frame_dt(series) -> float:
    ts = np.unique(np.concatenate([s['t'] for s in series.values()])) if series else np.zeros(0)
    return float(np.median(np.diff(ts))) if ts.size > 2 else 1.0 / 90.0


def estimator_scores(series: dict, est: str, dt: float | None = None) -> dict:
    """lag / bias / RMS / noise floor of ``est`` ('v_res' or 'v_trk') against the truth."""
    cv = _cv()
    dt = dt or _frame_dt(series)
    lags, err_all, noise_all, per_cp, pooled = [], [], [], {}, []
    for lbl, s in sorted(series.items()):
        m = s['on_sphere'] > 0.5
        e = s[est]
        # compare_vobs: the untracked samples read as the safe 0.0 the consumer sees
        e = np.where(np.isfinite(e), e, 0.0) if est == 'v_trk' else e
        tru = s['v_true']
        if m.any():
            pooled.append(e[m] - tru[m])
        if m.sum() < MIN_SAMPLES:
            continue
        lag, r = cv.cross_correlation_lag(tru[m], e[m], dt)
        err = e[m] - tru[m]
        stat = cv.static_windows(s['d'], s['qdot']) & m & (s['v_true_norm'] < 1e-6)
        per_cp[lbl] = dict(n=int(m.sum()), lag_s=lag, peak_r=r, bias=float(np.mean(err)),
                           rms=cv.rms(err), n_static=int(stat.sum()),
                           noise=cv.rms(e[stat]) if stat.any() else float('nan'))
        if np.isfinite(lag):
            lags.append(lag)
        err_all.append(err)
        if stat.any():
            noise_all.append(e[stat])
    err = np.concatenate(err_all) if err_all else np.zeros(0)
    noise = np.concatenate(noise_all) if noise_all else np.zeros(0)
    pool = np.concatenate(pooled) if pooled else np.zeros(0)
    # pooled over EVERY CP, also those under compare_vobs' 30-sample cut: a 4 m/s
    # ball is in front of one CP for a handful of frames, so the per-CP cut
    # leaves nothing to score exactly where the speed is highest
    return dict(estimator=est, n_cp=len(per_cp), n=int(err.size),
                pooled_n=int(pool.size),
                pooled_bias=float(np.mean(pool)) if pool.size >= POOLED_MIN else float('nan'),
                pooled_rms=cv.rms(pool) if pool.size >= POOLED_MIN else float('nan'),
                lag_ms=1e3 * float(np.median(lags)) if lags else float('nan'),
                bias=float(np.mean(err)) if err.size else float('nan'),
                rms=cv.rms(err) if err.size else float('nan'),
                noise_floor=cv.rms(noise) if noise.size else float('nan'),
                n_static=int(noise.size), per_cp=per_cp)


def track_scores(series: dict, trust_n: int) -> dict:
    """First sight -> confirmed track -> trusted; identity switches; sigma calibration."""
    frames = defaultdict(lambda: defaultdict(list))    # obs -> t -> [track ids]
    first_row, z = {}, []
    for s in series.values():
        for i in np.flatnonzero(s['on_sphere'] > 0.5):
            k, t = int(s['obs'][i]), float(s['t'][i])
            first_row[k] = min(first_row.get(k, np.inf), t)
            frames[k][t].append(int(s['track_id'][i]))
            if s['track_id'][i] and s['frames_seen'][i] >= trust_n and s['sigma'][i] > 0:
                z.append(abs(s['v_trk'][i] - s['v_true'][i]) / s['sigma'][i])
    per_obs = {}
    for k, by_t in frames.items():
        ts = sorted(by_t)
        tracked = [t for t in ts if any(by_t[t])]
        trusted = [float(s['t'][i]) for s in series.values()
                   for i in np.flatnonzero((s['on_sphere'] > 0.5) & (s['obs'] == k)
                                           & (s['track_id'] > 0) & (s['frames_seen'] >= trust_n))]
        major = [Counter(x for x in by_t[t] if x).most_common(1)[0][0] for t in tracked]
        switches = int(sum(1 for a, b in zip(major, major[1:]) if a != b))
        per_obs[k] = dict(
            t_first_row=first_row[k],
            confirm_ms=1e3 * (tracked[0] - first_row[k]) if tracked else float('nan'),
            trust_ms=1e3 * (min(trusted) - first_row[k]) if trusted else float('nan'),
            n_ids=len(set(major)), id_switches=switches,
            tracked_frames=len(tracked), row_frames=len(ts))
    z = np.asarray(z)
    calib = {f'k{k}': float(np.mean(z <= k)) if z.size else float('nan') for k in (1, 2, 3)}
    return dict(trust_frames=int(trust_n), per_obstacle=per_obs, sigma_calibration=calib,
                n_calibration=int(z.size))


def score_run(run_dir: str, *, condition: str, t_from=None, t_to=None,
              sets: dict | None = None) -> dict:
    """Everything above for one run; the condition decides which estimator the CBF used."""
    from . import analysis as A
    man = A.load_json(os.path.join(run_dir, 'run_manifest_sim.json')) or {}
    ep = man.get('episode', {})
    t0 = ep.get('t_start_sim', -np.inf) if t_from is None else t_from
    t1 = ep.get('t_end_sim', np.inf) if t_to is None else t_to
    radii = [o['radius'] for o in (man.get('scenario_resolved') or {}).get('obstacles', [])]
    if not radii:
        return {}
    data = read_run(run_dir)
    series = build_series(data, radii, t_from=t0, t_to=t1, condition=condition, sets=sets)
    dt = _frame_dt(series)
    out = dict(frame_dt=dt, n_frames=len(data['dist']),
               residual=estimator_scores(series, 'v_res', dt),
               tracker=estimator_scores(series, 'v_trk', dt),
               used='v_res' if condition == 'residual' else 'v_trk',
               tracks=track_scores(series, trust_frames(1.0 / dt)))
    np.savez_compressed(os.path.join(run_dir, 'compare_vobs_sim.npz'),
                        **{f'{lbl}|{k}': v for lbl, s in series.items() for k, v in s.items()})
    return out
