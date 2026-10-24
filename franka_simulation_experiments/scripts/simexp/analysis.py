"""Read back a Gazebo run: CBFDIAG lines, tracking error, run files. Pure (no rclpy).

The shipped digests (experiment_summary.py, smoothness_report.py) run on the
run as they are; this adds what they do not report:

* CBFDIAG ``qdd_cmd_rad`` vs ``qdd_real_rad``: the commanded vs realised
  acceleration along the most-binding row (``logging_utils.format_cbf_diag``).
  In Gazebo the plant is known, so the ratio is the reference point for the
  hardware gap;
* end-effector tracking error from the commander's own CSV (``ee_*`` vs
  ``ee_*_des``, the same P(s) and FK the overlay draws).
"""
from __future__ import annotations

import csv
import glob
import json
import math
import os
import re

import numpy as np

_FIELD = re.compile(r'([A-Za-z_\[\]/]+)=([^\s|]+)')


def parse_cbfdiag(path: str) -> list[dict]:
    """Every CBFDIAG line of a launch log as {field: str}."""
    rows = []
    with open(path, errors='replace') as f:
        for line in f:
            i = line.find('CBFDIAG ')
            if i < 0:
                continue
            rows.append(dict(_FIELD.findall(line[i + 8:])))
    return rows


def _num(s, default=math.nan):
    try:
        return float(s)
    except (TypeError, ValueError):
        return default


def qdd_ratio(rows: list[dict], min_cmd: float = 0.5) -> dict:
    """Realised over commanded acceleration along the binding row.

    Only lines where |qdd_cmd_rad| >= min_cmd [rad/s²] count. Smaller commands
    are dominated by noise in the realised acceleration.
    """
    cmd = np.array([_num(r.get('qdd_cmd_rad')) for r in rows])
    real = np.array([_num(r.get('qdd_real_rad')) for r in rows])
    trk = np.array([_num(r.get('trk_err')) for r in rows])
    ok = np.isfinite(cmd) & np.isfinite(real) & (np.abs(cmd) >= min_cmd)
    out = {'n_lines': len(rows), 'n_used': int(ok.sum()), 'min_cmd_rad': min_cmd}
    if ok.any():
        ratio = real[ok] / cmd[ok]
        out.update(ratio_median=float(np.median(ratio)),
                   ratio_p25=float(np.percentile(ratio, 25)),
                   ratio_p75=float(np.percentile(ratio, 75)),
                   abs_cmd_median=float(np.median(np.abs(cmd[ok]))))
    if np.isfinite(trk).any():
        out['trk_err_median'] = float(np.nanmedian(trk))
        out['trk_err_p95'] = float(np.nanpercentile(trk, 95))
    return out


def slack_max(rows: list[dict]) -> dict:
    """Largest slack per family from s[obs/sc/qlim/sing/cap/spd]=a/b/c/d/e/f."""
    names = ('obs', 'sc', 'qlim', 'sing', 'cap', 'spd')
    best = {n: 0.0 for n in names}
    for r in rows:
        v = r.get('s[obs/sc/qlim/sing/cap/spd]')
        if not v:
            continue
        for n, x in zip(names, v.split('/')):
            best[n] = max(best[n], _num(x, 0.0))
    return best


def find_files(run_dir: str) -> dict:
    lg = sorted(glob.glob(os.path.join(run_dir, 'logger', '*', 'experiment_log.csv')))
    pent = sorted(glob.glob(os.path.join(run_dir, 'logger', 'pentagon_run_*.csv')))
    return {'logger_dir': os.path.dirname(lg[-1]) if lg else None,
            'pentagon_csv': pent[-1] if pent else None,
            'bag': os.path.join(run_dir, 'bag') if os.path.isdir(os.path.join(run_dir, 'bag'))
            else None,
            'launch_log': os.path.join(run_dir, 'launch.log')
            if os.path.isfile(os.path.join(run_dir, 'launch.log')) else None,
            'reflex': os.path.join(run_dir, 'reflex_summary.json')
            if os.path.isfile(os.path.join(run_dir, 'reflex_summary.json')) else None,
            'manifest': os.path.join(run_dir, 'run_manifest_sim.json')}


def pentagon_tracking(csv_path: str, t_from: float | None = None) -> dict:
    """|ee - ee_des| [m] over the commander's CSV (optionally from time t_from on)."""
    with open(csv_path) as f:
        rows = list(csv.DictReader(f))
    t = np.array([float(r['time']) for r in rows])
    e = np.array([[float(r[k]) for k in ('ee_x', 'ee_y', 'ee_z')] for r in rows])
    d = np.array([[float(r[k]) for k in ('ee_x_des', 'ee_y_des', 'ee_z_des')] for r in rows])
    m = np.ones(len(t), bool) if t_from is None else t >= t_from
    err = np.linalg.norm(e[m] - d[m], axis=1)
    if err.size == 0:
        return {'n': 0}
    return {'n': int(err.size), 'duration_s': float(t[m][-1] - t[m][0]),
            'rms_m': float(np.sqrt(np.mean(err ** 2))), 'p50_m': float(np.median(err)),
            'p95_m': float(np.percentile(err, 95)), 'max_m': float(err.max())}


def load_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError, TypeError):
        return None


def read_bag_topics(bag_dir: str, topics: dict) -> dict:
    """{topic: [(t_recv_s, msg)]} from a sqlite3 rosbag2 directory (sim-time stamps)."""
    import sqlite3
    from rclpy.serialization import deserialize_message
    db = sorted(glob.glob(os.path.join(bag_dir, '*.db3')))[0]
    c = sqlite3.connect(db)
    tid = {n: i for i, n in c.execute('select id,name from topics')}
    out = {}
    for topic, typ in topics.items():
        if topic not in tid:
            out[topic] = []
            continue
        out[topic] = [(ts * 1e-9, deserialize_message(data, typ)) for ts, data in c.execute(
            'select timestamp,data from messages where topic_id=? order by timestamp',
            (tid[topic],))]
    return out


def qdd_projection(bag_dir: str, t_from: float = -np.inf, t_to: float = np.inf,
                   min_cmd: float = 0.5, smooth_ms: float = 10.0) -> dict:
    """How much of the commanded q̈_safe the arm delivered, in joint space.

    ``frac = (q̈_real · q̈_cmd) / ‖q̈_cmd‖²`` per 1 kHz sample: the same
    projection cbf_safety_filter uses for its braking-authority check
    (``_brake_authority_fault``), but over every joint instead of one row.
    q̈_real is the derivative of the reported q̇, box-smoothed over
    ``smooth_ms``; q̈_cmd is the last q̈_safe, zero-order hold, delayed by the
    same half window so the two are aligned.
    """
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray
    names = [f'fr3_joint{i}' for i in range(1, 8)]
    d = read_bag_topics(bag_dir, {'/NS_1/franka/joint_states': JointState,
                                  '/NS_1/qddot_safe': Float64MultiArray})
    js = [(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m) for _, m in
          d['/NS_1/franka/joint_states']]
    js = [(t, m) for t, m in js if t_from <= t <= t_to]
    if len(js) < 50:
        return {'n': 0}
    idx = {n: i for i, n in enumerate(js[0][1].name)}
    t = np.array([x[0] for x in js])
    v = np.array([[m.velocity[idx[n]] for n in names] for _, m in js])
    k = max(1, int(round(smooth_ms * 1e-3 / max(np.median(np.diff(t)), 1e-4))))
    a = np.gradient(v, t, axis=0)
    a = np.apply_along_axis(lambda x: np.convolve(x, np.ones(k) / k, mode='same'), 0, a)
    qs = d['/NS_1/qddot_safe']
    ts = np.array([x[0] for x in qs])
    A = np.array([x[1].data[:7] for x in qs])
    j = np.clip(np.searchsorted(ts, t - 0.5 * smooth_ms * 1e-3, side='right') - 1, 0, len(ts) - 1)
    cmd = A[j]
    n2 = np.sum(cmd * cmd, axis=1)
    ok = n2 >= min_cmd ** 2
    if not ok.any():
        return {'n': 0, 'min_cmd': min_cmd}
    frac = np.sum(a[ok] * cmd[ok], axis=1) / n2[ok]
    return {'n': int(ok.sum()), 'min_cmd': min_cmd, 'frac_median': float(np.median(frac)),
            'frac_p10': float(np.percentile(frac, 10)), 'frac_p90': float(np.percentile(frac, 90)),
            'err_rms': float(np.sqrt(np.mean(np.sum((a[ok] - cmd[ok]) ** 2, axis=1))))}


def per_link_entries(bag_dir: str, t_from: float = -np.inf, t_to: float = np.inf) -> list[dict]:
    """Every LinkDistance on /cbf/per_link_distances, flattened, with its message stamp."""
    from franka_msgs.msg import MultiLinkDistance
    d = read_bag_topics(bag_dir, {'/cbf/per_link_distances': MultiLinkDistance})
    out = []
    for t_rec, m in d['/cbf/per_link_distances']:
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if not (t_from <= t <= t_to):
            continue
        for ld in m.links:
            out.append(dict(
                t=t, t_recv=t_rec, link=ld.robot_link_name, valid=bool(ld.valid),
                d=float(ld.distance), track_id=int(ld.track_id), frames_seen=int(ld.frames_seen),
                pr=np.array([ld.closest_point_robot.x, ld.closest_point_robot.y,
                             ld.closest_point_robot.z]),
                ph=np.array([ld.closest_point_human.x, ld.closest_point_human.y,
                             ld.closest_point_human.z]),
                n=np.array([ld.direction.x, ld.direction.y, ld.direction.z]),
                v=np.array([ld.obstacle_velocity.x, ld.obstacle_velocity.y,
                            ld.obstacle_velocity.z]),
                P_vv=np.asarray(ld.velocity_covariance, float).reshape(3, 3)))
    return out


def load_truth(run_dir: str) -> dict | None:
    p = os.path.join(run_dir, 'truth.npz')
    if not os.path.isfile(p):
        return None
    with np.load(p, allow_pickle=False) as z:
        return {k: z[k] for k in z.files}


def ros_log_lines(run_dir: str, needle: str) -> list[str]:
    out = []
    for f in glob.glob(os.path.join(run_dir, 'ros_log', '**', '*.log'), recursive=True):
        with open(f, errors='replace') as fh:
            out += [ln for ln in fh if needle in ln]
    return out
