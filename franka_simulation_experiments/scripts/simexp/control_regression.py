"""SIM-only paired regression protocol and full-rate CBF diagnostic scoring."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from . import analysis as A

SCENARIOS = ('control_clear', 'control_static', 'control_approach', 'control_stick')


def cells(seeds=(0, 1, 2)):
    return [dict(scenario=s, seed=int(k)) for k in seeds for s in SCENARIOS]


def summarize_diagnostics(samples, fields):
    """Score only fields present on the wire; old bags never invent zeros."""
    out = dict(n_samples=len(samples), fields=list(fields))
    if not samples:
        return out
    for i, name in enumerate(fields):
        values = np.array([m[i] for m in samples if len(m) > i], dtype=float)
        values = values[np.isfinite(values)]
        if not len(values):
            continue
        if name == 'fallback_level':
            out['fallback_ticks'] = {str(k): int((values == k).sum()) for k in (0, 1, 2)}
        elif name == 'slew_conflict_joints':
            out['slew_conflict_ticks'] = int((values > 0).sum())
            out['slew_conflict_joint_ticks'] = int(values.sum())
        elif name.startswith(('slack_', 'violation_', 'distance_', 'joint_receipt_',
                              'constraint_', 'tick_gap_', 'solve_wall_')):
            out[name] = dict(p50=float(np.median(values)),
                             p95=float(np.percentile(values, 95)), max=float(values.max()))
    return out


def tracking_from_stamped_points(actual, desired, start, end):
    """Join simultaneous TCP samples by HEADER stamp, not recorder/CSV clocks."""
    def stamp(m):
        return m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
    reference = {stamp(m): np.array([m.point.x, m.point.y, m.point.z]) for _, m in desired}
    errors = []
    for _, m in actual:
        t = stamp(m)
        if start <= t*1e-9 <= end and t in reference:
            p = np.array([m.point.x, m.point.y, m.point.z])
            errors.append(float(np.linalg.norm(p-reference[t])))
    if not errors:
        return {'n': 0}
    e = np.array(errors)
    return dict(n=len(e), rms_m=float(np.sqrt(np.mean(e**2))),
                p50_m=float(np.median(e)), p95_m=float(np.percentile(e, 95)),
                max_m=float(e.max()), source='equal PointStamped.header.stamp')


def check_obstacle_motion(truth, scenario, start, end):
    """Compare recorded physics-rate poses with the declared analytic laws."""
    if truth is None:
        return {}
    t = truth['t_pose']
    select = (t > start+.002) & (t <= end)
    dt = t[select] - start
    out = {}
    for k, spec in enumerate(scenario.get('obstacles', [])):
        if not len(dt):
            continue
        law = spec['law']
        if law not in ('static', 'oscillate'):
            continue
        target = np.tile(spec['p0'], (len(dt), 1)).astype(float)
        if law == 'oscillate':
            axis = np.asarray(spec['axis'], dtype=float)
            axis /= np.linalg.norm(axis)
            target += (spec['amplitude'] * np.sin(2*np.pi*dt/spec['period']))[:, None]*axis
        error = np.linalg.norm(truth['p'][select, k] - target, axis=1)
        out[spec['name']] = dict(law=law, n_samples=len(error),
                                 max_position_error_m=float(error.max()),
                                 passed=bool(error.max() < 1e-6))
    return out


def score(run_dir):
    from std_msgs.msg import Float64MultiArray
    from geometry_msgs.msg import PointStamped
    from .closed_loop import summarize_run
    run_dir = Path(run_dir)
    man = A.load_json(str(run_dir / 'run_manifest_sim.json')) or {}
    episode = man.get('episode') or {}
    if 't_start_sim' not in episode or 't_end_sim' not in episode:
        return dict(error='incomplete episode', execution_complete=False)
    t0, t1 = episode['t_start_sim'], episode['t_end_sim']
    closed = summarize_run(str(run_dir))
    truth = A.load_truth(str(run_dir))
    if (not (man.get('scenario_resolved') or {}).get('obstacles')
            and truth is not None and np.any((truth['t'] >= t0) & (truth['t'] <= t1))):
        closed.update(contact=False, min_h=None, min_gap=None, neg_duration_s=0.)
    snap = A.load_json(str(run_dir / 'provenance/snapshot.json')) or {}
    data = A.read_bag_topics(str(run_dir / 'bag'), {
        '/NS_1/cbf_status': Float64MultiArray,
        '/NS_1/ee_actual': PointStamped, '/NS_1/ee_desired': PointStamped})
    # The commander's CSV time is relative to node startup; the supervisor
    # uses absolute simulation time. Never compare those two clocks directly.
    closed['pentagon_csv_legacy'] = closed.pop('pentagon', {})
    closed['pentagon'] = tracking_from_stamped_points(
        data['/NS_1/ee_actual'], data['/NS_1/ee_desired'], t0, t1)
    samples = [list(m.data) for t, m in data['/NS_1/cbf_status'] if t0 <= t <= t1]
    diagnostics = summarize_diagnostics(samples, snap.get('status_fields', []))
    return dict(execution_complete=True, closed_loop=closed, diagnostics=diagnostics,
                obstacle_motion=check_obstacle_motion(
                    truth, man.get('scenario_resolved') or {}, t0, t1),
                # These classify evidence; an unsafe run is still retained.
                contact_free=closed.get('contact') is False,
                reflex_free=closed.get('reflex_trips') == 0,
                qp_rate_at_least_240hz=(closed.get('qp_rate_hz') or 0.) >= 240.,
                snapshot=str(run_dir / 'provenance/snapshot.json'))


def write_summary(root):
    root = Path(root)
    results = []
    for path in sorted(root.glob('*/control_*__s*/result.json')):
        if '_incomplete_' in str(path.parent):
            continue
        data = json.loads(path.read_text())
        data['result_path'] = str(path)
        results.append(data)
    (root / 'summary.json').write_text(json.dumps(dict(SIM=True, results=results), indent=2))
    return results
