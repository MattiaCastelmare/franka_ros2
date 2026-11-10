"""Closed-loop metrics of a Gazebo run, as cbf_scenarios.summarize() defines them (spec Phase 5.2).

Over the episode window [t_start, t_end] (sim time, run_manifest_sim.json):

* ``min_h``, ``neg_duration_s``, ``contact``: from the TRUE per-control-point
  surface gaps (truth.npz, 1 kHz): h = gap − d_safe, contact = gap < 0 (=
  cbf_scenarios' h < −d_safe). Real CP set and radii, not cbf_scenarios' R_CAP;
* ``slack_max``: per family from the CBFDIAG lines (``s[obs/sc/qlim/sing/cap/
  spd]``, throttled to ``diag_period_s``, so a SAMPLED maximum), and
  ``slack_max_any`` from /NS_1/cbf_status (every QP tick, the largest family);
* ``fallback_levels``: QP ticks per level (full / box / brake), the difference
  of the filter's cumulative ``fb=[..]`` counter between the last tick line
  before the start and the last one before the end (100-tick resolution);
* ``livelock_escapes``: LIVELOCK lines inside the episode;
* ``task_done``: ‖q(t_end) − goal‖ < 0.05 rad (cbf_scenarios: the goal is Q0
  for a Cartesian reference); pentagon runs report the commander's EE error;
* ``jerk_p99`` [rad/s³]: ‖Δq̈_safe‖ / Δt over consecutive /NS_1/qddot_safe
  messages (bag receive times, sim clock), Δt = their median spacing;
* reflex verdict and RTF from reflex_summary.json (the monitor's FINAL write;
  run_manifest_sim.json holds the supervisor's copy, taken up to 2 s before
  the monitor's last summary), trips and torque-rate clips counted from
  reflex_trips.csv inside the episode window;
* latency, sim time: ``rows_latency_ms`` = bag receipt − capture stamp of
  /cbf/per_link_distances (capture → rows on the wire: the perception hop of
  latency_budget.py, which needs depth images in the bag and has none here),
  and ``capage_ms`` = the CBF's own sensing age at the QP (CBFDIAG ``capage``).

Log lines are bracketed by the supervisor's START / DONE lines (same wall
clock as every other line of the launch log).
"""
from __future__ import annotations

import os
import re

import numpy as np

from . import analysis as A
from .reflex_checks import CLIP_KIND

#: The QP's constraint families, in decision-vector order. Pinned to
#: ``cbf_state_rows.GROUP_NAMES`` by test_closed_loop.test_names_are_the_filters,
#: because the CBFDIAG slack line is parsed POSITIONALLY: a family added to the
#: QP and not added here would shift every slack a scorer reads.
FAMILIES = ('obs', 'sc', 'qlim', 'sing', 'cap', 'spd', 'ssb')
#: Task 5.5 of ``OSCBF_ALIGNMENT_PLAN.md``: the achieved QP rate is a pass/fail
#: INPUT, not just a reported number. A cell that misses the rate its own
#: condition asked for by more than this fraction is not a valid data point —
#: the slew box makes jerk = max_qddot_delta/dt, so two arms clocked
#: differently are not being compared on their barriers (the filter has been
#: measured starved to 18 Hz against a 100 Hz target). ``batch`` gates ``ok``
#: on the verdict; a batch that wants the old behaviour must say so.
QP_RATE_TOL = 0.05
LEVELS = ('full', 'box-only', 'brake')     # cbf_qp_assembly.LEVEL_NAMES (test pins it)
_STAMP = re.compile(r'\[(\d{9,}\.\d+)\]')
_FB = re.compile(r'fb=\[(\d+), (\d+), (\d+)\]')
#: cbf_safety_filter._report: cumulative [ticks, joint-ticks] where the slew
#: box and the hard state box were DISJOINT, i.e. the ticks where
#: slew_yields_to_state_box decides which box the solver is handed.
_SLEWCONF = re.compile(r'slewconf=\[(\d+), (\d+)\]')


def rate_verdict(achieved, target, tol: float = QP_RATE_TOL):
    """``(ok, relative error)`` for an achieved QP rate against its nominal.

    ``None`` for either input means the verdict is UNKNOWN, not a failure: the
    bag may carry no ``cbf_status`` (the filter never ran) or the manifest may
    not record the nominal rate (a run from before this gate). Unknown is
    reported as ``None`` so such a cell is neither silently accepted as
    rate-matched nor silently discarded.
    """
    if achieved is None or target is None:
        return None, None
    target = float(target)
    if not target > 0.0:
        return None, None
    err = float(achieved) / target - 1.0
    # the band INCLUDES its edge: 95.0/100.0 - 1 is -0.050000000000000044 in
    # binary, which would otherwise reject a cell that is exactly on tolerance
    return bool(abs(err) <= float(tol) + 1e-9), err


def _wall(line: str) -> float | None:
    m = _STAMP.search(line)
    return float(m.group(1)) if m else None


def log_window(lines: list[str]) -> tuple[float, float]:
    t0 = t1 = None
    for ln in lines:
        if 'SIM episode START' in ln and t0 is None:
            t0 = _wall(ln)
        if 'SIM episode DONE' in ln:
            t1 = _wall(ln)
    return (t0 if t0 is not None else -np.inf), (t1 if t1 is not None else np.inf)


def _cumulative_delta(lines: list[str], w0: float, w1: float, pat, n: int):
    """Episode delta of a CUMULATIVE counter printed on the filter's log line.

    The counter runs from node start, so the episode's value is the last
    reading inside the window minus the last reading before it opened.
    """
    before, last = np.zeros(n, int), None
    for ln in lines:
        if 'cbf_safety_filter' not in ln:
            continue
        m = pat.search(ln)
        w = _wall(ln)
        if not m or w is None:
            continue
        v = np.array([int(x) for x in m.groups()])
        if w <= w0:
            before = v
        if w <= w1:
            last = v
    return None if last is None else (last - before)


def fallback_counts(lines: list[str], w0: float, w1: float) -> dict:
    d = _cumulative_delta(lines, w0, w1, _FB, 3)
    if d is None:
        return {k: None for k in LEVELS}
    return {k: int(x) for k, x in zip(LEVELS, d)}


def slew_conflict_counts(lines: list[str], w0: float, w1: float) -> dict:
    """``{'ticks': n, 'joint_ticks': m}`` for the episode, or Nones.

    Nones mean the filter never printed the field, which is what an older bag
    does — not a measured zero.
    """
    d = _cumulative_delta(lines, w0, w1, _SLEWCONF, 2)
    if d is None:
        return dict(ticks=None, joint_ticks=None)
    return dict(ticks=int(d[0]), joint_ticks=int(d[1]))


def cbfdiag_capage(lines: list[str], t0: float, t1: float) -> dict:
    ages = []
    for ln in lines:
        i = ln.find('CBFDIAG ')
        if i < 0:
            continue
        f = dict(A._FIELD.findall(ln[i + 8:]))
        if t0 <= A._num(f.get('t')) <= t1 and np.isfinite(A._num(f.get('capage'))):
            ages.append(1e3 * A._num(f.get('capage')))
    if not ages:
        return {}
    return dict(capage_ms_p50=float(np.median(ages)), capage_ms_p95=float(np.percentile(ages, 95)))


#: The CBFDIAG keys the per-family slack is spread across, and which slot of
#: each holds the slack. The six original families share one '/'-joined field;
#: the SSB family was APPENDED as its own key (``ssb[n/d/s]`` = rows, closest
#: gap, slack) rather than widened into the first, because every parser of that
#: bracketed group counts six. Both are read here so no family's slack is
#: silently scored as a measured 0.0.
_SLACK_KEYS = (('s[obs/sc/qlim/sing/cap/spd]', FAMILIES[:6], None),
               ('ssb[n/d/s]', ('ssb',), 2))


def cbfdiag_slack(lines: list[str], t0: float, t1: float) -> dict:
    """Per-family max slack and, for SSB, its row count and closest gap."""
    best = {n: 0.0 for n in FAMILIES}
    ssb_rows_max, ssb_gap_min = 0.0, float('inf')
    n = 0
    for ln in lines:
        i = ln.find('CBFDIAG ')
        if i < 0:
            continue
        f = dict(A._FIELD.findall(ln[i + 8:]))
        t = A._num(f.get('t'))
        if not (t0 <= t <= t1):
            continue
        n += 1
        for key, names, slot in _SLACK_KEYS:
            parts = (f.get(key) or '').split('/')
            if slot is None:
                for name, x in zip(names, parts):
                    best[name] = max(best[name], A._num(x, 0.0))
            elif len(parts) > slot:
                best[names[0]] = max(best[names[0]], A._num(parts[slot], 0.0))
        ssb = (f.get('ssb[n/d/s]') or '').split('/')
        if len(ssb) == 3:
            ssb_rows_max = max(ssb_rows_max, A._num(ssb[0], 0.0))
            g = A._num(ssb[1], float('inf'))
            if np.isfinite(g):
                ssb_gap_min = min(ssb_gap_min, g)
    return dict(best, n_lines=n, ssb_rows_max=ssb_rows_max,
                ssb_gap_min=(None if not np.isfinite(ssb_gap_min)
                             else float(ssb_gap_min)))


def summarize_run(run_dir: str) -> dict:
    from std_msgs.msg import Float64MultiArray
    man = A.load_json(os.path.join(run_dir, 'run_manifest_sim.json')) or {}
    ep = man.get('episode') or {}
    t0, t1 = ep.get('t_start_sim'), ep.get('t_end_sim')
    if t0 is None or t1 is None:
        return {'error': 'no episode in run_manifest_sim.json'}
    # the rate this cell's own condition asked for: the gate's reference (Task 5.5)
    qp_target = ((man.get('condition') or {}).get('cbf_params') or {}).get('qp_rate_hz')
    rs = A.load_json(os.path.join(run_dir, 'reflex_summary.json')) or {}
    rtf = rs.get('rtf') or man.get('rtf') or {}
    counts = rs.get('counts') or man.get('reflex_counts') or {}
    out = dict(qp_rate_target_hz=qp_target, qp_rate_hz=None, qp_rate_err=None,
               qp_rate_ok=None,
               rtf=rtf.get('rtf_run'), rtf_p10=rtf.get('rtf_p10_window'),
               reflex_verdict=rs.get('verdict') or man.get('reflex_verdict'),
               reflex_trips=sum(v for k, v in counts.items() if k != CLIP_KIND),
               rate_clips=counts.get(CLIP_KIND, 0),
               controller_active_at_start=ep.get('controller_active_at_start'))
    trips_csv = os.path.join(run_dir, 'reflex_trips.csv')
    if os.path.isfile(trips_csv):
        import csv as _csv
        with open(trips_csv) as f:
            ev = [(float(r['t_sim']), r['kind']) for r in _csv.DictReader(f)]
        inside = [k for t, k in ev if t0 <= t <= t1]
        out.update(reflex_trips=sum(1 for k in inside if k != CLIP_KIND),
                   rate_clips=sum(1 for k in inside if k == CLIP_KIND),
                   reflex_trips_whole_run=sum(1 for _, k in ev if k != CLIP_KIND))

    T = A.load_truth(run_dir)
    if T is not None and T['gap'].size:
        w = (T['t'] >= t0) & (T['t'] <= t1)
        gap = T['gap'][w]
        gmin = gap.reshape(gap.shape[0], -1).min(axis=1)
        h = gmin - float(T['d_safe'])
        dt = float(np.median(np.diff(T['t'][w]))) if w.sum() > 2 else 1e-3
        k = int(np.argmin(h)) if h.size else 0
        cp = int(np.argmin(gap[k].min(axis=1))) if h.size else 0
        out.update(min_h=float(h.min()), min_gap=float(gmin.min()),
                   neg_duration_s=float((h < 0).sum() * dt), contact=bool((gmin < 0).any()),
                   min_h_cp=str(T['labels'][cp]) if len(T['labels']) else '',
                   t_min_h=float(T['t'][w][k] - t0))
        sc = man.get('scenario_resolved') or {}
        nom = sc.get('nominal') or {}
        if nom.get('law') in ('joint_pd', 'cartesian'):
            goal = np.asarray(nom.get('goal_q') or sc.get('q0'), float)
            err = float(np.linalg.norm(T['q'][w][-1] - goal))
            out.update(task_final_err=err, task_done=bool(err < 0.05))

    bag = os.path.join(run_dir, 'bag')
    if os.path.isdir(bag):
        from franka_msgs.msg import MultiLinkDistance
        b = A.read_bag_topics(bag, {'/NS_1/qddot_safe': Float64MultiArray,
                                    '/NS_1/cbf_status': Float64MultiArray,
                                    '/cbf/per_link_distances': MultiLinkDistance})
        lat = [1e3 * (t - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))
               for t, m in b['/cbf/per_link_distances'] if t0 <= t <= t1]
        if lat:
            out.update(rows_latency_ms_p50=float(np.median(lat)),
                       rows_latency_ms_p95=float(np.percentile(lat, 95)),
                       rows_rate_hz=len(lat) / max(t1 - t0, 1e-9))
        qs = [(t, np.asarray(m.data[:7])) for t, m in b['/NS_1/qddot_safe'] if t0 <= t <= t1]
        if len(qs) > 3:
            tq = np.array([t for t, _ in qs])
            Q = np.array([q for _, q in qs])
            dtq = float(np.median(np.diff(tq)))
            jerk = np.linalg.norm(np.diff(Q, axis=0), axis=1) / dtq
            out.update(jerk_p50=float(np.percentile(jerk, 50)),
                       jerk_p99=float(np.percentile(jerk, 99)), jerk_max=float(jerk.max()),
                       qp_dt_s=dtq)
        st = [m.data for t, m in b['/NS_1/cbf_status'] if t0 <= t <= t1]
        if st:
            S = np.array([list(d[:5]) for d in st])
            r_hz = len(S) / max(t1 - t0, 1e-9)
            ok_r, err_r = rate_verdict(r_hz, qp_target)
            out.update(slack_max_any=float(S[:, 1].max()), fault_ticks=int((S[:, 2] > 0).sum()),
                       status_ticks=len(S), n_rows_max=int(S[:, 0].max()),
                       # achieved QP rate in SIM time, against the nominal this
                       # condition asked for (cbf_params.qp_rate_hz, 100 Hz as
                       # shipped) and the +-QP_RATE_TOL verdict on it
                       qp_rate_hz=r_hz, qp_rate_err=err_r, qp_rate_ok=ok_r)

    lp = os.path.join(run_dir, 'launch.log')
    if os.path.isfile(lp):
        with open(lp, errors='replace') as f:
            lines = f.readlines()
        w0, w1 = log_window(lines)
        out['fallback_levels'] = fallback_counts(lines, w0, w1)
        out['slew_conflict'] = slew_conflict_counts(lines, w0, w1)
        out['livelock_escapes'] = sum(1 for ln in lines if 'LIVELOCK:' in ln
                                      and (_wall(ln) or -np.inf) >= w0
                                      and (_wall(ln) or np.inf) <= w1)
        out['slack_max'] = cbfdiag_slack(lines, t0, t1)
        out.update(cbfdiag_capage(lines, t0, t1))
    pc = A.find_files(run_dir).get('pentagon_csv')
    if pc:
        out['pentagon'] = A.pentagon_tracking(pc, t_from=t0)     # the episode, as Phase 2
    return out
