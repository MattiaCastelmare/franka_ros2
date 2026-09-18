#!/usr/bin/env python3
"""Turn one recorded run into a verdict per ISO-derived check.

READ THIS FIRST
---------------
**This report cannot tell you whether the cell is ISO compliant.** Conformity is
established by a risk assessment, by safety functions rated PL d / SIL 2, and by
validation from a competent person — not by a log file, and not by this script.
No verdict below means "compliant".

What it does tell you, which is the useful half:

* whether the robot ever exceeded the limits **its own configuration claims to
  respect**;
* whether any control point got closer than the separation distance its speed
  demanded;
* whether the data was good enough for either answer to mean anything;
* which questions are structurally unanswerable from logs, named explicitly, so
  they cannot be mistaken for questions that passed.

Every check reports one of:

    PASS           ran on adequate data, no violation found
    FAIL           violation found — worst value, when, how long, how often
    INCONCLUSIVE   not enough data (channel absent, coverage too low)
    NOT FROM LOGS  structurally impossible to answer this way

USAGE
-----
    python3 scripts/iso_evidence_report.py <run_dir> [--verbose] [--json out.json]

``<run_dir>`` is one directory written by ``iso_evidence_logger``, containing
``iso_manifest.json``, ``iso_evidence.csv`` and ``iso_events.csv``.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys

PASS, FAIL, INCONC, NOTLOG = 'PASS', 'FAIL', 'INCONCLUSIVE', 'NOT FROM LOGS'
#: A failure that no run can avoid and no tuning can fix, because the
#: configuration asks for something the workspace cannot deliver. Kept distinct
#: from FAIL so a foregone conclusion does not read like a run-specific finding
#: — and so it does not drown out the checks that ARE about this run.
STRUCT = 'STRUCTURAL'

#: Minimum fraction of samples that must carry a usable value before a check is
#: allowed to say PASS. A limit nobody measured is not a limit that held. [E]
MIN_COVERAGE = 0.5


class Result:
    def __init__(self, key, clause, tag, verdict, headline, detail=()):
        self.key, self.clause, self.tag = key, clause, tag
        self.verdict, self.headline = verdict, headline
        self.detail = list(detail)

    def as_dict(self):
        return dict(check=self.key, clause=self.clause, tag=self.tag,
                    verdict=self.verdict, headline=self.headline,
                    detail=self.detail)


# ── Loading ──────────────────────────────────────────────────────────────────

def load(run_dir):
    man_p = os.path.join(run_dir, 'iso_manifest.json')
    csv_p = os.path.join(run_dir, 'iso_evidence.csv')
    ev_p = os.path.join(run_dir, 'iso_events.csv')
    for p in (man_p, csv_p):
        if not os.path.exists(p):
            sys.exit(f'missing {os.path.basename(p)} in {run_dir} — send the '
                     f'WHOLE run directory, the CSV alone is uninterpretable')
    with open(man_p) as fh:
        man = json.load(fh)
    with open(csv_p) as fh:
        rows = list(csv.DictReader(fh))
    events = []
    if os.path.exists(ev_p):
        with open(ev_p) as fh:
            events = list(csv.DictReader(fh))
    return man, rows, events


def col(rows, name):
    """A column as floats, NaN where absent or unparseable."""
    out = []
    for r in rows:
        v = r.get(name, '')
        try:
            out.append(float(v))
        except (TypeError, ValueError):
            out.append(float('nan'))
    return out


def finite(xs):
    return [x for x in xs if not math.isnan(x)]


def coverage(xs):
    return (len(finite(xs)) / len(xs)) if xs else 0.0


def worst(rows, values, key, cmp):
    """(value, t, label) of the extreme sample, or (nan, nan, '')."""
    best, bt, bl = float('nan'), float('nan'), ''
    for r, v in zip(rows, values):
        if math.isnan(v):
            continue
        if math.isnan(best) or cmp(v, best):
            best, bt = v, float(r.get('t', 'nan') or 'nan')
            bl = r.get(key, '') if key else ''
    return best, bt, bl


def exceed_stats(rows, values, limit, above=True):
    """How many samples breached, for how long, and the worst one."""
    n = sum(1 for v in values
            if not math.isnan(v) and ((v > limit) if above else (v < limit)))
    dt = _period(rows)
    w, wt, _ = worst(rows, values, None,
                     (lambda a, b: a > b) if above else (lambda a, b: a < b))
    return n, n * dt, w, wt


def _period(rows):
    ts = finite(col(rows, 't'))
    if len(ts) < 2:
        return 0.0
    return (ts[-1] - ts[0]) / (len(ts) - 1)


def ev_episodes(events, on_kind, off_kind=None):
    """Pair rising/falling events into (t_on, t_off_or_None, detail)."""
    out, open_t, open_d = [], None, ''
    for e in events:
        k, t = e.get('kind'), float(e.get('t', 'nan') or 'nan')
        if k == on_kind:
            if open_t is not None:
                out.append((open_t, None, open_d))
            open_t, open_d = t, e.get('detail', '')
        elif off_kind and k == off_kind and open_t is not None:
            out.append((open_t, t, open_d))
            open_t = None
    if open_t is not None:
        out.append((open_t, None, open_d))
    return out


# ── Checks ───────────────────────────────────────────────────────────────────

def _d1_active(man):
    """Is the cell in deviation D1 — a floor larger than the robot can reach?

    With the ISO 13855:2024 body-detection C = 0.85 m the irreducible part of
    S_p is 0.92 m against an FR3 reach of 0.855 m. Every separation verdict is
    then a foregone conclusion, and reporting them as ordinary failures would
    bury the checks that actually say something about the run.

    Returns ``(active, floor, reach, c)``.
    """
    lim = man.get('limits', {})
    floor = float(lim.get('floor_c_zd_zr', float('nan')))
    reach = float(lim.get('robot_reach_m', 0.855))
    c = float(man.get('iso_params', {}).get('iso_c_intrusion', float('nan')))
    return (not math.isnan(floor) and floor > reach), floor, reach, c


def check_workspace_feasibility(man, rows, events):
    """Can the configured separation distance be satisfied in this cell at all?

    Runs FIRST, because it decides how to read everything after it.
    """
    clause = 'ISO 13855:2024 (C) + ISO 10218-2:2025 Annex L'
    active, floor, reach, c = _d1_active(man)
    if math.isnan(floor):
        return Result('workspace-feasibility', clause, '[R]', INCONC,
                      'the manifest carries no C + Z_d + Z_r')
    detail = [f'C (intrusion distance) {c:8.3f} m',
              f'C + Z_d + Z_r          {floor:8.3f} m',
              f'robot reach            {reach:8.3f} m']
    if not active:
        return Result('workspace-feasibility', clause, '[R]', PASS,
                      f'the irreducible separation {floor:.3f} m fits inside the '
                      f'{reach:.3f} m reach — conformant SSM is geometrically '
                      f'possible here', detail)
    detail += [
        '',
        'This is SAFETY.md deviation D1, not a run-specific finding and not',
        'something tuning can fix: the separation distance the standard',
        'requires is larger than the arm can reach, so no motion satisfies it.',
        'The separation checks below are therefore foregone conclusions — read',
        'their "excluding C" figures, which are the part the control system',
        'actually governs.',
        '',
        'The only route to a conformant C is demonstrating a detection',
        'capability d <= 40 mm (scripts/iso_constants_measure.py detection),',
        'giving C = 8*(d-14) mm. Even then the depth pipeline is not a rated',
        'protective device (no IEC/TS 61496-4-3 assessment).']
    return Result('workspace-feasibility', clause, '[R]', STRUCT,
                  f'conformant SSM is UNACHIEVABLE in this workspace: the '
                  f'irreducible separation is {floor:.3f} m against a '
                  f'{reach:.3f} m reach', detail)


def check_integrity(man, rows, events):
    if not rows:
        return Result('data-integrity', '—', '[E]', INCONC,
                      'the evidence CSV is empty')
    ts = finite(col(rows, 't'))
    dur = ts[-1] - ts[0] if len(ts) > 1 else 0.0
    want = float(man.get('sample_rate_hz', 100.0))
    got = (len(ts) - 1) / dur if dur > 0 else 0.0
    cov_js = coverage(col(rows, 'tcp_speed'))
    cov_pc = coverage(col(rows, 'd_min'))
    detail = [
        f'duration            {dur:8.2f} s   ({len(rows)} samples)',
        f'sample rate         {got:8.1f} Hz  (configured {want:.0f} Hz)',
        f'joint-state cover   {cov_js:8.1%}  (rows with a TCP speed)',
        f'perception cover    {cov_pc:8.1%}  (rows with a valid distance)',
    ]
    lost = ev_episodes(events, 'perception_lost', 'perception_back')
    if lost:
        detail.append(f'perception dropouts {len(lost):8d}   '
                      f'longest {max((b - a) for a, b, _ in lost if b):.2f} s'
                      if any(b for _, b, _ in lost) else
                      f'perception dropouts {len(lost):8d}')
    verdict, head = PASS, 'the run is long enough and the channels were alive'
    if dur < 5.0:
        verdict, head = INCONC, f'only {dur:.1f} s of data — too short to conclude anything'
    elif cov_js < MIN_COVERAGE:
        verdict, head = INCONC, f'joint state present in only {cov_js:.0%} of samples'
    elif got < 0.7 * want:
        verdict, head = INCONC, (f'the logger achieved {got:.0f} Hz against '
                                 f'{want:.0f} Hz configured — samples were dropped, '
                                 f'short excursions may be missing')
    elif cov_pc < MIN_COVERAGE:
        verdict, head = INCONC, (f'perception valid in only {cov_pc:.0%} of samples — '
                                 f'every separation verdict below is weak')
    return Result('data-integrity', '—', '[E]', verdict, head, detail)


def check_tcp_speed(man, rows, events):
    lim = float(man['iso_params'].get('iso_v_pfl', float('nan')))
    v = col(rows, 'tcp_speed_max')
    if math.isnan(lim):
        return Result('tcp-speed-vs-pfl', 'ISO 10218-2:2025 5.14.6 / Annex M',
                      '[E] from [S]', INCONC, 'iso_v_pfl not in the manifest')
    if coverage(v) < MIN_COVERAGE:
        return Result('tcp-speed-vs-pfl', 'ISO 10218-2:2025 5.14.6 / Annex M',
                      '[E] from [S]', INCONC,
                      'no TCP speed recorded — was the joint-state topic alive?')
    n, secs, w, wt = exceed_stats(rows, v, lim)
    detail = [f'v_PFL ceiling       {lim:8.3f} m/s',
              f'peak TCP speed      {w:8.3f} m/s  at t = {wt:.2f} s',
              f'samples over        {n:8d}   ({secs:.2f} s)']
    if n == 0:
        return Result('tcp-speed-vs-pfl', 'ISO 10218-2:2025 5.14.6 / Annex M',
                      '[E] from [S]', PASS,
                      f'TCP peaked at {w:.3f} m/s, under the {lim:.3f} m/s PFL '
                      f'ceiling', detail)
    return Result('tcp-speed-vs-pfl', 'ISO 10218-2:2025 5.14.6 / Annex M',
                  '[E] from [S]', FAIL,
                  f'TCP exceeded the PFL ceiling: peak {w:.3f} m/s vs '
                  f'{lim:.3f} m/s, for {secs:.2f} s', detail)


def check_reduced_speed(man, rows, events):
    clause = 'ISO 10218-1:2025 5.5.3 / -2:2025 5.5.6'
    mode = man.get('iso_layer_active', {}).get('iso_mode', 'automatic')
    lim = float(man['iso_params'].get('iso_tcp_reduced_speed', 0.25))
    v = col(rows, 'tcp_speed_max')
    if str(mode) != 'reduced':
        return Result('reduced-speed', clause, '[S] value', NOTLOG,
                      f'the run was in iso_mode={mode}; reduced speed was not '
                      f'claimed, so there is nothing to check')
    if coverage(v) < MIN_COVERAGE:
        return Result('reduced-speed', clause, '[S] value', INCONC,
                      'no TCP speed recorded')
    n, secs, w, wt = exceed_stats(rows, v, lim)
    detail = [f'reduced-speed limit {lim:8.3f} m/s',
              f'peak TCP speed      {w:8.3f} m/s  at t = {wt:.2f} s',
              f'samples over        {n:8d}   ({secs:.2f} s)']
    v_ok = (PASS if n == 0 else FAIL)
    head = (f'TCP stayed under {lim * 1000:.0f} mm/s (peak {w:.3f} m/s)' if n == 0
            else f'TCP exceeded {lim * 1000:.0f} mm/s: peak {w:.3f} m/s for {secs:.2f} s')
    return Result('reduced-speed', clause, '[S] value', v_ok, head, detail)


def check_separation(man, rows, events):
    clause = 'ISO 10218-2:2025 5.14.5 / Annex L'
    m = col(rows, 'margin_min')
    if coverage(m) < MIN_COVERAGE:
        return Result('ssm-separation', clause, '[R]', INCONC,
                      'no valid separation samples — perception was down or '
                      'nothing was in range')
    n, secs, w, wt = exceed_stats(rows, m, 0.0, above=False)
    _, _, lbl = worst(rows, m, 'margin_min_label', lambda a, b: a < b)
    detail = [f'worst margin d − S_p {w:+8.4f} m  at t = {wt:.2f} s'
              + (f'  ({lbl})' if lbl else ''),
              f'samples with d < S_p {n:8d}   ({secs:.2f} s)']
    eps = ev_episodes(events, 'ssm_margin_negative')
    if eps:
        detail.append(f'episodes             {len(eps):8d}   first at t = {eps[0][0]:.2f} s')

    # S_p is linear in C, so the margin WITHOUT the intrusion distance is just
    # margin + C. That is the part the control system governs — the motion
    # terms S_h + S_r + S_s plus the sensing uncertainties — and it is the only
    # actionable number when C alone already exceeds the workspace.
    d1, floor, reach, c = _d1_active(man)
    if d1 and not math.isnan(c):
        m_excl = [x + c for x in m]
        n2, secs2, w2, wt2 = exceed_stats(rows, m_excl, 0.0, above=False)
        detail += ['',
                   f'EXCLUDING C (deviation D1 — see workspace-feasibility):',
                   f'  worst margin       {w2:+8.4f} m  at t = {wt2:.2f} s',
                   f'  samples breaching  {n2:8d}   ({secs2:.2f} s)']
        if n2 == 0:
            return Result('ssm-separation', clause, '[R]', STRUCT,
                          f'breached only because C = {c:.2f} m exceeds the '
                          f'workspace (D1). Excluding C, every control point '
                          f'kept {w2:+.4f} m of margin — the motion terms held',
                          detail)
        return Result('ssm-separation', clause, '[R]', FAIL,
                      f'separation breached even EXCLUDING the intrusion '
                      f'distance: margin reached {w2:+.4f} m for {secs2:.2f} s. '
                      f'This one is not D1 — the motion terms did not hold',
                      detail)

    if n == 0:
        return Result('ssm-separation', clause, '[R]', PASS,
                      f'every control point kept at least {w:+.4f} m of margin '
                      f'over its own separation demand', detail)
    return Result('ssm-separation', clause, '[R]', FAIL,
                  f'separation distance breached: d − S_p reached {w:+.4f} m '
                  f'(needed ≥ 0), for {secs:.2f} s', detail)


def check_speed_cap(man, rows, events):
    clause = 'ISO 10218-2:2025 5.14.5 / Annex L'
    tol = float(man['iso_params'].get('iso_speed_tol', 0.05))
    e = col(rows, 'cap_excess_max')
    if coverage(e) < MIN_COVERAGE:
        return Result('ssm-speed-cap', clause, '[R]', INCONC,
                      'no valid cap samples')
    n, secs, w, wt = exceed_stats(rows, e, tol)
    detail = [f'tolerance            {tol:8.3f} m/s',
              f'worst v_closing − cap{w:+8.4f} m/s  at t = {wt:.2f} s',
              f'samples over         {n:8d}   ({secs:.2f} s)']
    if n == 0:
        return Result('ssm-speed-cap', clause, '[R]', PASS,
                      f'closing speed stayed within the SSM cap '
                      f'(worst excess {w:+.4f} m/s)', detail)
    return Result('ssm-speed-cap', clause, '[R]', FAIL,
                  f'closing speed exceeded the SSM cap by up to {w:+.4f} m/s '
                  f'for {secs:.2f} s', detail)


def check_floor(man, rows, events):
    clause = 'ISO 13855:2024 (C) + ISO 10218-2:2025 Annex L'
    floor = float(man.get('limits', {}).get('floor_c_zd_zr', float('nan')))
    n_in = col(rows, 'n_inside_floor')
    d = col(rows, 'd_min')
    if coverage(d) < MIN_COVERAGE:
        return Result('irreducible-floor', clause, '[R]', INCONC,
                      'no valid distance samples')
    n, secs, _, _ = exceed_stats(rows, n_in, 0.0)
    w, wt, lbl = worst(rows, d, 'd_min_label', lambda a, b: a < b)
    detail = [f'C + Z_d + Z_r        {floor:8.3f} m',
              f'closest approach     {w:8.4f} m  at t = {wt:.2f} s'
              + (f'  ({lbl})' if lbl else ''),
              f'samples inside       {n:8d}   ({secs:.2f} s)']
    d1, _, reach, c = _d1_active(man)
    if n == 0:
        return Result('irreducible-floor', clause, '[R]', PASS,
                      f'nothing ever came inside {floor:.3f} m '
                      f'(closest {w:.4f} m)', detail)
    if d1:
        detail += ['',
                   f'The floor ({floor:.3f} m) exceeds the reach ({reach:.3f} m),',
                   f'so anything perception can see at all is "inside" it. This',
                   f'counts D1, not an approach the controller allowed.',
                   f'Against d_safe = {man.get("limits", {}).get("d_safe", float("nan")):.3f} m'
                   f' the closest approach was {w:.4f} m.']
        return Result('irreducible-floor', clause, '[R]', STRUCT,
                      f'everything is inside the {floor:.3f} m floor because the '
                      f'floor exceeds the {reach:.3f} m reach (D1); closest '
                      f'approach was {w:.4f} m', detail)
    return Result('irreducible-floor', clause, '[R]', FAIL,
                  f'a control point entered the irreducible {floor:.3f} m '
                  f'(closest {w:.4f} m) for {secs:.2f} s', detail)


def check_joint_speed(man, rows, events):
    clause = 'ISO 10218-1:2025 5.5.3 (speed limiting)'
    margin = float(man.get('limits', {}).get('velocity_box_margin', 1.0))
    r = col(rows, 'qdot_ratio_max_run')
    if coverage(r) < MIN_COVERAGE:
        return Result('joint-speed-box', clause, '[E]', INCONC,
                      'no joint-speed samples')
    n, secs, w, wt = exceed_stats(rows, r, margin)
    _, _, j = worst(rows, r, 'qdot_ratio_joint', lambda a, b: a > b)
    detail = [f'software box         {margin:8.1%} of q̇_max',
              f'peak |q̇|/q̇_max       {w:8.1%}  at t = {wt:.2f} s'
              + (f'  (joint {j})' if j else ''),
              f'samples over         {n:8d}   ({secs:.2f} s)']
    if n == 0:
        return Result('joint-speed-box', clause, '[E]', PASS,
                      f'joint speed peaked at {w:.1%} of the limit, inside the '
                      f'{margin:.0%} box', detail)
    return Result('joint-speed-box', clause, '[E]', FAIL,
                  f'joint speed reached {w:.1%} of q̇_max, over the {margin:.0%} '
                  f'software box, for {secs:.2f} s', detail)


def check_stops(man, rows, events):
    """Did a commanded stop behave the way ``iso_a_stop`` assumes?"""
    clause = 'ISO 10218-1:2025 5.5.6 / 5.5.7 / Annex H'
    a_s = float(man['iso_params'].get('iso_a_stop', float('nan')))
    eps = ev_episodes(events, 'iso_stop_latched', 'iso_stop_cleared')
    if not eps:
        return Result('stop-behaviour', clause, '[S] procedure, [E] values',
                      INCONC,
                      'no stop was latched in this run — nothing to measure. '
                      'Provoke one (or run iso_constants_measure.py stop) if you '
                      'want a_s validated')
    ts = col(rows, 't')
    v = col(rows, 'tcp_speed')
    detail, bad = [], 0
    for i, (t_on, t_off, why) in enumerate(eps, 1):
        v0 = next((vv for tt, vv in zip(ts, v)
                   if not math.isnan(tt) and tt >= t_on and not math.isnan(vv)), float('nan'))
        t_rest = next((tt for tt, vv in zip(ts, v)
                       if not math.isnan(tt) and tt >= t_on
                       and not math.isnan(vv) and vv <= 0.01), float('nan'))
        t_s = t_rest - t_on if not math.isnan(t_rest) else float('nan')
        a_real = (v0 / t_s) if (t_s and t_s > 1e-6 and not math.isnan(v0)) else float('nan')
        detail.append(
            f'stop {i}: t = {t_on:7.2f} s  v_tcp = {v0:.3f} m/s  '
            f'T_s = {t_s:.3f} s  a_realized = {a_real:.3f} m/s²  [{why}]')
        if not math.isnan(a_real) and not math.isnan(a_s) and a_real < a_s:
            bad += 1
    detail.append(f'iso_a_stop assumed   {a_s:8.3f} m/s²')
    if bad:
        return Result('stop-behaviour', clause, '[S] procedure, [E] values', FAIL,
                      f'{bad} of {len(eps)} stops decelerated SLOWER than '
                      f'iso_a_stop = {a_s:.3f} m/s² — every stopping distance '
                      f'derived from it is optimistic', detail)
    return Result('stop-behaviour', clause, '[S] procedure, [E] values', PASS,
                  f'{len(eps)} stop(s), all at or above the assumed '
                  f'{a_s:.3f} m/s²', detail)


def check_braking_authority(man, rows, events):
    clause = 'ISO 10218-1:2025 5.5.6 (stopping time limiting)'
    fmin = float(man['iso_params'].get('iso_brake_frac_min', 0.5))
    f = col(rows, 'brake_frac')
    if coverage(f) < 0.05:
        return Result('braking-authority', clause, '[E]', INCONC,
                      'the arm was barely commanded to accelerate — nothing to '
                      'judge (brake_frac is only defined above 0.5 rad/s²)')
    n, secs, w, wt = exceed_stats(rows, f, fmin, above=False)
    tot = len(finite(f))
    detail = [f'floor                {fmin:8.2f}',
              f'worst realized/cmd   {w:8.2f}  at t = {wt:.2f} s',
              f'samples under        {n:8d} of {tot}  ({secs:.2f} s)']
    frac = n / tot if tot else 0.0
    if frac < 0.05:
        return Result('braking-authority', clause, '[E]', PASS,
                      f'the arm delivered the commanded acceleration in '
                      f'{1 - frac:.0%} of samples', detail)
    return Result('braking-authority', clause, '[E]', FAIL,
                  f'realized acceleration was below {fmin:.0%} of commanded in '
                  f'{frac:.0%} of samples — a_s is not being delivered', detail)


def check_saturation(man, rows, events):
    clause = 'ISO 10218-1:2025 5.5.6 (command feasibility)'
    s = col(rows, 'tau_sat_count')
    if coverage(s) < MIN_COVERAGE:
        return Result('torque-saturation', clause, '[E]', INCONC,
                      '/torque_saturation was not published — is qddot_to_torque '
                      'the version with the ISO layer?')
    n, secs, w, wt = exceed_stats(rows, s, 0.0)
    tot = len(finite(s))
    frac = n / tot if tot else 0.0
    detail = [f'samples with a saturated joint {n:6d} of {tot}  ({frac:.1%}, {secs:.2f} s)',
              f'worst simultaneous joints      {w:6.0f}  at t = {wt:.2f} s']
    if frac < 0.01:
        return Result('torque-saturation', clause, '[E]', PASS,
                      f'the torque command was inside the joint limits in '
                      f'{1 - frac:.1%} of samples', detail)
    return Result('torque-saturation', clause, '[E]', FAIL,
                  f'the torque command hit a joint bound in {frac:.1%} of '
                  f'samples — the commanded deceleration is not physically '
                  f'available', detail)


def check_chain_faults(man, rows, events):
    clause = 'ISO 13849-1:2023 (fault behaviour)'
    f = col(rows, 'cbf_fault')
    if coverage(f) < MIN_COVERAGE:
        return Result('safety-chain-faults', clause, '[E]', INCONC,
                      '/cbf_status was not published')
    n, secs, _, _ = exceed_stats(rows, f, 0.5)
    tot = len(finite(f))
    frac = n / tot if tot else 0.0
    eps = ev_episodes(events, 'cbf_fault_on', 'cbf_fault_off')
    detail = [f'fault_braking active {n:8d} of {tot}  ({frac:.1%}, {secs:.2f} s)',
              f'episodes             {len(eps):8d}']
    for t_on, t_off, why in eps[:5]:
        detail.append(f'  t = {t_on:7.2f} s  '
                      + (f'for {t_off - t_on:.2f} s' if t_off else '(never cleared)'))
    if frac < 0.01:
        return Result('safety-chain-faults', clause, '[E]', PASS,
                      f'the safety chain reported a fault in {frac:.1%} of samples',
                      detail)
    return Result('safety-chain-faults', clause, '[E]', FAIL,
                  f'the safety chain was FAULTED for {frac:.1%} of the run — '
                  f'every other verdict here is about a chain that was partly '
                  f'blind', detail)


def check_two_channels(man, rows, events):
    """Does the logger's independent reading agree with what the stack said?"""
    clause = '—'
    mine = col(rows, 'd_min')
    theirs = col(rows, 'cbf_d_min')
    pairs = [(a, b) for a, b in zip(mine, theirs)
             if not math.isnan(a) and not math.isnan(b) and math.isfinite(b)]
    if len(pairs) < 20:
        return Result('cross-check', clause, '[E]', INCONC,
                      'not enough samples where both this logger and the filter '
                      'reported a distance')
    diffs = [abs(a - b) for a, b in pairs]
    w = max(diffs)
    med = sorted(diffs)[len(diffs) // 2]
    detail = [f'samples compared     {len(pairs):8d}',
              f'median |Δd_min|      {med:8.4f} m',
              f'worst  |Δd_min|      {w:8.4f} m']
    if w < 0.02:
        return Result('cross-check', clause, '[E]', PASS,
                      f'this logger and cbf_safety_filter agree on the closest '
                      f'gap to within {w * 1000:.0f} mm', detail)
    return Result('cross-check', clause, '[E]', FAIL,
                  f'this logger and cbf_safety_filter disagree on the closest '
                  f'gap by up to {w * 1000:.0f} mm — one of the two is reading '
                  f'the scene wrong, and that is the finding', detail)


CHECKS = [check_workspace_feasibility, check_integrity, check_tcp_speed, check_reduced_speed,
          check_separation, check_speed_cap, check_floor, check_joint_speed,
          check_stops, check_braking_authority, check_saturation,
          check_chain_faults, check_two_channels]

#: Questions no log can answer, printed so they cannot be mistaken for
#: questions that passed.
UNANSWERABLE = [
    ('PL d / SIL 2 rating', 'ISO 13849-1:2023 / IEC 62061:2021',
     'The chain is single-channel Python over best-effort DDS. No log shows a '
     'performance level; a rating comes from architecture, diagnostic coverage '
     'and failure rates.'),
    ('PFL biomechanical limits', 'ISO 10218-2:2025 6.3.3 / Annex N',
     'Requires force and pressure MEASUREMENT with a PFMD. iso_v_pfl is a '
     'calculation; no speed log substitutes for it.'),
    ('Detection capability d → C', 'ISO 13855:2024',
     'Requires a dedicated test with objects of known size across the field of '
     'view. Run scripts/iso_constants_measure.py detection.'),
    ('Stopping data, categories 0 and 1', 'ISO 10218-1:2025 Annex H',
     'Firmware/brake events. Only a category 2 stop is visible to this stack; '
     'take 0 and 1 from the product manual.'),
    ('Risk assessment and validation', 'ISO 10218-2:2025 clause 6',
     'A document and a competent person. Not a property of a run.'),
]


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run_dir')
    ap.add_argument('--verbose', action='store_true', help='show every detail line')
    ap.add_argument('--json', default='', help='also write the verdicts as JSON')
    args = ap.parse_args()

    man, rows, events = load(args.run_dir)
    results = [c(man, rows, events) for c in CHECKS]

    active = man.get('iso_layer_active', {})
    git = man.get('git', {}) or {}
    print('=' * 78)
    print('ISO EVIDENCE REPORT  —  NOT A CONFORMITY STATEMENT')
    print('=' * 78)
    print(f'  run            {args.run_dir}')
    print(f'  recorded       {man.get("created", "?")}')
    print(f'  config         {man.get("config_path", "?")}')
    print(f'  git            {str(git.get("sha") or "?")[:12]}'
          f'{" (DIRTY)" if git.get("dirty") else ""}  '
          f'branch {git.get("branch", "?")}')
    print(f'  ISO layer      iso_enabled={active.get("iso_enabled")}  '
          f'mode={active.get("iso_mode")}  '
          f'ssm_rows={active.get("iso_ssm_speed_rows")}  '
          f'monitor={active.get("iso_monitor_enabled")}')
    if not active.get('iso_enabled'):
        print('                 (the layer was OFF — the criteria below were '
              'evaluated anyway,')
        print('                  so this says what the CURRENT system would and '
              'would not satisfy)')
    print()

    width = max(len(r.key) for r in results)
    for r in results:
        print(f'  {r.verdict:<13} {r.key:<{width}}  {r.headline}')
        if args.verbose or r.verdict in (FAIL, STRUCT):
            print(f'  {"":<13} {"":<{width}}  clause: {r.clause}  {r.tag}')
            for d in r.detail:
                print(f'  {"":<13} {"":<{width}}    {d}')
            print()

    n_fail = sum(1 for r in results if r.verdict == FAIL)
    n_inc = sum(1 for r in results if r.verdict == INCONC)
    n_pass = sum(1 for r in results if r.verdict == PASS)
    n_str = sum(1 for r in results if r.verdict == STRUCT)
    print()
    print(f'  {n_pass} passed, {n_fail} failed, {n_inc} inconclusive, '
          f'{n_str} structural')
    if n_str:
        print('  STRUCTURAL = the configuration asks for something this '
              'workspace cannot deliver.')
        print('  No run avoids it and no tuning fixes it. See SAFETY.md '
              'deviation D1.')
    print()
    print('-' * 78)
    print('WHAT THIS REPORT CANNOT TELL YOU')
    print('-' * 78)
    for name, clause, why in UNANSWERABLE:
        print(f'  {name}')
        print(f'    {clause}')
        print(f'    {why}')
    print()
    if n_fail:
        print(f'  {n_fail} check(s) FAILED: the cell did not respect limits its '
              f'own configuration')
        print('  claims. Fix those before any of this is worth assessing.')
    elif n_str:
        print('  Nothing this run controls went wrong. What failed is '
              'STRUCTURAL: the')
        print('  configured intrusion distance is larger than the workspace, '
              'so conformant')
        print('  SSM is unachievable here regardless of how the robot moves. '
              'That is a')
        print('  cell-design finding, not a controller finding.')
    elif n_inc:
        print('  No check failed, but some were INCONCLUSIVE — a limit nobody '
              'measured is not')
        print('  a limit that held. Re-run covering the missing channels.')
    else:
        print('  Every check passed. That means the implemented measures behaved '
              'as designed')
        print('  IN THIS RUN. It does not mean the cell is compliant, and one '
              'run is not a')
        print('  validation — see franka_experiments/SAFETY.md.')

    if args.json:
        with open(args.json, 'w') as fh:
            json.dump({'run_dir': args.run_dir, 'manifest': man,
                       'results': [r.as_dict() for r in results]},
                      fh, indent=2, default=str)
        print(f'\n  verdicts also written to {args.json}')
    return 1 if n_fail else 0


if __name__ == '__main__':
    sys.exit(main())
