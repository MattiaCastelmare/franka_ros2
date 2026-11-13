"""Timing summaries kept separate from contact and controller diagnostics.

``ctrl_ms`` measures wall time inside ``Controller.update`` only. Comparing it
with ``1000 / control_rate`` is a nominal real-time (RTF = 1) comparison, not a
measurement of the complete callback deadline. Plant sample gaps are measured
separately in simulation time.
``callback_ms`` additionally measures the RUN callback work, excluding final
file saving, executor wait time and transport before callback entry.
"""
from __future__ import annotations

import numpy as np


def timing_metrics(run: dict) -> dict:
    """Summarize actual controller calls and the 1 kHz plant sample stream.

    New logs mark controller calls with ``controller_updated``. In old logs the
    last ``ctrl_ms`` is repeated on samples without an update, so it must not be
    counted as another call. The fallback only accepts 1 kHz runs where every
    recorded interval allows an update and no final safety/controller exception
    could have left a repeated measurement. Otherwise call statistics are None.
    Missing plant samples count only the interval covered by the saved log.
    """
    t = np.asarray(run['t'], dtype=float)
    dt = np.diff(t)
    meta = run.get('meta', {})
    rate = float(meta.get('control_rate', 1000.0))
    if not np.isfinite(rate) or rate <= 0.0:
        raise ValueError('control_rate must be finite and positive')
    period = 1.0 / rate
    expected = max(round((t[-1] - t[0]) / 1e-3) + 1, 1) if t.size else 0
    missing = max(expected - len(t), 0)
    gaps = int(np.count_nonzero(dt > 1e-3 + 1e-7))
    out = {
        'expected_samples': int(expected),
        'missed_samples': int(missing),
        'missed_samples_pct': 100.0 * missing / expected if expected else 0.0,
        'sample_intervals_over_1ms': gaps,
        'sample_intervals_over_1ms_pct': 100.0 * gaps / len(dt) if len(dt) else 0.0,
        'sample_interval_ms_max': 1e3 * float(np.max(dt)) if len(dt) else 0.0,
        'controller_period_ms': 1e3 * period,
        'controller_timing_scope': 'Controller.update wall time; comparison assumes RTF=1',
    }

    durations = np.asarray(run['ctrl_ms'], dtype=float)
    if durations.shape != t.shape:
        raise ValueError('ctrl_ms must contain one value per timestamp')
    if 'controller_updated' in run:
        updated = np.asarray(run['controller_updated'], dtype=bool)
        if updated.shape != t.shape:
            raise ValueError('controller_updated must contain one flag per timestamp')
        source = 'logged update flags'
    elif (np.isclose(rate, 1000.0) and np.all(dt >= period - 1e-6)
          and not str(meta.get('stop_reason', '')).startswith(('SAFETY', 'exception'))):
        updated = np.ones(t.shape, dtype=bool)
        source = 'inferred: legacy 1 kHz run with an update at every sample'
    else:
        updated = None
        source = 'unavailable: legacy log has no controller update flags'
    out['controller_timing_source'] = source
    calls = durations[updated] if updated is not None else None
    if calls is not None and calls.size and not np.all(np.isfinite(calls)):
        raise ValueError('controller update times must be finite')
    count = int(calls.size) if calls is not None else None
    over = int(np.count_nonzero(calls > 1e3 * period)) if count else (0 if count == 0 else None)
    out.update({
        'controller_updates': count,
        'controller_ms_mean': float(np.mean(calls)) if count else None,
        'controller_ms_p95': float(np.percentile(calls, 95)) if count else None,
        'controller_ms_max': float(np.max(calls)) if count else None,
        'controller_update_over_period_count': over,
        'controller_update_over_period_pct': 100.0 * over / count if count else None,
    })
    requested_rtf = meta.get('real_time_factor_requested')
    if requested_rtf is not None:
        requested_rtf = float(requested_rtf)
        if not np.isfinite(requested_rtf) or requested_rtf <= 0.0:
            raise ValueError('real_time_factor_requested must be finite and positive')
    controller_wall_period = 1e3 * period / requested_rtf if requested_rtf else None
    plant_wall_period = 1.0 / requested_rtf if requested_rtf else None
    wall_over = (int(np.count_nonzero(calls > controller_wall_period))
                 if count is not None and controller_wall_period is not None else None)
    out.update({
        'real_time_factor_requested': requested_rtf,
        'controller_wall_period_ms_requested': controller_wall_period,
        'plant_wall_period_ms_requested': plant_wall_period,
        'controller_update_over_requested_wall_period_count': wall_over,
        'controller_update_over_requested_wall_period_pct':
        100.0 * wall_over / count if count and wall_over is not None else None,
    })
    callbacks = np.asarray(run['callback_ms'], dtype=float) if 'callback_ms' in run else None
    if callbacks is not None:
        if callbacks.shape != t.shape or not np.all(np.isfinite(callbacks)) or np.any(callbacks < 0):
            raise ValueError('callback_ms must contain one finite nonnegative value per timestamp')
    callback_count = int(callbacks.size) if callbacks is not None else None
    callback_over = (int(np.count_nonzero(callbacks > plant_wall_period))
                     if callback_count is not None and plant_wall_period is not None else None)
    out.update({
        'callback_timing_scope': 'RUN callback work; excludes save/shutdown, executor wait and transport',
        'callback_samples': callback_count,
        'callback_ms_mean': float(np.mean(callbacks)) if callback_count else None,
        'callback_ms_p95': float(np.percentile(callbacks, 95)) if callback_count else None,
        'callback_ms_max': float(np.max(callbacks)) if callback_count else None,
        'callback_over_requested_wall_period_count': callback_over,
        'callback_over_requested_wall_period_pct':
        100.0 * callback_over / callback_count if callback_count and callback_over is not None else None,
    })
    return out
