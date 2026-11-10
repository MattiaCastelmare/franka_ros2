"""First compression/release cycle from recorded wall-contact signals.

The interval starts at the first recorded positive normal force and ends at
the first detachment or the onset of recompression, whichever occurs first.
Positive normal velocity means motion into the wall. This is a sampled
first-impact metric, not a measure of energy absorbed by an object.
"""
from __future__ import annotations

import numpy as np


def _scalar(run, key, samples, *, finite=True):
    if key not in run:
        return None
    values = np.asarray(run[key], dtype=float)
    if values.shape != (samples,):
        raise ValueError(f'{key} must contain one scalar per timestamp')
    if finite and not np.all(np.isfinite(values)):
        raise ValueError(f'{key} must contain finite values')
    return values


def first_impact(run: dict) -> dict:
    """Return raw signals, first-cycle curves and scalar ``metrics``.

    Raw keys are ``t``, ``normal_force_N``, ``normal_speed_mps``,
    ``penetration_mm`` and ``mass_kg``. ``contact_start_index`` and
    ``precontact_index`` identify recorded samples. The ``impact_t`` curve
    uses absolute simulation time; ``impact_force_N``,
    ``impact_penetration_mm`` and ``impact_impulse_Ns`` share its timestamps.

    Recompression begins when velocity first becomes positive after a
    strictly negative release velocity. Its zero crossing is interpolated;
    detachment uses the first nonpositive-force sample. Impulse starts at
    zero at the recorded onset and uses trapezoids with actual timestamps.
    A log ending first, or starting already in contact, gives partial metrics
    and ``first_impact_complete`` false. Without normal velocity, the cycle
    cannot be isolated reliably.

    Entry mass/speed use only the sample before contact. Inactive or invalid
    logged mass is unavailable; no model is evaluated and no current-contact
    sample substitutes for missing precontact data.
    """
    t = np.asarray(run['t'], dtype=float)
    if (t.ndim != 1 or not np.all(np.isfinite(t))
            or np.any(np.diff(t) <= 0.0)):
        raise ValueError('First-impact metrics require finite, increasing timestamps')
    force = _scalar(run, 'F_n', len(t))
    speed = _scalar(run, 'v_n', len(t))
    delta = _scalar(run, 'delta', len(t))
    mass = _scalar(run, 'user_mass_kg', len(t), finite=False)
    metric_keys = (
        't_contact_s', 'first_impact_end_s', 'first_impact_duration_s',
        'first_impact_complete', 'first_impact_end_reason',
        'precontact_sample_s', 'entry_mass_kg', 'entry_speed_mps',
        'entry_normal_kinetic_energy_J', 'peak_force_N',
        'normal_impulse_Ns', 'peak_penetration_mm',
    )
    metrics = dict.fromkeys(metric_keys)
    metrics['contact'] = False
    empty = np.empty(0, dtype=float)
    out = {
        't': t, 'normal_force_N': force, 'normal_speed_mps': speed,
        'penetration_mm': None if delta is None else 1e3 * np.maximum(delta, 0.0),
        'mass_kg': mass, 'contact_start_index': None, 'precontact_index': None,
        'impact_t': empty, 'impact_force_N': empty.copy(),
        'impact_penetration_mm': None if delta is None else empty.copy(),
        'impact_impulse_Ns': empty.copy(), 'metrics': metrics,
    }
    contact = np.flatnonzero(force > 0.0) if force is not None else empty
    if not contact.size:
        return out
    i0 = int(contact[0])
    out['contact_start_index'] = i0
    metrics.update(contact=True, t_contact_s=float(t[i0]))
    if i0 > 0:
        before = i0 - 1
        out['precontact_index'] = before
        metrics['precontact_sample_s'] = float(t[before])
        if mass is not None and np.isfinite(mass[before]) and mass[before] > 0.0:
            metrics['entry_mass_kg'] = float(mass[before])
        if speed is not None:
            metrics['entry_speed_mps'] = float(speed[before])
        if metrics['entry_mass_kg'] is not None and metrics['entry_speed_mps'] is not None:
            metrics['entry_normal_kinetic_energy_J'] = (
                0.5 * metrics['entry_mass_kg'] * metrics['entry_speed_mps']**2)
    if speed is None:
        metrics.update(first_impact_complete=False,
                       first_impact_end_reason='normal_speed_not_logged')
        return out

    events = []
    detached = np.flatnonzero(force[i0 + 1:] <= 0.0)
    if detached.size:
        events.append((float(t[i0 + 1 + int(detached[0])]), 'detachment'))
    released = np.flatnonzero(speed[i0:] < 0.0)
    if released.size:
        release_index = i0 + int(released[0])
        positive = np.flatnonzero(speed[release_index + 1:] > 0.0)
        if positive.size:
            right = release_index + 1 + int(positive[0])
            left = right - 1
            fraction = -speed[left] / (speed[right] - speed[left])
            crossing = float(t[left] + fraction * (t[right] - t[left]))
            events.append((crossing, 'recompression'))
    if events:
        end, reason = min(events, key=lambda event: event[0])
    else:
        end, reason = float(t[-1]), 'end_of_log'
    interval_t = t[i0:np.searchsorted(t, end, side='right')].copy()
    if interval_t[-1] < end:
        interval_t = np.append(interval_t, end)
    interval_force = np.interp(interval_t, t, force)
    impulse = np.zeros(len(interval_t), dtype=float)
    impulse[1:] = np.cumsum(
        0.5 * (interval_force[:-1] + interval_force[1:]) * np.diff(interval_t))
    penetration = (None if delta is None else
                   1e3 * np.maximum(np.interp(interval_t, t, delta), 0.0))
    out.update(impact_t=interval_t, impact_force_N=interval_force,
               impact_penetration_mm=penetration, impact_impulse_Ns=impulse)
    metrics.update({
        'first_impact_end_s': end,
        'first_impact_duration_s': end - float(t[i0]),
        'first_impact_complete': reason != 'end_of_log' and i0 > 0,
        'first_impact_end_reason': reason,
        'peak_force_N': float(np.max(interval_force)),
        'normal_impulse_Ns': float(impulse[-1]),
        'peak_penetration_mm': None if penetration is None else float(np.max(penetration)),
    })
    return out
