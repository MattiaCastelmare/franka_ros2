"""Offline contact summary from signals already stored in a run log.

Positive outgoing contact work integrates positive power leaving the robot
through contact, using logged external joint torques and joint velocities.
External torque acts on the robot. This sampled estimate is neither motor work,
a damage metric, nor an exact applied-energy measurement from the simulator.
No dynamics are evaluated.
"""
from __future__ import annotations

import numpy as np


def _logged_scalar(run: dict, key: str, samples: int):
    """Read an optional scalar time series without replacing inactive NaNs."""
    if key not in run:
        return None
    values = np.asarray(run[key], dtype=float)
    if values.shape != (samples,):
        raise ValueError(f'{key} must contain one scalar per timestamp')
    return values


def collision_signals(run: dict) -> dict:
    """Extract contact diagnostics; missing signals are returned as None.

    Positive outgoing contact power is ``max(-sum(tau_ext * dq), 0)``: the
    generalized dot product is taken before the positive part. ``tau_ext`` is
    the external torque acting on the robot; legacy logs use ``tau_wall`` when
    ``tau_ext`` is absent. No observer residual or actuator torque is used.
    The cumulative trapezoidal integral uses actual logged time intervals over
    the entire run, before the figure selects its window.
    Position error uses the logged reference and measured tool position when
    available, otherwise the logged error vector. Inactive NaNs are preserved.
    """
    t = np.asarray(run['t'], dtype=float)
    if (t.ndim != 1 or not np.all(np.isfinite(t))
            or np.any(np.diff(t) <= 0.0)):
        raise ValueError('Contact summary requires finite, strictly increasing timestamps')
    samples = len(t)
    force = _logged_scalar(run, 'F_n', samples)
    speed = _logged_scalar(run, 'v_n', samples)
    delta = _logged_scalar(run, 'delta', samples)
    mass = _logged_scalar(run, 'user_mass_kg', samples)

    contact_time = None
    if force is not None:
        contact = np.flatnonzero(force > 0.0)
        if contact.size:
            contact_time = float(t[contact[0]])

    work = None
    external_key = 'tau_ext' if 'tau_ext' in run else 'tau_wall'
    if external_key in run and 'dq' in run:
        external_torque = np.asarray(run[external_key], dtype=float)
        joint_velocity = np.asarray(run['dq'], dtype=float)
        if (external_torque.ndim != 2 or external_torque.shape[0] != samples
                or external_torque.shape[1] == 0
                or joint_velocity.shape != external_torque.shape):
            raise ValueError('External torque and dq must have matching (samples, joints) shapes')
        power = np.maximum(-np.sum(external_torque * joint_velocity, axis=1), 0.0)
        work = np.zeros(samples, dtype=float)
        work[1:] = np.cumsum(0.5 * (power[:-1] + power[1:]) * np.diff(t))

    error = None
    if 'user_p_ref' in run and 'p' in run:
        reference = np.asarray(run['user_p_ref'], dtype=float)
        position = np.asarray(run['p'], dtype=float)
        if reference.shape != (samples, 3) or position.shape != (samples, 3):
            raise ValueError('Tool position and reference must contain one 3-vector per timestamp')
        error = reference - position
    elif 'user_position_error' in run:
        error = np.asarray(run['user_position_error'], dtype=float)
        if error.shape != (samples, 3):
            raise ValueError('Position error must contain one 3-vector per timestamp')

    return {
        't': t,
        'mass_kg': mass,
        'normal_speed_mps': speed,
        'normal_force_N': force,
        'penetration_mm': None if delta is None else 1e3 * np.maximum(delta, 0.0),
        'positive_contact_work_J': work,
        'position_error_mm': None if error is None else 1e3 * np.linalg.norm(error, axis=1),
        'contact_time_s': contact_time,
    }


def collision_figure(run: dict, plt):
    """Return a five-panel figure centred on the first recorded wall contact.

    Each axis shows the same half-second interval before and after contact,
    clipped to available samples. With no contact the complete logged interval
    is shown in absolute time. Missing diagnostics are labelled, never inferred
    by evaluating a robot model or substituted with fabricated values.
    """
    signals = collision_signals(run)
    t = signals['t']
    contact_time = signals['contact_time_s']
    relative = contact_time is not None
    time = t - contact_time if relative else t
    if t.size:
        limits = (max(float(time[0]), -0.5), min(float(time[-1]), 0.5)) if relative else (
            float(time[0]), float(time[-1]))
    else:
        limits = None
    visible = ((time >= limits[0]) & (time <= limits[1]) if limits is not None
               else np.ones(t.size, dtype=bool))

    panels = (
        ('mass_kg', 'Directional mass', r'$m_\mathrm{eff}$ [kg]'),
        ('normal_speed_mps', 'Normal speed (+ into wall)', r'$v_n$ [m/s]'),
        ('normal_force_N', 'Normal contact force', r'$F_n$ [N]'),
        ('penetration_mm', 'Penetration', r'$\delta$ [mm]'),
        ('positive_contact_work_J', 'Generalized positive contact work', r'$W_\mathrm{out}^{+}$ [J]'),
    )
    fig = plt.figure(figsize=(11, 10), facecolor='white')
    grid = fig.add_gridspec(3, 2)
    first_axis = fig.add_subplot(grid[0, 0])
    axes = [
        first_axis,
        fig.add_subplot(grid[0, 1], sharex=first_axis),
        fig.add_subplot(grid[1, 0], sharex=first_axis),
        fig.add_subplot(grid[1, 1], sharex=first_axis),
        fig.add_subplot(grid[2, :], sharex=first_axis),
    ]
    experiment_handle = None
    contact_handle = None
    for axis, (key, title, units) in zip(axes, panels):
        axis.set_facecolor('white')
        values = signals[key]
        if values is None:
            axis.text(0.5, 0.5, 'Not logged', transform=axis.transAxes,
                      ha='center', va='center', color='0.45')
        else:
            line, = axis.plot(time[visible], values[visible], color='tab:blue',
                              lw=1.5, label='Experiment')
            if experiment_handle is None:
                experiment_handle = line
        if relative:
            marker = axis.axvline(0.0, color='#c49a00', ls='--', lw=1.1,
                                  label='First contact')
            if contact_handle is None:
                contact_handle = marker
        if limits is not None and limits[0] < limits[1]:
            axis.set_xlim(*limits)
        axis.set_title(title, fontsize=11)
        axis.set_ylabel(units, fontsize=10)
        axis.tick_params(labelsize=9)
        axis.grid(alpha=0.3)
    for axis in axes[:-1]:
        axis.tick_params(labelbottom=False)
    axes[-1].set_xlabel(r'$t-t_c$ [s]' if relative else 't [s]')

    fig.suptitle('Contact summary', fontsize=14, y=0.99)
    subtitle = (f'First contact at t = {contact_time:.3f} s' if relative
                else 'No contact recorded')
    fig.text(0.5, 0.96, subtitle, ha='center', fontsize=10)
    handles = [handle for handle in (experiment_handle, contact_handle) if handle is not None]
    if handles:
        fig.legend(handles=handles, loc='upper center', bbox_to_anchor=(0.5, 0.947),
                   ncol=2, frameon=False, fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.91), h_pad=2.0)
    return fig
