"""Where the real FR3 would have aborted: the firmware checks Gazebo does not run (plan H5).

Gazebo has no velocity-violation reflex, no torque-rate limiter and no
position-limit reflex, so a sim run can "pass" where the robot would have
stopped with red lights. ``ReflexChecker`` replays those checks on every
1 kHz joint-state sample and reports each tick at which the real robot WOULD
have tripped. Any trip makes a run a FAIL.

The limits are the robot's own, imported rather than restated:

* velocity: ``cbf_hard_limits.fr3_velocity_envelope(q, margin=1.0)``, the
  libfranka position-dependent curve that ``joint_velocity_violation`` applies
  (rate_limiting.h), plus libfranka's 1e-3 rad/s tolerance;
* position: the mechanical limits (``joint_position_limits_violation``);
* torque: |tau| > effort limit (87/12 N·m, franka_description) on the TOTAL
  joint torque the actuator applies (in Gazebo: the JSB effort field, which
  Phase 1 measured to equal tau_cmd + g(q));
* torque rate: |d tau_user / dt| > 1000 N·m/s (libfranka ``kMaxTorqueRate``) on
  the user torque (effort − g(q)). On THIS robot that is not a reflex:
  franka_hardware keeps libfranka's rate limiter on
  (``torque_command_rate_limiter_active_{true}``, robot.hpp:318, applied in
  robot.cpp:163-166), so the command is CLIPPED to 1 N·m per ms, not aborted.
  These ticks are therefore counted separately as ``torque_rate_clipped``.
  They are not trips, but they are a fidelity gap: Gazebo executed a torque
  the robot would have smoothed. (The spec listed torque rate as a trip; the
  code says otherwise, plan §2 H5.)

Not checked (no counterpart in sim): the collision reflexes against the
SetFullCollisionBehavior thresholds (plan §3.3), communication reflexes.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

#: libfranka rate_limiting.h: kMaxTorqueRate = 1000 - kLimitEps, kJointVelocityLimitsTolerance ≈ kLimitEps.
TORQUE_RATE_MAX = 1000.0 - 1e-3
VEL_TOL = 1e-3
#: Abort reflexes (a trip = the run FAILS) and the limiter event (counted, not a trip).
TRIP_KINDS = ('velocity', 'position', 'torque')
CLIP_KIND = 'torque_rate_clipped'
KINDS = TRIP_KINDS + (CLIP_KIND,)


def robot_limits() -> dict:
    """The FR3 limits, from franka_experiments (= franka_description joint_limits.yaml)."""
    from franka_experiments.utils.cbf_state_rows import FR3_JOINT_KEYS
    from franka_experiments.utils.config import load_franka_joint_limits
    jl = load_franka_joint_limits(FR3_JOINT_KEYS)
    return dict(q_min=np.asarray(jl['q_min'], float), q_max=np.asarray(jl['q_max'], float),
                tau_max=np.asarray(jl['effort_max'], float))


def envelope(q):
    from franka_experiments.utils.cbf_hard_limits import fr3_velocity_envelope
    return fr3_velocity_envelope(np.asarray(q, float), margin=1.0)


@dataclass
class ReflexChecker:
    q_min: np.ndarray
    q_max: np.ndarray
    tau_max: np.ndarray
    rate_max: float = TORQUE_RATE_MAX
    vel_tol: float = VEL_TOL
    max_gap_s: float = 0.0025          # longer gaps (dropped samples) do not count as rate
    counts: dict = field(default_factory=lambda: {k: 0 for k in KINDS})
    worst: dict = field(default_factory=lambda: {'vel_ratio': 0.0, 'tau_ratio': 0.0,
                                                 'rate_nms': 0.0, 'pos_margin_rad': np.inf})
    n_samples: int = 0
    _prev: tuple | None = None

    def check(self, t: float, q, qdot, tau_total, tau_user) -> list[tuple]:
        """Events at this sample: [(kind, joint_index, value, limit), ...] (trips and clips)."""
        q, qdot = np.asarray(q, float), np.asarray(qdot, float)
        tau_total, tau_user = np.asarray(tau_total, float), np.asarray(tau_user, float)
        self.n_samples += 1
        trips = []
        up, lo = envelope(q)
        for i in range(7):
            if qdot[i] > up[i] + self.vel_tol:
                trips.append(('velocity', i, float(qdot[i]), float(up[i])))
            elif qdot[i] < lo[i] - self.vel_tol:
                trips.append(('velocity', i, float(qdot[i]), float(lo[i])))
            if q[i] < self.q_min[i] or q[i] > self.q_max[i]:
                trips.append(('position', i, float(q[i]),
                              float(self.q_min[i] if q[i] < self.q_min[i] else self.q_max[i])))
            if abs(tau_total[i]) > self.tau_max[i]:
                trips.append(('torque', i, float(tau_total[i]), float(self.tau_max[i])))
        # velocity ratio against the side the joint is moving toward
        side = np.where(qdot >= 0.0, up, -lo)
        with np.errstate(divide='ignore', invalid='ignore'):
            vr = np.where(side > 1e-9, np.abs(qdot) / side, np.where(np.abs(qdot) > self.vel_tol,
                                                                      np.inf, 0.0))
        self.worst['vel_ratio'] = max(self.worst['vel_ratio'], float(np.max(vr)))
        self.worst['tau_ratio'] = max(self.worst['tau_ratio'],
                                      float(np.max(np.abs(tau_total) / self.tau_max)))
        self.worst['pos_margin_rad'] = min(self.worst['pos_margin_rad'],
                                           float(np.min(np.minimum(q - self.q_min, self.q_max - q))))
        if self._prev is not None:
            t0, u0 = self._prev
            dt = t - t0
            if 0.0 < dt <= self.max_gap_s:
                rate = np.abs(tau_user - u0) / dt
                self.worst['rate_nms'] = max(self.worst['rate_nms'], float(np.max(rate)))
                for i in np.flatnonzero(rate > self.rate_max):
                    trips.append((CLIP_KIND, int(i), float(rate[i]), float(self.rate_max)))
        self._prev = (t, tau_user)
        for k, *_ in trips:
            self.counts[k] += 1
        return trips

    @property
    def n_trips(self) -> int:
        """Abort reflexes only; limiter clips are in ``counts[CLIP_KIND]``."""
        return int(sum(self.counts[k] for k in TRIP_KINDS))


class RtfMeter:
    """Real-time factor from (sim, wall) pairs: per-window values and the run total."""

    def __init__(self, window_s: float = 1.0):
        self.window_s = window_s
        self.first = None
        self.win_start = None
        self.last = None
        self.windows: list[float] = []

    def push(self, sim_s: float, wall_s: float) -> None:
        if self.first is None:
            self.first = self.win_start = (sim_s, wall_s)
        self.last = (sim_s, wall_s)
        if wall_s - self.win_start[1] >= self.window_s:
            self.windows.append((sim_s - self.win_start[0]) / (wall_s - self.win_start[1]))
            self.win_start = (sim_s, wall_s)

    def summary(self, flag_below: float = 0.9) -> dict:
        if self.first is None or self.last[1] <= self.first[1]:
            return {'rtf_run': None, 'rtf_min_window': None, 'frac_windows_below': None,
                    'flagged': True}
        rtf = (self.last[0] - self.first[0]) / (self.last[1] - self.first[1])
        w = np.asarray(self.windows) if self.windows else np.asarray([rtf])
        return {'rtf_run': float(rtf), 'rtf_min_window': float(w.min()),
                'rtf_p10_window': float(np.percentile(w, 10)),
                'frac_windows_below': float(np.mean(w < flag_below)),
                'flag_below': flag_below, 'flagged': bool(rtf < flag_below),
                'sim_s': float(self.last[0] - self.first[0]),
                'wall_s': float(self.last[1] - self.first[1])}
