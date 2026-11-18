"""SIM harness (H2): phase-aligned episode start for the pentagon/circle commander.

Opt-in logic of ``episode_supervisor`` with ``start_mode:=phase``. The legacy start
(``start_mode:=legacy``, the default) does not import this module.

Why
---
In the legacy start the commander begins its path on its own timer, independently of the
controller and of the episode, so the phase of the path at the obstacle's birth depends on
wall-clock timers, DDS discovery, lost service responses and the real-time factor (E3-04:
the ball was born 5.9 cm from the hand after a 10 s spawner retry). Here the obstacle is born
only after the arm has tracked one full clean cycle, at a fixed, pre-registered phase.

Phase semantics (verified against pentagon_qddot_commander.py)
--------------------------------------------------------------
* The commander advances ONE phase ``s`` on its node clock (sim time under use_sim_time):
  ``s += s_dot * clip(dt, dt/2, 2 dt) * sigma`` (sigma = obstacle governor, 1 with no obstacle).
* ``s < S_a = approach_time / cycle_time`` is a straight lead-in from the EE position at start;
  ``s >= S_a`` is the cyclic curve evaluated at the local phase ``phi = (s - S_a) mod 1``.
* ``/NS_1/ee_desired`` carries ``P(s)`` of the tick that published it, stamped with the same
  sim stamp as ``/NS_1/ee_actual`` (the measured EE of that tick). It is published only on
  running ticks: not in warm-up, not on a stale joint state, not in a singularity recovery,
  not under an ISO stop. Its FIRST message is the start of the lead-in, not phi = 0, and a
  recovery restarts the lead-in. A HARD reset snaps only the joint reference: the phase keeps
  advancing (E3-04: 4.8 s of path advanced while the controller was inactive).
* Therefore the phase is recovered GEOMETRICALLY from each ee_desired sample (nearest point of
  the commander's own curve, rebuilt from the commander's own parameters), never from the time
  of the first message.

Everything here uses only header stamps (sim time); nothing reads the wall clock, so the real-
time factor can only change the spacing of the samples, never the selected phase.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import asdict, dataclass, field

import numpy as np

#: commander parameters the supervisor reads from the running commander (GetParameters)
COMMANDER_PARAMS = ('path_type', 'center_xyz', 'radius', 'plane', 'cycle_time', 'smoothness',
                    'lissajous_a', 'lissajous_b', 'approach_time', 'timing_law',
                    'hard_reset_thr', 'soft_reset_thr', 'governor_enabled')


def build_cycle_path(p: dict):
    """The commander's cyclic path, built exactly as pentagon_qddot_commander.__init__ does.

    Same classes, same branch order: 'pentagon' -> PentagonPath(PentagonTrajectory),
    'lissajous' -> LissajousPath, anything else -> CircularPath (the commander also falls
    back to a circle on an unknown type).
    """
    from franka_experiments.utils.trajectory import (CircularPath, LissajousPath, PentagonPath,
                                                     PentagonTrajectory)
    center = np.array([float(v) for v in p['center_xyz']])
    kind = str(p['path_type']).lower()
    if kind == 'pentagon':
        return PentagonPath(PentagonTrajectory(center=center, radius=float(p['radius']),
                                               plane=p['plane'], cycle_time=float(p['cycle_time']),
                                               smoothness=float(p['smoothness'])))
    if kind == 'lissajous':
        return LissajousPath(center=center, A=float(p['lissajous_a']), B=float(p['lissajous_b']),
                             plane=p['plane'])
    return CircularPath(center=center, radius=float(p['radius']), plane=p['plane'])


class PhaseEstimator:
    """phi in [0, 1) and the distance from a point to the curve, by nearest point.

    A dense table of P(phi) plus a projection on the two chords next to the nearest sample.
    Exact to ~1e-12 cycles on points that lie on the curve (checked on the E3-03/E3-04 commander
    logs). The curves used here are simple closed curves, so the nearest point is unique for
    points on the curve; the on-path tolerance rejects everything else (the lead-in).
    """

    def __init__(self, path, n: int = 10000):
        self.n = int(n)
        self.tab = np.array([np.asarray(path.position(i / self.n), float).copy()
                             for i in range(self.n)])

    def locate(self, p) -> tuple[float, float]:
        p = np.asarray(p, float)
        d = np.linalg.norm(self.tab - p, axis=1)
        i = int(np.argmin(d))
        best_d, best_phi = float(d[i]), i / self.n
        for j in (i - 1, i):
            a, b = self.tab[j % self.n], self.tab[(j + 1) % self.n]
            ab = b - a
            den = float(np.dot(ab, ab))
            if den <= 0.0:
                continue
            u = min(1.0, max(0.0, float(np.dot(p - a, ab)) / den))
            dd = float(np.linalg.norm(a + u * ab - p))
            if dd < best_d:
                best_d, best_phi = dd, ((j + u) / self.n) % 1.0
        return best_phi % 1.0, best_d


def wrap(x: float) -> float:
    """x mod 1 into [-0.5, 0.5)."""
    return (x + 0.5) % 1.0 - 0.5


@dataclass
class StartConfig:
    """Pre-registered H2 start conditions (defaults = the values proposed for E3-05/06)."""
    phase0: float = 0.0              # [cycles] obstacle birth phase on the cyclic curve
    settle_cycles: float = 1.0       # full clean cycles required BEFORE the announcement
    track_err_max: float = 0.02      # [m] = the commander's soft_reset_thr (no anti-windup action)
    hard_reset_thr: float = 0.05     # [m] the commander's HARD-reset condition (counted)
    on_path_tol: float = 1e-3        # [m] ee_desired farther than this from the curve = not on it
    max_sample_gap_s: float = 0.1    # [s sim] longer ee_desired silence = stale -> restart window
    rate_window_s: float = 1.0       # [s sim] least-squares phase rate over this window
    rate_rel_tol: float = 0.02       # |rate * cycle_time - 1| above this = not at nominal speed
    lead_min_s: float = 0.25         # [s sim] announce at least this far ahead (legacy start_lead_s)
    lead_announce_s: float = 0.5     # [s sim] ... and at most this far ahead
    timeout_s: float = 90.0          # [s sim] from t_first_poll, then the start FAILS explicitly;
    #                                  the supervisor sets t_first_poll to its FIRST sim tick
    #                                  (PREREGISTRATION.md §1, F-8-24), not to its first poll
    cycle_time: float = 6.0          # [s] the commander's own (overwritten from its parameters)


@dataclass
class _Window:
    t0: float
    u0: float


@dataclass
class StartQualifier:
    """Decides WHEN to start; never decides anything after the birth except recording it.

    Feed every (ee_desired, ee_actual) pair with its sim stamp and the controller state, poll
    at the supervisor's tick. States: 'qualifying' -> 'announced' (t_start fixed) or 'failed'.
    """
    cfg: StartConfig
    est: PhaseEstimator
    state: str = 'qualifying'
    t_first_poll: float | None = None
    t_ctrl_active: float | None = None          # first sim time the controller was seen active
    last_t: float | None = None
    last_phi: float | None = None
    u: float = 0.0                              # unwrapped phase of the current window
    win: _Window | None = None
    max_err_win: float = 0.0
    recent: deque = field(default_factory=deque)       # (t, u) for the rate fit
    n_pairs: int = 0
    n_out_of_order: int = 0
    hard_ticks: int = 0                         # |e| > hard_reset_thr (the commander's condition)
    hard_last_t: float | None = None
    resets: dict = field(default_factory=dict)  # reason -> count of window restarts
    reset_log: list = field(default_factory=list)      # first 50 (t, reason, value)
    fail_reason: str | None = None
    decision: dict | None = None
    placement: dict | None = None
    lead_violations: dict = field(default_factory=dict)
    _prev_pair: tuple | None = None

    # ------------------------------------------------------------------ input
    def _restart(self, t: float, reason: str, value: float) -> None:
        if self.win is not None or reason not in self.resets:
            if len(self.reset_log) < 50:
                self.reset_log.append((round(t, 4), reason, round(float(value), 6)))
        self.resets[reason] = self.resets.get(reason, 0) + 1
        self.win = None
        self.recent.clear()
        self.max_err_win = 0.0

    def feed(self, t: float, p_des, p_act, controller_active: bool) -> None:
        """One commander tick: desired and measured EE with their common sim stamp."""
        if controller_active and self.t_ctrl_active is None:
            self.t_ctrl_active = t
        if self.state == 'failed' or self.placement is not None:
            return                                   # nothing after the birth is a start condition
        if self.last_t is not None and t <= self.last_t:
            self.n_out_of_order += 1
            return
        self.n_pairs += 1
        phi, dist = self.est.locate(p_des)
        err = float(np.linalg.norm(np.asarray(p_des, float) - np.asarray(p_act, float)))
        if err > self.cfg.hard_reset_thr:
            self.hard_ticks += 1
            self.hard_last_t = t
        gap = None if self.last_t is None else t - self.last_t
        step = None if self.last_phi is None else wrap(phi - self.last_phi)
        prev_pair = self._prev_pair
        self._prev_pair = (t, phi, err)
        self.last_t, self.last_phi = t, phi
        if self.state == 'announced':
            self._record_lead(t, phi, err, dist, controller_active, gap, step, prev_pair)
            return
        if self.state != 'qualifying':
            return
        max_step = 2.0 * self.cfg.max_sample_gap_s / self.cfg.cycle_time
        bad = None
        if not controller_active:
            bad = ('controller_inactive', 0.0)
        elif dist > self.cfg.on_path_tol:
            bad = ('not_on_cycle', dist)
        elif err > self.cfg.track_err_max:
            bad = ('tracking_error', err)
        elif gap is not None and gap > self.cfg.max_sample_gap_s:
            bad = ('stale_trajectory', gap)
        elif step is not None and self.win is not None and not (-1e-9 <= step <= max_step):
            bad = ('phase_jump', step)
        if bad is not None:
            self._restart(t, *bad)
            return
        if self.win is None:
            self.u = phi
            self.win = _Window(t0=t, u0=phi)
        else:
            self.u += step
        self.max_err_win = max(self.max_err_win, err)
        self.recent.append((t, self.u))
        while self.recent and self.recent[0][0] < t - self.cfg.rate_window_s:
            self.recent.popleft()

    # ------------------------------------------------------------------ decision
    def _rate(self):
        if len(self.recent) < 10:
            return None
        a = np.array(self.recent)
        span = a[-1, 0] - a[0, 0]
        if span < 0.8 * self.cfg.rate_window_s:
            return None
        tt = a[:, 0] - a[-1, 0]
        return float(np.polyfit(tt, a[:, 1], 1)[0])

    def poll(self, now: float, allow: bool = True) -> dict | None:
        """Call at every supervisor tick (sim time). Returns the decision once, when announced.

        ``allow`` = the supervisor's own readiness (streams, perception, warm-up): the timeout runs
        regardless, the announcement waits for it. ``now`` <= 0 (no /clock yet) is ignored.
        """
        if now <= 0.0:
            return None
        if self.t_first_poll is None:
            self.t_first_poll = now
        if self.state != 'qualifying':
            return None
        if now - self.t_first_poll > self.cfg.timeout_s:
            self.state = 'failed'
            last = max(self.resets, key=lambda k: self.resets[k]) if self.resets else None
            self.fail_reason = (f'start conditions not met within {self.cfg.timeout_s:.1f} s sim: '
                                f'{"no ee_desired/ee_actual pair received" if not self.n_pairs else ""}'
                                f'{"" if not self.n_pairs else f"window restarts {self.resets}; most frequent {last}"}'
                                f'; controller active since {self.t_ctrl_active}')
            return None
        if self.win is None or self.last_t is None:
            return None
        if now - self.last_t > self.cfg.max_sample_gap_s:
            self._restart(now, 'stale_trajectory', now - self.last_t)
            return None
        cycles = self.u - self.win.u0
        if cycles < self.cfg.settle_cycles:
            return None
        rate = self._rate()
        if rate is None:
            return None
        nominal = 1.0 / self.cfg.cycle_time
        if abs(rate / nominal - 1.0) > self.cfg.rate_rel_tol:
            self._restart(self.last_t, 'phase_rate_off_nominal', rate / nominal)
            return None
        u_now = self.u + rate * (now - self.last_t)
        k = math.ceil(u_now + rate * self.cfg.lead_min_s - self.cfg.phase0 - 1e-12)
        u_star = k + self.cfg.phase0
        lead = (u_star - u_now) / rate
        if lead > self.cfg.lead_announce_s or not allow:
            return None
        t_start = self.last_t + (u_star - self.u) / rate
        self.state = 'announced'
        self.decision = dict(
            t_start=t_start, t_announce=now, lead_s=t_start - now, phase0=self.cfg.phase0,
            target_unwrapped=u_star, phase_now=u_now % 1.0, rate_per_s=rate,
            rate_over_nominal=rate / nominal, window_t0=self.win.t0,
            window_cycles=self.u - self.win.u0, window_max_err_m=self.max_err_win,
            last_sample_t=self.last_t, controller_active_since=self.t_ctrl_active,
            hard_reset_ticks_before_announce=self.hard_ticks, hard_reset_last_t=self.hard_last_t,
            window_restarts=dict(self.resets))
        return self.decision

    # ------------------------------------------------------------------ after the announcement
    def _record_lead(self, t, phi, err, dist, active, gap, step, prev_pair) -> None:
        ts = self.decision['t_start']
        if self.placement is None and t < ts:           # the lead: announced, not yet born
            if not active:
                self.lead_violations['controller_inactive'] = self.lead_violations.get('controller_inactive', 0) + 1
            if err > self.cfg.track_err_max:
                self.lead_violations['tracking_error'] = self.lead_violations.get('tracking_error', 0) + 1
            if dist > self.cfg.on_path_tol:
                self.lead_violations['not_on_cycle'] = self.lead_violations.get('not_on_cycle', 0) + 1
            if gap is not None and gap > self.cfg.max_sample_gap_s:
                self.lead_violations['stale_trajectory'] = self.lead_violations.get('stale_trajectory', 0) + 1
        if self.placement is None and t >= ts and prev_pair is not None:
            t_a, phi_a, err_a = prev_pair
            f = 0.0 if t == t_a else (ts - t_a) / (t - t_a)
            phi_at = (phi_a + f * wrap(phi - phi_a)) % 1.0
            self.placement = dict(
                t_start=ts, bracket=[t_a, t], phase_at_start=phi_at,
                phase_error_cycles=wrap(phi_at - self.cfg.phase0),
                phase_error_s=wrap(phi_at - self.cfg.phase0) * self.cfg.cycle_time,
                ee_err_at_start_m=err_a + f * (err - err_a),
                lead_violations=dict(self.lead_violations),
                hard_reset_ticks_total=self.hard_ticks, hard_reset_last_t=self.hard_last_t)

    def summary(self) -> dict:
        return dict(state=self.state, fail_reason=self.fail_reason, config=asdict(self.cfg),
                    decision=self.decision, placement=self.placement, pairs=self.n_pairs,
                    out_of_order=self.n_out_of_order, controller_active_since=self.t_ctrl_active,
                    hard_reset_ticks_before_birth=self.hard_ticks, hard_reset_last_t=self.hard_last_t,
                    window_restarts=dict(self.resets), restart_log=list(self.reset_log))
