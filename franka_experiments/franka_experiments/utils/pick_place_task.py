"""Cyclic Cartesian pick-and-place task shared by the commanders and the MPC.

Pure NumPy, no ROS. Positions are in ``fr3_link0`` (+y = robot left, +z = up).

The trajectory is parameterised by a VIRTUAL time ``s`` rather than by wall
time. ``TaskClock`` advances ``s`` at a rate ``s_dot`` in [0, 1] that drops when
the end-effector falls behind the reference, so an avoidance manoeuvre makes
the task wait instead of letting the reference run ahead and skip phases:

    p_d = p(s),   v_d = p'(s) s_dot,   a_d = p''(s) s_dot^2 + p'(s) s_ddot

where p'(s), p''(s) are the timed trajectory's velocity and acceleration.
"""

from __future__ import annotations

from typing import List, Tuple
import numpy as np


class TaskPhase:
    """One timed phase of the cyclic pick-and-place task."""
    def __init__(self, name: str, start: np.ndarray, goal: np.ndarray, duration: float, moving: bool):
        self.name = name
        self.start = start
        self.goal = goal
        self.duration = duration
        self.moving = moving


class PickPlaceTrajectory:
    """Timed sequence of minimum-jerk Cartesian moves and stationary waits matching the trapezoidal profile."""

    def __init__(
        self,
        home: np.ndarray,
        lateral_offset: float,
        down_offset: float,
        drop_offset: float,
        move_time: float,
        drop_time: float,
        transfer_time: float,
        return_time: float,
        wait_pick: float,
        wait_drop: float,
        wait_transfer: float,
        wait_home: float,
    ) -> None:
        home = np.asarray(home, dtype=float).copy()

        # fr3_link0 convention: +y = robot left, -y = robot right, +z = up
        pick = home + np.array([0.0, -lateral_offset, -down_offset])
        pick_down = pick - np.array([0.0, 0.0, drop_offset])

        place = home + np.array([0.0, lateral_offset, -down_offset])
        place_down = place - np.array([0.0, 0.0, drop_offset])

        self.home = home
        self.pick = pick
        self.pick_down = pick_down
        self.place = place
        self.place_down = place_down

        self.phases: List[TaskPhase] = [
            TaskPhase('MOVE_TO_PICK', home, pick, move_time, True),
            TaskPhase('WAIT_AT_PICK', pick, pick, wait_pick, False),
            TaskPhase('SMALL_DESCENT_1', pick, pick_down, drop_time, True),
            TaskPhase('WAIT_AT_DROP_1', pick_down, pick_down, wait_drop, False),
            TaskPhase('SMALL_ASCENT_1', pick_down, pick, drop_time, True),
            TaskPhase('WAIT_AFTER_ASCENT_1', pick, pick, wait_drop, False),
            TaskPhase('LATERAL_TRANSFER', pick, place, transfer_time, True),
            TaskPhase('WAIT_AT_PLACE', place, place, wait_transfer, False),
            TaskPhase('SMALL_DESCENT_2', place, place_down, drop_time, True),
            TaskPhase('WAIT_AT_DROP_2', place_down, place_down, wait_drop, False),
            TaskPhase('SMALL_ASCENT_2', place_down, place, drop_time, True),
            TaskPhase('WAIT_AFTER_ASCENT_2', place, place, wait_drop, False),
            TaskPhase('RETURN_HOME', place, home, return_time, True),
            TaskPhase('WAIT_AT_HOME', home, home, wait_home, False),
        ]

        for phase in self.phases:
            if phase.duration <= 0.0:
                raise ValueError(f'Phase {phase.name} must have positive duration')

        self._end_times = np.cumsum([phase.duration for phase in self.phases])
        self.cycle_time = float(self._end_times[-1])
        self._p_out = np.zeros(3)
        self._v_out = np.zeros(3)
        self._a_out = np.zeros(3)

    @staticmethod
    def _minimum_jerk(u: float) -> Tuple[float, float, float]:
        u = float(np.clip(u, 0.0, 1.0))
        u2 = u * u
        u3 = u2 * u
        u4 = u3 * u
        u5 = u4 * u
        sigma = 10.0 * u3 - 15.0 * u4 + 6.0 * u5
        dsigma_du = 30.0 * u2 - 60.0 * u3 + 30.0 * u4
        d2sigma_du2 = 60.0 * u - 180.0 * u2 + 120.0 * u3
        return sigma, dsigma_du, d2sigma_du2

    def evaluate(self, t: float):
        t = max(0.0, float(t))
        cycle_index = int(t // self.cycle_time)
        cycle_t = t - cycle_index * self.cycle_time

        phase_index = int(np.searchsorted(self._end_times, cycle_t, side='right'))
        if phase_index >= len(self.phases):
            phase_index = len(self.phases) - 1

        phase = self.phases[phase_index]
        phase_start_t = 0.0 if phase_index == 0 else self._end_times[phase_index - 1]
        local_t = cycle_t - phase_start_t

        if not phase.moving:
            np.copyto(self._p_out, phase.goal)
            self._v_out[:] = 0.0
            self._a_out[:] = 0.0
            return self._p_out, self._v_out, self._a_out, phase.name, cycle_index

        u = local_t / phase.duration
        sigma, dsigma_du, d2sigma_du2 = self._minimum_jerk(u)
        delta = phase.goal - phase.start

        self._p_out[:] = phase.start + sigma * delta
        self._v_out[:] = (dsigma_du / phase.duration) * delta
        self._a_out[:] = (d2sigma_du2 / (phase.duration ** 2)) * delta
        return self._p_out, self._v_out, self._a_out, phase.name, cycle_index

    def reference(self, s: float, s_dot: float, s_ddot: float):
        """Reference at virtual time s advancing at rate s_dot (chain rule on the timed trajectory).

        Returns new arrays (p_d, v_d, a_d), the phase name and the cycle index.
        """
        p, v, a, phase, cycle = self.evaluate(s)
        return p.copy(), v * s_dot, a * (s_dot * s_dot) + v * s_ddot, phase, cycle


class TaskClock:
    """Virtual task time s that advances only while the end-effector keeps up with the reference.

        s_dot_target = clip((e_stop - e) / (e_stop - e_ok), 0, 1)
        s_dot        -> s_dot_target with |s_ddot| <= s_ddot_max
        s           += s_dot * dt

    e is the Cartesian tracking error. Below e_ok the clock runs at real time;
    at e_stop and above it stops, so the reference waits for the robot. The
    rate limit keeps the reference C^1 (a_d stays bounded by |p'| s_ddot_max).
    ``rate_cap`` lets an external supervisor (e.g. separation monitoring)
    slow the task further.
    """

    def __init__(self, e_ok: float, e_stop: float, s_ddot_max: float, enabled: bool = True) -> None:
        if enabled and not 0.0 <= e_ok < e_stop:
            raise ValueError('TaskClock needs 0 <= e_ok < e_stop')
        if s_ddot_max <= 0.0:
            raise ValueError('TaskClock needs s_ddot_max > 0')
        self.e_ok = float(e_ok)
        self.e_stop = float(e_stop)
        self.s_ddot_max = float(s_ddot_max)
        self.enabled = bool(enabled)
        self.reset()

    def reset(self) -> None:
        self.s = 0.0
        self.s_dot = 1.0
        self.s_ddot = 0.0

    def target_rate(self, tracking_error: float, rate_cap: float = 1.0) -> float:
        if not self.enabled:
            return 1.0
        rate = (self.e_stop - float(tracking_error)) / (self.e_stop - self.e_ok)
        return float(np.clip(min(rate, rate_cap), 0.0, 1.0))

    def step(self, dt: float, tracking_error: float, rate_cap: float = 1.0) -> Tuple[float, float, float]:
        """Advance by dt [s]; returns (s, s_dot, s_ddot)."""
        dt = max(0.0, float(dt))
        if not self.enabled:
            self.s_dot, self.s_ddot = 1.0, 0.0
        elif dt > 0.0:
            target = self.target_rate(tracking_error, rate_cap)
            max_step = self.s_ddot_max * dt
            delta = float(np.clip(target - self.s_dot, -max_step, max_step))
            self.s_dot += delta
            self.s_ddot = delta / dt
        self.s += self.s_dot * dt
        return self.s, self.s_dot, self.s_ddot