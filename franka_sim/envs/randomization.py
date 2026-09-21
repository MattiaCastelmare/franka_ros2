"""Domain randomisation and observation-noise models (backlog P1.4 / P1.5).

Everything here is OFF by default and, when off, must be a no-op that leaves
the environment bit-identical — the current results depend on that.

Four mechanisms, in the order the roadmap ranks them:

1. **Latency** — the sim hands the policy an obstacle position measured *now*;
   the robot hands it one that has travelled camera → distance engine → CBF
   first. `scripts/latency_budget.py` measures ~13 ms of camera hop at the
   pinned 30 fps profile, and the pipeline adds more on top. This is the gap
   §10 calls the largest unmodelled one, and a pure delay is the one kind of
   plant change a CBF cannot absorb: the barrier is certifying a state that is
   already stale.
2. **Observation noise** — the sim feeds an exact `d_min` and an exact obstacle
   centre. The robot feeds an LPF'd distance, an EMA-smoothed normal and a
   point cloud that occasionally drops out, **clamped at 0** so penetration is
   never reported.
3. **Sensor noise** on `q`/`q̇` — the FR3's encoders are good, so this is the
   smallest of the four, but it is nearly free to model.
4. **Dynamics** — link masses, joint damping and friction. The roadmap argues
   the CBF absorbs much of this; it is included for completeness and ranked
   last for that reason.

WHY NOISE IS APPLIED TO THE OBSERVATION AND NOT TO THE BARRIER. The CBF rows
are built from MuJoCo's true geometry, and they stay that way. Injecting noise
into the shield's own inputs would be modelling a *different, worse* safety
filter than the robot runs — the robot's engine is noisy, but the filter still
treats what it receives as the truth. The policy is what must become robust to
a noisy estimate, so the policy is what sees the noise.
"""

from __future__ import annotations

from collections import deque

import numpy as np


class Randomizer:
    """Per-episode dynamics randomisation + per-step observation corruption."""

    def __init__(self, cfg: dict | None, dt: float, rng: np.random.Generator):
        c = cfg or {}
        self.enabled = bool(c.get('enabled', False))
        self.dt = float(dt)
        self.rng = rng

        # ── 1. Latency ────────────────────────────────────────────────────
        lat = c.get('latency', {}) or {}
        self.lat_enabled = bool(lat.get('enabled', False))
        self.lat_mean_s = float(lat.get('mean_s', 0.04))
        self.lat_jitter_s = float(lat.get('jitter_s', 0.01))
        self._delay_buf: deque = deque()
        self._n_delay = 0

        # ── 2. Observation noise ──────────────────────────────────────────
        obs = c.get('obs_noise', {}) or {}
        self.obs_enabled = bool(obs.get('enabled', False))
        self.obs_pos_std = float(obs.get('obstacle_pos_std', 0.01))
        self.obs_d_std = float(obs.get('d_min_std', 0.005))
        self.obs_lpf_alpha = float(obs.get('lpf_alpha', 0.5))
        self.obs_clamp_zero = bool(obs.get('clamp_d_min_at_zero', True))
        self.obs_dropout_p = float(obs.get('dropout_prob', 0.0))
        self._lpf_state = None
        self._last_good = None

        # ── 3. Joint sensor noise ─────────────────────────────────────────
        js = c.get('joint_noise', {}) or {}
        self.js_enabled = bool(js.get('enabled', False))
        self.q_std = float(js.get('q_std', 0.0005))
        self.qdot_std = float(js.get('qdot_std', 0.005))

        # ── 4. Dynamics ───────────────────────────────────────────────────
        dyn = c.get('dynamics', {}) or {}
        self.dyn_enabled = bool(dyn.get('enabled', False))
        self.mass_range = tuple(dyn.get('mass_scale', [0.95, 1.05]))
        self.damping_range = tuple(dyn.get('damping_scale', [0.8, 1.2]))
        self.friction_range = tuple(dyn.get('friction_scale', [0.8, 1.2]))
        self._nominal = None          # (body_mass, dof_damping, dof_frictionloss)

    # ── Episode lifecycle ────────────────────────────────────────────────

    def reset(self, model, rng: np.random.Generator) -> None:
        """Re-draw the per-episode constants and clear all per-step state."""
        self.rng = rng
        self._lpf_state = None
        self._last_good = None
        self._delay_buf.clear()

        if self.enabled and self.lat_enabled:
            # One delay per EPISODE, not per step: a delay that changes every
            # tick reorders samples, which is a different (and unphysical)
            # corruption from a pipeline that runs a few frames behind.
            d = self.rng.normal(self.lat_mean_s, self.lat_jitter_s)
            self._n_delay = int(np.clip(round(d / self.dt), 0, 100))
        else:
            self._n_delay = 0

        if not (self.enabled and self.dyn_enabled):
            # Restore the nominal model if a previous episode perturbed it —
            # MjModel is mutated in place and persists across resets.
            if self._nominal is not None:
                model.body_mass[:] = self._nominal[0]
                model.dof_damping[:] = self._nominal[1]
                model.dof_frictionloss[:] = self._nominal[2]
            return

        if self._nominal is None:
            self._nominal = (model.body_mass.copy(), model.dof_damping.copy(),
                             model.dof_frictionloss.copy())
        m0, d0, f0 = self._nominal
        model.body_mass[:] = m0 * self.rng.uniform(*self.mass_range, size=m0.shape)
        model.dof_damping[:] = d0 * self.rng.uniform(*self.damping_range, size=d0.shape)
        model.dof_frictionloss[:] = f0 * self.rng.uniform(*self.friction_range,
                                                          size=f0.shape)

    # ── Per-step corruption ──────────────────────────────────────────────

    def joint_state(self, q, qdot):
        if not (self.enabled and self.js_enabled):
            return q, qdot
        return (q + self.rng.normal(0.0, self.q_std, q.shape),
                qdot + self.rng.normal(0.0, self.qdot_std, qdot.shape))

    def obstacle(self, p_obs, d_min):
        """(centre, d_min) as the perception pipeline would report them.

        Order matters and follows the real chain: delay first (the estimate is
        old), then sensor noise, then the engine's LPF, then the clamp. The
        clamp is LAST because that is where `distance_engine` puts it — so the
        robot never reports penetration, however noisy the input was.
        """
        if not self.enabled:
            return p_obs, d_min

        p, d = np.asarray(p_obs, float), float(d_min)

        if self.lat_enabled:
            self._delay_buf.append((p.copy(), d))
            while len(self._delay_buf) > self._n_delay + 1:
                self._delay_buf.popleft()
            p, d = self._delay_buf[0]
            p = p.copy()

        if self.obs_enabled:
            if self.obs_dropout_p > 0.0 and self.rng.random() < self.obs_dropout_p:
                # A dropped frame HOLDS the last good value, as a perception
                # pipeline does; it does not teleport the obstacle to zero.
                if self._last_good is not None:
                    p, d = self._last_good[0].copy(), self._last_good[1]
            else:
                p = p + self.rng.normal(0.0, self.obs_pos_std, 3)
                d = d + self.rng.normal(0.0, self.obs_d_std)

            a = self.obs_lpf_alpha
            if 0.0 < a < 1.0:
                if self._lpf_state is None:
                    self._lpf_state = (p.copy(), d)
                else:
                    sp, sd = self._lpf_state
                    p = a * p + (1.0 - a) * sp
                    d = a * d + (1.0 - a) * sd
                    self._lpf_state = (p.copy(), d)

            if self.obs_clamp_zero:
                d = max(d, 0.0)
            self._last_good = (p.copy(), d)

        return p, d
