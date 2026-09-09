"""Rate-limited logging, formatting, and lightweight profiling.

OWNS
----
Observability helpers that any node may use:
``ThrottledLogger`` (rate-limited log lines), ``vec_to_str``
(compact vector formatting) and ``PerfTimer`` (named-stage
wall-clock profiler, moved here from ``utils.node_utils`` in Phase 2).

DOES NOT OWN
------------
Anything domain-specific: no distance, no CBF, no message construction.
"""

from __future__ import annotations

from typing import Optional

import numpy as np


class ThrottledLogger:
    """Rate-limited logging wrapper.

    Usage::

        self._tlog = ThrottledLogger(self.get_logger(), period_s=1.0)

        # inside callback:
        if self._tlog.due(t):
            self._tlog.info(f'[t={t:.1f}s] some message')
    """

    def __init__(self, logger, *, period_s: float = 1.0) -> None:
        self._logger = logger
        self._period = period_s
        self._last_t: float = -period_s  # ensures first call fires

    def due(self, t: float) -> bool:
        """Return ``True`` (and update timestamp) if at least *period_s* elapsed."""
        if t - self._last_t >= self._period:
            self._last_t = t
            return True
        return False

    def info(self, msg: str) -> None:
        """Log at INFO level (unconditionally — gate with :meth:`due`)."""
        self._logger.info(msg)

    def debug(self, msg: str) -> None:
        """Log at DEBUG level (unconditionally — gate with :meth:`due`)."""
        self._logger.debug(msg)

    def warn(self, msg: str) -> None:
        """Log at WARN level (unconditionally — gate with :meth:`due`)."""
        self._logger.warn(msg)

    @property
    def last_t(self) -> float:
        return self._last_t

    @last_t.setter
    def last_t(self, value: float) -> None:
        self._last_t = value


def vec_to_str(v: Optional[np.ndarray], fmt: str = '.4f') -> str:
    """Format a numpy vector as a comma-separated string, or ``'?'``."""
    if v is None:
        return '?'
    return ', '.join(f'{x:{fmt}}' for x in v)


# MOVED here from utils/node_utils.py (Phase 2): a wall-clock profiler is
# observability, not perception-message construction.
import time  # noqa: E402


class PerfTimer:
    """Accumulates wall-clock timings for a set of named stages.

    Usage::

        perf = PerfTimer()
        with perf('tf'):
            ...
        print(perf.summary())   # 'tf=1.2ms  mask=3.4ms'
    """

    def __init__(self):
        self._ms: dict[str, float] = {}

    def __call__(self, key: str) -> '_TimerCtx':
        return _TimerCtx(self._ms, key)

    def summary(self) -> str:
        return '  '.join(f'{k}={v:.1f}ms' for k, v in self._ms.items())


class _TimerCtx:
    """Context manager returned by PerfTimer.__call__."""

    def __init__(self, store: dict, key: str):
        self._s = store
        self._k = key

    def __enter__(self):
        self._t0 = time.perf_counter()
        return self

    def __exit__(self, *_):
        self._s[self._k] = (time.perf_counter() - self._t0) * 1000.0


def format_velocity_summary(
    qdot: np.ndarray,
    ratio: np.ndarray,
    bite: np.ndarray,
) -> str:
    """Format a compact per-joint velocity-saturation summary for one log line.

    MOVED here from ``CBFSafetyFilter._fmt_vel`` in Phase 3; body unchanged, the
    two diagnostic arrays it used to read off ``self`` are now arguments.

    Args:
        qdot: (n,) measured joint velocities [rad/s].
        ratio: (n,) ``|qdot| / qdot_max`` per joint.
        bite: (n,) boolean mask — True where the velocity bound tightened that
            joint's acceleration box this tick.

    Returns:
        A single line: the worst joint (signed qdot and its ratio), all ratios
        in joint order, and an n-character bite mask (``X`` = biting).
    """
    k     = int(np.argmax(ratio))
    rats  = '/'.join(f'{r:.2f}' for r in ratio)
    mask  = ''.join('X' if b else '.' for b in bite)
    return (f'worst=j{k+1}:q̇={qdot[k]:+.2f}({ratio[k]:.2f}) '
            f'vrat=[{rats}] vbite={mask}')


# ── CBF episode diagnostic ───────────────────────────────────────────────────

def format_cbf_diag(*, now, con, rows, caps, h_qp, qdot, qdot_cbf,
                    qddot_safe, qddot_nom, qddot_real, slack, n_active_cps,
                    vel_ratio, vel_bite, slew_bite, cap_age) -> str:
    """One compact, CSV-like line describing the whole constraint episode.

    Field guide, in the order they appear. Units in brackets.

    ``n_c``      rows in the QP — every control point inside the horizon, most
                 of them non-binding.
    ``n_act``    rows actually VIOLATED (h̄ < 0). More than one is simultaneous
                 multi-CP activation, expected on an angled approach.
    ``d_min``    [m] closest OBSTACLE surface gap. NOT ``min(h_bar) + d_safe``:
                 ``h_bar`` mixes metres (obstacle, self-collision) with radians
                 (joint limits) and m/rad (singularity), so its argmin is not a
                 distance at all.
    ``link``     which row labels this line — the argmin of ``h_bar``, for
                 LABELLING only. Every row is in the QP regardless.
    ``hdot``     [m/s] approach rate ``aᵀq̇`` of that row, filtered, with the raw
                 value in brackets. Filtered ≠ raw is the ``k1`` smoothing
                 working; filtered == raw to three decimals for many ticks in a
                 row means q̇ is FROZEN, which is how two hardware aborts were
                 finally diagnosed.
    ``h_qp``     [m/s²] that row's bound. More positive ⇒ looser. Negative means
                 the row is demanding positive ``aᵀq̈``, i.e. retreat.
    ``hhold``    [m] largest barrier RECOVERY the asymmetric smoothing held back.
    ``vobs``     [m/s] fastest approaching obstacle this rebuild.
    ``dnorm``    [rad/s²] ‖q̈_safe − q̈_nom‖ — how hard the QP bends the nominal.
    ``s[...]``   the six per-family slacks. One exploding while the others stay
                 small is exactly what a shared slack used to hide. ``s[cap]``
                 > 0 is NORMAL: the barrier is overruling the retreat cap.
    ``d_sc``     [m] closest self-collision capsule gap.
    ``sigma``    [m/rad] σ_min of the EE Jacobian — the singularity barrier's
                 argument.
    ``capage``   [s] SENSING age of the distance frame, ``now − header.stamp``.
                 The real perception latency, not the transport age the
                 staleness check uses.
    ``vapp``     [m/s] largest closing speed the velocity feedforward USED (0
                 while a track is under its minimum frame count, or flag off).
    ``hbrake``   [m] largest braking-distance tightening applied. 0 with the
                 flag off — this is the field that makes that flag visible.
    ``w=[..]``   per-OBSTACLE-row slack weight. 1.00 = relaxes as it does with
                 the weighting off; ``w_max`` = treated as maximally critical.
    ``wq=[..]``  per-JOINT-LIMIT-row weight, LABELLED by row because which
                 joints are present changes tick to tick. Watch this when
                 ``vbite`` shows a joint stuck on its braking curve: ``-`` means
                 the row was never emitted and the hard box is acting alone,
                 which is the case that dumps everything into ``dq_ort``.
    ``retreat``  [m/s] fastest separation rate / tightest cap applied to it.
    ``vlink``    [m/s] fastest capped control point / tightest cap. A cap well
                 under ``link_speed_max`` is the GEOMETRIC term biting.
    ``dq_rad``/``dq_ort``  split of ``q̈_safe − q̈_nom`` along / ⊥ the labelled
                 row. Large ``dq_ort`` means the correction is leaking into
                 UNconstrained joints — the "throws itself backward" signature.
    ``cart_rad`` [m/s²] Cartesian accel change along n̂ at that control point.
    ``qdd_cmd_rad``/``qdd_real_rad``/``trk_err``  commanded vs realised, and the
                 joint-space tracking error. ``qdd_real ≪ qdd_cmd`` while
                 pushing away ⇒ the command is not being executed.
    ``slew``     which joints sit on the accel-continuity edge.
    ``worst``/``vrat``/``vbite``  per-joint velocity saturation summary.

    Every argument is already computed by the caller; this only formats. Runs
    behind the caller's throttle, so its cost is amortised to nothing.
    """
    from franka_experiments.utils.cbf_state_rows import (
        G_CAP, G_OBS, G_QLIM, G_SC, G_SING, G_SPD)

    dq = qddot_safe - qddot_nom
    i = int(np.argmin(con.h_bar))
    a_i = con.A[i]
    a_n = float(np.linalg.norm(a_i))
    a_hat = a_i / a_n if a_n > 1e-12 else a_i
    dq_rad = float(a_hat @ dq)
    dq_ort = float(np.linalg.norm(dq - dq_rad * a_hat))
    link_i = con.links[i] if i < len(con.links) else '?'

    w_txt = ('-' if rows.diag_w is None or rows.diag_w.size == 0
             else '/'.join(f'{x:.2f}' for x in rows.diag_w))
    if rows.diag_wq is None or rows.diag_wq.size == 0:
        wq_txt = '-'
    else:
        q_lbls = [l for l, g in zip(con.links, con.group) if g == G_QLIM]
        wq_txt = '/'.join(f'{l}:{x:.2f}' for l, x in zip(q_lbls, rows.diag_wq))
    rtr, rtr_cap, spd, spd_cap = caps

    return (
        f'CBFDIAG t={now:.3f} n_c={con.A.shape[0]} n_act={n_active_cps} '
        f'd_min={con.d_obs_min:.3f} '
        f'link={link_i} hdot={float(a_i @ qdot_cbf):+.3f}'
        f'(raw{float(a_i @ qdot):+.3f}) h_qp={float(h_qp[i]):+.3f} '
        f'hhold={rows.diag_h_hold:.4f} vobs={rows.diag_v_obs:+.3f} '
        f'dnorm={float(np.linalg.norm(dq)):.3f} '
        f's[obs/sc/qlim/sing/cap/spd]={slack[G_OBS]:.3f}/{slack[G_SC]:.3f}/'
        f'{slack[G_QLIM]:.3f}/{slack[G_SING]:.3f}/{slack[G_CAP]:.3f}/'
        f'{slack[G_SPD]:.3f} '
        f'd_sc={con.d_sc_min:.3f} sigma={rows.diag_sigma:.3f} '
        f'capage={cap_age:.3f} '
        f'vapp={rows.diag_vapp:.3f} hbrake={rows.diag_hbrake:.4f} '
        f'w=[{w_txt}] wq=[{wq_txt}] '
        f'retreat={rtr:+.3f}/{rtr_cap:.3f} vlink={spd:+.3f}/{spd_cap:.3f} '
        f'dq_rad={dq_rad:+.3f} dq_ort={dq_ort:.3f} '
        f'cart_rad={float(a_i @ dq):+.3f} '
        f'qdd_cmd_rad={float(a_i @ qddot_safe):+.3f} '
        f'qdd_real_rad={float(a_i @ qddot_real):+.3f} '
        f'trk_err={float(np.linalg.norm(qddot_safe - qddot_real)):.3f} '
        f'slew={"".join("X" if b else "." for b in slew_bite)} '
        f'| {format_velocity_summary(qdot, vel_ratio, vel_bite)}')
