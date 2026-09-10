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

from franka_experiments.utils.cbf_zones import ZONE_NAMES


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

def _zone_field(con, i: int, w_task, n_rot: int) -> str:
    """The ``zone=``/``nrot=`` fields, or the empty string when the ladder is off.

    Empty rather than a placeholder: a log line that changes WIDTH when a flag
    is toggled is easier to grep than one whose meaning changes silently, and
    every downstream parser in scripts/ splits on ``key=`` rather than on
    position for exactly this reason.
    """
    k0r = getattr(con, 'k0_row', None)
    if k0r is None or w_task is None:
        return ''
    k1r = getattr(con, 'k1_row', None)
    zr = getattr(con, 'zone_row', None)
    name = '?'
    if zr is not None and i < len(zr):
        z = int(zr[i])
        if 0 <= z < len(ZONE_NAMES):
            name = ZONE_NAMES[z]
    k0i = float(k0r[i]) if i < len(k0r) else float('nan')
    k1i = float(k1r[i]) if (k1r is not None and i < len(k1r)) else float('nan')
    return (f'zone={name} k={k0i:.1f}/{k1i:.1f} task={float(w_task):.2f} '
            f'nrot={int(n_rot)} ')


def format_cbf_diag(*, now, con, rows, caps, h_qp, qdot, qdot_cbf,
                    qddot_safe, qddot_nom, qddot_real, slack, n_active_cps,
                    vel_ratio, vel_bite, slew_bite, cap_age,
                    w_task=None) -> str:
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
    ``vobs``     [m/s] fastest approaching obstacle this rebuild, over ALL
                 rows — not this row's. Use ``vel[...]`` for that.
    ``vel[obs/rob/rel]``  [m/s] the three speeds of the row this line is
                 about, along its normal n̂, all directly comparable:

                 * ``obs`` — the OBSTACLE's closing speed, the same
                   (conditioned, clamped) number the barrier and the retreat
                   cap consume. Positive means closing in.
                 * ``rob`` — the ROBOT's own speed at that control point,
                   i.e. how fast it is getting away. Positive means
                   separating. This is the avoidance speed. Identical to
                   ``hdot``'s raw value, printed here so the three read as
                   one balance.
                 * ``rel`` — their difference ``rob − obs``, which is ḣ, the
                   rate the gap is ACTUALLY changing. Positive means the gap
                   is opening. This is the number that says whether the robot
                   is winning: ``rel`` ≥ 0 with a positive gap means the
                   situation is under control however alarming ``obs`` looks.

                 The fastest separation over all control points is the first
                 number of ``retreat``, so a jerk on a point OTHER than the
                 one labelling the line shows up there.
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
    ``outr``     Phase-2 outrun test: largest closing/outrunnable ratio this
                 rebuild (> 1 = some point cannot outrun its obstacle along
                 n̂) / largest lateral blend weight actually applied.
    ``zone``     which rung of the gap ladder this row is on, the scheduled
                 gains it is being driven with, and how much of the TASK
                 survives — ``zone=active k=25.0/10.5 task=1.00``. Only
                 present when ``enable_zone_ladder`` is on.

                 The rung name is the argmax of the blend weights and is a
                 LABEL: every actual decision uses the blended gains printed
                 next to it, so seeing ``active`` with k = 18.5/8.75 is not a
                 contradiction, it is the notice/active transition band.

                 ``task`` is the fraction of the commander's q̈_nom that
                 survived. 1.00 is normal, 0.50 is the priority rung giving
                 way, 0.00 is the hold rung: the trajectory is suspended and
                 the nominal has become a braking command, while every barrier
                 row keeps full authority. It falls instantly and climbs over
                 ``zone_resume_s``, so a value between 0 and 1 with the gap
                 already clear is the RESUME ramp, not a fault.

    ``nrot``     count of residual frames discarded because n̂ rotated further
                 than ``obstacle_velocity_normal_rot_max`` between them, i.e.
                 the nearest obstacle point hopped to another surface patch and
                 ``aᵀq̇ − ḋ`` stopped being an obstacle velocity. CUMULATIVE
                 since the node started. A number that climbs steadily while a
                 STATIC obstacle is in view is the guard doing its job; one
                 that never moves means the fabricated closing speed on a
                 static obstacle comes from somewhere else and the hypothesis
                 behind the guard is wrong.

    ``hlat``     [m] largest latency-compensation tightening (prediction +
                 propagated position uncertainty). 0 with the flag off.
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

    # ── The three velocities of the row this line is about, in m/s ──────────
    # a_i is n̂ᵀJ_p with n̂ a unit vector, so aᵀq̇ IS the control point's speed
    # along the normal — no scaling needed, and the numbers are directly
    # comparable with d_min and with each other.
    v_rob_i = float(a_i @ qdot)                 # + = the robot is separating
    v_obs_i = float(con.v_obs[i]) if i < con.v_obs.size else 0.0
    v_rel_i = v_rob_i - v_obs_i                 # + = the gap is opening

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
        # WHICH control point reported that maximum. vobs is a max over ALL
        # rows and vel[obs/...] below belongs to the row holding d_min, so with
        # two obstacles in the scene the two fields legitimately disagree — and
        # a line reading `vobs=+1.850 vel[obs/...]=+0.067/...` with no way to
        # tell which row the 1.850 came from is unreadable. Kept as its own
        # key rather than appended to vobs= so a parser doing float() on the
        # value still works. '-' when no row reported an approach.
        f'vobs_cp={getattr(rows, "diag_v_obs_link", "") or "-"} '
        # Cumulative count of control points whose closing-speed state was
        # discarded because their nearest obstacle changed identity. Outside
        # _zone_field on purpose: unlike nrot= this has nothing to do with the
        # zone ladder and must be visible whatever the flags.
        f'nid={int(getattr(rows, "diag_ident_reset", 0))} '
        f'vel[obs/rob/rel]={v_obs_i:+.3f}/{v_rob_i:+.3f}/{v_rel_i:+.3f} '
        f'dnorm={float(np.linalg.norm(dq)):.3f} '
        f's[obs/sc/qlim/sing/cap/spd]={slack[G_OBS]:.3f}/{slack[G_SC]:.3f}/'
        f'{slack[G_QLIM]:.3f}/{slack[G_SING]:.3f}/{slack[G_CAP]:.3f}/'
        f'{slack[G_SPD]:.3f} '
        f'd_sc={con.d_sc_min:.3f} sigma={rows.diag_sigma:.3f} '
        f'capage={cap_age:.3f} '
        f'vapp={rows.diag_vapp:.3f} hbrake={rows.diag_hbrake:.4f} '
        # Phase-3 / evasion terms. hunc is the barrier tightening bought by
        # the tracker's admitted uncertainty; esc is the largest evasion
        # urgency in [0, 1], where 1.0 means the acceleration box says this
        # closing rate CANNOT be nulled before the gap reaches zero. Both
        # read 0.0000/0.00 with their flags off, which is the whole point:
        # the line says whether a term is doing anything without needing the
        # config open next to it.
        f'hunc={getattr(rows, "diag_hunc", 0.0):.4f} '
        f'esc={getattr(rows, "diag_esc_w", 0.0):.2f} '
        f'outr={getattr(rows, "diag_outrun_r", 0.0):.2f}/{getattr(rows, "diag_outrun_w", 0.0):.2f} '
        f'hlat={getattr(rows, "diag_hlat", 0.0):.4f} '
        + _zone_field(con, i, w_task, getattr(rows, 'diag_rot_reject', 0)) +
        f'w=[{w_txt}] wq=[{wq_txt}] '
        f'retreat={rtr:+.3f}/{rtr_cap:.3f} vlink={spd:+.3f}/{spd_cap:.3f} '
        f'dq_rad={dq_rad:+.3f} dq_ort={dq_ort:.3f} '
        f'cart_rad={float(a_i @ dq):+.3f} '
        f'qdd_cmd_rad={float(a_i @ qddot_safe):+.3f} '
        f'qdd_real_rad={float(a_i @ qddot_real):+.3f} '
        f'trk_err={float(np.linalg.norm(qddot_safe - qddot_real)):.3f} '
        f'slew={"".join("X" if b else "." for b in slew_bite)} '
        f'| {format_velocity_summary(qdot, vel_ratio, vel_bite)}')
