"""Gap-scheduled aggressiveness for the obstacle barrier: one ladder, four rungs.

WHY THIS EXISTS
---------------
The filter already changed its mind about an obstacle at five independent
distances, none of which knew about the others:

    outrun_evasion_engage_gap   0.30 m
    livelock_engage_gap         0.25 m
    d_safe                      0.15 m
    retreat_cap_engage_gap      0.15 m
    governor_d_full             0.05 m   (in the commander)

and the barrier itself engaged at a SIXTH distance that is not a parameter at
all. Solving ``h_qp = k1·ḣ + k0·h̄ = 0`` for the gap gives

    d_bind = d_safe + h_unc + (k1/k0)·|ḣ|

so with the shipped k0 = 25, k1 = 10.5 the row starts biting at 0.23 m when the
arm walks in at 0.12 m/s and at 0.29 m when it walks in at 0.25 m/s. Measured
on hardware: the obstacle slack first went non-zero at d = 0.227 m against a
predicted 0.235 m. The engagement distance was a function of how fast you
happened to be arriving, which is exactly the property that makes the behaviour
feel unpredictable, and NOTHING in the configuration said so.

This module replaces that with an explicit ladder on the surface gap. The
boundaries are the contract; the gains are what the ladder schedules.

THE FOUR RUNGS
--------------
==========  ==============  ====================================================
zone        gap             what changes
==========  ==============  ====================================================
notice      >= d_notice     low bandwidth. Engages EARLY (the k1/k0 ratio is
                            larger, so d_bind moves out) and pushes GENTLY. The
                            speed dependence of d_bind is a feature here: a fast
                            obstacle is noticed further out, which is the
                            "prepare sooner when it is coming fast" behaviour,
                            obtained without a single extra parameter.
active      d_active..      today's shipped gains, unchanged. This is the rung
            d_priority      the arm is expected to work on when something is
                            genuinely in the way.
priority    d_priority..    high bandwidth AND a slack multiplier below 1, so
            d_hold          the QP can no longer buy its way out of the row by
                            paying tracking error. Inside d_safe already.
hold        < d_hold        priority gains, plus the task is switched off: the
                            nominal is faded into a braking command. The arm
                            stops PURSUING THE PATH; it does not stop moving,
                            because the barrier rows are constraints and the
                            braking is only an objective.
==========  ==============  ====================================================

THE BOUNDARIES SCALE WITH d_safe
--------------------------------
The rungs are configured as MULTIPLES of ``d_safe`` (``zone_r_*`` in
fr3_control.yaml), never in metres. With absolute boundaries, lowering
``d_safe`` moved the barrier but left the ladder where it was: at
``d_safe = 0.03`` the task was still halved from 0.12 m and suspended at
0.05 m, i.e. the hold rung sat OUTSIDE the barrier and the robot could not be
brought any closer whatever ``d_safe`` said. The shipped ratios
``1.5 / 1.0 / 0.5 / 0.25`` put ``d_safe`` at 2/3 of the outermost boundary and
reproduce the original hardware ladder (0.30 / 0.20 / 0.10 / 0.05 m) exactly
at ``d_safe = 0.20``. :class:`ZoneLadder` itself still takes metres; only
:func:`ladder_from_params` knows about the ratios.

WHY "HOLD" IS NOT A FREEZE, AND WHY THAT MATTERS
------------------------------------------------
The obvious reading of "below 5 cm the robot stops" is a latch that zeroes the
command. Three measurements from the same hardware log say that would be worse
than useless:

* the calibration residual reported 4.2 cm with 4.2 cm of spread, so the 5 cm
  boundary sits INSIDE the calibration error of the sensor that defines it;
* the depth pipeline dropped a control point from 0.242 m to 0.077 m and back
  to 0.253 m in three consecutive frames, and the approach-jump limiter still
  passed 67 mm of that through;
* ``h_unc``, the filter's own admission of what it does not know, ranges from
  0.032 to 0.092 m.

A latch at 5 cm would therefore fire on noise, repeatedly, and each firing is a
hard stop of a moving arm — itself a violent event. Worse, a freeze is the
WRONG answer to the one case that matters: an obstacle still closing at 0.3 m/s
onto a frozen arm makes contact in 130 ms, whereas a retreating arm does not.

So the hold rung suppresses the TASK and leaves the SAFETY. The nominal fades
to ``−k_brake·q̇`` (the arm sheds its speed and holds station), while every CBF
row keeps its full authority to push it away. When the obstacle leaves, the
task fades back in over ``resume_s`` rather than snapping — see
:meth:`ZoneLadder.task_weight`, which is deliberately asymmetric.

WHY SMOOTHSTEP AND NOT A SWITCH
-------------------------------
A hard switch between rungs would put a step of ``Δk0·h̄ + Δk1·ḣ`` into the
right-hand side, and the measured correlation between the commanded radial
acceleration and ``−h_qp`` is 0.9999 — whatever moves the right-hand side comes
straight out of the joints. With the defaults below, switching from active to
priority at d = 0.10 would be a step of 25·h̄, and the whole point of the
exercise is to stop putting steps there. Every weight in here is a smoothstep,
which is C¹ at both ends of its band, and the weights form a partition of unity
so the blended gain is always a convex combination of the rung values and can
never overshoot outside ``[min, max]``.

Pure numpy. No ROS. Every decision this module makes is a function of one
scalar gap and the configured numbers, so it is fully testable headless.
"""

from __future__ import annotations

from typing import NamedTuple, Sequence

import numpy as np

#: Names of the four rungs, ordered from the closest to the furthest. Index
#: into every weight vector this module returns, and the string CBFDIAG prints.
ZONE_NAMES = ('hold', 'priority', 'active', 'notice')

ZONE_HOLD, ZONE_PRIORITY, ZONE_ACTIVE, ZONE_NOTICE = 0, 1, 2, 3


def smoothstep(x: float, lo: float, hi: float) -> float:
    """Hermite 3t²−2t³ ramp from 0 at ``lo`` to 1 at ``hi``.

    Chosen over a linear ramp for one reason: its DERIVATIVE vanishes at both
    ends. A linear ramp is continuous but its slope jumps at each end, and a
    slope jump in a gain that multiplies h̄ is a jerk in the command. Over a
    4 cm band with k0 stepping 25 → 50 that difference is not cosmetic.

    ``hi <= lo`` degenerates to a step at ``lo``, which is what a caller that
    configures a zero-width blend band has asked for.
    """
    if hi <= lo:
        return 0.0 if x < lo else 1.0
    t = (float(x) - lo) / (hi - lo)
    if t <= 0.0:
        return 0.0
    if t >= 1.0:
        return 1.0
    return t * t * (3.0 - 2.0 * t)


class ZoneGains(NamedTuple):
    """What the ladder schedules at one gap."""
    k0: float          #: HOCBF position gain at this gap
    k1: float          #: HOCBF velocity gain at this gap
    slack_m: float     #: slack-column multiplier, <= 1. See ZoneLadder.
    weights: np.ndarray  #: (4,) partition of unity over ZONE_NAMES
    zone: int          #: argmax of ``weights`` — DIAGNOSTIC ONLY, never a gate


class ZoneLadder:
    """Boundaries and per-rung values; answers "what should this gap cost me".

    Args:
        d_notice / d_active / d_priority / d_hold: [m] rung boundaries on the
            SURFACE gap, strictly descending. ``d_notice`` is where the ladder
            starts having an opinion at all; above it the notice values apply
            unchanged, which is harmless because the row is nowhere near
            binding out there.
        k0 / k1: sequences of four gains, ordered as :data:`ZONE_NAMES`
            (hold, priority, active, notice).
        slack_m: (4,) slack-column multipliers, ordered the same way. The row
            reads ``aᵀq̈ + m·s_g >= b``, so m is how much relief one unit of
            paid slack buys: m < 1 means the QP must pay MORE slack — and
            ``½ρs²`` grows with the square — to violate the row by the same
            amount. m = 0.25 is a 16x more expensive violation. This is the
            "maximum priority" lever, and it is preferred over changing ρ
            because ρ lives in the OSQP cost matrix, and touching that forces a
            refactorization on a 100 Hz thread that already starves its own IO.
        blend_m: [m] width of the transition band centred on each boundary.
            Must be no wider than the narrowest gap between boundaries or the
            bands would overlap and the weights would stop being a partition of
            unity; the constructor clamps it and says so via
            :attr:`blend_clamped`.
        task_priority_cut: fraction of the nominal REMOVED at full priority
            weight. 1.0 there would make the priority rung indistinguishable
            from hold.
        resume_s: [s] time for the task weight to climb back from 0 to 1 once
            the gap recovers. Only the climb is rate limited; the fall is
            instant. Same asymmetry as every other estimate in this filter:
            tightening is immediate, relaxing is earned.
    """

    def __init__(
        self,
        *,
        d_notice: float = 0.30,
        d_active: float = 0.20,
        d_priority: float = 0.10,
        d_hold: float = 0.05,
        k0: Sequence[float] = (50.0, 50.0, 25.0, 12.0),
        k1: Sequence[float] = (14.0, 14.0, 10.5, 7.0),
        slack_m: Sequence[float] = (0.25, 0.5, 1.0, 1.0),
        blend_m: float = 0.04,
        task_priority_cut: float = 0.5,
        resume_s: float = 0.5,
    ) -> None:
        b = (float(d_hold), float(d_priority), float(d_active), float(d_notice))
        if not (b[0] < b[1] < b[2] < b[3]):
            raise ValueError(
                f'zone boundaries must ascend hold<priority<active<notice, got {b}')
        self.bounds = b
        self.k0 = np.asarray(k0, dtype=np.float64)
        self.k1 = np.asarray(k1, dtype=np.float64)
        self.slack_m = np.asarray(slack_m, dtype=np.float64)
        for name, arr in (('k0', self.k0), ('k1', self.k1), ('slack_m', self.slack_m)):
            if arr.shape != (4,):
                raise ValueError(f'{name} must have 4 entries (hold, priority, '
                                 f'active, notice), got {arr.shape}')
        # The bands must not overlap or the weights stop summing to one and the
        # blended gain could leave the convex hull of the rung values.
        gap_min = min(b[1] - b[0], b[2] - b[1], b[3] - b[2])
        blend = float(blend_m)
        self.blend_clamped = blend > gap_min
        self.blend_m = min(max(blend, 0.0), gap_min)
        self.task_priority_cut = float(np.clip(task_priority_cut, 0.0, 1.0))
        self.resume_s = float(max(resume_s, 0.0))
        #: Last task weight emitted, so the resume ramp has somewhere to live.
        #: Starts at 1.0 = "the task is running", which is the state a filter
        #: that has never seen an obstacle is in.
        self._task_w = 1.0

    # ── The ladder ──────────────────────────────────────────────────────────

    def weights(self, d: float) -> np.ndarray:
        """(4,) partition of unity over the rungs at surface gap ``d``.

        Built from the three cumulative ramps and differenced, rather than from
        four independent bumps: differencing GUARANTEES the sum is exactly 1.0
        in floating point for any boundary layout, where four bumps would only
        approximately do so and would drift where two bands nearly touch.
        A non-unit sum would show up as a gain that is not a convex combination
        of the rungs, i.e. a gain nobody configured.
        """
        half = 0.5 * self.blend_m
        # t[j] rises through the j-th boundary: 0 below it, 1 above it.
        t = [smoothstep(d, bj - half, bj + half) for bj in self.bounds[:3]]
        w = np.empty(4, dtype=np.float64)
        w[ZONE_HOLD]     = 1.0 - t[0]
        w[ZONE_PRIORITY] = t[0] - t[1]
        w[ZONE_ACTIVE]   = t[1] - t[2]
        w[ZONE_NOTICE]   = t[2]
        # A NaN gap (no measurement) must not silently select the hold rung and
        # stop the robot. Treat it as "far away": the caller's own staleness
        # and finiteness guards are what handle a missing measurement, and this
        # module must not become a second, quieter one.
        if not np.isfinite(d):
            w[:] = 0.0
            w[ZONE_NOTICE] = 1.0
        return w

    def gains(self, d: float) -> ZoneGains:
        """Blended gains and slack multiplier at one surface gap."""
        w = self.weights(d)
        return ZoneGains(float(w @ self.k0), float(w @ self.k1),
                         float(w @ self.slack_m), w, int(np.argmax(w)))

    def gains_array(self, d: np.ndarray):
        """Vectorised :meth:`gains` for a whole row block.

        Returns ``(k0 (n,), k1 (n,), slack_m (n,), zone (n,))``. The obstacle
        rows are scheduled INDIVIDUALLY and never off the global minimum gap:
        a control point half a metre from anything has no business being driven
        at priority bandwidth because a different control point is at 6 cm.
        """
        d = np.atleast_1d(np.asarray(d, dtype=np.float64))
        W = np.empty((d.size, 4), dtype=np.float64)
        for i, di in enumerate(d):
            W[i] = self.weights(float(di))
        return (W @ self.k0, W @ self.k1, W @ self.slack_m,
                np.argmax(W, axis=1).astype(np.int64))

    # ── The task switch ─────────────────────────────────────────────────────

    def task_weight(self, d: float, dt: float) -> float:
        """How much of the commander's q̈_nom survives, in [0, 1].

        ``0`` means the arm is no longer pursuing the path; the caller fades the
        nominal into a braking command, NOT into zero. Zero nominal minimises
        ‖q̈‖², whose solution is q̈ = 0 — constant velocity, i.e. the arm coasts
        into the obstacle at whatever speed it had. That is the opposite of what
        "stop" means and it is the trap this docstring exists to mark.

        Falls instantly, climbs no faster than ``1/resume_s`` per second. The
        fall is a safety decision made on this frame's measurement; the climb is
        a claim that the danger has passed, and a 5 cm boundary read by a sensor
        with 4 cm of calibration spread will cross itself repeatedly on noise.
        Rate limiting only the climb turns that chatter into a single decision.
        """
        w = self.weights(d)
        target = 1.0 - (self.task_priority_cut * float(w[ZONE_PRIORITY])
                        + float(w[ZONE_HOLD]))
        target = float(np.clip(target, 0.0, 1.0))
        if target <= self._task_w:
            self._task_w = target                      # tighten now
        elif self.resume_s <= 0.0:
            self._task_w = target
        else:
            step = float(max(dt, 0.0)) / self.resume_s
            self._task_w = min(target, self._task_w + step)
        return self._task_w

    def reset_task(self, w: float = 1.0) -> None:
        """Force the task weight (node restart, perception reset)."""
        self._task_w = float(np.clip(w, 0.0, 1.0))

    @property
    def task_w(self) -> float:
        """Last task weight emitted, without advancing the ramp."""
        return self._task_w

    def describe(self) -> str:
        b = self.bounds
        return (f'zones hold<{b[0]:.3f} prio<{b[1]:.3f} active<{b[2]:.3f} '
                f'notice<{b[3]:.3f} blend={self.blend_m:.3f} '
                f'k0={list(self.k0)} k1={list(self.k1)} m={list(self.slack_m)}')


def ladder_from_params(P, log=None):
    """Build a :class:`ZoneLadder` from a parameter object, or ``None``.

    ``None`` whenever ``enable_zone_ladder`` is off, and every caller treats
    ``None`` as "no ladder" rather than as "a ladder of ones" — that is what
    keeps a flags-off build byte-for-byte identical instead of merely
    numerically equal.

    One factory rather than two construction sites because the ladder is used
    from two threads at two rates: the 50 Hz constraint builder schedules the
    ROWS, and the 100 Hz QP tick runs the TASK SWITCH, whose resume ramp is
    stateful and must advance once per control tick. They are separate objects
    on purpose (sharing one would put a 100 Hz mutation behind a 50 Hz lock for
    no benefit) and they must not be allowed to drift apart in configuration.

    Boundaries and blend width are ``zone_r_* · d_safe``, so the whole ladder
    follows the barrier when ``d_safe`` is retuned (see the module docstring).
    """
    if not getattr(P, 'enable_zone_ladder', False):
        return None
    d_safe = float(P.d_safe)
    if d_safe <= 0.0:
        raise ValueError(f'zone ladder boundaries are multiples of d_safe, which '
                         f'must be > 0 when enable_zone_ladder is on (got {d_safe})')
    z = ZoneLadder(
        d_notice=P.zone_r_notice * d_safe, d_active=P.zone_r_active * d_safe,
        d_priority=P.zone_r_priority * d_safe, d_hold=P.zone_r_hold * d_safe,
        k0=(P.zone_k0_priority, P.zone_k0_priority, P.k0_cbf, P.zone_k0_notice),
        k1=(P.zone_k1_priority, P.zone_k1_priority, P.k1_cbf, P.zone_k1_notice),
        slack_m=(P.zone_slack_m_hold, P.zone_slack_m_priority, 1.0, 1.0),
        blend_m=P.zone_blend_r * d_safe,
        task_priority_cut=P.zone_task_priority_cut,
        resume_s=P.zone_resume_s)
    if z.blend_clamped and log is not None:
        log.warn(f'zone_blend_r={P.zone_blend_r} (x d_safe) is wider than the '
                 f'narrowest gap between zone boundaries; clamped to '
                 f'{z.blend_m:.4f} m so the blend bands stay disjoint and the '
                 f'blended gains stay a convex combination of the rung values')
    return z
