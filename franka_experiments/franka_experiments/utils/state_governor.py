"""Task attenuation keyed on the robot's OWN state, not on the obstacle.

WHY THIS EXISTS
---------------
The filter already had one task switch — :class:`~.cbf_zones.ZoneLadder`, which
fades the trajectory as an OBSTACLE closes. It has no opinion about the arm
itself. Three of the arm's own states can make a command infeasible with no
obstacle anywhere near, and for each of them the filter's only answer was a
constraint that fights the trajectory locally:

* a joint approaching the FR3 **velocity envelope** (see
  :func:`~.cbf_hard_limits.fr3_velocity_envelope`). The hard box clamps it, but
  a box clamps ONE joint: the QP then dumps the rest of the tracking demand on
  the others, which is the ``dq_ort`` blowup the joint-limit rows were added to
  fix, one layer down.
* a **singularity**: σ_min of the task Jacobian collapsing means a bounded
  Cartesian demand becomes an unbounded joint demand. The singularity row
  pushes σ back up, but it is a slack-backed row, and the QP can buy its way
  out of it while the trajectory keeps asking.
* an imminent **self-collision**: same shape, same slack.

All three are cases where the honest answer is not "push harder against the
constraint" but "stop asking for so much". That is what this does, and it is
the same lever the zone ladder pulls: one scalar in [0, 1] multiplying the
nominal, with the remainder going to a braking command.

WHY IT FADES TO BRAKING AND NOT TO ZERO
---------------------------------------
``q̈_nom = 0`` minimises ‖q̈‖², whose solution is q̈ = 0 — "hold this velocity".
The arm coasts into whatever it was heading for at the speed it already had.
Every fade in this filter goes to ``−k_brake·q̇`` for this reason, and this one
is no exception. See :meth:`~.cbf_zones.ZoneLadder.task_weight`.

WHY IT DOES NOT DEADLOCK
------------------------
A task switch that keys on a state the robot cannot leave is a trap. All three
terms here are SELF-HEALING, and deliberately so:

* the velocity margin is measured only in the DIRECTION OF TRAVEL. A joint
  sitting on its envelope with q̇ = 0 has full margin (the bound that matters is
  the one ahead of it), so the governor releases the moment the joint stops. A
  joint still creeping INTO the wall is braked by the fade itself, which
  reverses q̇ and restores the margin within a tick or two.
* σ_min and the self-collision gap are both pushed back up by their own CBF
  rows, whose authority this never touches — the fade is applied to the TASK,
  before the biases and rows, exactly like the zone ladder's.

WHY THE COMMANDER IS NOT TOLD
-----------------------------
``cbf_status`` carries ``d_obs`` so the commander's phase governor can hold the
reference on genuine proximity. This weight is deliberately NOT published there.
The reason is in ``_publish_status``: freezing the phase on something that is
not obstacle proximity deadlocks — error grows, phase freezes, the reference
parks on the blocked pose, and the blocking state never clears. A joint pinned
against its position limit is precisely such a state: parking the reference
there is permanent. So the trajectory keeps running, the arm falls behind, and
if it falls far enough the commander does its own HARD reset and re-plans from
where the arm actually is. A controlled re-plan is a much better outcome than a
firmware abort, which is what happened instead in every run this was written
for.

Pure numpy. No ROS. Every decision is a function of the four scalars passed in.
"""

from __future__ import annotations

from typing import NamedTuple

import numpy as np

from franka_experiments.utils.cbf_hard_limits import fr3_velocity_envelope
from franka_experiments.utils.cbf_zones import smoothstep

#: Order of the terms in :attr:`GovernorState.terms`, and the string the
#: CBFDIAG ``gov=`` field prints for the binding one.
TERM_NAMES = ('vel', 'sing', 'sc')

TERM_VEL, TERM_SING, TERM_SC = 0, 1, 2


class GovernorState(NamedTuple):
    """What the governor decided this tick."""
    w: float               #: task weight actually emitted, after the ramp
    target: float          #: weight this tick's margins asked for, before it
    binding: str           #: name of the term that set *target*, or ``'-'``
    weights: tuple         #: (3,) per-term weights, ordered as TERM_NAMES
    margins: tuple         #: (3,) raw margins, each in its own unit


class StateGovernor:
    """How much of the task survives the arm's own state.

    Args:
        qdot_band: [rad/s] headroom below the velocity envelope over which the
            task fades from full to zero. The margin is measured per joint in
            the direction that joint is travelling, and the worst joint wins.
            Sized against what one tick of authority can add: at 10 rad/s² and
            a 100 Hz QP a joint gains 0.1 rad/s per tick, so a band under that
            cannot act before the box does and the governor is decoration.
        sigma_floor: σ_min at which the task is fully suspended. Pass the same
            value as ``singularity_sigma_floor`` so the governor and the
            singularity row agree on where the wall is.
        sigma_band: width of the σ_min fade above *sigma_floor*.
        sc_margin: [m] self-collision gap at which the task is fully suspended.
        sc_band: [m] width of the self-collision fade above *sc_margin*.
        resume_s: [s] time to climb back from 0 to 1 once the margins recover.
            Only the climb is rate limited. Same asymmetry as everything else
            in this filter: tightening is a measurement, relaxing is a claim.
        envelope_margin: fraction of the firmware velocity envelope the
            governor measures against. Keep it at or below the hard box's
            ``velocity_box_margin`` so the governor starts fading BEFORE the box
            starts clamping — a governor that engages after the clamp has
            already bitten has nothing left to prevent.
    """

    def __init__(
        self,
        *,
        qdot_band: float = 0.30,
        sigma_floor: float = 0.05,
        sigma_band: float = 0.04,
        sc_margin: float = 0.02,
        sc_band: float = 0.04,
        resume_s: float = 0.5,
        envelope_margin: float = 0.9,
    ) -> None:
        if qdot_band <= 0.0 or sigma_band <= 0.0 or sc_band <= 0.0:
            raise ValueError(
                f'governor bands must be positive, got qdot={qdot_band}, '
                f'sigma={sigma_band}, sc={sc_band}')
        self.qdot_band = float(qdot_band)
        self.sigma_floor = float(sigma_floor)
        self.sigma_band = float(sigma_band)
        self.sc_margin = float(sc_margin)
        self.sc_band = float(sc_band)
        self.resume_s = float(resume_s)
        self.envelope_margin = float(envelope_margin)
        self._w = 1.0

    # ── the three margins ────────────────────────────────────────────────

    def velocity_margin(self, q: np.ndarray, qdot: np.ndarray) -> float:
        """[rad/s] worst per-joint headroom to the envelope, ahead of the joint.

        ``bound − |q̇|`` where *bound* is the envelope on the side the joint is
        MOVING TOWARD. Direction matters: a joint parked on its lower envelope
        has no headroom downward and all of it upward, and treating that as
        "zero margin" would suspend the task on a stationary arm that is in no
        danger at all — and keep it suspended, since nothing would move.

        Negative when a joint is already outside the envelope, which the caller
        maps to a fully suspended task.
        """
        up, lo = fr3_velocity_envelope(np.asarray(q, dtype=float),
                                       margin=self.envelope_margin)
        qd = np.asarray(qdot, dtype=float)
        bound = np.where(qd >= 0.0, up, -lo)
        return float(np.min(bound - np.abs(qd)))

    # ── the decision ─────────────────────────────────────────────────────

    def weight(self, *, q, qdot, sigma=None, d_sc=None, dt: float = 0.0
               ) -> GovernorState:
        """Fold the three margins into one weight, and advance the ramp.

        Args:
            q: (7,) joint positions [rad].
            qdot: (7,) joint velocities [rad/s].
            sigma: σ_min of the task Jacobian, or None/NaN when the singularity
                rows are off. A term with no measurement has no opinion — it
                contributes weight 1.0 rather than 0.0, because a governor that
                suspends the task whenever a diagnostic is missing is a governor
                that will be switched off within a day.
            d_sc: [m] closest self-collision capsule gap, or None/inf.
            dt: [s] tick period, for the resume ramp.
        """
        m_v = self.velocity_margin(q, qdot)
        w_v = smoothstep(m_v, 0.0, self.qdot_band)

        m_s = float('inf')
        w_s = 1.0
        if sigma is not None and np.isfinite(sigma):
            m_s = float(sigma) - self.sigma_floor
            w_s = smoothstep(m_s, 0.0, self.sigma_band)

        m_c = float('inf')
        w_c = 1.0
        if d_sc is not None and np.isfinite(d_sc):
            m_c = float(d_sc) - self.sc_margin
            w_c = smoothstep(m_c, 0.0, self.sc_band)

        weights = (w_v, w_s, w_c)
        target = min(weights)
        i = int(np.argmin(weights))
        binding = TERM_NAMES[i] if target < 1.0 else '-'

        if target <= self._w:
            self._w = target                         # tighten now
        elif self.resume_s <= 0.0:
            self._w = target
        else:
            self._w = min(target, self._w + max(dt, 0.0) / self.resume_s)

        return GovernorState(w=self._w, target=target, binding=binding,
                             weights=weights, margins=(m_v, m_s, m_c))

    def reset(self, w: float = 1.0) -> None:
        """Force the weight (node restart, perception reset)."""
        self._w = float(np.clip(w, 0.0, 1.0))

    @property
    def w(self) -> float:
        """Last weight emitted, without advancing the ramp."""
        return self._w

    def describe(self) -> str:
        return (f'state governor: qdot_band={self.qdot_band:.3f} rad/s @ '
                f'{self.envelope_margin:.0%} envelope, sigma>{self.sigma_floor:.3f}'
                f'+{self.sigma_band:.3f}, d_sc>{self.sc_margin:.3f}'
                f'+{self.sc_band:.3f} m, resume={self.resume_s:.2f} s')


def governor_from_params(P, log=None):
    """Build a :class:`StateGovernor` from the parameter object, or ``None``.

    ``None`` when ``state_governor_enabled`` is false, so the caller's
    ``if self._gov is not None`` gate is the only branch and the off path is
    bit-identical to not having the module at all.
    """
    if not getattr(P, 'state_governor_enabled', False):
        return None
    gov = StateGovernor(
        qdot_band=P.governor_qdot_band,
        sigma_floor=P.singularity_sigma_floor,
        sigma_band=P.governor_sigma_band,
        sc_margin=P.self_collision_row_margin,
        sc_band=P.governor_sc_band,
        resume_s=P.governor_resume_s,
        envelope_margin=P.governor_envelope_margin,
    )
    if log is not None:
        log.info(gov.describe())
    return gov
