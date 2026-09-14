"""Self-collision rows: engage EARLY on a fast approach, and stop flickering.

The failure this file is the regression for: the arm stopping with
``joint_velocity_violation`` — the red lights — in poses where it was heading
into a self-collision. The collision itself was never reached; what stopped the
arm was the AVOIDANCE.

With a fixed 0.08 m engagement horizon, a capsule pair closing at 0.5 m/s got
~160 ms of warning. The HOCBF row that then appeared demanded
``aᵀq̈ >= −k1·(aᵀq̇) − k0·h̄``, which at that gap and that speed is a large
deceleration materialising in ONE 20 ms rebuild: a step in q̈_safe, a torque
spike through M·q̈, measured velocity overshooting the envelope, firmware
reflex.

Two changes, both tested here:

* the horizon is widened by ``lead_s · (closing speed)``, so a fast approach
  engages while h̄ is still large and the row is still slack, and tightens
  continuously from there;
* an engaged pair keeps its row until the gap opens past ``release ·
  horizon_eff``, so a pair sitting on the boundary stops adding and removing
  its row on alternate rebuilds.

Both are STRICTLY CONSERVATIVE — they can only make a row appear earlier or
persist longer, never suppress one the old code would have emitted — and that
is asserted rather than assumed.

Pure numpy: the builder is driven with a stub kinematics object, so this needs
no Pinocchio and no ROS.
"""

import numpy as np

from franka_experiments.utils.cbf_state_rows import SelfCollisionRowBuilder
from franka_experiments.utils.self_collision import Capsule

NV = 7


class _StubPlacement:
    """Pinocchio's oMf[fid] reduced to what _world_endpoints reads."""

    def __init__(self, translation):
        self.rotation = np.eye(3)
        self.translation = np.asarray(translation, dtype=np.float64)


class _StubData:
    def __init__(self):
        self.oMf = {}


class _StubKin:
    """Stands in for CBFKinematics: capsule placement + point Jacobians.

    The Jacobians are arbitrary but FIXED and full rank on the arm columns —
    this file is about WHICH pairs produce a row and WHEN, not about the row's
    numeric content, which test_cbf_state_rows already covers.
    """

    def __init__(self, nv_full=NV):
        self.data = _StubData()
        self._nv = nv_full

    def place(self, fid, translation):
        self.data.oMf[fid] = _StubPlacement(translation)

    def point_jacobian(self, fid, p):
        J = np.zeros((3, self._nv))
        J[0, 0] = J[1, 1] = J[2, 2] = 1.0
        J[0, 3] = 0.5                      # a little coupling, still finite
        return J, np.zeros((3, self._nv))


def _builder(**kw):
    """Two unit-radius-free capsules on the x axis, one pair between them."""
    caps = [
        Capsule(frame='a_sc', body='link_a', p1=np.zeros(3), p2=np.zeros(3),
                radius=0.0),
        Capsule(frame='b_sc', body='link_b', p1=np.zeros(3), p2=np.zeros(3),
                radius=0.0),
    ]
    kw.setdefault('margin', 0.0)
    kw.setdefault('horizon', 0.08)
    kw.setdefault('max_rows', 8)
    return SelfCollisionRowBuilder(caps, [(0, 1)], [10, 11],
                                   np.arange(NV), **kw)


def _kin_at(gap):
    """Stub kinematics with the pair's surfaces exactly *gap* apart."""
    kin = _StubKin()
    kin.place(10, [0.0, 0.0, 0.0])
    kin.place(11, [gap, 0.0, 0.0])
    return kin


def _rows_at(builder, gap, stamp):
    return builder.build(_kin_at(gap), np.zeros(NV), stamp=stamp)


# ── The old behaviour is still the floor ─────────────────────────────────────

def test_stationary_pair_keeps_the_configured_horizon():
    """A pair that is not closing must engage at exactly the old distance —
    the anticipation may not make the rows chatty in a normal pose."""
    b = _builder(lead_s=0.35)
    for step in range(5):                     # genuinely stationary at 0.20 m
        assert _rows_at(b, 0.20, 0.02 * step) == []
    # Held stationary just OUTSIDE the fixed horizon: still nothing.
    b2 = _builder(lead_s=0.35)
    for step in range(5):
        assert _rows_at(b2, 0.081, 0.02 * step) == []
    # Held stationary just INSIDE it: the plain horizon still engages.
    b3 = _builder(lead_s=0.35)
    for step in range(5):
        assert len(_rows_at(b3, 0.079, 0.02 * step)) == 1


def test_anticipation_never_shrinks_the_horizon():
    """v_close is clamped to the approaching half, so a pair moving APART must
    engage no later than the fixed horizon would have."""
    b = _builder(lead_s=0.35)
    _rows_at(b, 0.05, 0.00)                   # engaged, inside the horizon
    # Now opening fast. It stays engaged (hysteresis), and at no point does a
    # receding pair get a horizon smaller than the configured one.
    assert len(_rows_at(b, 0.07, 0.02)) == 1


def test_disabled_anticipation_reproduces_the_fixed_horizon():
    """lead_s = 0 and release = 1 is the exact pre-change behaviour, so the
    new code path is reachable-off from configuration."""
    b = _builder(lead_s=0.0, release=1.0)
    _rows_at(b, 0.30, 0.00)
    assert _rows_at(b, 0.20, 0.02) == []      # closing at 5 m/s, still ignored
    assert _rows_at(b, 0.10, 0.04) == []
    assert len(_rows_at(b, 0.07, 0.06)) == 1  # only the plain gap matters


# ── The fix: a fast approach engages early ───────────────────────────────────

def test_fast_approach_engages_far_outside_the_fixed_horizon():
    """The regression. A pair closing at ~0.5 m/s must have its row well
    before the 0.08 m fixed horizon — that early, slack row is what lets the
    barrier tighten over hundreds of ms instead of stepping."""
    b = _builder(lead_s=0.35, gap_vel_alpha=0.0)   # alpha 0: no EMA lag
    _rows_at(b, 0.40, 0.00)
    rows = _rows_at(b, 0.39, 0.02)                 # 0.5 m/s closure
    # lead = 0.35 * 0.5 = 0.175 m, so the horizon is 0.255 m: 0.39 is still
    # outside, but the barrier is now being tracked.
    assert rows == []
    rows = _rows_at(b, 0.25, 0.04)                 # still ~0.5 m/s and closer
    assert len(rows) == 1, 'a fast approach must engage before 0.08 m'
    # ...and the row it produced is SLACK, not an emergency: h is large.
    _, h, _, _ = rows[0]
    assert h > 0.20


def test_engagement_distance_grows_with_closing_speed():
    """Monotone in the driver: the faster the approach, the earlier the row.
    A sign slip or a clamp on the wrong half would flatten this.

    Every case closes at its own constant speed over the same 20 ms rebuild,
    so the only thing separating them is the speed itself. The expected
    engagement distances are 0.08 + 0.35·v = 0.115 / 0.185 / 0.36 m, spaced far
    wider than the one-step sampling granularity (v·0.02 <= 16 mm).
    """
    dt = 0.02
    reached = []
    for v in (0.1, 0.3, 0.8):
        b = _builder(lead_s=0.35, gap_vel_alpha=0.0)
        gap, t, first = 0.60, 0.0, None
        while gap > 0.02 and first is None:
            if _rows_at(b, gap, t):
                first = gap
            gap -= v * dt
            t += dt
        reached.append(first)
    assert all(g is not None for g in reached), reached
    assert reached[1] > reached[0], reached
    assert reached[2] > reached[1], reached
    # ...and each lands near the predicted 0.08 + 0.35·v, within one step.
    for v, g in zip((0.1, 0.3, 0.8), reached):
        assert abs(g - (0.08 + 0.35 * v)) < 2 * v * dt + 1e-9, (v, g)


def test_closing_speed_ignores_an_unusable_timestamp():
    """First sight, a missing stamp or a clock jump must fall back to "no
    anticipation" rather than to a garbage rate — the same policy the obstacle
    path already uses for an implausible header stamp."""
    b = _builder(lead_s=0.35)
    assert b._closing_speed(0, 0.30, None) == 0.0        # no stamp
    assert b._closing_speed(0, 0.30, 1.0) == 0.0         # first sight
    assert b._closing_speed(0, 0.20, 1.0) == 0.0         # dt = 0, duplicate
    assert b._closing_speed(0, 0.20, 99.0) == 0.0        # dt = 98 s, clock jump


# ── The fix: no flicker on the boundary ──────────────────────────────────────

def test_hysteresis_keeps_a_boundary_pair_engaged():
    """A pair parked exactly on the horizon used to add and remove its row on
    alternate rebuilds — a constraint-set discontinuity every 20 ms, each one
    an OSQP setup() with the warm start discarded."""
    b = _builder(lead_s=0.0, release=1.6)
    assert len(_rows_at(b, 0.079, 0.00)) == 1        # engages
    assert len(_rows_at(b, 0.085, 0.02)) == 1        # would have dropped at 1.0
    assert len(_rows_at(b, 0.090, 0.04)) == 1        # still held
    assert _rows_at(b, 0.140, 0.06) == []            # past 1.6 * 0.08: released


def test_release_of_one_means_no_hysteresis():
    """release = 1.0 is the old, flicker-prone behaviour, kept reachable."""
    b = _builder(lead_s=0.0, release=1.0)
    assert len(_rows_at(b, 0.079, 0.00)) == 1
    assert _rows_at(b, 0.081, 0.02) == []


def test_engaged_set_is_dropped_when_everything_opens_up():
    """Leaving the horizon entirely must clear the engaged set, or the
    hysteresis would resurrect a stale membership on the next approach."""
    b = _builder(lead_s=0.0, release=1.6)
    _rows_at(b, 0.05, 0.00)
    assert b._engaged
    _rows_at(b, 1.00, 0.02)
    assert not b._engaged


# ── Bookkeeping the diagnostics rely on ──────────────────────────────────────

def test_min_gap_is_reported_even_when_no_row_is_emitted():
    """last_min_gap feeds d_sc on the CBFDIAG line and must track the geometry
    whether or not the pair was close enough to constrain anything."""
    b = _builder(lead_s=0.0)
    _rows_at(b, 0.50, 0.00)
    assert np.isclose(b.last_min_gap, 0.50)
    assert b.last_pair


def test_lead_is_reported_for_diagnostics():
    b = _builder(lead_s=0.35, gap_vel_alpha=0.0)
    _rows_at(b, 0.40, 0.00)
    _rows_at(b, 0.39, 0.02)
    assert b.last_lead > 0.0
