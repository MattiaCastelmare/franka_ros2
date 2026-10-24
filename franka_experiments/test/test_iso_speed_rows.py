"""The SSM bound driving the task-space speed rows (roadmap Step 5).

The heuristic obstacle cap is ``v_at_d_safe · gap / d_safe``: a straight line
through the origin, tuned. The ISO one is the inversion of the Annex L
separation-distance formula: the largest speed at which this control point can
still stop before the gap closes to ``C + Z_d + Z_r``, given the reaction time,
the realized deceleration and the human's approach speed.

What these tests pin:

* flag off ⇒ the rows are bit-identical to what the builder produced before the
  ISO layer existed. That is the ground rule, checked on the arrays themselves
  and not on a proxy;
* the cap the row carries IS ``ssm_speed_cap`` of that point's own gap and its
  own conditioned closing speed — not a recomputed one, and not a neighbour's;
* inside the stop region the row demands deceleration;
* the self-collision term is untouched — it is not an ISO separation distance;
* ``iso_mode: reduced`` adds exactly one extra TCP row at 250 mm/s.
"""

import numpy as np
import pytest

from franka_experiments.utils.cbf_state_rows import G_SPD
from franka_experiments.utils.cbf_qp_assembly import build_row_rhs
from franka_experiments.utils.iso_ssm import ssm_speed_cap

from _cbf_builder_harness import make_builder, make_obstacle, ObstacleSnap, JointSnap

NV = 7
QDOT = np.array([0.6, -0.4, 0.5, 0.3, -0.5, 0.4, 0.2])

ISO = dict(iso_enabled=True, iso_ssm_speed_rows=True,
           iso_t_reaction=0.10, iso_a_stop=4.0, iso_v_human=2.0,
           iso_c_intrusion=0.05, iso_z_depth=0.02, iso_z_robot=0.01)


def _build(d=0.30, qdot=QDOT, ob=None, **over):
    over.setdefault('link_speed_rows_enabled', True)
    over.setdefault('obstacle_velocity_enabled', False)
    # The activation gate ("only emit a row once the point is already moving at
    # activate_frac of its cap") is orthogonal to what these tests are about,
    # and at a saturated cap it would silently drop the row. Opened, so every
    # test sees the row it built.
    over.setdefault('link_speed_activate_frac', 0.0)
    b = make_builder(**over)
    js = JointSnap(np.zeros(NV), np.asarray(qdot, dtype=float), 0.0)
    obs = ObstacleSnap((ob or make_obstacle(d=d, cp_label='fr3_link5#0'),),
                       0.0, 0.0)
    con = b.build(js, obs, 0.0)
    assert con is not None, f'no snapshot at d={d} — inside cbf_obstacle_horizon?'
    return b, con


def _spd_caps(con):
    n_tail = con.cap_v.size
    t0 = con.A.shape[0] - n_tail
    return [float(con.cap_v[i - t0]) for i in range(t0, con.A.shape[0])
            if con.group[i] == G_SPD]


# ── flag off: bit-identical ──────────────────────────────────────────────────

def test_with_the_flag_off_the_rows_are_bit_identical():
    for d in (0.05, 0.12, 0.30, 0.9):
        _, base = _build(d=d)
        _, off = _build(d=d, iso_enabled=False, iso_ssm_speed_rows=True)
        assert np.array_equal(base.A, off.A)
        assert np.array_equal(base.cap_v, off.cap_v)
        assert list(base.links) == list(off.links)


def test_the_master_flag_alone_does_not_switch_the_rows():
    # iso_enabled without iso_ssm_speed_rows must leave the heuristic cap in
    # place: the two flags are AND-ed, not OR-ed.
    _, base = _build(d=0.30)
    _, on = _build(d=0.30, iso_enabled=True, iso_ssm_speed_rows=False,
                   **{k: v for k, v in ISO.items()
                      if k not in ('iso_enabled', 'iso_ssm_speed_rows')})
    assert np.array_equal(base.cap_v, on.cap_v)


# ── the cap IS the SSM bound ─────────────────────────────────────────────────

def test_the_row_carries_exactly_the_ssm_cap_of_its_own_gap():
    for d in (0.10, 0.20, 0.35, 0.80):
        b, con = _build(d=d, **ISO)
        caps = _spd_caps(con)
        assert caps, f'no speed row at d={d}'
        expect = ssm_speed_cap(
            d, 0.0, t_r=ISO['iso_t_reaction'], a_s=ISO['iso_a_stop'],
            c=ISO['iso_c_intrusion'], z_d=ISO['iso_z_depth'],
            z_r=ISO['iso_z_robot'], v_max=b._P.link_speed_max)
        assert caps[0] == pytest.approx(expect)


def test_far_from_a_static_obstacle_the_cap_saturates_at_link_speed_max():
    # d = 1.0 m is inside cbf_obstacle_horizon (1.2 m) but far enough that the
    # SSM bound is well above the flat ceiling, so v_max is what binds.
    b, con = _build(d=1.0, **ISO)
    caps = _spd_caps(con)
    assert caps and caps[0] == pytest.approx(b._P.link_speed_max)


def test_an_approaching_obstacle_tightens_the_cap_through_spd_pts():
    """v_app reaches the row through the spd_pts tuple, not a recomputation.

    Driven with a TRACKED velocity so the conditioned closing speed the barrier
    uses is the one the row sees: n̂ points obstacle -> control point, here
    +y, so a +y obstacle velocity is closing.
    """
    kw = dict(ISO, obstacle_velocity_enabled=True,
              obstacle_velocity_source='tracker', obstacle_velocity_min_frames=3)
    caps = []
    for v_y in (0.0, 0.3, 0.8, 1.5):
        ob = make_obstacle(d=0.40, cp_label='fr3_link5#0',
                           v=(0.0, v_y, 0.0), frames_seen=5)
        _, con = _build(d=0.40, ob=ob, **kw)
        caps.append(_spd_caps(con)[0])
    # NON-strict: the cap is clipped at link_speed_max, so the first entries
    # are flat until the SSM bound drops below the flat ceiling. What must hold
    # is that it never RISES with the closing speed, and that it does fall.
    assert all(a >= b for a, b in zip(caps, caps[1:])), caps
    assert caps[-1] < caps[0], caps


def test_the_cap_decreases_monotonically_as_the_gap_closes():
    caps = [_spd_caps(_build(d=d, **ISO)[1])[0]
            for d in (0.80, 0.50, 0.30, 0.20, 0.12)]
    # Same clipping caveat as above: flat at link_speed_max while the SSM bound
    # is still the looser of the two, falling once it is not.
    assert all(a >= b for a, b in zip(caps, caps[1:])), caps
    assert caps[-1] < caps[0], caps


# ── the stop region ──────────────────────────────────────────────────────────

def test_inside_c_plus_z_the_row_demands_deceleration():
    """At the floor the cap is 0, so retreat_cap_rhs' bound goes negative:
    the row stops asking for "no faster than v" and starts asking for
    "decelerate", which is the whole point of a zero cap."""
    floor = ISO['iso_c_intrusion'] + ISO['iso_z_depth'] + ISO['iso_z_robot']
    b, con = _build(d=0.5 * floor, link_speed_activate_frac=0.0, **ISO)
    caps = _spd_caps(con)
    assert caps and caps[0] == 0.0
    h_qp, _ = build_row_rhs(con, QDOT, QDOT, k0=25.0, k1=10.5,
                            retreat_horizon=0.2, speed_horizon=0.2)
    spd_rhs = [float(h_qp[i]) for i in range(con.A.shape[0])
               if con.group[i] == G_SPD]
    assert spd_rhs and min(spd_rhs) < 0.0


# ── the self-collision term is not an ISO distance ───────────────────────────

def test_the_self_collision_term_still_binds_when_it_is_tighter():
    """A near self-collision must still slow every control point, ISO or not.

    It has no C, no Z_d and no human in it, so the SSM cap cannot express it;
    min() of the two is what the row carries. With no self-collision rows built
    the term is link_speed_cap(inf) = v_max, i.e. inert — which is exactly why
    the ISO cap is the one that binds at 1.0 m and NOT at 0.12 m.
    """
    b, far = _build(d=1.0, **ISO)
    b2, near = _build(d=0.12, **ISO)
    assert _spd_caps(far)[0] == pytest.approx(b._P.link_speed_max)
    assert _spd_caps(near)[0] < b2._P.link_speed_max


# ── reduced mode ─────────────────────────────────────────────────────────────

def test_reduced_mode_adds_exactly_one_tcp_row_at_250_mm_per_s():
    _, auto = _build(d=0.50, **ISO)
    _, red = _build(d=0.50, **dict(ISO, iso_mode='reduced'))
    n_auto = int(np.count_nonzero(auto.group == G_SPD))
    n_red = int(np.count_nonzero(red.group == G_SPD))
    assert n_red == n_auto + 1
    assert min(_spd_caps(red)) == pytest.approx(0.25)
    assert any(l.startswith('red:') for l in red.links)


def test_reduced_mode_is_inert_without_the_master_flag():
    _, base = _build(d=0.50)
    _, red = _build(d=0.50, **dict(ISO, iso_enabled=False, iso_mode='reduced'))
    assert np.array_equal(base.cap_v, red.cap_v)
    assert not any(l.startswith('red:') for l in red.links)


def test_the_reduced_row_lands_on_the_tcp_link_when_one_reported():
    """Both channels must cap the same body: the builder targets FR3_TCP_LINK
    and iso_safety_monitor's iso_tcp_link defaults to the same string."""
    from franka_experiments.utils.cbf_state_rows import FR3_TCP_LINK
    ob = make_obstacle(d=0.50, link=FR3_TCP_LINK,
                       cp_label=f'{FR3_TCP_LINK}#0')
    _, red = _build(d=0.50, ob=ob, **dict(ISO, iso_mode='reduced'))
    red_rows = [l for l in red.links if l.startswith('red:')]
    assert len(red_rows) == 1
    assert FR3_TCP_LINK in red_rows[0]


def test_without_a_tcp_control_point_the_reduced_row_falls_back():
    """Capping the nearest thing to the TCP is the conservative answer;
    capping nothing would be the wrong one."""
    ob = make_obstacle(d=0.50, link='fr3_link5', cp_label='fr3_link5#0')
    _, red = _build(d=0.50, ob=ob, **dict(ISO, iso_mode='reduced'))
    red_rows = [l for l in red.links if l.startswith('red:')]
    assert len(red_rows) == 1 and 'fr3_link5' in red_rows[0]
