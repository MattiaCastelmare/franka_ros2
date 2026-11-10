"""The Self-Shadow Barrier as a CONSTRAINT: rows, slack, and inertness.

Phase 5 of the SSB work. The geometry and its derivatives are verified in
test_self_shadow_geometry.py and test_self_shadow_derivatives.py; the
perception side in test_ssb_perception.py. What is left, and what is checked
here, is whether the rows reach the QP correctly:

  * flag off ⇒ BIT-IDENTICAL constraint matrices (the whole backward
    compatibility claim, and the only honest way to state it),
  * the row's own family, its own slack column and nobody else's,
  * the engage/release hysteresis, the row cap, the hold and its erosion,
  * the leverage gate,
  * and that an SSB row actually BINDS in a solved QP — a row that is built
    but never binds would be a decoration, not a constraint.

Everything here RUNS the shipped ConstraintBuilder on the shipped harness.
"""
from __future__ import annotations

import numpy as np
import pytest

from _cbf_builder_harness import (
    make_builder, make_obstacle, make_shadow, make_js, make_obs, NV, ACC)
from franka_experiments.utils.cbf_state_rows import (
    G_OBS, G_SSB, GROUP_NAMES, NV as NV_ROWS, NX, N_SLACK)
from franka_experiments.utils.cbf_diagnostics import (
    FAMILIES_ALL, STATUS_FIELDS, family_violations)

SSB_ON = dict(ssb_rows_enabled=True, ssb_hold_s=0.0,
              ssb_inflate_speed_mps=0.0)

#: Where the harness' control points sit. The harness leaves cp_positions
#: empty by default (only the obstacle memory needed it), and SSB rows are
#: rebuilt at the CURRENT control-point position, so every SSB test has to
#: place them.
CP_XYZ = {
    'fr3_link4#0': np.array([0.40, 0.00, 0.60]),
    'fr3_link5#0': np.array([0.50, 0.00, 0.50]),
    'fr3_link5#1': np.array([0.52, 0.00, 0.46]),
    'fr3_link5#2': np.array([0.54, 0.00, 0.42]),
    'fr3_link6#0': np.array([0.56, 0.00, 0.38]),
    'fr3_link6#1': np.array([0.58, 0.00, 0.34]),
    'fr3_link6#2': np.array([0.60, 0.00, 0.30]),
}


def ssb_kin():
    from _cbf_builder_harness import _Kin
    kin = _Kin()
    kin.cp_positions.update({k: v.copy() for k, v in CP_XYZ.items()})
    return kin


def ssb_builder(**over):
    """``(builder, kin)`` with the control points placed."""
    kin = ssb_kin()
    opts = dict(SSB_ON)
    opts.update(over)
    return make_builder(kin=kin, cp_jacobian=True, **opts), kin


def shadow_at(kin, label='fr3_link5#0', gap=0.10, direction=(0.0, -1.0, 0.0),
              radius=0.0):
    """A shadow whose SURFACE gap at the current control point is ``gap``.

    ``direction`` points FROM the control point TOWARD the shadow, so the
    barrier normal n̂ = (p_j − x)/‖·‖ is its negation.

    ``radius`` is the capsule radius perception had already subtracted, so the
    centre distance is ``gap + radius`` and the builder must recover the same
    radius from ``|pr - ph| - d`` to get ``gap`` back.
    """
    cp = np.asarray(kin.cp_positions[label], dtype=np.float64)
    u = np.asarray(direction, dtype=np.float64)
    u = u / np.linalg.norm(u)
    ph = cp + u * (float(gap) + float(radius))
    return make_shadow(cp_label=label, link=label.partition('#')[0],
                       d=float(gap), pr=cp, ph=ph)


# ─────────────────────────────────────────────────────────────────────────────
#  The layout contract
# ─────────────────────────────────────────────────────────────────────────────

def test_the_family_is_appended_not_inserted():
    """Every pre-existing family keeps its index, and ssb is last.

    This is what makes a bag recorded before SSB still readable: the slack
    vector is zipped against GROUP_NAMES positionally in several places, and
    an INSERTED family would silently re-label every column after it.
    """
    assert GROUP_NAMES[:6] == ('obs', 'sc', 'qlim', 'sing', 'cap', 'spd')
    assert GROUP_NAMES[6] == 'ssb'
    assert G_SSB == 6 and N_SLACK == 7 and NX == NV_ROWS + 7
    # cbf_diagnostics must agree with the decision vector, or family_violations
    # reports the wrong column.
    assert FAMILIES_ALL == GROUP_NAMES
    # The three v3 fields are the LAST three, after constraint_build_wall_ms.
    assert STATUS_FIELDS[-4] == 'constraint_build_wall_ms'
    assert STATUS_FIELDS[-3:] == ('slack_ssb', 'violation_without_slack_ssb',
                                  'violation_with_slack_ssb')


# ─────────────────────────────────────────────────────────────────────────────
#  Flag off is bit-identical
# ─────────────────────────────────────────────────────────────────────────────

def _drive(b, kin, shadows, *, n_frames=1, dt=0.02):
    con = None
    for k in range(n_frames):
        t = k * dt
        sh = shadows(k, kin) if callable(shadows) else shadows
        con = b.build(make_js(stamp=t),
                      make_obs([make_obstacle()], stamp=t, shadows=sh), t)
    return con


def test_flag_off_is_bit_identical_even_with_shadows_on_the_wire():
    """A shadow measurement with ssb_rows_enabled=False changes NOTHING.

    Not "nearly nothing": the same bytes. The comparison includes a frame that
    carries a shadow well inside the horizon, so it fails if any part of the
    build reads obs.shadows outside the flag.
    """
    kin_a, kin_b = ssb_kin(), ssb_kin()
    a = _drive(make_builder(kin=kin_a, cp_jacobian=True), kin_a, [], n_frames=3)
    b = _drive(make_builder(kin=kin_b, cp_jacobian=True), kin_b,
               [shadow_at(kin_b, gap=0.02)], n_frames=3)
    assert np.array_equal(a.A, b.A)
    assert np.array_equal(a.h_bar, b.h_bar)
    assert np.array_equal(a.G, b.G)
    assert np.array_equal(a.group, b.group)
    assert np.array_equal(a.v_obs, b.v_obs)
    assert a.links == b.links
    assert a.d_obs_min == b.d_obs_min
    assert b.n_ssb == 0 and not np.isfinite(b.d_ssb_min)


def test_flag_on_with_no_shadow_is_bit_identical_too():
    """Arming the family costs nothing until something is actually shadowed."""
    kin_a, kin_b = ssb_kin(), ssb_kin()
    a = _drive(make_builder(kin=kin_a, cp_jacobian=True), kin_a, [], n_frames=3)
    bb, _ = ssb_builder()
    b = _drive(bb, kin_b, [], n_frames=3)
    assert np.array_equal(a.A, b.A) and np.array_equal(a.G, b.G)
    assert np.array_equal(a.h_bar, b.h_bar)
    assert b.n_ssb == 0


# ─────────────────────────────────────────────────────────────────────────────
#  The row itself
# ─────────────────────────────────────────────────────────────────────────────

def test_a_shadow_inside_the_horizon_adds_exactly_one_ssb_row():
    b, kin = ssb_builder()
    con = _drive(b, kin, [shadow_at(kin, gap=0.10)])
    assert con.n_ssb == 1
    idx = np.flatnonzero(con.group == G_SSB)
    assert idx.size == 1
    i = int(idx[0])
    assert con.links[i] == 'ssb:fr3_link5#0'
    # h = gap - ssb_d_safe = 0.10 - 0.05
    assert con.h_bar[i] == pytest.approx(0.05, abs=2e-9)
    # The row is OUTSIDE the obstacle family, so d_obs_min is untouched: the
    # commander's phase governor reads that number off /cbf_status and must not
    # see a lower-trust measurement through it.
    assert con.d_obs_min == pytest.approx(0.25)
    assert con.d_ssb_min == pytest.approx(0.10, abs=2e-9)


def test_the_capsule_radius_is_recovered_from_the_wire():
    """|pr - ph| - d is the radius perception subtracted; the row re-uses it.

    Without this the row would be built on the CENTRE distance and would sit
    one capsule radius further away than the measurement said — a silent
    relaxation of exactly the quantity the barrier is about.
    """
    b, kin = ssb_builder()
    con = _drive(b, kin, [shadow_at(kin, gap=0.08, radius=0.06)])
    i = int(np.flatnonzero(con.group == G_SSB)[0])
    assert con.h_bar[i] == pytest.approx(0.08 - 0.05, abs=2e-9)
    assert con.d_ssb_min == pytest.approx(0.08, abs=2e-9)


def test_the_row_can_only_be_relaxed_by_its_own_slack():
    b, kin = ssb_builder()
    con = _drive(b, kin, [shadow_at(kin, gap=0.10)])
    i = int(np.flatnonzero(con.group == G_SSB)[0])
    slack_cols = con.G[i, NV:]
    assert slack_cols[G_SSB] == -1.0
    assert np.count_nonzero(slack_cols) == 1
    # and no OTHER row borrows the ssb column
    others = np.flatnonzero(con.group != G_SSB)
    assert np.count_nonzero(con.G[others, NV + G_SSB]) == 0


def test_the_row_direction_points_away_from_the_shadow():
    """a = n̂ᵀJ with n̂ from the shadow TO the control point.

    aᵀq̈ ≥ b then reads "accelerate away", the sign every other family in this
    QP uses. A flipped normal would build a constraint that drives the arm INTO
    the surface while reporting a satisfied row.
    """
    b, kin = ssb_builder()
    cp = kin.cp_positions['fr3_link5#0']
    con = _drive(b, kin, [shadow_at(kin, gap=0.12)])
    i = int(np.flatnonzero(con.group == G_SSB)[0])
    Jp, _ = kin.point_jacobian(1, cp)
    # shadow_at's default direction is -y, so the shadow is 12 cm in -y and
    # n̂ = (p_j − x)/‖·‖ points back at +y.
    n_hat = np.array([0.0, 1.0, 0.0])
    assert np.allclose(con.A[i], n_hat @ Jp, atol=1e-12)
    # and the whole row, not just its direction: ḣ = aᵀq̇ must be the speed
    # along that normal, which is what makes h_bar and A dimensionally one row.
    assert con.v_obs[i] == 0.0


# ─────────────────────────────────────────────────────────────────────────────
#  Gating
# ─────────────────────────────────────────────────────────────────────────────

def test_a_shadow_outside_the_horizon_builds_no_row_but_is_still_reported():
    b, kin = ssb_builder()
    con = _drive(b, kin, [shadow_at(kin, gap=0.40)])
    assert con.n_ssb == 0
    assert np.count_nonzero(con.group == G_SSB) == 0
    # The MEASUREMENT is still logged, so "nothing was close" and "the family
    # is off" stay distinguishable on a bag.
    assert con.d_ssb_min == pytest.approx(0.40, abs=2e-9)


def test_the_release_hysteresis_holds_an_engaged_row_past_the_horizon():
    """horizon 0.20, release 1.5 ⇒ engage below 0.20, drop above 0.30.

    A row that appears and disappears on alternate rebuilds changes n_c, and a
    changed n_c throws away OSQP's warm start every time — while something is
    close, which is the worst moment for it.
    """
    b, kin = ssb_builder()
    got = [_drive(b, kin, [shadow_at(kin, gap=g)]).n_ssb
           for g in (0.18, 0.25, 0.29, 0.31)]
    assert got == [1, 1, 1, 0]


def test_the_row_cap_is_respected_and_keeps_the_closest():
    b, kin = ssb_builder(ssb_max_rows=2)
    labels = ['fr3_link5#0', 'fr3_link5#1', 'fr3_link5#2',
              'fr3_link6#0', 'fr3_link6#1', 'fr3_link6#2']
    sh = [shadow_at(kin, label=l, gap=0.02 * (i + 1))
          for i, l in enumerate(labels)]
    con = _drive(b, kin, sh)
    assert con.n_ssb == 2
    hs = sorted(float(con.h_bar[i]) for i in np.flatnonzero(con.group == G_SSB))
    assert hs == pytest.approx([-0.03, -0.01], abs=2e-9)
    # the diagnostic reports the CLOSEST measurement, capped or not
    assert con.d_ssb_min == pytest.approx(0.02, abs=2e-9)


def test_the_hold_keeps_a_flickering_shadow_and_erodes_it():
    """A measurement missing for one frame keeps its row, tightened by age.

    The erosion is what makes the hold conservative rather than a stale
    certainty: h must DECREASE while the row is held, never sit still.
    """
    b, kin = ssb_builder(ssb_hold_s=0.10, ssb_inflate_speed_mps=0.20)
    dt, h = 0.02, []
    for k in range(7):
        t = k * dt
        sh = [shadow_at(kin, gap=0.10)] if k == 0 else []
        con = b.build(make_js(stamp=t),
                      make_obs([make_obstacle()], stamp=t, shadows=sh), t)
        idx = np.flatnonzero(con.group == G_SSB)
        h.append(float(con.h_bar[idx[0]]) if idx.size else None)
    # frames 0..5 are within the 0.10 s hold; frame 6 (t = 0.12) is not
    assert all(x is not None for x in h[:6]) and h[6] is None
    assert h[0] == pytest.approx(0.05, abs=2e-9)
    # strictly tightening: 0.20 m/s * 0.02 s = 4 mm per frame
    assert np.all(np.diff(np.array(h[:6])) < -3.9e-3)
    assert h[5] == pytest.approx(0.05 - 0.20 * 5 * dt, abs=2e-9)


def test_hold_zero_drops_the_row_the_moment_the_measurement_does():
    b, kin = ssb_builder()
    assert _drive(b, kin, [shadow_at(kin, gap=0.10)]).n_ssb == 1
    con = b.build(make_js(stamp=0.02),
                  make_obs([make_obstacle()], stamp=0.02, shadows=[]), 0.02)
    assert con.n_ssb == 0


def test_a_shadow_on_an_unknown_control_point_is_dropped_not_crashed():
    b, kin = ssb_builder()
    bad = make_shadow(cp_label='no_such_link#9', link='no_such_link', d=0.05,
                      pr=(0.5, 0.0, 0.5), ph=(0.5, -0.05, 0.5))
    con = _drive(b, kin, [bad])
    assert con.n_ssb == 0
    assert np.count_nonzero(con.group == G_SSB) == 0


def test_a_clock_that_went_backwards_releases_the_hold():
    """A bag jump must not leave a row standing on a measurement from the future.

    Holding forward is conservative; holding BACKWARD is unexplainable, so the
    held entry is released instead of carried with a negative age — which would
    UN-erode it, i.e. relax the row.
    """
    b, kin = ssb_builder(ssb_hold_s=0.5, ssb_inflate_speed_mps=0.2)
    con = b.build(make_js(stamp=10.0),
                  make_obs([make_obstacle()], stamp=10.0,
                           shadows=[shadow_at(kin, gap=0.10)]), 10.0)
    assert con.n_ssb == 1
    con = b.build(make_js(stamp=1.0),
                  make_obs([make_obstacle()], stamp=1.0, shadows=[]), 1.0)
    assert con.n_ssb == 0


def test_a_shadow_with_no_leverage_is_dropped():
    """cbf_min_leverage gates SSB rows exactly as it gates obstacle rows.

    ‖n̂ᵀJ‖ below the gate means no q̈ can move the barrier at a useful rate, so
    the row could only ever be paid for in slack. The one case where this is
    EXACT rather than merely small is proved in
    test_self_shadow_derivatives.py: a region owned by a segment's own start
    link gives dh/dq ≡ 0, because control points interpolate frame ORIGINS and
    a revolute joint's child origin is rigid in its parent frame.

    Here the kinematics is given a genuinely dead z axis at one control point,
    and the shadow is placed along it — the same branch, reached the way the
    robot reaches it, and with the OBSTACLE row left untouched so the gate's
    effect on the SSB family is unambiguous.
    """
    from _cbf_builder_harness import _Kin

    class _DeadZ(_Kin):
        """Jp with a zero third row: no joint can move this point in z."""

        def point_jacobian(self, fid, p_world):
            Jp, Jpd = super().point_jacobian(fid, p_world)
            Jp = Jp.copy()
            Jp[2] = 0.0
            return Jp, Jpd

    kin = _DeadZ()
    kin.cp_positions.update({k: v.copy() for k, v in CP_XYZ.items()})
    b = make_builder(kin=kin, cp_jacobian=True, **SSB_ON)
    # along z: n̂ᵀJ = 0 exactly
    dead = _drive(b, kin, [shadow_at(kin, gap=0.10, direction=(0.0, 0.0, 1.0))])
    assert dead.n_ssb == 0
    assert b._ssb_rows.diag['leverage'] == 1
    # the measurement is still reported, so the drop is visible on a bag rather
    # than looking like "nothing was shadowed"
    assert dead.d_ssb_min == pytest.approx(0.10, abs=2e-9)
    # and the SAME shadow along a live axis DOES produce a row, so what the
    # gate rejected was the leverage and not the shadow
    live = _drive(b, kin, [shadow_at(kin, gap=0.10, direction=(0.0, -1.0, 0.0))])
    assert live.n_ssb == 1


# ─────────────────────────────────────────────────────────────────────────────
#  Solved QP
# ─────────────────────────────────────────────────────────────────────────────

def _solve(con, *, rho_ssb, qdd_nom, slack_max=1e6):
    """Minimal dense QP: ½‖q̈ − q̈_nom‖² + ½ρ s², subject to the snapshot's rows.

    Not the node's OSQP setup — that needs a live node — but the same
    constraint matrices, the same slack columns and the same per-family price,
    which is what these two tests are about. The RHS is the k0 term alone,
    which is exact here because the harness drives q̇ = 0.
    """
    import qpsolvers
    n = NV + N_SLACK
    P_ = np.eye(n)
    for g in range(N_SLACK):
        P_[NV + g, NV + g] = rho_ssb if g == G_SSB else 1e4
    q_ = np.zeros(n)
    q_[:NV] = -qdd_nom
    b = -25.0 * con.h_bar
    lb = np.concatenate([-ACC, np.zeros(N_SLACK)])
    ub = np.concatenate([ACC, np.full(N_SLACK, slack_max)])
    h_qp = -b                       # the node's OSQP upper bound, G x <= h_qp
    x = qpsolvers.solve_qp(P_, q_, con.G, h_qp, lb=lb, ub=ub, solver='osqp')
    assert x is not None, 'the reference QP was infeasible'
    return x[:NV], x[NV:], b, h_qp


def test_an_ssb_row_actually_binds_and_spends_its_own_slack():
    """The row changes the command, and the slack it buys is the ssb one.

    A row that is built but never binds is a decoration, not a constraint.
    Driven with a nominal acceleration straight INTO the shadow and h already
    negative, so the row must either bend the command or be paid for.
    """
    b, kin = ssb_builder()
    con = b.build(make_js(stamp=0.0),
                  make_obs([make_obstacle(d=1.0)], stamp=0.0,
                           shadows=[shadow_at(kin, gap=0.03)]), 0.0)
    i = int(np.flatnonzero(con.group == G_SSB)[0])
    assert con.h_bar[i] < 0.0
    qdd_nom = -3.0 * con.A[i] / np.linalg.norm(con.A[i])
    qdd, slack, rhs, h_qp = _solve(con, rho_ssb=400.0, qdd_nom=qdd_nom)
    # the command was bent away from the nominal along the row's own direction
    assert float(con.A[i] @ qdd) > float(con.A[i] @ qdd_nom) + 1e-3
    # whatever relief it still needed came from the ssb slack alone
    assert np.all(np.delete(slack, G_SSB) <= 1e-6)
    # and the diagnostic attributes it to the ssb family, in the ssb column
    raw, rel = family_violations(con.G, h_qp, qdd, slack, con.group)
    assert raw.shape == (len(FAMILIES_ALL),)
    assert rel[G_SSB] <= 1e-4
    assert raw[G_OBS] <= 1e-4


def test_a_hard_ssb_row_is_hard_and_says_so():
    """ssb_slack_max = 0 removes the family's relief entirely.

    A legitimate experiment, and the honest way to run it: the row is then
    either satisfiable by some q̈ inside the box, or the QP is infeasible and
    the node's fallback ladder reports that rather than absorbing it. This test
    asserts the MECHANISM — it is not a claim that the hard setting is safe.
    """
    b, kin = ssb_builder()
    con = b.build(make_js(stamp=0.0),
                  make_obs([make_obstacle(d=1.0)], stamp=0.0,
                           shadows=[shadow_at(kin, gap=0.03)]), 0.0)
    i = int(np.flatnonzero(con.group == G_SSB)[0])
    qdd_nom = -3.0 * con.A[i] / np.linalg.norm(con.A[i])
    qdd, slack, rhs, _h = _solve(con, rho_ssb=400.0, qdd_nom=qdd_nom,
                                 slack_max=0.0)
    assert slack[G_SSB] == pytest.approx(0.0, abs=1e-7)
    # satisfied without any relief, to solver tolerance
    assert float(con.A[i] @ qdd) >= float(rhs[i]) - 1e-4


def test_a_capped_shadow_row_is_announced_not_merely_counted():
    """Dropping a candidate safety row must be said out loud.

    `ShadowRowBuilder` discards candidates three ways — the `max_rows` cap, the
    `cbf_min_leverage` gate, and a control point it cannot place — and
    `/cbf_status`'s `n_ssb` reports only what SURVIVED. Without a log line, a
    cap that saturates on every tick is indistinguishable from a quiet scene,
    which is precisely the silent discarding of a safety constraint that the
    design rules forbid. So: more shadows than the cap, and the builder has to
    say how many it threw away.
    """
    b, kin = ssb_builder(ssb_max_rows=2)
    shadows = [shadow_at(kin, lbl, gap)
               for lbl, gap in (('fr3_link4#0', 0.06), ('fr3_link5#0', 0.07),
                                ('fr3_link6#0', 0.08), ('fr3_link6#1', 0.09))]
    con = _drive(b, kin, shadows)
    assert con is not None
    assert con.n_ssb == 2, con.n_ssb                 # the cap held
    said = b.test_log.said('self-shadow:')
    assert said, 'the cap dropped 2 rows and said nothing'
    assert 'capped 2' in said[0], said[0]
    assert '2 kept' in said[0], said[0]


def test_nothing_is_announced_when_no_row_is_dropped():
    """The warning must not cry wolf, or it will be filtered out and ignored."""
    b, kin = ssb_builder(ssb_max_rows=4)
    con = _drive(b, kin, [shadow_at(kin, 'fr3_link5#0', 0.07)])
    assert con is not None and con.n_ssb == 1
    assert b.test_log.said('self-shadow:') == []
