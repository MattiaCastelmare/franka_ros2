"""SSB barrier derivatives against independent finite differences.

The analytic expressions under test are the ones :func:`shadow_row` returns and
``build_row_rhs`` consumes:

    h   = ||p_j(q) - x(q)|| - rho - r_j - d_safe
    a   = dh/dq = n^T (J_j - J_x)
    hdot  = a qdot
    hddot = a qddot + n^T (Jdot_j - Jdot_x) qdot + ||(I - n n^T) v_rel||^2 / L
          = a qddot + cdot + kappa

Both regimes are exercised, because they differ in exactly the term the task
warns about: a CANDIDATE point is frozen in the world (J_x = 0) and a VOID slab
is carried by the link that casts it (J_x = that link's point Jacobian). The
FR3 has seven revolute joints and nq == nv == 7, so q(t) = q + t qdot is the
exact integral curve of a constant joint velocity and the time differences
below need no Lie-group care.
"""
from pathlib import Path

import numpy as np
import pytest
import yaml

pin = pytest.importorskip('pinocchio')
from franka_experiments.utils.kinematics import CBFKinematics, build_urdf_no_hand
from franka_experiments.utils.self_shadow import shadow_barrier, shadow_row

RHO, R_J, D_SAFE = 0.03, 0.05, 0.10
Q_HOME = np.array([0.0, -0.7, 0.0, -2.35, 0.0, 1.57, 0.7])
EPSILONS = (1e-4, 1e-5, 1e-6)


@pytest.fixture(scope='module')
def geom():
    cfg = yaml.safe_load(
        (Path(__file__).parents[1] / 'config/fr3_complete.yaml').read_text())
    model = pin.buildModelFromUrdf(build_urdf_no_hand())
    assert model.nq == model.nv == 7, 'the time differences below assume R^7'
    kin = CBFKinematics(model)
    kin.configure_control_points(cfg['robot'], cfg['distance'])
    return model, kin, sorted(kin._control_points)


def _place(kin, q, label, owner=None, local=None):
    """(p_j, x, J_j, Jd_j, J_x, Jd_x) at ``q``.

    ``owner``/``local`` None gives the FROZEN regime, with ``x`` supplied by the
    caller instead.
    """
    kin.update(q, np.zeros(7))
    p_j = kin.control_point_position(label)
    J_j, Jd_j = kin.control_point_jacobian(label)
    if owner is None:
        return p_j, None, J_j, Jd_j, None, None
    fid = kin.resolve_frame_id(owner)
    x = np.asarray(kin.data.oMf[fid].act(local), dtype=np.float64)
    J_x, Jd_x = kin.point_jacobian(fid, x)
    return p_j, x, J_j, Jd_j, J_x, Jd_x


def _h_of_q(kin, q, label, x_fixed=None, owner=None, local=None):
    kin.update(q, np.zeros(7))
    p_j = kin.control_point_position(label)
    x = x_fixed if owner is None else np.asarray(
        kin.data.oMf[kin.resolve_frame_id(owner)].act(local), dtype=np.float64)
    return shadow_barrier(p_j, x, rho=RHO, r_j=R_J, d_safe=D_SAFE)[0]


def _cases(model, kin, labels, rng, n=6):
    """Random valid configurations, one frozen and one attached case each."""
    out = []
    for _ in range(n):
        q = np.clip(Q_HOME + rng.uniform(-0.4, 0.4, 7),
                    model.lowerPositionLimit, model.upperPositionLimit)
        qdot = rng.uniform(-0.8, 0.8, 7)
        label = labels[rng.integers(len(labels))]
        kin.update(q, np.zeros(7))
        p_j = kin.control_point_position(label)
        # FROZEN: a shadow return 4-14 cm off the control point.
        d = rng.normal(size=3)
        x = p_j + d / np.linalg.norm(d) * rng.uniform(0.04, 0.14)
        out.append((q, qdot, label, None, None, x))
        # ATTACHED: a void ball carried by a DIFFERENT link.
        owner = 'fr3_link3' if not label.startswith('fr3_link3') else 'fr3_link6'
        local = rng.uniform(-0.08, 0.08, 3)
        out.append((q, qdot, label, owner, local, None))
    return out


def test_the_barrier_gradient_matches_central_differences(geom):
    model, kin, labels = geom
    rng = np.random.default_rng(17)
    worst = 0.0
    for q, _qd, label, owner, local, x in _cases(model, kin, labels, rng):
        _, _, J_j, Jd_j, J_x, Jd_x = _place(kin, q, label, owner, local)
        kin.update(q, np.zeros(7))
        p_j = kin.control_point_position(label)
        xx = x if owner is None else np.asarray(
            kin.data.oMf[kin.resolve_frame_id(owner)].act(local), dtype=np.float64)
        row = shadow_row(p_j, J_j, Jd_j, xx, np.zeros(7),
                         J_x=J_x, Jd_x=Jd_x, rho=RHO, r_j=R_J, d_safe=D_SAFE)
        a = row[0]
        for eps in EPSILONS:
            fd = np.empty(7)
            for j in range(7):
                e = np.zeros(7); e[j] = eps
                fd[j] = (_h_of_q(kin, q + e, label, x, owner, local)
                         - _h_of_q(kin, q - e, label, x, owner, local)) / (2 * eps)
            err = float(np.max(np.abs(a - fd)))
            worst = max(worst, err)
            assert err < 1e-6 + 10 * eps ** 2, (eps, err, label, owner)
    assert worst < 1e-6


def test_hdot_is_the_time_derivative_of_the_barrier(geom):
    model, kin, labels = geom
    rng = np.random.default_rng(23)
    for q, qdot, label, owner, local, x in _cases(model, kin, labels, rng):
        _, _, J_j, Jd_j, J_x, Jd_x = _place(kin, q, label, owner, local)
        kin.update(q, np.zeros(7))
        p_j = kin.control_point_position(label)
        xx = x if owner is None else np.asarray(
            kin.data.oMf[kin.resolve_frame_id(owner)].act(local), dtype=np.float64)
        a = shadow_row(p_j, J_j, Jd_j, xx, qdot, J_x=J_x, Jd_x=Jd_x,
                       rho=RHO, r_j=R_J, d_safe=D_SAFE)[0]
        hdot = float(a @ qdot)
        dt = 1e-6
        fd = (_h_of_q(kin, q + dt * qdot, label, x, owner, local)
              - _h_of_q(kin, q - dt * qdot, label, x, owner, local)) / (2 * dt)
        assert abs(hdot - fd) < 1e-6 * max(1.0, abs(hdot))


def test_the_frozen_and_attached_regimes_disagree_by_exactly_the_region_term(geom):
    """Dropping J_x is not a small error: it is the whole relative-motion term."""
    model, kin, labels = geom
    rng = np.random.default_rng(29)
    seen = 0
    for q, qdot, label, owner, local, x in _cases(model, kin, labels, rng, n=8):
        if owner is None:
            continue
        _, xx, J_j, Jd_j, J_x, Jd_x = _place(kin, q, label, owner, local)
        kin.update(q, np.zeros(7))
        p_j = kin.control_point_position(label)
        full = shadow_row(p_j, J_j, Jd_j, xx, qdot, J_x=J_x, Jd_x=Jd_x,
                          rho=RHO, r_j=R_J, d_safe=D_SAFE)
        naive = shadow_row(p_j, J_j, Jd_j, xx, qdot,
                           rho=RHO, r_j=R_J, d_safe=D_SAFE)
        n_hat = full[4]
        np.testing.assert_allclose(naive[0] - full[0], n_hat @ J_x, atol=1e-12)
        # The true hdot is the one with the region's own motion in it.
        dt = 1e-6
        fd = (_h_of_q(kin, q + dt * qdot, label, None, owner, local)
              - _h_of_q(kin, q - dt * qdot, label, None, owner, local)) / (2 * dt)
        assert abs(float(full[0] @ qdot) - fd) < 1e-6
        seen += 1
    assert seen >= 4


def test_hddot_matches_the_second_time_difference_with_zero_acceleration(geom):
    model, kin, labels = geom
    rng = np.random.default_rng(31)
    for q, qdot, label, owner, local, x in _cases(model, kin, labels, rng):
        kin.update(q, qdot)                      # Jdot needs the real qdot
        p_j = kin.control_point_position(label)
        J_j, Jd_j = kin.control_point_jacobian(label)
        if owner is None:
            xx, J_x, Jd_x = x, None, None
        else:
            fid = kin.resolve_frame_id(owner)
            xx = np.asarray(kin.data.oMf[fid].act(local), dtype=np.float64)
            J_x, Jd_x = kin.point_jacobian(fid, xx)
        row = shadow_row(p_j, J_j, Jd_j, xx, qdot, J_x=J_x, Jd_x=Jd_x,
                         rho=RHO, r_j=R_J, d_safe=D_SAFE, with_kappa=True)
        a, _h, cdot, kappa, _n, _L = row
        pred = cdot + kappa                      # qddot = 0
        dt = 2e-4
        h0 = _h_of_q(kin, q, label, x, owner, local)
        hp = _h_of_q(kin, q + dt * qdot, label, x, owner, local)
        hm = _h_of_q(kin, q - dt * qdot, label, x, owner, local)
        fd = (hp - 2 * h0 + hm) / dt ** 2
        assert abs(pred - fd) < 2e-4 * max(1.0, abs(pred)), (pred, fd, owner)


def test_the_full_predicted_barrier_acceleration_matches(geom):
    """a qddot + cdot + kappa against a second difference along q + t qdot + t^2/2 qddot."""
    model, kin, labels = geom
    rng = np.random.default_rng(37)
    for q, qdot, label, owner, local, x in _cases(model, kin, labels, rng):
        qddot = rng.uniform(-4.0, 4.0, 7)
        kin.update(q, qdot)
        p_j = kin.control_point_position(label)
        J_j, Jd_j = kin.control_point_jacobian(label)
        if owner is None:
            xx, J_x, Jd_x = x, None, None
        else:
            fid = kin.resolve_frame_id(owner)
            xx = np.asarray(kin.data.oMf[fid].act(local), dtype=np.float64)
            J_x, Jd_x = kin.point_jacobian(fid, xx)
        a, _h, cdot, kappa, _n, _L = shadow_row(
            p_j, J_j, Jd_j, xx, qdot, J_x=J_x, Jd_x=Jd_x,
            rho=RHO, r_j=R_J, d_safe=D_SAFE, with_kappa=True)
        pred = float(a @ qddot) + cdot + kappa
        dt = 2e-4
        def h_at(t):
            return _h_of_q(kin, q + t * qdot + 0.5 * t * t * qddot,
                           label, x, owner, local)
        fd = (h_at(dt) - 2 * h_at(0.0) + h_at(-dt)) / dt ** 2
        assert abs(pred - fd) < 3e-4 * max(1.0, abs(pred)), (pred, fd, owner)


def test_the_curvature_term_is_non_negative_and_is_the_relaxing_part(geom):
    model, kin, labels = geom
    rng = np.random.default_rng(41)
    any_pos = False
    for q, qdot, label, owner, local, x in _cases(model, kin, labels, rng):
        kin.update(q, qdot)
        p_j = kin.control_point_position(label)
        J_j, Jd_j = kin.control_point_jacobian(label)
        if owner is None:
            xx, J_x, Jd_x = x, None, None
        else:
            fid = kin.resolve_frame_id(owner)
            xx = np.asarray(kin.data.oMf[fid].act(local), dtype=np.float64)
            J_x, Jd_x = kin.point_jacobian(fid, xx)
        row_on = shadow_row(p_j, J_j, Jd_j, xx, qdot, J_x=J_x, Jd_x=Jd_x,
                            rho=RHO, r_j=R_J, d_safe=D_SAFE, with_kappa=True)
        row_off = shadow_row(p_j, J_j, Jd_j, xx, qdot, J_x=J_x, Jd_x=Jd_x,
                             rho=RHO, r_j=R_J, d_safe=D_SAFE)
        assert row_on[3] >= 0.0
        assert row_off[3] == 0.0
        any_pos |= row_on[3] > 1e-9
    assert any_pos, 'the test never produced tangential motion'


def test_near_coincidence_blows_kappa_up_and_exact_coincidence_is_refused(geom):
    model, kin, labels = geom
    kin.update(Q_HOME, np.ones(7) * 0.5)
    label = labels[0]
    p_j = kin.control_point_position(label)
    J_j, Jd_j = kin.control_point_jacobian(label)
    prev = 0.0
    for L in (1e-2, 1e-3, 1e-4):
        x = p_j + np.array([L, 0.0, 0.0])
        row = shadow_row(p_j, J_j, Jd_j, x, np.ones(7) * 0.5,
                         rho=0.0, r_j=0.0, d_safe=0.0, with_kappa=True)
        assert row is not None
        assert row[3] >= prev          # kappa ~ 1/L
        prev = row[3]
    assert shadow_row(p_j, J_j, Jd_j, p_j.copy(), np.ones(7) * 0.5) is None


def test_a_non_finite_input_is_refused_rather_than_passed_to_the_qp(geom):
    model, kin, labels = geom
    kin.update(Q_HOME, np.zeros(7))
    label = labels[0]
    p_j = kin.control_point_position(label)
    J_j, Jd_j = kin.control_point_jacobian(label)
    bad = np.array([np.nan, 0.0, 0.0])
    assert shadow_row(p_j, J_j, Jd_j, p_j + 0.1 + bad, np.zeros(7)) is None
    Jb = J_j.copy(); Jb[0, 0] = np.inf
    assert shadow_row(p_j, Jb, Jd_j, p_j + np.array([0.1, 0, 0]),
                      np.zeros(7)) is None


def test_a_region_owned_by_a_segments_own_start_link_has_no_leverage(geom):
    """Why adjacent links must be excluded, and it is not a heuristic.

    Every control point is ``(1-t)·o_start + t·o_end`` over two FRAME ORIGINS,
    and a revolute joint's child origin is rigid in its parent's frame.  So a
    control point of segment ``k`` is rigid with respect to that segment's
    START link, and a shadow region carried by that link gives
    ``dh/dq ≡ 0``: the row is unactuated.  Emitting it would be at best
    vacuous and at worst a permanently violated constraint the QP can only pay
    slack for — which is precisely the "artificial self-collision constraint"
    the formulation has to avoid.  ``cbf_min_leverage`` is the gate that stops
    it in the builder; this test is the reason the gate is not optional.
    """
    model, kin, labels = geom
    rng = np.random.default_rng(53)
    pairs = {'fr3_link4': 'fr3_link3', 'fr3_link5': 'fr3_link4',
             'fr3_link6': 'fr3_link5', 'fr3_link7': 'fr3_link6',
             'fr3_link8': 'fr3_link7'}
    for label in labels:
        owner = pairs[label.partition('#')[0]]
        q = np.clip(Q_HOME + rng.uniform(-0.4, 0.4, 7),
                    model.lowerPositionLimit, model.upperPositionLimit)
        local = rng.uniform(-0.08, 0.08, 3)
        kin.update(q, np.zeros(7))
        fid = kin.resolve_frame_id(owner)
        x = np.asarray(kin.data.oMf[fid].act(local), dtype=np.float64)
        p_j = kin.control_point_position(label)
        J_j, Jd_j = kin.control_point_jacobian(label)
        J_x, Jd_x = kin.point_jacobian(fid, x)
        a = shadow_row(p_j, J_j, Jd_j, x, np.zeros(7), J_x=J_x, Jd_x=Jd_x,
                       rho=RHO, r_j=R_J, d_safe=D_SAFE)[0]
        assert float(np.linalg.norm(a)) < 1e-12, (label, owner, a)
        # ... and h really is constant along q, not merely stationary.
        for _ in range(3):
            dq = rng.uniform(-0.2, 0.2, 7)
            assert abs(_h_of_q(kin, q + dq, label, None, owner, local)
                       - _h_of_q(kin, q, label, None, owner, local)) < 1e-12
