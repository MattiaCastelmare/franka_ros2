"""cbf.enable_curvature_term — the κ = n̂̇ᵀv_rel term of ḧ (Task 2).

The obstacle row enforces ``ḧ + k₁ḣ + k₀h̄ ≥ 0``. Our ``ḧ`` used to be
``aᵀq̈ + n̂ᵀJ̇ₚq̇``, which omits the part produced by the ROTATION of the contact
normal while the robot slides AROUND the obstacle:

    ḧ = aᵀq̈ + n̂ᵀJ̇ₚq̇ + κ,   κ = n̂̇ᵀv_rel = ‖(I − n̂n̂ᵀ)v_rel‖²/ρ_c ≥ 0,

so the row's bound becomes ``aᵀq̈ ≥ −k₁ḣ − k₀h̄ − ċ − κ``: κ enters the RHS
exactly where ``ċ`` does, and because κ ≥ 0 it RELAXES the row.

Row convention (as in test_vobs_in_hdot): ``build_row_rhs`` returns ``h_qp``
with ``aᵀq̈ + s ≥ −h_qp``, so the row's ``b = −h_qp`` and a LARGER ``b`` is a
MORE stringent row. κ must therefore make ``b`` SMALLER by exactly κ.

What is checked here:
  1. flag off → the snapshot carries no κ at all and every array, including
     the QP's RHS, is bit-identical to the pre-curvature build;
  2. the value is the analytic ``v_⊥²/ρ_c`` with ρ_c the centre distance, and
     it agrees with a finite-difference ``n̂̇ᵀv_rel`` on the true geometry;
  3. the evidence gates and the clamps: radial motion, untracked rows, a young
     track, a non-finite velocity, the κ ceiling, a larger assumed obstacle
     radius (= smaller, i.e. safer, κ);
  4. a closed-loop double integrator sliding past a sphere: with κ the point
     deviates LESS from its straight path and still never penetrates.

Pure numpy, no ROS, no Pinocchio.
"""
import numpy as np
import pytest

from _cbf_builder_harness import _Kin, make_builder, make_obstacle, run
from franka_experiments.utils.cbf_qp_assembly import build_row_rhs

NV = 7
K0, K1 = 25.0, 10.5
Q = 0.1          # the harness' joint configuration for every run below
R_OBS = 0.08     # assumed (and, in the analytic tests, true) obstacle radius

# Sphere geometry used throughout: the obstacle CENTRE, the control point, and
# therefore the normal (obstacle → control point) and the centre distance.
C_OBS = np.array([0.5, -0.25, 0.5])
P_CP = np.array([0.5, 0.0, 0.5])
N_HAT = np.array([0.0, 1.0, 0.0])
RHO_C = float(np.linalg.norm(P_CP - C_OBS))          # 0.25 m
PH = C_OBS + R_OBS * N_HAT                            # closest SURFACE point
D_CTR = float(np.linalg.norm(P_CP - PH))              # 0.17 m = ρ_c − r_o


def _Jp(q=Q):
    """The harness' analytic point Jacobian at a uniform configuration."""
    kin = _Kin()
    kin.update(np.full(NV, q), np.zeros(NV))
    return kin.point_jacobian(1, P_CP)[0]


def _qdot_for(v_point, q=Q):
    """A joint velocity whose point velocity is ``v_point`` (least squares)."""
    Jp = _Jp(q)
    qd = np.linalg.lstsq(Jp, np.asarray(v_point, dtype=np.float64), rcond=None)[0]
    assert np.allclose(Jp @ qd, v_point, atol=1e-12)
    return qd


def _obstacle(v=None, frames_seen=5, d=D_CTR - R_OBS, **over):
    return make_obstacle(d=d, pr=P_CP, ph=PH, v=v, frames_seen=frames_seen,
                         track_id=7, cp_label='fr3_link5#0', **over)


def _snap(v_point, v_obs, *, curv, frames_seen=5, q=Q, **over):
    """One rebuild with the curvature flag on/off; returns (snap, qdot)."""
    b = make_builder(enable_curvature_term=curv, enable_vobs_in_hdot=True,
                     **over)
    qd = _qdot_for(v_point, q)
    con = run(b, [_obstacle(v=v_obs, frames_seen=frames_seen)], qdot=qd, q=q)
    return con, qd, b


def _rhs(con, qdot):
    h_qp, _ = build_row_rhs(con, qdot, qdot, k0=K0, k1=K1,
                            retreat_horizon=0.15, speed_horizon=0.10,
                            zoned=False)
    return h_qp


def _kappa_analytic(v_point, v_obs, rho_c=RHO_C, n_hat=N_HAT):
    v_rel = np.asarray(v_point, float) - np.asarray(v_obs, float)
    v_perp = v_rel - n_hat * float(n_hat @ v_rel)
    return float(v_perp @ v_perp) / rho_c


# ── 1. flags off: nothing moves ──────────────────────────────────────────────

def test_with_the_flag_off_the_snapshot_carries_no_curvature_term():
    con, qd, _ = _snap((0.3, 0.0, 0.0), (0.0, 0.0, 0.0), curv=False)
    assert con.kappa is None


def test_flag_off_is_bit_identical_to_the_pre_curvature_build():
    """Every array a tangentially-moving, fully tracked obstacle produces."""
    v_point, v_obs = (0.3, -0.05, 0.1), (0.1, 0.2, 0.0)
    off, qd, _ = _snap(v_point, v_obs, curv=False)
    on, _, _ = _snap(v_point, v_obs, curv=True)
    for field in ('A', 'h_bar', 'jdot_qdot', 'G', 'v_obs', 'group'):
        a, b = getattr(off, field), getattr(on, field)
        assert np.array_equal(a, b), field
    # ... and the RHS differs by EXACTLY κ, nothing else.
    assert np.array_equal(_rhs(off, qd) + on.kappa, _rhs(on, qd))


def test_the_flag_off_rhs_is_byte_identical_when_nothing_is_tracked():
    off, qd, _ = _snap((0.3, 0.0, 0.0), None, curv=False)
    on, _, _ = _snap((0.3, 0.0, 0.0), None, curv=True)
    assert np.array_equal(_rhs(off, qd), _rhs(on, qd))
    assert np.array_equal(on.kappa, np.zeros_like(on.kappa))


# ── 2. the value ─────────────────────────────────────────────────────────────

@pytest.mark.parametrize('v_point,v_obs', [
    ((0.30, 0.00, 0.00), (0.0, 0.0, 0.0)),      # robot slides, obstacle still
    ((0.00, 0.00, 0.00), (0.25, 0.0, 0.0)),     # obstacle slides, robot still
    ((0.20, 0.10, -0.15), (0.05, -0.20, 0.10)),  # both, with normal parts too
])
def test_kappa_is_the_analytic_perpendicular_speed_squared_over_rho_c(
        v_point, v_obs):
    con, qd, _ = _snap(v_point, v_obs, curv=True,
                       curvature_obstacle_radius=R_OBS)
    assert con.kappa.shape == con.h_bar.shape
    assert con.kappa[0] == pytest.approx(_kappa_analytic(v_point, v_obs),
                                         abs=1e-9)


def test_the_normal_component_of_the_relative_velocity_does_not_enter():
    """κ is a function of the PERPENDICULAR part only.

    That is what makes it consistent with ``enable_vobs_in_hdot``, whose clamp
    acts on the NORMAL component: clamping n̂ᵀv_o cannot change κ.
    """
    base = _snap((0.3, 0.0, 0.0), (0.0, 0.0, 0.0), curv=True,
                 curvature_obstacle_radius=R_OBS)[0]
    # +y is the normal here, so this adds 5 m/s of pure closing speed.
    fast_normal = _snap((0.3, -5.0, 0.0), (0.0, 0.0, 0.0), curv=True,
                        curvature_obstacle_radius=R_OBS)[0]
    assert fast_normal.kappa[0] == pytest.approx(base.kappa[0], abs=1e-12)


def test_kappa_matches_a_finite_difference_of_the_true_normal():
    """κ == n̂̇ᵀv_rel on the exact sphere geometry, by central differences."""
    v_point = np.array([0.22, -0.04, 0.13])
    v_obs = np.array([0.07, 0.15, -0.06])
    dt = 1e-6

    def n_hat(t):
        p = P_CP + v_point * t
        c = C_OBS + v_obs * t
        return (p - c) / np.linalg.norm(p - c)

    ndot = (n_hat(dt) - n_hat(-dt)) / (2.0 * dt)
    kappa_fd = float(ndot @ (v_point - v_obs))

    con, _, _ = _snap(v_point, v_obs, curv=True,
                      curvature_obstacle_radius=R_OBS)
    assert con.kappa[0] == pytest.approx(kappa_fd, rel=1e-3, abs=1e-6)


def test_kappa_relaxes_the_row_by_exactly_itself():
    con, qd, _ = _snap((0.3, 0.0, 0.0), (0.0, 0.0, 0.0), curv=True,
                       curvature_obstacle_radius=R_OBS)
    off, qd_off, _ = _snap((0.3, 0.0, 0.0), (0.0, 0.0, 0.0), curv=False)
    b_on, b_off = -_rhs(con, qd), -_rhs(off, qd_off)
    assert b_on[0] == pytest.approx(b_off[0] - con.kappa[0], abs=1e-12)
    assert b_on[0] < b_off[0]       # relaxed, never tightened


# ── 3. gates and clamps ──────────────────────────────────────────────────────

def test_purely_radial_motion_gets_no_curvature_term():
    # ±y is the normal: approach and recession both leave n̂ where it is.
    for v in ((0.0, 0.4, 0.0), (0.0, -0.4, 0.0)):
        con, _, _ = _snap(v, (0.0, 0.0, 0.0), curv=True,
                          curvature_obstacle_radius=R_OBS)
        assert con.kappa[0] == pytest.approx(0.0, abs=1e-12)


def test_a_row_with_no_track_velocity_gets_no_curvature_term():
    con, _, _ = _snap((0.3, 0.0, 0.0), None, curv=True)
    assert con.kappa[0] == 0.0
    assert con.kappa.shape == con.h_bar.shape


def test_a_young_track_gets_no_curvature_term():
    """Same evidence gate as enable_vobs_in_hdot: frames_seen ≥ min_frames."""
    young, _, b = _snap((0.3, 0.0, 0.0), (0.0, 0.0, 0.0), curv=True,
                        frames_seen=1, curvature_obstacle_radius=R_OBS)
    assert b._min_frames_eff == 3
    assert young.kappa[0] == 0.0
    grown, _, _ = _snap((0.3, 0.0, 0.0), (0.0, 0.0, 0.0), curv=True,
                        frames_seen=3, curvature_obstacle_radius=R_OBS)
    assert grown.kappa[0] > 0.0


def test_a_non_finite_track_velocity_gets_no_curvature_term():
    con, _, _ = _snap((0.3, 0.0, 0.0), (np.nan, 0.0, 0.0), curv=True,
                      curvature_obstacle_radius=R_OBS)
    assert con.kappa[0] == 0.0


def test_the_clamp_is_respected():
    con, _, _ = _snap((3.0, 0.0, 2.0), (0.0, 0.0, 0.0), curv=True,
                      curvature_obstacle_radius=R_OBS,
                      curvature_kappa_max=0.75)
    raw = _kappa_analytic((3.0, 0.0, 2.0), (0.0, 0.0, 0.0))
    assert raw > 0.75                      # the test would be vacuous otherwise
    assert con.kappa[0] == pytest.approx(0.75, abs=1e-12)


def test_a_larger_assumed_obstacle_radius_is_the_safer_one():
    """ρ_c = d_centre + r_o, so a bigger r_o gives a SMALLER κ."""
    small, _, _ = _snap((0.3, 0.0, 0.0), (0.0, 0.0, 0.0), curv=True,
                        curvature_obstacle_radius=0.02)
    big, _, _ = _snap((0.3, 0.0, 0.0), (0.0, 0.0, 0.0), curv=True,
                      curvature_obstacle_radius=1.00)
    assert big.kappa[0] < small.kappa[0]
    # and each is the analytic value at ITS assumed radius
    for snap, r in ((small, 0.02), (big, 1.00)):
        assert snap.kappa[0] == pytest.approx(
            _kappa_analytic((0.3, 0.0, 0.0), (0.0, 0.0, 0.0),
                            rho_c=D_CTR + r), abs=1e-9)


def test_kappa_is_never_negative_and_always_finite():
    rng = np.random.default_rng(7)
    for _ in range(25):
        v_p = rng.normal(scale=0.4, size=3)
        v_o = rng.normal(scale=0.4, size=3)
        con, _, _ = _snap(v_p, v_o, curv=True, curvature_obstacle_radius=R_OBS)
        assert np.all(np.isfinite(con.kappa))
        assert np.all(con.kappa >= 0.0)
        assert np.all(con.kappa <= 2.0 + 1e-12)      # the shipped ceiling


def test_only_obstacle_rows_carry_a_curvature_term():
    """Joint-limit rows share the array and must stay at exactly zero."""
    b = make_builder(enable_curvature_term=True, enable_vobs_in_hdot=True,
                     curvature_obstacle_radius=R_OBS,
                     joint_limit_rows_enabled=True)
    qd = _qdot_for((0.3, 0.0, 0.0))
    # q close to the (±2.8) limit the harness configures, so the rows exist.
    con = run(b, [_obstacle(v=(0.0, 0.0, 0.0))], qdot=qd, q=2.65)
    from franka_experiments.utils.cbf_state_rows import G_OBS
    n_other = int(np.count_nonzero(con.group != G_OBS))
    assert n_other > 0, 'no non-obstacle row was built — test is vacuous'
    assert np.all(con.kappa[con.group != G_OBS] == 0.0)


def test_the_diagnostic_reports_the_largest_kappa_and_its_row_count():
    b = make_builder(enable_curvature_term=True, enable_vobs_in_hdot=True,
                     curvature_obstacle_radius=R_OBS)
    qd = _qdot_for((0.3, 0.0, 0.0))
    con = run(b, [_obstacle(v=(0.0, 0.0, 0.0))], qdot=qd)
    assert b.diag_kappa == pytest.approx(float(np.max(con.kappa)), abs=1e-12)
    assert b.diag_kappa_n == int(np.count_nonzero(con.kappa > 0.0))

    off = make_builder(enable_curvature_term=False, enable_vobs_in_hdot=True)
    run(off, [_obstacle(v=(0.0, 0.0, 0.0))], qdot=qd)
    assert off.diag_kappa == 0.0 and off.diag_kappa_n == 0


# ── 4. closed loop: does it actually let the point slide around? ─────────────

def _slide_past_sphere(*, with_kappa, k0=K0, k1=K1, d_safe=0.10,
                       r_obs=0.08, miss=0.14, dt=1e-3, n=4000):
    """A 3-D double integrator asked to fly straight past a fixed sphere.

    One HOCBF row, min-norm correction (‖n̂‖ = 1, so the projection is exact):

        n̂ᵀa ≥ b,  b = −k₁ḣ − k₀h̄ − κ_used  ⇒  a = a_nom + max(0, b − n̂ᵀa_nom)n̂

    With ``with_kappa`` the row uses the true κ of this geometry, so the row is
    the exact HOCBF condition on ``h``; without it, the same κ is left out of
    the bound and the filter pays for it in normal acceleration.

    Returns (max lateral deviation [m], min h̄ [m], mean |n̂ᵀ(a−a_nom)|).
    """
    c = np.zeros(3)                       # obstacle centre, static
    # The NOMINAL path misses the centre by `miss` < r_obs + d_safe, so flying
    # it would penetrate the barrier by (r_obs + d_safe − miss) = 4 cm: the row
    # has to bind and move the point sideways. That is the situation κ is about.
    p = np.array([-0.60, miss, 0.0])
    v = np.array([0.45, 0.0, 0.0])        # flying along +x, tangentially
    a_nom_const = np.zeros(3)             # constant velocity is the task
    y0, dev, h_min, push = p[1], 0.0, np.inf, []
    for _ in range(n):
        delta = p - c
        rho = float(np.linalg.norm(delta))
        n_hat = delta / rho
        h = rho - r_obs - d_safe
        hdot = float(n_hat @ v)
        v_perp = v - n_hat * hdot
        kappa = float(v_perp @ v_perp) / rho
        b = -k1 * hdot - k0 * h - (kappa if with_kappa else 0.0)
        lam = max(0.0, b - float(n_hat @ a_nom_const))
        a = a_nom_const + lam * n_hat
        push.append(abs(lam))
        v = v + a * dt
        p = p + v * dt
        dev = max(dev, abs(p[1] - y0))
        h_min = min(h_min, h)
        if p[0] > 0.60:
            break
    return dev, h_min, float(np.mean(push))


def test_sliding_past_a_sphere_deviates_less_with_the_curvature_term():
    dev_on, h_on, push_on = _slide_past_sphere(with_kappa=True)
    dev_off, h_off, push_off = _slide_past_sphere(with_kappa=False)
    # Safety first: neither version penetrates, and the κ one is not worse.
    assert h_off >= -1e-3
    assert h_on >= -1e-3
    # The point of the term: less outward push, less deviation from the path.
    assert push_on < push_off
    assert dev_on < dev_off


def test_the_curvature_term_does_not_buy_a_penetration_on_a_real_approach():
    """Head-on, κ = 0, so the two versions must be identical."""
    def head_on(with_kappa):
        c, p = np.zeros(3), np.array([-0.60, 0.0, 0.0])
        v = np.array([0.45, 0.0, 0.0])
        h_min = np.inf
        for _ in range(4000):
            delta = p - c
            rho = float(np.linalg.norm(delta))
            n_hat = delta / rho
            h = rho - 0.08 - 0.10
            hdot = float(n_hat @ v)
            v_perp = v - n_hat * hdot
            kappa = float(v_perp @ v_perp) / rho
            b = -K1 * hdot - K0 * h - (kappa if with_kappa else 0.0)
            a = max(0.0, b) * n_hat
            v = v + a * 1e-3
            p = p + v * 1e-3
            h_min = min(h_min, h)
        return h_min
    assert head_on(True) == pytest.approx(head_on(False), abs=1e-12)
