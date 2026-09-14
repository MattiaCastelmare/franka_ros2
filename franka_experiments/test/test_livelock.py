"""Phase 3c/3d: the livelock detector, the nullspace escape, and the proof
that no row family can freeze every direction at once.

Pure numpy for the detector and the builder half; the QP half needs OSQP and
skips without it.
"""

import numpy as np
import pytest

from _cbf_builder_harness import (NV, QD, make_builder, make_obstacle, run)
from franka_experiments.utils.livelock import LivelockDetector, ProgressWindow

PR = (0.5, 0.0, 0.5)
PH = (0.5, -0.25, 0.5)


def _det(**kw):
    d = dict(stall_s=1.0, ramp_s=0.3, max_s=2.0, cooldown_s=2.0)
    d.update(kw)
    return LivelockDetector(**d)


def _drive(det, seq, dt=0.01, t0=0.0):
    """seq: list of (duration_s, blocked, moving). Returns the magnitude trace."""
    out, t = [], float(t0)
    for dur, blocked, moving in seq:
        for _ in range(int(round(dur / dt))):
            out.append(det.update(t, blocked=blocked, moving=moving))
            t += dt
    return np.array(out)


# ── The detector ─────────────────────────────────────────────────────────────

def test_nothing_happens_while_the_arm_moves_or_the_qp_is_not_bending():
    assert not _drive(_det(), [(5.0, True, True)]).any()
    assert not _drive(_det(), [(5.0, False, False)]).any()


def test_the_escape_starts_after_stall_s_and_ramps_in():
    m = _drive(_det(), [(3.0, True, False)])
    t = np.arange(m.size) * 0.01
    assert not m[t < 1.0].any()
    assert m[(t > 1.0) & (t < 1.3)].max() < 1.0            # still ramping
    assert np.isclose(m[(t > 1.31) & (t < 2.9)].min(), 1.0)  # held at full
    assert np.all(np.diff(m[(t > 1.0) & (t < 1.3)]) >= 0.0)


def test_the_escape_is_bounded_in_duration_and_then_rests():
    det = _det()
    m = _drive(det, [(6.0, True, False)])
    t = np.arange(m.size) * 0.01
    assert m[(t > 1.31) & (t < 2.99)].min() > 0.99
    # max_s at t = 3.0, then the release ramp (ramp_s = 0.3), then nothing.
    assert not m[(t > 3.32) & (t < 5.0)].any()
    assert det.last_reason == 'max duration'
    # after the cooldown it may start again: stall counts from 5.0
    m2 = _drive(det, [(1.5, True, False)], t0=6.0)
    assert m2[-1] > 0.0 and det.n_escapes == 2


def test_the_escape_ends_the_moment_the_task_is_no_longer_blocked():
    det = _det()
    m = _drive(det, [(1.5, True, False), (0.5, False, True)])
    assert m[149] > 0.0
    assert det.last_reason == 'task no longer blocked'
    assert det.state == det.COOLDOWN
    assert m[-1] == 0.0


def test_the_escape_is_released_on_a_ramp_and_never_steps():
    """Ending an escape by dropping the magnitude to zero in one tick is a
    step of the full gain in q̈_nom: the arm feels the end exactly as it feels
    the start. Both edges must be ramps, and neither may move by more than one
    tick's worth of the ramp."""
    for seq in ([(1.5, True, False), (1.0, False, False)],      # released early
                [(6.0, True, False)]):                          # released on max_s
        m = _drive(_det(), seq)
        assert np.max(np.abs(np.diff(m))) <= 0.01 / 0.3 + 1e-9   # dt / ramp_s
        assert m[-1] == 0.0
        # and the release is monotone once it starts
        pk = int(np.argmax(m))
        tail = m[pk:][m[pk:] > 0.0]
        assert np.all(np.diff(tail) <= 1e-12)


def test_the_escape_does_not_end_on_its_own_displacement():
    """The nudge moves the arm (moving=True) while the task is still
    blocked: that is the escape working, not progress — it must continue."""
    det = _det()
    m = _drive(det, [(1.5, True, False), (0.4, True, True)])
    assert m[150:].min() > 0.0
    assert det.escaping


def test_a_short_stall_does_not_trigger():
    det = _det()
    m = _drive(det, [(0.9, True, False), (0.2, True, True), (0.9, True, False)])
    assert not m.any() and det.n_escapes == 0


# ── The builder's escape direction ───────────────────────────────────────────

def _con(flag, **ob):
    b = make_builder(obstacle_velocity_source='tracker', enable_livelock_escape=flag)
    return run(b, [make_obstacle(pr=PR, ph=PH, **ob)], n_frames=3, qdot=0.0)


def test_flag_off_carries_no_direction_and_identical_rows():
    off, on = _con(False), _con(True)
    assert off.livelock_dir is None
    np.testing.assert_array_equal(off.A, on.A)
    np.testing.assert_array_equal(off.h_bar, on.h_bar)
    np.testing.assert_array_equal(off.G, on.G)


def test_direction_is_unit_and_in_the_row_nullspace_for_a_static_obstacle():
    on = _con(True)
    d = on.livelock_dir
    assert d is not None and np.isclose(np.linalg.norm(d), 1.0)
    a = on.A[0]
    assert abs(float(a @ d)) < 1e-9 * max(1.0, np.linalg.norm(a))


def test_direction_is_in_the_row_nullspace_for_a_tracked_obstacle():
    on = _con(True, v=(0.3, 0.6, 0.0), frames_seen=20, cov=np.eye(3) * 1e-4, track_id=2)
    d = on.livelock_dir
    assert d is not None
    assert abs(float(on.A[0] @ d)) < 1e-9


def test_direction_is_deterministic_frame_to_frame():
    a = _con(True).livelock_dir
    b = _con(True).livelock_dir
    np.testing.assert_array_equal(a, b)


# ── 3d: no family can freeze every direction ─────────────────────────────────

def test_fewer_rows_than_joints_always_leave_a_nullspace_escape():
    """Any set of k < NV rows (obstacle, self-collision, singularity — the
    algebra does not care which) leaves an (NV − k)-dimensional nullspace, and
    the projected escape direction is non-zero for a generic lateral seed."""
    rng = np.random.default_rng(0)
    for k in range(1, NV):
        A = rng.normal(size=(k, NV))
        g = rng.normal(size=NV)
        # projection onto the nullspace of A
        P = np.eye(NV) - A.T @ np.linalg.pinv(A @ A.T) @ A
        e = P @ g
        assert np.linalg.norm(e) > 1e-6
        assert np.allclose(A @ e, 0.0, atol=1e-9)


def test_binding_rows_plus_the_escape_bias_still_produce_motion():
    """The freeze: a nominal pointing straight into k binding rows yields
    q̈ ≈ 0. With the nullspace escape added to the target, the QP returns a
    finite command inside the box that moves the arm."""
    osqp = pytest.importorskip('osqp')
    sparse = pytest.importorskip('scipy.sparse')
    from franka_experiments.utils.cbf_qp_assembly import build_osqp_A, build_osqp_bounds
    from franka_experiments.utils.cbf_state_rows import NX, N_SLACK
    QDD = np.array([6.0, 2.585, 3.5, 4.0, 17.0, 5.5, 17.0])
    rng = np.random.default_rng(1)
    Pm = np.eye(NX); Pm[NV:, NV:] *= 1000.0
    P_csc = sparse.csc_matrix(Pm)
    lb = np.concatenate([-QDD, np.zeros(N_SLACK)])
    ub = np.concatenate([QDD, np.full(N_SLACK, 1e6)])
    for k in (1, 3, 6):
        A = rng.normal(size=(k, NV))
        G = np.zeros((k, NX)); G[:, :NV] = -A; G[:, NV] = -1.0
        h = np.zeros(k)                                  # aᵀq̈ + s ≥ 0: binding
        nom = -A.T @ np.ones(k)                          # straight INTO the rows
        Pn = np.eye(NV) - A.T @ np.linalg.pinv(A @ A.T) @ A
        esc = Pn @ rng.normal(size=NV); esc /= np.linalg.norm(esc)

        def solve(target):
            qv = np.zeros(NX); qv[:NV] = -target
            l, u = build_osqp_bounds(G, h, lb, ub)
            prob = osqp.OSQP()
            prob.setup(P=P_csc, q=qv, A=build_osqp_A(G, NV, N_SLACK), l=l, u=u,
                       verbose=False, eps_abs=1e-6, eps_rel=1e-6)
            r = prob.solve()
            assert r.info.status_val == osqp.constant('OSQP_SOLVED')
            return r.x[:NV], float(r.x[NV])

        frozen, _ = solve(nom)
        assert np.linalg.norm(frozen) < 0.05 * np.linalg.norm(nom), 'fixture: should freeze'
        freed, s_obs = solve(nom + 1.5 * esc)
        assert np.all(np.isfinite(freed))
        assert np.all(freed <= QDD + 1e-6) and np.all(freed >= -QDD - 1e-6)
        assert np.linalg.norm(freed) > 1.0
        # rows still satisfied up to the (priced) slack — the escape lives in
        # their nullspace, so it asked for none of it
        assert np.all(A @ freed + s_obs >= -1e-4)
        assert s_obs < 1e-2


# ── The progress window ──────────────────────────────────────────────────────

def test_a_jiggling_arm_shows_no_progress_but_a_drifting_one_does():
    """0.5 rad/s of oscillation at 5 Hz displaces the joints by 16 mrad peak
    to peak — 'moving' by any speed threshold, stuck by displacement. A slow
    0.2 rad/s drift covers 0.1 rad in the same window."""
    w = ProgressWindow(0.5)
    t = np.arange(0, 1.0, 0.01)
    jiggle = [w.push(tt, np.full(NV, 0.008 * np.sin(2 * np.pi * 5 * tt))) for tt in t]
    assert max(jiggle[50:]) < 0.05
    w = ProgressWindow(0.5)
    drift = [w.push(tt, np.full(NV, 0.2 * tt / np.sqrt(NV))) for tt in t]
    assert np.isclose(drift[-1], 0.1, atol=0.01)


def test_the_window_over_reports_progress_before_it_is_full():
    w = ProgressWindow(0.5)
    assert w.push(0.0, np.zeros(NV)) == 0.0
    assert np.isclose(w.push(0.1, np.full(NV, 0.01)), 0.01 * np.sqrt(NV))
