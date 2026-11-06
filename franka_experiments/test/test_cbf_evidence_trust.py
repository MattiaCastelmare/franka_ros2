"""Evidence-conditioned barrier trust: the three mechanisms of 2026-11-06.

One thesis, three flags, each OFF by default: a row should be tightened, held
or relaxed by what the perception chain has actually EARNED on that row,
measured, rather than by a hand-set constant or by a velocity covariance that
does not predict the error that actually hurts. The error that hurts is the
reported DISTANCE being optimistic — measured on `pentagon_wander_close`
(DIAG, ground truth at 1 kHz): on 4.4 % of ticks the filter believed
everything was at or beyond `d_safe` while the arm was truly inside it, and at
the worst instant it read 0.100 m against a true 0.0022 m.

  N1  ``enable_evidence_margin``        — one scalar tightening per row,
      chosen by an evidence class built from ``LinkDistance`` fields plus the
      one thing only a consumer with history knows: has this track stopped
      being UPDATED? (``frames_seen`` counts measurements, so it freezes while
      a track coasts.)
  N2  ``enable_obstacle_memory``        — a confirmed track that was CLOSE and
      then vanished keeps its row at a conservatively propagated position,
      because on a manipulator the body that hides the obstacle is usually the
      robot's own and the self-mask then deletes it from the depth image.
  N3  ``curvature_radius_from_normal``  — ρ_c measured from the rotation of the
      contact normal instead of assumed, so a flat surface gets no κ at all.

Row convention (as in test_cbf_curvature_term): ``build_row_rhs`` returns
``h_qp`` with ``aᵀq̈ + s ≥ −h_qp``, so a row's ``b = −h_qp`` and a LARGER ``b``
is a MORE stringent row.

Pure numpy, no ROS, no Pinocchio.
"""
import numpy as np
import pytest

from _cbf_builder_harness import _Kin, make_builder, make_obstacle, make_js, make_obs, run
from franka_experiments.utils.cbf_qp_assembly import build_row_rhs
from franka_experiments.utils.cbf_state_rows import (
    EV_COASTING, EV_FRESH, EV_UNTRACKED, EV_YOUNG, evidence_class,
    evidence_margin, fitted_rate, in_self_mask, in_self_shadow,
    measured_curvature_radius,
    memory_surface_gap, segment_point_distance)

NV = 7
K0, K1 = 25.0, 10.5
PR = np.array([0.5, 0.0, 0.5])
PH = np.array([0.5, -0.25, 0.5])
DT = 1.0 / 30.0


def _hqp(con, qdot=0.0, k=0):
    """Row ``k``'s RHS as the QP sees it: ``aᵀq̈ + s ≥ −h_qp``.

    A SMALLER h_qp is a more stringent row (b = −h_qp is larger).
    """
    h_qp, _ = build_row_rhs(con, np.full(NV, qdot), np.full(NV, qdot),
                            k0=K0, k1=K1, retreat_horizon=0.15,
                            speed_horizon=0.10, zoned=False)
    return float(np.atleast_1d(h_qp)[k])


# ═══════════════════════════════════════════════════════════════════════════
# Ground rule 1: flags off changes nothing, bit for bit
# ═══════════════════════════════════════════════════════════════════════════

def test_all_three_flags_off_is_bit_identical():
    """The three mechanisms must not exist unless asked for."""
    ob = make_obstacle(d=0.20, pr=PR, ph=PH, v=(0.0, 0.3, 0.0),
                       frames_seen=12, track_id=7, cp_label='fr3_link5#0')
    base = run(make_builder(), [ob], n_frames=4)
    for over in (dict(enable_evidence_margin=False),
                 dict(enable_obstacle_memory=False),
                 dict(curvature_radius_from_normal=True),  # inert without κ
                 ):
        got = run(make_builder(**over), [ob], n_frames=4)
        assert np.array_equal(got.h_bar, base.h_bar), over
        assert np.array_equal(got.A, base.A), over
        assert np.array_equal(_hqp(base), _hqp(got)), over


def test_flags_off_leave_the_diagnostics_at_zero():
    b = make_builder()
    run(b, [make_obstacle(d=0.20, pr=PR, ph=PH, track_id=3, frames_seen=20,
                          cp_label='fr3_link5#0')], n_frames=3)
    assert b.diag_ev_margin == 0.0
    assert b.diag_ev_class == ''
    assert b.diag_mem_rows == 0
    assert b.diag_mem_held == 0
    assert b.diag_rho_n == 0


# ═══════════════════════════════════════════════════════════════════════════
# N1 — evidence class and margin
# ═══════════════════════════════════════════════════════════════════════════

def test_evidence_class_reads_the_wire_plus_history():
    assert evidence_class(0, 0, min_frames=8, stale=False) == EV_UNTRACKED
    assert evidence_class(0, 99, min_frames=8, stale=True) == EV_UNTRACKED
    assert evidence_class(5, 3, min_frames=8, stale=False) == EV_YOUNG
    assert evidence_class(5, 20, min_frames=8, stale=False) == EV_FRESH
    # Staleness outranks a long history: the count is large precisely BECAUSE
    # the track was measured a lot in the past, which says nothing about now.
    assert evidence_class(5, 999, min_frames=8, stale=True) == EV_COASTING


def test_evidence_margin_can_only_tighten():
    m = {EV_FRESH: 0.01, EV_COASTING: 0.08}
    assert evidence_margin(EV_FRESH, m) == 0.01
    assert evidence_margin(EV_COASTING, m) == 0.08
    assert evidence_margin(EV_YOUNG, m) == 0.0          # unknown class
    assert evidence_margin(EV_FRESH, {EV_FRESH: -0.5}) == 0.0   # negative
    assert evidence_margin(EV_FRESH, {EV_FRESH: float('nan')}) == 0.0


def test_evidence_margin_tightens_h_by_exactly_the_class_value():
    ob = make_obstacle(d=0.20, pr=PR, ph=PH, track_id=4, frames_seen=20,
                       cp_label='fr3_link5#0')
    base = run(make_builder(), [ob], n_frames=1)
    got = run(make_builder(enable_evidence_margin=True,
                           evidence_margin_fresh_m=0.03,
                           obstacle_velocity_min_frames=8), [ob], n_frames=1)
    assert got.h_bar[0] == pytest.approx(base.h_bar[0] - 0.03, abs=1e-12)


def test_an_untracked_row_gets_the_untracked_margin():
    ob = make_obstacle(d=0.20, pr=PR, ph=PH, track_id=0, cp_label='fr3_link5#0')
    base = run(make_builder(), [ob], n_frames=1)
    got = run(make_builder(enable_evidence_margin=True,
                           evidence_margin_untracked_m=0.02,
                           evidence_margin_fresh_m=0.0), [ob], n_frames=1)
    assert got.h_bar[0] == pytest.approx(base.h_bar[0] - 0.02, abs=1e-12)


def test_a_track_that_stops_being_updated_becomes_coasting():
    """The regime the whole mechanism exists for.

    frames_seen frozen while the track is still reported: no new measurement
    stands behind this row, and that is invisible to any single frame.
    """
    b = make_builder(enable_evidence_margin=True,
                     evidence_margin_fresh_m=0.0,
                     evidence_margin_coasting_m=0.08,
                     evidence_coast_grace_s=0.07,
                     obstacle_velocity_min_frames=8)
    ob = lambda k: [make_obstacle(d=0.20, pr=PR, ph=PH, track_id=9,
                                  frames_seen=20,   # FROZEN
                                  cp_label='fr3_link5#0')]
    # First frame establishes the count; by ~3 frames (0.1 s > grace) the row
    # is coasting and carries the tail margin.
    run(b, ob, n_frames=1, dt=DT)
    assert b.diag_ev_class in ('', EV_FRESH)
    con = run(b, ob, n_frames=4, dt=DT)
    assert b.diag_ev_class == EV_COASTING
    assert b.diag_ev_margin == pytest.approx(0.08)
    assert con.h_bar[0] == pytest.approx(0.20 - 0.15 - 0.08, abs=1e-9)


def test_a_track_still_being_measured_never_becomes_coasting():
    b = make_builder(enable_evidence_margin=True,
                     evidence_margin_fresh_m=0.01,
                     evidence_margin_coasting_m=0.08,
                     obstacle_velocity_min_frames=8)
    run(b, lambda k: [make_obstacle(d=0.20, pr=PR, ph=PH, track_id=9,
                                    frames_seen=20 + k,     # growing
                                    cp_label='fr3_link5#0')],
        n_frames=8, dt=DT)
    assert b.diag_ev_class == EV_FRESH
    assert b.diag_ev_margin == pytest.approx(0.01)


def test_the_evidence_margin_makes_the_row_more_stringent():
    ob = make_obstacle(d=0.20, pr=PR, ph=PH, track_id=4, frames_seen=20,
                       cp_label='fr3_link5#0')
    off = _hqp(run(make_builder(), [ob]))
    on = _hqp(run(make_builder(enable_evidence_margin=True,
                               evidence_margin_fresh_m=0.03,
                               obstacle_velocity_min_frames=8), [ob]))
    # b = −h_qp, so a more stringent row has a SMALLER h_qp, by exactly k0·m.
    assert on < off
    assert off - on == pytest.approx(K0 * 0.03, rel=1e-9)


# ═══════════════════════════════════════════════════════════════════════════
# N3 — curvature radius measured from the normal's rotation
# ═══════════════════════════════════════════════════════════════════════════

def test_fitted_rate_recovers_a_known_slope():
    w = 2.5
    samples = [(0.01 * k, np.array([w * 0.01 * k, 0.0, 0.0])) for k in range(8)]
    assert fitted_rate(samples) == pytest.approx(w, rel=1e-9)


def test_fitted_rate_is_zero_without_enough_evidence():
    assert fitted_rate([]) == 0.0
    assert fitted_rate([(0.0, np.zeros(3)), (0.1, np.ones(3))]) == 0.0   # 2 pts
    assert fitted_rate([(0.0, np.zeros(3))] * 5) == 0.0                  # no span


def test_measured_radius_is_v_perp_over_the_rate():
    # 0.3 m/s perpendicular, normal turning at 1.0 rad/s, no floor -> 0.3 m
    assert measured_curvature_radius(0.3, 1.0, rate_floor=0.0) == pytest.approx(0.3)


def test_a_flat_surface_measures_an_infinite_radius():
    """The failure this mechanism exists to remove.

    A desk or a wall produces no rotation of the contact normal, so the
    estimate diverges and κ vanishes — instead of the curvature of a 6 cm ball
    being attributed to the furniture.
    """
    assert measured_curvature_radius(0.3, 0.0, rate_floor=0.0) == float('inf')
    assert measured_curvature_radius(0.3, 0.2, rate_floor=0.30) == float('inf')


def test_the_rate_floor_errs_toward_a_bigger_radius():
    """Noise inflates the measured rate, which would inflate κ."""
    raw = measured_curvature_radius(0.3, 1.0, rate_floor=0.0)
    floored = measured_curvature_radius(0.3, 1.0, rate_floor=0.3)
    assert floored > raw                      # bigger ρ_c => smaller κ => safer


def test_measured_radius_degenerates_safely():
    assert measured_curvature_radius(0.0, 1.0, rate_floor=0.0) == float('inf')
    assert measured_curvature_radius(float('nan'), 1.0, rate_floor=0.0) == float('inf')
    assert measured_curvature_radius(0.3, float('nan'), rate_floor=0.0) == float('inf')


def test_measured_radius_only_ever_shrinks_kappa_in_the_builder():
    """ρ_c = max(constant, measured), so the row is never relaxed further.

    A STATIC obstacle produces no normal rotation at all, so the measured
    radius is infinite and the κ the constant-radius arm would have granted is
    withdrawn entirely.
    """
    kw = dict(enable_curvature_term=True, enable_vobs_in_hdot=True,
              curvature_obstacle_radius=0.06, curvature_kappa_max=2.0,
              obstacle_velocity_min_frames=3)
    # A tangentially moving arm against a STATIC obstacle: v_perp is nonzero
    # (so the constant-radius arm grants a κ) but the normal does not turn in
    # the harness' fixed geometry, so the measurement says "flat".
    ob = make_obstacle(d=0.20, pr=PR, ph=PH, v=(0.0, 0.0, 0.0),
                       frames_seen=20, track_id=5, cp_label='fr3_link5#0')
    con_const = run(make_builder(**kw), [ob], n_frames=6, qdot=0.4, dt=DT)
    con_meas = run(make_builder(curvature_radius_from_normal=True, **kw),
                   [ob], n_frames=6, qdot=0.4, dt=DT)
    assert con_const.kappa is not None and con_meas.kappa is not None
    assert con_const.kappa[0] > 0.0                  # the constant granted one
    assert con_meas.kappa[0] == 0.0                  # the measurement refused
    # κ sits on the RHS, so withdrawing it lowers h_qp: a tighter row.
    assert _hqp(con_meas, qdot=0.4) < _hqp(con_const, qdot=0.4)


# ═══════════════════════════════════════════════════════════════════════════
# N2 — occlusion-aware obstacle memory
# ═══════════════════════════════════════════════════════════════════════════

def test_memory_surface_gap_propagates_and_erodes():
    cp = np.array([0.0, 0.0, 0.0])
    p = np.array([0.5, 0.0, 0.0])
    v = np.array([-0.2, 0.0, 0.0])          # approaching at 0.2 m/s
    gap, n, d_ctr = memory_surface_gap(cp, p, v, offset=0.06, age=0.5,
                                       inflate_mps=0.0)
    assert d_ctr == pytest.approx(0.4)      # 0.5 − 0.2·0.5
    assert gap == pytest.approx(0.34)       # minus the carried offset
    assert n == pytest.approx(np.array([-1.0, 0.0, 0.0]))


def test_a_longer_hold_is_strictly_more_conservative():
    cp, p, v = np.zeros(3), np.array([0.5, 0.0, 0.0]), np.zeros(3)
    g0 = memory_surface_gap(cp, p, v, offset=0.0, age=0.0, inflate_mps=0.2)[0]
    g1 = memory_surface_gap(cp, p, v, offset=0.0, age=0.25, inflate_mps=0.2)[0]
    g2 = memory_surface_gap(cp, p, v, offset=0.0, age=0.50, inflate_mps=0.2)[0]
    assert g0 > g1 > g2
    assert g0 - g2 == pytest.approx(0.1)    # 0.2 m/s × 0.5 s


def test_segment_point_distance():
    a, b = np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0])
    assert segment_point_distance(a, b, np.array([0.5, 0.3, 0.0])) == pytest.approx(0.3)
    assert segment_point_distance(a, b, np.array([2.0, 0.0, 0.0])) == pytest.approx(1.0)
    assert segment_point_distance(a, a, np.array([0.0, 2.0, 0.0])) == pytest.approx(2.0)


def test_self_shadow_sees_only_what_is_between_camera_and_point():
    cam = np.array([2.0, 0.0, 1.0])
    p = np.array([0.4, 0.0, 0.5])
    on_ray = cam + 0.5 * (p - cam)                       # halfway along
    assert in_self_shadow(cam, p, [on_ray], 0.08)
    # The same arm point moved sideways hides nothing.
    assert not in_self_shadow(cam, p, [on_ray + np.array([0.0, 0.5, 0.0])], 0.08)
    # Nor does a part of the arm BEHIND the obstacle, or behind the camera.
    assert not in_self_shadow(cam, p, [p + (p - cam)], 0.08)
    assert not in_self_shadow(cam, p, [cam + (cam - p)], 0.08)
    assert not in_self_shadow(cam, p, [], 0.08)


def _mem_builder(**over):
    kin = _Kin()
    kin.cp_positions['fr3_link5'] = PR
    kw = dict(enable_obstacle_memory=True, memory_min_frames=8,
              memory_trigger_gap_m=0.25, memory_hold_s=0.50,
              memory_inflate_speed_mps=0.20)
    kw.update(over)
    return make_builder(kin=kin, cp_jacobian=True, **kw), kin


def test_a_vanished_close_track_keeps_its_row():
    """The near-miss this mechanism exists to prevent.

    Frames 0–3 see a confirmed track 8 cm away; from frame 4 perception reports
    nothing at all, which is what the self-mask does when the hand moves in
    front of the ball. Without the memory there is no obstacle row and the
    barrier is silent.
    """
    b, _ = _mem_builder()
    seen = [make_obstacle(d=0.08, pr=PR, ph=PH, v=(0.0, 0.0, 0.0),
                          frames_seen=20, track_id=11, cp_label='fr3_link5')]
    con = run(b, lambda k: seen if k < 4 else [], n_frames=5, dt=DT)
    assert b.diag_mem_rows == 1
    assert b.diag_mem_held == 1
    assert con is not None and con.h_bar.size == 1
    assert con.links[0].startswith('mem:')

    off = run(make_builder(), lambda k: seen if k < 4 else [], n_frames=5, dt=DT)
    assert off is None or off.h_bar.size == 0        # nothing constrains it


def test_the_memory_expires():
    b, _ = _mem_builder(memory_hold_s=0.10)
    seen = [make_obstacle(d=0.08, pr=PR, ph=PH, frames_seen=20, track_id=11,
                          cp_label='fr3_link5')]
    run(b, lambda k: seen if k < 1 else [], n_frames=8, dt=DT)   # 0.23 s > hold
    assert b.diag_mem_held == 0
    assert b.diag_mem_rows == 0


def test_a_track_seen_again_is_released_not_doubled():
    b, _ = _mem_builder()
    seen = [make_obstacle(d=0.08, pr=PR, ph=PH, frames_seen=20, track_id=11,
                          cp_label='fr3_link5')]
    con = run(b, lambda k: [] if k == 3 else seen, n_frames=6, dt=DT)
    assert b.diag_mem_rows == 0            # live again: no memory row
    assert con.h_bar.size == 1             # and exactly one row, the live one
    assert not con.links[0].startswith('mem:')


def test_only_confirmed_and_close_tracks_are_remembered():
    for over, ob_kw in ((dict(), dict(frames_seen=2)),            # too young
                        (dict(), dict(d=0.60)),                   # too far
                        (dict(), dict(track_id=0, frames_seen=20))):  # no track
        b, _ = _mem_builder(**over)
        kw = dict(d=0.08, pr=PR, ph=PH, frames_seen=20, track_id=11,
                  cp_label='fr3_link5')
        kw.update(ob_kw)
        run(b, lambda k: [make_obstacle(**kw)] if k < 3 else [], n_frames=4, dt=DT)
        assert b.diag_mem_rows == 0, ob_kw
        assert b.diag_mem_held == 0, ob_kw


def test_the_memory_row_tightens_as_it_ages():
    b, _ = _mem_builder()
    seen = [make_obstacle(d=0.08, pr=PR, ph=PH, frames_seen=20, track_id=11,
                          cp_label='fr3_link5')]
    hs = []
    for n in (2, 3, 4, 5):
        bb, _ = _mem_builder()
        con = run(bb, lambda k: seen if k < 1 else [], n_frames=n, dt=DT)
        hs.append(float(con.h_bar[0]))
    assert all(hs[i] > hs[i + 1] for i in range(len(hs) - 1)), hs


def test_self_mask_ignores_depth_order_but_obeys_the_depth_gate():
    """The mask is a dilated SILHOUETTE with a depth gate, not an occluder.

    fr3_complete.yaml says it in as many words: "a hand a metre in front of the
    arm is as invisible as one touching it". So a point inside the halo is
    masked whether the arm is nearer or farther -- and a point inside the halo
    but outside the depth tolerance is KEPT, which is the gate buying the halo
    back. Requiring true occlusion instead explained 0 of 10 close-track
    disappearances on pentagon_wander_close and 0 of 7 on pentagon_sweep.
    """
    cam = np.zeros(3)
    p = np.array([2.0, 0.0, 0.0])
    halo, tol = 0.0278, 0.16

    # Same range, inside the halo: masked. 2 cm lateral at 2 m = 0.010 rad.
    near = np.array([2.0, 0.02, 0.0])
    assert in_self_mask(cam, p, [(near, False)], halo_rad=halo, depth_tol=tol)

    # The arm NEARER than p, within the gate: still masked (no occlusion
    # needed, and in_self_shadow would also say yes here).
    closer = np.array([1.9, 0.01, 0.0])
    assert in_self_mask(cam, p, [(closer, False)], halo_rad=halo, depth_tol=tol)

    # The arm FARTHER than p, within the gate: masked too -- the case
    # in_self_shadow rejects by construction, and the real failure mode.
    farther = np.array([2.1, 0.01, 0.0])
    assert in_self_mask(cam, p, [(farther, False)], halo_rad=halo, depth_tol=tol)
    assert not in_self_shadow(cam, p, [farther], 0.08)

    # Inside the halo but far in depth: the gate keeps it.
    deep = np.array([2.5, 0.02, 0.0])
    assert not in_self_mask(cam, p, [(deep, False)], halo_rad=halo, depth_tol=tol)

    # Outside the halo at the same range: kept. 20 cm at 2 m = 0.100 rad.
    wide = np.array([2.0, 0.20, 0.0])
    assert not in_self_mask(cam, p, [(wide, False)], halo_rad=halo, depth_tol=tol)

    # The EE halo is doubled, exactly as ee_mask_dilate_px doubles it.
    mid = np.array([2.0, 0.08, 0.0])          # 0.040 rad: > halo, < 2*halo
    assert not in_self_mask(cam, p, [(mid, False)], halo_rad=halo, depth_tol=tol)
    assert in_self_mask(cam, p, [(mid, True)], halo_rad=halo, depth_tol=tol)

    assert not in_self_mask(cam, p, [], halo_rad=halo, depth_tol=tol)


def test_the_mask_test_releases_a_memory_the_mask_cannot_explain():
    """With the release test on, only a MASK-ERASED obstacle is remembered.

    The camera sits where the DIAG rig's does. A control point inside the halo
    at the last-seen point's own range explains the disappearance, so the row is
    held; move the arm wide of it and the obstacle most likely left, so the
    memory is released at once instead of constraining a ghost.
    """
    cam = np.array([1.993009, 0.032024, 1.080394])
    kw = dict(memory_require_occlusion=True, memory_camera_x=float(cam[0]),
              memory_camera_y=float(cam[1]), memory_camera_z=float(cam[2]),
              memory_mask_halo_rad=0.0278, memory_mask_depth_tol_m=0.16)
    d = PH - cam
    L = float(np.linalg.norm(d))
    perp = np.cross(d / L, np.array([0.0, 0.0, 1.0]))
    perp /= float(np.linalg.norm(perp))
    inside = PH + 0.02 * perp         # ~0.012 rad off axis, same range
    outside = PH + 0.80 * perp        # far outside the halo
    seen = lambda pr, pos: [make_obstacle(d=0.08, pr=pr, ph=PH, frames_seen=20,
                                          track_id=11, cp_label='fr3_link5')]

    b_hid, kin_hid = _mem_builder(**kw)
    kin_hid.cp_positions['fr3_link5'] = inside
    run(b_hid, lambda k: seen(inside, inside) if k < 3 else [], n_frames=4, dt=DT)
    assert b_hid.diag_mem_rows == 1

    b_gone, kin_gone = _mem_builder(**kw)
    kin_gone.cp_positions['fr3_link5'] = outside
    run(b_gone, lambda k: seen(outside, outside) if k < 3 else [], n_frames=4, dt=DT)
    assert b_gone.diag_mem_rows == 0
    assert b_gone.diag_mem_held == 0


def test_a_memory_row_carries_no_curvature_relaxation():
    """κ relaxes a row and must be paid for with a measurement."""
    b, _ = _mem_builder(enable_curvature_term=True, enable_vobs_in_hdot=True,
                        curvature_obstacle_radius=0.06,
                        obstacle_velocity_min_frames=3)
    seen = [make_obstacle(d=0.08, pr=PR, ph=PH, v=(0.0, 0.3, 0.0),
                          frames_seen=20, track_id=11, cp_label='fr3_link5')]
    con = run(b, lambda k: seen if k < 3 else [], n_frames=4, dt=DT, qdot=0.3)
    assert b.diag_mem_rows == 1
    assert con.kappa is not None
    assert float(con.kappa[0]) == 0.0
