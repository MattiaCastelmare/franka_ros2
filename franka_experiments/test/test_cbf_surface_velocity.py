"""The centroid may not outrun the surface (P4), and the place-keyed scene (P3).

WHY THIS FILE EXISTS
--------------------
P1/P2 (``test_cbf_static_scene.py``) withhold a phantom velocity once they can
prove the TRACK is the room. Measured on ``datasets/experiments_irim2.zip`` that
leaves half the problem standing:

* **48-52 %** of nearest-obstacle rows carry no confirmed ``track_id`` at all,
  and the ids that do exist live a **median of 0.27 s** — so a track-keyed
  memory cannot reach most rows however it is tuned (0.2 s windows measured
  WORSE than 1.0 s);
* the number itself is still a CENTROID velocity, and a cluster that merges with
  the wall behind it steps its centroid: ``obstacle_velocity`` reached
  **7.78 m/s** on the wire in a cell whose fastest occupant was a hand.

So the two properties under test here are:

**P3** the same static-scene question keyed on a PLACE (a voxel of the obstacle
surface point), consulted only by rows that have no identity to ask about.

**P4** the obstacle's own SURFACE is the better witness for the one quantity the
row consumes (``n̂ᵀv``), because a surface point cannot teleport: the tracked
centroid may claim at most ``k·v_surface + floor``, in BOTH channels the
velocity reaches the row through. And — the property that makes it safe —
**no estimate means no cap**: every refusal of the estimator (an argmin hop, a
dropped frame, the first frames of a sighting) leaves the tracker believed.

The estimator's SHAPE is load-bearing and was chosen against ground truth: a
median of consecutive differences attenuated a real approach ninefold and would
have capped 69.5 % of genuinely-closing rows below the truth, where the fitted
slope this ships caps 3.4 % — against the tracker's OWN 15.8 %. The table and
the refutation are in ``utils/surface_velocity.py``; the tests here pin the
properties that table depends on, above all that a window must span real time
before it reports anything.

Drives the real ConstraintBuilder through ``_cbf_builder_harness``. No ROS.
"""

import numpy as np
import pytest

from _cbf_builder_harness import make_builder, make_obstacle, run

from franka_experiments.utils.static_scene import (
    StaticSceneVoxels, voxels_from_params)
from franka_experiments.utils.surface_velocity import (
    SurfaceNormalSpeed, surface_speed_from_params)

# n̂ = (pr − ph)/‖·‖ = +y, so a surface point moving along +y is CLOSING.
PR = np.array([0.5, 0.0, 0.5])
PH = np.array([0.5, -0.25, 0.5])
NHAT = np.array([0.0, 1.0, 0.0])
RATE = 90.0
DT = 1.0 / RATE


# ── 1. the surface estimator ────────────────────────────────────────────────

def _est(**kw):
    kw.setdefault('min_samples', 8)
    kw.setdefault('min_span_s', 0.08)
    return SurfaceNormalSpeed(**kw)


def _drive(e, path, *, n=24, key='cp', t0=0.0):
    """Feed ``n`` frames of ``path(k) -> p_h`` and return every answer."""
    return [e.update(key, path(k), NHAT, t0 + k * DT) for k in range(n)]


def test_a_static_surface_reads_zero_however_the_arm_slides_along_it():
    """The reason the projection is onto n̂: the argmin SLIDES along a static
    surface as the control point moves, and that sliding is tangential."""
    # the surface point slides 10 cm/s in z (tangential), never in y
    out = _drive(_est(), lambda k: PH + np.array([0.0, 0.0, 0.10 * k * DT]))
    assert out[0] is None and out[6] is None        # no span yet
    assert all(abs(v) < 1e-9 for v in out if v is not None)
    assert out[-1] is not None


def test_a_window_must_span_real_time_before_it_reports_anything():
    """The property the whole feature rests on: a slope fitted over an
    interval shorter than the noise is the noise. min_span_s, not a count."""
    e = _est(min_samples=2, min_span_s=0.08)
    out = _drive(e, lambda k: PH, n=10)
    # 0.08 s is 7.2 frames at 90 Hz: nothing before that, something after
    assert all(v is None for v in out[:8])
    assert out[-1] is not None


def test_a_surface_closing_at_30_cm_per_second_reads_30_cm_per_second():
    v = _drive(_est(), lambda k: PH + np.array([0.0, 0.30 * k * DT, 0.0]))[-1]
    assert v == pytest.approx(0.30, abs=1e-6)


def test_a_receding_surface_reads_negative():
    v = _drive(_est(), lambda k: PH - np.array([0.0, 0.20 * k * DT, 0.0]))[-1]
    assert v == pytest.approx(-0.20, abs=1e-6)


def test_the_slope_survives_the_per_frame_sliding_that_defeated_a_median():
    """THE measured failure of the first version, as a unit test.

    A real 0.15 m/s approach with 5 mm of per-frame argmin jitter on top — the
    situation every row on a moving body is in. Compared over a whole run, not
    one window, because the claim is statistical: the fitted slope recovers the
    approach, while the median of consecutive differences (what this module
    shipped first) is dominated by the jitter, which at 90 Hz is +-0.45 m/s per
    difference against a 0.15 m/s signal. On the real data that cost a factor
    of nine; see the table in utils/surface_velocity.py.
    """
    rng = np.random.default_rng(7)
    n = 200
    jitter = rng.normal(0.0, 0.005, n)
    path = [PH + np.array([0.0, 0.15 * k * DT + jitter[k], 0.0]) for k in range(n)]

    e = _est()
    slope = [v for v in (e.update('cp', path[k], NHAT, k * DT) for k in range(n))
             if v is not None]
    diffs = [float(NHAT @ (path[k] - path[k - 1])) / DT for k in range(1, n)]
    median5 = [float(np.median(diffs[k - 5:k])) for k in range(5, n - 1)]

    rms = lambda a: float(np.sqrt(np.mean((np.asarray(a) - 0.15) ** 2)))  # noqa: E731
    assert rms(slope) < 0.05, 'the slope must recover the approach'
    assert rms(median5) > 3.0 * rms(slope), 'the shape it replaced must not'


def test_an_argmin_hop_yields_NO_ESTIMATE_not_a_spike():
    """THE safety property. A 15 cm reattachment over one 11 ms frame would be
    13 m/s of 'approach'; the estimator must refuse rather than report it."""
    e = _est(jump_m=0.10)
    for k in range(20):
        e.update('cp', PH, NHAT, k * DT)
    hopped = PH + np.array([0.0, 0.15, 0.0])
    assert e.update('cp', hopped, NHAT, 20 * DT) is None
    # and it starts a NEW window on the new surface rather than fitting across
    assert e.update('cp', hopped, NHAT, 21 * DT) is None
    out = [e.update('cp', hopped, NHAT, k * DT) for k in range(22, 40)]
    assert out[-1] == pytest.approx(0.0, abs=1e-9)


def test_a_fast_obstacle_the_hop_test_cannot_follow_gets_no_cap_at_all():
    """The failure direction that matters: a ball at 4 m/s seen at 30 Hz moves
    13 cm per frame, trips the hop test, and must therefore produce None —
    which the consumer reads as 'no opinion', never as 'it is not moving'."""
    e = _est(jump_m=0.10, v_max=2.0)
    dt = 1.0 / 30.0
    outs = [e.update('cp', PH + np.array([0.0, 4.0 * k * dt, 0.0]), NHAT, k * dt)
            for k in range(10)]
    assert all(o is None for o in outs)


def test_a_dropped_frame_is_not_differenced():
    e = _est(max_dt_s=0.2)
    for k in range(20):
        e.update('cp', PH, NHAT, k * DT)
    assert e.update('cp', PH, NHAT, 20 * DT + 0.5) is None


def test_being_called_faster_than_the_camera_returns_the_cached_estimate():
    """The QP rebuilds at ~100 Hz on a 30-90 Hz stream: the same frame must not
    be fitted against itself (that would be a division by ~0)."""
    e = _est(min_dt_s=0.004)
    path = lambda k: PH + np.array([0.0, 0.30 * k * DT, 0.0])   # noqa: E731
    v = _drive(e, path)[-1]
    again = [e.update('cp', path(23), NHAT, 23 * DT + j * 1e-3) for j in range(4)]
    assert all(a == pytest.approx(v) for a in again)


def test_one_bad_frame_cannot_dominate_the_slope():
    """Not rejected (a least-squares fit uses every sample) but BOUNDED: one
    5 cm outlier inside a 20-sample window is worth a fraction of what it
    would be worth as a single difference (4.5 m/s)."""
    def path(k):
        return PH + (np.array([0.0, 0.05, 0.0]) if k == 15 else np.zeros(3))

    v = _drive(_est(), path, n=24)[-1]
    assert abs(v) < 0.25


def test_control_points_do_not_share_a_history():
    e = _est()
    for k in range(24):
        e.update('a', PH + np.array([0.0, 0.30 * k * DT, 0.0]), NHAT, k * DT)
        e.update('b', PH, NHAT, k * DT)
    assert e.last('a') == pytest.approx(0.30, abs=1e-6)
    assert e.last('b') == pytest.approx(0.0, abs=1e-9)
    assert e.n_keys == 2


def test_pruning_bounds_the_memory():
    e = _est()
    for k in range(24):
        e.update('cp', PH, NHAT, k * DT)
    e.prune(24 * DT + 5.0)
    assert e.n_keys == 0


def test_non_finite_input_is_ignored():
    e = _est()
    assert e.update('cp', [np.nan, 0.0, 0.0], NHAT, 0.0) is None
    assert e.n_keys == 0


def test_surface_speed_from_params_is_none_when_the_flag_is_off():
    class P:
        enable_surface_velocity_cap = False
    assert surface_speed_from_params(P) is None
    P.enable_surface_velocity_cap = True
    assert isinstance(surface_speed_from_params(P), SurfaceNormalSpeed)


# ── 2. the place-keyed scene memory ─────────────────────────────────────────

def _vox(**kw):
    kw.setdefault('voxel_m', 0.10)
    kw.setdefault('window_s', 0.2)
    kw.setdefault('min_samples', 8)
    return StaticSceneVoxels(**kw)


def test_a_quiet_place_becomes_scene_once_it_has_a_window():
    v = _vox()
    for k in range(40):
        v.observe(PH, 0.01, k * DT)
    assert v.is_static(PH) and v.n_static == 1


def test_a_place_with_motion_through_it_is_never_scene():
    v = _vox()
    for k in range(40):
        v.observe(PH, 0.30, k * DT)
    assert not v.is_static(PH)


def test_motion_releases_a_place_already_classified_as_scene():
    v = _vox(release_speed_m_s=0.25, release_hits=3)
    for k in range(40):
        v.observe(PH, 0.01, k * DT)
    assert v.is_static(PH)
    for k in range(40, 42):
        v.observe(PH, 0.5, k * DT)
        assert v.is_static(PH), 'one or two frames is a merge artefact'
    v.observe(PH, 0.5, 42 * DT)
    assert not v.is_static(PH)


def test_a_released_place_must_earn_a_whole_window_again():
    v = _vox(window_s=0.2, release_speed_m_s=0.25, release_hits=3)
    for k in range(40):
        v.observe(PH, 0.01, k * DT)
    for k in range(40, 43):
        v.observe(PH, 0.5, k * DT)
    assert not v.is_static(PH)
    # quiet again, but not yet for a full window
    for k in range(43, 52):
        v.observe(PH, 0.0, k * DT)
    assert not v.is_static(PH)
    for k in range(52, 80):
        v.observe(PH, 0.0, k * DT)
    assert v.is_static(PH)


def test_isolated_spikes_do_not_disqualify_a_place():
    v = _vox(release_speed_m_s=0.25, release_hits=3)
    for k in range(60):
        v.observe(PH, 0.5 if k % 13 == 0 else 0.0, k * DT)
    assert v.is_static(PH)


def test_places_are_independent_and_the_key_is_the_voxel():
    v = _vox(voxel_m=0.10)
    near = PH + np.array([0.0, 0.02, 0.0])      # same 10 cm cell
    far = PH + np.array([0.0, 0.40, 0.0])       # another cell
    for k in range(40):
        v.observe(PH, 0.0, k * DT)
        v.observe(far, 0.5, k * DT)
    assert v.is_static(PH) and v.is_static(near)
    assert not v.is_static(far)
    assert v.n_cells == 2


def test_the_same_capture_stamp_twice_is_one_sample():
    v = _vox(min_samples=8, window_s=0.2)
    for _ in range(40):
        v.observe(PH, 0.0, 0.0)
    assert not v.is_static(PH), 'forty rebuilds of one frame are not a window'


def test_pruning_and_the_cell_cap_bound_the_memory():
    v = _vox(max_cells=32)
    for k in range(200):
        v.observe(np.array([0.5, -0.25, 0.5 + 0.2 * k]), 0.0, k * DT)
    assert v.n_cells <= 32
    v.prune(200 * DT + 5.0)
    assert v.n_cells == 0


def test_voxels_from_params_is_none_when_the_flag_is_off():
    class P:
        enable_static_scene_voxels = False
    assert voxels_from_params(P) is None
    P.enable_static_scene_voxels = True
    assert isinstance(voxels_from_params(P), StaticSceneVoxels)


def test_the_place_memory_reads_its_OWN_window_and_shares_the_thresholds():
    """The split that the coverage measurement forced: a place is sampled far
    more sparsely than a track, so the window and the sample count are its own
    — while the speed and release thresholds stay shared, because the same wall
    must not be scene by one definition and a body by the other."""
    class P:
        enable_static_scene_voxels = True
        static_scene_window_s = 9.0          # the TRACK classifier's: ignored
        static_scene_min_samples = 99        # ditto
        static_scene_voxel_window_s = 0.2
        static_scene_voxel_min_samples = 4
        static_scene_voxel_m = 0.15
        static_scene_speed_m_s = 0.08
        static_scene_release_speed_m_s = 0.25
        static_scene_release_hits = 3

    v = voxels_from_params(P)
    for k in range(20):                      # 0.21 s of quiet, 20 samples
        v.observe(PH, 0.0, k * DT)
    assert v.is_static(PH), 'it must not have waited for the track window'
    # the shared thresholds still bite
    for k in range(20, 23):
        v.observe(PH, 0.5, k * DT)
    assert not v.is_static(PH)


# ── 3. the rows: OFF is bit-identical ───────────────────────────────────────

BASE = dict(obstacle_velocity_source='tracker', enable_vobs_in_hdot=True,
            enable_velocity_standoff=True, enable_velocity_feedforward=True,
            enable_uncertainty_margin=True, obstacle_velocity_min_frames=3,
            velocity_feedforward_min_frames=3)


def _assert_identical(a, b):
    np.testing.assert_array_equal(a.A, b.A)
    np.testing.assert_array_equal(a.h_bar, b.h_bar)
    np.testing.assert_array_equal(a.jdot_qdot, b.jdot_qdot)
    np.testing.assert_array_equal(a.G, b.G)
    np.testing.assert_array_equal(a.v_obs, b.v_obs)


def _moving_surface(v_claim, v_true, *, track_id=7, frames_seen=30, d0=0.30):
    """A row whose TRACK claims ``v_claim`` while its SURFACE moves ``v_true``."""
    def frame(k):
        t = k * DT
        return [make_obstacle(d=d0 - v_true * t, pr=PR,
                              ph=PH + np.array([0.0, v_true * t, 0.0]),
                              v=(0.0, v_claim, 0.0), frames_seen=frames_seen,
                              cov=np.eye(3) * 0.01, track_id=track_id,
                              cp_label='fr3_link5#0')]
    return frame


def test_with_the_flags_off_the_rows_are_bit_identical():
    """The regression this repo holds every feature to: OFF changes nothing."""
    f = _moving_surface(3.0, 0.0)
    a = run(make_builder(**BASE), f, n_frames=40, dt=DT, qdot=0.0)
    c = run(make_builder(**dict(BASE, enable_static_scene_voxels=False,
                                enable_surface_velocity_cap=False)),
            f, n_frames=40, dt=DT, qdot=0.0)
    _assert_identical(a, c)


# ── 4. P4: the cap, and what it refuses to do ───────────────────────────────

def test_a_phantom_centroid_velocity_is_capped_at_what_the_surface_does():
    """The measured pathology: the track claims 3 m/s while the surface it was
    measured from has not moved."""
    f = _moving_surface(3.0, 0.0)
    off = make_builder(**BASE)
    on = make_builder(**dict(BASE, enable_surface_velocity_cap=True,
                             surface_velocity_cap_k=2.0,
                             surface_velocity_cap_floor=0.05))
    a = run(off, f, n_frames=40, dt=DT, qdot=0.0)
    c = run(on, f, n_frames=40, dt=DT, qdot=0.0)
    # the row's GEOMETRY is untouched — this feature only ever touches a speed
    np.testing.assert_allclose(a.A, c.A, atol=0, rtol=0)
    assert on.diag_surf_capped >= 1 and off.diag_surf_capped == 0
    assert float(c.v_obs[0]) == pytest.approx(0.05, abs=1e-9)
    assert float(a.v_obs[0]) > 1.0
    # h is LOOSER by exactly the margins the phantom bought, never tighter
    assert float(c.h_bar[0]) > float(a.h_bar[0])
    assert on.diag_hbrake < off.diag_hbrake
    assert on.diag_hstand < off.diag_hstand
    # and the ḣ channel — the big one, k1 x this — is capped too
    assert abs(on.diag_vobs_hdot) <= 0.05 + 1e-9 < abs(off.diag_vobs_hdot)


def test_a_genuine_approach_is_left_alone():
    """The safety direction: when the two estimators agree, nothing happens."""
    f = _moving_surface(0.30, 0.30)
    off = make_builder(**BASE)
    on = make_builder(**dict(BASE, enable_surface_velocity_cap=True))
    a = run(off, f, n_frames=40, dt=DT, qdot=0.0)
    c = run(on, f, n_frames=40, dt=DT, qdot=0.0)
    assert float(c.v_obs[0]) == pytest.approx(float(a.v_obs[0]), abs=1e-9)
    assert on.diag_surf_capped == 0
    np.testing.assert_allclose(a.h_bar, c.h_bar, atol=0, rtol=0)


def test_the_cap_can_only_lower_a_claim_never_raise_one():
    """A track that under-reports a real approach is not 'corrected' upward by
    this feature: raising a claim on a geometric estimate is the residual
    floor's job, which has its own guards."""
    f = _moving_surface(0.0, 0.40)
    off = make_builder(**BASE)
    on = make_builder(**dict(BASE, enable_surface_velocity_cap=True))
    a = run(off, f, n_frames=40, dt=DT, qdot=0.0)
    c = run(on, f, n_frames=40, dt=DT, qdot=0.0)
    assert float(c.v_obs[0]) <= float(a.v_obs[0]) + 1e-12


def test_a_fast_trusted_track_is_exempt_from_the_cap():
    """Same convention as P1/P2: a track that cleared a speed test against its
    own covariance is evidence of motion no geometry may contradict."""
    fast = dict(BASE, enable_surface_velocity_cap=True,
                obstacle_velocity_fast_trust=True,
                obstacle_velocity_fast_min_frames=3,
                obstacle_velocity_fast_speed=1.0,
                obstacle_velocity_fast_max=6.0, obstacle_velocity_max=6.0)
    f = _moving_surface(3.0, 0.0)
    b = make_builder(**fast)
    c = run(b, f, n_frames=40, dt=DT, qdot=0.0)
    assert b.diag_fast >= 1
    assert b.diag_surf_capped == 0
    assert float(c.v_obs[0]) > 1.0


# ── 5. P3: the rows with no identity ────────────────────────────────────────

VOX_ON = dict(BASE, enable_static_scene_voxels=True, static_scene_voxel_m=0.10,
              static_scene_window_s=0.2, static_scene_min_samples=8,
              obstacle_velocity_residual_floor=True,
              obstacle_velocity_residual_floor_gap=0.0)


def _untracked(v_surface=0.0, d0=0.30, *, q=0.1):
    """A row with NO track id — the half of the rows P2 cannot reach. Its
    velocity can therefore only come from the residual floor."""
    def frame(k):
        t = k * DT
        return [make_obstacle(d=d0 - v_surface * t, pr=PR,
                              ph=PH + np.array([0.0, v_surface * t, 0.0]),
                              v=None, frames_seen=0, track_id=0,
                              cp_label='fr3_link5#0')]
    return frame


def test_an_untracked_row_on_a_quiet_place_loses_its_velocity_claim():
    """P3's whole point: no track id, so P1/P2 cannot speak, and the residual
    floor is what would hand this row a velocity."""
    # the residual sees a closing distance (the robot is moving toward it)
    f = _untracked(0.0)
    off = make_builder(**dict(VOX_ON, enable_static_scene_voxels=False))
    on = make_builder(**VOX_ON)
    a = run(off, f, n_frames=60, dt=DT, qdot=0.3)
    c = run(on, f, n_frames=60, dt=DT, qdot=0.3)
    np.testing.assert_allclose(a.A, c.A, atol=0, rtol=0)
    assert on.diag_vox_rows >= 1 and on.diag_vox_static >= 1
    assert off.diag_vox_rows == 0
    assert float(c.v_obs[0]) <= 1e-12 < float(a.v_obs[0])
    assert float(c.h_bar[0]) > float(a.h_bar[0])


def test_a_place_the_obstacle_is_moving_through_keeps_its_velocity():
    """The safety direction for P3: a surface closing at 0.4 m/s is never a
    place the memory may call scene."""
    f = _untracked(0.4, d0=0.50)
    off = make_builder(**dict(VOX_ON, enable_static_scene_voxels=False))
    on = make_builder(**VOX_ON)
    a = run(off, f, n_frames=40, dt=DT, qdot=0.0)
    c = run(on, f, n_frames=40, dt=DT, qdot=0.0)
    assert on.diag_vox_rows == 0
    assert float(c.v_obs[0]) == pytest.approx(float(a.v_obs[0]), abs=1e-12)


def _hold_frames(flags, frame, n=180):
    """Frames on which the place memory withheld a row's velocity.

    diag_vox_rows is a PER-REBUILD counter, so reading it after the last frame
    measures that frame alone — which is how the first version of the test
    below passed while measuring nothing at all.
    """
    from _cbf_builder_harness import make_js, make_obs
    b = make_builder(**flags)
    held = 0
    for k in range(n):
        t = k * DT
        b.build(make_js(q=0.1, qdot=0.0, stamp=t), make_obs(frame(k), stamp=t), t)
        held += 1 if b.diag_vox_rows else 0
    return held, b


@pytest.mark.parametrize('v_true,expect_held', [
    (0.05, True), (0.08, True), (0.10, False), (0.15, False), (0.25, False)])
def test_where_the_place_memory_stops_believing_a_slow_approacher(v_true, expect_held):
    """The one case the WINDOW cannot protect, so the SPEED TEST must — and the
    boundary it actually draws, measured rather than assumed.

    A body crossing a 0.15 m cell at 0.10 m/s is inside it for 1.5 s, longer
    than the cell's 0.5 s window, so unlike a sweeping ball it CAN accumulate a
    full window of history in one place. What decides is then
    static_scene_speed_m_s (0.08 m/s), and that is exactly where the boundary
    falls: at and below it an untracked body's velocity claim IS withheld (a
    deliberate trade — it keeps its full geometric barrier and loses only the
    anticipation, which at 0.08 m/s is 0.8 mm of braking margin and 1.6 cm of
    standoff), and from 0.10 m/s upward it is not.

    That the test works at all depends on the estimator being a fitted slope:
    the median-of-differences form this module shipped first read 0.02 m/s on a
    body moving at 0.15 and would have called every one of these cells quiet.
    """
    held, b = _hold_frames(VOX_ON, _untracked(v_true, d0=0.60))
    if expect_held:
        assert held > 0
    else:
        assert held == 0, f'a body closing at {v_true} m/s was held on {held} frames'
        assert b.diag_surf_v == pytest.approx(v_true, abs=0.01), 'and it was measured'


def test_a_place_a_body_has_LEFT_can_go_quiet_again():
    """The other half of that: once the body is gone the place is just a place,
    and the room behind it may be held again — after a fresh full window."""
    def frame(k):
        t = k * DT
        # a body closes at 0.2 m/s for 1 s, then the row falls back to a
        # static surface at the point it reached
        y = min(0.20 * t, 0.20)
        return [make_obstacle(d=0.30, pr=PR, ph=PH + np.array([0.0, y, 0.0]),
                              v=None, frames_seen=0, track_id=0,
                              cp_label='fr3_link5#0')]

    early, _ = _hold_frames(VOX_ON, frame, n=90)
    assert early == 0, 'not while the body is still moving'
    late, _ = _hold_frames(VOX_ON, frame, n=270)
    assert late > 0, 'but the place may settle once it is still'


def test_a_row_that_HAS_a_track_is_decided_by_the_track_not_by_the_place():
    """Where an identity exists the place must not overrule it: a body that
    walks into a spot the room used to occupy keeps its velocity."""
    def frame(k):
        t = k * DT
        tid = 0 if k < 40 else 55        # the place goes quiet, then a body
        return [make_obstacle(d=0.30, pr=PR, ph=PH,
                              v=(0.0, 0.5, 0.0) if tid else None,
                              frames_seen=30 if tid else 0,
                              cov=np.eye(3) * 0.01, track_id=tid,
                              cp_label='fr3_link5#0')]
    b = make_builder(**VOX_ON)
    c = run(b, frame, n_frames=60, dt=DT, qdot=0.0)
    assert b.diag_vox_static >= 1, 'the place itself was classified quiet'
    assert b.diag_vox_rows == 0, 'but the row has a track id, so P3 stays out'
    assert float(c.v_obs[0]) == pytest.approx(0.5, abs=1e-6)
