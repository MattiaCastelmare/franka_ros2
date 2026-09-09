"""Track lifecycle: identity across a crossing, across an occlusion, and against
a one-frame artefact.

These three cases are the whole reason a tracker exists rather than a per-frame
nearest-cluster lookup, and each has a distinct consequence when it fails:

* an identity SWAP during a crossing reverses the velocity attached to each
  obstacle in a single frame — the largest error the estimator can make, and it
  makes it exactly when two people are close to the robot;
* a track KILLED by a brief occlusion is replaced by a newborn whose velocity
  starts at zero, so a continuously approaching obstacle reads as stationary for
  the ~5 frames it takes to re-converge;
* a one-frame artefact CONFIRMED as a track hands the barrier a velocity
  measured on noise.

A minimal stand-in for Cluster is used so these tests exercise the manager only
— clustering has its own file.

Pure numpy, no ROS.
"""

import numpy as np

from franka_experiments.utils.obstacle_tracker import KalmanTrack, TrackManager

DT = 1.0 / 30.0


class _C:
    """Just enough of a Cluster: the manager only reads ``centroid_cam``."""

    def __init__(self, p):
        self.centroid_cam = np.asarray(p, dtype=np.float64)
        self.n_points = 500
        self.radius = 0.1


def _noisy(p, rng, sigma=0.005):
    return _C(np.asarray(p, dtype=np.float64) + rng.normal(scale=sigma, size=3))


def _confirm(mgr, p, rng, n=4):
    """Run `n` frames of a stationary blob at p so its track is confirmed."""
    for _ in range(n):
        mgr.step([_noisy(p, rng)], DT)
    return mgr


# ── Crossing: ids must not swap ─────────────────────────────────────────────

def _crossing(n=61, sep=0.02, span=1.6, seed=0, sigma=0.006):
    """Two obstacles moving through each other in x, `sep` metres apart in y.

    The geometry of two people passing. At the crossing frame the two centroids
    are `sep` apart while each has advanced ~2.7 cm since the previous frame, so
    a memoryless "nearest cluster to where I last saw it" lookup is a coin flip
    there — see test_a_memoryless_associator_actually_swaps_on_this_data.
    """
    rng = np.random.default_rng(seed)
    t = np.arange(n) * DT
    s = span * (t / t[-1] - 0.5)
    A = np.stack([+s, np.full(n, +0.5 * sep), np.ones(n)], axis=1)
    B = np.stack([-s, np.full(n, -0.5 * sep), np.ones(n)], axis=1)
    frames = [(_noisy(A[k], rng, sigma), _noisy(B[k], rng, sigma)) for k in range(n)]
    return A, B, frames


def test_two_crossing_trajectories_keep_their_ids():
    """The headline case. An identity swap here reverses the velocity attached
    to each obstacle in a single frame — the largest error the estimator can
    make, made exactly when two people are close to the robot. Only the KF's
    PREDICTION separates them at the crossing: the positions are 2 cm apart,
    the predictions are 5 cm apart and moving oppositely."""
    A, B, frames = _crossing()
    mgr = TrackManager()

    id_a = id_b = None
    swaps = 0
    for k, (ca, cb) in enumerate(frames):
        mgr.step([ca, cb], DT)
        conf = mgr.confirmed_tracks()
        if len(conf) != 2:
            continue
        near_a = min(conf, key=lambda tr: np.linalg.norm(tr.position - A[k]))
        near_b = min(conf, key=lambda tr: np.linalg.norm(tr.position - B[k]))
        if id_a is None:
            id_a, id_b = near_a.track_id, near_b.track_id
            continue
        if near_a.track_id != id_a or near_b.track_id != id_b:
            swaps += 1
    assert id_a is not None and id_a != id_b, 'both tracks must exist and differ'
    assert swaps == 0, f'{swaps} frames with swapped identities'


def test_the_crossing_fixture_is_actually_ambiguous():
    """Guards the guard. The test above proves nothing unless the two clusters
    genuinely enter the regime where a per-frame lookup cannot tell them apart:
    their closest approach must be SMALLER than the distance each of them
    travels between frames. Below that ratio, "the nearest cluster to where I
    last saw it" and "the nearest cluster to the other one" are the same
    answer, and only a motion model separates them."""
    A, B, _ = _crossing()
    closest = float(np.linalg.norm(A - B, axis=1).min())
    per_frame = float(np.linalg.norm(np.diff(A, axis=0), axis=1).mean())
    assert closest < per_frame, f'closest approach {closest:.3f} m vs ' \
                                f'{per_frame:.3f} m travelled per frame'


def test_the_crossing_holds_across_seeds():
    """One seed is an anecdote. The identity must survive the crossing for every
    noise realisation, not for the one that happened to be checked in."""
    for seed in range(8):
        A, B, frames = _crossing(seed=seed)
        mgr = TrackManager()
        ids, swaps = None, 0
        for k, (ca, cb) in enumerate(frames):
            mgr.step([ca, cb], DT)
            conf = mgr.confirmed_tracks()
            if len(conf) != 2:
                continue
            cur = (min(conf, key=lambda tr: np.linalg.norm(tr.position - A[k])).track_id,
                   min(conf, key=lambda tr: np.linalg.norm(tr.position - B[k])).track_id)
            if ids is None:
                ids = cur
            elif cur != ids:
                swaps += 1
        assert ids is not None and ids[0] != ids[1], f'seed {seed}: tracks missing'
        assert swaps == 0, f'seed {seed}: {swaps} swapped frames'


def test_the_crossing_leaves_each_track_with_its_own_velocity_sign():
    """An id that survives but carries the other obstacle's velocity is no
    better than a swap. x-velocities must stay opposite throughout."""
    _, _, frames = _crossing(seed=1)
    mgr = TrackManager()
    for ca, cb in frames:
        mgr.step([ca, cb], DT)
    conf = sorted(mgr.confirmed_tracks(), key=lambda tr: tr.track_id)
    assert len(conf) == 2
    vx = sorted(tr.velocity[0] for tr in conf)
    assert vx[0] < -0.3 and vx[1] > 0.3, f'velocities {vx}'


# ── Occlusion: same id after coasting ───────────────────────────────────────

def test_a_three_frame_occlusion_is_coasted_and_keeps_the_same_id():
    """A hand passing behind the arm, or the exclusion mask briefly swallowing
    the blob. The track must coast on predict() and be re-associated to the SAME
    id, not replaced by a newborn whose velocity starts at zero."""
    rng = np.random.default_rng(2)
    mgr = TrackManager()
    v = np.array([0.0, 0.0, -0.4])
    p = np.array([0.0, 0.0, 1.4])

    for _ in range(8):
        mgr.step([_noisy(p, rng)], DT)
        p = p + v * DT
    before = mgr.confirmed_tracks()
    assert len(before) == 1
    id0 = before[0].track_id

    for _ in range(3):                      # occluded: no cluster at all
        mgr.step([], DT)
        p = p + v * DT
    coasting = mgr.tracks
    assert len(coasting) == 1 and coasting[0].track_id == id0, 'track was killed'

    mgr.step([_noisy(p, rng)], DT)          # reappears where it should be
    after = mgr.confirmed_tracks()
    assert len(after) == 1, f'{len(after)} tracks after re-appearance'
    assert after[0].track_id == id0, 'a duplicate id was spawned'
    assert after[0].velocity[2] < -0.2, 'the coasted velocity was not preserved'


def test_a_track_dies_after_max_missed_frames():
    """Coasting is bounded. An obstacle that has left must stop contributing a
    velocity, and its id must not be handed to whatever appears next."""
    rng = np.random.default_rng(3)
    mgr = _confirm(TrackManager(max_missed=5), [0.0, 0.0, 1.0], rng)
    id0 = mgr.confirmed_tracks()[0].track_id
    for _ in range(6):
        mgr.step([], DT)
    assert mgr.tracks == [] and mgr.confirmed_tracks() == []
    mgr.step([_C([0.0, 0.0, 1.0])], DT)
    assert mgr.tracks[0].track_id != id0, 'a dead id was reused'


def test_a_coasting_track_is_still_invisible_to_nothing_but_stays_confirmed():
    """A confirmed track that is briefly missing must remain confirmed — losing
    confirmation on one dropped frame would restart the M-of-N clock and make a
    flickering obstacle permanently invisible to the barrier."""
    rng = np.random.default_rng(4)
    mgr = _confirm(TrackManager(), [0.0, 0.0, 1.0], rng)
    mgr.step([], DT)
    assert len(mgr.confirmed_tracks()) == 1


# ── Spurious blobs: never confirmed ─────────────────────────────────────────

def test_a_one_frame_spurious_blob_is_never_confirmed():
    """A depth artefact appears once and is gone. It may create a TENTATIVE
    track — it has to, the manager cannot know yet — but nothing downstream may
    ever see it."""
    mgr = TrackManager()
    mgr.step([_C([0.3, 0.3, 1.0])], DT)
    assert mgr.confirmed_tracks() == []
    for _ in range(10):
        mgr.step([], DT)
        assert mgr.confirmed_tracks() == []
    assert mgr.tracks == []


def test_a_two_frame_blob_is_still_never_confirmed():
    """The default rule is 3-of-5, so two frames must not be enough — this is
    the boundary the default is chosen at."""
    rng = np.random.default_rng(5)
    mgr = TrackManager()
    for _ in range(2):
        mgr.step([_noisy([0.3, 0.3, 1.0], rng)], DT)
    assert mgr.confirmed_tracks() == []


def test_a_persistent_blob_is_confirmed_on_the_third_frame():
    rng = np.random.default_rng(6)
    mgr = TrackManager()
    seen = [len(mgr.step([_noisy([0.3, 0.3, 1.0], rng)], DT)) for _ in range(4)]
    assert seen == [0, 0, 1, 1], seen


def test_confirmation_is_m_of_n_not_m_consecutive():
    """A real obstacle at the edge of the depth range flickers. Demanding
    CONSECUTIVE hits would keep restarting its clock and it would never become
    visible, while a one-frame artefact fails both rules anyway."""
    rng = np.random.default_rng(7)
    mgr = TrackManager(confirm_hits=3, confirm_window=5)
    p = [0.3, 0.3, 1.0]
    for hit in (True, False, True, True):
        mgr.step([_noisy(p, rng)] if hit else [], DT)
    assert len(mgr.confirmed_tracks()) == 1


# ── Gating ──────────────────────────────────────────────────────────────────

def test_a_teleporting_cluster_is_not_associated():
    """The Mahalanobis gate widens while a track coasts, without bound. The hard
    Euclidean ceiling is the backstop that keeps a resurrected track physically
    plausible — without it, after enough missed frames a track would adopt a
    cluster anywhere in the room."""
    rng = np.random.default_rng(8)
    mgr = _confirm(TrackManager(gate_max_m=0.5), [0.0, 0.0, 1.0], rng)
    id0 = mgr.confirmed_tracks()[0].track_id
    for _ in range(4):
        mgr.step([], DT)
    mgr.step([_C([3.0, 0.0, 1.0])], DT)          # 3 m away
    ids = {t.track_id for t in mgr.tracks}
    assert id0 in ids, 'the original track should still be coasting'
    assert len(ids) == 2, 'the far cluster must have spawned its own track'


def test_the_gate_is_mahalanobis_so_a_coasting_track_reaches_further():
    """A fixed Euclidean radius cannot do both jobs: sized for the occlusion
    case it would swap ids between two close obstacles every frame. The gate has
    to be in units of the filter's own uncertainty."""
    rng = np.random.default_rng(9)
    mgr = _confirm(TrackManager(), [0.0, 0.0, 1.0], rng, n=8)
    trk = mgr.confirmed_tracks()[0]
    offset = np.array([0.10, 0.0, 0.0])
    tight = trk.mahalanobis(trk.position + offset)
    for _ in range(4):
        trk.predict(DT)
    assert trk.mahalanobis(trk.position + offset) < tight
    assert tight > mgr.gate_mahalanobis > trk.mahalanobis(trk.position + offset)


def test_two_nearby_obstacles_get_two_tracks_not_one():
    rng = np.random.default_rng(10)
    mgr = TrackManager()
    for _ in range(5):
        mgr.step([_noisy([0.0, 0.0, 1.0], rng), _noisy([0.25, 0.0, 1.0], rng)], DT)
    assert len(mgr.confirmed_tracks()) == 2


# ── Ids and bookkeeping ─────────────────────────────────────────────────────

def test_track_ids_are_stable_positive_integers():
    """0 is reserved: LinkDistance.track_id = 0 means "no track", so a real
    track must never be given it."""
    rng = np.random.default_rng(11)
    mgr = _confirm(TrackManager(), [0.0, 0.0, 1.0], rng)
    t = mgr.confirmed_tracks()[0]
    assert isinstance(t.track_id, int) and t.track_id >= 1
    ids = []
    for _ in range(10):
        mgr.step([_noisy([0.0, 0.0, 1.0], rng)], DT)
        ids.append(mgr.confirmed_tracks()[0].track_id)
    assert set(ids) == {t.track_id}


def test_frames_seen_counts_updates_not_predicts():
    """Step 7 gates on frames_seen >= 3. If coasting counted, a track could
    reach the threshold while being observed only once."""
    rng = np.random.default_rng(12)
    mgr = _confirm(TrackManager(), [0.0, 0.0, 1.0], rng, n=4)
    t = mgr.confirmed_tracks()[0]
    n0 = t.frames_seen
    mgr.step([], DT)
    assert t.frames_seen == n0
    mgr.step([_noisy([0.0, 0.0, 1.0], rng)], DT)
    assert t.frames_seen == n0 + 1


def test_max_tracks_bounds_the_track_set():
    """A degenerate frame — the exclusion mask failing, the whole scene reading
    as obstacle — must not produce hundreds of tracks: association is O(T*C)."""
    mgr = TrackManager(max_tracks=4)
    mgr.step([_C([0.4 * k, 0.0, 1.0]) for k in range(20)], DT)
    assert len(mgr.tracks) == 4


def test_reset_drops_everything_but_does_not_rewind_ids():
    """A consumer holding an old id must see it disappear, never see it silently
    refer to a different obstacle."""
    rng = np.random.default_rng(13)
    mgr = _confirm(TrackManager(), [0.0, 0.0, 1.0], rng)
    old = mgr.confirmed_tracks()[0].track_id
    mgr.reset()
    assert mgr.tracks == [] and mgr.confirmed_tracks() == []
    mgr.step([_C([0.0, 0.0, 1.0])], DT)
    assert mgr.tracks[0].track_id > old


def test_an_empty_frame_coasts_every_track_and_returns_nothing_new():
    rng = np.random.default_rng(14)
    mgr = _confirm(TrackManager(), [0.0, 0.0, 1.0], rng)
    n0 = mgr.tracks[0].missed
    mgr.step([], DT)
    assert mgr.tracks[0].missed == n0 + 1


def test_association_is_deterministic_under_input_reordering():
    """Greedy assignment with an index tie-break: the same frame presented in a
    different cluster order must produce the same pairing, or ids would shuffle
    whenever cluster_obstacles reorders equal-sized blobs."""
    rng = np.random.default_rng(15)
    a, b = [0.0, 0.0, 1.0], [0.3, 0.0, 1.0]
    m1, m2 = TrackManager(), TrackManager()
    for _ in range(6):
        ca, cb = _noisy(a, rng), _noisy(b, rng)
        m1.step([ca, cb], DT)
        m2.step([cb, ca], DT)
    p1 = sorted(np.round(t.position, 6).tolist() for t in m1.tracks)
    p2 = sorted(np.round(t.position, 6).tolist() for t in m2.tracks)
    assert p1 == p2


def test_a_newborn_is_not_itself_an_association_candidate_that_frame():
    """Births happen after the update pass. Otherwise two clusters from one
    split blob would spawn a track and immediately feed it, confirming a
    fragment in a single frame."""
    mgr = TrackManager()
    mgr.step([_C([0.0, 0.0, 1.0]), _C([0.02, 0.0, 1.0])], DT)
    assert len(mgr.tracks) == 2
    assert all(t.frames_seen == 1 for t in mgr.tracks)
