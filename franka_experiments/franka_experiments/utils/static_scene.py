"""Which obstacle tracks are the ROOM, and therefore have no velocity to anticipate.

WHY THIS EXISTS
---------------
A depth camera in a real cell returns the cell. The size guard
(``cluster_max_radius_m`` 1.2 m) drops a room-sized blob, but a desk with a
monitor and a laptop on it is PERSON-SIZED: it survives the guard, gets
clusters, gets tracks, and gets a velocity. Measured on the DIAG lab bag
(``datasets/experiments_irim2.zip``, 157.6 s, the arm on a 0.25 m circle):

* the QP carried a median of 46 obstacle rows and a p95 of 67 — the room, not
  the task;
* ``LinkDistance.obstacle_velocity`` reached **7.78 m/s** on the wire and the
  conditioned ``v_obs`` the filter consumed reached **3.11 m/s**, in a cell
  where the fastest thing present was a hand;
* a violated row (h < 0) was present on 23.1 % of QP ticks, and in **90.6 %**
  of those the GEOMETRIC gap was still larger than ``d_safe`` — the barrier was
  being crossed by the velocity-derived margins, not by geometry;
* ``‖q̈_safe − q̈_nom‖`` had a median of 1.64 rad/s² against a nominal of
  1.85 rad/s², i.e. the correction was the size of the task, and 16 % of ticks
  pushed AGAINST the task direction.

The same pathology reproduces in simulation in the furnished ``diag`` scene —
44 obstacle rows where the bare floor gives 22, and phantom closing speeds on
furniture that nothing in the world model is moving — which is where this
module was developed and measured (``sim_runs/batches/static_scene_ab``).

Those velocities are not measurements. A static surface's cluster MERGES AND
SPLITS as the robot mask carves different holes in it, and each merge steps the
centroid, which a differentiator reads as metres per second. The velocity then
enters the row four times over — ``k1·ḣ``, the braking tightening, the
speed-proportional standoff and the covariance margin — so one phantom number
both pushes the arm and moves the barrier outwards.

WHAT IT DOES
------------
Nothing about the geometry. A track classified STATIC keeps its barrier row at
full authority, with its own ``k0``/``k1`` gains; what it loses is the right to
claim a VELOCITY, and with it the four velocity-derived tightenings. That is
exactly the state the pipeline already falls back to whenever a track drops
out, which on hardware is routine (1727 tracks in 91 s, median life 70 ms).

THE CLASSIFIER, AND WHY IT IS MEDIANS
-------------------------------------
Per track id, over a window of ``window_s``:

    static  iff  ‖median(v)‖ <= drift_m / window_s   and   median(|v|) <= speed_m_s

Medians, not means: the phantom is made of ISOLATED excursions (a merge step is
one or two frames), and a mean lets a single 3 m/s spike through — two spikes
in the same direction would be enough to un-static a wall. A median of a
90-sample window is untouched by a handful of them, while a person walking at
0.2 m/s moves the median itself. The first test is about NET motion (a surface
that is being pushed across the bench is not scene), the second about speed
(a surface that is somehow moving fast is not scene either).

RELEASE IS INSTANT AND SUSTAINED-MOTION-TRIGGERED
-------------------------------------------------
``release_hits`` CONSECUTIVE samples above ``release_speed_m_s`` release the
track at once, and it must then earn a full fresh window before it can be
called static again. At 90 Hz, 3 samples is 33 ms: a merge artefact (one or two
frames) cannot release a wall, while a hand that starts moving at 0.5 m/s is
released after 1.7 cm of travel. A person standing still IS classified static,
and that is the intended answer: a stationary obstacle has no approach to
anticipate, the geometric barrier is unchanged, and the release above is what
covers them starting to move.

The classification is advanced by :meth:`observe`, which must be called at most
once per perception frame per track — the caller runs faster than the camera,
so a repeated capture stamp replaces the previous sample rather than appending
a duplicate.

Pure logic: no ROS, no numpy state beyond the window. Unit-tested in
``test/test_cbf_static_scene.py``.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

#: Drop a track this long after its last observation. Bounds the memory: the
#: tracker's ids are never reused, so without this the dict grows with the run.
IDLE_DROP_S = 2.0
#: Hard cap on samples per track, so a stalled stream cannot grow the window.
MAX_SAMPLES = 256


class _Track:
    __slots__ = ('t_first', 't_last', 't_quiet', 'ts', 'v', 'hot', 'static')

    def __init__(self, t: float):
        self.t_first = float(t)
        self.t_last = float(t)
        #: When the UNINTERRUPTED history counting towards a full window
        #: starts: creation, and again at every release. Separate from
        #: ``t_first`` (the identity's age, which a release must not reset)
        #: and from the samples, which are trimmed to the window.
        self.t_quiet = float(t)
        self.ts: list[float] = []
        self.v: list[np.ndarray] = []
        self.hot = 0            # consecutive samples above release_speed_m_s
        self.static = False


class StaticSceneHold:
    """Per-track static/moving classification of the obstacle tracks.

    Args:
        window_s: [s] history a track needs before it can be called static,
            and the window both medians are taken over.
        drift_m: [m] net displacement allowed over that window. Enters as the
            rate ``drift_m / window_s``, so the test is on ‖median(v)‖.
        speed_m_s: [m/s] largest median speed a static track may have.
        release_speed_m_s: [m/s] speed that, sustained, releases the track.
        release_hits: consecutive samples above that speed needed to release.
        min_samples: samples the window must hold, whatever its span.
    """

    def __init__(self, *, window_s: float = 1.0, drift_m: float = 0.04,
                 speed_m_s: float = 0.08, release_speed_m_s: float = 0.25,
                 release_hits: int = 3, min_samples: int = 8):
        self._window = max(float(window_s), 1e-3)
        self._drift_rate = max(float(drift_m), 0.0) / self._window
        self._speed = max(float(speed_m_s), 0.0)
        self._rel_speed = max(float(release_speed_m_s), 0.0)
        self._rel_hits = max(int(release_hits), 1)
        self._min_n = max(int(min_samples), 2)
        self._tr: dict[int, _Track] = {}

    # ── bookkeeping ──────────────────────────────────────────────────────────
    def observe(self, track_id: int, v_vec, t: float) -> None:
        """Feed this track's reported velocity for the perception frame at ``t``.

        ``track_id`` <= 0 ("no confirmed track") is ignored: with no identity
        there is no history to judge, and the conservative answer is to leave
        the row exactly as it is today. A repeated ``t`` REPLACES the last
        sample, because the consumer rebuilds faster than the camera delivers.
        """
        tid = int(track_id)
        if tid <= 0 or v_vec is None:
            return
        v = np.asarray(v_vec, dtype=np.float64).ravel()
        if v.size != 3 or not np.all(np.isfinite(v)):
            return
        tr = self._tr.get(tid)
        if tr is None:
            tr = self._tr[tid] = _Track(t)
        tr.t_last = float(t)
        if tr.ts and abs(float(t) - tr.ts[-1]) <= 1e-4:
            tr.ts[-1], tr.v[-1] = float(t), v     # same frame seen twice
        else:
            tr.ts.append(float(t))
            tr.v.append(v)
        t_min = float(t) - self._window
        while len(tr.ts) > 2 and tr.ts[0] < t_min:
            tr.ts.pop(0)
            tr.v.pop(0)
        while len(tr.ts) > MAX_SAMPLES:
            tr.ts.pop(0)
            tr.v.pop(0)
        self._classify(tr, float(t))

    def _classify(self, tr: _Track, t: float) -> None:
        speed = float(np.linalg.norm(tr.v[-1]))
        if speed > self._rel_speed:
            tr.hot += 1
        else:
            tr.hot = 0
        if tr.hot >= self._rel_hits:
            # Sustained motion: release now and make the track earn a fresh
            # window. Keeping the samples would let the old static history
            # outvote the motion that just started.
            #
            # ``t_first`` is deliberately NOT touched. It is when this IDENTITY
            # was first seen, which is what the age gate asks about, and a
            # moving track is exactly the one whose velocity that gate must
            # eventually admit: resetting it here made a track that keeps
            # moving re-enter the release branch every frame and never age, so
            # its velocity was withheld forever — the opposite of safe.
            tr.static = False
            tr.ts = tr.ts[-1:]
            tr.v = tr.v[-1:]
            tr.hot = 0
            tr.t_quiet = t
            return
        # ENOUGH HISTORY is measured from when the uninterrupted history
        # started, NOT from the oldest surviving sample. The samples are
        # trimmed to the window, so the span between the first and the last of
        # them is at most the window and, with the irregular capture stamps a
        # real camera delivers, essentially never exactly it: testing the span
        # against the window made the classifier reject almost everything. On
        # the DIAG bag it held 0.6 % of the rows whose track was quiet; 23 of
        # its 31 tracks had a median speed under the threshold.
        if len(tr.ts) < self._min_n or (t - tr.t_quiet) < self._window:
            tr.static = False
            return
        V = np.asarray(tr.v)
        tr.static = bool(np.linalg.norm(np.median(V, axis=0)) <= self._drift_rate
                         and float(np.median(np.linalg.norm(V, axis=1))) <= self._speed)

    def prune(self, t: float, idle_s: float = IDLE_DROP_S) -> None:
        """Forget tracks not observed for ``idle_s`` (ids are never reused)."""
        dead = [k for k, tr in self._tr.items() if t - tr.t_last > idle_s]
        for k in dead:
            del self._tr[k]

    # ── queries ──────────────────────────────────────────────────────────────
    def is_static(self, track_id: int) -> bool:
        tr = self._tr.get(int(track_id))
        return bool(tr is not None and tr.static)

    def age_s(self, track_id: int, t: float) -> float:
        """How long THIS consumer has been seeing this identity, in seconds.

        The continuity the frame counters do not measure: ``frames_seen`` says
        how often the track was updated, and at 90 Hz the shipped gate of 8
        frames is 89 ms — shorter than the measured median track life, so it
        admits tracks that never survived anything. 0.0 for an unknown id.
        """
        tr = self._tr.get(int(track_id))
        return 0.0 if tr is None else max(float(t) - tr.t_first, 0.0)

    @property
    def n_static(self) -> int:
        return sum(1 for tr in self._tr.values() if tr.static)

    @property
    def n_tracks(self) -> int:
        return len(self._tr)

    def snapshot(self) -> dict:
        """Diagnostic: ``{track_id: (static, age_s_at_last_observation)}``."""
        return {k: (tr.static, tr.t_last - tr.t_first) for k, tr in self._tr.items()}


class _Cell:
    __slots__ = ('t_first', 't_last', 't_quiet', 'ts', 'v', 'hot', 'static')

    def __init__(self, t: float):
        self.t_first = float(t)
        self.t_last = float(t)
        self.t_quiet = float(t)
        self.ts: list[float] = []
        self.v: list[float] = []
        self.hot = 0
        self.static = False


class StaticSceneVoxels:
    """Which PLACES are the room — the half of the rows that have no identity.

    WHY A PLACE AND NOT A TRACK
    ---------------------------
    :class:`StaticSceneHold` can only answer about a track, and measured on the
    DIAG lab bag the tracker cannot answer for most rows:

    * **48–52 %** of nearest-obstacle rows carry no confirmed ``track_id`` at
      all — the nearest point fell outside every confirmed cluster;
    * the ids that do exist have a **median life of 0.27 s** (18 of 65 reached
      1 s), so even a 1 s window is unreachable for most of them.

    Shortening the window does not rescue it (0.2 s measured WORSE than 1.0 s):
    the problem is not the window, it is that there is nothing to attach a
    history to. A place, on the other hand, is always available — the surface
    point is published on every frame, tracked or not — and the room's defining
    property is that it is in the same place as it was a second ago.

    So the history is keyed on a VOXEL of the obstacle surface point, and the
    question becomes: has everything seen at this place been still for a window?

    WHAT A SAMPLE IS
    ----------------
    The NORMAL closing speed from ``utils/surface_velocity.py``, not the
    tracker's centroid velocity — the untracked rows have no centroid velocity
    (it is exactly 0 on the wire), so the centroid cannot answer here either.
    That also makes the two features compose: the estimator that measures the
    speed is the one that refuses to measure across an argmin hop.

    THE VOXEL SIZE IS A COVERAGE KNOB
    ---------------------------------
    The nearest point SLIDES along a static surface as the arm sweeps past it,
    so too fine a grid splits the history across cells and nothing ever
    accumulates a window — the same failure that an earlier version of the track
    classifier had for a different reason. Too coarse a grid lets a body share a
    cell with the surface behind it. The release below is what bounds the second
    risk: motion anywhere in a cell releases the whole cell within
    ``release_hits`` frames, so a coarse cell is self-correcting, while a cell
    that never fills is simply inert. ``vox=`` on the CBFDIAG line reports the
    coverage this is actually getting (rows held / cells static / cells known).

    Classification, release and the "earn a fresh window after a release" rule
    are identical to :class:`StaticSceneHold`'s, on the scalar:

        static  iff  median(|v_n|) <= speed_m_s  over a full window

    THE WINDOW IS ITS OWN, AND THAT IS MEASURED
    -------------------------------------------
    The speed and release thresholds are shared with the track classifier (the
    same wall must not be scene by one definition and a body by the other), but
    the WINDOW and the SAMPLE COUNT are this class's own, because a place is
    observed far more sparsely than a track: a cell is only sampled when some
    control point's nearest obstacle point lands in it. Measured on
    ``sim_runs/live_p34`` (one ball sweeping through the arm's circle, ground
    truth from ``truth.npz``), rows whose velocity claim this withheld:

        cell window / min samples / voxel     rows held     held ON the ball
        1.0 s   8   0.10 m                      11.5 %             0
        1.0 s   8   0.15 m                      14.6 %             0
        0.5 s   5   0.15 m                      21.8 %             0
        0.5 s   3   0.15 m                      25.0 %             0

    0.5 s / 5 is what ships: it reaches 21.8 % of the rows — against 27.1 % of
    TICKS for the track-keyed hold, on a quite different population — and held
    nothing on the real obstacle in any of the 99 settings swept. A place being
    sampled sparsely is also why the count matters more than the span here.

    WHERE THIS STOPS BELIEVING A SLOW APPROACHER
    -------------------------------------------
    A body crossing a 0.15 m cell at 0.1 m/s is inside it for 1.5 s, longer than
    the window, so unlike a sweeping ball it CAN fill a window in one place and
    the WINDOW does not protect it. ``speed_m_s`` is then the whole of the
    decision, and that is exactly where the boundary falls — measured, per
    ``test_where_the_place_memory_stops_believing_a_slow_approacher``:

        untracked body closing at   velocity withheld
                0.05 m/s                   yes
                0.08 m/s (= speed_m_s)     yes
                0.10 m/s                   no
                0.20 m/s                   no

    So an untracked body closing at or below ``speed_m_s`` DOES lose its
    velocity claim. That is the deliberate trade this class makes and it is the
    same one :class:`StaticSceneHold` already ships: the row keeps its full
    geometric barrier and its gains, and what it loses at 0.08 m/s is 0.8 mm of
    braking margin, 1.6 cm of standoff and the k1 anticipation term. Lower
    ``static_scene_speed_m_s`` to narrow that band; it is shared with the track
    classifier, so it narrows both.

    The boundary is only where it is because the estimate behind it is a fitted
    slope — the second reason ``utils/surface_velocity.py`` had to be rewritten.
    The median-of-differences form it shipped first read 0.02 m/s on a body
    genuinely moving at 0.15, and would have called every one of those cells
    quiet.

    Args:
        voxel_m: [m] cell edge of the position key.
        window_s: [s] uninterrupted history a cell needs before it can be
            called static, and the window the median is taken over.
        speed_m_s: [m/s] largest median normal speed a static cell may show.
        release_speed_m_s: [m/s] speed that, sustained, releases the cell.
        release_hits: consecutive samples above that speed needed to release.
        min_samples: samples the window must hold, whatever its span.
        max_cells: hard cap on the number of remembered places.
    """

    def __init__(self, *, voxel_m: float = 0.15, window_s: float = 0.5,
                 speed_m_s: float = 0.08, release_speed_m_s: float = 0.25,
                 release_hits: int = 3, min_samples: int = 5,
                 max_cells: int = 4096):
        self._vox = max(float(voxel_m), 1e-3)
        self._window = max(float(window_s), 1e-3)
        self._speed = max(float(speed_m_s), 0.0)
        self._rel_speed = max(float(release_speed_m_s), 0.0)
        self._rel_hits = max(int(release_hits), 1)
        self._min_n = max(int(min_samples), 2)
        self._max_cells = max(int(max_cells), 16)
        self._c: dict = {}

    def key(self, p) -> Optional[tuple]:
        """The voxel of a base-frame point, or None when it is unusable."""
        q = np.asarray(p, dtype=np.float64).ravel()
        if q.size != 3 or not np.all(np.isfinite(q)):
            return None
        return tuple(np.floor(q / self._vox).astype(np.int64).tolist())

    # ── bookkeeping ──────────────────────────────────────────────────────────
    def observe(self, p, speed: float, t: float) -> None:
        """Record ``speed`` [m/s] seen at place ``p`` on the frame at ``t``.

        ``speed`` is a MAGNITUDE (the caller passes ``|v_n|``): a cell is a
        place, and what disqualifies a place is motion through it, in either
        direction. A repeated ``t`` replaces the last sample, because the
        consumer rebuilds faster than the camera delivers.
        """
        k = self.key(p)
        if k is None:
            return
        s = float(speed)
        if not np.isfinite(s):
            return
        s = abs(s)
        c = self._c.get(k)
        if c is None:
            if len(self._c) >= self._max_cells:
                self._evict()
            c = self._c[k] = _Cell(t)
        c.t_last = float(t)
        if c.ts and abs(float(t) - c.ts[-1]) <= 1e-4:
            c.ts[-1], c.v[-1] = float(t), s
        else:
            c.ts.append(float(t))
            c.v.append(s)
        t_min = float(t) - self._window
        while len(c.ts) > 2 and c.ts[0] < t_min:
            c.ts.pop(0)
            c.v.pop(0)
        while len(c.ts) > MAX_SAMPLES:
            c.ts.pop(0)
            c.v.pop(0)
        self._classify(c, float(t))

    def _classify(self, c: _Cell, t: float) -> None:
        if c.v[-1] > self._rel_speed:
            c.hot += 1
        else:
            c.hot = 0
        if c.hot >= self._rel_hits:
            # Something is moving through this place. Release it now and make
            # it earn a fresh window, exactly as a released track must.
            c.static = False
            c.ts = c.ts[-1:]
            c.v = c.v[-1:]
            c.hot = 0
            c.t_quiet = t
            return
        if len(c.ts) < self._min_n or (t - c.t_quiet) < self._window:
            c.static = False
            return
        c.static = bool(float(np.median(c.v)) <= self._speed)

    def _evict(self) -> None:
        """Drop the oldest half when the cap is reached (ids are places, and a
        place the arm has left is of no further use)."""
        keep = sorted(self._c.items(), key=lambda kv: kv[1].t_last)
        for k, _ in keep[:max(len(keep) // 2, 1)]:
            del self._c[k]

    def prune(self, t: float, idle_s: float = IDLE_DROP_S) -> None:
        """Forget places not observed for ``idle_s``."""
        dead = [k for k, c in self._c.items() if t - c.t_last > idle_s]
        for k in dead:
            del self._c[k]

    # ── queries ──────────────────────────────────────────────────────────────
    def is_static(self, p) -> bool:
        k = self.key(p)
        if k is None:
            return False
        c = self._c.get(k)
        return bool(c is not None and c.static)

    @property
    def n_static(self) -> int:
        return sum(1 for c in self._c.values() if c.static)

    @property
    def n_cells(self) -> int:
        return len(self._c)

    def snapshot(self) -> dict:
        """Diagnostic: ``{voxel: (static, samples)}``."""
        return {k: (c.static, len(c.v)) for k, c in self._c.items()}


def hold_from_params(P, enabled: Optional[bool] = None) -> Optional[StaticSceneHold]:
    """Build the hold from the parameter namespace, or None when it is off.

    None is what keeps the rows bit-identical: the consumer then never asks a
    question and no code path changes.
    """
    on = bool(getattr(P, 'enable_static_scene_hold', False)) if enabled is None else enabled
    if not on:
        return None
    return StaticSceneHold(
        window_s=float(getattr(P, 'static_scene_window_s', 1.0)),
        drift_m=float(getattr(P, 'static_scene_drift_m', 0.04)),
        speed_m_s=float(getattr(P, 'static_scene_speed_m_s', 0.08)),
        release_speed_m_s=float(getattr(P, 'static_scene_release_speed_m_s', 0.25)),
        release_hits=int(getattr(P, 'static_scene_release_hits', 3)),
        min_samples=int(getattr(P, 'static_scene_min_samples', 8)))


def voxels_from_params(P, enabled: Optional[bool] = None) -> Optional[StaticSceneVoxels]:
    """Build the place-keyed memory from the parameters, or None when off.

    The SPEED and RELEASE thresholds are shared with :func:`hold_from_params` on
    purpose: the two classifiers answer the same question about the same cell,
    one keyed on an identity and one on a place, and tuning those apart would
    mean the same wall could be scene by one definition and a body by the
    other. The WINDOW, the SAMPLE COUNT, the key size and the memory cap are
    its own — see the class docstring for the measurement that sized them.
    """
    on = (bool(getattr(P, 'enable_static_scene_voxels', False))
          if enabled is None else enabled)
    if not on:
        return None
    return StaticSceneVoxels(
        voxel_m=float(getattr(P, 'static_scene_voxel_m', 0.15)),
        window_s=float(getattr(P, 'static_scene_voxel_window_s', 0.5)),
        speed_m_s=float(getattr(P, 'static_scene_speed_m_s', 0.08)),
        release_speed_m_s=float(getattr(P, 'static_scene_release_speed_m_s', 0.25)),
        release_hits=int(getattr(P, 'static_scene_release_hits', 3)),
        min_samples=int(getattr(P, 'static_scene_voxel_min_samples', 5)),
        max_cells=int(getattr(P, 'static_scene_max_cells', 4096)))
