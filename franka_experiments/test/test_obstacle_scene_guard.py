"""The scene guard: a room-sized blob is not an obstacle.

Added in step 6, from what the rosbag replay actually showed rather than from
first principles. With the shipped ``max_depth_m`` of 4 m, a depth camera
pointed at a room returns the room: on rosbag/arm_complex the ~490 obstacle
pixels of every frame form ONE cluster of radius 2.2-2.6 m, and its centroid
alternates between z = 2.28 m and z = 2.57 m as the far patch connects and
disconnects across frames. Differentiated, that flicker is metres per second —
and it gave the tracked estimate a 0.198 m/s noise floor on segments where the
true obstacle speed is zero, against 0.008 m/s for the residual it replaces.

The guard is OFF by default (``max_radius=None``), so this is an opt-in
behaviour change and every pre-guard result is reproducible.

Pure numpy + cv2.
"""

import numpy as np

from franka_experiments.utils.obstacle_clusters import cluster_obstacles

FX = FY = 430.0
CX, CY = 424.0, 240.0


def _wall(u_lo, u_hi, v_lo, v_hi, Z, step=10):
    """A large fronto-parallel patch — the background, in miniature."""
    vg, ug = np.mgrid[v_lo:v_hi:step, u_lo:u_hi:step]
    u, v = ug.ravel().astype(np.int32), vg.ravel().astype(np.int32)
    p = np.empty((u.size, 3), dtype=np.float32)
    p[:, 0] = (u - CX) * Z / FX
    p[:, 1] = (v - CY) * Z / FY
    p[:, 2] = Z
    return p, u, v


def _blob(u0, v0, half, Z, step=10):
    return _wall(u0 - half, u0 + half, v0 - half, v0 + half, Z, step)


def _merge(*b):
    return (np.concatenate([x[0] for x in b]),
            np.concatenate([x[1] for x in b]),
            np.concatenate([x[2] for x in b]))


def test_without_the_guard_the_scene_is_one_giant_cluster():
    """The measured baseline, pinned. Nothing about the default changes."""
    p, u, v = _wall(60, 800, 40, 440, Z=3.0)
    cl = cluster_obstacles(p, u, v, step=10)
    assert len(cl) == 1
    assert cl[0].radius > 1.0, cl[0].radius


def test_the_guard_drops_a_room_sized_cluster():
    p, u, v = _wall(60, 800, 40, 440, Z=3.0)
    assert cluster_obstacles(p, u, v, step=10, max_radius=0.6) == []


def test_the_guard_keeps_a_person_sized_one():
    """A cluster wider than a person is not a person; one narrower than the
    limit must survive untouched, radius and centroid included."""
    p, u, v = _blob(400, 240, 60, Z=1.2)
    kept = cluster_obstacles(p, u, v, step=10, max_radius=0.6)
    plain = cluster_obstacles(p, u, v, step=10)
    assert len(kept) == 1 and len(plain) == 1
    assert kept[0].radius == plain[0].radius
    assert np.array_equal(kept[0].centroid_cam, plain[0].centroid_cam)


def test_the_guard_keeps_the_obstacle_and_drops_the_background():
    """The case the replay is made of: a person-sized object in front of a
    room-sized background, separated in the image."""
    scene = _merge(_wall(60, 360, 40, 440, Z=3.0), _blob(600, 240, 50, Z=1.2))
    assert len(cluster_obstacles(*scene, step=10)) == 2
    kept = cluster_obstacles(*scene, step=10, max_radius=0.6)
    assert len(kept) == 1
    assert kept[0].centroid_cam[2] < 1.5, 'the near object must be the survivor'


def test_dropping_everything_is_the_SAFE_failure():
    """The whole reason a hard drop is acceptable. No cluster means no track,
    no track means the message's zero defaults, and the zero defaults mean the
    barrier falls back to exactly the behaviour it has today. A guard that
    instead SPLIT the blob arbitrarily would invent an obstacle position, which
    is the unsafe direction."""
    p, u, v = _wall(60, 800, 40, 440, Z=3.0)
    assert cluster_obstacles(p, u, v, step=10, max_radius=0.6) == []


def test_the_guard_is_monotone_in_its_threshold():
    """No threshold may ADD a cluster: this is a filter, and a non-monotone one
    would mean the limit is interacting with the labelling instead of only
    selecting from its output."""
    scene = _merge(_wall(60, 360, 40, 440, Z=3.0),
                   _blob(600, 200, 50, Z=1.2),
                   _blob(600, 380, 20, Z=0.9))
    counts = [len(cluster_obstacles(*scene, step=10, max_radius=r))
              for r in (0.02, 0.05, 0.1, 0.3, 0.6, 1.0, 5.0)]
    assert counts == sorted(counts), counts
    assert counts[-1] == len(cluster_obstacles(*scene, step=10))


def test_none_reproduces_the_pre_guard_result_exactly():
    scene = _merge(_wall(60, 360, 40, 440, Z=3.0), _blob(600, 240, 50, Z=1.2))
    a = cluster_obstacles(*scene, step=10)
    b = cluster_obstacles(*scene, step=10, max_radius=None)
    assert len(a) == len(b)
    for x, y in zip(a, b):
        assert np.array_equal(x.centroid_cam, y.centroid_cam)
        assert x.radius == y.radius and x.n_points == y.n_points
