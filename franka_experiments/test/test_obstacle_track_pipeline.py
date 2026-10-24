"""End-to-end pipeline: a synthetic depth sequence → tracked velocity on every
control point of a MultiLinkDistance.

Launch-free by construction. ``real_time_distance`` hosts the tracker but owns
none of its logic: that is ``ObstacleTrackPipeline`` plus the free function
``perception_msgs.annotate_track_fields``, and both are driven here directly, so
the whole decision chain — cluster, associate, track, match a control point to a
track, write the message fields — is exercised without a camera, a clock, TF or
a running executor.

The sequence is a blob approaching the robot at a known constant velocity, so
the assertions are against a number that is known exactly rather than against
whatever the pipeline happened to produce.
"""

import numpy as np
import pytest

from franka_experiments.utils.distance_engine import ObstacleCloud
from franka_experiments.utils.obstacle_track_pipeline import ObstacleTrackPipeline

FX = FY = 430.0
CX, CY = 424.0, 240.0
DT = 1.0 / 30.0
STEP = 10                       # the pixel_step the real config uses

# A camera looking down the base +x axis: base = R @ cam + t. Deliberately NOT
# the identity, so a frame confusion anywhere in the chain shows up as a
# velocity pointing the wrong way rather than as a coincidentally-right answer.
R_BASE = np.array([[0.0, 0.0, 1.0],
                   [-1.0, 0.0, 0.0],
                   [0.0, -1.0, 0.0]], dtype=np.float64)
T_BASE = np.array([1.6, 0.0, 0.8])


def _cloud(centre_cam, half_px=9, stamp=None):
    """An ObstacleCloud holding one square blob centred on `centre_cam`.

    The pixels are laid out on the same strided ROI grid DistanceEngine walks,
    so the cluster module's `step` handling is exercised, not bypassed.
    """
    Z = float(centre_cam[2])
    u0 = int(round(CX + centre_cam[0] * FX / Z))
    v0 = int(round(CY + centre_cam[1] * FY / Z))
    lo_u = (u0 - half_px * STEP) - (u0 - half_px * STEP) % STEP
    lo_v = (v0 - half_px * STEP) - (v0 - half_px * STEP) % STEP
    vg, ug = np.mgrid[lo_v:lo_v + (2 * half_px + 1) * STEP:STEP,
                      lo_u:lo_u + (2 * half_px + 1) * STEP:STEP]
    u, v = ug.ravel().astype(np.int32), vg.ravel().astype(np.int32)
    p = np.empty((u.size, 3), dtype=np.float32)
    p[:, 0] = (u - CX) * Z / FX
    p[:, 1] = (v - CY) * Z / FY
    p[:, 2] = Z
    # Recentre exactly on the requested point: the pixel rounding above would
    # otherwise put a few millimetres of quantisation into the "known" truth.
    p[:, :3] += (np.asarray(centre_cam, np.float32)
                 - np.array([p[:, 0].mean(), p[:, 1].mean(), Z], np.float32))
    return ObstacleCloud(p_cam=p, u=u, v=v, step=STEP, stamp=stamp)


def _to_base(p_cam):
    return R_BASE @ np.asarray(p_cam, dtype=np.float64) + T_BASE


def _approach(n=20, v_cam=(0.0, 0.0, -0.5), p0=(0.0, 0.0, 1.6)):
    """(pipeline, centres_cam) after running a constant-velocity approach."""
    pipe = ObstacleTrackPipeline(min_cluster_points=10)
    centres = []
    for k in range(n):
        c = np.asarray(p0) + np.asarray(v_cam) * (k * DT)
        centres.append(c)
        pipe.update(_cloud(c, stamp=k * DT), R_BASE, T_BASE, stamp=k * DT)
    return pipe, centres


# ── The pipeline on a synthetic sequence ────────────────────────────────────

def test_a_moving_blob_produces_one_confirmed_track_with_the_right_velocity():
    """The whole chain in one assertion. The blob closes at 0.5 m/s along the
    camera's +Z, which the fixture's extrinsic maps to -x in base — so the
    estimate must come out as base-frame velocity, not camera-frame."""
    pipe, _ = _approach(v_cam=(0.0, 0.0, -0.5))
    conf = pipe.tracker.confirmed_tracks()
    assert len(conf) == 1
    v_base_true = R_BASE @ np.array([0.0, 0.0, -0.5])
    assert np.linalg.norm(conf[0].velocity - v_base_true) < 0.05, conf[0].velocity


def test_a_static_blob_produces_a_track_with_near_zero_velocity():
    pipe, _ = _approach(v_cam=(0.0, 0.0, 0.0))
    conf = pipe.tracker.confirmed_tracks()
    assert len(conf) == 1
    assert np.linalg.norm(conf[0].velocity) < 0.05


def test_a_control_point_is_matched_to_the_track_covering_its_obstacle_point():
    """point -> cluster -> track, never "nearest track centre": a large
    cluster's far edge can be closer to another track's centre than to its own."""
    pipe, centres = _approach()
    tid, seen, v, P = pipe.velocity_for_point(_to_base(centres[-1]))
    assert tid != 0 and seen >= 3
    assert np.linalg.norm(v - R_BASE @ np.array([0.0, 0.0, -0.5])) < 0.05
    assert P.shape == (3, 3) and np.trace(P) > 0.0


def test_a_control_point_far_from_every_cluster_gets_the_no_track_defaults():
    """The fallback that keeps this whole feature safe: no match means the
    message's documented zeros, which the consumer treats as "contribute
    nothing" and which reduce it to today's behaviour."""
    pipe, _ = _approach()
    tid, seen, v, P = pipe.velocity_for_point(np.array([-5.0, -5.0, -5.0]))
    assert tid == 0 and seen == 0
    assert np.array_equal(v, np.zeros(3))
    assert np.array_equal(P, np.zeros((3, 3)))


def test_a_tentative_track_is_not_yet_visible_to_a_control_point():
    """Two frames is not a track. Publishing one would put a velocity dominated
    by its own initial uncertainty on the wire, at exactly the moment a consumer
    cannot tell it apart from a real one."""
    pipe = ObstacleTrackPipeline()
    c = np.array([0.0, 0.0, 1.5])
    for k in range(2):
        pipe.update(_cloud(c, stamp=k * DT), R_BASE, T_BASE, stamp=k * DT)
    assert pipe.velocity_for_point(_to_base(c))[0] == 0


def test_two_blobs_give_two_tracks_and_each_control_point_gets_its_own():
    """The case the scalar residual cannot express at all: two obstacles moving
    in OPPOSITE directions. A single per-control-point estimate has no way to
    say which one a given control point is looking at."""
    pipe = ObstacleTrackPipeline()
    a0, b0 = np.array([-0.6, 0.0, 1.6]), np.array([0.6, 0.0, 1.6])
    va, vb = np.array([0.0, 0.0, -0.5]), np.array([0.0, 0.0, +0.5])
    for k in range(20):
        t = k * DT
        ca, cb = a0 + va * t, b0 + vb * t
        cl = _cloud(ca, half_px=6, stamp=t)
        cr = _cloud(cb, half_px=6, stamp=t)
        merged = ObstacleCloud(
            p_cam=np.concatenate([cl.p_cam, cr.p_cam]),
            u=np.concatenate([cl.u, cr.u]), v=np.concatenate([cl.v, cr.v]),
            step=STEP, stamp=t)
        pipe.update(merged, R_BASE, T_BASE, stamp=t)
        last = (ca, cb)
    assert len(pipe.tracker.confirmed_tracks()) == 2
    id_a, _, v_a, _ = pipe.velocity_for_point(_to_base(last[0]))
    id_b, _, v_b, _ = pipe.velocity_for_point(_to_base(last[1]))
    assert id_a != 0 and id_b != 0 and id_a != id_b
    assert np.dot(v_a, R_BASE @ va) > 0.0 and np.dot(v_b, R_BASE @ vb) > 0.0
    assert np.dot(v_a, v_b) < 0.0, 'the two must be reported as opposing'


def test_an_empty_frame_coasts_rather_than_freezing_the_estimate():
    """Perception seeing nothing is a measurement, not a gap. The tracks must
    advance (and eventually die), not sit frozen reporting a stale velocity."""
    pipe, centres = _approach()
    trk = pipe.tracker.confirmed_tracks()[0]
    p0 = trk.position.copy()
    pipe.update(None, R_BASE, T_BASE, stamp=20 * DT)
    assert not np.array_equal(trk.position, p0), 'the track did not coast'
    assert pipe.velocity_for_point(_to_base(centres[-1]))[0] == 0, \
        'a coasted frame has no clusters, so no point can be matched'


def test_a_missing_stamp_falls_back_to_the_nominal_period():
    """Without the fallback a stampless frame would freeze every filter
    (predict ignores dt <= 0) and the tracks would report the velocity they had
    when the clock broke."""
    pipe = ObstacleTrackPipeline(default_dt=DT)
    for k in range(12):
        c = np.array([0.0, 0.0, 1.6 - 0.5 * k * DT])
        pipe.update(_cloud(c), R_BASE, T_BASE, stamp=None)
    conf = pipe.tracker.confirmed_tracks()
    assert len(conf) == 1
    assert np.linalg.norm(conf[0].velocity - R_BASE @ np.array([0, 0, -0.5])) < 0.1


def test_reset_clears_tracks_and_the_clock():
    pipe, centres = _approach()
    pipe.reset()
    assert pipe.tracker.tracks == []
    assert pipe.velocity_for_point(_to_base(centres[-1]))[0] == 0


# ── Message annotation ──────────────────────────────────────────────────────

franka_msgs = pytest.importorskip('franka_msgs.msg')


def _mld(points_base, valid=True):
    msg = franka_msgs.MultiLinkDistance()
    links = []
    for p in points_base:
        ld = franka_msgs.LinkDistance()
        ld.robot_link_name = 'fr3_link5'
        ld.distance = 0.2
        ld.valid = valid
        ld.closest_point_human.x = float(p[0])
        ld.closest_point_human.y = float(p[1])
        ld.closest_point_human.z = float(p[2])
        links.append(ld)
    msg.links = links
    return msg


def test_annotate_links_fills_the_track_fields_of_a_matched_entry():
    from franka_experiments.utils.perception_msgs import (
        annotate_track_fields as annotate_links)

    pipe, centres = _approach()
    msg = _mld([_to_base(centres[-1])])
    assert annotate_links(msg, pipe) == 1
    ld = msg.links[0]
    assert ld.track_id != 0 and ld.frames_seen >= 3
    v = np.array([ld.obstacle_velocity.x, ld.obstacle_velocity.y,
                  ld.obstacle_velocity.z])
    assert np.linalg.norm(v - R_BASE @ np.array([0.0, 0.0, -0.5])) < 0.05
    assert len(ld.velocity_covariance) == 9


def test_annotate_links_leaves_an_unmatched_entry_at_the_safe_defaults():
    from franka_experiments.utils.perception_msgs import (
        annotate_track_fields as annotate_links)

    pipe, _ = _approach()
    msg = _mld([np.array([-5.0, -5.0, -5.0])])
    assert annotate_links(msg, pipe) == 0
    ld = msg.links[0]
    assert ld.track_id == 0 and ld.frames_seen == 0
    assert np.array_equal(np.asarray(ld.velocity_covariance), np.zeros(9))


def test_annotate_links_skips_invalid_entries():
    """An invalid entry has no meaningful closest_point_human; annotating it
    would attach a velocity to a point that was never measured."""
    from franka_experiments.utils.perception_msgs import (
        annotate_track_fields as annotate_links)

    pipe, centres = _approach()
    msg = _mld([_to_base(centres[-1])], valid=False)
    assert annotate_links(msg, pipe) == 0
    assert msg.links[0].track_id == 0


def test_annotation_never_touches_any_pre_existing_field():
    """The node is a PASS-THROUGH with annotation. If it could change a
    distance, a direction or a validity flag it would be inside the safety path,
    which nothing before step 8 is allowed to be."""
    from franka_experiments.utils.perception_msgs import (
        annotate_track_fields as annotate_links)

    pipe, centres = _approach()
    msg = _mld([_to_base(centres[-1])])
    ld = msg.links[0]
    ld.direction.x, ld.direction.y, ld.direction.z = 0.0, 0.0, 1.0
    ld.confidence, ld.zone = 1.0, 'danger'
    before = (ld.robot_link_name, ld.distance, ld.valid, ld.confidence, ld.zone,
              ld.direction.x, ld.direction.y, ld.direction.z,
              ld.closest_point_human.x, ld.closest_point_human.y,
              ld.closest_point_human.z)
    annotate_links(msg, pipe)
    after = (ld.robot_link_name, ld.distance, ld.valid, ld.confidence, ld.zone,
             ld.direction.x, ld.direction.y, ld.direction.z,
             ld.closest_point_human.x, ld.closest_point_human.y,
             ld.closest_point_human.z)
    assert before == after
