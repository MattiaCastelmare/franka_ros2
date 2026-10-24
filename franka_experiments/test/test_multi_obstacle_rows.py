"""perception.multi_obstacle_k — top-k obstacle points per control point.

Pure numpy (+ scipy for the labelling): no ROS. The franka_msgs/geometry_msgs
message classes are stubbed when the real ones are not importable.
"""
import sys
import types

import numpy as np
import pytest


def _install_msg_stubs():
    try:
        import franka_msgs.msg  # noqa: F401
        import geometry_msgs.msg  # noqa: F401
        return
    except ImportError:
        pass

    class _Msg:
        F: dict = {}

        def __init__(self, **kw):
            for k, f in self.F.items():
                setattr(self, k, f())
            for k, v in kw.items():
                setattr(self, k, v)

    def mk(name, **fields):
        return type(name, (_Msg,), {'F': fields})

    Point = mk('Point', x=float, y=float, z=float)
    Vector3 = mk('Vector3', x=float, y=float, z=float)
    Header = mk('Header', stamp=lambda: None, frame_id=str)
    fm = types.ModuleType('franka_msgs.msg')
    gm = types.ModuleType('geometry_msgs.msg')
    gm.Point, gm.Vector3 = Point, Vector3
    fm.LinkDistance = mk(
        'LinkDistance', robot_link_name=str, closest_point_robot=Point,
        closest_point_human=Point, direction=Vector3, distance=float, valid=bool,
        confidence=float, zone=str, track_id=int, frames_seen=int,
        obstacle_velocity=Vector3, velocity_covariance=lambda: [0.0] * 9,
        obstacle_acceleration=Vector3, position_covariance=lambda: [0.0] * 9,
        position_velocity_covariance=lambda: [0.0] * 9)
    fm.MultiLinkDistance = mk('MultiLinkDistance', header=Header, links=list)
    fm.HumanRobotDistance = mk(
        'HumanRobotDistance', header=Header, robot_link_name=str, valid=bool,
        distance=float, closest_point_robot=Point, direction=Vector3, zone=str,
        confidence=float)
    fm.MultiDistance = mk('MultiDistance', header=Header, distances=list)
    sys.modules.setdefault('franka_msgs', types.ModuleType('franka_msgs'))
    sys.modules.setdefault('geometry_msgs', types.ModuleType('geometry_msgs'))
    sys.modules['franka_msgs.msg'] = fm
    sys.modules['geometry_msgs.msg'] = gm


_install_msg_stubs()

from franka_experiments.utils.distance_engine import DistanceEngine  # noqa: E402
from franka_experiments.utils.obstacle_track_pipeline import (  # noqa: E402
    NO_TRACK, ObstacleTrackPipeline, TrackInfo)
from franka_experiments.utils.perception_msgs import (  # noqa: E402
    annotate_track_fields, build_cp_messages, labelled_links)

H, W, CX, CY, F = 60, 80, 40.0, 30.0, 100.0
LINKS = ['fr3_link5', 'fr3_link7']


def _cp(p, seg=0, c=0, link='fr3_link5'):
    return {'point': np.array(p, dtype=np.float64), 'seg_idx': seg, 'cp_idx': c,
            'radius': 0.05, 'start_link': 'x', 'end_link': link}


def _engine(k=None, **kw):
    cfg = {'min_depth_m': 0.1, 'max_depth_m': 4.0}
    if k is not None:
        cfg['multi_obstacle_k'] = k
    cfg.update(kw)
    return DistanceEngine(cfg)


def _compute(eng, depth, cps, stamp=None):
    res, n = eng.compute(depth, np.float32(CX), np.float32(CY),
                         np.float32(1 / F), np.float32(1 / F),
                         np.eye(3, dtype=np.float32), np.zeros(3, np.float32), cps,
                         np.array([0, W]), np.array([0, H]), 1, None,
                         frame_stamp=stamp)
    return res, n


def _stamp():
    try:
        from builtin_interfaces.msg import Time
        return Time()
    except ImportError:
        return None


def _msgs(res, n, **kw):
    return build_cp_messages(res, n, _stamp(), 'fr3_link0', LINKS,
                             {'min_thresh': 0.0, 'max_thresh': 5.0}, 2.0, {}, **kw)


def _row(ld):
    p, h, v = ld.closest_point_robot, ld.closest_point_human, ld.direction
    return (ld.robot_link_name, (p.x, p.y, p.z), (h.x, h.y, h.z), (v.x, v.y, v.z),
            ld.distance, ld.valid, ld.zone)


# ── Scenes ───────────────────────────────────────────────────────────────────

def _regression_depth():
    d = np.zeros((H, W), np.uint16)
    d[10:18, 10:20] = 1000
    d[35:45, 50:65] = 1200
    d[50:55, 25:45] = 900
    return d


REG_CPS = [_cp([0.0, 0.0, 0.5], 0, 0, 'fr3_link5'),
           _cp([0.1, 0.05, 0.6], 0, 1, 'fr3_link5'),
           _cp([-0.1, -0.1, 0.7], 1, 0, 'fr3_link7')]

# MultiLinkDistance.links of REG scene, dumped from the pre-change code (HEAD
# b7cc621) with the same stubs. Exact float equality on purpose.
GOLDEN = [
    ('fr3_link5', (0.0, 0.0, 0.5), (0.0, 0.18000000715255737, 0.9000000357627869),
     (0.0, -0.41036465764045715, -0.911921501159668), 0.38863426446914673, True, 'unknown'),
    ('fr3_link5', (0.1, 0.05, 0.6), (0.036000002175569534, 0.18000000715255737, 0.9000000357627869),
     (0.19209951162338257, -0.3902021646499634, -0.9004665613174438), 0.28316062688827515, True, 'unknown'),
    ('fr3_link7', (-0.1, -0.1, 0.7), (-0.20999999344348907, -0.12999999523162842, 1.0),
     (0.342747300863266, 0.09347652643918991, -0.9347654581069946), 0.27093613147735596, True, 'unknown'),
]


def _two_blobs():
    """Two separate blobs, mirror images about the optical axis."""
    d = np.zeros((H, W), np.uint16)
    d[27:34, 22:27] = 1000
    d[27:34, 54:59] = 1000
    return d


def _four_blobs():
    d = np.zeros((H, W), np.uint16)
    d[5:12, 5:15] = 800
    d[5:12, 60:75] = 1100
    d[45:55, 5:15] = 1500
    d[45:55, 60:75] = 2000
    return d


# ── Tests ────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize('k', [None, 1])
def test_k1_multilinkdistance_identical_to_today(k):
    res, n = _compute(_engine(k), _regression_depth(), REG_CPS)
    _, mld = _msgs(res, n)
    assert [_row(ld) for ld in mld.links] == GOLDEN
    assert [lbl for lbl, _ in labelled_links(mld)] == \
        ['fr3_link5#0', 'fr3_link5#1', 'fr3_link7#0']
    for r in res:
        assert r.extras == [] and r.cluster_id == -1
        (hit,) = r.obstacles
        assert (hit.distance, hit.point is r.closest_obstacle_point,
                hit.direction is r.direction) == (r.distance, True, True)


def test_k2_rank0_rows_are_the_k1_rows():
    res, n = _compute(_engine(2), _regression_depth(), REG_CPS)
    _, mld = _msgs(res, n)
    primaries = [_row(ld) for lbl, ld in labelled_links(mld) if '.' not in lbl]
    assert primaries == GOLDEN
    assert len(mld.links) > len(GOLDEN)


def test_two_blobs_equidistant_k2_gives_two_rows():
    cps = [_cp([0.0, 0.0, 0.5])]
    res, n = _compute(_engine(2), _two_blobs(), cps)
    (r,) = res
    hits = r.obstacles
    assert len(hits) == 2
    assert hits[0].cluster_id != hits[1].cluster_id
    assert hits[0].distance == pytest.approx(hits[1].distance, abs=1e-7)
    assert abs(float(np.dot(hits[0].direction, hits[1].direction))) < 0.99

    _, mld, cids = _msgs(res, n, return_cluster_ids=True)
    assert [lbl for lbl, _ in labelled_links(mld)] == ['fr3_link5#0', 'fr3_link5#0.1']
    assert cids == [hits[0].cluster_id, hits[1].cluster_id]
    assert all(ld.valid for ld in mld.links)


def test_same_input_k1_one_row_the_nearest():
    cps = [_cp([0.0, 0.0, 0.5])]
    res1, n = _compute(_engine(1), _two_blobs(), cps)
    res2, _ = _compute(_engine(2), _two_blobs(), cps)
    (hit,) = res1[0].obstacles
    near = min(res2[0].obstacles, key=lambda h: h.distance)
    assert hit.distance == near.distance
    np.testing.assert_array_equal(hit.point, res2[0].obstacles[0].point)
    _, mld = _msgs(res1, n)
    assert len(mld.links) == 1


def test_two_points_of_one_cluster_deduplicated():
    d = np.zeros((H, W), np.uint16)
    d[27:34, 22:59] = 1000              # one bar spanning both sides of the CP
    res, _ = _compute(_engine(3), d, [_cp([0.0, 0.0, 0.5])])
    assert len(res[0].obstacles) == 1


def test_extras_beyond_horizon_dropped():
    cps = [_cp([0.0, 0.0, 0.4])]
    full, _ = _compute(_engine(4), _four_blobs(), cps)
    ds = [h.distance for h in full[0].obstacles]
    assert len(ds) == 4 and ds == sorted(ds)
    horizon = 0.5 * (ds[1] + ds[2])
    cut, _ = _compute(_engine(4, multi_obstacle_horizon=horizon), _four_blobs(), cps)
    assert [h.distance for h in cut[0].obstacles] == ds[:2]


def test_max_rows_keeps_nearest_deterministically():
    cps = [_cp([0.0, 0.0, 0.4], 0, 0, 'fr3_link5'),
           _cp([0.05, 0.05, 0.5], 1, 0, 'fr3_link7')]
    full, _ = _compute(_engine(4), _four_blobs(), cps)
    extras = sorted((h.distance, r.seg_idx, r.cp_idx, j)
                    for r in full for j, h in enumerate(r.extras))
    assert len(extras) == 6

    max_rows = 4                        # 2 primaries + the 2 nearest extras
    runs = []
    for _ in range(2):
        res, n = _compute(_engine(4, multi_obstacle_max_rows=max_rows),
                          _four_blobs(), cps)
        _, mld, cids = _msgs(res, n, return_cluster_ids=True)
        runs.append(([_row(ld) for ld in mld.links], cids,
                     [lbl for lbl, _ in labelled_links(mld)]))
        kept = sorted((h.distance, r.seg_idx, r.cp_idx)
                      for r in res for h in r.extras)
        assert kept == [e[:3] for e in extras[:max_rows - len(cps)]]
        assert [r.distance for r in res] == [r.distance for r in full]
        assert len(mld.links) == max_rows
    assert runs[0] == runs[1]


class _FakePipeline:
    """Track by cluster label; the point lookup deliberately returns a WRONG
    track, so any entry that falls back to it is caught."""

    def __init__(self, tracks):
        self.tracks = tracks

    def track_info_for_cluster(self, label):
        tid = self.tracks.get(label)
        if tid is None:
            return NO_TRACK
        return TrackInfo(tid, 5, np.full(3, float(tid)), np.eye(3), np.zeros(3),
                         np.zeros((3, 3)), np.zeros((3, 3)))

    def track_info_for_point(self, p):
        return TrackInfo(999, 5, np.zeros(3), np.eye(3), np.zeros(3),
                         np.zeros((3, 3)), np.zeros((3, 3)))


def test_each_row_gets_its_own_cluster_track():
    cps = [_cp([0.0, 0.0, 0.5])]
    res, n = _compute(_engine(2), _two_blobs(), cps)
    _, mld, cids = _msgs(res, n, return_cluster_ids=True)
    pipe = _FakePipeline({cids[0]: 11, cids[1]: 22})
    assert annotate_track_fields(mld, pipe, cluster_ids=cids) == 2
    assert [ld.track_id for ld in mld.links] == [11, 22]
    assert [ld.obstacle_velocity.x for ld in mld.links] == [11.0, 22.0]
    # Without cluster ids: the legacy point lookup, unchanged.
    _, mld2 = _msgs(res, n)
    annotate_track_fields(mld2, pipe)
    assert [ld.track_id for ld in mld2.links] == [999, 999]


def test_shared_labels_reach_the_real_tracker():
    cps = [_cp([0.0, 0.0, 0.5])]
    eng = _engine(2, export_obstacle_cloud=True)
    pipe = ObstacleTrackPipeline()
    for f in range(6):
        stamp = f / 30.0
        res, n = _compute(eng, _two_blobs(), cps, stamp=stamp)
        assert eng.last_obstacle_cloud.labels is not None
        pipe.update(eng.last_obstacle_cloud, np.eye(3), np.zeros(3), stamp=stamp)
    _, mld, cids = _msgs(res, n, return_cluster_ids=True)
    assert annotate_track_fields(mld, pipe, cluster_ids=cids) == 2
    tids = [ld.track_id for ld in mld.links]
    assert tids[0] != tids[1] and min(tids) > 0
    for cid, tid in zip(cids, tids):
        assert pipe.track_for_cluster(cid).track_id == tid
