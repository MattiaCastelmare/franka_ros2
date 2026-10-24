"""Phase 3 plumbing: the raw range `range_m` from camera pixel to CBF row.

Covers the whole chain a `sensor_range_uncertainty` tightening depends on,
one hop at a time:

    DistanceEngine (raw camera-frame Z)
        -> ControlPointResult.range_m
        -> perception_msgs.build_cp_messages -> LinkDistance.range_m
        -> CBFSafetyFilter._range_field -> Obstacle.range_m

The actual barrier math is covered by test_cbf_sensor_range_uncertainty.py;
this file only proves the number that reaches the barrier is the right one
and survives (or is correctly dropped by) every hop.
"""

import sys
import types

import numpy as np
import pytest

from franka_experiments.utils.distance_engine import DistanceEngine


# ── Stub ROS messages so this runs without a sourced workspace too ──────────
# Same approach as test_distance_surface_semantics.py: only stub when the real
# franka_msgs is not importable, and do so exactly once for the whole pytest
# session (colcon test runs test/ in one process).

def _install_msg_stubs():
    try:
        import franka_msgs.msg          # noqa: F401
        import geometry_msgs.msg        # noqa: F401
        return False
    except ImportError:
        pass

    class _Bag:
        def __init__(self, **kw):
            for k, v in kw.items():
                setattr(self, k, v)

    class _Header(_Bag):
        def __init__(self):
            super().__init__(stamp=None, frame_id='')

    def _msg(name, **defaults):
        def __init__(self):
            _Bag.__init__(self, **{k: (v() if callable(v) else v)
                                   for k, v in defaults.items()})
        return type(name, (_Bag,), {'__init__': __init__})

    Point   = type('Point',   (_Bag,), {})
    Vector3 = type('Vector3', (_Bag,), {})

    franka = types.ModuleType('franka_msgs')
    fmsg   = types.ModuleType('franka_msgs.msg')
    fmsg.LinkDistance = _msg(
        'LinkDistance', robot_link_name='',
        closest_point_robot=lambda: Point(x=0.0, y=0.0, z=0.0),
        closest_point_human=lambda: Point(x=0.0, y=0.0, z=0.0),
        direction=lambda: Vector3(x=0.0, y=0.0, z=0.0),
        distance=0.0, valid=False, confidence=0.0, zone='', range_m=0.0)
    fmsg.HumanRobotDistance = _msg(
        'HumanRobotDistance', header=_Header, robot_link_name='',
        closest_point_robot=lambda: Point(x=0.0, y=0.0, z=0.0),
        closest_point_human=lambda: Point(x=0.0, y=0.0, z=0.0),
        direction=lambda: Vector3(x=0.0, y=0.0, z=0.0),
        distance=0.0, valid=False, confidence=0.0, zone='')
    fmsg.MultiDistance     = _msg('MultiDistance',     header=_Header, distances=list)
    fmsg.MultiLinkDistance = _msg('MultiLinkDistance', header=_Header, links=list)
    franka.msg = fmsg

    geo  = types.ModuleType('geometry_msgs')
    gmsg = types.ModuleType('geometry_msgs.msg')
    gmsg.Point, gmsg.Vector3 = Point, Vector3
    geo.msg = gmsg

    sys.modules.update({'franka_msgs': franka, 'franka_msgs.msg': fmsg,
                        'geometry_msgs': geo, 'geometry_msgs.msg': gmsg})
    return True


_STUBBED = _install_msg_stubs()

if _STUBBED:
    _STAMP = None
else:
    from builtin_interfaces.msg import Time
    _STAMP = Time()

from franka_experiments.utils.perception_msgs import build_cp_messages  # noqa: E402


# ── Synthetic scene (identity extrinsics, same convention as
#    test_distance_surface_semantics.py so a reader can compare the two) ────

W = H  = 320
FX = FY = 400.0
CX = CY = 160.0
STEP    = 10
WALL_Z  = 1.0
RADIUS  = 0.05

R_EYE  = np.eye(3, dtype=np.float32)
T_ZERO = np.zeros(3, dtype=np.float32)


def _wall_depth(z_m=WALL_Z):
    return np.full((H, W), int(round(z_m * 1000.0)), dtype=np.uint16)


def _empty_depth():
    """No pixel survives the depth-range gate (min_depth_m = 0.15)."""
    return np.zeros((H, W), dtype=np.uint16)


def _cp(point, radius=RADIUS, seg_idx=7, cp_idx=0,
        start_link='fr3_link7', end_link='fr3_link8'):
    return {'point': np.asarray(point, dtype=np.float64), 'seg_idx': seg_idx,
            'cp_idx': cp_idx, 'radius': radius,
            'start_link': start_link, 'end_link': end_link}


def _compute(engine, control_points, depth, R_base=R_EYE, t_base=T_ZERO,
            frame_stamp=None):
    results, _ = engine.compute(
        depth=depth, cx_f32=np.float32(CX), cy_f32=np.float32(CY),
        fx_inv_f32=np.float32(1.0 / FX), fy_inv_f32=np.float32(1.0 / FY),
        R_base_f32=R_base, t_base_f32=t_base,
        control_points=control_points,
        x=np.array([0, W]), y=np.array([0, H]), step=STEP,
        search_exclusion_mask=None, frame_stamp=frame_stamp)
    return results


# ═════════════════════════════════════════════════════════════════════════════
#  DistanceEngine: range_m is the RAW camera-frame Z, not the surface gap
# ═════════════════════════════════════════════════════════════════════════════

def test_range_m_is_the_raw_camera_depth_not_the_surface_gap():
    """CP 0.5 m in front of a wall at 1.0 m: distance (surface gap) is
    0.5 - RADIUS, but range_m is the wall's own depth, 1.0 m."""
    engine = DistanceEngine({'min_depth_m': 0.15, 'max_depth_m': 4.0, 'lpf_alpha': 0.0})
    r = _compute(engine, [_cp([0.0, 0.0, 0.5])], _wall_depth())[0]
    assert abs(r.distance - (0.5 - RADIUS)) < 1e-5
    assert abs(r.range_m - WALL_Z) < 1e-5


def test_range_m_is_camera_frame_not_base_frame():
    """A non-identity extrinsic changes closest_obstacle_point (base frame)
    but must NOT change range_m — it is read straight off p_cam, before the
    R_base/t_base transform, and any base-frame recomputation would
    reintroduce the extrinsic's own calibration bias."""
    engine = DistanceEngine({'min_depth_m': 0.15, 'max_depth_m': 4.0, 'lpf_alpha': 0.0})
    t_shifted = np.array([0.3, -0.2, 0.1], dtype=np.float32)
    r = _compute(engine, [_cp([0.3, -0.2, 0.6])], _wall_depth(),
                t_base=t_shifted)[0]
    assert abs(r.range_m - WALL_Z) < 1e-5
    assert not np.allclose(r.closest_obstacle_point, [0.0, 0.0, WALL_Z], atol=1e-3)


def test_range_m_is_none_when_no_obstacle_is_seen():
    engine = DistanceEngine({'min_depth_m': 0.15, 'max_depth_m': 4.0, 'lpf_alpha': 0.0})
    r = _compute(engine, [_cp([0.0, 0.0, 0.5])], _empty_depth())[0]
    assert not np.isfinite(r.distance)
    assert r.range_m is None


def test_range_m_survives_the_lpf_normal_path():
    """With lpf_alpha > 0 the distance is smoothed on recovery; range_m must
    still carry THIS frame's raw pixel, unaffected by the smoothing (same
    convention as closest_obstacle_point/closest_pixel, which _mk_result
    already always carries from the current raw result)."""
    engine = DistanceEngine({'min_depth_m': 0.15, 'max_depth_m': 4.0, 'lpf_alpha': 0.5})
    _compute(engine, [_cp([0.0, 0.0, 0.5])], _wall_depth(), frame_stamp=0.0)
    r2 = _compute(engine, [_cp([0.0, 0.0, 0.5])], _wall_depth(1.2),
                 frame_stamp=1.0 / 30.0)[0]
    assert abs(r2.range_m - 1.2) < 1e-5


def test_range_m_is_dropped_on_hold():
    """Frame 1 sees the wall; frame 2 sees nothing (obstacle left the image).
    _lpf_pass holds the last smoothed DISTANCE, but range_m must NOT be held
    — a held range_m would misrepresent a frame that took no new depth
    reading as if it had, understating the sensor's actual uncertainty."""
    engine = DistanceEngine({'min_depth_m': 0.15, 'max_depth_m': 4.0, 'lpf_alpha': 0.5})
    r1 = _compute(engine, [_cp([0.0, 0.0, 0.5])], _wall_depth(), frame_stamp=0.0)[0]
    assert r1.range_m is not None
    r2 = _compute(engine, [_cp([0.0, 0.0, 0.5])], _empty_depth(),
                 frame_stamp=1.0 / 30.0)[0]
    assert np.isfinite(r2.distance), 'the hold must still report the held distance'
    assert r2.range_m is None, 'a held row must not carry a stale raw range'


# ═════════════════════════════════════════════════════════════════════════════
#  perception_msgs.build_cp_messages: ControlPointResult.range_m -> LinkDistance
# ═════════════════════════════════════════════════════════════════════════════

def _build_links(results, segment_links=('fr3_link8',)):
    _, mld = build_cp_messages(
        cp_results=results, n_pts=500, stamp=_STAMP, frame_id='fr3_link0',
        segment_links=list(segment_links),
        thresholds={'min_thresh': 0.08, 'max_thresh': 0.7},
        fallback=2.0, zones={'warning': 0.3, 'danger': 0.2, 'critical': 0.1})
    return mld.links


def test_build_cp_messages_carries_range_m_onto_the_wire():
    engine = DistanceEngine({'min_depth_m': 0.15, 'max_depth_m': 4.0, 'lpf_alpha': 0.0})
    results = _compute(engine, [_cp([0.0, 0.0, 0.5])], _wall_depth())
    ld = _build_links(results)[0]
    assert abs(ld.range_m - WALL_Z) < 1e-5


def test_build_cp_messages_publishes_the_zero_sentinel_when_absent():
    """No obstacle -> ControlPointResult.range_m is None -> the wire default
    (0.0) is left untouched, never a negative or NaN placeholder."""
    engine = DistanceEngine({'min_depth_m': 0.15, 'max_depth_m': 4.0, 'lpf_alpha': 0.0})
    results = _compute(engine, [_cp([0.0, 0.0, 0.5])], _empty_depth())
    ld = _build_links(results)[0]
    assert ld.range_m == 0.0


# ═════════════════════════════════════════════════════════════════════════════
#  CBFSafetyFilter._range_field: LinkDistance -> Obstacle kwarg
# ═════════════════════════════════════════════════════════════════════════════

def test_range_field_is_empty_for_the_wire_default():
    franka_msgs = pytest.importorskip('franka_msgs.msg')
    CBFSafetyFilter = pytest.importorskip(
        'franka_experiments.nodes.cbf_safety_filter').CBFSafetyFilter
    ld = franka_msgs.LinkDistance()
    assert CBFSafetyFilter._range_field(ld) == {}


def test_range_field_carries_a_positive_range():
    franka_msgs = pytest.importorskip('franka_msgs.msg')
    CBFSafetyFilter = pytest.importorskip(
        'franka_experiments.nodes.cbf_safety_filter').CBFSafetyFilter
    ld = franka_msgs.LinkDistance()
    ld.range_m = 0.73
    assert CBFSafetyFilter._range_field(ld) == {'range_m': 0.73}


def test_range_field_treats_a_negative_value_as_absent():
    """Defensive: range_m should never be negative on the wire, but a
    consumer must not trust one if it somehow arrives."""
    franka_msgs = pytest.importorskip('franka_msgs.msg')
    CBFSafetyFilter = pytest.importorskip(
        'franka_experiments.nodes.cbf_safety_filter').CBFSafetyFilter
    ld = franka_msgs.LinkDistance()
    ld.range_m = -1.0
    assert CBFSafetyFilter._range_field(ld) == {}


def test_range_field_is_empty_for_a_message_without_the_field():
    """Forward-compat: an older franka_msgs build (predating this field)
    must degrade to 'no range', not crash."""
    class _OldLinkDistance:
        pass
    CBFSafetyFilter = pytest.importorskip(
        'franka_experiments.nodes.cbf_safety_filter').CBFSafetyFilter
    assert CBFSafetyFilter._range_field(_OldLinkDistance()) == {}


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__, '-v']))
