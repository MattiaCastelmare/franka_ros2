"""Surface-gap semantics of the depth-space distance pipeline.

Covers the two defects fixed in the "distance semantics" pass, plus the
baseline geometry they depend on.  Pure numpy + stubbed ROS message classes —
no rclpy, no franka_msgs, no Pinocchio — so it runs in CI without a sourced
workspace.

What is asserted
----------------
1. ``DistanceEngine`` reports the gap from a control point to the nearest
   OBSERVED obstacle surface point, minus the capsule radius and minus the
   pixel-dilation margin.  A depth camera only ever returns the front surface
   of an obstacle, so this is already an edge/surface distance — there is no
   interior term and no distance transform anywhere in the pipeline.
2. That gap CLAMPS AT EXACTLY 0.0 on contact (``np.maximum(..., 0.0)``).
3. REGRESSION (a): a control point reporting exactly 0.0 is published with
   ``valid=True``.  It used to be ``d > 0.0``, which marked the single most
   dangerous sample invalid; ``cbf_safety_filter._on_distances`` filters on
   ``ld.valid``, so that link's HOCBF row vanished from the QP precisely when
   it mattered.
4. REGRESSION (b): ``‖closest_point_robot − closest_point_human‖`` exceeds the
   published ``LinkDistance.distance`` by exactly (radius + dilation margin).
   ``cbf_safety_filter`` used to rebuild its barrier from those two points and
   therefore carried that whole quantity as an optimistic bias.  This test
   pins the size of what the filter now no longer discards.

Run with pytest, or directly:  python3 test_distance_surface_semantics.py
"""

import sys
import types

import numpy as np

from franka_experiments.utils.distance_engine import DistanceEngine


# ── Stub ROS messages so perception_msgs imports without a sourced workspace ──

def _install_msg_stubs():
    """Register minimal franka_msgs / geometry_msgs stubs in sys.modules.

    Only when the real packages are unavailable.  `colcon test` runs the whole
    test/ directory in ONE pytest process, so unconditionally shadowing
    franka_msgs would poison every other module in that session; a sourced
    workspace must keep the generated messages.  build_cp_messages only sets
    plain attributes, so the attribute bags below stand in exactly.

    Returns True when stubs were installed (no ROS present).
    """
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
        distance=0.0, valid=False, confidence=0.0, zone='')
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

# Real messages type-check header.stamp on assignment; the stubs do not care.
if _STUBBED:
    _STAMP = None
else:
    from builtin_interfaces.msg import Time
    _STAMP = Time()

from franka_experiments.utils.perception_msgs import build_cp_messages  # noqa: E402


# ── Synthetic scene ──────────────────────────────────────────────────────────
#
# Identity extrinsics, so camera frame == base frame and every expected value
# below is readable by hand.  A fronto-parallel wall at Z = WALL_Z fills the
# image; the pixel grid is chosen so the principal point (CX, CY) is sampled,
# and that pixel unprojects to exactly (0, 0, WALL_Z).

W = H  = 320
FX = FY = 400.0
CX = CY = 160.0
STEP    = 10          # (CX - 0) / STEP is an integer → the axis pixel is sampled
WALL_Z  = 1.0         # [m]
RADIUS  = 0.05        # [m] capsule radius

R_EYE  = np.eye(3, dtype=np.float32)
T_ZERO = np.zeros(3, dtype=np.float32)

_ENGINE_CFG = {'min_depth_m': 0.15, 'max_depth_m': 4.0, 'lpf_alpha': 0.0}


def _wall_depth(z_m=WALL_Z):
    """Uniform depth image (uint16 millimetres) of a fronto-parallel wall."""
    return np.full((H, W), int(round(z_m * 1000.0)), dtype=np.uint16)


def _cp(point, radius=RADIUS, seg_idx=7, cp_idx=0,
        start_link='fr3_link7', end_link='fr3_link8'):
    return {'point': np.asarray(point, dtype=np.float64), 'seg_idx': seg_idx,
            'cp_idx': cp_idx, 'radius': radius,
            'start_link': start_link, 'end_link': end_link}


def _run(control_points, depth=None, margins_px=None):
    """Run one engine frame; returns the ControlPointResult list."""
    engine = DistanceEngine(dict(_ENGINE_CFG))
    ee_src = np.zeros((H, W), dtype=bool) if margins_px is not None else None
    results, _ = engine.compute(
        depth=_wall_depth() if depth is None else depth,
        cx_f32=np.float32(CX), cy_f32=np.float32(CY),
        fx_inv_f32=np.float32(1.0 / FX), fy_inv_f32=np.float32(1.0 / FY),
        R_base_f32=R_EYE, t_base_f32=T_ZERO,
        control_points=control_points,
        x=np.array([0, W]), y=np.array([0, H]), step=STEP,
        search_exclusion_mask=None,
        ee_source_mask=ee_src, dilation_margins_px=margins_px)
    return results


# ── 1. Surface gap == distance to the nearest OBSERVED point, minus radius ───

def test_gap_is_nearest_surface_point_minus_radius():
    # CP on the optical axis, 0.5 m in front of a wall at 1.0 m. The nearest
    # observed point is the axis pixel at (0, 0, 1.0) → centre gap 0.5 m.
    r = _run([_cp([0.0, 0.0, 0.5])])[0]
    assert abs(r.distance - (0.5 - RADIUS)) < 1e-5, r.distance
    assert np.allclose(r.closest_obstacle_point, [0.0, 0.0, WALL_Z], atol=1e-5)
    # Direction points obstacle → control point, i.e. back along −Z here.
    assert np.allclose(r.direction, [0.0, 0.0, -1.0], atol=1e-5)


def test_gap_matches_independent_bruteforce():
    """Cross-check the vectorised argmin against a plain Python loop.

    Guards the Step-4/Step-6 unprojection arithmetic (which the closed-form
    on-axis case above cannot distinguish from several wrong formulas) using a
    CP placed off-axis, so the winning pixel is not the principal point.
    """
    cp_pt  = np.array([0.11, -0.07, 0.42])
    depth  = _wall_depth()
    result = _run([_cp(cp_pt)])[0]

    best = np.inf
    for v in range(0, H, STEP):
        for u in range(0, W, STEP):
            z = float(depth[v, u]) / 1000.0
            p = np.array([(u - CX) * z / FX, (v - CY) * z / FY, z])
            best = min(best, float(np.linalg.norm(p - cp_pt)))
    assert abs(result.distance - max(best - RADIUS, 0.0)) < 1e-5


# ── 2. Clamping at contact ───────────────────────────────────────────────────

def test_gap_clamps_to_exactly_zero_on_contact():
    # Centre gap 0.03 m < radius 0.05 m → np.maximum(..., 0.0) floors it.
    r = _run([_cp([0.0, 0.0, WALL_Z - 0.03])])[0]
    assert r.distance == 0.0, r.distance


def test_dilation_margin_is_subtracted_at_pixel_depth():
    # margin_px = 12 at Z = 1.0 m with fx = 400 → 12 / 400 = 0.03 m.
    r = _run([_cp([0.0, 0.0, 0.5])], margins_px=(12, 24))[0]
    assert abs(r.distance - (0.5 - RADIUS - 0.03)) < 1e-5, r.distance


# ── 3. REGRESSION (a): a zero-distance link must reach the QP ────────────────

def _build_links(results, segment_links=('fr3_link8',)):
    _, mld = build_cp_messages(
        cp_results=results, n_pts=500, stamp=_STAMP, frame_id='fr3_link0',
        segment_links=list(segment_links),
        thresholds={'min_thresh': 0.08, 'max_thresh': 0.7},
        fallback=2.0, zones={'warning': 0.3, 'danger': 0.2, 'critical': 0.1})
    return mld.links


def test_zero_distance_link_is_published_valid():
    """The contact sample must survive into MultiLinkDistance.

    Under the old `ld.valid = ... d > 0.0`, this entry was published invalid;
    cbf_safety_filter._on_distances keeps only `if ld.valid`, so the closest
    link contributed NO HOCBF row at the moment of contact.
    """
    results = _run([_cp([0.0, 0.0, WALL_Z - 0.03])])
    assert results[0].distance == 0.0                     # precondition
    links = _build_links(results)
    assert len(links) == 1
    assert links[0].distance == 0.0
    assert links[0].valid is True, 'zero-distance link must stay valid'


def test_invalid_only_when_measurement_is_missing():
    """valid=False is still reserved for a genuinely absent measurement."""
    results   = _run([_cp([0.0, 0.0, 0.5])])
    results[0].direction = None          # engine could not form a normal
    assert _build_links(results)[0].valid is False


# ── 4. REGRESSION (b): centre distance overstates the gap ────────────────────

def test_centre_distance_exceeds_published_gap_by_radius_plus_margin():
    """Quantify the bias cbf_safety_filter used to carry.

    The filter rebuilt its barrier as h = ‖pr − ph‖ − d_safe.  pr is the
    control point on the segment AXIS, so that norm is the CENTRE distance and
    omits both the capsule radius and the dilation margin already subtracted
    upstream.  It now reads LinkDistance.distance directly; this test pins how
    much clearance the old form invented.
    """
    margin_m = 12 / FX * WALL_Z                       # 0.03 m at this depth
    results  = _run([_cp([0.0, 0.0, 0.5])], margins_px=(12, 24))
    ld       = _build_links(results)[0]

    pr = np.array([ld.closest_point_robot.x,
                   ld.closest_point_robot.y,
                   ld.closest_point_robot.z])
    ph = np.array([ld.closest_point_human.x,
                   ld.closest_point_human.y,
                   ld.closest_point_human.z])
    centre_d = float(np.linalg.norm(pr - ph))

    assert abs(centre_d - 0.5) < 1e-5
    assert abs((centre_d - ld.distance) - (RADIUS + margin_m)) < 1e-5

    # With d_safe = 0.20 the old barrier read +0.30 while the true surface gap
    # was 0.42 - 0.20 = +0.22: the filter believed it had 8 cm it did not have.
    d_safe = 0.20
    assert (centre_d - d_safe) - (ld.distance - d_safe) > 0.0


def test_barrier_reaches_negative_before_contact():
    """With the corrected gap, h̄ goes negative while the centre form is still
    positive — the window in which the old filter applied no repulsion."""
    d_safe   = 0.20
    results  = _run([_cp([0.0, 0.0, WALL_Z - 0.22])], margins_px=(12, 24))
    ld       = _build_links(results)[0]
    assert ld.distance - d_safe < 0.0, 'corrected barrier must be violated'
    assert 0.22 - d_safe > 0.0,        'centre-distance barrier was not'


# ── 5. Multi-CP activation: every control point must reach the QP ───────────

def _two_cps_on_one_link(near_z, far_z):
    """Two control points on fr3_link8 at different standoffs from the wall."""
    return [_cp([0.0, 0.0, near_z], cp_idx=0),
            _cp([0.06, 0.0, far_z], cp_idx=1)]


def test_every_control_point_gets_its_own_entry():
    """One LinkDistance per CP, not argmin over each link.

    Under the old per-link pooling these two CPs collapsed to a single entry,
    so the farther one contributed no HOCBF row: a link could be pushed clear
    at its closest CP while a second CP on the SAME link kept closing.
    """
    results = _run(_two_cps_on_one_link(0.55, 0.62))
    links   = _build_links(results)
    assert len(links) == 2, f'expected one entry per CP, got {len(links)}'
    assert all(ld.robot_link_name == 'fr3_link8' for ld in links)
    assert all(ld.valid for ld in links)
    # Distinct measurements, not one value duplicated.
    assert links[0].distance != links[1].distance


def test_multiple_cps_below_threshold_are_all_valid():
    """2+ CPs inside d_safe simultaneously — the Phase 2 synthetic scenario."""
    d_safe  = 0.20
    results = _run(_two_cps_on_one_link(WALL_Z - 0.18, WALL_Z - 0.21))
    links   = _build_links(results)
    breached = [ld for ld in links if ld.valid and ld.distance - d_safe < 0.0]
    assert len(breached) == 2, (
        f'both CPs must breach and stay valid, got {len(breached)}: '
        f'{[ld.distance for ld in links]}')


def test_cp_entry_order_is_stable_across_frames():
    """Row order must not permute — OSQP warm-starts on a fixed sparsity."""
    cps = _two_cps_on_one_link(0.55, 0.62)
    order_a = [(ld.robot_link_name, round(ld.distance, 9))
               for ld in _build_links(_run(cps))]
    order_b = [(ld.robot_link_name, round(ld.distance, 9))
               for ld in _build_links(_run(list(reversed(cps))))]
    assert order_a == order_b, 'entry order must not depend on input order'


def test_entries_span_multiple_links_in_segment_link_order():
    """Ordering follows segment_links, then (seg_idx, cp_idx) within a link."""
    results = _run([
        _cp([0.0, 0.0, 0.55], seg_idx=7, cp_idx=1,
            start_link='fr3_link7', end_link='fr3_link8'),
        _cp([0.0, 0.0, 0.50], seg_idx=7, cp_idx=0,
            start_link='fr3_link7', end_link='fr3_link8'),
        _cp([0.2, 0.0, 0.60], seg_idx=6, cp_idx=0,
            start_link='fr3_link6', end_link='fr3_link7'),
    ])
    links = _build_links(results, segment_links=('fr3_link7', 'fr3_link8'))
    assert [ld.robot_link_name for ld in links] == [
        'fr3_link7', 'fr3_link8', 'fr3_link8']
    # Within fr3_link8, cp_idx 0 (the 0.50 CP, gap 0.45) precedes cp_idx 1.
    assert abs(links[1].distance - (0.5 - RADIUS)) < 1e-5


def test_control_point_absent_from_segment_links_is_dropped():
    """A CP whose end_link is not in segment_links must not be published."""
    results = _run([_cp([0.0, 0.0, 0.5], end_link='fr3_link8')])
    assert _build_links(results, segment_links=('fr3_link7',)) == []


if __name__ == '__main__':
    import pytest
    raise SystemExit(pytest.main([__file__, '-v']))
