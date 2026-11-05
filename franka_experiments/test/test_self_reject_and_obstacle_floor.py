"""Two things the depth buffer was computing but nothing was acting on.

**The arm as its own obstacle.** ``mask.depth_gate_tol_m`` can only ever
UN-exclude: it rescues a pixel the silhouette covered, so an obstacle in front
of the arm is seen. It cannot touch a pixel OUTSIDE the silhouette, and the
dilation is finite (12 px on the body) while calibration and the mask's pose
lag can put the true arm beyond the projected model. Those pixels were kept as
obstacles at a gap of ~0, and ``DistanceEngine.last_self_leak_px`` had been
counting them with nothing acting on the count. Measured 2026-10-30 in the
furnished diag scene: 38.1 % of the BINDING barrier rows were the arm's own
surface. ``mask.self_reject_tol_m`` removes them.

**The surface the robot is bolted to.** The camera returns the room, and the
room includes the table under the arm (``diag_robot_table``, top at base-frame
z = 0). 17.9 % of binding rows were that table, reported at 0.000 m, while the
sphere under test sat at 0.505 m. ``distance.obstacle_min_z_m`` is a plane in
the BASE frame, so it does not move when the camera does.

Both default to off / -inf, and the off path must stay bit-identical.
"""

import numpy as np

from franka_experiments.utils.distance_engine import DistanceEngine, _DEPTH_TO_M

H, W = 40, 40
ROBOT_Z = 1.50          # [m] where the model puts the arm
TOL = 0.06              # mask.depth_gate_tol_m


def engine(**over):
    cfg = {'min_depth_m': 0.15, 'max_depth_m': 4.0, 'lpf_alpha': 0.0,
           'depth_gate_tol_m': TOL}
    cfg.update(over)
    return DistanceEngine(cfg)


def cp(point):
    return [dict(point=np.asarray(point, float), radius=0.0, seg_idx=0, cp_idx=0,
                 start_link='fr3_link6', end_link='fr3_link6')]


def run(e, depth, excl, robot_depth, point=(0.0, 0.0, 1.0)):
    """One frame through the REAL pipeline.

    ``R_base = I`` and ``t_base = 0``, so base frame == camera frame: the
    base-frame height of a pixel is its DEPTH, which is what makes the floor
    assertions below readable.
    """
    return e.compute(
        depth=depth, cx_f32=np.float32(W / 2), cy_f32=np.float32(H / 2),
        fx_inv_f32=np.float32(1.0 / 400.0), fy_inv_f32=np.float32(1.0 / 400.0),
        R_base_f32=np.eye(3, dtype=np.float32),
        t_base_f32=np.zeros(3, dtype=np.float32),
        control_points=cp(point), x=np.array([0, W]), y=np.array([0, H]), step=1,
        search_exclusion_mask=excl, robot_depth=robot_depth)


def leaking_scene():
    """The arm at ROBOT_Z, with a band of its own pixels OUTSIDE the mask.

    This is the leak: the model knows the arm is there (``robot_depth`` is
    finite) and the measured depth agrees, but the silhouette stops one band
    short, exactly as a finite dilation does.
    """
    depth = np.full((H, W), np.uint16(3.0 / _DEPTH_TO_M), dtype=np.uint16)
    excl = np.zeros((H, W), dtype=bool)
    robot_depth = np.full((H, W), np.inf, dtype=np.float32)
    robot_depth[10:30, 10:30] = ROBOT_Z
    depth[10:30, 10:30] = np.uint16(ROBOT_Z / _DEPTH_TO_M)
    excl[10:30, 10:28] = True            # the mask stops 2 columns short
    return depth, excl, robot_depth


def test_the_leak_is_counted_but_kept_while_self_rejection_is_off():
    e = engine()
    depth, excl, robot_depth = leaking_scene()
    res, n_px = run(e, depth, excl, robot_depth)
    assert e.last_self_leak_px > 0, 'the scene must actually leak'
    assert e.last_self_reject_px == 0
    # the leaked band is at the arm's own range, so it reads as an obstacle
    assert np.isfinite(res[0].distance)
    assert res[0].distance < ROBOT_Z


def test_self_rejection_removes_exactly_those_pixels():
    depth, excl, robot_depth = leaking_scene()
    off = engine()
    res_off, n_off = run(off, depth, excl, robot_depth)
    on = engine(self_reject_tol_m=0.04)
    res_on, n_on = run(on, depth, excl, robot_depth)
    assert on.last_self_reject_px == off.last_self_leak_px
    assert n_on == n_off - on.last_self_reject_px
    # the arm is gone; only the 3 m background is left, which is beyond
    # nothing here, so the CP sees the background and not its own surface
    assert res_on[0].distance > res_off[0].distance


def test_self_rejection_keeps_a_real_obstacle_in_front_of_the_arm():
    """The whole point of the depth gate must survive the new rule."""
    depth, excl, robot_depth = leaking_scene()
    depth[14:26, 14:26] = np.uint16(1.20 / _DEPTH_TO_M)   # a hand 30 cm in front
    e = engine(self_reject_tol_m=0.04)
    res, _ = run(e, depth, excl, robot_depth)
    assert res[0].closest_obstacle_point is not None
    assert abs(float(res[0].closest_obstacle_point[2]) - 1.20) < 0.05


def test_obstacle_floor_drops_points_below_it_and_counts_them():
    depth = np.full((H, W), np.uint16(2.0 / _DEPTH_TO_M), dtype=np.uint16)
    depth[0:20, :] = np.uint16(0.50 / _DEPTH_TO_M)   # "below" with R_base = I
    excl = np.zeros((H, W), dtype=bool)
    free = engine()
    res_free, n_free = run(free, depth, excl, None)
    assert free.last_floor_drop_px == 0
    floored = engine(obstacle_min_z_m=1.0)
    res_f, n_f = run(floored, depth, excl, None)
    assert floored.last_floor_drop_px == 20 * W
    assert n_f == n_free - 20 * W
    assert abs(float(res_f[0].closest_obstacle_point[2]) - 2.0) < 0.05
    assert abs(float(res_free[0].closest_obstacle_point[2]) - 0.50) < 0.05


def test_the_floor_is_a_base_frame_plane_not_a_depth_crop():
    """With the camera rotated, the floor must follow the BASE frame."""
    depth = np.full((H, W), np.uint16(2.0 / _DEPTH_TO_M), dtype=np.uint16)
    excl = np.zeros((H, W), dtype=bool)
    e = engine(obstacle_min_z_m=1.0)
    # base z = -(camera z): everything the camera sees is BELOW the plane
    R = np.diag([1.0, 1.0, -1.0]).astype(np.float32)
    out, n = e.compute(
        depth=depth, cx_f32=np.float32(W / 2), cy_f32=np.float32(H / 2),
        fx_inv_f32=np.float32(1.0 / 400.0), fy_inv_f32=np.float32(1.0 / 400.0),
        R_base_f32=R, t_base_f32=np.zeros(3, dtype=np.float32),
        control_points=cp((0.0, 0.0, 1.0)), x=np.array([0, W]), y=np.array([0, H]),
        step=1, search_exclusion_mask=excl, robot_depth=None)
    assert n == 0 and e.last_floor_drop_px == H * W


def test_both_features_default_to_inert():
    e = engine()
    assert e._self_reject_tol == 0.0
    assert e._obstacle_min_z == float('-inf')
    depth = np.full((H, W), np.uint16(2.0 / _DEPTH_TO_M), dtype=np.uint16)
    excl = np.zeros((H, W), dtype=bool)
    res, n = run(e, depth, excl, None)
    assert n == H * W and e.last_floor_drop_px == 0 and e.last_self_reject_px == 0


def test_the_window_covers_a_well_calibrated_model_too():
    """The bug this file's measured numbers came from.

    ``depth_gate_bias_m`` is the MEDIAN of (measured - model) and belongs to
    the calibration: ~6 cm on the rig, ~0 in simulation where the model and
    the depth agree. A self-test centred on ``robot_depth - bias`` therefore
    tests 6 cm in front of where a well-calibrated arm actually is and rejects
    nothing — measured in Gazebo on 2026-10-30. The window must cover the arm
    at BOTH ends of the bias.
    """
    depth, excl, robot_depth = leaking_scene()         # depth == robot_depth
    on = engine(self_reject_tol_m=0.04, depth_gate_bias_m=0.06)
    run(on, depth, excl, robot_depth)
    assert on.last_self_reject_px > 0, 'a perfectly calibrated arm is still the arm'
    # and the biased case, the rig's own: the arm reads 6 cm nearer than model
    depth2, excl2, rd2 = leaking_scene()
    depth2[10:30, 10:30] = np.uint16((ROBOT_Z - 0.06) / _DEPTH_TO_M)
    on2 = engine(self_reject_tol_m=0.04, depth_gate_bias_m=0.06)
    run(on2, depth2, excl2, rd2)
    assert on2.last_self_reject_px > 0, 'the biased arm must be rejected as well'


def test_the_blind_shell_stays_inside_the_gates_own_threshold():
    """A real obstacle the gate rescues must never be rejected as the arm."""
    depth, excl, robot_depth = leaking_scene()
    # 0.20 m in front of the model: well past bias + tol (0.10), so an obstacle
    depth[14:26, 14:26] = np.uint16((ROBOT_Z - 0.20) / _DEPTH_TO_M)
    e = engine(self_reject_tol_m=0.04, depth_gate_bias_m=0.06)
    res, _ = run(e, depth, excl, robot_depth)
    assert abs(float(res[0].closest_obstacle_point[2]) - (ROBOT_Z - 0.20)) < 0.05
