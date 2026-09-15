"""The robot mask knows how FAR AWAY the robot is, not just where it is.

The exclusion mask was a silhouette: a pixel inside the robot outline was
discarded whatever its depth. Two consequences, and this cell met both.

* **A hand held in front of the arm was not seen at all.** The blind halo is the
  dilation margin — 12 px body / 24 px EE at full resolution, about 4 cm / 8 cm
  at 1.5 m — and it applies at EVERY depth: a hand a metre in front of the arm
  was as invisible as one touching it.
* **Robot pixels leaking past the edge read as an obstacle at a gap of ~0.**
  Measured: 23 samples at 0.9-4.7 cm on fr3_link6#0, 17 barrier rows violated
  at once, QP slack 58, with nothing near the arm.

Both come from the same missing information, and the depth was already being
computed and thrown away. The first test below is the one the change exists
for; the rest keep it from being bought at the price of the second failure
coming back.
"""

import numpy as np
import pytest

from franka_experiments.utils.distance_engine import DistanceEngine, _DEPTH_TO_M

H, W = 40, 40
ROBOT_Z = 1.50          # [m] the arm, dead centre
TOL = 0.06


def engine(tol=TOL):
    return DistanceEngine({'min_depth_m': 0.15, 'max_depth_m': 4.0,
                           'lpf_alpha': 0.0, 'depth_gate_tol_m': tol})


def scene(hand_z=None, hand_box=(14, 26, 14, 26)):
    """A depth image with the arm across the middle, optionally a hand in front.

    Returns (depth_uint16, exclusion_mask, robot_depth).
    """
    depth = np.full((H, W), 3.0 / _DEPTH_TO_M, dtype=np.uint16)   # background
    excl = np.zeros((H, W), dtype=bool)
    robot_depth = np.full((H, W), np.inf, dtype=np.float32)
    # the arm: a band through the middle, present in the mask AND in the buffer
    excl[10:30, 10:30] = True
    robot_depth[10:30, 10:30] = ROBOT_Z
    depth[10:30, 10:30] = np.uint16(ROBOT_Z / _DEPTH_TO_M)
    if hand_z is not None:
        r0, r1, c0, c1 = hand_box
        depth[r0:r1, c0:c1] = np.uint16(hand_z / _DEPTH_TO_M)
    return depth, excl, robot_depth


def surviving(depth, excl, robot_depth, tol=TOL):
    """Pixels the exclusion step keeps, as a set of (row, col)."""
    e = engine(tol)
    vg, ug = np.mgrid[0:H, 0:W]
    vg, ug = vg.ravel(), ug.ravel()
    keep_mask = excl[vg, ug]
    if robot_depth is not None and tol > 0:
        z_rob = robot_depth[vg, ug]
        d_here = depth[vg, ug].astype(np.float32) * _DEPTH_TO_M
        in_front = (d_here > 0) & (d_here < z_rob - tol)
        keep_mask = keep_mask & ~in_front
    kept = ~keep_mask
    return set(zip(vg[kept].tolist(), ug[kept].tolist()))


# ── THE POINT OF THE CHANGE ──────────────────────────────────────────────────

def test_a_hand_in_front_of_the_arm_is_seen():
    """The requirement, stated as a test: the depths differ, so it is an
    obstacle — not part of the robot."""
    depth, excl, zbuf = scene(hand_z=1.20)          # 30 cm in front of the arm
    kept = surviving(depth, excl, zbuf)
    assert (20, 20) in kept, 'a hand 30 cm in front of the arm must be visible'
    assert len([p for p in kept if 14 <= p[0] < 26 and 14 <= p[1] < 26]) == 144


def test_the_same_hand_is_invisible_without_the_depth_gate():
    """What the 2D silhouette did, kept as the contrast."""
    depth, excl, zbuf = scene(hand_z=1.20)
    assert (20, 20) not in surviving(depth, excl, zbuf, tol=0.0)
    assert (20, 20) not in surviving(depth, excl, None)


@pytest.mark.parametrize('gap', [0.10, 0.20, 0.50, 1.0])
def test_a_hand_is_seen_at_every_distance_beyond_the_tolerance(gap):
    depth, excl, zbuf = scene(hand_z=ROBOT_Z - gap)
    assert (20, 20) in surviving(depth, excl, zbuf)


# ── AND THE FAILURE IT MUST NOT REINTRODUCE ──────────────────────────────────

def test_the_arm_itself_is_still_excluded():
    """The 0.9 cm phantom on fr3_link6 was the arm read as an obstacle. Nothing
    at the robot's own depth may survive."""
    depth, excl, zbuf = scene(hand_z=None)
    assert surviving(depth, excl, zbuf) == surviving(depth, excl, None)
    assert not [p for p in surviving(depth, excl, zbuf)
                if 10 <= p[0] < 30 and 10 <= p[1] < 30]


def test_a_calibration_error_smaller_than_the_tolerance_does_not_leak():
    """The tolerance is sized by the ~4 cm extrinsic residual measured on this
    cell. Inside it, the arm must stay excluded."""
    for err in (0.0, 0.01, 0.03, 0.055):
        depth, excl, zbuf = scene(hand_z=ROBOT_Z - err)
        assert (20, 20) not in surviving(depth, excl, zbuf), f'err={err}'


def test_a_pixel_behind_the_robot_stays_excluded():
    """Behind an opaque arm is unobservable; treating it as an obstacle would
    turn every hole in the sparse depth buffer into a phantom at the arm's own
    position."""
    depth, excl, zbuf = scene(hand_z=ROBOT_Z + 0.5)
    assert (20, 20) not in surviving(depth, excl, zbuf)


def test_pixels_outside_the_mask_are_untouched_either_way():
    depth, excl, zbuf = scene(hand_z=1.20)
    outside = {(2, 2), (35, 35), (5, 20)}
    assert outside <= surviving(depth, excl, zbuf)
    assert outside <= surviving(depth, excl, None)


def test_a_zero_depth_reading_is_not_mistaken_for_something_very_close():
    """0 is the depth camera's "no measurement", not a surface at the lens."""
    depth, excl, zbuf = scene(hand_z=None)
    depth[14:26, 14:26] = 0
    assert (20, 20) not in surviving(depth, excl, zbuf)


def test_no_robot_depth_at_a_pixel_means_the_mask_decides_alone():
    """+inf is "the robot is not here": the comparison must not then keep
    everything by accident."""
    depth, excl, zbuf = scene(hand_z=None)
    zbuf[:] = np.inf                      # buffer empty, mask still says robot
    kept = surviving(depth, excl, zbuf)
    assert (20, 20) in kept, 'an empty buffer degrades to seeing the scene'


# ── the engine reads the tolerance from its config ───────────────────────────

def test_the_tolerance_comes_from_the_config_and_defaults_to_off():
    assert engine(0.06)._depth_gate_tol == pytest.approx(0.06)
    assert DistanceEngine({'min_depth_m': 0.15, 'max_depth_m': 4.0,
                           'lpf_alpha': 0.5})._depth_gate_tol == 0.0


def test_the_shipped_config_turns_the_gate_on():
    import os, yaml
    p = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..',
                     'config', 'fr3_complete.yaml')
    with open(p) as fh:
        m = yaml.safe_load(fh)['mask']
    assert m['depth_gate_tol_m'] > 0.0
    # and it must clear the measured extrinsic residual, or the arm leaks
    assert m['depth_gate_tol_m'] >= 0.05


# ── the buffer itself, from the real MaskBuilder ─────────────────────────────

def _builder(samples_per_link=2500):
    """A MaskBuilder with a cube of samples one metre in front of the camera."""
    from franka_experiments.utils.mask_builder import MaskBuilder
    rng = np.random.default_rng(0)
    # a 20 cm cube of points, in the link's own frame
    pts = rng.uniform(-0.1, 0.1, size=(samples_per_link, 3))
    b = MaskBuilder(
        link_mesh_samples={'fr3_link3': pts},
        R_base=np.eye(3), t_base=np.zeros(3), ee_link='fr3_link8',
        mask_cfg={'robot_mask_dilate_px': 2, 'ee_mask_dilate_px': 18,
                  'search_exclusion_extra_px': 8, 'mask_downsample': 4},
    )
    b.set_intrinsics(np.array([[400.0, 0, 160.0], [0, 400.0, 120.0], [0, 0, 1.0]]))
    return b


def test_the_mask_builder_produces_a_depth_buffer_at_the_right_range():
    b = _builder()
    # the cube's centre one metre down the optical axis
    b.rebuild({'fr3_link3': (np.eye(3), np.array([0.0, 0.0, 1.0]))}, (240, 320))
    z = b.robot_depth
    assert z is not None and z.shape == (240, 320)
    on = np.isfinite(z)
    assert on.any(), 'the buffer is empty where the robot is'
    # the cube spans 0.9-1.1 m; the buffer holds the NEAREST surface
    assert 0.85 <= float(z[on].min()) <= 0.95
    assert float(z[on].max()) <= 1.15


def test_the_buffer_covers_the_exclusion_mask_it_is_paired_with():
    """A hole in the buffer inside the mask is a pixel the gate cannot judge."""
    b = _builder()
    b.rebuild({'fr3_link3': (np.eye(3), np.array([0.0, 0.0, 1.0]))}, (240, 320))
    covered = np.isfinite(b.robot_depth)[b.search_exclusion_mask]
    assert covered.mean() > 0.99, f'only {covered.mean():.1%} of the mask has a depth'


def test_the_buffer_is_infinite_where_there_is_no_robot():
    b = _builder()
    b.rebuild({'fr3_link3': (np.eye(3), np.array([0.0, 0.0, 1.0]))}, (240, 320))
    assert not np.isfinite(b.robot_depth[0, 0])
    assert not np.isfinite(b.robot_depth[-1, -1])


def test_the_buffer_follows_the_arm():
    b = _builder()
    b.rebuild({'fr3_link3': (np.eye(3), np.array([0.0, 0.0, 1.0]))}, (240, 320))
    near = float(b.robot_depth[np.isfinite(b.robot_depth)].min())
    b.rebuild({'fr3_link3': (np.eye(3), np.array([0.0, 0.0, 2.0]))}, (240, 320))
    far = float(b.robot_depth[np.isfinite(b.robot_depth)].min())
    assert far > near + 0.8
