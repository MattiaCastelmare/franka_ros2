"""Self-detection: telling the robot's own arm apart from an obstacle.

The distance pipeline defines "obstacle" as "not robot", so when the robot is
not fully removed from the depth image the arm becomes its own obstacle and
nothing downstream can tell. With the scalar residual that was a nuisance; with
a TRACKER it is a feedback loop — the arm's own body gets a real velocity, that
velocity is fed back as v_obs, the barrier tightens because the arm is moving,
the arm brakes, and the filter chases itself.

The discriminator under test is kinematic and not geometric, which is the whole
point: geometry is exactly what a stale hand-eye calibration breaks. A point
that is part of the robot moves WITH the robot, so the offset
``p_human − p_robot`` stays rigid while the control point sweeps the workspace.

Pure numpy.
"""

import numpy as np

from franka_experiments.utils.self_detection import SelfDetectionMonitor


def _run(mon, robot_pts, human_pts, key='fr3_link5#0'):
    out = []
    for pr, ph in zip(robot_pts, human_pts):
        out.append(mon.update(key, np.asarray(pr, float), np.asarray(ph, float)))
    return out


def _sweep(n=60, span=0.5):
    """A control point sweeping `span` metres — a normal arm motion."""
    t = np.linspace(0.0, 1.0, n)
    return np.stack([0.3 + span * t, np.zeros(n), np.full(n, 0.5)], axis=1)


# ── The two cases that must be told apart ───────────────────────────────────

def test_an_obstacle_rigidly_attached_to_the_arm_is_flagged():
    """Mask leakage / bad calibration: the "obstacle" is a fixed offset from the
    control point, so it sweeps the workspace with it."""
    robot = _sweep()
    human = robot + np.array([0.09, 0.0, 0.0])
    mon = SelfDetectionMonitor()
    assert _run(mon, robot, human)[-1] is True
    assert mon.is_self('fr3_link5#0')


def test_a_genuinely_static_obstacle_is_not_flagged():
    """The arm sweeps past a fixed obstacle. The offset changes by exactly the
    arm's travel, which is what a real obstacle looks like."""
    robot = _sweep()
    human = np.tile(np.array([0.95, 0.0, 0.5]), (robot.shape[0], 1))
    mon = SelfDetectionMonitor()
    assert not any(_run(mon, robot, human))


def test_a_stationary_arm_is_never_flagged_however_rigid_the_offset():
    """THE false-positive guard. A parked arm next to a parked obstacle also has
    a perfectly constant offset — flagging that would condemn every static
    scene, which is most of them."""
    robot = np.tile(np.array([0.3, 0.0, 0.5]), (80, 1))
    human = robot + np.array([0.09, 0.0, 0.0])
    mon = SelfDetectionMonitor()
    assert not any(_run(mon, robot, human))


def test_a_moving_obstacle_tracked_by_the_arm_is_not_flagged():
    """The case most likely to be confused with self-detection: the CBF is
    holding a near-constant distance to an approaching obstacle, so `d` barely
    changes. The OFFSET still changes — the two are moving in the world
    independently — which is why the test is on the offset and not on d."""
    n = 60
    robot = _sweep(n)
    human = robot + np.stack([np.linspace(0.09, 0.0, n),
                              np.linspace(0.0, 0.09, n),
                              np.zeros(n)], axis=1)
    mon = SelfDetectionMonitor()
    assert not any(_run(mon, robot, human))


# ── A wrong extrinsic must not defeat it ────────────────────────────────────

def test_a_displaced_calibration_is_still_caught():
    """The reason the test is kinematic. A stale extrinsic offsets EVERY
    unprojected point by the same rigid transform, so a geometric "is this
    inside my capsule model?" test finds nothing — while the offset is still
    perfectly constant, which is what this measures."""
    robot = _sweep()
    human = robot + np.array([0.18, -0.11, 0.07])
    mon = SelfDetectionMonitor()
    assert _run(mon, robot, human)[-1] is True


def test_a_rotating_arm_carrying_its_own_leakage_is_caught():
    """Self-detection is not only translational: an arm rotating about the base
    carries the leaked pixels around with it."""
    n = 60
    th = np.linspace(0.0, 1.1, n)
    robot = np.stack([0.6 * np.cos(th), 0.6 * np.sin(th), np.full(n, 0.4)], 1)
    human = robot * 1.12                    # radially outward, rides along
    mon = SelfDetectionMonitor(offset_tol_m=0.05)
    assert _run(mon, robot, human)[-1] is True


# ── Hysteresis ──────────────────────────────────────────────────────────────

def test_the_verdict_needs_confirmation_frames():
    """One frame of coincidence is not a verdict — the flag suppresses a real
    input to the barrier."""
    robot = _sweep(n=40)
    human = robot + np.array([0.09, 0.0, 0.0])
    mon = SelfDetectionMonitor(confirm=5)
    flags = _run(mon, robot, human)
    assert not flags[0] and flags[-1]
    assert sum(flags) < len(flags), 'it must not flag from the very first frame'


def test_the_verdict_is_sticky():
    """A calibration does not come and go, so one clean frame must not clear a
    standing verdict — otherwise the suppression would chatter and the barrier
    would see the arm's own velocity intermittently."""
    robot = _sweep()
    human = robot + np.array([0.09, 0.0, 0.0])
    mon = SelfDetectionMonitor(release=15)
    _run(mon, robot, human)
    assert mon.is_self('fr3_link5#0')
    mon.update('fr3_link5#0', robot[-1], np.array([0.95, 0.0, 0.5]))
    assert mon.is_self('fr3_link5#0'), 'cleared after a single clean frame'


def test_the_verdict_is_eventually_withdrawn():
    """Sticky, not permanent: a recalibration mid-session, or an obstacle that
    really does appear, must be able to clear it."""
    robot = _sweep()
    mon = SelfDetectionMonitor(release=15)
    _run(mon, robot, robot + np.array([0.09, 0.0, 0.0]))
    assert mon.is_self('fr3_link5#0')
    _run(mon, _sweep(), np.tile(np.array([0.95, 0.0, 0.5]), (60, 1)))
    assert not mon.is_self('fr3_link5#0')


# ── Bookkeeping ─────────────────────────────────────────────────────────────

def test_control_points_are_judged_independently():
    """One leaking control point must not condemn the whole arm."""
    robot = _sweep()
    mon = SelfDetectionMonitor()
    _run(mon, robot, robot + np.array([0.09, 0.0, 0.0]), key='fr3_link5#0')
    _run(mon, robot, np.tile(np.array([0.95, 0.0, 0.5]), (robot.shape[0], 1)),
         key='fr3_link8#0')
    assert mon.flagged == {'fr3_link5#0'}


def test_the_report_names_the_offender_and_the_cause():
    robot = _sweep()
    mon = SelfDetectionMonitor()
    assert mon.report() is None
    _run(mon, robot, robot + np.array([0.09, 0.0, 0.0]))
    msg = mon.report()
    assert msg is not None and 'fr3_link5#0' in msg
    # Actionable, not merely alarming: it must name what to go and look at.
    assert 'camera_extrinsics' in msg and 'dilate' in msg


def test_non_finite_input_is_ignored_not_trusted():
    robot = _sweep()
    mon = SelfDetectionMonitor()
    for pr in robot:
        mon.update('cp', pr, np.array([np.nan, 0.0, 0.0]))
    assert not mon.is_self('cp')


def test_reset_clears_everything():
    robot = _sweep()
    mon = SelfDetectionMonitor()
    _run(mon, robot, robot + np.array([0.09, 0.0, 0.0]))
    mon.reset()
    assert mon.flagged == set() and mon.report() is None


# ── Suppression is of the ESTIMATE, never of the constraint ─────────────────

def test_annotation_skips_a_flagged_control_point():
    """The safety-relevant half: a flagged control point keeps the all-zero
    "no track" defaults — no velocity, no covariance — while its DISTANCE and
    every other field go out untouched. Suppressing an estimate degrades to
    today's behaviour; suppressing a distance would delete a barrier."""
    import pytest
    msgs = pytest.importorskip('franka_msgs.msg')
    from franka_experiments.utils.perception_msgs import annotate_track_fields

    class _Pipe:
        def velocity_for_point(self, p):
            return 9, 42, np.array([1.0, 0.0, 0.0]), np.eye(3) * 0.5

    msg = msgs.MultiLinkDistance()
    links = []
    for name in ('fr3_link5', 'fr3_link5', 'fr3_link8'):
        ld = msgs.LinkDistance()
        ld.robot_link_name = name
        ld.valid = True
        ld.distance = 0.21
        links.append(ld)
    msg.links = links

    n = annotate_track_fields(msg, _Pipe(), skip_keys={'fr3_link5#1'})
    assert n == 2
    assert msg.links[0].track_id == 9 and msg.links[2].track_id == 9
    # The flagged one: no velocity, but the distance survives.
    assert msg.links[1].track_id == 0 and msg.links[1].frames_seen == 0
    assert np.array_equal(np.asarray(msg.links[1].velocity_covariance),
                          np.zeros(9))
    assert msg.links[1].distance == 0.21 and msg.links[1].valid is True
