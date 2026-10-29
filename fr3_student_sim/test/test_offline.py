"""Offline checks (no Gazebo): URDF, model, wall, controller loading.

    cd /ros2_ws && colcon test --packages-select fr3_student_sim && colcon test-result --verbose
"""
import os

import numpy as np
import pytest

from fr3_student_sim.robot_description import build_urdf, kdl_tip_link, process_xacro
from fr3_student_sim.wall import Wall

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
Q0 = np.array([0.0, -0.1517, 0.0, -2.6105, 0.0, 4.0295, 0.785])


@pytest.fixture(scope='module')
def urdf():
    return build_urdf(process_xacro(), initial_q=Q0)


@pytest.fixture(scope='module')
def robot(urdf):
    from fr3_student_sim.fr3_model import FR3Model
    return FR3Model(urdf)


def test_kdl_gravity_chain_covers_the_hand(urdf):
    # The plugin compensates gravity along root -> tip; with the fingers lumped
    # into the hand the tip must lie beyond fr3_hand, else the hand mass is missed.
    assert kdl_tip_link(urdf) == 'fr3_fingertip'


def test_model_matches_known_values(robot):
    p, R = robot.pose(Q0)
    np.testing.assert_allclose(p, [0.56, 0.0, 0.50], atol=1e-3)
    np.testing.assert_allclose(R[:, 2], [1, 0, 0], atol=1e-3)      # tool points at the wall (+x)
    tcp = robot.position(Q0, 'fr3_hand_tcp')
    np.testing.assert_allclose(p - tcp, [0.0095, 0, 0], atol=1e-6)  # fingertip beyond the TCP
    assert abs(sum(i.mass for i in robot.model.inertias) - 18.604) < 1e-2


def test_coriolis_is_skew_consistent(robot):
    rng = np.random.default_rng(0)
    dq = rng.standard_normal(7)
    C = robot.coriolis_matrix(Q0, dq)
    np.testing.assert_allclose(C @ dq, robot.coriolis(Q0, dq), atol=1e-10)
    Md = (robot.mass_matrix(Q0 + 1e-6 * dq) - robot.mass_matrix(Q0 - 1e-6 * dq)) / 2e-6
    S = Md - 2 * C
    np.testing.assert_allclose(S, -S.T, atol=1e-6)


def test_effective_mass_formula(robot):
    n = np.array([1.0, 0.0, 0.0])
    J, Mi = robot.jacobian_pos(Q0), np.linalg.inv(robot.mass_matrix(Q0))
    assert robot.effective_mass(Q0, n) == pytest.approx(1.0 / (n @ J @ Mi @ J.T @ n))
    assert robot.effective_mass(Q0, 3 * n) == pytest.approx(robot.effective_mass(Q0, -n))


def test_mass_error_only_in_the_plant(urdf):
    from fr3_student_sim.fr3_model import FR3Model
    plant = FR3Model(build_urdf(process_xacro(), initial_q=Q0, mass_error=0.05, seed=3))
    nominal = FR3Model(urdf)
    m_p = np.array([i.mass for i in plant.model.inertias])
    m_n = np.array([i.mass for i in nominal.model.inertias])
    ratio = m_p[m_n > 0] / m_n[m_n > 0]
    assert np.all(np.abs(ratio - 1) <= 0.05 + 1e-12) and np.any(np.abs(ratio - 1) > 0.01)
    again = FR3Model(build_urdf(process_xacro(), initial_q=Q0, mass_error=0.05, seed=3))
    np.testing.assert_allclose(again.mass_matrix(Q0), plant.mass_matrix(Q0))   # seeded


def test_hand_depth(robot):
    from fr3_student_sim.body_check import body_depth
    tip = robot.position(Q0)
    w = Wall(point=tip, normal=[-1, 0, 0])            # wall touching the fingertip
    assert body_depth(robot, Q0, w) == pytest.approx(0.0, abs=1e-3)   # the tips lead
    w_side = Wall(point=tip, normal=[0, 0, 1])        # floor at fingertip height
    assert body_depth(robot, Q0, w_side) > 0.02       # the hand hangs below the tip line


def test_batch_merge():
    from fr3_student_sim.run_batch import merge
    base = {'a': 1, 'circle': {'radius': 0.04, 'period': 8.0}}
    out = merge(base, {'circle': {'radius': 0.05}, 'mode': 'x'})
    assert out == {'a': 1, 'circle': {'radius': 0.05, 'period': 8.0}, 'mode': 'x'}
    assert base['circle']['radius'] == 0.04


def test_jdot_dq_matches_finite_difference(robot):
    dq = np.array([0.1, -0.2, 0.1, 0.3, -0.1, 0.2, 0.1])
    h = 1e-6
    Jdot = (robot.jacobian_pos(Q0 + h * dq) - robot.jacobian_pos(Q0 - h * dq)) / (2 * h)
    np.testing.assert_allclose(robot.jdot_dq(Q0, dq), Jdot @ dq, atol=1e-6)


def test_wall_spring_damper():
    w = Wall(point=[0.5, 0, 0], normal=[-1, 0, 0], stiffness=1500, damping=15)
    F, f_n, delta, _ = w.contact(np.array([0.49, 0, 0]), np.array([0.1, 0, 0]))
    assert f_n == 0 and delta < 0                                  # outside
    F, f_n, delta, v_n = w.contact(np.array([0.51, 0, 0]), np.array([0.1, 0, 0]))
    assert delta == pytest.approx(0.01) and v_n == pytest.approx(0.1)
    assert f_n == pytest.approx(1500 * 0.01 + 15 * 0.1)            # compressing
    np.testing.assert_allclose(F, [-f_n, 0, 0])
    _, f_n, _, _ = w.contact(np.array([0.51, 0, 0]), np.array([-0.1, 0, 0]))
    assert f_n == pytest.approx(15.0)                               # no damping releasing


@pytest.mark.parametrize('path', [
    'students/examples/zero_torque.py',
    'students/examples/joint_pd.py', 'students/examples/cartesian_pd.py'])
def test_examples_load_and_run(robot, path):
    # only the files we provide: the students' own controllers may do anything at t = 0
    import yaml
    from fr3_student_sim.runner import load_controller_class
    with open(os.path.join(PKG, 'config', 'task.yaml')) as f:
        params = yaml.safe_load(f)
    ctrl = load_controller_class(os.path.join(PKG, path))(robot, params, 1e-3)
    tau = np.asarray(ctrl.update(0.0, Q0, np.zeros(7)))
    assert tau.shape == (7,) and np.all(np.isfinite(tau))
    if 'joint_pd' not in path:            # joint_pd starts with a velocity reference
        assert np.max(np.abs(tau)) < 1.0  # at rest on the reference: ~no torque


def test_controller_can_import_a_sibling_file(tmp_path):
    from fr3_student_sim.runner import load_controller_class
    (tmp_path / 'my_helpers_xyz.py').write_text('K = 3\n')
    (tmp_path / 'ctrl.py').write_text(
        'from my_helpers_xyz import K\n'
        'from fr3_student_sim.controller_base import StudentController\n'
        'class Controller(StudentController):\n    k = K\n')
    assert load_controller_class(str(tmp_path / 'ctrl.py')).k == 3
