"""Derivatives of the actual perception geometry, not a second CP model."""
from pathlib import Path

import numpy as np
import pytest
import yaml

pin = pytest.importorskip('pinocchio')
from franka_experiments.utils.distance_utils import define_control_points
from franka_experiments.utils.kinematics import CBFKinematics, build_urdf_no_hand


@pytest.fixture(scope='module')
def geometry():
    cfg = yaml.safe_load((Path(__file__).parents[1] / 'config/fr3_complete.yaml').read_text())
    model = pin.buildModelFromUrdf(build_urdf_no_hand())
    kin = CBFKinematics(model)
    kin.configure_control_points(cfg['robot'], cfg['distance'])
    return cfg, model, kin


def points(cfg, model, q):
    data = model.createData()
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    tf = {name: (data.oMf[model.getFrameId(name)].rotation,
                 data.oMf[model.getFrameId(name)].translation)
          for name in cfg['robot']['segment_links']}
    return define_control_points(tf, cfg['robot'], cfg['distance'])


def labelled(cps):
    seen = {}
    for cp in sorted(cps, key=lambda p: (p['seg_idx'], p['cp_idx'])):
        link = cp['end_link']
        k = seen.get(link, 0)
        seen[link] = k + 1
        yield f'{link}#{k}', cp


def test_all_configured_cp_jacobians_match_perception_finite_differences(geometry):
    cfg, model, kin = geometry
    rng = np.random.default_rng(27)
    for _ in range(8):
        q = np.array([0., -.7, 0., -2.35, 0., 1.57, .7]) + rng.uniform(-.3, .3, 7)
        kin.update(q, np.zeros(model.nv))
        eps = 1e-6
        fd = np.empty((11, 3, 7))
        for j in range(7):
            d = np.zeros(model.nv); d[j] = eps
            pp = np.array([p['point'] for p in points(cfg, model, pin.integrate(model, q, d))])
            pm = np.array([p['point'] for p in points(cfg, model, pin.integrate(model, q, -d))])
            fd[:, :, j] = (pp - pm) / (2*eps)
        for i, (label, _) in enumerate(labelled(points(cfg, model, q))):
            J, _ = kin.control_point_jacobian(label)
            np.testing.assert_allclose(J, fd[i], atol=2e-8, rtol=1e-7)
            np.testing.assert_array_equal(J, kin.control_point_jacobian(label+'.1')[0])


def test_cp_jdot_is_the_directional_derivative_of_the_same_jacobian(geometry):
    cfg, model, kin = geometry
    rng = np.random.default_rng(12)
    for _ in range(8):
        q = np.array([0., -.7, 0., -2.35, 0., 1.57, .7]) + rng.uniform(-.3, .3, 7)
        v = rng.uniform(-1.2, 1.2, model.nv)
        for label, _ in labelled(points(cfg, model, q)):
            eps = 1e-6
            kin.update(q, v)
            _, dJ = kin.control_point_jacobian(label)
            kin.update(pin.integrate(model, q, eps*v), v)
            Jp, _ = kin.control_point_jacobian(label)
            kin.update(pin.integrate(model, q, -eps*v), v)
            Jm, _ = kin.control_point_jacobian(label)
            np.testing.assert_allclose(dJ, (Jp-Jm)/(2*eps), atol=2e-8, rtol=1e-6)


def test_rigid_point_jdot_includes_rotating_world_offset(geometry):
    _, model, kin = geometry
    q = np.array([.2, -.6, .3, -2.1, -.2, 1.8, .5])
    v = np.array([.8, -.7, .6, -.5, .4, -.3, .2])
    fid = model.getFrameId('fr3_link7')
    local = np.array([.12, -.06, .09])
    kin.update(q, v)
    _, dJ = kin.point_jacobian(fid, kin.data.oMf[fid].act(local))
    eps = 1e-6
    kin.update(pin.integrate(model, q, eps*v), v)
    Jp = kin.point_jacobian_pos(fid, kin.data.oMf[fid].act(local))
    kin.update(pin.integrate(model, q, -eps*v), v)
    Jm = kin.point_jacobian_pos(fid, kin.data.oMf[fid].act(local))
    np.testing.assert_allclose(dJ, (Jp-Jm)/(2*eps), atol=2e-8)


def test_builder_uses_the_configured_geometry_for_obstacle_rows(geometry):
    from _cbf_builder_harness import P
    from franka_experiments.utils.cbf_state_rows import (
        ConstraintBuilder, JointSnap, Obstacle, ObstacleSnap)
    cfg, model, kin = geometry
    q = np.array([.2, -.6, .3, -2.1, -.2, 1.8, .5])
    v = np.zeros(7)
    kin.update(q, v)
    label, cp = next(labelled(points(cfg, model, q)))
    n = np.array([1., 0., 0.])
    ob = Obstacle(cp['end_link'], .3, cp['point'], cp['point']-.4*n, 1., cp_label=label)
    b = ConstraintBuilder(P(), kin, q_min=q-1., q_max=q+1.,
                          acc_lb=-np.ones(7)*10, acc_ub=np.ones(7)*10,
                          control_point_jacobian=kin.control_point_jacobian)
    con = b.build(JointSnap(q, v, 1.), ObstacleSnap((ob,), 1., 1.), 1.)
    J, _ = kin.control_point_jacobian(label)
    np.testing.assert_allclose(con.A[0], n @ J)
