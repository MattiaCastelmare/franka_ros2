"""Independent FD/classical-acceleration and perception-to-QP regressions.

Requires the sourced ROS workspace; never connects to or launches a robot.
"""
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

pin = pytest.importorskip('pinocchio')
from franka_experiments.utils.kinematics import CBFKinematics, build_urdf_no_hand
from franka_experiments.utils.distance_utils import define_control_points

Q = np.array([.2, -.6, .3, -2.1, -.2, 1.8, .5])
V = np.array([.8, -.7, .6, -.5, .4, -.3, .2])
ACC = np.array([.2, -.1, .3, -.2, .4, -.3, .1])
EPS = 1e-6


@pytest.fixture(scope='module')
def model_config():
    cfg = yaml.safe_load((Path(__file__).parents[1]/'config/fr3_complete.yaml').read_text())
    path = Path(build_urdf_no_hand())
    try:
        model = pin.buildModelFromUrdf(str(path))
    finally:
        path.unlink()
    return model, cfg


def perception(model, cfg, q):
    data = model.createData()
    pin.framesForwardKinematics(model, data, q)
    tf = {f: (data.oMf[model.getFrameId(f)].rotation.copy(),
              data.oMf[model.getFrameId(f)].translation.copy())
          for f in cfg['robot']['segment_links']}
    return define_control_points(tf, cfg['robot'], cfg['distance'])


def position_fd(model, cfg, q):
    return np.stack([
        (np.array([c['point'] for c in perception(model, cfg, q+e)])
         - np.array([c['point'] for c in perception(model, cfg, q-e)]))/(2*EPS)
        for e in np.eye(model.nv)*EPS], axis=-1)


@pytest.mark.parametrize('link,offset', [
    ('fr3_link4', [.08, -.05, .12]), ('fr3_link7', [.12, -.06, .09]),
    ('fr3_link8', [0., 0., .1]), ('fr3_link6', [0., 0., 0.])])
@pytest.mark.parametrize('scale', [0., 1., 2.])
def test_rigid_complete_acceleration_matches_pinocchio_and_fd(model_config, link, offset, scale):
    model, _ = model_config
    kin, fid = CBFKinematics(model), model.getFrameId(link)
    local, v = np.array(offset), scale*V
    kin.update(Q, v)
    point = kin.data.oMf[fid].act(local)
    J, D = kin.point_jacobian(fid, point)
    fr = model.frames[fid]
    data = model.createData()
    pin.forwardKinematics(model, data, Q, v, ACC)
    placement = fr.placement*pin.SE3(np.eye(3), local)
    classical = pin.getFrameClassicalAcceleration(
        model, data, fr.parentJoint, placement, pin.LOCAL_WORLD_ALIGNED).linear
    np.testing.assert_allclose(J@ACC+D@v, classical, atol=3e-12)
    js, ps = [], []
    h = 2e-4
    for sign in (1., -1.):
        kin.update(Q+sign*EPS*v, v, with_jdot=False)
        js.append(kin.point_jacobian_pos(fid, kin.data.oMf[fid].act(local)))
        kin.update(Q+sign*h*v+.5*h*h*ACC, v, with_jdot=False)
        ps.append(kin.data.oMf[fid].act(local))
    np.testing.assert_allclose(D, (js[0]-js[1])/(2*EPS), atol=2e-8)
    np.testing.assert_allclose(J@ACC+D@v, (ps[0]-2*point+ps[1])/h**2, atol=2e-6)


def test_nonconsecutive_interpolation_and_single_rigid_tip(model_config):
    """Parent attachment is not assumed, and lambda=1 is replaced by the tip."""
    model, original = model_config
    cfg = dict(robot=dict(original['robot'], segments=[
        dict(seg_idx=0, start_link='fr3_link1', end_link='fr3_link5', control_points=2),
        dict(seg_idx=1, start_link='fr3_link7', end_link='fr3_link8', control_points=1)]),
        distance=dict(original['distance'], ee_tip_axis=0, ee_tip_offset=.13))
    kin = CBFKinematics(model)
    kin.configure_control_points(cfg['robot'], cfg['distance'])
    labels = ['fr3_link5#0', 'fr3_link5#1', 'fr3_link8#0']
    for q in (Q, Q+.12, Q-.1):
        fd = position_fd(model, cfg, q)
        kin.update(q, V)
        pairs = [kin.control_point_jacobian(label) for label in labels]
        for i, (J, _) in enumerate(pairs):
            np.testing.assert_allclose(J, fd[i], atol=2e-8)
        kin.update(q+EPS*V, V)
        jp = [kin.control_point_jacobian(label)[0] for label in labels]
        kin.update(q-EPS*V, V)
        jm = [kin.control_point_jacobian(label)[0] for label in labels]
        for i, (_, D) in enumerate(pairs):
            np.testing.assert_allclose(D, (jp[i]-jm[i])/(2*EPS), atol=2e-8)
        kin.update(q, V, with_jdot=False)
        for i, label in enumerate(labels):
            J, D = kin.control_point_jacobian(label+'.2', with_jdot=False)
            assert D is None
            np.testing.assert_allclose(J, fd[i], atol=2e-8)


@pytest.mark.parametrize('invalid_first', [False, True])
def test_perception_messages_and_filter_callback_reach_consistent_qp(model_config, invalid_first):
    from builtin_interfaces.msg import Time
    from scipy import sparse
    import osqp
    from _cbf_builder_harness import make_params
    from franka_experiments.nodes.cbf_safety_filter import CBFSafetyFilter
    from franka_experiments.utils.distance_engine import ControlPointResult, ObstacleHit
    from franka_experiments.utils.perception_msgs import build_cp_messages, labelled_links
    from franka_experiments.utils.cbf_state_rows import ConstraintBuilder, JointSnap, NV, NX, N_SLACK
    from franka_experiments.utils.cbf_qp_assembly import build_row_rhs, build_osqp_A, build_osqp_bounds

    model, cfg = model_config
    cps = perception(model, cfg, Q)
    fd = position_fd(model, cfg, Q)
    kin = CBFKinematics(model)
    kin.configure_control_points(cfg['robot'], cfg['distance'])
    results = []
    for i, cp in enumerate(cps):
        n = np.linalg.svd(fd[i])[0][:, 0]
        r = ControlPointResult(**cp, distance=.12, direction=n,
                               closest_obstacle_point=cp['point']-.17*n)
        if cp['end_link']=='fr3_link7' and cp['cp_idx']==0:
            r.extras.append(ObstacleHit(.14, cp['point']-.19*n, n, 2))
        results.append(r)
    if invalid_first:
        results[0].distance = float('inf')
    _, msg = build_cp_messages(results, 1000, Time(sec=1), 'fr3_link0',
        cfg['robot']['segment_links'], dict(min_thresh=.01, max_thresh=1.), 2., {})
    assert len(msg.links)==12
    pairs = list(labelled_links(msg))
    assert 'fr3_link7#0.1' in [p[0] for p in pairs]
    for _, ld in pairs:
        # Publisher geometry must be the original perception point, unchanged.
        assert any(np.array_equal(c['point'], [ld.closest_point_robot.x,
                   ld.closest_point_robot.y, ld.closest_point_robot.z]) for c in cps)
    P = make_params()
    P.distance_capture_skew_tol, P.distance_capture_age_max = .05, 1.
    stub = SimpleNamespace(P=P, _now=lambda: 1., _track_input_rate=lambda t: None,
        _latency_fields=CBFSafetyFilter._latency_fields, _range_field=CBFSafetyFilter._range_field)
    CBFSafetyFilter._on_distances(stub, msg)
    builder = ConstraintBuilder(P, kin, q_min=Q-1., q_max=Q+1.,
        acc_lb=-np.ones(NV)*10, acc_ub=np.ones(NV)*10,
        control_point_jacobian=kin.control_point_jacobian)
    con = builder.build(JointSnap(Q, V, 1.), stub._obs, 1.)
    assert len(con.links)==12-int(invalid_first)
    rhs, _ = build_row_rhs(con, V, V, k0=P.k0_cbf, k1=P.k1_cbf,
                          retreat_horizon=.1, speed_horizon=.1)
    kin.update(Q+EPS*V, V)
    jp = {label: kin.control_point_jacobian(label)[0] for label in con.links}
    kin.update(Q-EPS*V, V)
    jm = {label: kin.control_point_jacobian(label)[0] for label in con.links}
    for i, label in enumerate(con.links):
        ob = next(o for o in stub._obs.items if o.cp_label==label)
        n = (ob.pr-ob.ph)/np.linalg.norm(ob.pr-ob.ph)
        index = list(kin._control_points).index(label.partition('.')[0])
        Aref = n@fd[index]
        drift = n@((jp[label]-jm[label])/(2*EPS))@V
        np.testing.assert_allclose(con.A[i], Aref, atol=2e-8)
        np.testing.assert_allclose(con.jdot_qdot[i], drift, atol=2e-8)
        np.testing.assert_allclose(-rhs[i], -P.k1_cbf*Aref@V-P.k0_cbf*con.h_bar[i]-drift, atol=2e-8)
    np.testing.assert_array_equal(con.G[:, :NV], -con.A)
    lower, upper = build_osqp_bounds(con.G, rhs, np.r_[-np.ones(NV)*10, np.zeros(N_SLACK)],
                                     np.r_[np.ones(NV)*10, np.ones(N_SLACK)*1e6])
    solver = osqp.OSQP()
    solver.setup(P=sparse.diags(np.r_[np.ones(NV), np.ones(N_SLACK)*1000], format='csc'),
        q=np.zeros(NX), A=build_osqp_A(con.G, NV, N_SLACK), l=lower, u=upper,
        eps_abs=1e-9, eps_rel=1e-9, max_iter=50000, verbose=False)
    sol = solver.solve()
    assert sol.info.status_val==1
    assert np.max(con.G@sol.x-rhs)<2e-7
