"""The SSB scenarios put the obstacle in the self-filter's deleted band.

Every number in ``ssb_poke`` and ``ssb_band_sweep`` is DERIVED from
fr3_complete.yaml's ``mask:`` block, the ``diag`` camera extrinsics and the
control points at q0 — none of them is a chosen constant. This test recomputes
the derivation and checks the catalog against it, so a change to
``depth_gate_bias_m``, ``depth_gate_tol_m``, ``ssb_guard_m``, the camera pose
or the control-point set fails here instead of silently turning the scenarios
into ordinary approach runs.

THE DERIVATION. A masked pixel survives the gate as an obstacle only when it is
in front of ``surf - tol``, with ``surf = z_model - bias``; ``self_shadow``
calls a deleted return a CANDIDATE when it is in front of ``surf`` by more than
``guard``. Both are OPTICAL-AXIS depths, so

    bias + guard  <  z_model - z_near  <  bias + tol

where ``z_near`` is the obstacle's CAMERA-facing surface. The barrier's surface
gap is measured to the ROBOT-facing surface, one diameter deeper:

    gap  =  (z_model - z_near)  -  2 * r_ball

At the shipped bias/tol that band is gap in (0.10 - 2r, 0.16 - 2r), which for
any ball over 3 cm in radius lies ENTIRELY inside d_safe = 0.10 m. That bound
is itself asserted below, because it is the limit on everything SSB can claim.
"""
import os
import sys

import numpy as np
import pytest
import yaml

from simexp import scenario_catalog as SC

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..'))
CP = 'fr3_link8#0'
R_BALL = 0.04


@pytest.fixture(scope='module')
def geom():
    """Camera pose, gate numbers, and the control points at q0."""
    from scipy.spatial.transform import Rotation as R
    from simexp.truth_geometry import TruthGeometry, load_perception_config, cp_label
    sys.path.insert(0, os.path.join(REPO, 'franka_simulation', 'scripts'))
    from utils import gz_robot_description as G

    cfg = SC.load()
    pc = load_perception_config(os.path.join(REPO, 'franka_experiments', 'config',
                                             'fr3_complete.yaml'))
    ctrl = yaml.safe_load(open(os.path.join(REPO, 'franka_experiments', 'config',
                                            'fr3_control.yaml')))
    e = yaml.safe_load(open(os.path.join(REPO, 'franka_simulation', 'config', 'sim',
                                         'camera_extrinsics_diag.yaml')))
    q = e['rotation']
    _, rsp = G.build_descriptions(G.process_xacro(), initial_q=cfg['q0'])
    cps = TruthGeometry(rsp, pc).control_points(cfg['q0'])
    cp = next(c for c in cps if cp_label(c) == CP)
    return dict(
        cfg=cfg, cps=cps, cp_label=cp_label,
        p=np.asarray(cp['point'], float), r_cp=float(cp['radius']),
        axis=R.from_quat([q['x'], q['y'], q['z'], q['w']]).as_matrix()[:, 2],
        bias=float(pc['mask']['depth_gate_bias_m']),
        tol=float(pc['mask']['depth_gate_tol_m']),
        guard=float(pc['distance']['ssb_guard_m']),
        d_safe=float(ctrl['params']['d_safe']),
    )


def band_gap(g, r_ball=R_BALL):
    """``(gap_near, gap_far)``: the surface gaps the deleted band corresponds to."""
    return (g['bias'] + g['guard'] - 2 * r_ball, g['bias'] + g['tol'] - 2 * r_ball)


def test_the_deleted_band_lies_inside_d_safe(geom):
    """The finding that bounds every SSB claim, asserted rather than asserted-about.

    If this ever stops holding — a smaller bias, a smaller tol, or a much
    smaller obstacle — then SSB could prevent a d_safe violation rather than
    only bound the approach after one, and SSB_VALIDATION_REPORT.md's central
    caveat would need rewriting.
    """
    near, far = band_gap(geom)
    assert far <= geom['d_safe'] + 1e-12, (
        f'the deleted band now reaches gap {far:.3f} m, OUTSIDE '
        f"d_safe {geom['d_safe']:.3f} — re-derive the validation report's "
        'central caveat')
    assert near >= 0.0, 'the band now starts inside the capsule surface'


def _on_ray(g, gap, r_ball=R_BALL):
    """The ball centre whose SURFACE GAP to the control point is ``gap``."""
    return g['p'] - g['axis'] * (gap + r_ball + g['r_cp'])


def test_ssb_poke_is_on_the_camera_ray_and_crosses_the_band(geom):
    sc = SC.resolve('ssb_poke', cfg=geom['cfg'])
    assert sc['scene'] == 'diag_bare'        # a furnished room floods the QP
    assert sc['nominal']['law'] == 'joint_pd'     # only the obstacle moves
    o, = sc['obstacles']
    assert o['law'] == 'linear' and o['radius'] == R_BALL and not o['park']

    gap0 = 0.40
    np.testing.assert_allclose(o['p0'], _on_ray(geom, gap0), atol=5e-4)
    # the velocity is ALONG the optical axis, i.e. straight at the camera's
    # own line of sight to that control point, which is what puts the ball in
    # front of the robot's silhouette rather than beside it
    v = np.asarray(o['v0'], float)
    speed = float(np.linalg.norm(v))
    assert speed == pytest.approx(0.08, abs=5e-4)
    np.testing.assert_allclose(v / speed, geom['axis'], atol=5e-4)

    # the whole band is crossed well inside T, and contact is reached
    near, far = band_gap(geom)
    t_in, t_out, t_touch = ((gap0 - far) / speed, (gap0 - near) / speed,
                            gap0 / speed)
    assert 0.0 < t_in < t_out < t_touch < sc['T']
    assert t_out - t_in == pytest.approx(0.75, abs=0.02)

    # nothing starts inside the barrier (franka_rl's rule)
    gaps = [np.linalg.norm(np.asarray(o['p0']) - np.asarray(c['point']))
            - R_BALL - float(c['radius']) for c in geom['cps']]
    assert min(gaps) >= geom['d_safe']


def test_ssb_band_sweep_dwells_in_the_band_and_is_born_outside(geom):
    sc = SC.resolve('ssb_band_sweep', cfg=geom['cfg'])
    assert sc['scene'] == 'diag_bare' and sc['nominal']['law'] == 'joint_pd'
    o, = sc['obstacles']
    assert o['law'] == 'oscillate' and o['radius'] == R_BALL and not o['park']
    np.testing.assert_allclose(o['axis'], geom['axis'], atol=5e-4)
    assert o['phase_deg'] == 0.0           # born AT p0, the far end of the swing

    gap_far = 0.26
    np.testing.assert_allclose(o['p0'], _on_ray(geom, gap_far), atol=5e-4)
    # born outside the barrier, and the near extreme reaches past the band
    gaps = [np.linalg.norm(np.asarray(o['p0']) - np.asarray(c['point']))
            - R_BALL - float(c['radius']) for c in geom['cps']]
    assert min(gaps) >= geom['d_safe']
    near, _far = band_gap(geom)
    assert gap_far - o['amplitude'] <= near + 1e-9, (
        'the swing no longer reaches the near edge of the deleted band')
    # and the band is crossed several times in T
    assert sc['T'] / o['period'] >= 4.0


def test_the_scenarios_aimed_at_the_band_are_exactly_these(geom):
    """A guard against a later scenario quietly becoming an SSB scenario.

    SSB's arms are only meaningful on a run that puts an obstacle inside the
    deleted band; anywhere else they measure the mechanism's COST. This test
    records which scenarios are the former, so the validation report's scope
    stays checkable.
    """
    assert {n for n in SC.all_names(geom['cfg']) if n.startswith('ssb_')} == {
        'ssb_poke', 'ssb_band_sweep', 'ssb_poke_fast', 'ssb_poke_slow',
        'ssb_poke_noisy', 'ssb_poke_dropout'}


def test_the_poke_variants_change_exactly_one_thing_each(geom):
    """The sensitivity variants must stay attributable.

    Each one differs from ``ssb_poke`` in a single respect -- speed, or a depth
    adapter stage -- and shares its birth point and its direction, so a change
    in the detection numbers can be read as caused by that one thing. A variant
    that drifted off the camera ray would measure a different geometry and
    silently invalidate the comparison, so the ray is checked, not assumed.
    """
    cfg = geom['cfg']
    base = SC.resolve('ssb_poke', cfg=cfg)['obstacles'][0]
    v0 = np.asarray(base['v0'], float)
    u0 = v0 / np.linalg.norm(v0)
    expect = {'ssb_poke_fast': 2.0, 'ssb_poke_slow': 0.5,
              'ssb_poke_noisy': 1.0, 'ssb_poke_dropout': 1.0}
    seen_stages = {}
    for name, factor in expect.items():
        sc = SC.resolve(name, cfg=cfg)
        ob = sc['obstacles'][0]
        assert len(sc['obstacles']) == 1, name
        assert np.allclose(ob['p0'], base['p0']), name
        assert ob['radius'] == base['radius'], name
        assert ob['law'] == base['law'] == 'linear', name
        v = np.asarray(ob['v0'], float)
        # same ray to 0.2 degrees, speed scaled by the intended factor to 1 %
        assert float(u0 @ (v / np.linalg.norm(v))) > 0.999994, name
        assert abs(np.linalg.norm(v) / np.linalg.norm(v0) - factor) < 0.01, name
        # the GAZEBO block is consumed by resolve() into the launch config, so
        # it has to be read off the raw entry, not off the resolved one. These
        # scenarios live under `tests:`, not `catalog:`.
        seen_stages[name] = ((cfg['tests'][name].get('gazebo') or {})
                             .get('depth_adapter') or {})
    # the degradation arms carry a stage and the speed arms carry none
    assert seen_stages['ssb_poke_fast'] == {}
    assert seen_stages['ssb_poke_slow'] == {}
    assert seen_stages['ssb_poke_noisy'] == {'range_noise': True}
    assert seen_stages['ssb_poke_dropout'] == {'dropout_p': 0.1}
    assert not (cfg['tests']['ssb_poke'].get('gazebo') or {}).get(
        'depth_adapter'), 'ssb_poke itself must stay un-degraded'


def test_the_fast_variant_still_resolves_the_band_crossing(geom):
    """2x speed must not shrink the crossing below what perception can see.

    The band is ``tol - guard`` = 6 cm deep in depth, which the derivation maps
    to a 6 cm span of surface gap along this ray. At the measured distance-row
    rate (~32 Hz with the SSB pass on) the crossing has to last long enough for
    the cluster-support test to fire at all; below ~5 frames a negative result
    would say nothing about the classifier.
    """
    cfg = geom['cfg']
    depth = geom['tol'] - geom['guard']
    for name, lo in (('ssb_poke', 20.0), ('ssb_poke_fast', 10.0)):
        v = np.linalg.norm(np.asarray(
            SC.resolve(name, cfg=cfg)['obstacles'][0]['v0'], float))
        frames = depth / v * 32.0
        assert frames >= lo, (name, frames)
