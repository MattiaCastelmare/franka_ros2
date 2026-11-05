"""``phase_deg`` on law: oscillate -- the truth side, and that it reaches the plugin.

The motion the arm has to avoid exists TWICE: in the Gazebo world system
(franka_simulation/src/scenario_obstacles_system.cpp, which moves the sphere)
and in simexp.obstacle_laws (which is what every truth-based metric compares
against). The phase has to mean the same thing in both, and the only full check
of that is a running Gazebo (test/slow/test_phase4_obstacles.py, whose p4_laws
obstacle 4 oscillates with a non-zero phase). These fast tests cover what can
be checked without one: the truth law's own algebra, that the SDF writer emits
the tag the C++ reads, and that the default leaves every shipped scenario alone.
"""
import os

import numpy as np
import pytest
import yaml

from simexp import obstacle_laws as L
from simexp import scenario_catalog as SC

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..'))
PLUGIN_CPP = os.path.join(REPO, 'franka_simulation', 'src', 'scenario_obstacles_system.cpp')

SPEC = dict(p0=[0.4, -0.35, 0.62], law='oscillate', axis=[0.0, 1.0, 0.0],
            amplitude=0.6, period=10.0, phase_deg=-90.0)


def test_minus_90_degrees_is_minus_cosine():
    """The property the scenarios rely on: born at the FAR end of the swing."""
    A, P = SPEC['amplitude'], SPEC['period']
    for t in (0.0, 0.3, 2.5, 7.7, 13.1):
        w = 2.0 * np.pi / P
        np.testing.assert_allclose(L.position(SPEC, t),
                                   np.array(SPEC['p0']) + [0, -A * np.cos(w * t), 0], atol=1e-12)
        np.testing.assert_allclose(L.velocity(SPEC, t),
                                   [0, A * w * np.sin(w * t), 0], atol=1e-12)
    # t = 0 is the far end, one amplitude from the centre, and at rest there.
    np.testing.assert_allclose(L.position(SPEC, 0.0), [0.4, -0.95, 0.62], atol=1e-12)
    np.testing.assert_allclose(L.velocity(SPEC, 0.0), [0, 0, 0], atol=1e-12)


def test_velocity_is_the_derivative_of_position():
    for t in (0.0, 1.7, 4.2, 9.9):
        h = 1e-6
        fd = (L.position(SPEC, t + h) - L.position(SPEC, t - h)) / (2 * h)
        np.testing.assert_allclose(L.velocity(SPEC, t), fd, atol=1e-7)


def test_a_missing_or_zero_phase_is_the_old_pure_sine():
    """The default must leave every pre-existing oscillate byte-identical."""
    bare = {k: v for k, v in SPEC.items() if k != 'phase_deg'}
    zero = dict(bare, phase_deg=0.0)
    w = 2.0 * np.pi / SPEC['period']
    for t in (0.0, 1.3, 6.6):
        want = np.array(SPEC['p0']) + [0, SPEC['amplitude'] * np.sin(w * t), 0]
        for spec in (bare, zero):
            np.testing.assert_allclose(L.position(spec, t), want, atol=1e-12)
        np.testing.assert_allclose(L.position(bare, t), L.position(zero, t), atol=0)
        np.testing.assert_allclose(L.velocity(bare, t), L.velocity(zero, t), atol=0)


def test_the_resolver_carries_the_phase_and_defaults_it_to_zero():
    """Dropping the key here would silently unshift the oscillation."""
    r = SC.resolve('pentagon_lateral_osc')
    assert r['obstacles'][0]['phase_deg'] == -90.0
    assert SC.resolve('pentagon_sweep')['obstacles'][0]['phase_deg'] == 0.0
    assert SC.resolve('pentagon_on_path')['obstacles'][0]['phase_deg'] == 0.0


def test_the_sdf_tag_is_the_name_the_plugin_reads():
    """gz_world writes <phase_deg>; the C++ must Get<double>("phase_deg")."""
    from utils import gz_world
    sdf = gz_world.scenario_obstacles_plugin([dict(SPEC, name='obstacle_0', radius=0.06)])
    assert '<phase_deg>-90</phase_deg>' in sdf
    src = open(PLUGIN_CPP).read()
    assert 'o->Get<double>("phase_deg", 0.0).first * M_PI / 180.0' in src
    # Both sides add the phase INSIDE the sine's argument, not to the result.
    assert 'std::sin(w * t + ob.phase)' in src and 'std::cos(w * t + ob.phase)' in src
    # And an obstacle with no phase still emits the neutral value.
    plain = gz_world.scenario_obstacles_plugin([dict(name='o', radius=0.05, p0=[0, 0, 0],
                                                     law='static')])
    assert '<phase_deg>0</phase_deg>' in plain


def test_the_spawn_clearance_rule_asks_the_law_not_p0():
    """A phase moves the birth, so check_start/enforce must follow it there."""
    # 20 mm from the BIRTH position, so the gap there is 0.02 - 0.06 - 0.05 =
    # -0.09 m while p0 is 0.6 m away and clear: a rule reading p0 would call
    # this start safe. (Not ON the birth: then the shift direction is 0/0.)
    cps = [dict(point=[0.4, -0.93, 0.62], radius=0.05, end_link='fr3_link8', cp_idx=0)]
    r = dict(obstacles=[dict(SPEC, name='obstacle_0', radius=0.06)])
    chk = SC.check_start(r, cps, d_safe=0.10)
    assert not chk['ok'] and chk['min_gap'] == pytest.approx(-0.09, abs=1e-9)
    out = SC.enforce_start_clearance(r, cps, d_safe=0.10)
    assert out['start_shift'] and SC.check_start(out, cps, d_safe=0.10)['ok']
    # p0 is what moved, and the whole trajectory moved rigidly with it.
    d = np.asarray(out['obstacles'][0]['p0']) - np.asarray(SPEC['p0'])
    np.testing.assert_allclose(L.position(out['obstacles'][0], 3.3) - L.position(SPEC, 3.3),
                               d, atol=1e-12)


def test_the_scenario_is_born_far_from_the_arm_and_crosses_the_path():
    """What the entry is for: a distant spawn, then repeated real encounters."""
    with open(os.path.join(REPO, 'franka_experiments', 'config', 'fr3_control.yaml')) as f:
        P = yaml.safe_load(f)['params']
    r = SC.resolve('pentagon_lateral_osc')
    spec = r['obstacles'][0]
    cy, cz = P['path_center_xyz'][1], P['path_center_xyz'][2]
    half = np.sqrt(P['path_radius'] ** 2 - (spec['p0'][2] - cz) ** 2)   # chord at z = 0.62
    y = np.array([L.position(spec, t)[1] for t in np.linspace(0, r['T'], 60001)])
    assert y[0] == min(y)                      # born at the far end
    assert y.min() < -0.9 and y.max() > half   # sweeps in past the far crossing
    inside = np.abs(y - cy) <= half
    assert int(np.sum(np.diff(inside.astype(int)) == 1)) == 12   # 2 per 10 s period
