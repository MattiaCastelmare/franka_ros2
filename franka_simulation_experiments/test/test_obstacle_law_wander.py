"""The ``wander`` obstacle law: a quasi-random tour of a box.

    p_i(t) = p0_i + box_i · sin(2π ratio_i t / period + phase_i)

One independent sinusoid per WORLD axis, with incommensurate frequency ratios.
This module tests the TRUTH side (``simexp.obstacle_laws``), which is the same
motion ``src/scenario_obstacles_system.cpp`` runs in Gazebo — the two are the
truth and the sim of one law, so the constants are asserted to agree here and
the C++ defaults are quoted in the test that checks them.

What matters about this law, and is therefore checked:
  * it is BOUNDED by the box, so a scenario's box is a promise about where the
    ball can be — no clamp, no drift, no integrator;
  * ``velocity`` is the exact derivative of ``position`` (finite differences),
    because the CBF consumes that velocity as truth;
  * the ratios are IRRATIONAL, so the path does not close into a repeating
    Lissajous figure — that is the whole difference from ``oscillate``;
  * the birth point is p0 + box·sin(phase), which is what the catalog's
    start-clearance rule reads.

Pure numpy, no ROS, no Gazebo.
"""
import os
import sys

import numpy as np
import pytest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
sys.path.insert(0, os.path.join(REPO, 'franka_simulation_experiments', 'scripts'))
sys.path.insert(0, os.path.join(REPO, 'franka_simulation', 'scripts'))

from simexp import obstacle_laws as L  # noqa: E402

SPEC = dict(law='wander', p0=[0.52, 0.0, 0.45], box=[0.22, 0.40, 0.15],
            period=12.0, phase_deg_xyz=[0.0, -90.0, 0.0],
            ratios=list(L.WANDER_RATIOS))


def test_the_default_ratios_are_the_same_three_numbers_everywhere():
    """The plugin, the SDF writer and this mirror must agree, or sim ≠ truth."""
    from utils.gz_world import WANDER_RATIOS as GZ
    assert tuple(L.WANDER_RATIOS) == tuple(GZ)
    # The C++ default is one string literal, so compare the PARSED numbers:
    # the two sources print the same double with a different digit count.
    import re
    src = open(os.path.join(REPO, 'franka_simulation', 'src',
                            'scenario_obstacles_system.cpp')).read()
    # Anchored on the <ratios> element: "1 0 0" (the oscillate axis default)
    # appears earlier in the file and would otherwise match first.
    m = re.search(r'"ratios",\s*\n?\s*"([^"]+)"', src)
    assert m, 'the C++ wander default ratios string is gone'
    assert np.allclose([float(x) for x in m.group(1).split()],
                       L.WANDER_RATIOS, rtol=0, atol=1e-15)


def test_it_is_born_at_p0_plus_box_sin_phase():
    p = L.position(SPEC, 0.0)
    ph = np.deg2rad(SPEC['phase_deg_xyz'])
    assert np.allclose(p, np.array(SPEC['p0']) + np.array(SPEC['box']) * np.sin(ph))
    # this spec's phases put it at the far -y end, at the centre in x and z
    assert np.allclose(p, [0.52, -0.40, 0.45])


def test_the_tour_stays_inside_the_box():
    t = np.linspace(0.0, 10 * SPEC['period'], 200001)
    P = np.array([L.position(SPEC, float(x)) for x in t[::200]])
    lo = np.array(SPEC['p0']) - np.array(SPEC['box'])
    hi = np.array(SPEC['p0']) + np.array(SPEC['box'])
    assert np.all(P.min(axis=0) >= lo - 1e-12)
    assert np.all(P.max(axis=0) <= hi + 1e-12)
    # and it actually USES the box: every axis reaches within 1 % of both faces
    span = P.max(axis=0) - P.min(axis=0)
    assert np.all(span >= 1.98 * np.array(SPEC['box']))


def test_velocity_is_the_derivative_of_position():
    h = 1e-6
    for t in (0.0, 0.37, 2.5, 7.91, 23.4):
        fd = (L.position(SPEC, t + h) - L.position(SPEC, t - h)) / (2 * h)
        assert np.allclose(L.velocity(SPEC, t), fd, rtol=1e-6, atol=1e-8)


def test_the_peak_speed_per_axis_is_box_times_omega():
    w = 2.0 * np.pi / SPEC['period'] * np.array(SPEC['ratios'])
    expect = np.array(SPEC['box']) * w
    t = np.linspace(0.0, 50 * SPEC['period'], 500001)
    V = np.array([L.velocity(SPEC, float(x)) for x in t[::100]])
    assert np.allclose(np.abs(V).max(axis=0), expect, rtol=1e-3)
    # the shipped scenario's speed band, quoted in its comment
    assert 0.39 < np.linalg.norm(V, axis=1).max() < 0.42


def test_the_path_does_not_repeat_the_way_an_oscillate_does():
    """Irrational ratios ⇒ the state never returns to the birth point."""
    t = np.linspace(1.0, 20 * SPEC['period'], 20001)
    p0 = L.position(SPEC, 0.0)
    d = np.array([np.linalg.norm(L.position(SPEC, float(x)) - p0) for x in t])
    assert d.min() > 1e-3
    # a RATIONAL ratio set, by contrast, closes exactly after the common period
    rat = dict(SPEC, ratios=[1.0, 2.0, 4.0])
    assert np.allclose(L.position(rat, 0.0), L.position(rat, SPEC['period']))


def test_a_non_positive_period_is_refused():
    with pytest.raises(ValueError):
        L.position(dict(SPEC, period=0.0), 1.0)
    with pytest.raises(ValueError):
        L.velocity(dict(SPEC, period=-3.0), 1.0)


def test_a_wrong_shaped_box_is_refused():
    for bad in ({'box': [0.1, 0.2]}, {'ratios': [1.0]},
                {'phase_deg_xyz': [0.0, 0.0, 0.0, 0.0]}):
        with pytest.raises(ValueError):
            L.position(dict(SPEC, **bad), 1.0)


def test_the_sdf_writer_carries_every_wander_field():
    from utils.gz_world import scenario_obstacles_plugin
    xml = scenario_obstacles_plugin([dict(name='obstacle_0', radius=0.06, **SPEC)])
    assert '<law>wander</law>' in xml
    assert '<box>0.22 0.4 0.15</box>' in xml
    assert '<period>12</period>' in xml
    assert '<phase_deg_xyz>0 -90 0</phase_deg_xyz>' in xml
    assert '1.61803399' in xml      # the golden ratio reached the SDF


def test_the_catalog_resolves_the_shipped_scenario_and_keeps_its_fields():
    from simexp import scenario_catalog as SC
    cfg = SC.load()
    r = SC.resolve('pentagon_wander', cfg=cfg)
    o = r['obstacles'][0]
    assert r['scene'] == 'diag' and o['law'] == 'wander'
    assert o['box'] == [0.22, 0.40, 0.15] and o['period'] == 12.0
    assert o['phase_deg_xyz'] == [0.0, -90.0, 0.0]
    assert np.allclose(L.position(o, 0.0), [0.52, -0.40, 0.45])
    # jitter_m translates the WHOLE tour on a non-zero seed, and seed 0 is the
    # nominal, so a replay of seed 0 is the scenario as written.
    s3 = SC.resolve('pentagon_wander', seed=3, cfg=cfg)['obstacles'][0]
    assert s3['p0'] != o['p0']
    assert np.all(np.abs(np.array(s3['p0']) - np.array(o['p0'])) <= 0.04 + 1e-12)
    assert s3['box'] == o['box']
