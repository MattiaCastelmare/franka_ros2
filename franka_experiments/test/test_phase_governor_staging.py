"""The phase governor must ease off BEFORE the barrier starts pushing.

WHY THIS FILE EXISTS
--------------------
The governor exists so the reference does not run away from an arm the CBF is
holding back: `pentagon_qddot_commander` scales the phase by sigma(d), 1 above
`governor_r_full * d_safe` and `governor_sigma_min` below
`governor_r_stop * d_safe`.

Measured 2026-09-21 on a static-obstacle run: at the shipped 0.25 / 0.10 those
thresholds were 3.75 cm and 1.5 cm while the barrier binds at d_safe = 15 cm,
and d_min never went below 13 cm — so sigma was 1.000 for 100% of the run, the
governor was inert, the end effector sat 13.3 cm behind its reference, and
34-43% of the commanded acceleration's energy was above 5 Hz. The two stages
were a factor of four apart, which is the bug this file pins shut.
"""

import os

import pytest
import yaml

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _params():
    with open(os.path.join(HERE, 'config', 'fr3_control.yaml')) as fh:
        return yaml.safe_load(fh)['params']


def test_the_governor_engages_before_the_barrier_binds():
    """r_full * d_safe must be OUTSIDE d_safe, or the staging is inverted.

    The barrier's rows start pushing at d_safe. A governor that only reacts
    inside that distance is reacting after the fact: the reference has already
    spent the whole engagement running away.
    """
    p = _params()
    d_safe = float(p['d_safe'])
    d_full = float(p['governor_r_full']) * d_safe
    assert d_full > d_safe, (
        f'the phase only starts easing at {d_full * 100:.1f} cm, inside the '
        f'{d_safe * 100:.1f} cm at which the barrier already binds')


def test_the_governor_floor_is_not_reached_in_normal_operation():
    """r_stop must sit below d_safe, so the floor is a fault case.

    sigma_min is a creep, and creeping is for a path that is genuinely blocked.
    An obstacle the arm is meant to work next to must slow the path, not stall
    it — the closest approach measured on the reference run was 13 cm.
    """
    p = _params()
    d_safe = float(p['d_safe'])
    d_stop = float(p['governor_r_stop']) * d_safe
    assert d_stop < d_safe
    assert d_stop <= 0.13, (
        f'the floor at {d_stop * 100:.1f} cm would be reached by the 13 cm '
        f'closest approach of the reference run')


def test_the_ramp_is_ordered_and_finite():
    p = _params()
    assert 0.0 < float(p['governor_r_stop']) < float(p['governor_r_full'])


def test_the_commander_reads_the_radii_from_the_config():
    """They are tunables, so they must not be hard-coded in the node.

    The node keeps 0.25 / 0.10 as its fallback for an older config; what it
    must not do is ignore the file.
    """
    src = open(os.path.join(HERE, 'franka_experiments', 'nodes',
                            'pentagon_qddot_commander.py')).read()
    for key in ('governor_r_full', 'governor_r_stop'):
        assert f"_params.get('{key}'" in src, (
            f'{key} is not read from fr3_control.yaml')


def test_the_joint_ceiling_is_the_published_braking_limit():
    """Joint 2's q̈ ceiling is the robot's deceleration limit, and stays there.

    `pentagon_qddot_commander` clips its nominal at column 4 of joint_limits,
    which is the only q̈-scale number the robot publishes. Joint 2's 2.585 is
    the tightest of the seven and the commander asked past it in 9.3% of the
    samples of the reference run — which is an argument for demanding LESS (the
    governor above), not for raising a braking limit the arm then cannot meet.
    That failure has already happened once on this rig: see qddot_max_abs.
    """
    with open(os.path.join(HERE, 'config', 'fr3_control.yaml')) as fh:
        lim = yaml.safe_load(fh)['joint_limits']
    ceilings = [float(lim[f'joint{j}'][3]) for j in range(1, 8)]
    assert ceilings[1] == pytest.approx(2.585)
    assert min(ceilings) == pytest.approx(2.585), 'joint 2 is the binding one'
    # And nothing may exceed what libfranka admits per joint (10 rad/s^2),
    # once qddot_max_abs has capped it.
    assert float(_params()['qddot_max_abs']) <= 10.0
