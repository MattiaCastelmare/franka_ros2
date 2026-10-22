"""external_plant / motion_source:=none ship OFF, and with them off nothing moved.

Two launch options exist for the Gazebo replica of these experiments
(franka_simulation_experiments, docs/gz_cbf_experiments_plan.md §5.2):

* ``external_plant:=true`` drops the franka_bringup include, because Gazebo
  provides the robot;
* ``motion_source:=none`` starts no commander, because the scenario commander
  owns /NS_1/qddot_nom.

The promise that comes with a default-off switch is that the default launch is
the one that existed before the switch did. This test makes that promise
mechanical. It expands ``_launch_all`` without launching anything
(``_launch_snapshot.py``) and compares the result with a GOLDEN snapshot of the
file at commit 94cd7b0, the last commit before either option existed, for
three variants: defaults, ``use_fake_hardware:=true`` and ``motion_source:=rl``.
Then it pins what each option does when on: one action replaced by one LogInfo.

When this fails after a DELIBERATE change to torque_control_stack.launch.py,
regenerate the golden from the new file with the snippet in
``data/README_snapshot.md`` and review the diff; a failure you did not expect is
the default path having changed.
"""
import copy
import json
import os

import pytest

from _launch_snapshot import snapshot

HERE = os.path.dirname(os.path.realpath(__file__))
LAUNCH = os.path.join(HERE, '..', 'launch', 'torque_control_stack.launch.py')
GOLDEN = os.path.join(HERE, 'data', 'torque_control_stack_94cd7b0_snapshot.json')


@pytest.fixture(scope='module')
def golden():
    with open(GOLDEN) as f:
        return json.load(f)


@pytest.mark.parametrize('variant', ['defaults', 'fake_hardware', 'rl'])
def test_the_default_launch_is_the_one_at_94cd7b0(golden, variant):
    over = golden['variants'][variant]
    assert snapshot(LAUNCH, **over) == golden['snapshots'][variant]


@pytest.mark.parametrize('variant', ['defaults', 'fake_hardware', 'rl'])
def test_external_plant_false_explicitly_is_the_default(golden, variant):
    over = dict(golden['variants'][variant], external_plant='false')
    assert snapshot(LAUNCH, **over) == golden['snapshots'][variant]


def _is_bringup_include(a):
    return (a.get('__type__') == 'IncludeLaunchDescription'
            and 'franka_bringup' in json.dumps(a) and 'franka.launch.py' in json.dumps(a))


def test_external_plant_drops_exactly_the_bringup_include(golden):
    ref = golden['snapshots']['defaults']
    got = snapshot(LAUNCH, external_plant='true')
    assert len(got) == len(ref)
    idx = [i for i, a in enumerate(ref) if _is_bringup_include(a)]
    assert len(idx) == 1, 'the golden must hold exactly one franka bringup include'
    i = idx[0]
    assert got[i]['__type__'] == 'LogInfo' and 'external_plant' in json.dumps(got[i])
    assert not any(_is_bringup_include(a) for a in got)
    # Everything else, the rt_torque_controller spawner included, is unchanged.
    assert got[:i] + got[i + 1:] == ref[:i] + ref[i + 1:]
    assert any('rt_torque_controller' in json.dumps(a) and 'spawner' in json.dumps(a)
               for a in got)


def _is_commander(a):
    s = json.dumps(a)
    return a.get('__type__') == 'TimerAction' and 'pentagon_qddot_commander' in s


def test_motion_source_none_drops_exactly_the_commander(golden):
    ref = golden['snapshots']['defaults']
    got = snapshot(LAUNCH, motion_source='none')
    idx = [i for i, a in enumerate(ref) if _is_commander(a)]
    assert len(idx) == 1
    i = idx[0]
    # Commander timer + its LogInfo  ->  one LogInfo.
    assert len(got) == len(ref) - 1
    assert got[i]['__type__'] == 'LogInfo' and 'motion_source:=none' in json.dumps(got[i])
    assert got[:i] == ref[:i]
    assert got[i + 1:] == ref[i + 2:]
    assert not any('rl_policy_commander' in json.dumps(a) or 'pentagon_qddot_commander'
                   in json.dumps(a) for a in got)


def test_an_unknown_motion_source_still_refuses_to_start():
    with pytest.raises(RuntimeError, match='motion_source'):
        snapshot(LAUNCH, motion_source='scenario')


def test_neither_option_is_a_launch_default():
    """Both must be passed on the command line; launch_defaults.yaml cannot flip them."""
    import yaml
    with open(os.path.join(HERE, '..', 'config', 'launch_defaults.yaml')) as f:
        ld = yaml.safe_load(f)
    assert 'external_plant' not in ld
    assert ld.get('motion_source') == 'pentagon'


def test_the_golden_is_not_degenerate(golden):
    """A serialiser that dropped the fields would make every comparison pass."""
    s = json.dumps(golden['snapshots']['defaults'])
    for needle in ('cbf_safety_filter', 'qddot_to_torque', 'real_time_distance',
                   'obstacle_velocity_source', 'enable_vobs_in_hdot', 'rt_torque_controller',
                   'franka.launch.py', 'camera_extrinsics.yaml', 'OPENBLAS_NUM_THREADS'):
        assert needle in s, needle
    # The copies of the snapshot are independent of the fixture (no aliasing).
    assert copy.deepcopy(golden) == golden
