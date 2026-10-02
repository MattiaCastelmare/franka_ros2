#!/usr/bin/env python3
"""Render a real-hardware rt_torque_controller controllers.yaml with
p_gains/d_gains explicitly zeroed — the hardware equivalent of the sim's
``actuation.enabled: false``.

WHY THIS SCRIPT EXISTS
-----------------------
``torque_control_stack.launch.py`` auto-generates its controllers.yaml via
``utils.launch_support.generate_rt_controllers_yaml`` and does not expose
``p_gains``/``d_gains`` as launch arguments at all — on real hardware
(``is_real=True``) they are always the compiled-in non-zero defaults
(p_gains={120,120,120,100,40,40,20}, d_gains={30,30,30,25,10,10,5}). The
only way to run with zero gains on real hardware is a hand-supplied
``controllers_yaml``, and ``pick_controllers_yaml`` treats a non-``__auto__``
value as a COMPLETE replacement — every key the controller manager expects,
not just p_gains/d_gains.

This script calls the exact same generator function the launch file calls,
with the SAME parameters (arm_id, torque_command_topic, gazebo, lpf_alpha,
tau_max_scale — read from config/launch_defaults.yaml, the single source of
truth) and only d_gains/p_gains overridden to zero, so the output is
byte-identical to what a default real-hardware launch would auto-generate
except for those two lines. No hand-typed YAML, no schema-guessing.

USAGE
-----
    python3 scripts/gen_zero_gain_controllers_yaml.py
        -> /tmp/controllers_torque_zero_gain.yaml

    ros2 launch franka_experiments torque_control_stack.launch.py \\
        isolation_test:=true \\
        controllers_yaml:=/tmp/controllers_torque_zero_gain.yaml

Needs a sourced ROS 2 environment (imports franka_experiments.utils).
"""
from __future__ import annotations

import argparse
import os

from franka_experiments.utils.config import load_launch_defaults
from franka_experiments.utils.launch_support import generate_rt_controllers_yaml


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default='/tmp/controllers_torque_zero_gain.yaml')
    ap.add_argument('--fake-hardware', action='store_true',
                     help='Generate for use_fake_hardware:=true instead of real '
                          'hardware (is_real=False). Mostly for testing this '
                          'script itself — the real default already zeroes gains '
                          'on fake hardware, so this flag has no purpose in the '
                          'actual A/B test.')
    args = ap.parse_args()

    d, _ = load_launch_defaults()
    content = generate_rt_controllers_yaml(
        is_real=not args.fake_hardware,
        arm_id=d['arm_id'],
        controller_type='torque',
        torque_command_topic=d['torque_command_topic'],
        gazebo=d['gazebo'],
        lpf_alpha=float(d['lpf_alpha']),
        tau_max_scale=float(d['tau_max_scale']),
        d_gains=[0.0] * 7,
        p_gains=[0.0] * 7,
    )
    with open(args.out, 'w') as fh:
        fh.write(content)
    print(f'wrote {args.out}')
    print('d_gains / p_gains lines:')
    for line in content.splitlines():
        if 'd_gains' in line or 'p_gains' in line:
            print(f'  {line.strip()}')


if __name__ == '__main__':
    main()
