#!/usr/bin/env python3
"""Compute ``iso_v_pfl`` — the power-and-force-limited speed ceiling.

WHAT THIS IS
------------
The absolute ceiling that survives every software layer, including a total
perception failure. Below the SSM cap, below the barrier, below everything: if
the depth camera is unplugged and the CBF has nothing to build a row from, this
is still the speed at which a contact stays inside the biomechanical limit of
the body region it hits.

    mu    = 1 / (1/m_H + 1/m_R)          reduced mass  [kg]
    v_PFL = F_max / sqrt(mu * k)         [m/s]

**[S]** formula and the ``m_R = M/2 + payload`` simplification: ISO
10218-2:2025 **Annex M** (informative; formerly ISO/TS 15066 Annex A).
**[E]** the choice of which region and which payload to bind the cell to.

WHAT THIS IS NOT
----------------
**[R]** Claiming PFL requires force and pressure MEASUREMENT with a
pressure/force measurement device (PFMD) per ISO 10218-2:2025 clause 6.3.3 and
Annex N. This calculation supports a preliminary risk assessment and NOTHING
beyond it. No number it prints may be described as a verified PFL limit.

READ THE TABLE FROM THE STANDARD
--------------------------------
``--region`` looks the row up in a small table of the most frequently cited
Annex M values, provided so the script runs out of the box and so its worked
example is reproducible. THOSE VALUES ARE A CONVENIENCE, NOT A SOURCE. For any
real assessment pass ``--f-max``, ``--k`` and ``--m-human`` read off the edition
of the standard your cell is assessed against: the tables were renumbered and
partly revised when ISO/TS 15066 was absorbed into ISO 10218-2:2025.

USAGE
-----
    python3 scripts/iso_pfl_speed.py                      # the worked example
    python3 scripts/iso_pfl_speed.py --payload 3.0 --gripper 0.7
    python3 scripts/iso_pfl_speed.py --region upper-arm --transient
    python3 scripts/iso_pfl_speed.py --f-max 140 --k 75 --m-human 0.6
"""
from __future__ import annotations

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, PKG)

from franka_experiments.utils.iso_ssm import pfl_speed  # noqa: E402

#: Manipulator mass of the Franka Research 3, from its datasheet [kg]. The
#: annex's effective robot mass for a contact is M/2 plus whatever is mounted.
FR3_MASS_KG = 17.8

#: A CONVENIENCE COPY of the most frequently cited Annex M rows, so this script
#: runs without the standard open and so the worked example is reproducible:
#:     region: (F_quasi_static [N], k [N/mm], m_human [kg])
#: The transient limit for a region is roughly TWICE the quasi-static one; that
#: factor is applied by --transient rather than tabulated, because it is an
#: approximation and tabulating it would make it look like a value.
#: DO NOT CITE THIS DICT. Read the row from the edition your cell is assessed
#: against and pass --f-max/--k/--m-human.
ANNEX_M_ROWS = {
    'hands-and-fingers': (140.0, 75.0, 0.6),
    'forearm':           (160.0, 40.0, 4.0),
    'upper-arm':         (150.0, 30.0, 4.0),
    'chest':             (140.0, 25.0, 40.0),
    'abdomen':           (110.0, 10.0, 40.0),
    'thigh':             (220.0, 50.0, 75.0),
}


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--region', default='hands-and-fingers',
                    choices=sorted(ANNEX_M_ROWS),
                    help='Annex M body region (convenience table — see the docstring)')
    ap.add_argument('--f-max', type=float, default=None, help='[N] override F_max')
    ap.add_argument('--k', type=float, default=None, help='[N/mm] override k')
    ap.add_argument('--m-human', type=float, default=None,
                    help='[kg] override the effective mass of the region')
    ap.add_argument('--transient', action='store_true',
                    help='use the transient limit (~2x quasi-static) instead')
    ap.add_argument('--robot-mass', type=float, default=FR3_MASS_KG,
                    help='[kg] manipulator mass M (FR3 datasheet)')
    ap.add_argument('--payload', type=float, default=0.0, help='[kg] payload')
    ap.add_argument('--gripper', type=float, default=0.0, help='[kg] mounted gripper')
    args = ap.parse_args()

    f_qs, k_nmm, m_h = ANNEX_M_ROWS[args.region]
    if args.f_max is not None:
        f_qs = args.f_max
    if args.k is not None:
        k_nmm = args.k
    if args.m_human is not None:
        m_h = args.m_human
    f_max = f_qs * (2.0 if args.transient else 1.0)

    m_r = args.robot_mass / 2.0 + args.payload + args.gripper
    mu = 1.0 / (1.0 / m_h + 1.0 / m_r)
    v = pfl_speed(f_max, k_nmm * 1000.0, m_r, m_h)

    print('== iso_pfl_speed =================================================')
    print(f'  region                  : {args.region}'
          f'{"  (TRANSIENT)" if args.transient else "  (quasi-static)"}')
    print(f'  F_max                   : {f_max:8.1f} N'
          + ('   [S] Annex M' if args.f_max is None else '   [operator]'))
    print(f'  k                       : {k_nmm:8.1f} N/mm'
          + ('  [S] Annex M' if args.k is None else '  [operator]'))
    print(f'  m_human (effective)     : {m_h:8.2f} kg'
          + ('  [S] Annex M' if args.m_human is None else '  [operator]'))
    print(f'  M (manipulator)         : {args.robot_mass:8.2f} kg   [datasheet]')
    print(f'  payload + gripper       : {args.payload + args.gripper:8.2f} kg')
    print(f'  m_robot = M/2 + mounted : {m_r:8.2f} kg   [S] Annex M simplification')
    print(f'  mu = reduced mass       : {mu:8.4f} kg')
    print(f'  ----------------------------------------------------------')
    print(f'  iso_v_pfl               : {v:8.3f} m/s   [E] from [S] inputs')
    print()
    print('  WARNINGS — all three apply to the number above:')
    print()
    print('  1. QUASI-STATIC vs TRANSIENT. The transient limit for a region is')
    print('     roughly twice the quasi-static one. The quasi-static value is')
    print('     the default here because a CLAMPING event cannot be excluded in')
    print('     this cell — a benchtop arm with a table under it and no')
    print('     dedicated clamping-hazard analysis. [E] Using --transient is a')
    print('     claim that clamping is impossible, and that claim needs a risk')
    print('     assessment behind it.')
    print()
    print('  2. MASS. A mounted gripper and a payload both raise m_R and lower')
    print('     v_PFL. Recompute PER CONFIGURATION — the ceiling is not a')
    print('     property of the robot, it is a property of the robot as it is')
    print('     currently built.')
    if args.payload + args.gripper == 0.0:
        print('     (computed here with NO payload and NO gripper)')
    print()
    print('  3. [R] THIS IS NOT PFL COMPLIANCE. Claiming PFL requires force and')
    print('     pressure MEASUREMENT with a PFMD per ISO 10218-2:2025 clause')
    print('     6.3.3 and Annex N. This calculation supports a preliminary risk')
    print('     assessment only. It is also PER REGION: the binding ceiling is')
    print('     the smallest v_PFL over every region a contact can reach, which')
    print('     one invocation of this script cannot tell you.')
    print()
    print(f'  To use it:  set  iso_v_pfl: {v:.2f}  in config/fr3_control.yaml, and')
    print(f'  keep  link_speed_max <= {v:.2f}  with  retreat_cap_max_speed')
    print(f'  strictly below that. cbf_safety_filter RAISES otherwise when')
    print(f'  iso_enabled is true — it does not clamp silently.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
