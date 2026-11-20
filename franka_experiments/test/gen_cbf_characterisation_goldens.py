#!/usr/bin/env python3
"""Generate (or check) the CBF characterisation goldens in ``data/cbf_characterisation/``.

Not collected by pytest (no ``test_`` prefix). Run inside the ROS environment, against an install
that equals the source tree (``colcon build`` first; the harness refuses a stale install):

    source /ros2_ws/install/setup.bash
    GOLDEN_SOURCE_COMMIT=<sha> python3 -B test/gen_cbf_characterisation_goldens.py --write
    python3 -B test/gen_cbf_characterisation_goldens.py --check      # a second process: bit-exact?

Regenerate ONLY at a commit whose behaviour is the intended reference (the refactoring baseline),
never to make a failing characterisation test pass after a production change: a difference is the
signal the tests exist to give. The behaviour change, if intended, needs its own approval and record.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _cbf_characterisation as H  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--write', action='store_true', help='write the goldens')
    g.add_argument('--check', action='store_true', help='compare a fresh run with the goldens')
    ap.add_argument('scenarios', nargs='*', help='subset (default: all)')
    a = ap.parse_args()

    stale = H.installed_vs_source()
    if stale:
        sys.exit(f'installed franka_experiments differs from the source tree: {stale[:8]} '
                 f'... rebuild first')
    node, _ = H.make_node()
    try:
        kin = H.harness_kinematics(node)
    finally:
        H.close_node(node)
    rc = 0
    for sc in H.scenarios(kin):
        if a.scenarios and sc.name not in a.scenarios:
            continue
        num, meta = H.record(sc)
        meta['source_commit'] = os.environ.get('GOLDEN_SOURCE_COMMIT', 'unknown')
        if a.write:
            H.save(sc.name, num, meta)
            npz, _ = H.golden_paths(sc.name)
            print(f'{sc.name}: wrote {len(num)} arrays, {meta["n_ticks"]} ticks, '
                  f'{meta["n_rebuilds"]} rebuilds, {os.path.getsize(npz)} bytes')
        else:
            num_g, meta_g = H.load(sc.name)
            diffs = H.compare(num_g, meta_g, num, meta)
            print(f'{sc.name}: {"IDENTICAL" if not diffs else "DIFFERS"}')
            for d in diffs[:20]:
                print('   ', d)
            rc |= bool(diffs)
    return rc


if __name__ == '__main__':
    sys.exit(main())
