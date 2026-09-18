#!/usr/bin/env python3
"""Refuse to launch an ISO-enabled stack that cannot keep its own promises.

WHY THIS EXISTS
---------------
``iso_enabled: true`` is a claim: that the separation-distance bound in
``config/fr3_control.yaml`` was derived from measured constants and that the
layers enforcing it are running. Every one of those can be false while the
configuration still parses, and the failure mode is the worst kind — the stack
comes up, the logs say ISO, and the numbers are the placeholders somebody typed
as an example.

So this runs BEFORE the controller spawner and exits non-zero, aborting the
launch, when:

* a measured constant still holds its placeholder value;
* ``iso_c_intrusion`` was lowered below the ISO 13855:2024 body-detection value
  with no detection-capability measurement on record;
* ``d_safe < C + Z_d + Z_r``;
* ``link_speed_max > iso_v_pfl``, or the ordering invariant is broken;
* ``/NS_1/iso_safety`` is not being published within ``--wait`` seconds of
  startup, with ``iso_monitor_enabled`` on.

With ``iso_enabled: false`` it prints one line and exits 0: the ISO layer is not
in use and there is nothing to check.

USAGE
-----
    python3 scripts/iso_preflight_check.py                    # config only
    python3 scripts/iso_preflight_check.py --wait 5           # + the live topic
    python3 scripts/iso_preflight_check.py --detection-record iso_constants.json
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)

#: The placeholder values shipped in fr3_control.yaml. A parameter still sitting
#: on one of these has not been measured, whatever the comment next to it says.
#: Keep in step with the iso_* block and with scripts/iso_constants_measure.py.
PLACEHOLDERS = {
    'iso_t_reaction': 0.10,
    'iso_a_stop': 1.0,
    'iso_z_depth': 0.06,
    'iso_z_robot': 0.01,
}

#: ISO 13855:2024 body-detection intrusion distance. Below it, a detection
#: capability d <= 40 mm has to have been DEMONSTRATED — see
#: scripts/iso_constants_measure.py detection.
C_BODY_DETECTION = 0.85


class Check:
    def __init__(self):
        self.failures: list[str] = []
        self.notes: list[str] = []

    def fail(self, msg: str) -> None:
        self.failures.append(msg)

    def note(self, msg: str) -> None:
        self.notes.append(msg)

    def report(self) -> int:
        for n in self.notes:
            print(f'  [note] {n}')
        if not self.failures:
            print('\n== iso_preflight_check: PASS ==============================')
            print('  The ISO layer\'s own preconditions hold. This is NOT a')
            print('  statement of conformance, let alone certification — read')
            print('  franka_experiments/SAFETY.md for what is actually claimed.')
            return 0
        print('\n== iso_preflight_check: FAIL =============================')
        for i, f in enumerate(self.failures, 1):
            print(f'  {i}. {f}')
        print('\n  Launch ABORTED. Fix the above, or run with iso_enabled:=false')
        print('  and make no ISO claim.')
        return 1


def check_config(P: dict, chk: Check, detection_record: str) -> None:
    # ── 1. placeholders ─────────────────────────────────────────────────
    stale = [k for k, v in PLACEHOLDERS.items()
             if abs(float(P.get(k, v)) - v) < 1e-12]
    if stale:
        chk.fail(
            f'{len(stale)} measured constant(s) still at the shipped '
            f'placeholder: {", ".join(stale)}. Run '
            f'scripts/iso_constants_measure.py (reaction / stop / uncertainty) '
            f'and paste its `yaml` block. S_p is linear in T_r and quadratic in '
            f'1/a_s — a guessed a_s makes every stopping distance optimistic by '
            f'exactly that ratio.')

    # ── 2. C, and whether it was earned ─────────────────────────────────
    c = float(P['iso_c_intrusion'])
    if c < C_BODY_DETECTION:
        record = _detection_record(detection_record)
        if record is None:
            chk.fail(
                f'iso_c_intrusion = {c:.3f} m is below the ISO 13855:2024 '
                f'body-detection value ({C_BODY_DETECTION} m) with no detection '
                f'measurement on record. C is NOT a free parameter: it follows '
                f'from the detection capability d of the protective device. '
                f'Measure d with scripts/iso_constants_measure.py detection and '
                f'pass --detection-record <its json>, or put C back to '
                f'{C_BODY_DETECTION}.')
        elif not record.get('reliable'):
            chk.fail(
                f'iso_c_intrusion = {c:.3f} m rests on a detection test that '
                f'did NOT reach 100 % ({record.get("detections")} of '
                f'{record.get("samples")} samples). Anything below 100 % is a '
                f'FAIL for that object size.')
        else:
            d_mm = float(record['detection_capability_mm'])
            expect = (max(8.0 * (d_mm - 14.0), 0.0) * 1e-3 if d_mm <= 40.0
                      else C_BODY_DETECTION)
            if abs(expect - c) > 1e-3:
                chk.fail(
                    f'iso_c_intrusion = {c:.3f} m does not match the recorded '
                    f'detection capability d = {d_mm:.1f} mm, which gives '
                    f'C = {expect:.3f} m.')
            else:
                chk.note(
                    f'C = {c:.3f} m from a demonstrated d = {d_mm:.1f} mm on '
                    f'{record.get("surface", "?")} at '
                    f'{record.get("range_m", "?")} m ({record.get("measured")}). '
                    f'The depth pipeline is still not a rated protective device '
                    f'(no IEC/TS 61496-4-3 assessment) — record that deviation '
                    f'in SAFETY.md.')

    # ── 3. the d_safe floor ─────────────────────────────────────────────
    floor = c + float(P['iso_z_depth']) + float(P['iso_z_robot'])
    if float(P['d_safe']) < floor:
        chk.fail(
            f'd_safe = {P["d_safe"]:.3f} m < C + Z_d + Z_r = {floor:.3f} m, the '
            f'irreducible part of S_p (ISO 10218-2:2025 Annex L). '
            f'cbf_safety_filter would refuse to construct; this catches it one '
            f'process earlier.')
    else:
        chk.note(f'd_safe = {P["d_safe"]:.3f} m >= C+Z_d+Z_r = {floor:.3f} m')

    # ── 4. the PFL ceiling and the ordering invariant ───────────────────
    if float(P['link_speed_max']) > float(P['iso_v_pfl']):
        chk.fail(
            f'link_speed_max = {P["link_speed_max"]:.3f} m/s exceeds iso_v_pfl '
            f'= {P["iso_v_pfl"]:.3f} m/s. Relaunch with '
            f'link_speed_max:={P["iso_v_pfl"]:.2f} '
            f'retreat_cap_max_speed:={0.9 * float(P["iso_v_pfl"]):.2f}, or lower '
            f'both in fr3_control.yaml.')
    if float(P['retreat_cap_max_speed']) >= float(P['link_speed_max']):
        chk.fail(
            f'retreat_cap_max_speed = {P["retreat_cap_max_speed"]:.3f} must stay '
            f'strictly below link_speed_max = {P["link_speed_max"]:.3f}.')

    # ── 5. things that are not failures but must be said out loud ───────
    if str(P.get('iso_mode')) == 'reduced':
        chk.note('iso_mode = reduced: a 250 mm/s TCP row is added. [S] value, '
                 '[E] cell-wide application — not a rated speed-monitoring '
                 'function.')
    if not P.get('iso_monitor_enabled'):
        chk.note('iso_monitor_enabled is FALSE: the SSM bound is SHAPED by the '
                 'QP and ENFORCED by nothing. The speed rows are '
                 'slack-relaxable by design.')
    if not P.get('iso_stop_requires_reset'):
        chk.note('iso_stop_requires_reset is FALSE: motion resumes '
                 'automatically once S >= S_p. Permitted by SSM; record it.')


def _detection_record(path: str):
    if not path or not os.path.exists(path):
        return None
    try:
        with open(path) as fh:
            return json.load(fh).get('iso_c_intrusion')
    except Exception:
        return None


def check_topic(wait_s: float, topic: str, chk: Check) -> None:
    """Is the monitor actually on the wire? A monitor that is configured but not
    running is worse than one that is off: the configuration says it is there."""
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float64MultiArray
    except Exception as exc:
        chk.note(f'topic check skipped (no rclpy: {exc})')
        return

    rclpy.init()
    node = Node('iso_preflight_check')
    seen = {'n': 0}
    node.create_subscription(Float64MultiArray, topic,
                             lambda _m: seen.__setitem__('n', seen['n'] + 1), 10)
    t0 = node.get_clock().now().nanoseconds * 1e-9
    try:
        while seen['n'] == 0 and (
                node.get_clock().now().nanoseconds * 1e-9 - t0) < wait_s:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if seen['n'] == 0:
        chk.fail(
            f'{topic} is not being published after {wait_s:.0f} s, but '
            f'iso_monitor_enabled is true. The monitor publishes EVERY tick '
            f'whether or not anything is wrong, so silence means it is not '
            f'running — and cbf_safety_filter will brake on that within one '
            f'distance_timeout.')
    else:
        chk.note(f'{topic}: {seen["n"]} message(s) in {wait_s:.0f} s')


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--config', default=os.path.join(PKG, 'config', 'fr3_control.yaml'))
    ap.add_argument('--wait', type=float, default=0.0,
                    help='seconds to wait for /NS_1/iso_safety (0 = skip)')
    ap.add_argument('--topic', default='')
    ap.add_argument('--detection-record', default='',
                    help='JSON written by iso_constants_measure.py detection')
    args = ap.parse_args()

    with open(args.config) as fh:
        cfg = yaml.safe_load(fh)
    P = cfg['params']

    if not P.get('iso_enabled'):
        print('iso_preflight_check: iso_enabled is false — ISO layer not in '
              'use, nothing to check.')
        return 0

    print(f'== iso_preflight_check  ({args.config})')
    chk = Check()
    check_config(P, chk, args.detection_record)
    if args.wait > 0 and P.get('iso_monitor_enabled'):
        topic = args.topic or cfg.get('topics', {}).get(
            'iso_safety', '/NS_1/iso_safety')
        check_topic(args.wait, topic, chk)
    return chk.report()


if __name__ == '__main__':
    sys.exit(main())
