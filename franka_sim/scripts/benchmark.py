"""The benchmark table: one policy against both reference baselines.

Backlog item P0.3. `evaluate_policy` scores ONE controller, which is how an
absolute collision rate ends up in a paper; this scores the policy, the
zero-action arm and a random policy in a single run, against the same seeds and
the same config, and prints the DELTA against zero next to every safety number.

Why the delta, and why it is not optional
-----------------------------------------
A barrier can only bound the ROBOT's motion. Whatever share of a collision rate
comes from the obstacle moving into an arm that cannot clear it is measured by
the zero-action row and by nothing else, so the zero row is the only thing that
makes the policy's row readable. This has bitten the project twice:

* the actuation defect, where a trained, a random and a motionless policy
  scored identically because the action never reached the plant;
* the reset defect, where all three penetrated to an identical -0.1467 m
  because 9 of 50 episodes started inside the obstacle.

Both were invisible in a single-controller run and obvious the moment the
baselines sat in the same table.

The header records the obstacle regime the numbers came from. A collision rate
means nothing without it — the same policy scored 20% and 0% under two regimes
of this very benchmark.

    cd /ros2_ws/src && PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl \
        python3 -m franka_sim.scripts.benchmark \
            --model franka_sim/models/<exp>/best_model.onnx --episodes 50

    # paper-ready table as well
    python3 -m franka_sim.scripts.benchmark --latest franka_sim/models/<exp> \
        --markdown results.md
"""

from __future__ import annotations

import argparse
import os

import yaml

from franka_sim.scripts.compare_checkpoints import score
from franka_sim.scripts.evaluate_policy import latest_checkpoint

_HERE = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_CONFIG = os.path.join(_HERE, '..', 'config.yaml')

#: (key, heading, format, is_delta_worthy). Only the safety rows get a delta:
#: "success rate vs a motionless arm" is not a comparison anyone is tempted to
#: misread, whereas "collision rate" absolutely is.
_ROWS = [
    ('success', 'success rate',       '{:.1f} %',  False),
    ('final',   'final EE error',     '{:.4f} m',  False),
    ('ret',     'episode return',     '{:.1f}',    False),
    ('steps',   'episode length',     '{:.1f}',    False),
    ('coll',    'collision rate',     '{:.1f} %',  True),
    ('dmin',    'min surface dist',   '{:+.4f} m', True),
    ('dmean',   'mean surface dist',  '{:.4f} m',  True),
    ('interv',  'mean intervention',  '{:.3f}',    False),
    ('slack',   'mean slack',         '{:.5f}',    False),
]


def _regime(cfg_path: str) -> dict:
    """The knobs a reported number is only meaningful relative to."""
    with open(cfg_path) as fh:
        cfg = yaml.safe_load(fh)
    obs, cbf, task = cfg['obstacle'], cfg['cbf'], cfg.get('task', {})
    speed, amp = float(obs['speed']), float(obs['amplitude'])
    rc = task.get('reset_min_clearance')
    return {
        'obstacle': f'{speed} Hz x {amp} m  (peak {2 * 3.141592653589793 * speed * amp:.3f} m/s)',
        'obstacle_radius': f'{obs["radius"]} m',
        'd_safe': f'{cbf["d_safe"]} m',
        'reset_min_clearance': (f'{rc} m' if rc is not None
                                else f'{cbf["d_safe"]} m (= d_safe)'),
        'qddot_max_abs': str(cbf.get('qddot_max_abs', '<uncapped>')),
    }


def run(model: str, cfg: str, episodes: int, seed: int) -> dict:
    return {
        'policy': score(model, cfg, episodes, seed),
        'zero':   score('zero', cfg, episodes, seed),
        'random': score('random', cfg, episodes, seed),
    }


def _delta(key, pol, zero):
    d = pol[key] - zero[key]
    if key == 'coll':
        return f'{d:+.1f} pp'
    if key in ('dmin', 'dmean'):
        return f'{d:+.4f} m'
    return f'{d:+.3f}'


def print_table(res: dict, regime: dict, episodes: int, seed: int, model: str):
    pol, zero, rnd = res['policy'], res['zero'], res['random']
    print(f'\npolicy   : {model}')
    print(f'episodes : {episodes} per controller, seed {seed}')
    for k, v in regime.items():
        print(f'{k:9}: {v}')
    fb = pol['fallbacks'] + zero['fallbacks'] + rnd['fallbacks']
    if fb:
        print(f'\n  WARNING: {fb} episode resets fell back to a non-clearing '
              'start (rejection sampling exhausted). The safety rows below are '
              'NOT all measured from inside the safe set.')

    w = max(len(h) for _, h, _, _ in _ROWS)
    print(f'\n{"":{w}}  {"policy":>12} {"zero":>12} {"random":>12} {"Δ vs zero":>12}')
    print('-' * (w + 54))
    for key, head, fmt, delta in _ROWS:
        line = (f'{head:{w}}  {fmt.format(pol[key]):>12} '
                f'{fmt.format(zero[key]):>12} {fmt.format(rnd[key]):>12} ')
        line += f'{_delta(key, pol, zero):>12}' if delta else f'{"":>12}'
        print(line)

    print()
    if pol['coll'] > zero['coll']:
        print('  READ THIS: the policy collides MORE than a motionless arm — it '
              'is driving into the obstacle, not merely being swept into.')
    elif pol['coll'] == zero['coll'] and zero['coll'] > 0:
        print('  READ THIS: the policy and a motionless arm collide at the SAME '
              'rate. That share is the obstacle moving into the robot and no '
              'controller can remove it — quote the delta, never the absolute.')
    if pol['success'] <= max(zero['success'], rnd['success']):
        print('  READ THIS: the policy does not beat its baselines on the task. '
              'Suspect the plant before the policy (scripts/validate_actuation).')


def markdown(res: dict, regime: dict, episodes: int, seed: int, model: str) -> str:
    pol, zero, rnd = res['policy'], res['zero'], res['random']
    out = [f'### Policy vs baselines ({episodes} episodes each, seed {seed})', '']
    out.append(f'`{os.path.basename(model)}` — obstacle {regime["obstacle"]}, '
               f'`d_safe` {regime["d_safe"]}, reset clearance '
               f'{regime["reset_min_clearance"]}.')
    out += ['', '| | trained policy | zero-action | random | Δ vs zero |',
            '|---|---|---|---|---|']
    for key, head, fmt, delta in _ROWS:
        cells = [fmt.format(pol[key]), fmt.format(zero[key]), fmt.format(rnd[key])]
        cells.append(_delta(key, pol, zero) if delta else '—')
        best = f'**{cells[0]}**'
        out.append(f'| {head} | {best} | {cells[1]} | {cells[2]} | {cells[3]} |')
    out += ['', '_Safety rows must be read against the zero-action column: a '
            'barrier bounds the robot\'s motion, not the obstacle\'s, so the '
            'delta is the part attributable to the controller._']
    return '\n'.join(out) + '\n'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default=None, help='.onnx (preferred) or SB3 .zip')
    ap.add_argument('--latest', metavar='MODEL_DIR', default=None,
                    help='benchmark the newest snapshot in this run directory')
    ap.add_argument('--config', default=None,
                    help='config.yaml (default: the one frozen next to the model)')
    ap.add_argument('--episodes', type=int, default=50)
    ap.add_argument('--seed', type=int, default=12345)
    ap.add_argument('--markdown', metavar='PATH', nargs='?', const='-',
                    help='also emit a paste-ready Markdown table (to PATH, or '
                         'stdout when given no value)')
    args = ap.parse_args()

    if not args.model and not args.latest:
        ap.error('pass --model, or --latest <model-dir>')
    if args.latest:
        _, args.model = latest_checkpoint(args.latest)
        if args.config is None:
            frozen = os.path.join(args.latest, 'config.yaml')
            if os.path.isfile(frozen):
                args.config = frozen

    cfg = args.config
    if cfg is None:
        frozen = os.path.join(os.path.dirname(os.path.abspath(args.model)),
                              'config.yaml')
        cfg = frozen if os.path.isfile(frozen) else _DEFAULT_CONFIG

    regime = _regime(cfg)
    res = run(args.model, cfg, args.episodes, args.seed)
    print_table(res, regime, args.episodes, args.seed, args.model)

    if args.markdown:
        md = markdown(res, regime, args.episodes, args.seed, args.model)
        if args.markdown == '-':
            print('\n' + md)
        else:
            with open(args.markdown, 'w') as fh:
                fh.write(md)
            print(f'markdown table -> {args.markdown}')


if __name__ == '__main__':
    main()
