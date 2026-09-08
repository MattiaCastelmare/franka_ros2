"""Score every checkpoint of a training run and print one comparison table.

The question this answers is "is the policy actually getting better, and is it
getting better SAFELY" — which a single final number cannot show. It replays
each snapshot through the same rollout loop `evaluate_policy` uses, against the
same seeds, and puts the zero-action and random baselines in the same table.

Read the safety columns against the ZERO row, never in isolation: a share of
any collision rate is the obstacle sweeping into the arm, and the zero-action
row is the only measurement of how large that share is.

    cd /ros2_ws/src && PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl \
        python3 -m franka_sim.scripts.compare_checkpoints \
            --model-dir franka_sim/models/sac_v3 --episodes 5

    # watch the best one move
    python3 -m franka_sim.scripts.compare_checkpoints \
        --model-dir franka_sim/models/sac_v3 --render-best
"""

from __future__ import annotations

import argparse
import os

import numpy as np

from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
from franka_sim.scripts.evaluate_policy import (
    _load_policy, find_checkpoints, rollout)

_HERE = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_CONFIG = os.path.join(_HERE, '..', 'config.yaml')


def score(path: str, cfg: str, episodes: int, seed: int, render: bool = False):
    predict, _ = _load_policy(path)
    env = FrankaCBFEnv(config=cfg, render_mode='human' if render else None)
    r = rollout(predict, env, episodes, seed)
    env.close()
    return dict(
        success=100.0 * float(np.mean(r['successes'])),
        final=float(np.mean(r['finals'])),
        ret=float(np.mean(r['returns'])),
        steps=float(np.mean(r['lengths'])),
        coll=100.0 * float(np.mean(r['collisions'])),
        dmin=float(np.min(r['d_mins'])),
        dmean=float(np.mean(r['d_mins'])),
        interv=float(np.mean(r['intervs'])),
        slack=float(np.mean(r['slacks'])),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model-dir', required=True,
                    help='a franka_sim/models/<exp> directory')
    ap.add_argument('--config', default=None,
                    help='config.yaml (default: the one frozen in --model-dir)')
    ap.add_argument('--episodes', type=int, default=5,
                    help='episodes per checkpoint — keep small, this is a '
                         'progress check, not the 50-episode benchmark')
    ap.add_argument('--seed', type=int, default=12345)
    ap.add_argument('--baselines', action='store_true', default=True)
    ap.add_argument('--no-baselines', dest='baselines', action='store_false')
    ap.add_argument('--render-best', action='store_true',
                    help='after the table, replay the best checkpoint in the viewer')
    ap.add_argument('--last', type=int, default=0,
                    help='score only the last N checkpoints (0 = all)')
    args = ap.parse_args()

    frozen = os.path.join(args.model_dir, 'config.yaml')
    cfg = args.config or (frozen if os.path.isfile(frozen) else _DEFAULT_CONFIG)

    ckpts = find_checkpoints(args.model_dir)
    if args.last > 0:
        ckpts = ckpts[-args.last:]
    if not ckpts:
        raise SystemExit(f'no checkpoints found under {args.model_dir}')
    if args.baselines:
        ckpts = [('zero', 'zero'), ('random', 'random')] + ckpts

    print(f'config={cfg}  episodes={args.episodes}  seed={args.seed}\n')
    hdr = (f'{"checkpoint":>12} {"succ%":>7} {"final m":>9} {"return":>9} '
           f'{"steps":>7} {"coll%":>7} {"min d":>8} {"mean d":>8} '
           f'{"interv":>8} {"slack":>8}')
    print(hdr)
    print('-' * len(hdr))

    rows = []
    for label, path in ckpts:
        try:
            s = score(path, cfg, args.episodes, args.seed)
        except Exception as exc:                      # noqa: BLE001
            print(f'{label:>12}  SKIPPED: {exc}')
            continue
        rows.append((label, path, s))
        print(f'{label:>12} {s["success"]:7.1f} {s["final"]:9.4f} {s["ret"]:9.1f} '
              f'{s["steps"]:7.1f} {s["coll"]:7.1f} {s["dmin"]:8.4f} '
              f'{s["dmean"]:8.4f} {s["interv"]:8.3f} {s["slack"]:8.5f}')

    trained = [r for r in rows if r[0] not in ('zero', 'random')]
    if not trained:
        return
    # Rank on success first, then on final error — a policy that reaches more
    # often wins even if its average miss is slightly worse.
    best = max(trained, key=lambda r: (r[2]['success'], -r[2]['final']))
    zero = next((r for r in rows if r[0] == 'zero'), None)
    print(f'\nbest: {best[0]}  ({best[1]})')
    if zero is not None:
        print(f'  vs zero-action baseline: success {best[2]["success"]:.1f}% vs '
              f'{zero[2]["success"]:.1f}%   collisions {best[2]["coll"]:.1f}% vs '
              f'{zero[2]["coll"]:.1f}%')
        if best[2]['coll'] > zero[2]['coll']:
            print('  NOTE: this policy collides MORE than a motionless arm — '
                  'it is driving into the obstacle, not merely being swept.')
    if args.render_best:
        print(f'\nreplaying {best[0]} in the viewer …')
        score(best[1], cfg, args.episodes, args.seed, render=True)


if __name__ == '__main__':
    main()
