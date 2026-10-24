"""Watch a policy over N episodes of INCREASING obstacle difficulty.

`evaluate_policy` scores one fixed regime; this sweeps the regime upward across
episodes so you can see where the policy starts to struggle, in one window.

Difficulty is the obstacle's speed and amplitude, ramped linearly from `--from`
to `--to` as a multiple of whatever the config carries. Those two are the knobs
`config.yaml` itself identifies as the difficulty dial (peak speed is
2*pi*f*A), and they are the pair that was lowered when the benchmark was made
measurable — so raising them walks back toward the harder regime rather than
inventing a new one.

The SHIELD is never touched. `d_safe`, the gains and the reset clearance stay
exactly as trained: this changes the world, not the safety filter, so every
episode is still certified by the same barrier the policy met in training.

    cd /ros2_ws/src && PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl \
        python3 -m franka_sim.scripts.watch_ramp \
            --latest franka_sim/models/sac_v4 --episodes 20

    # no window, just the table (works over ssh)
    python3 -m franka_sim.scripts.watch_ramp --latest franka_sim/models/sac_v4 \
        --episodes 20 --no-render

One env instance for the whole run, deliberately: opening a second MuJoCo
viewer in one interpreter segfaults, which is why `compare_checkpoints` renders
only at the end. Here the env is built once and its obstacle parameters are
mutated between episodes — they are read per step, so the ramp takes effect
without a rebuild.
"""

from __future__ import annotations

import argparse
import math
import os

import numpy as np

from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
from franka_sim.scripts.evaluate_policy import _load_policy, latest_checkpoint

_HERE = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_CONFIG = os.path.join(_HERE, '..', 'config.yaml')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default=None, help='.onnx, .zip, "zero" or "random"')
    ap.add_argument('--latest', metavar='MODEL_DIR', default=None,
                    help='use the newest snapshot in this run directory')
    ap.add_argument('--config', default=None)
    ap.add_argument('--episodes', type=int, default=20)
    ap.add_argument('--seed', type=int, default=12345)
    ap.add_argument('--from', dest='lo', type=float, default=1.0,
                    help='difficulty multiplier for episode 1 (default 1.0 = the '
                         'trained regime)')
    ap.add_argument('--to', dest='hi', type=float, default=3.0,
                    help='difficulty multiplier for the last episode (default 3.0; '
                         '3.0x of 0.20 Hz x 0.20 m is ~2.26 m/s peak, well past '
                         'the old 1.13 m/s regime where avoidance was impossible)')
    ap.add_argument('--blocking', type=float, default=None, metavar='FRAC',
                    help='force this fraction of episodes to put the obstacle ON '
                         'the EE->target path (1.0 = always). Without it the '
                         'config value is used, and under uniform sampling HALF '
                         'the episodes need no avoidance at all')
    ap.add_argument('--no-render', action='store_true')
    args = ap.parse_args()

    if not args.model and not args.latest:
        ap.error('pass --model, or --latest <model-dir>')
    label = args.model
    if args.latest:
        label, args.model = latest_checkpoint(args.latest)
        if args.config is None:
            frozen = os.path.join(args.latest, 'config.yaml')
            if os.path.isfile(frozen):
                args.config = frozen

    cfg = args.config
    if cfg is None:
        frozen = os.path.join(os.path.dirname(os.path.abspath(args.model)),
                              'config.yaml')
        cfg = frozen if os.path.isfile(frozen) else _DEFAULT_CONFIG

    predict, kind = _load_policy(args.model)
    env = FrankaCBFEnv(config=cfg,
                       render_mode=None if args.no_render else 'human')
    if args.blocking is not None:
        env.blocking_fraction = float(args.blocking)
    base_speed, base_amp = env.obs_speed, env.obs_amp

    print(f'policy   : {args.model} ({kind})')
    print(f'config   : {cfg}')
    print(f'ramp     : {args.lo:.2f}x -> {args.hi:.2f}x over {args.episodes} episodes')
    print(f'base     : {base_speed} Hz x {base_amp} m '
          f'(peak {2 * math.pi * base_speed * base_amp:.3f} m/s)')
    how = 'forzato' if env.blocking_fraction > 0 else 'solo ~50% per caso'
    print(f'blocking : {env.blocking_fraction:.2f} '
          f'(frazione di episodi con ostacolo SUL percorso EE->target; {how})')
    print(f'shield   : UNCHANGED (d_safe={env.cbf.d_safe} m) — the world gets '
          f'harder, the barrier does not move\n')

    hdr = (f'{"ep":>3} {"x":>5} {"Hz":>6} {"amp":>6} {"peak m/s":>9} '
           f'{"esito":>9} {"passi":>6} {"err m":>7} {"min d":>8} {"interv":>7}')
    print(hdr)
    print('-' * len(hdr))

    rows = []
    for ep in range(args.episodes):
        f = ep / max(args.episodes - 1, 1)
        mult = args.lo + (args.hi - args.lo) * f
        env.obs_speed = base_speed * mult
        env.obs_amp = base_amp * mult
        peak = 2 * math.pi * env.obs_speed * env.obs_amp

        obs, _ = env.reset(seed=args.seed + ep)
        done, steps, dmin, interv = False, 0, math.inf, []
        info = {'dist': float('nan'), 'is_success': False, 'collision': False}
        while not done:
            obs, _, term, trunc, info = env.step(predict(obs))
            steps += 1
            dmin = min(dmin, info['d_min'])
            interv.append(info['cbf_intervention'])
            done = term or trunc

        if info['collision']:
            esito = 'COLLISIONE'
        elif info['is_success']:
            esito = 'successo'
        else:
            esito = 'timeout'
        rows.append((mult, peak, info['is_success'], info['collision'], dmin))
        print(f'{ep + 1:>3} {mult:>5.2f} {env.obs_speed:>6.3f} {env.obs_amp:>6.3f} '
              f'{peak:>9.3f} {esito:>9} {steps:>6} {info["dist"]:>7.3f} '
              f'{dmin:>8.4f} {np.mean(interv):>7.2f}')

    env.close()

    succ = [r for r in rows if r[2]]
    coll = [r for r in rows if r[3]]
    print(f'\nsuccessi {len(succ)}/{len(rows)}   collisioni {len(coll)}/{len(rows)}'
          f'   min d complessivo {min(r[4] for r in rows):+.4f} m')
    if succ:
        print(f'difficolta massima superata : {max(r[0] for r in succ):.2f}x '
              f'({max(r[1] for r in succ):.3f} m/s di picco)')
    if coll:
        print(f'prima collisione a          : {min(r[0] for r in coll):.2f}x '
              f'({min(r[1] for r in coll):.3f} m/s di picco)')
    print('\nNOTA: oltre ~1.1 m/s di picco una parte delle collisioni e\'\n'
          '      STRUTTURALE — la sfera investe un braccio che non puo\'\n'
          '      scansarsi, e nessuna barriera puo\' evitarlo perche\' vincola\n'
          '      il moto del ROBOT, non quello dell\'ostacolo. A quelle\n'
          '      difficolta\' confronta sempre con il baseline "zero".')


if __name__ == '__main__':
    main()
