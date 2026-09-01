"""Evaluate a policy in FrankaCBF-v0 and report task + safety metrics.

Accepts either the SB3 ``.zip`` or — preferably — the exported ``.onnx``.  The
ONNX graph is what ``rl_policy_commander`` actually runs on the robot, so
scoring THAT closes the last gap in the sim-to-real chain: the numbers reported
here belong to the deployed artifact, not to a training-time model that is one
serialisation step away from it.

    cd /ros2_ws/src && PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl \
        python3 -m franka_sim.scripts.evaluate_policy \
            --model franka_sim/models/<exp>/best_model.onnx --episodes 50

Reported (the paper's safe-exploration table):
    success rate, mean/median final EE error, mean episode return and length,
    collision rate, min/mean surface distance, CBF-active fraction, mean
    intervention ‖q̈_safe − q̈_nom‖ and mean slack, plus per-step inference time.
"""

from __future__ import annotations

import argparse
import os
import time

import numpy as np

from franka_sim.envs.franka_cbf_env import FrankaCBFEnv

_HERE = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_CONFIG = os.path.join(_HERE, '..', 'config.yaml')


def _load_policy(path: str, deterministic: bool = True):
    """Return ``predict(obs) -> action`` for a ``.onnx`` or SB3 ``.zip``.

    ``zero`` and ``random`` are accepted in place of a path: the two reference
    baselines. ``zero`` (q̈_nom = 0 every tick) is the important one — it
    measures how much of the collision rate is the OBSTACLE sweeping into a
    stationary arm rather than the policy driving into it, which is the only
    way to read the safety numbers honestly.
    """
    if path == 'zero':
        return (lambda obs: np.zeros(7, np.float32)), 'zero-action baseline'
    if path == 'random':
        rng = np.random.default_rng(0)
        return (lambda obs: rng.uniform(-1, 1, 7).astype(np.float32)), 'random baseline'
    if path.endswith('.onnx'):
        import onnxruntime as ort
        so = ort.SessionOptions()
        so.intra_op_num_threads = 1
        sess = ort.InferenceSession(path, sess_options=so,
                                    providers=['CPUExecutionProvider'])
        name = sess.get_inputs()[0].name

        def predict(obs):
            return sess.run(None, {name: obs[None].astype(np.float32)})[0][0]
        return predict, 'onnx'

    from stable_baselines3 import SAC
    model = SAC.load(path, device='cpu')

    def predict(obs):
        return model.predict(obs, deterministic=deterministic)[0]
    return predict, 'sb3'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True,
                    help='.onnx (preferred), SB3 .zip, or "zero" / "random"')
    ap.add_argument('--config', default=None,
                    help='config.yaml (default: the one frozen next to the model)')
    ap.add_argument('--episodes', type=int, default=25)
    ap.add_argument('--seed', type=int, default=12345)
    ap.add_argument('--render', action='store_true')
    args = ap.parse_args()

    cfg = args.config
    if cfg is None:
        frozen = os.path.join(os.path.dirname(os.path.abspath(args.model)),
                              'config.yaml')
        cfg = frozen if os.path.isfile(frozen) else _DEFAULT_CONFIG
    if args.model in ('zero', 'random') and args.config is None:
        cfg = _DEFAULT_CONFIG

    predict, kind = _load_policy(args.model)
    env = FrankaCBFEnv(config=cfg, render_mode='human' if args.render else None)
    print(f'policy={args.model} ({kind})  config={cfg}  episodes={args.episodes}')

    successes, returns, lengths, finals = [], [], [], []
    collisions, d_mins, actives, intervs, slacks, infer_ms = [], [], [], [], [], []

    for ep in range(args.episodes):
        obs, _ = env.reset(seed=args.seed + ep)
        ret, done, steps, collided = 0.0, False, 0, False
        info = {'dist': float('nan'), 'is_success': False}
        while not done:
            t0 = time.perf_counter()
            action = predict(obs)
            infer_ms.append((time.perf_counter() - t0) * 1e3)
            obs, r, term, trunc, info = env.step(action)
            ret += r
            steps += 1
            d_mins.append(info['d_min'])
            actives.append(info['cbf_n_c'] > 0)
            intervs.append(info['cbf_intervention'])
            slacks.append(info['cbf_slack'])
            collided |= bool(info['collision'])
            done = term or trunc
        successes.append(bool(info['is_success']))
        collisions.append(collided)
        returns.append(ret)
        lengths.append(steps)
        finals.append(info['dist'])
    env.close()

    def pct(x):
        return 100.0 * float(np.mean(x))

    print('\n── Task ─────────────────────────────────────────────')
    print(f'  success rate        : {pct(successes):6.1f} %  '
          f'({sum(successes)}/{len(successes)})')
    print(f'  final EE error      : mean {np.mean(finals):.4f} m   '
          f'median {np.median(finals):.4f} m')
    print(f'  episode return      : {np.mean(returns):8.2f} ± {np.std(returns):.2f}')
    print(f'  episode length      : {np.mean(lengths):8.1f} steps')
    print('── Safety (CBF shield in the loop) ──────────────────')
    print(f'  collision rate      : {pct(collisions):6.1f} %  (episodes with d < 0)')
    print(f'  min surface dist    : {np.min(d_mins):.4f} m')
    print(f'  mean surface dist   : {np.mean(d_mins):.4f} m')
    print(f'  CBF-active fraction : {pct(actives):6.1f} %  (steps with ≥1 row)')
    print(f'  mean intervention   : {np.mean(intervs):.4f} rad/s²')
    print(f'  mean slack          : {np.mean(slacks):.5f}')
    print('── Inference (deployment-relevant) ──────────────────')
    print(f'  per-step            : mean {np.mean(infer_ms):.3f} ms   '
          f'p99 {np.percentile(infer_ms, 99):.3f} ms   '
          f'max {np.max(infer_ms):.3f} ms')


if __name__ == '__main__':
    main()
