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
import glob
import os
import re
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


def _sort_key(path: str):
    """Order checkpoints by the number in their filename (episodes or steps)."""
    m = re.findall(r'(\d+)', os.path.basename(path))
    return int(m[-1]) if m else 0


def find_checkpoints(model_dir: str, prefer_onnx: bool = True):
    """Return [(label, path)] for every snapshot in `model_dir`, in order.

    Prefers the `.onnx` beside a `.zip`: that graph is the artifact
    `rl_policy_commander` runs on the robot, so scoring it — rather than the
    training-time `.zip` one serialisation step away — is what makes the number
    a statement about the thing that gets deployed.

    Lives here rather than in `compare_checkpoints` so `--latest` and the
    comparison table resolve snapshots through the same code.
    """
    out = []
    ckpt_dir = os.path.join(model_dir, 'checkpoints')
    for pat, tag in ((os.path.join(ckpt_dir, 'sac_ep*.zip'), 'ep'),
                     (os.path.join(ckpt_dir, 'sac_*_steps.zip'), 'step')):
        for z in sorted(glob.glob(pat), key=_sort_key):
            onnx = z.replace('.zip', '.onnx')
            path = onnx if (prefer_onnx and os.path.isfile(onnx)) else z
            out.append((f'{tag}{_sort_key(z)}', path))
    for name in ('best_model', 'final_model'):
        for ext in ('.onnx', '.zip'):
            p = os.path.join(model_dir, name + ext)
            if os.path.isfile(p):
                out.append((name.replace('_model', ''), p))
                break
    return out


def latest_checkpoint(model_dir: str) -> tuple:
    """(label, path) of the most recently WRITTEN snapshot in `model_dir`.

    Ordered by mtime, not by filename: `best_model` is rewritten whenever eval
    improves, so "the newest file" and "the highest episode number" are
    different questions and the newest file is the one you just trained.
    """
    ckpts = find_checkpoints(model_dir)
    if not ckpts:
        raise SystemExit(
            f'no checkpoints found under {model_dir} — expected '
            f'{model_dir}/checkpoints/*.zip or best_model.zip')
    return max(ckpts, key=lambda lp: os.path.getmtime(lp[1]))


def rollout(predict, env, episodes: int, seed: int) -> dict:
    """Run `episodes` episodes and return the raw per-episode / per-step series.

    Split out of :func:`main` so `compare_checkpoints.py` scores a whole
    training run through exactly the same loop — a second copy would be free to
    drift, and then two "evaluations" of one policy would disagree.

    Episode `i` always runs with `seed + i`, so two policies compared with the
    same seed meet the same targets and the same obstacle trajectories.
    """
    successes, returns, lengths, finals = [], [], [], []
    collisions, d_mins, actives, intervs, slacks, infer_ms = [], [], [], [], [], []

    for ep in range(episodes):
        obs, _ = env.reset(seed=seed + ep)
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

    return dict(successes=successes, returns=returns, lengths=lengths,
                finals=finals, collisions=collisions, d_mins=d_mins,
                actives=actives, intervs=intervs, slacks=slacks,
                infer_ms=infer_ms)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default=None,
                    help='.onnx (preferred), SB3 .zip, or "zero" / "random"')
    ap.add_argument('--latest', metavar='MODEL_DIR', default=None,
                    help='evaluate the newest snapshot in this run directory '
                         '(e.g. franka_sim/models/sac_v3) instead of --model')
    ap.add_argument('--config', default=None,
                    help='config.yaml (default: the one frozen next to the model)')
    ap.add_argument('--episodes', type=int, default=25)
    ap.add_argument('--seed', type=int, default=12345)
    ap.add_argument('--render', action='store_true')
    args = ap.parse_args()

    if not args.model and not args.latest:
        ap.error('pass --model, or --latest <model-dir>')
    label = None
    if args.latest:
        label, args.model = latest_checkpoint(args.latest)
        if args.config is None:
            frozen = os.path.join(args.latest, 'config.yaml')
            if os.path.isfile(frozen):
                args.config = frozen
        print(f'latest snapshot in {args.latest}: {label} -> {args.model}')

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

    r = rollout(predict, env, args.episodes, args.seed)
    env.close()

    def pct(x):
        return 100.0 * float(np.mean(x))

    print('\n── Task ─────────────────────────────────────────────')
    print(f'  success rate        : {pct(r["successes"]):6.1f} %  '
          f'({sum(r["successes"])}/{len(r["successes"])})')
    print(f'  final EE error      : mean {np.mean(r["finals"]):.4f} m   '
          f'median {np.median(r["finals"]):.4f} m')
    print(f'  episode return      : {np.mean(r["returns"]):8.2f} ± {np.std(r["returns"]):.2f}')
    print(f'  episode length      : {np.mean(r["lengths"]):8.1f} steps')
    print('── Safety (CBF shield in the loop) ──────────────────')
    print(f'  collision rate      : {pct(r["collisions"]):6.1f} %  (episodes with d < 0)')
    print(f'  min surface dist    : {np.min(r["d_mins"]):.4f} m')
    print(f'  mean surface dist   : {np.mean(r["d_mins"]):.4f} m')
    print(f'  CBF-active fraction : {pct(r["actives"]):6.1f} %  (steps with ≥1 row)')
    print(f'  mean intervention   : {np.mean(r["intervs"]):.4f} rad/s²')
    print(f'  mean slack          : {np.mean(r["slacks"]):.5f}')
    print('── Inference (deployment-relevant) ──────────────────')
    print(f'  per-step            : mean {np.mean(r["infer_ms"]):.3f} ms   '
          f'p99 {np.percentile(r["infer_ms"], 99):.3f} ms   '
          f'max {np.max(r["infer_ms"]):.3f} ms')


if __name__ == '__main__':
    main()
