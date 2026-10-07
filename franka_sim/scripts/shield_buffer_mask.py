"""Which replay-buffer transitions are valid in the obstacle-SHIELD-OFF MDP?  (b4 / C3, 2026-10-06)

A buffer collected with the obstacle CBF rows ON stores the POLICY's action, but its next state was produced by
q̈_safe, which the obstacle rows may have bent. Fine-tuning with the rows OFF on such a transition teaches the
critic dynamics the shield-off env does not have — and exactly near the obstacle, where avoidance is learned.

Every input of the shield is in the stored observation: q, q̇ (joint/velocity/slew box, floor, base keep-out,
workspace rows) and the obstacle centre (obstacle rows; J̇q̇ is 0 in the sim because _build_obstacles refreshes its
Jacobian cache on the same state before every filter call). So each transition can be replayed offline, WITH and
WITHOUT the obstacle rows, from the same slew anchor:

    q̈_on  = shield(q, q̇, a·q̈_max, obstacle rows at p_obs)
    q̈_off = shield(q, q̇, a·q̈_max, no obstacle rows)
    valid = |q̈_on − q̈_off| < tol          (the obstacle rows did not bend this action)

Comparing q̈_off with the EXECUTED q̈ = (q̇' − q̇)/dt instead would also flag transitions where physics (a contact,
a joint driven past its limit) made q̈ differ from the command — real in both MDPs, so not a reason to drop them.
That error is still reported (err_exec) as a diagnostic. The slew anchor is the previous transition's executed q̈
in the same episode (0 after a reset, as cbf.reset() does); the oldest transition, whose predecessor is gone, is
marked invalid.

    python3 -m franka_sim.scripts.shield_buffer_mask --buffer B.pkl --config CFG.yaml --out MASK.npz [--workers 10]

MASK.npz: valid (n,) bool indexed by RAW buffer slot, effect (n,), err_exec (n,). train.py --buffer-mask applies it.
"""
from __future__ import annotations

import argparse
import multiprocessing as mp

import mujoco
import numpy as np
import yaml

from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
from franka_sim.envs.obs_layout import spec_from_config

_G = {}


def chronological(n: int, pos: int, full: bool) -> np.ndarray:
    """Raw slot indices oldest → newest (SB3 buffers are circular once full)."""
    return (np.concatenate([np.arange(pos, n), np.arange(0, pos)]) if full
            else np.arange(n))


def _slots(cfg):
    s = {k: (i, w) for k, i, w in spec_from_config(cfg).slots}
    return s['q'][0], s['qdot'][0], s['obstacle'][0]


def _init(cfg):
    cfg = dict(cfg)
    cfg['env'] = dict(cfg['env'], cbf_obstacle_enabled=False)
    env = FrankaCBFEnv(config=cfg)
    env.reset(seed=0)
    _G['env'] = env


def replay_errors(cfg, obs, nxt, act, dones, order, workers=1, chunk=5000):
    """Per transition in `order` (raw slots): obstacle-row effect |q̈_on − q̈_off| and |q̈_off − q̈_exec|."""
    iq, iqd, ip = _slots(cfg)
    dt = 1.0 / float(cfg['env'].get('control_rate_hz', 100.0))
    o, x, a = obs[order], nxt[order], act[order]
    q, qd = o[:, iq:iq + 7].astype(float), o[:, iqd:iqd + 7].astype(float)
    p_obs = o[:, ip:ip + 3].astype(float)
    qdd_exec = (x[:, iqd:iqd + 7].astype(float) - qd) / dt
    d = dones[order].astype(bool)
    anchor = np.zeros_like(qdd_exec)
    anchor[1:] = qdd_exec[:-1]
    anchor[1:][d[:-1]] = 0.0                  # episode start → cbf.reset()
    jobs = [(q[s:s + chunk], qd[s:s + chunk], p_obs[s:s + chunk], a[s:s + chunk], anchor[s:s + chunk])
            for s in range(0, len(q), chunk)]
    if workers > 1:
        with mp.get_context('fork').Pool(workers, initializer=_init, initargs=(cfg,)) as pool:
            parts = pool.map(_replay_cf, jobs)
    else:
        _init(cfg)
        parts = [_replay_cf(j) for j in jobs]
    on = np.concatenate([p[0] for p in parts])
    off = np.concatenate([p[1] for p in parts])
    return np.linalg.norm(on - off, axis=1), np.linalg.norm(off - qdd_exec, axis=1)


def _replay_cf(args):
    """Shield outputs (q̈_on, q̈_off), each (k, 7), with / without the obstacle rows."""
    q, qd, p_obs, act, anchor = args
    env = _G['env']
    on, off = np.empty((len(q), 7)), np.empty((len(q), 7))
    for k in range(len(q)):
        env.data.qpos[env._qadr] = q[k]
        env.data.qvel[env._dadr] = qd[k]
        env.data.mocap_pos[env._obs_mocap] = p_obs[k]
        mujoco.mj_forward(env.model, env.data)
        env._build_obstacles(qd[k])           # cache refresh → J̇q̇ = 0, as in step()
        rows, ee_pos, ee_Jp, ee_jd, _, _ = env._build_obstacles(qd[k])
        a = np.clip(act[k], -1.0, 1.0) * env.qddot_max
        kw = dict(ee_pos=ee_pos, ee_Jp=ee_Jp, ee_jd_qd=ee_jd,
                  floor_pts=env._floor_pts, base_pts=env._base_pts)
        for out, r in ((on, rows), (off, [])):
            env.cbf._qddot_prev = anchor[k].copy()
            out[k], _ = env.cbf.filter(q[k], qd[k], a, r, **kw)
    return on, off


def buffer_mask(cfg, buf, tol=0.5, workers=1):
    """(valid, effect, err_exec), each (n,) over RAW slots of an SB3 ReplayBuffer (n_envs = 1)."""
    n = buf.buffer_size if buf.full else buf.pos
    order = chronological(n, buf.pos, buf.full)
    eff_c, err_c = replay_errors(cfg, buf.observations[:n, 0], buf.next_observations[:n, 0],
                                 buf.actions[:n, 0], buf.dones[:n, 0], order, workers=workers)
    effect, err = np.empty(n), np.empty(n)
    effect[order], err[order] = eff_c, err_c
    valid = effect < tol
    valid[order[0]] = False                   # oldest: slew anchor unknown
    return valid, effect, err


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--buffer', required=True)
    ap.add_argument('--config', required=True, help='the config the buffer was COLLECTED with')
    ap.add_argument('--out', required=True)
    ap.add_argument('--tol', type=float, default=0.5, help='[rad/s²] obstacle-row effect |q̈_on − q̈_off| still counted valid')
    ap.add_argument('--workers', type=int, default=10)
    args = ap.parse_args()
    from stable_baselines3.common.save_util import load_from_pkl
    cfg = yaml.safe_load(open(args.config))
    buf = load_from_pkl(args.buffer)
    valid, effect, err = buffer_mask(cfg, buf, tol=args.tol, workers=args.workers)
    np.savez(args.out, valid=valid, effect=effect, err_exec=err, tol=args.tol)
    print(f'{args.buffer}: {valid.sum()}/{len(valid)} valid ({valid.mean():.3f}); '
          f'exec-replay agrees on {np.mean((err < args.tol) == valid):.4f}; '
          f'err_exec p99 among valid {np.percentile(err[valid], 99):.3g}')


if __name__ == '__main__':
    main()
