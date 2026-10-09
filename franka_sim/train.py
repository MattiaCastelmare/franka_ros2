"""Train a Safe-RL policy (SAC) on FrankaCBF with the CBF shield in the loop.

Stable-Baselines3 SAC on CUDA (RTX 4070), TensorBoard convergence + safety
curves, periodic checkpoints and a best-model eval callback. The policy learns
the TASK; the CBF filter inside the env certifies SAFETY every step, so this is
safe exploration by construction.

    cd /ros2_ws/src && PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl \
        python3 -m franka_sim.train                 # full run (config.yaml)
        python3 -m franka_sim.train --total-timesteps 5000 --exp-name smoke

Outputs (under franka_sim/):
    runs/<exp>/           TensorBoard logs
    models/<exp>/best_model.zip     best eval model  (→ export_onnx.py)
    models/<exp>/checkpoints/       periodic snapshots
    models/<exp>/final_model.zip    end-of-run model
    models/<exp>/config.yaml        frozen config for reproducibility
"""

from __future__ import annotations

import argparse
import os
import shutil
from datetime import datetime

import numpy as np
import yaml

from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import (
    BaseCallback, CheckpointCallback, EvalCallback,
)
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv

from franka_sim.envs.franka_cbf_env import FrankaCBFEnv, precision_bonus
from franka_sim.envs.obs_layout import spec_from_config

_HERE = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_CONFIG = os.path.join(_HERE, 'config.yaml')


def make_env(config: dict, seed: int, render_mode=None):
    def _init():
        env = FrankaCBFEnv(config=config, render_mode=render_mode)
        env = Monitor(env, info_keywords=('is_success', 'collision'))
        env.reset(seed=seed)
        return env
    return _init


class SafetyMetricsCallback(BaseCallback):
    """Log CBF/safety scalars to TensorBoard: the paper's safe-exploration curves.

    Accumulates per-step signals from the env `info` dict and writes rolling
    means every `log_freq` steps: collision rate, min surface distance, CBF
    intervention magnitude, slack, and fraction of steps with the shield active.
    """

    def __init__(self, log_freq: int = 1000, verbose: int = 0):
        super().__init__(verbose)
        self.log_freq = log_freq
        self._reset_buffers()

    def _reset_buffers(self):
        self._d_min, self._interv, self._slack = [], [], []
        self._active, self._collisions, self._n = 0, 0, 0
        self._viol, self._con_term = {}, 0

    def _on_step(self) -> bool:
        for info in self.locals.get('infos', []):
            if 'd_min' not in info:
                continue
            self._n += 1
            self._d_min.append(info['d_min'])
            self._interv.append(info['cbf_intervention'])
            self._slack.append(info['cbf_slack'])
            self._active += int(info['cbf_n_c'] > 0)
            self._collisions += int(info.get('collision', False))
            for k, v in (info.get('constraint_excess') or {}).items():
                self._viol[k] = self._viol.get(k, 0) + int(v > 0)
            self._con_term += int(info.get('constraint_terminated', False))
        if self._n >= self.log_freq:
            self.logger.record('safety/collision_rate', self._collisions / self._n)
            self.logger.record('safety/min_surface_dist', float(np.min(self._d_min)))
            self.logger.record('safety/mean_surface_dist', float(np.mean(self._d_min)))
            self.logger.record('safety/cbf_active_frac', self._active / self._n)
            self.logger.record('safety/mean_intervention', float(np.mean(self._interv)))
            self.logger.record('safety/mean_slack', float(np.mean(self._slack)))
            for k, c in self._viol.items():
                self.logger.record(f'constraints/{k}_step_rate', c / self._n)
            self.logger.record('constraints/terminations_per_step', self._con_term / self._n)
            self._reset_buffers()
        return True


class EpisodeCheckpointCallback(BaseCallback):
    """Snapshot the model every `every` COMPLETED EPISODES.

    SB3's own CheckpointCallback counts environment STEPS, which is the wrong
    axis for watching a policy come up: episodes end early on success and on
    collision, so the number of steps per episode changes by 5x over a run and
    step-spaced snapshots land at wildly different amounts of task experience.
    Episode-spaced ones are directly comparable, and they are what
    `compare_checkpoints.py` plots against.

    Episodes are counted from the Monitor wrapper's `episode` key rather than
    from `dones`, so a truncated episode counts exactly once and a vectorised
    env with several sub-envs still totals correctly.

    Each snapshot is also exported to ONNX when `export` is set, because the
    ONNX graph — not the .zip — is what the robot runs; a checkpoint you cannot
    hand to `rl_policy_commander` is a checkpoint you cannot actually test.
    """

    def __init__(self, every: int, save_path: str, export: bool = True,
                 name_prefix: str = 'sac_ep', start_episodes: int = 0,
                 save_buffer: bool = False, verbose: int = 1):
        super().__init__(verbose)
        self.every = int(every)
        self.save_path = save_path
        self.export = export
        self.name_prefix = name_prefix
        # On --resume, continue the count so the new segment's snapshots do not
        # overwrite the first segment's sac_ep000200..N files.
        self.n_episodes = int(start_episodes)
        # SAC.save() does not include the replay buffer, so a resume from a
        # plain .zip restarts on an empty buffer with no learning_starts
        # warm-up (sac_b2 segment 2: ent_coef 0.75 -> ~10, critic_loss 1e4 ->
        # 1e6). Keep ONE rolling copy (~0.5 GB) next to the snapshots.
        self.save_buffer = save_buffer
        self._next_at = (self.n_episodes // self.every + 1) * self.every if self.every > 0 else 0

    def _on_step(self) -> bool:
        for info in self.locals.get('infos', []):
            if 'episode' in info:
                self.n_episodes += 1
        self.logger.record('time/episodes_completed', self.n_episodes)

        if self.every > 0 and self.n_episodes >= self._next_at:
            self._next_at += self.every
            base = os.path.join(self.save_path,
                                f'{self.name_prefix}{self.n_episodes:06d}')
            self.model.save(base)
            if self.save_buffer:
                self.model.save_replay_buffer(
                    os.path.join(self.save_path, 'replay_buffer_latest.pkl'))
            if self.verbose:
                print(f'\n[episode-ckpt] {self.n_episodes} episodes '
                      f'({self.num_timesteps} steps) -> {base}.zip')
            if self.export:
                self._export_onnx(base + '.zip')
        return True

    def _export_onnx(self, zip_path: str):
        """Best-effort ONNX export; a failure must never kill a training run."""
        try:
            from franka_sim.export_onnx import export
            export(zip_path, zip_path.replace('.zip', '.onnx'), verbose=False)
        except Exception as exc:                      # noqa: BLE001
            if self.verbose:
                print(f'[episode-ckpt] ONNX export skipped: {exc}')


class PolicyWarmupCallback(BaseCallback):
    """No gradient steps for the first `steps` env steps of a resumed run.

    A resume without a replay buffer starts on an EMPTY buffer, and SB3's own
    warm-up (learning_starts) would fill it with RANDOM actions. Holding
    gradient_steps at 0 instead fills it with the loaded policy's own
    (stochastic) behaviour under the NEW config's reward, then restores the
    configured value — the clean way to fine-tune on a reward term that
    relabel_buffer cannot reconstruct from the observation (e.g. w_slack).
    """

    def __init__(self, steps: int, verbose: int = 1):
        super().__init__(verbose)
        self.steps = int(steps)
        self._restore = None

    def _on_training_start(self) -> None:
        self._end = self.model.num_timesteps + self.steps
        self._restore = self.model.gradient_steps
        self.model.gradient_steps = 0
        if self.verbose:
            print(f'[policy-warmup] collecting {self.steps} steps with the loaded policy, no updates')

    def _on_step(self) -> bool:
        if self._restore is not None and self.model.num_timesteps >= self._end:
            self.model.gradient_steps = self._restore
            self._restore = None
            if self.verbose:
                print(f'[policy-warmup] done at {self.model.num_timesteps}: '
                      f'buffer {self.model.replay_buffer.size()}, gradient_steps {self.model.gradient_steps}')
        return True


def _obs_reward_terms(cfg: dict, obs, nxt, terminated, slot):
    """The reward terms of `cfg` that are a function of (obs, next_obs) alone."""
    rw = cfg.get('reward', {})
    e, t, d = slot['ee_pos'][0], slot['target'][0], slot['d_min'][0]
    dist = np.linalg.norm(nxt[..., e:e + 3] - nxt[..., t:t + 3], axis=-1)
    add = np.asarray(precision_bonus(rw, dist), dtype=np.float64) * np.ones_like(dist)
    w_obs = float(rw.get('w_obs_margin', 0.0))
    if w_obs > 0.0:
        margin = float(rw.get('obs_soft_margin', 0.10))
        pen = lambda x: np.clip((margin - x) / margin, 0.0, 1.0)
        pen_s, pen_n = pen(obs[..., d]), pen(nxt[..., d])
        if str(rw.get('obs_shaping', 'penalty')) == 'potential':
            gamma = float(rw.get('shaping_gamma', cfg['rl'].get('gamma', 0.99)))
            add += gamma * (1.0 - terminated) * (-w_obs * pen_n) + w_obs * pen_s
        else:
            add -= w_obs * pen_n
    return add


def relabel_buffer(buf, cfg: dict, old_cfg: dict | None = None):
    """Swap the obs-derivable reward terms of `old_cfg` for those of `cfg`.

    For a fine-tune whose config changes w_prec and/or w_obs_margin (or its
    shaping mode): the stored rewards become exactly what the new env would
    have paid (up to float32 obs rounding), so the critic is not fed two
    reward functions at once. `old_cfg` is the config the buffer was collected
    with; None means it had both terms at 0 (the v11 behaviour). Everything
    else (intervention, slack, …) is not in the obs and must be unchanged
    between the two configs.
    """
    slot = {k: (s, w) for k, s, w in spec_from_config(cfg).slots}
    n = buf.buffer_size if buf.full else buf.pos
    obs, nxt = buf.observations[:n], buf.next_observations[:n]
    terminated = buf.dones[:n] * (1.0 - buf.timeouts[:n])
    add = _obs_reward_terms(cfg, obs, nxt, terminated, slot)
    if old_cfg is not None:
        add -= _obs_reward_terms(old_cfg, obs, nxt, terminated, slot)
    buf.rewards[:n] += add.astype(buf.rewards.dtype)
    print(f'relabelled {n} transitions: mean added reward {add.mean():+.4f}')


def _chronological(buf) -> np.ndarray:
    """Raw slot indices oldest → newest (an SB3 buffer is circular once full)."""
    n = buf.buffer_size if buf.full else buf.pos
    return (np.concatenate([np.arange(buf.pos, n), np.arange(0, buf.pos)])
            if buf.full else np.arange(n))


def widen_buffer(old, new, cfg: dict):
    """Copy a 24-D buffer into a `new` one whose obs also carries v_obs (b4 / C4).

    v_obs is what the env would have observed: the finite difference of the
    observed obstacle centre, (p_t − p_{t−1})/dt, and 0 on the first obs after
    a reset. A transition's next_obs velocity is (p' − p)/dt; its obs velocity
    is the previous transition's next_obs velocity in the same episode. The
    oldest transition, whose predecessor is gone, reuses its own next velocity.
    Raw slots are preserved, so a mask computed on `old` indexes `new` too.
    """
    slot = {k: (s, w) for k, s, w in spec_from_config(cfg).slots}
    po, (pv, wv) = slot['obstacle'][0], slot['v_obs']
    d_old = old.observations.shape[-1]
    assert pv == d_old and new.observations.shape[-1] == d_old + wv, 'v_obs must be appended'
    assert new.buffer_size == old.buffer_size and old.n_envs == new.n_envs == 1
    dt = 1.0 / float(cfg['env'].get('control_rate_hz', 100.0))
    order = _chronological(old)
    o, x = old.observations[order, 0], old.next_observations[order, 0]
    v_next = (x[:, po:po + 3].astype(np.float64) - o[:, po:po + 3]) / dt
    v_obs = np.empty_like(v_next)
    v_obs[0] = v_next[0]
    v_obs[1:] = v_next[:-1]
    v_obs[1:][old.dones[order[:-1], 0].astype(bool)] = 0.0
    n = len(order)
    for name in ('observations', 'next_observations'):
        getattr(new, name)[:n, :, :d_old] = getattr(old, name)[:n]
    new.observations[order, 0, pv:pv + wv] = v_obs
    new.next_observations[order, 0, pv:pv + wv] = v_next
    for name in ('actions', 'rewards', 'dones', 'timeouts'):
        getattr(new, name)[:n] = getattr(old, name)[:n]
    new.pos, new.full = old.pos, old.full
    print(f'widened {n} transitions {d_old}→{d_old + wv}-D: |v_obs| p50 '
          f'{np.median(np.linalg.norm(v_next, axis=1)):.3f} max {np.linalg.norm(v_next, axis=1).max():.3f} m/s')


def mask_buffer(buf, valid: np.ndarray):
    """Keep only the transitions with valid[raw slot] (b4 / C3), compacted, oldest first."""
    order = _chronological(buf)
    keep = order[valid[order]]
    k = len(keep)
    for name in ('observations', 'next_observations', 'actions', 'rewards', 'dones', 'timeouts'):
        arr = getattr(buf, name)
        arr[:k] = arr[keep]
    buf.pos, buf.full = k % buf.buffer_size, False
    print(f'buffer mask: kept {k}/{len(order)} transitions ({k / max(1, len(order)):.3f})')


def widen_model(old, new):
    """Copy `old`'s weights + optimizer state into `new`, whose obs is wider (b4 / C4).

    New input columns get ZERO weights (and zero Adam moments), so `new` is the
    same function as `old` on the old slots whatever the new slots carry:
    actor first layer [out, obs] → columns appended at the end; critic first
    layers [out, obs + act] → columns inserted between obs and action.
    """
    import torch as th
    d_old = old.observation_space.shape[0]
    d_new = new.observation_space.shape[0]

    def pad(key, t_old, shape):
        if tuple(t_old.shape) == tuple(shape):
            return t_old.clone()
        assert t_old.dim() == 2 and t_old.shape[0] == shape[0] and shape[1] - t_old.shape[1] == d_new - d_old, key
        z = th.zeros(shape[0], d_new - d_old, dtype=t_old.dtype, device=t_old.device)
        if key.startswith('critic'):
            return th.cat([t_old[:, :d_old], z, t_old[:, d_old:]], dim=1)
        return th.cat([t_old, z], dim=1)

    sd_old, sd_new = old.policy.state_dict(), new.policy.state_dict()
    new.policy.load_state_dict({k: pad(k, sd_old[k].to(v.device), v.shape) for k, v in sd_new.items()})

    def copy_opt(o_old, o_new, names):
        s_old, s_new = o_old.state_dict(), o_new.state_dict()
        shapes = {i: sd_new[nm].shape for i, nm in enumerate(names)}
        for i, st in s_old['state'].items():
            s_new['state'][i] = {k: (pad(names[i], v.to(sd_new[names[i]].device), shapes[i])
                                     if th.is_tensor(v) and v.dim() == 2 else v.clone())
                                 for k, v in st.items()}
        o_new.load_state_dict(s_new)

    actor_names = [k for k in sd_new if k.startswith('actor.')]
    critic_names = [k for k in sd_new if k.startswith('critic.')]
    copy_opt(old.actor.optimizer, new.actor.optimizer, actor_names)
    copy_opt(old.critic.optimizer, new.critic.optimizer, critic_names)
    if getattr(old, 'log_ent_coef', None) is not None:
        with th.no_grad():
            new.log_ent_coef.copy_(old.log_ent_coef.to(new.log_ent_coef.device))
        new.ent_coef_optimizer.load_state_dict(old.ent_coef_optimizer.state_dict())
    new.num_timesteps = old.num_timesteps
    new._n_updates = old._n_updates
    print(f'widened model {d_old}→{d_new}-D obs (zero columns), ent_coef '
          f'{float(th.exp(new.log_ent_coef.detach())):.4f}, timesteps {new.num_timesteps}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default=_DEFAULT_CONFIG)
    ap.add_argument('--exp-name', default=None, help='run name (default: timestamp)')
    ap.add_argument('--total-timesteps', type=int, default=None)
    ap.add_argument('--seed', type=int, default=None)
    ap.add_argument('--device', default=None, help='cuda | cpu (default: config)')
    ap.add_argument('--resume', default=None, help='path to a .zip to continue')
    ap.add_argument('--checkpoint-every-episodes', type=int, default=None,
                    help='snapshot every N completed episodes (0 disables); '
                         'default: rl.checkpoint_freq_episodes in the config')
    ap.add_argument('--start-episode', type=int, default=0,
                    help='episode count to continue from on --resume '
                         '(e.g. 3000 for sac_ep003000.zip)')
    ap.add_argument('--save-replay-buffer', action='store_true',
                    help='with each episode checkpoint, overwrite '
                         'checkpoints/replay_buffer_latest.pkl')
    ap.add_argument('--resume-buffer', default=None,
                    help='replay buffer .pkl to load on --resume')
    ap.add_argument('--relabel-buffer', action='store_true',
                    help='on --resume-buffer, add the obs-derivable reward terms '
                         '(w_prec, w_obs_margin) of THIS config to the stored '
                         'rewards (the old run must have had them at 0)')
    ap.add_argument('--relabel-from', default=None,
                    help='with --relabel-buffer: the config the buffer was COLLECTED with; '
                         'its obs-derivable terms are subtracted (default: assume they were 0)')
    ap.add_argument('--buffer-mask', default=None,
                    help='on --resume-buffer: .npz from scripts/shield_buffer_mask.py; keep only '
                         'the transitions valid with the obstacle shield OFF')
    ap.add_argument('--widen-obs', action='store_true',
                    help='on --resume: the config observes MORE slots than the loaded model '
                         '(e.g. + obs.obstacle_velocity); new inputs start with zero weights and '
                         'the loaded buffer gets the new slots relabelled')
    ap.add_argument('--policy-warmup', type=int, default=0,
                    help='on --resume: first N steps collect data with the loaded '
                         'policy and do no gradient updates (use without --resume-buffer)')
    ap.add_argument('--no-episode-onnx', action='store_true',
                    help='skip the ONNX export of each episode checkpoint')
    args = ap.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    rl = cfg['rl']

    seed   = args.seed if args.seed is not None else int(rl.get('seed', 0))
    device = args.device or rl.get('device', 'cuda')
    total  = args.total_timesteps or int(rl.get('total_timesteps', 2_000_000))
    exp    = args.exp_name or datetime.now().strftime('sac_%Y%m%d_%H%M%S')

    tb_dir    = os.path.join(_HERE, rl.get('tensorboard_log', 'runs'))
    model_dir = os.path.join(_HERE, rl.get('save_path', 'models'), exp)
    ckpt_dir  = os.path.join(model_dir, 'checkpoints')
    os.makedirs(ckpt_dir, exist_ok=True)
    shutil.copy(args.config, os.path.join(model_dir, 'config.yaml'))

    # ── Envs (train + eval share the config; eval env is deterministic-ish) ──
    # n_envs > 1 runs MuJoCo + the CBF QP in separate OS processes (spawn, not
    # fork — MUJOCO_GL=egl holds a CUDA/GL context that a fork would corrupt).
    # Each sub-env owns its own FrankaCBFEnv/cbf_filter instance (no shared
    # mutable state, see envs/franka_cbf_env.py), so the shield each rollout
    # meets is exactly the same per-step certificate as with n_envs=1 — this
    # only parallelizes the CPU-bound sim/QP throughput that was starving the
    # GPU, it does not relax or batch the safety filter itself.
    n_envs = max(1, int(rl.get('n_envs', 1)))
    env_fns = [make_env(cfg, seed + i) for i in range(n_envs)]
    train_env = (SubprocVecEnv(env_fns, start_method='spawn') if n_envs > 1
                else DummyVecEnv(env_fns))
    eval_env  = DummyVecEnv([make_env(cfg, seed + 1000)])

    policy_kwargs = dict(net_arch=list(rl.get('net_arch', [256, 256])))
    ent_coef = rl.get('ent_coef', 'auto')
    # 'auto' = SB3's −dim(A) = −7. The average over the batch meets it with
    # the saturated transit actions, which leaves the policy at std ≈ 0.5 on
    # the target itself (c1 probe) — a lower target forces it to settle.
    target_entropy = rl.get('target_entropy', 'auto')
    if target_entropy != 'auto':
        target_entropy = float(target_entropy)

    def fresh_model():
        return SAC(
            rl.get('policy', 'MlpPolicy'), train_env,
            learning_rate=float(rl.get('learning_rate', 3e-4)),
            buffer_size=int(rl.get('buffer_size', 1_000_000)),
            batch_size=int(rl.get('batch_size', 512)),
            gamma=float(rl.get('gamma', 0.99)),
            tau=float(rl.get('tau', 0.005)),
            train_freq=int(rl.get('train_freq', 1)),
            gradient_steps=int(rl.get('gradient_steps', 1)),
            learning_starts=int(rl.get('learning_starts', 10_000)),
            ent_coef=ent_coef,
            target_entropy=target_entropy,
            policy_kwargs=policy_kwargs,
            device=device, seed=seed, verbose=1, tensorboard_log=tb_dir,
        )

    if args.resume:
        print(f'Resuming from {args.resume}')
        # SAC.load restores the SAVED hyper-parameters; take the ones this
        # config asks for, so a fine-tune can lower the learning rate.
        lr = float(rl.get('learning_rate', 3e-4))
        if args.widen_obs:
            # The saved model cannot be loaded against the wider env (SB3
            # checks the observation space): build a fresh model from THIS
            # config and graft the old weights in, zero on the new inputs.
            old = SAC.load(args.resume, device=device)
            model = fresh_model()
            widen_model(old, model)
        else:
            model = SAC.load(args.resume, env=train_env, device=device,
                             tensorboard_log=tb_dir,
                             custom_objects=dict(
                                 learning_rate=lr, lr_schedule=lambda _: lr,
                                 gradient_steps=int(rl.get('gradient_steps', 1)),
                                 batch_size=int(rl.get('batch_size', 512)),
                                 **({} if target_entropy == 'auto'
                                    else dict(target_entropy=target_entropy))))
        model.set_random_seed(seed)
        print(f'resume hparams: lr={lr} gradient_steps={model.gradient_steps} '
              f'batch_size={model.batch_size} seed={seed} '
              f'target_entropy={model.target_entropy}')
        if args.resume_buffer:
            if args.widen_obs:
                from stable_baselines3.common.save_util import load_from_pkl
                old_buf = load_from_pkl(args.resume_buffer)
                widen_buffer(old_buf, model.replay_buffer, cfg)
                del old_buf
            else:
                model.load_replay_buffer(args.resume_buffer)
            print(f'Loaded replay buffer ({model.replay_buffer.size()} '
                  f'transitions) from {args.resume_buffer}')
            if args.relabel_buffer:
                old_cfg = None
                if args.relabel_from:
                    with open(args.relabel_from) as f:
                        old_cfg = yaml.safe_load(f)
                relabel_buffer(model.replay_buffer, cfg, old_cfg)
            if args.buffer_mask:
                mask_buffer(model.replay_buffer, np.load(args.buffer_mask)['valid'])
        else:
            print('WARNING: resuming with an EMPTY replay buffer')
    else:
        model = fresh_model()

    print(f'device={model.device}  n_envs={n_envs}  total_timesteps={total}  exp={exp}')

    ep_every = (args.checkpoint_every_episodes
                if args.checkpoint_every_episodes is not None
                else int(rl.get('checkpoint_freq_episodes', 0)))

    callbacks = [
        CheckpointCallback(
            save_freq=int(rl.get('checkpoint_freq', 50_000)),
            save_path=ckpt_dir, name_prefix='sac'),
        EvalCallback(
            eval_env, best_model_save_path=model_dir,
            log_path=model_dir, eval_freq=int(rl.get('eval_freq', 25_000)),
            n_eval_episodes=10, deterministic=True, render=False),
        SafetyMetricsCallback(log_freq=2000),
    ]
    if args.policy_warmup > 0:
        callbacks.insert(0, PolicyWarmupCallback(args.policy_warmup))
    if ep_every > 0:
        callbacks.append(EpisodeCheckpointCallback(
            every=ep_every, save_path=ckpt_dir,
            export=not args.no_episode_onnx,
            start_episodes=args.start_episode,
            save_buffer=args.save_replay_buffer))
        print(f'episode checkpoints: every {ep_every} episodes → {ckpt_dir}')

    model.learn(total_timesteps=total, callback=callbacks, tb_log_name=exp,
                progress_bar=True, reset_num_timesteps=not bool(args.resume))
    model.save(os.path.join(model_dir, 'final_model'))
    print(f'\nDone. Models in {model_dir}\n'
          f'  export:  python3 -m franka_sim.export_onnx '
          f'--model {os.path.join(model_dir, "best_model.zip")}')


if __name__ == '__main__':
    main()
