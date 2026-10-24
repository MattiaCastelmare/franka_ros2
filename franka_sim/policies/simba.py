"""SimBa actor/critic for Stable-Baselines3 SAC.

SimBa — "Simplicity Bias for Scaling Up Parameters in Deep RL" (Lee, Hwang,
Kim, Stone, Choo, Seno — ICLR 2025, arXiv 2410.09754) — is a network, not an
algorithm: plug it into SAC and the same SAC scales with parameters where a
plain MLP degrades. Three parts, all here:

1. **RSNorm** — running-statistics observation normalisation. Our 51-D vector
   mixes joint angles (rad, ±3), velocities (rad/s, ±5), metres (0-1) and unit
   normals: an MLP sees a feature scale, not a feature. Every model so far was
   trained on raw inputs, which is one reason the per-link geometry block
   (obs.control_point_geometry) "failed to learn" in v10/b3.
2. **Residual pre-LayerNorm blocks** with a 4× inverted bottleneck:
   x ← x + W₂ ReLU(W₁ LN(x)). Residual + LN keep the function simple at init
   and let the critic grow without the overfitting that breaks a deep MLP.
3. **Post-LayerNorm** before the output heads.

Paper defaults for SAC (Table 7): critic 2 blocks × 512, actor 1 block × 128,
AdamW lr 1e-4 wd 1e-2, batch 256, replay ratio 2, target entropy |A|/2,
α₀ = 1e-2. Scaling the critic (width > depth) helps; scaling the actor hurts.

Integration
-----------
`SimbaSACPolicy` is a drop-in `policy` for `SAC(...)` (train.py selects it with
`rl.policy: SimbaPolicy`). `SimbaActor` keeps SB3's `Actor` interface so
`export_onnx.py` / `export_ensemble_onnx.py` and `rl_policy_commander` are
unchanged: the RSNorm is part of the actor graph, so the exported ONNX is
self-contained (the robot does not need the statistics separately).

The running statistics live in buffers named ``running_*``: SB3's SAC copies
every critic buffer with that prefix into the target critic at each update
(`batch_norm_stats`), so critic and target critic always normalise alike.
`RSNormUpdateCallback` feeds every collected observation to the normalisers
(actor + critics), the way the paper updates them online.
"""

from __future__ import annotations

from typing import Any, Optional

import numpy as np
import torch as th
from torch import nn
from torch.nn import functional as F

from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.policies import ContinuousCritic
from stable_baselines3.common.torch_layers import FlattenExtractor
from stable_baselines3.common.type_aliases import Schedule
from stable_baselines3.sac.policies import Actor, SACPolicy

#: Paper defaults (Table 7) — overridable per run from config `rl.simba`.
DEFAULTS = dict(actor_hidden=128, actor_blocks=1, critic_hidden=512,
                critic_blocks=2, expansion=4, weight_decay=1e-2)


class RSNorm(nn.Module):
    """Running mean/variance normaliser over the first ``dim`` input columns.

    Welford-style batched update in float64 (the count reaches 10⁷ samples;
    float32 accumulation drifts). Columns past ``dim`` (the critic's action
    columns) pass through untouched. Statistics only change through
    :meth:`update`, never inside ``forward``, so a forward pass is pure and the
    traced ONNX graph is a fixed affine map.
    """

    def __init__(self, dim: int, eps: float = 1e-8):
        super().__init__()
        self.dim = int(dim)
        self.eps = float(eps)
        self.register_buffer('running_mean', th.zeros(self.dim, dtype=th.float64))
        self.register_buffer('running_var', th.ones(self.dim, dtype=th.float64))
        self.register_buffer('running_count', th.tensor(1e-4, dtype=th.float64))

    @th.no_grad()
    def update(self, x: th.Tensor) -> None:
        x = x.detach().to(self.running_mean.device, th.float64).reshape(-1, x.shape[-1])[:, :self.dim]
        b = float(x.shape[0])
        if b == 0:
            return
        bm = x.mean(0)
        bv = x.var(0, unbiased=False)
        n = self.running_count
        tot = n + b
        delta = bm - self.running_mean
        new_mean = self.running_mean + delta * (b / tot)
        new_var = (self.running_var * n + bv * b + delta * delta * (n * b / tot)) / tot
        self.running_mean.copy_(new_mean)
        self.running_var.copy_(new_var)
        self.running_count.copy_(tot)

    def forward(self, x: th.Tensor) -> th.Tensor:
        mean = self.running_mean.to(x.dtype)
        inv = th.rsqrt(self.running_var + self.eps).to(x.dtype)
        if x.shape[-1] == self.dim:
            return (x - mean) * inv
        head = (x[..., :self.dim] - mean) * inv
        return th.cat([head, x[..., self.dim:]], dim=-1)

    def extra_repr(self) -> str:
        return f'dim={self.dim}, eps={self.eps}'


class ResidualBlock(nn.Module):
    """x + W₂ ReLU(W₁ LN(x)), hidden width ``expansion·dim``."""

    def __init__(self, dim: int, expansion: int = 4):
        super().__init__()
        self.ln = nn.LayerNorm(dim)
        self.fc1 = nn.Linear(dim, expansion * dim)
        self.fc2 = nn.Linear(expansion * dim, dim)
        # He-normal, as in the reference implementation (scale_rl/networks/layers.py).
        for fc in (self.fc1, self.fc2):
            nn.init.kaiming_normal_(fc.weight, nonlinearity='relu')
            nn.init.zeros_(fc.bias)

    def forward(self, x: th.Tensor) -> th.Tensor:
        return x + self.fc2(F.relu(self.fc1(self.ln(x))))


class SimbaTrunk(nn.Module):
    """Linear embed → N residual blocks → LayerNorm. Output width ``hidden``."""

    def __init__(self, in_dim: int, hidden: int, num_blocks: int, expansion: int = 4):
        super().__init__()
        self.embed = nn.Linear(in_dim, hidden)
        nn.init.orthogonal_(self.embed.weight, gain=1.0)
        nn.init.zeros_(self.embed.bias)
        self.blocks = nn.Sequential(*[ResidualBlock(hidden, expansion) for _ in range(num_blocks)])
        self.post_ln = nn.LayerNorm(hidden)
        self.out_dim = hidden

    def forward(self, x: th.Tensor) -> th.Tensor:
        return self.post_ln(self.blocks(self.embed(x)))


class SimbaActor(Actor):
    """SB3 SAC actor with ``latent_pi = RSNorm → SimbaTrunk``.

    ``forward`` / ``action_log_prob`` / ``get_action_dist_params`` are inherited
    unchanged: they only call ``self.latent_pi``, ``self.mu`` and
    ``self.log_std``, which is exactly what is swapped here.
    """

    def __init__(self, observation_space, action_space, net_arch, features_extractor,
                 features_dim: int, activation_fn=nn.ReLU, use_sde: bool = False,
                 log_std_init: float = -3, full_std: bool = True, use_expln: bool = False,
                 clip_mean: float = 2.0, normalize_images: bool = True,
                 hidden_dim: int = 128, num_blocks: int = 1, expansion: int = 4):
        if use_sde:
            raise ValueError('SimbaActor does not support gSDE')
        super().__init__(observation_space, action_space, [], features_extractor,
                         features_dim, activation_fn, use_sde, log_std_init, full_std,
                         use_expln, clip_mean, normalize_images)
        self.hidden_dim, self.num_blocks, self.expansion = int(hidden_dim), int(num_blocks), int(expansion)
        self.obs_norm = RSNorm(features_dim)
        self.latent_pi = nn.Sequential(self.obs_norm,
                                       SimbaTrunk(features_dim, self.hidden_dim, self.num_blocks, self.expansion))
        action_dim = int(np.prod(action_space.shape))
        self.mu = nn.Linear(self.hidden_dim, action_dim)
        self.log_std = nn.Linear(self.hidden_dim, action_dim)
        # Small output heads: the actor starts near a = tanh(0) with the configured log-std.
        for head in (self.mu, self.log_std):
            nn.init.orthogonal_(head.weight, gain=0.01)
            nn.init.zeros_(head.bias)
        with th.no_grad():
            self.log_std.bias.fill_(float(log_std_init))

    def _get_constructor_parameters(self) -> dict[str, Any]:
        data = super()._get_constructor_parameters()
        data.update(hidden_dim=self.hidden_dim, num_blocks=self.num_blocks, expansion=self.expansion)
        return data


class SimbaCritic(ContinuousCritic):
    """n_critics × (RSNorm(obs) ⊕ action → SimbaTrunk → Linear(1))."""

    def __init__(self, observation_space, action_space, net_arch, features_extractor,
                 features_dim: int, activation_fn=nn.ReLU, normalize_images: bool = True,
                 n_critics: int = 2, share_features_extractor: bool = True,
                 hidden_dim: int = 512, num_blocks: int = 2, expansion: int = 4):
        super().__init__(observation_space, action_space, [], features_extractor, features_dim,
                         activation_fn, normalize_images, n_critics, share_features_extractor)
        self.hidden_dim, self.num_blocks, self.expansion = int(hidden_dim), int(num_blocks), int(expansion)
        action_dim = int(np.prod(action_space.shape))
        self.q_networks = []
        for idx in range(n_critics):
            head = nn.Linear(self.hidden_dim, 1)
            nn.init.orthogonal_(head.weight, gain=1.0)
            nn.init.zeros_(head.bias)
            q_net = nn.Sequential(RSNorm(features_dim),
                                  SimbaTrunk(features_dim + action_dim, self.hidden_dim,
                                             self.num_blocks, self.expansion),
                                  head)
            self.add_module(f'qf{idx}', q_net)      # replaces the MLP super() registered
            self.q_networks.append(q_net)


class SimbaSACPolicy(SACPolicy):
    """SACPolicy whose actor/critics are SimBa networks (AdamW, weight decay).

    Extra constructor arguments (saved with the model, so ``SAC.load`` rebuilds
    the same architecture): ``actor_hidden``, ``actor_blocks``,
    ``critic_hidden``, ``critic_blocks``, ``expansion``, ``weight_decay``.
    ``net_arch`` is accepted for interface compatibility and ignored.
    """

    def __init__(self, observation_space, action_space, lr_schedule: Schedule,
                 net_arch=None, activation_fn=nn.ReLU, use_sde: bool = False,
                 log_std_init: float = -3, use_expln: bool = False, clip_mean: float = 2.0,
                 features_extractor_class=FlattenExtractor, features_extractor_kwargs=None,
                 normalize_images: bool = True, optimizer_class=th.optim.AdamW,
                 optimizer_kwargs: Optional[dict] = None, n_critics: int = 2,
                 share_features_extractor: bool = False,
                 actor_hidden: int = DEFAULTS['actor_hidden'], actor_blocks: int = DEFAULTS['actor_blocks'],
                 critic_hidden: int = DEFAULTS['critic_hidden'], critic_blocks: int = DEFAULTS['critic_blocks'],
                 expansion: int = DEFAULTS['expansion'], weight_decay: float = DEFAULTS['weight_decay']):
        self.simba = dict(actor_hidden=int(actor_hidden), actor_blocks=int(actor_blocks),
                          critic_hidden=int(critic_hidden), critic_blocks=int(critic_blocks),
                          expansion=int(expansion), weight_decay=float(weight_decay))
        optimizer_kwargs = dict(optimizer_kwargs or {})
        if optimizer_class is th.optim.AdamW:
            optimizer_kwargs.setdefault('weight_decay', float(weight_decay))
        super().__init__(observation_space, action_space, lr_schedule, net_arch, activation_fn,
                         use_sde, log_std_init, use_expln, clip_mean, features_extractor_class,
                         features_extractor_kwargs, normalize_images, optimizer_class,
                         optimizer_kwargs, n_critics, share_features_extractor)

    def make_actor(self, features_extractor=None) -> SimbaActor:
        kw = self._update_features_extractor(self.actor_kwargs, features_extractor)
        kw.pop('full_std', None)
        return SimbaActor(**kw, hidden_dim=self.simba['actor_hidden'],
                          num_blocks=self.simba['actor_blocks'],
                          expansion=self.simba['expansion']).to(self.device)

    def make_critic(self, features_extractor=None) -> SimbaCritic:
        kw = self._update_features_extractor(self.critic_kwargs, features_extractor)
        return SimbaCritic(**kw, hidden_dim=self.simba['critic_hidden'],
                           num_blocks=self.simba['critic_blocks'],
                           expansion=self.simba['expansion']).to(self.device)

    def _get_constructor_parameters(self) -> dict[str, Any]:
        data = super()._get_constructor_parameters()
        data.update(self.simba)
        return data


class RSNormUpdateCallback(BaseCallback):
    """Feed every collected observation to the policy's RSNorm layers.

    Runs in ``collect_rollouts`` (``new_obs`` of each vec-env step), so the
    statistics follow the state distribution the policy actually visits, as in
    the paper. The target critic's copies are overwritten from the critic by
    SB3 at each gradient step (``running_`` buffers); updating them here as
    well keeps them identical in between.
    """

    def __init__(self, verbose: int = 0):
        super().__init__(verbose)
        self._norms: list[RSNorm] = []

    def _on_training_start(self) -> None:
        self._norms = [m for m in self.model.policy.modules() if isinstance(m, RSNorm)]
        if self.verbose:
            print(f'[rsnorm] tracking {len(self._norms)} normalisers, '
                  f'count {float(self._norms[0].running_count):.0f}' if self._norms else '[rsnorm] none')

    def _on_step(self) -> bool:
        obs = self.locals.get('new_obs')
        if obs is None or not self._norms:
            return True
        x = th.as_tensor(np.asarray(obs, dtype=np.float32), device=self.model.device)
        for m in self._norms:
            m.update(x)
        return True


def count_parameters(policy) -> dict:
    """{'actor': n, 'critic': n} trainable parameter counts (for the run log)."""
    return {'actor': sum(p.numel() for p in policy.actor.parameters()),
            'critic': sum(p.numel() for p in policy.critic.parameters())}
