"""franka_sim environments."""

from .franka_cbf_env import FrankaCBFEnv
from .cbf_filter import AccelCBFFilter, Obstacle, CBFInfo

__all__ = ['FrankaCBFEnv', 'AccelCBFFilter', 'Obstacle', 'CBFInfo']

# Register with Gymnasium so `gymnasium.make("FrankaCBF-v0")` works.
try:
    from gymnasium.envs.registration import register

    register(
        id='FrankaCBF-v0',
        entry_point='franka_sim.envs.franka_cbf_env:FrankaCBFEnv',
        max_episode_steps=None,   # env handles truncation via max_episode_steps
    )
except Exception:
    pass
