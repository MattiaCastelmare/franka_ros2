"""franka_sim — standalone MuJoCo + CBF training module for Safe RL Sim-to-Real.

Independent of ROS 2 (import-light) so it can run on a training box / GPU. The
CBF math in ``franka_sim.envs.cbf_filter`` mirrors the on-robot
``franka_experiments/nodes/cbf_safety_filter.py`` so a policy trained here meets
the identical safety filter on hardware.
"""

__all__ = ['envs']
