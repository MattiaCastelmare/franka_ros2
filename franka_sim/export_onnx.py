"""Export a trained SAC policy (actor) to ONNX for ROS 2 deployment.

Isolates the Actor network and traces it to a self-contained `.onnx`, so the
real-time node (`rl_policy_commander.py`, Step 3) needs only `onnxruntime` — no
torch / stable-baselines3 on the robot. The exported graph maps

    observation (N, obs_dim)  →  action (N, 7) ∈ [−1, 1]      (deterministic)

which the deployment node scales by q̈_max and publishes on /NS_1/qddot_nom.

    cd /ros2_ws/src && PYTHONPATH=/ros2_ws/src \
        python3 -m franka_sim.export_onnx --model franka_sim/models/<exp>/best_model.zip
"""

from __future__ import annotations

import argparse
import os

import numpy as np
import torch as th

from stable_baselines3 import SAC


class OnnxableSAC(th.nn.Module):
    """Wrap the SAC actor to emit the DETERMINISTic (mode) action in [−1, 1]."""

    def __init__(self, policy):
        super().__init__()
        self.actor = policy.actor

    def forward(self, observation: th.Tensor) -> th.Tensor:
        # SAC actor already squashes with tanh → action is in the [−1, 1] box.
        return self.actor(observation, deterministic=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True, help='path to a SAC .zip')
    ap.add_argument('--output', default=None, help='.onnx path (default: alongside model)')
    ap.add_argument('--opset', type=int, default=17)
    args = ap.parse_args()

    out = args.output or os.path.splitext(args.model)[0] + '.onnx'

    # CPU load: the exported graph must be device-agnostic for the robot.
    model = SAC.load(args.model, device='cpu')
    obs_dim = int(np.prod(model.observation_space.shape))
    act_dim = int(np.prod(model.action_space.shape))
    print(f'loaded {args.model}  obs_dim={obs_dim}  act_dim={act_dim}')

    onnxable = OnnxableSAC(model.policy).eval()
    dummy = th.zeros(1, obs_dim, dtype=th.float32)
    export_kwargs = dict(
        input_names=['observation'], output_names=['action'],
        dynamic_axes={'observation': {0: 'batch'}, 'action': {0: 'batch'}},
        opset_version=args.opset,
    )
    try:
        # Legacy TorchScript exporter: honors names/dynamic_axes exactly and
        # needs no onnxscript — a clean static graph for onnxruntime on the robot.
        th.onnx.export(onnxable, dummy, out, dynamo=False, **export_kwargs)
    except TypeError:
        th.onnx.export(onnxable, dummy, out, **export_kwargs)  # older torch
    print(f'exported → {out}')

    # ── Validate: onnxruntime output must match SB3 predict(deterministic) ───
    import onnxruntime as ort
    sess = ort.InferenceSession(out, providers=['CPUExecutionProvider'])
    rng = np.random.default_rng(0)
    max_err = 0.0
    for _ in range(100):
        obs = model.observation_space.sample().astype(np.float32)
        onnx_a = sess.run(None, {'observation': obs[None]})[0][0]
        sb3_a, _ = model.predict(obs, deterministic=True)
        max_err = max(max_err, float(np.max(np.abs(onnx_a - sb3_a))))
    print(f'validation: max |onnx − sb3| over 100 obs = {max_err:.2e}')
    assert max_err < 1e-4, 'ONNX output diverges from SB3 policy!'
    print('ONNX export VALID — action within 1e-4 of the SB3 deterministic policy')


if __name__ == '__main__':
    main()
