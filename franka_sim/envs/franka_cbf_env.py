"""Gymnasium environment: FR3 reaching with a moving human-proxy obstacle,
shielded by the SAME acceleration-level HOCBF filter that runs on the robot.

Loop (one env step = one control tick, default 100 Hz)
──────────────────────────────────────────────────────
  1. policy action  a ∈ [−1, 1]⁷   →   q̈_nom = a · q̈_max          (nominal accel)
  2. build CBF geometry from MuJoCo: per control-point surface distance dᵢ to the
     obstacle sphere, obstacle→robot normal n̂ᵢ, point Jacobian Jpᵢ, drift ċᵢ
  3. q̈_safe, info = CBFFilter.filter(q, q̇, q̈_nom, obstacles, ee_*)   (shield)
  4. integrate (state-seeded, drift-free):  q̇_des = q̇ + q̈_safe·dt ,
     q_des = q + q̇_des·dt  →  position-servo ctrl = q_des
  5. mj_step × n_substeps ; advance obstacle + target ; reward ; obs ; done

This mirrors the deployment chain qddot_nom → cbf_safety_filter → qddot_safe:
the policy learns the TASK, the CBF certifies SAFETY, in sim and on hardware
alike. The obstacle never produces a physical MuJoCo contact (contype=0); a
penetration (surface distance < 0) is a safety violation handled by the reward.
"""

from __future__ import annotations

import os
from typing import Optional

import numpy as np
import gymnasium as gym
from gymnasium import spaces

import mujoco

from .cbf_filter import AccelCBFFilter, Obstacle

_HERE = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_SCENE = os.path.join(_HERE, '..', 'assets', 'franka_fr3', 'scene_cbf.xml')
_DEFAULT_CONFIG = os.path.join(_HERE, '..', 'config.yaml')

ARM_JOINTS = [f'fr3_joint{i}' for i in range(1, 8)]
NV = 7


def _load_config(config) -> dict:
    if isinstance(config, dict):
        return config
    import yaml
    path = config or _DEFAULT_CONFIG
    with open(path) as f:
        return yaml.safe_load(f)


class FrankaCBFEnv(gym.Env):
    metadata = {'render_modes': ['human', 'rgb_array'], 'render_fps': 100}

    def __init__(self, config=None, render_mode: Optional[str] = None):
        super().__init__()
        cfg = _load_config(config)
        self.cfg = cfg
        env_c   = cfg.get('env', {})
        task_c  = cfg.get('task', {})
        obs_c   = cfg.get('obstacle', {})
        rew_c   = cfg.get('reward', {})
        cbf_c   = cfg.get('cbf', {})
        lim_c   = cfg.get('joint_limits', {})

        self.render_mode = render_mode

        # ── Rates ─────────────────────────────────────────────────────────────
        self.control_hz  = float(env_c.get('control_rate_hz', 100.0))
        self.dt          = 1.0 / self.control_hz
        self.sim_dt      = float(env_c.get('sim_timestep', 0.002))
        self.max_steps   = int(env_c.get('max_episode_steps', 500))
        # Obstacle-avoidance rows on/off (hard state-limit + workspace box always
        # stay on). False = ablation baseline: shield without obstacle CBF.
        self.cbf_obstacle_enabled = bool(env_c.get('cbf_obstacle_enabled', True))

        # ── MuJoCo model ─────────────────────────────────────────────────────
        scene = env_c.get('scene_xml')
        scene = os.path.join(_HERE, '..', scene) if scene else _DEFAULT_SCENE
        self.model = mujoco.MjModel.from_xml_path(os.path.abspath(scene))
        self.model.opt.timestep = self.sim_dt
        self.data = mujoco.MjData(self.model)
        self.n_substeps = max(1, int(round(self.dt / self.sim_dt)))

        # Ids / addresses.
        self._jid   = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)
                       for j in ARM_JOINTS]
        self._qadr  = np.array([self.model.jnt_qposadr[j] for j in self._jid])
        self._dadr  = np.array([self.model.jnt_dofadr[j] for j in self._jid])
        self._act   = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, j)
                       for j in ARM_JOINTS]
        self._key_home = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, 'home')
        self._ee_site  = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE,
                                           env_c.get('ee_site', 'attachment_site'))
        # Mocap indices for the obstacle + target markers.
        self._obs_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'obstacle')
        self._tgt_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'target')
        self._obs_mocap = int(self.model.body_mocapid[self._obs_body])
        self._tgt_mocap = int(self.model.body_mocapid[self._tgt_body])
        # Obstacle sphere radius (geom size) — used for the surface distance.
        og = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, 'obstacle_geom')
        self.r_obs = float(self.model.geom_size[og, 0])
        # The MJCF is authoritative, but the deployment node (rl_policy_commander)
        # cannot parse it — it reads `obstacle.radius` from this same config to
        # rebuild the observation's obstacle slot. A silent divergence would shift
        # the obstacle position the policy sees on the robot, so check it here.
        cfg_r_obs = obs_c.get('radius')
        if cfg_r_obs is not None and abs(float(cfg_r_obs) - self.r_obs) > 1e-9:
            raise ValueError(
                f'config obstacle.radius={cfg_r_obs} != scene obstacle_geom '
                f'radius={self.r_obs} — sim/deployment observation mismatch')

        # ── Torque actuation (mirror of qddot_to_torque on the robot) ────────
        # The Menagerie MJCF ships POSITION servos. Driving them from a
        # state-seeded reference (q_des = q + (q̇ + q̈·dt)·dt) does NOT integrate:
        # the reference is re-anchored to the measurement every tick, so the
        # commanded lead saturates at q̈·dt² (~6e-4 rad) and the servo settles at
        # q̇ ≈ 0.004 rad/s no matter how large q̈ is — the action had almost no
        # authority and every policy produced the same trajectory. Measured, not
        # theorised; see franka_sim_to_real_implementation_status.md.
        #
        # Fix = do what the real chain does: q̈_safe → τ = M(q)q̈ + C(q,q̇)q̇ (+g,
        # which the FR3 firmware adds on hardware) → joint torque. The actuators
        # are converted in-place to direct-force actuators so the vendored MJCF
        # stays untouched, and τ comes from MuJoCo's own inverse dynamics
        # (mj_inverse), i.e. the exact model the forward pass will integrate.
        # Torque limits: the MJCF puts them on the JOINT (`actuatorfrcrange`,
        # ±87/±12 N·m), not on the actuator. The actuators' own ctrlrange comes
        # from the position class and equals the joint POSITION range — clipping
        # a torque with it silently pins τ to a few N·m (joint4's range ends at
        # −0.15), which is exactly the failure this whole block replaces.
        tau_lo, tau_hi = [], []
        for k, i in enumerate(self._act):
            self.model.actuator_gaintype[i] = mujoco.mjtGain.mjGAIN_FIXED
            self.model.actuator_gainprm[i, :] = 0.0
            self.model.actuator_gainprm[i, 0] = 1.0
            self.model.actuator_biastype[i] = mujoco.mjtBias.mjBIAS_NONE
            self.model.actuator_biasprm[i, :] = 0.0

            fr = self.model.actuator_forcerange[i]
            if fr[1] <= fr[0]:                       # unset on the actuator
                jid = self._jid[k]
                if self.model.jnt_actfrclimited[jid]:
                    fr = self.model.jnt_actfrcrange[jid]
                else:
                    fr = None
            if fr is None:
                self.model.actuator_ctrllimited[i] = 0
                tau_lo.append(-np.inf)
                tau_hi.append(np.inf)
            else:
                self.model.actuator_ctrllimited[i] = 1
                self.model.actuator_ctrlrange[i] = fr
                tau_lo.append(float(fr[0]))
                tau_hi.append(float(fr[1]))
        self._tau_lo = np.array(tau_lo)
        self._tau_hi = np.array(tau_hi)

        # ── Joint limits (from config, mirroring fr3_control.yaml) ────────────
        keys = [f'joint{i}' for i in range(1, 8)]
        self.q_min     = np.array([lim_c[k][0] for k in keys])
        self.q_max     = np.array([lim_c[k][1] for k in keys])
        self.qdot_max  = np.array([lim_c[k][2] for k in keys])
        self.qddot_max = np.array([lim_c[k][3] for k in keys])

        # ── Control points for the CBF (body + link radius) ───────────────────
        default_cps = [
            {'body': 'fr3_link4', 'radius': 0.09},
            {'body': 'fr3_link5', 'radius': 0.09},
            {'body': 'fr3_link6', 'radius': 0.08},
            {'body': 'fr3_link7', 'radius': 0.07},
        ]
        cps = cbf_c.get('control_points', default_cps)
        self._cp_body = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, c['body'])
                         for c in cps]
        self._cp_radius = np.array([float(c['radius']) for c in cps])
        self._cp_name = [c['body'] for c in cps]
        self._n_cp = len(cps)
        self._use_jdot = bool(cbf_c.get('include_jdot', True))

        # ── CBF filter (shared math with the real node) ───────────────────────
        self.cbf = AccelCBFFilter(cbf_c, self.qddot_max, self.qdot_max,
                                  self.q_min, self.q_max, self.dt)

        # ── Task / obstacle / reward params ───────────────────────────────────
        self.target_box_min = np.array(task_c.get('target_box_min', [0.30, -0.35, 0.25]))
        self.target_box_max = np.array(task_c.get('target_box_max', [0.60, 0.35, 0.65]))
        self.target_tol     = float(task_c.get('target_tol', 0.05))
        self.q_init_noise   = float(task_c.get('q_init_noise', 0.05))

        self.obs_mode   = str(obs_c.get('mode', 'sinusoidal'))
        self.obs_box_min = np.array(obs_c.get('box_min', [0.30, -0.40, 0.30]))
        self.obs_box_max = np.array(obs_c.get('box_max', [0.65, 0.40, 0.70]))
        self.obs_amp    = float(obs_c.get('amplitude', 0.30))
        self.obs_speed  = float(obs_c.get('speed', 0.6))          # [Hz-ish]
        self.obs_rw_std = float(obs_c.get('random_walk_std', 0.01))

        self.rw = rew_c  # dict of reward weights, read per-step

        # ── Spaces ────────────────────────────────────────────────────────────
        # action = q̈_nom / q̈_max  ∈ [−1, 1]⁷
        self.action_space = spaces.Box(-1.0, 1.0, shape=(NV,), dtype=np.float32)
        # obs = [q(7), q̇(7), ee_pos(3), target(3), obstacle(3), d_min(1)]
        high = np.concatenate([
            self.q_max, self.qdot_max,
            np.full(3, 2.0), np.full(3, 2.0), np.full(3, 2.0), np.array([5.0]),
        ]).astype(np.float32)
        low = np.concatenate([
            self.q_min, -self.qdot_max,
            np.full(3, -2.0), np.full(3, -2.0), np.full(3, -2.0), np.array([-1.0]),
        ]).astype(np.float32)
        self.observation_space = spaces.Box(low, high, dtype=np.float32)

        # ── Episode state ─────────────────────────────────────────────────────
        self._step = 0
        self._target = np.zeros(3)
        self._obs_base = np.zeros(3)
        self._obs_dir = np.array([0.0, 1.0, 0.0])
        self._obs_phase = 0.0
        self._prev_a = [None] * self._n_cp       # prev point Jacobians (finite-diff ċ)
        self._prev_ee_J = None
        self._qddot_prev = np.zeros(NV)
        self._viewer = None
        self._renderer = None

        self._rng = np.random.default_rng()

    # ── Helpers ───────────────────────────────────────────────────────────────

    @property
    def _q(self):
        return self.data.qpos[self._qadr].copy()

    @property
    def _qdot(self):
        return self.data.qvel[self._dadr].copy()

    def _ee_pos(self):
        return self.data.site_xpos[self._ee_site].copy()

    def _point_jac(self, body_id, point):
        """(3, NV) linear point Jacobian for the arm dofs at `point` on `body`."""
        jacp = np.zeros((3, self.model.nv))
        mujoco.mj_jac(self.model, self.data, jacp, None, point, body_id)
        return jacp[:, self._dadr]

    def _ee_jac(self):
        jacp = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self.data, jacp, None, self._ee_site)
        return jacp[:, self._dadr]

    def _build_obstacles(self, qdot):
        """From MuJoCo geometry → list[Obstacle] + EE workspace-row ingredients."""
        p_obs = self.data.mocap_pos[self._obs_mocap].copy()
        obstacles = []
        d_min = np.inf
        for i, bid in enumerate(self._cp_body):
            p_cp = self.data.xpos[bid].copy()
            diff = p_cp - p_obs
            dist = float(np.linalg.norm(diff))
            if dist < 1e-6:
                continue
            n_hat = diff / dist                              # obstacle → robot
            d = dist - self.r_obs - self._cp_radius[i]       # surface distance
            d_min = min(d_min, d)
            Jp = self._point_jac(bid, p_cp)
            a = n_hat @ Jp                                   # (NV,)
            # ċ = n̂ᵀ(J̇p q̇) via finite difference of the point Jacobian.
            cdot = 0.0
            if self._use_jdot and self._prev_a[i] is not None:
                Jdot = (Jp - self._prev_a[i]) / self.dt
                cdot = float(n_hat @ (Jdot @ qdot))
            self._prev_a[i] = Jp
            obstacles.append(Obstacle(self._cp_name[i], d, a, cdot))
        # EE point + Jacobian for the hard workspace-box rows.
        ee_pos = self._ee_pos()
        ee_Jp = self._ee_jac()
        ee_jd = np.zeros(3)
        if self._use_jdot and self._prev_ee_J is not None:
            ee_jd = ((ee_Jp - self._prev_ee_J) / self.dt) @ qdot
        self._prev_ee_J = ee_Jp
        return obstacles, ee_pos, ee_Jp, ee_jd, (d_min if np.isfinite(d_min) else 99.0)

    def _inverse_dynamics(self, qddot_des):
        """q̈_des → joint torque via MuJoCo inverse dynamics, clipped to limits.

        ``mj_inverse`` returns the generalized force that produces ``qacc`` given
        the current (q, q̇): M(q)q̈ + C(q,q̇)q̇ + g(q). On hardware the first two
        terms come from ``qddot_to_torque`` and g(q) from the FR3 firmware, so
        the sum is the same command the robot ends up executing.

        ``qacc`` is restored afterwards: it is an output of the forward pass and
        leaving a fabricated value in it would corrupt any later read.
        """
        qacc_saved = self.data.qacc.copy()
        self.data.qacc[self._dadr] = qddot_des
        mujoco.mj_inverse(self.model, self.data)
        tau = self.data.qfrc_inverse[self._dadr].copy()
        self.data.qacc[:] = qacc_saved
        return np.clip(tau, self._tau_lo, self._tau_hi)

    def _get_obs(self, d_min):
        return np.concatenate([
            self._q, self._qdot, self._ee_pos(), self._target,
            self.data.mocap_pos[self._obs_mocap].copy(), [d_min],
        ]).astype(np.float32)

    # ── Obstacle / target motion ──────────────────────────────────────────────

    def _advance_obstacle(self):
        if self.obs_mode == 'static':
            p = self._obs_base
        elif self.obs_mode == 'random_walk':
            p = self.data.mocap_pos[self._obs_mocap] + \
                self._rng.normal(0.0, self.obs_rw_std, 3)
        else:  # 'sinusoidal' / 'linear' — oscillate along _obs_dir
            self._obs_phase += 2.0 * np.pi * self.obs_speed * self.dt
            s = np.sin(self._obs_phase)
            if self.obs_mode == 'linear':
                s = 2.0 * np.abs(((self._obs_phase / np.pi) % 2.0) - 1.0) - 1.0  # triangle
            p = self._obs_base + self.obs_amp * s * self._obs_dir
        p = np.clip(p, self.obs_box_min, self.obs_box_max)
        self.data.mocap_pos[self._obs_mocap] = p

    # ── Gym API ────────────────────────────────────────────────────────────────

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self._rng = np.random.default_rng(seed)

        mujoco.mj_resetDataKeyframe(self.model, self.data, self._key_home)
        # Small joint perturbation around home (kept inside limits).
        noise = self._rng.uniform(-self.q_init_noise, self.q_init_noise, NV)
        q0 = np.clip(self._q + noise, self.q_min + 0.02, self.q_max - 0.02)
        self.data.qpos[self._qadr] = q0
        self.data.qvel[self._dadr] = 0.0
        mujoco.mj_forward(self.model, self.data)
        # The `home` keyframe carries a POSITION ctrl vector; with torque
        # actuators that would be a nonsense command. Seed with the q̈ = 0 hold
        # torque (gravity compensation) instead, so the arm starts at rest.
        self.data.ctrl[self._act] = self._inverse_dynamics(np.zeros(NV))
        mujoco.mj_forward(self.model, self.data)

        # Sample task target + obstacle trajectory.
        self._target = self._rng.uniform(self.target_box_min, self.target_box_max)
        self.data.mocap_pos[self._tgt_mocap] = self._target
        self._obs_base = self._rng.uniform(self.obs_box_min, self.obs_box_max)
        d = self._rng.normal(0, 1, 3); n = np.linalg.norm(d)
        self._obs_dir = d / n if n > 1e-6 else np.array([0.0, 1.0, 0.0])
        self._obs_phase = self._rng.uniform(0, 2 * np.pi)
        self.data.mocap_pos[self._obs_mocap] = self._obs_base
        mujoco.mj_forward(self.model, self.data)

        # Reset filter + finite-diff caches.
        self.cbf.reset()
        self._prev_a = [None] * self._n_cp
        self._prev_ee_J = None
        self._qddot_prev = np.zeros(NV)
        self._step = 0

        _, _, _, _, d_min = self._build_obstacles(self._qdot)
        return self._get_obs(d_min), {}

    def step(self, action):
        action = np.clip(np.asarray(action, np.float32), -1.0, 1.0)
        qddot_nom = action * self.qddot_max

        q = self._q
        qdot = self._qdot
        obstacles, ee_pos, ee_Jp, ee_jd, d_min = self._build_obstacles(qdot)
        rows = obstacles if self.cbf_obstacle_enabled else []
        qddot_safe, info = self.cbf.filter(q, qdot, qddot_nom, rows,
                                           ee_pos=ee_pos, ee_Jp=ee_Jp, ee_jd_qd=ee_jd)

        # q̈_safe → torque, exactly like qddot_to_torque + the FR3 firmware.
        # τ is recomputed at every SUBSTEP from the current state while q̈_safe
        # is held — the sim analogue of rt_torque_controller re-evaluating the
        # command at 1 kHz against the latest measured q̇ between two 100 Hz
        # q̈_safe samples. Holding a single 100 Hz feedforward torque instead
        # leaves a zero-order-hold drift the robot does not have.
        for _ in range(self.n_substeps):
            self.data.ctrl[self._act] = self._inverse_dynamics(qddot_safe)
            mujoco.mj_step(self.model, self.data)
        self._advance_obstacle()
        mujoco.mj_forward(self.model, self.data)   # refresh xpos/site for obs/reward

        self._step += 1
        ee = self._ee_pos()
        dist = float(np.linalg.norm(ee - self._target))
        # d_min AFTER stepping (what the state actually reached).
        _, _, _, _, d_min = self._build_obstacles(self._qdot)

        # ── Reward ────────────────────────────────────────────────────────────
        rw = self.rw
        success = dist < self.target_tol
        collision = d_min < float(rw.get('collision_dist', 0.0))
        jerk = float(np.linalg.norm(qddot_safe - self._qddot_prev))
        reward = (
            - float(rw.get('w_dist', 1.0)) * dist
            + float(rw.get('w_success', 5.0)) * float(success)
            - float(rw.get('w_action', 0.001)) * float(action @ action)
            - float(rw.get('w_intervention', 0.02)) * info.intervention
            - float(rw.get('w_slack', 0.5)) * info.slack
            - float(rw.get('w_smooth', 0.001)) * jerk
            - float(rw.get('w_qdot', 0.001)) * float(qdot @ qdot)
        )
        if collision:
            reward -= float(rw.get('collision_penalty', 10.0))
        self._qddot_prev = qddot_safe

        terminated = bool(collision) or (success and bool(rw.get('terminate_on_success', True)))
        truncated = self._step >= self.max_steps

        info_out = {
            'dist': dist, 'd_min': d_min, 'success': bool(success),
            'is_success': bool(success),   # always present (Monitor info_keyword)
            'collision': bool(collision), 'cbf_slack': info.slack,
            'cbf_intervention': info.intervention, 'cbf_n_c': info.n_c,
            'cbf_braking': info.braking,
        }

        obs = self._get_obs(d_min)
        if self.render_mode == 'human':
            self.render()
        return obs, float(reward), terminated, truncated, info_out

    # ── Rendering ──────────────────────────────────────────────────────────────

    def render(self):
        if self.render_mode == 'human':
            if self._viewer is None:
                import mujoco.viewer
                self._viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self._viewer.sync()
            return None
        if self.render_mode == 'rgb_array':
            if self._renderer is None:
                self._renderer = mujoco.Renderer(self.model, 480, 640)
            self._renderer.update_scene(self.data)
            return self._renderer.render()
        return None

    def close(self):
        if self._viewer is not None:
            self._viewer.close(); self._viewer = None
        if self._renderer is not None:
            self._renderer.close(); self._renderer = None


# ── Standalone smoke test ─────────────────────────────────────────────────────

if __name__ == '__main__':
    from gymnasium.utils.env_checker import check_env
    env = FrankaCBFEnv()
    print('obs space :', env.observation_space.shape)
    print('act space :', env.action_space.shape)
    print('n_substeps:', env.n_substeps, ' r_obs:', env.r_obs)
    check_env(env.unwrapped, skip_render_check=True)
    print('check_env: OK')

    obs, _ = env.reset(seed=0)
    ret, dmins = 0.0, []
    for _ in range(500):
        a = env.action_space.sample()
        obs, r, term, trunc, info = env.step(a)
        ret += r; dmins.append(info['d_min'])
        if term or trunc:
            break
    print(f'random rollout: return={ret:.2f} steps={env._step} '
          f'min d_min={min(dmins):.3f} last dist={info["dist"]:.3f} '
          f'collision_ever={any(d < 0 for d in dmins)}')
    env.close()
