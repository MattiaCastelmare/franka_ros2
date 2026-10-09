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
import time
from typing import Optional

import numpy as np
import gymnasium as gym
from gymnasium import spaces

import mujoco

from .cbf_filter import AccelCBFFilter, CBFInfo, Obstacle, fr3_velocity_envelope
from .constraints import ConstraintMonitor
from .obs_layout import (
    CP_WIDTH, DEFAULT_CONTROL_POINTS, assemble as assemble_obs,
    bounds as obs_bounds, spec_from_config,
)
from .randomization import Randomizer

_HERE = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_SCENE = os.path.join(_HERE, '..', 'assets', 'franka_fr3', 'scene_cbf.xml')
_DEFAULT_CONFIG = os.path.join(_HERE, '..', 'config.yaml')

ARM_JOINTS = [f'fr3_joint{i}' for i in range(1, 8)]
NV = 7


def precision_bonus(rw: dict, dist: float):
    """reward.w_prec · (1 − tanh(dist / prec_scale)): a steep bonus near the target.

    -w_dist·dist has a slope of 1 /m, so a policy parked 6 cm from the target
    (just outside target_tol) loses almost nothing against one at 4 cm; this
    term has slope w_prec/prec_scale there. It depends on dist alone, so
    train.py can relabel an old replay buffer from next_obs. Works on arrays.
    """
    w = float(rw.get('w_prec', 0.0))
    if w == 0.0:
        return 0.0
    return w * (1.0 - np.tanh(dist / float(rw.get('prec_scale', 0.05))))


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
        # Obstacle-avoidance rows on/off (hard state-limit + workspace box stay
        # on while env.shield is true). False = ablation baseline: shield
        # without obstacle CBF. env.shield false removes the whole QP.
        self.cbf_obstacle_enabled = bool(env_c.get('cbf_obstacle_enabled', True))
        # Shield mixing (b4): when set, each episode draws the obstacle rows ON
        # with this probability and overrides cbf_obstacle_enabled. A shield-off
        # fine-tune keeps meeting shielded episodes, where the policy still
        # practises reaching past obstacles it has not yet learned to avoid.
        # The policy is not told which kind of episode it is in. None = off
        # (no extra RNG draw → existing seeds replay bit-identically).
        _p_on = env_c.get('cbf_obstacle_on_prob')
        self.cbf_obstacle_on_prob = None if _p_on is None else float(_p_on)
        # Viewer playback rate: 1.0 = wall-clock real time, 0.5 = half speed for
        # a closer look, 2.0 = fast-forward. Visualisation only.
        self.render_speed = float(env_c.get('render_speed', 1.0))
        self._frame_due = 0.0

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
        self._floor_geom = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, 'floor')
        self._floor_pts = []
        self._base_pts = []
        # Finger collision geoms: the hand control-point sphere does not reach
        # the fingertips, so they get their own floor rows (bounding radius).
        self._finger_geoms = [
            g for g in range(self.model.ngeom)
            if (mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, g) or '')
            .endswith('finger_collision')]
        self._prev_finger_J = {}
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
        # config.yaml is the source of truth; the fallback now lives in
        # obs_layout next to the sim→robot link-name map, because
        # `obs.control_point_geometry` puts one observation slot per entry and
        # the two lists have to be the same list.
        cps = cbf_c.get('control_points') or DEFAULT_CONTROL_POINTS
        self._cp_body = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, c['body'])
                         for c in cps]
        self._cp_radius = np.array([float(c['radius']) for c in cps])
        self._cp_name = [c['body'] for c in cps]
        self._n_cp = len(cps)
        self._use_jdot = bool(cbf_c.get('include_jdot', True))

        # ── CBF filter (shared math with the real node) ───────────────────────
        self.cbf = AccelCBFFilter(cbf_c, self.qddot_max, self.qdot_max,
                                  self.q_min, self.q_max, self.dt)

        # ── Policy alone (env.shield: false) ─────────────────────────────────
        # True (default) = every number measured so far: q̈_nom goes through the
        # QP above, which with cbf_obstacle_enabled false STILL enforces the
        # joint position/velocity/acceleration box, slew, workspace, floor and
        # base keep-out. False = nothing in between: q̈_nom drives the torque
        # chain directly and those limits become measurements (constraints:).
        self.shield = bool(env_c.get('shield', True))
        con_c = cfg.get('constraints') or {}
        self.constraints = (ConstraintMonitor(self, con_c)
                            if bool(con_c.get('enabled', False)) else None)
        #: Largest |τ| − limit seen by _inverse_dynamics this tick (0 = none).
        self._tau_excess = 0.0

        # ── Task / obstacle / reward params ───────────────────────────────────
        self.target_box_min = np.array(task_c.get('target_box_min', [0.30, -0.35, 0.25]))
        self.target_box_max = np.array(task_c.get('target_box_max', [0.60, 0.35, 0.65]))
        self.target_tol     = float(task_c.get('target_tol', 0.05))
        self.q_init_noise   = float(task_c.get('q_init_noise', 0.05))

        self.obs_mode   = str(obs_c.get('mode', 'sinusoidal'))
        # Per-episode mix: this fraction of episodes uses a STATIC obstacle, the
        # rest the configured mode. 0 leaves the RNG stream (and every existing
        # run) unchanged.
        self._obs_mode_cfg = self.obs_mode
        self.static_fraction = float(obs_c.get('static_fraction', 0.0))
        self.obs_box_min = np.array(obs_c.get('box_min', [0.30, -0.40, 0.30]))
        self.obs_box_max = np.array(obs_c.get('box_max', [0.65, 0.40, 0.70]))
        self.obs_amp    = float(obs_c.get('amplitude', 0.30))
        self.obs_speed  = float(obs_c.get('speed', 0.6))          # [Hz-ish]
        self.obs_rw_std = float(obs_c.get('random_walk_std', 0.01))

        # ── Reset-time feasibility (see _sample_episode) ──────────────────────
        # Minimum surface distance the START state must clear. None → d_safe,
        # i.e. the episode begins with h = d − d_safe ≥ 0. That is the
        # precondition the HOCBF's forward-invariance guarantee is stated
        # under; starting outside the safe set certifies nothing.
        _rc = task_c.get('reset_min_clearance')
        self.reset_min_clearance = (float(_rc) if _rc is not None
                                    else float(self.cbf.d_safe))
        self.reset_max_tries = int(task_c.get('reset_max_tries', 100))
        # Fraction of episodes whose obstacle must actually SIT ON the straight
        # EE→target path. 0.0 reproduces the uniform sampling exactly.
        #
        # Measured over the 50-episode benchmark with uniform sampling: the
        # obstacle sweep never came within r_obs + d_safe of the direct path in
        # 25 of 50 episodes, and intersected it in 1. Half the benchmark
        # therefore required NO avoidance at all and scored "can it reach a
        # target", which is not what this environment is for.
        self.blocking_fraction = float(task_c.get('blocking_fraction', 0.0))
        # Targeted curriculum (v12): with probability hard_fraction the target
        # is drawn LOW (z ≤ hard_target_z_max) and the episode must be a
        # blocking one — the geometry where v11 still failed most (static,
        # z < 0.35 m: 41 % lost vs 7-12 % higher up). Unlike raising
        # obstacle.static_fraction it leaves the static/moving mix untouched.
        self.hard_fraction = float(task_c.get('hard_fraction', 0.0))
        self.hard_target_z_max = float(task_c.get('hard_target_z_max', 0.35))
        # [m] how close the sweep must come to the path to count as blocking.
        # None → r_obs + d_safe, i.e. "the barrier would engage on the direct
        # route", which is exactly the condition that forces a detour.
        _bm = task_c.get('blocking_margin')
        self.blocking_margin = (float(_bm) if _bm is not None
                                else self.r_obs + float(self.cbf.d_safe))
        #: Episodes that asked for a blocking obstacle and could not get one.
        self.blocking_misses = 0
        #: Episodes that exhausted reset_max_tries and fell back to the best
        #: draw seen. Non-zero means the boxes are over-constrained — read it
        #: before trusting a safety number from the run.
        self.reset_fallbacks = 0
        #: Phases used to test whether a target is blocked at EVERY sweep phase.
        self._phases = np.linspace(0.0, 2.0 * np.pi, 72, endpoint=False)
        # [m] obstacle-centre → target distance below which the target counts
        # as blocked. None → r_obs + d_safe (the historical rule). That rule
        # ignores the hand control point (radius 0.13 at the hand origin,
        # ~0.10 m behind the TCP): holding the TCP on the target actually needs
        # ~r_obs + d_safe + r_hand − 0.10 ≈ 0.26 m, so targets between 0.23 and
        # 0.26 m were kept and are unreachable without driving h < 0.
        _tc = task_c.get('target_clearance')
        self.target_clearance = (float(_tc) if _tc is not None
                                 else self.r_obs + float(self.cbf.d_safe))
        # False → reject only targets blocked at EVERY phase (transiently
        # blocked ones are kept: they can be touched but never held).
        # True  → reject targets blocked at ANY phase, i.e. every target can be
        # reached AND held for the whole episode.
        self.target_free_always = bool(task_c.get('target_free_always', False))
        # Constrained-IK feasibility of the target (see target_feasibility.py):
        # target_clearance bounds the target POINT, but the barrier acts on the
        # hand/elbow spheres, and 7 of the 400 held-out static episodes had no
        # pose within target_tol with every sphere at d >= d_safe. OFF by
        # default: it changes which episodes a seed draws, so every existing
        # run and benchmark stays bit-identical while it is false.
        self.target_ik_check = bool(task_c.get('target_ik_check', False))
        # Only targets whose obstacle sweep comes within this distance [m] are
        # checked; all 22 shared static failures sat at 0.26-0.29 m. None →
        # target_clearance + 0.06.
        _ig = task_c.get('ik_check_gate')
        self.ik_check_gate = (float(_ig) if _ig is not None
                              else self.target_clearance + 0.06)
        # A target is feasible when some pose puts the TCP within this distance
        # [m]. None → target_tol (the physical limit; 0.5·target_tol also drops
        # the targets that are reachable only in a sliver of the tolerance ball).
        _it = task_c.get('ik_check_tol')
        self.ik_check_tol = float(_it) if _it is not None else self.target_tol
        self.ik_check_starts = int(task_c.get('ik_check_starts', 8))
        # Moving obstacle: at most this many sweep centres inside the gate are
        # checked (closest first, >= 5 cm apart), each one separately.
        self.ik_check_max_points = int(task_c.get('ik_check_max_points', 3))
        self._target_ik = None
        #: Draws rejected by the IK check.
        self.ik_rejects = 0

        self.rw = rew_c  # dict of reward weights, read per-step

        # ── Actuation law (see _servo_torque) ────────────────────────────────
        # OFF by default: the feedforward-only path is the one every number in
        # the docs was measured with, and it stays bit-identical while this is
        # false. Turning it on changes the PLANT — retrain and re-benchmark.
        act_c = cfg.get('actuation', {}) or {}
        self.actuation_feedback = bool(act_c.get('enabled', False))
        self._act_kd = np.asarray(act_c.get(
            'd_gains', [30.0, 30.0, 30.0, 25.0, 10.0, 10.0, 5.0]), float)
        self._act_kp = np.asarray(act_c.get(
            'p_gains', [120.0, 120.0, 120.0, 100.0, 40.0, 40.0, 20.0]), float)
        self._act_e_max = float(act_c.get('e_max', 1.0))
        self._act_p_max = float(act_c.get('p_max', 0.15))
        self._act_qdot_margin = float(act_c.get('qdot_margin', 0.95))
        self._act_fade_band = float(act_c.get('ff_fade_band', 0.25))
        for name, arr in (('d_gains', self._act_kd), ('p_gains', self._act_kp)):
            if arr.shape != (NV,):
                raise ValueError(f'actuation.{name} must have {NV} entries, '
                                 f'got {arr.shape[0]}')
        self._qdot_des = np.zeros(NV)
        self._q_des = np.zeros(NV)

        # ── Domain randomisation / observation noise (OFF by default) ────────
        # Applied to the OBSERVATION only. The CBF rows keep MuJoCo's true
        # geometry: the robot's estimate is noisy, but its filter still treats
        # what it receives as truth, so corrupting the shield's own inputs would
        # model a different and worse safety filter than the one deployed.
        self.randomizer = Randomizer(cfg.get('randomization'), self.dt,
                                     np.random.default_rng())

        # ── Spaces ────────────────────────────────────────────────────────────
        # action = q̈_nom / q̈_max  ∈ [−1, 1]⁷
        self.action_space = spaces.Box(-1.0, 1.0, shape=(NV,), dtype=np.float32)
        # obs layout — see envs/obs_layout.py. A config with no `obs:` block
        # yields the legacy 24-dim vector, so every model frozen before the
        # optional blocks existed keeps loading and deploying unchanged.
        self.obs_spec = spec_from_config(cfg)
        if self.obs_spec.n_cp and self.obs_spec.n_cp != len(cps):
            raise ValueError(
                f'obs.control_point_geometry declares {self.obs_spec.n_cp} '
                f'control points but cbf.control_points has {len(cps)}')
        low, high = obs_bounds(self.obs_spec, self.q_min, self.q_max,
                               self.qdot_max)
        self.observation_space = spaces.Box(low, high, dtype=np.float32)

        # ── Episode state ─────────────────────────────────────────────────────
        self._step = 0
        self._target = np.zeros(3)
        self._obs_base = np.zeros(3)
        self._obs_dir = np.array([0.0, 1.0, 0.0])
        self._obs_phase = 0.0
        self._prev_a = [None] * self._n_cp       # prev point Jacobians (finite-diff ċ)
        #: Previous OBSERVED obstacle centre, for the finite-difference
        #: velocity. Deliberately the post-randomizer value: latency, noise and
        #: the LPF then propagate into v_obs exactly as they do on the robot,
        #: which differentiates the same estimate it receives. None = first
        #: tick of an episode, where the velocity reads zero rather than a step
        #: from wherever the previous episode left the sphere.
        self._p_obs_prev = None
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

    def _floor_contact(self):
        """True when any robot geom touches the floor plane this step."""
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            if self._floor_geom in (c.geom1, c.geom2):
                return True
        return False

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
        """MuJoCo geometry → CBF rows, EE workspace ingredients, obs geometry.

        ``cp_geom`` is ``(n_cp, 4)`` — ``[dᵢ, n̂ᵢ]`` per control point, in
        ``cbf.control_points`` order — and is what the observation's optional
        control-point block carries. It is the SAME dᵢ and n̂ᵢ the CBF rows are
        built from, computed once: the policy and the filter that corrects it
        then reason over one geometric picture instead of the filter seeing six
        links and the policy seeing a single scalar.
        """
        p_obs = self.data.mocap_pos[self._obs_mocap].copy()
        obstacles = []
        d_min = np.inf
        # Rows are dropped when the direction is degenerate, but the geometry
        # block is positional, so every control point ALWAYS gets a row here.
        cp_geom = np.zeros((self._n_cp, CP_WIDTH))
        # (z, r, Jz, J̇z q̇) per control point for the shield's floor rows,
        # and (p, r, Jp, J̇p q̇) for the base keep-out rows.
        self._floor_pts = []
        self._base_pts = []
        cp_pts = {}
        for i, bid in enumerate(self._cp_body):
            p_cp = self.data.xpos[bid].copy()
            diff = p_cp - p_obs
            dist = float(np.linalg.norm(diff))
            if dist < 1e-6:
                # Control point ON the obstacle centre: the distance is real
                # and maximally negative, the direction is undefined. Report
                # both honestly — a "far away" placeholder here would hide the
                # worst state the policy can be in.
                cp_geom[i, 0] = -(self.r_obs + self._cp_radius[i])
                continue
            n_hat = diff / dist                              # obstacle → robot
            d = dist - self.r_obs - self._cp_radius[i]       # surface distance
            d_min = min(d_min, d)
            cp_geom[i, 0] = d
            cp_geom[i, 1:] = n_hat
            Jp = self._point_jac(bid, p_cp)
            a = n_hat @ Jp                                   # (NV,)
            # ċ = n̂ᵀ(J̇p q̇) via finite difference of the point Jacobian.
            jdq = np.zeros(3)
            if self._use_jdot and self._prev_a[i] is not None:
                jdq = ((Jp - self._prev_a[i]) / self.dt) @ qdot
            cdot = float(n_hat @ jdq)
            self._prev_a[i] = Jp
            self._floor_pts.append((p_cp[2], self._cp_radius[i], Jp[2], jdq[2]))
            cp_pts[self._cp_name[i]] = (p_cp, self._cp_radius[i], Jp, jdq)
            obstacles.append(Obstacle(self._cp_name[i], d, a, cdot))
        for name in ('fr3_link4', 'fr3_link5', 'fr3_link6', 'fr3_link7', 'fr3_hand'):
            if name in cp_pts:
                self._base_pts.append(cp_pts[name])
        # Forearm midpoint: link5's collision mesh spans link4 → link5 origins,
        # and it is the forearm that hit the base.
        if 'fr3_link4' in cp_pts and 'fr3_link5' in cp_pts:
            a, b = cp_pts['fr3_link4'], cp_pts['fr3_link5']
            self._base_pts.append((0.5 * (a[0] + b[0]), 0.08,
                                   0.5 * (a[2] + b[2]), 0.5 * (a[3] + b[3])))
        for g in self._finger_geoms:
            p_g = self.data.geom_xpos[g].copy()
            Jg = self._point_jac(int(self.model.geom_bodyid[g]), p_g)
            jdq_z = 0.0
            prev = self._prev_finger_J.get(g)
            if self._use_jdot and prev is not None:
                jdq_z = float(((Jg[2] - prev) / self.dt) @ qdot)
            self._prev_finger_J[g] = Jg[2].copy()
            self._floor_pts.append((p_g[2], float(self.model.geom_rbound[g]), Jg[2], jdq_z))
        # EE point + Jacobian for the hard workspace-box rows.
        ee_pos = self._ee_pos()
        ee_Jp = self._ee_jac()
        ee_jd = np.zeros(3)
        if self._use_jdot and self._prev_ee_J is not None:
            ee_jd = ((ee_Jp - self._prev_ee_J) / self.dt) @ qdot
        self._prev_ee_J = ee_Jp
        return (obstacles, ee_pos, ee_Jp, ee_jd,
                (d_min if np.isfinite(d_min) else 99.0), cp_geom)

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
        self._tau_excess = max(self._tau_excess, float(np.max(np.maximum(
            tau - self._tau_hi, self._tau_lo - tau))))
        return np.clip(tau, self._tau_lo, self._tau_hi)

    def _gravity(self):
        """g(q) alone, as the FR3 firmware compensates it.

        ``mj_inverse`` returns M q̈ + C q̇ + g, so the FEEDFORWARD half of the
        robot's law — the part `ffScale` is allowed to fade — is that minus
        g(q).  Evaluated at q̈ = q̇ = 0, `qfrc_inverse` is exactly g(q).

        Keeping the two apart matters: `ffScale` may only scale the term that
        ACCELERATES the joint.  Scaling gravity would make the arm sag, or fall,
        precisely when the fade engages — a torque gate that can drop the load
        is not a safety feature.
        """
        qacc_saved = self.data.qacc.copy()
        qvel_saved = self.data.qvel.copy()
        self.data.qacc[:] = 0.0
        self.data.qvel[:] = 0.0
        mujoco.mj_inverse(self.model, self.data)
        g = self.data.qfrc_inverse[self._dadr].copy()
        self.data.qacc[:] = qacc_saved
        self.data.qvel[:] = qvel_saved
        return g

    # ── rt_torque_controller's actual law (optional, default OFF) ────────────

    def _servo_reset(self):
        """Clear the 1 kHz servo's integrated references at an episode start."""
        self._qdot_des = self._qdot.copy()
        self._q_des = self._q.copy()

    def _ff_scale(self, tau_ff, q, qdot):
        """Directional smoothstep fade of the feedforward near the envelope.

        Mirrors ``RtTorqueController::ffScale``: the margin is measured in the
        direction τ_ff PUSHES, so a τ_ff that decelerates always passes at full
        strength.  The gate can only reduce |τ| toward zero, never add torque,
        so it cannot itself cause a violation.
        """
        if self._act_fade_band <= 0.0 or self._act_qdot_margin <= 0.0:
            return np.ones(NV)
        up, lo = fr3_velocity_envelope(q, margin=self._act_qdot_margin)
        margin = np.where(tau_ff > 0.0, up - qdot, qdot - lo)
        x = np.clip(margin / self._act_fade_band, 0.0, 1.0)
        s = x * x * (3.0 - 2.0 * x)                       # smoothstep
        return np.where(tau_ff == 0.0, 1.0, s)

    def _servo_torque(self, qddot_safe, dt):
        """τ = ffScale·τ_ff + Kp·p + Kd·e + g(q), the robot's 1 kHz law.

        Ported term for term from ``rt_torque_controller.cpp::update()``:

            q̇_des += q̈_safe·dt
            q̇_des  = clamp(q̇_des, qdotFloor(q), qdotCeiling(q))   # envelope
            e       = clamp(q̇_des − q̇, ±e_max) ; q̇_des = q̇ + e    # anti-windup
            q_des  += q̇_des·dt
            p       = clamp(q_des − q, ±p_max) ; q_des  = q + p    # anti-windup

        The position reference integrates q̈_SAFE, not the nominal: it corrects
        EXECUTION, and a position term built on the nominal would fight the
        barrier.
        """
        q, qdot = self._q, self._qdot

        self._qdot_des += qddot_safe * dt
        if self._act_qdot_margin > 0.0:
            up, lo = fr3_velocity_envelope(q, margin=self._act_qdot_margin)
            np.clip(self._qdot_des, lo, up, out=self._qdot_des)

        e = np.clip(self._qdot_des - qdot, -self._act_e_max, self._act_e_max)
        self._qdot_des[:] = qdot + e                       # post-clamp reference

        self._q_des += self._qdot_des * dt
        p = np.clip(self._q_des - q, -self._act_p_max, self._act_p_max)
        self._q_des[:] = q + p

        # τ_ff is the part the firmware does NOT add: M q̈ + C q̇, i.e.
        # mj_inverse minus g(q). Only this is faded and only this is what
        # qddot_to_torque publishes on the robot.
        g = self._gravity()
        tau_ff = self._inverse_dynamics(qddot_safe) - g
        tau = (self._ff_scale(tau_ff, q, qdot) * tau_ff
               + self._act_kp * p + self._act_kd * e + g)
        return np.clip(tau, self._tau_lo, self._tau_hi)

    def _get_obs(self, d_min, cp_geom=None):
        """The vector the policy sees — the ONLY place noise is injected.

        Width and slot order come from ``obs_layout.ObsSpec``;
        ``utils/rl_policy.build_observation`` rebuilds the same layout on the
        robot. The randomizer models what the robot's PERCEPTION does to the
        values, never what the layout is.

        MUST be called exactly once per control tick: the randomizer's delay
        buffer and LPF advance on every call, and so does the finite-difference
        velocity below.
        """
        q, qdot = self.randomizer.joint_state(self._q, self._qdot)
        p_obs, d_min = self.randomizer.obstacle(
            self.data.mocap_pos[self._obs_mocap], d_min)

        # v_obs from the OBSERVED centres, so the same delay/noise/LPF the
        # position carries is the noise the velocity inherits — the robot
        # differentiates its estimate, not the truth, and so does this.
        v_obs = None
        if self.obs_spec.obstacle_velocity:
            v_obs = (np.zeros(3) if self._p_obs_prev is None
                     else (np.asarray(p_obs, float) - self._p_obs_prev) / self.dt)
            self._p_obs_prev = np.asarray(p_obs, float).copy()

        return assemble_obs(self.obs_spec, q, qdot, self._ee_pos(),
                            self._target, p_obs, d_min,
                            v_obs=v_obs, cp_geometry=cp_geom)

    # ── Obstacle / target motion ──────────────────────────────────────────────

    def _obstacle_at(self, phase, base=None, direction=None):
        """Obstacle centre at oscillation *phase* — the ONE place that maps
        phase → position, so ``reset`` and ``step`` cannot disagree.

        They used to. ``reset`` parked the sphere at ``_obs_base`` while
        ``_advance_obstacle`` evaluated ``base + amp·sin(phase)`` with a phase
        seeded uniformly in [0, 2π), so the FIRST tick displaced the obstacle
        by up to the full amplitude in one control period: measured mean
        0.113 m, max 0.200 m per 10 ms tick — 11 to 20 m/s, against the
        configured peak of 2π·speed·amplitude = 0.251 m/s.
        No barrier can bound a 20 m/s teleport that lands inside d_safe, and it
        is why lowering ``obstacle.speed`` never helped: the jump is set by
        ``amplitude``, which the config deliberately kept.
        """
        base = self._obs_base if base is None else base
        direction = self._obs_dir if direction is None else direction
        if self.obs_mode in ('static', 'random_walk'):
            p = base
        else:  # 'sinusoidal' / 'linear' — oscillate along `direction`
            s = np.sin(phase)
            if self.obs_mode == 'linear':
                s = 2.0 * np.abs(((phase / np.pi) % 2.0) - 1.0) - 1.0   # triangle
            p = base + self.obs_amp * s * direction
        return np.clip(p, self.obs_box_min, self.obs_box_max)

    def _advance_obstacle(self):
        if self.obs_mode == 'random_walk':
            p = self.data.mocap_pos[self._obs_mocap] + \
                self._rng.normal(0.0, self.obs_rw_std, 3)
            self.data.mocap_pos[self._obs_mocap] = np.clip(
                p, self.obs_box_min, self.obs_box_max)
            return
        if self.obs_mode != 'static':
            self._obs_phase += 2.0 * np.pi * self.obs_speed * self.dt
        self.data.mocap_pos[self._obs_mocap] = self._obstacle_at(self._obs_phase)

    # ── Reset-time feasibility ────────────────────────────────────────────────
    #
    # Both rejection rules exist because the uniform sampler put the obstacle on
    # top of the arm. Measured over 50 episodes before they were added:
    #
    #   * 9/50 episodes started already PENETRATING (d_min < 0);
    #   * the median episode started at d_min = 0.063 m, inside d_safe = 0.15;
    #   * EVERY collision in the whole benchmark happened at step 1.
    #
    # So the collision rate measured the reset distribution, not the
    # controller: the trained policy, a random policy and an arm that never
    # moved all scored an identical 20 % with an identical −0.1467 m worst
    # penetration — the §5 "one trajectory" signature, from a different cause.
    #
    # It also broke the barrier's own premise. Forward invariance is a claim
    # about trajectories that START in the safe set; with h < 0 at t = 0 the
    # HOCBF certifies nothing, and most episodes began there.

    def _start_clearance(self, p_obs):
        """Smallest control-point surface distance to a sphere centred at p_obs.

        Same geometry as :meth:`_build_obstacles`, evaluated against the pose
        already written into ``data`` by :meth:`reset`.
        """
        d = np.inf
        for i, bid in enumerate(self._cp_body):
            gap = float(np.linalg.norm(self.data.xpos[bid] - p_obs))
            d = min(d, gap - self.r_obs - self._cp_radius[i])
        return d

    def _sweep_points(self, base, direction):
        """Obstacle centres over one oscillation — a single point when static.

        ``random_walk`` cannot be predicted, so its start point stands in.
        """
        if self.obs_mode in ('static', 'random_walk'):
            return np.clip(np.asarray(base, float), self.obs_box_min,
                           self.obs_box_max)[None, :]
        return np.array([self._obstacle_at(p, base, direction)
                         for p in self._phases])

    def _target_always_blocked(self, target, base, direction):
        """True when the target is inside ``target_clearance`` at EVERY phase
        (at ANY phase when ``task.target_free_always`` is set).

        Only the ALWAYS case is rejected. Measured over 20 000 draws: 41.6 % of
        targets are TRANSIENTLY blocked (the obstacle passes over them), and
        those are kept on purpose — waiting for an obstacle to clear is the
        behaviour the policy should learn. 4.29 % are blocked at every phase,
        and those cannot be reached without driving h < 0: pure reward noise,
        and a hard ceiling on the success rate.
        """
        dist = np.linalg.norm(self._sweep_points(base, direction) - target, axis=1)
        worst = dist.min() if self.target_free_always else dist.max()
        return bool(worst < self.target_clearance)

    def _target_ik_feasible(self, target, base, direction):
        """True unless constrained IK finds NO pose holding the TCP within
        ik_check_tol of the target with every control point at d >= d_safe.

        Only obstacle centres within ik_check_gate of the target are checked,
        so the common case costs one distance computation.
        """
        pts = self._sweep_points(base, direction)
        dist = np.linalg.norm(pts - target, axis=1)
        near = np.argsort(dist)
        near = near[dist[near] < self.ik_check_gate]
        if near.size == 0:
            return True
        chosen = []
        for i in near:
            if all(np.linalg.norm(pts[i] - pts[j]) >= 0.05 for j in chosen):
                chosen.append(i)
            if len(chosen) >= self.ik_check_max_points:
                break
        if self._target_ik is None:
            from .target_feasibility import TargetIK
            cbf_c = self.cfg.get('cbf', {}) or {}
            floor = ((self.cbf.floor_z, self.cbf.floor_margin)
                     if self.cbf.floor_enable else None)
            self._target_ik = TargetIK(
                self.model, self.data.qpos.copy(), self._qadr, self._dadr, self._ee_site,
                self._cp_body, self._cp_radius, self.r_obs, self.cbf.d_safe,
                self.q_min, self.q_max, q_margin=float(cbf_c.get('position_margin_rad', 0.05)),
                floor=floor, n_starts=self.ik_check_starts)
        err = self._target_ik.min_error(target, pts[chosen], self.data.qpos[self._qadr].copy(),
                                        self.ik_check_tol)
        return err < self.ik_check_tol

    def _path_clearance(self, target, base, direction):
        """Closest approach of the obstacle SWEEP to the straight EE→target line.

        A proxy for "does this obstacle interfere with the task at all". The arm
        does not travel in a straight line and the whole body must clear, not
        just the EE — but if the sweep never comes near the direct route, the
        episode is solvable by ignoring the obstacle, and that is the case this
        measures.
        """
        a = self._ee_pos()
        b = np.asarray(target, float)
        pts = self._sweep_points(base, direction)
        ab = b - a
        L = float(ab @ ab)
        t = np.clip((pts - a) @ ab / max(L, 1e-12), 0.0, 1.0)
        return float(np.linalg.norm(pts - (a + np.outer(t, ab)), axis=1).min())

    def _sample_episode(self):
        """Draw (target, obstacle base, direction, phase) for a solvable episode.

        Bounded: after ``reset_max_tries`` rejected draws it returns the one
        with the largest start clearance and counts a fallback. An unbounded
        loop inside a training reset is a hang, and silently accepting a bad
        start is what this method exists to prevent — so it does neither.
        """
        # Decide up front whether THIS episode must be a blocking one, so the
        # draw is a Bernoulli over episodes rather than a bias inside the loop.
        want_block = (self.blocking_fraction > 0.0
                      and self._rng.random() < self.blocking_fraction)
        tgt_max = self.target_box_max
        if self.hard_fraction > 0.0 and self._rng.random() < self.hard_fraction:
            want_block = True
            tgt_max = self.target_box_max.copy()
            tgt_max[2] = min(tgt_max[2], max(self.target_box_min[2], self.hard_target_z_max))

        best = None
        best_blocking = None
        for _ in range(self.reset_max_tries):
            target = self._rng.uniform(self.target_box_min, tgt_max)
            base   = self._rng.uniform(self.obs_box_min, self.obs_box_max)
            v = self._rng.normal(0, 1, 3)
            n = float(np.linalg.norm(v))
            direction = v / n if n > 1e-6 else np.array([0.0, 1.0, 0.0])
            phase = self._rng.uniform(0, 2 * np.pi)

            # Clearance is tested at the position the obstacle will ACTUALLY
            # occupy at t = 0 — _obstacle_at(phase), not the base.
            clearance = self._start_clearance(
                self._obstacle_at(phase, base, direction))
            if best is None or clearance > best[0]:
                best = (clearance, target, base, direction, phase)
            if clearance < self.reset_min_clearance:
                continue
            if self._target_always_blocked(target, base, direction):
                continue
            if self.target_ik_check and not self._target_ik_feasible(target, base, direction):
                self.ik_rejects += 1
                continue

            if want_block:
                # Keep the most obstructing FEASIBLE draw seen, so a miss still
                # returns the hardest legal episode instead of a uniform one.
                pc = self._path_clearance(target, base, direction)
                if best_blocking is None or pc < best_blocking[0]:
                    best_blocking = (pc, target, base, direction, phase)
                if pc > self.blocking_margin:
                    continue
            return target, base, direction, phase

        if want_block and best_blocking is not None:
            self.blocking_misses += 1
            return best_blocking[1], best_blocking[2], best_blocking[3], best_blocking[4]
        self.reset_fallbacks += 1
        return best[1], best[2], best[3], best[4]

    # ── Gym API ────────────────────────────────────────────────────────────────

    def _obs_proximity(self, d_min):
        """Normalised obstacle-proximity ramp: 0 beyond obs_soft_margin, 1 at contact."""
        margin = float(self.rw.get('obs_soft_margin', 0.10))
        return min(1.0, max(0.0, margin - d_min) / margin)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self._rng = np.random.default_rng(seed)

        # Before mj_resetData: dynamics randomisation mutates MjModel in place,
        # and body_mass/damping must be settled before the first forward pass.
        self.randomizer.reset(self.model, self._rng)

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

        if self.static_fraction > 0.0:
            self.obs_mode = ('static' if self._rng.random() < self.static_fraction
                             else self._obs_mode_cfg)
        if self.cbf_obstacle_on_prob is not None:
            self.cbf_obstacle_enabled = bool(self._rng.random() < self.cbf_obstacle_on_prob)

        # Sample task target + obstacle trajectory, rejecting starts that make
        # the episode unsolvable or that begin outside the safe set.
        (self._target, self._obs_base,
         self._obs_dir, self._obs_phase) = self._sample_episode()
        self.data.mocap_pos[self._tgt_mocap] = self._target
        # At the phase-consistent position, NOT at _obs_base: the first
        # _advance_obstacle must move the sphere by one tick's worth of travel,
        # not by up to a full amplitude. See _obstacle_at.
        self.data.mocap_pos[self._obs_mocap] = self._obstacle_at(self._obs_phase)
        mujoco.mj_forward(self.model, self.data)

        # Reset filter + finite-diff caches.
        self.cbf.reset()
        self._servo_reset()
        self._prev_a = [None] * self._n_cp
        self._prev_ee_J = None
        self._prev_finger_J = {}
        self._qddot_prev = np.zeros(NV)
        self._step = 0
        self._p_obs_prev = None

        *_, d_min, cp_geom = self._build_obstacles(self._qdot)
        self._obs_pen_prev = self._obs_proximity(d_min)
        return self._get_obs(d_min, cp_geom), {}

    def step(self, action):
        action = np.clip(np.asarray(action, np.float32), -1.0, 1.0)
        qddot_nom = action * self.qddot_max

        q = self._q
        qdot = self._qdot
        obstacles, ee_pos, ee_Jp, ee_jd, d_min, _ = self._build_obstacles(qdot)
        if self.shield:
            rows = obstacles if self.cbf_obstacle_enabled else []
            qddot_safe, info = self.cbf.filter(q, qdot, qddot_nom, rows,
                                               ee_pos=ee_pos, ee_Jp=ee_Jp, ee_jd_qd=ee_jd,
                                               floor_pts=self._floor_pts,
                                               base_pts=self._base_pts)
        else:
            # Policy alone: the command IS the nominal, clipped only by the
            # action scale. Nothing here knows about limits or obstacles.
            qddot_safe, info = np.asarray(qddot_nom, float), CBFInfo()
        self._tau_excess = 0.0

        # q̈_safe → torque, exactly like qddot_to_torque + the FR3 firmware.
        # τ is recomputed at every SUBSTEP from the current state while q̈_safe
        # is held — the sim analogue of rt_torque_controller re-evaluating the
        # command at 1 kHz against the latest measured q̇ between two 100 Hz
        # q̈_safe samples. Holding a single 100 Hz feedforward torque instead
        # leaves a zero-order-hold drift the robot does not have.
        sub_dt = self.model.opt.timestep
        for _ in range(self.n_substeps):
            self.data.ctrl[self._act] = (
                self._servo_torque(qddot_safe, sub_dt)
                if self.actuation_feedback
                else self._inverse_dynamics(qddot_safe))
            mujoco.mj_step(self.model, self.data)
        self._advance_obstacle()
        mujoco.mj_forward(self.model, self.data)   # refresh xpos/site for obs/reward

        self._step += 1
        ee = self._ee_pos()
        dist = float(np.linalg.norm(ee - self._target))
        # d_min AFTER stepping (what the state actually reached).
        *_, d_min, cp_geom = self._build_obstacles(self._qdot)

        # Constraint measurements (read-only; see envs/constraints.py).
        con_ex = con_diag = None
        if self.constraints is not None:
            con_ex, con_diag = self.constraints.measure(
                self._q, self._qdot, qddot_safe, self._qddot_prev,
                max(0.0, self._tau_excess), ee)

        # ── Reward ────────────────────────────────────────────────────────────
        rw = self.rw
        success = dist < self.target_tol
        collision = d_min < float(rw.get('collision_dist', 0.0))
        jerk = float(np.linalg.norm(qddot_safe - self._qddot_prev))
        # Anticipatory workspace-wall penalty (see reward.w_ws_margin in
        # config.yaml): a linear ramp that starts well outside the hard CBF
        # box (cbf.ws_margin) so the policy is pushed off the walls before
        # the shield ever has to. 0 when ws_enable is off (sim-only feature).
        ws_pen = 0.0
        if self.cbf.ws_enable:
            wall_dist = min(
                min(ee[k] - self.cbf.ws_min[k], self.cbf.ws_max[k] - ee[k])
                for k in range(3))
            soft_margin = float(rw.get('ws_soft_margin', 0.15))
            ws_pen = max(0.0, soft_margin - wall_dist)
        # Dense obstacle-proximity penalty (reward.w_obs_margin): normalised
        # linear ramp on the closest control-point surface distance, 0 at
        # obs_soft_margin, 1 at contact. With the obstacle CBF rows ON this is
        # redundant with w_intervention/w_slack; with cbf_obstacle_enabled:
        # false it is the ONLY dense avoidance signal — without it the sole
        # obstacle term is the sparse collision_penalty, and an early crash
        # (which ends the episode) out-scores a full 500-step episode.
        #
        # obs_shaping selects HOW the ramp enters the reward:
        #   penalty   — −w·pen(s') every tick (sac_b1). Raises the per-step
        #               living cost, which in sac_b1 brought a full episode
        #               (≈−260 discounted) within ~15 % of an early crash and
        #               re-opened the crash-to-end-the-episode hack.
        #   potential — Ng et al. 1999 shaping F = γΦ(s') − Φ(s), Φ = −w·pen:
        #               the same gradient away from the obstacle, but it
        #               telescopes to ≈0 over an episode, so it adds no living
        #               cost and leaves the optimal policy unchanged. Φ of a
        #               TERMINATED state is 0 (standard episodic convention);
        #               truncation bootstraps, so Φ(s') is used there.
        obs_pen = self._obs_proximity(d_min)
        reward = (
            - float(rw.get('w_dist', 1.0)) * dist
            + float(rw.get('w_success', 5.0)) * float(success)
            - float(rw.get('w_action', 0.001)) * float(action @ action)
            - float(rw.get('w_intervention', 0.02)) * info.intervention
            - float(rw.get('w_slack', 0.5)) * info.slack
            - float(rw.get('w_smooth', 0.001)) * jerk
            - float(rw.get('w_qdot', 0.001)) * float(qdot @ qdot)
            - float(rw.get('w_ws_margin', 0.0)) * ws_pen
            + precision_bonus(rw, dist)
        )
        # terminate_on_collision: false keeps the episode running and charges
        # collision_penalty on EVERY tick spent inside the obstacle.
        terminated = ((collision and bool(rw.get('terminate_on_collision', True)))
                      or (success and bool(rw.get('terminate_on_success', True)))
                      or (con_ex is not None and self.constraints.should_terminate(con_ex)))

        w_obs = float(rw.get('w_obs_margin', 0.0))
        if str(rw.get('obs_shaping', 'penalty')) == 'potential':
            gamma = float(rw.get('shaping_gamma',
                                 self.cfg.get('rl', {}).get('gamma', 0.99)))
            phi_next = 0.0 if terminated else -w_obs * obs_pen
            reward += gamma * phi_next - (-w_obs * self._obs_pen_prev)
        else:
            reward -= w_obs * obs_pen
        self._obs_pen_prev = obs_pen

        if collision:
            reward -= float(rw.get('collision_penalty', 10.0))
            # collision_step_cost > 0: also charge every tick the crash skips,
            # at a rate ≥ the worst per-step cost. Ending the episode early
            # then always costs more than living it out, whatever the other
            # weights — the structural fix for the early-termination hack
            # (an undiscounted sum bounds the discounted one from above).
            if terminated:
                reward -= (float(rw.get('collision_step_cost', 0.0))
                           * (self.max_steps - self._step))
        self._qddot_prev = qddot_safe
        truncated = self._step >= self.max_steps

        info_out = {
            'dist': dist, 'd_min': d_min, 'success': bool(success),
            'is_success': bool(success),   # always present (Monitor info_keyword)
            'collision': bool(collision), 'cbf_slack': info.slack,
            'cbf_intervention': info.intervention, 'cbf_n_c': info.n_c,
            'cbf_braking': info.braking, 'obs_pen': obs_pen,
            'floor_contact': self._floor_contact(),
        }
        if con_ex is not None:
            info_out['constraint_excess'] = con_ex
            info_out.update(con_diag)

        obs = self._get_obs(d_min, cp_geom)
        if self.render_mode == 'human':
            self.render()
        return obs, float(reward), terminated, truncated, info_out

    # ── Rendering ──────────────────────────────────────────────────────────────

    def render(self):
        if self.render_mode == 'human':
            if self._viewer is None:
                import mujoco.viewer
                self._viewer = mujoco.viewer.launch_passive(self.model, self.data)
                self._frame_due = time.perf_counter()
            # Pace the viewer to wall-clock time. Without this the loop runs as
            # fast as the CPU allows — measured 2.4x real time — so everything,
            # the obstacle most visibly, looks far faster than it is and you
            # cannot judge speeds by eye. Training and headless evaluation are
            # unaffected: this branch only runs with render_mode='human'.
            self._frame_due += self.dt / self.render_speed
            lag = self._frame_due - time.perf_counter()
            if lag > 0:
                time.sleep(lag)
            else:
                # Behind schedule (a slow machine, or a paused viewer): drop the
                # debt instead of sprinting to catch up, which would burst.
                self._frame_due = time.perf_counter()
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
