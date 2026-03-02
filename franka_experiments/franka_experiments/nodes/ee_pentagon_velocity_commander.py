#!/usr/bin/env python3
"""End-effector pentagon trajectory tracker via joint-velocity commands.

Publishes ``std_msgs/Float64MultiArray`` (7 joints) to the
``fr3_forward_velocity_controller/commands`` topic using Pinocchio for
Jacobian-based resolved-rate control.

The end-effector follows a **smooth pentagon** trajectory on a configurable
plane (default XY) around a given centre, with minimum-jerk (5th-order) time
profiles *per side* to guarantee C2 continuity at vertices.  A cosine-ramp
warm-up prevents velocity/acceleration discontinuities at startup.

Usage
-----
::

    # default parameters (auto-resolved namespace + joint-state topic):
    ros2 run franka_experiments ee_pentagon_velocity_commander

    # override some parameters:
    ros2 run franka_experiments ee_pentagon_velocity_commander \\
        --ros-args \\
        -p radius:=0.04 \\
        -p cycle_time:=20.0 \\
        -p center_xyz:="[0.4, 0.0, 0.4]" \\
        -p kp_cart:=3.0 \\
        -p ee_frame:=fr3_hand_tcp

    # force a specific joint-state topic (disables auto-detection):
    ros2 run franka_experiments ee_pentagon_velocity_commander \\
        --ros-args -p joint_state_topic:=/my_robot/joint_states

Parameters
----------
command_topic : str
    Velocity-command topic (auto-resolved by default).
joint_state_topic : str
    Joint-state subscription topic.  Default ``"__auto__"`` triggers
    auto-detection: the node reads the namespace from
    ``franka.config.yaml``, builds a priority list of candidates, and
    subscribes to the first one that actually exists on the ROS graph.
    Set to any other value to force that topic directly.
ee_frame : str
    End-effector frame name in the URDF (default ``fr3_hand_tcp``).
rate_hz : float
    Timer frequency [Hz] (default 200).
warmup_s : float
    Seconds of zero-command before starting (default 2.0).
ramp_s : float
    Cosine-ramp duration after warm-up (default 2.0).
center_xyz : list[float]
    Pentagon centre in base frame [m] (default [0.4, 0.0, 0.4]).
radius : float
    Circumscribed-circle radius [m] (default 0.03).
plane : str
    Trajectory plane: ``"xy"``, ``"xz"``, ``"yz"``, or ``"front"``
    (default ``"front"``).  ``"front"`` = YZ plane of ``plane_frame``
    (X held constant); the other values use world-frame axes.
plane_frame : str
    Reference frame for ``plane="front"`` (default ``"fr3_link0"``).
    The pentagon lives in the YZ plane of this frame; X stays constant.
    Ignored when ``plane`` is ``"xy"``/``"xz"``/``"yz"``.
cycle_time : float
    Time for one full pentagon loop [s] (default 15.0).
kp_cart : float
    Cartesian proportional gain (default 2.0).
damping : float
    Damped-least-squares λ (default 0.02).
qdot_max : float
    Per-joint velocity clamp [rad/s] (default 0.3).
lpf_alpha : float
    Low-pass filter coefficient on qdot (default 0.8).

Shutdown
--------
On SIGINT / SIGTERM / Ctrl-C the node publishes zero velocities for ~0.5 s,
then sets ``self.done = True`` so the explicit executor loop in ``main()``
exits cleanly.  No ``rclpy.shutdown()`` inside callbacks — no deadlocks,
no tracebacks, one Ctrl-C is enough.

Assumptions
-----------
* ``pinocchio`` is installed (``import pinocchio``).
* ``franka_description`` package is available and built.
* ``xacro`` CLI is available (used to generate URDF from xacro).
* The FR3 arm joint names follow the ``fr3_joint1 … fr3_joint7`` convention.
* The forward-velocity controller is already loaded and active.
"""

from __future__ import annotations

import math
import os
import subprocess
import tempfile
import time
from typing import List, Optional

import numpy as np
import yaml

import rclpy
import rclpy.executors
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# ---------------------------------------------------------------------------
# Pinocchio
# ---------------------------------------------------------------------------
try:
    import pinocchio as pin
except ImportError as exc:
    raise ImportError(
        'pinocchio is required but not installed. '
        'Install with: pip install pin'
    ) from exc

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
NUM_JOINTS = 7
CONTROLLER_NAME = 'fr3_forward_velocity_controller'
FR3_JOINT_NAMES = [f'fr3_joint{i}' for i in range(1, NUM_JOINTS + 1)]

# Sentinel value: when joint_state_topic equals this, auto-detect is used.
_AUTO = '__auto__'


# ---------------------------------------------------------------------------
# Namespace helper (shared by command-topic and joint-state resolution)
# ---------------------------------------------------------------------------
def _get_namespace_from_config(robot_key: str = 'ROBOT1') -> str:
    """Read namespace from ``franka_bringup/config/franka.config.yaml``.

    Returns the namespace string, or ``''`` if not found / any error.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        bringup_share = get_package_share_directory('franka_bringup')
        config_path = os.path.join(
            bringup_share, 'config', 'franka.config.yaml')
        with open(config_path, 'r') as fh:
            config = yaml.safe_load(fh)
        if config and robot_key in config:
            return str(config[robot_key].get('namespace', '')).strip()
    except Exception:  # noqa: BLE001
        pass
    return ''


# ---------------------------------------------------------------------------
# Topic auto-resolution (command topic)
# ---------------------------------------------------------------------------
def _resolve_default_topic(robot_key: str = 'ROBOT1') -> str:
    """Auto-detect namespace → build velocity-command topic."""
    topic_suffix = f'{CONTROLLER_NAME}/commands'
    ns = _get_namespace_from_config(robot_key)
    if ns:
        return f'/{ns}/{topic_suffix}'
    return f'/{topic_suffix}'


# ---------------------------------------------------------------------------
# URDF loading helpers
# ---------------------------------------------------------------------------
def _generate_urdf_from_xacro() -> str:
    """Generate a plain URDF string by running xacro on the FR3 xacro file.

    Returns the URDF XML as a string.
    """
    from ament_index_python.packages import get_package_share_directory
    desc_share = get_package_share_directory('franka_description')
    xacro_path = os.path.join(desc_share, 'robots', 'fr3', 'fr3.urdf.xacro')
    if not os.path.isfile(xacro_path):
        raise FileNotFoundError(f'FR3 xacro not found at {xacro_path}')

    result = subprocess.run(
        ['xacro', xacro_path, 'hand:=true', 'ee_id:=franka_hand'],
        capture_output=True, text=True, check=True,
    )
    return result.stdout


def _load_pinocchio_model(urdf_xml: str):
    """Build a Pinocchio model+data from a URDF XML string."""
    # Write to a temp file because pin.buildModelFromXML expects plain URDF
    # and some versions handle it differently; file-based is most portable.
    with tempfile.NamedTemporaryFile(
        mode='w', suffix='.urdf', delete=False
    ) as tmp:
        tmp.write(urdf_xml)
        tmp_path = tmp.name

    try:
        model = pin.buildModelFromUrdf(tmp_path)
    finally:
        os.unlink(tmp_path)

    data = model.createData()
    return model, data


# ---------------------------------------------------------------------------
# Pentagon trajectory generator
# ---------------------------------------------------------------------------
class PentagonTrajectory:
    """Smooth periodic pentagon trajectory with minimum-jerk per side.

    Each side of the pentagon is traversed using a 5th-order (minimum-jerk)
    polynomial in normalised time *s ∈ [0,1]*::

        s(τ)  = 10τ³ − 15τ⁴ + 6τ⁵
        ṡ(τ)  = 30τ² − 60τ³ + 30τ⁴

    where τ = (t mod T_side) / T_side.  This guarantees zero velocity and
    acceleration at each vertex → C2-continuous overall loop.

    Parameters
    ----------
    center : np.ndarray, shape (3,)
        Centre of the pentagon in the base frame.
    radius : float
        Circumscribed-circle radius [m].
    plane : str
        ``"xy"``, ``"xz"``, or ``"yz"``.
    cycle_time : float
        Total time for one full loop [s].
    """

    N_SIDES = 5

    def __init__(
        self,
        center: np.ndarray,
        radius: float,
        plane: str,
        cycle_time: float,
    ):
        self.center = np.asarray(center, dtype=float)
        self.radius = radius
        self.plane = plane.lower()
        self.cycle_time = cycle_time
        self.side_time = cycle_time / self.N_SIDES

        # Compute 5 vertices on the chosen plane.
        # Vertex angles start from "top" (π/2) and go counter-clockwise.
        self.vertices: List[np.ndarray] = []
        for k in range(self.N_SIDES):
            angle = 2.0 * math.pi * k / self.N_SIDES + math.pi / 2.0
            u = radius * math.cos(angle)
            v = radius * math.sin(angle)
            pt = self.center.copy()
            if self.plane == 'xy':
                pt[0] += u
                pt[1] += v
            elif self.plane == 'xz':
                pt[0] += u
                pt[2] += v
            elif self.plane in ('yz', 'front'):
                pt[1] += u
                pt[2] += v
            else:
                raise ValueError(
                    f'Unknown plane "{self.plane}", use xy/xz/yz/front')
            self.vertices.append(pt)

    # -- minimum-jerk basis ------------------------------------------------
    @staticmethod
    def _min_jerk(tau: float):
        """Return (s, sdot_normalised) for normalised time tau ∈ [0, 1].

        s     = 10τ³ − 15τ⁴ +  6τ⁵          (position 0→1)
        ds/dτ = 30τ² − 60τ³ + 30τ⁴          (normalised velocity)
        """
        tau = np.clip(tau, 0.0, 1.0)
        t2 = tau * tau
        t3 = t2 * tau
        t4 = t3 * tau
        t5 = t4 * tau
        s = 10.0 * t3 - 15.0 * t4 + 6.0 * t5
        sdot = 30.0 * t2 - 60.0 * t3 + 30.0 * t4
        return s, sdot

    # -- evaluate trajectory -----------------------------------------------
    def evaluate(self, t: float):
        """Return ``(p_d, v_d)`` at time *t* (seconds since trajectory start).

        Parameters
        ----------
        t : float
            Time since the beginning of the trajectory (already offset by
            warmup+ramp).

        Returns
        -------
        p_d : np.ndarray, shape (3,)
            Desired Cartesian position.
        v_d : np.ndarray, shape (3,)
            Desired Cartesian velocity.
        """
        # Wrap t to [0, cycle_time)
        t_mod = t % self.cycle_time
        side_idx = int(t_mod / self.side_time)
        if side_idx >= self.N_SIDES:
            side_idx = self.N_SIDES - 1  # guard against float rounding

        t_in_side = t_mod - side_idx * self.side_time
        tau = t_in_side / self.side_time  # normalised time in [0,1]

        p_start = self.vertices[side_idx]
        p_end = self.vertices[(side_idx + 1) % self.N_SIDES]

        s, sdot_norm = self._min_jerk(tau)

        p_d = p_start + s * (p_end - p_start)
        # v_d = ds/dt * (p_end - p_start) = (sdot_norm / T_side) * delta
        v_d = (sdot_norm / self.side_time) * (p_end - p_start)
        return p_d, v_d


# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------
DEFAULT_TOPIC = _resolve_default_topic()


class EEPentagonVelocityCommander(Node):
    """Resolved-rate pentagon tracker publishing joint velocities."""

    def __init__(self):
        super().__init__('ee_pentagon_velocity_commander')

        # ---- Done flag (checked by main-loop executor) ------------------
        self.done = False

        # ---- Stopping state machine -------------------------------------
        self._stopping = False
        self._stop_end_time = 0.0

        # ---- Declare parameters -----------------------------------------
        self.declare_parameter('command_topic', DEFAULT_TOPIC)
        self.declare_parameter('joint_state_topic', _AUTO)
        self.declare_parameter('ee_frame', 'fr3_hand_tcp')
        self.declare_parameter('rate_hz', 200.0)
        self.declare_parameter('warmup_s', 2.0)
        self.declare_parameter('ramp_s', 2.0)
        self.declare_parameter('center_xyz', [0.4, 0.0, 0.4])
        self.declare_parameter('radius', 0.03)
        self.declare_parameter('plane', 'front')
        self.declare_parameter('plane_frame', 'fr3_link0')
        self.declare_parameter('cycle_time', 15.0)
        self.declare_parameter('kp_cart', 2.0)
        self.declare_parameter('damping', 0.02)
        self.declare_parameter('qdot_max', 0.3)
        self.declare_parameter('lpf_alpha', 0.8)

        # ---- Read parameters --------------------------------------------
        self.cmd_topic: str = self.get_parameter('command_topic').value
        js_topic_param: str = self.get_parameter('joint_state_topic').value
        self.ee_frame_name: str = self.get_parameter('ee_frame').value
        self.rate_hz: float = self.get_parameter('rate_hz').value
        self.warmup_s: float = self.get_parameter('warmup_s').value
        self.ramp_s: float = self.get_parameter('ramp_s').value
        center_xyz: List[float] = list(self.get_parameter('center_xyz').value)
        radius: float = self.get_parameter('radius').value
        plane: str = self.get_parameter('plane').value
        self._plane_frame_name: str = self.get_parameter('plane_frame').value
        cycle_time: float = self.get_parameter('cycle_time').value
        self.kp: float = self.get_parameter('kp_cart').value
        self.damping: float = self.get_parameter('damping').value
        self.qdot_max: float = self.get_parameter('qdot_max').value
        self.lpf_alpha: float = self.get_parameter('lpf_alpha').value

        # ---- Validate ----------------------------------------------------
        if len(center_xyz) != 3:
            self.get_logger().error('center_xyz must have 3 elements')
            raise SystemExit(1)

        # ---- Load Pinocchio model ----------------------------------------
        self.get_logger().info('Generating URDF via xacro …')
        try:
            urdf_xml = _generate_urdf_from_xacro()
        except Exception as exc:
            self.get_logger().error(f'Failed to generate URDF: {exc}')
            raise SystemExit(1) from exc

        self.get_logger().info('Building Pinocchio model …')
        self.pin_model, self.pin_data = _load_pinocchio_model(urdf_xml)

        # Resolve EE frame id
        if not self.pin_model.existFrame(self.ee_frame_name):
            available = [
                self.pin_model.frames[i].name
                for i in range(self.pin_model.nframes)
            ]
            self.get_logger().error(
                f'EE frame "{self.ee_frame_name}" not found in model.\n'
                f'Available frames: {available}')
            raise SystemExit(1)
        self.ee_frame_id = self.pin_model.getFrameId(self.ee_frame_name)
        self.get_logger().info(
            f'EE frame: "{self.ee_frame_name}" (id={self.ee_frame_id})')

        # ---- Resolve plane_frame (for plane="front") --------------------
        self._use_plane_frame = (plane == 'front')
        self._plane_frame_id = -1
        if self._use_plane_frame:
            if not self.pin_model.existFrame(self._plane_frame_name):
                available = [
                    self.pin_model.frames[i].name
                    for i in range(self.pin_model.nframes)
                ]
                self.get_logger().error(
                    f'Plane frame "{self._plane_frame_name}" not found.\n'
                    f'Available frames: {available}')
                raise SystemExit(1)
            self._plane_frame_id = self.pin_model.getFrameId(
                self._plane_frame_name)
            self.get_logger().info(
                f'Plane frame: "{self._plane_frame_name}" '
                f'(id={self._plane_frame_id})\n'
                f'  plane="front" → pentagon in YZ of '
                f'{self._plane_frame_name}, X={center_xyz[0]:.3f} m constant')

        # Map Pinocchio joint indices to FR3 joint order.  The URDF may
        # contain the gripper joints; we only care about the 7 arm joints.
        self._pin_joint_ids: List[int] = []
        for jname in FR3_JOINT_NAMES:
            if not self.pin_model.existJointName(jname):
                self.get_logger().error(
                    f'Joint "{jname}" not in Pinocchio model')
                raise SystemExit(1)
            self._pin_joint_ids.append(
                self.pin_model.getJointId(jname))

        # Map from joint_states message to ordered q array.
        # We'll fill this in the first JointState callback.
        self._js_index_map: Optional[List[int]] = None

        # ---- Joint state storage -----------------------------------------
        self._q: Optional[np.ndarray] = None       # latest 7 joint positions
        self._q_full: Optional[np.ndarray] = None   # full pinocchio q vector
        self._js_stamp = self.get_clock().now()

        # ==================================================================
        # AUTO-DETECT JOINT STATE TOPIC
        # ==================================================================
        self._js_auto_detect = (js_topic_param == _AUTO)
        self._js_sub: Optional[object] = None
        self._js_topic_resolved: Optional[str] = None

        if self._js_auto_detect:
            ns = _get_namespace_from_config()
            self._js_candidates: List[str] = []
            if ns:
                self._js_candidates.append(f'/{ns}/franka/joint_states')
            if ns:
                self._js_candidates.append(f'/{ns}/joint_states')
            self._js_candidates.append('/franka/joint_states')
            self._js_candidates.append('/joint_states')

            self._js_discovery_start = time.monotonic()
            self._js_on_fallback = False
            self._js_discovery_timer = self.create_timer(
                1.0, self._discover_js_topic)
            self.get_logger().info(
                f'Joint-state auto-detect enabled  '
                f'candidates={self._js_candidates}')
        else:
            # User explicitly chose a topic
            self._js_candidates = []
            self._js_topic_resolved = js_topic_param
            self._js_sub = self.create_subscription(
                JointState, js_topic_param, self._joint_state_cb, 10)
            self.get_logger().info(
                f'Joint-state topic (explicit): {js_topic_param}')

        # ---- Pentagon trajectory -----------------------------------------
        self.traj = PentagonTrajectory(
            center=np.array(center_xyz),
            radius=radius,
            plane=plane,
            cycle_time=cycle_time,
        )

        # ---- Publisher + timer -------------------------------------------
        self.pub = self.create_publisher(Float64MultiArray, self.cmd_topic, 10)
        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(period, self._timer_cb)
        self.t0 = self.get_clock().now()

        # ---- Internal state for filter -----------------------------------
        self._qdot_prev = np.zeros(NUM_JOINTS)
        self._last_debug_sec = -1.0

        # ---- Prebuilt zero message ---------------------------------------
        self._zero_msg = Float64MultiArray()
        self._zero_msg.data = [0.0] * NUM_JOINTS

        # ---- Startup log -------------------------------------------------
        topic_note = ('(auto-resolved from franka.config.yaml)'
                      if self.cmd_topic == DEFAULT_TOPIC
                      else '(overridden via parameter)')
        self.get_logger().info(
            f'ee_pentagon_velocity_commander started\n'
            f'  cmd topic   : {self.cmd_topic}  {topic_note}\n'
            f'  js topic    : {self._js_topic_resolved or "(auto-detecting…)"}\n'
            f'  ee frame    : {self.ee_frame_name}\n'
            f'  rate        : {self.rate_hz} Hz\n'
            f'  warmup/ramp : {self.warmup_s} / {self.ramp_s} s\n'
            f'  center      : {center_xyz}\n'
            f'  radius      : {radius} m\n'
            f'  plane       : {plane}\n'
            f'  plane_frame : {self._plane_frame_name if self._use_plane_frame else "(N/A — world-frame axes)"}\n'
            f'  cycle_time  : {cycle_time} s\n'
            f'  Kp          : {self.kp}\n'
            f'  damping     : {self.damping}\n'
            f'  qdot_max    : {self.qdot_max} rad/s\n'
            f'  lpf_alpha   : {self.lpf_alpha}')

    # ==================================================================
    # AUTO-DETECT JOINT STATE TOPIC  — discovery timer (1 Hz)
    # ==================================================================
    def _discover_js_topic(self):
        """Try to find a JointState topic on the ROS graph.

        Called at 1 Hz.  Once we are receiving valid joint data
        (``self._q is not None``) the discovery timer cancels itself.
        """
        # Already receiving data → discovery complete
        if self._q is not None:
            self.get_logger().info(
                f'Receiving joint states on '
                f'{self._js_topic_resolved} — discovery complete')
            self._js_discovery_timer.cancel()
            return

        # Query the ROS graph for all topics of type JointState
        available = self.get_topic_names_and_types()
        js_topics = {
            name
            for name, types in available
            if 'sensor_msgs/msg/JointState' in types
        }

        # 1) Check our priority candidates
        chosen: Optional[str] = None
        for candidate in self._js_candidates:
            if candidate in js_topics:
                chosen = candidate
                break

        # 2) If none matched, try any other JointState topic on the graph
        if chosen is None:
            extras = sorted(js_topics - set(self._js_candidates))
            if extras:
                chosen = extras[0]

        # 3) Subscribe if we found something new
        if chosen is not None and chosen != self._js_topic_resolved:
            self._subscribe_js(chosen, source='auto-detected')
            self._js_on_fallback = False
            # Don't cancel yet — wait until _q is actually populated
            return

        # 4) After 2 s with no subscription at all → fallback
        elapsed = time.monotonic() - self._js_discovery_start
        if elapsed > 2.0 and self._js_sub is None:
            fallback = '/joint_states'
            self._subscribe_js(fallback, source='fallback (2 s timeout)')
            self._js_on_fallback = True
            self.get_logger().warn(
                f'No JointState topic discovered after {elapsed:.1f} s\n'
                f'  candidates : {self._js_candidates}\n'
                f'  graph      : {sorted(js_topics) or "(none)"}\n'
                f'  → falling back to {fallback}, will keep retrying …')

    def _subscribe_js(self, topic: str, *, source: str = ''):
        """(Re-)create the JointState subscription on *topic*."""
        if self._js_sub is not None:
            self.destroy_subscription(self._js_sub)
        self._js_sub = self.create_subscription(
            JointState, topic, self._joint_state_cb, 10)
        self._js_topic_resolved = topic
        self.get_logger().info(
            f'Joint-state topic: {topic}  ({source})\n'
            f'  candidates tried: {self._js_candidates}')

    # ------------------------------------------------------------------
    # Joint state callback
    # ------------------------------------------------------------------
    def _joint_state_cb(self, msg: JointState):
        """Store latest joint positions for the 7 arm joints."""
        # Build index map on first message
        if self._js_index_map is None:
            try:
                self._js_index_map = [
                    msg.name.index(jn) for jn in FR3_JOINT_NAMES
                ]
            except ValueError:
                # Not all arm joints present in this message → skip
                return

        if len(msg.position) < max(self._js_index_map) + 1:
            return

        q7 = np.array([msg.position[i] for i in self._js_index_map])
        self._q = q7

        # Build full Pinocchio q vector (nq may be > 7 due to gripper joints)
        q_full = pin.neutral(self.pin_model)
        for k, pid in enumerate(self._pin_joint_ids):
            # Joint id → configuration index
            idx_q = self.pin_model.joints[pid].idx_q
            q_full[idx_q] = q7[k]
        self._q_full = q_full
        self._js_stamp = self.get_clock().now()

    # ------------------------------------------------------------------
    # Stopping
    # ------------------------------------------------------------------
    def request_stop(self, stop_duration_s: float = 0.5):
        """Enter stopping state (idempotent)."""
        if self._stopping:
            return
        self._stopping = True
        self._stop_end_time = time.monotonic() + stop_duration_s
        self.get_logger().info(
            f'Stopping: publishing zero velocities for {stop_duration_s} s')

    # ------------------------------------------------------------------
    # Timer callback
    # ------------------------------------------------------------------
    def _timer_cb(self):  # noqa: C901 – complexity is inherent
        # ---- CLEAN SHUTDOWN: publish zeros, then signal done -------------
        if self._stopping:
            try:
                self.pub.publish(self._zero_msg)
            except Exception:
                pass
            if time.monotonic() >= self._stop_end_time:
                self.get_logger().info('Stop complete')
                self.timer.cancel()
                # Cancel discovery timer too if still active
                if (self._js_auto_detect
                        and hasattr(self, '_js_discovery_timer')):
                    try:
                        self._js_discovery_timer.cancel()
                    except Exception:
                        pass
                self.done = True      # ← main-loop will exit
            return

        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9

        # ---- Warmup phase: zeros ----
        if t < self.warmup_s:
            self.pub.publish(self._zero_msg)
            self._log_throttle(t, phase='warmup', env=0.0)
            return

        # ---- Cosine-ramp envelope ----
        tr = t - self.warmup_s
        if self.ramp_s > 0.0 and tr < self.ramp_s:
            envelope = 0.5 * (1.0 - math.cos(math.pi * tr / self.ramp_s))
        else:
            envelope = 1.0

        # ---- Need joint state ----
        if self._q is None or self._q_full is None:
            self.pub.publish(self._zero_msg)
            if t - self._last_debug_sec >= 1.0:
                self._last_debug_sec = t
                self.get_logger().warn(
                    'No joint state received yet — publishing zeros')
            return

        # Check staleness (>0.1 s without update → zeros + warn)
        js_age = (self.get_clock().now() - self._js_stamp).nanoseconds * 1e-9
        if js_age > 0.1:
            self.pub.publish(self._zero_msg)
            if t - self._last_debug_sec >= 1.0:
                self._last_debug_sec = t
                self.get_logger().warn(
                    f'Joint-state stale ({js_age:.3f} s) — publishing zeros')
            return

        # ---- Forward kinematics via Pinocchio ----------------------------
        q_full = self._q_full.copy()
        pin.forwardKinematics(self.pin_model, self.pin_data, q_full)
        pin.updateFramePlacement(self.pin_model, self.pin_data,
                                 self.ee_frame_id)
        oMee = self.pin_data.oMf[self.ee_frame_id]

        # ---- Jacobian (LOCAL_WORLD_ALIGNED → expressed in world frame) ---
        J_full = pin.computeFrameJacobian(
            self.pin_model, self.pin_data, q_full, self.ee_frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )  # 6 × nv

        # Extract columns for the 7 arm joints only
        arm_v_ids = [self.pin_model.joints[pid].idx_v
                     for pid in self._pin_joint_ids]
        J_arm = J_full[:, arm_v_ids]   # 6 × 7

        # ---- Plane-frame transform (for plane="front") ------------------
        if self._use_plane_frame:
            pin.updateFramePlacement(self.pin_model, self.pin_data,
                                     self._plane_frame_id)
            oMplane = self.pin_data.oMf[self._plane_frame_id]
            # EE position expressed in plane_frame
            p_ee = np.array(oMplane.actInv(oMee.translation))  # (3,)
            # Translational Jacobian rotated into plane_frame
            R_plane = np.array(oMplane.rotation)               # (3,3)
            J_pos = R_plane.T @ J_arm[:3, :]                   # 3 × 7
        else:
            p_ee = oMee.translation.copy()                     # (3,) world
            J_pos = J_arm[:3, :]                                # 3 × 7

        # ---- Trajectory reference ----------------------------------------
        # tr is time since end of warmup  (p_d, v_d in same frame as p_ee)
        p_d, v_d = self.traj.evaluate(tr)

        # ---- Cartesian control law ---------------------------------------
        e_pos = p_d - p_ee              # position error (3,)
        v_cmd = v_d + self.kp * e_pos   # feed-forward + proportional

        # ---- Damped pseudo-inverse of J_pos (3×7) → qdot (7,) -----------
        lam = self.damping
        # Increase damping when Jacobian is ill-conditioned
        try:
            cond = np.linalg.cond(J_pos)
            if cond > 100.0:
                lam = max(lam, 0.1)
        except np.linalg.LinAlgError:
            lam = 0.1

        # J_pinv = Jᵀ (J Jᵀ + λ² I)⁻¹
        JJt = J_pos @ J_pos.T + (lam ** 2) * np.eye(3)
        try:
            J_pinv = J_pos.T @ np.linalg.inv(JJt)
        except np.linalg.LinAlgError:
            J_pinv = np.linalg.pinv(J_pos)

        qdot_raw = J_pinv @ v_cmd       # (7,)

        # ---- Apply envelope ----------------------------------------------
        qdot_scaled = envelope * qdot_raw

        # ---- Per-joint velocity clamp ------------------------------------
        qdot_clamped = np.clip(qdot_scaled, -self.qdot_max, self.qdot_max)

        # ---- Low-pass filter ---------------------------------------------
        alpha = self.lpf_alpha
        qdot_filt = alpha * self._qdot_prev + (1.0 - alpha) * qdot_clamped
        self._qdot_prev = qdot_filt.copy()

        # ---- Publish -----------------------------------------------------
        msg = Float64MultiArray()
        msg.data = qdot_filt.tolist()
        self.pub.publish(msg)

        # ---- Throttled log (~1 Hz) ---------------------------------------
        self._log_throttle(
            t, phase='ramp' if envelope < 1.0 else 'active',
            env=envelope, p_ee=p_ee, p_d=p_d, e_norm=np.linalg.norm(e_pos),
            qdot_norm=np.linalg.norm(qdot_filt))

    # ------------------------------------------------------------------
    # Throttled logger
    # ------------------------------------------------------------------
    def _log_throttle(
        self, t: float, *, phase: str, env: float,
        p_ee: Optional[np.ndarray] = None,
        p_d: Optional[np.ndarray] = None,
        e_norm: float = 0.0,
        qdot_norm: float = 0.0,
    ):
        if t - self._last_debug_sec < 1.0:
            return
        self._last_debug_sec = t

        def _v2s(v):
            return ', '.join(f'{x:.4f}' for x in v) if v is not None else '?'

        self.get_logger().info(
            f'[t={t:.1f}s {phase} env={env:.3f}] '
            f'p=[{_v2s(p_ee)}] p_d=[{_v2s(p_d)}] '
            f'|e|={e_norm:.4f} |qdot|={qdot_norm:.4f}')


# ======================================================================
# CLEAN SHUTDOWN — explicit executor, no rclpy.shutdown() in callbacks
# ======================================================================
def main(args=None):
    rclpy.init(args=args, signal_handler_options=rclpy.SignalHandlerOptions.NO)
    node = EEPentagonVelocityCommander()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok() and not node.done:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        node.request_stop()
        # Keep spinning so the timer can publish zeros and set node.done
        try:
            while rclpy.ok() and not node.done:
                executor.spin_once(timeout_sec=0.1)
        except KeyboardInterrupt:
            pass                    # 2nd Ctrl-C → exit immediately, no crash
    finally:
        try:
            executor.remove_node(node)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
