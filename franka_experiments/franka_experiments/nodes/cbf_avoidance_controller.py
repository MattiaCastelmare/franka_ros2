#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from franka_msgs.msg import HumanRobotDistance
import threading
import pinocchio as pin
import numpy as np
import qpsolvers as qp
from franka_experiments.utils.ros_setup import init_pinocchio_only, make_joint_state_callback
from franka_experiments.utils.cbf_utils import load_robot_config, skew, select_gamma


class AvoidanceControl(Node):
    def __init__(self):
        super().__init__('cbf_avoidance_controller')
        self.get_logger().info("🚀 CBF Avoidance Controller node initialized")

        self.vision_config = load_robot_config('distance') # distance config also contains vision topics
        self.robot_cfg = self.vision_config['robot']
        self.topics_vis = self.vision_config['topics']

        self.control_config = load_robot_config('control') # control config contains control parameters and topics
        self.topics_ctr = self.control_config['topics']
        self.params = self.control_config['params']
        self.limits = self.control_config['joint_limits']

        # robot model, kinematics, and actual configuration(data)
        self.pin_ok, self.model, self.data = init_pinocchio_only(self) 
        if not self.pin_ok:
            self.get_logger().error("Pinocchio initialization failed")
            return

        # Build frame-name lookup and log available frames for debugging
        self._frame_id_cache: dict[str, int] = {}
        all_frame_names = [f.name for f in self.model.frames]
        self.get_logger().info(
            f"📋 Pinocchio frames ({len(all_frame_names)}): {all_frame_names}"
        )

        # Pre-resolve every link name used by the distance estimator
        for link in self.robot_cfg.get('segment_links', []):
            fid = self.model.getFrameId(link)
            if fid < len(self.model.frames):
                self._frame_id_cache[link] = fid
                self.get_logger().info(f"  ✓ frame '{link}' → id {fid}")
            else:
                # Try common fallbacks (e.g. fr3_link8 → fr3_hand)
                resolved = self._try_resolve_frame(link)
                if resolved is not None:
                    self._frame_id_cache[link] = resolved
                    actual = self.model.frames[resolved].name
                    self.get_logger().warn(
                        f"  ⚠ frame '{link}' not found, "
                        f"using fallback '{actual}' (id {resolved})"
                    )
                else:
                    self.get_logger().error(
                        f"  ✗ frame '{link}' not found and no fallback available!"
                    )

        # Joint limits (position, velocity, acceleration)
        _joint_order = [f'joint{i}' for i in range(1, 8)]
        self.q_min     = np.array([self.limits[j][0] for j in _joint_order], dtype=np.float64)
        self.q_max     = np.array([self.limits[j][1] for j in _joint_order], dtype=np.float64)
        self.qdot_max  = np.array([self.limits[j][2] for j in _joint_order], dtype=np.float64)
        self.qdot_min  = -self.qdot_max
        self.qddot_max = np.array([self.limits[j][3] for j in _joint_order], dtype=np.float64)

        # Robot state
        self.q = None
        self.qdot = None
        self.qdot_prev      = np.zeros(self.model.nv, dtype=np.float64)
        self.qdot_prev_prev = np.zeros(self.model.nv, dtype=np.float64)
        self._last_solve_time: float | None = None
        self._last_dt: float = 0.033
        self.qdddot_max: float = 50.0  # jerk limit [rad/s³]
        self.w_jerk: float = 0.1      # soft jerk penalty weight (relative to tracking)

        # Distance message state
        self.link_name = None # closest robot link involved
        self.closest_distance = None
        self.closest_point_robot = None 
        self.zone = None
        self.confidence = 0.0 
        self.direction = None
        self.distance_valid = False

        # if this variable improve, it means the QP is using the slack variable to relax the CBF constraint
        self.last_qp_slack = 0.0 

        # Subscribers
        self.create_subscription(
            JointState,
            self.topics_ctr['joint_states_topic'],
            make_joint_state_callback(
                controller=self,
                joint_names=self.robot_cfg['joint_names']
            ),
            10
        )

        self.create_subscription(
            HumanRobotDistance,
            self.topics_vis['closest_distance'],
            self.distance_callback,
            10
        )

        # Publisher for joint velocity commands
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            self.topics_ctr['velocity_topic'],
            10
        )

        self._new_distance = threading.Event()
        self._solver_thread = threading.Thread(
            target=self._solver_loop, daemon=True
        )
        self._solver_thread.start()

    # === Frame resolution ===
    def _try_resolve_frame(self, link_name: str):
        """Try common name variations to find a matching Pinocchio frame."""
        # Mapping of known aliases (distance estimator name → possible Pinocchio names)
        aliases = {
            'fr3_link8': ['fr3_hand', 'panda_link8', 'panda_hand'],
        }
        candidates = aliases.get(link_name, [])
        # Also try substring matching as last resort
        for candidate in candidates:
            fid = self.model.getFrameId(candidate)
            if fid < len(self.model.frames):
                return fid
        # Substring fallback: any frame whose name contains the link name
        for i, f in enumerate(self.model.frames):
            if link_name in f.name or f.name in link_name:
                return i
        return None

    def _resolve_frame_id(self, link_name: str):
        """Get frame ID from cache, or try to resolve and cache it."""
        if link_name in self._frame_id_cache:
            return self._frame_id_cache[link_name]
        # Try direct lookup first
        fid = self.model.getFrameId(link_name)
        if fid < len(self.model.frames):
            self._frame_id_cache[link_name] = fid
            return fid
        # Try fallback
        resolved = self._try_resolve_frame(link_name)
        if resolved is not None:
            self._frame_id_cache[link_name] = resolved
            actual = self.model.frames[resolved].name
            self.get_logger().warn(
                f"Resolved '{link_name}' → '{actual}' (id {resolved})"
            )
            return resolved
        return None

    # === Callback ===
    def distance_callback(self, msg: HumanRobotDistance):
        self.link_name = msg.robot_link_name # closest robot link name
        self.closest_distance = float(msg.distance) # closest distance from the cp to the obstacle
        self.closest_point_robot = np.array(
            [
                msg.closest_point_robot.x,
                msg.closest_point_robot.y,
                msg.closest_point_robot.z
            ],
            dtype=np.float64
        ) # closest cp position
        self.zone = msg.zone 
        self.confidence = float(msg.confidence)
        self.direction = np.array(
            [msg.direction.x, msg.direction.y, msg.direction.z],
            dtype=np.float64
        )
        self.distance_valid = bool(msg.valid)

        self._new_distance.set()

    def _solver_loop(self):
        while rclpy.ok():
            triggered = self._new_distance.wait(timeout=0.5)
            self._new_distance.clear()
            if triggered and self.q is not None:
                self.control_loop()

    # === CBF Logic ===
    def compute_nominal_qdot(self, q):
        """
        Placeholder nominal controller, (task-space control or joint-space control can be implemented here).
        """
        return np.zeros(self.model.nv, dtype=np.float64)
    
    def compute_point_jacobian(self, q, link_name, p_world):
        frame_id = self._resolve_frame_id(link_name)
        if frame_id is None:
            self.get_logger().warn(f"Cannot resolve frame: {link_name}")
            return None

        # Kinematics
        pin.forwardKinematics(self.model, self.data, q) # compute forward kinematics to update data
        pin.updateFramePlacements(self.model, self.data) # update frame placements to get the latest oMf for frames

        oMf = self.data.oMf[frame_id] # transformation of the closest link frame in world coordinates
        R = oMf.rotation
        t = oMf.translation

        # r_world is the vector from the closest link frame origin to the closest point, expressed in world coordinates
        r_world = p_world - t # used to compute the point Jacobian from the frame Jacobian

        # Frame Jacobian 
        J6 = pin.computeFrameJacobian(
            self.model,
            self.data,
            q,
            frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        ) 
        Jv = J6[:3, :] # linear velocity part of the frame Jacobian
        Jw = J6[3:, :] # angular velocity part of the frame Jacobian

        # Point Jacobian(r_world translation from the frame origin to the point)
        Jp = Jv - skew(r_world) @ Jw 

        # Check for non-finite values in the Jacobian
        if not np.all(np.isfinite(Jp)):
            self.get_logger().error("Jacobian contains non-finite values")
            return None

        return Jp
    
    def build_cbf_constraint(self, q):
        """
        Build one linear CBF constraint of the form:
            a @ qdot >= b
        """
        if not self.distance_valid:
            return False, None, None

        if self.link_name is None or self.direction is None or self.closest_point_robot is None:
            return False, None, None

        if self.closest_distance is None or self.closest_distance <= 0.0:
            return False, None, None

        link_name = self.link_name # closest link name
        n_world = self.direction.copy() # vector from the obstacle to the robot in world coordinates
        p_world = self.closest_point_robot.copy() # closest point position in world coordinates

        norm_n = np.linalg.norm(n_world)
        if norm_n < 1e-8:
            return False, None, None
        n_world /= norm_n # versor from the obstacle to the robot in world coordinates

        h = float(self.closest_distance) - float(self.params['d_safe']) # barrier function
        gamma = float(select_gamma(self.zone, self.confidence)) # gamma based on the zone

        Jp = self.compute_point_jacobian(q, link_name, p_world) # point jacobian
        if Jp is None:
            return False, None, None

        a = (n_world @ Jp).astype(np.float64) # CBF constraint coefficient for qdot
        b = float(-gamma * h) # CBF constraint constant term

        # Check for non-finite values in a and b
        if not np.all(np.isfinite(a)) or not np.isfinite(b):
            return False, None, None

        return True, a, b

    def solve_cbf_qp(self, q, qdot_nom):
        """
        Solve:
            min_{qdot, delta} 0.5 * ||qdot - qdot_nom||^2 + 0.5 * rho * delta^2

        subject to:
            a @ qdot + delta >= b   (CBF)
            qdot_min <= qdot <= qdot_max
            delta >= 0
        """
        n = self.model.nv
        rho_slack = float(self.params['rho_slack'])

        # ── Timing (must precede cost construction) ───────────────────────────
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_solve_time is not None:
            dt = float(np.clip(now - self._last_solve_time, 0.005, 0.2))
        else:
            dt = 0.033
        self._last_solve_time = now
        self._last_dt = dt

        # ── Kinematic extrapolation (first-order, constant-acceleration) ──────
        # qdot_ref = 2·qdot_{k-1} − qdot_{k-2}: the velocity the robot "should
        # have" if acceleration were held constant.  Both the soft penalty and
        # the hard jerk constraint are centred on this point.
        qdot_ref = 2.0 * self.qdot_prev - self.qdot_prev_prev  # shape (n,)

        # ── Cost: ½ xᵀPx + qᵀx,  x = [qdot(n), δ(1)] ───────────────────────
        #
        # Tracking term:    ½ ||qdot − qdot_nom||²
        # Soft jerk term:   ½ w · ||qdot − qdot_ref||²
        #
        # After expansion, the constant terms vanish and we get:
        #   P_diag = 1 + w       (joints),  ρ (slack)
        #   q[:n]  = −(qdot_nom + w · qdot_ref)
        #
        # The effective unconstrained minimiser is the weighted average
        #   qdot* = (qdot_nom + w·qdot_ref) / (1+w)
        # which blends the nominal command with the smooth kinematic target.
        w = self.w_jerk
        P = np.zeros((n + 1, n + 1), dtype=np.float64)
        np.fill_diagonal(P[:n, :n], 1.0 + w)
        P[n, n] = rho_slack

        q_vec = np.zeros(n + 1, dtype=np.float64)
        q_vec[:n] = -(np.asarray(qdot_nom, dtype=np.float64) + w * qdot_ref)

        # ── Bounds ────────────────────────────────────────────────────────────
        lb = np.zeros(n + 1, dtype=np.float64)
        ub = np.zeros(n + 1, dtype=np.float64)
        lb[:n] = self.qdot_min
        ub[:n] = self.qdot_max
        lb[-1] = 0.0    # slack δ ≥ 0
        ub[-1] = np.inf

        # ── Inequality constraints Gx ≤ h ─────────────────────────────────────
        G_rows = []
        h_rows = []

        # CBF: a @ qdot + δ ≥ b  →  −a @ qdot − δ ≤ −b
        ok, a, b = self.build_cbf_constraint(q)
        if ok:
            row = np.zeros(n + 1, dtype=np.float64)
            row[:n] = -a
            row[-1] = -1.0
            G_rows.append(row)
            h_rows.append(-b)

        # Acceleration constraint (per-joint): |qdot − qdot_prev| ≤ qddot_max·dt
        # (qddot_max = deceleration_limit, franka_description/robots/fr3/joint_limits.yaml)
        accel_limit = self.qddot_max * dt          # shape (n,)
        row_acc_up = np.zeros((n, n + 1), dtype=np.float64)
        row_acc_up[:, :n] = np.eye(n)
        G_rows.extend(row_acc_up.tolist())
        h_rows.extend((self.qdot_prev + accel_limit).tolist())
        row_acc_lo = np.zeros((n, n + 1), dtype=np.float64)
        row_acc_lo[:, :n] = -np.eye(n)
        G_rows.extend(row_acc_lo.tolist())
        h_rows.extend((-self.qdot_prev + accel_limit).tolist())

        # Jerk constraint (hard): |qdot − qdot_ref| ≤ qdddot_max·dt²
        # Equivalent to: |qddot_k − qddot_{k-1}| ≤ qdddot_max·dt
        # At constant acceleration qdot_ref tracks qdot_prev exactly, so the
        # robot can sustain any (within-accel-limit) acceleration freely —
        # only *changes* in acceleration are bounded.
        jerk_limit = self.qdddot_max * dt * dt     # scalar, broadcast over n
        row_jrk_up = np.zeros((n, n + 1), dtype=np.float64)
        row_jrk_up[:, :n] = np.eye(n)
        G_rows.extend(row_jrk_up.tolist())
        h_rows.extend((qdot_ref + jerk_limit).tolist())
        row_jrk_lo = np.zeros((n, n + 1), dtype=np.float64)
        row_jrk_lo[:, :n] = -np.eye(n)
        G_rows.extend(row_jrk_lo.tolist())
        h_rows.extend((-qdot_ref + jerk_limit).tolist())

        # Position limit constraints: q_min <= q + qdot*dt <= q_max
        # Rewritten as two sets of inequality rows (slack column = 0):
        #   qdot <=  (q_max - q) / dt  →  [I | 0] x <=  (q_max - q) / dt
        #   qdot >= -(q - q_min) / dt  →  [-I | 0] x <= -(q_min - q) / dt = (q - q_min) / dt
        q_cur = np.asarray(self.q[:n], dtype=np.float64)
        pos_upper = np.clip((self.q_max - q_cur) / dt, self.qdot_min, self.qdot_max)
        pos_lower = np.clip((self.q_min - q_cur) / dt, self.qdot_min, self.qdot_max)

        row_pos_up = np.zeros((n, n + 1), dtype=np.float64)
        row_pos_up[:, :n] = np.eye(n)
        G_rows.extend(row_pos_up.tolist())
        h_rows.extend(pos_upper.tolist())

        row_pos_lo = np.zeros((n, n + 1), dtype=np.float64)
        row_pos_lo[:, :n] = -np.eye(n)
        G_rows.extend(row_pos_lo.tolist())
        h_rows.extend((-pos_lower).tolist())

        G = np.vstack(G_rows).astype(np.float64) if G_rows else None # G matrix for inequality constraints
        h = np.array(h_rows, dtype=np.float64) if h_rows else None # h vector for inequality constraints

        try:
            x = qp.solve_qp(
                P=P,
                q=q_vec,
                G=G,
                h=h,
                A=None,
                b=None,
                lb=lb,
                ub=ub,
                solver=str(self.params['qp_solver']),
                verbose=False
            ) # solve the QP to get optimal qdot and slack variable
        except Exception as e:
            self.get_logger().error(f"QP solver exception: {e}")
            return np.zeros(n, dtype=np.float64)

        # Check if the solver returned a solution
        if x is None:
            self.get_logger().warn("QP failed, returning zero velocity")
            return np.zeros(n, dtype=np.float64)

        x = np.asarray(x, dtype=np.float64).reshape(-1) # ensure x is a 1D array

        # Check for non-finite values in the solution
        if not np.all(np.isfinite(x)):
            self.get_logger().error("QP solution contains non-finite values")
            return np.zeros(n, dtype=np.float64)

        qdot_cmd = x[:n]
        self.last_qp_slack = float(x[-1])

        self.qdot_prev_prev = self.qdot_prev.copy()
        self.qdot_prev = qdot_cmd.copy()

        return qdot_cmd
    

    # === Main Loop ===
    def control_loop(self):
        if self.q is None:
            return

        q = self.q.copy()

        qdot_nom = self.compute_nominal_qdot(q)

        self.get_logger().info(
            f"🎯 [CBF] d={self.closest_distance:.3f} m | "
            f"link={self.link_name} | zone={self.zone}"
            if self.distance_valid and self.closest_distance is not None
            else "⚪ [CBF] no valid distance — running QP with nominal only"
        )
        qdot_cmd = self.solve_cbf_qp(q, qdot_nom)

        # Publish command
        self.get_logger().info(
            f"⚙️ [CMD] qdot={np.array2string(qdot_cmd, precision=3, suppress_small=True)} | "
            f"slack={self.last_qp_slack:.6f}"
        )

        msg = Float64MultiArray()
        msg.data = [float(v) for v in qdot_cmd]
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceControl()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()