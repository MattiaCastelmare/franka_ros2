#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from franka_msgs.msg import MultiDistance
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
        self.multi_distances = []
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
        self.qdot_nom = np.zeros(self.model.nv, dtype=np.float64)

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
            MultiDistance,
            self.topics_vis['multi_distance'],
            self.multi_distance_callback,
            10
        )

        self.create_subscription(
            Float64MultiArray,
            self.topics_ctr['qdot_nom'],
            self.compute_nominal_qdot,
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
    def multi_distance_callback(self, msg):
        distances = []

        for item in msg.distances:   
            entry = {
                'link_name': item.robot_link_name,
                'distance': float(item.distance),
                'closest_point_robot': np.array([
                    item.closest_point_robot.x,
                    item.closest_point_robot.y,
                    item.closest_point_robot.z
                ], dtype=np.float64),
                'direction': np.array([
                    item.direction.x,
                    item.direction.y,
                    item.direction.z
                ], dtype=np.float64),
                'zone': item.zone,
                'confidence': float(item.confidence),
                'valid': bool(item.valid),
            }
            distances.append(entry)

        self.multi_distances = distances
        self._new_distance.set()

    def _solver_loop(self):
        while rclpy.ok():
            triggered = self._new_distance.wait(timeout=0.5)
            self._new_distance.clear()
            if triggered and self.q is not None:
                self.control_loop()

    # === CBF Logic ===
    def compute_nominal_qdot(self, msg):
        if self.qdot_nom is None:
            self.qdot_nom = np.zeros(self.model.nv, dtype=np.float64)
        self.qdot_nom = np.array(msg.data, dtype=np.float64)
    
    def compute_point_jacobian_from_updated_data(self, q, link_name, p_world):
        frame_id = self._resolve_frame_id(link_name)
        if frame_id is None:
            self.get_logger().warn(f"Cannot resolve frame: {link_name}")
            return None

        oMf = self.data.oMf[frame_id]
        t = oMf.translation
        r_world = p_world - t

        J6 = pin.computeFrameJacobian(
            self.model,
            self.data,
            q,
            frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )
        Jv = J6[:3, :]
        Jw = J6[3:, :]

        Jp = Jv - skew(r_world) @ Jw

        if not np.all(np.isfinite(Jp)):
            self.get_logger().error("Jacobian contains non-finite values")
            return None

        return Jp
    
    def build_cbf_constraints(self, q, multi_distances):
        constraints = []

        if not multi_distances:
            return constraints

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        for item in multi_distances:
            if not item['valid']:
                continue

            link_name = item['link_name']
            d = item['distance']
            p_world = item['closest_point_robot']
            n_world = item['direction'].copy()

            if not np.isfinite(d):
                continue
            if p_world is None or n_world is None:
                continue

            norm_n = np.linalg.norm(n_world)
            if norm_n < 1e-8:
                continue
            n_world /= norm_n

            h = float(d) - float(self.params['d_safe'])
            gamma = float(select_gamma(item['zone'], item['confidence']))

            Jp = self.compute_point_jacobian_from_updated_data(q, link_name, p_world)
            if Jp is None:
                continue

            a = (n_world @ Jp).astype(np.float64)
            b = float(-gamma * h)

            if not np.all(np.isfinite(a)) or not np.isfinite(b):
                continue

            constraints.append((a, b, link_name, d))

        return constraints

    def solve_cbf_qp(self, qdot_nom, constraints):
        n = self.model.nv
        rho_slack = float(self.params['rho_slack'])

        P = np.eye(n + 1, dtype=np.float64)
        P[-1, -1] = rho_slack

        q_vec = np.zeros(n + 1, dtype=np.float64)
        q_vec[:n] = -np.asarray(qdot_nom, dtype=np.float64)

        lb = np.zeros(n + 1, dtype=np.float64)
        ub = np.zeros(n + 1, dtype=np.float64)

        lb[:n] = self.qdot_min
        ub[:n] = self.qdot_max
        lb[-1] = 0.0
        ub[-1] = np.inf

        G_rows = []
        h_rows = []

        for a, b, _, _ in constraints:
            row = np.zeros(n + 1, dtype=np.float64)
            row[:n] = -a
            row[-1] = -1.0
            G_rows.append(row)
            h_rows.append(-b)

        G = np.vstack(G_rows).astype(np.float64) if G_rows else None
        h = np.array(h_rows, dtype=np.float64) if h_rows else None

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
            )
        except Exception as e:
            self.get_logger().error(f"QP solver exception: {e}")
            return np.zeros(n, dtype=np.float64)

        if x is None:
            self.get_logger().warn("QP failed, returning zero velocity")
            return np.zeros(n, dtype=np.float64)

        x = np.asarray(x, dtype=np.float64).reshape(-1)

        if not np.all(np.isfinite(x)):
            self.get_logger().error("QP solution contains non-finite values")
            return np.zeros(n, dtype=np.float64)

        qdot_cmd = x[:n]
        self.last_qp_slack = float(x[-1])
        return qdot_cmd

    # === Main Loop ===
    def control_loop(self):
        if self.q is None:
            return

        q = self.q.copy()
        qdot_nom = self.qdot_nom.copy()

        multi_distances = list(self.multi_distances)
        constraints = self.build_cbf_constraints(q, multi_distances)

        if constraints:
            min_d = min(d for _, _, _, d in constraints)
            active_links = sorted(set(link for _, _, link, _ in constraints))
            self.get_logger().info(
                f"🎯 [CBF] active_constraints={len(constraints)} | "
                f"min_d={min_d:.3f} m | links={active_links}"
            )
        else:
            self.get_logger().info("⚪ [CBF] no valid distances — running QP with nominal only")

        qdot_cmd = self.solve_cbf_qp(qdot_nom, constraints)

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