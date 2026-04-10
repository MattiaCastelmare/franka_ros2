import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from franka_msgs.msg import HumanRobotDistance
import pinocchio as pin
import numpy as np
import qpsolvers as qp
from utils.ros_setup import init_pinocchio_only, make_joint_state_callback
from utils.cbf_utils import load_robot_config, skew, select_gamma


class AvoidanceControl(Node):
    def __init__(self):
        super().__init__('cbf_avoidance_controller')
        self.get_logger().info("CBF Avoidance Controller node initialized")

        self.vision_config = load_robot_config('distance')
        self.robot_cfg = self.vision_config['robot']
        self.topics_vis = self.vision_config['topics']

        self.control_config = load_robot_config('control')
        self.topics_ctr = self.control_config['topics']
        self.params = self.control_config['params']

        self.pin_ok, self.model, self.data = init_pinocchio_only(self)
        if not self.pin_ok:
            self.get_logger().error("Pinocchio initialization failed")
            return

        # Robot state
        self.q = None
        self.qdot = None

        # Distance message state
        self.link_name = None
        self.closest_distance = None
        self.closest_point_robot = None
        self.zone = None
        self.confidence = 0.0
        self.direction = None
        self.distance_valid = False

        # Debug / monitoring
        self.last_qp_slack = 0.0

        # Subscribers
        self.create_subscription(
            JointState,
            self.topics_ctr['joint_states_topic'],
            make_joint_state_callback(
                controller=self, 
                joint_names=self.robot_cfg['joint_names']),
            10
        )

        self.create_subscription(
            HumanRobotDistance,
            self.topics_vis['closest_distance'],
            self.distance_callback,
            10
        )

        # Publisher
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            self.topics_ctr['velocity_topic'],
            10
        )

        self.create_timer(0.1, self.control_loop)


    # === Callback ===
    def distance_callback(self, msg: HumanRobotDistance):
        self.link_name = msg.robot_link_name
        self.closest_distance = float(msg.distance)
        self.closest_point_robot = np.array(
            [
                msg.closest_point_robot.x,
                msg.closest_point_robot.y,
                msg.closest_point_robot.z
            ],
            dtype=np.float64
        )
        self.zone = msg.zone
        self.confidence = float(msg.confidence)
        self.direction = np.array(
            [msg.direction.x, msg.direction.y, msg.direction.z],
            dtype=np.float64
        )
        self.distance_valid = bool(msg.valid)


    # === CBF Logic ===
    def compute_nominal_qdot(self, q):
        """
        Placeholder nominal controller.
        For now returns zero velocity.
        Later you can replace this with task-space tracking or posture control.
        """
        return np.zeros(self.model.nv, dtype=np.float64)
    
    def compute_point_jacobian(self, q, link_name, p_world):
        try:
            frame_id = self.model.getFrameId(link_name)
            if frame_id >= len(self.model.frames):
                self.get_logger().warn(f"Invalid frame name: {link_name}")
                return None
        except Exception:
            self.get_logger().warn(f"Invalid frame name: {link_name}")
            return None

        # Kinematics
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        oMf = self.data.oMf[frame_id]
        R = oMf.rotation
        t = oMf.translation

        # Convert point from world to local frame coordinates
        p_local = R.T @ (p_world - t)

        # Re-express offset in world coordinates for LOCAL_WORLD_ALIGNED Jacobian
        r_world = R @ p_local

        # Frame Jacobian
        J6 = pin.computeFrameJacobian(
            self.model,
            self.data,
            q,
            frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )
        Jv = J6[:3, :]
        Jw = J6[3:, :]

        # Point Jacobian
        Jp = Jv - skew(r_world) @ Jw

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

        link_name = self.link_name
        n_world = self.direction.copy()
        p_world = self.closest_point_robot.copy()

        norm_n = np.linalg.norm(n_world)
        if norm_n < 1e-8:
            return False, None, None
        n_world /= norm_n

        h = float(self.closest_distance) - float(self.params['d_safe'])
        gamma = float(select_gamma(self.zone, self.confidence))

        Jp = self.compute_point_jacobian(q, link_name, p_world)
        if Jp is None:
            return False, None, None

        a = (n_world @ Jp).astype(np.float64)
        b = float(-gamma * h)

        return True, a, b

    def solve_cbf_qp(self, q, qdot_nom):
        """
        Solve:
            min_{qdot, delta} 0.5 * ||qdot - qdot_nom||^2 + 0.5 * rho * delta^2

        subject to:
            a @ qdot + delta >= b     (CBF)
            qdot_min <= qdot <= qdot_max
            delta >= 0
        """
        n = self.model.nv

        rho_slack = float(self.params['rho_slack'])

        # Cost: 1/2 x^T P x + q^T x
        # x = [qdot(7), delta]
        P = np.eye(n + 1, dtype=np.float64)
        P[-1, -1] = rho_slack

        q_vec = np.zeros(n + 1, dtype=np.float64)
        q_vec[:n] = -np.asarray(qdot_nom, dtype=np.float64)

        # Bounds
        qdot_min = np.asarray(self.params['qdot_min'], dtype=np.float64)
        qdot_max = np.asarray(self.params['qdot_max'], dtype=np.float64)

        lb = np.zeros(n + 1, dtype=np.float64)
        ub = np.zeros(n + 1, dtype=np.float64)

        lb[:n] = qdot_min
        ub[:n] = qdot_max

        # Slack delta >= 0
        lb[-1] = 0.0
        ub[-1] = np.inf

        # Inequality constraints Gx <= h
        G_rows = []
        h_rows = []

        ok, a, b = self.build_cbf_constraint(q)
        if ok:
            row = np.zeros(n + 1, dtype=np.float64)
            # a @ qdot + delta >= b
            #  -> -a @ qdot - delta <= -b
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
                solver=str(self.params.get('qp_solver', 'osqp')),
                verbose=False
            )
        except Exception as e:
            self.get_logger().error(f"QP solver exception: {e}")
            return np.zeros(n, dtype=np.float64)

        if x is None:
            self.get_logger().warn("QP failed, returning zero velocity")
            return np.zeros(n, dtype=np.float64)

        x = np.asarray(x, dtype=np.float64).reshape(-1)

        qdot_cmd = x[:n]
        self.last_qp_slack = float(x[-1])

        return qdot_cmd
    

    # === Main Loop ===
    def control_loop(self):
        if self.q is None:
            return

        q = self.q.copy()

        # Nominal task command
        qdot_nom = self.compute_nominal_qdot(q)

        # Control distance validity
        if not self.distance_valid or self.closest_distance is None:
            self.get_logger().warn(
                "No valid distance → using nominal control"
            )
            qdot_cmd = qdot_nom
        else:
            qdot_cmd = self.solve_cbf_qp(q, qdot_nom)

        # Publish command
        self.get_logger().debug(
            f"Publishing avoidance velocity: {qdot_cmd}, slack={self.last_qp_slack:.4f}"
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