#!/usr/bin/env python3

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

        self.vision_config = load_robot_config('distance') # distance config also contains vision topics
        self.robot_cfg = self.vision_config['robot']
        self.topics_vis = self.vision_config['topics']

        self.control_config = load_robot_config('control') # control config contains control parameters and topics
        self.topics_ctr = self.control_config['topics']
        self.params = self.control_config['params']

        # robot model, kinematics, and actual configuration(data)
        self.pin_ok, self.model, self.data = init_pinocchio_only(self) 
        if not self.pin_ok:
            self.get_logger().error("Pinocchio initialization failed")
            return

        # Robot state
        self.q = None
        self.qdot = None

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
                joint_names=self.robot_cfg['joint_names']),
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

        self.create_timer(0.1, self.control_loop)


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


    # === CBF Logic ===
    def compute_nominal_qdot(self, q):
        """
        Placeholder nominal controller, (task-space control or joint-space control can be implemented here).
        """
        return np.zeros(self.model.nv, dtype=np.float64)
    
    def compute_point_jacobian(self, q, link_name, p_world):
        try:
            frame_id = self.model.getFrameId(link_name) # frame_id of the closest link
            if frame_id >= len(self.model.frames):
                self.get_logger().warn(f"Invalid frame name: {link_name}")
                return None
        except Exception:
            self.get_logger().warn(f"Invalid frame name: {link_name}")
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
        n = self.model.nv # number of joints

        rho_slack = float(self.params['rho_slack']) # slack penalty param

        # Cost: 1/2 x^T P x + q^T x
        # x = [qdot(7), delta]
        P = np.eye(n + 1, dtype=np.float64) # quadratic cost matrix
        P[-1, -1] = rho_slack 

        q_vec = np.zeros(n + 1, dtype=np.float64) # linear cost vector
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
            row = np.zeros(n + 1, dtype=np.float64) # one row for the CBF constraint
            # a @ qdot + delta >= b
            #  -> -a @ qdot - delta <= -b
            row[:n] = -a 
            row[-1] = -1.0
            G_rows.append(row)
            h_rows.append(-b)

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
                solver=str(self.params.get('qp_solver', 'osqp')),
                verbose=True
            ) # solve the QP to get optimal qdot and slack variable
        except Exception as e:
            self.get_logger().error(f"QP solver exception: {e}")
            return np.zeros(n, dtype=np.float64)

        if x is None:
            self.get_logger().warn("QP failed, returning zero velocity")
            return np.zeros(n, dtype=np.float64)

        x = np.asarray(x, dtype=np.float64).reshape(-1) # ensure x is a 1D array

        qdot_cmd = x[:n] # optimal joint velocity command from the QP
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
            qdot_cmd = self.solve_cbf_qp(q, qdot_nom) # solve CBF-QP to get the avoidance velocity

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