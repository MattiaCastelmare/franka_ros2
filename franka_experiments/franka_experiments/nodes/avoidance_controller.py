#!/usr/bin/env python3
"""
HUMAN-ROBOT AVOIDANCE CONTROLLER
=================================
CBF-QP velocity avoidance driven by pre-computed human-robot distance.

This node is the ``franka_experiments`` counterpart of
``franka_simulation/.../online_avoidance_controller.py``.

Instead of reading a ``PlanningScene`` with collision objects, it subscribes
to a ``HumanRobotDistance`` message published by an external human-distance
estimation node and applies the **same** CBF-QP safety filter.

All math utilities (CBF-QP, Jacobian, capsule weighting, POCS projection)
and ROS helpers (publishers, Pinocchio init, RViz markers) are reused from
``franka_simulation/scripts/utils/`` via the bridge module
:mod:`franka_experiments.utils.simulation_imports`.
"""

from typing import List, Optional

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from visualization_msgs.msg import MarkerArray

from franka_msgs.msg import HumanRobotDistance

from franka_experiments.utils.constants import (
    AUTO_SENTINEL,
    FR3_JOINT_NAMES,
    NUM_JOINTS,
)
from franka_experiments.utils.kinematics import (
    generate_urdf_from_xacro,
)

from franka_experiments.utils.simulation_imports import (
    PublishersBundle,
    build_joint_to_joint_capsules,
    build_marker_array,
    build_reduced_pinocchio_model_from_urdf,
    make_joint_state_callback,
)

# ---------------------------------------------------------------------------
# Link-name → capsule-index mapping for the FR3 (8 capsules, 0-based).
# Used to assign ``capsule_idx`` for risk weighting when the human-distance
# message reports which robot link is closest.
# ---------------------------------------------------------------------------
_LINK_TO_CAPSULE_IDX = {
    "fr3_link0": 0,
    "fr3_link1": 1,
    "fr3_link2": 2,
    "fr3_link3": 3,
    "fr3_link4": 4,
    "fr3_link5": 5,
    "fr3_link6": 6,
    "fr3_link7": 7,
    "fr3_link8": 7,  # end-effector ≈ last capsule
}


class HumanAvoidanceController(Node):
    """CBF-QP avoidance controller driven by human-robot distance estimates."""

    def __init__(self):
        super().__init__("human_avoidance_controller")

        self.joint_names = list(FR3_JOINT_NAMES)
        self.n_dof = NUM_JOINTS

        # ── Parameters ────────────────────────────────────────────────
        self._declare_params()
        self._load_params()

        # ── Mutable state ─────────────────────────────────────────────
        self.q: Optional[np.ndarray] = None
        self.human_distance: Optional[HumanRobotDistance] = None
        self.frame_ids: dict = {}
        self.capsules: dict = {}
        self.pin_ok: bool = False
        self.distances_data: List[dict] = []
        self.last_marker_array = MarkerArray()
        self._log_last_wall: float = 0.0

        # ── Pinocchio + capsule geometry ─────────────────────────────
        # Generate URDF from xacro (no robot_state_publisher service
        # needed → immune to namespace issues).
        # Capsule construction logic reused from franka_simulation.
        self._init_pinocchio_and_capsules()

        # Build link_name → Pinocchio frame-ID mapping for fast lookup.
        self._link_frame_ids: dict[str, int] = {}
        if self.pin_ok:
            for i in range(self.model.nframes):
                self._link_frame_ids[self.model.frames[i].name] = i

        # Resolve end-effector frame ID once (used by the simple control law).
        self._ee_frame_id: int = -1
        if self.pin_ok:
            for ee_name in ('fr3_link8', 'fr3_hand', 'fr3_link7'):
                fid = self._link_frame_ids.get(ee_name, -1)
                if fid >= 0:
                    self._ee_frame_id = fid
                    self.get_logger().info(f"EE frame resolved: '{ee_name}' (fid={fid})")
                    break
            if self._ee_frame_id < 0:
                self.get_logger().warn("Could not find EE frame in Pinocchio model")

        # ── Subscriptions ─────────────────────────────────────────────
        self.create_subscription(
            JointState,
            "/NS_1/joint_states",
            make_joint_state_callback(
                controller=self, joint_names=self.joint_names,
            ),
            10,
        )
        self.create_subscription(
            HumanRobotDistance,
            self._human_distance_topic,
            self._human_distance_cb,
            10,
        )

        # ── Publishers (same set as online_avoidance_controller) ──────
        marker_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.capsule_marker_pub = self.create_publisher(
            MarkerArray, "/robot_capsules_markers", marker_qos,
        )

        # ── Primary output: avoidance velocity → RT blender ─────────
        # The RT blender subscribes with SensorDataQoS (BestEffort,
        # KeepLast depth=1).  We MUST publish with BestEffort too,
        # otherwise the blender silently ignores us.
        self._avoidance_output_topic = self._resolve_avoidance_output()
        self.pub = self.create_publisher(
            Float64MultiArray,
            self._avoidance_output_topic,
            qos_profile_sensor_data,
        )
        self.min_dist_pub = self.create_publisher(
            Float64MultiArray, "/avoidance/min_distance", 10)
        self.min_dist_raw_pub = self.create_publisher(
            Float64MultiArray, "/avoidance/min_distance_raw", 10)
        self.closest_constraint_pub = self.create_publisher(
            Float64MultiArray, "/avoidance/closest_constraint", 10)
        self.closest_hazard_pub = self.create_publisher(
            String, "/avoidance/closest_hazard", 10)
        self.constraints_pub = self.create_publisher(
            Float64MultiArray, "/avoidance/constraints", 10)
        self.jac_pub = self.create_publisher(
            Float64MultiArray, "/avoidance/jacobian", 10)
        self.hazard_pub = self.create_publisher(
            String, "/avoidance/hazard", 10)

        self._pubs = PublishersBundle(
            pub=self.pub,
            min_dist_pub=self.min_dist_pub,
            min_dist_raw_pub=self.min_dist_raw_pub,
            closest_constraint_pub=self.closest_constraint_pub,
            closest_hazard_pub=self.closest_hazard_pub,
            constraints_pub=self.constraints_pub,
            jac_pub=self.jac_pub,
            hazard_pub=self.hazard_pub,
            capsule_marker_pub=self.capsule_marker_pub,
        )

        # ── Timers ────────────────────────────────────────────────────
        self.create_timer(1.0 / self._control_rate, self._control_loop)
        self.create_timer(0.1, self._publish_markers_only)

        self.get_logger().info(
            f"🟢 Human Avoidance Controller READY (CBF-QP: "
            f"d_safe={self._d_safe}, k_alpha={self._k_alpha}, "
            f"k_rep={self._k_rep}, max_vel={self._max_joint_vel}, "
            f"output_topic='{self._avoidance_output_topic}')")

    # ================================================================
    # Parameters
    # ================================================================

    def _declare_params(self) -> None:
        self.declare_parameter("control_rate", 100.0)
        self.declare_parameter("human_distance_topic",
                               "human_robot/closest_distance")
        self.declare_parameter("avoidance_output_topic", AUTO_SENTINEL)
        self.declare_parameter("influence_distance", 0.30)
        self.declare_parameter("d_safe", 0.05)
        self.declare_parameter("k_alpha", 1.0)
        self.declare_parameter("k_rep", 0.5)
        self.declare_parameter("max_joint_vel", 0.5)
        self.declare_parameter("min_confidence", 0.3)
        self.declare_parameter("capsule_radii", [0.15, 0.12, 0.13])
        self.declare_parameter("capsule_weight_last2", 2.0)
        self.declare_parameter("capsule_weight_last3", 1.5)
        self.declare_parameter("capsule_weight_default", 1.0)
        self.declare_parameter("debug_capsule_index", -1)
        self.declare_parameter("distance_inflation", 0.0)

    def _load_params(self) -> None:
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self._control_rate = float(p("control_rate"))
        self._human_distance_topic = str(p("human_distance_topic"))
        self._avoidance_output_topic_param = str(p("avoidance_output_topic"))
        self._influence_distance = float(p("influence_distance"))
        self._d_safe = float(p("d_safe"))
        self._k_alpha = float(p("k_alpha"))
        self._k_rep = float(p("k_rep"))
        self._max_joint_vel = float(p("max_joint_vel"))
        self._min_confidence = float(p("min_confidence"))
        self._capsule_radii = [float(x) for x in list(p("capsule_radii"))]
        self._capsule_weight_last2 = float(p("capsule_weight_last2"))
        self._capsule_weight_last3 = float(p("capsule_weight_last3"))
        self._capsule_weight_default = float(p("capsule_weight_default"))
        self._debug_capsule_index = int(p("debug_capsule_index"))
        self._distance_inflation = float(p("distance_inflation"))

    def _resolve_avoidance_output(self) -> str:
        """Resolve the avoidance output topic.

        ``__auto__`` → ``avoidance_qdot`` (relative, inherits node namespace so
        it resolves to ``/<namespace>/avoidance_qdot`` matching the RT blender).
        Any other value is used verbatim.
        """
        raw = self._avoidance_output_topic_param
        if raw == AUTO_SENTINEL:
            topic = '/NS_1/avoidance_qdot'
            self.get_logger().info(
                f"avoidance_output_topic='__auto__' → '{topic}' (relative, inherits namespace)")
            return topic
        if raw.startswith('/') and len(raw.split('/')) > 2:
            self.get_logger().warn(
                f"Using namespaced avoidance topic '{raw}' may break RT controller compatibility")
        return raw

    def _init_pinocchio_and_capsules(self) -> None:
        """Build Pinocchio model + capsules from xacro (no service call).

        Uses ``generate_urdf_from_xacro()`` to obtain the URDF string
        directly, then calls the same ``build_reduced_pinocchio_model_from_urdf``
        and ``build_joint_to_joint_capsules`` functions that
        ``franka_simulation``'s ``init_pinocchio_and_capsules`` uses
        internally.

        This avoids calling ``/robot_state_publisher/get_parameters``
        (which fails under namespace) and matches the approach used by
        ``human_distance_estimator``.
        """
        try:
            urdf_xml = generate_urdf_from_xacro()
            if not urdf_xml:
                self.get_logger().error(
                    "generate_urdf_from_xacro() returned empty URDF")
                return

            self.model, self.data = build_reduced_pinocchio_model_from_urdf(
                urdf_xml)
            self.capsules = build_joint_to_joint_capsules(
                model=self.model,
                capsule_radii=list(self._capsule_radii),
            )
            self.frame_ids = {}  # capsules store their own IDs
            self.pin_ok = True

            self.get_logger().info(
                f"✓ Pinocchio model loaded from xacro — "
                f"{self.model.nq} DoF, "
                f"{len(self.capsules)} joint-to-joint capsules")
        except Exception as e:
            self.get_logger().error(
                f"Failed to init Pinocchio/capsules from xacro: {e}")

    # ================================================================
    # Callbacks
    # ================================================================

    def _human_distance_cb(self, msg: HumanRobotDistance) -> None:
        """Store the latest human-robot distance measurement."""
        self.get_logger().warn("CALLBACK TRIGGERED")
        self.get_logger().warn(f"distance={msg.distance}, valid={msg.valid}")
        self.human_distance = msg

    # ================================================================
    # Candidate construction
    # ================================================================

    def _build_candidate(
        self, msg: HumanRobotDistance,
    ) -> Optional[dict]:
        """Convert a ``HumanRobotDistance`` message into a candidate dict.

        The returned dict has the same structure expected by
        :func:`compute_closest_constraint_jacobian` (``kind='external'``).

        Returns ``None`` when the message should be ignored (invalid flag,
        low confidence, unknown link, …).
        """
        if not msg.valid:
            return None
        if msg.confidence < self._min_confidence:
            return None

        link_name = str(msg.robot_link_name).strip()
        if not link_name:
            return None

        # Resolve Pinocchio frame ID
        fid = self._link_frame_ids.get(link_name, -1)
        if fid < 0:
            # fr3_link8 (EE mount) is not a Pinocchio frame in the FR3 model;
            # fall back to fr3_link7 (wrist), the nearest upstream frame.
            fallback = "fr3_link7"
            fid = self._link_frame_ids.get(fallback, -1)
            if fid < 0:
                self.get_logger().warn(
                    f"Unknown link '{link_name}' in HumanRobotDistance — skipping",
                    throttle_duration_sec=2.0,
                )
                return None
            self.get_logger().warn(
                f"[HUMAN-AVOIDANCE] Unknown link '{link_name}', falling back to '{fallback}' (fid={fid})",
                throttle_duration_sec=2.0,
            )

        d = float(msg.distance)
        if not np.isfinite(d) or d > 900.0:
            return None

        direction = np.array(
            [msg.direction.x, msg.direction.y, msg.direction.z], dtype=float,
        )
        if float(np.linalg.norm(direction)) < 1e-9:
            return None

        closest_pt = np.array(
            [msg.closest_point_robot.x,
             msg.closest_point_robot.y,
             msg.closest_point_robot.z],
            dtype=float,
        )

        capsule_idx = _LINK_TO_CAPSULE_IDX.get(link_name, 4)
        link_idx = capsule_idx  # same mapping for FR3

        self.get_logger().info(
            f"[HUMAN-AVOIDANCE] Using capsule_idx={capsule_idx} for link '{link_name}' (d={msg.distance:.3f})",
            throttle_duration_sec=1.0,
        )

        return {
            "kind": "external",
            "hazard": f"human@{link_name}",
            "d": d,
            "fid": fid,
            "p": closest_pt,
            "n": direction,
            "link": link_name,
            "link_idx": link_idx,
            "capsule_idx": capsule_idx,
            "cap_name": link_name,
        }

    # ================================================================
    # Control loop
    # ================================================================

    def _control_loop(self) -> None:
        # ── CHECK 1: Pinocchio ──────────────────────────────────────────
        if not self.pin_ok:
            self.get_logger().warn("[DBG] pin_ok=False", throttle_duration_sec=2.0)
            msg_out = Float64MultiArray()
            msg_out.data = [0.0] * self.n_dof
            self.pub.publish(msg_out)
            return

        # ── CHECK 2: EE frame ID ────────────────────────────────────────
        self.get_logger().warn(
            f"[DBG] ee_frame_id={self._ee_frame_id}  model.nq={self.model.nq}",
            throttle_duration_sec=5.0,
        )
        if self._ee_frame_id < 0:
            self.get_logger().warn(
                f"[DBG] ABORT: ee_frame_id=-1. Available frames: "
                f"{list(self._link_frame_ids.keys())}",
                throttle_duration_sec=5.0,
            )
            msg_out = Float64MultiArray()
            msg_out.data = [0.0] * self.n_dof
            self.pub.publish(msg_out)
            return

        # ── CHECK 3: joint state ────────────────────────────────────────
        if self.q is None:
            self.get_logger().warn("[DBG] q is None → substituting zeros")
            self.q = np.zeros(self.n_dof)

        self.get_logger().warn(
            f"[DBG] q={np.round(self.q, 3).tolist()}  ||q||={np.linalg.norm(self.q):.4f}",
            throttle_duration_sec=2.0,
        )

        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)

        # ── CHECK 4: human distance message ────────────────────────────
        msg = self.human_distance
        qdot_cmd = np.zeros(self.n_dof, dtype=float)

        if msg is None:
            self.get_logger().warn("[DBG] human_distance is None → zero output",
                                   throttle_duration_sec=2.0)
        elif not msg.valid:
            self.get_logger().warn("[DBG] msg.valid=False → zero output",
                                   throttle_duration_sec=2.0)
        else:
            # ── CHECK 5: direction ──────────────────────────────────────
            direction = np.array(
                [msg.direction.x, msg.direction.y, msg.direction.z], dtype=float)
            norm = float(np.linalg.norm(direction))
            self.get_logger().warn(
                f"[DBG] direction={np.round(direction, 4).tolist()}  "
                f"||dir||={norm:.6f}  d={msg.distance:.4f}",
                throttle_duration_sec=1.0,
            )

            if norm <= 1e-6:
                self.get_logger().warn("[DBG] direction norm ≤ 1e-6 → zero output")
            else:
                direction = direction / norm
                v_avoid = -self._k_rep * direction
                self.get_logger().warn(
                    f"[DBG] v_avoid={np.round(v_avoid, 4).tolist()}  k_rep={self._k_rep}",
                    throttle_duration_sec=1.0,
                )

                # ── CHECK 6: Jacobian ───────────────────────────────────
                J = pin.computeFrameJacobian(
                    self.model, self.data, self.q,
                    self._ee_frame_id,
                    pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
                )[:3, :]  # (3, nq)
                self.get_logger().warn(
                    f"[DBG] J.shape={J.shape}  ||J||={np.linalg.norm(J):.6f}",
                    throttle_duration_sec=1.0,
                )

                # ── CHECK 7: projection ─────────────────────────────────
                qdot_raw = J.T @ v_avoid
                self.get_logger().warn(
                    f"[DBG] qdot_raw={np.round(qdot_raw, 5).tolist()}  "
                    f"||qdot_raw||={np.linalg.norm(qdot_raw):.6f}",
                    throttle_duration_sec=1.0,
                )

                # ── CHECK 8: clipping ───────────────────────────────────
                qdot_cmd = np.clip(qdot_raw, -self._max_joint_vel, self._max_joint_vel)
                self.get_logger().warn(
                    f"[DBG] max_joint_vel={self._max_joint_vel}  "
                    f"||qdot_clipped||={np.linalg.norm(qdot_cmd):.6f}",
                    throttle_duration_sec=1.0,
                )

        # ── PUBLISH ─────────────────────────────────────────────────────
        self.get_logger().warn(
            f"[DBG] publishing ||qdot||={np.linalg.norm(qdot_cmd):.6f}",
            throttle_duration_sec=0.5,
        )
        msg_out = Float64MultiArray()
        msg_out.data = qdot_cmd.tolist()
        self.pub.publish(msg_out)
        self._update_markers()

    # ================================================================
    # Markers
    # ================================================================

    def _update_markers(self) -> None:
        """Rebuild capsule + distance markers for RViz."""
        if not self.pin_ok:
            return
        self.last_marker_array = build_marker_array(
            capsules=self.capsules,
            frame_ids=self.frame_ids,
            data=self.data,
            distances_data=self.distances_data,
            influence_distance=self._influence_distance,
            distance_inflation=self._distance_inflation,
            stamp_msg=self.get_clock().now().to_msg(),
            logger=self.get_logger(),
            debug_capsule_index=self._debug_capsule_index,
        )

    def _publish_markers_only(self) -> None:
        if len(self.last_marker_array.markers) > 0:
            self.capsule_marker_pub.publish(self.last_marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = HumanAvoidanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
