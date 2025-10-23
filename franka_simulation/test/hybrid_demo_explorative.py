#!/usr/bin/env python3
"""
🦾 HYBRID PLANNING – OBSTACLE AVOIDANCE DEMO
=============================================
Simulazione semplice:
- Il robot parte in HOME.
- Pianifica un path globale verso un target a destra dell'ostacolo.
- Il planner locale (Servo) rifinisce la traiettoria online evitando il box.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from moveit_msgs.msg import MoveItErrorCodes
from franka_simulation.action import MoveToJoint, PlanGlobalPath, ExecuteHybridMotion
import time

class HybridAvoidanceDemo(Node):
    def __init__(self):
        super().__init__('hybrid_avoidance_demo')

        # Action clients
        self.joint_client = ActionClient(self, MoveToJoint, '/move_to_joint')
        self.global_client = ActionClient(self, PlanGlobalPath, '/plan_global_path')
        self.hybrid_client = ActionClient(self, ExecuteHybridMotion, '/execute_hybrid_motion')

        self.path_pub = self.create_publisher(Path, '/demo/global_path', 10)

        self.get_logger().info("⏳ Attesa dei server...")
        self.joint_client.wait_for_server()
        self.global_client.wait_for_server()
        self.hybrid_client.wait_for_server()
        self.get_logger().info("✅ Tutti i server connessi!")

        # Home joints
        self.home_config = [0.0, 0.0, -0.785, -2.356, 0.0, 1.571, 0.785]

    # Utility per aspettare input utente
    def wait_user(self, msg):
        input(f"\n👉 {msg}")

    # Movimento a configurazione di giunti
    def move_to_joint(self, joints, label=""):
        goal = MoveToJoint.Goal()
        goal.joint_target = joints
        goal.max_velocity_scaling_factor = 0.3

        self.get_logger().info(f"🔧 Movimento: {label}")
        future = self.joint_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()

        if not handle.accepted:
            self.get_logger().error("❌ Goal rifiutato")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        ok = (getattr(result, "result", 0) == MoveItErrorCodes.SUCCESS)
        self.get_logger().info("✅ Completato" if ok else "⚠️ Fallito")
        return ok

    # Pianificazione globale
    def plan_global(self, pose: PoseStamped):
        goal = PlanGlobalPath.Goal()
        goal.target_pose = pose
        goal.planner_id = "RRTstar"
        goal.planning_time = 5.0

        self.get_logger().info(f"🌍 Pianificazione globale verso {pose.pose.position}")
        future = self.global_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("❌ Planning rifiutato")
            return None

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error("❌ Planning fallito")
            return None

        path = result.waypoints_path
        path.header.frame_id = "fr3_link0"
        self.path_pub.publish(path)
        self.get_logger().info(f"✅ {result.num_waypoints} waypoints pubblicati in RViz")
        return path

    # Esecuzione ibrida
    def execute_hybrid(self, pose: PoseStamped):
        goal = ExecuteHybridMotion.Goal()
        goal.target_pose = pose
        goal.use_hybrid_planning = True

        self.get_logger().info(f"🤖 Avvio esecuzione ibrida verso {pose.pose.position}")
        future = self.hybrid_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()

        if not handle.accepted:
            self.get_logger().error("❌ Goal ibrido rifiutato")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        ok = (getattr(result, "error_code", 0) == MoveItErrorCodes.SUCCESS)
        self.get_logger().info("🎯 Successo" if ok else "⚠️ Fallito")
        return ok


def main():
    rclpy.init()
    node = HybridAvoidanceDemo()

    try:
        # Pose helper
        def pose(x, y, z):
            p = PoseStamped()
            p.header.frame_id = 'fr3_link0'
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = z
            p.pose.orientation.x = 0.0
            p.pose.orientation.y = 0.0
            p.pose.orientation.z = 0.0
            p.pose.orientation.w = 1.0
            return p

        node.wait_user("Premi INVIO per portare il robot in HOME")
        node.move_to_joint(node.home_config, "HOME")

        # Target a destra dell’ostacolo (x costante, varia y)
        start_left = pose(0.4, -0.25, 0.4)  # lato sinistro
        goal_right = pose(0.4, 0.25, 0.4)   # lato destro

        node.wait_user("Premi INVIO per pianificare il path globale (sinistra → destra)")
        node.plan_global(goal_right)

        node.wait_user("Premi INVIO per eseguire il planning ibrido (Servo avoidance)")
        node.execute_hybrid(goal_right)

        node.wait_user("🏁 Premi INVIO per tornare in HOME")
        node.move_to_joint(node.home_config, "HOME finale")

        node.get_logger().info("✅ DEMO COMPLETATA.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
