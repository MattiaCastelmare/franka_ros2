#!/usr/bin/env python3
"""
Hybrid Planning Demo for Franka FR3
===================================

Esegue un ciclo completo:
1. Torna alla home (configurazione di giunto)
2. Pianifica e si muove verso 4 pose cartesiane evitando ostacoli
3. Ritorna infine in home

MODIFICATO: Richiede conferma utente dopo ogni punto raggiunto
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from franka_simulation.action import MoveToPose, PlanGlobalPath, MoveToJoint
from moveit_msgs.msg import MoveItErrorCodes
import time


class HybridPlanningDemo(Node):
    def __init__(self):
        super().__init__('hybrid_planning_demo')

        # ✅ Usa i nomi corretti delle action
        self.plan_client = ActionClient(self, PlanGlobalPath, '/plan_global_path')
        self.move_client = ActionClient(self, MoveToPose, '/move_to_pose')
        self.joint_client = ActionClient(self, MoveToJoint, '/move_to_joint')


        self.get_logger().info("🚀 Hybrid Planning Demo node initialized.")

        self.plan_client.wait_for_server(timeout_sec=10.0)
        self.move_client.wait_for_server(timeout_sec=10.0)
        self.joint_client.wait_for_server(timeout_sec=10.0)


        # Home configuration (joint space)
        self.home_config = [0.0, 0.0, -0.785, -2.356, 0.0, 1.571, 0.785]

        # 4 target pose nello spazio cartesiano (evitando ostacoli)
        self.targets = [
            self._make_pose(0.45,  0.20, 0.30, 0.0, 1.0, 0.0, 0.0),
            self._make_pose(0.55, -0.25, 0.40, 0.0, 1.0, 0.0, 0.0),
            self._make_pose(0.35,  0.15, 0.45, 0.0, 1.0, 0.0, 0.0),
            self._make_pose(0.50,  0.25, 0.35, 0.0, 1.0, 0.0, 0.0),
        ]

    # ========================================
    # Utility
    # ========================================
    def _make_pose(self, x, y, z, qx, qy, qz, qw):
        pose = PoseStamped()
        pose.header.frame_id = "fr3_link0"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def send_goal_and_wait(self, client, goal_msg, label):
        """Invia un goal e attende il risultato"""
        client.wait_for_server()
        self.get_logger().info(f"🎯 Sending {label} goal...")
        future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"❌ {label} goal rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        # ✅ CORREZIONE qui — il tuo campo è `error_code`
        if hasattr(result, "result") and hasattr(result.result, "val"):
            # compatibilità retro
            code = result.result.val
        elif hasattr(result, "error_code"):
            code = result.error_code
        else:
            code = -999

        if code == MoveItErrorCodes.SUCCESS or code == 1:
            self.get_logger().info(f"✅ {label} completed successfully.")
            return True
        else:
            self.get_logger().warn(f"⚠️ {label} failed (code {code})")
            return False


    # ========================================
    # Motion Functions
    # ========================================
    def move_to_home(self):
        """Torna alla configurazione Home nello spazio dei giunti"""
        goal = MoveToJoint.Goal()
        goal.joint_target = self.home_config
        goal.joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3',
            'fr3_joint4', 'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]
        goal.max_velocity_scaling_factor = 0.3
        goal.max_acceleration_scaling_factor = 0.3
        goal.tolerance_joint = 0.001
        goal.planner_id = 'RRTConnect'
        goal.planning_time = 5.0
        goal.validate_target = True

        self.send_goal_and_wait(self.joint_client, goal, "Return to Home (Joint Space)")


    def perform_demo(self):
        """Esegue la demo ibrida con conferma utente dopo ogni punto raggiunto"""
        self.get_logger().info("🟢 Starting Hybrid Planning Demo")

        # === STEP 1: Torna a Home ===
        self.move_to_home()

        # 🔸 Chiedi conferma prima di partire
        input("\n✅ Il robot è in posizione HOME. Premi INVIO per iniziare la demo... ")

        # === STEP 2: Definizione punti target nello spazio ===
        targets = [
            self._make_pose(0.35,  -0.3, 0.25, 0.0, 1.0, 0.0, 0.0),
            self._make_pose(0.40, 0.45, 0.55, 0.0, 1.0, 0.0, 0.0),
            self._make_pose(0.35,  -0.3, 0.35, 0.0, 1.0, 0.0, 0.0),
            self._make_pose(0.40, 0.45, 0.45, 0.0, 1.0, 0.0, 0.0),
        ]

        # === STEP 3: Ciclo con conferma dopo ogni punto ===
        for i, pose in enumerate(targets):
            self.get_logger().info(f"\n{'='*50}")
            self.get_logger().info(f"📍 Target {i+1}/{len(targets)}")
            self.get_logger().info(f"{'='*50}")
            
            # 🔹 PIANIFICAZIONE del punto corrente
            self.get_logger().info(f"🧠 Pianificazione verso Target {i+1}...")
            plan_goal = PlanGlobalPath.Goal()
            plan_goal.target_pose = pose
            plan_goal.planner_id = "RRTConnect"
            plan_goal.planning_time = 15.0
            plan_goal.max_attempts = 3

            success_plan = self.send_goal_and_wait(self.plan_client, plan_goal, f"Global Planning #{i+1}")
            if not success_plan:
                self.get_logger().warn(f"⚠️ Pianificazione fallita per Target {i+1}. Salto questo punto.")
                continue

            # 🔹 ESECUZIONE del movimento
            self.get_logger().info(f"🤖 Esecuzione movimento verso Target {i+1}...")
            move_goal = MoveToPose.Goal()
            move_goal.pose_target = pose
            move_goal.cartesian_motion = False
            move_goal.max_velocity_scaling_factor = 0.3
            move_goal.max_acceleration_scaling_factor = 0.3
            move_goal.tolerance_position = 0.002
            move_goal.tolerance_orientation = 0.05
            move_goal.planner_id = "RRTConnect"
            move_goal.planning_time = 10.0
            
            success_move = self.send_goal_and_wait(self.move_client, move_goal, f"Execution #{i+1}")
            
            if success_move:
                # ⭐ PUNTO RAGGIUNTO - Chiedi conferma prima di procedere
                self.get_logger().info(f"🎯 Target {i+1} raggiunto!")
                
                if i < len(targets) - 1:
                    # Non è l'ultimo punto - chiedi conferma per continuare
                    input(f"\n✋ Premi INVIO per pianificare e muovere verso il Target {i+2}... ")
                else:
                    # È l'ultimo punto
                    self.get_logger().info("✅ Tutti i target sono stati raggiunti!")
            else:
                self.get_logger().warn(f"⚠️ Esecuzione fallita per Target {i+1}")
                risposta = input(f"\n❓ Vuoi continuare con il prossimo punto? (INVIO=Sì, n=No): ")
                if risposta.lower() == 'n':
                    self.get_logger().info("🛑 Demo interrotta dall'utente")
                    return

        # === STEP 4: Torna a Home finale ===
        self.get_logger().info(f"\n{'='*50}")
        input("✋ Premi INVIO per tornare alla posizione HOME finale... ")
        self.move_to_home()
        self.get_logger().info("🏁 Hybrid Planning Demo Completata!")



def main(args=None):
    rclpy.init(args=args)
    node = HybridPlanningDemo()
    try:
        node.perform_demo()
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Demo interrotta dall'utente.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()