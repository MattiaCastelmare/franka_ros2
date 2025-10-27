#!/usr/bin/env python3
"""
Hybrid Planning Test - VERSIONE MIGLIORATA E INCREMENTALE
==========================================================

Questo script risolve i problemi identificati nel demo originale:
1. ✅ Usa ExecuteHybridMotion per vero hybrid planning
2. ✅ Test incrementali (singolo target → multipli → con ostacoli)
3. ✅ Validazione pose prima di eseguire
4. ✅ Monitoring dettagliato Servo e collision
5. ✅ Fallback intelligente su errori

Modalità di esecuzione:
- MODE 1: Test singolo target (validazione base)
- MODE 2: Test multipli target (sequenza)
- MODE 3: Test hybrid completo (global + servo)
- MODE 4: Test con ostacoli (obstacle avoidance)

Author: Improved by Claude
Date: 2025-01-23
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Path
from std_msgs.msg import String

from franka_simulation.action import (
    MoveToPose, 
    MoveToJoint, 
    PlanGlobalPath,
    ExecuteHybridMotion
)
from moveit_msgs.msg import MoveItErrorCodes

import time
import numpy as np
from enum import Enum
from typing import Optional, List, Tuple
from dataclasses import dataclass


class TestMode(Enum):
    """Modalità di test disponibili"""
    SINGLE_TARGET = 1      # Test singolo target con MoveToPose
    MULTIPLE_TARGETS = 2   # Test multipli target sequenziali
    HYBRID_PLANNING = 3    # Test hybrid completo con Servo
    WITH_OBSTACLES = 4     # Test con obstacle avoidance


@dataclass
class TestMetrics:
    """Metriche per ogni test"""
    success: bool = False
    planning_time: float = 0.0
    execution_time: float = 0.0
    error_message: str = ""
    waypoints_count: int = 0
    servo_active: bool = False


class HybridPlanningTest(Node):
    """
    Test node per validazione incrementale Hybrid Planning
    """
    
    def __init__(self, test_mode: TestMode = TestMode.SINGLE_TARGET):
        super().__init__('hybrid_planning_test')
        
        self.test_mode = test_mode
        self.callback_group = ReentrantCallbackGroup()
        
        # Action clients
        self.move_client = ActionClient(
            self, MoveToPose, '/move_to_pose',
            callback_group=self.callback_group
        )
        self.joint_client = ActionClient(
            self, MoveToJoint, '/move_to_joint',
            callback_group=self.callback_group
        )
        self.plan_client = ActionClient(
            self, PlanGlobalPath, '/plan_global_path',
            callback_group=self.callback_group
        )
        self.hybrid_client = ActionClient(
            self, ExecuteHybridMotion, '/execute_hybrid_motion',
            callback_group=self.callback_group
        )
        
        # Subscribers per monitoring
        self.servo_status_sub = self.create_subscription(
            String, '/servo_server/status',
            self.servo_status_callback, 10
        )
        self.servo_twist_sub = self.create_subscription(
            TwistStamped, '/servo_server/delta_twist_cmds',
            self.servo_twist_callback, 10
        )
        self.global_path_sub = self.create_subscription(
            Path, '/franka/global_path',
            self.global_path_callback, 10
        )
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states',
            self.joint_states_callback, 10
        )
        
        # Stato interno
        self.servo_active = False
        self.servo_status = None
        self.last_twist = None
        self.global_path = None
        self.current_joint_state = None
        self.test_results: List[TestMetrics] = []
        
        # Home configuration
        self.home_config = [0.0, 0.0, -0.785, -2.356, 0.0, 1.571, 0.785]
        
        # Target poses validate (dentro workspace FR3)
        self.safe_targets = self._define_safe_targets()
        
        self.get_logger().info(f"🚀 Hybrid Planning Test Node - Mode: {test_mode.name}")
        
        # Wait for servers
        self._wait_for_servers()
    
    # ========================================
    # Callbacks per Monitoring
    # ========================================
    
    def servo_status_callback(self, msg):
        """Monitor Servo status"""
        self.servo_status = msg.data
        if "ACTIVE" in msg.data or "RUNNING" in msg.data:
            if not self.servo_active:
                self.get_logger().info("✅ MoveIt Servo ATTIVO")
                self.servo_active = True
        elif "COLLISION" in msg.data:
            self.get_logger().warn("⚠️ Collisione rilevata da Servo!")
        elif "STOPPED" in msg.data:
            if self.servo_active:
                self.get_logger().info("🛑 Servo fermato")
                self.servo_active = False
    
    def servo_twist_callback(self, msg):
        """Monitor Servo twist commands"""
        self.last_twist = msg
        # Log solo se ci sono comandi significativi
        linear_norm = np.sqrt(msg.twist.linear.x**2 + 
                             msg.twist.linear.y**2 + 
                             msg.twist.linear.z**2)
        if linear_norm > 0.01:
            self.get_logger().debug(f"Servo twist: linear={linear_norm:.3f} m/s")
    
    def global_path_callback(self, msg):
        """Monitor global path pubblicato"""
        self.global_path = msg
        self.get_logger().info(f"📍 Global path ricevuto: {len(msg.poses)} waypoints")
    
    def joint_states_callback(self, msg):
        """Monitor joint states"""
        self.current_joint_state = msg
    
    # ========================================
    # Setup e Utility
    # ========================================
    
    def _wait_for_servers(self):
        """Attendi tutti i server con timeout"""
        timeout = 15.0
        
        self.get_logger().info("⏳ Attendo action servers...")
        
        servers = [
            (self.move_client, "MoveToPose"),
            (self.joint_client, "MoveToJoint"),
            (self.plan_client, "PlanGlobalPath"),
        ]
        
        # ExecuteHybridMotion solo per mode 3 e 4
        if self.test_mode in [TestMode.HYBRID_PLANNING, TestMode.WITH_OBSTACLES]:
            servers.append((self.hybrid_client, "ExecuteHybridMotion"))
        
        for client, name in servers:
            if not client.wait_for_server(timeout_sec=timeout):
                self.get_logger().error(f"❌ Timeout waiting for {name} server")
                raise RuntimeError(f"Server {name} non disponibile")
            self.get_logger().info(f"✅ {name} server pronto")
    
    def _define_safe_targets(self) -> List[PoseStamped]:
        """
        Define target poses validati dentro workspace FR3.
        
        Workspace FR3 tipico:
        - x: [0.2, 0.8] forward/backward
        - y: [-0.6, 0.6] left/right  
        - z: [0.1, 0.8] height
        
        Orientamento: end-effector pointing down (standard pick)
        """
        targets = []
        
        # Quaternion per end-effector pointing down
        # (equivalente a rotation di 180° attorno a Y)
        quat_down = [1.0, 0.0, 0.0, 0.0]  # [qx, qy, qz, qw]
        
        # Target 1: Centro workspace (safe)
        targets.append(self._make_pose(0.4, 0.3, 0.3, *quat_down))
        
        # Target 2: Destra
        targets.append(self._make_pose(0.4, -0.25, 0.35, *quat_down))
        
        # Target 3: Sinistra
        targets.append(self._make_pose(0.5, 0.25, 0.35, *quat_down))
        
        # Target 4: Alto (sopra ostacolo potenziale)
        targets.append(self._make_pose(0.45, 0.0, 0.55, *quat_down))
        
        # Target 5: Forward
        targets.append(self._make_pose(0.6, 0.15, 0.4, *quat_down))
        
        return targets
    
    def _make_pose(self, x, y, z, qx, qy, qz, qw) -> PoseStamped:
        """Crea PoseStamped con frame corretto"""
        pose = PoseStamped()
        pose.header.frame_id = "fr3_link0"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose
    
    def _validate_pose(self, pose: PoseStamped) -> bool:
        """Valida che la pose sia nel workspace"""
        p = pose.pose.position
        
        # Workspace limits FR3
        if not (0.2 <= p.x <= 0.8):
            self.get_logger().warn(f"⚠️ x={p.x:.2f} fuori range [0.2, 0.8]")
            return False
        if not (-0.6 <= p.y <= 0.6):
            self.get_logger().warn(f"⚠️ y={p.y:.2f} fuori range [-0.6, 0.6]")
            return False
        if not (0.1 <= p.z <= 0.8):
            self.get_logger().warn(f"⚠️ z={p.z:.2f} fuori range [0.1, 0.8]")
            return False
        
        return True
    
    # ========================================
    # Motion Primitives
    # ========================================
    
    def move_to_home(self) -> bool:
        """Torna alla home configuration"""
        self.get_logger().info("🏠 Movimento verso HOME...")
        
        goal = MoveToJoint.Goal()
        goal.joint_target = self.home_config
        goal.joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]
        goal.max_velocity_scaling_factor = 0.2
        goal.max_acceleration_scaling_factor = 0.2
        goal.planner_id = 'RRTConnect'
        goal.planning_time = 10.0
        
        return self._send_goal_and_wait(
            self.joint_client, goal, "Home Position"
        )
    
    def move_to_pose(self, pose: PoseStamped, label: str = "Target") -> TestMetrics:
        """
        Movimento cartesiano usando MoveToPose (planning + execution)
        """
        metrics = TestMetrics()
        
        if not self._validate_pose(pose):
            metrics.error_message = "Pose validation failed"
            return metrics
        
        self.get_logger().info(f"🎯 Planning verso {label}...")
        self.get_logger().info(f"   Pose: ({pose.pose.position.x:.2f}, "
                              f"{pose.pose.position.y:.2f}, "
                              f"{pose.pose.position.z:.2f})")
        
        start_time = time.time()
        
        goal = MoveToPose.Goal()
        goal.pose_target = pose
        goal.cartesian_motion = False  # Usa planning OMPL
        goal.max_velocity_scaling_factor = 0.15
        goal.max_acceleration_scaling_factor = 0.15
        goal.tolerance_position = 0.005
        goal.tolerance_orientation = 0.05
        goal.planner_id = 'RRTConnect'
        goal.planning_time = 15.0
        
        success = self._send_goal_and_wait(
            self.move_client, goal, label
        )
        
        metrics.success = success
        metrics.execution_time = time.time() - start_time
        
        if success:
            self.get_logger().info(f"✅ {label} raggiunto in {metrics.execution_time:.2f}s")
        else:
            metrics.error_message = "MoveToPose failed"
            self.get_logger().error(f"❌ {label} fallito")
        
        return metrics
    
    def execute_hybrid_motion(self, pose: PoseStamped, label: str = "Target") -> TestMetrics:
        """
        Esecuzione hybrid: global plan (OMPL) + local tracking (Servo)
        """
        metrics = TestMetrics()
        
        if not self._validate_pose(pose):
            metrics.error_message = "Pose validation failed"
            return metrics
        
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"🔄 HYBRID PLANNING verso {label}")
        self.get_logger().info("=" * 60)
        
        # Reset monitoring
        self.servo_active = False
        self.global_path = None
        
        start_time = time.time()
        
        goal = ExecuteHybridMotion.Goal()
        goal.target_pose = pose
        goal.use_hybrid_planning = True
        goal.velocity_scaling = 0.2
        goal.acceleration_scaling = 0.2
        
        # Invia goal
        self.hybrid_client.wait_for_server()
        future = self.hybrid_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Hybrid motion goal RIFIUTATO")
            metrics.error_message = "Goal rejected"
            return metrics
        
        self.get_logger().info("✅ Goal accettato, attendo esecuzione...")
        
        # Attendi risultato con monitoring
        result_future = goal_handle.get_result_async()
        
        # Monitoring loop
        monitor_rate = self.create_rate(2)  # 2 Hz
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # Log stato ogni 0.5s
            if self.servo_active:
                self.get_logger().info("🔄 Servo attivo - tracking in corso...")
            if self.global_path:
                self.get_logger().info(f"📍 Path: {len(self.global_path.poses)} waypoints")
            
            monitor_rate.sleep()
        
        # Risultato
        result = result_future.result().result
        
        metrics.execution_time = time.time() - start_time
        metrics.servo_active = self.servo_active
        
        if self.global_path:
            metrics.waypoints_count = len(self.global_path.poses)
        
        # Check success - ExecuteHybridMotion usa codici custom
        if hasattr(result, 'error_code'):
            code = result.error_code
        elif hasattr(result, 'result'):
            code = result.result.val
        else:
            code = -999
        
        # SUCCESS = 0 per ExecuteHybridMotion
        if code == 0:
            metrics.success = True
            self.get_logger().info("=" * 60)
            self.get_logger().info("✅ HYBRID MOTION COMPLETATO")
            self.get_logger().info(f"   Tempo: {metrics.execution_time:.2f}s")
            self.get_logger().info(f"   Waypoints: {metrics.waypoints_count}")
            self.get_logger().info(f"   Servo usato: {metrics.servo_active}")
            self.get_logger().info(f"   Replans: {result.replans_count if hasattr(result, 'replans_count') else 0}")
            self.get_logger().info("=" * 60)
        else:
            error_msg = result.error_message if hasattr(result, 'error_message') else f"Code {code}"
            metrics.error_message = error_msg
            self.get_logger().error(f"❌ Hybrid motion fallito: {error_msg}")
        
        return metrics
    
    # ========================================
    # Goal Handling
    # ========================================
    
    def _send_goal_and_wait(self, client, goal_msg, label: str) -> bool:
        """Helper per inviare goal e attendere risultato"""
        client.wait_for_server()
        
        future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if not future.done():
            self.get_logger().error(f"❌ Timeout sending {label} goal")
            return False
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"❌ {label} goal rejected")
            return False
        
        self.get_logger().info(f"⏳ Attendo completamento {label}...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        
        # Extract error code
        if hasattr(result, 'error_code'):
            code = result.error_code
        elif hasattr(result, 'result') and hasattr(result.result, 'val'):
            code = result.result.val
        else:
            code = -999
        
        return code == MoveItErrorCodes.SUCCESS or code == 1
    
    # ========================================
    # Test Modes
    # ========================================
    
    def run_test_mode_1(self):
        """
        MODE 1: Test singolo target
        Valida che MoveToPose funziona per una pose safe
        """
        self.get_logger().info("🧪 TEST MODE 1: Single Target")
        self.get_logger().info("=" * 60)
        
        # Home
        if not self.move_to_home():
            self.get_logger().error("❌ Failed to reach home")
            return
        
        input("\n✋ Premi INVIO per testare il primo target sicuro...")
        
        # Test primo target safe
        target = self.safe_targets[0]
        metrics = self.move_to_pose(target, "Target Safe #1")
        self.test_results.append(metrics)
        
        # Ritorna home
        input("\n✋ Premi INVIO per tornare a home...")
        self.move_to_home()
        
        self._print_summary()
    
    def run_test_mode_2(self):
        """
        MODE 2: Test multipli target
        Sequenza di 3-4 pose validate
        """
        self.get_logger().info("🧪 TEST MODE 2: Multiple Targets")
        self.get_logger().info("=" * 60)
        
        # Home
        if not self.move_to_home():
            return
        
        input(f"\n✋ Premi INVIO per testare {len(self.safe_targets[:4])} target...")
        
        # Test primi 4 target
        for i, target in enumerate(self.safe_targets[:4]):
            self.get_logger().info(f"\n🎯 Target {i+1}/4")
            metrics = self.move_to_pose(target, f"Target #{i+1}")
            self.test_results.append(metrics)
            
            if not metrics.success:
                risposta = input(f"\n⚠️ Target {i+1} fallito. Continuare? (INVIO=sì, n=no): ")
                if risposta.lower() == 'n':
                    break
            elif i < 3:
                input("\n✋ Premi INVIO per il prossimo target...")
        
        # Ritorna home
        input("\n✋ Premi INVIO per tornare a home...")
        self.move_to_home()
        
        self._print_summary()
    
    def run_test_mode_3(self):
        """
        MODE 3: Test hybrid planning completo
        Usa ExecuteHybridMotion con Servo attivo
        """
        self.get_logger().info("🧪 TEST MODE 3: Hybrid Planning (Global + Servo)")
        self.get_logger().info("=" * 60)
        
        # Verifica che Servo sia disponibile
        self.get_logger().info("⏳ Verifica disponibilità MoveIt Servo...")
        time.sleep(2.0)  # Attendi Servo startup
        
        # Home
        if not self.move_to_home():
            return
        
        input("\n✋ Premi INVIO per testare HYBRID PLANNING su primo target...")
        
        # Test hybrid su primi 2 target
        for i, target in enumerate(self.safe_targets[:2]):
            self.get_logger().info(f"\n🔄 Hybrid Motion {i+1}/2")
            metrics = self.execute_hybrid_motion(target, f"Hybrid Target #{i+1}")
            self.test_results.append(metrics)
            
            if not metrics.success:
                self.get_logger().error(f"❌ Hybrid motion {i+1} fallito")
                risposta = input("\n⚠️ Continuare? (INVIO=sì, n=no): ")
                if risposta.lower() == 'n':
                    break
            elif i < 1:
                input("\n✋ Premi INVIO per il prossimo hybrid motion...")
        
        # Ritorna home
        input("\n✋ Premi INVIO per tornare a home...")
        self.move_to_home()
        
        self._print_summary()
    
    def run_test_mode_4(self):
        """
        MODE 4: Test con ostacoli
        Scenario interessante: target dietro ostacolo centrale
        """
        self.get_logger().info("🧪 TEST MODE 4: Obstacle Avoidance")
        self.get_logger().info("=" * 60)
        self.get_logger().info("⚠️  Assicurati che obstacle_synchronizer sia attivo!")
        
        input("\n✋ Premi INVIO quando gli ostacoli sono caricati in RViz...")
        
        # Home
        if not self.move_to_home():
            return
        
        # Target che dovrebbe aggirare ostacolo centrale
        # (assumendo ostacolo a x=0.4, y=0.0, z=0.3)
        obstacle_avoidance_target = self._make_pose(0.6, 0.0, 0.4, 0.0, 1.0, 0.0, 0.0)
        
        self.get_logger().info("\n🎯 Test obstacle avoidance con hybrid planning...")
        metrics = self.execute_hybrid_motion(
            obstacle_avoidance_target, 
            "Target Behind Obstacle"
        )
        self.test_results.append(metrics)
        
        if metrics.success:
            self.get_logger().info("✅ Ostacolo aggirato con successo!")
        
        # Ritorna home
        input("\n✋ Premi INVIO per tornare a home...")
        self.move_to_home()
        
        self._print_summary()
    
    # ========================================
    # Summary e Report
    # ========================================
    
    def _print_summary(self):
        """Stampa summary dei test"""
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("📊 TEST SUMMARY")
        self.get_logger().info("=" * 60)
        
        total = len(self.test_results)
        success = sum(1 for m in self.test_results if m.success)
        
        self.get_logger().info(f"Totale test: {total}")
        self.get_logger().info(f"Successi: {success}")
        self.get_logger().info(f"Fallimenti: {total - success}")
        self.get_logger().info(f"Success rate: {100*success/total if total > 0 else 0:.1f}%")
        
        if self.test_results:
            avg_time = np.mean([m.execution_time for m in self.test_results])
            self.get_logger().info(f"Tempo medio: {avg_time:.2f}s")
        
        # Details
        for i, metrics in enumerate(self.test_results):
            status = "✅" if metrics.success else "❌"
            self.get_logger().info(
                f"{status} Test {i+1}: {metrics.execution_time:.2f}s"
                f"{' (Servo: ' + str(metrics.servo_active) + ')' if metrics.servo_active else ''}"
            )
            if not metrics.success:
                self.get_logger().info(f"   Error: {metrics.error_message}")
        
        self.get_logger().info("=" * 60)
    
    # ========================================
    # Main Run
    # ========================================
    
    def run(self):
        """Esegue il test mode selezionato"""
        if self.test_mode == TestMode.SINGLE_TARGET:
            self.run_test_mode_1()
        elif self.test_mode == TestMode.MULTIPLE_TARGETS:
            self.run_test_mode_2()
        elif self.test_mode == TestMode.HYBRID_PLANNING:
            self.run_test_mode_3()
        elif self.test_mode == TestMode.WITH_OBSTACLES:
            self.run_test_mode_4()


# ========================================
# Main
# ========================================

def main(args=None):
    rclpy.init(args=args)
    
    # Selezione modalità test
    print("\n" + "=" * 60)
    print("🧪 HYBRID PLANNING TEST - Selezione Modalità")
    print("=" * 60)
    print("1. Single Target Test (validazione base)")
    print("2. Multiple Targets Test (sequenza)")
    print("3. Hybrid Planning Test (global + servo)")
    print("4. Obstacle Avoidance Test (con ostacoli)")
    print("=" * 60)
    
    while True:
        try:
            choice = input("\nSeleziona modalità (1-4): ").strip()
            mode_map = {
                '1': TestMode.SINGLE_TARGET,
                '2': TestMode.MULTIPLE_TARGETS,
                '3': TestMode.HYBRID_PLANNING,
                '4': TestMode.WITH_OBSTACLES,
            }
            
            if choice in mode_map:
                test_mode = mode_map[choice]
                break
            else:
                print("❌ Selezione non valida, riprova.")
        except KeyboardInterrupt:
            print("\n🛑 Test annullato")
            return
    
    # Crea e esegui test node
    node = HybridPlanningTest(test_mode)
    
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("\n🛑 Test interrotto dall'utente")
    except Exception as e:
        node.get_logger().error(f"\n❌ Errore durante test: {e}")
        import traceback
        traceback.print_exc()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()