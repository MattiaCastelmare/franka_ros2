#!/usr/bin/env python3
"""
TEST SEMPLICE CON WAYPOINT SICURI
=================================

Questo test usa waypoint al centro del workspace per verificare
il tracking senza complicazioni di IK.

Waypoint scelti:
- Tutti in un'area centrale (0.3-0.5m davanti, ±0.3m laterale, 0.3-0.5m alto)
- Orientamento sempre gripper verso il basso
- Nessun ostacolo vicino

Author: Test semplificato
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
import time

from franka_simulation.action import MoveToPose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import MoveItErrorCodes


class SimpleWaypointTest(Node):
    def __init__(self):
        super().__init__('simple_waypoint_test')
        
        # Action client
        self.move_client = ActionClient(self, MoveToPose, 'move_to_pose')
        
        # FK service
        self.fk_client = self.create_client(GetPositionFK, 'compute_fk')
        
        # Joint state
        self.joint_positions = None
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        
        # Velocity monitoring
        self.last_cmd_vel = None
        self.create_subscription(
            Float64MultiArray, 
            '/fr3_velocity_controller/commands',
            self.vel_cb, 10
        )
        
        self.joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]
        
        self.get_logger().info("🎯 Simple Waypoint Test ready")
    
    def joint_cb(self, msg):
        positions = []
        for name in self.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
        if len(positions) == 7:
            self.joint_positions = np.array(positions)
    
    def vel_cb(self, msg):
        self.last_cmd_vel = np.array(msg.data)
    
    def compute_fk(self):
        """Calcola posizione EE corrente."""
        if not self.fk_client.wait_for_service(timeout_sec=2.0):
            return None
        
        request = GetPositionFK.Request()
        request.header.frame_id = 'fr3_link0'
        request.fk_link_names = ['fr3_hand_tcp']
        request.robot_state.joint_state.name = self.joint_names
        request.robot_state.joint_state.position = self.joint_positions.tolist()
        
        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() and future.result().error_code.val == MoveItErrorCodes.SUCCESS:
            pose = future.result().pose_stamped[0].pose
            return (pose.position.x, pose.position.y, pose.position.z)
        return None
    
    def move_to_pose(self, x, y, z, timeout=30.0):
        """Muove il robot a una posa cartesiana."""
        if not self.move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("❌ Move action server not available")
            return False
        
        goal = MoveToPose.Goal()
        goal.pose_target.header.frame_id = 'fr3_link0'
        goal.pose_target.pose.position.x = x
        goal.pose_target.pose.position.y = y
        goal.pose_target.pose.position.z = z
        # Gripper verso il basso
        goal.pose_target.pose.orientation.x = 1.0
        goal.pose_target.pose.orientation.y = 0.0
        goal.pose_target.pose.orientation.z = 0.0
        goal.pose_target.pose.orientation.w = 0.0
        
        goal.cartesian_motion = False
        goal.max_velocity_scaling_factor = 0.3
        
        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejected")
            return False
        
        # Aspetta completamento
        result_future = goal_handle.get_result_async()
        
        start_time = time.time()
        while not result_future.done() and time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
            
            # Monitora velocità
            if self.last_cmd_vel is not None:
                vel_norm = np.linalg.norm(self.last_cmd_vel)
                
                # Verifica se siamo fermi (settling completato)
                if vel_norm < 0.001:
                    # Calcola errore
                    current_pos = self.compute_fk()
                    if current_pos:
                        error = np.sqrt(
                            (current_pos[0] - x)**2 + 
                            (current_pos[1] - y)**2 + 
                            (current_pos[2] - z)**2
                        )
                        if error < 0.01:  # < 1cm
                            self.get_logger().info(f"✅ Target reached! Error: {error*100:.1f} cm")
                            return True
        
        return result_future.done()


def main():
    print("\n" + "="*70)
    print("🎯 TEST WAYPOINT SEMPLICI")
    print("="*70)
    print("""
Questo test usa waypoint sicuri al centro del workspace:
- Posizioni facili da raggiungere
- Nessun ostacolo
- Orientamento sempre gripper-giù

I waypoint sono:
1. Centro alto:     (0.40, 0.00, 0.45)
2. Sinistra:        (0.40, 0.20, 0.40)
3. Destra:          (0.40, -0.20, 0.40)
4. Avanti basso:    (0.50, 0.00, 0.30)
5. Ritorno centro:  (0.40, 0.00, 0.45)

""")
    
    rclpy.init()
    node = SimpleWaypointTest()
    
    # Aspetta joint state
    print("Attendo joint_states...")
    while node.joint_positions is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    print("✅ Joint state ricevuto\n")
    
    # Waypoint sicuri
    waypoints = [
        ("Centro Alto", 0.40, 0.00, 0.45),
        ("Sinistra", 0.40, 0.20, 0.40),
        ("Destra", 0.40, -0.20, 0.40),
        ("Avanti Basso", 0.50, 0.00, 0.30),
        ("Ritorno Centro", 0.40, 0.00, 0.45),
    ]
    
    try:
        for i, (name, x, y, z) in enumerate(waypoints):
            print(f"\n{'='*60}")
            print(f"[{i+1}/{len(waypoints)}] {name}")
            print(f"   Target: ({x:.2f}, {y:.2f}, {z:.2f})")
            print(f"{'='*60}")
            
            # Posizione iniziale
            initial_pos = node.compute_fk()
            if initial_pos:
                print(f"   Posizione attuale: ({initial_pos[0]:.3f}, {initial_pos[1]:.3f}, {initial_pos[2]:.3f})")
            
            input("   Premi ENTER per muovere...")
            
            # Movimento
            print(f"   🚀 Movimento in corso...")
            success = node.move_to_pose(x, y, z, timeout=20.0)
            
            # Aspetta settling
            time.sleep(2.0)
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=0.1)
            
            # Verifica posizione finale
            final_pos = node.compute_fk()
            if final_pos:
                error_x = final_pos[0] - x
                error_y = final_pos[1] - y
                error_z = final_pos[2] - z
                error_total = np.sqrt(error_x**2 + error_y**2 + error_z**2)
                
                print(f"\n   📍 RISULTATO:")
                print(f"      Target:   ({x:.4f}, {y:.4f}, {z:.4f})")
                print(f"      Raggiunto: ({final_pos[0]:.4f}, {final_pos[1]:.4f}, {final_pos[2]:.4f})")
                print(f"      Errore: ΔX={error_x*100:+.1f}cm, ΔY={error_y*100:+.1f}cm, ΔZ={error_z*100:+.1f}cm")
                print(f"      Errore totale: {error_total*100:.1f} cm")
                
                if error_total < 0.02:  # < 2cm
                    print(f"      ✅ SUCCESSO (errore < 2cm)")
                elif error_total < 0.05:
                    print(f"      ⚠️ ACCETTABILE (errore < 5cm)")
                else:
                    print(f"      ❌ ERRORE TROPPO GRANDE")
        
        print("\n" + "="*60)
        print("TEST COMPLETATO")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n⏹️ Test interrotto")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()