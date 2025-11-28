#!/usr/bin/env python3
"""
Test DIRETTO del velocity controller
=====================================

Questo script bypassa tutto il sistema e manda comandi
direttamente al velocity controller per verificare se funziona.

ATTENZIONE: Il robot si muoverà! Assicurati che sia in una posizione sicura.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import time

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class DirectVelocityTest(Node):
    def __init__(self):
        super().__init__('direct_velocity_test')
        
        # Publisher diretto al velocity controller
        self.vel_pub = self.create_publisher(
            Float64MultiArray, 
            '/fr3_velocity_controller/commands', 
            10
        )
        
        # Monitor joint state
        self.joint_positions = None
        self.joint_velocities = None
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        
        self.joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]
        
        self.get_logger().info("🎮 Direct Velocity Test ready")
    
    def joint_cb(self, msg):
        positions = []
        velocities = []
        for name in self.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
                if msg.velocity:
                    velocities.append(msg.velocity[idx])
        if len(positions) == 7:
            self.joint_positions = np.array(positions)
            self.joint_velocities = np.array(velocities) if velocities else np.zeros(7)
    
    def send_velocity(self, velocities):
        """Invia comando di velocità."""
        msg = Float64MultiArray()
        msg.data = velocities
        self.vel_pub.publish(msg)
    
    def print_state(self):
        """Stampa stato corrente."""
        if self.joint_positions is not None:
            print(f"\n📊 Joint State:")
            for i in range(7):
                pos = self.joint_positions[i]
                vel = self.joint_velocities[i] if self.joint_velocities is not None else 0
                print(f"   J{i+1}: pos={pos:+.4f} rad, vel={vel:+.4f} rad/s")
    
    def test_single_joint(self, joint_idx, velocity, duration):
        """Testa un singolo giunto."""
        print(f"\n{'='*60}")
        print(f"🧪 TEST: Joint {joint_idx+1} a {velocity} rad/s per {duration}s")
        print(f"{'='*60}")
        
        # Posizione iniziale
        while self.joint_positions is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        initial_pos = self.joint_positions[joint_idx]
        print(f"   Posizione iniziale J{joint_idx+1}: {initial_pos:.4f} rad")
        
        # Comando di velocità
        cmd = [0.0] * 7
        cmd[joint_idx] = velocity
        
        print(f"   Invio comando: {cmd}")
        
        # Esegui per la durata specificata
        start_time = time.time()
        while time.time() - start_time < duration:
            self.send_velocity(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
        
        # Ferma
        self.send_velocity([0.0] * 7)
        rclpy.spin_once(self, timeout_sec=0.1)
        
        # Posizione finale
        final_pos = self.joint_positions[joint_idx]
        delta = final_pos - initial_pos
        expected_delta = velocity * duration
        
        print(f"   Posizione finale J{joint_idx+1}: {final_pos:.4f} rad")
        print(f"   Δ posizione: {delta:.4f} rad")
        print(f"   Δ atteso: {expected_delta:.4f} rad")
        print(f"   Errore: {abs(delta - expected_delta):.4f} rad")
        
        if abs(delta) < 0.01:
            print(f"   ❌ IL GIUNTO NON SI È MOSSO!")
            return False
        elif abs(delta - expected_delta) > 0.1:
            print(f"   ⚠️  Movimento impreciso")
            return True
        else:
            print(f"   ✅ Movimento OK")
            return True


def main():
    print("\n" + "="*70)
    print("TEST DIRETTO VELOCITY CONTROLLER")
    print("="*70)
    print("\n⚠️  ATTENZIONE: Il robot si muoverà!")
    print("Assicurati che sia in una posizione sicura.\n")
    
    input("Premi ENTER per continuare...")
    
    rclpy.init()
    node = DirectVelocityTest()
    
    # Aspetta joint state
    print("\nAttendo joint_states...")
    while node.joint_positions is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    print("✅ Joint state ricevuto")
    
    node.print_state()
    
    try:
        print("\n" + "="*70)
        print("TEST 1: Movimento piccolo su Joint 1")
        print("="*70)
        
        input("\nPremi ENTER per muovere J1 di +0.1 rad/s per 1 secondo...")
        success = node.test_single_joint(0, 0.1, 1.0)
        
        if not success:
            print("\n❌ Il velocity controller NON risponde!")
            print("   Possibili cause:")
            print("   1. fr3_velocity_controller non è attivo")
            print("   2. Il controller è in conflitto con un altro")
            print("   3. Problema di configurazione ros2_control")
        else:
            print("\n✅ Il velocity controller funziona!")
            
            print("\n" + "="*70)
            print("TEST 2: Movimento su Joint 4")
            print("="*70)
            input("\nPremi ENTER per muovere J4 di +0.1 rad/s per 1 secondo...")
            node.test_single_joint(3, 0.1, 1.0)
            
            print("\n" + "="*70)
            print("TEST 3: Movimento multiplo")
            print("="*70)
            input("\nPremi ENTER per muovere J1, J2, J4 insieme...")
            
            # Comando multiplo
            cmd = [0.1, -0.1, 0.0, 0.1, 0.0, 0.0, 0.0]
            print(f"   Comando: {cmd}")
            
            initial = node.joint_positions.copy()
            
            start_time = time.time()
            while time.time() - start_time < 2.0:
                node.send_velocity(cmd)
                rclpy.spin_once(node, timeout_sec=0.05)
            
            node.send_velocity([0.0] * 7)
            rclpy.spin_once(node, timeout_sec=0.1)
            
            final = node.joint_positions
            print(f"\n   Δ posizioni:")
            for i in range(7):
                delta = final[i] - initial[i]
                print(f"      J{i+1}: {delta:+.4f} rad")
        
        print("\n" + "="*70)
        print("TEST COMPLETATO")
        print("="*70)
        
        node.print_state()
        
    except KeyboardInterrupt:
        # Ferma il robot
        node.send_velocity([0.0] * 7)
        print("\n⏹️ Test interrotto")
    finally:
        # Assicurati che il robot sia fermo
        node.send_velocity([0.0] * 7)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()