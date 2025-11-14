#!/usr/bin/env python3
"""
Online Avoidance Controller Test Demo
======================================

Testa il controller Jacobian-based con null-space repulsion.

Sequenza:
1. Avvia e verifica che il controller sia attivo
2. Movimento lento verso ostacolo per verificare repulsione
3. Test con target multipli vicini a ostacoli
4. Visualizza metriche di performance
"""

import rclpy
import time
import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from franka_motion_client import FrankaMotionClient
from moveit_msgs.msg import MoveItErrorCodes
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class AvoidanceTestMonitor(Node):
    """Nodo ausiliario per monitorare comandi velocità."""
    
    def __init__(self):
        super().__init__('avoidance_test_monitor')
        self.last_velocity_cmd = None
        self.velocity_sub = self.create_subscription(
            Float64MultiArray,
            '/fr3_velocity_controller/commands',
            self.velocity_callback,
            10
        )
        
    def velocity_callback(self, msg):
        self.last_velocity_cmd = msg.data


def main():
    print("🧪 ONLINE AVOIDANCE CONTROLLER - TEST DEMO")
    print("=" * 60)

    rclpy.init()

    try:
        # Client per movimenti
        client = FrankaMotionClient(timeout_sec=30.0)
        
        # Monitor per velocità avoidance
        monitor = AvoidanceTestMonitor()
        
        time.sleep(3.0)
        print("✅ Client e monitor pronti!")

        # ========== TEST 1: Movimento Home ==========
        print("\n" + "=" * 60)
        print("TEST 1: Movimento verso HOME")
        print("=" * 60)
        
        home_joints = [0.0, 0.0, -0.785, -2.356, 0.0, 1.571, 0.785]
        result = client.move_to_joint(home_joints, velocity_scaling=0.1, tolerance=0.02)
        
        if result.val != MoveItErrorCodes.SUCCESS:
            print(f"❌ Errore movimento home: {client._error_code_to_string(result.val)}")
            return
        print("✅ Robot in HOME")
        
        input("\n👉 Premi INVIO per TEST 2 (movimento vicino a ostacolo)...")

        # ========== TEST 2: Target vicino a ostacolo ==========
        print("\n" + "=" * 60)
        print("TEST 2: Target VICINO a ostacolo (verifica repulsione)")
        print("=" * 60)
        
        # Target strategico vicino all'ostacolo ma non in collisione
        target_near_obstacle = {
            "x": 0.2, "y": -0.5, "z": 0.2,
            "name": "Vicino ostacolo frontale"
        }
        
        print(f"\n➡️ Target: {target_near_obstacle['name']}")
        print(f"   Coordinate: x={target_near_obstacle['x']:.2f}, "
              f"y={target_near_obstacle['y']:.2f}, z={target_near_obstacle['z']:.2f}")
        
        pose = client.create_pose_stamped(
            x=target_near_obstacle["x"],
            y=target_near_obstacle["y"],
            z=target_near_obstacle["z"],
            qx=1.0, qy=0.0, qz=0.0, qw=0.0,
            frame_id="world"
        )
        
        result = client.move_to_pose(
            pose_target=pose,
            cartesian_motion=False,
            velocity_scaling=0.05  # MOLTO lento per vedere l'avoidance
        )
        
        if result.val == MoveItErrorCodes.SUCCESS:
            print(f"✅ Target raggiunto!")
            
            # Monitora comandi velocità avoidance
            rclpy.spin_once(monitor, timeout_sec=0.1)
            if monitor.last_velocity_cmd:
                vel_norm = sum(abs(v) for v in monitor.last_velocity_cmd)
                print(f"📊 Ultimo comando velocità avoidance: |q_dot| = {vel_norm:.4f} rad/s")
            else:
                print("⚠️ Nessun comando velocità rilevato")
        else:
            print(f"❌ Fallito: {client._error_code_to_string(result.val)}")

        input("\n👉 Premi INVIO per TEST 3 (target multipli)...")

        # ========== TEST 3: Target multipli ==========
        print("\n" + "=" * 60)
        print("TEST 3: Sequenza target multipli vicini a ostacoli")
        print("=" * 60)
        
        targets = [
            {"x": 0.35, "y": 0.3, "z": 0.7, "name": "Laterale destro"},
            {"x": 0.35, "y": -0.3, "z": 0.4, "name": "Laterale sinistro"},
            {"x": 0.5, "y": 0.0, "z": 0.6, "name": "Alto centrale"},
            {"x": 0.3, "y": 0.4, "z": 0.2, "name": "Alto centrale"},
            {"x": 0.3, "y": -0.4, "z": 0.6, "name": "Alto centrale"},
        ]

        for i, t in enumerate(targets, 1):
            print(f"\n[{i}/{len(targets)}] Target: {t['name']}")
            print(f"    Coordinate: x={t['x']:.2f}, y={t['y']:.2f}, z={t['z']:.2f}")
            
            pose = client.create_pose_stamped(
                x=t["x"], y=t["y"], z=t["z"],
                qx=1.0, qy=0.0, qz=0.0, qw=0.0,
                frame_id="world"
            )
            
            result = client.move_to_pose(
                pose_target=pose,
                cartesian_motion=False,
                velocity_scaling=0.1
            )
            
            if result.val == MoveItErrorCodes.SUCCESS:
                print(f"    ✅ Raggiunto")
                
                # Verifica comando avoidance
                rclpy.spin_once(monitor, timeout_sec=0.1)
                if monitor.last_velocity_cmd:
                    vel_norm = sum(abs(v) for v in monitor.last_velocity_cmd)
                    if vel_norm > 0.01:
                        print(f"    🛡️ Avoidance attivo: |q_dot| = {vel_norm:.4f} rad/s")
                    else:
                        print(f"    ℹ️ Nessuna repulsione necessaria")
            else:
                print(f"    ❌ Fallito: {client._error_code_to_string(result.val)}")
                break
            
            time.sleep(1.5)

        # ========== TEST 4: Ritorno Home ==========
        print("\n" + "=" * 60)
        print("TEST 4: Ritorno a HOME")
        print("=" * 60)
        
        result = client.move_to_joint(home_joints, velocity_scaling=0.1, tolerance=0.02)
        if result.val == MoveItErrorCodes.SUCCESS:
            print("✅ Robot tornato in HOME")
        else:
            print(f"❌ Errore ritorno: {client._error_code_to_string(result.val)}")

        print("\n" + "=" * 60)
        print("🎉 TEST COMPLETATO!")
        print("=" * 60)
        print("\n📊 RIEPILOGO:")
        print("   - Verifica visiva in RViz: il robot evita ostacoli?")
        print("   - Comandi velocità pubblicati: sì/no")
        print("   - Movimenti completati senza collisioni")
        print("\n💡 Suggerimenti tuning:")
        print("   - Aumenta repulsive_gain per repulsione più forte")
        print("   - Aumenta influence_distance per campo repulsivo più ampio")
        print("   - Riduci safety_margin per avvicinarsi di più")

    except KeyboardInterrupt:
        print("\n⏹️ Test interrotto dall'utente")
    except Exception as e:
        print(f"\n❌ Errore test: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n🏁 Shutdown...")
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()