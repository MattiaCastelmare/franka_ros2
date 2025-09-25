#!/usr/bin/env python3
"""
Demo Script - Test Sistema Franka Motion Completo
================================================

Script interattivo per testare tutte le funzionalità:
1. Movimento joint space a home position
2. Movimento pose target con IK
3. Movimento cartesiano  
4. Query stato corrente robot
5. Sequenza movimenti coordinati

Uso:
    python3 franka_motion_demo.py
"""

import rclpy
import time
import math
import sys
import os

# Aggiunge ../scripts al path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from franka_motion_client import FrankaMotionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes


def main():
    """Demo completo delle funzionalità motion control."""
    
    print("🎮 DEMO FRANKA MOTION CONTROL")
    print("=" * 50)
    
    rclpy.init()
    
    try:
        # Inizializzazione client
        print("🚀 Inizializzazione Motion Client...")
        client = FrankaMotionClient(timeout_sec=45.0)
        time.sleep(2.0)  # Attesa inizializzazione
        
        print("✅ Client pronto! Inizio demo...")
        print()
        
        # ============================================================
        # TEST 1: Movimento a Home Position
        # ============================================================
        
        print("🏠 TEST 1: Movimento a Home Position")
        print("-" * 30)
        
        home_joints = [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0]  # Home classica FR3
        
        result = client.move_to_joint(
            joint_target=home_joints,
            velocity_scaling=0.2,  # 20% velocità per sicurezza
            tolerance=0.02
        )
        
        if result.val == MoveItErrorCodes.SUCCESS:
            print("✅ Movimento home completato!")
        else:
            print(f"❌ Movimento home fallito: {client._error_code_to_string(result.val)}")
            
        time.sleep(2.0)
        
        # ============================================================
        # TEST 2: Query Stato Corrente
        # ============================================================
        
        print()
        print("📊 TEST 2: Query Stato Robot Corrente")
        print("-" * 30)
        
        current_joints = client.get_current_joints()
        if current_joints:
            print(f"🔧 Joint correnti: {[f'{j:.3f}' for j in current_joints]}")
        else:
            print("❌ Impossibile ottenere joint correnti")
            
        current_pose = client.get_current_pose()
        if current_pose:
            pos = current_pose.pose.position
            print(f"📍 Pose corrente: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}")
        else:
            print("❌ Impossibile ottenere pose corrente")
            
        time.sleep(2.0)
        
        # ============================================================
        # TEST 3: Movimento Pose Target (Joint Space Planning)
        # ============================================================
        
        print()
        print("🎯 TEST 3: Movimento Pose Target (IK + Joint Planning)")
        print("-" * 30)
        
        # Crea pose target: avanti e leggermente in alto rispetto a home
        target_pose = client.create_pose_stamped(
            x=0.4, y=0.0, z=0.5,  # 40cm avanti, 50cm alto
            qx=1.0, qy=0.0, qz=0.0, qw=0.0,  # Orientamento punta in basso  
            frame_id="fr3_link0"
        )
        
        print(f"  Target: x={target_pose.pose.position.x}, "
              f"y={target_pose.pose.position.y}, z={target_pose.pose.position.z}")
        
        result = client.move_to_pose(
            pose_target=target_pose,
            cartesian_motion=False,  # Planning joint space
            velocity_scaling=0.15,
            tolerance_position=0.005  # 5mm tolerance
        )
        
        if result.val == MoveItErrorCodes.SUCCESS:
            print("✅ Movimento pose completato!")
        else:
            print(f"❌ Movimento pose fallito: {client._error_code_to_string(result.val)}")
            
        time.sleep(3.0)
        
        # ============================================================
        # TEST 4: Movimento Cartesiano
        # ============================================================
        
        print()
        print("📐 TEST 4: Movimento Cartesiano")
        print("-" * 30)
        
        # Movimento cartesiano: spostamento laterale di 10cm
        cartesian_pose = client.create_pose_stamped(
            x=0.4, y=0.1, z=0.5,  # Sposta lateralmente 10cm
            qx=1.0, qy=0.0, qz=0.0, qw=0.0,
            frame_id="fr3_link0"
        )
        
        print(f"  Cartesian target: spostamento laterale a y=0.1")
        
        result = client.move_to_pose(
            pose_target=cartesian_pose,
            cartesian_motion=True,  # Planning cartesiano
            velocity_scaling=0.1    # Più lento per sicurezza
        )
        
        if result.val == MoveItErrorCodes.SUCCESS:
            print("✅ Movimento cartesiano completato!")
        else:
            print(f"❌ Movimento cartesiano fallito: {client._error_code_to_string(result.val)}")
            
        time.sleep(3.0)
        
        # ============================================================
        # TEST 5: Sequenza Coordinata
        # ============================================================
        
        print()
        print("🎭 TEST 5: Sequenza Movimenti Coordinata")
        print("-" * 30)
        
        # Sequenza: Pick-like movement simulation
        pick_positions = [
            [0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.0],   # Pre-grasp
            [0.2, -0.5, 0.0, -2.2, 0.0, 1.7, 0.0],   # Approach  
            [0.2, -0.3, 0.0, -2.0, 0.0, 1.5, 0.0],   # Retreat
        ]
        
        for i, joints in enumerate(pick_positions):
            print(f"  Movimento {i+1}/3...")
            
            result = client.move_to_joint(
                joint_target=joints,
                velocity_scaling=0.15,
                tolerance=0.015
            )
            
            if result.val == MoveItErrorCodes.SUCCESS:
                print(f"    ✅ Movimento {i+1} completato")
            else:
                print(f"    ❌ Movimento {i+1} fallito")
                break
                
            time.sleep(1.5)
            
        # Ritorna a home
        print("  Ritorno a home...")
        result = client.move_to_joint(home_joints, velocity_scaling=0.2)
        
        if result.val == MoveItErrorCodes.SUCCESS:
            print("✅ Sequenza completata - robot in home position")
        else:
            print("❌ Errore ritorno home")
            
        print()
        print("🎉 DEMO COMPLETATO!")
        print("=" * 50)
        print("Tutti i test eseguiti. Il sistema Franka Motion Control è operativo!")
        
    except KeyboardInterrupt:
        print("\n⏹️ Demo interrotto dall'utente")
    except Exception as e:
        print(f"\n❌ Errore demo: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n🏁 Shutdown demo...")
        rclpy.shutdown()


if __name__ == '__main__':
    main()