#!/usr/bin/env python3
"""
Demo Script - Test Sistema Franka Motion Completo (Workspace Esteso)
===================================================================

Funzionalità:
1. Check iniziale se il robot è in home position
2. Se non in home, movimento verso home
3. Attesa conferma utente
4. Traiettorie cartesiane lunghe in tutto il workspace
5. Ritorno finale in home
"""

import rclpy
import time
import sys
import os

# Aggiunge ../scripts al path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from franka_motion_client import FrankaMotionClient
from moveit_msgs.msg import MoveItErrorCodes


def main():
    print("🎮 DEMO FRANKA MOTION CONTROL - WORKSPACE ESTESO")
    print("=" * 50)

    rclpy.init()

    try:
        # Inizializzazione client
        print("🚀 Inizializzazione Motion Client...")
        client = FrankaMotionClient(timeout_sec=45.0)
        time.sleep(2.0)

        print("✅ Client pronto!")

        # ============================================================
        # CHECK HOME POSITION
        # ============================================================
        home_joints = [0.0, 0.0, -0.785, -2.356, 0.0, 1.571, 0.785]  # Home desiderata
        tol = 0.05  # tolleranza rad

        current_joints = client.get_current_joints()
        if current_joints:
            print(f"🔧 Joint correnti: {[f'{j:.3f}' for j in current_joints]}")
            diff = [abs(c - h) for c, h in zip(current_joints, home_joints)]
            if all(d < tol for d in diff):
                print("✅ Robot già in home position!")
            else:
                print("➡️  Robot non in home, spostamento in corso...")
                result = client.move_to_joint(
                    joint_target=home_joints,
                    velocity_scaling=0.2,
                    tolerance=0.02
                )
                if result.val == MoveItErrorCodes.SUCCESS:
                    print("✅ Movimento verso home completato!")
                else:
                    print(f"❌ Movimento home fallito: {client._error_code_to_string(result.val)}")
                    return
        else:
            print("❌ Impossibile ottenere joint correnti")
            return

        # ============================================================
        # ATTESA CONFERMA UTENTE
        # ============================================================
        input("\n👉 Premi INVIO per far partire le traiettorie cartesiane...")

        # ============================================================
        # TRAIETTORIE CARTESIANE ESTESE
        # ============================================================
        print("\n📐 Esecuzione traiettorie cartesiane in tutto il workspace...")
        print("-" * 30)

        waypoints = [
            (0.5,  0.0, 0.5),   # avanti centro
            (0.3,  0.3, 0.4),   # avanti sinistra
            (0.3, -0.3, 0.4),   # avanti destra
            (0.6,  0.0, 0.3),   # avanti basso
            (0.4,  0.2, 0.6),   # in alto a sinistra
            (0.4, -0.2, 0.6),   # in alto a destra
            (0.5,  0.0, 0.5),   # ritorno al centro
        ]

        for i, (x, y, z) in enumerate(waypoints, start=1):
            target_pose = client.create_pose_stamped(
                x=x, y=y, z=z,
                qx=1.0, qy=0.0, qz=0.0, qw=0.0,
                frame_id="fr3_link0"
            )
            print(f"  📍 Movimento {i}: x={x:.2f}, y={y:.2f}, z={z:.2f}")
            result = client.move_to_pose(
                pose_target=target_pose,
                cartesian_motion=True,
                velocity_scaling=0.2
            )
            if result.val == MoveItErrorCodes.SUCCESS:
                print(f"    ✅ Movimento {i} completato")
            else:
                print(f"    ❌ Movimento {i} fallito")
                break
            time.sleep(2.0)

        # ============================================================
        # RITORNO A HOME
        # ============================================================
        print("\n🏠 Ritorno alla home position...")
        result = client.move_to_joint(
            joint_target=home_joints,
            velocity_scaling=0.2,
            tolerance=0.02
        )
        if result.val == MoveItErrorCodes.SUCCESS:
            print("✅ Robot tornato in home position")
        else:
            print(f"❌ Errore ritorno home: {client._error_code_to_string(result.val)}")

        print("\n🎉 DEMO COMPLETATO - Robot ha esplorato il workspace e ora è in home!")
        print("=" * 50)

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
