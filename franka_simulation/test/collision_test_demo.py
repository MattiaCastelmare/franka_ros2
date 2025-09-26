#!/usr/bin/env python3
"""
Explorative Collision-Free Demo - Franka Motion
================================================

Sequenza:
1. Sposta in HOME
2. Partenza verso target “dietro” l’ostacolo
3. Altri target audaci
4. Ritorno in HOME
"""

import rclpy
import time
import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from franka_motion_client import FrankaMotionClient
from moveit_msgs.msg import MoveItErrorCodes


def main():
    print("🔍 DEMO EXPLORATIVE COLLISION-FREE")
    print("=" * 60)

    rclpy.init()

    try:
        client = FrankaMotionClient(timeout_sec=30.0)
        time.sleep(2.0)
        print("✅ Client pronto!")

        # HOME position
        home_joints = [0.0, 0.0, -0.785, -2.356, 0.0, 1.571, 0.785]
        result = client.move_to_joint(home_joints, velocity_scaling=0.2, tolerance=0.02)
        if result.val != MoveItErrorCodes.SUCCESS:
            print(f"❌ Errore movimento home: {client._error_code_to_string(result.val)}")
            return
        print("✅ Robot in HOME")

        input("\n👉 Premi INVIO per partire con i target esplorativi...")

        # Lista di target “dietro l’ostacolo” o “di passaggio difficile”
        targets = [
            {"x": 0.6, "y": 0.0, "z": 0.7, "name": "dietro ostacolo avanti"},      # direttamente avanti ma sopra
            {"x": 0.55, "y": 0.25, "z": 0.6, "name": "a destra dietro ostacolo"},  # spostato lateralmente
            {"x": 0.55, "y": -0.25, "z": 0.2, "name": "a sinistra dietro ostacolo"},# spostato lateralmente opposto
            {"x": 0.45, "y": 0.0, "z": 0.1, "name": "sopra ostacolo"},               # sopra centrale
        ]

        for t in targets:
            print(f"\n➡️ Target “ambizioso”: {t['name']} → x={t['x']:.2f}, y={t['y']:.2f}, z={t['z']:.2f}")
            pose = client.create_pose_stamped(
                x=t["x"], y=t["y"], z=t["z"],
                qx=1.0, qy=0.0, qz=0.0, qw=0.0,
                frame_id="world"
            )
            result = client.move_to_pose(
                pose_target=pose,
                cartesian_motion=False,    # planning in joint space con collision checking
                velocity_scaling=0.2
            )
            if result.val == MoveItErrorCodes.SUCCESS:
                print(f"✅ {t['name']} raggiunto senza collisioni")
            else:
                print(f"❌ Fallito {t['name']}: {client._error_code_to_string(result.val)}")
                break
            time.sleep(2.0)

        # Ritorno in HOME
        print("\n🏠 Ritorno a HOME...")
        result = client.move_to_joint(home_joints, velocity_scaling=0.2, tolerance=0.02)
        if result.val == MoveItErrorCodes.SUCCESS:
            print("✅ Robot tornato in HOME")
        else:
            print(f"❌ Errore ritorno home: {client._error_code_to_string(result.val)}")

        print("\n🎉 DEMO ESPLORATIVO COMPLETATO!")
        print("=" * 60)

    except KeyboardInterrupt:
        print("\n⏹️ Demo interrotto dall’utente")
    except Exception as e:
        print(f"\n❌ Errore demo: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n🏁 Shutdown...")
        rclpy.shutdown()


if __name__ == '__main__':
    main()
