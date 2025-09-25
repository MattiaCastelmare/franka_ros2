#!/usr/bin/env python3
"""
Collision Test Demo - con Franka Motion Client
==============================================

Script per testare il collision checking con MoveIt, usando il Motion Client.
Sequenza:
1. Movimento a HOME
2. Tentativo di movimento dentro ostacolo
3. Verifica collision checking
4. Movimenti sicuri (collision-free)

Uso:
    python3 collision_test_demo.py
"""

import rclpy
import time
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped
import os
import sys
# Aggiunge ../scripts al path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from franka_motion_client import FrankaMotionClient


def run_demo(client: FrankaMotionClient):
    print("\n🤖 COLLISION TEST DEMO")
    print("=" * 60)

    # STEP 1: HOME
    print("\nSTEP 1: Movimento in HOME")
    home_joints = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    result = client.move_to_joint(home_joints, velocity_scaling=0.2)
    if result.val == MoveItErrorCodes.SUCCESS:
        print("✅ Robot in HOME")
    else:
        print("❌ Movimento HOME fallito")
    time.sleep(2.0)

    # STEP 2: Tentativo collisione
    print("\nSTEP 2: Movimento dentro ostacolo (collision check)")
    collision_pose = client.create_pose_stamped(
        x=0.5, y=0.0, z=0.4,
        qx=1.0, qy=0.0, qz=0.0, qw=0.0,
        frame_id="world"
    )
    result = client.move_to_pose(collision_pose, cartesian_motion=False, velocity_scaling=0.1)
    if result.val == MoveItErrorCodes.SUCCESS:
        print("⚠️ Planner ha trovato un percorso (forse aggira l’ostacolo)")
    else:
        print("✅ Collisione rilevata, pianificazione fallita come previsto")
    time.sleep(2.0)

    # STEP 3: Ritorno HOME
    print("\nSTEP 3: Ritorno in HOME")
    result = client.move_to_joint(home_joints, velocity_scaling=0.2)
    if result.val == MoveItErrorCodes.SUCCESS:
        print("✅ Robot in HOME")
    else:
        print("❌ Ritorno HOME fallito")
    time.sleep(2.0)

    # STEP 4: Movimenti collision-free
    print("\nSTEP 4: Movimenti sicuri vicino all'ostacolo")
    safe_targets = [
        {"x": 0.5, "y": 0.0, "z": 0.6, "name": "sopra ostacolo"},
        {"x": 0.3, "y": -0.3, "z": 0.4, "name": "a lato ostacolo"},
        {"x": 0.35, "y": 0.0, "z": 0.4, "name": "davanti ostacolo"},
    ]

    for target in safe_targets:
        print(f"\n➡️ Movimento verso posizione sicura: {target['name']}")
        pose = client.create_pose_stamped(
            x=target["x"], y=target["y"], z=target["z"],
            qx=1.0, qy=0.0, qz=0.0, qw=0.0,
            frame_id="world"
        )
        result = client.move_to_pose(pose, cartesian_motion=False, velocity_scaling=0.2)
        if result.val == MoveItErrorCodes.SUCCESS:
            print(f"✅ Raggiunta posizione sicura: {target['name']}")
        else:
            print(f"❌ Fallito movimento a {target['name']}")

    print("\n🎉 DEMO COMPLETATA!")


def main(args=None):
    rclpy.init(args=args)
    client = FrankaMotionClient(timeout_sec=45.0)

    try:
        run_demo(client)
    except KeyboardInterrupt:
        print("\n⏹️ Interrotto dall'utente")
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
