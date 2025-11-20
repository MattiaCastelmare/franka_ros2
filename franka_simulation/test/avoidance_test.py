#!/usr/bin/env python3
"""
ONLINE AVOIDANCE CONTROLLER - COMBINED TEST
==========================================

Usa ESATTAMENTE i punti del primo test funzionante:
- Target vicino all’ostacolo frontale (Test 2)
- Sequenza di target multipli (Test 3)
- MoveIt deve pianificare SEMPRE
- L’online avoidance deve intervenire durante l’esecuzione
"""

import rclpy
import time
import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from moveit_msgs.msg import MoveItErrorCodes
from franka_motion_client import FrankaMotionClient


# ============================================================
# MONITOR AVOIDANCE VELOCITY
# ============================================================

class AvoidanceMonitor(Node):
    """Legge i comandi di velocità del controller avoidance."""

    def __init__(self):
        super().__init__("avoidance_monitor")
        self.last_cmd = None

        self.create_subscription(
            Float64MultiArray,
            "/fr3_velocity_controller/commands",
            self.cb,
            10
        )

    def cb(self, msg):
        self.last_cmd = msg.data


# ============================================================
# UTILITY PER CREARE POSE
# ============================================================

def make_pose(client, x, y, z):
    return client.create_pose_stamped(
        x=x, y=y, z=z,
        qx=1.0, qy=0.0, qz=0.0, qw=0.0,
        frame_id="world"
    )


# ============================================================
# MAIN
# ============================================================

def main():
    print("\n🧪 === ONLINE AVOIDANCE CONTROLLER — COMBINED TEST ===\n")

    rclpy.init()

    client = FrankaMotionClient(timeout_sec=30.0)
    monitor = AvoidanceMonitor()

    time.sleep(2.0)

    # ============================================================
    # HOME
    # ============================================================

    HOME = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

    print("➡️  Movimento in HOME...")
    res = client.move_to_joint(HOME, velocity_scaling=0.12)

    if res.val != MoveItErrorCodes.SUCCESS:
        print(f"❌ Errore HOME: {client._error_code_to_string(res.val)}")
        return

    print("   ✔ HOME OK\n")
    input("Premi INVIO per iniziare il test...\n")

    # ============================================================
    # TEST 2 — Target vicino all’ostacolo (dal tuo test originale)
    # ============================================================

    print("\n======================================================")
    print("TEST 2 — Target vicino a ostacolo (repulsione)")
    print("======================================================\n")

    target_close = {
        "x": 0.20, "y": -0.50, "z": 0.20,
        "name": "Target vicino ostacolo frontale"
    }

    pose_close = make_pose(client, target_close["x"], target_close["y"], target_close["z"])

    print(f"➡️  Target: {target_close['name']}")
    print(f"    (x={target_close['x']:.2f}, y={target_close['y']:.2f}, z={target_close['z']:.2f})")

    res = client.move_to_pose(
        pose_close,
        cartesian_motion=False,
        velocity_scaling=0.05
    )

    if res.val == MoveItErrorCodes.SUCCESS:
        print("   ✔ Raggiunto (MoveIt)")

        # Lettura comando avoidance
        rclpy.spin_once(monitor, timeout_sec=0.1)
        if monitor.last_cmd:
            norm = sum(abs(v) for v in monitor.last_cmd)
            print(f"   🛡️  Avoidance attivo → |q_dot| = {norm:.4f}")
        else:
            print("   ⚠️ Nessun comando avoidance rilevato")
    else:
        print(f"   ❌ Fallito: {client._error_code_to_string(res.val)}")

    input("\nPremi INVIO per TEST 3 (target multipli)...")

    # ============================================================
    # TEST 3 — Target multipli vicini ostacoli (dal tuo test originale)
    # ============================================================

    print("\n======================================================")
    print("TEST 3 — Sequenza target multipli")
    print("======================================================")

    targets = [
        {"x": 0.1, "y": 0.40, "z": 0.40, "name": "Laterale destro"},
        {"x": 0.1, "y": -0.40, "z": 0.10, "name": "Laterale sinistro"},
        {"x": 0.1, "y": 0.40, "z": 0.40, "name": "Alto centrale"},
        {"x": 0.1, "y": -0.40, "z": 0.10, "name": "Basso destro"},
        {"x": 0.1, "y": 0.40, "z": 0.40, "name": "Alto sinistro"}
    ]

    for i, t in enumerate(targets, 1):

        pose_t = make_pose(client, t["x"], t["y"], t["z"])
        print(f"\n[{i}/{len(targets)}] ➡️  Target: {t['name']}")
        print(f"     (x={t['x']:.2f}, y={t['y']:.2f}, z={t['z']:.2f})")

        res = client.move_to_pose(
            pose_t,
            cartesian_motion=False,
            velocity_scaling=0.10
        )

        if res.val == MoveItErrorCodes.SUCCESS:
            print("     ✔ Raggiunto")

            # controlla avoidance
            rclpy.spin_once(monitor, timeout_sec=0.1)
            if monitor.last_cmd:
                vel_norm = sum(abs(v) for v in monitor.last_cmd)

                if vel_norm > 0.01:
                    print(f"     🛡️ Avoidance → |q_dot| = {vel_norm:.4f}")
                else:
                    print("     ℹ️ Nessuna repulsione necessaria")
        else:
            print(f"     ❌ Fallito: {client._error_code_to_string(res.val)}")
            break

        time.sleep(1.0)

    # ============================================================
    # HOME FINALE
    # ============================================================

    print("\n======================================================")
    print("TEST 4 — Ritorno in HOME")
    print("======================================================")

    res = client.move_to_joint(HOME, velocity_scaling=0.12)
    if res.val == MoveItErrorCodes.SUCCESS:
        print("✔ Robot tornato in HOME")
    else:
        print(f"❌ Errore ritorno: {client._error_code_to_string(res.val)}")

    print("\n🎉 TEST COMPLETATO!")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
