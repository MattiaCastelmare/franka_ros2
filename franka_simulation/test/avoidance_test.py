#!/usr/bin/env python3
"""
=== ONLINE AVOIDANCE CONTROLLER - WAYPOINT STRESS TEST ===

Obiettivo:
- MoveIt deve SEMPRE pianificare un path
- Il path attraversa zone vicine agli ostacoli (ma NON in collisione)
- L'online collision avoidance DEVE intervenire durante l'esecuzione
- Nessun blocco, nessun errore MoveIt

Strategia:
    HOME → WP1 (sopra ostacolo) → WP2 (laterale ostacolo) → TARGET sicuro
"""

import rclpy
import time
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from rclpy.node import Node
from moveit_msgs.msg import MoveItErrorCodes
from std_msgs.msg import Float64MultiArray
from franka_motion_client import FrankaMotionClient


# ============================================================
# MONITOR comandi /avoidance/velocity
# ============================================================

class AvoidanceMonitor(Node):
    def __init__(self):
        super().__init__('avoidance_monitor')
        self.last_cmd = None
        self.create_subscription(
            Float64MultiArray,
            "/avoidance/velocity",
            self.cb,
            10
        )
    def cb(self, msg):
        self.last_cmd = msg.data


# ============================================================
# Traiettorie
# ============================================================

def create_pose(client, x, y, z):
    return client.create_pose_stamped(
        x=x, y=y, z=z,
        qx=1.0, qy=0.0, qz=0.0, qw=0.0,
        frame_id="world"
    )


# ============================================================
# MAIN
# ============================================================

def main():
    print("\n🧪 === ONLINE AVOIDANCE WAYPOINT TEST ===\n")

    rclpy.init()

    client = FrankaMotionClient(timeout_sec=30.0)
    monitor = AvoidanceMonitor()

    time.sleep(2)

    # ============================================================
    # HOME
    # ============================================================

    HOME = [0.0, 0.0, -0.785, -2.356, 0.0, 1.571, 0.785]
    print("➡️  Movimento in HOME...")
    res = client.move_to_joint(HOME, velocity_scaling=0.15)
    print("   ✔ HOME OK\n")

    input("Premi INVIO per iniziare il percorso con Avoidance...")

    # ============================================================
    # WAYPOINTS DEFINITI IN BASE AI TUOI OSTACOLI
    # Ostacolo rosso:  (0.5, -0.3)
    # Ostacolo giallo: (0.4,  0.3)
    # ============================================================

    trajectories = [

        {
            "name": "Percorso vicino al box ROSSO",
            "wps": [
                # WP1 = sopra il box rosso
                (0.50, -0.30, 0.70),
                # WP2 = lato del box rosso
                (0.45, -0.50, 0.50),
            ],
            "target": (0.55, -0.10, 0.55),
        },

        {
            "name": "Percorso vicino al box GIALLO",
            "wps": [
                # WP1 = sopra il box giallo
                (0.40, 0.30, 0.70),
                # WP2 = lato del box giallo
                (0.30, 0.55, 0.50),
            ],
            "target": (0.50, 0.15, 0.60),
        },
    ]

    # ============================================================
    # LOOP PRINCIPALE
    # ============================================================

    for N, traj in enumerate(trajectories, 1):
        print(f"\n🔵 Sequenza {N}: {traj['name']}")
        print("------------------------------------------------")

        # WAYPOINT 1
        pose_wp1 = create_pose(client, *traj["wps"][0])
        print(f"  ➤ Muovo verso WP1: {traj['wps'][0]}")

        res = client.move_to_pose(pose_wp1, velocity_scaling=0.10)
        print(f"    • MoveIt = {client._error_code_to_string(res.val)}")

        # WAYPOINT 2
        pose_wp2 = create_pose(client, *traj["wps"][1])
        print(f"  ➤ Muovo verso WP2: {traj['wps'][1]}")

        res = client.move_to_pose(pose_wp2, velocity_scaling=0.07)
        print(f"    • MoveIt = {client._error_code_to_string(res.val)}")

        # TARGET FINALE
        pose_target = create_pose(client, *traj["target"])
        print(f"  ➤ Muovo verso TARGET: {traj['target']}")

        res = client.move_to_pose(pose_target, velocity_scaling=0.07)
        print(f"    • MoveIt = {client._error_code_to_string(res.val)}")

        # ====================================================
        # LETTURA AVOIDANCE DURANTE L'ESECUZIONE
        # ====================================================
        print("   🛡️  Analizzo avoidance...")

        for _ in range(30):
            rclpy.spin_once(monitor, timeout_sec=0.05)
            if monitor.last_cmd:
                norm = sum(abs(v) for v in monitor.last_cmd)
                print(f"     → |qdot_avoid| = {norm:.4f}")
            time.sleep(0.05)

    # ============================================================
    # RITORNO HOME
    # ============================================================

    print("\n🏠 Ritorno in HOME...")
    client.move_to_joint(HOME, velocity_scaling=0.15)
    print("✔ Test completato!\n")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
