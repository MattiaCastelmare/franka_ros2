#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import numpy as np
import matplotlib.pyplot as plt

class EETrackingLogger(Node):
    def __init__(self):
        super().__init__("ee_tracking_logger")

        # --- LOG ---
        self.real_positions = []
        self.desired_positions = []
        self.timestamps = []

        # Sottoscrivi la stima EE reale (dal tuo nodo di stato)
        self.real_sub = self.create_subscription(
            PoseStamped,
            "/franka/ee_pose",
            self.callback_real,
            10
        )

        # Sottoscrivi EE desiderato (quello che invia la tua pipeline)
        self.des_sub = self.create_subscription(
            PoseStamped,
            "/franka/ee_desired",
            self.callback_desired,
            10
        )

        self.des_pose = None
        self.real_pose = None

        # timer per salvare dati
        self.timer = self.create_timer(0.01, self.save_data)

    def callback_real(self, msg):
        self.real_pose = msg

    def callback_desired(self, msg):
        self.des_pose = msg

    def save_data(self):
        if self.real_pose is None or self.des_pose is None:
            return

        self.timestamps.append(self.get_clock().now().nanoseconds * 1e-9)

        real = self.real_pose.pose.position
        des = self.des_pose.pose.position

        self.real_positions.append([real.x, real.y, real.z])
        self.desired_positions.append([des.x, des.y, des.z])

    def plot(self):
        t = np.array(self.timestamps)
        real = np.array(self.real_positions)
        des = np.array(self.desired_positions)

        fig, ax = plt.subplots(3, 1, figsize=(8, 10))

        labels = ["X", "Y", "Z"]
        for i in range(3):
            ax[i].plot(t, real[:, i], label=f"Real {labels[i]}")
            ax[i].plot(t, des[:, i], "--", label=f"Desired {labels[i]}")
            ax[i].grid()
            ax[i].legend()

        plt.tight_layout()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = EETrackingLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.plot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
