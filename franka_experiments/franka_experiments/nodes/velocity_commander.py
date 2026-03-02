"""Publish sinusoidal joint-velocity commands to a ForwardCommandController."""

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

NUM_JOINTS = 7

DEFAULT_TOPIC = '/fr3_forward_velocity_controller/commands'
DEFAULT_RATE_HZ = 100.0
DEFAULT_AMPLITUDES = [0.0, 0.0, 0.0, 0.3, 0.3, 0.0, 0.0]
DEFAULT_FREQUENCIES = [0.0, 0.0, 0.0, 0.5, 0.5, 0.0, 0.0]
DEFAULT_OFFSETS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class VelocityCommander(Node):
    """Generate sinusoidal velocity references for 7 joints."""

    def __init__(self):
        super().__init__('velocity_commander')

        # ---- Declare parameters -----------------------------------------
        self.declare_parameter('command_topic', DEFAULT_TOPIC)
        self.declare_parameter('rate_hz', DEFAULT_RATE_HZ)
        self.declare_parameter('amplitudes', DEFAULT_AMPLITUDES)
        self.declare_parameter('frequencies', DEFAULT_FREQUENCIES)
        self.declare_parameter('offsets', DEFAULT_OFFSETS)

        # ---- Read parameters --------------------------------------------
        self.topic = self.get_parameter('command_topic').value
        self.rate_hz = self.get_parameter('rate_hz').value
        self.amplitudes = list(self.get_parameter('amplitudes').value)
        self.frequencies = list(self.get_parameter('frequencies').value)
        self.offsets = list(self.get_parameter('offsets').value)

        # ---- Validate lengths -------------------------------------------
        for name, vec in [('amplitudes', self.amplitudes),
                          ('frequencies', self.frequencies),
                          ('offsets', self.offsets)]:
            if len(vec) != NUM_JOINTS:
                self.get_logger().error(
                    f'Parameter "{name}" must have {NUM_JOINTS} elements, '
                    f'got {len(vec)}. Shutting down.')
                raise SystemExit(1)

        # ---- Publisher & timer ------------------------------------------
        self.pub = self.create_publisher(Float64MultiArray, self.topic, 10)
        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(period, self._timer_cb)
        self.t0 = self.get_clock().now()
        self._last_debug_sec = 0.0

        # ---- Startup log ------------------------------------------------
        self.get_logger().info(
            f'velocity_commander started\n'
            f'  topic : {self.topic}\n'
            f'  rate  : {self.rate_hz} Hz\n'
            f'  amp   : {self.amplitudes}\n'
            f'  freq  : {self.frequencies}\n'
            f'  offset: {self.offsets}')

    # -----------------------------------------------------------------
    def _timer_cb(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        msg = Float64MultiArray()
        data = []
        for i in range(NUM_JOINTS):
            if self.frequencies[i] == 0.0:
                val = self.offsets[i]
            else:
                val = (self.offsets[i]
                       + self.amplitudes[i]
                       * math.sin(2.0 * math.pi * self.frequencies[i] * t))
            data.append(val)
        msg.data = data
        self.pub.publish(msg)

        # Throttled debug log (~1 Hz)
        if t - self._last_debug_sec >= 1.0:
            self._last_debug_sec = t
            self.get_logger().info(
                f'[t={t:.1f}s] cmd[0:3]='
                f'[{data[0]:.4f}, {data[1]:.4f}, {data[2]:.4f}]')


def main(args=None):
    rclpy.init(args=args)
    node = VelocityCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send zero velocities a few times so the robot stops cleanly
        if rclpy.ok():
            stop_msg = Float64MultiArray()
            stop_msg.data = [0.0] * NUM_JOINTS
            for _ in range(3):
                try:
                    node.pub.publish(stop_msg)
                    time.sleep(0.05)
                except Exception:
                    break
            node.get_logger().info('Published zero velocities — shutting down.')
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
