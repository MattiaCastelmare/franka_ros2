"""Publish smooth sinusoidal joint-velocity commands with warmup + cosine ramp.

Timeline:
  t < warmup_s          → publish offsets only (typically zeros)
  warmup_s ≤ t          → sinusoidal commands with cosine-ramp envelope
      tr = t - warmup_s
      envelope = 0.5*(1 - cos(π·tr/ramp_s))  while tr < ramp_s, then 1.0
      cmd[i]  = offsets[i] + envelope * amplitudes[i] * sin(2π·freq[i]·tr)

On shutdown (Ctrl-C) the node publishes zero-velocity commands for 0.5 s
at the configured rate before exiting cleanly.
"""

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

NUM_JOINTS = 7

DEFAULT_TOPIC = '/fr3_forward_velocity_controller/commands'
DEFAULT_RATE_HZ = 200.0
DEFAULT_AMPLITUDES = [0.0, 0.0, 0.0, 0.05, 0.05, 0.0, 0.0]
DEFAULT_FREQUENCIES = [0.0, 0.0, 0.0, 0.2, 0.2, 0.0, 0.0]
DEFAULT_OFFSETS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
DEFAULT_WARMUP_S = 2.0
DEFAULT_RAMP_S = 2.0


class SmoothVelocityCommander(Node):
    """Generate smooth sinusoidal velocity references for 7 joints."""

    def __init__(self):
        super().__init__('smooth_velocity_commander')

        # ---- Declare parameters -----------------------------------------
        self.declare_parameter('command_topic', DEFAULT_TOPIC)
        self.declare_parameter('rate_hz', DEFAULT_RATE_HZ)
        self.declare_parameter('amplitudes', DEFAULT_AMPLITUDES)
        self.declare_parameter('frequencies', DEFAULT_FREQUENCIES)
        self.declare_parameter('offsets', DEFAULT_OFFSETS)
        self.declare_parameter('warmup_s', DEFAULT_WARMUP_S)
        self.declare_parameter('ramp_s', DEFAULT_RAMP_S)

        # ---- Read parameters --------------------------------------------
        self.topic = self.get_parameter('command_topic').value
        self.rate_hz = self.get_parameter('rate_hz').value
        self.amplitudes = list(self.get_parameter('amplitudes').value)
        self.frequencies = list(self.get_parameter('frequencies').value)
        self.offsets = list(self.get_parameter('offsets').value)
        self.warmup_s = self.get_parameter('warmup_s').value
        self.ramp_s = self.get_parameter('ramp_s').value

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
        self._last_debug_sec = -1.0  # force first debug log at t≈0

        # ---- Startup log ------------------------------------------------
        self.get_logger().info(
            f'smooth_velocity_commander started\n'
            f'  topic  : {self.topic}\n'
            f'  rate   : {self.rate_hz} Hz\n'
            f'  warmup : {self.warmup_s} s\n'
            f'  ramp   : {self.ramp_s} s\n'
            f'  amp    : {self.amplitudes}\n'
            f'  freq   : {self.frequencies}\n'
            f'  offset : {self.offsets}')

    # -----------------------------------------------------------------
    def _timer_cb(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        msg = Float64MultiArray()

        if t < self.warmup_s:
            # Warmup phase: publish offsets only (typically zeros)
            msg.data = list(self.offsets)
        else:
            tr = t - self.warmup_s
            # Cosine ramp envelope: 0 → 1 over ramp_s seconds
            if self.ramp_s > 0.0 and tr < self.ramp_s:
                envelope = 0.5 * (1.0 - math.cos(math.pi * tr / self.ramp_s))
            else:
                envelope = 1.0

            data = []
            for i in range(NUM_JOINTS):
                if self.frequencies[i] == 0.0:
                    val = self.offsets[i]
                else:
                    val = (self.offsets[i]
                           + envelope
                           * self.amplitudes[i]
                           * math.sin(2.0 * math.pi * self.frequencies[i] * tr))
                data.append(val)
            msg.data = data

        self.pub.publish(msg)

        # Throttled debug log (~1 Hz)
        if t - self._last_debug_sec >= 1.0:
            self._last_debug_sec = t
            d = msg.data
            if t < self.warmup_s:
                phase = 'warmup'
                env_val = 0.0
            else:
                phase = 'active'
                tr_now = t - self.warmup_s
                if self.ramp_s > 0.0 and tr_now < self.ramp_s:
                    env_val = 0.5 * (1.0 - math.cos(math.pi * tr_now / self.ramp_s))
                else:
                    env_val = 1.0
            cmd_str = ', '.join(f'{v:.4f}' for v in d)
            self.get_logger().info(
                f'[t={t:.1f}s {phase} env={env_val:.3f}] cmd=[{cmd_str}]')


def main(args=None):
    rclpy.init(args=args)
    node = SmoothVelocityCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send zero velocities for 0.5 s at rate_hz then exit
        if rclpy.ok():
            stop_msg = Float64MultiArray()
            stop_msg.data = [0.0] * NUM_JOINTS
            period = 1.0 / node.rate_hz
            stop_end = time.monotonic() + 0.5
            node.get_logger().info(
                'Shutdown: publishing zero velocities for 0.5 s …')
            while time.monotonic() < stop_end:
                try:
                    node.pub.publish(stop_msg)
                    time.sleep(period)
                except Exception:
                    break
            node.get_logger().info('Zero-velocity ramp complete — exiting.')
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
