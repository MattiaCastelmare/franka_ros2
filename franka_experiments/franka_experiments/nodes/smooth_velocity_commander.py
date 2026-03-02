"""Publish smooth sinusoidal joint-velocity commands with warmup + cosine ramp.

Timeline:
  t < warmup_s          → publish offsets only (typically zeros)
  warmup_s ≤ t          → sinusoidal commands with cosine-ramp envelope
      tr = t - warmup_s
      envelope = 0.5*(1 - cos(π·tr/ramp_s))  while tr < ramp_s, then 1.0
      cmd[i]  = offsets[i] + envelope * amplitudes[i] * sin(2π·freq[i]·tr)

On shutdown (SIGINT / SIGTERM / Ctrl-C):
  1. The timer callback switches to a *stopping* state.
  2. Zero-velocity messages are published for ~0.5 s inside the timer.
  3. After the stop window expires the timer calls rclpy.shutdown(),
     which unblocks spin() and lets main() tear down cleanly.
  → The process returns to the shell prompt within ~1 s, with no
    blocking loops in the ``finally`` block.
"""

import math
import os
import time

import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

NUM_JOINTS = 7
CONTROLLER_NAME = 'fr3_forward_velocity_controller'


def _resolve_default_topic(robot_key='ROBOT1'):
    """Auto-detect namespace from franka.config.yaml → build command topic.

    Reads  franka_bringup/config/franka.config.yaml  (installed share),
    extracts the *namespace* field for *robot_key*, and returns
      /<namespace>/<controller>/commands   (if namespace is non-empty)
      /<controller>/commands               (if namespace is empty / missing)

    Falls back silently to the non-namespaced topic on any error.
    """
    topic_suffix = f'{CONTROLLER_NAME}/commands'
    try:
        from ament_index_python.packages import get_package_share_directory
        bringup_share = get_package_share_directory('franka_bringup')
        config_path = os.path.join(bringup_share, 'config', 'franka.config.yaml')
        with open(config_path, 'r') as fh:
            config = yaml.safe_load(fh)
        if config and robot_key in config:
            ns = str(config[robot_key].get('namespace', '')).strip()
            if ns:
                return f'/{ns}/{topic_suffix}'
    except Exception:  # noqa: BLE001
        pass
    return f'/{topic_suffix}'


DEFAULT_TOPIC = _resolve_default_topic()
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

        # ---- Stopping state machine ------------------------------------
        self._stopping = False
        self._stop_end_time = 0.0        # time.monotonic() deadline

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

        # ---- Prebuilt stop message (avoid allocation in hot path) ------
        self._stop_msg = Float64MultiArray()
        self._stop_msg.data = [0.0] * NUM_JOINTS

        # ---- Startup log ------------------------------------------------
        topic_note = ('(auto-resolved from franka.config.yaml)'
                      if self.topic == DEFAULT_TOPIC
                      else '(overridden via parameter)')
        self.get_logger().info(
            f'smooth_velocity_commander started\n'
            f'  topic  : {self.topic}  {topic_note}\n'
            f'  rate   : {self.rate_hz} Hz\n'
            f'  warmup : {self.warmup_s} s\n'
            f'  ramp   : {self.ramp_s} s\n'
            f'  amp    : {self.amplitudes}\n'
            f'  freq   : {self.frequencies}\n'
            f'  offset : {self.offsets}')

    # -----------------------------------------------------------------
    def request_stop(self, stop_duration_s: float = 0.5):
        """Enter the *stopping* state: publish zeros for *stop_duration_s*.

        Safe to call multiple times (idempotent).
        """
        if self._stopping:
            return
        self._stopping = True
        self._stop_end_time = time.monotonic() + stop_duration_s
        self.get_logger().info(
            f'Stopping: publishing zero velocities for {stop_duration_s} s')

    # -----------------------------------------------------------------
    def _timer_cb(self):
        # ---- Stopping state: publish zeros, then trigger shutdown ----
        if self._stopping:
            try:
                self.pub.publish(self._stop_msg)
            except Exception:
                pass
            if time.monotonic() >= self._stop_end_time:
                self.get_logger().info('Stop complete, shutting down')
                self.timer.cancel()
                try:
                    rclpy.shutdown()
                except Exception:
                    pass
            return

        # ---- Normal operation: warmup + cosine-ramp sinusoids -------
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
    rclpy.init(args=args, signal_handler_options=rclpy.SignalHandlerOptions.NO)
    node = SmoothVelocityCommander()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # CTRL-C: enter stopping state, then let spin() continue
        # so the timer can publish zeros and call rclpy.shutdown().
        node.request_stop()
        try:
            rclpy.spin(node)
        except Exception:
            pass
    except Exception as exc:
        try:
            node.get_logger().error(f'Unexpected error: {exc}')
        except Exception:
            pass
    finally:
        # Minimal teardown — no blocking loops, no time.sleep()
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
