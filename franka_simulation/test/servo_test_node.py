#!/usr/bin/env python3
"""
Servo Test Node
===============

Nodo per testare MoveIt Servo con vari pattern di movimento.
Utile per validare collision checking e velocity scaling.

Comandi disponibili:
- Twist cartesiano (linear/angular)
- Joint velocity commands
- Pattern predefiniti (circle, line, etc.)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from moveit_servo_msgs.msg import ServoStatus
from std_srvs.srv import SetBool

import math
import time
from enum import Enum


class ServoTestPattern(Enum):
    """Pattern di movimento predefiniti"""
    IDLE = 0
    LINEAR_X = 1
    LINEAR_Y = 2
    LINEAR_Z = 3
    CIRCULAR_XY = 4
    JOINT_SINE = 5
    APPROACH_TEST = 6  # Per test collision


class ServoTestNode(Node):
    """Nodo per testare MoveIt Servo"""
    
    def __init__(self):
        super().__init__('servo_test_node')
        
        # Parametri configurabili
        self.declare_parameter('test_pattern', 'idle')
        self.declare_parameter('velocity_scale', 0.1)  # 0.0-1.0
        self.declare_parameter('command_rate', 50.0)  # Hz
        
        self.test_pattern = ServoTestPattern[
            self.get_parameter('test_pattern').value.upper()
        ]
        self.velocity_scale = self.get_parameter('velocity_scale').value
        self.command_rate = self.get_parameter('command_rate').value
        
        # QoS per real-time (important for Servo)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Publishers
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_server/delta_twist_cmds',
            qos_profile
        )
        
        self.joint_jog_pub = self.create_publisher(
            JointJog,
            '/servo_server/delta_joint_cmds',
            qos_profile
        )
        
        # Subscriber per Servo status
        self.servo_status_sub = self.create_subscription(
            ServoStatus,
            '/servo_server/status',
            self.servo_status_callback,
            10
        )
        
        # State
        self.last_servo_status = None
        self.is_servo_active = False
        self.collision_detected = False
        self.phase = 0.0  # Per pattern circolari/sinusoidali
        
        # Timer per pubblicazione comandi
        self.command_timer = self.create_timer(
            1.0 / self.command_rate,
            self.command_callback
        )
        
        self.get_logger().info(f"🎮 Servo Test Node started")
        self.get_logger().info(f"  • Pattern: {self.test_pattern.name}")
        self.get_logger().info(f"  • Velocity scale: {self.velocity_scale}")
        self.get_logger().info(f"  • Command rate: {self.command_rate} Hz")
        
    def servo_status_callback(self, msg: ServoStatus):
        """Callback per stato Servo"""
        self.last_servo_status = msg
        
        # Check collision warning
        if msg.code == ServoStatus.COLLISION_DETECTED:
            if not self.collision_detected:
                self.get_logger().warn("⚠️ COLLISION DETECTED - Servo stopped")
                self.collision_detected = True
        else:
            self.collision_detected = False
            
        # Check if servo is ready
        self.is_servo_active = (msg.code == ServoStatus.NO_WARNING)
        
    def command_callback(self):
        """Pubblica comandi basati sul pattern selezionato"""
        
        if not self.is_servo_active and self.test_pattern != ServoTestPattern.IDLE:
            # Servo not ready, skip this cycle
            return
            
        # Increment phase for periodic patterns
        self.phase += (2.0 * math.pi) / (self.command_rate * 5.0)  # 5 sec period
        if self.phase > 2.0 * math.pi:
            self.phase -= 2.0 * math.pi
        
        # Generate command based on pattern
        if self.test_pattern == ServoTestPattern.IDLE:
            # No command
            pass
            
        elif self.test_pattern == ServoTestPattern.LINEAR_X:
            twist = self.create_twist_command(
                linear_x=0.05 * self.velocity_scale,
                linear_y=0.0,
                linear_z=0.0
            )
            self.twist_pub.publish(twist)
            
        elif self.test_pattern == ServoTestPattern.LINEAR_Y:
            twist = self.create_twist_command(
                linear_x=0.0,
                linear_y=0.05 * self.velocity_scale,
                linear_z=0.0
            )
            self.twist_pub.publish(twist)
            
        elif self.test_pattern == ServoTestPattern.LINEAR_Z:
            twist = self.create_twist_command(
                linear_x=0.0,
                linear_y=0.0,
                linear_z=0.05 * self.velocity_scale
            )
            self.twist_pub.publish(twist)
            
        elif self.test_pattern == ServoTestPattern.CIRCULAR_XY:
            # Movimento circolare nel piano XY
            twist = self.create_twist_command(
                linear_x=0.05 * self.velocity_scale * math.cos(self.phase),
                linear_y=0.05 * self.velocity_scale * math.sin(self.phase),
                linear_z=0.0
            )
            self.twist_pub.publish(twist)
            
        elif self.test_pattern == ServoTestPattern.JOINT_SINE:
            # Movimento sinusoidale joint space
            joint_jog = self.create_joint_jog_command(
                joint_velocities=[
                    0.1 * self.velocity_scale * math.sin(self.phase),
                    0.0,
                    0.1 * self.velocity_scale * math.cos(self.phase),
                    0.0,
                    0.0,
                    0.0,
                    0.0
                ]
            )
            self.joint_jog_pub.publish(joint_jog)
            
        elif self.test_pattern == ServoTestPattern.APPROACH_TEST:
            # Movimento lento in avanti per test collision
            twist = self.create_twist_command(
                linear_x=0.02 * self.velocity_scale,  # Slow approach
                linear_y=0.0,
                linear_z=0.0
            )
            self.twist_pub.publish(twist)
            
    def create_twist_command(self, linear_x=0.0, linear_y=0.0, linear_z=0.0,
                            angular_x=0.0, angular_y=0.0, angular_z=0.0) -> TwistStamped:
        """Crea messaggio TwistStamped"""
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'fr3_link0'
        
        twist.twist.linear.x = linear_x
        twist.twist.linear.y = linear_y
        twist.twist.linear.z = linear_z
        
        twist.twist.angular.x = angular_x
        twist.twist.angular.y = angular_y
        twist.twist.angular.z = angular_z
        
        return twist
        
    def create_joint_jog_command(self, joint_velocities) -> JointJog:
        """Crea messaggio JointJog"""
        jog = JointJog()
        jog.header.stamp = self.get_clock().now().to_msg()
        jog.header.frame_id = 'fr3_link0'
        
        jog.joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]
        jog.velocities = joint_velocities
        jog.duration = 0.0  # Continuous
        
        return jog


def main(args=None):
    """Entry point"""
    print("🎮 Starting Servo Test Node...")
    rclpy.init(args=args)
    
    try:
        node = ServoTestNode()
        
        print("✅ Servo Test Node ready")
        print("   Change pattern with: ros2 param set /servo_test_node test_pattern <pattern>")
        print("   Patterns: idle, linear_x, linear_y, linear_z, circular_xy, joint_sine, approach_test")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n⏹️ Shutdown requested")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        try:
            rclpy.shutdown()
            print("🏁 Servo Test Node terminated")
        except:
            pass


if __name__ == '__main__':
    main()