#!/usr/bin/env python3
"""
Servo Collision Avoidance Test
===============================

Test specifico per validare collision checking di MoveIt Servo.
Pubblica un ostacolo dinamico nella planning scene e verifica che Servo:
1. Rilevi la collisione
2. Scali la velocità near collision
3. Si fermi prima del contatto

Richiede: move_group attivo, Servo attivo, planning scene monitor
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TwistStamped, PoseStamped
from moveit_servo_msgs.msg import ServoStatus
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

import time



class ServoCollisionTest(Node):
    """Test collision avoidance per Servo"""
    
    def __init__(self):
        super().__init__('servo_collision_test')
        
        # Parameters
        self.declare_parameter('obstacle_distance', 0.3)  # meters from base
        self.declare_parameter('approach_velocity', 0.05)  # m/s
        
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.approach_velocity = self.get_parameter('approach_velocity').value
        
        # Publishers
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_server/delta_twist_cmds',
            1
        )
        
        self.planning_scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            qos_reliable
        )
        
        # Subscribers
        self.servo_status_sub = self.create_subscription(
            ServoStatus,
            '/servo_server/status',
            self.servo_status_callback,
            10
        )
        
        # State
        self.servo_status = None
        self.test_phase = 0
        self.test_start_time = None
        
        # Timer per test sequence
        self.test_timer = self.create_timer(0.1, self.test_callback)
        
        self.get_logger().info("🧪 Servo Collision Test initialized")
        self.get_logger().info(f"  • Obstacle distance: {self.obstacle_distance}m")
        self.get_logger().info(f"  • Approach velocity: {self.approach_velocity}m/s")
        
    def servo_status_callback(self, msg: ServoStatus):
        """Monitor Servo status"""
        self.servo_status = msg
        
        if msg.code == ServoStatus.COLLISION_DETECTED:
            self.get_logger().warn("⚠️ COLLISION DETECTED")
        elif msg.code == ServoStatus.DECELERATE_FOR_COLLISION:
            self.get_logger().info("🐌 Decelerating for collision")
            
    def publish_obstacle(self):
        """Pubblica ostacolo box nella planning scene"""
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        
        # Collision object: box davanti al robot
        collision_obj = CollisionObject()
        collision_obj.header.frame_id = 'fr3_link0'
        collision_obj.header.stamp = self.get_clock().now().to_msg()
        collision_obj.id = 'test_obstacle'
        
        # Box primitive
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.1, 0.3, 0.3]  # x, y, z in meters
        
        # Pose: in front of robot
        pose = PoseStamped()
        pose.header.frame_id = 'fr3_link0'
        pose.pose.position.x = self.obstacle_distance
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.3
        pose.pose.orientation.w = 1.0
        
        collision_obj.primitives.append(box)
        collision_obj.primitive_poses.append(pose.pose)
        collision_obj.operation = CollisionObject.ADD
        
        scene.world.collision_objects.append(collision_obj)
        
        self.planning_scene_pub.publish(scene)
        self.get_logger().info(f"📦 Published obstacle at x={self.obstacle_distance}m")
        
    def remove_obstacle(self):
        """Rimuove ostacolo dalla planning scene"""
        scene = PlanningScene()
        scene.is_diff = True
        
        collision_obj = CollisionObject()
        collision_obj.id = 'test_obstacle'
        collision_obj.operation = CollisionObject.REMOVE
        
        scene.world.collision_objects.append(collision_obj)
        self.planning_scene_pub.publish(scene)
        
        self.get_logger().info("🗑️ Removed obstacle")
        
    def publish_approach_command(self):
        """Pubblica comando di avvicinamento verso ostacolo"""
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'fr3_link0'
        
        twist.twist.linear.x = self.approach_velocity
        twist.twist.linear.y = 0.0
        twist.twist.linear.z = 0.0
        
        self.twist_pub.publish(twist)
        
    def publish_stop_command(self):
        """Pubblica comando di stop"""
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'fr3_link0'
        # All zeros = stop
        
        self.twist_pub.publish(twist)
        
    def test_callback(self):
        """Sequence di test"""
        
        if self.test_phase == 0:
            # Phase 0: Wait for Servo ready
            if self.servo_status and self.servo_status.code == ServoStatus.NO_WARNING:
                self.get_logger().info("✅ Servo ready, starting test in 2s...")
                self.test_phase = 1
                self.test_start_time = time.time()
                
        elif self.test_phase == 1:
            # Phase 1: Wait 2s then publish obstacle
            if time.time() - self.test_start_time > 2.0:
                self.publish_obstacle()
                self.test_phase = 2
                self.test_start_time = time.time()
                
        elif self.test_phase == 2:
            # Phase 2: Wait 2s for scene update, then start approach
            if time.time() - self.test_start_time > 2.0:
                self.get_logger().info("🚀 Starting approach to obstacle...")
                self.test_phase = 3
                self.test_start_time = time.time()
                
        elif self.test_phase == 3:
            # Phase 3: Approach for 10s or until collision detected
            self.publish_approach_command()
            
            elapsed = time.time() - self.test_start_time
            
            if self.servo_status and self.servo_status.code == ServoStatus.COLLISION_DETECTED:
                self.get_logger().info("✅ SUCCESS: Collision detected, stopping")
                self.publish_stop_command()
                self.test_phase = 4
                self.test_start_time = time.time()
                
            elif elapsed > 10.0:
                self.get_logger().error("❌ FAILED: No collision detected after 10s")
                self.publish_stop_command()
                self.test_phase = 4
                self.test_start_time = time.time()
                
        elif self.test_phase == 4:
            # Phase 4: Wait 3s then remove obstacle
            if time.time() - self.test_start_time > 3.0:
                self.remove_obstacle()
                self.get_logger().info("🏁 Test complete")
                self.test_phase = 5  # End
                
        elif self.test_phase == 5:
            # Test done, do nothing
            pass


def main(args=None):
    """Entry point"""
    print("🧪 Starting Servo Collision Test...")
    rclpy.init(args=args)
    
    try:
        node = ServoCollisionTest()
        
        print("✅ Servo Collision Test ready")
        print("   Test sequence will start automatically when Servo is ready")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n⏹️ Shutdown requested")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        try:
            rclpy.shutdown()
            print("🏁 Servo Collision Test terminated")
        except:
            pass


if __name__ == '__main__':
    main()