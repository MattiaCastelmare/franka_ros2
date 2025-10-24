#!/usr/bin/env python3
"""
Dynamic Obstacles Test Node
============================

Pubblica ostacoli che si muovono per testare replanning online.

USAGE:
1. Terminal 1: ros2 launch franka_simulation fr3_hybrid_planning.launch.py
2. Terminal 2: ros2 run franka_simulation test_dynamic_obstacles
3. Terminal 3: ros2 run franka_simulation hybrid_demo_fixed
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
import time
import math


class DynamicObstaclesTest(Node):
    def __init__(self):
        super().__init__('dynamic_obstacles_test')
        
        self.scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )
        
        # Timer per muovere ostacoli
        self.timer = self.create_timer(0.5, self.update_obstacles)
        
        self.time_elapsed = 0.0
        
        self.get_logger().info("🎬 Dynamic obstacles test node ready")
    
    def update_obstacles(self):
        """Aggiorna posizione ostacoli"""
        self.time_elapsed += 0.5
        
        # Ostacolo che oscilla
        scene = PlanningScene()
        scene.is_diff = True
        
        obstacle = CollisionObject()
        obstacle.id = "moving_box"
        obstacle.header.frame_id = "fr3_link0"
        
        # Box oscillante
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.15, 0.15, 0.3]
        
        pose = PoseStamped()
        pose.header.frame_id = "fr3_link0"
        
        # Movimento sinusoidale
        y_pos = 0.2 * math.sin(self.time_elapsed * 0.5)
        pose.pose.position.x = 0.4
        pose.pose.position.y = y_pos
        pose.pose.position.z = 0.3
        pose.pose.orientation.w = 1.0
        
        obstacle.primitives.append(box)
        obstacle.primitive_poses.append(pose.pose)
        obstacle.operation = CollisionObject.ADD
        
        scene.world.collision_objects.append(obstacle)
        
        self.scene_pub.publish(scene)
        
        self.get_logger().debug(f"Moving obstacle to y={y_pos:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstaclesTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()