#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import moveit_commander
from geometry_msgs.msg import PoseStamped
import sys
import time


class InteractiveCollisionTest(Node):
    def __init__(self):
        super().__init__('interactive_collision_test')
        
        # Inizializza MoveIt
        moveit_commander.roscpp_initialize([])
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("fr3_manipulator")
        
        self.move_group.set_planning_time(10.0)
        self.move_group.set_max_velocity_scaling_factor(0.1)
        self.move_group.set_max_acceleration_scaling_factor(0.1)
        
        self.get_logger().info('Interactive Collision Test Ready')
        self.get_logger().info('Waiting for planning scene...')
        time.sleep(3.0)
        
        self.print_menu()

    def print_menu(self):
        """Stampa menu interattivo"""
        print("\n" + "="*50)
        print("INTERACTIVE COLLISION TEST MENU")
        print("="*50)
        print("1. Go to HOME position")
        print("2. Try to reach obstacle (will fail)")
        print("3. Move ABOVE obstacle (safe)")
        print("4. Move BESIDE obstacle (safe)")
        print("5. Move IN FRONT of obstacle (safe)")
        print("6. Test cartesian path TO obstacle")
        print("7. Test cartesian path AROUND obstacle")
        print("8. Print current position")
        print("9. Print obstacle positions")
        print("0. Exit")
        print("-"*50)

    def run(self):
        """Loop principale interattivo"""
        while rclpy.ok():
            self.print_menu()
            
            try:
                choice = input("Enter choice (0-9): ").strip()
                
                if choice == '0':
                    break
                elif choice == '1':
                    self.go_home()
                elif choice == '2':
                    self.try_reach_obstacle()
                elif choice == '3':
                    self.move_above_obstacle()
                elif choice == '4':
                    self.move_beside_obstacle()
                elif choice == '5':
                    self.move_front_obstacle()
                elif choice == '6':
                    self.cartesian_to_obstacle()
                elif choice == '7':
                    self.cartesian_around_obstacle()
                elif choice == '8':
                    self.print_current_position()
                elif choice == '9':
                    self.print_obstacle_info()
                else:
                    print("Invalid choice!")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f'Error: {e}')
            
            time.sleep(1.0)

    def go_home(self):
        """HOME position"""
        print("\nMoving to HOME...")
        home_joints = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        self.move_group.go(home_joints, wait=True)
        self.move_group.stop()
        print("✓ At HOME position")

    def try_reach_obstacle(self):
        """Tenta di raggiungere l'ostacolo (fallirà)"""
        print("\nAttempting to reach obstacle at (0.5, 0.0, 0.4)...")
        print("This SHOULD FAIL due to collision!")
        
        target = PoseStamped()
        target.header.frame_id = "world"
        target.pose.position.x = 0.5
        target.pose.position.y = 0.0
        target.pose.position.z = 0.4
        target.pose.orientation.x = 1.0
        target.pose.orientation.w = 0.0
        
        self.move_group.set_pose_target(target)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        if success:
            print("⚠ WARNING: Reached target (unexpected!)")
        else:
            print("✓ Motion blocked - collision detected!")

    def move_above_obstacle(self):
        """Muove sopra l'ostacolo"""
        print("\nMoving ABOVE obstacle...")
        target = PoseStamped()
        target.header.frame_id = "world"
        target.pose.position.x = 0.5
        target.pose.position.y = 0.0
        target.pose.position.z = 0.6  # 20cm sopra l'ostacolo
        target.pose.orientation.x = 1.0
        target.pose.orientation.w = 0.0
        
        self.move_group.set_pose_target(target)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        if success:
            print("✓ Reached position above obstacle")
        else:
            print("✗ Failed to reach position")

    def move_beside_obstacle(self):
        """Muove di lato all'ostacolo"""
        print("\nMoving BESIDE obstacle...")
        target = PoseStamped()
        target.header.frame_id = "world"
        target.pose.position.x = 0.5
        target.pose.position.y = -0.25  # 25cm a lato
        target.pose.position.z = 0.4
        target.pose.orientation.x = 1.0
        target.pose.orientation.w = 0.0
        
        self.move_group.set_pose_target(target)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        if success:
            print("✓ Reached position beside obstacle")
        else:
            print("✗ Failed to reach position")

    def move_front_obstacle(self):
        """Muove davanti all'ostacolo"""
        print("\nMoving IN FRONT of obstacle...")
        target = PoseStamped()
        target.header.frame_id = "world"
        target.pose.position.x = 0.35  # 15cm davanti
        target.pose.position.y = 0.0
        target.pose.position.z = 0.4
        target.pose.orientation.x = 1.0
        target.pose.orientation.w = 0.0
        
        self.move_group.set_pose_target(target)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        if success:
            print("✓ Reached position in front of obstacle")
        else:
            print("✗ Failed to reach position")

    def cartesian_to_obstacle(self):
        """Path cartesiano verso l'ostacolo"""
        print("\nComputing cartesian path TO obstacle...")
        
        waypoints = []
        current = self.move_group.get_current_pose().pose
        waypoints.append(current)
        
        # Target nell'ostacolo
        target = current
        target.position.x = 0.5
        target.position.y = 0.0
        target.position.z = 0.4
        waypoints.append(target)
        
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0
        )
        
        print(f"Path computed: {fraction*100:.1f}% of trajectory")
        if fraction < 1.0:
            print("✓ Path stopped before obstacle (collision detected)")
        else:
            print("⚠ Full path computed (check collision detection)")

    def cartesian_around_obstacle(self):
        """Path cartesiano attorno all'ostacolo"""
        print("\nComputing cartesian path AROUND obstacle...")
        
        waypoints = []
        current = self.move_group.get_current_pose().pose
        waypoints.append(current)
        
        # Waypoint 1: a lato
        wp1 = current
        wp1.position.x = 0.4
        wp1.position.y = -0.2
        wp1.position.z = 0.4
        waypoints.append(wp1)
        
        # Waypoint 2: dietro l'ostacolo
        wp2 = current
        wp2.position.x = 0.6
        wp2.position.y = 0.0
        wp2.position.z = 0.4
        waypoints.append(wp2)
        
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0
        )
        
        print(f"Path computed: {fraction*100:.1f}% of trajectory")
        if fraction == 1.0:
            print("✓ Full path around obstacle computed")
            print("Execute? (y/n): ", end='')
            if input().strip().lower() == 'y':
                self.move_group.execute(plan, wait=True)
                print("✓ Executed")

    def print_current_position(self):
        """Stampa posizione corrente"""
        pose = self.move_group.get_current_pose().pose
        print(f"\nCurrent end-effector position:")
        print(f"  x: {pose.position.x:.3f}")
        print(f"  y: {pose.position.y:.3f}")
        print(f"  z: {pose.position.z:.3f}")
        
        joints = self.move_group.get_current_joint_values()
        print(f"Joint values: {[f'{j:.3f}' for j in joints]}")

    def print_obstacle_info(self):
        """Stampa info ostacoli (dalle proprietà note)"""
        print("\nObstacle positions (from URDF):")
        print("  Obstacle 1 (RED): center at (0.5, 0.0, 0.4), size (0.15, 0.15, 0.3)")
        print("  Obstacle 2 (BLUE): center at (0.3, 0.3, 0.2), size (0.1, 0.1, 0.2)")
        print("  Table (GRAY): center at (0.4, 0.0, -0.025), size (1.0, 1.0, 0.05)")


def main(args=None):
    rclpy.init(args=args)
    
    node = InteractiveCollisionTest()
    
    # Thread separato per l'interazione
    import threading
    thread = threading.Thread(target=node.run)
    thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()