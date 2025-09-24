#!/usr/bin/env python3
"""
Test funzionalità robot Franka - Movimenti di base e controlli
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
import time
import math

class FrankaRobotTester(Node):
    def __init__(self):
        super().__init__('franka_robot_tester')
        
        # Action client per il controllo del braccio
        self.action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/fr3_arm_controller/follow_joint_trajectory'
        )
        
        # Subscriber per joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.current_positions = None
        self.joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]
        
        self.get_logger().info("Franka Robot Tester initialized")

    def joint_state_callback(self, msg):
        """Callback per ricevere lo stato dei joint"""
        if set(self.joint_names).issubset(set(msg.name)):
            self.current_positions = []
            for joint_name in self.joint_names:
                idx = msg.name.index(joint_name)
                self.current_positions.append(msg.position[idx])

    def wait_for_current_state(self, timeout=5.0):
        """Aspetta di ricevere lo stato corrente dei joint"""
        self.get_logger().info("Waiting for current joint states...")
        
        start_time = time.time()
        while self.current_positions is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().error("Timeout waiting for joint states")
                return False
        
        self.get_logger().info("Current joint states received")
        return True

    def send_joint_trajectory(self, target_positions, duration=3.0):
        """Invia una traiettoria ai joint del robot"""
        
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available")
            return False

        # Crea il messaggio di traiettoria
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.joint_names
        
        # Punto di partenza (posizione attuale)
        start_point = JointTrajectoryPoint()
        start_point.positions = self.current_positions
        start_point.time_from_start = rclpy.duration.Duration(seconds=0).to_msg()
        
        # Punto finale
        end_point = JointTrajectoryPoint()
        end_point.positions = target_positions
        end_point.time_from_start = rclpy.duration.Duration(seconds=duration).to_msg()
        
        goal.trajectory.points = [start_point, end_point]
        
        self.get_logger().info(f"Sending trajectory to positions: {target_positions}")
        
        # Invia il goal
        future = self.action_client.send_goal_async(goal)
        
        # Aspetta il risultato
        rclpy.spin_until_future_complete(self, future, timeout_sec=duration + 2.0)
        
        if future.done():
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info("Trajectory goal accepted")
                
                # Aspetta il completamento
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 2.0)
                
                if result_future.done():
                    result = result_future.result()
                    self.get_logger().info(f"Trajectory completed with result: {result.result.error_code}")
                    return result.result.error_code == 0
                else:
                    self.get_logger().error("Timeout waiting for trajectory completion")
                    return False
            else:
                self.get_logger().error("Trajectory goal rejected")
                return False
        else:
            self.get_logger().error("Timeout sending trajectory goal")
            return False

    def test_basic_movements(self):
        """Test movimenti di base del robot"""
        
        if not self.wait_for_current_state():
            return False
        
        self.get_logger().info("Starting basic movement tests...")
        
        # Test 1: Home position
        home_position = [0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398]
        self.get_logger().info("Test 1: Moving to home position")
        
        if not self.send_joint_trajectory(home_position, duration=4.0):
            self.get_logger().error("Failed to reach home position")
            return False
        
        time.sleep(2.0)
        
        # Test 2: Joint 1 movement
        joint1_moved = home_position.copy()
        joint1_moved[0] = 0.5  # Muove joint1 di 0.5 rad
        self.get_logger().info("Test 2: Moving joint 1")
        
        if not self.send_joint_trajectory(joint1_moved, duration=3.0):
            self.get_logger().error("Failed joint 1 movement")
            return False
        
        time.sleep(2.0)
        
        # Test 3: Torna a home
        self.get_logger().info("Test 3: Returning to home")
        
        if not self.send_joint_trajectory(home_position, duration=3.0):
            self.get_logger().error("Failed to return home")
            return False
        
        # Test 4: Movimento complesso (ready position)
        ready_position = [0.0, -0.3, 0.0, -2.0, 0.0, 1.8, 0.785398]
        self.get_logger().info("Test 4: Moving to ready position")
        
        if not self.send_joint_trajectory(ready_position, duration=4.0):
            self.get_logger().error("Failed to reach ready position")
            return False
        
        time.sleep(2.0)
        
        # Torna a home finale
        self.get_logger().info("Final: Returning to home position")
        if not self.send_joint_trajectory(home_position, duration=4.0):
            self.get_logger().error("Failed final return to home")
            return False
        
        self.get_logger().info("All basic movement tests completed successfully!")
        return True

def main():
    print("Franka Robot Movement Tester")
    print("Make sure the robot simulation is running!")
    
    rclpy.init()
    
    try:
        tester = FrankaRobotTester()
        
        print("Starting robot functionality tests...")
        success = tester.test_basic_movements()
        
        if success:
            print("✅ All robot tests passed!")
        else:
            print("❌ Some robot tests failed!")
            
    except Exception as e:
        print(f"❌ Error during testing: {str(e)}")
        success = False
    finally:
        rclpy.shutdown()
    
    return 0 if success else 1

if __name__ == "__main__":
    exit(main())