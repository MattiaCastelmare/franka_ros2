#!/usr/bin/env python3
"""
FR3 Safe Online Avoidance Test - Production Version
===================================================

Comprehensive test of velocity-based trajectory tracking with online
obstacle avoidance, using carefully designed safe waypoints that:
1. Challenge the avoidance system
2. Maintain safe distances (> 0.30m influence, > 0.08m safety margin)
3. Test multiple scenarios (lateral, bilateral, vertical, diagonal)

Obstacle Configuration (from multi_obstacle_scene.urdf.xacro):
- Red Box Central: (0.5, -0.3, 0.15), size 0.15×0.15×0.55
- Yellow Box Lateral: (0.4, 0.3, 0.20), size 0.10×0.10×0.60
- Ground Plane: (0.0, 0.0, -0.005)

Safety Parameters (from avoidance_params.yaml):
- Influence Distance: 0.30 m
- Safety Margin: 0.08 m
- Repulsive Gain: 0.4

Author: Custom designed for Maurizio's FR3 framework
Date: 2025
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

import numpy as np
import time
from typing import Optional, List, Tuple, Dict
from dataclasses import dataclass
from enum import Enum

# ROS messages
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from moveit_msgs.msg import MoveItErrorCodes

# Actions
from franka_simulation.action import MoveToPose

# Per FK
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState

import math

# Terminal colors
class Colors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    END = '\033[0m'
    BOLD = '\033[1m'


class ExecutionState(Enum):
    IDLE = "idle"
    PLANNING = "planning"
    EXECUTING = "executing"
    AVOIDANCE_ACTIVE = "avoidance_active"
    COMPLETED = "completed"
    FAILED = "failed"


@dataclass
class Waypoint:
    """Waypoint with metadata"""
    x: float
    y: float
    z: float
    name: str
    description: str
    expected_avoidance: str
    critical: bool = False
    
    def distance_to_obstacle(self, obs_x: float, obs_y: float, obs_z: float) -> float:
        """Calculate Euclidean distance to obstacle"""
        return np.sqrt((self.x - obs_x)**2 + (self.y - obs_y)**2 + (self.z - obs_z)**2)


@dataclass
class VelocityDebugData:
    """Container for velocity debug information"""
    timestamp: float
    waypoint_name: str
    tracking_velocity: np.ndarray
    avoidance_velocity: np.ndarray
    blended_velocity: np.ndarray
    joint_positions: np.ndarray
    
    @property
    def tracking_norm(self) -> float:
        return np.linalg.norm(self.tracking_velocity)
    
    @property
    def avoidance_norm(self) -> float:
        return np.linalg.norm(self.avoidance_velocity)
    
    @property
    def blended_norm(self) -> float:
        return np.linalg.norm(self.blended_velocity)
    
    @property
    def avoidance_contribution(self) -> float:
        if self.blended_norm < 1e-6:
            return 0.0
        return (self.avoidance_norm / self.blended_norm) * 100.0


class SafeAvoidanceTest(Node):
    """Safe online avoidance test with production-ready trajectory"""
    
    def __init__(self):
        super().__init__('safe_avoidance_test')
        
        # Callback groups
        self.action_callback_group = ReentrantCallbackGroup()
        self.monitor_callback_group = MutuallyExclusiveCallbackGroup()
        
        # State
        self.state = ExecutionState.IDLE
        self.current_waypoint: Optional[Waypoint] = None
        
        # Joint state
        self.joint_positions = np.zeros(7)
        self.joint_velocities = np.zeros(7)
        self.initial_joint_positions: Optional[np.ndarray] = None
        
        # Velocity components
        self.tracking_velocity = np.zeros(7)
        self.avoidance_velocity = np.zeros(7)
        self.blended_velocity = np.zeros(7)
        
        # Debug history
        self.debug_history: List[VelocityDebugData] = []
        self.max_history = 2000
        
        # Avoidance tracking
        self.avoidance_activations: Dict[str, int] = {}
        self.max_avoidance_per_waypoint: Dict[str, float] = {}
        self.min_distance_achieved = float('inf')
        
        # Action client
        self.move_action_client = ActionClient(
            self,
            MoveToPose,
            'move_to_pose',
            callback_group=self.action_callback_group
        )
        
        # FK client per calcolare la posizione cartesiana
        self.fk_client = self.create_client(GetPositionFK, 'compute_fk')
        
        # Subscribers
        self.create_subscription(
            JointState, '/joint_states',
            self.joint_state_callback, 10,
            callback_group=self.monitor_callback_group
        )
        
        self.create_subscription(
            Float64MultiArray, '/avoidance/velocity',
            self.avoidance_velocity_callback, 10,
            callback_group=self.monitor_callback_group
        )
        
        self.create_subscription(
            Float64MultiArray, '/fr3_velocity_controller/commands',
            self.controller_commands_callback, 10,
            callback_group=self.monitor_callback_group
        )
        
        # Timers
        self.create_timer(0.5, self.debug_output_callback,
                         callback_group=self.monitor_callback_group)
        
        # Joint names per FK
        self.joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]
        
        self.get_logger().info(f"{Colors.BOLD}{Colors.CYAN}{'='*70}{Colors.END}")
        self.get_logger().info(f"{Colors.BOLD}{Colors.CYAN}Safe Avoidance Test Node Initialized{Colors.END}")
        self.get_logger().info(f"{Colors.BOLD}{Colors.CYAN}{'='*70}{Colors.END}")
    
    def joint_state_callback(self, msg: JointState):
        positions = []
        velocities = []
        for name in ['fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
                     'fr3_joint5', 'fr3_joint6', 'fr3_joint7']:
            if name in msg.name:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
                velocities.append(msg.velocity[idx] if msg.velocity else 0.0)
        
        if len(positions) == 7:
            self.joint_positions = np.array(positions)
            self.joint_velocities = np.array(velocities)
            if self.initial_joint_positions is None:
                self.initial_joint_positions = self.joint_positions.copy()
    
    def avoidance_velocity_callback(self, msg: Float64MultiArray):
        if len(msg.data) == 7:
            self.avoidance_velocity = np.array(msg.data)
            norm = np.linalg.norm(self.avoidance_velocity)
            
            if norm > 0.001 and self.current_waypoint:
                wp_name = self.current_waypoint.name
                self.avoidance_activations[wp_name] = \
                    self.avoidance_activations.get(wp_name, 0) + 1
                
                if wp_name not in self.max_avoidance_per_waypoint or \
                   norm > self.max_avoidance_per_waypoint[wp_name]:
                    self.max_avoidance_per_waypoint[wp_name] = norm
                
                if norm > 0.01 and self.state != ExecutionState.AVOIDANCE_ACTIVE:
                    self.state = ExecutionState.AVOIDANCE_ACTIVE
                    self.get_logger().info(
                        f"{Colors.YELLOW}🛡️  AVOIDANCE ACTIVE at {wp_name}{Colors.END}"
                    )
            else:
                if self.state == ExecutionState.AVOIDANCE_ACTIVE:
                    self.state = ExecutionState.EXECUTING
    
    def controller_commands_callback(self, msg: Float64MultiArray):
        if len(msg.data) == 7:
            self.blended_velocity = np.array(msg.data)
            self.tracking_velocity = self.blended_velocity - self.avoidance_velocity
            
            if self.state in [ExecutionState.EXECUTING, ExecutionState.AVOIDANCE_ACTIVE]:
                if self.current_waypoint:
                    debug_data = VelocityDebugData(
                        timestamp=self.get_clock().now().nanoseconds * 1e-9,
                        waypoint_name=self.current_waypoint.name,
                        tracking_velocity=self.tracking_velocity.copy(),
                        avoidance_velocity=self.avoidance_velocity.copy(),
                        blended_velocity=self.blended_velocity.copy(),
                        joint_positions=self.joint_positions.copy()
                    )
                    self.debug_history.append(debug_data)
                    if len(self.debug_history) > self.max_history:
                        self.debug_history.pop(0)
    
    def compute_fk_position(self) -> Optional[Tuple[float, float, float]]:
        """Calcola la posizione cartesiana dell'end-effector usando FK."""
        if not self.fk_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("FK service not available")
            return None
        
        request = GetPositionFK.Request()
        request.header.frame_id = 'fr3_link0'
        request.fk_link_names = ['fr3_link8']  # End-effector
        
        robot_state = RobotState()
        robot_state.joint_state.name = self.joint_names
        robot_state.joint_state.position = self.joint_positions.tolist()
        request.robot_state = robot_state
        
        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is None:
            self.get_logger().warn("FK call failed")
            return None
        
        response = future.result()
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn(f"FK error: {response.error_code.val}")
            return None
        
        if response.pose_stamped:
            pose = response.pose_stamped[0].pose
            return (pose.position.x, pose.position.y, pose.position.z)
        
        return None
    
    def log_reached_position(self, waypoint: Waypoint):
        """Logga la posizione raggiunta e calcola l'errore rispetto al target."""
        print(f"\n{Colors.BOLD}{Colors.HEADER}{'='*70}{Colors.END}")
        print(f"{Colors.BOLD}{Colors.HEADER}📍 POSITION REACHED - {waypoint.name}{Colors.END}")
        print(f"{Colors.BOLD}{Colors.HEADER}{'='*70}{Colors.END}")
        
        # Target
        print(f"\n{Colors.BOLD}🎯 TARGET:{Colors.END}")
        print(f"   X: {waypoint.x:.4f} m")
        print(f"   Y: {waypoint.y:.4f} m")
        print(f"   Z: {waypoint.z:.4f} m")
        
        # Calcola FK
        reached_pos = self.compute_fk_position()
        
        if reached_pos:
            rx, ry, rz = reached_pos
            print(f"\n{Colors.BOLD}📌 REACHED (FK):{Colors.END}")
            print(f"   X: {rx:.4f} m")
            print(f"   Y: {ry:.4f} m")
            print(f"   Z: {rz:.4f} m")
            
            # Errore
            error_x = rx - waypoint.x
            error_y = ry - waypoint.y
            error_z = rz - waypoint.z
            error_total = np.sqrt(error_x**2 + error_y**2 + error_z**2)
            
            print(f"\n{Colors.BOLD}❌ ERROR:{Colors.END}")
            print(f"   ΔX: {error_x:+.4f} m")
            print(f"   ΔY: {error_y:+.4f} m")
            print(f"   ΔZ: {error_z:+.4f} m")
            print(f"   {Colors.YELLOW}Total: {error_total:.4f} m{Colors.END}")
        else:
            print(f"\n{Colors.RED}⚠️  FK calculation failed{Colors.END}")
        
        # Joint positions
        print(f"\n{Colors.BOLD}🔧 JOINT POSITIONS (rad):{Colors.END}")
        for i, pos in enumerate(self.joint_positions):
            print(f"   Joint {i+1}: {pos:+.4f} rad ({np.degrees(pos):+.2f}°)")
        
        print(f"{Colors.BOLD}{Colors.HEADER}{'='*70}{Colors.END}\n")
    
    def plan_and_execute_waypoint(self, waypoint: Waypoint,
                                   velocity_scaling: float = 0.08) -> bool:
        """Plan and execute motion to waypoint"""
        self.current_waypoint = waypoint
        
        self.get_logger().info(f"\n{Colors.CYAN}{'─'*70}{Colors.END}")
        self.get_logger().info(
            f"{Colors.BOLD}{Colors.CYAN}→ {waypoint.name}{Colors.END}"
        )
        self.get_logger().info(
            f"{Colors.CYAN}   Position: ({waypoint.x:.3f}, {waypoint.y:.3f}, {waypoint.z:.3f}){Colors.END}"
        )
        self.get_logger().info(
            f"{Colors.CYAN}   Expected: {waypoint.expected_avoidance}{Colors.END}"
        )
        self.get_logger().info(f"{Colors.CYAN}{'─'*70}{Colors.END}\n")
        
        self.state = ExecutionState.PLANNING
        
        # Create goal
        goal = MoveToPose.Goal()
        goal.pose_target.header.frame_id = 'world'
        goal.pose_target.header.stamp = self.get_clock().now().to_msg()
        goal.pose_target.pose.position.x = waypoint.x
        goal.pose_target.pose.position.y = waypoint.y
        goal.pose_target.pose.position.z = waypoint.z
        goal.pose_target.pose.orientation.x = 1.0
        goal.pose_target.pose.orientation.y = 0.0
        goal.pose_target.pose.orientation.z = 0.0
        goal.pose_target.pose.orientation.w = 0.0

        goal.cartesian_motion = False
        goal.max_velocity_scaling_factor = velocity_scaling
        
        # Wait for action server
        if not self.move_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"{Colors.RED}❌ Action server unavailable{Colors.END}")
            self.state = ExecutionState.FAILED
            return False
        
        # Send goal
        send_goal_future = self.move_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"{Colors.RED}❌ Goal rejected{Colors.END}")
            self.state = ExecutionState.FAILED
            return False
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        
        result = result_future.result().result
        if result.result.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"{Colors.RED}❌ Planning failed: {result.result.val}{Colors.END}"
            )
            self.state = ExecutionState.FAILED
            return False
        
        self.get_logger().info(f"{Colors.GREEN}✅ Planning successful{Colors.END}")
        
        # Execute (trajectory published to /velocity_blender/trajectory)
        self.state = ExecutionState.EXECUTING
        self._wait_for_motion_completion(duration=15.0, waypoint=waypoint)
        
        return True
    
    def _wait_for_motion_completion(self, duration: float, waypoint: Waypoint):
        """Wait for motion to complete"""
        start_time = time.time()
        last_motion = start_time
        
        while time.time() - start_time < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            vel_norm = np.linalg.norm(self.joint_velocities)
            if vel_norm > 0.01:
                last_motion = time.time()
            
            if time.time() - last_motion > 1.5:
                self.get_logger().info(
                    f"{Colors.GREEN}✅ Motion completed for {waypoint.name}{Colors.END}"
                )
                return
        
        self.get_logger().warn(f"{Colors.YELLOW}⏱️  Timeout for {waypoint.name}{Colors.END}")
    
    def debug_output_callback(self):
        """Print debug info"""
        if self.state not in [ExecutionState.EXECUTING, ExecutionState.AVOIDANCE_ACTIVE]:
            return
        
        tracking_norm = np.linalg.norm(self.tracking_velocity)
        avoidance_norm = np.linalg.norm(self.avoidance_velocity)
        blended_norm = np.linalg.norm(self.blended_velocity)
        
        print(f"\n{Colors.BOLD}{'='*70}{Colors.END}")
        print(f"{Colors.BOLD}⏱️  VELOCITY DEBUG{Colors.END}")
        print(f"{Colors.BOLD}{'='*70}{Colors.END}")
        
        if self.current_waypoint:
            state_color = Colors.YELLOW if self.state == ExecutionState.AVOIDANCE_ACTIVE else Colors.CYAN
            print(f"\n{Colors.BOLD}Waypoint:{Colors.END} {state_color}{self.current_waypoint.name}{Colors.END}")
            print(f"{Colors.BOLD}State:{Colors.END} {state_color}{self.state.value.upper()}{Colors.END}")
        
        print(f"\n{Colors.BOLD}Velocities (rad/s):{Colors.END}")
        print(f"  📊 Tracking:  {Colors.BLUE}{tracking_norm:>8.4f}{Colors.END}")
        print(f"  🛡️  Avoidance: {Colors.YELLOW}{avoidance_norm:>8.4f}{Colors.END}")
        print(f"  ⚡ Blended:   {Colors.GREEN}{blended_norm:>8.4f}{Colors.END}")
        
        if blended_norm > 1e-6:
            contrib = (avoidance_norm / blended_norm) * 100.0
            if contrib > 5.0:
                print(f"  💥 Avoidance: {Colors.YELLOW}{contrib:>5.1f}%{Colors.END}")
        
        print(f"{Colors.BOLD}{'='*70}{Colors.END}\n")
    
    def print_final_report(self):
        """Comprehensive final report"""
        print(f"\n{Colors.BOLD}{Colors.HEADER}{'='*70}{Colors.END}")
        print(f"{Colors.BOLD}{Colors.HEADER}FINAL TEST REPORT{Colors.END}")
        print(f"{Colors.BOLD}{Colors.HEADER}{'='*70}{Colors.END}\n")
        
        # Avoidance summary
        print(f"{Colors.BOLD}Avoidance Activations by Waypoint:{Colors.END}")
        if self.avoidance_activations:
            for wp_name, count in sorted(self.avoidance_activations.items()):
                max_avoid = self.max_avoidance_per_waypoint.get(wp_name, 0.0)
                print(f"  {wp_name}:")
                print(f"    Activations: {count}")
                print(f"    Max magnitude: {max_avoid:.4f} rad/s")
        else:
            print(f"  {Colors.YELLOW}⚠️  No avoidance activated{Colors.END}")
        
        # Statistics
        if self.debug_history:
            tracking_norms = [d.tracking_norm for d in self.debug_history]
            avoidance_norms = [d.avoidance_norm for d in self.debug_history]
            
            print(f"\n{Colors.BOLD}Overall Statistics:{Colors.END}")
            print(f"  Tracking velocity:")
            print(f"    Mean: {np.mean(tracking_norms):.4f} rad/s")
            print(f"    Max:  {np.max(tracking_norms):.4f} rad/s")
            print(f"  Avoidance velocity:")
            print(f"    Mean: {np.mean(avoidance_norms):.4f} rad/s")
            print(f"    Max:  {np.max(avoidance_norms):.4f} rad/s")
            print(f"  Data points: {len(self.debug_history)}")
        
        print(f"\n{Colors.BOLD}{Colors.GREEN}✅ Test Complete{Colors.END}")
        print(f"{Colors.BOLD}{Colors.HEADER}{'='*70}{Colors.END}\n")


def main():
    """Main test execution"""
    print(f"\n{Colors.BOLD}{Colors.CYAN}{'='*70}{Colors.END}")
    print(f"{Colors.BOLD}{Colors.CYAN}FR3 SAFE ONLINE AVOIDANCE TEST{Colors.END}")
    print(f"{Colors.BOLD}{Colors.CYAN}{'='*70}{Colors.END}\n")
    
    print(f"{Colors.BOLD}Obstacle Configuration:{Colors.END}")
    print(f"  🔴 Red Box:    (0.50, -0.30, 0.15) [0.15×0.15×0.55]")
    print(f"  🟡 Yellow Box: (0.40,  0.30, 0.20) [0.10×0.10×0.60]")
    print(f"  ⬜ Ground:     (0.00,  0.00, -0.01) [5.0×5.0×0.01]\n")
    
    print(f"{Colors.BOLD}Safety Parameters:{Colors.END}")
    print(f"  Influence Distance: 0.30 m")
    print(f"  Safety Margin: 0.08 m")
    print(f"  Target Distance: 0.20-0.40 m\n")
    
    input(f"{Colors.YELLOW}Press ENTER to start test...{Colors.END}\n")
    
    rclpy.init()
    test_node = SafeAvoidanceTest()
    
    executor = MultiThreadedExecutor()
    executor.add_node(test_node)
    
    import threading
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        time.sleep(2.0)
        
        # DEFINE SAFE TRAJECTORY
        waypoints = [
            Waypoint(0.30, 0.0, 0.45, "WP0_Home", "Safe starting position", "None", False),
            Waypoint(0.20, -0.65, 0.20, "WP1_RedApproach", "Approach red box laterally", "Lateral push away", False),
            Waypoint(0.20, 0.65, 0.20, "WP2_OtherSide", "Other side", "Lateral push away", False),
            Waypoint(0.20, -0.65, 0.20, "WP3_Back", "Back to first side", "Lateral push away", False),
            Waypoint(0.20, 0.65, 0.20, "WP4_OtherSideAgain", "Other side again", "Lateral push away", False),
        ]
        
        velocity_scaling = 0.06  # Slow for observation
        
        print(f"\n{Colors.BOLD}Executing {len(waypoints)} waypoints...{Colors.END}\n")
        time.sleep(2.0)
        
        # Execute trajectory
        for i, wp in enumerate(waypoints, 1):
            print(f"\n{Colors.BOLD}[{i}/{len(waypoints)}]{Colors.END}")
            success = test_node.plan_and_execute_waypoint(wp, velocity_scaling)
            
            if not success:
                print(f"{Colors.RED}❌ Failed at {wp.name}{Colors.END}")
                break
            
            # ========================================
            # LOG POSIZIONE RAGGIUNTA E ATTENDI INVIO
            # ========================================
            test_node.log_reached_position(wp)
            
            if i < len(waypoints):
                input(f"{Colors.YELLOW}Press ENTER to continue to next waypoint...{Colors.END}\n")
        
        test_node.state = ExecutionState.COMPLETED
        time.sleep(2.0)
        
        test_node.print_final_report()
        
    except KeyboardInterrupt:
        print(f"\n{Colors.YELLOW}Test interrupted{Colors.END}")
    except Exception as e:
        print(f"\n{Colors.RED}Error: {e}{Colors.END}")
        import traceback
        traceback.print_exc()
    finally:
        executor.shutdown()
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()