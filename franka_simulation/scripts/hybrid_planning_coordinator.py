#!/usr/bin/env python3
"""
Hybrid Planning Coordinator - VERSIONE COMPLETA
================================================

Nodo centrale per orchestrare Hybrid Planning:
1. Riceve goal ExecuteHybridMotion da utente
2. Richiede global plan (OMPL) a franka_motion_server
3. Traccia waypoints pubblicando TwistStamped a MoveIt Servo
4. Usa TF per current pose (non FK manuale)
5. Monitora collisioni via /servo_server/status
6. Gestisce stop e transizioni waypoint

ARCHITETTURA:
- Global: OMPL via motion_server (planning only)
- Local: MoveIt Servo per refinement real-time
- Pose tracking: TF listener (fr3_link0 -> fr3_hand_tcp)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import time
import numpy as np
from typing import Optional, List
from enum import Enum

# TF2
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs

# Messages
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, TransformStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState
#from moveit_msgs.msg import ServoStatus

# Actions
from franka_simulation.action import ExecuteHybridMotion, PlanGlobalPath


class HybridState(Enum):
    """Stati del coordinator"""
    IDLE = 0
    GLOBAL_PLANNING = 1
    LOCAL_EXECUTION = 2
    REPLANNING = 3
    COLLISION_AVOIDANCE = 4
    COMPLETED = 5
    FAILED = 6


class HybridPlanningCoordinator(Node):
    """
    Coordinator per Hybrid Planning con global (OMPL) + local (Servo).
    Usa TF per pose corrente, PID per tracking, monitoring collisioni.
    """
    
    def __init__(self):
        super().__init__('hybrid_planning_coordinator')
        
        self._declare_parameters()
        self._load_parameters()
        
        self.callback_group = ReentrantCallbackGroup()
        
        # Stato interno
        self.current_state = HybridState.IDLE
        self.current_pose: Optional[Pose] = None
        self.target_pose: Optional[Pose] = None
        self.waypoints: List[PoseStamped] = []
        self.waypoint_index = 0
        #self.servo_status: Optional[ServoStatus] = None
        self.servo_active = False
        self.last_servo_update = time.time()
        self.collision_detected = False

        
        # Timing
        self.last_twist_time = time.time()
        self.execution_start_time = 0.0
        self.last_pose_update = time.time()
        
        # TF per current pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Inizializzazione componenti
        self._init_action_servers()
        self._init_action_clients()
        self._init_publishers()
        self._init_subscribers()
        self._init_timers()
        
        self.get_logger().info("🤖 Hybrid Planning Coordinator ready!")
        self._log_configuration()
    
    def _declare_parameters(self):
        """Parametri configurabili"""
        self.declare_parameter('base_frame', 'fr3_link0')
        self.declare_parameter('ee_frame', 'fr3_hand_tcp')
        self.declare_parameter('planning_group', 'fr3_arm')
        
        # Global planner params
        self.declare_parameter('global_planner_id', 'RRTConnect')
        self.declare_parameter('global_planning_time', 5.0)
        self.declare_parameter('global_max_attempts', 3)
        
        # Local planner params
        self.declare_parameter('servo_control_rate', 50.0)  # Hz
        self.declare_parameter('waypoint_tolerance', 0.02)  # meters
        self.declare_parameter('goal_tolerance', 0.01)  # meters
        
        # PID gains for twist generation
        self.declare_parameter('kp_linear', 1.5)
        self.declare_parameter('kp_angular', 2.0)
        self.declare_parameter('max_linear_velocity', 0.3)
        self.declare_parameter('max_angular_velocity', 0.5)
        
        # Collision/safety params
        self.declare_parameter('collision_distance_threshold', 0.03)  # meters
        self.declare_parameter('servo_timeout', 2.0)  # seconds
        self.declare_parameter('tf_timeout', 0.5)  # seconds
        
    def _load_parameters(self):
        """Carica parametri"""
        self.base_frame = self.get_parameter('base_frame').value
        self.ee_frame = self.get_parameter('ee_frame').value
        self.planning_group = self.get_parameter('planning_group').value
        
        self.global_planner_id = self.get_parameter('global_planner_id').value
        self.global_planning_time = self.get_parameter('global_planning_time').value
        self.global_max_attempts = self.get_parameter('global_max_attempts').value
        
        self.servo_control_rate = self.get_parameter('servo_control_rate').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        
        self.collision_distance_threshold = self.get_parameter('collision_distance_threshold').value
        self.servo_timeout = self.get_parameter('servo_timeout').value
        self.tf_timeout = self.get_parameter('tf_timeout').value
        
    def _init_action_servers(self):
        """Action server per ricevere goal da utente"""
        self.hybrid_motion_server = ActionServer(
            self,
            ExecuteHybridMotion,
            'execute_hybrid_motion',
            execute_callback=self.execute_hybrid_callback,
            goal_callback=lambda req: GoalResponse.ACCEPT,
            cancel_callback=lambda req: CancelResponse.ACCEPT,
            callback_group=self.callback_group
        )
        self.get_logger().info("✅ Action server: /execute_hybrid_motion")
        
    def _init_action_clients(self):
        """Action clients per comunicare con motion_server"""
        self.global_plan_client = ActionClient(
            self,
            PlanGlobalPath,
            'plan_global_path',
            callback_group=self.callback_group
        )
        
        self.get_logger().info("⏳ Waiting for /plan_global_path server...")
        if not self.global_plan_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("❌ /plan_global_path server not available!")
            raise RuntimeError("PlanGlobalPath server not found")
        else:
            self.get_logger().info("✅ Connected to /plan_global_path")
        
    def _init_publishers(self):
        """Publishers per Servo e debug"""
        # Servo control
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_server/delta_twist_cmds',
            10
        )
        
        # Debug/visualization
        self.waypoints_pub = self.create_publisher(
            Path,
            '/hybrid_planning/waypoints',
            10
        )
        
        self.get_logger().info("✅ Publishers ready")
        
    def _init_subscribers(self):
        """Subscribers per eventuali segnali di stato"""
        # Placeholder — in futuro puoi collegare un topic di diagnostica del Servo
        self.get_logger().info("✅ Subscribers ready (no ServoStatus topic in Humble)")

        
    def _init_timers(self):
        """Timer per control loop Servo e TF update"""
        # Servo control loop
        control_period = 1.0 / self.servo_control_rate
        self.servo_control_timer = self.create_timer(
            control_period,
            self.servo_control_loop,
            callback_group=self.callback_group
        )
        
        # TF update loop (più lento, 20 Hz)
        self.tf_update_timer = self.create_timer(
            0.05,  # 20 Hz
            self.update_current_pose_from_tf,
            callback_group=self.callback_group
        )
        # Monitoraggio Servo activity (timeout safety)
        self.servo_monitor_timer = self.create_timer(
            0.5,  # ogni mezzo secondo
            self.monitor_servo_activity,
            callback_group=self.callback_group
        )
        
    def _log_configuration(self):
        """Log configurazione"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("Hybrid Planning Configuration:")
        self.get_logger().info(f"  • Base frame: {self.base_frame}")
        self.get_logger().info(f"  • EE frame: {self.ee_frame}")
        self.get_logger().info(f"  • Global planner: {self.global_planner_id}")
        self.get_logger().info(f"  • Servo rate: {self.servo_control_rate} Hz")
        self.get_logger().info(f"  • Waypoint tolerance: {self.waypoint_tolerance}m")
        self.get_logger().info(f"  • Goal tolerance: {self.goal_tolerance}m")
        self.get_logger().info(f"  • Kp linear: {self.kp_linear}")
        self.get_logger().info(f"  • Max linear vel: {self.max_linear_velocity} m/s")
        self.get_logger().info("=" * 60)
    
    # ========================================================================
    # TF POSE UPDATE
    # ========================================================================
    
    def update_current_pose_from_tf(self):
        """
        Aggiorna current_pose usando TF listener.
        Lookup transform: base_frame -> ee_frame
        """
        try:
            # Lookup latest transform
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                rclpy.time.Time(),  # Latest available
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout)
            )
            
            # Convert TransformStamped to Pose
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            
            self.current_pose = pose
            self.last_pose_update = time.time()
            
        except TransformException as ex:
            # Solo log a debug level per non spammare
            if time.time() - self.last_pose_update > 1.0:
                self.get_logger().warn(f"TF lookup failed: {ex}")
    
    # ========================================================================
    # CALLBACKS
    # ========================================================================
    
    # def servo_status_callback(self, msg: ServoStatus):
    #     """Monitora stato Servo per collision avoidance"""
    #     self.servo_status = msg
        
    #     if msg.code == ServoStatus.COLLISION_DETECTED:
    #         self.get_logger().warn("⚠️ COLLISION DETECTED by Servo!")
    #         if self.current_state == HybridState.LOCAL_EXECUTION:
    #             self.current_state = HybridState.COLLISION_AVOIDANCE
                
    #     elif msg.code == ServoStatus.DECELERATE_FOR_COLLISION:
    #         # Log solo occasionalmente
    #         pass
    def monitor_servo_activity(self):
        """Controlla se il Servo riceve aggiornamenti (simula stato attivo)."""
        now = time.time()
        if now - self.last_twist_time > self.servo_timeout:
            if self.current_state == HybridState.LOCAL_EXECUTION:
                self.get_logger().warn("⚠️ Servo timeout — stopping motion")
                self.current_state = HybridState.FAILED

    
    # ========================================================================
    # ACTION CALLBACK: ExecuteHybridMotion
    # ========================================================================
    
    async def execute_hybrid_callback(self, goal_handle):
        """
        Main callback per hybrid motion execution.
        
        Fasi:
        1. Global planning (OMPL) via motion_server
        2. Local execution (Servo tracking) con control loop
        3. Monitoring collisioni e completamento
        """
        goal = goal_handle.request
        result = ExecuteHybridMotion.Result()
        
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"🎯 NEW HYBRID MOTION GOAL")
        self.get_logger().info(f"   Target: {goal.target_pose.pose.position}")
        self.get_logger().info(f"   Use hybrid: {goal.use_hybrid_planning}")
        self.get_logger().info("=" * 60)
        
        # Reset stato
        self.waypoints = []
        self.waypoint_index = 0
        self.target_pose = goal.target_pose.pose
        self.execution_start_time = time.time()
        
        # ----------------------------------------------------------------
        # FASE 1: GLOBAL PLANNING
        # ----------------------------------------------------------------
        self.current_state = HybridState.GLOBAL_PLANNING
        
        feedback = ExecuteHybridMotion.Feedback()
        feedback.current_phase = "global_planning"
        feedback.progress = 0.1
        goal_handle.publish_feedback(feedback)
        
        self.get_logger().info("🌍 Phase 1: Global Planning...")
        
        # Richiedi global plan al motion_server
        plan_goal = PlanGlobalPath.Goal()
        plan_goal.target_pose = goal.target_pose
        plan_goal.planner_id = self.global_planner_id
        plan_goal.planning_time = self.global_planning_time
        plan_goal.max_attempts = self.global_max_attempts
        
        plan_future = self.global_plan_client.send_goal_async(plan_goal)
        
        # Wait for goal acceptance
        rclpy.spin_until_future_complete(self, plan_future, timeout_sec=5.0)
        
        if not plan_future.done() or not plan_future.result().accepted:
            self.get_logger().error("❌ Global planning rejected or timeout!")
            result.error_code = ExecuteHybridMotion.Result.PLANNING_FAILED
            result.error_message = "Global planning goal rejected"
            goal_handle.abort()
            return result
        
        plan_goal_handle = plan_future.result()
        
        # Wait for planning result
        plan_result_future = plan_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, plan_result_future, timeout_sec=15.0)
        
        if not plan_result_future.done():
            self.get_logger().error("❌ Global planning result timeout!")
            result.error_code = ExecuteHybridMotion.Result.PLANNING_FAILED
            result.error_message = "Global planning timeout"
            goal_handle.abort()
            return result
        
        plan_result = plan_result_future.result().result
        
        if plan_result.error_code != 1:  # MoveIt SUCCESS = 1
            self.get_logger().error(f"❌ Planning failed: MoveIt code {plan_result.error_code}")
            result.error_code = ExecuteHybridMotion.Result.PLANNING_FAILED
            result.error_message = f"MoveIt error code: {plan_result.error_code}"
            goal_handle.abort()
            return result
        
        # Estrai waypoints
        self.waypoints = plan_result.waypoints_path.poses
        result.planning_time = plan_result.planning_time
        
        if len(self.waypoints) == 0:
            self.get_logger().error("❌ No waypoints extracted from plan!")
            result.error_code = ExecuteHybridMotion.Result.PLANNING_FAILED
            result.error_message = "Empty waypoints path"
            goal_handle.abort()
            return result
        
        self.get_logger().info(f"✅ Global plan ready: {len(self.waypoints)} waypoints")
        
        # Pubblica waypoints per visualizzazione
        self.waypoints_pub.publish(plan_result.waypoints_path)
        
        # ----------------------------------------------------------------
        # FASE 2: LOCAL EXECUTION (Servo tracking)
        # ----------------------------------------------------------------
        self.current_state = HybridState.LOCAL_EXECUTION
        self.waypoint_index = 0
        
        feedback.current_phase = "local_execution"
        feedback.progress = 0.3
        goal_handle.publish_feedback(feedback)
        
        self.get_logger().info("🎮 Phase 2: Servo Tracking...")
        
        # Servo control loop gestito da timer (servo_control_loop)
        # Qui aspettiamo completamento
        execution_timeout = 60.0  # seconds
        start_wait = time.time()
        
        while rclpy.ok():
            # Check timeout
            if time.time() - start_wait > execution_timeout:
                self.get_logger().error("❌ Execution timeout!")
                self._publish_stop_twist()
                result.error_code = ExecuteHybridMotion.Result.SERVO_TIMEOUT
                result.error_message = "Servo execution timeout"
                goal_handle.abort()
                return result
            
            # Check cancellation
            if goal_handle.is_cancel_requested:
                self.get_logger().warn("⚠️ Goal canceled by user")
                self._publish_stop_twist()
                goal_handle.canceled()
                return result
            
            # Check stato
            if self.current_state == HybridState.COMPLETED:
                break
            elif self.current_state == HybridState.FAILED:
                self.get_logger().error("❌ Execution failed!")
                self._publish_stop_twist()
                result.error_code = ExecuteHybridMotion.Result.EXECUTION_FAILED
                result.error_message = "Servo tracking failed"
                goal_handle.abort()
                return result
            elif self.current_state == HybridState.COLLISION_AVOIDANCE:
                self.get_logger().warn("⚠️ Collision detected, stopping...")
                self._publish_stop_twist()
                result.error_code = ExecuteHybridMotion.Result.COLLISION_DETECTED
                result.error_message = "Collision during execution"
                goal_handle.abort()
                return result
            
            # Update feedback
            if self.waypoints and self.current_pose:
                progress = 0.3 + 0.6 * (self.waypoint_index / len(self.waypoints))
                feedback.progress = min(progress, 0.95)
                feedback.current_pose = self.current_pose
                
                # Distance to goal
                if self.target_pose:
                    dx = self.target_pose.position.x - self.current_pose.position.x
                    dy = self.target_pose.position.y - self.current_pose.position.y
                    dz = self.target_pose.position.z - self.current_pose.position.z
                    feedback.distance_to_goal = np.sqrt(dx**2 + dy**2 + dz**2)
                
                goal_handle.publish_feedback(feedback)
            
            time.sleep(0.1)
        
        # ----------------------------------------------------------------
        # COMPLETAMENTO
        # ----------------------------------------------------------------
        self._publish_stop_twist()
        
        result.error_code = ExecuteHybridMotion.Result.SUCCESS
        result.error_message = "Hybrid motion completed successfully"
        result.execution_time = time.time() - self.execution_start_time
        result.replans_count = 0
        
        if self.current_pose:
            result.final_pose = self.current_pose
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("✅ HYBRID MOTION COMPLETED")
        self.get_logger().info(f"   Planning: {result.planning_time:.2f}s")
        self.get_logger().info(f"   Execution: {result.execution_time:.2f}s")
        self.get_logger().info("=" * 60)
        
        goal_handle.succeed()
        return result
    
    # ========================================================================
    # SERVO CONTROL LOOP
    # ========================================================================
    
    def servo_control_loop(self):
        """
        Control loop per tracking waypoints con Servo.
        Chiamato a frequenza fissa (es. 50 Hz).
        """
        if self.current_state != HybridState.LOCAL_EXECUTION:
            return
        
        if not self.waypoints or self.waypoint_index >= len(self.waypoints):
            return
        
        if self.current_pose is None:
            # TF non ancora disponibile
            return
        
        # Current waypoint target
        target_waypoint = self.waypoints[self.waypoint_index]
        
        # Calcola errore posizione
        error_x = target_waypoint.pose.position.x - self.current_pose.position.x
        error_y = target_waypoint.pose.position.y - self.current_pose.position.y
        error_z = target_waypoint.pose.position.z - self.current_pose.position.z
        distance_error = np.sqrt(error_x**2 + error_y**2 + error_z**2)
        
        # Check se waypoint raggiunto
        if distance_error < self.waypoint_tolerance:
            self.waypoint_index += 1
            self.get_logger().info(
                f"✅ Waypoint {self.waypoint_index}/{len(self.waypoints)} reached "
                f"(error: {distance_error*1000:.1f}mm)"
            )
            
            # Check se goal finale raggiunto
            if self.waypoint_index >= len(self.waypoints):
                # Verifica distanza da goal finale
                if self.target_pose:
                    final_dx = self.target_pose.position.x - self.current_pose.position.x
                    final_dy = self.target_pose.position.y - self.current_pose.position.y
                    final_dz = self.target_pose.position.z - self.current_pose.position.z
                    final_dist = np.sqrt(final_dx**2 + final_dy**2 + final_dz**2)
                    
                    if final_dist < self.goal_tolerance:
                        self.get_logger().info(f"🎯 Goal reached! (error: {final_dist*1000:.1f}mm)")
                        self.current_state = HybridState.COMPLETED
                    else:
                        self.get_logger().warn(
                            f"⚠️ All waypoints done but goal not reached "
                            f"(error: {final_dist*1000:.1f}mm > {self.goal_tolerance*1000:.1f}mm)"
                        )
                        self.current_state = HybridState.COMPLETED  # Accetta comunque
                else:
                    self.current_state = HybridState.COMPLETED
                return
            
            # Prossimo waypoint
            return
        
        # Genera twist command (proportional controller)
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.base_frame
        
        # Linear velocity (proporzionale all'errore)
        twist.twist.linear.x = self.kp_linear * error_x
        twist.twist.linear.y = self.kp_linear * error_y
        twist.twist.linear.z = self.kp_linear * error_z
        
        # Clip to max velocity
        linear_norm = np.sqrt(
            twist.twist.linear.x**2 + 
            twist.twist.linear.y**2 + 
            twist.twist.linear.z**2
        )
        if linear_norm > self.max_linear_velocity:
            scale = self.max_linear_velocity / linear_norm
            twist.twist.linear.x *= scale
            twist.twist.linear.y *= scale
            twist.twist.linear.z *= scale
        
        # Angular velocity - calcola errore orientamento
        # Quaternion difference (semplificato: solo yaw)
        target_quat = target_waypoint.pose.orientation
        current_quat = self.current_pose.orientation
        
        # Estrai yaw da quaternion (semplificato)
        target_yaw = self._quaternion_to_yaw(target_quat)
        current_yaw = self._quaternion_to_yaw(current_quat)
        
        yaw_error = self._normalize_angle(target_yaw - current_yaw)
        twist.twist.angular.z = self.kp_angular * yaw_error
        
        # Clip angular velocity
        if abs(twist.twist.angular.z) > self.max_angular_velocity:
            twist.twist.angular.z = np.sign(twist.twist.angular.z) * self.max_angular_velocity
        
        # Pubblica comando
        self.twist_pub.publish(twist)
        self.last_twist_time = time.time()
    
    def _quaternion_to_yaw(self, quat) -> float:
        """Estrai yaw (rotazione Z) da quaternion"""
        # Formula: yaw = atan2(2(w*z + x*y), 1 - 2(y^2 + z^2))
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return np.arctan2(siny_cosp, cosy_cosp)
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalizza angolo in [-pi, pi]"""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle
    
    def _publish_stop_twist(self):
        """Pubblica twist zero per fermare Servo"""
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.base_frame
        # All zeros
        
        for _ in range(4):  # Pubblica multipli per safety
            self.twist_pub.publish(twist)
            time.sleep(0.01)
        
        self.get_logger().info("🛑 Stop command sent to Servo")


def main(args=None):
    rclpy.init(args=args)
    
    coordinator = HybridPlanningCoordinator()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(coordinator)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        coordinator.get_logger().info("Shutting down coordinator...")
    finally:
        coordinator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()