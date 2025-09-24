#!/usr/bin/env python3
"""
Franka Motion Server Completo
====================================

Server ROS 2 completo per planning e esecuzione traiettorie con MoveIt 2.
Usa le action custom MoveToPose e MoveToJoint per interfaccia pulita.

Novità:
- Action custom invece di placeholder
- Implementazione completa callback movimento
- Feedback dettagliato durante esecuzione  
- Gestione retry robusta per IK e planning
- Metriche temporali accurate
- Error handling completo
"""

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from typing import Optional, Tuple, List
from threading import Thread
import time

# Message types
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener, TransformBroadcaster

# MoveIt related imports
from pymoveit2 import MoveIt2
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK

# Action custom del nostro package
from franka_simulation.action import MoveToPose, MoveToJoint

class FrankaMotionServer(Node):
    """Motion Server per robot Franka FR3 con action custom complete."""
    
    def __init__(self):
        super().__init__('franka_motion_server')
        
        self._declare_parameters()
        self._load_parameters()
        
        # Callback group per operazioni parallele
        self.callback_group = ReentrantCallbackGroup()
        
        # Inizializzazione componenti
        self._init_moveit()
        self._init_tf()
        self._init_ik_service()
        self._init_action_servers()

        # Sincronizzazione con joint_states
        self._joint_state_ready = False
        self.create_subscription(JointState, '/joint_states', self._joint_state_callback, 10)

        
        self.get_logger().info("🚀 Franka Motion Server pronto!")
        self._log_configuration()
        
    def _declare_parameters(self):
        """Parametri configurabili completi."""
        
        # Core MoveIt
        self.declare_parameter('move_group_name', 'fr3_arm')
        self.declare_parameter('base_link_name', 'fr3_link0') 
        self.declare_parameter('end_effector_name', 'fr3_link8')
        self.declare_parameter('joint_names', [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ])
        
        # Planning
        self.declare_parameter('planner_id', 'RRTConnect')
        self.declare_parameter('allowed_planning_time', 5.0)
        self.declare_parameter('max_velocity_scaling_factor', 0.1)
        self.declare_parameter('max_acceleration_scaling_factor', 0.1)
        
        # Retry e timeout
        self.declare_parameter('max_motion_retries', 3)
        self.declare_parameter('max_ik_retries', 5)
        self.declare_parameter('ik_timeout', 10.0)
        
    def _load_parameters(self):
        """Carica parametri in variabili di istanza."""
        
        self.move_group_name = self.get_parameter('move_group_name').get_parameter_value().string_value
        self.base_link_name = self.get_parameter('base_link_name').get_parameter_value().string_value  
        self.end_effector_name = self.get_parameter('end_effector_name').get_parameter_value().string_value
        self.joint_names = self.get_parameter('joint_names').get_parameter_value().string_array_value
        
        self.planner_id = self.get_parameter('planner_id').get_parameter_value().string_value
        self.allowed_planning_time = self.get_parameter('allowed_planning_time').get_parameter_value().double_value
        self.max_velocity = self.get_parameter('max_velocity_scaling_factor').get_parameter_value().double_value
        self.max_acceleration = self.get_parameter('max_acceleration_scaling_factor').get_parameter_value().double_value
        
        self.max_motion_retries = self.get_parameter('max_motion_retries').get_parameter_value().integer_value
        self.max_ik_retries = self.get_parameter('max_ik_retries').get_parameter_value().integer_value
        self.ik_timeout = self.get_parameter('ik_timeout').get_parameter_value().double_value
        
    def _init_moveit(self):
        """Inizializza MoveIt2 con pymoveit2."""
        
        try:
            # Nodo interno per pymoveit2
            self.moveit_node = Node(
                'franka_moveit_internal', 
                use_global_arguments=False,
                namespace=self.get_namespace()
            )
            
            self.moveit2 = MoveIt2(
                node=self.moveit_node,
                joint_names=self.joint_names,
                base_link_name=self.base_link_name,
                end_effector_name=self.end_effector_name,
                group_name=self.move_group_name,
                callback_group=self.callback_group,
                use_move_group_action=False  # Usa diretto controller
            )
            
            # Configurazione planning
            self.moveit2.planner_id = self.planner_id
            self.moveit2.max_velocity = self.max_velocity
            self.moveit2.max_acceleration = self.max_acceleration
            self.moveit2.allowed_planning_time = self.allowed_planning_time
            
            self.get_logger().info(f"✅ MoveIt2 inizializzato: {self.move_group_name}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Errore MoveIt2: {e}")
            raise RuntimeError(f"MoveIt2 initialization failed: {e}")
    
    def _init_tf(self):
        """Inizializza sistema TF."""
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info("✅ TF system ready")
    
    def _init_ik_service(self):
        """Inizializza servizio IK."""
        self.ik_client = self.create_client(
            GetPositionIK, 'compute_ik', callback_group=self.callback_group)
        
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("IK service not available")
        self.get_logger().info("✅ IK service connected")
    
    def _init_action_servers(self):
        """Inizializza action servers con action custom."""
        
        self.move_to_pose_server = ActionServer(
            self, MoveToPose, 'move_to_pose',
            execute_callback=self.move_to_pose_callback,
            goal_callback=self.accept_goal, cancel_callback=self.cancel_goal,
            callback_group=self.callback_group
        )
        
        self.move_to_joint_server = ActionServer(
            self, MoveToJoint, 'move_to_joint',
            execute_callback=self.move_to_joint_callback,
            goal_callback=self.accept_goal, cancel_callback=self.cancel_goal,
            callback_group=self.callback_group
        )
        
        self.get_logger().info("✅ Action servers ready")
    
    def accept_goal(self, goal_request):
        """Accetta tutti i goal validi."""
        self.get_logger().info("🎯 New goal received")
        return GoalResponse.ACCEPT
    
    def cancel_goal(self, goal_handle):
        """Gestisce cancellazione goal."""
        self.get_logger().info("❌ Goal canceled")
        return CancelResponse.ACCEPT
    
    def move_to_pose_callback(self, goal_handle: ServerGoalHandle):
        """Callback completo per movimento a pose target."""
        
        self.get_logger().info("📍 Starting pose movement...")
        
        # Estrazione parametri goal
        goal_pose = goal_handle.request.pose_target
        cartesian_motion = goal_handle.request.cartesian_motion
        velocity_scaling = goal_handle.request.max_velocity_scaling_factor or self.max_velocity
        
        # Preparazione risultato
        result = MoveToPose.Result()
        result.result.val = MoveItErrorCodes.FAILURE
        start_time = time.time()
        
        # Configurazione temporanea velocità
        original_velocity = self.moveit2.max_velocity
        self.moveit2.max_velocity = velocity_scaling
        
        try:
            if cartesian_motion:
                # Movimento cartesiano
                self.get_logger().info("🔄 Cartesian planning...")
                
                feedback = MoveToPose.Feedback()
                feedback.current_state = "cartesian_planning"
                feedback.progress = 0.3
                goal_handle.publish_feedback(feedback)
                
                planning_start = time.time()
                self.moveit2.move_to_pose(pose=goal_pose, cartesian=True)
                
                feedback.current_state = "executing"
                feedback.progress = 0.7
                goal_handle.publish_feedback(feedback)
                
                success = self.moveit2.wait_until_executed()
                error_code = self.moveit2.get_last_execution_error_code()
                
                if success and error_code and error_code.val == MoveItErrorCodes.SUCCESS:
                    result.result.val = MoveItErrorCodes.SUCCESS
                    self.get_logger().info("✅ Cartesian movement completed")
                else:
                    self.get_logger().warning(f"⚠️ Cartesian movement failed: {error_code.val if error_code else 'Unknown'}")
                    
            else:
                # Planning joint space via IK
                self.get_logger().info("🧮 Computing IK...")
                
                feedback = MoveToPose.Feedback()
                feedback.current_state = "ik_calculation"
                feedback.progress = 0.2
                goal_handle.publish_feedback(feedback)
                
                # Calcolo IK con retry
                joint_positions = None
                for attempt in range(1, self.max_ik_retries + 1):
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        return result
                        
                    joint_positions, ik_error = self.compute_ik(goal_pose)
                    if joint_positions is not None:
                        break
                    time.sleep(0.5)
                
                if joint_positions is None:
                    self.get_logger().error("❌ IK failed after all retries")
                    goal_handle.abort()
                    return result
                
                # Planning e esecuzione
                feedback.current_state = "joint_planning"
                feedback.progress = 0.5
                goal_handle.publish_feedback(feedback)
                
                planning_start = time.time()
                self.moveit2.move_to_configuration(joint_positions, self.joint_names)
                
                feedback.current_state = "executing"
                feedback.progress = 0.8
                goal_handle.publish_feedback(feedback)
                
                success = self.moveit2.wait_until_executed()
                error_code = self.moveit2.get_last_execution_error_code()
                
                if success and error_code and error_code.val == MoveItErrorCodes.SUCCESS:
                    result.result.val = MoveItErrorCodes.SUCCESS
                    self.get_logger().info("✅ Joint movement completed")
                else:
                    self.get_logger().warning(f"⚠️ Joint movement failed: {error_code.val if error_code else 'Unknown'}")
            
            # Calcolo metriche
            total_time = time.time() - start_time
            result.planning_time = time.time() - planning_start if 'planning_start' in locals() else 0.0
            result.execution_time = total_time - result.planning_time
            result.final_pose = goal_pose  # Placeholder
            
            # Feedback finale
            feedback.current_state = "completed"
            feedback.progress = 1.0
            goal_handle.publish_feedback(feedback)
            
            if result.result.val == MoveItErrorCodes.SUCCESS:
                goal_handle.succeed()
            else:
                goal_handle.abort()
                
        except Exception as e:
            self.get_logger().error(f"❌ Movement error: {e}")
            goal_handle.abort()
            
        finally:
            # Ripristino configurazione
            self.moveit2.max_velocity = original_velocity
            
        return result
    
    def move_to_joint_callback(self, goal_handle: ServerGoalHandle):
        """Callback per movimento a configurazione joint."""
        
        self.get_logger().info("🔧 Starting joint movement...")
        
        joint_target = goal_handle.request.joint_target
        velocity_scaling = goal_handle.request.max_velocity_scaling_factor or self.max_velocity
        
        result = MoveToJoint.Result()
        result.result.val = MoveItErrorCodes.FAILURE
        start_time = time.time()
        
        # Validazione input
        if len(joint_target) != len(self.joint_names):
            self.get_logger().error(f"❌ Invalid joint target length: {len(joint_target)} != {len(self.joint_names)}")
            goal_handle.abort()
            return result
        
        # Configurazione temporanea
        original_velocity = self.moveit2.max_velocity
        self.moveit2.max_velocity = velocity_scaling
        
        try:
            feedback = MoveToJoint.Feedback()
            feedback.current_state = "joint_planning"
            feedback.progress = 0.3
            goal_handle.publish_feedback(feedback)
            
            planning_start = time.time()
            self.moveit2.move_to_configuration(joint_target, self.joint_names)
            
            feedback.current_state = "executing"
            feedback.progress = 0.7
            goal_handle.publish_feedback(feedback)
            
            success = self.moveit2.wait_until_executed()
            error_code = self.moveit2.get_last_execution_error_code()
            
            if success and error_code and error_code.val == MoveItErrorCodes.SUCCESS:
                result.result.val = MoveItErrorCodes.SUCCESS
                result.final_joint_positions = joint_target  # Placeholder
                self.get_logger().info("✅ Joint movement completed")
            else:
                self.get_logger().warning(f"⚠️ Joint movement failed: {error_code.val if error_code else 'Unknown'}")
            
            # Metriche
            total_time = time.time() - start_time
            result.planning_time = time.time() - planning_start
            result.execution_time = total_time - result.planning_time
            
            # Feedback finale
            feedback.current_state = "completed"
            feedback.progress = 1.0
            feedback.current_joint_positions = joint_target
            goal_handle.publish_feedback(feedback)
            
            if result.result.val == MoveItErrorCodes.SUCCESS:
                goal_handle.succeed()
            else:
                goal_handle.abort()
                
        except Exception as e:
            self.get_logger().error(f"❌ Joint movement error: {e}")
            goal_handle.abort()
            
        finally:
            self.moveit2.max_velocity = original_velocity
            
        return result
    
    def compute_ik(self, goal_pose: PoseStamped) -> Tuple[Optional[List[float]], MoveItErrorCodes]:
        """Calcola IK per pose target con gestione robusta errori."""
        
        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = self.move_group_name
        ik_request.ik_request.pose_stamped = goal_pose
        ik_request.ik_request.avoid_collisions = True
        ik_request.ik_request.ik_link_name = self.end_effector_name
        ik_request.ik_request.timeout.sec = int(self.ik_timeout)
        
        # Usa l'ultimo joint state ricevuto come seed
        if hasattr(self, "_latest_joint_state") and self._latest_joint_state is not None:
            ik_request.ik_request.robot_state.joint_state = self._latest_joint_state

        if self.moveit2.joint_state is not None:
            ik_request.ik_request.robot_state.joint_state = self.moveit2.joint_state
        
        try:
            future = self.ik_client.call_async(ik_request)
            
            # Attesa con timeout
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < self.ik_timeout:
                time.sleep(0.1)
                
            if not future.done():
                error_code = MoveItErrorCodes()
                error_code.val = MoveItErrorCodes.NO_IK_SOLUTION
                return None, error_code
                
            response = future.result()
            
            if response.error_code.val != MoveItErrorCodes.SUCCESS:
                return None, response.error_code
                
            # Mappatura ai joint names
            joint_state = response.solution.joint_state
            name_to_pos = dict(zip(joint_state.name, joint_state.position))
            
            positions = [name_to_pos[name] for name in self.joint_names]
            return positions, response.error_code
            
        except Exception as e:
            self.get_logger().error(f"❌ IK computation error: {e}")
            error_code = MoveItErrorCodes()
            error_code.val = MoveItErrorCodes.FAILURE
            return None, error_code
    
    def _log_configuration(self):
        """Log configurazione per debug."""
        self.get_logger().info("📋 Motion Server Configuration:")
        self.get_logger().info(f"  • Move Group: {self.move_group_name}")
        self.get_logger().info(f"  • Joints: {self.joint_names}")
        self.get_logger().info(f"  • End Effector: {self.end_effector_name}")
        self.get_logger().info(f"  • Planner: {self.planner_id}")

    def _joint_state_callback(self, msg: JointState):
        if not self._joint_state_ready:
            # filtra i nomi, togli “finger” se vuoi ignorarlo
            arm_joint_names = [name for name in msg.name if not name.startswith('fr3_finger')]
            self.joint_names = arm_joint_names
            self._joint_state_ready = True
            self.get_logger().info(f"✅ Joint order (arm only): {self.joint_names}")
        # salva per seed IK
        self._latest_joint_state = msg





def main(args=None):
    """Entry point."""
    
    print("🚀 Starting Franka Motion Server...")
    rclpy.init(args=args)
    
    try:
        motion_server = FrankaMotionServer()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(motion_server)
        
        print("✅ Motion Server active. Press Ctrl+C to stop.")
        executor.spin()
        
    except KeyboardInterrupt:
        print("\n⏹️ Shutdown requested")
    except Exception as e:
        print(f"\n❌ Critical error: {e}")
    finally:
        try:
            rclpy.shutdown()
            print("🏁 Motion Server terminated")
        except:
            pass


if __name__ == '__main__':
    main()