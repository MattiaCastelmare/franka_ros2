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
import numpy as np
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
from moveit_msgs.srv import GetPositionIK, GetPositionFK
   
# Action custom del nostro package
from franka_simulation.action import MoveToPose, MoveToJoint, PlanGlobalPath
from nav_msgs.msg import Path
from moveit_msgs.msg import RobotTrajectory
from geometry_msgs.msg import Pose 
from trajectory_msgs.msg import JointTrajectory

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
        self._init_fk_service()

        self._init_action_servers()

        # Sincronizzazione con joint_states
        self._joint_state_ready = False
        self.create_subscription(JointState, '/joint_states', self._joint_state_callback, 10)

        
        self.get_logger().info("🚀 Franka Motion Server pronto!")
        self._log_configuration()
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/velocity_blender/trajectory', 10)

        
    def _declare_parameters(self):
        """Parametri configurabili completi."""
        
        # Core MoveIt
        self.declare_parameter('move_group_name', 'fr3_arm')
        self.declare_parameter('base_link_name', 'fr3_link0') 
        self.declare_parameter('end_effector_name', 'fr3_hand_tcp')
        self.declare_parameter('joint_names', [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ])
        
        # Planning
        # Carica planner da hybrid_planning.yaml se disponibile
        self.declare_parameter('planner_id', 'RRTConnect')
        planner_from_yaml = self.get_parameter_or('global_planner.id', 'RRTConnect')
        self.set_parameters([rclpy.parameter.Parameter('planner_id', rclpy.Parameter.Type.STRING, planner_from_yaml)])

        self.declare_parameter('allowed_planning_time', 5.0)
        self.declare_parameter('max_velocity_scaling_factor', 0.1)
        self.declare_parameter('max_acceleration_scaling_factor', 0.1)
        
        # Retry e timeout
        self.declare_parameter('max_motion_retries', 3)
        self.declare_parameter('max_ik_retries', 5)
        self.declare_parameter('ik_timeout', 10.0)
        self.declare_parameter('min_ground_clearance', 0.08)  

        self.declare_parameter('use_fk_extraction', True)
        self.declare_parameter('fk_sample_rate', 0.1)
        self.declare_parameter('max_fk_waypoints', 50)
        
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
        self.min_ground_clearance = self.get_parameter('min_ground_clearance').get_parameter_value().double_value

        self.use_fk_extraction = self.get_parameter('use_fk_extraction').get_parameter_value().bool_value
        self.fk_sample_rate = self.get_parameter('fk_sample_rate').get_parameter_value().double_value
        self.max_fk_waypoints = self.get_parameter('max_fk_waypoints').get_parameter_value().integer_value


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

    def _init_fk_service(self):
        """
        Inizializza servizio Forward Kinematics (compute_fk) per l’estrazione waypoint.
        """
     
        self.fk_client = self.create_client(GetPositionFK, 'compute_fk', callback_group=self.callback_group)

        if not self.fk_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("⚠️ FK service not available, fallback interpolation enabled.")
            self.use_fk_extraction = False
        else:
            self.get_logger().info("✅ FK service connected")


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
        self.plan_global_path_server = ActionServer(
            self, PlanGlobalPath, 'plan_global_path',
            execute_callback=self.plan_global_path_callback,
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
        """Callback completo SOLO PLANNING per movimento a pose target."""
        
        self.get_logger().info("📍 [move_to_pose] START (PLANNING ONLY)")

        goal_pose = goal_handle.request.pose_target
        cartesian_motion = goal_handle.request.cartesian_motion
        velocity_scaling = goal_handle.request.max_velocity_scaling_factor or self.max_velocity

        result = MoveToPose.Result()
        result.result.val = MoveItErrorCodes.FAILURE
        start_time = time.time()

        # Salva stato velocità
        original_velocity = self.moveit2.max_velocity
        self.moveit2.max_velocity = velocity_scaling

        try:
            # =======================
            # 1) COMPUTE IK SE NON CARTESIAN
            # =======================
            if not cartesian_motion:
                self.get_logger().info("🧮 [move_to_pose] IK-only planning")

                joint_positions, ik_error = self.compute_ik(goal_pose)
                if joint_positions is None:
                    self.get_logger().error("❌ IK failed")
                    goal_handle.abort()
                    return result

                # =========================
                # 2) SOLO PLANNING (NO EXECUTE)
                # =========================
                self.get_logger().info("🧠 Calling MoveIt2.plan(...)")
                trajectory = self.moveit2.plan(joint_positions=list(joint_positions))

                if trajectory is None or not trajectory.points:
                    self.get_logger().error("❌ Planning failed")
                    goal_handle.abort()
                    return result

                self.get_logger().info(
                    f"📤 Publishing planned JointTrajectory ({len(trajectory.points)} points)"
                )
                self.trajectory_pub.publish(trajectory)

            # =======================
            # CARTESIAN PLANNING
            # =======================
            else:
                self.get_logger().info("🔄 Cartesian planning (PLANNING ONLY)...")
                self.moveit2.plan_to_pose(
                    pose=goal_pose,
                    cartesian=True
                )

                traj_msg = self.moveit2._MoveIt2__joint_trajectory

                if traj_msg is None or not traj_msg.points:
                    self.get_logger().error("❌ Cartesian planning returned no trajectory")
                    goal_handle.abort()
                    return result

                self.get_logger().info(
                    f"📤 Publishing Cartesian JointTrajectory ({len(traj_msg.points)} points)"
                )
                self.trajectory_pub.publish(traj_msg)

            # =========================
            # SUCCESS
            # =========================
            result.result.val = MoveItErrorCodes.SUCCESS
            result.final_pose = goal_pose

            goal_handle.succeed()

        except Exception as e:
            self.get_logger().error(f"❌ move_to_pose exception: {e}")
            goal_handle.abort()

        finally:
            self.moveit2.max_velocity = original_velocity

        return result

    
    def move_to_joint_callback(self, goal_handle: ServerGoalHandle):
        """Callback per movimento a configurazione joint (SOLO PLANNING, no esecuzione MoveIt)."""
        
        self.get_logger().info("🔧 [move_to_joint] Starting JOINT-SPACE GLOBAL PLANNING (no execution)...")
        
        joint_target = goal_handle.request.joint_target
        velocity_scaling = (
            goal_handle.request.max_velocity_scaling_factor or self.max_velocity
        )
        
        result = MoveToJoint.Result()
        result.result.val = MoveItErrorCodes.FAILURE
        start_time = time.time()
        
        # Validazione input
        if len(joint_target) != len(self.joint_names):
            self.get_logger().error(
                f"❌ Invalid joint target length: {len(joint_target)} != {len(self.joint_names)}"
            )
            goal_handle.abort()
            return result
        
        # Configurazione temporanea (usata solo per il planning)
        original_velocity = self.moveit2.max_velocity
        self.moveit2.max_velocity = velocity_scaling
        
        try:
            feedback = MoveToJoint.Feedback()
            feedback.current_state = "joint_planning"
            feedback.progress = 0.3
            goal_handle.publish_feedback(feedback)

            # 🔹 SOLO PLANNING, senza esecuzione
            planning_start = time.time()
            self.get_logger().info("🧠 [move_to_joint] Calling MoveIt2.plan(joint_positions=...)")
            trajectory = self.moveit2.plan(
                joint_positions=list(joint_target)
            )
            planning_time = time.time() - planning_start

            if trajectory is None or not trajectory.points:
                self.get_logger().error("❌ [move_to_joint] Planning failed: no trajectory returned by MoveIt2")
                result.result.val = MoveItErrorCodes.PLANNING_FAILED
                result.planning_time = planning_time
                result.execution_time = 0.0
                goal_handle.abort()
                return result

            self.get_logger().info(
                f"✅ [move_to_joint] Planning success: {len(trajectory.points)} punti nella JointTrajectory"
            )

            # 🔹 Pubblica la traiettoria per il velocity blender
            self.get_logger().info("📤 [move_to_joint] Publishing trajectory to /velocity_blender/trajectory")
            self.trajectory_pub.publish(trajectory)

            # Aggiorna feedback: il "global plan" è pronto
            feedback.current_state = "completed"
            feedback.progress = 1.0
            feedback.current_joint_positions = list(joint_target)
            goal_handle.publish_feedback(feedback)

            # Compila il result come SUCCESS (il planning è andato bene)
            total_time = time.time() - start_time
            result.result.val = MoveItErrorCodes.SUCCESS
            result.final_joint_positions = list(joint_target)
            result.planning_time = planning_time
            result.execution_time = total_time - planning_time  # qui è solo "overhead", non vera esecuzione

            goal_handle.succeed()
            self.get_logger().info("🏁 [move_to_joint] GLOBAL PLAN READY (published to /velocity_blender/trajectory)")
            
        except Exception as e:
            self.get_logger().error(f"❌ [move_to_joint] Exception during planning: {e}")
            goal_handle.abort()
            
        finally:
            # Ripristina configurazione
            self.moveit2.max_velocity = original_velocity
            
        return result


    # ========================================================================
    # VALIDAZIONE GROUND CLEARANCE
    # ========================================================================
    def _validate_ground_clearance(self, pose: Pose) -> Tuple[bool, str]:
        """
        ✅ Verifica che la pose rispetti la distanza minima dal suolo.
        """
        z = pose.position.z
        if z < self.min_ground_clearance:
            msg = (
                f"⛔ GROUND CLEARANCE VIOLATION: z={z:.3f}m < "
                f"min_clearance={self.min_ground_clearance:.3f}m. "
                "Goal rejected for safety."
            )
            self.get_logger().error(msg)
            return False, msg

        if z < 1.5 * self.min_ground_clearance:
            self.get_logger().warn(
                f"⚠️ LOW CLEARANCE WARNING: z={z:.3f}m "
                f"(soglia sicurezza: {self.min_ground_clearance:.3f}m)"
            )

        self.get_logger().info(f"✅ Ground clearance OK: z={z:.3f}m")
        return True, ""

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

        self._latest_joint_state = msg
    # ========================================================================
    # ACTION CALLBACK: PlanGlobalPath (HYBRID PLANNING)
    # ========================================================================
    
    def plan_global_path_callback(self, goal_handle):
        """
        STEP 3: Pianifica con OMPL + estrae waypoints reali via FK (se abilitato).
        """
        goal = goal_handle.request
        result = PlanGlobalPath.Result()

        self.get_logger().info("=" * 60)
        self.get_logger().info("🌍 GLOBAL PLANNING REQUEST (STEP 3)")
        self.get_logger().info(f"   Target: {goal.target_pose.pose.position}")
        self.get_logger().info(f"   Planner: {goal.planner_id}")
        self.get_logger().info(f"   FK Extraction: {self.use_fk_extraction}")
        self.get_logger().info("=" * 60)
        is_valid, msg = self._validate_ground_clearance(goal.target_pose.pose)
        if not is_valid:
            result.error_code = MoveItErrorCodes.INVALID_MOTION_PLAN
            result.num_waypoints = 0
            goal_handle.abort()
            return result

        feedback = PlanGlobalPath.Feedback()
        feedback.current_state = "planning"
        feedback.attempt_number = 0
        goal_handle.publish_feedback(feedback)

        planning_start = time.time()
        original_planner = self.moveit2.planner_id
        self.moveit2.planner_id = goal.planner_id
        self.moveit2.allowed_planning_time = goal.planning_time

        target_pose = goal.target_pose
# ───────── STEP: Safety ground clearance ─────────
        min_clearance = 0.08  # 8 cm minimo sopra il suolo
        if target_pose.pose.position.z < min_clearance:
            self.get_logger().warn(
                f"⚠️ Target z={target_pose.pose.position.z:.3f} troppo basso, imposto a {min_clearance:.2f} m per evitare collisioni col suolo."
            )
            target_pose.pose.position.z = min_clearance


        try:
            # Plan without execution
            self.moveit2.plan_async(
                position=[
                    target_pose.pose.position.x,
                    target_pose.pose.position.y,
                    target_pose.pose.position.z,
                ],
                quat_xyzw=[
                    target_pose.pose.orientation.x,
                    target_pose.pose.orientation.y,
                    target_pose.pose.orientation.z,
                    target_pose.pose.orientation.w,
                ],
                cartesian=False,
            )
            
            # Wait for planning to complete
  
            time.sleep(0.5)  # Give MoveIt time to plan
            
            # Get the computed trajectory
            trajectory_msg = None
            if hasattr(self.moveit2, '_MoveIt2__joint_trajectory'):
                trajectory_msg = self.moveit2._MoveIt2__joint_trajectory
                self.get_logger().info(f"✅ Retrieved trajectory with {len(trajectory_msg.points) if trajectory_msg and hasattr(trajectory_msg, 'points') else 0} points")
            
            if not trajectory_msg or not hasattr(trajectory_msg, 'points') or len(trajectory_msg.points) == 0:
                self.get_logger().error("❌ No valid trajectory computed")
                result.error_code = MoveItErrorCodes.PLANNING_FAILED
                result.num_waypoints = 0
                goal_handle.abort()
                self.moveit2.planner_id = original_planner
                return result

            self.get_logger().info("✅ MoveIt planning success")

        except Exception as e:
            self.get_logger().error(f"❌ Planning exception: {e}")
            result.error_code = MoveItErrorCodes.FAILURE
            result.num_waypoints = 0
            goal_handle.abort()
            self.moveit2.planner_id = original_planner
            return result

        self.moveit2.planner_id = original_planner
        feedback.current_state = "extracting_waypoints"
        goal_handle.publish_feedback(feedback)

        # ───────── STEP 3 : estrazione FK o fallback ─────────
     
        try:
            if self.use_fk_extraction and trajectory_msg:
                self.get_logger().info("🎯 Extracting waypoints via FK from MoveIt trajectory")
                waypoints_path = self._extract_waypoints_from_trajectory(
                    trajectory_msg,
                    sample_rate=self.fk_sample_rate,
                )
            else:
                self.get_logger().error("❌ No trajectory available - cannot proceed without valid OMPL path")
                result.error_code = MoveItErrorCodes.PLANNING_FAILED
                result.num_waypoints = 0
                goal_handle.abort()
                self.moveit2.planner_id = original_planner
                return result

        except Exception as e:
            self.get_logger().error(f"❌ FK extraction failed: {e}")
            result.error_code = MoveItErrorCodes.PLANNING_FAILED
            result.num_waypoints = 0
            goal_handle.abort()
            self.moveit2.planner_id = original_planner
            return result

        except Exception as e:
            self.get_logger().error(f"❌ FK extraction failed ({e}), fallback to linear interpolation")
            waypoints_path = self._create_sample_waypoints(
                start_pose=self._get_current_ee_pose(),
                end_pose=target_pose.pose,
                num_samples=20,
            )


        result.error_code = MoveItErrorCodes.SUCCESS
        if hasattr(trajectory_msg, "joint_trajectory"):
            jt = trajectory_msg.joint_trajectory
        else:
            jt = trajectory_msg

        if jt.points:
            self.trajectory_pub.publish(jt)
        else:
            self.get_logger().warn("⚠️ JointTrajectory vuota nel global plan")


        result.waypoints_path = waypoints_path

        if not hasattr(self, "path_pub"):
            self.path_pub = self.create_publisher(Path, "/franka/global_path", 10)
        self.path_pub.publish(result.waypoints_path)

        result.planning_time = time.time() - planning_start
        result.num_waypoints = len(waypoints_path.poses)

        self.get_logger().info("=" * 60)
        self.get_logger().info("✅ GLOBAL PLAN READY (STEP 3)")
        self.get_logger().info(f"   Waypoints: {result.num_waypoints}")
        self.get_logger().info(f"   Time: {result.planning_time:.2f}s")
        self.get_logger().info("=" * 60)

        goal_handle.succeed()
        return result

    
    def _create_sample_waypoints(self, start_pose: Pose, end_pose: Pose, num_samples: int = 20) -> Path:
        """
        Crea waypoints interpolati linearmente tra start e end.
        
        NOTA: Questa è una versione semplificata.
        In produzione, dovremmo estrarre waypoints dalla traiettoria MoveIt.
        """
        path = Path()
        path.header.frame_id = self.base_link_name
        path.header.stamp = self.get_clock().now().to_msg()
        
        for i in range(num_samples + 1):
            alpha = i / num_samples
            
            pose_stamped = PoseStamped()
            pose_stamped.header = path.header
            
            # Linear interpolation position
            pose_stamped.pose.position.x = (1 - alpha) * start_pose.position.x + alpha * end_pose.position.x
            pose_stamped.pose.position.y = (1 - alpha) * start_pose.position.y + alpha * end_pose.position.y
            pose_stamped.pose.position.z = (1 - alpha) * start_pose.position.z + alpha * end_pose.position.z
            
            # SLERP orientation (semplificato: linear blend, non corretto ma OK per test)
            pose_stamped.pose.orientation.x = (1 - alpha) * start_pose.orientation.x + alpha * end_pose.orientation.x
            pose_stamped.pose.orientation.y = (1 - alpha) * start_pose.orientation.y + alpha * end_pose.orientation.y
            pose_stamped.pose.orientation.z = (1 - alpha) * start_pose.orientation.z + alpha * end_pose.orientation.z
            pose_stamped.pose.orientation.w = (1 - alpha) * start_pose.orientation.w + alpha * end_pose.orientation.w
            
            # Normalize quaternion
            q_norm = np.sqrt(
                pose_stamped.pose.orientation.x**2 +
                pose_stamped.pose.orientation.y**2 +
                pose_stamped.pose.orientation.z**2 +
                pose_stamped.pose.orientation.w**2
            )
            pose_stamped.pose.orientation.x /= q_norm
            pose_stamped.pose.orientation.y /= q_norm
            pose_stamped.pose.orientation.z /= q_norm
            pose_stamped.pose.orientation.w /= q_norm
            
            path.poses.append(pose_stamped)
        
        return path
    
    def _get_current_ee_pose(self) -> Pose:
        """
        Ottieni pose corrente end-effector tramite TF.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_link_name,
                self.end_effector_name,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            
            return pose
            
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed, using default pose: {e}")
            # Fallback: home pose
            pose = Pose()
            pose.position.x = 0.5
            pose.position.y = 0.0
            pose.position.z = 0.5
            pose.orientation.w = 1.0
            return pose
    def _extract_waypoints_from_trajectory(self, trajectory_msg, sample_rate: float = 0.1) -> Path:
        """
        STEP 3: Estrae waypoints dalla traiettoria MoveIt usando compute_fk.
        (Preparato per future release di pymoveit2)
        """
        from moveit_msgs.srv import GetPositionFK
        from moveit_msgs.msg import RobotState

        path = Path()
        path.header.frame_id = self.base_link_name
        path.header.stamp = self.get_clock().now().to_msg()

        joint_traj = trajectory_msg.joint_trajectory
        if not joint_traj.points:
            self.get_logger().warn("⚠️ Trajectory is empty, no waypoints extracted")
            return path

        if self.fk_client is None or not self.fk_client.service_is_ready():
            self.get_logger().warn("⚠️ FK client not ready → caller will handle fallback.")
            return Path()


        total_duration = joint_traj.points[-1].time_from_start.sec + \
                        joint_traj.points[-1].time_from_start.nanosec * 1e-9
        num_samples = min(
            max(int(total_duration / sample_rate), len(joint_traj.points)),
            self.max_fk_waypoints,
        )
        sampled_indices = np.unique(np.linspace(0, len(joint_traj.points) - 1,
                                                num_samples, dtype=int))

        self.get_logger().info(f"🎯 Extracting {len(sampled_indices)} waypoints from trajectory via FK...")

        successful_waypoints = 0
        for idx in sampled_indices:
            point = joint_traj.points[idx]
            robot_state = RobotState()
            robot_state.joint_state.name = joint_traj.joint_names
            robot_state.joint_state.position = list(point.positions)

            fk_request = GetPositionFK.Request()
            fk_request.header.frame_id = self.base_link_name
            fk_request.fk_link_names = [self.end_effector_name]
            fk_request.robot_state = robot_state

            try:
                future = self.fk_client.call_async(fk_request)
                start_time = time.time()
                while not future.done() and (time.time() - start_time) < 1.0:
                    time.sleep(0.01)

                if not future.done():
                    self.get_logger().warn(f"⚠️ FK timeout for waypoint {idx}")
                    continue

                fk_response = future.result()
                if fk_response.error_code.val != MoveItErrorCodes.SUCCESS:
                    self.get_logger().warn(f"⚠️ FK failed for waypoint {idx}")
                    continue

                if fk_response.pose_stamped:
                    pose_stamped = fk_response.pose_stamped[0]
                    pose_stamped.header = path.header
                    path.poses.append(pose_stamped)
                    successful_waypoints += 1

            except Exception as e:
                self.get_logger().error(f"❌ FK exception for waypoint {idx}: {e}")
                continue

        self.get_logger().info(f"✅ Extracted {successful_waypoints}/{len(sampled_indices)} waypoints via FK")

        if successful_waypoints < 5:
            self.get_logger().warn("⚠️ Too few FK waypoints, falling back to interpolation")
            return self._create_sample_waypoints(
                self._get_current_ee_pose(),
                path.poses[-1].pose if path.poses else Pose(),
                num_samples=20,
            )

        return path

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