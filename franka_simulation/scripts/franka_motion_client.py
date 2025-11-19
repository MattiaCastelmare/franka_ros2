#!/usr/bin/env python3
"""
Franka Motion Client - Interface Semplificata
=============================================

Client ROS 2 per comunicare con il Franka Motion Server.
Fornisce API Python semplici e sincrone per controllare il robot FR3.

Funzionalità principali:
- move_to_pose(): Movimento a pose target con/senza cartesian planning
- move_to_joint(): Movimento a configurazione joint specifica  
- get_current_pose(): Ottieni pose corrente end-effector
- get_current_joints(): Ottieni configurazione joint corrente
- API sincrone e asincrone disponibili

Adattato da DRIMS2 Motion Client con miglioramenti:
- Gestione feedback dettagliato
- Timeout configurabili
- Error handling robusto
- Logging informativo
- Support per multiple goal simultaneously
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from typing import Tuple, List, Optional
import time

# Message types
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# MoveIt error codes
from moveit_msgs.msg import MoveItErrorCodes

# Action custom del nostro package
from franka_simulation.action import MoveToPose, MoveToJoint

# TF per trasformazioni coordinate
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from franka_simulation.action import PlanGlobalPath



class FrankaMotionClient(Node):
    """
    Client semplificato per controllo robot Franka FR3.
    
    Fornisce API Python intuitive per:
    - Movimento a pose target (con/senza planning cartesiano)
    - Movimento a configurazione joint
    - Query stato corrente robot
    - Monitoraggio feedback durante esecuzione
    """
    
    def __init__(self, 
                 move_to_pose_action_name: str = 'move_to_pose',
                 move_to_joint_action_name: str = 'move_to_joint',
                 timeout_sec: float = 30.0):
        """
        Inizializza Motion Client.
        
        Args:
            move_to_pose_action_name: Nome action server per pose movements
            move_to_joint_action_name: Nome action server per joint movements  
            timeout_sec: Timeout default per action calls
        """
        super().__init__('franka_motion_client')
        
        # Parametri configurabili
        self.declare_parameter('default_timeout', timeout_sec)
        self.declare_parameter('default_velocity_scaling', 0.1)
        self.declare_parameter('default_acceleration_scaling', 0.1)
        self.declare_parameter('enable_feedback_logging', True)
        
        self.default_timeout = self.get_parameter('default_timeout').get_parameter_value().double_value
        self.default_velocity = self.get_parameter('default_velocity_scaling').get_parameter_value().double_value
        self.default_acceleration = self.get_parameter('default_acceleration_scaling').get_parameter_value().double_value
        self.enable_feedback_logging = self.get_parameter('enable_feedback_logging').get_parameter_value().bool_value
        
        # Callback group per operazioni parallele
        self.callback_group = ReentrantCallbackGroup()
        
        # Inizializzazione action clients
        self._init_action_clients(move_to_pose_action_name, move_to_joint_action_name)
        
        # Inizializzazione TF per trasformazioni coordinate
        self._init_tf()
        
        # Sottoscrizione a joint_states per stato corrente
        self._init_state_subscribers()
        
        # Variabili stato
        self.current_joint_state: Optional[JointState] = None
        self.last_feedback: Optional[str] = None
        
        self.get_logger().info("🚀 Franka Motion Client inizializzato e pronto!")
        self._log_configuration()
        
    def _init_action_clients(self, pose_action_name: str, joint_action_name: str):
        """Inizializza i client per gli action servers."""
        
        self.move_to_pose_client = ActionClient(
            self, MoveToPose, pose_action_name, 
            callback_group=self.callback_group
        )
        
        self.move_to_joint_client = ActionClient(
            self, MoveToJoint, joint_action_name,
            callback_group=self.callback_group
        )
        
        # NEW — Plan Global Path client
        self.plan_global_path_client = ActionClient(
            self, PlanGlobalPath, 'plan_global_path',
            callback_group=self.callback_group
        )

        if not self.plan_global_path_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("Action server 'plan_global_path' non disponibile")

        # Attesa disponibilità action servers
        self.get_logger().info("⏳ Attesa action servers...")
        
        if not self.move_to_pose_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f"Action server '{pose_action_name}' non disponibile")
            
        if not self.move_to_joint_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f"Action server '{joint_action_name}' non disponibile")
        
        self.get_logger().info("✅ Action servers collegati")
        
    def _init_tf(self):
        """Inizializza sistema TF per trasformazioni coordinate."""
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info("✅ TF system ready")
        
    def _init_state_subscribers(self):
        """Inizializza sottoscrizioni per stato robot corrente."""
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, 10,
            callback_group=self.callback_group
        )
        self.get_logger().info("✅ State subscribers ready")
        
    def _joint_state_callback(self, msg: JointState):
        """Callback per aggiornamento stato joint corrente."""
        self.current_joint_state = msg
        
    def _feedback_callback(self, feedback_msg):
        """Callback per feedback durante esecuzione movimento."""
        feedback = feedback_msg.feedback
        
        if self.enable_feedback_logging:
            progress = feedback.progress * 100
            self.get_logger().info(f"🔄 {feedback.current_state}: {progress:.1f}%")
            
        self.last_feedback = feedback.current_state
        
    # ========================================================================
    # API PUBBLICHE - MOVIMENTO
    # ========================================================================
    
    def move_to_pose(self, 
                     pose_target: PoseStamped,
                     cartesian_motion: bool = False,
                     velocity_scaling: Optional[float] = None,
                     acceleration_scaling: Optional[float] = None,
                     tolerance_position: float = 0.001,
                     tolerance_orientation: float = 0.017,
                     timeout_sec: Optional[float] = None) -> MoveItErrorCodes:
        """
        Muove il robot a una pose target.
        
        Args:
            pose_target: Pose target (PoseStamped)
            cartesian_motion: Se True, usa planning cartesiano
            velocity_scaling: Factor velocità (0.0-1.0), None=default
            acceleration_scaling: Factor accelerazione (0.0-1.0), None=default  
            tolerance_position: Tolleranza posizione [m]
            tolerance_orientation: Tolleranza orientamento [rad]
            timeout_sec: Timeout movimento, None=default
            
        Returns:
            MoveItErrorCodes: Risultato movimento
            
        Raises:
            RuntimeError: Se action server non disponibile o goal rifiutato
            TimeoutError: Se movimento supera timeout
        """
        
        self.get_logger().info(f"📍 Richiesta movimento a pose target...")
        self.get_logger().info(f"  • Frame: {pose_target.header.frame_id}")
        self.get_logger().info(f"  • Position: [{pose_target.pose.position.x:.3f}, "
                              f"{pose_target.pose.position.y:.3f}, {pose_target.pose.position.z:.3f}]")
        self.get_logger().info(f"  • Cartesian: {cartesian_motion}")
        
        # Preparazione goal
        goal_msg = MoveToPose.Goal()
        goal_msg.pose_target = pose_target
        goal_msg.cartesian_motion = cartesian_motion
        goal_msg.max_velocity_scaling_factor = velocity_scaling or self.default_velocity
        goal_msg.max_acceleration_scaling_factor = acceleration_scaling or self.default_acceleration
        goal_msg.tolerance_position = tolerance_position
        goal_msg.tolerance_orientation = tolerance_orientation
        
        # Verifica disponibilità server
        if not self.move_to_pose_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("Action server move_to_pose non disponibile")
        
        # Invio goal
        send_goal_future = self.move_to_pose_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )
        
        # Attesa accettazione goal
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            raise RuntimeError("Goal movimento pose rifiutato dal server")
            
        self.get_logger().info("✅ Goal accettato, esecuzione in corso...")
        
        # Attesa completamento con timeout
        result_future = goal_handle.get_result_async()
        timeout = timeout_sec or self.default_timeout
        
        start_time = time.time()
        while not result_future.done():
            if time.time() - start_time > timeout:
                # Tentativo cancellazione goal
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                raise TimeoutError(f"Movimento timeout dopo {timeout}s")
                
            rclpy.spin_once(self, timeout_sec=0.1)
            
        # Estrazione risultato
        result = result_future.result().result
        
        if result.result.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(f"✅ Movimento completato con successo!")
            self.get_logger().info(f"  • Planning time: {result.planning_time:.2f}s")
            self.get_logger().info(f"  • Execution time: {result.execution_time:.2f}s")
        else:
            self.get_logger().warning(f"⚠️ Movimento fallito: {self._error_code_to_string(result.result.val)}")
            
        return result.result

    def plan_global_path(self, pose_target: PoseStamped,
                         planner_id: str = "RRTConnect",
                         planning_time: float = 5.0,
                         timeout_sec: Optional[float] = None):
        """
        Pianifica un path globale con OMPL usando l'action plan_global_path.

        Restituisce:
            result.robot_trajectory  (RobotTrajectory MoveIt)
            result.waypoints_path    (Path per RViz)
            result.error_code        (MoveItErrorCodes)
        """

        self.get_logger().info("🌍 Richiesta di Global Planning...")

        goal_msg = PlanGlobalPath.Goal()
        goal_msg.target_pose = pose_target
        goal_msg.planner_id = planner_id
        goal_msg.planning_time = planning_time

        # Invio goal
        send_future = self.plan_global_path_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            raise RuntimeError("Goal plan_global_path rifiutato dal server")

        # Attesa risultato
        result_future = goal_handle.get_result_async()
        timeout = timeout_sec or self.default_timeout

        start_time = time.time()
        while not result_future.done():
            if time.time() - start_time > timeout:
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                raise TimeoutError("Timeout durante global planning")
            rclpy.spin_once(self, timeout_sec=0.1)

        result = result_future.result().result

        if result.error_code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info("✅ Global Planning completato con successo!")
        else:
            self.get_logger().warn(
                f"⚠️ Global Planning fallito: {self._error_code_to_string(result.error_code)}"
            )

        return result

    
    def move_to_joint(self,
                     joint_target: List[float],
                     joint_names: Optional[List[str]] = None,
                     velocity_scaling: Optional[float] = None,
                     acceleration_scaling: Optional[float] = None, 
                     tolerance: float = 0.01,
                     validate_limits: bool = True,
                     timeout_sec: Optional[float] = None) -> MoveItErrorCodes:
        """
        Muove il robot a una configurazione joint specifica.
        
        Args:
            joint_target: Posizioni target joint [rad]
            joint_names: Nomi joint (opzionale, usa default server se None)
            velocity_scaling: Factor velocità (0.0-1.0), None=default
            acceleration_scaling: Factor accelerazione (0.0-1.0), None=default
            tolerance: Tolleranza posizione joint [rad] 
            validate_limits: Verifica limiti joint prima esecuzione
            timeout_sec: Timeout movimento, None=default
            
        Returns:
            MoveItErrorCodes: Risultato movimento
            
        Raises:
            ValueError: Se configurazione joint invalida
            RuntimeError: Se action server non disponibile o goal rifiutato
            TimeoutError: Se movimento supera timeout
        """
        
        # Validazione input
        if len(joint_target) != 7:  # FR3 ha 7 DOF
            raise ValueError(f"FR3 richiede 7 joint values, ricevuti {len(joint_target)}")
            
        self.get_logger().info(f"🔧 Richiesta movimento joint...")
        self.get_logger().info(f"  • Target: {[f'{j:.3f}' for j in joint_target]}")
        self.get_logger().info(f"  • Validate limits: {validate_limits}")
        
        # Preparazione goal
        goal_msg = MoveToJoint.Goal()
        goal_msg.joint_target = joint_target
        goal_msg.joint_names = joint_names or []  # Server userà default se vuoto
        goal_msg.max_velocity_scaling_factor = velocity_scaling or self.default_velocity
        goal_msg.max_acceleration_scaling_factor = acceleration_scaling or self.default_acceleration
        goal_msg.tolerance_joint = tolerance
        goal_msg.validate_target = validate_limits
        
        # Verifica disponibilità server
        if not self.move_to_joint_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("Action server move_to_joint non disponibile")
        
        # Invio goal
        send_goal_future = self.move_to_joint_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )
        
        # Attesa accettazione goal
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            raise RuntimeError("Goal movimento joint rifiutato dal server")
            
        self.get_logger().info("✅ Goal accettato, esecuzione in corso...")
        
        # Attesa completamento con timeout
        result_future = goal_handle.get_result_async()
        timeout = timeout_sec or self.default_timeout
        
        start_time = time.time()
        while not result_future.done():
            if time.time() - start_time > timeout:
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                raise TimeoutError(f"Movimento timeout dopo {timeout}s")
                
            rclpy.spin_once(self, timeout_sec=0.1)
            
        # Estrazione risultato
        result = result_future.result().result
        
        if result.result.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(f"✅ Movimento joint completato!")
            self.get_logger().info(f"  • Planning time: {result.planning_time:.2f}s")
            self.get_logger().info(f"  • Execution time: {result.execution_time:.2f}s")
        else:
            self.get_logger().warning(f"⚠️ Movimento joint fallito: {self._error_code_to_string(result.result.val)}")
            
        return result.result
    
    # ========================================================================
    # API PUBBLICHE - STATO ROBOT
    # ========================================================================
    
    def get_current_joints(self, timeout_sec: float = 5.0) -> Optional[List[float]]:
        """
        Ottieni configurazione joint corrente del robot.
        
        Args:
            timeout_sec: Timeout attesa joint state
            
        Returns:
            Lista posizioni joint [rad] o None se non disponibile
        """
        
        # Attendi joint state se non ancora ricevuto
        start_time = time.time()
        while self.current_joint_state is None and (time.time() - start_time) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            
        if self.current_joint_state is None:
            self.get_logger().warning("⚠️ Joint state non disponibile")
            return None
            
        # Filtra solo joint FR3 (assumendo nomi standard)
        fr3_positions = []
        for i, name in enumerate(self.current_joint_state.name):
            if 'fr3_joint' in name and i < len(self.current_joint_state.position):
                fr3_positions.append(self.current_joint_state.position[i])
                
        if len(fr3_positions) == 7:
            return fr3_positions
        else:
            self.get_logger().warning(f"⚠️ Expected 7 FR3 joints, found {len(fr3_positions)}")
            return None
    
    def get_current_pose(self, target_frame: str = "fr3_link0") -> Optional[PoseStamped]:
        """
        Ottieni pose corrente end-effector.
        
        Args:
            target_frame: Frame di riferimento per la pose
            
        Returns:
            PoseStamped corrente o None se non disponibile
            
        Note:
            Richiede TF transform da end-effector a target_frame
        """
        
        try:
            # Lookup transform end-effector -> target_frame
            transform = self.tf_buffer.lookup_transform(
                target_frame, 'fr3_link8',  # end-effector frame
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            
            # Converti transform in PoseStamped
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = target_frame
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            
            pose_stamped.pose.position.x = transform.transform.translation.x
            pose_stamped.pose.position.y = transform.transform.translation.y
            pose_stamped.pose.position.z = transform.transform.translation.z
            
            pose_stamped.pose.orientation = transform.transform.rotation
            
            return pose_stamped
            
        except Exception as e:
            self.get_logger().warning(f"⚠️ Impossibile ottenere pose corrente: {e}")
            return None
    
    # ========================================================================
    # UTILITY FUNCTIONS
    # ========================================================================
    
    def create_pose_stamped(self, 
                           x: float, y: float, z: float,
                           qx: float = 0.0, qy: float = 0.0, qz: float = 0.0, qw: float = 1.0,
                           frame_id: str = "fr3_link0") -> PoseStamped:
        """
        Utility per creare PoseStamped facilmente.
        
        Args:
            x, y, z: Posizione [m]
            qx, qy, qz, qw: Quaternione orientamento
            frame_id: Frame di riferimento
            
        Returns:
            PoseStamped configurato
        """
        
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        
        return pose
    
    def _error_code_to_string(self, error_code: int) -> str:
        """Converte MoveItErrorCode in stringa leggibile."""
        
        error_map = {
            MoveItErrorCodes.SUCCESS: "SUCCESS",
            MoveItErrorCodes.FAILURE: "FAILURE", 
            MoveItErrorCodes.PLANNING_FAILED: "PLANNING_FAILED",
            MoveItErrorCodes.INVALID_MOTION_PLAN: "INVALID_MOTION_PLAN",
            MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: "PLAN_INVALIDATED",
            MoveItErrorCodes.CONTROL_FAILED: "CONTROL_FAILED",
            MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA: "SENSOR_DATA_FAILED",
            MoveItErrorCodes.TIMED_OUT: "TIMED_OUT",
            MoveItErrorCodes.PREEMPTED: "PREEMPTED",
            MoveItErrorCodes.START_STATE_IN_COLLISION: "START_STATE_COLLISION",
            MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS: "START_STATE_CONSTRAINT_VIOLATION",
            MoveItErrorCodes.GOAL_IN_COLLISION: "GOAL_COLLISION",
            MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS: "GOAL_CONSTRAINT_VIOLATION",
            MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: "GOAL_CONSTRAINTS_VIOLATED",
            MoveItErrorCodes.INVALID_GROUP_NAME: "INVALID_GROUP_NAME",
            MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: "INVALID_GOAL_CONSTRAINTS",
            MoveItErrorCodes.INVALID_ROBOT_STATE: "INVALID_ROBOT_STATE",
            MoveItErrorCodes.INVALID_LINK_NAME: "INVALID_LINK_NAME",
            MoveItErrorCodes.INVALID_OBJECT_NAME: "INVALID_OBJECT_NAME",
            MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: "FRAME_TRANSFORM_FAILURE",
            MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE: "COLLISION_CHECKING_UNAVAILABLE",
            MoveItErrorCodes.ROBOT_STATE_STALE: "ROBOT_STATE_STALE",
            MoveItErrorCodes.SENSOR_INFO_STALE: "SENSOR_INFO_STALE",
            MoveItErrorCodes.NO_IK_SOLUTION: "NO_IK_SOLUTION"
        }
        
        return error_map.get(error_code, f"UNKNOWN_ERROR_{error_code}")
    
    def _log_configuration(self):
        """Log configurazione client per debug."""
        self.get_logger().info("📋 Motion Client Configuration:")
        self.get_logger().info(f"  • Default timeout: {self.default_timeout}s")
        self.get_logger().info(f"  • Default velocity: {self.default_velocity}")
        self.get_logger().info(f"  • Feedback logging: {self.enable_feedback_logging}")


def main(args=None):
    """Entry point per test standalone del client."""
    
    print("🚀 Starting Franka Motion Client...")
    rclpy.init(args=args)
    
    try:
        client = FrankaMotionClient()
        
        print("✅ Motion Client ready. Spinning for state updates...")
        print("   Use the client object in your code to control the robot.")
        print("   Example: client.move_to_joint([0, 0, 0, -1.57, 0, 1.57, 0])")
        print("   Press Ctrl+C to stop.")
        
        rclpy.spin(client)
        
    except KeyboardInterrupt:
        print("\n⏹️ Shutdown requested")
    except Exception as e:
        print(f"\n❌ Client error: {e}")
    finally:
        try:
            rclpy.shutdown()
            print("🏁 Motion Client terminated")
        except:
            pass


if __name__ == '__main__':
    main()