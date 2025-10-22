#!/usr/bin/env python3
"""
🔥 HYBRID PLANNING REPLANNING TEST - OSTACOLO SINGOLO
======================================================

Demo ottimizzata per ostacolo box rosso a (0.5, 0.0, 0.15)
Dimensioni: 0.15 x 0.15 x 0.3 m

OSTACOLO RANGE:
- X: [0.425, 0.575]
- Y: [-0.075, 0.075]
- Z: [0.0, 0.3]

STRATEGIA: Traiettorie che ATTRAVERSANO l'ostacolo
per forzare OMPL a trovare path alternativi.
"""

import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from franka_simulation.action import ExecuteHybridMotion
from rclpy.node import Node
import time
from typing import Tuple


class ReplanningTestSingleObstacle(Node):
    def __init__(self):
        super().__init__('replanning_test_single')
        
        self.client = ActionClient(
            self,
            ExecuteHybridMotion,
            '/execute_hybrid_motion'
        )
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('🔥 TEST REPLANNING - OSTACOLO SINGOLO')
        self.get_logger().info('=' * 70)
        self.get_logger().info('⏳ Attesa server...')
        
        if not self.client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('❌ Server non disponibile')
            raise RuntimeError("Timeout")
        
        self.get_logger().info('✅ Server connesso!')
        self._print_obstacle_info()
    
    def _print_obstacle_info(self):
        """Info ostacolo"""
        self.get_logger().info('')
        self.get_logger().info('📦 OSTACOLO NEL WORKSPACE:')
        self.get_logger().info('-' * 70)
        self.get_logger().info('🔴 Box Rosso:')
        self.get_logger().info('   Centro: (0.5, 0.0, 0.15)')
        self.get_logger().info('   Dimensioni: 0.15 x 0.15 x 0.3 m')
        self.get_logger().info('   Range X: [0.425, 0.575]')
        self.get_logger().info('   Range Y: [-0.075, 0.075]')
        self.get_logger().info('   Range Z: [0.0, 0.3]')
        self.get_logger().info('-' * 70)
        self.get_logger().info('')
    
    def create_pose(self, x: float, y: float, z: float,
                   qw: float = 1.0, qx: float = 0.0,
                   qy: float = 0.0, qz: float = 0.0) -> PoseStamped:
        """Crea PoseStamped"""
        pose = PoseStamped()
        pose.header.frame_id = 'fr3_link0'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = qw
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        return pose
    
    def send_goal(self, pose: PoseStamped, name: str = "target",
                 description: str = "") -> bool:
        """Invia goal"""
        goal_msg = ExecuteHybridMotion.Goal()
        goal_msg.target_pose = pose
        goal_msg.use_hybrid_planning = True
        
        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'🚀 {name}')
        if description:
            self.get_logger().info(f'📝 {description}')
        self.get_logger().info(
            f'📍 ({pose.pose.position.x:.3f}, '
            f'{pose.pose.position.y:.3f}, '
            f'{pose.pose.position.z:.3f})'
        )
        self.get_logger().info('=' * 70)
        
        future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error(f'❌ Rifiutato')
            return False
        
        self.get_logger().info(f'✅ Accettato, esecuzione...')
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        
        self.get_logger().info('')
        self.get_logger().info('-' * 70)
        if result.error_code == ExecuteHybridMotion.Result.SUCCESS:
            self.get_logger().info(f'🎯 RAGGIUNTO!')
            self.get_logger().info(f'   Planning: {result.planning_time:.2f}s')
            self.get_logger().info(f'   Execution: {result.execution_time:.2f}s')
            if hasattr(result, 'replans_count'):
                self.get_logger().info(f'   🔄 Replans: {result.replans_count}')
        else:
            self.get_logger().warn(f'⚠️ FALLITO')
            if hasattr(result, 'error_message'):
                self.get_logger().warn(f'   {result.error_message}')
        self.get_logger().info('-' * 70)
        
        return ExecuteHybridMotion.Result.SUCCESS


def print_test_header(num: int, title: str,
                     start: Tuple[float, float, float],
                     target: Tuple[float, float, float],
                     reason: str):
    """Header test case"""
    print('\n' + '=' * 70)
    print(f'TEST #{num}: {title}')
    print('=' * 70)
    print(f'START:  ({start[0]:.2f}, {start[1]:.2f}, {start[2]:.2f})')
    print(f'TARGET: ({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f})')
    print(f'PERCHÉ: {reason}')
    print('=' * 70)


def main():
    rclpy.init()
    node = None
    
    try:
        node = ReplanningTestSingleObstacle()
        time.sleep(2.0)
        
        home = node.create_pose(0.3, 0.0, 0.5)
        
        # ==================================================================
        # TEST 1: ATTRAVERSAMENTO ASSE Y - Altezza BASSA (dentro ostacolo)
        # ==================================================================
        print_test_header(
            1,
            'Attraversamento Asse Y (Z basso - DENTRO ostacolo)',
            start=(0.5, -0.25, 0.15),
            target=(0.5, 0.25, 0.15),
            reason='Path lineare attraversa CENTRO ostacolo a Z=0.15!'
        )
        
        node.send_goal(home, "Home", "Start safe")
        time.sleep(1.0)
        
        # Start SINISTRA dell'ostacolo, stessa altezza centro (Z=0.15)
        start_1 = node.create_pose(0.5, -0.25, 0.15)
        node.send_goal(start_1, "Start TC1", "Sinistra ostacolo")
        time.sleep(1.0)
        
        # Target DESTRA dell'ostacolo, stessa altezza
        target_1 = node.create_pose(0.5, 0.25, 0.15)
        success_1 = node.send_goal(
            target_1,
            "Target TC1",
            "⚠️ Path lineare attraversa CENTRO box a Z=0.15!"
        )
        
        print('\n' + ('✅ TC1 PASSED' if success_1 else '❌ TC1 FAILED'))
        time.sleep(2.0)
        
        # ==================================================================
        # TEST 2: ATTRAVERSAMENTO ASSE X - Altezza BASSA
        # ==================================================================
        print_test_header(
            2,
            'Attraversamento Asse X (davanti→dietro, Z basso)',
            start=(0.35, 0.0, 0.15),
            target=(0.65, 0.0, 0.15),
            reason='Path attraversa centro ostacolo lungo asse X!'
        )
        
        node.send_goal(home, "Home", "Reset")
        time.sleep(1.0)
        
        # Start DAVANTI ostacolo
        start_2 = node.create_pose(0.35, 0.0, 0.15)
        node.send_goal(start_2, "Start TC2", "Davanti ostacolo")
        time.sleep(1.0)
        
        # Target DIETRO ostacolo
        target_2 = node.create_pose(0.65, 0.0, 0.15)
        success_2 = node.send_goal(
            target_2,
            "Target TC2",
            "⚠️ Path passa attraverso centro box lungo X!"
        )
        
        print('\n' + ('✅ TC2 PASSED' if success_2 else '❌ TC2 FAILED'))
        time.sleep(2.0)
        
        # ==================================================================
        # TEST 3: DIAGONALE BASSA (XY plane dentro ostacolo)
        # ==================================================================
        print_test_header(
            3,
            'Diagonale XY a Z basso',
            start=(0.4, -0.15, 0.15),
            target=(0.6, 0.15, 0.15),
            reason='Diagonale nel piano XY attraversa ostacolo a Z=0.15!'
        )
        
        node.send_goal(home, "Home", "Reset")
        time.sleep(1.0)
        
        start_3 = node.create_pose(0.4, -0.15, 0.15)
        node.send_goal(start_3, "Start TC3", "Angolo basso-sinistra")
        time.sleep(1.0)
        
        target_3 = node.create_pose(0.6, 0.15, 0.15)
        success_3 = node.send_goal(
            target_3,
            "Target TC3",
            "⚠️ Diagonale XY passa dentro ostacolo!"
        )
        
        print('\n' + ('✅ TC3 PASSED' if success_3 else '❌ TC3 FAILED'))
        time.sleep(2.0)
        
        # ==================================================================
        # TEST 4: SOPRA→SOTTO (Verticale attraverso ostacolo)
        # ==================================================================
        print_test_header(
            4,
            'Verticale SOPRA→SOTTO ostacolo',
            start=(0.5, 0.0, 0.35),
            target=(0.5, 0.0, 0.05),
            reason='Path verticale attraversa ostacolo (top Z=0.3 → bottom Z=0)!'
        )
        
        node.send_goal(home, "Home", "Reset")
        time.sleep(1.0)
        
        # Start SOPRA ostacolo (Z=0.35 > top=0.3)
        start_4 = node.create_pose(0.5, 0.0, 0.35)
        node.send_goal(start_4, "Start TC4", "Sopra ostacolo")
        time.sleep(1.0)
        
        # Target SOTTO ostacolo (Z=0.05 < bottom=0.0)
        target_4 = node.create_pose(0.5, 0.0, 0.05)
        success_4 = node.send_goal(
            target_4,
            "Target TC4",
            "⚠️ Path verticale scende attraverso box!"
        )
        
        print('\n' + ('✅ TC4 PASSED' if success_4 else '❌ TC4 FAILED'))
        time.sleep(2.0)
        
        # ==================================================================
        # TEST 5: DIAGONALE 3D (attraversa angolo ostacolo)
        # ==================================================================
        print_test_header(
            5,
            'Diagonale 3D attraverso angolo ostacolo',
            start=(0.4, -0.12, 0.25),
            target=(0.6, 0.12, 0.05),
            reason='Diagonale 3D attraversa angolo interno ostacolo!'
        )
        
        node.send_goal(home, "Home", "Reset")
        time.sleep(1.0)
        
        start_5 = node.create_pose(0.4, -0.12, 0.25)
        node.send_goal(start_5, "Start TC5", "Angolo 3D alto")
        time.sleep(1.0)
        
        target_5 = node.create_pose(0.6, 0.12, 0.05)
        success_5 = node.send_goal(
            target_5,
            "Target TC5",
            "⚠️ Diagonale 3D passa angolo ostacolo!"
        )
        
        print('\n' + ('✅ TC5 PASSED' if success_5 else '❌ TC5 FAILED'))
        time.sleep(2.0)
        
        # ==================================================================
        # TEST 6: PATH AL LIMITE (rasente ostacolo)
        # ==================================================================
        print_test_header(
            6,
            'Path rasente al bordo ostacolo',
            start=(0.5, -0.25, 0.32),
            target=(0.5, 0.25, 0.32),
            reason='Path passa appena sopra top ostacolo (Z=0.32 vs top=0.3)!'
        )
        
        node.send_goal(home, "Home", "Reset")
        time.sleep(1.0)
        
        # Path rasente sopra top ostacolo (Z=0.32, top=0.3, clearance 2cm)
        start_6 = node.create_pose(0.5, -0.25, 0.32)
        node.send_goal(start_6, "Start TC6", "Rasente lato sinistro")
        time.sleep(1.0)
        
        target_6 = node.create_pose(0.5, 0.25, 0.32)
        success_6 = node.send_goal(
            target_6,
            "Target TC6",
            "Path molto vicino a top ostacolo (2cm clearance)"
        )
        
        print('\n' + ('✅ TC6 PASSED' if success_6 else '❌ TC6 FAILED'))
        time.sleep(2.0)
        
        # ==================================================================
        # RITORNO HOME
        # ==================================================================
        node.get_logger().info('\n🏠 Ritorno HOME finale...')
        node.send_goal(home, "Home Finale")
        
        # ==================================================================
        # REPORT FINALE
        # ==================================================================
        results = [
            ('TC1: Attraversamento Y (Z basso)', success_1),
            ('TC2: Attraversamento X (Z basso)', success_2),
            ('TC3: Diagonale XY', success_3),
            ('TC4: Verticale SOPRA→SOTTO', success_4),
            ('TC5: Diagonale 3D', success_5),
            ('TC6: Path Rasente', success_6),
        ]
        
        passed = sum(1 for _, s in results if s)
        total = len(results)
        
        print('\n' + '=' * 70)
        print('📊 REPORT FINALE')
        print('=' * 70)
        
        for name, success in results:
            status = '✅ PASSED' if success else '❌ FAILED'
            print(f'{status} - {name}')
        
        print('-' * 70)
        print(f'TOTALE: {passed}/{total} test ({passed*100//total}%)')
        print('=' * 70)
        
        if passed == total:
            print('\n🎉 PERFETTO! OMPL evita correttamente l\'ostacolo.')
        elif passed >= total * 0.8:
            print(f'\n⚠️ BUONO ({passed}/{total}). Alcuni path complessi falliti.')
        else:
            print(f'\n❌ PROBLEMI ({passed}/{total}). Verifica config ostacolo.')
        
    except KeyboardInterrupt:
        print('\n⏹️ Interrotto.')
    except Exception as e:
        print(f'\n❌ Errore: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()