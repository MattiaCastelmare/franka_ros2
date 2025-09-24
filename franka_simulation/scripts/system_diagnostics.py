#!/usr/bin/env python3
"""
Sistema di diagnostica completo per franka_simulation
Verifica tutti i nodi, controller, topic e servizi
"""

import subprocess
import time
import sys
import json
import re

class FrankaSimulationDiagnostics:
    def __init__(self):
        self.results = {}
        
    def run_command(self, command):
        """Esegue un comando e restituisce output e return code"""
        try:
            result = subprocess.run(command, shell=True, capture_output=True, text=True, timeout=10)
            return result.stdout.strip(), result.stderr.strip(), result.returncode
        except subprocess.TimeoutExpired:
            return "", "Command timeout", -1
        except Exception as e:
            return "", f"Error: {str(e)}", -1

    def check_nodes(self):
        """Verifica tutti i nodi attivi"""
        print("🔍 Checking ROS2 nodes...")
        stdout, stderr, code = self.run_command("ros2 node list")
        
        if code == 0:
            nodes = [node.strip() for node in stdout.split('\n') if node.strip()]
            expected_nodes = [
                "/robot_state_publisher",
                "/controller_manager",
                "/rviz2"
            ]
            
            print(f"✅ Found {len(nodes)} active nodes:")
            for node in nodes:
                print(f"   - {node}")
                
            missing = [node for node in expected_nodes if node not in nodes]
            if missing:
                print(f"⚠️  Missing expected nodes: {missing}")
            
            self.results['nodes'] = {'count': len(nodes), 'list': nodes, 'missing': missing}
            return True
        else:
            print(f"❌ Failed to get node list: {stderr}")
            self.results['nodes'] = {'error': stderr}
            return False

    def check_controllers(self):
        """Verifica lo stato dei controller"""
        print("\n🎮 Checking controllers...")
        
        # Lista controller disponibili
        stdout, stderr, code = self.run_command("ros2 control list_controllers")
        
        if code == 0:
            print("📋 Controller status:")
            print(stdout)
            
            # Parsing dello stato dei controller
            controllers = {}
            for line in stdout.split('\n'):
                if '[' in line and ']' in line:
                    # Estrae nome controller e stato
                    parts = line.split()
                    if len(parts) >= 2:
                        name = parts[0]
                        state_match = re.search(r'\[(\w+)\]', line)
                        if state_match:
                            state = state_match.group(1)
                            controllers[name] = state
            
            expected_controllers = {
                'joint_state_broadcaster': 'active',
                'fr3_arm_controller': 'active'
            }
            
            all_good = True
            for expected, expected_state in expected_controllers.items():
                if expected in controllers:
                    actual_state = controllers[expected]
                    if actual_state == expected_state:
                        print(f"✅ {expected}: {actual_state}")
                    else:
                        print(f"⚠️  {expected}: {actual_state} (expected: {expected_state})")
                        all_good = False
                else:
                    print(f"❌ {expected}: NOT FOUND")
                    all_good = False
            
            self.results['controllers'] = {
                'all_controllers': controllers,
                'status': 'good' if all_good else 'issues'
            }
            return all_good
            
        else:
            print(f"❌ Failed to get controller list: {stderr}")
            self.results['controllers'] = {'error': stderr}
            return False

    def check_topics(self):
        """Verifica i topic principali"""
        print("\n📡 Checking key topics...")
        
        stdout, stderr, code = self.run_command("ros2 topic list")
        
        if code == 0:
            topics = [topic.strip() for topic in stdout.split('\n') if topic.strip()]
            
            critical_topics = [
                '/joint_states',
                '/robot_description',
                '/fr3_arm_controller/joint_trajectory',
                '/tf',
                '/tf_static'
            ]
            
            print("📋 Critical topics status:")
            missing_topics = []
            
            for topic in critical_topics:
                if topic in topics:
                    print(f"✅ {topic}")
                    
                    # Verifica se ci sono messaggi
                    echo_cmd = f"timeout 3 ros2 topic echo {topic} --once"
                    echo_out, echo_err, echo_code = self.run_command(echo_cmd)
                    
                    if echo_code == 0 and echo_out.strip():
                        print(f"   📨 Publishing data")
                    else:
                        print(f"   ⚠️  No data or timeout")
                        
                else:
                    print(f"❌ {topic}")
                    missing_topics.append(topic)
            
            self.results['topics'] = {
                'total': len(topics),
                'critical_missing': missing_topics,
                'status': 'good' if not missing_topics else 'issues'
            }
            
            return len(missing_topics) == 0
            
        else:
            print(f"❌ Failed to get topic list: {stderr}")
            self.results['topics'] = {'error': stderr}
            return False

    def check_tf_tree(self):
        """Verifica l'albero TF"""
        print("\n🌳 Checking TF tree...")
        
        stdout, stderr, code = self.run_command("ros2 run tf2_tools view_frames.py --help")
        
        if code == 0:
            # Genera il file PDF dell'albero TF
            stdout, stderr, code = self.run_command("timeout 10 ros2 run tf2_tools view_frames.py")
            
            if code == 0:
                print("✅ TF tree generated successfully")
                
                # Verifica alcuni frame critici
                critical_frames = ['fr3_link0', 'fr3_link7', 'fr3_hand_tcp']
                
                frame_cmd = "ros2 run tf2_ros tf2_echo fr3_link0 fr3_link7"
                frame_out, frame_err, frame_code = self.run_command(f"timeout 5 {frame_cmd}")
                
                if frame_code == 0:
                    print("✅ TF transforms working")
                    self.results['tf'] = {'status': 'good'}
                    return True
                else:
                    print(f"⚠️  TF transform issues: {frame_err}")
                    self.results['tf'] = {'status': 'issues', 'error': frame_err}
                    return False
            else:
                print(f"⚠️  TF tree generation failed: {stderr}")
                self.results['tf'] = {'status': 'warning', 'error': stderr}
                return False
        else:
            print("❌ tf2_tools not available")
            self.results['tf'] = {'status': 'unavailable'}
            return False

    def check_moveit_integration(self):
        """Verifica se MoveIt è disponibile e configurato"""
        print("\n🤖 Checking MoveIt integration...")
        
        # Verifica se move_group è running (se lanciato)
        stdout, stderr, code = self.run_command("ros2 node list | grep move_group")
        
        if code == 0 and stdout.strip():
            print("✅ move_group node is running")
            
            # Verifica servizi MoveIt
            moveit_services = [
                '/compute_cartesian_path',
                '/get_planning_scene',
                '/plan_kinematic_path'
            ]
            
            service_stdout, service_stderr, service_code = self.run_command("ros2 service list")
            
            if service_code == 0:
                available_services = service_stdout.split('\n')
                moveit_available = []
                
                for service in moveit_services:
                    if any(service in s for s in available_services):
                        moveit_available.append(service)
                        print(f"✅ {service}")
                    else:
                        print(f"❌ {service}")
                
                self.results['moveit'] = {
                    'node_running': True,
                    'services_available': moveit_available,
                    'status': 'good' if len(moveit_available) > 0 else 'issues'
                }
                
            else:
                print("⚠️  Could not check MoveIt services")
                self.results['moveit'] = {'node_running': True, 'status': 'unknown'}
                
        else:
            print("ℹ️  move_group not currently running (launch separately if needed)")
            self.results['moveit'] = {'node_running': False, 'status': 'not_running'}

    def check_gazebo_status(self):
        """Verifica lo stato di Gazebo"""
        print("\n🌍 Checking Gazebo status...")
        
        # Verifica se gazebo è in running
        stdout, stderr, code = self.run_command("pgrep -f 'gz sim'")
        
        if code == 0 and stdout.strip():
            print("✅ Gazebo/Ignition is running")
            
            # Verifica topic di Gazebo
            gz_topics = [
                '/clock',
                '/world/empty/dynamic_pose/info'
            ]
            
            topic_stdout, topic_stderr, topic_code = self.run_command("ros2 topic list")
            
            if topic_code == 0:
                available_topics = topic_stdout.split('\n')
                gz_working = False
                
                for topic in gz_topics:
                    if topic in available_topics:
                        gz_working = True
                        print(f"✅ {topic}")
                        break
                
                if not gz_working:
                    print("⚠️  Gazebo topics not found in ROS2")
                
                self.results['gazebo'] = {
                    'running': True,
                    'ros_bridge': gz_working,
                    'status': 'good' if gz_working else 'bridge_issues'
                }
                
            else:
                print("⚠️  Could not check Gazebo topics")
                self.results['gazebo'] = {'running': True, 'status': 'unknown'}
                
        else:
            print("❌ Gazebo/Ignition not running")
            self.results['gazebo'] = {'running': False, 'status': 'not_running'}

    def generate_summary(self):
        """Genera un riassunto della diagnostica"""
        print("\n" + "="*60)
        print("📊 DIAGNOSTIC SUMMARY")
        print("="*60)
        
        total_checks = 0
        passed_checks = 0
        
        for component, data in self.results.items():
            total_checks += 1
            status = data.get('status', 'unknown')
            
            if status in ['good', 'not_running']:  # not_running per MoveIt è OK
                passed_checks += 1
                icon = "✅"
            elif status in ['issues', 'bridge_issues']:
                icon = "⚠️"
            elif status in ['warning', 'unknown']:
                icon = "🟡"
            else:
                icon = "❌"
            
            print(f"{icon} {component.upper()}: {status}")
        
        print(f"\n🎯 Overall Status: {passed_checks}/{total_checks} components OK")
        
        if passed_checks == total_checks:
            print("🎉 All systems operational!")
            return True
        elif passed_checks >= total_checks * 0.75:
            print("⚠️  System mostly functional with minor issues")
            return True
        else:
            print("❌ System has significant issues that need attention")
            return False

    def run_full_diagnostic(self):
        """Esegue la diagnostica completa"""
        print("🔬 Starting Franka Simulation Full Diagnostic")
        print("="*60)
        
        # Lista di controlli
        checks = [
            ("ROS2 Nodes", self.check_nodes),
            ("Controllers", self.check_controllers),
            ("Topics", self.check_topics),
            ("TF Tree", self.check_tf_tree),
            ("MoveIt", self.check_moveit_integration),
            ("Gazebo", self.check_gazebo_status)
        ]
        
        for check_name, check_func in checks:
            try:
                check_func()
            except Exception as e:
                print(f"❌ Error during {check_name} check: {str(e)}")
                self.results[check_name.lower().replace(' ', '_')] = {'error': str(e)}
            
            time.sleep(1)  # Piccola pausa tra i controlli
        
        return self.generate_summary()

def main():
    print("Franka Simulation Diagnostics Tool")
    print("Make sure your simulation is running before starting diagnostics")
    
    input("Press Enter when ready to start diagnostics...")
    
    diagnostics = FrankaSimulationDiagnostics()
    success = diagnostics.run_full_diagnostic()
    
    # Salva risultati in JSON
    with open('/tmp/franka_diagnostics.json', 'w') as f:
        json.dump(diagnostics.results, f, indent=2)
    
    print(f"\n📄 Detailed results saved to: /tmp/franka_diagnostics.json")
    
    return 0 if success else 1

if __name__ == "__main__":
    sys.exit(main())