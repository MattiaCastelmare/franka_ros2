import glob
import os
from setuptools import setup

package_name = 'franka_experiments'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, package_name + '.nodes', package_name + '.utils'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mattia',
    maintainer_email='mattia@todo.todo',
    description='Experiment launch files for Franka robots',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'velocity_commander = franka_experiments.nodes.velocity_commander:main',
            'smooth_velocity_commander = franka_experiments.nodes.smooth_velocity_commander:main',
            'ee_pentagon_velocity_commander = franka_experiments.nodes.ee_pentagon_velocity_commander:main',
            'ee_circle_velocity_commander = franka_experiments.nodes.ee_circle_velocity_commander:main',
            'ee_random_waypoints_velocity_commander = franka_experiments.nodes.ee_random_waypoints_velocity_commander:main',
            'handeye_calibration_node = franka_experiments.nodes.handeye_calibration_node:main',
            'avoidance_controller = franka_experiments.nodes.avoidance_controller:main',
            'distance_estimator = franka_experiments.nodes.distance_estimator:main',
            'capsule_overlay_node = franka_experiments.nodes.capsule_overlay_node:main',
            'real_time_distance = franka_experiments.nodes.real_time_distance:main',
            'cbf_safety_filter = franka_experiments.nodes.cbf_safety_filter:main',
            'cbf_avoidance_controller = franka_experiments.nodes.cbf_avoidance_controller:main',
            'pentagon_torque_commander = franka_experiments.nodes.pentagon_torque_commander:main',
            'pentagon_qddot_commander = franka_experiments.nodes.pentagon_qddot_commander:main',
            'experiment_logger = franka_experiments.nodes.experiment_logger:main',
        ],
    },
)
