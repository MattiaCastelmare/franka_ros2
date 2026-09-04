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
        (os.path.join('share', package_name, 'config'),
         glob.glob('config/*.yaml') + glob.glob('config/*.rviz')),
        (os.path.join('share', package_name, 'scripts'),
         glob.glob('scripts/*.py') + glob.glob('scripts/*.sh')),
        # Test infrastructure
        (os.path.join('share', package_name, 'test', 'launch'),
         glob.glob('test/launch/*.launch.py')),
        (os.path.join('share', package_name, 'test', 'config'),
         glob.glob('test/config/*.yaml')),
    ],
    install_requires=['setuptools'],
    # Makes `colcon test` run the pytest suite in test/ instead of falling back
    # to `python -m unittest`, which discovers none of the function-style tests.
    # NOTE: it must be extras_require['test'], not tests_require — modern
    # setuptools dropped tests_require, so colcon reads back None and its
    # pytest step never matches.
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='mattia',
    maintainer_email='mattia@todo.todo',
    description='Experiment launch files for Franka robots',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # === Velocity pipeline ===
            'ee_pentagon_velocity_commander = franka_experiments.nodes.ee_pentagon_velocity_commander:main',
            'ee_circle_velocity_commander = franka_experiments.nodes.ee_circle_velocity_commander:main',
            'ee_random_waypoints_velocity_commander = franka_experiments.nodes.ee_random_waypoints_velocity_commander:main',
            # === Velocity CBF safety layer ===
            'cbf_velocity_filter = franka_experiments.nodes.cbf_velocity_filter:main',
            # === Torque pipeline 1 (acceleration-level) ===
            'pentagon_qddot_commander = franka_experiments.nodes.pentagon_qddot_commander:main',
            'rl_policy_commander = franka_experiments.nodes.rl_policy_commander:main',
            'qddot_to_torque = franka_experiments.nodes.qddot_to_torque:main',
            'cbf_safety_filter = franka_experiments.nodes.cbf_safety_filter:main',
            # === Torque pipeline 2 (OSCBF torque-level) ===
            'pentagon_torque_commander = franka_experiments.nodes.pentagon_torque_commander:main',
            'cbf_oscbf_filter = franka_experiments.nodes.cbf_OSCBF_filter:main',
            # === Shared nodes ===
            'real_time_distance = franka_experiments.nodes.real_time_distance:main',
            'experiment_logger = franka_experiments.nodes.experiment_logger:main',
            'capsule_overlay_node = franka_experiments.nodes.capsule_overlay_node:main',
            'handeye_calibration_node = franka_experiments.nodes.handeye_calibration_node:main',
            'frame_grabber = franka_experiments.nodes.frame_grabber:main',
        ],
    },
)
