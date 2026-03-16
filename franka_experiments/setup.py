from setuptools import setup

package_name = 'franka_experiments'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, package_name + '.nodes', package_name + '.utils'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/wrapper_forward_velocity.launch.py',
            'launch/handeye_calibration_bringup.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/fake_hw_controller_params.yaml',
            'config/controllers_rt_velocity_blender.yaml',
            'config/controllers_rt_velocity_blender_real.yaml',
            'config/camera_extrinsics.yaml',
            'config/launch_defaults.yaml',
        ]),
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
            'ee_random_waypoints_velocity_commander = franka_experiments.nodes.ee_random_waypoints_velocity_commander:main',
            'handeye_calibration_node = franka_experiments.nodes.handeye_calibration_node:main',
        ],
    },
)
