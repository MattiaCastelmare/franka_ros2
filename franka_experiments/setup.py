from setuptools import setup

package_name = 'franka_experiments'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, package_name + '.nodes'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/wrapper_min.launch.py',
            'launch/wrapper_velocity.launch.py',
            'launch/wrapper_forward_velocity.launch.py',
            'launch/experiment_velocity_forward.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/fake_hw_controller_params.yaml',
            'config/controllers_velocity_forward.yaml',
            'config/controllers_velocity_forward_real.yaml',
            'config/controllers_rt_velocity_blender.yaml',
            'config/controllers_rt_velocity_blender_real.yaml',
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
            'velocity_blender = franka_experiments.nodes.velocity_blender_node:main',
        ],
    },
)
