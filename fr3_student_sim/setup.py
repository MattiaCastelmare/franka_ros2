from glob import glob

from setuptools import find_packages, setup

package_name = 'fr3_student_sim'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=['fr3_sim', 'fr3_sim.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mattia Castelmare',
    maintainer_email='castelmare@diag.uniroma1.it',
    description='Minimal Gazebo FR3 torque-control kit for student projects',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'runner = fr3_sim.simulation.runner:main',
            'plot_run = fr3_sim.analysis.plot_run:main',
            'run_batch = fr3_sim.simulation.run_batch:main',
            'compare_runs = fr3_sim.analysis.compare_runs:main',
        ],
    },
)
