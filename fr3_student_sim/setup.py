import os
from glob import glob

from setuptools import setup

package_name = 'fr3_student_sim'


def tree(src):
    """data_files entries for every file under ``src``, keeping the layout."""
    out = {}
    for path in glob(os.path.join(src, '**', '*'), recursive=True):
        if os.path.isfile(path) and '__pycache__' not in path:
            out.setdefault(os.path.join('share', package_name, os.path.dirname(path)), []).append(path)
    return list(out.items())


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ] + tree('students'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mattia Castelmare',
    maintainer_email='castelmare@diag.uniroma1.it',
    description='Minimal Gazebo FR3 torque-control kit for student projects',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'runner = fr3_student_sim.runner:main',
            'plot_run = fr3_student_sim.plot_run:main',
            'run_batch = fr3_student_sim.run_batch:main',
        ],
    },
)
