from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='coelacanth',
    maintainer_email='ax90l0tl@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_control_node = robot_control.pid_control_node:main',
            'active_compliance_control_node = robot_control.active_compliance_control_node:main',
            'impedence_control_node = robot_control.impedence_control_node:main',
            'hardware_node = robot_control.hardware_node:main',
        ],
    },
)
