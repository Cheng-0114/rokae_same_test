import os
from glob import glob

from setuptools import setup

package_name = 'arm_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ckt',
    maintainer_email='ketaocheng4@gmail.com',
    description='终端关节指令输入节点 + 转发到 arm_controller 的桥接节点',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'joint_input_node = arm_teleop.joint_input_node:main',
            'trajectory_bridge_node = arm_teleop.trajectory_bridge_node:main',
        ],
    },
)
