import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'arm_endeffector_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 添加launch文件目录，修正通配符匹配
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='lp@example.com',
    description='Package for controlling robot arm end-effector using MoveIt2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'endeffector_control_node = arm_endeffector_control.endeffector_control_node:main',
            'move_to_pose = arm_endeffector_control.move_to_pose:main',
            'cartesian_path_planner = arm_endeffector_control.cartesian_path_planner:main',
        ],
    },
)