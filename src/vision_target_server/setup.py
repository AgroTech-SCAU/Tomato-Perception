import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'vision_target_server'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'),
            [os.path.join('..', '..', 'best.pt')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Service-only ROS 2 RGB-D 3D target detector.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'vision_server = vision_target_server.vision_server:main',
            'sim_target_server = vision_target_server.sim_target_server:main',
            'mujoco_target_server = vision_target_server.mujoco_target_server:main',
        ],
    },
)
