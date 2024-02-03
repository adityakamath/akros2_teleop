from setuptools import setup
import os
from glob import glob

package_name = 'akros2_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aditya Kamath',
    maintainer_email='adityakamath@live.com',
    description='Nodes to teleoperate the AKROS2 robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist_mixer = akros2_teleop.twist_mixer_node:main',
            'joy_mode_handler = akros2_teleop.joy_mode_handler:main',
            'teleop_node = akros2_teleop.teleop_node:main',
        ],
    },
)
