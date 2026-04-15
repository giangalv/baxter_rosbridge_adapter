from setuptools import setup
from glob import glob
import os

package_name = 'baxter_rosbridge_adapter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gian_galv',
    maintainer_email='gianluca.galvagni@edu.unige.it',
    description='ROS2 adapter for Baxter via rosbridge',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_bridge = baxter_rosbridge_adapter.joint_state_bridge:main',
            'baxter_grippers_cli = baxter_rosbridge_adapter.baxter_grippers_cli:main',
            'baxter_cli = baxter_rosbridge_adapter.baxter_cli:main',
        ],
    },
)
