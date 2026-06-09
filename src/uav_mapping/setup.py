import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'uav_mapping'

setup(
    name=package_name,
    version='0.1.0',
    # find_packages() discovers the python module dir (uav_mapping/),
    # excluding the test/ dir so it isn't installed as an importable package.
    packages=find_packages(exclude=['test']),
    data_files=[
        # Registers the package in the ament resource index so ROS 2 tools
        # (ros2 run, ros2 launch, ros2 pkg) can find it after install.
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Installs package.xml alongside the package's share dir.
        ('share/' + package_name, ['package.xml']),
        # Installs launch files so `ros2 launch uav_mapping <file>` works.
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Raafay',
    maintainer_email='raafaykazmi@yahoo.com',
    description='Leader-drone detection-to-heatmap node (OccupancyGrid over '
                'the thesis sector grid).',
    license='MIT',
    entry_points={
        # Maps executable names (used by ros2 run and in launch files) to the
        # main() of each module.
        'console_scripts': [
            'leader_node = uav_mapping.leader_node:main',
            'fake_detection_publisher = '
            'uav_mapping.fake_detection_publisher:main',
        ],
    },
)
