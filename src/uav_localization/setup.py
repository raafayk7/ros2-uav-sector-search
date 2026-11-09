import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'uav_localization'

setup(
    name=package_name,
    version='0.1.0',
    # find_packages() discovers the python module dir (uav_localization/),
    # excluding the test/ dir so it isn't installed as an importable package.
    packages=find_packages(exclude=['test']),
    data_files=[
        # Registers the package in the ament resource index so ROS 2 tools
        # (ros2 run, ros2 launch, ros2 pkg) can find it after install.
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Installs package.xml alongside the package's share dir.
        ('share/' + package_name, ['package.xml']),
        # Installs launch files so `ros2 launch uav_localization <file>` works.
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Raafay',
    maintainer_email='182409092+raafayk7@users.noreply.github.com',
    description='Per-drone GPS->ENU arena-frame localization node.',
    license='MIT',
    entry_points={
        # Maps the `localization_node` executable name (used by ros2 run and
        # in launch files) to the main() function in our module.
        'console_scripts': [
            'localization_node = uav_localization.localization_node:main',
        ],
    },
)
