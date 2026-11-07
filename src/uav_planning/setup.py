from setuptools import find_packages, setup

package_name = 'uav_planning'

setup(
    name=package_name,
    version='0.1.0',
    # find_packages() discovers uav_planning/ and uav_planning/core/,
    # excluding the test/ dir so it isn't installed as an importable package.
    packages=find_packages(exclude=['test']),
    data_files=[
        # Registers the package in the ament resource index so ROS 2 tools
        # (ros2 run, ros2 launch, ros2 pkg) can find it after install.
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Installs package.xml alongside the package's share dir.
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Raafay',
    maintainer_email='raafaykazmi@yahoo.com',
    description='Follower sector planner: ROS-free core library '
                '(follower_planner_node and helpers come next).',
    license='MIT',
    # No executables yet: this session is the library only.
    entry_points={'console_scripts': []},
)
