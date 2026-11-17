from glob import glob

from setuptools import find_packages, setup

package_name = 'uav_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Raafay',
    maintainer_email='182409092+raafayk7@users.noreply.github.com',
    description='Ground-truth detector: Gazebo target poses projected into '
                'the camera, with misses, false positives and pixel noise.',
    license='MIT',
    entry_points={'console_scripts': [
        'ground_truth_detector = uav_perception.ground_truth_detector:main',
    ]},
)
