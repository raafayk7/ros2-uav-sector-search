from setuptools import find_packages, setup

package_name = 'uav_eval'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Raafay',
    maintainer_email='182409092+raafayk7@users.noreply.github.com',
    description='The 2D kinematic eval of language-prior search.',
    license='MIT',
    entry_points={'console_scripts': [
        'run_eval = uav_eval.run:main',
    ]},
)
