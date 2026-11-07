import os
from glob import glob
from setuptools import setup

package_name = 'daedalus_slam'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='giuliano',
    maintainer_email='giuliano@example.com',
    description='SLAM con RTAB-Map: launch, parametri, odometria da PX4 e configurazione RViz',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'px4_odometry = daedalus_slam.px4_odometry:main',
            'vision_to_px4 = daedalus_slam.vision_to_px4:main',
        ],
    },
)
