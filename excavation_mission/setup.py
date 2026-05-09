import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'excavation_mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='kpetrov@quickbase.com',
    description='ROS 2 nodes for mission planning and scoop execution',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission_controller_node = excavation_mission.mission_controller_node:main',
            'scoop_executor_node = excavation_mission.scoop_executor_node:main',
        ],
    },
)
