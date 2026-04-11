import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'excavation_world'

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
    description='3D grid world model and excavation terrain for the excavation project',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'world_node = excavation_world.world_node:main',
            'raw_urdf_publisher = excavation_world.raw_urdf_publisher:main',
            'base_motion_node = excavation_world.base_motion_node:main',
            'scoop_executor_node = excavation_world.scoop_executor_node:main',
            'mission_controller_node = excavation_world.mission_controller_node:main',
            'debug_visualizer_node = excavation_world.debug_visualizer_node:main',
        ],
    },
)
