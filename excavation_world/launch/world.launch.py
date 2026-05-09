"""
Launch excavation world node — grid state and visualization service.

Usage
-----
ros2 launch excavation_world world.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from excavation_core.parameters import (
    PRM_RESOLUTION,
    DEFAULT_RESOLUTION,
    PRM_HOLE_ORIGIN_X,
    DEFAULT_HOLE_ORIGIN_X,
    PRM_HOLE_ORIGIN_Y,
    DEFAULT_HOLE_ORIGIN_Y,
    PRM_HOLE_ORIGIN_Z,
    DEFAULT_HOLE_ORIGIN_Z,
    PRM_HOLE_SIZE_X,
    DEFAULT_HOLE_SIZE_X,
    PRM_HOLE_SIZE_Y,
    DEFAULT_HOLE_SIZE_Y,
    PRM_HOLE_DEPTH,
    DEFAULT_HOLE_DEPTH,
    PRM_PUBLISH_RATE,
    DEFAULT_PUBLISH_RATE,
    PRM_WORKING_POSITION_X,
    DEFAULT_WORKING_POSITION_X,
    PRM_WORKING_POSITION_Y,
    DEFAULT_WORKING_POSITION_Y,
    PRM_WORKING_POSITION_Z,
    DEFAULT_WORKING_POSITION_Z,
    PRM_WORKING_POSITION_YAW,
    DEFAULT_WORKING_POSITION_YAW,
)


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    world_node = Node(
        package='excavation_world',
        executable='world_node',
        name='excavation_world',
        output='screen',
        parameters=[{
            PRM_RESOLUTION: DEFAULT_RESOLUTION,
            PRM_HOLE_ORIGIN_X: DEFAULT_HOLE_ORIGIN_X,
            PRM_HOLE_ORIGIN_Y: DEFAULT_HOLE_ORIGIN_Y,
            PRM_HOLE_ORIGIN_Z: DEFAULT_HOLE_ORIGIN_Z,
            PRM_HOLE_SIZE_X: DEFAULT_HOLE_SIZE_X,
            PRM_HOLE_SIZE_Y: DEFAULT_HOLE_SIZE_Y,
            PRM_HOLE_DEPTH: DEFAULT_HOLE_DEPTH,
            PRM_PUBLISH_RATE: DEFAULT_PUBLISH_RATE,
            PRM_WORKING_POSITION_X: DEFAULT_WORKING_POSITION_X,
            PRM_WORKING_POSITION_Y: DEFAULT_WORKING_POSITION_Y,
            PRM_WORKING_POSITION_Z: DEFAULT_WORKING_POSITION_Z,
            PRM_WORKING_POSITION_YAW: DEFAULT_WORKING_POSITION_YAW,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        world_node,
    ])
