"""
Launch debug visualization nodes — monitoring and debug overlays.

Usage
-----
ros2 launch excavation_debug debug.launch.py goal_x:=2.0 goal_y:=-0.5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from excavation_core.parameters import (
    PRM_BASE_X,
    DEFAULT_BASE_X,
    PRM_BASE_Y,
    DEFAULT_BASE_Y,
    PRM_BASE_YAW,
    DEFAULT_BASE_YAW,
    PRM_TRAIL_MAX_POINTS,
    DEFAULT_TRAIL_MAX_POINTS,
    PRM_PUBLISH_RATE,
)


def generate_launch_description():
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')
    use_sim_time = LaunchConfiguration('use_sim_time')

    debug_visualizer = Node(
        package='excavation_debug',
        executable='debug_visualizer_node',
        name='debug_visualizer',
        output='screen',
        parameters=[{
            PRM_BASE_X: goal_x,
            PRM_BASE_Y: goal_y,
            PRM_BASE_YAW: goal_yaw,
            PRM_TRAIL_MAX_POINTS: DEFAULT_TRAIL_MAX_POINTS,
            PRM_PUBLISH_RATE: 4.0,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('goal_x', default_value='2.0'),
        DeclareLaunchArgument('goal_y', default_value='-0.5'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),
        debug_visualizer,
    ])
