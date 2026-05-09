"""
Launch base motion node — controls mobile base to working position.

Usage
-----
ros2 launch excavation_base_motion base_motion.launch.py goal_x:=3.0 goal_y:=0.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')
    use_sim_time = LaunchConfiguration('use_sim_time')

    base_motion_node = Node(
        package='excavation_base_motion',
        executable='base_motion_node',
        name='base_motion',
        output='screen',
        parameters=[{
            'start_x': 0.0,
            'start_y': 0.0,
            'start_yaw': 0.0,
            'goal_x': goal_x,
            'goal_y': goal_y,
            'goal_yaw': goal_yaw,
            'linear_speed': 0.5,
            'angular_speed': 0.3,
            'publish_rate': 20.0,
            'auto_start': True,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('goal_x', default_value='2.0'),
        DeclareLaunchArgument('goal_y', default_value='-0.5'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),
        base_motion_node,
    ])
