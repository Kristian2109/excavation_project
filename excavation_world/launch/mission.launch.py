"""
Launch the complete excavation mission.

Brings up:
  1. excavator_control.launch.py  (robot, ros2_control, base motion, Foxglove)
  2. world_node                   (grid, markers, grid_state)
  3. mission_controller_node      (state machine + scoop execution)

Usage
-----
Full mission (arm execution):
    ros2 launch excavation_world mission.launch.py

Headless (grid-only, no arm):
    ros2 launch excavation_world mission.launch.py execute_arm:=false
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --- Shared configs ---
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')
    execute_arm = LaunchConfiguration('execute_arm')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # --- Include the control stack (robot + controllers + base motion + foxglove) ---
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('excavator_control'),
                'launch',
                'excavator_control.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'goal_x': goal_x,
            'goal_y': goal_y,
            'goal_yaw': goal_yaw,
        }.items(),
    )

    # --- World node (grid + markers) ---
    world_node = Node(
        package='excavation_world',
        executable='world_node',
        name='excavation_world',
        output='screen',
        parameters=[{
            'resolution': 0.25,
            'hole_origin_x': 5.0,
            'hole_origin_y': -2.0,
            'hole_origin_z': 0.0,
            'hole_size_x': 4.0,
            'hole_size_y': 3.0,
            'hole_depth': 2.0,
            'publish_rate': 2.0,
            'working_position_x': 3.0,
            'working_position_y': 0.0,
            'working_position_z': 0.0,
            'working_position_yaw': 0.0,
            'use_sim_time': use_sim_time,
        }],
    )

    # --- Mission controller (delayed so arm_controller is ready) ---
    mission_controller = Node(
        package='excavation_world',
        executable='mission_controller_node',
        name='mission_controller',
        output='screen',
        parameters=[{
            'hole_origin_x': 5.0,
            'hole_origin_y': -2.0,
            'hole_origin_z': 0.0,
            'hole_size_x': 4.0,
            'hole_size_y': 3.0,
            'hole_depth': 2.0,
            'resolution': 0.25,
            'base_x': goal_x,
            'base_y': goal_y,
            'base_yaw': goal_yaw,
            'execute_arm': execute_arm,
            'auto_start': True,
            'scoop_delay': 0.5,
            'arm_timeout': 30.0,
            'use_sim_time': use_sim_time,
        }],
    )

    # Delay the mission controller so that controllers have been spawned
    delayed_mission = TimerAction(
        period=6.0,
        actions=[mission_controller],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('goal_x', default_value='3.0'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),
        DeclareLaunchArgument('execute_arm', default_value='true',
                              description='Set false for headless / grid-only mode'),

        control_launch,
        world_node,
        delayed_mission,
    ])
