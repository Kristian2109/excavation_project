"""
Launch the complete excavation mission system.

Brings up:
  1. excavator_control.launch.py    (robot, ros2_control, Foxglove)
    2. excavation_world                (grid state & visualization)
    3. excavation_mission              (mission orchestration & execution)
    4. excavation_debug                (debug visualization)

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
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --- Shared launch arguments ---
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')
    execute_arm = LaunchConfiguration('execute_arm')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # --- Control stack (robot + ros2_control + Foxglove) ---
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

    
    # --- World node (grid state + visualization) ---
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('excavation_world'),
                'launch',
                'world.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # --- Mission controller (orchestration + scoop execution) ---
    mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('excavation_mission'),
                'launch',
                'mission.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'goal_x': goal_x,
            'goal_y': goal_y,
            'goal_yaw': goal_yaw,
            'execute_arm': execute_arm,
        }.items(),
    )

    # --- Debug visualization ---
    debug_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('excavation_debug'),
                'launch',
                'debug.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'goal_x': goal_x,
            'goal_y': goal_y,
            'goal_yaw': goal_yaw,
        }.items(),
    )


    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('goal_x', default_value='2.0'),
        DeclareLaunchArgument('goal_y', default_value='-0.5'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),
        DeclareLaunchArgument('execute_arm', default_value='true',
                              description='Set false for headless / grid-only mode'),

        control_launch,
        world_launch,
        debug_launch,
        mission_launch,
    ])
