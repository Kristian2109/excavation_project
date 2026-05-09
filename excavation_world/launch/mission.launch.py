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

from excavation_core.parameters import (
    DEFAULT_EXECUTION_SPEED,
    DEFAULT_HOLE_ORIGIN_X,
    DEFAULT_HOLE_ORIGIN_Y,
    DEFAULT_HOLE_ORIGIN_Z,
    DEFAULT_HOLE_SIZE_X,
    DEFAULT_HOLE_SIZE_Y,
    DEFAULT_HOLE_DEPTH,
)
from excavation_core.excavation_grid import HoleSpec
from excavation_core.position_planner import compute_work_positions


def generate_launch_description():
    # Compute first work position from hole geometry
    hole = HoleSpec(
        origin_x=DEFAULT_HOLE_ORIGIN_X,
        origin_y=DEFAULT_HOLE_ORIGIN_Y,
        origin_z=DEFAULT_HOLE_ORIGIN_Z,
        size_x=DEFAULT_HOLE_SIZE_X,
        size_y=DEFAULT_HOLE_SIZE_Y,
        depth=DEFAULT_HOLE_DEPTH,
    )
    first_pos = compute_work_positions(hole)[0]

    # --- Shared launch arguments ---
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')
    execute_arm = LaunchConfiguration('execute_arm')
    execution_speed = LaunchConfiguration('execution_speed')
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
            'execution_speed': execution_speed,
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
            'execution_speed': execution_speed,
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
        DeclareLaunchArgument('goal_x', default_value=str(first_pos.x)),
        DeclareLaunchArgument('goal_y', default_value=str(first_pos.y)),
        DeclareLaunchArgument('goal_yaw', default_value=str(first_pos.yaw)),
        DeclareLaunchArgument('execute_arm', default_value='true',
                              description='Set false for headless / grid-only mode'),
        DeclareLaunchArgument('execution_speed', default_value=str(DEFAULT_EXECUTION_SPEED),
                      description='Mission speed multiplier (e.g. 2.0 = ~2x faster)'),

        control_launch,
        world_launch,
        debug_launch,
        mission_launch,
    ])
