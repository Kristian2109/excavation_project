"""
Launch mission controller node — orchestrates excavation mission execution.

Usage
-----
Full mission (arm execution):
    ros2 launch excavation_mission mission.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from excavation_core.position_planner import compute_work_positions
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
    PRM_BASE_X,
    PRM_BASE_Y,
    PRM_BASE_YAW,
    PRM_AUTO_START,
    DEFAULT_AUTO_START,
    PRM_SCOOP_DELAY,
    DEFAULT_SCOOP_DELAY,
    PRM_EXECUTION_SPEED,
    DEFAULT_EXECUTION_SPEED,
)


def generate_launch_description():
    # Compute the first work position from hole geometry so base_motion
    # drives to the right place automatically.
    from excavation_core.parameters import default_hole_geometry
    hole = default_hole_geometry().to_hole_spec()
    positions = compute_work_positions(hole)
    first_pos = positions[0]

    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')
    execution_speed = LaunchConfiguration('execution_speed')
    use_sim_time = LaunchConfiguration('use_sim_time')

    mission_controller = Node(
        package='excavation_mission',
        executable='mission_controller_node',
        name='mission_controller',
        output='screen',
        parameters=[{
            PRM_HOLE_ORIGIN_X: DEFAULT_HOLE_ORIGIN_X,
            PRM_HOLE_ORIGIN_Y: DEFAULT_HOLE_ORIGIN_Y,
            PRM_HOLE_ORIGIN_Z: DEFAULT_HOLE_ORIGIN_Z,
            PRM_HOLE_SIZE_X: DEFAULT_HOLE_SIZE_X,
            PRM_HOLE_SIZE_Y: DEFAULT_HOLE_SIZE_Y,
            PRM_HOLE_DEPTH: DEFAULT_HOLE_DEPTH,
            PRM_RESOLUTION: DEFAULT_RESOLUTION,
            PRM_BASE_X: goal_x,
            PRM_BASE_Y: goal_y,
            PRM_BASE_YAW: goal_yaw,
            PRM_AUTO_START: DEFAULT_AUTO_START,
            PRM_SCOOP_DELAY: DEFAULT_SCOOP_DELAY,
            PRM_EXECUTION_SPEED: execution_speed,
            'use_sim_time': use_sim_time,
        }],
    )

    # Delay so that arm_controller is ready
    delayed_mission = TimerAction(
        period=14.0,
        actions=[mission_controller],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('goal_x', default_value=str(first_pos.x)),
        DeclareLaunchArgument('goal_y', default_value=str(first_pos.y)),
        DeclareLaunchArgument('goal_yaw', default_value=str(first_pos.yaw)),
        DeclareLaunchArgument('execution_speed', default_value=str(DEFAULT_EXECUTION_SPEED),
                              description='Mission speed multiplier (e.g. 2.0 = ~2x faster)'),
        delayed_mission,
    ])
