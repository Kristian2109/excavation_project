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

# Import parameter constants to ensure single source of truth
from excavation_world.parameters import (
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
    PRM_BASE_X,
    DEFAULT_BASE_X,
    PRM_BASE_Y,
    DEFAULT_BASE_Y,
    PRM_BASE_YAW,
    DEFAULT_BASE_YAW,
    PRM_EXECUTE_ARM,
    DEFAULT_EXECUTE_ARM,
    PRM_AUTO_START,
    DEFAULT_AUTO_START,
    PRM_SCOOP_DELAY,
    DEFAULT_SCOOP_DELAY,
    PRM_ARM_TIMEOUT,
    DEFAULT_ARM_TIMEOUT,
    PRM_TRAIL_MAX_POINTS,
    DEFAULT_TRAIL_MAX_POINTS,
)


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

    # --- Mission controller (delayed so arm_controller is ready) ---
    mission_controller = Node(
        package='excavation_world',
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
            PRM_EXECUTE_ARM: execute_arm,
            PRM_AUTO_START: DEFAULT_AUTO_START,
            PRM_SCOOP_DELAY: DEFAULT_SCOOP_DELAY,
            PRM_ARM_TIMEOUT: DEFAULT_ARM_TIMEOUT,
            'use_sim_time': use_sim_time,
        }],
    )

    # Delay the mission controller so that controllers have been spawned
    delayed_mission = TimerAction(
        period=14.0,
        actions=[mission_controller],
    )

    # --- Debug visualizer (starts immediately) ---
    debug_visualizer = Node(
        package='excavation_world',
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
        DeclareLaunchArgument('execute_arm', default_value='true',
                              description='Set false for headless / grid-only mode'),

        control_launch,
        world_node,
        debug_visualizer,
        delayed_mission,
    ])
