"""
Launch the excavator with ros2_control + Foxglove Bridge.

Brings up:
  1. robot_state_publisher (from URDF/Xacro)
  2. raw_urdf_publisher   (publishes full URDF with ros2_control tags)
  3. controller_manager    (mock hardware)
  4. joint_state_broadcaster
  5. arm_controller        (JointTrajectoryController)
  6. foxglove_bridge
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
)
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --- Package paths ---
    desc_share = FindPackageShare('excavator_description')
    ctrl_share = FindPackageShare('excavator_control')

    # --- URDF ---
    urdf_file = PathJoinSubstitution([desc_share, 'urdf', 'excavator.urdf.xacro'])
    robot_description = Command(['xacro ', urdf_file])

    # --- Controller config ---
    controllers_yaml = PathJoinSubstitution(
        [ctrl_share, 'config', 'excavator_controllers.yaml'])

    use_sim_time = LaunchConfiguration('use_sim_time')

    # --- Nodes ---

    # robot_state_publisher – publishes TF but strips <ros2_control> tags
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'use_sim_time': use_sim_time,
        }],
    )

    # raw_urdf_publisher – publishes FULL URDF (with ros2_control) on
    # /robot_description_raw for the controller_manager
    raw_urdf_publisher = Node(
        package='excavation_world',
        executable='raw_urdf_publisher',
        name='raw_urdf_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
        }],
    )

    # controller_manager – subscribes to /robot_description_raw instead of
    # /robot_description so it gets the full URDF including ros2_control tags
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_yaml],
        output='screen',
        remappings=[
            ('/robot_description', '/robot_description_raw'),
        ],
    )

    # Spawn controllers after controller_manager is up
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'use_sim_time': use_sim_time,
        }],
    )

    # Base motion node – drives base from start to working position
    base_motion_node = Node(
        package='excavation_world',
        executable='base_motion_node',
        name='base_motion',
        output='screen',
        parameters=[{
            'start_x': 0.0,
            'start_y': 0.0,
            'start_yaw': 0.0,
            'goal_x': 3.0,
            'goal_y': 0.0,
            'goal_yaw': 0.0,
            'linear_speed': 0.5,
            'angular_speed': 0.3,
            'auto_start': True,
            'use_sim_time': use_sim_time,
        }],
    )

    # Delay spawners so controller_manager has time to initialise
    delayed_spawners = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner, arm_controller_spawner],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        robot_state_publisher,
        raw_urdf_publisher,
        controller_manager,
        base_motion_node,
        delayed_spawners,
        foxglove_bridge,
    ])
