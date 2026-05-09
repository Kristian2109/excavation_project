"""
Launch the excavator with ros2_control + Foxglove Bridge.

Brings up:
  1. robot_state_publisher  (from URDF/Xacro – publishes TF)
  2. ros2_control_node      (mock hardware, receives URDF as parameter)
  3. joint_state_broadcaster
  4. arm_controller          (JointTrajectoryController)
  5. base_motion_node        (drives base to working position)
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
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')
    foxglove_port = LaunchConfiguration('foxglove_port')

    # --- Nodes ---

    # robot_state_publisher – publishes TF (strips <ros2_control> tags, that's fine)
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

    # controller_manager – gets full URDF (with ros2_control tags) directly
    # as a parameter so the ResourceManager can set up hardware interfaces.
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': ParameterValue(robot_description, value_type=str)},
            controllers_yaml,
        ],
        output='screen',
    )

    # Spawn controllers after controller_manager is up.
    # Spawn sequentially (not in parallel) to avoid race conditions.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': foxglove_port,
            'use_sim_time': use_sim_time,
        }],
    )

    # Base motion node – drives base from start to working position
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
            'auto_start': True,
            'use_sim_time': use_sim_time,
        }],
    )

    # Delay spawners so controller_manager has time to initialise.
    delayed_joint_state_spawner = TimerAction(
        period=8.0,
        actions=[joint_state_broadcaster_spawner],
    )

    delayed_arm_spawner = TimerAction(
        period=10.0,
        actions=[arm_controller_spawner],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('goal_x', default_value='2.0'),
        DeclareLaunchArgument('goal_y', default_value='-0.5'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),

        robot_state_publisher,
        controller_manager,
        base_motion_node,
        delayed_joint_state_spawner,
        delayed_arm_spawner,
        foxglove_bridge,
    ])
