"""
Launch the full excavator stack with MoveIt 2 for arm trajectory planning.

Brings up:
  1. robot_state_publisher  (URDF/Xacro)
  2. ros2_control_node      (mock hardware)
  3. joint_state_broadcaster + arm_controller
  4. move_group              (MoveIt 2 planning)
  5. base_motion_node        (drives base to working position)
  6. foxglove_bridge
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name: str, file_path: str):
    """Load a YAML file from a package share directory."""
    full_path = os.path.join(
        get_package_share_directory(package_name), file_path)
    with open(full_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    # --- Package paths ---
    desc_share = FindPackageShare('excavator_description')
    ctrl_share = FindPackageShare('excavator_control')
    moveit_share_dir = get_package_share_directory('excavator_moveit_config')

    # --- URDF ---
    urdf_file = PathJoinSubstitution([desc_share, 'urdf', 'excavator.urdf.xacro'])
    robot_description = Command(['xacro ', urdf_file])

    # --- SRDF ---
    srdf_path = os.path.join(moveit_share_dir, 'config', 'excavator.srdf')
    with open(srdf_path, 'r') as f:
        robot_description_semantic = f.read()

    # --- MoveIt config files ---
    kinematics_yaml = load_yaml('excavator_moveit_config', 'config/kinematics.yaml')
    joint_limits_yaml = {
        'robot_description_planning': load_yaml(
            'excavator_moveit_config', 'config/joint_limits.yaml')
    }
    ompl_yaml = load_yaml('excavator_moveit_config', 'config/ompl_planning.yaml')
    ompl_planning = {'ompl': ompl_yaml}
    moveit_controllers = load_yaml(
        'excavator_moveit_config', 'config/moveit_controllers.yaml')

    # --- Controller config ---
    controllers_yaml = PathJoinSubstitution(
        [ctrl_share, 'config', 'excavator_controllers.yaml'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')

    # --- Nodes ---

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

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': ParameterValue(robot_description, value_type=str)},
            controllers_yaml,
        ],
        output='screen',
    )

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

    # --- MoveIt 2 move_group node ---
    move_group_params = {
        'robot_description': ParameterValue(robot_description, value_type=str),
        'robot_description_semantic': robot_description_semantic,
        'robot_description_kinematics': kinematics_yaml,
        'use_sim_time': use_sim_time,
        'publish_robot_description_semantic': True,
    }
    move_group_params.update(joint_limits_yaml)
    move_group_params.update(moveit_controllers)

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            move_group_params,
            ompl_planning,
        ],
    )

    # --- Base motion ---
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

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{'port': 8765, 'use_sim_time': use_sim_time}],
    )

    # Delay spawners so controller_manager loads the URDF first
    delayed_spawners = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner, arm_controller_spawner],
    )

    # Delay move_group until controllers are active
    delayed_move_group = TimerAction(
        period=6.0,
        actions=[move_group_node],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('goal_x', default_value='3.0'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),

        robot_state_publisher,
        controller_manager,
        base_motion_node,
        delayed_spawners,
        delayed_move_group,
        foxglove_bridge,
    ])
