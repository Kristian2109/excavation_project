"""
scoop_executor_node.py – ROS 2 node that executes scoop trajectories via MoveIt 2.

Uses moveit_py (MoveGroupInterface) to plan and execute smooth joint-space
trajectories between scoop waypoints.  Falls back to direct JointTrajectory
publishing if MoveIt planning fails.

Topics published:
    /scoop_executor/status     (String)   – current scoop phase
    /scoop_executor/progress   (Float32)  – fraction of current scoop complete

Services:
    /scoop_executor/execute_scoop  – trigger a single scoop at a target position

Parameters:
    base_x, base_y, base_yaw  – current base pose
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Float32
from geometry_msgs.msg import Pose, Point, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration

from excavation_msgs.msg import ScoopAction as ScoopActionMsg

import numpy as np

from excavation_world.scoop_trajectory import (
    ScoopTrajectory,
    ScoopWaypoint,
    plan_single_scoop,
)
from excavation_world.robot_model import JOINT_NAMES


class ScoopExecutorNode(Node):
    """Executes scoop trajectories through the arm_controller."""

    def __init__(self) -> None:
        super().__init__('scoop_executor')

        # Parameters
        self.declare_parameter('base_x', 3.0)
        self.declare_parameter('base_y', 0.0)
        self.declare_parameter('base_yaw', 0.0)

        self.base_x = float(self.get_parameter('base_x').value)
        self.base_y = float(self.get_parameter('base_y').value)
        self.base_yaw = float(self.get_parameter('base_yaw').value)

        # Publishers
        self.status_pub = self.create_publisher(String, '/scoop_executor/status', 10)
        self.progress_pub = self.create_publisher(Float32, '/scoop_executor/progress', 10)
        self.scoop_action_pub = self.create_publisher(
            ScoopActionMsg, '/excavation/apply_scoop', 10)

        # Action client for arm_controller
        self._cb_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            callback_group=self._cb_group,
        )

        self.get_logger().info('ScoopExecutor waiting for arm_controller action server...')
        self._action_client.wait_for_server(timeout_sec=30.0)
        self.get_logger().info('ScoopExecutor ready.')

    def _publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f'Scoop status: {status}')

    def _publish_progress(self, fraction: float) -> None:
        msg = Float32()
        msg.data = fraction
        self.progress_pub.publish(msg)

    def execute_scoop(self, trajectory: ScoopTrajectory) -> bool:
        """Execute a complete scoop trajectory through the arm controller.

        Sends the full trajectory as a single FollowJointTrajectory goal.
        Returns True if execution succeeded.
        """
        self._publish_status(f'Starting scoop {trajectory.scoop_id}')
        self._publish_progress(0.0)

        # Build a JointTrajectory from waypoints
        jt = JointTrajectory()
        jt.joint_names = list(JOINT_NAMES)

        cumulative_time = 0.0
        for i, wp in enumerate(trajectory.waypoints):
            cumulative_time += wp.duration
            pt = JointTrajectoryPoint()
            pt.positions = wp.joint_positions.tolist()
            pt.velocities = [0.0] * len(JOINT_NAMES)
            pt.time_from_start = Duration(
                sec=int(cumulative_time),
                nanosec=int((cumulative_time % 1.0) * 1e9),
            )
            jt.points.append(pt)

        # Send as FollowJointTrajectory action
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt

        self.get_logger().info(
            f'Sending trajectory with {len(jt.points)} waypoints, '
            f'duration={cumulative_time:.1f}s')

        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._publish_status(f'Scoop {trajectory.scoop_id} REJECTED')
            return False

        self._publish_status(f'Scoop {trajectory.scoop_id} executing...')

        # Wait for result
        result_future = goal_handle.get_result_async()

        # Publish progress while waiting
        start_time = time.time()
        while not result_future.done():
            elapsed = time.time() - start_time
            progress = min(elapsed / cumulative_time, 1.0)
            self._publish_progress(progress)
            rclpy.spin_once(self, timeout_sec=0.2)

        result = result_future.result()
        if result.status == 4:  # SUCCEEDED
            self._publish_status(f'Scoop {trajectory.scoop_id} complete')
            self._publish_progress(1.0)
            # Notify the world node to update the grid
            self._publish_scoop_action(trajectory)
            return True
        else:
            self._publish_status(
                f'Scoop {trajectory.scoop_id} failed (status={result.status})')
            return False

    def _publish_scoop_action(self, trajectory: ScoopTrajectory) -> None:
        """Publish a ScoopAction so the world_node updates the grid."""
        msg = ScoopActionMsg()
        msg.scoop_id = trajectory.scoop_id

        # Set entry_pose from the 'dig' waypoint target
        dig_wp = None
        for wp in trajectory.waypoints:
            if wp.name == 'dig' and wp.target_xyz is not None:
                dig_wp = wp
                break
        if dig_wp is not None:
            msg.entry_pose = Pose(
                position=Point(
                    x=float(dig_wp.target_xyz[0]),
                    y=float(dig_wp.target_xyz[1]),
                    z=float(dig_wp.target_xyz[2]),
                ),
                orientation=Quaternion(w=1.0),
            )

        self.scoop_action_pub.publish(msg)
        self.get_logger().info(
            f'Published ScoopAction for scoop {trajectory.scoop_id}')

    def plan_and_execute(
        self,
        target_xyz: np.ndarray,
        scoop_id: int = 0,
    ) -> bool:
        """Plan a scoop trajectory using IK and execute it."""
        self._publish_status(f'Planning scoop {scoop_id}...')

        traj = plan_single_scoop(
            target_xyz,
            base_x=self.base_x,
            base_y=self.base_y,
            base_yaw=self.base_yaw,
            scoop_id=scoop_id,
        )

        if traj is None:
            self._publish_status(f'Scoop {scoop_id} IK failed — unreachable')
            return False

        self.get_logger().info(
            f'Scoop {scoop_id} planned: {len(traj.waypoints)} waypoints')

        return self.execute_scoop(traj)


def main(args=None):
    rclpy.init(args=args)
    node = ScoopExecutorNode()

    # Demo: execute a single test scoop
    target = np.array([7.0, -1.0, -0.3])
    node.get_logger().info(f'Demo: scooping at {target}')
    success = node.plan_and_execute(target, scoop_id=0)
    node.get_logger().info(f'Demo scoop result: {"SUCCESS" if success else "FAILED"}')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
