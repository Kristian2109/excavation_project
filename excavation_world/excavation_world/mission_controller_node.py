"""
mission_controller_node.py – ROS 2 node that orchestrates the full excavation.

Sequence
--------
1. Subscribes to ``/base_motion/done`` and waits for the base to arrive.
2. Generates the excavation plan (pure-Python planner, fast).
3. For every scoop:
   a. Solves IK to build a ``ScoopTrajectory``.
   b. Sends the trajectory to ``/arm_controller/follow_joint_trajectory``.
   c. Publishes a ``ScoopAction`` on ``/excavation/apply_scoop`` so the
      world node updates the grid & markers.
4. Publishes ``MissionStatus`` on ``/mission/status`` at 2 Hz.

Parameters
----------
    hole_origin_x/y/z, hole_size_x/y, hole_depth, resolution
        Same as world_node – define the target hole.
    base_x, base_y, base_yaw
        Robot working position (must match base_motion_node goal).
    execute_arm (bool, default True)
        When *False*, scoops are published without driving the arm
        (headless mode – useful for fast testing / grid-only demos).
    auto_start (bool, default True)
        Immediately start the mission (base motion already running).
    scoop_delay (float, default 0.5)
        Minimum seconds between consecutive scoops (headless mode only).
    arm_timeout (float, default 30.0)
        Seconds to wait for arm controller action server at startup.
"""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Point, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration

from excavation_msgs.msg import (
    MissionStatus as MissionStatusMsg,
    ScoopAction as ScoopActionMsg,
)

import numpy as np

from excavation_world.mission_controller import (
    MissionController,
    MissionState,
)
from excavation_world.excavation_grid import ExcavationGrid, HoleSpec
from excavation_world.excavation_planner import PlannedScoop
from excavation_world.scoop_trajectory import (
    ScoopTrajectory,
    plan_single_scoop,
)
from excavation_world.robot_model import JOINT_NAMES


class MissionControllerNode(Node):
    """Orchestrates base-motion → plan → excavate → done."""

    def __init__(self) -> None:
        super().__init__('mission_controller')

        # ----- Parameters -----
        self.declare_parameter('hole_origin_x', 5.0)
        self.declare_parameter('hole_origin_y', -2.0)
        self.declare_parameter('hole_origin_z', 0.0)
        self.declare_parameter('hole_size_x', 4.0)
        self.declare_parameter('hole_size_y', 3.0)
        self.declare_parameter('hole_depth', 2.0)
        self.declare_parameter('resolution', 0.25)
        self.declare_parameter('base_x', 3.0)
        self.declare_parameter('base_y', 0.0)
        self.declare_parameter('base_yaw', 0.0)
        self.declare_parameter('execute_arm', True)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('scoop_delay', 0.5)
        self.declare_parameter('arm_timeout', 30.0)

        hole = HoleSpec(
            origin_x=self.get_parameter('hole_origin_x').value,
            origin_y=self.get_parameter('hole_origin_y').value,
            origin_z=self.get_parameter('hole_origin_z').value,
            size_x=self.get_parameter('hole_size_x').value,
            size_y=self.get_parameter('hole_size_y').value,
            depth=self.get_parameter('hole_depth').value,
        )
        resolution = float(self.get_parameter('resolution').value)
        grid = ExcavationGrid.from_hole_spec(hole, resolution=resolution)

        base_x = float(self.get_parameter('base_x').value)
        base_y = float(self.get_parameter('base_y').value)
        base_yaw = float(self.get_parameter('base_yaw').value)

        self._execute_arm = bool(self.get_parameter('execute_arm').value)
        self._scoop_delay = float(self.get_parameter('scoop_delay').value)
        self._arm_timeout = float(self.get_parameter('arm_timeout').value)

        # ----- State machine -----
        self.controller = MissionController(
            hole=hole, grid=grid,
            base_x=base_x, base_y=base_y, base_yaw=base_yaw,
        )

        # ----- Scoop execution tracking -----
        self._scoop_active = False
        self._current_scoop: PlannedScoop | None = None
        self._last_scoop_time = 0.0

        # ----- Publishers -----
        self.status_pub = self.create_publisher(
            MissionStatusMsg, '/mission/status', 10)
        self.scoop_action_pub = self.create_publisher(
            ScoopActionMsg, '/excavation/apply_scoop', 10)

        # ----- Subscriber: base motion done -----
        self.create_subscription(
            Bool, '/base_motion/done', self._base_done_cb, 10)

        # ----- Action client for arm (only if needed) -----
        self._action_client = None
        if self._execute_arm:
            self._cb_group = ReentrantCallbackGroup()
            self._action_client = ActionClient(
                self, FollowJointTrajectory,
                '/arm_controller/follow_joint_trajectory',
                callback_group=self._cb_group,
            )

        # ----- Timer: tick at 2 Hz to drive the state machine -----
        self.create_timer(0.5, self._tick)

        # ----- Auto-start -----
        if self.get_parameter('auto_start').value:
            self.controller.start_mission()
            self.get_logger().info('Mission auto-started → MOVING_TO_WORK_POS')

        self._publish_status()
        self.get_logger().info(
            f'MissionController ready  (execute_arm={self._execute_arm})')

    # ------------------------------------------------------------------ #
    #  Tick – main loop
    # ------------------------------------------------------------------ #
    def _tick(self) -> None:
        state = self.controller.state

        if state == MissionState.PLANNING:
            self._do_planning()

        elif state == MissionState.EXCAVATING and not self._scoop_active:
            # Enforce minimum delay between scoops
            if time.monotonic() - self._last_scoop_time >= self._scoop_delay:
                self._advance_excavation()

        self._publish_status()

    # ------------------------------------------------------------------ #
    #  Base-motion callback
    # ------------------------------------------------------------------ #
    def _base_done_cb(self, msg: Bool) -> None:
        if not msg.data:
            return
        if self.controller.state != MissionState.MOVING_TO_WORK_POS:
            return
        self.get_logger().info('Base motion complete → PLANNING')
        self.controller.on_base_arrived()
        self._publish_status()

    # ------------------------------------------------------------------ #
    #  Planning
    # ------------------------------------------------------------------ #
    def _do_planning(self) -> None:
        self.get_logger().info('Generating excavation plan …')
        if not self.controller.generate_plan():
            self.get_logger().error(
                f'Planning failed: {self.controller.progress.status_text}')
            return

        plan = self.controller.plan
        cov = plan.coverage_fraction(self.controller.grid)
        self.get_logger().info(
            f'Plan ready: {plan.total_scoops} scoops, '
            f'coverage={cov:.0%}')

    # ------------------------------------------------------------------ #
    #  Scoop execution
    # ------------------------------------------------------------------ #
    def _advance_excavation(self) -> None:
        """Start (or immediately complete) the next scoop."""
        scoop = self.controller.get_next_scoop()
        if scoop is None:
            return

        idx = self.controller.progress.current_scoop_index + 1
        total = self.controller.progress.total_scoops
        self.get_logger().info(
            f'Scoop [{idx}/{total}] target='
            f'({scoop.dig_target[0]:.2f}, {scoop.dig_target[1]:.2f}, '
            f'{scoop.dig_target[2]:.2f})')

        # -- Headless mode: publish immediately, no arm motion --
        if not self._execute_arm:
            self._publish_scoop_action(scoop)
            self.controller.on_scoop_completed(True)
            self._last_scoop_time = time.monotonic()
            return

        # -- Arm mode: build trajectory via IK --
        traj = plan_single_scoop(
            scoop.dig_target,
            base_x=self.controller.base_x,
            base_y=self.controller.base_y,
            base_yaw=self.controller.base_yaw,
            scoop_id=scoop.scoop_id,
        )

        if traj is None:
            self.get_logger().warn(
                f'Scoop {scoop.scoop_id}: IK failed — skipping')
            self._publish_scoop_action(scoop)
            self.controller.on_scoop_completed(False)
            self._last_scoop_time = time.monotonic()
            return

        # -- Send to arm controller (fully async) --
        self._scoop_active = True
        self._current_scoop = scoop

        jt = self._build_joint_trajectory(traj)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt

        self.get_logger().info(
            f'Sending arm trajectory ({len(jt.points)} waypoints, '
            f'{self._trajectory_duration(traj):.1f}s)')

        send_future = self._action_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Arm goal rejected')
            self._finish_scoop(False)
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        result = future.result()
        success = (result.status == 4)  # GoalStatus.STATUS_SUCCEEDED
        if success:
            self.get_logger().info(
                f'Scoop {self._current_scoop.scoop_id} arm execution OK')
        else:
            self.get_logger().warn(
                f'Scoop {self._current_scoop.scoop_id} arm execution '
                f'failed (status={result.status})')
        self._finish_scoop(success)

    def _finish_scoop(self, success: bool) -> None:
        """Common finalisation after a scoop attempt."""
        self._publish_scoop_action(self._current_scoop)
        self.controller.on_scoop_completed(success)
        self._scoop_active = False
        self._current_scoop = None
        self._last_scoop_time = time.monotonic()

    # ------------------------------------------------------------------ #
    #  Trajectory helpers
    # ------------------------------------------------------------------ #
    @staticmethod
    def _build_joint_trajectory(traj: ScoopTrajectory) -> JointTrajectory:
        jt = JointTrajectory()
        jt.joint_names = list(JOINT_NAMES)
        cumulative = 0.0
        for wp in traj.waypoints:
            cumulative += wp.duration
            pt = JointTrajectoryPoint()
            pt.positions = wp.joint_positions.tolist()
            pt.velocities = [0.0] * len(JOINT_NAMES)
            pt.time_from_start = Duration(
                sec=int(cumulative),
                nanosec=int((cumulative % 1.0) * 1e9),
            )
            jt.points.append(pt)
        return jt

    @staticmethod
    def _trajectory_duration(traj: ScoopTrajectory) -> float:
        return sum(wp.duration for wp in traj.waypoints)

    # ------------------------------------------------------------------ #
    #  Grid update publisher
    # ------------------------------------------------------------------ #
    def _publish_scoop_action(self, scoop: PlannedScoop) -> None:
        """Publish ScoopAction so the world_node updates its grid."""
        msg = ScoopActionMsg()
        msg.scoop_id = scoop.scoop_id
        msg.entry_pose = Pose(
            position=Point(
                x=float(scoop.dig_target[0]),
                y=float(scoop.dig_target[1]),
                z=float(scoop.dig_target[2]),
            ),
            orientation=Quaternion(w=1.0),
        )

        if scoop.affected_cells:
            nx, ny, nz = self.controller.grid.shape
            flat = []
            for (ix, iy, iz) in scoop.affected_cells:
                flat.append(ix * ny * nz + iy * nz + iz)
            msg.affected_cell_indices = [int(f) for f in flat]

        self.scoop_action_pub.publish(msg)

    # ------------------------------------------------------------------ #
    #  Status publisher
    # ------------------------------------------------------------------ #
    def _publish_status(self) -> None:
        p = self.controller.progress
        msg = MissionStatusMsg()
        msg.header.stamp = self.get_clock().now().to_msg()

        STATE_MAP = {
            MissionState.IDLE: MissionStatusMsg.IDLE,
            MissionState.MOVING_TO_WORK_POS: MissionStatusMsg.MOVING_TO_WORK_POS,
            MissionState.PLANNING: MissionStatusMsg.EXCAVATING,
            MissionState.EXCAVATING: MissionStatusMsg.EXCAVATING,
            MissionState.COMPLETED: MissionStatusMsg.COMPLETED,
            MissionState.FAILED: MissionStatusMsg.FAILED,
        }
        msg.state = STATE_MAP.get(p.state, MissionStatusMsg.IDLE)
        msg.current_scoop_id = p.current_scoop_index
        msg.total_scoops = p.total_scoops
        msg.completion_fraction = p.fraction_complete
        msg.remaining_volume = self.controller.grid.remaining_volume
        msg.status_text = p.status_text

        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
