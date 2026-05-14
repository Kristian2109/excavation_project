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
    auto_start (bool, default True)
        Immediately start the mission (base motion already running).
    scoop_delay (float, default 0.5)
        Minimum seconds between consecutive scoops.
    execution_speed (float, default 1.0)
        Speed multiplier for excavation execution timing.
        Example: 2.0 runs about 2x faster, 0.5 runs about 2x slower.
"""

from __future__ import annotations

import time
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
from visualization_msgs.msg import MarkerArray

from excavation_msgs.msg import (
    MissionStatus as MissionStatusMsg,
    ScoopAction as ScoopActionMsg,
)

from excavation_core.mission_controller import (
    MissionController,
    MissionState,
)
from excavation_core.excavation_grid import ExcavationGrid
from excavation_core.excavation_planner import PlannedScoop
from excavation_core.scoop_trajectory import (
    ScoopTrajectory,
    plan_single_scoop,
)
from excavation_core.robot_model import JOINT_NAMES
from excavation_core.position_planner import compute_work_positions
from excavation_core.parameters import (
    declare_mission_controller_node_parameters,
    retrieve_mission_controller_node_parameters,
)
from excavation_mission.mission_viz import (
    build_scoop_target_markers,
    build_arm_trajectory_markers,
)


class MissionControllerNode(Node):
    """Orchestrates base-motion → plan → excavate → done."""

    def __init__(self) -> None:
        super().__init__('mission_controller')

        params = self._load_parameters()
        self.controller = self._build_controller(params)
        self._cache_execution_settings(params)
        self._initialize_runtime_state()
        self._create_publishers()
        self._create_subscribers()
        self._create_action_client()
        self._create_timers()
        self._maybe_auto_start(params)

        self._publish_status()
        self.get_logger().info(
            f'MissionController ready  (execution_speed={self._execution_speed:.2f}x)')

    def _load_parameters(self):
        """Declare and retrieve node parameters from the shared parameter module."""
        declare_mission_controller_node_parameters(self)
        params = retrieve_mission_controller_node_parameters(self)
        self.get_logger().info(
            f'Parameters loaded: hole_depth={params.hole_geometry.hole_depth}m')
        return params

    def _build_controller(self, params) -> MissionController:
        """Create hole/grid/work-position objects and initialize mission controller."""
        hole = params.hole_geometry.to_hole_spec()
        grid = ExcavationGrid.from_hole_spec(
            hole,
            resolution=params.hole_geometry.resolution,
        )
        work_positions = compute_work_positions(hole)

        self.get_logger().info(
            f'Computed {len(work_positions)} work position(s): '
            + ', '.join(
                f'({p.x:.2f}, {p.y:.2f}, yaw={math.degrees(p.yaw):.0f}°)'
                for p in work_positions
            ))

        return MissionController(
            hole=hole, grid=grid,
            work_positions=work_positions,
        )

    def _cache_execution_settings(self, params) -> None:
        """Cache execution-related settings for mission timing."""
        self._scoop_delay = params.scoop_delay
        self._execution_speed = float(params.execution_speed)
        if self._execution_speed <= 0.0:
            self.get_logger().warn(
                f'Invalid execution_speed={self._execution_speed}; using 1.0')
            self._execution_speed = 1.0

    def _initialize_runtime_state(self) -> None:
        """Initialize mutable runtime state used during mission execution."""
        self._scoop_active = False
        self._current_scoop: PlannedScoop | None = None
        self._last_scoop_time = 0.0
        self._goal_handle = None
        self._relocate_sent = False

    def _create_publishers(self) -> None:
        """Create all publishers used by this node."""
        self.status_pub = self.create_publisher(
            MissionStatusMsg, '/mission/status', 10)
        self.scoop_action_pub = self.create_publisher(
            ScoopActionMsg, '/excavation/apply_scoop', 10)
        self._goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        
        self.arm_traj_pub = self.create_publisher(
            MarkerArray, '/debug/arm_trajectory', QoSProfile(depth=5))
        self.scoop_targets_pub = self.create_publisher(
            MarkerArray, '/debug/scoop_targets', QoSProfile(depth=5))


    def _create_subscribers(self) -> None:
        """Create all subscribers used by this node."""
        self.create_subscription(
            Bool, '/base_motion/done', self._base_done_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def _create_action_client(self) -> None:
        """Create FollowJointTrajectory action client for arm execution."""
        self._cb_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            callback_group=self._cb_group,
        )

    def _create_timers(self) -> None:
        """Create periodic timers."""
        self.create_timer(0.5, self._tick)

    def _maybe_auto_start(self, params) -> None:
        """Start mission immediately when configured."""
        if params.auto_start:
            self.controller.start_mission()
            self.get_logger().info('Mission auto-started → MOVING_TO_WORK_POS')

    # ------------------------------------------------------------------ #
    #  Tick – main loop
    # ------------------------------------------------------------------ #
    def _tick(self) -> None:
        state = self.controller.state

        if state == MissionState.PLANNING:
            self._do_planning()

        elif state == MissionState.EXCAVATING:
            self._handle_excavating_state()

        elif state == MissionState.RELOCATING:
            self._do_relocate()

        self._publish_status()

    def _handle_excavating_state(self) -> None:
        """Handle excavation state work for this tick."""
        if self._scoop_active:
            return
        if not self._scoop_delay_elapsed():
            return
        self._advance_excavation()

    def _scoop_delay_elapsed(self) -> bool:
        """Return True when enough time passed before starting next scoop."""
        effective_delay = self._scoop_delay / self._execution_speed
        return (time.monotonic() - self._last_scoop_time) >= effective_delay

    # ------------------------------------------------------------------ #
    #  Base-motion callback
    # ------------------------------------------------------------------ #
    def _base_done_cb(self, msg: Bool) -> None:
        if not msg.data:
            return
        if self.controller.state == MissionState.MOVING_TO_WORK_POS:
            pass  # initial motion — accept
        elif (self.controller.state == MissionState.RELOCATING
              and self._relocate_sent):
            pass  # relocation goal was sent — accept
        else:
            return
        pos = self.controller.current_work_position
        self.get_logger().info(
            f'Base arrived at position {self.controller._position_index + 1}'
            f'/{len(self.controller.work_positions)} '
            f'({pos.x:.2f}, {pos.y:.2f}) → PLANNING')
        self.controller.on_base_arrived()
        self._publish_status()

    # ------------------------------------------------------------------ #
    #  Relocation
    # ------------------------------------------------------------------ #
    def _do_relocate(self) -> None:
        """Send goal_pose to base_motion_node for the next position."""
        if self._relocate_sent:
            return  # waiting for base_done_cb

        pos = self.controller.current_work_position
        self.get_logger().info(
            f'Relocating base to position '
            f'{self.controller._position_index + 1}'
            f'/{len(self.controller.work_positions)} '
            f'({pos.x:.2f}, {pos.y:.2f}, '
            f'yaw={math.degrees(pos.yaw):.0f}°)')

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'world'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = pos.x
        goal_msg.pose.position.y = pos.y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation = Quaternion(
            x=0.0, y=0.0,
            z=math.sin(pos.yaw / 2.0),
            w=math.cos(pos.yaw / 2.0),
        )
        self._goal_pub.publish(goal_msg)
        self._relocate_sent = True

    # ------------------------------------------------------------------ #
    #  Planning
    # ------------------------------------------------------------------ #
    def _do_planning(self) -> None:
        self._relocate_sent = False
        self.get_logger().info('Generating excavation plan …')
        if not self.controller.generate_plan():
            self.get_logger().error(
                f'Planning failed: {self.controller.progress.status_text}')
            return

        # Pre-filter unreachable scoops via the controller's
        # filter_unreachable() method (IK check lives in the node layer).
        if self.controller.plan is not None:
            def _ik_check(scoop):
                return plan_single_scoop(
                    scoop.dig_target,
                    base_x=self.controller.base_x,
                    base_y=self.controller.base_y,
                    base_yaw=self.controller.base_yaw,
                    scoop_id=scoop.scoop_id,
                )

            removed = self.controller.filter_unreachable(_ik_check)
            if removed > 0:
                self.get_logger().warn(
                    f'Filtered {removed} unreachable scoops during planning')

            if self.controller.plan.total_scoops == 0:
                self.get_logger().warn(
                    'No reachable scoops at this position')
                self._publish_status()
                return

        plan = self.controller.plan
        cov = plan.coverage_fraction(self.controller.grid)
        self.get_logger().info(
            f'Plan ready: {plan.total_scoops} scoops, '
            f'coverage={cov:.0%}')

        # Visualise all planned scoop targets
        self._publish_scoop_targets()

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

        # Use pre-planned trajectory when available.
        traj = scoop.trajectory
        if traj is None:
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
            self.controller.on_scoop_completed(False)
            self._last_scoop_time = time.monotonic()
            return

        # -- Send to arm controller (fully async, no blocking) --
        self._scoop_active = True
        self._current_scoop = scoop

        # Visualise this scoop's arm trajectory
        self._publish_arm_trajectory(traj)

        jt = self._build_joint_trajectory(traj)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt

        self.get_logger().info(
            f'Sending arm trajectory ({len(jt.points)} waypoints, '
            f'{self._trajectory_duration(traj):.1f}s at '
            f'{self._execution_speed:.2f}x)')

        send_future = self._action_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Arm goal rejected')
            self._finish_scoop(False)
            return

        self._goal_handle = goal_handle
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
        # Only apply terrain change when arm execution actually succeeded.
        # This prevents "free" excavation when trajectories fail/reject.
        if success:
            self._publish_scoop_action(self._current_scoop)
        self.controller.on_scoop_completed(success)
        self._scoop_active = False
        self._current_scoop = None
        self._last_scoop_time = time.monotonic()

    # ------------------------------------------------------------------ #
    #  Trajectory helpers
    # ------------------------------------------------------------------ #
    def _build_joint_trajectory(self, traj: ScoopTrajectory) -> JointTrajectory:
        jt = JointTrajectory()
        jt.joint_names = list(JOINT_NAMES)
        cumulative = 0.0
        for wp in traj.waypoints:
            cumulative += wp.duration / self._execution_speed
            pt = JointTrajectoryPoint()
            pt.positions = wp.joint_positions.tolist()
            pt.velocities = [0.0] * len(JOINT_NAMES)
            pt.time_from_start = Duration(
                sec=int(cumulative),
                nanosec=int((cumulative % 1.0) * 1e9),
            )
            jt.points.append(pt)
        return jt

    def _trajectory_duration(self, traj: ScoopTrajectory) -> float:
        return sum(wp.duration for wp in traj.waypoints) / self._execution_speed

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
            MissionState.RELOCATING: MissionStatusMsg.MOVING_TO_WORK_POS,
        }
        msg.state = STATE_MAP.get(p.state, MissionStatusMsg.IDLE)
        msg.current_scoop_id = p.current_scoop_index
        msg.total_scoops = p.total_scoops
        msg.completion_fraction = p.fraction_complete
        msg.remaining_volume = self.controller.grid.remaining_volume
        msg.status_text = p.status_text

        self.status_pub.publish(msg)

    # ------------------------------------------------------------------ #
    #  Visualization: scoop plan targets
    # ------------------------------------------------------------------ #
    def _publish_scoop_targets(self) -> None:
        """Publish sphere markers for all planned scoop dig targets."""
        if self.controller.plan is None:
            return
        ma = build_scoop_target_markers(
            self.controller.plan.scoops,
            stamp=self.get_clock().now().to_msg(),
        )
        self.scoop_targets_pub.publish(ma)

    # ------------------------------------------------------------------ #
    #  Visualization: current arm trajectory
    # ------------------------------------------------------------------ #
    def _publish_arm_trajectory(self, traj: ScoopTrajectory) -> None:
        """Publish a LINE_STRIP of the bucket-tip path for the current scoop."""
        lifetime = int(self._trajectory_duration(traj)) + 2
        ma = build_arm_trajectory_markers(
            traj,
            base_x=self.controller.base_x,
            base_y=self.controller.base_y,
            base_yaw=self.controller.base_yaw,
            stamp=self.get_clock().now().to_msg(),
            lifetime_sec=lifetime,
        )
        self.arm_traj_pub.publish(ma)


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
