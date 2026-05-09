"""
debug_visualizer_node.py – Visualization & debug overlay for the excavation.

Publishes:
    /debug/bucket_trail        MarkerArray  – trail of the bucket tip over time
    /debug/scoop_targets       MarkerArray  – planned scoop dig targets
    /debug/status_text         MarkerArray  – floating text with mission stats
    /debug/arm_trajectory      MarkerArray  – waypoints of the current scoop

Subscribes:
    /joint_states              JointState   – used to compute bucket tip via FK
    /mission/status            MissionStatus
    /excavation/grid_state     ExcavationGrid

This node is purely for visualization; it does not affect execution.
"""

from __future__ import annotations

import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

from excavation_msgs.msg import (
    MissionStatus as MissionStatusMsg,
    ExcavationGrid as ExcavationGridMsg,
)

from excavation_world.robot_model import ExcavatorModel, JOINT_NAMES
from excavation_world.parameters import (
    declare_debug_visualizer_node_parameters,
    retrieve_debug_visualizer_node_parameters,
)

import numpy as np


class DebugVisualizerNode(Node):
    """Aggregates debug data and publishes visual markers."""

    def __init__(self) -> None:
        super().__init__('debug_visualizer')

        # ----- Declare all parameters at once (single source of truth) -----
        declare_debug_visualizer_node_parameters(self)

        # ----- Read all parameters at once (validated + type-safe) -----
        params = retrieve_debug_visualizer_node_parameters(self)

        # ----- Store base position & trail parameters -----
        self.base_x = params.base_position.base_x
        self.base_y = params.base_position.base_y
        self.base_yaw = params.base_position.base_yaw
        self.trail_max = params.trail_max_points

        # State
        self._joint_positions: dict[str, float] = {}
        self._mission_status: MissionStatusMsg | None = None
        self._grid_state: ExcavationGridMsg | None = None
        self._bucket_trail: deque[Point] = deque(maxlen=self.trail_max)

        # Publishers
        qos = QoSProfile(depth=5)
        self.trail_pub = self.create_publisher(
            MarkerArray, '/debug/bucket_trail', qos)
        self.text_pub = self.create_publisher(
            MarkerArray, '/debug/status_text', qos)

        # Subscribers
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10)
        self.create_subscription(
            MissionStatusMsg, '/mission/status', self._mission_cb, 10)
        self.create_subscription(
            ExcavationGridMsg, '/excavation/grid_state', self._grid_cb, 10)

        # Timer
        self.create_timer(1.0 / params.publish_rate, self._tick)

        self.get_logger().info('DebugVisualizer ready')

    # ------------------------------------------------------------------ #
    #  Subscribers
    # ------------------------------------------------------------------ #
    def _joint_state_cb(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self._joint_positions[name] = pos

        # Compute bucket tip and append to trail
        if all(j in self._joint_positions for j in JOINT_NAMES):
            joints = np.array([self._joint_positions[j] for j in JOINT_NAMES])
            model = ExcavatorModel(
                joint_positions=joints,
                base_x=self.base_x,
                base_y=self.base_y,
                base_yaw=self.base_yaw,
            )
            tip = model.bucket_tip_position()
            self._bucket_trail.append(
                Point(x=float(tip[0]), y=float(tip[1]), z=float(tip[2])))

    def _mission_cb(self, msg: MissionStatusMsg) -> None:
        self._mission_status = msg

    def _grid_cb(self, msg: ExcavationGridMsg) -> None:
        self._grid_state = msg

    # ------------------------------------------------------------------ #
    #  Timer
    # ------------------------------------------------------------------ #
    def _tick(self) -> None:
        self._publish_trail()
        self._publish_status_text()

    # ------------------------------------------------------------------ #
    #  Bucket trail
    # ------------------------------------------------------------------ #
    def _publish_trail(self) -> None:
        ma = MarkerArray()
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'bucket_trail'
        m.id = 0
        m.pose.orientation.w = 1.0

        if len(self._bucket_trail) >= 2:
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.04  # line width
            m.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.7)
            m.points = list(self._bucket_trail)
        else:
            m.action = Marker.DELETEALL

        ma.markers.append(m)
        self.trail_pub.publish(ma)

    # ------------------------------------------------------------------ #
    #  Status text overlay
    # ------------------------------------------------------------------ #
    def _publish_status_text(self) -> None:
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # --- Mission status ---
        lines = []
        if self._mission_status is not None:
            ms = self._mission_status
            state_names = {0: 'IDLE', 1: 'MOVING', 2: 'EXCAVATING',
                           3: 'COMPLETED', 4: 'FAILED'}
            state = state_names.get(ms.state, f'?{ms.state}')
            lines.append(f'Mission: {state}')
            if ms.total_scoops > 0:
                lines.append(
                    f'Scoop: {ms.current_scoop_id}/{ms.total_scoops}')
                lines.append(
                    f'Done: {ms.completion_fraction:.0%}')
            lines.append(ms.status_text)

        if self._grid_state is not None:
            gs = self._grid_state
            lines.append(f'Remaining: {gs.remaining_volume:.1f}m3')
            lines.append(
                f'Cells: {gs.excavated_cells}/{gs.total_cells}')

        # Joint states
        if self._joint_positions:
            joint_str = '  '.join(
                f'{j[:3]}={math.degrees(self._joint_positions.get(j, 0.0)):.0f}°'
                for j in JOINT_NAMES)
            lines.append(f'Joints: {joint_str}')

        text = '\n'.join(lines) if lines else 'Waiting for data...'

        txt = Marker()
        txt.header.frame_id = 'world'
        txt.header.stamp = now
        txt.ns = 'debug_text'
        txt.id = 0
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.scale.z = 0.35  # text height
        txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
        txt.pose.position = Point(
            x=self.base_x, y=self.base_y, z=3.0)
        txt.pose.orientation.w = 1.0
        txt.text = text
        ma.markers.append(txt)

        self.text_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = DebugVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
