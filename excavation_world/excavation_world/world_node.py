"""
world_node.py – ROS 2 node that holds the excavation grid, publishes its
state as visualization markers and an ExcavationGrid message.

Parameters (ROS):
    resolution          (double, default 0.25)       cell size [m]
    hole_origin_x/y/z   (double)                     world position of the hole
    hole_size_x/y        (double)                     hole footprint [m]
    hole_depth           (double)                     target depth [m]
    publish_rate         (double, default 2.0)        Hz
    working_position_x/y/z/yaw (double)              predefined working position

Subscriptions:
    /excavation/apply_scoop  (ScoopAction)  – apply a scoop to the grid

Published topics:
    /excavation/markers           (MarkerArray)  – excavated cells
    /excavation/target_markers    (MarkerArray)  – target volume
    /excavation/working_position  (Marker)       – working position arrow
    /excavation/grid_state        (ExcavationGrid) – volumetric state
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import StaticTransformBroadcaster

from excavation_msgs.msg import ScoopAction as ScoopActionMsg
from excavation_msgs.msg import ExcavationGrid as ExcavationGridMsg

from excavation_world.excavation_grid import ExcavationGrid, HoleSpec, EXCAVATED
from excavation_world.excavation_model import (
    apply_scoop_to_grid,
)
from excavation_world.parameters import (
    declare_world_node_parameters,
    retrieve_world_node_parameters,
)

import math
import numpy as np


class WorldNode(Node):
    """Maintains the excavation grid and publishes visual / state topics."""

    def __init__(self) -> None:
        super().__init__('excavation_world')

        # --- Declare all parameters at once (single source of truth) ---
        declare_world_node_parameters(self)

        # --- Read all parameters at once (validated + type-safe) ---
        params = retrieve_world_node_parameters(self)
        self.get_logger().info(f'Parameters loaded: resolution={params.hole_geometry.resolution}')

        # --- Build grid from hole geometry ---
        hole = HoleSpec(
            origin_x=params.hole_geometry.hole_origin_x,
            origin_y=params.hole_geometry.hole_origin_y,
            origin_z=params.hole_geometry.hole_origin_z,
            size_x=params.hole_geometry.hole_size_x,
            size_y=params.hole_geometry.hole_size_y,
            depth=params.hole_geometry.hole_depth,
        )
        self.grid = ExcavationGrid.from_hole_spec(hole, resolution=params.hole_geometry.resolution)
        self.get_logger().info(f'Initialised: {self.grid}')

        # --- Cache working position & hole geometry for later use ---
        self._params = params
        self.working_position = {
            'x': params.working_position.working_position_x,
            'y': params.working_position.working_position_y,
            'z': params.working_position.working_position_z,
            'yaw': params.working_position.working_position_yaw,
        }

        # --- Publishers ---
        # Use depth=5 volatile QoS so Foxglove bridge can subscribe.
        # The timer re-publishes periodically, so latching is not needed.
        marker_qos = QoSProfile(depth=5)

        self.marker_pub = self.create_publisher(
            MarkerArray, '/excavation/markers', marker_qos)
        self.target_marker_pub = self.create_publisher(
            MarkerArray, '/excavation/target_markers', marker_qos)
        self.work_pos_pub = self.create_publisher(
            Marker, '/excavation/working_position', marker_qos)
        self.grid_state_pub = self.create_publisher(
            ExcavationGridMsg, '/excavation/grid_state', 10)

        # --- Subscribers ---
        self.create_subscription(
            ScoopActionMsg, '/excavation/apply_scoop',
            self._scoop_cb, 10)

        # --- Static TF: publish a world frame so Foxglove has a ---
        # --- stable fixed frame even before base_motion_node starts ---
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_world_tf()

        # --- Timers ---
        # Fast timer: grid state only (lightweight, no markers)
        rate = params.publish_rate
        self.create_timer(1.0 / rate, self._fast_timer_cb)

        # Slow timer: republish static markers as keepalive for late
        # Foxglove connections (every 10 s).  Target cubes + hole frame
        # + working position never change, so 2 Hz is overkill and
        # causes blinking.
        self.create_timer(10.0, self._slow_timer_cb)

        # Publish everything once immediately
        self._publish_target_and_frame_markers()
        self._publish_working_position()
        self._publish_excavation_markers()
        self._publish_grid_state()

    # ------------------------------------------------------------------ #
    #  Periodic publish
    # ------------------------------------------------------------------ #
    def _fast_timer_cb(self) -> None:
        """Lightweight tick — only grid state (no marker re-rendering)."""
        self._publish_grid_state()

    def _slow_timer_cb(self) -> None:
        """Infrequent keepalive so late Foxglove subscribers see markers."""
        self._publish_target_and_frame_markers()
        self._publish_working_position()
        self._publish_excavation_markers()

    def _initial_republish(self) -> None:
        """One-shot republish after startup delay for Foxglove."""
        if not self._initial_done:
            self._initial_done = True
            self._publish_target_and_frame_markers()
            self._publish_working_position()

    # ------------------------------------------------------------------ #
    #  Scoop subscription callback
    # ------------------------------------------------------------------ #
    def _scoop_cb(self, msg: ScoopActionMsg) -> None:
        """Apply a scoop received as a ScoopAction message.

        The message may contain pre-computed ``affected_cell_indices`` (flat
        indices).  If it does, those are used directly.  Otherwise, the scoop
        is computed from the entry_pose position using the excavation model.
        """
        if len(msg.affected_cell_indices) > 0:
            count = self.grid.excavate_flat_indices(
                list(msg.affected_cell_indices))
        else:
            # Derive dig target from the entry_pose
            p = msg.entry_pose.position
            target = np.array([p.x, p.y, p.z])
            result = apply_scoop_to_grid(
                self.grid, target,
                base_yaw=self.working_position['yaw'],
                scoop_id=msg.scoop_id,
            )
            count = result.target_cells_removed

        self.get_logger().info(
            f'Scoop {msg.scoop_id}: removed {count} target cells, '
            f'remaining={self.grid.remaining_target_cells}, '
            f'completion={self.grid.completion_fraction:.1%}')
        # Immediately update visualization
        self._publish_excavation_markers()
        self._publish_grid_state()

    # ------------------------------------------------------------------ #
    #  Marker publishers
    # ------------------------------------------------------------------ #
    def _publish_static_world_tf(self) -> None:
        """Publish static identity TF so 'world' is always available."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'world_fixed'
        t.transform.rotation.w = 1.0
        self._static_tf_broadcaster.sendTransform(t)

    def _publish_target_and_frame_markers(self) -> None:
        """Publish target cubes AND hole frame in ONE MarkerArray.

        Combining them avoids flickering: Foxglove replaces all markers
        from a topic when a new MarkerArray arrives, so two separate
        publishes would alternate and blink.
        """
        now = self.get_clock().now().to_msg()
        ma = MarkerArray()

        # --- Target cubes ---
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = now
        marker.ns = 'target_hole'
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale = Vector3(
            x=self.grid.resolution * 0.95,
            y=self.grid.resolution * 0.95,
            z=self.grid.resolution * 0.95,
        )
        marker.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.5)
        marker.pose.orientation.w = 1.0

        indices = self.grid.target_flat_indices()
        nx, ny, nz = self.grid.shape
        for fi in indices:
            ix = int(fi) // (ny * nz)
            rem = int(fi) % (ny * nz)
            iy = rem // nz
            iz = rem % nz
            cx, cy, cz = self.grid.cell_centre(ix, iy, iz)
            marker.points.append(Point(x=cx, y=cy, z=cz))

        ma.markers.append(marker)

        # --- Hole frame (lines + text) ---
        ox = self._params.hole_geometry.hole_origin_x
        oy = self._params.hole_geometry.hole_origin_y
        oz = self._params.hole_geometry.hole_origin_z
        sx = self._params.hole_geometry.hole_size_x
        sy = self._params.hole_geometry.hole_size_y
        depth = self._params.hole_geometry.hole_depth

        # Top rectangle
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = now
        m.ns = 'hole_frame'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.08
        m.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        m.pose.orientation.w = 1.0
        m.points = [
            Point(x=ox, y=oy, z=oz),
            Point(x=ox + sx, y=oy, z=oz),
            Point(x=ox + sx, y=oy + sy, z=oz),
            Point(x=ox, y=oy + sy, z=oz),
            Point(x=ox, y=oy, z=oz),
        ]
        ma.markers.append(m)

        # Vertical depth lines
        for i, (cx, cy) in enumerate([
            (ox, oy), (ox + sx, oy),
            (ox + sx, oy + sy), (ox, oy + sy),
        ]):
            vm = Marker()
            vm.header.frame_id = 'world'
            vm.header.stamp = now
            vm.ns = 'hole_frame'
            vm.id = 1 + i
            vm.type = Marker.LINE_STRIP
            vm.action = Marker.ADD
            vm.scale.x = 0.06
            vm.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.6)
            vm.pose.orientation.w = 1.0
            vm.points = [
                Point(x=cx, y=cy, z=oz),
                Point(x=cx, y=cy, z=oz - depth),
            ]
            ma.markers.append(vm)

        # Text label
        txt = Marker()
        txt.header.frame_id = 'world'
        txt.header.stamp = now
        txt.ns = 'hole_frame'
        txt.id = 10
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.scale.z = 0.5
        txt.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        txt.pose.position = Point(
            x=ox + sx / 2.0, y=oy + sy / 2.0, z=oz + 0.6)
        txt.pose.orientation.w = 1.0
        txt.text = f'HOLE ({sx}x{sy}x{depth}m)'
        ma.markers.append(txt)

        self.target_marker_pub.publish(ma)

    def _publish_excavation_markers(self) -> None:
        """Publish cubes for excavated cells (orange)."""
        ma = MarkerArray()
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'excavated'
        marker.id = 0
        marker.pose.orientation.w = 1.0

        nx, ny, nz = self.grid.shape
        points = []
        for ix in range(nx):
            for iy in range(ny):
                for iz in range(nz):
                    if self.grid.is_excavated(ix, iy, iz):
                        cx, cy, cz = self.grid.cell_centre(ix, iy, iz)
                        points.append(Point(x=cx, y=cy, z=cz))

        if points:
            marker.type = Marker.CUBE_LIST
            marker.action = Marker.ADD
            marker.scale = Vector3(
                x=self.grid.resolution * 0.92,
                y=self.grid.resolution * 0.92,
                z=self.grid.resolution * 0.92,
            )
            marker.color = ColorRGBA(r=0.9, g=0.5, b=0.1, a=0.8)
            marker.points = points
        else:
            # Delete any previously shown marker (avoid empty CUBE_LIST)
            marker.action = Marker.DELETEALL

        ma.markers.append(marker)
        self.marker_pub.publish(ma)

    def _publish_working_position(self) -> None:
        """Publish an arrow marker at the predefined working position."""
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'working_position'
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.scale = Vector3(x=1.5, y=0.3, z=0.3)
        m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)

        yaw = self.working_position['yaw']
        m.pose = Pose(
            position=Point(
                x=self.working_position['x'],
                y=self.working_position['y'],
                z=self.working_position['z'] + 0.5,
            ),
            orientation=Quaternion(
                x=0.0, y=0.0,
                z=math.sin(yaw / 2.0),
                w=math.cos(yaw / 2.0),
            ),
        )
        self.work_pos_pub.publish(m)


    # ------------------------------------------------------------------ #
    #  Public API (called by other nodes via service / direct)
    # ------------------------------------------------------------------ #
    def apply_scoop(self, flat_indices):
        """Mark cells as excavated; returns number of newly excavated target cells."""
        count = self.grid.excavate_flat_indices(flat_indices)
        self.get_logger().info(
            f'Scoop applied: {count} new target cells excavated, '
            f'remaining={self.grid.remaining_target_cells}')
        return count

    def apply_scoop_at(self, dig_target_xyz, base_yaw=0.0, cabin_angle=0.0,
                       footprint=None, scoop_id=0):
        """Apply a scoop via the excavation model (world-frame target)."""
        result = apply_scoop_to_grid(
            self.grid, np.asarray(dig_target_xyz),
            base_yaw=base_yaw, cabin_angle=cabin_angle,
            footprint=footprint, scoop_id=scoop_id,
        )
        self.get_logger().info(
            f'Scoop {scoop_id}: removed {result.target_cells_removed} target cells, '
            f'remaining={result.remaining_target_cells}, '
            f'completion={result.completion_fraction:.1%}')
        self._publish_excavation_markers()
        self._publish_grid_state()
        return result

    # ------------------------------------------------------------------ #
    #  Grid state publisher
    # ------------------------------------------------------------------ #
    def _publish_grid_state(self) -> None:
        """Publish the current grid summary as an ExcavationGrid message."""
        msg = ExcavationGridMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.resolution = self.grid.resolution
        msg.size_x, msg.size_y, msg.size_z = self.grid.shape
        gx, gy, gz = self.grid.grid_origin
        msg.origin = Point(x=gx, y=gy, z=gz)
        msg.total_cells = self.grid.total_target_cells
        msg.excavated_cells = self.grid.excavated_target_cells
        msg.remaining_cells = self.grid.remaining_target_cells
        msg.remaining_volume = self.grid.remaining_volume
        msg.completion_fraction = self.grid.completion_fraction
        self.grid_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WorldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
