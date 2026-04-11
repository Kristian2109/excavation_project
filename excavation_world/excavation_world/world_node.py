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
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from visualization_msgs.msg import Marker, MarkerArray

from excavation_msgs.msg import ScoopAction as ScoopActionMsg
from excavation_msgs.msg import ExcavationGrid as ExcavationGridMsg

from excavation_world.excavation_grid import ExcavationGrid, HoleSpec, EXCAVATED
from excavation_world.excavation_model import (
    ScoopFootprint,
    apply_scoop_to_grid,
)

import math
import numpy as np


class WorldNode(Node):
    """Maintains the excavation grid and publishes visual / state topics."""

    def __init__(self) -> None:
        super().__init__('excavation_world')

        # --- Declare parameters ---
        self.declare_parameter('resolution', 0.25)
        self.declare_parameter('hole_origin_x', 5.0)
        self.declare_parameter('hole_origin_y', -2.0)
        self.declare_parameter('hole_origin_z', 0.0)
        self.declare_parameter('hole_size_x', 4.0)
        self.declare_parameter('hole_size_y', 3.0)
        self.declare_parameter('hole_depth', 2.0)
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('working_position_x', 3.0)
        self.declare_parameter('working_position_y', 0.0)
        self.declare_parameter('working_position_z', 0.0)
        self.declare_parameter('working_position_yaw', 0.0)

        # --- Read parameters ---
        res = self.get_parameter('resolution').value
        hole = HoleSpec(
            origin_x=self.get_parameter('hole_origin_x').value,
            origin_y=self.get_parameter('hole_origin_y').value,
            origin_z=self.get_parameter('hole_origin_z').value,
            size_x=self.get_parameter('hole_size_x').value,
            size_y=self.get_parameter('hole_size_y').value,
            depth=self.get_parameter('hole_depth').value,
        )

        # --- Build grid ---
        self.grid = ExcavationGrid.from_hole_spec(hole, resolution=res)
        self.get_logger().info(f'Initialised: {self.grid}')

        # Store working position for other nodes
        self.working_position = {
            'x': self.get_parameter('working_position_x').value,
            'y': self.get_parameter('working_position_y').value,
            'z': self.get_parameter('working_position_z').value,
            'yaw': self.get_parameter('working_position_yaw').value,
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

        # --- Timer ---
        rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / rate, self._timer_cb)

        # Publish once immediately
        self._publish_target_markers()
        self._publish_working_position()
        self._publish_excavation_markers()

    # ------------------------------------------------------------------ #
    #  Periodic publish
    # ------------------------------------------------------------------ #
    def _timer_cb(self) -> None:
        self._publish_target_markers()
        self._publish_working_position()
        self._publish_excavation_markers()
        self._publish_grid_state()

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
    def _publish_target_markers(self) -> None:
        """Publish translucent cubes for the full target volume."""
        ma = MarkerArray()
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'target_hole'
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale = Vector3(
            x=self.grid.resolution * 0.95,
            y=self.grid.resolution * 0.95,
            z=self.grid.resolution * 0.95,
        )
        marker.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.25)
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
        self.target_marker_pub.publish(ma)
        self.get_logger().info(
            f'Published target markers: {len(marker.points)} cells')

    def _publish_excavation_markers(self) -> None:
        """Publish cubes for excavated cells (orange)."""
        ma = MarkerArray()
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'excavated'
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale = Vector3(
            x=self.grid.resolution * 0.92,
            y=self.grid.resolution * 0.92,
            z=self.grid.resolution * 0.92,
        )
        marker.color = ColorRGBA(r=0.9, g=0.5, b=0.1, a=0.8)
        marker.pose.orientation.w = 1.0

        nx, ny, nz = self.grid.shape
        for ix in range(nx):
            for iy in range(ny):
                for iz in range(nz):
                    if self.grid.is_excavated(ix, iy, iz):
                        cx, cy, cz = self.grid.cell_centre(ix, iy, iz)
                        marker.points.append(Point(x=cx, y=cy, z=cz))

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
