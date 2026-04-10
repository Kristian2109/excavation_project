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
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from visualization_msgs.msg import Marker, MarkerArray

from excavation_world.excavation_grid import ExcavationGrid, HoleSpec, EXCAVATED

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
        latching = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.marker_pub = self.create_publisher(
            MarkerArray, '/excavation/markers', latching)
        self.target_marker_pub = self.create_publisher(
            MarkerArray, '/excavation/target_markers', latching)
        self.work_pos_pub = self.create_publisher(
            Marker, '/excavation/working_position', latching)

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
        self._publish_excavation_markers()

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
