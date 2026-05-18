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

from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import StaticTransformBroadcaster

from excavation_msgs.msg import ScoopAction as ScoopActionMsg
from excavation_msgs.msg import ExcavationGrid as ExcavationGridMsg

from excavation_core.excavation_grid import ExcavationGrid
from excavation_core.excavation_model import (
    apply_scoop_to_grid,
)
from excavation_core.ik_solver import solve_ik_nearest
from excavation_core.position_planner import compute_work_positions
from excavation_core.parameters import (
    declare_world_node_parameters,
    retrieve_world_node_parameters,
)
from excavation_world.world_markers import (
    build_target_and_frame_markers,
    build_excavation_markers,
    build_working_position_markers,
)

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
        hole = params.hole_geometry.to_hole_spec()
        self.grid = ExcavationGrid.from_hole_spec(hole, resolution=params.hole_geometry.resolution)
        self.get_logger().info(f'Initialised: {self.grid}')

        # --- Cache working position & hole geometry for later use ---
        self._params = params
        self._hole = hole

        # Compute work positions from hole geometry
        self._work_positions = compute_work_positions(hole)
        self.get_logger().info(
            f'Computed {len(self._work_positions)} work position(s)')

        # Use first position for reachability scanning
        first_pos = self._work_positions[0]
        self.working_position = {
            'x': first_pos.x,
            'y': first_pos.y,
            'z': 0.0,
            'yaw': first_pos.yaw,
        }
        self._target_point_by_flat: dict[int, Point] = {}
        self._target_reachable_by_flat: dict[int, bool] = {}
        self._target_flat_indices: list[int] = []
        self._reachability_scan_index = 0
        self._reachability_scan_batch = 20
        self._reachability_scan_timer = None
        self._reachability_reachable = 0
        self._reachability_unreachable = 0
        self._cache_target_marker_data()

        # --- Publishers ---
        # Use depth=5 volatile QoS so Foxglove bridge can subscribe.
        # The timer re-publishes periodically, so latching is not needed.
        marker_qos = QoSProfile(depth=5)

        self.marker_pub = self.create_publisher(
            MarkerArray, '/excavation/markers', marker_qos)
        self.target_marker_pub = self.create_publisher(
            MarkerArray, '/excavation/target_markers', marker_qos)
        self.grid_state_pub = self.create_publisher(
            ExcavationGridMsg, '/excavation/grid_state', 10)

        # --- Subscribers ---
        self.create_subscription(
            ScoopActionMsg, '/excavation/apply_scoop',
            self._scoop_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._recompute_target_reachability, 10)

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
        self._publish_excavation_markers()
        self._publish_grid_state()

        # Reachability coloring is computed incrementally to avoid startup freeze.
        self._reachability_scan_timer = self.create_timer(
            0.05, self._reachability_scan_tick)

    def _recompute_target_reachability(self, msg: PoseStamped) -> None:
        """Recompute reachability for all target cells from a new working position.

        Called when a new goal pose arrives on /goal_pose.  Extracts x/y/yaw
        from the PoseStamped, updates ``self.working_position``, resets all
        reachability state, and restarts the incremental scan timer.
        """
        q = msg.pose.orientation
        # yaw from quaternion (rotation around Z)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = float(np.arctan2(siny_cosp, cosy_cosp))

        self.working_position = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z,
            'yaw': yaw,
        }
        self.get_logger().info(
            f'Working position updated: x={self.working_position["x"]:.3f}, '
            f'y={self.working_position["y"]:.3f}, yaw={yaw:.3f} rad — '
            'restarting reachability scan')

        # Reset per-cell reachability to True (optimistic) while scan runs
        for fi in self._target_flat_indices:
            self._target_reachable_by_flat[fi] = True

        self._reachability_scan_index = 0
        self._reachability_reachable = 0
        self._reachability_unreachable = 0
        if self._reachability_scan_timer is not None:
            self._reachability_scan_timer.cancel()
        self._reachability_scan_timer = self.create_timer(
            0.05, self._reachability_scan_tick)
    # ------------------------------------------------------------------ #
    #  Periodic publish
    # ------------------------------------------------------------------ #
    def _fast_timer_cb(self) -> None:
        """Lightweight tick — only grid state (no marker re-rendering)."""
        self._publish_grid_state()

    def _slow_timer_cb(self) -> None:
        """Infrequent keepalive so late Foxglove subscribers see markers."""
        self._publish_target_and_frame_markers()
        self._publish_excavation_markers()

    def _initial_republish(self) -> None:
        """One-shot republish after startup delay for Foxglove."""
        if not self._initial_done:
            self._initial_done = True
            self._publish_target_and_frame_markers()

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
        self._publish_target_and_frame_markers()
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
        """Publish target cubes AND hole frame in ONE MarkerArray."""
        hg = self._params.hole_geometry
        ma = build_target_and_frame_markers(
            self.grid,
            self._target_point_by_flat,
            self._target_reachable_by_flat,
            hole_origin_x=hg.hole_origin_x,
            hole_origin_y=hg.hole_origin_y,
            hole_origin_z=hg.hole_origin_z,
            hole_size_x=hg.hole_size_x,
            hole_size_y=hg.hole_size_y,
            hole_depth=hg.hole_depth,
            stamp=self.get_clock().now().to_msg(),
        )
        self.target_marker_pub.publish(ma)

    def _cache_target_marker_data(self) -> None:
        """Cache target cube positions; compute reachability colors asynchronously."""
        point_by_flat: dict[int, Point] = {}
        reachable_by_flat: dict[int, bool] = {}

        indices = self.grid.target_flat_indices()
        self._target_flat_indices = [int(fi) for fi in indices]
        nx, ny, nz = self.grid.shape
        for fi in indices:
            ix = int(fi) // (ny * nz)
            rem = int(fi) % (ny * nz)
            iy = rem // nz
            iz = rem % nz
            cx, cy, cz = self.grid.cell_centre(ix, iy, iz)

            point_by_flat[int(fi)] = Point(x=cx, y=cy, z=cz)
            reachable_by_flat[int(fi)] = True

        self._target_point_by_flat = point_by_flat
        self._target_reachable_by_flat = reachable_by_flat
        self.get_logger().info(
            f'Cached {len(self._target_flat_indices)} target cells; '
            'starting background reachability scan')

    def _reachability_scan_tick(self) -> None:
        """Incrementally compute reachability colors to keep startup responsive."""
        if self._reachability_scan_index >= len(self._target_flat_indices):
            if self._reachability_scan_timer is not None:
                self._reachability_scan_timer.cancel()
                self._reachability_scan_timer = None
            self.get_logger().info(
                'Target reachability scan complete: '
                f'{self._reachability_reachable} reachable, '
                f'{self._reachability_unreachable} unreachable')
            return

        end = min(
            self._reachability_scan_index + self._reachability_scan_batch,
            len(self._target_flat_indices),
        )
        for idx in range(self._reachability_scan_index, end):
            fi = self._target_flat_indices[idx]
            pt = self._target_point_by_flat[fi]
            ik_result = solve_ik_nearest(
                np.array([pt.x, pt.y, pt.z]),
                x_base_world_frame=self.working_position['x'],
                y_base_world_frame=self.working_position['y'],
                yaw_base_world_frame=self.working_position['yaw'],
            )
            if ik_result.success:
                self._target_reachable_by_flat[fi] = True
                self._reachability_reachable += 1
            else:
                self._target_reachable_by_flat[fi] = False
                self._reachability_unreachable += 1

        self._reachability_scan_index = end
        self._publish_target_and_frame_markers()

    def _publish_excavation_markers(self) -> None:
        """Publish cubes for excavated cells (orange)."""
        ma = build_excavation_markers(
            self.grid,
            stamp=self.get_clock().now().to_msg(),
        )
        self.marker_pub.publish(ma)

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
        self._publish_target_and_frame_markers()
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
