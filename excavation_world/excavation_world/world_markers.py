"""
world_markers.py – Visualization helpers for the WorldNode.

Pure builder functions that construct ROS marker messages for the
excavation grid.  Extracted from ``world_node.py`` to keep the node
class focused on subscriptions, timers, and state.
"""

from __future__ import annotations

import math
from typing import Dict, List, Sequence

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Time

from excavation_core.excavation_grid import ExcavationGrid
from excavation_core.base_planner import BasePose


# ------------------------------------------------------------------ #
#  Target + hole-frame markers (combined to avoid Foxglove flickering)
# ------------------------------------------------------------------ #

def build_target_and_frame_markers(
    grid: ExcavationGrid,
    target_point_by_flat: Dict[int, Point],
    target_reachable_by_flat: Dict[int, bool],
    hole_origin_x: float,
    hole_origin_y: float,
    hole_origin_z: float,
    hole_size_x: float,
    hole_size_y: float,
    hole_depth: float,
    stamp: Time,
) -> MarkerArray:
    """Build target cubes (blue/red by reachability) + hole wireframe."""
    ma = MarkerArray()

    # --- Target cubes ---
    blue = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.55)
    red = ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.55)
    reachable_pts: List[Point] = []
    unreachable_pts: List[Point] = []

    for fi in grid.unexcavated_target_flat_indices():
        key = int(fi)
        pt = target_point_by_flat[key]
        if target_reachable_by_flat.get(key, True):
            reachable_pts.append(pt)
        else:
            unreachable_pts.append(pt)

    cube_scale = Vector3(
        x=grid.resolution * 0.95,
        y=grid.resolution * 0.95,
        z=grid.resolution * 0.95,
    )
    for marker_id, pts, color, ns in [
        (0, reachable_pts, blue, 'target_reachable'),
        (1, unreachable_pts, red, 'target_unreachable'),
    ]:
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = stamp
        m.ns = ns
        m.id = marker_id
        m.type = Marker.CUBE_LIST
        m.pose.orientation.w = 1.0
        m.scale = cube_scale
        m.color = color
        if pts:
            m.action = Marker.ADD
            m.points = pts
        else:
            m.action = Marker.DELETE
        ma.markers.append(m)

    # --- Hole wireframe ---
    ox, oy, oz = hole_origin_x, hole_origin_y, hole_origin_z
    sx, sy, depth = hole_size_x, hole_size_y, hole_depth

    # Top rectangle
    m = Marker()
    m.header.frame_id = 'world'
    m.header.stamp = stamp
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
        vm.header.stamp = stamp
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
    txt.header.stamp = stamp
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

    return ma


# ------------------------------------------------------------------ #
#  Excavated-cell markers (orange cubes)
# ------------------------------------------------------------------ #

def build_excavation_markers(
    grid: ExcavationGrid,
    stamp: Time,
) -> MarkerArray:
    """Build CUBE_LIST for all excavated cells (orange)."""
    ma = MarkerArray()
    marker = Marker()
    marker.header.frame_id = 'world'
    marker.header.stamp = stamp
    marker.ns = 'excavated'
    marker.id = 0
    marker.pose.orientation.w = 1.0

    nx, ny, nz = grid.shape
    points: List[Point] = []
    for ix in range(nx):
        for iy in range(ny):
            for iz in range(nz):
                if grid.is_excavated(ix, iy, iz):
                    cx, cy, cz = grid.cell_centre(ix, iy, iz)
                    points.append(Point(x=cx, y=cy, z=cz))

    if points:
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale = Vector3(
            x=grid.resolution * 0.92,
            y=grid.resolution * 0.92,
            z=grid.resolution * 0.92,
        )
        marker.color = ColorRGBA(r=0.9, g=0.5, b=0.1, a=0.8)
        marker.points = points
    else:
        marker.action = Marker.DELETEALL

    ma.markers.append(marker)
    return ma


# ------------------------------------------------------------------ #
#  Working-position arrow markers
# ------------------------------------------------------------------ #

def build_working_position_markers(
    work_positions: Sequence[BasePose],
    stamp: Time,
) -> List[Marker]:
    """Build an arrow Marker for each working position."""
    markers: List[Marker] = []
    for i, pos in enumerate(work_positions):
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = stamp
        m.ns = 'working_position'
        m.id = i
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.scale = Vector3(x=1.5, y=0.3, z=0.3)
        if i == 0:
            m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
        else:
            m.color = ColorRGBA(r=0.0, g=0.8, b=1.0, a=0.6)

        yaw = pos.yaw
        m.pose = Pose(
            position=Point(x=pos.x, y=pos.y, z=0.5),
            orientation=Quaternion(
                x=0.0, y=0.0,
                z=math.sin(yaw / 2.0),
                w=math.cos(yaw / 2.0),
            ),
        )
        markers.append(m)
    return markers
