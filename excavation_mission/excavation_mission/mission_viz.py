"""
mission_viz.py – Visualization helpers for the mission controller node.

Pure builder functions that construct ROS marker messages.  Keeping them
separate from the node makes the node class shorter and the marker logic
independently testable.
"""

from __future__ import annotations

from typing import List, Sequence

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Time

from excavation_core.excavation_planner import PlannedScoop
from excavation_core.scoop_trajectory import ScoopTrajectory
from excavation_core.robot_model import ExcavatorModel


# ------------------------------------------------------------------ #
#  Scoop target markers (sphere list)
# ------------------------------------------------------------------ #

def build_scoop_target_markers(
    scoops: Sequence[PlannedScoop],
    stamp: Time,
) -> MarkerArray:
    """Create a SPHERE_LIST showing all planned dig targets."""
    ma = MarkerArray()
    m = Marker()
    m.header.frame_id = 'world'
    m.header.stamp = stamp
    m.ns = 'scoop_targets'
    m.id = 0
    m.type = Marker.SPHERE_LIST
    m.action = Marker.ADD
    m.scale = Vector3(x=0.15, y=0.15, z=0.15)
    m.color = ColorRGBA(r=0.0, g=1.0, b=0.5, a=0.6)
    m.pose.orientation.w = 1.0

    for s in scoops:
        t = s.dig_target
        m.points.append(Point(
            x=float(t[0]), y=float(t[1]), z=float(t[2])))

    ma.markers.append(m)
    return ma


# ------------------------------------------------------------------ #
#  Arm trajectory markers (line strip + waypoint spheres)
# ------------------------------------------------------------------ #

def build_arm_trajectory_markers(
    traj: ScoopTrajectory,
    base_x: float,
    base_y: float,
    base_yaw: float,
    stamp: Time,
    lifetime_sec: int,
) -> MarkerArray:
    """Create a LINE_STRIP + per-waypoint spheres for *traj*."""
    ma = MarkerArray()

    # FK → bucket tip positions
    points: List[Point] = []
    for wp in traj.waypoints:
        model = ExcavatorModel(
            joint_positions=wp.joint_positions.copy(),
            base_x=base_x, base_y=base_y, base_yaw=base_yaw,
        )
        tip = model.bucket_tip_position()
        points.append(Point(
            x=float(tip[0]), y=float(tip[1]), z=float(tip[2])))

    # Line strip
    m = Marker()
    m.header.frame_id = 'world'
    m.header.stamp = stamp
    m.ns = 'arm_trajectory'
    m.id = 0
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.scale.x = 0.06
    m.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.9)
    m.pose.orientation.w = 1.0
    m.points = points
    m.lifetime.sec = lifetime_sec
    ma.markers.append(m)

    # Per-waypoint spheres
    for i, pt in enumerate(points):
        sm = Marker()
        sm.header.frame_id = 'world'
        sm.header.stamp = stamp
        sm.ns = 'arm_waypoints'
        sm.id = i
        sm.type = Marker.SPHERE
        sm.action = Marker.ADD
        sm.scale = Vector3(x=0.12, y=0.12, z=0.12)
        sm.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.8)
        sm.pose.position = pt
        sm.pose.orientation.w = 1.0
        sm.lifetime.sec = lifetime_sec
        ma.markers.append(sm)

    return ma
