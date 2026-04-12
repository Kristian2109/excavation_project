"""
base_planner.py – Trajectory generation for moving the robot base.

Generates a smooth trajectory from an initial pose (x, y, yaw) to a goal
pose in a free (obstacle-less) environment.  The trajectory is a sequence
of timestamped (x, y, yaw) waypoints.

This module is ROS-free and can be tested standalone.
"""

from __future__ import annotations

import math
import numpy as np
from dataclasses import dataclass
from typing import List


@dataclass
class BasePose:
    """2D pose of the robot base."""
    x: float
    y: float
    yaw: float        # radians

    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.yaw])

    @classmethod
    def from_array(cls, a: np.ndarray) -> "BasePose":
        return cls(x=float(a[0]), y=float(a[1]), yaw=float(a[2]))

    def distance_to(self, other: "BasePose") -> float:
        """Euclidean distance (position only)."""
        return math.hypot(self.x - other.x, self.y - other.y)

    def angle_distance_to(self, other: "BasePose") -> float:
        """Shortest angular distance."""
        return abs(_wrap_angle(other.yaw - self.yaw))


@dataclass
class TrajectoryPoint:
    """A single point on a base trajectory."""
    time: float       # seconds from start
    pose: BasePose


@dataclass
class BaseTrajectory:
    """Ordered list of timestamped base poses."""
    points: List[TrajectoryPoint]

    @property
    def duration(self) -> float:
        if not self.points:
            return 0.0
        return self.points[-1].time

    @property
    def start_pose(self) -> BasePose:
        return self.points[0].pose

    @property
    def end_pose(self) -> BasePose:
        return self.points[-1].pose

    def sample(self, t: float) -> BasePose:
        """Interpolate the trajectory at time *t*.

        Before the first point returns the first pose;
        after the last point returns the last pose;
        in between uses linear interpolation (x, y) and slerp-style
        angular interpolation (yaw).
        """
        if not self.points:
            raise ValueError("Empty trajectory")

        if t <= self.points[0].time:
            return self.points[0].pose
        if t >= self.points[-1].time:
            return self.points[-1].pose

        # Find the bracketing segment
        for i in range(len(self.points) - 1):
            t0 = self.points[i].time
            t1 = self.points[i + 1].time
            if t0 <= t <= t1:
                alpha = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
                p0 = self.points[i].pose
                p1 = self.points[i + 1].pose
                x = p0.x + alpha * (p1.x - p0.x)
                y = p0.y + alpha * (p1.y - p0.y)
                yaw = p0.yaw + alpha * _wrap_angle(p1.yaw - p0.yaw)
                return BasePose(x=x, y=y, yaw=yaw)

        return self.points[-1].pose

    def positions_array(self) -> np.ndarray:
        """Return Nx3 array of [x, y, yaw] for all points."""
        return np.array([[p.pose.x, p.pose.y, p.pose.yaw]
                         for p in self.points])


# ====================================================================== #
#  Helpers
# ====================================================================== #

def _wrap_angle(a: float) -> float:
    """Wrap angle to [-π, π]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


# ====================================================================== #
#  Planner
# ====================================================================== #

def plan_base_trajectory(
    start: BasePose,
    goal: BasePose,
    linear_speed: float = 0.5,       # m/s
    angular_speed: float = 0.3,      # rad/s
    dt: float = 0.1,                 # time step for waypoints
) -> BaseTrajectory:
    """Plan a three-phase base trajectory: rotate → drive → rotate.

    Phase 1: Rotate in place to face the goal.
    Phase 2: Drive in a straight line to the goal position.
    Phase 3: Rotate in place to the goal orientation.

    Parameters
    ----------
    start, goal : BasePose
    linear_speed : float – max translation speed (m/s)
    angular_speed : float – max rotation speed (rad/s)
    dt : float – time between consecutive waypoints

    Returns
    -------
    BaseTrajectory – the planned trajectory
    """
    points: List[TrajectoryPoint] = []
    t = 0.0

    # ----- Phase 1: Rotate to face goal ----- #
    dx = goal.x - start.x
    dy = goal.y - start.y
    dist = math.hypot(dx, dy)

    if dist > 1e-3:
        heading_to_goal = math.atan2(dy, dx)
    else:
        heading_to_goal = start.yaw   # no translation needed

    angle_to_rotate_1 = _wrap_angle(heading_to_goal - start.yaw)
    t_rotate_1 = abs(angle_to_rotate_1) / angular_speed if angular_speed > 0 else 0.0

    n_steps = max(1, int(math.ceil(t_rotate_1 / dt)))
    for i in range(n_steps + 1):
        alpha = i / n_steps
        yaw = start.yaw + alpha * angle_to_rotate_1
        points.append(TrajectoryPoint(
            time=t + alpha * t_rotate_1,
            pose=BasePose(x=start.x, y=start.y, yaw=yaw),
        ))
    t += t_rotate_1

    # ----- Phase 2: Drive straight to goal ----- #
    t_drive = dist / linear_speed if linear_speed > 0 else 0.0
    n_steps = max(1, int(math.ceil(t_drive / dt)))
    for i in range(1, n_steps + 1):          # skip i=0 (already added)
        alpha = i / n_steps
        x = start.x + alpha * dx
        y = start.y + alpha * dy
        points.append(TrajectoryPoint(
            time=t + alpha * t_drive,
            pose=BasePose(x=x, y=y, yaw=heading_to_goal),
        ))
    t += t_drive

    # ----- Phase 3: Rotate to goal yaw ----- #
    angle_to_rotate_2 = _wrap_angle(goal.yaw - heading_to_goal)
    t_rotate_2 = abs(angle_to_rotate_2) / angular_speed if angular_speed > 0 else 0.0
    n_steps = max(1, int(math.ceil(t_rotate_2 / dt)))
    for i in range(1, n_steps + 1):
        alpha = i / n_steps
        yaw = heading_to_goal + alpha * angle_to_rotate_2
        points.append(TrajectoryPoint(
            time=t + alpha * t_rotate_2,
            pose=BasePose(x=goal.x, y=goal.y, yaw=yaw),
        ))

    return BaseTrajectory(points=points)
