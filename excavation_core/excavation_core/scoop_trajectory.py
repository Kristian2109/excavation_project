"""
scoop_trajectory.py – Defines scoop motion as a sequence of arm configurations.

A single scoop is defined as an ordered list of named waypoints:
  1. READY    – arm in a safe configuration above the dig site
  2. APPROACH – arm positioned at the entry point of the excavation
  3. DIG      – arm at the bottom of the scoop arc
  4. SCOOP    – bucket curled to capture material
  5. LIFT     – arm lifted with the loaded bucket
  6. READY    – return to the safe configuration

Each waypoint is a set of joint positions computed by the IK solver for
a target (x, y, z) in the world frame + a desired bucket pitch angle.

This module is ROS-free and can be tested standalone.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from excavation_core.ik_solver import (
    solve_ik_nearest,
)
from excavation_core.robot_model import ExcavatorModel, JOINT_NAMES


# ====================================================================== #
#  Waypoint definition
# ====================================================================== #

@dataclass
class ScoopWaypoint:
    """A named waypoint in a scoop trajectory."""
    name: str
    joint_positions: np.ndarray   # [cabin, boom, stick, bucket]
    target_xyz: Optional[np.ndarray] = None  # world-frame target used for IK
    duration: float = 2.0         # seconds to reach this waypoint


# ====================================================================== #
#  Scoop trajectory
# ====================================================================== #

@dataclass
class ScoopTrajectory:
    """Ordered sequence of waypoints defining one complete scoop."""
    waypoints: List[ScoopWaypoint] = field(default_factory=list)
    scoop_id: int = 0

    @property
    def joint_names(self) -> List[str]:
        return list(JOINT_NAMES)

    @property
    def total_duration(self) -> float:
        return sum(wp.duration for wp in self.waypoints)

    def validate(self) -> bool:
        """Check that all waypoints have valid joint positions."""
        for wp in self.waypoints:
            model = ExcavatorModel(joint_positions=wp.joint_positions.copy())
            if not model.validate():
                return False
        return True


# ====================================================================== #
#  Named joint configurations (from SRDF)
# ====================================================================== #

READY_JOINTS = np.array([0.0, 0.3, -1.2, 0.5])
HOME_JOINTS = np.array([0.0, 0.0, 0.0, 0.0])


# ====================================================================== #
#  Scoop planner
# ====================================================================== #

def plan_single_scoop(
    dig_target_xyz: np.ndarray,
    base_x: float = 0.0,
    base_y: float = 0.0,
    base_yaw: float = 0.0,
    scoop_depth: float = 0.3,
    approach_height: float = 0.5,
    lift_height: float = 0.8,
    bucket_dig_angle: float = -math.pi / 4,
    bucket_scoop_angle: float = 0.3,
    scoop_id: int = 0,
) -> Optional[ScoopTrajectory]:
    """
    Plan a complete scoop trajectory for a single excavation action.

    Parameters
    ----------
    dig_target_xyz : (3,) array
        The target dig point in the world frame [x, y, z].
    base_x, base_y, base_yaw : float
        Current base pose (fixed during scooping).
    scoop_depth : float
        How deep below the target the bucket goes (metres).
    approach_height : float
        Height above the target for the approach waypoint.
    lift_height : float
        Height above the target for the lift waypoint.
    bucket_dig_angle : float
        Bucket pitch during dig phase (negative = tilted down).
    bucket_scoop_angle : float
        Bucket pitch during scoop/lift phase (positive = curled up).
    scoop_id : int
        ID for this scoop in the excavation sequence.

    Returns
    -------
    ScoopTrajectory or None
        The planned trajectory, or None if any waypoint is unreachable.
    """
    target = np.asarray(dig_target_xyz, dtype=float)
    ik_kwargs = dict(base_x=base_x, base_y=base_y, base_yaw=base_yaw)

    waypoints: List[ScoopWaypoint] = []

    # Helper: try IK at the preferred height, then fall back to lower values.
    # Close-range targets at shallow depths can't reach high above ground
    # because the boom joint can't rotate far enough upward.
    MIN_CLEARANCE = 0.10  # metres – absolute minimum above/below target

    def _solve_with_fallback(
        xy: np.ndarray, preferred_z: float, min_z: float,
        bucket_angle: float, steps: int = 4,
    ) -> Optional[tuple]:
        """Try IK from preferred_z down to min_z.

        Returns (IKResult, actual_target_xyz) on success, else None.
        """
        delta = (preferred_z - min_z) / max(steps - 1, 1)
        for i in range(steps):
            z = preferred_z - i * delta
            pt = np.array([xy[0], xy[1], z])
            result = solve_ik_nearest(pt, bucket_angle_world=bucket_angle,
                                      **ik_kwargs)
            if result.success:
                return result, pt
        return None

    # --- 1. READY: safe starting configuration ---
    # Set the cabin angle to point toward the target
    ready_ik = solve_ik_nearest(
        target + np.array([0, 0, lift_height + 0.5]),
        bucket_angle_world=0.0,
        **ik_kwargs,
    )
    if ready_ik.success:
        ready_joints = ready_ik.joint_positions.copy()
    else:
        # Fall back to the default ready pose with cabin aimed at target
        ready_joints = READY_JOINTS.copy()
        # Solve just for cabin angle
        dx = target[0] - base_x
        dy = target[1] - base_y
        cos_yaw = math.cos(base_yaw)
        sin_yaw = math.sin(base_yaw)
        x_base = cos_yaw * dx + sin_yaw * dy
        y_base = -sin_yaw * dx + cos_yaw * dy
        ready_joints[0] = math.atan2(y_base, x_base)

    waypoints.append(ScoopWaypoint(
        name='ready_start',
        joint_positions=ready_joints,
        duration=2.0,
    ))

    # --- 2. APPROACH: above the dig point ---
    # Try the ideal approach height first; fall back to lower heights so that
    # near-range / surface-level scoops are not rejected unnecessarily.
    approach_result = _solve_with_fallback(
        target[:2],
        preferred_z=target[2] + approach_height,
        min_z=target[2] + MIN_CLEARANCE,
        bucket_angle=bucket_dig_angle,
    )
    if approach_result is None:
        return None
    approach_ik, approach_target = approach_result
    waypoints.append(ScoopWaypoint(
        name='approach',
        joint_positions=approach_ik.joint_positions,
        target_xyz=approach_target,
        duration=2.0,
    ))

    # --- 3. DIG: at/below the target depth ---
    # Try the full scoop_depth first; accept a shallower scoop if needed.
    dig_result = _solve_with_fallback(
        target[:2],
        preferred_z=target[2] - scoop_depth,
        min_z=target[2] - MIN_CLEARANCE,
        bucket_angle=bucket_dig_angle,
    )
    if dig_result is None:
        return None
    dig_ik, dig_target = dig_result
    waypoints.append(ScoopWaypoint(
        name='dig',
        joint_positions=dig_ik.joint_positions,
        target_xyz=dig_target,
        duration=2.5,
    ))

    # --- 4. SCOOP: curl the bucket in joint space (keep boom/stick fixed) ---
    # At near-full arm extension, re-solving IK with a different bucket angle
    # often fails.  A real excavator curls the bucket while holding the arm
    # roughly in place, so we model the SCOOP as a pure joint-space motion:
    #   copy the dig joint positions and increase the bucket joint angle.
    BUCKET_CURL_DELTA = 0.5   # radians to curl the bucket
    BUCKET_JOINT_MAX = 2.2    # upper limit from URDF
    scoop_joints = dig_ik.joint_positions.copy()
    scoop_joints[3] = min(scoop_joints[3] + BUCKET_CURL_DELTA, BUCKET_JOINT_MAX)
    waypoints.append(ScoopWaypoint(
        name='scoop',
        joint_positions=scoop_joints,
        target_xyz=dig_target.copy(),
        duration=1.5,
    ))

    # --- 5. LIFT: raise the loaded bucket ---
    # Try the ideal lift height first; fall back to lower heights.
    lift_result = _solve_with_fallback(
        target[:2],
        preferred_z=target[2] + lift_height,
        min_z=target[2] + MIN_CLEARANCE,
        bucket_angle=bucket_dig_angle,
    )
    if lift_result is None:
        return None
    lift_ik, lift_target = lift_result
    waypoints.append(ScoopWaypoint(
        name='lift',
        joint_positions=lift_ik.joint_positions,
        target_xyz=lift_target,
        duration=2.0,
    ))

    # --- 6. READY: return to safe position ---
    waypoints.append(ScoopWaypoint(
        name='ready_end',
        joint_positions=ready_joints.copy(),
        duration=2.0,
    ))

    traj = ScoopTrajectory(waypoints=waypoints, scoop_id=scoop_id)
    return traj if traj.validate() else None
