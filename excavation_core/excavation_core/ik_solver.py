"""
ik_solver.py – Analytical inverse kinematics for the excavator arm.

Joint chain: cabin (Z-axis swing) → boom (Y) → stick (Y) → bucket (Y)

Sign convention: positive Y-rotation tilts the arm DOWNWARD.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

import numpy as np

from excavation_core.robot_model import (
    JOINT_NAMES,
    CHASSIS_HEIGHT,
    CABIN_LENGTH,
    CABIN_HEIGHT,
    BOOM_LENGTH,
    STICK_LENGTH,
    BUCKET_LENGTH,
    BUCKET_DEPTH,
    _build_joint_defs,
)


# Geometry constants
SHOULDER_X_BASE_FRAME = CABIN_LENGTH
SHOULDER_Z_BASE_FRAME = CHASSIS_HEIGHT + CABIN_HEIGHT * 0.8
L1 = BOOM_LENGTH   # 3.5 m
L2 = STICK_LENGTH  # 2.9 m
MAX_CABIN_SWING = math.radians(120)


class IKStatus(Enum):
    SUCCESS = auto()
    OUT_OF_REACH = auto()
    JOINT_LIMITS_VIOLATED = auto()


@dataclass
class IKResult:
    status: IKStatus
    joint_positions: Optional[np.ndarray] = None
    message: str = ''

    @property
    def success(self) -> bool:
        return self.status == IKStatus.SUCCESS


def _world_to_base_frame(
    target: np.ndarray, base_x: float, base_y: float, base_yaw: float,
) -> tuple[float, float, float]:
    """Transform world-frame target into base-local (x, y, z)."""
    cos_yaw = math.cos(base_yaw)
    sin_yaw = math.sin(base_yaw)
    dx = target[0] - base_x
    dy = target[1] - base_y
    x_local = cos_yaw * dx + sin_yaw * dy
    y_local = -sin_yaw * dx + cos_yaw * dy
    return x_local, y_local, target[2]


def _bucket_tip_to_wrist(
    r_target: float, z_target: float, bucket_angle_world: float,
) -> tuple[float, float]:
    """Back-compute wrist position from the bucket tip in the swing plane.

    FK uses R_y where positive = down, so psi = -bucket_angle_world.
    """
    psi = -bucket_angle_world
    r_offset = BUCKET_LENGTH * math.cos(psi) - BUCKET_DEPTH * math.sin(psi)
    z_offset = -BUCKET_LENGTH * math.sin(psi) - BUCKET_DEPTH * math.cos(psi)
    return r_target - r_offset, z_target - z_offset


def _solve_two_link(
    horizontal_reach: float, vertical_drop: float, elbow_up: bool,
) -> Optional[tuple[float, float]]:
    """Solve 2-link planar IK for boom and stick angles, like a normal 2D manipulator arm.

    Works in the vertical swing plane.  The shoulder is the origin; positive
    vertical_drop means the wrist is BELOW the shoulder.

    Returns (boom_angle, stick_angle) or None if the wrist is unreachable.
    """
    # Straight-line distance from shoulder to wrist
    shoulder_to_wrist_sq = horizontal_reach**2 + vertical_drop**2
    shoulder_to_wrist = math.sqrt(shoulder_to_wrist_sq)

    if shoulder_to_wrist > L1 + L2 or shoulder_to_wrist < abs(L1 - L2):
        return None

    # Law of cosines: find the interior angle (q2) at the elbow joint
    cos_stick_angle = float(np.clip(
        (shoulder_to_wrist_sq - L1**2 - L2**2) / (2 * L1 * L2), -1.0, 1.0,
    ))
    stick_angle = math.acos(cos_stick_angle) if elbow_up else -math.acos(cos_stick_angle)

    # Boom angle = angle of shoulder→wrist line from horizontal
    #            − angle between boom and that line (from triangle geometry)
    angle_of_shoulder_to_wrist_line = math.atan2(vertical_drop, horizontal_reach)
    angle_boom_to_wrist_line = math.atan2(
        L2 * math.sin(stick_angle),
        L1 + L2 * math.cos(stick_angle),
    )
    boom_angle = angle_of_shoulder_to_wrist_line - angle_boom_to_wrist_line

    return boom_angle, stick_angle


def _check_joint_limits(joints: np.ndarray) -> Optional[str]:
    """Return violation message if any joint is out of limits, else None."""
    jdefs = _build_joint_defs()
    violations = [
        f'{name}: {joints[i]:.3f} rad outside [{jdefs[name].lower:.2f}, {jdefs[name].upper:.2f}]'
        for i, name in enumerate(JOINT_NAMES)
        if not jdefs[name].in_limits(joints[i])
    ]
    return '; '.join(violations) if violations else None


def solve_ik(
    target_xyz: np.ndarray,
    x_base_world_frame: float = 0.0,
    y_base_world_frame: float = 0.0,
    yaw_base_world_frame: float = 0.0,
    bucket_angle_world: float = -math.pi / 4,
    elbow_up: bool = True,
) -> IKResult:
    """Compute joint angles to place the bucket tip at *target_xyz*.

    Returns IKResult with status, joint_positions [cabin, boom, stick, bucket],
    and a diagnostic message.
    """
    target = np.asarray(target_xyz, dtype=float)

    x_base_base_frame, y_base_base_frame, z_base_base_frame = _world_to_base_frame(target, x_base_world_frame, y_base_world_frame, yaw_base_world_frame)

    cabin_rotation_angle = math.atan2(y_base_base_frame, x_base_base_frame)
    if abs(cabin_rotation_angle) > MAX_CABIN_SWING:
        return IKResult(status=IKStatus.OUT_OF_REACH, message=(
            f'Target behind robot: cabin swing {math.degrees(cabin_rotation_angle):.1f}° '
            f'exceeds ±{math.degrees(MAX_CABIN_SWING):.0f}° limit'))

    distance_base_to_target = math.hypot(x_base_base_frame, y_base_base_frame)
    
    distance_base_to_wrist, vertical_distance_base_to_wrist = _bucket_tip_to_wrist(distance_base_to_target, z_base_base_frame, bucket_angle_world)
    
    distance_shoulder_to_wrist = distance_base_to_wrist - SHOULDER_X_BASE_FRAME
    vertical_distance_shoulder_to_wrist = SHOULDER_Z_BASE_FRAME - vertical_distance_base_to_wrist

    two_link = _solve_two_link(distance_shoulder_to_wrist, vertical_distance_shoulder_to_wrist, elbow_up=elbow_up)
    if two_link is None:
        d = math.hypot(distance_shoulder_to_wrist, vertical_distance_shoulder_to_wrist)
        return IKResult(status=IKStatus.OUT_OF_REACH, message=(
            f'Wrist distance {d:.3f} m outside [{abs(L1-L2):.3f}, {L1+L2:.3f}] m range'))

    boom_angle, stick_angle = two_link

    bucket_angle = -bucket_angle_world - boom_angle - stick_angle

    joints = np.array([cabin_rotation_angle, boom_angle, stick_angle, bucket_angle])
    violation_msg = _check_joint_limits(joints)
    if violation_msg:
        return IKResult(
            status=IKStatus.JOINT_LIMITS_VIOLATED,
            joint_positions=joints,
            message=f'Joint limit violations: {violation_msg}',
        )

    return IKResult(status=IKStatus.SUCCESS, joint_positions=joints)


def solve_ik_nearest(
    target_xyz: np.ndarray,
    x_base_world_frame: float = 0.0,
    y_base_world_frame: float = 0.0,
    yaw_base_world_frame: float = 0.0,
    bucket_angle_world: float = -math.pi / 4,
) -> IKResult:
    """Try both elbow configs and sweep bucket angles to find a valid solution.

    Prefers elbow-up.
    """
    def _try(angle: float) -> Optional[IKResult]:
        r = solve_ik(target_xyz, x_base_world_frame, y_base_world_frame, yaw_base_world_frame, angle, elbow_up=True)
        if r.success:
            return r

        r = solve_ik(target_xyz, x_base_world_frame, y_base_world_frame, yaw_base_world_frame, angle, elbow_up=False)
        if r.success:            
            return r

        return None 

    # Exact requested angle
    if (r := _try(bucket_angle_world)):
        return r

    # Sweep from requested angle toward 0 (more horizontal)
    for i in range(1, 16):
        angle = bucket_angle_world * (1.0 - i / 15)
        if (r := _try(angle)):
            return r

    # Slight positive angles (bucket tilted up)
    for angle in (0.1, 0.2, 0.3, 0.5):
        if (r := _try(angle)):
            return r

    # Return last failure
    return solve_ik(target_xyz, x_base_world_frame, y_base_world_frame, yaw_base_world_frame, bucket_angle_world, elbow_up=True)
