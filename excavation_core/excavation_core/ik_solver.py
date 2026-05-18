"""
ik_solver.py – Analytical inverse kinematics for the excavator arm.

Solves for (cabin_joint, boom_joint, stick_joint, bucket_joint) given a
desired bucket-tip position in the world frame.

The arm has a simple planar geometry:
  cabin_joint (Z-axis)  → selects the swing plane
  boom_joint  (Y-axis)  → first link of a 2-link planar arm
  stick_joint (Y-axis)  → second link of a 2-link planar arm
  bucket_joint(Y-axis)  → orients the bucket (resolved last)

IMPORTANT – FK sign convention:
  R_y(+θ) rotates a vector's X-component toward -Z.  Therefore
  **positive joint angles tilt the arm DOWNWARD** in the swing plane.
  boom_joint=+1.0 → boom points ~57° below horizontal.

This module is ROS-free and can be tested standalone.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

import numpy as np

from excavation_core.robot_model import (
    ExcavatorModel,
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


# ====================================================================== #
#  Constants derived from URDF geometry
# ====================================================================== #

# Shoulder position in base_link frame (where the boom pivots)
SHOULDER_X_LOCAL = CABIN_LENGTH           # 1.0 m forward
SHOULDER_Z_LOCAL = CHASSIS_HEIGHT + CABIN_HEIGHT * 0.8  # 0.5 + 0.64 = 1.14 m up

# Arm link lengths used in 2-link IK
L1 = BOOM_LENGTH    # 3.5 m  (boom)
L2 = STICK_LENGTH   # 2.9 m  (stick)


# ====================================================================== #
#  IK result
# ====================================================================== #

class IKStatus(Enum):
    SUCCESS = auto()
    OUT_OF_REACH = auto()
    JOINT_LIMITS_VIOLATED = auto()


@dataclass
class IKResult:
    """Result of an IK query."""
    status: IKStatus
    joint_positions: Optional[np.ndarray] = None  # [cabin, boom, stick, bucket]
    message: str = ''

    @property
    def success(self) -> bool:
        return self.status == IKStatus.SUCCESS


# ====================================================================== #
#  Analytical IK
# ====================================================================== #

def solve_ik(
    target_xyz: np.ndarray,
    base_x: float = 0.0,
    base_y: float = 0.0,
    base_yaw: float = 0.0,
    bucket_angle_world: float = -math.pi / 4,
    elbow_up: bool = False,
) -> IKResult:
    """
    Compute joint angles to place the bucket tip at *target_xyz*.

    Parameters
    ----------
    target_xyz : (3,) array
        Desired bucket-tip position in the world frame [x, y, z].
    base_x, base_y, base_yaw : float
        Current base pose (world frame).  The base does not move during IK.
    bucket_angle_world : float
        Desired pitch of the bucket link in the swing plane, using the
        standard math convention (negative = downward from horizontal).
        Default -π/4 ≈ -45° which is a good general digging angle.
        Note: -π/2 (straight down) often violates the bucket joint limit.
    elbow_up : bool
        If True, prefer the elbow-up (stick_joint > 0) solution.
        Excavators normally use elbow-down (stick_joint < 0).

    Returns
    -------
    IKResult
        Contains status, joint_positions [cabin, boom, stick, bucket],
        and a human-readable message.
    """
    target = np.asarray(target_xyz, dtype=float)
    jdefs = _build_joint_defs()

    # ------------------------------------------------------------------
    # 1. Transform target into base_link frame
    # ------------------------------------------------------------------
    cos_yaw = math.cos(base_yaw)
    sin_yaw = math.sin(base_yaw)
    dx = target[0] - base_x
    dy = target[1] - base_y
    dz = target[2]

    # Rotate into base_link (undo base yaw)
    x_base = cos_yaw * dx + sin_yaw * dy
    y_base = -sin_yaw * dx + cos_yaw * dy
    z_base = dz

    # ------------------------------------------------------------------
    # 2. Solve cabin_joint (swing about Z)
    # ------------------------------------------------------------------
    cabin_angle = math.atan2(y_base, x_base)

    # Reject targets that require the cabin to swing beyond ±120°.
    # Real excavators do not reach directly behind themselves.
    MAX_CABIN_SWING = math.radians(120)
    if abs(cabin_angle) > MAX_CABIN_SWING:
        return IKResult(
            status=IKStatus.OUT_OF_REACH,
            message=f'Target behind robot: cabin swing {math.degrees(cabin_angle):.1f}° '
                    f'exceeds ±{math.degrees(MAX_CABIN_SWING):.0f}° limit',
        )

    # ------------------------------------------------------------------
    # 3. Swing-plane coordinates
    # ------------------------------------------------------------------
    # After the cabin swings, the entire arm lies in a vertical plane.
    #   r = horizontal distance from the base Z-axis
    #   z = height (upward positive)
    r_target = math.sqrt(x_base**2 + y_base**2)
    z_target = z_base

    # ------------------------------------------------------------------
    # 4. Back-compute wrist (bucket_joint origin) from bucket tip
    # ------------------------------------------------------------------
    # In the FK, the total Y-rotation at the bucket link is:
    #   psi = boom + stick + bucket   (FK joint values)
    # R_y(psi) applied to the bucket-tip offset (BL, 0, -BD) gives:
    #   r_offset = BL*cos(psi) - BD*sin(psi)
    #   z_offset = -BL*sin(psi) - BD*cos(psi)
    #
    # bucket_angle_world uses standard math convention (negative = down),
    # but FK uses R_y where positive = down.  So: psi = -bucket_angle_world.
    psi = -bucket_angle_world

    r_offset = BUCKET_LENGTH * math.cos(psi) - BUCKET_DEPTH * math.sin(psi)
    z_offset = -BUCKET_LENGTH * math.sin(psi) - BUCKET_DEPTH * math.cos(psi)

    r_wrist = r_target - r_offset
    z_wrist = z_target - z_offset

    # ------------------------------------------------------------------
    # 5. Two-link IK for boom + stick
    # ------------------------------------------------------------------
    # The FK convention: positive boom_joint tilts the arm DOWNWARD.
    # We solve in the (dr, dz_down) plane where:
    #   dr      = horizontal distance from shoulder to wrist
    #   dz_down = how far the wrist is BELOW the shoulder (positive = below)
    # This makes the 2-link IK angles directly match the FK joint values.
    dr = r_wrist - SHOULDER_X_LOCAL
    dz_down = SHOULDER_Z_LOCAL - z_wrist   # positive when wrist below shoulder

    d_sq = dr**2 + dz_down**2
    d = math.sqrt(d_sq)

    # Reachability check
    if d > L1 + L2:
        return IKResult(
            status=IKStatus.OUT_OF_REACH,
            message=f'Target too far: distance to wrist = {d:.3f} m, '
                    f'max reach = {L1 + L2:.3f} m',
        )
    if d < abs(L1 - L2):
        return IKResult(
            status=IKStatus.OUT_OF_REACH,
            message=f'Target too close: distance = {d:.3f} m, '
                    f'min reach = {abs(L1 - L2):.3f} m',
        )

    # Law of cosines for stick (elbow) angle
    cos_elbow = (d_sq - L1**2 - L2**2) / (2 * L1 * L2)
    cos_elbow = float(np.clip(cos_elbow, -1.0, 1.0))

    if elbow_up:
        stick_angle = math.acos(cos_elbow)    # positive (unusual for excavator)
    else:
        stick_angle = -math.acos(cos_elbow)   # negative (normal excavator posture)

    # Angle from shoulder to wrist line (in the dz_down-positive plane)
    alpha = math.atan2(dz_down, dr)

    # Offset due to the elbow triangle
    beta = math.atan2(
        L2 * math.sin(stick_angle),
        L1 + L2 * math.cos(stick_angle),
    )

    boom_angle = alpha - beta

    # ------------------------------------------------------------------
    # 6. Solve bucket_joint
    # ------------------------------------------------------------------
    # Total FK rotation at bucket: psi = boom + stick + bucket
    bucket_angle = psi - boom_angle - stick_angle

    # ------------------------------------------------------------------
    # 7. Check joint limits
    # ------------------------------------------------------------------
    joints = np.array([cabin_angle, boom_angle, stick_angle, bucket_angle])

    violations = []
    for i, name in enumerate(JOINT_NAMES):
        jd = jdefs[name]
        if not jd.in_limits(joints[i]):
            violations.append(
                f'{name}: {joints[i]:.3f} rad outside [{jd.lower:.2f}, {jd.upper:.2f}]'
            )

    if violations:
        return IKResult(
            status=IKStatus.JOINT_LIMITS_VIOLATED,
            joint_positions=joints,
            message='Joint limit violations: ' + '; '.join(violations),
        )

    return IKResult(
        status=IKStatus.SUCCESS,
        joint_positions=joints,
        message=f'IK solved: cabin={math.degrees(cabin_angle):.1f}°, '
                f'boom={math.degrees(boom_angle):.1f}°, '
                f'stick={math.degrees(stick_angle):.1f}°, '
                f'bucket={math.degrees(bucket_angle):.1f}°',
    )


def solve_ik_nearest(
    target_xyz: np.ndarray,
    base_x: float = 0.0,
    base_y: float = 0.0,
    base_yaw: float = 0.0,
    bucket_angle_world: float = -math.pi / 4,
) -> IKResult:
    """Try both elbow configs and sweep bucket angles if needed.

    1. Try the requested bucket_angle_world with elbow-down then elbow-up.
    2. If that fails (joint limits), sweep bucket angles from the requested
       value toward 0 (more horizontal).  This handles cases where the
       bucket joint limit prevents a steep digging angle.

    Prefers elbow-down (natural excavator posture).
    """
    # --- First: try the exact requested angle ---
    for elbow_up in (False, True):
        result = solve_ik(
            target_xyz, base_x, base_y, base_yaw,
            bucket_angle_world, elbow_up=elbow_up,
        )
        if result.success:
            return result

    # --- Second: sweep bucket angles from requested toward 0 ---
    best_result = result
    n_steps = 15
    for i in range(1, n_steps + 1):
        t = i / n_steps
        angle = bucket_angle_world * (1.0 - t)   # lerp toward 0
        for elbow_up in (False, True):
            result = solve_ik(
                target_xyz, base_x, base_y, base_yaw,
                angle, elbow_up=elbow_up,
            )
            if result.success:
                return result

    # --- Third: try slight positive angles (bucket tilted up) ---
    for angle in (0.1, 0.2, 0.3, 0.5):
        for elbow_up in (False, True):
            result = solve_ik(
                target_xyz, base_x, base_y, base_yaw,
                angle, elbow_up=elbow_up,
            )
            if result.success:
                return result

    return best_result


def verify_ik_solution(
    target_xyz: np.ndarray,
    ik_result: IKResult,
    base_x: float = 0.0,
    base_y: float = 0.0,
    base_yaw: float = 0.0,
    tolerance: float = 0.05,
) -> bool:
    """Verify an IK solution using forward kinematics.

    Returns True if the FK tip position matches target_xyz within
    *tolerance* metres.
    """
    if ik_result.joint_positions is None:
        return False

    model = ExcavatorModel(
        base_x=base_x, base_y=base_y, base_yaw=base_yaw,
        joint_positions=ik_result.joint_positions.copy(),
    )
    tip = model.bucket_tip_position()
    error = np.linalg.norm(tip - np.asarray(target_xyz))
    return float(error) < tolerance
