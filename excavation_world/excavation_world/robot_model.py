"""
robot_model.py – Kinematic model of the excavator robot.

The kinematic chain is:
    world → base_link → cabin_link → boom_link → stick_link → bucket_link → bucket_tip

Actuated joints (from the URDF):
    cabin_joint  – continuous, axis Z  (swing)
    boom_joint   – revolute,  axis Y  (limits: -0.3 … 1.2)
    stick_joint  – revolute,  axis Y  (limits: -2.4 … 0.0)
    bucket_joint – revolute,  axis Y  (limits: -1.0 … 2.2)

This module is ROS-free and can be tested standalone.
"""

from __future__ import annotations

import math
import numpy as np
from dataclasses import dataclass, field
from typing import Dict


# ====================================================================== #
#  Joint definition
# ====================================================================== #

@dataclass
class JointDef:
    """Definition of a single revolute / continuous joint."""
    name: str
    axis: np.ndarray              # unit vector in parent frame
    origin_xyz: np.ndarray        # translation from parent to child
    origin_rpy: np.ndarray        # fixed rotation (roll, pitch, yaw)
    lower: float = -math.inf      # continuous → ±inf
    upper: float = math.inf
    is_continuous: bool = False

    def in_limits(self, q: float) -> bool:
        if self.is_continuous:
            return True
        return self.lower <= q <= self.upper


# ====================================================================== #
#  Homogeneous transform helpers
# ====================================================================== #

def _rotation_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([
        [1, 0,  0, 0],
        [0, c, -s, 0],
        [0, s,  c, 0],
        [0, 0,  0, 1],
    ])


def _rotation_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([
        [ c, 0, s, 0],
        [ 0, 1, 0, 0],
        [-s, 0, c, 0],
        [ 0, 0, 0, 1],
    ])


def _rotation_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([
        [c, -s, 0, 0],
        [s,  c, 0, 0],
        [0,  0, 1, 0],
        [0,  0, 0, 1],
    ])


def _translation(x: float, y: float, z: float) -> np.ndarray:
    T = np.eye(4)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """RPY → 4×4 homogeneous rotation (ZYX convention)."""
    return _rotation_z(yaw) @ _rotation_y(pitch) @ _rotation_x(roll)


def rotation_about_axis(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues rotation about an arbitrary unit axis → 4×4."""
    ax = axis / np.linalg.norm(axis)
    if np.allclose(ax, [1, 0, 0]):
        return _rotation_x(angle)
    if np.allclose(ax, [0, 1, 0]):
        return _rotation_y(angle)
    if np.allclose(ax, [0, 0, 1]):
        return _rotation_z(angle)
    # General Rodrigues
    c, s = math.cos(angle), math.sin(angle)
    t = 1 - c
    x, y, z = ax
    R = np.array([
        [t*x*x + c,    t*x*y - s*z,  t*x*z + s*y, 0],
        [t*x*y + s*z,  t*y*y + c,    t*y*z - s*x, 0],
        [t*x*z - s*y,  t*y*z + s*x,  t*z*z + c,   0],
        [0,            0,            0,            1],
    ])
    return R


# ====================================================================== #
#  Excavator kinematic model
# ====================================================================== #

# Geometric constants from the URDF (keep in sync!)
CHASSIS_HEIGHT = 0.5
CABIN_LENGTH   = 1.0
CABIN_HEIGHT   = 0.8
BOOM_LENGTH    = 3.0
STICK_LENGTH   = 2.5
BUCKET_LENGTH  = 0.8
BUCKET_DEPTH   = 0.5


# Joint order – these names match the URDF joint names exactly
JOINT_NAMES = ['cabin_joint', 'boom_joint', 'stick_joint', 'bucket_joint']


def _build_joint_defs() -> Dict[str, JointDef]:
    """Create the joint definitions matching the URDF."""
    return {
        'cabin_joint': JointDef(
            name='cabin_joint',
            axis=np.array([0.0, 0.0, 1.0]),
            origin_xyz=np.array([0.0, 0.0, CHASSIS_HEIGHT]),
            origin_rpy=np.array([0.0, 0.0, 0.0]),
            is_continuous=True,
        ),
        'boom_joint': JointDef(
            name='boom_joint',
            axis=np.array([0.0, 1.0, 0.0]),
            origin_xyz=np.array([CABIN_LENGTH, 0.0, CABIN_HEIGHT * 0.8]),
            origin_rpy=np.array([0.0, 0.0, 0.0]),
            lower=-0.3, upper=1.2,
        ),
        'stick_joint': JointDef(
            name='stick_joint',
            axis=np.array([0.0, 1.0, 0.0]),
            origin_xyz=np.array([BOOM_LENGTH, 0.0, 0.0]),
            origin_rpy=np.array([0.0, 0.0, 0.0]),
            lower=-2.4, upper=0.0,
        ),
        'bucket_joint': JointDef(
            name='bucket_joint',
            axis=np.array([0.0, 1.0, 0.0]),
            origin_xyz=np.array([STICK_LENGTH, 0.0, 0.0]),
            origin_rpy=np.array([0.0, 0.0, 0.0]),
            lower=-1.0, upper=2.2,
        ),
    }


# Fixed transform from bucket_link to bucket_tip
_BUCKET_TIP_OFFSET = np.array([BUCKET_LENGTH, 0.0, -BUCKET_DEPTH])


@dataclass
class ExcavatorModel:
    """Kinematic model of the excavator.

    The base pose (x, y, yaw) is stored separately and represents the
    position of base_link in the world frame.  The arm joints are
    cabin_joint, boom_joint, stick_joint, bucket_joint.
    """

    # Base pose in world frame
    base_x: float = 0.0
    base_y: float = 0.0
    base_yaw: float = 0.0

    # Joint state (order: cabin, boom, stick, bucket)
    joint_positions: np.ndarray = field(
        default_factory=lambda: np.zeros(4))

    # Joint definitions (built once)
    _joint_defs: Dict[str, JointDef] = field(
        init=False, repr=False, default_factory=_build_joint_defs)

    # ------------------------------------------------------------------ #
    #  Joint access
    # ------------------------------------------------------------------ #
    @property
    def joint_defs(self) -> Dict[str, JointDef]:
        return self._joint_defs

    def set_joint(self, name: str, value: float) -> None:
        idx = JOINT_NAMES.index(name)
        self.joint_positions[idx] = value

    def get_joint(self, name: str) -> float:
        idx = JOINT_NAMES.index(name)
        return float(self.joint_positions[idx])

    # ------------------------------------------------------------------ #
    #  Validation
    # ------------------------------------------------------------------ #
    def validate(self) -> bool:
        """Return True if all joint positions are within limits."""
        for i, name in enumerate(JOINT_NAMES):
            if not self._joint_defs[name].in_limits(self.joint_positions[i]):
                return False
        return True

    def clamp_to_limits(self) -> None:
        """Clamp each joint to its allowed range."""
        for i, name in enumerate(JOINT_NAMES):
            jd = self._joint_defs[name]
            if not jd.is_continuous:
                self.joint_positions[i] = np.clip(
                    self.joint_positions[i], jd.lower, jd.upper)

    # ------------------------------------------------------------------ #
    #  Forward kinematics
    # ------------------------------------------------------------------ #
    def _base_transform(self) -> np.ndarray:
        """4×4 world → base_link."""
        T = _translation(self.base_x, self.base_y, 0.0)
        T = T @ _rotation_z(self.base_yaw)
        return T

    def _joint_transform(self, name: str, q: float) -> np.ndarray:
        """4×4 parent → child for joint *name* at angle *q*."""
        jd = self._joint_defs[name]
        T = _translation(*jd.origin_xyz)
        T = T @ rpy_to_matrix(*jd.origin_rpy)
        T = T @ rotation_about_axis(jd.axis, q)
        return T

    def fk_chain(self) -> Dict[str, np.ndarray]:
        """Compute forward kinematics for every link.

        Returns a dict mapping link name → 4×4 homogeneous transform
        in the world frame.
        """
        frames: Dict[str, np.ndarray] = {}

        T = self._base_transform()
        frames['base_link'] = T.copy()

        # cabin
        T = T @ self._joint_transform('cabin_joint', self.joint_positions[0])
        frames['cabin_link'] = T.copy()

        # boom
        T = T @ self._joint_transform('boom_joint', self.joint_positions[1])
        frames['boom_link'] = T.copy()

        # stick
        T = T @ self._joint_transform('stick_joint', self.joint_positions[2])
        frames['stick_link'] = T.copy()

        # bucket
        T = T @ self._joint_transform('bucket_joint', self.joint_positions[3])
        frames['bucket_link'] = T.copy()

        # bucket tip (fixed offset)
        T_tip = T @ _translation(*_BUCKET_TIP_OFFSET)
        frames['bucket_tip'] = T_tip.copy()

        return frames

    def bucket_tip_position(self) -> np.ndarray:
        """Return the (x, y, z) world position of the bucket tip."""
        frames = self.fk_chain()
        return frames['bucket_tip'][:3, 3]

    def bucket_tip_pose(self) -> np.ndarray:
        """Return the full 4×4 world-frame transform of the bucket tip."""
        return self.fk_chain()['bucket_tip']

    # ------------------------------------------------------------------ #
    #  Convenience
    # ------------------------------------------------------------------ #
    def __str__(self) -> str:
        pos = self.bucket_tip_position()
        return (
            f"Excavator(base=({self.base_x:.2f}, {self.base_y:.2f}, "
            f"yaw={math.degrees(self.base_yaw):.1f}°), "
            f"joints={np.round(self.joint_positions, 3).tolist()}, "
            f"tip=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}))"
        )
