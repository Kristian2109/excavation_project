"""
Tests for ik_solver.py – Stage 4: Working Arm Kinematics.

Covers:
  - IK for reachable points (elbow-down, elbow-up)
  - FK round-trip verification (IK → FK matches target)
  - Unreachable targets (too far, too close)
  - Joint-limit violations
  - Reachability checking helper
  - Non-zero base pose
  - Multiple targets inside the excavation area
  - Bucket angle variations
  - solve_ik_nearest fallback (bucket angle sweep)
"""

import math
import numpy as np
import pytest

from excavation_core.ik_solver import (
    IKResult,
    IKStatus,
    solve_ik,
    solve_ik_nearest,
    SHOULDER_X_BASE_FRAME,
    SHOULDER_Z_BASE_FRAME,
)
from excavation_core.robot_model import ExcavatorModel


# ====================================================================== #
#  Helpers
# ====================================================================== #

def _fk_tip(joints: np.ndarray,
            base_x: float = 0.0,
            base_y: float = 0.0,
            base_yaw: float = 0.0) -> np.ndarray:
    """Run FK and return the (x, y, z) bucket-tip position."""
    model = ExcavatorModel(
        base_x=base_x, base_y=base_y, base_yaw=base_yaw,
        joint_positions=joints.copy(),
    )
    return model.bucket_tip_position()


def _assert_ik_fk_roundtrip(target, result: IKResult,
                             base_x=0.0, base_y=0.0, base_yaw=0.0,
                             tol=0.05):
    """Assert that an IK result, when fed back through FK, hits the target."""
    assert result.success, f'IK failed: {result.message}'
    tip = _fk_tip(result.joint_positions, base_x, base_y, base_yaw)
    error = np.linalg.norm(tip - np.asarray(target))
    assert error < tol, (
        f'FK round-trip error {error:.4f} m > {tol} m. '
        f'Target={target}, FK tip={tip}, joints={result.joint_positions}'
    )


def _assert_within_limits(result: IKResult):
    """Assert that all joint angles are within limits."""
    assert result.joint_positions is not None
    model = ExcavatorModel(joint_positions=result.joint_positions.copy())
    assert model.validate(), (
        f'Joint limits violated: {result.joint_positions}'
    )


# ====================================================================== #
#  Basic IK on the centreline (y=0, base at origin)
# ====================================================================== #

def test_straight_ahead_near_ground():
    """Target a few metres in front near ground level."""
    target = np.array([4.0, 0.0, 0.5])
    result = solve_ik_nearest(target)
    _assert_ik_fk_roundtrip(target, result)
    _assert_within_limits(result)


def test_straight_ahead_at_ground():
    """Target right at ground level."""
    target = np.array([4.0, 0.0, 0.0])
    result = solve_ik_nearest(target)
    _assert_ik_fk_roundtrip(target, result)


def test_moderate_reach():
    """Target at moderate distance and height."""
    target = np.array([5.0, 0.0, 0.0])
    result = solve_ik_nearest(target)
    _assert_ik_fk_roundtrip(target, result)


def test_far_reach_low():
    """Target far out and slightly below shoulder."""
    target = np.array([5.5, 0.0, -0.5])
    result = solve_ik_nearest(target)
    _assert_ik_fk_roundtrip(target, result)


def test_above_shoulder():
    """Target slightly above the shoulder — arm reaches up a bit."""
    # boom_joint limit is -0.3, so the arm can only tilt up slightly.
    # (3.0, 0, 2.5) is too high; use a target within reach.
    target = np.array([4.0, 0.0, 1.5])
    result = solve_ik_nearest(target)
    _assert_ik_fk_roundtrip(target, result)


# ====================================================================== #
#  Elbow-up vs elbow-down
# ====================================================================== #

def test_elbow_down_default():
    target = np.array([4.0, 0.0, 0.0])
    result = solve_ik(target, bucket_angle_world=-0.5, elbow_up=False)
    if result.success:
        _assert_ik_fk_roundtrip(target, result)
        # Stick angle should be negative (elbow-down)
        assert result.joint_positions[2] <= 0.0


def test_elbow_up():
    target = np.array([4.0, 0.0, 0.0])
    result = solve_ik(target, bucket_angle_world=-0.3, elbow_up=True)
    if result.success:
        _assert_ik_fk_roundtrip(target, result)
        # Stick angle should be positive (elbow-up)
        assert result.joint_positions[2] >= 0.0


def test_nearest_prefers_elbow_down():
    """solve_ik_nearest should try elbow-down first."""
    target = np.array([4.5, 0.0, 0.0])
    result = solve_ik_nearest(target)
    if result.success:
        assert result.joint_positions[2] <= 0.05


# ====================================================================== #
#  Unreachable targets
# ====================================================================== #

def test_too_far():
    """Target beyond max reach."""
    target = np.array([15.0, 0.0, 0.0])
    result = solve_ik_nearest(target)
    assert not result.success
    assert result.status == IKStatus.OUT_OF_REACH


def test_too_close_to_shoulder():
    """Target right at the shoulder — inside the min-reach sphere."""
    target = np.array([SHOULDER_X_BASE_FRAME, 0.0, SHOULDER_Z_BASE_FRAME])
    result = solve_ik_nearest(target)
    assert not result.success  # OUT_OF_REACH or JOINT_LIMITS_VIOLATED


def test_behind_robot():
    """Target directly behind the base — out of workspace."""
    target = np.array([-8.0, 0.0, 0.0])
    result = solve_ik_nearest(target)
    assert not result.success


def test_far_to_the_side():
    target = np.array([0.0, 12.0, 0.0])
    result = solve_ik_nearest(target)
    assert not result.success


# ====================================================================== #
#  Non-zero base pose
# ====================================================================== #

def test_base_offset_x():
    """Robot at x=3, target at x=7 → same local geometry as 0→4."""
    target = np.array([7.0, 0.0, 0.5])
    result = solve_ik_nearest(target, base_x=3.0)
    _assert_ik_fk_roundtrip(target, result, base_x=3.0)


def test_base_offset_xy():
    """Base offset in both x and y with a reachable target."""
    target = np.array([7.0, 4.0, 0.0])
    result = solve_ik_nearest(target, base_x=3.0, base_y=2.0)
    _assert_ik_fk_roundtrip(target, result, base_x=3.0, base_y=2.0)


def test_base_rotated():
    """Robot facing +Y (yaw=π/2), target ahead of it."""
    target = np.array([-2.0, 4.0, 0.5])
    result = solve_ik_nearest(target, base_yaw=math.pi / 2)
    _assert_ik_fk_roundtrip(target, result, base_yaw=math.pi / 2)


def test_base_at_working_position():
    """From working position (3,0,0), reach toward excavation area."""
    target = np.array([7.0, 0.0, -0.5])
    result = solve_ik_nearest(target, base_x=3.0, base_y=0.0, base_yaw=0.0)
    _assert_ik_fk_roundtrip(target, result, base_x=3.0, base_y=0.0)


# ====================================================================== #
#  Targets inside the excavation area
# ====================================================================== #

@pytest.mark.parametrize("x,y,z", [
    (7.0, -1.0, -0.5),    # far edge of hole, shallow
    (7.5, -2.0, -0.5),    # mid-hole, moderate depth
    (6.5, -1.5, 0.0),     # near surface
    (7.0, -2.5, -0.5),    # off to the side
    (8.0, -2.0, -0.5),    # far reach
])
def test_excavation_points(x, y, z):
    """Targets inside the excavation area should be reachable."""
    target = np.array([x, y, z])
    result = solve_ik_nearest(target, base_x=3.0, base_y=0.0, base_yaw=0.0)
    _assert_ik_fk_roundtrip(target, result, base_x=3.0)


# ====================================================================== #
#  Bucket angle variations
# ====================================================================== #

def test_bucket_angled_45_down():
    """Default -π/4 ≈ -45° — good general digging angle."""
    target = np.array([4.0, 0.0, 0.0])
    result = solve_ik(target, bucket_angle_world=-math.pi / 4)
    if result.success:
        _assert_ik_fk_roundtrip(target, result)


def test_bucket_horizontal():
    """Bucket horizontal — level scoop."""
    target = np.array([4.0, 0.0, 0.0])
    result = solve_ik(target, bucket_angle_world=0.0)
    if result.success:
        _assert_ik_fk_roundtrip(target, result)


def test_bucket_steep():
    """Bucket at -60° — steep but usually still within limits."""
    target = np.array([5.0, 0.0, 0.0])
    result = solve_ik_nearest(target, bucket_angle_world=-1.0)
    if result.success:
        _assert_ik_fk_roundtrip(target, result)


def test_bucket_angle_sweep_finds_solution():
    """solve_ik_nearest sweeps bucket angles to find valid config."""
    target = np.array([4.0, 0.0, 0.0])
    # -π/2 may violate limits, but the sweep should find a valid angle
    result = solve_ik_nearest(target, bucket_angle_world=-math.pi / 2)
    assert result.success
    _assert_ik_fk_roundtrip(target, result)


# ====================================================================== #
#  Joint limit enforcement
# ====================================================================== #

def test_valid_solution_within_limits():
    target = np.array([4.0, 0.0, 0.5])
    result = solve_ik_nearest(target)
    if result.success:
        _assert_within_limits(result)


def test_multiple_targets_within_limits():
    """All reachable solutions should respect joint limits."""
    targets = [
        np.array([4.0, 0.0, 0.5]),
        np.array([4.5, 1.0, -0.5]),
        np.array([5.0, -1.0, 0.0]),
    ]
    for tgt in targets:
        result = solve_ik_nearest(tgt)
        if result.success:
            _assert_within_limits(result)


def test_explicit_bucket_angle_limits():
    """With a specific bucket angle that's borderline."""
    target = np.array([5.0, 0.0, 0.0])
    result = solve_ik(target, bucket_angle_world=-0.8)
    if result.success:
        _assert_within_limits(result)
        _assert_ik_fk_roundtrip(target, result)
