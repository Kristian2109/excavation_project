"""
Tests for scoop_trajectory.py – Stage 5: Single Scoop Trajectory.

Covers:
  - Planning a single scoop at a reachable target
  - All waypoints are kinematically valid
  - Waypoint order and naming
  - FK round-trip for each waypoint with a target
  - Scoop trajectory validation
  - Planning at various excavation targets
  - Planning a sequence of scoops
  - Unreachable target returns None
  - Duration and total_duration consistency
"""

import math
import numpy as np
import pytest

from excavation_world.scoop_trajectory import (
    ScoopTrajectory,
    plan_single_scoop,
    READY_JOINTS,
)
from excavation_world.ik_solver import verify_ik_solution, IKResult, IKStatus
from excavation_world.robot_model import ExcavatorModel, JOINT_NAMES


# ====================================================================== #
#  Helpers
# ====================================================================== #

BASE_X, BASE_Y, BASE_YAW = 3.0, 0.0, 0.0  # Default working position


def _fk_tip(joints, bx=BASE_X, by=BASE_Y, byaw=BASE_YAW) -> np.ndarray:
    model = ExcavatorModel(
        base_x=bx, base_y=by, base_yaw=byaw,
        joint_positions=joints.copy(),
    )
    return model.bucket_tip_position()


# ====================================================================== #
#  Basic single scoop
# ====================================================================== #

class TestSingleScoop:
    def test_plan_returns_trajectory(self):
        """A reachable target should produce a valid trajectory."""
        target = np.array([7.0, -1.0, -0.3])
        traj = plan_single_scoop(target, base_x=BASE_X)
        assert traj is not None
        assert isinstance(traj, ScoopTrajectory)

    def test_waypoint_count(self):
        """A scoop should have 6 waypoints (ready→approach→dig→scoop→lift→ready)."""
        target = np.array([7.0, -1.0, -0.3])
        traj = plan_single_scoop(target, base_x=BASE_X)
        assert traj is not None
        assert len(traj.waypoints) == 6

    def test_waypoint_names(self):
        """Verify the expected waypoint naming sequence."""
        target = np.array([7.0, -1.0, -0.3])
        traj = plan_single_scoop(target, base_x=BASE_X)
        assert traj is not None
        names = [wp.name for wp in traj.waypoints]
        assert names == ['ready_start', 'approach', 'dig', 'scoop', 'lift', 'ready_end']

    def test_all_waypoints_valid(self):
        """Every waypoint must have joint positions within limits."""
        target = np.array([7.0, -1.0, -0.3])
        traj = plan_single_scoop(target, base_x=BASE_X)
        assert traj is not None
        assert traj.validate()

    def test_trajectory_joint_names(self):
        target = np.array([7.0, -1.0, -0.3])
        traj = plan_single_scoop(target, base_x=BASE_X)
        assert traj is not None
        assert traj.joint_names == list(JOINT_NAMES)


# ====================================================================== #
#  FK round-trip for waypoints with targets
# ====================================================================== #

class TestWaypointFKRoundtrip:
    """Waypoints that were solved via IK should match their targets."""

    def test_approach_hits_target(self):
        target = np.array([7.0, -1.0, -0.3])
        traj = plan_single_scoop(target, base_x=BASE_X)
        assert traj is not None
        approach_wp = traj.waypoints[1]  # "approach"
        assert approach_wp.target_xyz is not None
        tip = _fk_tip(approach_wp.joint_positions)
        error = np.linalg.norm(tip - approach_wp.target_xyz)
        assert error < 0.1, f'Approach FK error: {error:.4f}'

    def test_dig_hits_target(self):
        target = np.array([7.0, -1.0, -0.3])
        traj = plan_single_scoop(target, base_x=BASE_X)
        assert traj is not None
        dig_wp = traj.waypoints[2]  # "dig"
        assert dig_wp.target_xyz is not None
        tip = _fk_tip(dig_wp.joint_positions)
        error = np.linalg.norm(tip - dig_wp.target_xyz)
        assert error < 0.1, f'Dig FK error: {error:.4f}'

    def test_lift_hits_target(self):
        target = np.array([7.0, -1.0, -0.3])
        traj = plan_single_scoop(target, base_x=BASE_X)
        assert traj is not None
        lift_wp = traj.waypoints[4]  # "lift"
        assert lift_wp.target_xyz is not None
        tip = _fk_tip(lift_wp.joint_positions)
        error = np.linalg.norm(tip - lift_wp.target_xyz)
        assert error < 0.1, f'Lift FK error: {error:.4f}'


# ====================================================================== #
#  Duration consistency
# ====================================================================== #

class TestDuration:
    def test_total_duration(self):
        target = np.array([7.0, -1.0, -0.3])
        traj = plan_single_scoop(target, base_x=BASE_X)
        assert traj is not None
        expected = sum(wp.duration for wp in traj.waypoints)
        assert abs(traj.total_duration - expected) < 1e-6

    def test_all_durations_positive(self):
        target = np.array([7.0, -1.0, -0.3])
        traj = plan_single_scoop(target, base_x=BASE_X)
        assert traj is not None
        for wp in traj.waypoints:
            assert wp.duration > 0


# ====================================================================== #
#  Various excavation targets
# ====================================================================== #

class TestExcavationTargets:
    @pytest.mark.parametrize("x,y,z", [
        (7.0, -1.0, -0.3),
        (7.5, -2.0, -0.5),
        (6.5, -1.5, 0.0),
        (8.0, -2.0, -0.3),
    ])
    def test_reachable_targets(self, x, y, z):
        target = np.array([x, y, z])
        traj = plan_single_scoop(target, base_x=BASE_X)
        assert traj is not None, f'Failed to plan scoop at ({x}, {y}, {z})'
        assert traj.validate()
        assert len(traj.waypoints) == 6


# ====================================================================== #
#  Unreachable targets
# ====================================================================== #

class TestUnreachable:
    def test_too_far(self):
        """Target beyond arm reach returns None."""
        target = np.array([20.0, 0.0, 0.0])
        traj = plan_single_scoop(target, base_x=BASE_X)
        assert traj is None

    def test_behind_robot(self):
        target = np.array([-5.0, 0.0, 0.0])
        traj = plan_single_scoop(target, base_x=BASE_X)
        assert traj is None

# ====================================================================== #
#  No large joint jumps
# ====================================================================== #

class TestSmoothness:
    def test_no_large_joint_jumps(self):
        """Consecutive waypoints should not have huge joint changes."""
        target = np.array([7.0, -1.0, -0.3])
        traj = plan_single_scoop(target, base_x=BASE_X)
        assert traj is not None
        max_jump_rad = 2.5  # ~143° — generous but catches wild swings
        for i in range(1, len(traj.waypoints)):
            prev = traj.waypoints[i - 1].joint_positions
            curr = traj.waypoints[i].joint_positions
            jump = np.max(np.abs(curr - prev))
            assert jump < max_jump_rad, (
                f'Large joint jump between {traj.waypoints[i-1].name} → '
                f'{traj.waypoints[i].name}: max delta = {jump:.2f} rad'
            )


# ====================================================================== #
#  Ready pose validation
# ====================================================================== #

class TestReadyPose:
    def test_ready_joints_valid(self):
        """The default READY configuration should be within limits."""
        model = ExcavatorModel(joint_positions=READY_JOINTS.copy())
        assert model.validate()

    def test_scoop_starts_and_ends_at_ready(self):
        """First and last waypoints should be 'ready' poses."""
        target = np.array([7.0, -1.0, -0.3])
        traj = plan_single_scoop(target, base_x=BASE_X)
        assert traj is not None
        assert traj.waypoints[0].name == 'ready_start'
        assert traj.waypoints[-1].name == 'ready_end'
        # They should have the same joint positions
        np.testing.assert_allclose(
            traj.waypoints[0].joint_positions,
            traj.waypoints[-1].joint_positions,
            atol=1e-6,
        )
