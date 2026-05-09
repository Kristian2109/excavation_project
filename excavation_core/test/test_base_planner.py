"""
Unit tests for base_planner.py (no ROS required).

Maps to Key Tests Checklist:
  Test 2: The base reaches the working position.
"""

import math
import pytest

from excavation_core.base_planner import (
    BasePose,
    BaseTrajectory,
    plan_base_trajectory,
    _wrap_angle,
)


# --------------------------------------------------------------------- #
#  Fixtures
# --------------------------------------------------------------------- #

@pytest.fixture
def start() -> BasePose:
    return BasePose(x=0.0, y=0.0, yaw=0.0)


@pytest.fixture
def goal() -> BasePose:
    """Working position from the world_node defaults."""
    return BasePose(x=3.0, y=0.0, yaw=0.0)


@pytest.fixture
def trajectory(start, goal) -> BaseTrajectory:
    return plan_base_trajectory(start, goal)


def test_trajectory_not_empty(trajectory: BaseTrajectory):
    """Convenience selector for direct single-test invocation."""
    assert len(trajectory.points) > 0


# --------------------------------------------------------------------- #
#  Test: Trajectory generation
# --------------------------------------------------------------------- #

class TestTrajectoryGeneration:
    def test_trajectory_not_empty(self, trajectory: BaseTrajectory):
        assert len(trajectory.points) > 0

    def test_starts_at_start(self, trajectory: BaseTrajectory, start: BasePose):
        p = trajectory.start_pose
        assert p.x == pytest.approx(start.x, abs=1e-6)
        assert p.y == pytest.approx(start.y, abs=1e-6)

    def test_ends_at_goal(self, trajectory: BaseTrajectory, goal: BasePose):
        p = trajectory.end_pose
        assert p.x == pytest.approx(goal.x, abs=0.05)
        assert p.y == pytest.approx(goal.y, abs=0.05)
        assert _wrap_angle(p.yaw - goal.yaw) == pytest.approx(0.0, abs=0.05)

    def test_duration_positive(self, trajectory: BaseTrajectory):
        assert trajectory.duration > 0

    def test_time_monotonic(self, trajectory: BaseTrajectory):
        times = [p.time for p in trajectory.points]
        for i in range(1, len(times)):
            assert times[i] >= times[i - 1]


# --------------------------------------------------------------------- #
#  Test: Trajectory reaches goal with different configurations
# --------------------------------------------------------------------- #

class TestDifferentGoals:
    @pytest.mark.parametrize('gx, gy, gyaw', [
        (3.0, 0.0, 0.0),             # straight ahead
        (0.0, 5.0, math.pi / 2),     # left, facing up
        (-2.0, -2.0, math.pi),       # behind and left, facing back
        (10.0, 10.0, -math.pi / 4),  # far diagonal
        (0.0, 0.0, math.pi),         # pure rotation
    ])
    def test_reaches_goal(self, gx, gy, gyaw):
        start = BasePose(0.0, 0.0, 0.0)
        goal = BasePose(gx, gy, gyaw)
        traj = plan_base_trajectory(start, goal)

        end = traj.end_pose
        assert end.x == pytest.approx(goal.x, abs=0.05)
        assert end.y == pytest.approx(goal.y, abs=0.05)
        assert abs(_wrap_angle(end.yaw - goal.yaw)) < 0.05

    def test_same_pose(self):
        """Start == goal should produce a short or trivial trajectory."""
        pose = BasePose(1.0, 2.0, 0.5)
        traj = plan_base_trajectory(pose, pose)
        assert traj.start_pose.distance_to(traj.end_pose) < 0.01


# --------------------------------------------------------------------- #
#  Test: Sampling / interpolation
# --------------------------------------------------------------------- #

class TestSampling:
    def test_sample_at_start(self, trajectory: BaseTrajectory, start: BasePose):
        p = trajectory.sample(0.0)
        assert p.x == pytest.approx(start.x, abs=1e-6)

    def test_sample_at_end(self, trajectory: BaseTrajectory, goal: BasePose):
        p = trajectory.sample(trajectory.duration)
        assert p.x == pytest.approx(goal.x, abs=0.05)

    def test_sample_before_start(self, trajectory: BaseTrajectory, start: BasePose):
        p = trajectory.sample(-1.0)
        assert p.x == pytest.approx(start.x, abs=1e-6)

    def test_sample_after_end(self, trajectory: BaseTrajectory, goal: BasePose):
        p = trajectory.sample(trajectory.duration + 100.0)
        assert p.x == pytest.approx(goal.x, abs=0.05)

    def test_sample_midway_between_start_and_goal(self, trajectory: BaseTrajectory):
        """At half duration, position should be roughly halfway."""
        mid = trajectory.sample(trajectory.duration / 2.0)
        # Should be between start and goal in X
        sx = trajectory.start_pose.x
        gx = trajectory.end_pose.x
        assert min(sx, gx) - 0.5 <= mid.x <= max(sx, gx) + 0.5


# --------------------------------------------------------------------- #
#  Test: Position error threshold
# --------------------------------------------------------------------- #

class TestPositionError:
    def test_final_error_below_threshold(self):
        """Per the spec: final position/orientation error below a threshold."""
        start = BasePose(0.0, 0.0, 0.0)
        goal = BasePose(3.0, 0.0, 0.0)
        traj = plan_base_trajectory(start, goal)

        end = traj.end_pose
        pos_error = end.distance_to(goal)
        ang_error = end.angle_distance_to(goal)

        assert pos_error < 0.1, f"Position error {pos_error} too large"
        assert ang_error < 0.1, f"Angular error {ang_error} too large"


# --------------------------------------------------------------------- #
#  Test: Wrap angle
# --------------------------------------------------------------------- #

class TestWrapAngle:
    @pytest.mark.parametrize('a, expected', [
        (0.0, 0.0),
        (math.pi, -math.pi),         # π wraps to −π (boundary)
        (-math.pi, -math.pi),
        (2 * math.pi, 0.0),
        (3 * math.pi, -math.pi),     # same boundary
        (-3 * math.pi, -math.pi),
    ])
    def test_wrap_angle(self, a, expected):
        assert _wrap_angle(a) == pytest.approx(expected, abs=1e-9)
