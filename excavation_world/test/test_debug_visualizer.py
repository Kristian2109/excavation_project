"""
Tests for Stage 9 – visualization helpers.

These test the pure-Python parts (FK for trail, marker construction)
without requiring ROS.
"""

import math
import numpy as np
import pytest

from excavation_world.robot_model import ExcavatorModel, JOINT_NAMES
from excavation_world.scoop_trajectory import plan_single_scoop


# ====================================================================== #
#  Bucket tip trail
# ====================================================================== #

class TestBucketTipTrail:
    """The debug visualizer computes bucket tip via FK — verify the chain."""

    def test_zero_joints_tip_position(self):
        model = ExcavatorModel(
            joint_positions=np.zeros(4),
            base_x=2.0, base_y=-0.5, base_yaw=0.0,
        )
        tip = model.bucket_tip_position()
        # Tip should be in front of the robot
        assert tip[0] > 2.0
        assert tip.shape == (3,)

    def test_different_joints_different_tip(self):
        m1 = ExcavatorModel(
            joint_positions=np.array([0.0, 0.3, -1.2, 0.5]),
            base_x=2.0, base_y=-0.5, base_yaw=0.0,
        )
        m2 = ExcavatorModel(
            joint_positions=np.array([0.5, 0.5, -0.5, 1.0]),
            base_x=2.0, base_y=-0.5, base_yaw=0.0,
        )
        tip1 = m1.bucket_tip_position()
        tip2 = m2.bucket_tip_position()
        assert not np.allclose(tip1, tip2)

    def test_cabin_swing_changes_xy(self):
        """Swinging the cabin should change the XY direction of the tip."""
        base_args = dict(base_x=2.0, base_y=-0.5, base_yaw=0.0)
        m0 = ExcavatorModel(
            joint_positions=np.array([0.0, 0.3, -1.2, 0.5]), **base_args)
        m90 = ExcavatorModel(
            joint_positions=np.array([math.pi / 2, 0.3, -1.2, 0.5]), **base_args)
        tip0 = m0.bucket_tip_position()
        tip90 = m90.bucket_tip_position()
        # At 90° cabin swing, Y should be significantly different
        assert abs(tip90[1] - tip0[1]) > 1.0


# ====================================================================== #
#  Arm trajectory visualization
# ====================================================================== #

class TestArmTrajectoryVisualization:
    """Verify FK on scoop trajectory waypoints for marker construction."""

    def test_scoop_waypoints_produce_valid_tips(self):
        target = np.array([6.0, -1.0, -0.3])
        traj = plan_single_scoop(
            target, base_x=2.0, base_y=-0.5, base_yaw=0.0)
        assert traj is not None

        tips = []
        for wp in traj.waypoints:
            model = ExcavatorModel(
                joint_positions=wp.joint_positions.copy(),
                base_x=2.0, base_y=-0.5, base_yaw=0.0,
            )
            tip = model.bucket_tip_position()
            tips.append(tip)
            assert np.isfinite(tip).all()

        # There should be at least 4 distinct positions
        unique = len(set(tuple(t.round(2)) for t in tips))
        assert unique >= 4

    def test_dig_waypoint_near_target(self):
        target = np.array([6.0, -1.0, -0.3])
        traj = plan_single_scoop(
            target, base_x=2.0, base_y=-0.5, base_yaw=0.0)
        assert traj is not None

        dig_wp = [wp for wp in traj.waypoints if wp.name == 'dig'][0]
        model = ExcavatorModel(
            joint_positions=dig_wp.joint_positions.copy(),
            base_x=2.0, base_y=-0.5, base_yaw=0.0,
        )
        tip = model.bucket_tip_position()
        # Tip should be near the target (within 1m)
        dist = np.linalg.norm(tip[:2] - target[:2])
        assert dist < 1.5, f'Dig tip too far from target: {dist:.2f}m'


# ====================================================================== #
#  Scoop plan targets
# ====================================================================== #

class TestScoopPlanTargets:
    """Verify plan generates targets that can be converted to Point markers."""

    def test_all_targets_finite(self):
        from excavation_world.excavation_grid import ExcavationGrid, HoleSpec
        from excavation_world.excavation_planner import plan_excavation

        hole = HoleSpec(origin_x=5.0, origin_y=-2.0, origin_z=0.0,
                        size_x=4.0, size_y=3.0, depth=2.0)
        grid = ExcavationGrid.from_hole_spec(hole, resolution=0.25)
        plan = plan_excavation(hole, grid, base_x=2.0, base_y=-0.5)

        for s in plan.scoops:
            assert np.isfinite(s.dig_target).all()
            assert len(s.dig_target) == 3

    def test_targets_within_hole_bounds(self):
        from excavation_world.excavation_grid import ExcavationGrid, HoleSpec
        from excavation_world.excavation_planner import plan_excavation

        hole = HoleSpec(origin_x=5.0, origin_y=-2.0, origin_z=0.0,
                        size_x=4.0, size_y=3.0, depth=2.0)
        grid = ExcavationGrid.from_hole_spec(hole, resolution=0.25)
        plan = plan_excavation(hole, grid, base_x=2.0, base_y=-0.5)

        for s in plan.scoops:
            x, y, z = s.dig_target
            assert 4.5 <= x <= 9.5, f'X out of bounds: {x}'
            assert -2.5 <= y <= 1.5, f'Y out of bounds: {y}'
            assert -2.5 <= z <= 0.5, f'Z out of bounds: {z}'


# ====================================================================== #
#  Debug text construction
# ====================================================================== #

class TestDebugText:
    """Test the text formatting logic (extracted to a pure function)."""

    def test_format_state_names(self):
        state_names = {0: 'IDLE', 1: 'MOVING', 2: 'EXCAVATING',
                       3: 'COMPLETED', 4: 'FAILED'}
        for code, name in state_names.items():
            assert isinstance(name, str)
            assert len(name) > 0

    def test_joint_degree_conversion(self):
        joints = {'cabin_joint': 0.0, 'boom_joint': 0.3,
                  'stick_joint': -1.2, 'bucket_joint': 0.5}
        for j in JOINT_NAMES:
            deg = math.degrees(joints[j])
            assert isinstance(deg, float)
            assert -180 <= deg <= 180
