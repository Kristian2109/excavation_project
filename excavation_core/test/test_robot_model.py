"""
Unit tests for ExcavatorModel forward kinematics (no ROS required).

Maps to Key Tests Checklist:
  Test 2 (partial): FK for various joint values; joint-limit rejection.
"""

import math
import pytest
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
)


# --------------------------------------------------------------------- #
#  Fixtures
# --------------------------------------------------------------------- #

@pytest.fixture
def robot() -> ExcavatorModel:
    return ExcavatorModel()


# --------------------------------------------------------------------- #
#  Test: Zero configuration
# --------------------------------------------------------------------- #

class TestZeroConfig:
    """All joints at 0 – the arm extends straight along +X."""

    def test_bucket_tip_z(self, robot: ExcavatorModel):
        """Tip Z should equal chassis_height + cabin_height*0.8 - bucket_depth."""
        tip = robot.bucket_tip_position()
        expected_z = CHASSIS_HEIGHT + CABIN_HEIGHT * 0.8 - BUCKET_DEPTH
        assert tip[2] == pytest.approx(expected_z, abs=1e-6)

    def test_bucket_tip_y(self, robot: ExcavatorModel):
        """With zero swing, tip Y should be 0."""
        tip = robot.bucket_tip_position()
        assert tip[1] == pytest.approx(0.0, abs=1e-6)

    def test_bucket_tip_x(self, robot: ExcavatorModel):
        """Tip X = cabin_length + boom_length + stick_length + bucket_length."""
        tip = robot.bucket_tip_position()
        expected_x = CABIN_LENGTH + BOOM_LENGTH + STICK_LENGTH + BUCKET_LENGTH
        assert tip[0] == pytest.approx(expected_x, abs=1e-6)

    def test_all_frames_exist(self, robot: ExcavatorModel):
        frames = robot.fk_chain()
        expected = ['base_link', 'cabin_link', 'boom_link',
                    'stick_link', 'bucket_link', 'bucket_tip']
        for name in expected:
            assert name in frames


# --------------------------------------------------------------------- #
#  Test: Cabin swing
# --------------------------------------------------------------------- #

class TestCabinSwing:
    def test_swing_90_degrees(self, robot: ExcavatorModel):
        """Swing cabin 90° (π/2) → tip should be along +Y."""
        robot.set_joint('cabin_joint', math.pi / 2)
        tip = robot.bucket_tip_position()
        # X should be ≈ 0, Y should be ≈ full reach
        full_reach = CABIN_LENGTH + BOOM_LENGTH + STICK_LENGTH + BUCKET_LENGTH
        assert tip[0] == pytest.approx(0.0, abs=1e-3)
        assert tip[1] == pytest.approx(full_reach, abs=1e-3)

    def test_swing_180_degrees(self, robot: ExcavatorModel):
        """Swing cabin 180° → tip should be along -X."""
        robot.set_joint('cabin_joint', math.pi)
        tip = robot.bucket_tip_position()
        full_reach = CABIN_LENGTH + BOOM_LENGTH + STICK_LENGTH + BUCKET_LENGTH
        assert tip[0] == pytest.approx(-full_reach, abs=1e-3)
        assert abs(tip[1]) < 1e-3


# --------------------------------------------------------------------- #
#  Test: Boom raises tip
# --------------------------------------------------------------------- #

class TestBoomMotion:
    def test_boom_positive_lowers_tip(self, robot: ExcavatorModel):
        """Positive rotation about Y tilts the arm forward/down."""
        z_zero = robot.bucket_tip_position()[2]
        robot.set_joint('boom_joint', 0.5)
        z_fwd = robot.bucket_tip_position()[2]
        assert z_fwd < z_zero

    def test_boom_negative_raises_tip(self, robot: ExcavatorModel):
        """Negative rotation about Y tilts the arm back/up."""
        z_zero = robot.bucket_tip_position()[2]
        robot.set_joint('boom_joint', -0.2)
        z_back = robot.bucket_tip_position()[2]
        assert z_back > z_zero


# --------------------------------------------------------------------- #
#  Test: Stick curl
# --------------------------------------------------------------------- #

class TestStickMotion:
    def test_stick_curl_reduces_reach(self, robot: ExcavatorModel):
        """Curling the stick inward (negative) should reduce horizontal reach."""
        tip_zero = robot.bucket_tip_position()
        robot.set_joint('stick_joint', -1.0)
        tip_curled = robot.bucket_tip_position()
        reach_zero = math.hypot(tip_zero[0], tip_zero[1])
        reach_curled = math.hypot(tip_curled[0], tip_curled[1])
        assert reach_curled < reach_zero


# --------------------------------------------------------------------- #
#  Test: Joint limits
# --------------------------------------------------------------------- #

class TestJointLimits:
    def test_valid_config(self, robot: ExcavatorModel):
        robot.joint_positions = np.array([0.0, 0.5, -1.0, 1.0])
        assert robot.validate() is True

    def test_boom_over_limit(self, robot: ExcavatorModel):
        robot.set_joint('boom_joint', 2.0)  # upper limit is 1.2
        assert robot.validate() is False

    def test_stick_under_limit(self, robot: ExcavatorModel):
        robot.set_joint('stick_joint', -3.0)  # lower limit is -2.4
        assert robot.validate() is False

    def test_cabin_always_valid(self, robot: ExcavatorModel):
        """Cabin is continuous, so any value should be valid."""
        robot.set_joint('cabin_joint', 100.0)
        assert robot.validate() is True

    def test_clamp_to_limits(self, robot: ExcavatorModel):
        robot.joint_positions = np.array([0.0, 5.0, -5.0, 5.0])
        robot.clamp_to_limits()
        assert robot.validate() is True


# --------------------------------------------------------------------- #
#  Test: Base pose affects FK
# --------------------------------------------------------------------- #

class TestBasePose:
    def test_base_translation(self, robot: ExcavatorModel):
        tip_origin = robot.bucket_tip_position().copy()
        robot.base_x = 10.0
        robot.base_y = 5.0
        tip_moved = robot.bucket_tip_position()
        assert tip_moved[0] == pytest.approx(tip_origin[0] + 10.0, abs=1e-6)
        assert tip_moved[1] == pytest.approx(tip_origin[1] + 5.0, abs=1e-6)

    def test_base_yaw(self, robot: ExcavatorModel):
        robot.base_yaw = math.pi / 2
        tip = robot.bucket_tip_position()
        # With base rotated 90°, the full reach goes along Y
        full_reach = CABIN_LENGTH + BOOM_LENGTH + STICK_LENGTH + BUCKET_LENGTH
        assert tip[1] == pytest.approx(full_reach, abs=1e-3)
        assert abs(tip[0]) < 1e-3


# --------------------------------------------------------------------- #
#  Test: FK consistency  – transform is proper rotation
# --------------------------------------------------------------------- #

class TestFKConsistency:
    @pytest.mark.parametrize('joints', [
        [0.0, 0.0, 0.0, 0.0],
        [0.5, 0.3, -1.0, 1.5],
        [math.pi, 1.2, -2.4, 2.2],
    ])
    def test_transform_is_valid_se3(self, joints):
        robot = ExcavatorModel(joint_positions=np.array(joints))
        for name, T in robot.fk_chain().items():
            R = T[:3, :3]
            # Orthogonal: R^T R ≈ I
            assert np.allclose(R.T @ R, np.eye(3), atol=1e-9), \
                f'{name}: R not orthogonal'
            # det(R) ≈ +1
            assert abs(np.linalg.det(R) - 1.0) < 1e-9, \
                f'{name}: det(R) != 1'
