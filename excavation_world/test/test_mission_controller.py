"""
Tests for mission_controller.py – the ROS-free state machine.

Run:
    cd /root/ws && colcon build --packages-select excavation_world --symlink-install
    cd /root/ws/src/excavation_project/excavation_world
    python -m pytest test/test_mission_controller.py -v
"""

import copy
import math
from typing import Optional

import numpy as np
import pytest

from excavation_world.excavation_grid import ExcavationGrid, HoleSpec
from excavation_world.excavation_model import ScoopFootprint
from excavation_world.excavation_planner import ExcavationPlan, PlannedScoop
from excavation_world.mission_controller import (
    MissionController,
    MissionProgress,
    MissionState,
)


# ====================================================================== #
#  Shared fixtures
# ====================================================================== #

@pytest.fixture
def hole():
    return HoleSpec(
        origin_x=5.0, origin_y=-2.0, origin_z=0.0,
        size_x=4.0, size_y=3.0, depth=2.0,
    )


@pytest.fixture
def grid(hole):
    return ExcavationGrid.from_hole_spec(hole, resolution=0.25)


@pytest.fixture
def mc(hole, grid):
    """A fresh MissionController in IDLE state."""
    return MissionController(
        hole=hole, grid=grid,
        base_x=3.0, base_y=0.0, base_yaw=0.0,
    )


# ====================================================================== #
#  MissionProgress tests
# ====================================================================== #

class TestMissionProgress:
    def test_initial_defaults(self):
        p = MissionProgress()
        assert p.state == MissionState.IDLE
        assert p.current_scoop_index == 0
        assert p.total_scoops == 0
        assert p.scoops_succeeded == 0
        assert p.scoops_failed == 0
        assert p.fraction_complete == 0.0
        # all_attempted is True when total_scoops=0 (vacuously true)
        assert p.all_attempted

    def test_fraction_complete(self):
        p = MissionProgress(total_scoops=10, scoops_succeeded=5)
        assert p.fraction_complete == pytest.approx(0.5)

    def test_fraction_complete_zero_total(self):
        p = MissionProgress(total_scoops=0)
        assert p.fraction_complete == 0.0

    def test_all_attempted(self):
        p = MissionProgress(total_scoops=3, current_scoop_index=3)
        assert p.all_attempted


# ====================================================================== #
#  State transition tests
# ====================================================================== #

class TestStateTransitions:
    def test_initial_state(self, mc):
        assert mc.state == MissionState.IDLE
        assert not mc.is_terminal()

    def test_start_mission(self, mc):
        assert mc.start_mission()
        assert mc.state == MissionState.MOVING_TO_WORK_POS

    def test_start_mission_double_call(self, mc):
        mc.start_mission()
        assert not mc.start_mission()  # already started
        assert mc.state == MissionState.MOVING_TO_WORK_POS

    def test_start_mission_wrong_state(self, mc):
        mc.state = MissionState.EXCAVATING
        assert not mc.start_mission()

    def test_on_base_arrived(self, mc):
        mc.start_mission()
        assert mc.on_base_arrived()
        assert mc.state == MissionState.PLANNING

    def test_on_base_arrived_wrong_state(self, mc):
        assert not mc.on_base_arrived()  # still IDLE
        assert mc.state == MissionState.IDLE

    def test_generate_plan(self, mc):
        mc.start_mission()
        mc.on_base_arrived()
        assert mc.generate_plan()
        assert mc.state == MissionState.EXCAVATING
        assert mc.plan is not None
        assert mc.plan.total_scoops > 0
        assert mc.progress.total_scoops == mc.plan.total_scoops

    def test_generate_plan_wrong_state(self, mc):
        assert not mc.generate_plan()  # IDLE, not PLANNING

    def test_abort_from_idle(self, mc):
        mc.abort('test abort')
        assert mc.state == MissionState.FAILED
        assert mc.is_terminal()
        assert 'test abort' in mc.progress.status_text

    def test_abort_from_excavating(self, mc):
        mc.start_mission()
        mc.on_base_arrived()
        mc.generate_plan()
        mc.abort('emergency')
        assert mc.state == MissionState.FAILED

    def test_is_terminal_completed(self, mc):
        mc.state = MissionState.COMPLETED
        assert mc.is_terminal()

    def test_is_terminal_failed(self, mc):
        mc.state = MissionState.FAILED
        assert mc.is_terminal()

    def test_is_terminal_running(self, mc):
        mc.state = MissionState.EXCAVATING
        assert not mc.is_terminal()


# ====================================================================== #
#  Scoop iteration tests
# ====================================================================== #

class TestScoopIteration:
    def _setup_excavating(self, mc):
        mc.start_mission()
        mc.on_base_arrived()
        mc.generate_plan()
        return mc

    def test_get_next_scoop_returns_first(self, mc):
        self._setup_excavating(mc)
        scoop = mc.get_next_scoop()
        assert scoop is not None
        assert scoop.scoop_id == 0

    def test_get_next_scoop_wrong_state(self, mc):
        assert mc.get_next_scoop() is None  # IDLE

    def test_scoop_completed_advances(self, mc):
        self._setup_excavating(mc)
        mc.on_scoop_completed(True)
        assert mc.progress.current_scoop_index == 1
        assert mc.progress.scoops_succeeded == 1

    def test_scoop_failed_advances(self, mc):
        self._setup_excavating(mc)
        mc.on_scoop_completed(False)
        assert mc.progress.current_scoop_index == 1
        assert mc.progress.scoops_failed == 1
        assert mc.progress.scoops_succeeded == 0

    def test_scoop_completed_wrong_state(self, mc):
        new_state = mc.on_scoop_completed(True)
        assert new_state == MissionState.IDLE  # no change

    def test_complete_all_scoops(self, mc):
        self._setup_excavating(mc)
        total = mc.progress.total_scoops
        for _i in range(total):
            mc.on_scoop_completed(True)
        assert mc.state == MissionState.COMPLETED
        assert mc.is_terminal()
        assert mc.progress.scoops_succeeded == total
        assert mc.progress.scoops_failed == 0

    def test_complete_with_failures(self, mc):
        self._setup_excavating(mc)
        total = mc.progress.total_scoops
        for i in range(total):
            mc.on_scoop_completed(i % 3 != 0)  # fail every 3rd
        assert mc.state == MissionState.COMPLETED
        assert mc.progress.scoops_succeeded + mc.progress.scoops_failed == total

    def test_scoop_sequence_ids(self, mc):
        """Scoops are returned in order by scoop_id."""
        self._setup_excavating(mc)
        ids = []
        while (s := mc.get_next_scoop()) is not None:
            ids.append(s.scoop_id)
            mc.on_scoop_completed(True)
        assert ids == list(range(len(ids)))

    def test_progress_fraction_midway(self, mc):
        self._setup_excavating(mc)
        total = mc.progress.total_scoops
        half = total // 2
        for _i in range(half):
            mc.on_scoop_completed(True)
        expected = half / total
        assert mc.progress.fraction_complete == pytest.approx(expected, abs=0.01)


# ====================================================================== #
#  Full mission flow
# ====================================================================== #

class TestFullMissionFlow:
    def test_end_to_end(self, mc):
        """Run the complete IDLE → COMPLETED sequence."""
        # 1. Start
        assert mc.start_mission()
        assert mc.state == MissionState.MOVING_TO_WORK_POS

        # 2. Base arrives
        assert mc.on_base_arrived()
        assert mc.state == MissionState.PLANNING

        # 3. Plan
        assert mc.generate_plan()
        assert mc.state == MissionState.EXCAVATING
        total = mc.progress.total_scoops
        assert total > 0

        # 4. Execute all scoops
        executed = 0
        while mc.get_next_scoop() is not None:
            mc.on_scoop_completed(True)
            executed += 1

        # 5. Mission complete
        assert mc.state == MissionState.COMPLETED
        assert mc.progress.scoops_succeeded == executed
        assert executed == total
        assert mc.is_terminal()

    def test_plan_coverage_sufficient(self, mc):
        """The generated plan covers ≥95% of the target volume."""
        mc.start_mission()
        mc.on_base_arrived()
        mc.generate_plan()
        cov = mc.plan.coverage_fraction(mc.grid)
        assert cov >= 0.95, f'Coverage too low: {cov:.1%}'

    def test_status_text_updates(self, mc):
        """Status text changes at each major transition."""
        texts = [mc.progress.status_text]
        mc.start_mission()
        texts.append(mc.progress.status_text)
        mc.on_base_arrived()
        texts.append(mc.progress.status_text)
        mc.generate_plan()
        texts.append(mc.progress.status_text)

        # All texts should be distinct (transitions produce new messages)
        assert len(set(texts)) == len(texts), f'Duplicate status texts: {texts}'

    def test_abort_during_excavation(self, mc):
        """Aborting mid-excavation freezes progress."""
        mc.start_mission()
        mc.on_base_arrived()
        mc.generate_plan()
        # Execute a few scoops
        for _ in range(3):
            mc.on_scoop_completed(True)
        mc.abort('test stop')
        assert mc.state == MissionState.FAILED
        assert mc.progress.scoops_succeeded == 3
        # Further scoop completions are ignored
        mc.on_scoop_completed(True)
        assert mc.progress.scoops_succeeded == 3  # unchanged

    def test_fresh_grid_not_mutated(self, hole, grid):
        """The planner should not mutate the grid during planning."""
        original_remaining = grid.remaining_target_cells
        mc = MissionController(hole=hole, grid=grid, base_x=3.0)
        mc.start_mission()
        mc.on_base_arrived()
        mc.generate_plan()
        assert grid.remaining_target_cells == original_remaining
