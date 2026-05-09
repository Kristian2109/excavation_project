"""
mission_controller.py – ROS-free state machine for the full excavation mission.

States
------
::

    IDLE → MOVING_TO_WORK_POS → PLANNING → EXCAVATING → COMPLETED
                                                ↘ FAILED

The controller tracks which scoop is next, how many succeeded / failed,
and whether the mission is complete.  All ROS interactions live in the
companion ``mission_controller_node.py``.

This module can be tested standalone without ROS.
"""

from __future__ import annotations

from enum import IntEnum
from dataclasses import dataclass, field
from typing import Optional

from excavation_core.excavation_grid import ExcavationGrid, HoleSpec
from excavation_core.excavation_planner import (
    ExcavationPlan,
    PlannedScoop,
    plan_excavation,
)
from excavation_core.excavation_model import ScoopFootprint


# ====================================================================== #
#  State enum
# ====================================================================== #

class MissionState(IntEnum):
    """Mission state constants (values match MissionStatus.msg)."""
    IDLE = 0
    MOVING_TO_WORK_POS = 1
    PLANNING = 2            # internal; maps to EXCAVATING in the ROS message
    EXCAVATING = 3
    COMPLETED = 4
    FAILED = 5


# ====================================================================== #
#  Progress snapshot
# ====================================================================== #

@dataclass
class MissionProgress:
    """Live progress counters for the mission."""
    state: MissionState = MissionState.IDLE
    current_scoop_index: int = 0
    total_scoops: int = 0
    scoops_succeeded: int = 0
    scoops_failed: int = 0
    status_text: str = 'Idle'

    @property
    def fraction_complete(self) -> float:
        """Fraction of scoops that succeeded so far."""
        if self.total_scoops == 0:
            return 0.0
        return self.scoops_succeeded / self.total_scoops

    @property
    def all_attempted(self) -> bool:
        return self.current_scoop_index >= self.total_scoops


# ====================================================================== #
#  Mission Controller
# ====================================================================== #

class MissionController:
    """Pure state-machine driving the excavation mission.

    Usage::

        mc = MissionController(hole, grid, ...)
        mc.start_mission()           # IDLE → MOVING_TO_WORK_POS

        # ... base drives ...
        mc.on_base_arrived()         # → PLANNING
        mc.generate_plan()           # → EXCAVATING

        while (s := mc.get_next_scoop()):
            ok = execute(s)          # arm trajectory
            mc.on_scoop_completed(ok)

        assert mc.state == MissionState.COMPLETED
    """

    def __init__(
        self,
        hole: HoleSpec,
        grid: ExcavationGrid,
        base_x: float = 3.0,
        base_y: float = 0.0,
        base_yaw: float = 0.0,
        footprint: Optional[ScoopFootprint] = None,
    ) -> None:
        self.hole = hole
        self.grid = grid
        self.base_x = base_x
        self.base_y = base_y
        self.base_yaw = base_yaw
        self.footprint = footprint or ScoopFootprint()

        self.progress = MissionProgress()
        self.plan: Optional[ExcavationPlan] = None

    # ------------------------------------------------------------------ #
    #  Convenience properties
    # ------------------------------------------------------------------ #
    @property
    def state(self) -> MissionState:
        return self.progress.state

    @state.setter
    def state(self, s: MissionState) -> None:
        self.progress.state = s

    def is_terminal(self) -> bool:
        """True when the mission cannot advance further."""
        return self.state in (MissionState.COMPLETED, MissionState.FAILED)

    # ------------------------------------------------------------------ #
    #  State transitions
    # ------------------------------------------------------------------ #
    def start_mission(self) -> bool:
        """``IDLE → MOVING_TO_WORK_POS``.  Returns *False* if not IDLE."""
        if self.state != MissionState.IDLE:
            return False
        self.state = MissionState.MOVING_TO_WORK_POS
        self.progress.status_text = 'Moving to working position'
        return True

    def on_base_arrived(self) -> bool:
        """``MOVING_TO_WORK_POS → PLANNING``."""
        if self.state != MissionState.MOVING_TO_WORK_POS:
            return False
        self.state = MissionState.PLANNING
        self.progress.status_text = 'Planning excavation sequence'
        return True

    def generate_plan(self) -> bool:
        """``PLANNING → EXCAVATING`` (or ``FAILED`` on empty plan)."""
        if self.state != MissionState.PLANNING:
            return False

        self.plan = plan_excavation(
            self.hole, self.grid,
            base_x=self.base_x,
            base_y=self.base_y,
            base_yaw=self.base_yaw,
            footprint=self.footprint,
        )

        if self.plan.total_scoops == 0:
            self.state = MissionState.FAILED
            self.progress.status_text = 'Planning produced zero scoops'
            return False

        self.progress.total_scoops = self.plan.total_scoops
        self.progress.current_scoop_index = 0
        self.progress.scoops_succeeded = 0
        self.progress.scoops_failed = 0
        self.state = MissionState.EXCAVATING
        self.progress.status_text = (
            f'Excavating: 0/{self.plan.total_scoops} scoops')
        return True

    def get_next_scoop(self) -> Optional[PlannedScoop]:
        """Return the next planned scoop, or *None* when done."""
        if self.state != MissionState.EXCAVATING or self.plan is None:
            return None
        idx = self.progress.current_scoop_index
        if idx >= self.plan.total_scoops:
            return None
        return self.plan.scoops[idx]

    def on_scoop_completed(self, success: bool) -> MissionState:
        """Advance after a scoop execution attempt.

        Returns the (potentially new) state.
        """
        if self.state != MissionState.EXCAVATING:
            return self.state

        if success:
            self.progress.scoops_succeeded += 1
        else:
            self.progress.scoops_failed += 1

        self.progress.current_scoop_index += 1

        if self.progress.all_attempted:
            self.state = MissionState.COMPLETED
            ok = self.progress.scoops_succeeded
            fail = self.progress.scoops_failed
            total = self.progress.total_scoops
            self.progress.status_text = (
                f'Complete: {ok}/{total} succeeded, {fail} failed')
        else:
            idx = self.progress.current_scoop_index
            self.progress.status_text = (
                f'Excavating: {idx}/{self.progress.total_scoops} scoops')

        return self.state

    def abort(self, reason: str = 'Mission aborted') -> None:
        """Force the mission into FAILED from any state."""
        self.state = MissionState.FAILED
        self.progress.status_text = reason
