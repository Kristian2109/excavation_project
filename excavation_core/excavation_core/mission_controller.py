"""
mission_controller.py – ROS-free state machine for the full excavation mission.

States
------
::

    IDLE → MOVING_TO_WORK_POS → PLANNING → EXCAVATING
                ↑                                │
                └──── RELOCATING ←───────────────┘
                                              → COMPLETED / FAILED

The controller supports multiple working positions.  After exhausting the
planned scoops at one position it transitions to RELOCATING, updates the
base pose, and goes through MOVING → PLANNING → EXCAVATING again for the
next position.

The controller tracks which scoop is next, how many succeeded / failed,
and whether the mission is complete.  All ROS interactions live in the
companion ``mission_controller_node.py``.

This module can be tested standalone without ROS.
"""

from __future__ import annotations

from enum import IntEnum
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

from excavation_core.excavation_grid import ExcavationGrid, HoleSpec
from excavation_core.excavation_planner import (
    ExcavationPlan,
    PlannedScoop,
    plan_excavation,
)
from excavation_core.excavation_model import ScoopFootprint
from excavation_core.base_planner import BasePose
from excavation_core.position_planner import compute_work_positions


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
    RELOCATING = 6          # moving to next working position


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
    current_position_index: int = 0
    total_positions: int = 1

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

    Supports multiple working positions.  The controller cycles through
    each position: move → plan (for reachable cells) → excavate → next.

    Usage::

        mc = MissionController(hole, grid, work_positions=[...])
        mc.start_mission()           # IDLE → MOVING_TO_WORK_POS

        # ... base drives to position 0 ...
        mc.on_base_arrived()         # → PLANNING
        mc.generate_plan()           # → EXCAVATING

        while (s := mc.get_next_scoop()):
            ok = execute(s)
            mc.on_scoop_completed(ok)

        # If more positions remain → RELOCATING
        # on_base_arrived() → PLANNING again …
    """

    def __init__(
        self,
        hole: HoleSpec,
        grid: ExcavationGrid,
        work_positions: Optional[List[BasePose]] = None,
        base_x: float = 3.0,
        base_y: float = 0.0,
        base_yaw: float = 0.0,
        footprint: Optional[ScoopFootprint] = None,
    ) -> None:
        self.hole = hole
        self.grid = grid
        self.footprint = footprint or ScoopFootprint()

        # Work positions (multi-position support)
        if work_positions and len(work_positions) > 0:
            self.work_positions = list(work_positions)
        else:
            # Legacy single-position fallback
            self.work_positions = [BasePose(x=base_x, y=base_y, yaw=base_yaw)]

        self._position_index = 0
        self._apply_position(0)

        # Cumulative counters across all positions
        self._total_scoops_all = 0
        self._succeeded_all = 0
        self._failed_all = 0

        self.progress = MissionProgress(
            total_positions=len(self.work_positions),
        )
        self.plan: Optional[ExcavationPlan] = None

    def _apply_position(self, idx: int) -> None:
        """Set base_x/y/yaw from the work_positions list."""
        pos = self.work_positions[idx]
        self.base_x = pos.x
        self.base_y = pos.y
        self.base_yaw = pos.yaw
        self._position_index = idx

    @property
    def current_work_position(self) -> BasePose:
        return self.work_positions[self._position_index]

    @property
    def next_work_position(self) -> Optional[BasePose]:
        """Return the next position, or None if this is the last."""
        nxt = self._position_index + 1
        if nxt < len(self.work_positions):
            return self.work_positions[nxt]
        return None

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
        self._apply_position(0)
        self.state = MissionState.MOVING_TO_WORK_POS
        self.progress.current_position_index = 0
        self.progress.status_text = (
            f'Moving to position 1/{len(self.work_positions)}')
        return True

    def on_base_arrived(self) -> bool:
        """``MOVING_TO_WORK_POS / RELOCATING → PLANNING``."""
        if self.state not in (MissionState.MOVING_TO_WORK_POS,
                              MissionState.RELOCATING):
            return False
        self.state = MissionState.PLANNING
        self.progress.status_text = (
            f'Planning at position {self._position_index + 1}'
            f'/{len(self.work_positions)}')
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
            # No scoops at this position.  Try next position if available.
            if self._advance_to_next_position():
                return True  # state is now RELOCATING
            self.state = MissionState.COMPLETED
            self.progress.status_text = (
                f'Complete: all positions exhausted '
                f'({self._succeeded_all} scoops succeeded)')
            return False

        self.progress.total_scoops = self.plan.total_scoops
        self.progress.current_scoop_index = 0
        self.progress.scoops_succeeded = 0
        self.progress.scoops_failed = 0
        self.state = MissionState.EXCAVATING
        self.progress.status_text = (
            f'Excavating at position {self._position_index + 1}: '
            f'0/{self.plan.total_scoops} scoops')
        return True

    def filter_unreachable(
        self,
        checker: Callable[[PlannedScoop], Optional[object]],
    ) -> int:
        """Remove scoops for which *checker* returns ``None``.

        *checker* is called with each :class:`PlannedScoop`. If it returns
        a non-``None`` value (e.g. a ``ScoopTrajectory``), the scoop is
        kept and ``scoop.trajectory`` is set to that value.  Otherwise the
        scoop is removed from the plan.

        After filtering, ``progress`` counters are updated and – when zero
        reachable scoops remain – the controller automatically advances to
        the next work position (or completes the mission).

        Returns the number of scoops removed.
        """
        if self.plan is None:
            return 0

        reachable = []
        for scoop in self.plan.scoops:
            result = checker(scoop)
            if result is not None:
                scoop.trajectory = result
                reachable.append(scoop)

        removed = len(self.plan.scoops) - len(reachable)
        self.plan.scoops = reachable
        self.progress.total_scoops = len(reachable)
        self.progress.current_scoop_index = 0

        if len(reachable) == 0:
            # Try to move on to the next position instead of aborting.
            if not self._advance_to_next_position():
                self.state = MissionState.COMPLETED
                self.progress.status_text = (
                    f'Complete: {self._succeeded_all} succeeded, '
                    f'{self._failed_all} failed '
                    f'across {len(self.work_positions)} position(s)')

        return removed

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

        # Update the grid so future plans skip already-excavated cells.
        if success and self.plan is not None:
            idx = self.progress.current_scoop_index
            if idx < len(self.plan.scoops):
                self.grid.excavate_cells(
                    self.plan.scoops[idx].affected_cells)

        if success:
            self.progress.scoops_succeeded += 1
            self._succeeded_all += 1
        else:
            self.progress.scoops_failed += 1
            self._failed_all += 1

        self._total_scoops_all += 1
        self.progress.current_scoop_index += 1

        if self.progress.all_attempted:
            # All scoops at this position done.  Move to next position?
            if self._advance_to_next_position():
                return self.state  # RELOCATING

            # All positions exhausted
            self.state = MissionState.COMPLETED
            ok = self._succeeded_all
            fail = self._failed_all
            self.progress.status_text = (
                f'Complete: {ok} succeeded, {fail} failed '
                f'across {len(self.work_positions)} position(s)')
        else:
            idx = self.progress.current_scoop_index
            self.progress.status_text = (
                f'Excavating at position {self._position_index + 1}: '
                f'{idx}/{self.progress.total_scoops} scoops')

        return self.state

    def _advance_to_next_position(self) -> bool:
        """Try to advance to the next work position.

        If no pre-planned positions remain but the grid still has
        unexcavated target cells, dynamically compute extra positions
        and continue.

        Returns True if we're now RELOCATING to a new position,
        False if no more positions remain or the grid is complete.
        """
        nxt = self._position_index + 1
        if nxt < len(self.work_positions):
            self._apply_position(nxt)
            self.state = MissionState.RELOCATING
            self.progress.current_position_index = nxt
            self.progress.status_text = (
                f'Relocating to position {nxt + 1}'
                f'/{len(self.work_positions)}')
            return True

        # All pre-planned positions exhausted — check for remaining cells.
        if self.grid.remaining_target_cells == 0:
            return False

        # Re-compute positions from the hole; skip any already visited.
        new_positions = compute_work_positions(self.hole)
        visited = {(round(p.x, 3), round(p.y, 3))
                   for p in self.work_positions}
        extras = [p for p in new_positions
                  if (round(p.x, 3), round(p.y, 3)) not in visited]

        if not extras:
            return False

        # Append the new positions and continue.
        self.work_positions.extend(extras)
        self.progress.total_positions = len(self.work_positions)
        nxt = self._position_index + 1
        self._apply_position(nxt)
        self.state = MissionState.RELOCATING
        self.progress.current_position_index = nxt
        self.progress.status_text = (
            f'Relocating to position {nxt + 1}'
            f'/{len(self.work_positions)} (re-planned)')
        return True

    def abort(self, reason: str = 'Mission aborted') -> None:
        """Force the mission into FAILED from any state."""
        self.state = MissionState.FAILED
        self.progress.status_text = reason
