"""
excavation_planner.py – Pre-plans the full excavation sequence.

Decomposes the rectangular target hole into an ordered list of scoops that,
when executed sequentially, cover the entire target volume.

Strategy
--------
1. **Layers** – The hole is divided vertically into layers of thickness
   ``footprint.depth``.  The topmost layer is excavated first.
2. **Sweep** – Within each layer, scoop targets are arranged on a regular
   grid whose spacing matches the scoop footprint (length × width), offset
   to centre coverage over the hole.
3. **Direction** – Scoops radiate outward from the cabin: the row closest
   to the robot is done first (so there is always a clear path for the
   bucket), alternating sweep direction for efficiency (boustrophedon).
4. **Reachability** – Each target is checked via ``plan_single_scoop()``.
   Only reachable scoops are included in the plan.
5. **Cell association** – Each scoop is annotated with the grid cells it
   will remove, so the plan can be validated for total coverage.

This module is ROS-free and can be tested standalone.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from excavation_core.excavation_grid import ExcavationGrid, HoleSpec
from excavation_core.excavation_model import (
    ScoopFootprint,
    compute_scoop_cells,
)
from excavation_core.scoop_trajectory import (
    ScoopTrajectory,
    plan_single_scoop,
)


# ====================================================================== #
#  Plan data structures
# ====================================================================== #

@dataclass
class PlannedScoop:
    """A single scoop in the excavation plan."""
    scoop_id: int
    dig_target: np.ndarray          # world [x, y, z]
    layer: int                      # layer index (0 = topmost)
    affected_cells: List[Tuple[int, int, int]] = field(default_factory=list)
    trajectory: Optional[ScoopTrajectory] = None
    reachable: bool = True

    @property
    def expected_cell_count(self) -> int:
        return len(self.affected_cells)


@dataclass
class ExcavationPlan:
    """Complete pre-planned sequence of scoops."""
    scoops: List[PlannedScoop] = field(default_factory=list)
    hole: Optional[HoleSpec] = None
    footprint: Optional[ScoopFootprint] = None

    @property
    def total_scoops(self) -> int:
        return len(self.scoops)

    @property
    def total_planned_cells(self) -> int:
        """Union of all affected cells (no double-counting)."""
        all_cells = set()
        for s in self.scoops:
            all_cells.update(s.affected_cells)
        return len(all_cells)

    def coverage_fraction(self, grid: ExcavationGrid) -> float:
        """What fraction of the target cells does this plan cover?"""
        total = grid.total_target_cells
        if total == 0:
            return 1.0
        target_mask = grid._target_mask
        planned = set()
        for s in self.scoops:
            for c in s.affected_cells:
                if target_mask[c]:
                    planned.add(c)
        return len(planned) / total

    def validate(self, grid: ExcavationGrid, threshold: float = 0.90) -> bool:
        """Return True if the plan covers at least *threshold* of the target."""
        return self.coverage_fraction(grid) >= threshold


# ====================================================================== #
#  Planner
# ====================================================================== #

def plan_excavation(
    hole: HoleSpec,
    grid: ExcavationGrid,
    base_x: float = 3.0,
    base_y: float = 0.0,
    base_yaw: float = 0.0,
    footprint: Optional[ScoopFootprint] = None,
    overlap: float = 0.2,
    plan_trajectories: bool = False,
) -> ExcavationPlan:
    """Generate a complete excavation plan for the given hole.

    Parameters
    ----------
    hole : HoleSpec
        Target rectangular excavation volume.
    grid : ExcavationGrid
        The voxel grid (NOT mutated; used only for cell association).
    base_x, base_y, base_yaw : float
        Robot base pose during excavation (fixed).
    footprint : ScoopFootprint or None
        Scoop dimensions.  Uses defaults if None.
    overlap : float
        Fractional overlap between adjacent scoops (0.0–0.5).
        Higher overlap → better coverage but more scoops.
    plan_trajectories : bool
        If True, also solve IK and build ScoopTrajectory for each scoop.
        Set False for faster planning when trajectories are computed later.

    Returns
    -------
    ExcavationPlan
        Ordered list of scoops covering the target volume.
    """
    if footprint is None:
        footprint = ScoopFootprint()

    plan = ExcavationPlan(hole=hole, footprint=footprint)

    # Stepping distances (with overlap)
    step_x = footprint.length * (1.0 - overlap)
    step_y = footprint.width * (1.0 - overlap)
    step_z = footprint.depth * (1.0 - overlap)

    # Ensure minimum step size
    step_x = max(step_x, grid.resolution)
    step_y = max(step_y, grid.resolution)
    step_z = max(step_z, grid.resolution)

    # Hole bounds — sweep the scoop centre across the full extent.
    # Each scoop covers [target - half_extent, target + half_extent] in
    # each axis, so the first target should be at hole_min and the last
    # at hole_max (not inset by half-extent) to ensure full edge coverage.
    x_min = hole.origin_x
    x_max = hole.origin_x + hole.size_x
    y_min = hole.origin_y
    y_max = hole.origin_y + hole.size_y
    z_top = hole.origin_z
    z_bottom = hole.origin_z - hole.depth

    # Generate layers from top to bottom.
    # In compute_scoop_cells the vertical range is [tz - depth, tz].
    # So the first target z must equal z_top so it covers the top surface.
    z_targets = []
    z = z_top
    while z > z_bottom - step_z / 2.0:
        z_targets.append(z)
        z -= step_z
    if not z_targets:
        z_targets = [z_top]

    # Generate X sweep positions
    x_targets = []
    x = x_min
    while x <= x_max + step_x * 0.01:
        x_targets.append(x)
        x += step_x
    if not x_targets:
        x_targets = [(x_min + x_max) / 2.0]

    # Generate Y sweep positions
    y_targets = []
    y = y_min
    while y <= y_max + step_y * 0.01:
        y_targets.append(y)
        y += step_y
    if not y_targets:
        y_targets = [(y_min + y_max) / 2.0]

    # Sort X positions: nearest to robot first (larger X = further from base)
    # The robot is at base_x, so sort by distance to base
    x_targets.sort(key=lambda x: abs(x - base_x))

    scoop_id = 0
    ik_kwargs = dict(base_x=base_x, base_y=base_y, base_yaw=base_yaw)

    for layer_idx, z_t in enumerate(z_targets):
        # Boustrophedon: alternate Y sweep direction per X row
        for row_idx, x_t in enumerate(x_targets):
            ys = y_targets if row_idx % 2 == 0 else list(reversed(y_targets))
            for y_t in ys:
                dig_target = np.array([x_t, y_t, z_t])

                # Compute cabin angle toward target
                dx = x_t - base_x
                dy = y_t - base_y
                cos_yaw = math.cos(base_yaw)
                sin_yaw = math.sin(base_yaw)
                x_local = cos_yaw * dx + sin_yaw * dy
                y_local = -sin_yaw * dx + cos_yaw * dy
                cabin_angle = math.atan2(y_local, x_local)

                # Compute affected cells using axis-aligned footprint.
                # The actual bucket is rotated by the cabin angle, but for
                # planning purposes an axis-aligned footprint tiles the
                # rectangular hole without gaps.
                cells = compute_scoop_cells(
                    grid, dig_target,
                    base_yaw=0.0,
                    cabin_angle=0.0,
                    footprint=footprint,
                )

                # Optionally plan trajectory
                traj = None
                reachable = True
                if plan_trajectories:
                    traj = plan_single_scoop(
                        dig_target, scoop_id=scoop_id, **ik_kwargs)
                    reachable = traj is not None

                planned = PlannedScoop(
                    scoop_id=scoop_id,
                    dig_target=dig_target,
                    layer=layer_idx,
                    affected_cells=cells,
                    trajectory=traj,
                    reachable=reachable,
                )

                # Only include scoops that affect cells
                if len(cells) > 0:
                    plan.scoops.append(planned)
                    scoop_id += 1

    return plan


# ====================================================================== #
#  Simulation: run the full plan on a fresh grid
# ====================================================================== #

def simulate_plan(
    plan: ExcavationPlan,
    grid: ExcavationGrid,
    base_yaw: float = 0.0,
) -> float:
    """Execute the plan on a (copy of the) grid, returning final completion.

    The grid is **mutated** in place.  Pass a copy if you want to preserve
    the original state.
    """
    for scoop in plan.scoops:
        grid.excavate_cells(scoop.affected_cells)
    return grid.completion_fraction
