"""
position_planner.py – Compute optimal robot base positions for excavation.

Given a rectangular hole and the arm's reach parameters, this module
computes one or more base positions from which the arm can cover the
entire hole.  When a single position is insufficient, multiple positions
are returned in a logical order so the robot can relocate between
excavation phases.

This module is ROS-free and can be tested standalone.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List

import numpy as np

from excavation_core.base_planner import BasePose
from excavation_core.excavation_grid import HoleSpec
from excavation_core.ik_solver import solve_ik_nearest
from excavation_core.robot_model import (
    BOOM_LENGTH,
    STICK_LENGTH,
    BUCKET_LENGTH,
    BUCKET_DEPTH,
    CABIN_LENGTH,
)


# Effective arm reach from shoulder pivot (conservative)
_ARM_REACH = BOOM_LENGTH + STICK_LENGTH + math.hypot(BUCKET_LENGTH, BUCKET_DEPTH)

# Shoulder offset from base_link origin along the facing direction
_SHOULDER_OFFSET = CABIN_LENGTH


def _optimal_standoff(hole_span: float, hole_depth: float,
                      arm_reach: float) -> float:
    """Compute the optimal base stand-off from the nearest hole edge.

    The base is positioned so the shoulder can reach the far-deep corner:
        sqrt((standoff + hole_span)² + depth²) ≤ arm_reach

    Solving for standoff:
        standoff = sqrt(arm_reach² - depth²) - hole_span

    We also enforce a minimum so the robot isn't inside the hole,
    and a maximum so near-edge cells are still reachable.
    """
    reach_sq = arm_reach ** 2
    depth_sq = hole_depth ** 2

    if reach_sq <= depth_sq:
        # Arm can't even reach full depth vertically — use minimum
        return max(1.0, arm_reach * 0.3)

    max_standoff = math.sqrt(reach_sq - depth_sq) - hole_span
    # Ensure we're at least 0.5m from the hole edge
    standoff = max(0.5, max_standoff)
    # But not so far that we can't reach the near edge at depth
    # Near edge at depth: sqrt(standoff² + depth²) ≤ arm_reach
    # This is always satisfied if standoff ≤ sqrt(arm_reach² - depth²)
    near_limit = math.sqrt(max(0, reach_sq - depth_sq))
    standoff = min(standoff, near_limit * 0.9)

    return standoff


def compute_work_positions(
    hole: HoleSpec,
    arm_reach: float | None = None,
    clearance: float | None = None,
) -> List[BasePose]:
    """Compute base positions that together cover the full hole.

    Strategy
    --------
    1. Compute optimal stand-off distance from hole edge based on arm
       reach and hole depth.
    2. Try positioning on the -X side (facing +X into the hole).
    3. Check IK coverage of a grid of sample points.
    4. If coverage is insufficient, add a position on the opposite side
       and/or additional lateral positions.

    Parameters
    ----------
    hole : HoleSpec
        Rectangular target excavation volume.
    arm_reach : float or None
        Arm tip reach from shoulder [m].  Auto-computed if None.
    clearance : float or None
        Override stand-off distance [m].  Auto-computed if None.

    Returns
    -------
    list of BasePose
        Ordered working positions.
    """
    if arm_reach is None:
        arm_reach = _ARM_REACH

    # Hole bounding box
    hx0 = hole.origin_x
    hx1 = hole.origin_x + hole.size_x
    hy0 = hole.origin_y
    hy1 = hole.origin_y + hole.size_y
    hcx = (hx0 + hx1) / 2.0
    hcy = (hy0 + hy1) / 2.0

    # Optimal stand-off considering the shoulder offset
    shoulder_reach = arm_reach  # from shoulder pivot
    if clearance is None:
        # Stand-off from hole edge to base_link origin.
        # Shoulder is _SHOULDER_OFFSET ahead of base, so effective
        # distance from shoulder to near edge = clearance - shoulder_offset.
        # But we want shoulder at standoff from near edge:
        standoff_shoulder = _optimal_standoff(
            hole.size_x, hole.depth, shoulder_reach)
        clearance = standoff_shoulder + _SHOULDER_OFFSET

    # Generate candidate positions on each side
    positions: list[BasePose] = []

    # Determine which side(s) to use based on hole aspect ratio.
    # Prefer the side where the hole is narrower (less lateral sweep).
    sides = _candidate_sides(hx0, hx1, hy0, hy1, hcx, hcy, clearance)

    # Score each side by IK coverage and select the best set
    from excavation_core.excavation_grid import ExcavationGrid
    grid = ExcavationGrid.from_hole_spec(hole, resolution=0.5)  # coarse for speed
    total = len(grid.target_flat_indices())
    if total == 0:
        return [sides[0][0]] if sides else [BasePose(x=hcx, y=hcy, yaw=0.0)]

    covered: set[int] = set()
    selected: list[BasePose] = []

    # Try sides in order; within each side try each position
    for side_positions in sides:
        for pos in side_positions:
            new_cells = _count_coverage(pos, grid, covered)
            if len(new_cells) > 0:
                selected.append(pos)
                covered.update(new_cells)

            if len(covered) / total >= 0.95:
                return selected

    return selected if selected else [sides[0][0]]


def _candidate_sides(
    hx0: float, hx1: float,
    hy0: float, hy1: float,
    hcx: float, hcy: float,
    clearance: float,
) -> list[list[BasePose]]:
    """Generate candidate positions on the 4 sides of the hole."""
    sides: list[list[BasePose]] = []

    # -X side (facing +X)
    sides.append([BasePose(x=hx0 - clearance, y=hcy, yaw=0.0)])
    # +X side (facing -X)
    sides.append([BasePose(x=hx1 + clearance, y=hcy, yaw=math.pi)])
    # -Y side (facing +Y)
    sides.append([BasePose(x=hcx, y=hy0 - clearance, yaw=math.pi / 2)])
    # +Y side (facing -Y)
    sides.append([BasePose(x=hcx, y=hy1 + clearance, yaw=-math.pi / 2)])

    return sides


def _count_coverage(
    pos: BasePose,
    grid,
    already_covered: set[int],
) -> set[int]:
    """Return flat indices reachable from *pos* not yet covered."""
    new_cells: set[int] = set()
    nx, ny, nz = grid.shape

    for fi in grid.target_flat_indices():
        fi_int = int(fi)
        if fi_int in already_covered:
            continue
        ix = fi_int // (ny * nz)
        rem = fi_int % (ny * nz)
        iy = rem // nz
        iz = rem % nz
        cx, cy, cz = grid.cell_centre(ix, iy, iz)

        result = solve_ik_nearest(
            np.array([cx, cy, cz]),
            base_x=pos.x,
            base_y=pos.y,
            base_yaw=pos.yaw,
        )
        if result.success:
            new_cells.add(fi_int)

    return new_cells
