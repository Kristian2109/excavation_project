"""
position_planner.py – Compute optimal robot base positions for excavation.

Given a rectangular hole and the arm's reach parameters, this module
computes one or more base positions from which the arm can cover the
entire hole.  When a single position is insufficient, multiple positions
are returned in an order that minimises total travel distance.

The algorithm:
  1. Generates dense candidate positions along all four sides of the hole,
     spaced by a fraction of the arm's lateral reach.
  2. Rejects any candidate whose base footprint overlaps the hole.
  3. Uses a greedy set-cover to select the smallest set of positions that
     together reach ≥ 95 % of target cells, preferring candidates that
     add the most new coverage at each step.
  4. Orders the selected positions by nearest-neighbour to minimise the
     total base travel distance.

This module is ROS-free and can be tested standalone.
"""

from __future__ import annotations

import math
from typing import List, Optional

import numpy as np

from excavation_core.excavation_grid import ExcavationGrid
from excavation_core.base_planner import BasePose
from excavation_core.excavation_grid import HoleSpec
from excavation_core.ik_solver import (
    solve_ik_nearest,
    SHOULDER_X_LOCAL,
    SHOULDER_Z_LOCAL,
    L1,
    L2,
)
from excavation_core.robot_model import (
    BOOM_LENGTH,
    STICK_LENGTH,
    BUCKET_LENGTH,
    BUCKET_DEPTH,
    CABIN_LENGTH,
    _build_joint_defs,
)


# Effective arm reach from shoulder pivot (conservative)
_ARM_REACH = BOOM_LENGTH + STICK_LENGTH + math.hypot(BUCKET_LENGTH, BUCKET_DEPTH)

# Shoulder offset from base_link origin along the facing direction
_SHOULDER_OFFSET = CABIN_LENGTH

# Minimum clearance from hole edge to robot centre (metres)
_MIN_HOLE_CLEARANCE = 0.5

# Candidate spacing as a fraction of arm reach
_CANDIDATE_SPACING_FRACTION = 0.45

# Coverage threshold — stop adding positions once this is reached
_COVERAGE_THRESHOLD = 0.95

# Geometric reach bounds (from shoulder pivot to bucket tip)
_MAX_SHOULDER_REACH = L1 + L2 + math.hypot(BUCKET_LENGTH, BUCKET_DEPTH)
_MIN_SHOULDER_REACH = max(0.0, abs(L1 - L2) - math.hypot(BUCKET_LENGTH, BUCKET_DEPTH))
_MAX_REACH_SQ = _MAX_SHOULDER_REACH ** 2
_MIN_REACH_SQ = _MIN_SHOULDER_REACH ** 2

# Joint limits (for the fast workspace check)
_JDEFS = _build_joint_defs()
_BOOM_LOWER = _JDEFS['boom_joint'].lower     # -0.5
_BOOM_UPPER = _JDEFS['boom_joint'].upper      # 1.5
_STICK_LOWER = _JDEFS['stick_joint'].lower    # -2.7
_STICK_UPPER = _JDEFS['stick_joint'].upper     # 0.3

# Wrist offset from bucket tip (most favourable bucket angle for
# minimising boom stress).  psi_opt maximises the vertical lift of
# the wrist above the dig target.
_BUCKET_HYPOT = math.hypot(BUCKET_LENGTH, BUCKET_DEPTH)
_PSI_OPT = math.atan2(BUCKET_LENGTH, BUCKET_DEPTH)  # ~1.01 rad
_WRIST_R_OFFSET = (BUCKET_LENGTH * math.cos(_PSI_OPT)
                   - BUCKET_DEPTH * math.sin(_PSI_OPT))  # ≈ 0.0
_WRIST_DZ_LIFT = (BUCKET_LENGTH * math.sin(_PSI_OPT)
                  + BUCKET_DEPTH * math.cos(_PSI_OPT))   # ≈ 0.94


def _precompute_cell_centers(grid) -> np.ndarray:
    """Return (N, 3) array of world centres for all target cells."""
    indices = grid.target_flat_indices()
    ny, nz = grid.shape[1], grid.shape[2]
    ix = indices // (ny * nz)
    rem = indices % (ny * nz)
    iy = rem // nz
    iz = rem % nz
    gx, gy, gz = grid.grid_origin
    half = grid.resolution / 2.0
    return np.column_stack([
        gx + ix * grid.resolution + half,
        gy + iy * grid.resolution + half,
        gz + iz * grid.resolution + half,
    ]).astype(np.float64)


def _fast_reachable_mask(pos: BasePose, centers: np.ndarray) -> np.ndarray:
    """Return boolean mask — True for cells reachable from *pos*.

    Performs two checks, all NumPy-vectorised:
      1. **Distance check** – the target must be within the arm's reach
         envelope (inner & outer radius from the shoulder pivot).
      2. **Joint-limit check** – the boom and stick angles required to
         reach the target's *wrist* point (accounting for one favourable
         bucket orientation) must lie within their URDF limits.

    This is orders of magnitude faster than per-cell IK while closely
    matching the true IK-feasible workspace.
    """
    # ---- swing-plane coordinates relative to shoulder ----
    dx = centers[:, 0] - pos.x
    dy = centers[:, 1] - pos.y
    r = np.sqrt(dx * dx + dy * dy)

    dr = r - SHOULDER_X_LOCAL
    dz = SHOULDER_Z_LOCAL - centers[:, 2]  # positive = below shoulder

    # ---- 1. outer / inner distance check (target) ----
    d_sq_target = dr * dr + dz * dz
    dist_ok = (d_sq_target <= _MAX_REACH_SQ) & (d_sq_target >= _MIN_REACH_SQ)

    # ---- 2. joint-limit feasibility (on wrist, not target) ----
    # Approximate wrist position using the most favourable bucket angle.
    # This shifts the effective target closer to the shoulder vertically.
    dr_w = dr - _WRIST_R_OFFSET
    dz_w = dz - _WRIST_DZ_LIFT       # wrist is higher → less depth

    d_sq_w = dr_w * dr_w + dz_w * dz_w
    d_w = np.sqrt(d_sq_w)

    # 2-link IK cosine for elbow
    cos_elbow = np.clip(
        (d_sq_w - L1 ** 2 - L2 ** 2) / (2.0 * L1 * L2), -1.0, 1.0)

    # Elbow-down (normal excavator posture)
    stick_dn = -np.arccos(cos_elbow)
    beta_dn = np.arctan2(
        L2 * np.sin(stick_dn), L1 + L2 * np.cos(stick_dn))
    alpha = np.arctan2(dz_w, dr_w)
    boom_dn = alpha - beta_dn
    dn_ok = ((boom_dn >= _BOOM_LOWER) & (boom_dn <= _BOOM_UPPER)
             & (stick_dn >= _STICK_LOWER))

    # Elbow-up (less common)
    stick_up = np.arccos(cos_elbow)
    beta_up = np.arctan2(
        L2 * np.sin(stick_up), L1 + L2 * np.cos(stick_up))
    boom_up = alpha - beta_up
    up_ok = ((boom_up >= _BOOM_LOWER) & (boom_up <= _BOOM_UPPER)
             & (stick_up <= _STICK_UPPER))

    # Wrist must itself be within 2-link reach
    wrist_reach_ok = (d_w <= L1 + L2) & (d_w >= abs(L1 - L2))

    joints_ok = wrist_reach_ok & (dn_ok | up_ok)

    return dist_ok & joints_ok


def _practical_clearance(cross_span: float, depth: float,
                        arm_reach: float) -> float:
    """Compute base-to-hole-edge distance for one side of the hole.

    Parameters
    ----------
    cross_span : float
        Hole extent in the direction the arm faces from this side.
    depth : float
        Hole depth (positive downward).
    arm_reach : float
        Maximum arm tip reach from shoulder.

    The shoulder is placed close enough that the far edge at full
    depth is just within reach, but no further than necessary.
    Being close to the hole maximises lateral sweep and keeps the
    arm in a comfortable region of its workspace.
    """
    # Horizontal reach available at the hole depth
    available = math.sqrt(max(0, arm_reach ** 2 - depth ** 2))

    if available <= cross_span:
        # Arm can't reach the far edge → get as close as possible
        shoulder_standoff = _MIN_HOLE_CLEARANCE
    else:
        # Place shoulder so that far edge at depth is just reachable,
        # with a 10 % safety margin.
        shoulder_standoff = (available - cross_span) * 0.4
        shoulder_standoff = max(_MIN_HOLE_CLEARANCE, shoulder_standoff)
        # Never stand further than needed to reach the near edge
        shoulder_standoff = min(shoulder_standoff, available * 0.5)

    return shoulder_standoff + _SHOULDER_OFFSET


def _is_inside_hole(x: float, y: float, hole: HoleSpec,
                    margin: float = _MIN_HOLE_CLEARANCE) -> bool:
    """Return True if (x, y) is inside or too close to the hole."""
    return (hole.origin_x - margin < x < hole.origin_x + hole.size_x + margin
            and hole.origin_y - margin < y < hole.origin_y + hole.size_y + margin)


def _generate_candidates(
    hole: HoleSpec,
    arm_reach: float,
    clearance_override: float | None = None,
) -> List[BasePose]:
    """Generate dense candidate positions along all four sides of the hole.

    Each side gets its own stand-off distance based on the cross-span
    the arm must bridge from that side.  Candidates are spaced along
    each side at intervals proportional to the arm reach.

    If *clearance_override* is given, it is used for every side instead
    of the per-side heuristic.
    """
    hx0 = hole.origin_x
    hx1 = hole.origin_x + hole.size_x
    hy0 = hole.origin_y
    hy1 = hole.origin_y + hole.size_y

    # Per-side clearance: use the hole dimension the arm faces.
    #   -X / +X sides face across size_x
    #   -Y / +Y sides face across size_y
    if clearance_override is not None:
        cl_x = clearance_override
        cl_y = clearance_override
    else:
        cl_x = _practical_clearance(hole.size_x, hole.depth, arm_reach)
        cl_y = _practical_clearance(hole.size_y, hole.depth, arm_reach)

    spacing = arm_reach * _CANDIDATE_SPACING_FRACTION
    candidates: list[BasePose] = []

    def _spread(lo: float, hi: float) -> list[float]:
        """Evenly spaced values covering [lo, hi]."""
        span = hi - lo
        if span <= 0:
            return [(lo + hi) / 2.0]
        n = max(1, int(math.ceil(span / spacing)))
        return [lo + i * span / n for i in range(n + 1)]

    # -X side (facing +X into the hole, yaw = 0)
    bx = hx0 - cl_x
    for by in _spread(hy0, hy1):
        if not _is_inside_hole(bx, by, hole):
            candidates.append(BasePose(x=bx, y=by, yaw=0.0))

    # +X side (facing -X, yaw = π)
    bx = hx1 + cl_x
    for by in _spread(hy0, hy1):
        if not _is_inside_hole(bx, by, hole):
            candidates.append(BasePose(x=bx, y=by, yaw=math.pi))

    # -Y side (facing +Y, yaw = π/2)
    by = hy0 - cl_y
    for bx_c in _spread(hx0, hx1):
        if not _is_inside_hole(bx_c, by, hole):
            candidates.append(BasePose(x=bx_c, y=by, yaw=math.pi / 2))

    # +Y side (facing -Y, yaw = -π/2)
    by = hy1 + cl_y
    for bx_c in _spread(hx0, hx1):
        if not _is_inside_hole(bx_c, by, hole):
            candidates.append(BasePose(x=bx_c, y=by, yaw=-math.pi / 2))

    return candidates


def _pose_distance(a: BasePose, b: BasePose) -> float:
    """Euclidean distance between two base poses."""
    return math.hypot(a.x - b.x, a.y - b.y)


def _order_nearest_neighbour(
    positions: List[BasePose],
    start: Optional[BasePose] = None,
) -> List[BasePose]:
    """Reorder positions using nearest-neighbour heuristic.

    Starting from *start* (or the first position), greedily visit the
    closest unvisited position.  This minimises total travel distance
    for small position sets typical for excavation.
    """
    if len(positions) <= 1:
        return list(positions)

    remaining = list(positions)
    if start is not None:
        # Find the closest position to the start
        remaining.sort(key=lambda p: _pose_distance(p, start))

    ordered: list[BasePose] = [remaining.pop(0)]
    while remaining:
        last = ordered[-1]
        nearest_idx = min(range(len(remaining)),
                          key=lambda i: _pose_distance(last, remaining[i]))
        ordered.append(remaining.pop(nearest_idx))
    return ordered


def compute_work_positions(
    hole: HoleSpec,
    arm_reach: float | None = None,
    clearance: float | None = None,
) -> List[BasePose]:
    """Compute base positions that together cover the full hole.

    Strategy
    --------
    1. Generate dense candidate positions along all four sides of the
       hole, rejecting any that fall inside or too close to the hole.
    2. Compute geometric reachability for each candidate using a fast
       vectorised distance check from the shoulder pivot (no IK needed).
    3. Greedy set-cover: repeatedly pick the candidate that adds the
       most new cells, until ≥ 95 % coverage or no improvement.
    4. Order the selected positions by nearest-neighbour to minimise
       travel distance.

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
        Ordered working positions (travel-distance optimised).
    """
    if arm_reach is None:
        arm_reach = _ARM_REACH

    # Hole bounding box
    hx0 = hole.origin_x
    hcy = (hole.origin_y + hole.origin_y + hole.size_y) / 2.0

    # Fallback clearance for empty-grid edge cases
    fallback_cl = clearance or _practical_clearance(
        min(hole.size_x, hole.size_y), hole.depth, arm_reach)

    # Build grid for geometric coverage evaluation.
    # 0.5m resolution is sufficient for the distance-based reachability
    # check and keeps the grid small for fast numpy operations.
    eval_res = min(0.5, min(hole.size_x, hole.size_y, hole.depth) / 2.0
                   if min(hole.size_x, hole.size_y, hole.depth) > 0 else 0.5)
    eval_res = max(eval_res, 0.1)  # floor to avoid huge grids
    grid = ExcavationGrid.from_hole_spec(hole, resolution=eval_res)
    target_indices = grid.target_flat_indices()
    total = len(target_indices)
    if total == 0:
        return [BasePose(x=hx0 - fallback_cl, y=hcy, yaw=0.0)]

    # --- Step 1: Generate candidate positions ---
    candidates = _generate_candidates(hole, arm_reach, clearance)
    if not candidates:
        return [BasePose(x=hx0 - fallback_cl, y=hcy, yaw=0.0)]

    # --- Step 2: Pre-compute coverage for each candidate (geometric) ---
    centers = _precompute_cell_centers(grid)
    n_cells = len(centers)
    # Build (num_candidates, n_cells) boolean matrix
    all_masks = np.empty((len(candidates), n_cells), dtype=bool)
    for i, pos in enumerate(candidates):
        all_masks[i] = _fast_reachable_mask(pos, centers)

    # --- Step 3: Greedy set-cover (vectorised) ---
    covered = np.zeros(n_cells, dtype=bool)
    selected: list[BasePose] = []
    used = np.zeros(len(candidates), dtype=bool)

    while covered.sum() / total < _COVERAGE_THRESHOLD:
        # New coverage each unused candidate would add
        not_covered = ~covered
        new_counts = (all_masks & not_covered).sum(axis=1)
        new_counts[used] = 0

        best_idx = int(new_counts.argmax())
        if new_counts[best_idx] == 0:
            break  # no further improvement possible

        selected.append(candidates[best_idx])
        covered |= all_masks[best_idx]
        used[best_idx] = True

    if not selected:
        # Nothing was reachable at all — return a safe default
        return [BasePose(x=hx0 - fallback_cl, y=hcy, yaw=0.0)]

    # --- Step 4: Order positions to minimise travel distance ---
    selected = _order_nearest_neighbour(selected)

    return selected


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
