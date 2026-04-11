"""
excavation_model.py – Connects scoop actions to the 3D excavation grid.

A scoop is geometrically modelled as a *swept volume* under the bucket: the
set of grid cells that the bucket passes through from the DIG waypoint to the
SCOOP waypoint.  Because our simulation does not require realistic soil
physics, we approximate this swept volume as a rectangular box centred on the
dig target, sized by ``scoop_footprint`` (x, y half-widths around the target)
and extending from the dig depth up to the approach depth.

This module is ROS-free and can be tested standalone.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from excavation_world.excavation_grid import ExcavationGrid, HoleSpec
from excavation_world.robot_model import (
    ExcavatorModel,
    BUCKET_LENGTH,
    BUCKET_DEPTH,
)
from excavation_world.scoop_trajectory import (
    ScoopTrajectory,
    ScoopWaypoint,
    plan_single_scoop,
)


# ====================================================================== #
#  Scoop footprint parameters
# ====================================================================== #

@dataclass
class ScoopFootprint:
    """Geometry of the volume removed by one scoop.

    All dimensions in metres.
    """
    width: float = 1.0        # bucket width  (Y extent)
    length: float = 0.8       # sweep length  (X extent along reach direction)
    depth: float = 0.3        # vertical depth of material removed


# ====================================================================== #
#  Cell mapping: scoop → affected grid cells
# ====================================================================== #

def compute_scoop_cells(
    grid: ExcavationGrid,
    dig_target_xyz: np.ndarray,
    base_yaw: float = 0.0,
    cabin_angle: float = 0.0,
    footprint: Optional[ScoopFootprint] = None,
) -> List[Tuple[int, int, int]]:
    """Determine which grid cells a scoop at *dig_target_xyz* would affect.

    The scoop volume is a rectangular box aligned with the swing direction
    of the arm (base_yaw + cabin_angle), centred on the dig target XY,
    and extending from ``dig_target_z - footprint.depth`` up to
    ``dig_target_z``.

    Parameters
    ----------
    grid : ExcavationGrid
        The voxel grid.
    dig_target_xyz : (3,) array
        World-frame dig target [x, y, z].
    base_yaw : float
        Base orientation (rad).
    cabin_angle : float
        Cabin joint angle (rad).
    footprint : ScoopFootprint or None
        Size of the scoop box.  Uses defaults if None.

    Returns
    -------
    list of (ix, iy, iz) tuples
        Grid cell indices that fall inside the scoop volume.
    """
    if footprint is None:
        footprint = ScoopFootprint()

    target = np.asarray(dig_target_xyz, dtype=float)
    tx, ty, tz = target

    # The swing direction in the world frame
    swing_angle = base_yaw + cabin_angle
    cos_a = math.cos(swing_angle)
    sin_a = math.sin(swing_angle)

    # Half-extents in the swing-aligned frame:
    #   "along" = reach direction (radial from cabin)
    #   "across" = perpendicular (bucket width)
    half_along = footprint.length / 2.0
    half_across = footprint.width / 2.0

    # Vertical range (dig from tz-depth up to tz)
    z_lo = tz - footprint.depth
    z_hi = tz

    # Pre-compute the bounding box in world X-Y to narrow the cell search
    corner_offsets = [
        ( half_along,  half_across),
        ( half_along, -half_across),
        (-half_along,  half_across),
        (-half_along, -half_across),
    ]
    world_corners_x = []
    world_corners_y = []
    for da, dc in corner_offsets:
        wx = tx + da * cos_a - dc * sin_a
        wy = ty + da * sin_a + dc * cos_a
        world_corners_x.append(wx)
        world_corners_y.append(wy)

    x_min_w = min(world_corners_x)
    x_max_w = max(world_corners_x)
    y_min_w = min(world_corners_y)
    y_max_w = max(world_corners_y)

    # Convert world bounding box to grid index ranges
    gx, gy, gz = grid.grid_origin
    r = grid.resolution
    nx, ny, nz = grid.shape

    ix_lo = max(0, int(math.floor((x_min_w - gx) / r)))
    ix_hi = min(nx - 1, int(math.floor((x_max_w - gx) / r)))
    iy_lo = max(0, int(math.floor((y_min_w - gy) / r)))
    iy_hi = min(ny - 1, int(math.floor((y_max_w - gy) / r)))
    iz_lo = max(0, int(math.floor((z_lo - gz) / r)))
    iz_hi = min(nz - 1, int(math.floor((z_hi - gz) / r)))

    affected: List[Tuple[int, int, int]] = []

    for ix in range(ix_lo, ix_hi + 1):
        for iy in range(iy_lo, iy_hi + 1):
            for iz in range(iz_lo, iz_hi + 1):
                cx, cy, cz = grid.cell_centre(ix, iy, iz)

                # Check vertical
                if cz < z_lo or cz > z_hi:
                    continue

                # Rotate cell centre into swing-aligned frame
                dx = cx - tx
                dy = cy - ty
                local_along = dx * cos_a + dy * sin_a
                local_across = -dx * sin_a + dy * cos_a

                if (abs(local_along) <= half_along and
                        abs(local_across) <= half_across):
                    affected.append((ix, iy, iz))

    return affected


def compute_scoop_cells_from_trajectory(
    grid: ExcavationGrid,
    trajectory: ScoopTrajectory,
    base_x: float = 0.0,
    base_y: float = 0.0,
    base_yaw: float = 0.0,
    footprint: Optional[ScoopFootprint] = None,
) -> List[Tuple[int, int, int]]:
    """Extract the dig target from a trajectory and compute affected cells.

    Uses the 'dig' waypoint's ``target_xyz`` and the cabin angle from its
    joint positions.
    """
    dig_wp = None
    for wp in trajectory.waypoints:
        if wp.name == 'dig':
            dig_wp = wp
            break

    if dig_wp is None or dig_wp.target_xyz is None:
        return []

    cabin_angle = float(dig_wp.joint_positions[0])
    return compute_scoop_cells(
        grid, dig_wp.target_xyz,
        base_yaw=base_yaw,
        cabin_angle=cabin_angle,
        footprint=footprint,
    )


# ====================================================================== #
#  Apply scoop to grid
# ====================================================================== #

@dataclass
class ScoopResult:
    """Outcome of applying one scoop to the grid."""
    scoop_id: int
    cells_affected: int          # total cells marked excavated by this scoop
    target_cells_removed: int    # of those, how many were target cells
    remaining_target_cells: int  # after this scoop
    completion_fraction: float   # after this scoop


def apply_scoop_to_grid(
    grid: ExcavationGrid,
    dig_target_xyz: np.ndarray,
    base_yaw: float = 0.0,
    cabin_angle: float = 0.0,
    footprint: Optional[ScoopFootprint] = None,
    scoop_id: int = 0,
) -> ScoopResult:
    """Apply a single scoop to the grid and return the result.

    Parameters
    ----------
    grid : ExcavationGrid
        The voxel grid (mutated in place).
    dig_target_xyz : (3,) array
        Where the scoop digs.
    base_yaw, cabin_angle : float
        Orientation of the bucket sweep.
    footprint : ScoopFootprint or None
        Volume removed.
    scoop_id : int
        Sequence number.

    Returns
    -------
    ScoopResult
    """
    cells = compute_scoop_cells(
        grid, dig_target_xyz,
        base_yaw=base_yaw,
        cabin_angle=cabin_angle,
        footprint=footprint,
    )
    target_removed = grid.excavate_cells(cells)

    return ScoopResult(
        scoop_id=scoop_id,
        cells_affected=len(cells),
        target_cells_removed=target_removed,
        remaining_target_cells=grid.remaining_target_cells,
        completion_fraction=grid.completion_fraction,
    )


def apply_trajectory_to_grid(
    grid: ExcavationGrid,
    trajectory: ScoopTrajectory,
    base_x: float = 0.0,
    base_y: float = 0.0,
    base_yaw: float = 0.0,
    footprint: Optional[ScoopFootprint] = None,
) -> ScoopResult:
    """Apply a scoop trajectory to the grid.

    Extracts the dig target and cabin angle from the trajectory's 'dig'
    waypoint and delegates to :func:`apply_scoop_to_grid`.
    """
    dig_wp = None
    for wp in trajectory.waypoints:
        if wp.name == 'dig':
            dig_wp = wp
            break

    if dig_wp is None or dig_wp.target_xyz is None:
        return ScoopResult(
            scoop_id=trajectory.scoop_id,
            cells_affected=0,
            target_cells_removed=0,
            remaining_target_cells=grid.remaining_target_cells,
            completion_fraction=grid.completion_fraction,
        )

    cabin_angle = float(dig_wp.joint_positions[0])
    return apply_scoop_to_grid(
        grid, dig_wp.target_xyz,
        base_yaw=base_yaw,
        cabin_angle=cabin_angle,
        footprint=footprint,
        scoop_id=trajectory.scoop_id,
    )
