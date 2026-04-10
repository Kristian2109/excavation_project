"""
excavation_grid.py – 3D voxel grid for excavation simulation.

Each cell is either UNEXCAVATED or EXCAVATED.  The target excavation volume
is a rectangular sub-region of the grid.

This module is intentionally free of ROS dependencies so that it can be
unit-tested without a running ROS system.
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional


# Cell states
UNEXCAVATED = 0
EXCAVATED = 1


@dataclass
class HoleSpec:
    """Axis-aligned rectangular target hole in world coordinates."""
    origin_x: float     # World X of the hole corner closest to origin
    origin_y: float     # World Y
    origin_z: float     # World Z (top surface of the hole)
    size_x: float       # Width  (m)
    size_y: float       # Length (m)
    depth: float        # Positive downward (m)


@dataclass
class ExcavationGrid:
    """3D occupancy grid for the excavation area.

    The grid is defined by:
        * ``grid_origin`` – the (x, y, z) world position of cell (0, 0, 0).
        * ``resolution``  – the side length of each cubic cell (m).
        * ``shape``       – (nx, ny, nz) number of cells along each axis.

    The Z axis points **upward**; excavation proceeds downward (decreasing Z).
    """

    resolution: float
    grid_origin: Tuple[float, float, float]
    shape: Tuple[int, int, int]                     # (nx, ny, nz)

    # Internal state – populated by __post_init__
    _cells: np.ndarray = field(init=False, repr=False)
    _target_mask: np.ndarray = field(init=False, repr=False)

    # ------------------------------------------------------------------ #
    #  Construction helpers
    # ------------------------------------------------------------------ #
    def __post_init__(self) -> None:
        nx, ny, nz = self.shape
        self._cells = np.full((nx, ny, nz), UNEXCAVATED, dtype=np.int8)
        self._target_mask = np.zeros((nx, ny, nz), dtype=bool)

    @classmethod
    def from_hole_spec(
        cls,
        hole: HoleSpec,
        resolution: float = 0.25,
        margin_cells: int = 2,
    ) -> "ExcavationGrid":
        """Build a grid that covers the hole plus a small margin.

        Parameters
        ----------
        hole : HoleSpec
            Rectangular target volume.
        resolution : float
            Side length of each cubic voxel (m).
        margin_cells : int
            Extra cells added around the hole in X and Y.

        Returns
        -------
        ExcavationGrid
            A new grid with the target mask already set.
        """
        # Number of cells required to cover the hole
        nx_hole = int(np.ceil(hole.size_x / resolution))
        ny_hole = int(np.ceil(hole.size_y / resolution))
        nz_hole = int(np.ceil(hole.depth / resolution))

        nx = nx_hole + 2 * margin_cells
        ny = ny_hole + 2 * margin_cells
        nz = nz_hole + margin_cells          # margin only on top

        # Grid origin sits margin_cells below and to the side of the hole
        gx = hole.origin_x - margin_cells * resolution
        gy = hole.origin_y - margin_cells * resolution
        gz = hole.origin_z - nz * resolution  # bottom of grid

        grid = cls(
            resolution=resolution,
            grid_origin=(gx, gy, gz),
            shape=(nx, ny, nz),
        )

        # Mark the target cells
        x_start = margin_cells
        y_start = margin_cells
        z_start = margin_cells   # lowest layer of the hole in grid coords
        # The hole fills from z_start … z_start + nz_hole - 1  (bottom up)
        grid._target_mask[
            x_start: x_start + nx_hole,
            y_start: y_start + ny_hole,
            z_start: z_start + nz_hole,
        ] = True

        return grid

    # ------------------------------------------------------------------ #
    #  Queries
    # ------------------------------------------------------------------ #
    @property
    def total_target_cells(self) -> int:
        return int(self._target_mask.sum())

    @property
    def excavated_target_cells(self) -> int:
        return int((self._cells[self._target_mask] == EXCAVATED).sum())

    @property
    def remaining_target_cells(self) -> int:
        return self.total_target_cells - self.excavated_target_cells

    @property
    def remaining_volume(self) -> float:
        """Remaining unexcavated volume in m^3."""
        return self.remaining_target_cells * (self.resolution ** 3)

    @property
    def total_target_volume(self) -> float:
        return self.total_target_cells * (self.resolution ** 3)

    @property
    def completion_fraction(self) -> float:
        total = self.total_target_cells
        if total == 0:
            return 1.0
        return self.excavated_target_cells / total

    def cell_centre(self, ix: int, iy: int, iz: int) -> Tuple[float, float, float]:
        """World coordinate of the centre of cell (ix, iy, iz)."""
        gx, gy, gz = self.grid_origin
        half = self.resolution / 2.0
        return (
            gx + ix * self.resolution + half,
            gy + iy * self.resolution + half,
            gz + iz * self.resolution + half,
        )

    def world_to_cell(self, x: float, y: float, z: float) -> Optional[Tuple[int, int, int]]:
        """Convert a world coordinate to a cell index, or None if out of bounds."""
        gx, gy, gz = self.grid_origin
        ix = int(np.floor((x - gx) / self.resolution))
        iy = int(np.floor((y - gy) / self.resolution))
        iz = int(np.floor((z - gz) / self.resolution))
        nx, ny, nz = self.shape
        if 0 <= ix < nx and 0 <= iy < ny and 0 <= iz < nz:
            return (ix, iy, iz)
        return None

    def is_target(self, ix: int, iy: int, iz: int) -> bool:
        return bool(self._target_mask[ix, iy, iz])

    def is_excavated(self, ix: int, iy: int, iz: int) -> bool:
        return self._cells[ix, iy, iz] == EXCAVATED

    # ------------------------------------------------------------------ #
    #  Mutations
    # ------------------------------------------------------------------ #
    def excavate_cells(self, indices: List[Tuple[int, int, int]]) -> int:
        """Mark a list of cells as EXCAVATED.

        Returns the number of newly excavated target cells.
        """
        count = 0
        for ix, iy, iz in indices:
            if self._cells[ix, iy, iz] == UNEXCAVATED:
                self._cells[ix, iy, iz] = EXCAVATED
                if self._target_mask[ix, iy, iz]:
                    count += 1
        return count

    def excavate_flat_indices(self, flat_indices: List[int]) -> int:
        """Mark cells given as flat (ravelled) indices."""
        nx, ny, nz = self.shape
        cells_3d = []
        for fi in flat_indices:
            ix = fi // (ny * nz)
            rem = fi % (ny * nz)
            iy = rem // nz
            iz = rem % nz
            cells_3d.append((ix, iy, iz))
        return self.excavate_cells(cells_3d)

    def reset(self) -> None:
        """Reset all cells to UNEXCAVATED."""
        self._cells[:] = UNEXCAVATED

    # ------------------------------------------------------------------ #
    #  Flat-index helpers (for message interchange)
    # ------------------------------------------------------------------ #
    def target_flat_indices(self) -> np.ndarray:
        """Return flat indices of all target cells."""
        return np.flatnonzero(self._target_mask.ravel())

    def unexcavated_target_flat_indices(self) -> np.ndarray:
        """Return flat indices of target cells that are still UNEXCAVATED."""
        mask = self._target_mask & (self._cells == UNEXCAVATED)
        return np.flatnonzero(mask.ravel())

    # ------------------------------------------------------------------ #
    #  String representation
    # ------------------------------------------------------------------ #
    def __str__(self) -> str:
        return (
            f"ExcavationGrid("
            f"shape={self.shape}, res={self.resolution}m, "
            f"target={self.total_target_cells} cells, "
            f"excavated={self.excavated_target_cells}, "
            f"remaining_vol={self.remaining_volume:.2f}m³)"
        )
