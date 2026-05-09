"""
Unit tests for ExcavationGrid (no ROS required).

Maps to the Key Tests Checklist:
  Test 1: The scene initialises correctly – grid, target hole, volumes.
"""

import pytest
import numpy as np

from excavation_core.excavation_grid import (
    ExcavationGrid,
    HoleSpec,
)


# --------------------------------------------------------------------- #
#  Fixtures
# --------------------------------------------------------------------- #

@pytest.fixture
def default_hole() -> HoleSpec:
    """A 4 × 3 × 2  m rectangular hole at world position (5, -2, 0)."""
    return HoleSpec(
        origin_x=5.0,
        origin_y=-2.0,
        origin_z=0.0,
        size_x=4.0,
        size_y=3.0,
        depth=2.0,
    )


@pytest.fixture
def grid(default_hole) -> ExcavationGrid:
    return ExcavationGrid.from_hole_spec(default_hole, resolution=0.5)


# --------------------------------------------------------------------- #
#  Test 1: Grid initialisation
# --------------------------------------------------------------------- #

class TestGridInitialisation:
    def test_shape_covers_hole(self, grid: ExcavationGrid, default_hole: HoleSpec):
        """Grid shape must be large enough to contain the target hole."""
        nx, ny, nz = grid.shape
        assert nx * grid.resolution >= default_hole.size_x
        assert ny * grid.resolution >= default_hole.size_y
        assert nz * grid.resolution >= default_hole.depth

    def test_target_cell_count(self, grid: ExcavationGrid, default_hole: HoleSpec):
        """Number of target cells should match the expected volume."""
        res = grid.resolution
        expected_nx = int(np.ceil(default_hole.size_x / res))
        expected_ny = int(np.ceil(default_hole.size_y / res))
        expected_nz = int(np.ceil(default_hole.depth / res))
        expected_total = expected_nx * expected_ny * expected_nz
        assert grid.total_target_cells == expected_total

    def test_initial_state_all_unexcavated(self, grid: ExcavationGrid):
        """Before any action, all cells should be UNEXCAVATED."""
        assert grid.excavated_target_cells == 0
        assert grid.remaining_target_cells == grid.total_target_cells

    def test_total_target_volume(self, grid: ExcavationGrid, default_hole: HoleSpec):
        """Total target volume should approximately equal the hole volume."""
        hole_volume = default_hole.size_x * default_hole.size_y * default_hole.depth
        # Grid approximation may differ slightly due to ceiling
        assert abs(grid.total_target_volume - hole_volume) < (grid.resolution ** 3) * 10

    def test_completion_starts_at_zero(self, grid: ExcavationGrid):
        assert grid.completion_fraction == pytest.approx(0.0)


# --------------------------------------------------------------------- #
#  Test: Cell coordinate conversions
# --------------------------------------------------------------------- #

class TestCoordinateConversion:
    def test_cell_centre_round_trip(self, grid: ExcavationGrid):
        """world_to_cell(cell_centre(i,j,k)) should return (i,j,k)."""
        nx, ny, nz = grid.shape
        for ix in range(0, nx, max(1, nx // 3)):
            for iy in range(0, ny, max(1, ny // 3)):
                for iz in range(0, nz, max(1, nz // 3)):
                    cx, cy, cz = grid.cell_centre(ix, iy, iz)
                    result = grid.world_to_cell(cx, cy, cz)
                    assert result == (ix, iy, iz), f'Failed at ({ix},{iy},{iz})'

    def test_out_of_bounds_returns_none(self, grid: ExcavationGrid):
        assert grid.world_to_cell(-9999, -9999, -9999) is None


# --------------------------------------------------------------------- #
#  Test: Excavation mutations
# --------------------------------------------------------------------- #

class TestExcavation:
    def test_excavate_decreases_remaining(self, grid: ExcavationGrid):
        """Excavating a target cell should reduce the remaining count."""
        before = grid.remaining_target_cells
        # Find the first target cell
        indices = grid.target_flat_indices()
        assert len(indices) > 0
        grid.excavate_flat_indices([int(indices[0])])
        assert grid.remaining_target_cells == before - 1

    def test_excavate_increases_completion(self, grid: ExcavationGrid):
        indices = grid.target_flat_indices()
        grid.excavate_flat_indices([int(i) for i in indices])
        assert grid.completion_fraction == pytest.approx(1.0)

    def test_excavate_all_leaves_zero_remaining(self, grid: ExcavationGrid):
        indices = grid.target_flat_indices()
        grid.excavate_flat_indices([int(i) for i in indices])
        assert grid.remaining_target_cells == 0
        assert grid.remaining_volume == pytest.approx(0.0)

    def test_double_excavate_is_idempotent(self, grid: ExcavationGrid):
        """Excavating the same cell twice should not double-count."""
        indices = grid.target_flat_indices()
        fi = int(indices[0])
        grid.excavate_flat_indices([fi])
        remaining_after_first = grid.remaining_target_cells
        grid.excavate_flat_indices([fi])
        assert grid.remaining_target_cells == remaining_after_first

    def test_reset(self, grid: ExcavationGrid):
        indices = grid.target_flat_indices()
        grid.excavate_flat_indices([int(i) for i in indices[:5]])
        grid.reset()
        assert grid.excavated_target_cells == 0


# --------------------------------------------------------------------- #
#  Test: Different resolutions
# --------------------------------------------------------------------- #

class TestResolutionVariants:
    @pytest.mark.parametrize('res', [0.1, 0.25, 0.5, 1.0])
    def test_varying_resolution(self, default_hole, res):
        grid = ExcavationGrid.from_hole_spec(default_hole, resolution=res)
        assert grid.total_target_cells > 0
        assert grid.remaining_volume > 0
        # Volume should approximately equal hole volume
        hole_vol = default_hole.size_x * default_hole.size_y * default_hole.depth
        assert abs(grid.total_target_volume - hole_vol) < hole_vol * 0.2
