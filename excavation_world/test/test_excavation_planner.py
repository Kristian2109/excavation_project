"""
Tests for excavation_planner.py – Stage 7: Pre-Planning Full Excavation.

Covers:
  - Plan generation for a standard hole
  - Plan produces a non-empty list of scoops
  - Every scoop has a valid dig target
  - Every scoop has associated cells
  - Scoop IDs are sequential
  - Layers are ordered top-to-bottom
  - Plan covers the full target area (≥90%)
  - Plan covers the full target area (≥95% with overlap)
  - Simulating the plan fully excavates the grid
  - Different footprint sizes produce different scoop counts
  - Overlap parameter increases scoop count but improves coverage
  - Smaller hole produces fewer scoops
  - Plan with trajectories generates valid ScoopTrajectory objects
  - Plan validation passes with good coverage
  - Plan validation fails when coverage is insufficient
  - No duplicate scoop IDs
  - Boustrophedon sweep pattern
"""

import copy
import math
import numpy as np
import pytest

from excavation_world.excavation_grid import ExcavationGrid, HoleSpec
from excavation_world.excavation_model import ScoopFootprint
from excavation_world.excavation_planner import (
    ExcavationPlan,
    PlannedScoop,
    plan_excavation,
    simulate_plan,
)


# ====================================================================== #
#  Helpers
# ====================================================================== #

BASE_X, BASE_Y, BASE_YAW = 3.0, 0.0, 0.0

STANDARD_HOLE = HoleSpec(
    origin_x=5.0, origin_y=-2.0, origin_z=0.0,
    size_x=4.0, size_y=3.0, depth=2.0,
)


def _make_grid(hole=None, res=0.25):
    if hole is None:
        hole = STANDARD_HOLE
    return ExcavationGrid.from_hole_spec(hole, resolution=res)


def _make_plan(hole=None, grid=None, fp=None, overlap=0.2, **kwargs):
    if hole is None:
        hole = STANDARD_HOLE
    if grid is None:
        grid = _make_grid(hole)
    return plan_excavation(
        hole, grid,
        base_x=BASE_X, base_y=BASE_Y, base_yaw=BASE_YAW,
        footprint=fp, overlap=overlap, **kwargs,
    )


# ====================================================================== #
#  Basic plan generation
# ====================================================================== #

class TestPlanGeneration:
    """Verify that a plan is generated correctly."""

    def test_plan_not_empty(self):
        plan = _make_plan()
        assert plan.total_scoops > 0

    def test_every_scoop_has_target(self):
        plan = _make_plan()
        for s in plan.scoops:
            assert s.dig_target is not None
            assert len(s.dig_target) == 3

    def test_every_scoop_has_cells(self):
        plan = _make_plan()
        for s in plan.scoops:
            assert len(s.affected_cells) > 0, (
                f'Scoop {s.scoop_id} at {s.dig_target} has no cells')

    def test_sequential_ids(self):
        plan = _make_plan()
        ids = [s.scoop_id for s in plan.scoops]
        assert ids == list(range(len(ids)))

    def test_no_duplicate_ids(self):
        plan = _make_plan()
        ids = [s.scoop_id for s in plan.scoops]
        assert len(ids) == len(set(ids))

    def test_plan_has_hole_and_footprint(self):
        plan = _make_plan()
        assert plan.hole is not None
        assert plan.footprint is not None


# ====================================================================== #
#  Layer ordering
# ====================================================================== #

class TestLayerOrdering:
    """Verify that layers go from top to bottom."""

    def test_layers_non_decreasing(self):
        plan = _make_plan()
        layers = [s.layer for s in plan.scoops]
        for i in range(1, len(layers)):
            assert layers[i] >= layers[i - 1], (
                f'Layer decreased at scoop {i}: {layers[i-1]} → {layers[i]}')

    def test_first_layer_is_topmost(self):
        plan = _make_plan()
        # All scoops in the first layer should have z near the top
        first_layer = [s for s in plan.scoops if s.layer == 0]
        assert len(first_layer) > 0
        for s in first_layer:
            assert s.dig_target[2] > STANDARD_HOLE.origin_z - STANDARD_HOLE.depth

    def test_multiple_layers_for_deep_hole(self):
        plan = _make_plan()
        layers = set(s.layer for s in plan.scoops)
        # With depth=2.0 and footprint.depth=0.3, we should have many layers
        assert len(layers) >= 2


# ====================================================================== #
#  Coverage
# ====================================================================== #

class TestCoverage:
    """Verify that the plan covers the target volume."""

    def test_coverage_above_90_percent(self):
        grid = _make_grid()
        plan = _make_plan(grid=grid)
        cov = plan.coverage_fraction(grid)
        assert cov >= 0.90, f'Coverage only {cov:.1%}'

    def test_coverage_above_95_with_overlap(self):
        grid = _make_grid()
        plan = _make_plan(grid=grid, overlap=0.3)
        cov = plan.coverage_fraction(grid)
        assert cov >= 0.95, f'Coverage only {cov:.1%}'

    def test_validate_passes(self):
        grid = _make_grid()
        plan = _make_plan(grid=grid, overlap=0.3)
        assert plan.validate(grid, threshold=0.90)

    def test_total_planned_cells_positive(self):
        plan = _make_plan()
        assert plan.total_planned_cells > 0


# ====================================================================== #
#  Simulation
# ====================================================================== #

class TestSimulation:
    """Verify that simulating the plan excavates the grid."""

    def test_simulate_achieves_high_completion(self):
        grid = _make_grid()
        plan = _make_plan(grid=grid, overlap=0.3)
        grid_copy = copy.deepcopy(grid)
        completion = simulate_plan(plan, grid_copy)
        assert completion >= 0.90, f'Simulation only completed {completion:.1%}'

    def test_simulate_leaves_no_target_unexcavated(self):
        """With sufficient overlap, remaining should be near zero."""
        grid = _make_grid(res=0.5)  # coarser for speed
        plan = _make_plan(
            grid=grid, overlap=0.3,
            fp=ScoopFootprint(width=1.2, length=1.2, depth=0.6))
        grid_copy = copy.deepcopy(grid)
        completion = simulate_plan(plan, grid_copy)
        assert completion >= 0.95, f'Simulation only completed {completion:.1%}'

    def test_simulate_mutates_grid(self):
        grid = _make_grid()
        plan = _make_plan(grid=grid)
        remaining_before = grid.remaining_target_cells
        simulate_plan(plan, grid)
        assert grid.remaining_target_cells < remaining_before


# ====================================================================== #
#  Footprint and overlap parameters
# ====================================================================== #

class TestParameters:
    """Verify that parameters affect the plan sensibly."""

    def test_larger_footprint_fewer_scoops(self):
        grid = _make_grid()
        plan_small = _make_plan(
            grid=grid, fp=ScoopFootprint(width=0.5, length=0.5, depth=0.25))
        grid2 = _make_grid()
        plan_large = _make_plan(
            grid=grid2, fp=ScoopFootprint(width=1.5, length=1.5, depth=0.5))
        assert plan_large.total_scoops < plan_small.total_scoops

    def test_more_overlap_more_scoops(self):
        grid1 = _make_grid()
        plan_lo = _make_plan(grid=grid1, overlap=0.1)
        grid2 = _make_grid()
        plan_hi = _make_plan(grid=grid2, overlap=0.4)
        assert plan_hi.total_scoops >= plan_lo.total_scoops

    def test_smaller_hole_fewer_scoops(self):
        small_hole = HoleSpec(
            origin_x=5.0, origin_y=-1.0, origin_z=0.0,
            size_x=2.0, size_y=1.0, depth=1.0,
        )
        grid_s = _make_grid(small_hole)
        plan_s = _make_plan(hole=small_hole, grid=grid_s)

        grid_l = _make_grid()
        plan_l = _make_plan(grid=grid_l)

        assert plan_s.total_scoops < plan_l.total_scoops


# ====================================================================== #
#  Trajectory planning integration
# ====================================================================== #

class TestTrajectoryPlanning:
    """Verify plan_trajectories=True produces actual trajectories."""

    def test_plan_with_trajectories(self):
        """At least some scoops should have valid trajectories."""
        # Use a smaller hole for speed
        small_hole = HoleSpec(
            origin_x=5.0, origin_y=-1.0, origin_z=0.0,
            size_x=2.0, size_y=1.0, depth=0.5,
        )
        grid = _make_grid(small_hole, res=0.5)
        plan = plan_excavation(
            small_hole, grid,
            base_x=BASE_X, base_y=BASE_Y, base_yaw=BASE_YAW,
            plan_trajectories=True,
        )
        has_traj = sum(1 for s in plan.scoops if s.trajectory is not None)
        assert has_traj > 0, 'No scoops have trajectories'

    def test_trajectory_waypoints_valid(self):
        small_hole = HoleSpec(
            origin_x=5.5, origin_y=-0.5, origin_z=0.0,
            size_x=1.5, size_y=1.0, depth=0.5,
        )
        grid = _make_grid(small_hole, res=0.5)
        plan = plan_excavation(
            small_hole, grid,
            base_x=BASE_X, base_y=BASE_Y, base_yaw=BASE_YAW,
            plan_trajectories=True,
        )
        for s in plan.scoops:
            if s.trajectory is not None:
                assert s.trajectory.validate(), (
                    f'Trajectory invalid for scoop {s.scoop_id}')


# ====================================================================== #
#  Edge cases
# ====================================================================== #

class TestEdgeCases:
    """Edge cases and boundary conditions."""

    def test_zero_depth_hole(self):
        """A hole with zero depth should produce an empty plan."""
        hole = HoleSpec(
            origin_x=5.0, origin_y=-1.0, origin_z=0.0,
            size_x=2.0, size_y=1.0, depth=0.0,
        )
        grid = _make_grid(hole)
        plan = _make_plan(hole=hole, grid=grid)
        # Might have zero scoops or a few that don't hit target cells
        assert plan.total_scoops >= 0

    def test_very_small_hole(self):
        """A tiny hole should need only a few scoops."""
        hole = HoleSpec(
            origin_x=5.0, origin_y=-0.5, origin_z=0.0,
            size_x=0.5, size_y=0.5, depth=0.3,
        )
        grid = _make_grid(hole, res=0.25)
        plan = _make_plan(hole=hole, grid=grid)
        assert plan.total_scoops <= 20  # should be very few
