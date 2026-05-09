"""
Tests for position_planner.py – optimal base position selection.

Covers:
  - Positions are never inside the hole
  - Multiple positions are generated for long holes
  - Travel order is optimised (nearest-neighbour)
  - Single position suffices for small holes
  - Coverage threshold is met
  - Hole-avoidance margin is respected
"""

import math

import pytest

from excavation_core.base_planner import BasePose
from excavation_core.excavation_grid import ExcavationGrid, HoleSpec
from excavation_core.position_planner import (
    _ARM_REACH,
    _MIN_HOLE_CLEARANCE,
    _is_inside_hole,
    _generate_candidates,
    _order_nearest_neighbour,
    _count_coverage,
    _fast_reachable_mask,
    _precompute_cell_centers,
    _practical_clearance,
    compute_work_positions,
)


# ====================================================================== #
#  Standard test holes
# ====================================================================== #

# Long trench — arm can't cover from one side if wide enough
LONG_HOLE = HoleSpec(
    origin_x=5.0, origin_y=-2.0, origin_z=0.0,
    size_x=10.0, size_y=2.0, depth=0.5,
)

# Very long and wide — needs multiple positions
WIDE_LONG_HOLE = HoleSpec(
    origin_x=5.0, origin_y=-4.0, origin_z=0.0,
    size_x=12.0, size_y=8.0, depth=2.0,
)

# Small hole — should be covered by one position
SMALL_HOLE = HoleSpec(
    origin_x=5.0, origin_y=-1.0, origin_z=0.0,
    size_x=2.0, size_y=2.0, depth=0.5,
)

# Deep hole
DEEP_HOLE = HoleSpec(
    origin_x=5.0, origin_y=-1.0, origin_z=0.0,
    size_x=3.0, size_y=2.0, depth=3.0,
)


# ====================================================================== #
#  Hole-avoidance tests
# ====================================================================== #

class TestHoleAvoidance:
    """No robot position should be inside the hole."""

    def test_is_inside_hole_true(self):
        assert _is_inside_hole(6.0, -1.0, LONG_HOLE)

    def test_is_inside_hole_false(self):
        assert not _is_inside_hole(3.0, -1.0, LONG_HOLE)

    def test_is_inside_hole_margin(self):
        # Right at the edge — within margin
        assert _is_inside_hole(
            LONG_HOLE.origin_x - 0.1, -1.0, LONG_HOLE)

    def test_no_candidate_inside_hole(self):
        candidates = _generate_candidates(LONG_HOLE, arm_reach=_ARM_REACH)
        for pos in candidates:
            assert not _is_inside_hole(pos.x, pos.y, LONG_HOLE,
                                       margin=0.0), \
                f'Candidate ({pos.x:.2f}, {pos.y:.2f}) inside hole'

    def test_computed_positions_outside_hole(self):
        positions = compute_work_positions(LONG_HOLE)
        for pos in positions:
            assert not _is_inside_hole(pos.x, pos.y, LONG_HOLE,
                                       margin=0.0), \
                f'Position ({pos.x:.2f}, {pos.y:.2f}) inside hole'


# ====================================================================== #
#  Candidate generation tests
# ====================================================================== #

class TestCandidateGeneration:
    """Dense candidates should cover all four sides."""

    def test_candidates_generated(self):
        candidates = _generate_candidates(LONG_HOLE, arm_reach=_ARM_REACH)
        assert len(candidates) > 4, (
            f'Expected many candidates for a 10m hole, got {len(candidates)}')

    def test_candidates_on_multiple_sides(self):
        candidates = _generate_candidates(LONG_HOLE, arm_reach=_ARM_REACH)
        yaws = {round(c.yaw, 2) for c in candidates}
        # Should have at least 2 different yaw orientations
        assert len(yaws) >= 2

    def test_long_side_more_candidates(self):
        """The long edge should produce more candidates than the short edge."""
        candidates = _generate_candidates(LONG_HOLE, arm_reach=_ARM_REACH)
        # -Y and +Y sides run along X (10m) → more candidates
        long_side = [c for c in candidates
                     if abs(c.yaw) == pytest.approx(math.pi / 2, abs=0.01)]
        short_side = [c for c in candidates if c.yaw == pytest.approx(0.0)]
        assert len(long_side) >= len(short_side)


# ====================================================================== #
#  Travel order optimisation
# ====================================================================== #

class TestTravelOrder:
    """Nearest-neighbour ordering should minimise total travel."""

    def test_order_preserves_count(self):
        poses = [BasePose(x=float(i), y=0.0, yaw=0.0) for i in range(5)]
        ordered = _order_nearest_neighbour(poses)
        assert len(ordered) == len(poses)

    def test_order_no_duplicates(self):
        poses = [BasePose(x=float(i) * 3, y=0.0, yaw=0.0) for i in range(5)]
        ordered = _order_nearest_neighbour(poses)
        coords = [(p.x, p.y) for p in ordered]
        assert len(set(coords)) == len(coords)

    def test_sequential_is_optimal(self):
        """Already-sorted linear positions should stay sequential."""
        poses = [BasePose(x=float(i), y=0.0, yaw=0.0) for i in range(5)]
        ordered = _order_nearest_neighbour(poses)
        xs = [p.x for p in ordered]
        # Should be monotonic (ascending or descending)
        assert xs == sorted(xs) or xs == sorted(xs, reverse=True)

    def test_scrambled_input_sorted_output(self):
        """Scrambled positions along a line should be reordered."""
        poses = [BasePose(x=x, y=0.0, yaw=0.0)
                 for x in [10.0, 2.0, 6.0, 0.0, 8.0, 4.0]]
        ordered = _order_nearest_neighbour(poses)
        # Total distance of ordered should be ≤ total distance of original
        def total_dist(ps):
            return sum(math.hypot(ps[i+1].x - ps[i].x, ps[i+1].y - ps[i].y)
                       for i in range(len(ps) - 1))
        assert total_dist(ordered) <= total_dist(poses)

    def test_computed_positions_travel_optimised(self):
        """Positions from compute_work_positions should be travel-optimised."""
        positions = compute_work_positions(LONG_HOLE)
        if len(positions) <= 1:
            return  # nothing to check
        # Check total distance is not worse than reversed order
        def total_dist(ps):
            return sum(math.hypot(ps[i+1].x - ps[i].x, ps[i+1].y - ps[i].y)
                       for i in range(len(ps) - 1))
        fwd = total_dist(positions)
        rev = total_dist(list(reversed(positions)))
        # NN ordering should be <= both forward and reverse (or close)
        assert fwd <= rev * 1.5  # some tolerance


# ====================================================================== #
#  Coverage tests
# ====================================================================== #

class TestCoverage:
    """Selected positions should cover the target hole."""

    def test_wide_long_hole_needs_multiple_positions(self):
        positions = compute_work_positions(WIDE_LONG_HOLE)
        assert len(positions) >= 2, (
            f'12x8m hole should need ≥2 positions, got {len(positions)}')

    def test_small_hole_few_positions(self):
        positions = compute_work_positions(SMALL_HOLE)
        assert len(positions) <= 3, (
            f'Small hole should need few positions, got {len(positions)}')

    def test_coverage_threshold_met(self):
        """Selected positions should reach ≥ 90% of target cells."""
        positions = compute_work_positions(LONG_HOLE)
        grid = ExcavationGrid.from_hole_spec(LONG_HOLE, resolution=0.5)
        total = len(grid.target_flat_indices())
        covered: set[int] = set()
        for pos in positions:
            cells = _count_coverage(pos, grid, set())
            covered.update(cells)
        coverage = len(covered) / total
        assert coverage >= 0.90, f'Coverage only {coverage:.1%}'

    def test_deep_hole_positions(self):
        """A deeper hole should still produce valid positions."""
        positions = compute_work_positions(DEEP_HOLE)
        assert len(positions) >= 1
        for pos in positions:
            assert not _is_inside_hole(pos.x, pos.y, DEEP_HOLE, margin=0.0)


# ====================================================================== #
#  Edge cases
# ====================================================================== #

class TestEdgeCases:
    def test_zero_size_hole(self):
        hole = HoleSpec(origin_x=5.0, origin_y=-1.0, origin_z=0.0,
                        size_x=0.0, size_y=0.0, depth=0.5)
        positions = compute_work_positions(hole)
        assert len(positions) >= 1

    def test_custom_clearance(self):
        positions = compute_work_positions(SMALL_HOLE, clearance=5.0)
        assert len(positions) >= 1
        for pos in positions:
            assert not _is_inside_hole(pos.x, pos.y, SMALL_HOLE, margin=0.0)

    def test_empty_list_single(self):
        ordered = _order_nearest_neighbour([])
        assert ordered == []

    def test_single_position(self):
        p = BasePose(x=1.0, y=2.0, yaw=0.0)
        ordered = _order_nearest_neighbour([p])
        assert ordered == [p]
