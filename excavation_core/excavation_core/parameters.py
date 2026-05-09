"""
Central parameter definitions for excavation system.

This module defines all ROS parameter names, default values, and helper
functions for declaring and retrieving parameters. This ensures a single
source of truth and prevents parameter duplication across nodes.

Pattern:
    1. Define parameter constants (PRM_*) with descriptive names
    2. Define default values (DEFAULT_*) to ensure visibility
    3. Use declare_excavation_parameters() in node __init__
    4. Use retrieve_*_parameters() to safely get multiple parameters at once
"""

from dataclasses import dataclass
from typing import Any

from rclpy.node import Node


# ============================================================================
#  EXCAVATION HOLE GEOMETRY (world_node + mission_controller)
# ============================================================================
PRM_RESOLUTION = 'resolution'
DEFAULT_RESOLUTION = 0.25  # meters

PRM_HOLE_ORIGIN_X = 'hole_origin_x'
DEFAULT_HOLE_ORIGIN_X = 10.0

PRM_HOLE_ORIGIN_Y = 'hole_origin_y'
DEFAULT_HOLE_ORIGIN_Y = 10.0

PRM_HOLE_ORIGIN_Z = 'hole_origin_z'
DEFAULT_HOLE_ORIGIN_Z = 0.0

PRM_HOLE_SIZE_X = 'hole_size_x'
DEFAULT_HOLE_SIZE_X = 2.0

PRM_HOLE_SIZE_Y = 'hole_size_y'
DEFAULT_HOLE_SIZE_Y = 2.0

PRM_HOLE_DEPTH = 'hole_depth'
DEFAULT_HOLE_DEPTH = 4

# ============================================================================
#  WORLD NODE SPECIFIC
# ============================================================================
PRM_PUBLISH_RATE = 'publish_rate'
DEFAULT_PUBLISH_RATE = 2.0

PRM_WORKING_POSITION_X = 'working_position_x'
DEFAULT_WORKING_POSITION_X = 2.0

PRM_WORKING_POSITION_Y = 'working_position_y'
DEFAULT_WORKING_POSITION_Y = -0.5

PRM_WORKING_POSITION_Z = 'working_position_z'
DEFAULT_WORKING_POSITION_Z = 0.0

PRM_WORKING_POSITION_YAW = 'working_position_yaw'
DEFAULT_WORKING_POSITION_YAW = 0.0

# ============================================================================
#  MISSION CONTROLLER SPECIFIC
# ============================================================================
PRM_BASE_X = 'base_x'
DEFAULT_BASE_X = 2.0

PRM_BASE_Y = 'base_y'
DEFAULT_BASE_Y = -0.5

PRM_BASE_YAW = 'base_yaw'
DEFAULT_BASE_YAW = 0.0

PRM_EXECUTE_ARM = 'execute_arm'
DEFAULT_EXECUTE_ARM = True

PRM_AUTO_START = 'auto_start'
DEFAULT_AUTO_START = True

PRM_SCOOP_DELAY = 'scoop_delay'
DEFAULT_SCOOP_DELAY = 0.1

PRM_EXECUTION_SPEED = 'execution_speed'
DEFAULT_EXECUTION_SPEED = 50.0

# ============================================================================
#  DEBUG VISUALIZER SPECIFIC
# ============================================================================
PRM_TRAIL_MAX_POINTS = 'trail_max_points'
DEFAULT_TRAIL_MAX_POINTS = 2000

# Reuses: PRM_BASE_X, PRM_BASE_Y, PRM_BASE_YAW, PRM_PUBLISH_RATE

# ============================================================================
#  BASE MOTION NODE SPECIFIC
# ============================================================================
PRM_START_X = 'start_x'
DEFAULT_START_X = 0.0

PRM_START_Y = 'start_y'
DEFAULT_START_Y = 0.0

PRM_START_YAW = 'start_yaw'
DEFAULT_START_YAW = 0.0

PRM_GOAL_X = 'goal_x'
DEFAULT_GOAL_X = 3.0

PRM_GOAL_Y = 'goal_y'
DEFAULT_GOAL_Y = 0.0

PRM_GOAL_YAW = 'goal_yaw'
DEFAULT_GOAL_YAW = 0.0

PRM_LINEAR_SPEED = 'linear_speed'
DEFAULT_LINEAR_SPEED = 0.5

PRM_ANGULAR_SPEED = 'angular_speed'
DEFAULT_ANGULAR_SPEED = 0.3

PRM_SPEED_MULTIPLIER = 'speed_multiplier'
DEFAULT_SPEED_MULTIPLIER = 1.0


def default_hole_geometry() -> "HoleGeometryParameters":
    """Return a :class:`HoleGeometryParameters` built from the module defaults."""
    return HoleGeometryParameters(
        resolution=DEFAULT_RESOLUTION,
        hole_origin_x=DEFAULT_HOLE_ORIGIN_X,
        hole_origin_y=DEFAULT_HOLE_ORIGIN_Y,
        hole_origin_z=DEFAULT_HOLE_ORIGIN_Z,
        hole_size_x=DEFAULT_HOLE_SIZE_X,
        hole_size_y=DEFAULT_HOLE_SIZE_Y,
        hole_depth=DEFAULT_HOLE_DEPTH,
    )


# ============================================================================
#  Data Classes for Type-Safe Parameter Retrieval
# ============================================================================


@dataclass
class HoleGeometryParameters:
    """Shared hole geometry parameters."""
    resolution: float
    hole_origin_x: float
    hole_origin_y: float
    hole_origin_z: float
    hole_size_x: float
    hole_size_y: float
    hole_depth: float

    def to_hole_spec(self) -> "HoleSpec":
        """Build a :class:`HoleSpec` from these parameters (single source of truth)."""
        from excavation_core.excavation_grid import HoleSpec
        return HoleSpec(
            origin_x=self.hole_origin_x,
            origin_y=self.hole_origin_y,
            origin_z=self.hole_origin_z,
            size_x=self.hole_size_x,
            size_y=self.hole_size_y,
            depth=self.hole_depth,
        )


@dataclass
class WorkingPositionParameters:
    """Predefined working position for the excavator base."""
    working_position_x: float
    working_position_y: float
    working_position_z: float
    working_position_yaw: float


@dataclass
class BasePositionParameters:
    """Current robot base position."""
    base_x: float
    base_y: float
    base_yaw: float


@dataclass
class WorldNodeParameters:
    """All parameters required by WorldNode."""
    hole_geometry: HoleGeometryParameters
    working_position: WorkingPositionParameters
    publish_rate: float


@dataclass
class MissionControllerNodeParameters:
    """All parameters required by MissionControllerNode."""
    hole_geometry: HoleGeometryParameters
    base_position: BasePositionParameters
    execute_arm: bool
    auto_start: bool
    scoop_delay: float
    execution_speed: float


@dataclass
class DebugVisualizerNodeParameters:
    """All parameters required by DebugVisualizerNode."""
    base_position: BasePositionParameters
    trail_max_points: int
    publish_rate: float


@dataclass
class BaseMotionNodeParameters:
    """All parameters required by BaseMotionNode."""
    start_x: float
    start_y: float
    start_yaw: float
    goal_x: float
    goal_y: float
    goal_yaw: float
    linear_speed: float
    angular_speed: float
    publish_rate: float
    auto_start: bool
    speed_multiplier: float


# ============================================================================
#  Helper Functions for Initial Declaration
# ============================================================================


def declare_hole_geometry_parameters(node: Node) -> None:
    """Declare all shared hole geometry parameters in one call."""
    node.declare_parameter(PRM_RESOLUTION, DEFAULT_RESOLUTION)
    node.declare_parameter(PRM_HOLE_ORIGIN_X, DEFAULT_HOLE_ORIGIN_X)
    node.declare_parameter(PRM_HOLE_ORIGIN_Y, DEFAULT_HOLE_ORIGIN_Y)
    node.declare_parameter(PRM_HOLE_ORIGIN_Z, DEFAULT_HOLE_ORIGIN_Z)
    node.declare_parameter(PRM_HOLE_SIZE_X, DEFAULT_HOLE_SIZE_X)
    node.declare_parameter(PRM_HOLE_SIZE_Y, DEFAULT_HOLE_SIZE_Y)
    node.declare_parameter(PRM_HOLE_DEPTH, DEFAULT_HOLE_DEPTH)


def declare_working_position_parameters(node: Node) -> None:
    """Declare all working position parameters in one call."""
    node.declare_parameter(PRM_WORKING_POSITION_X, DEFAULT_WORKING_POSITION_X)
    node.declare_parameter(PRM_WORKING_POSITION_Y, DEFAULT_WORKING_POSITION_Y)
    node.declare_parameter(PRM_WORKING_POSITION_Z, DEFAULT_WORKING_POSITION_Z)
    node.declare_parameter(PRM_WORKING_POSITION_YAW, DEFAULT_WORKING_POSITION_YAW)


def declare_base_position_parameters(node: Node) -> None:
    """Declare all base position parameters in one call."""
    node.declare_parameter(PRM_BASE_X, DEFAULT_BASE_X)
    node.declare_parameter(PRM_BASE_Y, DEFAULT_BASE_Y)
    node.declare_parameter(PRM_BASE_YAW, DEFAULT_BASE_YAW)


def declare_world_node_parameters(node: Node) -> None:
    """Declare all parameters needed by WorldNode in one call."""
    declare_hole_geometry_parameters(node)
    declare_working_position_parameters(node)
    node.declare_parameter(PRM_PUBLISH_RATE, DEFAULT_PUBLISH_RATE)


def declare_mission_controller_node_parameters(node: Node) -> None:
    """Declare all parameters needed by MissionControllerNode in one call."""
    declare_hole_geometry_parameters(node)
    declare_base_position_parameters(node)
    node.declare_parameter(PRM_EXECUTE_ARM, DEFAULT_EXECUTE_ARM)
    node.declare_parameter(PRM_AUTO_START, DEFAULT_AUTO_START)
    node.declare_parameter(PRM_SCOOP_DELAY, DEFAULT_SCOOP_DELAY)
    node.declare_parameter(PRM_EXECUTION_SPEED, DEFAULT_EXECUTION_SPEED)


def declare_debug_visualizer_node_parameters(node: Node) -> None:
    """Declare all parameters needed by DebugVisualizerNode in one call."""
    declare_base_position_parameters(node)
    node.declare_parameter(PRM_TRAIL_MAX_POINTS, DEFAULT_TRAIL_MAX_POINTS)
    node.declare_parameter(PRM_PUBLISH_RATE, DEFAULT_PUBLISH_RATE)


def declare_base_motion_node_parameters(node: Node) -> None:
    """Declare all parameters needed by BaseMotionNode in one call."""
    node.declare_parameter(PRM_START_X, DEFAULT_START_X)
    node.declare_parameter(PRM_START_Y, DEFAULT_START_Y)
    node.declare_parameter(PRM_START_YAW, DEFAULT_START_YAW)
    node.declare_parameter(PRM_GOAL_X, DEFAULT_GOAL_X)
    node.declare_parameter(PRM_GOAL_Y, DEFAULT_GOAL_Y)
    node.declare_parameter(PRM_GOAL_YAW, DEFAULT_GOAL_YAW)
    node.declare_parameter(PRM_LINEAR_SPEED, DEFAULT_LINEAR_SPEED)
    node.declare_parameter(PRM_ANGULAR_SPEED, DEFAULT_ANGULAR_SPEED)
    node.declare_parameter(PRM_PUBLISH_RATE, DEFAULT_PUBLISH_RATE)
    node.declare_parameter(PRM_AUTO_START, DEFAULT_AUTO_START)
    node.declare_parameter(PRM_SPEED_MULTIPLIER, DEFAULT_SPEED_MULTIPLIER)


# ============================================================================
#  Helper Functions for Safe Parameter Retrieval
# ============================================================================


def _get_param(node: Node, param_name: str) -> Any:
    """Safely retrieve a single parameter value.
    
    Raises an exception if the parameter was not declared.
    """
    try:
        return node.get_parameter(param_name).value
    except Exception as e:
        raise ValueError(
            f"Parameter '{param_name}' not declared. "
            f"Did you call declare_*_parameters()? Error: {e}"
        ) from e


def retrieve_hole_geometry_parameters(node: Node) -> HoleGeometryParameters:
    """Retrieve all hole geometry parameters with validation."""
    return HoleGeometryParameters(
        resolution=float(_get_param(node, PRM_RESOLUTION)),
        hole_origin_x=float(_get_param(node, PRM_HOLE_ORIGIN_X)),
        hole_origin_y=float(_get_param(node, PRM_HOLE_ORIGIN_Y)),
        hole_origin_z=float(_get_param(node, PRM_HOLE_ORIGIN_Z)),
        hole_size_x=float(_get_param(node, PRM_HOLE_SIZE_X)),
        hole_size_y=float(_get_param(node, PRM_HOLE_SIZE_Y)),
        hole_depth=float(_get_param(node, PRM_HOLE_DEPTH)),
    )


def retrieve_working_position_parameters(node: Node) -> WorkingPositionParameters:
    """Retrieve all working position parameters with validation."""
    return WorkingPositionParameters(
        working_position_x=float(_get_param(node, PRM_WORKING_POSITION_X)),
        working_position_y=float(_get_param(node, PRM_WORKING_POSITION_Y)),
        working_position_z=float(_get_param(node, PRM_WORKING_POSITION_Z)),
        working_position_yaw=float(_get_param(node, PRM_WORKING_POSITION_YAW)),
    )


def retrieve_base_position_parameters(node: Node) -> BasePositionParameters:
    """Retrieve all base position parameters with validation."""
    return BasePositionParameters(
        base_x=float(_get_param(node, PRM_BASE_X)),
        base_y=float(_get_param(node, PRM_BASE_Y)),
        base_yaw=float(_get_param(node, PRM_BASE_YAW)),
    )


def retrieve_world_node_parameters(node: Node) -> WorldNodeParameters:
    """Retrieve all WorldNode parameters with validation."""
    return WorldNodeParameters(
        hole_geometry=retrieve_hole_geometry_parameters(node),
        working_position=retrieve_working_position_parameters(node),
        publish_rate=float(_get_param(node, PRM_PUBLISH_RATE)),
    )


def retrieve_mission_controller_node_parameters(node: Node) -> MissionControllerNodeParameters:
    """Retrieve all MissionControllerNode parameters with validation."""
    return MissionControllerNodeParameters(
        hole_geometry=retrieve_hole_geometry_parameters(node),
        base_position=retrieve_base_position_parameters(node),
        execute_arm=bool(_get_param(node, PRM_EXECUTE_ARM)),
        auto_start=bool(_get_param(node, PRM_AUTO_START)),
        scoop_delay=float(_get_param(node, PRM_SCOOP_DELAY)),
        execution_speed=float(_get_param(node, PRM_EXECUTION_SPEED)),
    )


def retrieve_debug_visualizer_node_parameters(node: Node) -> DebugVisualizerNodeParameters:
    """Retrieve all DebugVisualizerNode parameters with validation."""
    return DebugVisualizerNodeParameters(
        base_position=retrieve_base_position_parameters(node),
        trail_max_points=int(_get_param(node, PRM_TRAIL_MAX_POINTS)),
        publish_rate=float(_get_param(node, PRM_PUBLISH_RATE)),
    )


def retrieve_base_motion_node_parameters(node: Node) -> BaseMotionNodeParameters:
    """Retrieve all BaseMotionNode parameters with validation."""
    return BaseMotionNodeParameters(
        start_x=float(_get_param(node, PRM_START_X)),
        start_y=float(_get_param(node, PRM_START_Y)),
        start_yaw=float(_get_param(node, PRM_START_YAW)),
        goal_x=float(_get_param(node, PRM_GOAL_X)),
        goal_y=float(_get_param(node, PRM_GOAL_Y)),
        goal_yaw=float(_get_param(node, PRM_GOAL_YAW)),
        linear_speed=float(_get_param(node, PRM_LINEAR_SPEED)),
        angular_speed=float(_get_param(node, PRM_ANGULAR_SPEED)),
        publish_rate=float(_get_param(node, PRM_PUBLISH_RATE)),
        auto_start=bool(_get_param(node, PRM_AUTO_START)),
        speed_multiplier=float(_get_param(node, PRM_SPEED_MULTIPLIER)),
    )
