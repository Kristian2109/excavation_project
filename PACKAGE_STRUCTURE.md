# Excavation Project — Package Structure

## Overview

The refactored excavation project is organized into 5 focused Python packages:

```
excavation_project/
├── excavator_description/          # URDF + hardware (unchanged)
├── excavator_control/               # ros2_control config (unchanged)
├── excavation_msgs/                 # Message definitions (unchanged)
│
├── excavation_core/                 # ⭐ Pure Python libraries (NO ROS)
│   ├── excavation_core/
│   │   ├── __init__.py
│   │   ├── excavation_grid.py       # 3D voxel grid data structure
│   │   ├── excavation_model.py      # Scoop physics & terrain interaction
│   │   ├── robot_model.py           # Excavator kinematics (FK)
│   │   ├── ik_solver.py             # Inverse kinematics solver
│   │   ├── scoop_trajectory.py      # Scoop trajectory generation
│   │   ├── base_planner.py          # Base motion planning (pure geometry)
│   │   ├── excavation_planner.py    # Scoop planning algorithm
│   │   ├── mission_controller.py    # Mission state machine logic
│   │   └── parameters.py            # Shared parameter definitions
│   ├── CMakeLists.txt
│   └── package.xml
│
├── excavation_world/                # Grid state & visualization ROS node
│   ├── excavation_world/
│   │   ├── __init__.py
│   │   └── world_node.py            # Publishes grid markers & state
│   ├── launch/
│   │   ├── world.launch.py          # Standalone world node
│   │   └── mission.launch.py        # Master launch (includes all packages)
│   ├── setup.py
│   └── package.xml
│
├── excavation_base_motion/          # Base motion control ROS node
│   ├── excavation_base_motion/
│   │   ├── __init__.py
│   │   └── base_motion_node.py      # Controls robot movement to work position
│   ├── setup.py
│   └── package.xml
│
├── excavation_mission/              # Mission orchestration ROS nodes
│   ├── excavation_mission/
│   │   ├── __init__.py
│   │   ├── mission_controller_node.py  # Mission state machine (ROS wrapper)
│   │   └── scoop_executor_node.py      # Scoop execution via arm controller
│   ├── launch/
│   │   └── mission.launch.py        # Standalone mission nodes
│   ├── setup.py
│   └── package.xml
│
└── excavation_debug/                # Debug visualization ROS nodes
    ├── excavation_debug/
    │   ├── __init__.py
    │   ├── debug_visualizer_node.py # Bucket trail & status visualization
    │   └── raw_urdf_publisher.py    # Utility for publishing URDF
    ├── launch/
    │   └── debug.launch.py          # Standalone debug visualization
    ├── setup.py
    └── package.xml

excavator_moveit_config/             # MoveIt 2 (generated, not used)
```

---

## Package Responsibilities

### `excavation_core` — Pure Python Libraries

**No ROS dependencies.** Can be imported and used independently in any Python environment.

**Contents**:
- `excavation_grid.py` — 3D array-based terrain representation
- `excavation_model.py` — Scoop-terrain interaction physics
- `robot_model.py` — Robot forward kinematics
- `ik_solver.py` — Inverse kinematics (numerical solver)
- `scoop_trajectory.py` — IK-based trajectory planning
- `base_planner.py` — Base motion planning (waypoint generation)
- `excavation_planner.py` — Algorithm to plan sequence of scoops
- `mission_controller.py` — State machine logic for mission execution
- `parameters.py` — All parameter names, defaults, and retrieval helpers

**Benefits**:
- Testable without ROS (unit tests don't need ROS running)
- Reusable in other projects or contexts
- Clear dependency graph (no circular deps with ROS)

---

### `excavation_world` — Grid State & Visualization Service

**Single ROS node**: `world_node`

**Role**: Maintains the excavation grid state and publishes visualization.

**Published topics**:
- `/excavation/markers` (MarkerArray) — excavated cells (orange cubes)
- `/excavation/target_markers` (MarkerArray) — target volume (blue cubes + frame)
- `/excavation/working_position` (Marker) — predefined working pose (green arrow)
- `/excavation/grid_state` (ExcavationGrid) — grid summary (state message)

**Subscriptions**:
- `/excavation/apply_scoop` (ScoopAction) — receives scoop requests from mission node

**Parameters**: All hole geometry + working position

**Launch files**:
- `world.launch.py` — Standalone (for testing)
- `mission.launch.py` — Master launch (includes all subsystems)

---

### `excavation_base_motion` — Robot Base Motion

**Single ROS node**: `base_motion_node`

**Role**: Simulates robot base movement to a goal pose.

**Published topics**:
- `/base_motion/done` (Bool) — signals when goal is reached
- Transform frames (TF) for base position

**Behavior**: Moves in straight lines at configurable speeds, simulates rotation.

**Parameters**:
- `start_x, start_y, start_yaw` — initial pose
- `goal_x, goal_y, goal_yaw` — target pose
- `linear_speed, angular_speed` — motion parameters
- `auto_start` — whether to start immediately

**Launch files**:
- No standalone launch file (base motion is launched via `excavator_control.launch.py`)

---

### `excavation_mission` — Mission Orchestration

**Two ROS nodes**:

#### `mission_controller_node`
- **Role**: Orchestrates the full mission (state machine)
- **Sequence**: Move base → Plan scoops → Execute scoops
- **Subscribes**: `/base_motion/done`
- **Publishes**: `/mission/status`, `/excavation/apply_scoop`

#### `scoop_executor_node`
- **Role**: Executes scoop trajectories via arm controller
- **Subscribes**: `/joint_states`
- **Publishes**: Arm trajectory commands

**Parameters**:
- All hole geometry + resolution
- Base position
- `scoop_delay`, `arm_timeout` — timing parameters

**Launch files**:
- `mission.launch.py` — Standalone mission

---

### `excavation_debug` — Debug Visualization

**Two ROS nodes**:

#### `debug_visualizer_node`
- **Role**: Publishes visualization overlays (bucket tip trail, status text)
- **Subscribes**: `/joint_states`, `/mission/status`, `/excavation/grid_state`
- **Publishes**: Debug markers

#### `raw_urdf_publisher`
- **Role**: Utility to publish robot URDF (useful for Foxglove)

**Parameters**:
- Base position (for coordinate frame)
- `trail_max_points` — trail smoothing
- `publish_rate` — update frequency

**Launch files**:
- `debug.launch.py` — Standalone visualization

---

## Launch File Hierarchy

### Running the Full System

**Master launch** (includes everything):
```bash
ros2 launch excavation_world mission.launch.py
```

This includes (in order):
1. excavator_control (robot + ros2_control + Foxglove)
2. excavation_base_motion (base movement)
3. excavation_world (grid state)
4. excavation_debug (visualization overlays)
5. excavation_mission (delayed 14s for controllers to spawn)

### Running Individual Subsystems

**World only** (for testing grid logic):
```bash
ros2 launch excavation_world world.launch.py
```

**Mission only** (after other systems are running):
```bash
ros2 launch excavation_mission mission.launch.py
```

**Debug visualization only**:
```bash
ros2 launch excavation_debug debug.launch.py
```

---

## Dependency Graph

```
excavation_core (pure Python, no ROS)
    ↓
    ├─→ excavation_world
    ├─→ excavation_base_motion
    ├─→ excavation_mission
    └─→ excavation_debug
        ↓
        all depend on excavation_msgs (messages)
```

---

## Adding a New Feature

### To add a new planning algorithm:
1. **If it's pure computation**: Add to `excavation_core/excavation_planner.py`
2. **If it needs ROS**: Create a new ROS node in `excavation_mission`
3. **Update tests**: Add tests in `excavation_core/test/`

### To add a new mission type:
1. Add logic to `excavation_core/mission_controller.py`
2. Create a new ROS node wrapper in `excavation_mission/`
3. Add launch file in `excavation_mission/launch/`

### To add visualization:
1. Add marker publishing to `excavation_debug/debug_visualizer_node.py`
2. Update `excavation_debug/launch/debug.launch.py` if needed

---

## Testing

### Unit tests (excavation_core)
```bash
cd /root/ws/src/excavation_project/excavation_core
python -m pytest excavation_core/test/
```

No ROS required — tests run independently.

### Integration tests
```bash
cd /root/ws && colcon test
```

Requires full ROS 2 environment.

---

## Conclusion

This modular structure provides:
- ✅ **Clear separation**: Libraries vs. ROS wrappers
- ✅ **Testability**: Pure Python code testable without ROS
- ✅ **Reusability**: excavation_core can be imported elsewhere
- ✅ **Maintainability**: Each package has one clear job
- ✅ **Scalability**: Easy to extend with new nodes/algorithms
- ✅ **Flexibility**: Run subsystems independently or together
