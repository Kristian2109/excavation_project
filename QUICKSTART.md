# Quick Reference: New Package Structure

## Packages at a Glance

| Package | Purpose | Nodes | Dependencies |
|---------|---------|-------|--------------|
| **excavation_core** | Pure Python libraries | None (no ROS) | NumPy |
| **excavation_world** | Grid visualization | `world_node` | excavation_core |
| **excavation_mission** | Mission orchestration | `mission_controller_node`, `scoop_executor_node` | excavation_core |
| **excavation_base_motion** | Base movement | `base_motion_node` | excavation_core |
| **excavation_debug** | Debug overlays | `debug_visualizer_node`, `raw_urdf_publisher` | excavation_core |

---

## Launching

### Full Mission
```bash
ros2 launch excavation_world mission.launch.py
```

### Headless (No Arm)
```bash
ros2 launch excavation_world mission.launch.py execute_arm:=false
```

### Custom Working Position
```bash
ros2 launch excavation_world mission.launch.py \
  goal_x:=3.0 goal_y:=1.0 goal_yaw:=1.57
```

### Individual Subsystems
```bash
ros2 launch excavation_world world.launch.py             # Grid only
ros2 launch excavation_mission mission.launch.py         # Mission only
ros2 launch excavation_debug debug.launch.py             # Visualization only
```

---

## Imports (Python)

### From excavation_core (no ROS)
```python
from excavation_core.excavation_grid import ExcavationGrid, HoleSpec
from excavation_core.robot_model import ExcavatorModel
from excavation_core.parameters import retrieve_hole_geometry_parameters
from excavation_core.scoop_trajectory import plan_single_scoop
```

### From ROS packages
```python
# These require ROS 2 running
from excavation_world.world_node import WorldNode
from excavation_mission.mission_controller_node import MissionControllerNode
from excavation_base_motion.base_motion_node import BaseMotionNode
from excavation_debug.debug_visualizer_node import DebugVisualizerNode
```

---

## File Organization

### To add a computation algorithm:
1. Add to `excavation_core/` (pure Python)
2. Import into ROS node wrapper as needed
3. Add tests to `excavation_core/test/`

### To add a ROS service/action:
1. Create in appropriate `excavation_*` package
2. Create new ROS node class
3. Update `launch/` file if needed

### To add visualization:
1. Add markers to `excavation_debug/debug_visualizer_node.py`
2. Update parameter definitions if needed

---

## Directory Tree

```
excavation_project/
├── excavation_core/              ← Pure Python (no ROS)
│   └── excavation_core/
│       ├── excavation_grid.py
│       ├── excavation_model.py
│       ├── robot_model.py
│       ├── ik_solver.py
│       ├── scoop_trajectory.py
│       ├── base_planner.py
│       ├── excavation_planner.py
│       ├── mission_controller.py
│       └── parameters.py
│
├── excavation_world/             ← Grid visualization
│   ├── excavation_world/
│   │   └── world_node.py
│   └── launch/
│       ├── world.launch.py
│       └── mission.launch.py  (↑ MASTER LAUNCH)
│
├── excavation_mission/           ← Mission execution
│   ├── excavation_mission/
│   │   ├── mission_controller_node.py
│   │   └── scoop_executor_node.py
│   └── launch/
│       └── mission.launch.py
│
├── excavation_base_motion/       ← Base movement
│   ├── excavation_base_motion/
│   │   └── base_motion_node.py
│
└── excavation_debug/             ← Debug viz
    ├── excavation_debug/
    │   ├── debug_visualizer_node.py
    │   └── raw_urdf_publisher.py
    └── launch/
        └── debug.launch.py
```

---

## Testing

### Test excavation_core (no ROS needed)
```bash
cd /root/ws/src/excavation_project/excavation_core
python -m pytest excavation_core/test/
```

### Full system integration test
```bash
cd /root/ws && colcon test
```

---

## Building

### Build all packages
```bash
cd /root/ws && colcon build --symlink-install
```

### Build specific package
```bash
colcon build --packages-select excavation_core
colcon build --packages-select excavation_mission
```

### Build with test
```bash
colcon build --packages-select excavation_core --cmake-args -DBUILD_TESTING=ON
```

---

## Troubleshooting

### Import error: `ModuleNotFoundError: No module named 'excavation_core'`
→ Run `colcon build` first, then source the setup file:
```bash
source /root/ws/install/setup.bash
```

### Can't find `excavation_msgs`
→ Make sure `excavation_msgs` package is built:
```bash
colcon build --packages-select excavation_msgs
```

### ROS nodes not found
→ Verify entry points in `setup.py`:
```bash
colcon list --packages-select <package_name>
```

---

## Parameters

All parameters defined in `excavation_core/parameters.py`:
- Hole geometry: `hole_origin_*, hole_size_*, hole_depth, resolution`
- Working position: `working_position_*`
- Base position: `base_*`
- Node-specific: `publish_rate`, `execute_arm`, `auto_start`, etc.

Override in launch file:
```python
parameters=[{
    'hole_origin_x': 5.0,
    'resolution': 0.25,
    'goal_x': goal_x,
    'execute_arm': execute_arm,
}]
```

---

## Key Files

| File | Purpose |
|------|---------|
| `excavation_world/launch/mission.launch.py` | 🎯 MASTER launch (start here) |
| `excavation_core/parameters.py` | All parameter definitions |
| `excavation_core/excavation_grid.py` | Grid data structure |
| `excavation_core/robot_model.py` | Robot kinematics |
| `excavation_core/excavation_planner.py` | Scoop planning algorithm |
| `excavation_world/world_node.py` | Grid visualization ROS node |
| `excavation_mission/mission_controller_node.py` | Mission orchestration |

---

## Documentation

- 📖 **ARCHITECTURE.md** — System overview and design
- 📖 **PACKAGE_STRUCTURE.md** — Detailed package breakdown
- 📖 **REFACTORING_SUMMARY.md** — What changed and why
- 📖 **This file** — Quick reference guide

---

**For detailed information, see PACKAGE_STRUCTURE.md**
