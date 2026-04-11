# Excavation Project — Architecture

## 1. High-Level Overview

The project is a **ROS 2 Jazzy** simulation of a hydraulic excavator that autonomously digs a rectangular hole. There is no physics engine — all motion is kinematic (mock hardware), and the "ground" is a 3D voxel grid. Visualization is via **Foxglove** (WebSocket bridge on port 8765).

The system is split into **5 ROS 2 packages**:

| Package | Language | Role |
|---------|----------|------|
| `excavator_description` | C++/Xacro | URDF robot model |
| `excavator_control` | C++/YAML | ros2_control config + launch |
| `excavation_msgs` | C++ (IDL) | Custom message definitions |
| `excavation_world` | Python | All logic: libraries + ROS nodes |
| `excavator_moveit_config` | C++ | MoveIt 2 config (generated, not used at runtime) |

```
excavation_project/
├── excavator_description/       # URDF + meshes
│   ├── urdf/
│   │   └── excavator.urdf.xacro
│   └── launch/
├── excavator_control/           # ros2_control config + launch
│   ├── config/
│   │   └── excavator_controllers.yaml
│   └── launch/
│       └── excavator_control.launch.py
├── excavation_msgs/             # Custom message types
│   └── msg/
│       ├── ExcavationGrid.msg
│       ├── MissionStatus.msg
│       ├── ScoopAction.msg
│       └── ExcavationPlan.msg
├── excavation_world/            # All Python logic
│   ├── excavation_world/        # Pure-Python libraries + ROS nodes
│   │   ├── robot_model.py
│   │   ├── ik_solver.py
│   │   ├── excavation_grid.py
│   │   ├── excavation_model.py
│   │   ├── scoop_trajectory.py
│   │   ├── base_planner.py
│   │   ├── excavation_planner.py
│   │   ├── mission_controller.py
│   │   ├── world_node.py
│   │   ├── base_motion_node.py
│   │   ├── mission_controller_node.py
│   │   ├── debug_visualizer_node.py
│   │   ├── scoop_executor_node.py
│   │   └── raw_urdf_publisher.py
│   ├── launch/
│   │   └── mission.launch.py
│   └── test/                    # 195 unit tests
│       ├── test_robot_model.py
│       ├── test_ik_solver.py
│       ├── test_excavation_grid.py
│       ├── test_excavation_model.py
│       ├── test_scoop_trajectory.py
│       ├── test_base_planner.py
│       ├── test_excavation_planner.py
│       ├── test_mission_controller.py
│       └── test_debug_visualizer.py
└── excavator_moveit_config/     # MoveIt 2 (generated, not used)
```

---

## 2. The Robot (excavator\_description)

Defined in `excavator_description/urdf/excavator.urdf.xacro`. The kinematic chain:

```
world (fixed)
  └→ base_link (chassis, 0.5m tall)
       └→ cabin_link (cabin_joint: continuous, Z-axis rotation)
            └→ boom_link (boom_joint: revolute [-0.3, 1.2] rad, Y-axis)
                 └→ stick_link (stick_joint: revolute [-2.4, 0.0] rad, Y-axis)
                      └→ bucket_link (bucket_joint: revolute [-1.0, 2.2] rad, Y-axis)
                           └→ bucket_tip (fixed)
```

**Physical dimensions**:

| Link | Size |
|------|------|
| Chassis | 0.5m tall |
| Cabin | 1.0 × 1.0 × 0.8m |
| Boom | 3.0m long |
| Stick | 2.5m long |
| Bucket | 0.8 × 1.0 × 0.5m |

Total arm reach ≈ 6.3m from the cabin pivot.

The URDF includes `<ros2_control>` tags with `mock_components/GenericSystem` — a simulated hardware interface that accepts position commands and echoes them back as state. No physics engine required.

---

## 3. Control Stack (excavator\_control)

Configured in `excavator_control/config/excavator_controllers.yaml`:

- **controller_manager** at 100 Hz
- **joint_state_broadcaster** — reads joint states from hardware, publishes `/joint_states`
- **arm_controller** (`JointTrajectoryController`) — accepts `FollowJointTrajectory` action goals for 4 joints: `cabin_joint`, `boom_joint`, `stick_joint`, `bucket_joint`

The launch file `excavator_control/launch/excavator_control.launch.py` starts:

1. `robot_state_publisher` — URDF → TF tree
2. `ros2_control_node` — controller manager with mock hardware
3. Controller spawners (delayed 8s for controller manager init)
4. `base_motion_node` — drives the base to the working position
5. `foxglove_bridge` — WebSocket visualization server on port 8765

---

## 4. Custom Messages (excavation\_msgs)

| Message | Purpose |
|---------|---------|
| `ExcavationGrid` | Grid dimensions, cell counts, completion fraction |
| `MissionStatus` | State enum (`IDLE`/`MOVING`/`EXCAVATING`/`COMPLETED`/`FAILED`), scoop progress, volume remaining |
| `ScoopAction` | Single scoop: entry/exit poses, waypoints, affected cell indices |
| `ExcavationPlan` | Ordered list of all `ScoopAction` messages |

---

## 5. Core Libraries (ROS-free Python)

All computation is in pure-Python modules under `excavation_world/excavation_world/`. They have **zero ROS dependencies** and are fully unit-testable (195 tests).

### 5.1 robot\_model.py

Kinematic model of the excavator. `ExcavatorModel` stores 4 joint positions + base pose and computes:

- **Forward kinematics** (`fk_chain()`) — full 4×4 homogeneous transform chain from world to bucket tip using rotation matrices (`R_z` for cabin, `R_y` for boom/stick/bucket)
- **`bucket_tip_position()`** — extracts the (x, y, z) world-frame tip position
- **Joint validation** — checks against URDF limits
- Constants for link lengths (`BOOM_LENGTH=3.0`, `STICK_LENGTH=2.5`, `BUCKET_LENGTH=0.8`, etc.)

### 5.2 ik\_solver.py

**Analytical 4-DOF planar IK** for the excavator arm given a target (x, y, z) + desired bucket pitch angle:

1. **Cabin angle** — `atan2(y_local, x_local)` to point the arm plane at the target
2. **Planar reach** — project target into the arm plane, compute distance from boom pivot
3. **2R elbow IK** — law of cosines for boom + stick angles (elbow-up and elbow-down solutions)
4. **Bucket angle** — solve for the bucket joint to achieve the desired world-frame pitch
5. **`solve_ik_nearest()`** — tries multiple bucket pitch angles and both elbow configs, returns the solution closest to the current joint positions
6. **Joint limit validation** — rejects solutions that violate URDF limits

### 5.3 excavation\_grid.py

A 3D numpy voxel grid (`UNEXCAVATED=0` / `EXCAVATED=1`). `HoleSpec` defines the target rectangular volume. `ExcavationGrid.from_hole_spec()` creates the grid, computing an internal `_target_mask` for cells inside the hole. Provides cell-centre ↔ world-coordinate conversions and excavation statistics (remaining volume, completion fraction, etc.).

### 5.4 excavation\_model.py

Bridges scoops and the grid. `ScoopFootprint` (`width=1.0m`, `length=0.8m`, `depth=0.3m`) defines the volume one scoop removes. `compute_scoop_cells()` maps a dig target + footprint → list of `(ix, iy, iz)` grid cells. `apply_scoop_to_grid()` marks those cells as excavated.

### 5.5 scoop\_trajectory.py

Plans a single scoop as **6 waypoints** (joint-space configurations):

| # | Waypoint | Description |
|---|----------|-------------|
| 1 | `ready_start` | Safe position above the dig site |
| 2 | `approach` | Bucket positioned above entry point |
| 3 | `dig` | Bucket at/below the target depth |
| 4 | `scoop` | Bucket curled (joint-space only, no IK) |
| 5 | `lift` | Raise the loaded bucket |
| 6 | `ready_end` | Return to safe position |

Each waypoint (except ready and scoop) is solved via `solve_ik_nearest()`.

**Adaptive fallback**: if the preferred approach/lift height or dig depth fails IK, the planner progressively tries lower/shallower values (4 steps down to 0.1m clearance) before giving up. This recovers targets near the close edge of the hole where joint limits prevent reaching high above ground.

### 5.6 base\_planner.py

Generates a smooth trajectory from start pose to working position. `plan_base_trajectory()` creates a `BaseTrajectory` with time-stamped `(x, y, yaw)` waypoints. The trajectory can be sampled at any time `t` via linear interpolation.

### 5.7 excavation\_planner.py

Decomposes the entire hole into an ordered scoop sequence:

1. **Layers** — top to bottom, step = `footprint.depth × (1 - overlap)`
2. **Sweep** — boustrophedon pattern within each layer (snake-like rows)
3. **X ordering** — nearest-to-robot rows first (clear path for bucket)
4. **Cell association** — each scoop annotated with affected grid cells
5. **Reachability** — optionally pre-checks IK feasibility

Current configuration: 252 planned scoops, 161 IK-reachable from working position (2.0, −0.5).

### 5.8 mission\_controller.py

ROS-free **finite state machine**:

```
IDLE → MOVING_TO_WORK_POS → PLANNING → EXCAVATING → COMPLETED / FAILED
```

`MissionController` class with event-driven transitions:

| Method | Trigger |
|--------|---------|
| `start_mission()` | IDLE → MOVING_TO_WORK_POS |
| `on_base_arrived()` | MOVING → PLANNING |
| `generate_plan()` | PLANNING → EXCAVATING |
| `get_next_scoop()` | Returns next `PlannedScoop` |
| `on_scoop_completed(success)` | Tracks progress, may → COMPLETED/FAILED |
| `abort(reason)` | Any state → FAILED |

Tracks progress via `MissionProgress` (succeeded / failed / total counts).

---

## 6. ROS 2 Nodes

### 6.1 base\_motion\_node

- Plans a trajectory from `(0, 0, 0)` to the working position `(2.0, −0.5, 0.0)`
- Ticks at 50 Hz, sampling the trajectory and broadcasting **`world → base_link` TF**
- Publishes the planned path as `nav_msgs/Path` for visualization
- Publishes `Bool` on `/base_motion/done` — **re-publishes `True` every tick** once arrived (so late-starting nodes don't miss it)

### 6.2 world\_node

The "ground truth" environment:

- Holds the `ExcavationGrid` in memory
- Subscribes to `/excavation/apply_scoop` (`ScoopAction`) — extracts affected cell indices and marks them as excavated
- **Two publish timers**:
  - **Fast (2 Hz)**: publishes `ExcavationGrid` msg on `/excavation/grid_state`
  - **Slow (every 5s)**: republishes static marker arrays (target cubes + hole wireframe) as keepalive for Foxglove
- **Markers** (all `frame_id='world'`):
  - Green cubes: unexcavated target cells (1536 cubes at resolution=0.25 in a 4×3×2m hole)
  - Brown cubes: excavated cells
  - Wireframe: hole boundary frame
  - Yellow sphere: working position
- Publishes a static TF `world → world` via `StaticTransformBroadcaster`

### 6.3 mission\_controller\_node

The main orchestrator — wraps `MissionController` with ROS 2 connectivity:

1. **Waits** for `/base_motion/done == True`
2. **Plans** using `excavation_planner.plan_excavation()`
3. **Excavation loop** (0.5s timer tick):
   - Gets next scoop from the state machine
   - Calls `plan_single_scoop()` for IK trajectory planning
   - If IK fails → skip (mark failed, move to next)
   - If IK succeeds → sends `FollowJointTrajectory` action goal to `arm_controller`
   - **Fully async execution**: `send_goal_async()` + callback chain (no `spin_until_future_complete` — that would deadlock from a timer callback)
   - On action completion → if success, publishes `ScoopAction` to `world_node` to update the grid
4. **Publishes**:
   - `/mission/status` (`MissionStatus`) every tick
   - `/debug/scoop_targets` (`MarkerArray`) — planned scoop positions as spheres
   - `/debug/arm_trajectory` (`MarkerArray`) — `LINE_STRIP` of current scoop waypoints

### 6.4 debug\_visualizer\_node

Passive visualization overlay:

- Subscribes to `/joint_states` → computes bucket tip via FK → accumulates a **trail** (`LINE_STRIP`, up to 2000 points)
- Subscribes to `/mission/status` + `/excavation/grid_state`
- Publishes at 4 Hz:
  - `/debug/bucket_trail` — green line following the bucket tip through space
  - `/debug/status_text` — `TEXT_VIEW_FACING` marker with mission state, scoop count, volume remaining, and current joint angles

---

## 7. ROS 2 Topic & Action Graph

```
                    ┌─────────────────┐
                    │  base_motion_   │
                    │     node        │
                    └──────┬──────────┘
                           │
            /tf (world→base_link)    /base_motion/done (Bool)
                           │                   │
                           ▼                   ▼
┌──────────────────┐   ┌──────────────────────────────┐
│ robot_state_     │   │   mission_controller_node     │
│   publisher      │   │                               │
└──────────────────┘   │ FollowJointTrajectory ──────────────► arm_controller
                       │                               │
                       │ /excavation/apply_scoop ──────────►┌───────────────┐
                       │      (ScoopAction)            │   │  world_node   │
                       │                               │   │               │
                       │ /mission/status ──────────────────►│ /excavation/  │
                       │ /debug/scoop_targets          │   │  grid_state   │
                       │ /debug/arm_trajectory         │   │ /excavation/  │
                       └──────────────────────────────┘   │  markers      │
                                                           │ /excavation/  │
┌──────────────────┐                                       │  target_      │
│ debug_visualizer │◄── /joint_states                      │  markers      │
│     _node        │◄── /mission/status                    └───────────────┘
│                  │◄── /excavation/grid_state
│                  │
│  ► /debug/bucket_trail
│  ► /debug/status_text
└──────────────────┘
                       All MarkerArray topics ───► foxglove_bridge ───► Browser
```

---

## 8. Launch Sequence

`ros2 launch excavation_world mission.launch.py` brings everything up:

| Time | What starts |
|------|-------------|
| t = 0s | `robot_state_publisher`, `ros2_control_node`, `base_motion_node`, `foxglove_bridge`, `world_node`, `debug_visualizer_node` |
| t = 8s | `joint_state_broadcaster` spawner, `arm_controller` spawner |
| t = 14s | `mission_controller_node` (delayed so controllers are ready) |

**Runtime flow**:

1. Base starts driving from `(0, 0)` toward `(2.0, −0.5)`, broadcasting TF
2. Base arrives → publishes `done=True`
3. Mission controller receives done, transitions: `IDLE → MOVING_TO_WORK_POS → PLANNING`
4. Planner generates 252 scoops in boustrophedon layers
5. Controller enters `EXCAVATING`, iterates through scoops:
   - IK solve → build `JointTrajectory` → send action goal → arm moves → scoop applied to grid → next
6. World node updates grid, publishes brown excavated cubes
7. After all scoops attempted → `COMPLETED`

---

## 9. Key Design Decisions

### Pure-Python library / ROS node separation

All computation (FK, IK, grid, planning, state machine) is in **ROS-free Python modules**. The ROS nodes are thin wrappers that wire topics/actions to library calls. This makes the core logic unit-testable without spinning up a ROS graph.

### Analytical IK (not MoveIt)

A 4-DOF planar arm doesn't benefit from sampling-based planners. The analytical solver runs in microseconds and is deterministic. MoveIt config is generated but not used at runtime.

### Mock hardware (no Gazebo)

`mock_components/GenericSystem` echoes position commands as state. This avoids Gazebo's overhead and arm64 compatibility issues while still exercising the full ros2_control + action pipeline.

### Async arm execution

The mission controller sends `FollowJointTrajectory` goals via `send_goal_async()` with callback chains. Using `spin_until_future_complete()` from a timer callback would deadlock the single-threaded executor.

### Adaptive trajectory heights

`plan_single_scoop()` tries progressively lower approach/lift heights and shallower dig depths (4 fallback steps) before rejecting a target. This recovers near-range scoops where joint limits prevent reaching high above ground.

### Split marker timers

Target markers (1536 cubes) are republished every 5s as a keepalive for Foxglove (which uses `VOLATILE` QoS). Grid state is published at 2 Hz. This prevents both marker flicker and excessive bandwidth.

### Base done re-publishing

`base_motion_node` re-publishes `done=True` every tick once arrived, rather than once. The mission controller starts 14s after the base node, so a single publish would be missed.

---

## 10. Configuration Parameters

### Hole specification

| Parameter | Default | Description |
|-----------|---------|-------------|
| `hole_origin_x` | 5.0 | Hole corner X (world frame) |
| `hole_origin_y` | −2.0 | Hole corner Y |
| `hole_origin_z` | 0.0 | Ground surface level |
| `hole_size_x` | 4.0 | Hole width (X direction) |
| `hole_size_y` | 3.0 | Hole width (Y direction) |
| `hole_depth` | 2.0 | Excavation depth |
| `resolution` | 0.25 | Voxel grid cell size (metres) |

### Working position

| Parameter | Default | Description |
|-----------|---------|-------------|
| `goal_x` / `base_x` | 2.0 | Robot working X position |
| `goal_y` / `base_y` | −0.5 | Robot working Y position |
| `goal_yaw` / `base_yaw` | 0.0 | Robot heading at work site |

### Mission controller

| Parameter | Default | Description |
|-----------|---------|-------------|
| `execute_arm` | `true` | `false` for headless/grid-only mode |
| `auto_start` | `true` | Start mission immediately on node startup |
| `scoop_delay` | 0.5 | Seconds between scoops |
| `arm_timeout` | 30.0 | Seconds before arm action times out |

---

## 11. Test Coverage

195 unit tests across 9 test files, all testing the pure-Python libraries without ROS:

| Test file | Tests | Covers |
|-----------|-------|--------|
| `test_robot_model.py` | ~19 | FK chain, joint limits, link lengths |
| `test_ik_solver.py` | ~33 | Analytical IK, round-trip FK ↔ IK, edge cases |
| `test_excavation_grid.py` | ~16 | Grid creation, cell operations, coverage |
| `test_excavation_model.py` | ~19 | Scoop footprint, cell mapping |
| `test_scoop_trajectory.py` | ~22 | Waypoint planning, FK round-trip, duration |
| `test_base_planner.py` | ~23 | Trajectory generation, sampling |
| `test_excavation_planner.py` | ~23 | Plan generation, coverage, ordering |
| `test_mission_controller.py` | ~31 | State transitions, progress tracking |
| `test_debug_visualizer.py` | ~9 | FK trail, trajectory viz, text formatting |

Run all tests:

```bash
cd excavation_world && python3 -m pytest test/ -q
```

---

## 12. How to Run

### Full mission (with arm execution)

```bash
cd /root/ws
colcon build
source install/setup.bash
ros2 launch excavation_world mission.launch.py
```

### Headless mode (grid-only, no arm motion)

```bash
ros2 launch excavation_world mission.launch.py execute_arm:=false
```

### Foxglove visualization

Connect Foxglove Studio to `ws://localhost:8765`. Add a 3D panel and subscribe to:

- `/excavation/target_markers` — green/brown grid cubes
- `/excavation/markers` — excavated cells
- `/debug/bucket_trail` — bucket tip path
- `/debug/status_text` — mission status overlay
- `/debug/scoop_targets` — planned dig positions
- `/debug/arm_trajectory` — current scoop waypoints

> **Tip**: Set the 3D panel's "Follow TF" to `world` (not `base_link`) to keep the camera fixed in world space.
