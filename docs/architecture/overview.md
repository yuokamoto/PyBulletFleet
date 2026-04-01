# PyBulletFleet - Design Documentation

## Architecture Overview

This package provides a modular, reusable PyBullet simulation framework designed for multi-agent scenarios. The architecture is organized into several key components, each with specific responsibilities.

> **Note:** The methods, attributes, and parameters listed in this document are representative highlights — not exhaustive lists. Refer to the source code or API reference for the full interface of each class.

```
┌─────────────────────────────────────────────────────────────┐
│                    User Application                          │
└───────────────────┬─────────────────────────────────────────┘
                    │
    ┌───────────────┴───────────────┐
    │  MultiRobotSimulationCore     │  ← Main simulation engine
    │  (core_simulation.py)         │
    └───────────────┬───────────────┘
                    │
    ┌───────────────┼───────────────┬───────────────┬──────────────┐
    │               │               │               │              │
┌───▼───┐    ┌─────▼──────┐  ┌────▼────┐    ┌────▼────┐   ┌────▼────┐
│ Agent │    │ Agent      │  │ Action  │    │ Tools   │   │Visualizer│
│       │    │ Manager    │  │ System  │    │ (utils) │   │ Monitor │
└───────┘    └────────────┘  └─────────┘    └─────────┘   └─────────┘
```

## Core Components

### 1. core_simulation.py

**Purpose**: Main simulation engine and foundational classes

#### Key Classes:

##### MultiRobotSimulationCore
The central orchestrator for PyBullet simulations.

**Responsibilities:**
- PyBullet engine initialization and configuration
- Simulation loop management (timestep control, speed multiplier)
- Camera setup (manual/automatic positioning)
- Visualization control (visual shapes, collision shapes, transparency)
- Performance monitoring integration
- Structure body tracking
- Callback management for user-defined updates
- Keyboard event handling (SPACE, v, c, t keys)
- Collision detection system integration

**Key Methods:**
- `from_dict(config)` / `from_yaml(path)`: Factory methods for initialization
- `run_simulation(update_callback, final_callback)`: Main simulation loop
- `step_once()`: Single simulation step
- `setup_camera()`: Camera positioning
- `configure_visualizer()`: Visual settings configuration
- `register_static_body(body_id)`: Track static structure elements
- `register_callback(callback, frequency)`: Register custom update callbacks
- `_handle_keyboard_events()`: Process keyboard inputs

**Associated Params:**
- **`SimulationParams`** — Configuration dataclass holding all parameters for `MultiRobotSimulationCore`. Passed to the constructor to configure the simulation engine.
  - Attributes: `gui`, `timestep`, `target_rtf`, `duration` (core settings), `physics`, `monitor` (feature toggles), `enable_floor` (plane.urdf loading, default `True`), `camera_*` (camera config), `enable_*` (visualization), `spatial_hash_*` (collision detection)
  - Creation: `SimulationParams(gui=False, target_rtf=0, ...)`, `SimulationParams.from_dict(config)`, `SimulationParams.from_config("config/config.yaml")`
  - `enable_floor=False` skips loading `plane.urdf` in both `setup_pybullet()` and `reset()`, allowing custom floor handling (e.g., transparent floors, environment SDF meshes)

##### SimObject
Base class for all simulation objects (single rigid body, no joints or links).

SimObject represents a **single-body** object in PyBullet — it has only a base link
and does **not** support joints, links, or URDF loading. For objects that require
multi-link bodies with joint control (e.g., URDF robots), use Agent instead.

**Responsibilities:**
- Common interface for objects in simulation
- Position and orientation management via Pose
- Metadata storage
- Object attachment system (base-link attachment only)
- Shared shape caching for performance

**Key Methods:**
- `from_params(spawn_params)` / `from_mesh(...)`: Factory methods for creation
- `get_pose()` / `set_pose(pose)`: Position and orientation management
- `set_collision_mode(mode)`: Change collision detection mode
- `attach_object(obj)` / `detach_object(obj)`: Parent-child attachment with constraints
- `get_attached_objects()` / `is_attached()`: Query attachment state
- `register_callback(callback, frequency)`: Register custom update callbacks

**Key Features:**
- Support for mesh and primitive shapes (created via `createMultiBody`)
- Collision and visual shape separation
- Pickable/non-pickable objects
- Parent-child attachment with constraints
- No joint/link support — single rigid body only

**Associated Params:**

- `SimObjectSpawnParams` — Parameters for spawning a SimObject (visual/collision shapes, initial pose, mass, pickable, collision mode, name, user_data). Pass to `SimObject.from_params()`.
- `ShapeParams` — Visual or collision shape definition (shape type, mesh path, half extents, radius, colour, frame offset). Referenced by `SimObjectSpawnParams.visual_shape` and `.collision_shape`.

##### LogLevelManager
Utility for managing PyBullet log verbosity.

**Key Methods:**
- `set_log_level(level)`: Control PyBullet logging output

---

### 2. agent.py

**Purpose**: Agent with goal-based navigation and action system

#### Key Classes:

##### Agent (extends SimObject)

Agent extends SimObject to support **URDF loading with multi-link bodies and joint control**.
While SimObject is limited to single rigid bodies, Agent can load URDF models that contain
multiple links connected by joints, and provides joint state management and link-level
object attachment via `update_attached_objects_kinematics()`.

**Responsibilities:**
- Goal-based navigation (move towards target pose)
- Action execution (MoveTo, Pick, Drop, Wait)
- Velocity and acceleration limiting
- Path following
- Object manipulation (supports link-level attachment for URDF robots)
- Collision handling
- URDF model loading with joint/link support
- Model name resolution — `from_urdf()` calls `resolve_urdf()` internally, accepting both model names (e.g., `"panda"`) and direct file paths

**Key Methods:**
- `from_urdf(urdf_path, ...)`: Factory method — accepts a model name (resolved via `resolve_urdf()`) or a direct URDF path
- `set_goal(pose)`: Set target destination
- `update(dt)`: Update agent state per timestep
- `execute_action(action)`: Execute high-level action
- `is_goal_reached()`: Check if at destination
- `pick(obj)`: Attach object to agent
- `drop()`: Detach currently held object

**Motion Modes:**
- Omnidirectional: Move in any direction without rotation
- Differential: Rotate towards goal then move forward

**Control Algorithm:**
- Proportional controller for position
- Linear interpolation for smooth motion
- Velocity clamping based on max_linear_vel and max_linear_accel

**Joint Control Modes:**
- **Physics mode** (`mass > 0`, `physics=True`): `setJointMotorControl2` — PyBullet motor control with torque limits
- **Kinematic mode** (`mass=0.0` or `physics=False`): `resetJointState` with per-step interpolation — joints move at URDF `<limit velocity="...">` rates, falling back to per-joint-type defaults when unspecified: `_KINEMATIC_JOINT_FALLBACK_VELOCITY` (2.0 rad/s) for revolute joints and `_KINEMATIC_PRISMATIC_FALLBACK_VELOCITY` (0.5 m/s) for prismatic joints. Mode selected once at init via `_compute_use_kinematic_joints()` and cached in `_use_kinematic_joints`.
- **Kinematic joint cache** (`_kinematic_joint_positions`): Joint positions cached in a Python dict, initialized via batch `p.getJointStates()`, updated after each `resetJointState()`. `get_joint_state()` returns cached values for kinematic robots — zero PyBullet calls per step.

**Key Joint Methods:**
- `set_joint_target(index, position)`: Set single joint target (transparent mode switching)
- `set_all_joints_targets(positions)`: Set all joint targets at once
- `are_all_joints_at_targets(targets, tolerance)`: Check if all joints reached targets
- `are_joints_at_targets(targets, tolerance)`: Unified check — accepts list, dict, or `None` (uses `_last_joint_targets`)
- `_update_kinematic_joints(dt)`: Internal per-step interpolation (called from `update()`)

**Inverse Kinematics (IK):**
- `move_end_effector(target_position, target_orientation, end_effector_link)`: High-level EE position command. Solves IK internally, checks reachability, sets joint targets. Returns `True` if reachable, `False` if not (best-effort targets still set).

**Associated Params:**

- `AgentSpawnParams` — Configuration for agent initialization: motion limits (`max_linear_vel`, `max_linear_accel`, `max_angular_vel`, `max_angular_accel`), motion mode (`"omnidirectional"` / `"differential"`), orientation, mass, collision toggle. Immutable after creation.
- `IKParams` — IK solver configuration dataclass: `max_outer_iterations`, `convergence_threshold`, `max_inner_iterations`, `residual_threshold`, `reachability_tolerance`, `seed_quartiles`, `ik_joint_names`. Passed to `Agent.from_urdf(ik_params=...)`. Default: 5 outer iterations, 0.01 m threshold.
  - `ik_joint_names` (optional `tuple[str, ...]`) — When set, only the named joints participate in IK; all other movable joints are locked at their current positions. When `None` (default), the solver auto-detects: `JOINT_FIXED` joints are skipped, and continuous joints (lower limit ≥ upper limit, e.g. wheels) are locked automatically. This makes IK work correctly on composite robots like mobile manipulators without manual configuration.

**Agent-Level Tolerance:**
- `Agent.joint_tolerance` — property (float, list, dict, or None) that provides a default tolerance for `JointAction` when `tolerance=None`. Supports dict keyed by joint name or list indexed by absolute joint index for per-joint thresholds (useful for mixed prismatic/revolute arms). Out-of-range list indices fall back to the class default (0.01). Fallback: instance value → class default (0.01). Can be set at construction via `Agent.from_urdf(joint_tolerance=...)` or updated via the property setter.

---

### 3. agent_manager.py

**Purpose**: Multi-agent coordination and spawning

#### Key Classes:

##### SimObjectManager
Base manager for all simulation objects.  Parametrised by an `object_class`
(default `SimObject`) so that spawning methods automatically create the right type.

**Key Methods:**
- `spawn_objects_grid(num_objects, grid_params, spawn_params)`: Create objects in grid pattern
- `spawn_grid_mixed(num_objects, grid_params, spawn_params_list)`: Mixed type spawning
- `spawn_grid_counts(grid_params, spawn_params_count_list)`: Exact count spawning
- `spawn_objects_batch(params_list)`: Batch spawn with explicit poses

##### AgentManager
Extends SimObjectManager with `object_class=Agent`.

**Additional Responsibilities:**
- Goal management and update callbacks
- Query moving/stopped agents

**Convenience Aliases:**
- `spawn_agents_grid(...)` → `spawn_objects_grid(...)`
- `spawn_agents_grid_mixed(...)` → `spawn_grid_mixed(...)`
- `spawn_agent_grid_counts(...)` → `spawn_grid_counts(...)`

**Agent-Specific Methods:**
- `register_callback(callback)`: Register custom goal logic
- `set_goal_pose(agent_index, goal)`: Set goal for a specific agent
- `get_moving_count()`: Count moving agents

**Note:**
- Agent.update() is automatically called by MultiRobotSimulationCore.step_once()
- AgentManager focuses on goal management, not movement updates

**Associated Params:**

- `GridSpawnParams` — Grid layout configuration: boundaries (`x_min`/`x_max`, `y_min`/`y_max`, `z_min`/`z_max`), spacing, offset. Automatically distributes agents evenly using `ceil(sqrt(n))`.

---

### 4. action.py

**Purpose**: High-level action system for agents

#### Key Classes:

##### Action (Base Class)
Abstract base class for all actions.

**Key Methods:**
- `start(agent)`: Initialize action
- `update(agent, dt)`: Update action state
- `is_complete()`: Check if action finished
- `stop(agent)`: Clean up action

##### MoveTo
Navigate agent to target pose.

**Key Parameters:**
- `target_pose`: Destination pose
- `tolerance`: Distance threshold for completion

##### Pick
Pick up an object and attach it to agent.

**Key Parameters:**
- `target_object_id`: Specific object body ID to pick (optional)
- `target_position`: Pick from position — auto-select nearest pickable object (optional)
- `search_radius`: Search radius when using `target_position` (default: 0.5m)
- `attach_link`: Link index or name to attach to (default: -1 for base)
- `attach_relative_pose`: Offset in link's frame as Pose
- `use_approach`: Whether to execute the approach/retreat phases (default: `True`). When `True`, the agent navigates to an approach pose → moves forward to the pick position → picks → retreats. When `False`, the pick is executed immediately at the agent's current position — useful for arm robots and mobile manipulators where the EE is already positioned via IK.
- `approach_offset`: Distance from target for auto-calculated approach pose (default: 1.0 m)

##### Drop
Drop an attached object at a specified location.

**Key Parameters:**
- `drop_pose`: Where to drop the object (position and orientation)
- `drop_relative_pose`: Optional `Pose` offset — when set, the object is placed at its current (pre-detach) position transformed by this offset instead of being teleported to `drop_pose`. Useful for EE-attached objects on mobile manipulators where the absolute world drop position is hard to predict.
- `target_object_id`: Specific object to drop (None = first attached)
- `place_gently`: Place at exact position vs drop from height (default: True)
- `use_approach`: Whether to execute the approach/retreat phases (default: `True`). When `True`, the agent navigates to an approach pose near `drop_pose` → moves forward → drops → retreats. When `False`, the drop is executed immediately — useful for arm robots and mobile manipulators where the EE is already positioned.
- `approach_offset`: Distance from drop pose for auto-calculated approach pose (default: 1.0 m)
- `drop_offset`: Distance from `drop_pose` where the actual drop occurs (default: 0.0 = at `drop_pose`)

##### Wait
Wait for specified duration.

**Key Parameters:**
- `duration`: Wait time in seconds

##### JointAction
Move all joints to target positions.

**Key Parameters:**
- `target_joint_positions`: List of target positions for all controllable joints (radians for revolute, metres for prismatic), or dict keyed by joint name
- `tolerance`: Completion threshold per joint — scalar `float`, `list` indexed by absolute joint index, `dict` keyed by joint name, or `None` (resolved from `agent.joint_tolerance` on first tick; default: 0.01). Out-of-range list indices fall back to the class default (0.01)
- `max_force`: Motor force for physics mode (default: 500.0 N·m)

**Tolerance resolution:** When `tolerance` is `None`, it is resolved once from `agent.joint_tolerance` at the first `execute()` call and written back to `action.tolerance`. Fallback chain: Action → Agent → class default (0.01). Dict tolerance enables per-joint thresholds for mixed prismatic/revolute arms.

**Completion:** All joints within `tolerance` of their targets. Works transparently
in both physics mode (motor control) and kinematic mode (interpolation).

##### PoseAction
Move end-effector to a Cartesian target position via IK.

**Key Parameters:**
- `target_position`: EE target `[x, y, z]` in world frame
- `target_orientation`: Optional quaternion `[x, y, z, w]` for orientation control
- `end_effector_link`: Link index, name, or `None` (auto-detect last link)
- `tolerance`: EE Cartesian distance threshold in metres (default: 0.02 m)
- `max_force`: Motor force for physics mode (default: 500.0 N·m)

**Completion:** Joints within default joint tolerance of the IK solution **and** EE within `tolerance`
of the target position. Calls `move_end_effector()` on start,
then monitors `are_joints_at_targets()` and `are_ee_at_target()` each step.

**Unreachable targets:** If the IK solver determines the target is unreachable, the action does not
fail immediately. Best-effort joint targets are set and joints move toward them. After settling,
the action completes with `ActionStatus.FAILED` (not `COMPLETED`). A warning is logged at start
and the `error_message` attribute is set to `"IK target was not reachable"`.

**IK integration in Pick/Drop:**
`PickAction` and `DropAction` accept an optional `ee_target_position` parameter.
When set, the action delegates to a `PoseAction` sub-action to position the EE via IK before
performing the pick/drop operation, as an alternative to `JointAction`-based positioning.
A `continue_on_ik_failure` flag (default: `True`) controls whether the pick/drop
proceeds even when the IK target is unreachable.

---

### 5. geometry.py

**Purpose**: Geometric data structures (`Pose`, `Path`) used throughout the codebase for position/orientation representation and waypoint sequences.

---

### 6. tools.py

**Purpose**: Utility functions for pose calculation (approach/offset poses for pick/drop actions).

---

### 7. data_monitor.py

**Purpose**: Optional real-time GUI monitor (`DataMonitor`) displaying FPS and step-time metrics in a tkinter window. Enabled via `monitor: true` in config.

---

### 8. robot_models.py

**Purpose**: Robot model resolution and introspection. Provides a tiered registry (`KNOWN_MODELS`) that maps model names to URDF paths across multiple sources, plus auto-detection of robot capabilities.

#### Key Functions:

##### resolve_urdf(name_or_path)
Resolves a model name to an absolute URDF file path by searching through tiers in order:

| Tier | Source | Example |
|------|--------|---------|
| 0 — `local` | `robots/` directory in the project | `arm_robot`, `mobile_robot` |
| 1 — `pybullet_data` | PyBullet's bundled data directory | `panda`, `kuka_iiwa`, `r2d2` |
| 2 — `ros` | ROS install paths (`$AMENT_PREFIX_PATH`) | `ur5e`, `turtlebot3_burger` |
| 3 — `robot_descriptions` | `robot_descriptions` pip package | `tiago`, `pr2` |

Direct file paths (containing `/` or ending in `.urdf`/`.sdf`) pass through unchanged.
Called internally by `Agent.from_urdf()`, so users can pass model names directly.
`KNOWN_MODELS` is a curated subset — not every model from each tier is pre-registered.
Unlisted models are resolved automatically via fallback scanning of `pybullet_data` and `robot_descriptions`.

For name-based lookup, user search paths (registered via `add_search_path()`) are checked **before** the `KNOWN_MODELS` tiers.

##### register_model(name, path_or_entry) / unregister_model(name)
Add or remove entries from `KNOWN_MODELS` at runtime. `path_or_entry` accepts an absolute path string (tier defaults to `"user"`) or a `ModelEntry` for full tier metadata. Prevents accidental overwrites by default (`force=True` to override).

##### discover_models(tier)
Scan an entire tier (`"pybullet_data"` or `"robot_descriptions"`) and return all discoverable models as `{name: path}`. Unlike `KNOWN_MODELS`, this returns every model found in the installed package. Also used internally by `resolve_urdf()` as a fallback for unlisted models.

##### add_search_path(directory) / remove_search_path(directory) / get_search_paths()
Register custom directories for name-based URDF lookup. User search paths take priority over `KNOWN_MODELS`, enabling users to shadow built-in models with custom versions. `add_search_path()` validates the directory exists and is idempotent.

##### auto_detect_profile(body_id_or_path, client)
Inspects a loaded PyBullet body (or loads a URDF temporarily) and returns a `RobotProfile` dataclass:
- `robot_type` — `"arm"`, `"mobile"`, `"mobile_manipulator"`, or `"static"`
- `num_joints`, `movable_joint_names`, `movable_joint_indices`
- `ee_link_name`, `ee_link_index` — end-effector detection
- `joint_lower_limits`, `joint_upper_limits`, `joint_max_velocities`

Accepts `Union[str, int]` — when given an `int` (body_id), skips load/removeBody overhead.

##### list_all_models()
Returns a dict of all registered models with tier, availability, and resolved path or error message.

##### detect_robot_type(body_id, client)
Lightweight type detection (arm/mobile/mobile_manipulator/static) without full profile analysis.

#### Key Data:

- **`KNOWN_MODELS`** — `dict[str, ModelEntry]` mapping model names to their tier and path resolver
- **`RobotProfile`** — Frozen dataclass with all introspection results
- **`ModelEntry`** — Named tuple: `(tier, path_func)` for each model

---


## Performance Considerations

### Bottlenecks:
1. **GUI Rendering**: 2-3x slower than headless mode
2. **Collision Detection**: O(N) with spatial hashing, O(N²) without
3. **Mesh Complexity**: High-poly meshes slow down rendering
4. **Monitor Updates**: tkinter GUI overhead
5. **Shape Creation**: Repeated shape creation for many objects

### Optimizations:
1. **Disable GUI**: `gui: false` for batch simulations
2. **Spatial Hashing**: Enabled by default for collision detection
3. **Shared Shapes**: Automatic shape caching reduces OpenGL overhead
4. **Increase Timestep**: Trade accuracy for speed (default: 1/240s)
5. **Simple Shapes**: Use boxes/cylinders instead of complex meshes
6. **Batch Operations**: Update all agents in single pass
7. **Disable Monitor**: `monitor: false` in production
8. **Cell Size Tuning**: Use `constant` mode with optimal cell_size for best performance


See `docs/PERFORMANCE_ANALYSIS.md` and `docs/OPTIMIZATION_RESULTS.md` for detailed benchmarks.
