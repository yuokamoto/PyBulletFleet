# PyBulletFleet - Design Documentation

## Architecture Overview

This package provides a modular, reusable PyBullet simulation framework designed for multi-robot scenarios. The architecture is organized into several key components, each with specific responsibilities.

```
┌─────────────────────────────────────────────────────────────┐
│                    User Application                          │
└───────────────────────┬─────────────────────────────────────┘
                        │
        ┌───────────────┴───────────────┐
        │  MultiRobotSimulationCore     │  ← Main simulation engine
        │  (core_simulation.py)         │
        └───────────────┬───────────────┘
                        │
        ┌───────────────┼───────────────┬───────────────┐
        │               │               │               │
   ┌────▼────┐    ┌────▼────┐    ┌────▼────┐    ┌────▼────┐
   │ Mobile  │    │ Robot   │    │ Tools   │    │Visualizer│
   │ Robot   │    │ Manager │    │ (grid)  │    │ Monitor │
   └─────────┘    └─────────┘    └─────────┘    └─────────┘
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
- Callback management for user-defined robot updates
- Keyboard event handling (SPACE, v, c, t keys)

**Key Methods:**
- `from_dict(config)` / `from_yaml(path)`: Factory methods for initialization
- `run_simulation(robot_update_callback, final_callback)`: Main simulation loop
- `setup_camera()`: Camera positioning
- `configure_visualizer()`: Visual settings configuration
- `register_structure_body(body_id)`: Track static structure elements
- `_handle_keyboard_events()`: Process keyboard inputs


##### SimulationParams
Configuration dataclass for simulation parameters.

**Attributes:**
- `gui`, `timestep`, `speed`, `duration`: Core simulation settings
- `physics`, `monitor`: Feature toggles
- `camera_*`: Camera configuration
- `enable_*`: Visualization settings

##### SimObject
Abstract base class for all simulation objects.

**Responsibilities:**
- Common interface for objects in simulation
- Position and orientation management via Pose
- Metadata storage

**Subclasses:**
- `MeshObject`: For static mesh-based objects
- `URDFObject`: For URDF-based objects

todo: merge with Robot class.

##### Pose
Position and orientation representation for any object in the simulation.
Compatible with ROS2 geometry_msgs/Pose and PyBullet's (position, orientation) tuples.

**Attributes:**
- `position`: np.ndarray [x, y, z]
- `orientation`: np.ndarray (quaternion) [x, y, z, w]

**Properties:**
- `x`, `y`, `z`: Convenient accessors for position components

**Methods:**
- `from_xyz()`: Create from position only
- `from_euler()`: Create from position and Euler angles
- `from_pybullet()`: Create from PyBullet getBasePositionAndOrientation()
- `as_euler()`: Get Euler angles (roll, pitch, yaw)
- `as_position_orientation()`: Get (position, orientation) tuple

##### LogLevelManager
Utility for managing PyBullet log verbosity.

**Methods:**
- `set_log_level(level)`: Control PyBullet logging output

---

### 2. robot.py

**Purpose**: Goal-based navigation for mobile robots

#### Key Classes:

##### Robot

**Responsibilities:**
- Goal-based navigation (move towards target pose)
- Velocity and acceleration limiting
- Kinematic teleportation (for physics-disabled mode)
- Path following
- Goal reached detection

**Key Methods:**
- `set_goal(pose)`: Set target destination
- `update(dt)`: Update robot state per timestep
- `is_goal_reached()`: Check if at destination
- `get_pose()`: Get current position and orientation
- `kinematic_teleport_base()`: Direct position/velocity control

**Control Algorithm:**
- Simple proportional controller for position
- Linear interpolation for smooth motion
- Velocity clamping based on max_linear_vel and max_linear_accel

##### RobotSpawnParams
Configuration dataclass for robot initialization.

**Attributes:**
- `max_linear_vel`, `max_linear_accel`: Motion limits
- `orientation_euler`: Initial orientation
- `base_mass`: Mass (0.0 for kinematic control)
- `use_collision`: Enable collision detection

**Design Notes:**
- Separate spawn params from runtime params for clarity
- Immutable after creation (dataclass)

---

### 3. robot_manager.py

**Purpose**: Multi-robot coordination and spawning

#### Key Classes:

##### RobotManager
High-level manager for multiple Robot instances.

**Responsibilities:**
- Batch robot spawning (grid-based layout)
- Centralized robot collection management
- Bulk operations (update all, set goals for all)
- Grid position calculation

**Key Methods:**
- `spawn_robots_grid(mesh_path, num_robots, grid_params, spawn_params)`: Create robots in grid pattern
- `update_all_robots(dt)`: Update all managed robots
- `get_robots()`: Access robot collection

##### GridSpawnParams
Configuration for grid-based robot placement.

**Attributes:**
- `x_min`, `x_max`, `y_min`, `z_max, `y_min`, `z_max`: Grid boundaries
- `spacing`: [x, y, z] spacing between robots
- `offset`: [x, y, z] global offset

**Grid Calculation:**
- Automatically distributes robots evenly within bounds
- Uses ceiling(sqrt(n)) for grid dimensions
- Handles non-perfect squares

---

### 4. tools.py

**Purpose**: Utility functions for coordinate conversion and legacy spawning

#### Key Functions:

##### Coordinate Conversion
- `grid_to_world(grid, spacing, offset)`: Convert grid indices to world coordinates
- `world_to_grid(pos, spacing, offset)`: Convert world coordinates to grid indices
- `grid_to_world_2d()`, `world_to_grid_2d()`: 2D versions

**Use Cases:**
- Path planning in grid space
- Collision detection on grid
- Visualization of grid-based structures

##### Legacy Spawning Functions
- `grid_execution()`: Execute function over grid
- `grid_spawn()`: Generic grid spawning
- `grid_spawn_urdf()`, `grid_spawn_mesh()`: Type-specific spawning


---

### 6. data_monitor.py

**Purpose**: Real-time performance monitoring

#### Key Class:

##### DataMonitor
GUI window displaying simulation metrics.

**Responsibilities:**
- FPS calculation
- Step time measurement
- Display in separate tkinter window
- Non-blocking updates

**Key Methods:**
- `start()`: Launch monitor window
- `update(step_time, fps)`: Update displayed metrics
- `stop()`: Close window

**Implementation:**
- Runs in main thread (tkinter requirement)
- Updates via method calls from simulation loop
- Uses after() for non-blocking behavior

**Use Cases:**
- Performance optimization
- Debugging slow simulations
- Benchmarking

---


## Performance Considerations

### Bottlenecks:
1. **GUI Rendering**: 2-3x slower than headless
2. **Collision Detection**: O(n²) for n robots
3. **Mesh Complexity**: High-poly meshes slow down
4. **Monitor Updates**: tkinter overhead

### Optimizations:
1. **Disable GUI**: `gui: false` for batch simulations
2. **Increase Timestep**: Trade accuracy for speed
3. **Simple Shapes**: Use boxes/cylinders instead of meshes
4. **Batch Operations**: Update all robots in single pass
5. **Disable Monitor**: `monitor: false` in production
