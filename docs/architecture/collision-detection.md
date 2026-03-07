# PyBullet Collision Detection Design

**Date**: 2026-01-22
**Version**: 3.0 (Restructured for clarity: Overview → Architecture → Details)

---

## Table of Contents

1. [Overview & Design Philosophy](#overview--design-philosophy)
2. [Collision Modes: Quick Overview](#collision-modes-quick-overview)
3. [Architecture Overview](#architecture-overview)
4. [System Flow & Object Lifecycle](#system-flow--object-lifecycle)
5. [Broad-Phase Details: Spatial Hash Grid](#broad-phase-details-spatial-hash-grid)
6. [Narrow-Phase Details: PyBullet APIs](#narrow-phase-details-pybullet-apis)
7. [Collision Mode Details](#collision-mode-details)
8. [Implementation & Configuration](#implementation--configuration)
9. [Advanced Features](#advanced-features)
10. [FAQ & Troubleshooting](#faq--troubleshooting)

---

## Overview & Design Philosophy

### Why This Design?

PyBulletFleet's collision detection system is designed for **scalability, determinism, and flexibility**:

**The Challenge**:
- Naive all-pairs collision detection: **O(N²)** complexity
- 1000 robots → 1,000,000 pairs to check every frame → **infeasible**

**Our Solution**:
- **Spatial Hash Grid** (Broad-Phase): O(N) → filters to ~1% of pairs
- **Incremental Updates**: Only check moved objects → 10-100x faster
- **Per-Object Modes**: Fine-grained control (3D, 2D, static, disabled)
- **Two-Layer Design**: Kinematics (fast) + Physics (accurate)

### Core Principles

1. **Scalability First**
   - O(N) broad-phase filtering via spatial hashing
   - Incremental updates (only moved objects)
   - Scales to **100-1000+ objects** in real-time

2. **Kinematics First, Physics When Needed**
   - **Default**: Pure kinematics (no `stepSimulation()`)
   - **Optional**: Physics verification mode

3. **Deterministic & Reproducible**
   - Same input → same output (critical for planning)
   - Stable at high speeds (10-100x real-time)
   - No race conditions or timing dependencies

4. **Per-Object Flexibility**
   - Each object can have different collision modes
   - Runtime mode changes supported
   - No global "one size fits all"

---

## Collision Modes: Quick Overview

PyBulletFleet supports **4 collision modes** per object. This section provides a quick reference; details are in [Section 7](#collision-mode-details).

### Available Modes

```python
from pybullet_fleet.types import CollisionMode

class CollisionMode(Enum):
    NORMAL_3D = "normal_3d"    # Full 3D collision detection
    NORMAL_2D = "normal_2d"    # 2D (XY plane) collision only
    STATIC = "static"          # Fixed objects (one-time setup)
    DISABLED = "disabled"      # No collision at all
```

### Quick Comparison

| Mode | Updates | Spatial Grid | Neighbors Checked | Use Case |
|------|---------|--------------|-------------------|----------|
| **NORMAL_3D** | Every move | Yes | 27 (3×3×3) | Flying drones, 3D robots |
| **NORMAL_2D** | Every move | Yes | 9 (3×3×1) | Ground robots, AGVs |
| **STATIC** | Once (at spawn) | Yes | 27 (3×3×3) | Walls, shelves, structures |
| **DISABLED** | Never | No | 0 | Visualization, markers |

**Key Insight**:
- **NORMAL_3D/2D**: Moving objects, AABB updated when it moves
- **STATIC**: Fixed structures, AABB never updated (optimization)
- **DISABLED**: No collision at all (neither spatial grid nor PyBullet)

---

## Architecture Overview

### Two-Phase Collision Detection

PyBulletFleet uses a classic **Broad-Phase → Narrow-Phase** pipeline:

```
┌─────────────────────────────────────────────────────────────┐
│                    Collision Detection Pipeline             │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Phase 1: BROAD-PHASE (Spatial Hash Grid)                  │
│  ────────────────────────────────────                       │
│  Input:  N objects (e.g., 1000 robots)                     │
│  Goal:   Filter to candidate pairs                          │
│  Method: Spatial hash grid + AABB overlap                   │
│  Complexity: O(N)                                           │
│                                                             │
│  ↓                                                          │
│                                                             │
│  Phase 2: NARROW-PHASE (PyBullet APIs)                     │
│  ───────────────────────────────────                        │
│  Input:  Candidate pairs from Phase 1                       │
│  Goal:   Exact collision detection                          │
│  Method: getClosestPoints() or getContactPoints()           │
│  Output: Confirmed collision pairs                          │
│  Complexity: O(k) where k << N²                            │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### Why Two Phases?

**Without Broad-Phase** (Naive All-Pairs):
```
1000 objects → 1000 × 999 / 2 = 499,500 pairs
Every frame: 499,500 × getClosestPoints()  ❌
```

**With Broad-Phase** (Spatial Hash):
```
1000 objects → Spatial grid → ~100 candidate pairs
Every frame: 100 × getClosestPoints()  ✅
```

**Performance Gain**: **500x faster** for large scenes!

### Spatial Hash Grid: How It Works

**Concept**: Divide 3D space into uniform grid cells

```
World Space (top view):
┌─────┬─────┬─────┬─────┬─────┬─────┐
│  ·  │  ·  │  R1 │  ·  │  ·  │  ·  │  Cell size: 2.0m × 2.0m
├─────┼─────┼─────┼─────┼─────┼─────┤
│  ·  │  ·  │  ·  │  ·  │  ·  │  ·  │  R1: Robot at (4, 10)
├─────┼─────┼─────┼─────┼─────┼─────┤  R2: Robot at (2, 4)
│  ·  │  R2 │  W  │  ·  │  ·  │  ·  │  W: Wall at (4, 4)
├─────┼─────┼─────┼─────┼─────┼─────┤  R3: Robot at (10, 2)
│  ·  │  ·  │  ·  │  ·  │  R3 │  ·  │  Only check neighbors!
├─────┼─────┼─────┼─────┼─────┼─────┤
│  ·  │  ·  │  ·  │  ·  │  ·  │  ·  │
└─────┴─────┴─────┴─────┴─────┴─────┘

R2 only checks: 9 neighbor cells (NORMAL_2D) or 27 cells (NORMAL_3D)
   → Finds W (wall) in adjacent cell
   → No need to check R1, R3 (not in neighbor cells)
```

**Key Optimizations**:
1. **Incremental Updates**: Only update AABBs for moved objects
2. **Mode-Based Neighbors**: 2D checks 9 cells, 3D checks 27 cells
3. **Multi-Cell Registration**: Large objects registered to multiple cells

---

## System Flow & Object Lifecycle

This section explains **when and how** AABBs and spatial grids are updated.

**Note**: In standard usage with `run_simulation()`, steps 2-3 (movement and collision detection) are automatically handled by the `step_once()` method. You don't need to manually call `check_collisions()` or `step_simulation()` unless you're implementing custom simulation logic.

### Object Lifecycle

```
┌──────────────────────────────────────────────────────────────┐
│ 1. Object Creation (add_object / spawn_agent)               │
├──────────────────────────────────────────────────────────────┤
│  → Calculate initial AABB via getAABB()                      │
│  → Determine collision mode (NORMAL_3D/2D/STATIC/DISABLED)  │
│  → Register to spatial grid (if not DISABLED)                │
│  → Mark as "moved" for first collision check                 │
└──────────────────────────────────────────────────────────────┘
         ↓
┌──────────────────────────────────────────────────────────────┐
│ 2. Object Movement (resetBasePositionAndOrientation)         │
├──────────────────────────────────────────────────────────────┤
│  → Update AABB via getAABB()                                 │
│  → Update spatial grid registration                          │
│  → Add to _moved_this_step set                              │
│                                                              │
│  Mode-specific behavior:                                     │
│  • NORMAL_3D/2D: Update every movement ✓                    │
│  • STATIC: Never updated (optimization) ✓                   │
│  • DISABLED: Not in grid (skip) ✓                           │
│                                                              │
│  Physics mode:                                               │
│  • stepSimulation() updates all physics objects every step   │
│  • All physics objects automatically marked as moved         │
│  • Checked every frame regardless of explicit movement       │
└──────────────────────────────────────────────────────────────┘
         ↓
┌──────────────────────────────────────────────────────────────┐
│ 3. Collision Detection (check_collisions)                    │
├──────────────────────────────────────────────────────────────┤
│  Step 1: Broad-Phase (Spatial Hash)                         │
│    → Iterate through _moved_this_step objects               │
│    → For each moved object:                                  │
│      - Get its grid cells                                    │
│      - Check 9 or 27 neighbor cells (mode-dependent)         │
│      - Collect potential pairs                               │
│    → AABB overlap test → Filter to candidates               │
│                                                              │
│  Step 2: Narrow-Phase (PyBullet API)                        │
│    → For each candidate pair:                                │
│      - getClosestPoints() or getContactPoints()              │
│      - Exact distance/contact check                          │
│    → Return confirmed collision pairs                        │
│                                                              │
│  Step 3: Cleanup                                             │
│    → Clear _moved_this_step                                 │
│    → Update _active_collision_pairs                         │
└──────────────────────────────────────────────────────────────┘
         ↓
┌──────────────────────────────────────────────────────────────┐
│ 4. Object Removal (remove_object)                            │
├──────────────────────────────────────────────────────────────┤
│  → Remove from _sim_objects_dict                            │
│  → Remove from spatial grid                                  │
│  → Remove from _moved_this_step                             │
│  → Clear AABB cache                                          │
└──────────────────────────────────────────────────────────────┘
```

### Mode-Specific Flow Examples

**Note**: In standard usage, `check_collisions()` and `step_simulation()` are automatically called within `run_simulation()`'s `step_once()` method. The examples below show explicit calls for educational purposes to illustrate the internal flow. You typically don't need to call these methods manually.

**Standard Usage**:
```python
# Typical simulation loop (automatic)
sim_core.run_simulation(
    agent_manager=agent_manager,
    num_steps=1000
)
# → step_once() called every iteration
#   → step_simulation() (if physics=True)
#   → check_collisions() (automatic)
#   → agent updates
```

**Educational Examples** (explicit calls to show internal flow):

#### NORMAL_3D (Moving Robot - Kinematics Mode)
```python
# 1. Create
robot = sim_core.add_object(body_id=1, collision_mode=CollisionMode.NORMAL_3D)
# → AABB calculated
# → Registered to cell (x, y, z)
# → Added to _moved_this_step

# 2. Move (explicit)
robot.set_pose([1.5, 2.0, 0.5])  # or resetBasePositionAndOrientation()
# → AABB updated
# → Grid cell updated (may change to new cell)
# → Added to _moved_this_step

# 3. Check collisions
pairs = sim_core.check_collisions()
# → Robot checked against 27 neighbor cells
# → AABB overlap test
# → getClosestPoints() for candidates
# → _moved_this_step cleared
```

#### NORMAL_3D (Moving Robot - Physics Mode)
```python
# 1. Create
robot = sim_core.add_object(body_id=1, collision_mode=CollisionMode.NORMAL_3D, mass=1.0)
# → AABB calculated
# → Registered to cell (x, y, z)
# → Physics object (mass > 0)

# 2. Every simulation step
sim_core.step_simulation()
# → stepSimulation() updates all physics objects
# → All physics objects automatically marked as moved
# → AABB updated every step

# 3. Check collisions (called every step)
pairs = sim_core.check_collisions()
# → Robot checked against 27 neighbor cells (every step)
# → AABB overlap test
# → getContactPoints() or getClosestPoints() for candidates
# → _moved_this_step cleared
```

#### STATIC (Fixed Wall)
```python
# 1. Create
wall = sim_core.add_structure(mesh="wall.obj", collision_mode=CollisionMode.STATIC)
# → AABB calculated ONCE
# → Registered to grid cells (may be multi-cell)
# → NOT added to _moved_this_step

# 2. (No movement - static!)

# 3. Check collisions
pairs = sim_core.check_collisions()
# → Wall is in grid, can be found by moving objects
# → Wall itself never triggers checks (not in _moved_this_step)
# → Efficient: No updates for static objects!
```

#### DISABLED (Visualization Marker)
```python
# 1. Create
marker = SimObject(body_id=3, collision_mode=CollisionMode.DISABLED)
# → AABB NOT calculated
# → NOT registered to grid
# → PyBullet collision disabled

# 2. Move (if needed for visualization)
marker.set_pose([x, y, z])
# → Visual update only
# → No AABB update
# → No grid update

# 3. Check collisions
pairs = sim_core.check_collisions()
# → Marker completely ignored ✓
```

**Reminder**: The above examples use explicit `check_collisions()` and `step_simulation()` calls for clarity. In production code using `run_simulation()`, these are called automatically in `step_once()`.

---

## Broad-Phase Details: Spatial Hash Grid

This section explains the spatial hash grid implementation in detail.

### Grid Cell Calculation

**Default Behavior**:
By default, `cell_size` is automatically calculated from the **median of object AABBs** when the spatial hash grid is first initialized. This happens:
- On the first call to `run_simulation()` (if not explicitly calculated before)
- When `calculate_optimal_cell_size()` is called explicitly

The automatic calculation ensures the cell size is appropriate for the typical object sizes in your simulation.

**Formula**:
```python
cell_size = 2.0  # meters (configurable or auto-calculated)

def get_cell(position: Tuple[float, float, float]) -> Tuple[int, int, int]:
    """Convert world position to grid cell indices"""
    x, y, z = position
    cell_x = int(x / cell_size)
    cell_y = int(y / cell_size)
    cell_z = int(z / cell_size)
    return (cell_x, cell_y, cell_z)
```

**Example**:
```
Object at position [3.2, 5.7, 1.1], cell_size=2.0
→ Cell = (int(3.2/2), int(5.7/2), int(1.1/2)) = (1, 2, 0)
```

**Automatic Calculation**:
```python
# Default: cell_size calculated from median AABB on first run_simulation()
sim_core = MultiRobotSimulationCore(params)
sim_core.add_object(...)  # Add objects first
sim_core.run_simulation(...)  # cell_size auto-calculated here (first call)

# Explicit: Calculate before simulation
sim_core = MultiRobotSimulationCore(params)
sim_core.add_object(...)  # Add objects first
sim_core.calculate_optimal_cell_size()  # Explicitly calculate
sim_core.run_simulation(...)  # Uses pre-calculated cell_size
```

### AABB Caching

**Purpose**: Avoid redundant `getAABB()` calls

**Data Structure**:
```python
_cached_aabbs_dict: Dict[int, Tuple[Tuple[float, float, float],
                                     Tuple[float, float, float]]]
# Key: object_id (body_id)
# Value: ((min_x, min_y, min_z), (max_x, max_y, max_z))
```

**Update Strategy**:
```python
def _update_object_aabb(self, object_id: int):
    """Called when object moves"""
    obj = self._sim_objects_dict[object_id]

    # Query PyBullet (expensive!)
    self._cached_aabbs_dict[object_id] = p.getAABB(
        obj.body_id,
        physicsClientId=self.client
    )

    # Update grid registration
    self._update_object_spatial_grid(object_id)
```

**Invalidation**: Cache invalidated on movement, mode change, or removal

### Neighbor Search Patterns

```
3D Mode (27 neighbors):
Check all 27 cells in 3×3×3 cube

    Z=+1 layer          Z=0 layer (object)   Z=-1 layer
    ┌───┬───┬───┐       ┌───┬───┬───┐       ┌───┬───┬───┐
    │ ✓ │ ✓ │ ✓ │       │ ✓ │ ✓ │ ✓ │       │ ✓ │ ✓ │ ✓ │
    ├───┼───┼───┤       ├───┼───┼───┤       ├───┼───┼───┤
    │ ✓ │ ✓ │ ✓ │       │ ✓ │ O │ ✓ │       │ ✓ │ ✓ │ ✓ │
    ├───┼───┼───┤       ├───┼───┼───┤       ├───┼───┼───┤
    │ ✓ │ ✓ │ ✓ │       │ ✓ │ ✓ │ ✓ │       │ ✓ │ ✓ │ ✓ │
    └───┴───┴───┘       └───┴───┴───┘       └───┴───┴───┘
    (9 cells)           (9 cells)           (9 cells)
                        Total: 27 cells


2D Mode (9 neighbors):
Check only 9 cells in 3×3 grid on same Z layer

    Z=0 layer (object's Z level only)
    ┌───┬───┬───┐
    │ ✓ │ ✓ │ ✓ │
    ├───┼───┼───┤
    │ ✓ │ O │ ✓ │  O: Object's cell
    ├───┼───┼───┤  ✓: Checked neighbor cells
    │ ✓ │ ✓ │ ✓ │
    └───┴───┴───┘
    Total: 9 cells (Z±1 layers ignored)
```

### Multi-Cell Registration

**Problem**: Large objects may span multiple cells

**Solution**: Register to all overlapping cells

**Detection**:
```python
def _should_use_multi_cell_registration(self, object_id: int) -> bool:
    aabb = self._cached_aabbs_dict[object_id]
    extent_x = aabb[1][0] - aabb[0][0]
    extent_y = aabb[1][1] - aabb[0][1]
    max_extent = max(extent_x, extent_y)

    threshold_size = self._cached_cell_size * self.params.multi_cell_threshold
    return max_extent > threshold_size  # Default: 1.5x cell_size
```

**Example**:
```
Cell size: 2.0m, Threshold: 1.5
Wall: 5.0m × 5.0m → max_extent=5.0 > 3.0 → Multi-cell ✓

Wall registered to 9 cells:
┌─────┬─────┬─────┐
│  W  │  W  │  W  │
├─────┼─────┼─────┤
│  W  │  W  │  W  │  (3×3 grid)
├─────┼─────┼─────┤
│  W  │  W  │  W  │
└─────┴─────┴─────┘
```

**Benefits**:
- Robot in any adjacent cell can find the wall
- No missed collisions for large objects
- Automatic (no manual configuration)

### Movement-Based Optimization

**Key Insight**: Most objects don't move every frame

**Strategy**: Only check pairs involving moved objects

```python
def check_collisions(self):
    # Track objects that moved since last check
    moved_objects = self._moved_this_step.copy()
    self._moved_this_step.clear()

    # Only iterate through moved objects
    for obj_id in moved_objects:
        # Find neighbors in grid
        # Check AABB overlap
        # Add to candidate pairs
```

---

## Narrow-Phase Details: PyBullet APIs

After broad-phase filtering, we use PyBullet's APIs for exact collision detection.

### API Comparison

| Feature | `getContactPoints()` | `getClosestPoints()` |
|---------|----------------------|----------------------|
| **Requires stepSimulation()** | ✅ Yes | ❌ No |
| **Works with kinematics** | ⚠️ Limited | ✅ Yes |
| **Safety margin support** | ❌ No | ✅ Yes (distance param) |
| **Physics-accurate** | ✅ Yes | ⚠️ Geometric only |
| **Speed (typical)** | Fast (cached) | Slightly slower (GJK) |
| **Best for** | Physics mode | Kinematics mode |

### `getClosestPoints(distance)` - Recommended for Kinematics

**What it does**: Geometric closest point query using GJK algorithm

**Key Parameters**:
```python
closest_points = p.getClosestPoints(
    bodyA=obj1.body_id,
    bodyB=obj2.body_id,
    distance=0.02,  # Safety margin (2cm)
    physicsClientId=self.client
)
```

**Behavior**:
- `distance > 0`: Returns points if objects are within margin
- `distance = 0`: Returns points only if actually touching
- `len(closest_points) > 0`: Collision detected

**Advantages**:
✅ Works with `resetBasePositionAndOrientation()` (kinematic)
✅ Stable for kinematic-kinematic pairs
✅ Safety margin support (detect "near miss")
✅ No `stepSimulation()` required

**Use cases**:
- Kinematics-focused planning
- Collision avoidance with safety buffer
- High-speed simulation (no physics overhead)

### `getContactPoints()` - For Physics Mode

**What it does**: Query contact manifold from physics engine

**Key Parameters**:
```python
contact_points = p.getContactPoints(
    bodyA=obj1.body_id,
    bodyB=obj2.body_id,
    physicsClientId=self.client
)
```

**Behavior**:
- Returns contact manifold from **last `stepSimulation()` call**
- Includes penetration depth, normal, contact position
- Empty list if no contact

**Advantages**:
✅ Fast (contact cache already exists)
✅ Physics-accurate (actual contact forces)
✅ Contact normals and depths available

**Limitations**:
❌ **Requires `stepSimulation()`**: Cannot skip physics update
❌ Limited for kinematics: Designed for physics simulation with dynamic objects
❌ No safety margin support: Only reports actual penetration

**Why not ideal for kinematics**:
- Kinematics mode uses `resetBasePositionAndOrientation()` to set positions directly
- `getContactPoints()` relies on contact cache updated by `stepSimulation()`
- Without physics simulation, contact information may be incomplete or outdated
- You must call `stepSimulation()` even for kinematic objects, adding overhead
- Physics verification mode
- Contact force analysis
- Debugging actual collisions

### stepSimulation() - The Physics Update

`stepSimulation()` performs **comprehensive physics update**:

1. **Time Integration**: Update positions, velocities (F=ma)
2. **Broad-Phase**: Internal collision detection
3. **Narrow-Phase**: Exact collision testing
4. **Contact Solving**: Resolve penetrations, apply friction
5. **Cache Update**: Update contact manifold

**Cost**: Computationally expensive (scene-dependent)

**When Required**:
- ✅ Physics ON mode (dynamic objects, mass > 0)
- ❌ Kinematics mode (pure planning)

### Two-Layer Design Philosophy

**Layer 1: Kinematics (Default)** ⚡
```yaml
physics: false
collision_detection_method: CLOSEST_POINTS
collision_margin: 0.02
```
- No `stepSimulation()` → Fast
- `getClosestPoints()` → Stable
- Use case: Planning, coordination, high-speed simulation

**Layer 2: Physics (Verification)** 🔬
```yaml
physics: true
collision_detection_method: CONTACT_POINTS  # or HYBRID
collision_margin: 0.02
```
- `stepSimulation()` every frame → Accurate
- `getContactPoints()` → Realistic contacts
- Use case: Verification, debugging, physics analysis

**Hybrid Mode** (Advanced):
```python
if obj_is_physics:
    use getContactPoints()  # Accurate for dynamics
else:
    use getClosestPoints()  # Stable for kinematics
```

---

## Collision Mode Details

This section provides comprehensive details for each collision mode.

### NORMAL_3D - Full 3D Collision

**Characteristics**:
- **Neighbor Check**: 27 cells (3×3×3 cube)
- **AABB Updates**: Every movement
- **Grid Registration**: Updated on movement
- **PyBullet Collision**: Enabled

**Use Cases**:
- Flying drones (need Z-axis collision)
- 3D manipulators
- Multi-level warehouses
- Any object that moves in 3D space

**Configuration**:
```python
drone = SimObject(
    body_id=body_id,
    sim_core=sim_core,
    collision_mode=CollisionMode.NORMAL_3D,
    mass=0.0  # Kinematic
)
```

**Performance**:
- Checks 27 neighbor cells vs 9 for 2D
- ~3x more cells, but filtered by AABB
- Typical overhead: <10% vs NORMAL_2D

### NORMAL_2D - 2D Collision (XY Plane)

**Characteristics**:
- **Neighbor Check**: 9 cells (3×3×1, XY plane only)
- **AABB Updates**: Every movement
- **Grid Registration**: Updated on movement
- **Z-Axis**: Ignored for neighbor search (but AABB Z must still overlap!)

**Use Cases**:
- Ground robots (AGVs, AMRs)
- Warehouse robots (single floor)
- 2D path planning
- Most typical use cases

**Configuration**:
```python
agv = SimObject(
    body_id=body_id,
    sim_core=sim_core,
    collision_mode=CollisionMode.NORMAL_2D,
    mass=0.0
)
```

### STATIC - Fixed Objects

**Characteristics**:
- **Neighbor Check**: 27 cells (when checked by moving objects)
- **AABB Updates**: NEVER (calculated once at spawn)
- **Grid Registration**: Once at spawn, never updated
- **Movement**: Assumed to never move

**Use Cases**:
- Walls, pillars, structures
- Shelves, racks
- Fixed obstacles
- Any object that never moves

**Configuration**:
```python
wall = SimObject(
    body_id=body_id,
    sim_core=sim_core,
    collision_mode=CollisionMode.STATIC,
    is_static=True
)
```

**Performance Benefits**:
```
Many static walls (e.g., 1000):
  NORMAL_3D: All walls updated every frame → High overhead
  STATIC: 0 AABB updates/frame → No overhead ✓

  → Moving robots can still collide with walls!
  → Walls never trigger updates themselves
```

**Important**: If a STATIC object needs to move, change mode first:
```python
wall.set_collision_mode(CollisionMode.NORMAL_3D)  # Enable updates
wall.set_pose([new_x, new_y, new_z])               # Move
wall.set_collision_mode(CollisionMode.STATIC)      # Re-optimize
```

### DISABLED - No Collision

**Characteristics**:
- **Neighbor Check**: NEVER (not in spatial grid)
- **AABB Updates**: NEVER
- **Grid Registration**: NEVER
- **PyBullet Collision**: Disabled via `setCollisionFilterPair()`

**Use Cases**:
- Visualization markers
- Debug geometry
- Temporary exclusion
- Non-physical objects

**Configuration**:
```python
marker = SimObject(
    body_id=body_id,
    sim_core=sim_core,
    collision_mode=CollisionMode.DISABLED
)
```

**Behavior**:
```python
# Object is completely invisible to collision system
marker.set_pose([x, y, z])  # Visual update only
pairs = sim_core.check_collisions()  # Marker ignored ✓
```

**Performance**:
- Zero overhead (not processed at all)
- Use liberally for visualization objects

### Runtime Mode Changes

**Supported Transitions**:
```python
# Any mode → Any mode
obj.set_collision_mode(new_mode)

# Example: Temporarily disable
obj.set_collision_mode(CollisionMode.DISABLED)
# ... do something ...
obj.set_collision_mode(CollisionMode.NORMAL_3D)  # Re-enable
```

**What Happens Internally**:
1. Remove from old mode's data structures
2. Update AABB if needed
3. Register to new mode's data structures
4. Update PyBullet collision filter if DISABLED ↔ other

**Use Cases**:
- Dynamic environment changes
- Temporary object exclusion
- Performance optimization
- A/B testing different modes

---

## Implementation & Configuration

### SimulationParams

```python
from pybullet_fleet.core_simulation import SimulationParams
from pybullet_fleet.types import CollisionDetectionMethod

# Kinematics Mode (Recommended Default)
params = SimulationParams(
    physics=False,
    collision_detection_method=CollisionDetectionMethod.CLOSEST_POINTS,
    collision_margin=0.02,  # 2cm safety buffer
    spatial_hash_cell_size=2.0,  # 2m cells
    multi_cell_threshold=1.5,  # Large object threshold
)

# Physics Mode (Verification)
params = SimulationParams(
    physics=True,
    collision_detection_method=CollisionDetectionMethod.CONTACT_POINTS,
    collision_margin=0.02,
    spatial_hash_cell_size=2.0,
)

# Hybrid Mode (Advanced)
params = SimulationParams(
    physics=True,
    collision_detection_method=CollisionDetectionMethod.HYBRID,
    collision_margin=0.05,  # Larger margin for kinematics safety
    spatial_hash_cell_size=2.0,
)
```

### Per-Object Configuration

```python
# Moving 3D robot
robot_3d = SimObject(
    body_id=body_id,
    sim_core=sim_core,
    collision_mode=CollisionMode.NORMAL_3D,
    mass=0.0,  # Kinematic
)

# Moving 2D AGV
agv = SimObject(
    body_id=body_id,
    sim_core=sim_core,
    collision_mode=CollisionMode.NORMAL_2D,
    mass=0.0,
)

# Fixed structure
wall = SimObject(
    body_id=body_id,
    sim_core=sim_core,
    collision_mode=CollisionMode.STATIC,
    is_static=True,
)

# Visualization marker
marker = SimObject(
    body_id=body_id,
    sim_core=sim_core,
    collision_mode=CollisionMode.DISABLED,
)
```

### Config File (YAML)

```yaml
# config.yaml
physics: false
collision_detection_method: "closest_points"
collision_margin: 0.02
spatial_hash_cell_size: 2.0
multi_cell_threshold: 1.5
ignore_static_collision: false
```

### Auto-Selection Logic

**Collision Detection Method**:
- `physics=False` → `CLOSEST_POINTS` (auto)
- `physics=True` → `CONTACT_POINTS` (auto)
- Explicit setting overrides auto-selection

**Collision Mode**:
- Default: `NORMAL_3D` if not specified
- Recommended: Explicitly set for clarity

### Collision Margin Recommendations

| Use Case | Margin | Rationale |
|----------|--------|-----------|
| Tight spaces | 0.01-0.02m | Minimal clearance |
| Normal operation | 0.02-0.05m | Comfortable safety |
| Conservative | 0.05-0.10m | Extra safety for high-speed |
| Contact detection | 0.0m | Actual touching only |

---

## Advanced Features

### Multi-Cell Registration (Large Objects)

**See dedicated section**: [Multi-Cell Registration](#multi-cell-registration)

**Summary**:
- Automatic size-based detection
- Objects > 1.5× cell_size → Multi-cell
- No manual configuration needed
- Prevents missed collisions for large objects

### Incremental Updates

**Concept**: Only update what changed

**Benefits**:
```
1000 objects, 10 move per frame:
  Full update: 1000 getAABB() calls
  Incremental: 10 getAABB() calls
  → 100x faster! ✓
```

**Implementation**:
- Track moved objects in `_moved_this_step`
- Only these objects trigger collision checks
- Static objects never added to `_moved_this_step`

### Cell Size Optimization

### Cell Size Optimization

**Cell Size Calculation Modes**:

The system supports three modes for determining `cell_size`:

1. **AUTO_INITIAL** (default): Calculate once at simulation start
   - Calculated on first `run_simulation()` call
   - Based on actual object sizes in your scene at startup
   - Uses formula: `cell_size = median_aabb_extent × scale_factor`
   - Best balance of performance and adaptability

2. **CONSTANT**: Use a fixed, manually-specified cell size
   - Must explicitly set both `spatial_hash_cell_size_mode` and `spatial_hash_cell_size`
   - No automatic calculation or overriding
   - Best when you know the optimal size in advance
   - Fastest (no calculation overhead)

3. **AUTO_ADAPTIVE**: Recalculate on object add/remove
   - Adapts to changing object distributions during simulation
   - Recalculates whenever objects are added or removed
   - Higher overhead, useful when object sizes vary significantly

**Trade-off**:
- **Small cells**: More grid lookups, fewer objects per cell
- **Large cells**: Fewer grid lookups, more objects per cell

**Manual Configuration (CONSTANT mode)**:
```python
from pybullet_fleet.types import SpatialHashCellSizeMode

# IMPORTANT: Must set mode to CONSTANT to use fixed cell_size
params = SimulationParams(
    spatial_hash_cell_size_mode=SpatialHashCellSizeMode.CONSTANT,
    spatial_hash_cell_size=2.0,  # Fixed 2m cells
)

# Rule of thumb: cell_size ≈ 2 × average_robot_size
# Example: 1m robots → cell_size = 2.0m

# WARNING: Setting only spatial_hash_cell_size without mode will be ignored!
# Default mode is AUTO_INITIAL, which overwrites the value.
```

**Auto-Initial Mode** (Default - Recommended for most cases):
```python
from pybullet_fleet.types import SpatialHashCellSizeMode

# Let the system calculate optimal cell_size once at startup
params = SimulationParams(
    spatial_hash_cell_size_mode=SpatialHashCellSizeMode.AUTO_INITIAL,
)
# cell_size calculated automatically on first run_simulation() call

# Or calculate explicitly before simulation:
sim_core.calculate_optimal_cell_size()  # Called before run_simulation()
```

**Auto-Adaptive Mode** (Advanced - for dynamic environments):
```python
from pybullet_fleet.types import SpatialHashCellSizeMode

# Recalculate cell_size whenever objects are added/removed
params = SimulationParams(
    spatial_hash_cell_size_mode=SpatialHashCellSizeMode.AUTO_ADAPTIVE,
)
# Useful when object sizes change significantly during simulation
# Higher computational overhead
```

**When to Use Each**:
- **AUTO_INITIAL** (default): Best for most cases, calculates once at startup
- **CONSTANT**: When you know the optimal size and want maximum performance
- **AUTO_ADAPTIVE**: When object sizes vary significantly during simulation (e.g., objects spawn/despawn frequently)
- **Explicit calculation**: When you need to inspect/debug the auto-calculated size

---

## FAQ & Troubleshooting

### Q: Why not always use `getClosestPoints()`?

**A**: In Physics ON mode with many actual contacts, `getContactPoints()` is:
- Faster (contact cache already exists)
- More accurate (actual contact manifold with normals, depths)

But for kinematics, `getClosestPoints()` is **required** for stability.

### Q: STATIC vs DISABLED - What's the difference?

**A**:
- **STATIC**: Collision enabled, AABB never updated (for fixed structures)
  - Still detectable by moving objects ✓
  - Optimization: No updates
- **DISABLED**: No collision at all (for visualization)
  - Completely ignored by collision system
  - Zero overhead

### Q: Can 2D and 3D objects collide?

**A**: Yes! Collision modes are per-object:
```python
robot_2d = SimObject(..., collision_mode=CollisionMode.NORMAL_2D)
drone_3d = SimObject(..., collision_mode=CollisionMode.NORMAL_3D)

# Collision check:
# drone_3d checks 27 neighbors → finds robot_2d ✓
# robot_2d checks 9 neighbors → finds drone_3d (if in XY neighbors) ✓
```

### Q: How do I debug collision detection?

**A**: Several options:

1. **Enable collision visualization**:
```python
params = SimulationParams(gui=True)
# Press 'c' key to toggle collision shapes
```

2. **Check AABB caches**:
```python
aabb = sim_core._cached_aabbs_dict.get(obj.body_id)
print(f"AABB: {aabb}")
```

3. **Inspect spatial grid**:
```python
cells = sim_core._cached_object_to_cell.get(obj.body_id)
print(f"Object in cells: {cells}")
```

4. **Run comprehensive tests**:
```bash
python -m pytest tests/test_collision_comprehensive.py -v
```

### Q: Performance is slow. How to optimize?

**A**: Checklist:

1. ✅ Use STATIC for fixed objects (walls, structures)
2. ✅ Use DISABLED for visualization objects
3. ✅ Set appropriate cell_size (~2× object size)
4. ✅ Use NORMAL_2D instead of NORMAL_3D for ground robots
5. ✅ Enable physics=False for kinematics (huge speedup!)
6. ✅ Check multi_cell_threshold (default 1.5 is usually good)

**Benchmark**:
```bash
cd benchmark
python performance_benchmark.py
```


---

## Conclusion

PyBulletFleet's collision detection system provides:

✅ **Scalability**: O(N) via spatial hashing (not O(N²))
✅ **Flexibility**: 4 collision modes per object
✅ **Performance**: Incremental updates, mode-based optimization
✅ **Determinism**: Same input → same output
✅ **Simplicity**: Automatic configuration, minimal setup

**Recommended Workflow**:
1. Start with default (physics=False, CLOSEST_POINTS)
2. Set collision modes (NORMAL_3D/2D, STATIC, DISABLED)
3. Run and profile
4. Optimize cell_size if needed
5. Use Physics mode only for verification

---

**For implementation details, see**:
- `pybullet_fleet/core_simulation.py` - Main implementation
- `pybullet_fleet/sim_object.py` - SimObject with collision modes
- `pybullet_fleet/types.py` - Enums and types
- `tests/test_collision_comprehensive.py` - Comprehensive tests (10/11 passing)

**Related documentation**:
- `docs/COLLISION_TEST_DESIGN.md` - Test design specification
- `docs/spatial_hash_cell_size_modes.md` - Cell size optimization
- `benchmark/PERFORMANCE_OPTIMIZATION_GUIDE.md` - Performance tuning
