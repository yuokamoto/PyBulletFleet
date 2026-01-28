# PyBullet Collision Detection Design

**Date**: 2026-01-20
**Version**: 2.0 (Updated with CollisionMode system)

## Design Philosophy

### Core Principles

1. **Kinematics First, Physics When Needed**
   - Main use case: Kinematic motion planning and collision avoidance
   - Secondary use case: Physics-based verification and debugging

3. **Per-Object Collision Control**
   - Each object can have different collision modes
   - Runtime collision mode changes supported

4. **Reproducibility, Stability, Scalability**
   - Deterministic results across runs
   - Stable at high simulation speeds (Nx speed)
   - Scales to 100-1000+ objects

---

## CollisionMode System

### Per-Object Collision Control

Each `SimObject` has a `collision_mode` property that controls its collision behavior:

```python
from pybullet_fleet.types import CollisionMode

# Available modes
class CollisionMode(Enum):
    NORMAL_3D = "normal_3d"    # Full 3D collision (27 neighbors in spatial grid)
    NORMAL_2D = "normal_2d"    # 2D collision only (9 neighbors, XY plane)
    STATIC = "static"          # One-time registration, never updated
    DISABLED = "disabled"      # No collision detection at all
```

### Mode Characteristics

| Mode | Spatial Grid | AABB Updates | PyBullet Collision | Use Case |
|------|--------------|--------------|-------------------|----------|
| **NORMAL_3D** | 27 neighbors | Every movement | Enabled | Moving robots (full 3D) |
| **NORMAL_2D** | 9 neighbors (XY) | Every movement | Enabled | Ground robots (2D only) |
| **STATIC** | 27 neighbors | Once (initial) | Enabled | Fixed structures |
| **DISABLED** | Not in grid | Never | **Disabled** | Visualization objects |

### Setting Collision Mode

```python
# At creation time
obj = SimObject(
    body_id=body_id,
    sim_core=sim_core,
    collision_mode=CollisionMode.NORMAL_2D  # 2D collision for ground robot
)

# Runtime change
obj.set_collision_mode(CollisionMode.DISABLED)  # Disable collision
obj.set_collision_mode(CollisionMode.STATIC)    # Make static (optimization)
```

### DISABLED Mode Behavior

When `collision_mode == CollisionMode.DISABLED`:

1. **PyBullet Physics Collision**: Disabled via `setCollisionFilterPair(enable=0)`
2. **Custom Collision Detection**: Object excluded from spatial grid
3. **AABB Cache**: Not maintained (no updates)
4. **Collision Pairs**: Never checked against other objects

**Use cases**:
- Visualization-only objects (no collision needed)
- Temporary object exclusion
- Performance optimization for non-colliding objects

---

## PyBullet APIs: Comparison

### `stepSimulation()` - The Core Update Function

`stepSimulation()` performs **all** of the following:
- Rigid body time integration (position, velocity, orientation)
- Broadphase collision detection (candidate pair generation)
- Narrowphase collision detection (exact collision testing)
- Contact manifold generation
- Contact constraint solving (penetration resolution, friction)
- Contact cache update

**Key Point**: `getContactPoints()` returns results **from the last `stepSimulation()` call**.

### When is `stepSimulation()` Required?

#### ❌ NOT Required (Physics OFF)
- Pure kinematics operation
- Pose updates via `resetBasePositionAndOrientation()` / `resetJointState()`
- No push-back, friction, or mass effects needed
- Collision detection via `getClosestPoints()` or `getAABB()`

✅ **Stable without `stepSimulation()`**

#### ✅ Required (Physics ON)
- Dynamic objects exist (mass > 0)
- Real contact behavior needed (push, slide, friction)
- `getContactPoints()` must be reliable
- Fixed Δt periodic execution assumed

✅ **Must call `stepSimulation()` regularly**

---

## Collision Detection APIs

### `getContactPoints()`

**What it does**: Returns contact manifold from physics engine

**Characteristics**:
- ✅ Fast (contact cache already exists after `stepSimulation()`)
- ✅ Physics-accurate (actual penetration depth, normals)
- ❌ Returns empty if no contact (can't detect "almost touching")
- ❌ Timing-dependent (requires `stepSimulation()`)
- ❌ **Unstable for kinematic-kinematic pairs**

**Best for**:
- Physics-physics actual contact logging
- Debug and incident reproduction
- Contact force analysis

### `getClosestPoints(distance)`

**What it does**: Geometric closest point query (distance-based)

**Characteristics**:
- ✅ `stepSimulation()`-independent (queries geometry directly)
- ✅ Stable with `resetBasePositionAndOrientation()` updates
- ✅ **Safety margin support** (`distance > 0` detects "near miss")
- ✅ **Stable for kinematic-kinematic pairs**
- ⚠️ Slightly slower than `getContactPoints()` (GJK algorithm)

**Best for**:
- Kinematics-focused collision avoidance
- Safety clearance / margin evaluation
- Mixed physics/kinematics scenarios

---

## Recommended Two-Layer Mode Design

### Physics OFF Mode (Default) ✅

**Configuration**:
```yaml
physics: false
collision_detection_method: "closest_points"
collision_margin: 0.02  # 2cm safety clearance
```

**Behavior**:
- `stepSimulation()`: **Not called** (skipped for performance)
- Pose updates: `resetBasePositionAndOrientation()` (kinematic)
- Collision detection: `getClosestPoints(distance=0.02)`
- Contact resolution: None (pure avoidance)

**Characteristics**:
- ⚡ **Fast**: No physics overhead
- 🎯 **Deterministic**: Same input → same output
- 📈 **Scales well**: Supports high speed multipliers (Nx speed)
- 🛡️ **Safe**: Detects near-misses before actual contact

**Use cases**:
- Warehouse robot path planning
- Multi-robot coordination
- High-speed simulation (100x real-time)
- What-if scenario analysis

---

### Physics ON Mode (Verification) 🔬

**Configuration**:
```yaml
physics: true
collision_detection_method: "contact_points"  # or "hybrid"
collision_margin: 0.02  # Still useful for hybrid mode
```

**Behavior**:
- `stepSimulation()`: **Called every step** (fixed Δt)
- Pose updates: Physics engine (force-based)
- Collision detection:
  - Primary: `getClosestPoints(distance=0.02)` (safety check)
  - Logging: `getContactPoints()` (actual contact events)
- Contact resolution: Yes (push-back, friction)

**Characteristics**:
- 🔬 **Realistic**: Actual physics behavior (mass, inertia, friction)
- 🐛 **Debuggable**: Contact logs for incident analysis
- ⏱️ **Real-time**: Limited speed multiplier (1-10x)
- 🎓 **Educational**: Visualize actual dynamics

**Use cases**:
- Verify collision-free paths under physics
- Debug actual collision incidents
- Analyze contact forces
- Validate simulation accuracy

---

## Implementation: Parameter Reference

### `SimulationParams` Configuration

```python
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.types import CollisionDetectionMethod, CollisionMode

# === Physics OFF (Recommended Default) ===
params_kinematics = SimulationParams(
    physics=False,                                      # No stepSimulation()
    collision_detection_method=CollisionDetectionMethod.CLOSEST_POINTS,  # Auto-selected
    collision_margin=0.02,                              # 2cm safety clearance
    timestep=1/240,                                     # Logical time unit
    speed=10.0,                                         # 10x speed (or 0 for max)
)

# === Physics ON (Verification Mode) ===
params_physics = SimulationParams(
    physics=True,                                       # Call stepSimulation()
    collision_detection_method=CollisionDetectionMethod.CONTACT_POINTS,  # Auto-selected
    collision_margin=0.02,                              # Still useful for hybrid
    timestep=1/240,                                     # Fixed physics timestep
    speed=1.0,                                          # Real-time (or slower)
)

# === Hybrid Mode (Advanced) ===
params_hybrid = SimulationParams(
    physics=True,                                       # Physics enabled
    collision_detection_method=CollisionDetectionMethod.HYBRID,
    collision_margin=0.05,                              # 5cm safety for kinematics
    timestep=1/240,
    speed=1.0,
)
```

### Per-Object CollisionMode Configuration

```python
# Ground robot (2D collision only)
robot_2d = SimObject(
    body_id=body_id,
    sim_core=sim_core,
    collision_mode=CollisionMode.NORMAL_2D,  # 9 neighbors (XY plane)
    mass=0.0,  # Kinematic
)

# Flying drone (3D collision)
drone = SimObject(
    body_id=drone_id,
    sim_core=sim_core,
    collision_mode=CollisionMode.NORMAL_3D,  # 27 neighbors (full 3D)
    mass=0.0,
)

# Fixed structure (static)
structure = SimObject(
    body_id=structure_id,
    sim_core=sim_core,
    collision_mode=CollisionMode.STATIC,  # One-time AABB, never updated
    is_static=True,
)

# Visualization object (no collision)
marker = SimObject(
    body_id=marker_id,
    sim_core=sim_core,
    collision_mode=CollisionMode.DISABLED,  # No collision at all
)

# Runtime mode change
robot_2d.set_collision_mode(CollisionMode.DISABLED)  # Temporarily disable
robot_2d.set_collision_mode(CollisionMode.NORMAL_2D)  # Re-enable
```

### Auto-Selection Logic

If `collision_detection_method` is not explicitly set:
- `physics=False` → `CLOSEST_POINTS` (kinematics-safe)
- `physics=True` → `CONTACT_POINTS` (physics-accurate)

If `collision_mode` is not explicitly set:
- Default: `CollisionMode.NORMAL_3D` (full 3D collision)

### Collision Margin Recommendations

| Use Case | Recommended Margin | Rationale |
|----------|-------------------|-----------|
| Tight spaces | 0.01 - 0.02m (1-2cm) | Minimal clearance |
| Normal operation | 0.02 - 0.05m (2-5cm) | Comfortable safety buffer |
| Conservative | 0.05 - 0.10m (5-10cm) | Extra safety for high-speed |
| Contact detection | 0.0m | Actual touching only |

---

## Broad-Phase Optimization

### AABB → Narrow-Phase Flow

PyBulletFleet uses a **three-stage** collision pipeline:

```
1. Broad-phase (AABB + Spatial Hash)
   ↓ (filter candidates)
2. Movement-based filtering
   ↓ (only check moved objects)
3. Narrow-phase (getContactPoints / getClosestPoints)
   ↓
Final collision pairs
```

### Broad-Phase APIs

#### `getAABB()` + Custom Spatial Hash ✅ Recommended
**When to use**: Multi-object collision detection (N > 10)

**Advantages**:
- Full control over grid size and update frequency
- Incremental updates (only moved objects)
- O(N) complexity with spatial hashing

**Implementation**: Already built-in to `PyBulletFleet`

#### `getOverlappingObjects()`
**When to use**: Single-object proximity queries

**Advantages**:
- Simple API (one body's neighbors)
- Good for specific queries (e.g., gripper collision check)

**Disadvantages**:
- Not suitable for all-pairs collision enumeration
- Less efficient for large N

---

## Kinematics Objects and `getContactPoints()`

### The Problem

Kinematic objects (mass=0, updated via `resetBasePositionAndOrientation()`) **can** return contacts via `getContactPoints()`, but:

1. **Kinematic-kinematic pairs are unstable**
   - Contacts may not be detected even when overlapping
   - Slight penetrations often ignored
   - Timing-dependent results

2. **Physics engine prioritizes dynamics**
   - Contact solver focuses on dynamic (mass > 0) bodies
   - Kinematic contacts are "low priority"

### The Solution

**Never rely on `getContactPoints()` for kinematic motion**

✅ **Use `getClosestPoints()`** for kinematics
✅ **Use `getContactPoints()`** only for physics logging

---

## Benchmark Results Interpretation

### Why `getContactPoints()` Often Appears Faster

1. **Post-stepSimulation cache hit**
   - Contact manifold already computed
   - Query is just a lookup

2. **Most pairs are non-colliding**
   - Empty contact list returned quickly
   - No distance computation needed

3. **Compiler-friendly code path**
   - Simple API, no branching

### Important Caveat

> **Speed ≠ Suitability**

- Fast `getContactPoints()` results are **only valid in Physics ON mode**
- For kinematics, `getClosestPoints()` is required **regardless of speed**

### Fair Comparison Requirements

Benchmarks must use:
- Same world state
- Same candidate pairs (after broad-phase)
- Same physics mode (ON/OFF)
- Same object types (physics/kinematic mix)

---

## FAQ

### Q: Why not always use `getClosestPoints()`?

**A**: In Physics ON mode with many actual contacts, `getContactPoints()` is:
- Faster (contact cache already exists)
- More accurate (actual contact manifold with normals, depths)

But for kinematics, `getClosestPoints()` is required for stability.

### Q: How do I disable collision for specific objects?

**A**: Use `CollisionMode.DISABLED`:
```python
obj.set_collision_mode(CollisionMode.DISABLED)  # No collision at all
```

This disables both PyBullet physics collision AND custom collision detection.

### Q: What's the difference between STATIC and DISABLED?

**A**:
- **STATIC**: Collision enabled but AABB never updated (for fixed structures)
- **DISABLED**: No collision at all (for visualization objects)

### Q: Can I change collision_mode at runtime?

**A**: Yes! Use `obj.set_collision_mode(new_mode)`:
```python
# Temporarily disable
obj.set_collision_mode(CollisionMode.DISABLED)
# ... do something ...
# Re-enable
obj.set_collision_mode(CollisionMode.NORMAL_3D)
```

All caches (AABB, spatial grid, movement tracking) are automatically updated.

---

## Conclusion

PyBulletFleet's **two-layer design** provides:

✅ **Default (Physics OFF)**:
- Fast, deterministic kinematics
- `getClosestPoints()` with safety margins
- No `stepSimulation()` overhead
- → **Best for production simulation**

✅ **Verification (Physics ON)**:
- Realistic physics behavior
- `getContactPoints()` for actual contacts
- Fixed-timestep `stepSimulation()`
- → **Best for debugging and validation**

This approach is **battle-tested**, **scales well**, and **breaks rarely**.

---

## Implementation Flow Diagram

### Physics OFF Mode (Kinematics)
```
User Code
    ↓
fleet.step()
    ↓
┌─────────────────────────────────────┐
│ Multi-Robot Simulation Step        │
├─────────────────────────────────────┤
│ 1. Update robot poses (kinematic)   │
│    - resetBasePositionAndOrientation│
│    - No stepSimulation() call       │
│                                     │
│ 2. Update AABB cache (if moved)     │
│    - getAABB() for moved objects    │
│    - Spatial hash update            │
│                                     │
│ 3. Broad-phase collision detection  │
│    - Grid-based AABB overlap        │
│    - Generate candidate pairs       │
│                                     │
│ 4. Narrow-phase collision check     │
│    - getClosestPoints(margin=0.02)  │
│    - Distance-based detection       │
│    - Returns near-miss pairs        │
│                                     │
│ 5. Return collision results         │
│    - List of (obj_i, obj_j, dist)   │
└─────────────────────────────────────┘
```

### Physics ON Mode (Dynamics)
```
User Code
    ↓
fleet.step()
    ↓
┌─────────────────────────────────────┐
│ Multi-Robot Simulation Step        │
├─────────────────────────────────────┤
│ 1. stepSimulation()                 │
│    - Integrate forces/velocities    │
│    - Solve contacts/constraints     │
│    - Update contact cache           │
│                                     │
│ 2. Update AABB cache                │
│    - getAABB() for all objects      │
│    - Spatial hash update            │
│                                     │
│ 3. Broad-phase collision detection  │
│    - Grid-based AABB overlap        │
│    - Generate candidate pairs       │
│                                     │
│ 4. Narrow-phase collision check     │
│    - getContactPoints()             │
│    - Contact manifold query         │
│    - Returns actual contacts        │
│                                     │
│ 5. Return collision results         │
│    - List of (obj_i, obj_j, depth)  │
└─────────────────────────────────────┘
```

---

**For implementation details, see**:
- `pybullet_fleet/core_simulation.py` - Main collision system implementation
- `pybullet_fleet/sim_object.py` - SimObject with collision_mode support
- `pybullet_fleet/types.py` - `CollisionDetectionMethod` and `CollisionMode` enums
- `benchmark/COLLISION_BENCHMARK_RESULTS.md` - Performance comparison

**Related documentation**:
- `docs/COLLISION_MODE_REDESIGN.md` - CollisionMode system design rationale
- `docs/spatial_hash_cell_size_modes.md` - Spatial hashing optimization
- `benchmark/PERFORMANCE_OPTIMIZATION_GUIDE.md` - Performance tuning guide

---

## Multi-Cell Registration for Large Objects

**Date**: 2026-01-21  
**Feature**: Automatic multi-cell spatial grid registration

### Problem Statement

Objects were previously registered to only a **single grid cell** (based on center point). This caused collision detection failures when:

- Large objects (AABB >> cell_size) span multiple cells
- Small robots near the edge of large objects are in different cells
- The spatial hash grid couldn't find neighbors across cell boundaries

**Example Failure Case**:
```
Grid cell size: 2.0m
Large wall AABB: 5.0m × 5.0m (center at Cell B)
Small robot: 0.5m × 0.5m (in Cell A)

[Cell A] [Cell B (wall center)] [Cell C]
  Robot                            

→ Robot only checks Cell A neighbors
→ Wall is in Cell B → NOT checked
→ Collision NOT detected ❌
```

### Solution: Automatic Multi-Cell Registration

Objects are now **automatically** registered to multiple cells based on their size:

1. **Size-based detection**: Objects with `max(extent_x, extent_y) > cell_size × threshold` use multi-cell
2. **No manual configuration**: The system automatically determines registration mode
3. **Configurable threshold**: Adjust sensitivity via `multi_cell_threshold` parameter (default: 1.5)

**How it works**:
```python
# Threshold calculation
threshold_size = cell_size × multi_cell_threshold
# Example: 2.0m × 1.5 = 3.0m

# Automatic decision
if object_extent > threshold_size:
    register_to_all_overlapping_cells()  # Multi-cell
else:
    register_to_single_cell()  # Normal
```

### Configuration

#### SimulationParams
```python
params = SimulationParams(
    spatial_hash_cell_size=2.0,  # 2m cells
    multi_cell_threshold=1.5,    # Objects > 3m use multi-cell
)
```

#### Behavior Examples

| Object Size | Cell Size | Threshold | Cells Used | Mode |
|-------------|-----------|-----------|------------|------|
| 0.8m        | 1.0m      | 1.5       | 1          | Single |
| 1.2m        | 1.0m      | 1.5       | 1          | Single |
| 2.0m        | 1.0m      | 1.5       | 4 (2×2)    | Multi |
| 5.0m        | 2.0m      | 1.5       | 9 (3×3)    | Multi |

#### Config File
```yaml
# config.yaml
spatial_hash_cell_size: 2.0
multi_cell_threshold: 1.5  # Default: 1.5
```

### Implementation Details

#### New Methods

**`_should_use_multi_cell_registration(object_id: int) -> bool`**
```python
# Automatically determines if multi-cell is needed
extent_x = aabb_max[0] - aabb_min[0]
extent_y = aabb_max[1] - aabb_min[1]
max_extent = max(extent_x, extent_y)  # XY plane only (2D compatible)

return max_extent > (self._cached_cell_size * self.params.multi_cell_threshold)
```

**`_get_overlapping_cells(object_id: int) -> List[Tuple[int, int, int]]`**
```python
# Calculates all cells that overlap with object's AABB
# Returns: [(x_min, y_min, z_min), ..., (x_max, y_max, z_max)]
```

#### Modified Data Structures

```python
# Before: Single cell per object
_cached_object_to_cell: Dict[int, Tuple[int, int, int]]

# After: Multiple cells per object
_cached_object_to_cell: Dict[int, List[Tuple[int, int, int]]]
```

#### Collision Detection Integration

```python
# _get_potential_collision_pairs_spatial_hash()
for obj_id_i in moved_objects:
    cells_i = self._cached_object_to_cell.get(obj_id_i, [])
    
    # Check neighbors in ALL cells this object occupies
    for cell_i in cells_i:
        for offset in neighbor_offsets:
            neighbor_cell = (cell_i[0] + offset[0], ...)
            
            # Find neighbors and check AABB overlap
            for obj_id_j in self._cached_spatial_grid.get(neighbor_cell, set()):
                # ... collision check
                pairs.add((min(i, j), max(i, j)))  # Auto-deduplication
```

### Performance Impact

#### Memory Usage
```python
# Example: 5m×5m wall, 2m cells → 3×3 = 9 cells
# Single cell mode: 1 object = 1 registration
# Multi-cell mode:   1 object = 9 registrations

# 100 large walls:
# Additional memory: 100 × (9-1) × 8 bytes ≈ 6.4 KB
# → Negligible impact
```

#### Computational Cost
- **Registration**: O(num_overlapping_cells) = O(1) for constant-size objects
- **Collision detection**: Duplicate pairs auto-filtered by `set()`
- **Update frequency**: Only when object moves or is added/removed

**Benchmark results**: No measurable performance degradation for typical scenarios (100-1000 objects)

### Design Rationale

#### Why Automatic vs Manual Flag?

**Considered**: Adding `enable_multi_cell: bool` flag to SimObject

**Chosen**: Automatic size-based detection

**Reasons**:
1. ✅ User doesn't need to configure each object
2. ✅ AABB changes automatically trigger re-evaluation
3. ✅ No configuration mistakes
4. ✅ Simpler API

#### Why 1.5x Default Threshold?

- **1.0x**: Too conservative, many objects at cell boundaries unnecessarily use multi-cell
- **1.5x**: Balanced - catches objects that clearly span multiple cells
- **2.0x**: Too aggressive, may miss some moderately large objects

#### Why XY Plane Only?

```python
max_extent = max(extent_x, extent_y)  # Ignore Z
```

**Reason**: Compatible with `CollisionMode.NORMAL_2D`, where Z-axis is typically height/elevation and doesn't affect horizontal collision detection.

### Usage Examples

#### Basic Usage (Fully Automatic)
```python
# No code changes needed!
sim_core = MultiRobotSimulationCore(params)

# Add large wall (5m × 5m)
wall = sim_core.add_structure(
    mesh_path="wall.obj",
    scale=[5.0, 5.0, 1.0],
    collision_mode=CollisionMode.STATIC
)
# → Automatically registered to 9 cells ✓

# Add small robot (0.5m × 0.5m)
robot = agent_manager.spawn_agent(spawn_params)
# → Automatically registered to 1 cell ✓

# Collision detection works correctly ✓
```

#### Custom Threshold
```python
# Conservative (more multi-cell objects)
params = SimulationParams(
    spatial_hash_cell_size=2.0,
    multi_cell_threshold=1.2,  # Objects > 2.4m use multi-cell
)

# Aggressive (fewer multi-cell objects)
params = SimulationParams(
    spatial_hash_cell_size=2.0,
    multi_cell_threshold=2.0,  # Objects > 4.0m use multi-cell
)
```

### Testing

**Test files**:
- `tests/test_multi_cell_internal.py` - Internal functionality test
- `examples/collision_features_demo.py` - Visual demonstration (multi-cell + 2D/3D modes)

**Run tests**:
```bash
python tests/test_multi_cell_internal.py
```

**Expected output**:
```
Robot 0 (ID=1):
  AABB extent: 0.80m
  Should use multi-cell: False ✓
  Registered to 1 cell(s) ✓

Distribution:
  4 object(s) registered to 1 cell(s) ✓
```

### Related Files

- `pybullet_fleet/core_simulation.py` - Implementation (`_should_use_multi_cell_registration`, `_get_overlapping_cells`)
- `pybullet_fleet/types.py` - Type definitions
- `tests/test_multi_cell_internal.py` - Unit tests
- `examples/collision_features_demo.py` - Demonstration
