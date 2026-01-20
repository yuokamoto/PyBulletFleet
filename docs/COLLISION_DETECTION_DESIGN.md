# PyBullet Collision Detection Design

**Based on**: "PyBullet Collision / Overlap 判定設計まとめ.docx"
**Date**: 2026-01-20
**Version**: 2.0 (Updated with CollisionMode system)

## Executive Summary

PyBulletFleet implements a **multi-mode collision detection system** with per-object collision control:

- **CollisionMode.NORMAL_3D**: Full 3D collision detection (27 neighbors)
- **CollisionMode.NORMAL_2D**: 2D-only collision detection (9 neighbors, XY plane)
- **CollisionMode.STATIC**: One-time AABB registration (never updated)
- **CollisionMode.DISABLED**: Collision completely disabled (no PyBullet collision, no custom detection)

This design prioritizes **collision avoidance** over **collision resolution**, making it ideal for warehouse automation, path planning, and safety verification.

---

## Design Philosophy

### Core Principles

1. **"Will it collide?" > "Did it collide?"**
   - Focus on predictive safety margins, not reactive contact resolution
   - Clearance-based detection (safety distance) preferred over penetration detection

2. **Kinematics First, Physics When Needed**
   - Main use case: Kinematic motion planning and collision avoidance
   - Secondary use case: Physics-based verification and debugging

3. **Per-Object Collision Control**
   - Each object can have different collision modes
   - Runtime collision mode changes supported
   - No global 2D/3D toggle (deprecated)

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

## Migration Guide

### From Old API (v1.x → v2.0)

#### 1. Global `collision_check_2d` → Per-Object `collision_mode`

**Before** (global 2D/3D toggle):
```python
params = SimulationParams(
    collision_check_2d=True,  # ❌ DEPRECATED: Global setting
)
```

**After** (per-object mode):
```python
# Simulation params (no global 2D setting)
params = SimulationParams(
    physics=False,
    collision_detection_method=CollisionDetectionMethod.CLOSEST_POINTS,
)

# Per-object collision mode
robot = SimObject(
    body_id=body_id,
    sim_core=sim_core,
    collision_mode=CollisionMode.NORMAL_2D,  # ✅ Per-object 2D
)
```

#### 2. Explicit Collision Detection Method

**Before** (implicit physics assumption):
```python
params = SimulationParams(
    physics=False,  # But collision detection assumes physics...
)
```

**After** (explicit kinematics mode):
```python
params = SimulationParams(
    physics=False,
    collision_detection_method=CollisionDetectionMethod.CLOSEST_POINTS,  # Auto-selected, but explicit is better
    collision_margin=0.02,  # NEW: Safety clearance
)
```

#### 3. Static Objects

**Before** (manual registration):
```python
# Old API had register_static_object() method
sim_core.register_static_object(obj)
```

**After** (CollisionMode.STATIC):
```python
structure = SimObject(
    body_id=body_id,
    sim_core=sim_core,
    collision_mode=CollisionMode.STATIC,  # ✅ Built-in static mode
    is_static=True,  # Also set is_static flag
)
```

#### 4. Disabling Collision

**Before** (no built-in support):
```python
# Had to manually exclude objects or use workarounds
```

**After** (CollisionMode.DISABLED):
```python
# At creation
marker = SimObject(
    body_id=body_id,
    sim_core=sim_core,
    collision_mode=CollisionMode.DISABLED,  # ✅ No collision
)

# Runtime toggle
obj.set_collision_mode(CollisionMode.DISABLED)  # Disable
obj.set_collision_mode(CollisionMode.NORMAL_3D)  # Re-enable
```

### Config File Migration

**Old `config.yaml`** (v1.x):
```yaml
simulation:
  physics: false
  collision_check_2d: true  # ❌ DEPRECATED: Global setting
```

**New `config.yaml`** (v2.0):
```yaml
simulation:
  physics: false
  collision_detection_method: "closest_points"  # Explicit (recommended)
  collision_margin: 0.02  # NEW: Safety margin
  # collision_check_2d removed - use per-object collision_mode instead
```

**Benchmark config migration**:
```yaml
# OLD
scenarios:
  collision_2d_10hz:
    simulation:
      collision_check_2d: true  # ❌ REMOVED

# NEW
scenarios:
  collision_10hz:
    simulation:
      collision_check_frequency: 10.0
    # Per-object collision_mode set in code, not config
```

---

## FAQ

### Q: Can I use PyBullet without calling `stepSimulation()`?

**A**: Yes! PyBullet is both:
1. **Physics Engine** (requires `stepSimulation()`)
2. **Collision Geometry Engine** (works without `stepSimulation()`)

For pure kinematics, you only use (2), which is perfectly valid.

### Q: Why not always use `getClosestPoints()`?

**A**: In Physics ON mode with many actual contacts, `getContactPoints()` is:
- Faster (contact cache already exists)
- More accurate (actual contact manifold with normals, depths)

But for kinematics, `getClosestPoints()` is required for stability.

### Q: What happened to `collision_check_2d` parameter?

**A**: **Removed in v2.0**. Use per-object `collision_mode` instead:
- Old: `collision_check_2d=True` (global setting for all objects)
- New: `collision_mode=CollisionMode.NORMAL_2D` (per-object setting)

This allows mixed 2D/3D collision in the same simulation (e.g., ground robots + drones).

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

### Q: How do I debug "missing" collisions?

**Checklist**:
1. ✅ Check `collision_detection_method` matches physics mode
2. ✅ Verify `collision_margin` is appropriate (not too small)
3. ✅ Ensure `stepSimulation()` is called if using `CONTACT_POINTS`
4. ✅ Check objects are not both static (static-static ignored if `ignore_static_collision=True`)
5. ✅ Verify objects are not in DISABLED mode
6. ✅ Check AABB/spatial grid is up to date

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
