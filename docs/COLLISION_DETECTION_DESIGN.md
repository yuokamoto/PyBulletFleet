# PyBullet Collision Detection Design

**Based on**: "PyBullet Collision / Overlap 判定設計まとめ.docx"  
**Date**: 2026-01-19  
**Version**: 1.0

## Executive Summary

PyBulletFleet implements a **dual-mode collision detection system** optimized for both kinematics-focused and physics-based simulations:

- **Physics OFF Mode** (Default): Fast, deterministic, kinematics-safe collision detection
- **Physics ON Mode** (Verification): Realistic physics simulation with contact logging

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

3. **Reproducibility, Stability, Scalability**
   - Deterministic results across runs
   - Stable at high simulation speeds (Nx speed)
   - Scales to 100-1000+ objects

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
from pybullet_fleet.types import CollisionDetectionMethod

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

### Auto-Selection Logic

If `collision_detection_method` is not explicitly set:
- `physics=False` → `CLOSEST_POINTS` (kinematics-safe)
- `physics=True` → `CONTACT_POINTS` (physics-accurate)

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

### From Old API

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
    collision_detection_method=CollisionDetectionMethod.CLOSEST_POINTS,  # Auto-selected
    collision_margin=0.02,  # NEW: Safety clearance
)
```

### Config File Migration

**Old `config.yaml`**:
```yaml
physics: false
# collision_detection_method not specified
```

**New `config.yaml`** (auto-migrates, but explicit is better):
```yaml
physics: false
collision_detection_method: "closest_points"  # Explicit (recommended)
collision_margin: 0.02  # NEW: Safety margin
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

### Q: What about the benchmark showing `getContactPoints()` is faster?

**A**: True in Physics ON mode, but:
- Kinematics needs `getClosestPoints()` for **correctness**, not speed
- The benchmark assumes `stepSimulation()` is already called
- In Physics OFF, `getContactPoints()` is unreliable

### Q: How do I debug "missing" collisions?

**Checklist**:
1. ✅ Check `collision_detection_method` matches physics mode
2. ✅ Verify `collision_margin` is appropriate (not too small)
3. ✅ Ensure `stepSimulation()` is called if using `CONTACT_POINTS`
4. ✅ Check objects are not both static (static-static ignored)
5. ✅ Verify AABB/spatial hash is up to date

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

**For implementation details, see**:
- `pybullet_fleet/core_simulation.py` - Main implementation
- `pybullet_fleet/types.py` - `CollisionDetectionMethod` enum
- `benchmark/COLLISION_METHODS_REALISTIC_ANALYSIS.md` - Performance analysis
