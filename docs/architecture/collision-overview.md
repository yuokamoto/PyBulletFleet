# Collision Detection Overview

A high-level overview of PyBulletFleet's collision detection system — design goals,
available modes, and the two-phase architecture.

---

## TL;DR

PyBulletFleet's collision detection system provides:

- ✅ **Scalability**: O(N) via spatial hashing (not O(N²))
- ✅ **Flexibility**: 4 collision modes per object
- ✅ **Performance**: Incremental updates, mode-based optimization
- ✅ **Determinism**: Same input → same output
- ✅ **Simplicity**: Automatic configuration, minimal setup

**Recommended Workflow**:
1. Start with default (physics=False, CLOSEST_POINTS)
2. Set collision modes (NORMAL_3D/2D, STATIC, DISABLED)
3. Run and profile
4. Optimize cell_size if needed
5. Use Physics mode when needed

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
- **Two-Mode Design**: Kinematics (fast) or Physics (accurate)

### Core Principles

1. **Scalability First**
   - O(N) broad-phase filtering via spatial hashing
   - Incremental updates (only moved objects)
   - Scales to **100-1000+ objects** in real-time

2. **Kinematics First, Physics When Needed**
   - **Default**: Pure kinematics (no `stepSimulation()`)
   - **Optional**: Physics mode

3. **Deterministic & Reproducible**
   - Same input → same output (critical for planning)
   - Stable at high speeds (10-100x real-time)
   - No race conditions or timing dependencies

4. **Per-Object Flexibility**
   - Each object can have different collision modes
   - Runtime mode changes supported
   - No global "one size fits all"

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

→ [Collision Detection Broad-Phase Details: Spatial Hash Grid](collision-spatial-hash)
→ [Collision Detection Narrow-Phase Details](narrow-phase-details-pybullet-apis)

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

**Concept**: Divide 3D space into uniform grid cells (→ [full details](collision-spatial-hash))

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

**Key Optimizations** (→ [Spatial Hash Grid](collision-spatial-hash)):
1. **[Incremental Updates](incremental-updates)**: Only update AABBs for moved objects
2. **[Mode-Based Neighbors](neighbor-search-patterns)**: 2D checks 9 cells, 3D checks 27 cells
3. **[Multi-Cell Registration](multi-cell-registration)**: Large objects registered to multiple cells

---

(system-flow-object-lifecycle)=
## System Flow & Object Lifecycle

This section explains **when and how** AABBs and spatial grids are updated throughout an object's lifetime.

**Note**: In standard usage with `run_simulation()`, movement and collision detection are automatically handled by the `step_once()` method. You don't need to manually call `check_collisions()` or `step_simulation()` unless you're implementing custom simulation logic.

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

**Note**: The examples below show explicit `check_collisions()` and `step_simulation()` calls for educational purposes. In production code using `run_simulation()`, these are called automatically in `step_once()`.

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
# → AABB calculated, registered to cell (x, y, z), added to _moved_this_step

# 2. Move
robot.set_pose([1.5, 2.0, 0.5])
# → AABB updated, grid cell updated, added to _moved_this_step

# 3. Check collisions
pairs = sim_core.check_collisions()
# → 27 neighbor cells checked → AABB overlap → getClosestPoints() → _moved_this_step cleared
```

#### NORMAL_3D (Moving Robot - Physics Mode)
```python
# 1. Create
robot = sim_core.add_object(body_id=1, collision_mode=CollisionMode.NORMAL_3D, mass=1.0)

# 2. Every simulation step
sim_core.step_simulation()
# → stepSimulation() updates all physics objects, auto-marked as moved, AABB updated

# 3. Check collisions (every step)
pairs = sim_core.check_collisions()
# → 27 neighbor cells checked → getContactPoints() or getClosestPoints()
```

#### STATIC (Fixed Wall)
```python
# 1. Create
wall = sim_core.add_structure(mesh="wall.obj", collision_mode=CollisionMode.STATIC)
# → AABB calculated ONCE, registered to grid cells (may be multi-cell)

# 2. No movement — static!

# 3. Check collisions
pairs = sim_core.check_collisions()
# → Wall is in grid, can be found by moving objects
# → Wall itself never triggers checks (not in _moved_this_step)
```

#### DISABLED (Visualization Marker)
```python
# 1. Create
marker = SimObject(body_id=3, collision_mode=CollisionMode.DISABLED)
# → AABB NOT calculated, NOT in grid, PyBullet collision disabled

# 2. Move (visual only)
marker.set_pose([x, y, z])
# → No AABB update, no grid update

# 3. Completely ignored in collision checks ✓
```

---

## FAQ & Troubleshooting

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

## Further Reading

- [Collision Detection Broad-Phase Details: Spatial Hash Grid](collision-spatial-hash) — Algorithm details, AABB caching, multi-cell registration
- [Collision Detection Narrow-Phase Details](collision-internals) — PyBullet APIs, collision mode implementation details, configuration
- [Collision Configuration Guide](../how-to/collision-config) — Practical configuration: detection method, modes, margins, cell size

**Source code**:
- `pybullet_fleet/core_simulation.py` — Main implementation
- `pybullet_fleet/sim_object.py` — SimObject with collision modes
- `pybullet_fleet/types.py` — Enums and types
- `tests/test_collision_comprehensive.py` — Comprehensive tests
