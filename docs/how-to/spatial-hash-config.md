# Spatial Hash Cell Size Configuration

## Overview

The spatial hash collision detection system uses a configurable cell size that determines how space is partitioned for efficient collision queries. Three modes are available to balance performance and adaptivity.

## Configuration Modes

### Mode 1: `constant` (Optimized, Manual Setting)

Fixed cell size provided by user. Fastest option with no runtime overhead.

**Use When:**
- Object sizes are known in advance
- Object size distribution doesn't change
- Maximum performance is critical
- You can benchmark to find optimal cell_size

**Configuration:**
```python
from pybullet_fleet import SimulationParams, MultiRobotSimulationCore

params = SimulationParams(
    spatial_hash_cell_size_mode="constant",
    spatial_hash_cell_size=2.5,  # meters (required for constant mode)
)

sim = MultiRobotSimulationCore(params)
```

**YAML Configuration:**
```yaml
spatial_hash_cell_size_mode: "constant"
spatial_hash_cell_size: 2.5  # meters
```

**Pros:**
- ✓ Zero runtime overhead (no recalculation)
- ✓ Predictable performance
- ✓ Best for production with known workloads

**Cons:**
- ✗ Requires manual tuning
- ✗ Suboptimal if object sizes change significantly

**Recommended cell_size:**
- Typical robots (0.5m-2m): `cell_size = 2.0 - 4.0m`
- Small objects (< 0.5m): `cell_size = 1.0 - 2.0m`
- Large structures (> 5m): `cell_size = 5.0 - 10.0m`
- Mixed sizes: `cell_size = 2.0 * median_extent`

---

### Mode 2: `auto_adaptive` (Auto-Adaptive, Dynamic Update)

Automatically recalculates cell_size when objects are added or removed.

**Use When:**
- Object count changes frequently during simulation
- Object size distribution varies significantly
- Adaptive performance is more important than consistency
- Development/prototyping phase

**Configuration:**
```python
params = SimulationParams(
    spatial_hash_cell_size_mode="auto_adaptive",
)

sim = MultiRobotSimulationCore(params)

# cell_size recalculates automatically on add_object/remove_object
obj = SimObject(body_id=body_id, sim_core=sim)  # Triggers recalculation
sim.remove_object(obj)  # Triggers recalculation again
```

**YAML Configuration:**
```yaml
spatial_hash_cell_size_mode: "auto_adaptive"
```

**Pros:**
- ✓ Always optimal for current object set
- ✓ No manual tuning required
- ✓ Handles dynamic workloads well

**Cons:**
- ✗ Overhead on every add_object/remove_object
- ✗ Spatial grid rebuild can be expensive (O(N))
- ✗ Performance may vary during simulation

**Performance Impact:**
- Recalculation time: ~0.5-2ms per 1000 objects
- Spatial grid rebuild: ~1-5ms per 1000 objects
- Acceptable for <100 add/remove operations per second

---

### Mode 3: `auto_initial` (Initial Auto-Calculation, Manual Trigger) **[DEFAULT]**

Calculates cell_size once at first collision check. Can be manually recalculated.

**Use When:**
- Most objects are spawned before simulation starts
- Object sizes are relatively consistent
- You want automatic tuning without runtime overhead
- **Most common use case**

**Configuration:**
```python
params = SimulationParams(
    spatial_hash_cell_size_mode="auto_initial",  # Default mode
)

sim = MultiRobotSimulationCore(params)

# Spawn all objects
for i in range(1000):
    obj = SimObject(body_id=body_ids[i], sim_core=sim)

# First collision check calculates cell_size from existing objects
sim.check_collisions()  # cell_size calculated here

# Add more objects later - cell_size does NOT recalculate
new_obj = SimObject(body_id=new_body, sim_core=sim)

# Manual recalculation (optional)
sim.set_collision_spatial_hash_cell_size_mode()
```

**YAML Configuration:**
```yaml
spatial_hash_cell_size_mode: "auto_initial"  # or omit (default)
```

**Pros:**
- ✓ Automatic tuning at startup
- ✓ Zero runtime overhead after initialization
- ✓ Best balance of performance and convenience
- ✓ Manual trigger available when needed

**Cons:**
- ✗ Suboptimal if objects added later differ significantly in size
- ✗ Requires manual recalculation for large changes

**Best Practice:**
1. Spawn representative objects before first simulation step
2. Let auto_initial calculate optimal cell_size
3. Manually recalculate if object distribution changes >50%

---

## Manual Recalculation API

### Basic Usage

For `auto_initial` mode, you can manually trigger recalculation:

```python
# Recalculate cell_size from current objects
new_cell_size = sim.set_collision_spatial_hash_cell_size_mode()
print(f"Cell size recalculated: {new_cell_size:.3f}m")

# Check current cell_size without recalculating
current_cell_size = sim._cached_cell_size
print(f"Current cell_size: {current_cell_size:.3f}m")
```

### Dynamic Mode Switching

You can dynamically change the mode and cell_size:

```python
# Switch to constant mode with specific size
sim.set_collision_spatial_hash_cell_size_mode(
    mode=SpatialHashCellSizeMode.CONSTANT,
    cell_size=3.0
)

# Switch to auto_adaptive mode
sim.set_collision_spatial_hash_cell_size_mode(
    mode=SpatialHashCellSizeMode.AUTO_ADAPTIVE
)

# Override just the cell_size while keeping current mode
sim.set_collision_spatial_hash_cell_size_mode(
    cell_size=5.0  # Only valid if mode is CONSTANT
)
```

**Note:** When you provide `mode` or `cell_size` parameters, they permanently update `params.spatial_hash_cell_size_mode` and `params.spatial_hash_cell_size`.
```

**When to recalculate:**
- After spawning many new objects with different sizes
- After removing large structures
- When collision detection performance degrades

**Note:** Cannot recalculate in `constant` mode (will log warning)

---

## Algorithm Details

### Cell Size Calculation

For all auto modes (`auto_adaptive`, `auto_initial`):

```python
# 1. Get all object AABBs
aabbs = [p.getAABB(obj.body_id) for obj in objects]

# 2. Calculate maximum extent for each object
extents = [max(aabb[1][i] - aabb[0][i] for i in range(3)) for aabb in aabbs]

# 3. Use median extent (robust to outliers)
median_extent = sorted(extents)[len(extents) // 2]

# 4. cell_size = 2 * median_extent (covers typical neighbors)
cell_size = max(median_extent * 2.0, 0.5)  # minimum 0.5m
```

**Why median?**
- Robust to outliers (large structures don't skew grid)
- Represents typical object size in mixed environments
- Better than mean for heterogeneous scenes

### Spatial Grid Mapping

```python
# Object center → grid cell
center = [(aabb[0][i] + aabb[1][i]) / 2 for i in range(3)]
cell = (int(center[0] / cell_size),
        int(center[1] / cell_size),
        int(center[2] / cell_size))
```

---

## Configuration from YAML

Example `config.yaml`:

```yaml
# Option 1: Constant (fastest)
spatial_hash_cell_size_mode: "constant"
spatial_hash_cell_size: 2.5

# Option 2: Auto-adaptive (most flexible)
spatial_hash_cell_size_mode: "auto_adaptive"

# Option 3: Auto-initial (recommended default)
spatial_hash_cell_size_mode: "auto_initial"

# Other collision settings
collision_check_frequency: 0.1  # seconds
# NOTE: collision_mode is per-agent (set on AgentSpawnParams, not here)
# Example: collision_mode: normal_2d  → 9 neighbors (XY plane)
# Example: collision_mode: normal_3d  → 27 neighbors (full 3D, default)
```

Load configuration:

```python
sim = MultiRobotSimulationCore.from_yaml("config.yaml")
```

---

## Performance Comparison

| Mode | Initialization | Add/Remove Object | Collision Check | Best For |
|------|---------------|-------------------|-----------------|----------|
| `constant` | Manual setup | O(1) | O(N) | Production, known workloads |
| `auto_adaptive` | O(N log N) | **O(N log N)** | O(N) | Dynamic environments |
| `auto_initial` | O(N log N) | O(1) | O(N) | **Most use cases** |

---

## FAQ

**Q: Which mode should I use?**
- Start with `auto_initial` (default)
- Switch to `constant` if you can benchmark optimal cell_size
- Use `auto_adaptive` only if object count changes frequently

**Q: Can I change mode at runtime?**
No, mode is set at initialization via `SimulationParams`.

**Q: What happens if cell_size is too small/large?**
- Too small: Many grid cells, higher memory usage, slower neighbor queries
- Too large: Fewer cells, more objects per cell, slower AABB filtering
- Optimal: ~2-8 objects per cell on average

**Q: Does this affect collision accuracy?**
No, cell_size only affects broad-phase performance. Narrow-phase collision detection (PyBullet's `getClosestPoints`) is always accurate.


---

## See Also

- PyBulletFleet README
- Collision Detection Optimization
- Benchmark Results
