# Collision Configuration Guide

All collision-related settings in one place — detection method, per-object
modes, safety margin, spatial hash cell size, and multi-cell threshold.

---

## Quick Decision Guide

```
┌─ What kind of simulation? ─────────────────────────────────┐
│                                                             │
│  Kinematics (path planning, fleet coordination)             │
│  → physics: false                                           │
│  → collision_detection_method: "closest_points" (auto)      │
│  → collision_margin: 0.02                                   │
│                                                             │
│  Physics (contact verification, dynamics)                   │
│  → physics: true                                            │
│  → collision_detection_method: "contact_points" (auto)      │
│                                                             │
│  Mixed (some physics + some kinematic objects)              │
│  → physics: true                                            │
│  → collision_detection_method: "hybrid"                     │
│  → collision_margin: 0.05                                   │
└─────────────────────────────────────────────────────────────┘
```

---

## Collision Detection Method

Controls **how** collisions are confirmed in the narrow phase.

| Method | Requires `stepSimulation()` | Safety Margin | Best For |
|--------|-----------------------------|---------------|----------|
| `closest_points` | No | ✅ Yes | Kinematics (default) |
| `contact_points` | Yes | No | Physics |
| `hybrid` | Yes | Partial | Mixed scenes |

### Configuration

**YAML**:
```yaml
physics: false
collision_detection_method: "closest_points"  # or "contact_points", "hybrid"
```

**Python**:
```python
from pybullet_fleet.core_simulation import SimulationParams
from pybullet_fleet.types import CollisionDetectionMethod

params = SimulationParams(
    physics=False,
    collision_detection_method=CollisionDetectionMethod.CLOSEST_POINTS,
)
```

### Auto-Selection

If `collision_detection_method` is not specified, it is selected automatically:

```python
if physics == False:
    collision_detection_method = "closest_points"  # Kinematics-safe
else:
    collision_detection_method = "contact_points"  # Physics-accurate
```

For architecture details, see [Narrow-Phase Details](narrow-phase-details-pybullet-apis).

---

## Per-Object Collision Mode

Controls collision behavior **per object**. Set at spawn time via `AgentSpawnParams` or `SimObject` constructor.

```python
from pybullet_fleet.types import CollisionMode
```

| Mode | Updates | Grid | Neighbors | Use Case |
|------|---------|------|-----------|----------|
| `NORMAL_3D` | Every move | Yes | 27 (3×3×3) | Drones, 3D robots |
| `NORMAL_2D` | Every move | Yes | 9 (3×3×1) | Ground robots, AGVs |
| `STATIC` | Once | Yes | — | Walls, shelves |
| `DISABLED` | Never | No | 0 | Markers, visualization |

### Setting via AgentSpawnParams

```python
from pybullet_fleet.types import CollisionMode
from pybullet_fleet.agent import Agent, AgentSpawnParams

# Ground robot (2D collision)
params = AgentSpawnParams(
    urdf_path="robot.urdf",
    collision_mode=CollisionMode.NORMAL_2D,
)
agent = Agent.from_params(params, sim_core)
```

### Setting via SimObject

```python
from pybullet_fleet.sim_object import SimObject

# Fixed structure
wall = SimObject(
    body_id=body_id,
    sim_core=sim_core,
    collision_mode=CollisionMode.STATIC,
    is_static=True,
)

# Visualization marker (no collision)
marker = SimObject(
    body_id=body_id,
    sim_core=sim_core,
    collision_mode=CollisionMode.DISABLED,
)
```

### Runtime Mode Changes

```python
# Temporarily disable collision
obj.set_collision_mode(CollisionMode.DISABLED)
# ... do something ...
obj.set_collision_mode(CollisionMode.NORMAL_3D)  # Re-enable
```

For detailed mode behavior, see [Collision Mode Details](collision-mode-details).

---

(collision-margin)=
## Collision Margin

Safety clearance for `getClosestPoints()`. Only affects `CLOSEST_POINTS` and `HYBRID` modes.

```yaml
collision_margin: 0.02  # meters (2cm default)
```

| Use Case | Margin | Rationale |
|----------|--------|-----------|
| Tight spaces | 0.01–0.02 m | Minimal clearance |
| Normal operation | 0.02–0.05 m | Comfortable safety |
| Conservative | 0.05–0.10 m | Extra safety for high-speed |
| Contact detection only | 0.0 m | Actual touching only |

---

(spatial-hash-cell-size)=
## Spatial Hash Cell Size

Controls how 3D space is partitioned for the broad-phase collision grid. Three modes are available.

### Mode 1: `auto_initial` (Default — Recommended)

Calculates `cell_size` once at first collision check from existing object sizes.

```yaml
spatial_hash_cell_size_mode: "auto_initial"  # or omit (default)
```

```python
params = SimulationParams(
    spatial_hash_cell_size_mode="auto_initial",
)
```

- ✅ Automatic tuning at startup
- ✅ Zero runtime overhead after initialization
- ✅ Best balance of performance and convenience
- Spawn representative objects before first simulation step
- Manually recalculate if object distribution changes >50%

### Mode 2: `constant` (Fastest)

Fixed cell size provided by user. No runtime overhead.

```yaml
spatial_hash_cell_size_mode: "constant"
spatial_hash_cell_size: 2.5  # meters (required)
```

```python
params = SimulationParams(
    spatial_hash_cell_size_mode="constant",
    spatial_hash_cell_size=2.5,
)
```

- ✅ Zero overhead, predictable performance
- ✅ Best for production with known workloads
- Requires manual tuning / benchmarking

**Recommended `cell_size`**:
- Typical robots (0.5–2 m): `2.0–4.0 m`
- Small objects (< 0.5 m): `1.0–2.0 m`
- Large structures (> 5 m): `5.0–10.0 m`
- Mixed sizes: `2.0 × median_extent`

### Mode 3: `auto_adaptive` (Dynamic)

Automatically recalculates `cell_size` when objects are added or removed.

```yaml
spatial_hash_cell_size_mode: "auto_adaptive"
```

```python
params = SimulationParams(
    spatial_hash_cell_size_mode="auto_adaptive",
)
```

- ✅ Always optimal for current object set
- ✅ Handles dynamic workloads
- Overhead on every add/remove (~0.5–2 ms per 1000 objects)

### Manual Recalculation

For `auto_initial` mode, you can manually trigger recalculation:

```python
# Recalculate from current objects
new_cell_size = sim.set_collision_spatial_hash_cell_size_mode()
print(f"Cell size: {new_cell_size:.3f} m")

# Switch mode at runtime
from pybullet_fleet.types import SpatialHashCellSizeMode

sim.set_collision_spatial_hash_cell_size_mode(
    mode=SpatialHashCellSizeMode.CONSTANT,
    cell_size=3.0,
)
```

### Performance Comparison

| Mode | Initialization | Add/Remove | Collision Check | Best For |
|------|---------------|------------|-----------------|----------|
| `auto_initial` | O(N log N) | O(1) | O(N) | **Most use cases** |
| `constant` | Manual | O(1) | O(N) | Production, known workloads |
| `auto_adaptive` | O(N log N) | O(N log N) | O(N) | Dynamic environments |

For the cell size calculation algorithm, see [Cell Size & Grid Mapping](cell-size-and-grid-mapping).

---

## Multi-Cell Threshold

Controls when large objects are registered in multiple spatial-hash cells.
Objects larger than `cell_size × multi_cell_threshold` span multiple cells.

```yaml
multi_cell_threshold: 1.5  # default
```

| Value | Meaning |
|-------|---------|
| 1.0 | Every object exceeding one cell is multi-registered |
| **1.5** | Default — objects >1.5× the cell size span multiple cells |
| 2.0+ | Only very large objects are multi-registered |

**Example**: With `cell_size=2.0` and `multi_cell_threshold=1.5`, objects larger than 3.0 m are multi-registered.

For implementation details, see [Multi-Cell Registration](multi-cell-registration).

---

## Complete YAML Reference

```yaml
# Simulation mode
physics: false  # true for physics, false for kinematics

# Detection method (auto-selected if omitted)
collision_detection_method: "closest_points"  # or "contact_points", "hybrid"

# Safety margin (CLOSEST_POINTS / HYBRID only)
collision_margin: 0.02  # meters

# Spatial hash cell size
spatial_hash_cell_size_mode: "auto_initial"  # "constant" or "auto_adaptive"
# spatial_hash_cell_size: 2.0  # required only for "constant" mode

# Large object threshold
multi_cell_threshold: 1.5

# Static-static collision (usually not needed)
ignore_static_collision: false

# NOTE: collision_mode is per-object (set on AgentSpawnParams / SimObject)
# collision_mode: normal_2d  → 9 neighbors (XY plane)
# collision_mode: normal_3d  → 27 neighbors (full 3D, default)
# collision_mode: static     → Fixed, never updated
# collision_mode: disabled   → No collision at all
```

---

## See Also

- [Collision Detection Overview](../architecture/collision-overview) — Design goals and architecture
- [Collision Detection Broad-Phase Details: Spatial Hash Grid](../architecture/collision-spatial-hash) — Algorithm details
- [Collision Detection Narrow-Phase Details](../architecture/collision-internals) — System flow and narrow-phase APIs
- [Configuration Files Guide](../configuration/reference) — Pre-built configuration files
