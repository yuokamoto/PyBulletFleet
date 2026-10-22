# Collision Detection Broad-Phase Details: Spatial Hash Grid

Implementation details of the broad-phase spatial hash grid — cell calculation,
AABB caching, neighbor search patterns, multi-cell registration, and
performance optimizations.

---

(cell-size-and-grid-mapping)=
## Cell Size & Grid Mapping

The spatial hash grid divides 3D space into uniform cells. The `cell_size` determines how large each cell is, and directly affects collision detection performance.

### Cell Size Calculation Modes

The system supports three modes. See [Collision Configuration Guide](spatial-hash-cell-size) for practical configuration.

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

### Calculation Algorithm

For auto modes (`auto_initial`, `auto_adaptive`):

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

### Position → Cell Mapping

```python
def get_cell(position: Tuple[float, float, float]) -> Tuple[int, int, int]:
    """Convert world position to grid cell indices"""
    x, y, z = position
    return (int(x / cell_size), int(y / cell_size), int(z / cell_size))
```

**Example**:
```
Object at position [3.2, 5.7, 1.1], cell_size=2.0
→ Cell = (int(3.2/2), int(5.7/2), int(1.1/2)) = (1, 2, 0)
```

### Usage

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

---

## AABB Caching

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

---

(neighbor-search-patterns)=
## Neighbor Search Patterns

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


2D Mode (9 neighbors between two NORMAL_2D bodies):
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

A NORMAL_2D body is registered in every z cell its AABB touches, so two
NORMAL_2D bodies that overlap in z always share a z layer. A NORMAL_3D body is
registered in its centre cell and can sit in the layer next to a NORMAL_2D body
it overlaps. A NORMAL_3D body searches all 27 cells, so it finds that pair
itself. A NORMAL_2D body searches 9 cells in each z layer it is registered in,
plus the 9 in a neighbouring layer only while a body that is not NORMAL_2D is
registered there (the simulation keeps a count per z layer), so a fleet of
NORMAL_2D robots pays for 9 cells. Either way the pair needs real overlap in z.

---

(multi-cell-registration)=
## Multi-Cell Registration

**Problem**: Large objects may span multiple cells

**Solution**: Register to all overlapping cells

With a positive `collision_margin` (the default), the rule below is not used. A
body whose AABB plus the margin fits in one cell on every axis is registered in
its centre cell only: two such bodies within the margin have centres less than a
cell apart, so the neighbour search finds them. Any larger body is registered in
every cell its AABB, enlarged by half the margin, touches. `multi_cell_threshold`
applies only when the margin is 0.

**Detection** (margin 0):
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

---

## Movement-Based Optimization

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

(incremental-updates)=
## Incremental Updates

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

---

## See Also

- [Collision Detection Overview](collision-overview) — Design goals, mode summary, architecture
- [Collision Detection Narrow-Phase Details](collision-internals) — System flow, narrow-phase APIs, mode details
- [Collision Configuration Guide](../how-to/collision-config) — Practical cell size configuration
