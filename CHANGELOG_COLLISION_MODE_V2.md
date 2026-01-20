# CollisionMode System Update (v2.0)

## Date: 2026-01-20

## Summary

Major refactoring of collision detection system with per-object collision control.

## Breaking Changes

### 1. Removed Global `collision_check_2d` Parameter

**Before (v1.x)**:
```python
params = SimulationParams(
    collision_check_2d=True  # ❌ REMOVED: Global 2D/3D toggle
)
```

**After (v2.0)**:
```python
# Per-object collision mode
robot = SimObject(
    body_id=body_id,
    sim_core=sim_core,
    collision_mode=CollisionMode.NORMAL_2D  # ✅ Per-object setting
)
```

### 2. New CollisionMode System

Added per-object collision control:
- `CollisionMode.NORMAL_3D`: Full 3D collision (27 neighbors) - default
- `CollisionMode.NORMAL_2D`: 2D-only collision (9 neighbors, XY plane)
- `CollisionMode.STATIC`: One-time AABB registration, never updated
- `CollisionMode.DISABLED`: No collision detection (PyBullet + custom)

## Migration Guide

### Config Files

**Remove**:
```yaml
collision_check_2d: true  # ❌ No longer valid
```

**Add** (if needed):
```python
# Set per-object in code
from pybullet_fleet.types import CollisionMode

robot = SimObject(..., collision_mode=CollisionMode.NORMAL_2D)
```

### Code Changes

**Old API**:
```python
# Global setting
params = SimulationParams(collision_check_2d=True)

# Per-object override (deprecated)
obj = SimObject(..., collision_check_2d=False)
```

**New API**:
```python
# No global setting

# Per-object mode
from pybullet_fleet.types import CollisionMode

robot_2d = SimObject(..., collision_mode=CollisionMode.NORMAL_2D)
robot_3d = SimObject(..., collision_mode=CollisionMode.NORMAL_3D)
static_obj = SimObject(..., collision_mode=CollisionMode.STATIC, is_static=True)
marker = SimObject(..., collision_mode=CollisionMode.DISABLED)
```

## Updated Files

### Documentation

- `docs/COLLISION_DETECTION_DESIGN.md`: Updated with CollisionMode system
  - Added CollisionMode section
  - Updated migration guide
  - Updated FAQ
  - Updated examples

### Configuration Files

- `benchmark/configs/general.yaml`:
  - Removed `collision_check_2d` parameter
  - Updated scenario descriptions
  - Added CollisionMode usage notes

- `config/config.yaml`:
  - Removed `collision_check_2d` global setting
  - Added CollisionMode documentation section
  - Updated examples

### Benchmark Scripts

- `benchmark/performance_benchmark.py`: Removed `collision_check_2d` parameter
- `benchmark/experiments/collision_detection_methods_benchmark.py`: Removed `collision_check_2d`

## Features

### Runtime Collision Mode Changes

```python
# Disable collision temporarily
obj.set_collision_mode(CollisionMode.DISABLED)

# Re-enable
obj.set_collision_mode(CollisionMode.NORMAL_3D)
```

All caches (AABB, spatial grid, movement tracking) are automatically updated.

### Collision Disabled Mode

When `collision_mode == CollisionMode.DISABLED`:
1. PyBullet physics collision: Disabled via `setCollisionFilterPair`
2. Custom collision detection: Object excluded from spatial grid
3. AABB cache: Not maintained
4. Performance: No collision overhead

## Benefits

1. ✅ **Per-Object Control**: Mixed 2D/3D collision in same simulation
2. ✅ **Runtime Changes**: Dynamic collision mode switching
3. ✅ **Better Organization**: Clear separation of concerns
4. ✅ **Performance**: Optimized per-object (no global toggle)
5. ✅ **Flexibility**: Support for static and disabled modes

## Testing

Verified with benchmark scenarios:
- `collision_3d_full`: Full 3D collision (all objects NORMAL_3D)
- `collision_10hz`: Reduced frequency
- `no_collision`: Disabled collision checks

All scenarios pass successfully.

## Related Issues

- Resolves confusion between global and per-object settings
- Improves collision system maintainability
- Enables advanced use cases (static structures, visualization objects)

## References

- `docs/COLLISION_DETECTION_DESIGN.md` - Complete design documentation
- `docs/COLLISION_MODE_REDESIGN.md` - Original design rationale
- `benchmark/COLLISION_BENCHMARK_RESULTS.md` - Performance comparison
