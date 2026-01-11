# Per-Object collision_check_2d Implementation Guide

## Overview

PyBullet Fleet now supports per-object `collision_check_2d` configuration, allowing mixed 2D/3D collision detection in the same simulation. This enables optimal performance for scenarios with both ground-based agents (AGVs) and aerial/3D-moving objects (drones, lifted pallets).

## Architecture

### Hierarchy

```
SimulationParams.collision_check_2d (global default)
    ↓
SimObject.collision_check_2d (per-object override)
    ↓
Collision detection uses object-specific setting
```

### Implementation Flow

1. **Global Default**: `SimulationParams.collision_check_2d` (default: `False` = 3D)
2. **Object Override**: `SimObject.collision_check_2d` (default: `None` = use global)
3. **Collision Check**: `filter_aabb_pairs()` checks each object's setting

### Priority

- If `obj.collision_check_2d` is `None`: Use global `params.collision_check_2d`
- If `obj.collision_check_2d` is `True/False`: Use object-specific setting

## API

### SimObjectSpawnParams

```python
@dataclass
class SimObjectSpawnParams:
    collision_check_2d: Optional[bool] = None
    # None = use global default
    # True = 2D collision (9 neighbors)
    # False = 3D collision (27 neighbors)
```

### AgentSpawnParams

```python
@dataclass
class AgentSpawnParams(SimObjectSpawnParams):
    collision_check_2d: Optional[bool] = None
    # Inherits from SimObjectSpawnParams
```

### SimObject

```python
class SimObject:
    def __init__(
        self,
        body_id: int,
        sim_core=None,
        collision_check_2d: Optional[bool] = None,
        ...
    ):
        self.collision_check_2d = collision_check_2d
```

## Usage Examples

### Example 1: All 2D (Simple Warehouse)

```python
# Global setting: 2D for all objects
params = SimulationParams(collision_check_2d=True)

# Agents automatically use 2D collision
agv_params = AgentSpawnParams(urdf_path="mobile_robot.urdf")
# agv_params.collision_check_2d = None → uses global True
```

### Example 2: Mixed 2D/3D (Warehouse with Drones)

```python
# Global setting: 3D (conservative default)
params = SimulationParams(collision_check_2d=False)

# AGVs: Override to 2D for performance
agv_params = AgentSpawnParams(
    urdf_path="mobile_robot.urdf",
    collision_check_2d=True,  # 2D optimization
)

# Drones: Use global 3D setting
drone_params = AgentSpawnParams(
    urdf_path="drone.urdf",
    collision_check_2d=False,  # Explicit 3D
    # Or collision_check_2d=None to use global
)
```

### Example 3: Dynamic Objects (Lifted Pallets)

```python
# Global setting: 2D for most objects
params = SimulationParams(collision_check_2d=True)

# Floor pallets: Use global 2D setting
floor_pallet = SimObjectSpawnParams(
    visual_shape=ShapeParams(...),
    mass=5.0,
    # collision_check_2d=None → uses global True
)

# Lifted pallets: Override to 3D
lifted_pallet = SimObjectSpawnParams(
    visual_shape=ShapeParams(...),
    mass=5.0,
    collision_check_2d=False,  # 3D when lifted
)
```

## Performance Impact

### 2D Collision (collision_check_2d=True)

- **Neighbor checks**: 9 (3×3 in XY plane)
- **Performance**: ~67% faster collision checks
- **Use case**: Ground-based robots, fixed Z-position objects

### 3D Collision (collision_check_2d=False)

- **Neighbor checks**: 27 (3×3×3 cube)
- **Performance**: Full accuracy, slower
- **Use case**: Drones, lifted objects, vertical movement

### Mixed Scenario

**Example: 100 AGVs + 10 drones**

```
AGVs (90%):  collision_check_2d=True  → 9 neighbors each
Drones (10%): collision_check_2d=False → 27 neighbors each

Effective neighbor checks:
  AGV-AGV:   9 neighbors (majority of checks)
  AGV-Drone: 9 neighbors (AGV uses 2D, drone ignored in Z-axis)
  Drone-Drone: 27 neighbors (both use 3D)

Overall: ~60% reduction in collision checks compared to all-3D
```

## Implementation Details

### Collision Pair Logic

```python
# In filter_aabb_pairs()
for obj_i in objects:
    use_2d_i = obj_i.collision_check_2d if obj_i.collision_check_2d is not None else global_default
    
    for offset in all_neighbor_offsets:
        # Skip Z-axis neighbors if object uses 2D mode
        if use_2d_i and offset[2] != 0:
            continue
        
        for obj_j in neighbor_objects:
            use_2d_j = obj_j.collision_check_2d if obj_j.collision_check_2d is not None else global_default
            
            # Skip this pair if object j uses 2D and offset has Z component
            if use_2d_j and offset[2] != 0:
                continue
            
            # Check collision...
```

### Key Design Decisions

1. **Independent per-object checking**: Each object determines its own neighbor search independently
2. **Conservative Z-axis handling**: If either object uses 2D mode, Z-axis neighbors are skipped for that object
3. **Global default fallback**: `None` means use global setting, allowing centralized control

## Configuration

### config.yaml

```yaml
# Global default: 3D collision for all objects
collision_check_2d: false

# Or: 2D collision for all objects
# collision_check_2d: true
```

### Per-Object Override (Code)

```python
# In agent spawning
params = AgentSpawnParams(
    urdf_path="robot.urdf",
    collision_check_2d=True,  # Override global setting
)
```

## Testing

### Unit Tests

```python
# Test per-object collision mode
def test_per_object_collision_mode():
    # Global: 3D
    params = SimulationParams(collision_check_2d=False)
    sim = MultiRobotSimulationCore(params=params)
    
    # Object 1: Override to 2D
    obj1 = SimObject.from_params(
        SimObjectSpawnParams(collision_check_2d=True),
        sim_core=sim
    )
    
    # Object 2: Use global (3D)
    obj2 = SimObject.from_params(
        SimObjectSpawnParams(),
        sim_core=sim
    )
    
    assert obj1.collision_check_2d == True
    assert obj2.collision_check_2d == None  # Will use global False
```

### Integration Test

See `examples/mixed_2d_3d_collision_demo.py` for full demo.

## Migration Guide

### From Global-Only to Per-Object

**Before:**
```python
# All objects use same setting
params = SimulationParams(collision_check_2d=True)
```

**After:**
```python
# Set global default
params = SimulationParams(collision_check_2d=False)

# Override specific objects
agv_params = AgentSpawnParams(collision_check_2d=True)
drone_params = AgentSpawnParams(collision_check_2d=False)
```

### No Breaking Changes

- Existing code continues to work
- `collision_check_2d=None` (default) uses global setting
- Global-only simulations work unchanged

## Best Practices

1. **Set conservative global default**: Use `collision_check_2d=False` (3D) as global default
2. **Override for performance**: Set `collision_check_2d=True` for ground-based agents
3. **Explicit for safety-critical**: Always set explicitly for drones, lifted objects
4. **Document per-object settings**: Add comments explaining why each object uses 2D/3D

## See Also

- `examples/mixed_2d_3d_collision_demo.py` - Full demonstration
- `examples/performance_demo.py` - Performance comparison
- `benchmark/README.md` - Performance benchmarking tools
- `config/config.yaml` - Global configuration example

---

**Last Updated:** 2026-01-08  
**Version:** PyBullet Fleet (post per-object collision_check_2d)
