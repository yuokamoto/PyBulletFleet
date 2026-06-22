# Unified Entity Spawning — Agent Specification

## Requirements

### Functional
- Entity class registry: `register_entity_class(name, cls)` + `create_entity(name, config, sim_core)`
- Built-in auto-registered: `"agent"` → `Agent`, `"sim_object"` → `SimObject`
- `parse_entity_config(entities_yaml)` dispatches by `type` field via entity registry
- `parse_sim_object_config(obj_def)` creates `SimObjectSpawnParams` from YAML dict (shape params, pose, mass, collision_mode)
- `MultiRobotSimulationCore.batch_spawn()` context manager: rendering off + deferred spatial grid
- `AgentManager.spawn_from_config()` uses batch_spawn context
- `AgentManager.spawn_entities_from_config(entities_yaml)` spawns mixed entities via registry
- Backward-compat: `parse_agent_config()`, `entities:` key, all existing spawn methods unchanged

### Non-functional
- All 815 existing tests pass
- Coverage ≥ 75%
- Batch of N objects → spatial grid rebuild called once (not N times)

## Constraints
- Python ≥ 3.8 (local), 3.12 (Docker)
- No new dependencies
- Config-driven code in `config_utils.py` (no PyBullet imports)
- Batch optimization in `core_simulation.py` (PyBullet API calls)
- Factory methods only — no direct `__init__()` on Agent or SimObject

## Approach

### File Changes

| File | Change |
|------|--------|
| `pybullet_fleet/entity_registry.py` | New: entity class registry (`register_entity_class`, `create_entity`) |
| `pybullet_fleet/config_utils.py` | Add `parse_sim_object_config()`, `parse_entity_config()` |
| `pybullet_fleet/core_simulation.py` | Add `batch_spawn()` context manager |
| `pybullet_fleet/agent_manager.py` | Add `spawn_entities_from_config()`, wrap `spawn_objects_batch()` in batch_spawn |
| `pybullet_fleet/__init__.py` | Export new functions + registry |
| `tests/test_config_spawn.py` | Tests for entity registry, parse_entity_config, parse_sim_object_config |
| `tests/test_core_simulation.py` | Tests for batch_spawn context manager |

### Implementation Order

**Task 1: `parse_sim_object_config()` — config_utils.py**

Parse a single YAML dict into `SimObjectSpawnParams`:

```python
def parse_sim_object_config(obj_def: Dict[str, Any]) -> "SimObjectSpawnParams":
    """Parse a sim_object definition dict into SimObjectSpawnParams.

    Keys:
        name (str, required): Object name.
        visual_shape (dict, optional): Visual shape definition.
            shape_type (str): "box", "sphere", "cylinder", "mesh"
            half_extents (list): For box [hx, hy, hz]
            radius (float): For sphere/cylinder
            height (float): For cylinder
            mesh_path (str): For mesh
            mesh_scale (list): For mesh [sx, sy, sz]
            rgba_color (list): [r, g, b, a]
        collision_shape (dict, optional): Same structure as visual_shape.
        pose (list): [x, y, z] position. Default [0, 0, 0].
        yaw (float): Yaw angle in radians. Default 0.0.
        mass (float): Mass in kg. Default 0.0 (static).
        pickable (bool): Default True.
        collision_mode (str): "normal_3d", "normal_2d", "static", "disabled".
            Default "normal_3d".
        user_data (dict): Custom metadata. Default {}.
    """
    from pybullet_fleet.sim_object import SimObjectSpawnParams, ShapeParams
    from pybullet_fleet.types import CollisionMode

    if "name" not in obj_def:
        raise ValueError(f"SimObject definition missing required 'name' field: {obj_def}")

    name = obj_def["name"]
    pose = obj_def.get("pose", [0.0, 0.0, 0.0])
    yaw = obj_def.get("yaw", 0.0)
    mass = obj_def.get("mass", 0.0)
    pickable = obj_def.get("pickable", True)
    user_data = obj_def.get("user_data", {})

    # Parse collision mode
    collision_mode_str = obj_def.get("collision_mode", "normal_3d")
    collision_mode = CollisionMode(collision_mode_str)

    # Parse shape params
    visual_shape = _parse_shape_params(obj_def.get("visual_shape")) if "visual_shape" in obj_def else None
    collision_shape = _parse_shape_params(obj_def.get("collision_shape")) if "collision_shape" in obj_def else None

    return SimObjectSpawnParams(
        visual_shape=visual_shape,
        collision_shape=collision_shape,
        initial_pose=Pose.from_yaw(pose[0], pose[1], pose[2], yaw),
        mass=mass,
        pickable=pickable,
        name=name,
        collision_mode=collision_mode,
        user_data=user_data,
    )


def _parse_shape_params(shape_dict: Dict[str, Any]) -> "ShapeParams":
    """Parse a shape definition dict into ShapeParams."""
    from pybullet_fleet.sim_object import ShapeParams

    return ShapeParams(
        shape_type=shape_dict.get("shape_type"),
        mesh_path=shape_dict.get("mesh_path"),
        mesh_scale=shape_dict.get("mesh_scale", [1.0, 1.0, 1.0]),
        half_extents=shape_dict.get("half_extents", [0.5, 0.5, 0.5]),
        radius=shape_dict.get("radius", 0.5),
        height=shape_dict.get("height", 1.0),
        rgba_color=shape_dict.get("rgba_color", [0.8, 0.8, 0.8, 1.0]),
    )
```

**Task 2: Entity class registry — entity_registry.py (NEW)**

Same pattern as controller registry:

```python
"""Entity class registry for config-driven spawning.

Mirrors the controller registry pattern: name → class mapping
with ``register_entity_class`` / ``create_entity``.

Built-in entity types are auto-registered at import time:
- ``"agent"`` → Agent
- ``"sim_object"`` → SimObject
"""

from typing import Dict, Optional, Type, Any

from pybullet_fleet.sim_object import SimObject

ENTITY_REGISTRY: Dict[str, Type[SimObject]] = {}


def register_entity_class(name: str, cls: Type[SimObject]) -> None:
    """Register an entity class under *name*."""
    ENTITY_REGISTRY[name] = cls


def create_entity(name: str, config: Dict[str, Any], sim_core: Any) -> SimObject:
    """Create an entity by registered *name* from *config* dict.

    The class must have a ``from_config(config, sim_core)`` classmethod
    or fall back to ``from_params(parse(config), sim_core)``.
    """
    if name not in ENTITY_REGISTRY:
        raise KeyError(f"Unknown entity type: {name!r}. Available: {list(ENTITY_REGISTRY)}")
    return ENTITY_REGISTRY[name]


# Auto-register built-ins (deferred to avoid circular imports):
def _register_builtins() -> None:
    from pybullet_fleet.agent import Agent
    register_entity_class("agent", Agent)
    register_entity_class("sim_object", SimObject)

_register_builtins()
```

**Task 3: `parse_entity_config()` — config_utils.py**

Dispatch by `type` field via entity registry:

```python
from typing import Union

def parse_entity_config(
    entities_yaml: List[Dict[str, Any]],
) -> List[Union["AgentSpawnParams", "SimObjectSpawnParams"]]:
    """Parse a list of entity definition dicts into spawn params.

    Each dict MUST have a ``type`` field. Built-in types:
    - ``"agent"`` → parsed by ``parse_agent_config`` (single item)
    - ``"sim_object"`` → parsed by ``parse_sim_object_config``
    Custom types registered via ``register_entity_class()`` are also supported.

    Args:
        entities_yaml: List of entity definition dicts.

    Returns:
        List of spawn params instances.

    Raises:
        ValueError: If ``type`` field is missing.
        KeyError: If ``type`` is not registered.
    """
    from pybullet_fleet.entity_registry import ENTITY_REGISTRY

    if not entities_yaml:
        return []

    result = []
    for entity_def in entities_yaml:
        entity_type = entity_def.get("type")
        if entity_type is None:
            raise ValueError(f"Entity definition missing required 'type' field: {entity_def}")

        if entity_type not in ENTITY_REGISTRY:
            raise KeyError(
                f"Unknown entity type: {entity_type!r}. "
                f"Available: {list(ENTITY_REGISTRY)}"
            )

        if entity_type == "agent":
            parsed = parse_agent_config([entity_def])
            result.extend(parsed)
        elif entity_type == "sim_object":
            result.append(parse_sim_object_config(entity_def))
        else:
            # Custom registered type — use same parsing as agent
            # (urdf + pose + name), class resolved at spawn time
            parsed = parse_agent_config([entity_def])
            result.extend(parsed)

    return result
```

**Task 3: `batch_spawn()` context manager — core_simulation.py**

```python
from contextlib import contextmanager

@contextmanager
def batch_spawn(self):
    """Context manager for optimized batch spawning.

    Disables rendering and defers spatial grid rebuild until
    the context exits. Use when spawning many objects at once.

    Example::

        with sim_core.batch_spawn():
            for params in params_list:
                Agent.from_params(params, sim_core)
        # rendering re-enabled, spatial grid rebuilt once
    """
    # Save current state
    was_adaptive = (
        self._params.spatial_hash_cell_size_mode == SpatialHashCellSizeMode.AUTO_ADAPTIVE
    )

    # Enter batch mode
    self._batch_spawning = True
    self.disable_rendering()

    try:
        yield
    finally:
        self._batch_spawning = False
        # One-shot spatial grid rebuild
        if was_adaptive:
            self.set_collision_spatial_hash_cell_size_mode()
        self._rebuild_spatial_grid()
        self.enable_rendering()
```

In `add_object()`, skip per-object rebuild when `_batch_spawning` is True:

```python
# In add_object(), replace the AUTO_ADAPTIVE block:
if self._params.spatial_hash_cell_size_mode == SpatialHashCellSizeMode.AUTO_ADAPTIVE:
    if not getattr(self, "_batch_spawning", False):
        self.set_collision_spatial_hash_cell_size_mode()
```

Add `_batch_spawning: bool = False` to `__init__`.

**Task 4: `spawn_entities_from_config()` — agent_manager.py**

```python
def spawn_entities_from_config(
    self, entities_yaml: List[Dict[str, Any]]
) -> Tuple[List[Agent], List[SimObject]]:
    """Spawn mixed Agent + SimObject entities from config dicts.

    Parses entities using ``parse_entity_config`` and spawns them
    inside a ``batch_spawn()`` context for optimal performance.

    Args:
        entities_yaml: List of entity definition dicts with ``type`` field.

    Returns:
        Tuple of (agents, sim_objects) spawned.
    """
    from .config_utils import parse_entity_config

    params_list = parse_entity_config(entities_yaml)

    agents: List[Agent] = []
    sim_objects: List[SimObject] = []

    with self.sim_core.batch_spawn():
        for params in params_list:
            if isinstance(params, AgentSpawnParams):
                agent = Agent.from_params(params, self.sim_core)
                self.add_object(agent)
                agents.append(agent)
            else:
                obj = SimObject.from_params(params, self.sim_core)
                sim_objects.append(obj)

    return agents, sim_objects
```

**Task 5: Wrap existing `spawn_objects_batch` — agent_manager.py**

Add batch_spawn context to existing method:

```python
def spawn_objects_batch(self, params_list: List[SimObjectSpawnParams]) -> List[T]:
    start = time.perf_counter() if self.enable_profiling else None
    objects = []

    # Use batch_spawn if sim_core supports it
    ctx = self.sim_core.batch_spawn() if self.sim_core and hasattr(self.sim_core, "batch_spawn") else nullcontext()
    with ctx:
        for params in params_list:
            obj = self._object_class.from_params(params, sim_core=self.sim_core)
            self.add_object(obj)
            objects.append(obj)

    if start is not None:
        elapsed = time.perf_counter() - start
        logger.info("spawn_objects_batch: %d objects in %.3f sec", len(params_list), elapsed)
    return objects
```

**Task 6: Exports — __init__.py**

```python
from pybullet_fleet.config_utils import parse_agent_config, parse_entity_config, parse_sim_object_config, load_yaml_config
from pybullet_fleet.entity_registry import register_entity_class, create_entity, ENTITY_REGISTRY
```

Add to `__all__`:
```python
"parse_entity_config",
"parse_sim_object_config",
"register_entity_class",
"create_entity",
```

### Test Plan

| Test | File | What it verifies |
|------|------|-----------------|
| `test_parse_sim_object_config_minimal` | test_config_spawn.py | name + defaults |
| `test_parse_sim_object_config_full` | test_config_spawn.py | All fields populated |
| `test_parse_sim_object_config_box_shape` | test_config_spawn.py | Visual + collision box |
| `test_parse_sim_object_config_missing_name` | test_config_spawn.py | ValueError |
| `test_parse_sim_object_config_collision_mode` | test_config_spawn.py | String → CollisionMode enum |
| `test_registry_builtins_registered` | test_config_spawn.py | "agent" and "sim_object" exist |
| `test_registry_register_and_lookup` | test_config_spawn.py | register custom + lookup |
| `test_registry_unknown_raises` | test_config_spawn.py | KeyError for unknown type |
| `test_registry_custom_class_in_parse_entity` | test_config_spawn.py | Custom type resolved via registry |
| `test_parse_entity_config_empty` | test_config_spawn.py | [] → [] |
| `test_parse_entity_config_agents_only` | test_config_spawn.py | All agent → AgentSpawnParams |
| `test_parse_entity_config_sim_objects_only` | test_config_spawn.py | All sim_object → SimObjectSpawnParams |
| `test_parse_entity_config_mixed` | test_config_spawn.py | Agent + SimObject mixed |
| `test_parse_entity_config_missing_type` | test_config_spawn.py | ValueError |
| `test_parse_entity_config_unknown_type` | test_config_spawn.py | KeyError |
| `test_batch_spawn_disables_rendering` | test_core_simulation.py | rendering off during context |
| `test_batch_spawn_defers_spatial_grid` | test_core_simulation.py | set_collision_spatial_hash_cell_size_mode called once at end |
| `test_batch_spawn_reenables_on_exception` | test_core_simulation.py | enable_rendering called even on error |
| `test_spawn_entities_from_config_mixed` | test_config_spawn.py | Agent + SimObject created and returned |
| `test_spawn_objects_batch_uses_batch_spawn` | test_config_spawn.py | batch_spawn context used |

### Existing Tests That Must Still Pass
- All 815 tests in full suite (including 32 tests in test_config_spawn.py)
- Coverage ≥ 75%

### References
- Controller registry pattern: `pybullet_fleet/controller.py` L50-73
- Config parsing: `pybullet_fleet/config_utils.py` (parse_agent_config at L14)
- SimObjectSpawnParams: `pybullet_fleet/sim_object.py` L83-137
- ShapeParams: `pybullet_fleet/sim_object.py` L26-82
- AgentManager spawn: `pybullet_fleet/agent_manager.py` L396-414 (spawn_objects_batch)
- Core add_object: `pybullet_fleet/core_simulation.py` L992-1055
- Spatial grid rebuild: `pybullet_fleet/core_simulation.py` L468
- disable/enable_rendering: `pybullet_fleet/core_simulation.py` L413-424
- Existing bridge config: `ros2_bridge/.../config/bridge_omni_demo.yaml`
