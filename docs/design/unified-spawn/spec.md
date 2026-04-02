# Unified Entity Spawning + Batch Optimization

**Date:** 2026-03-22
**Status:** Draft

## Context

Currently PyBulletFleet's config-driven spawning (`parse_agent_config`, `spawn_from_config`) handles only Agent instances. SimObjects (pallets, obstacles, fixtures) must be created via Python API — they cannot be spawned from YAML config.

Additionally, batch spawning (`spawn_objects_batch`) has no performance optimization: each object triggers rendering updates and spatial grid recalculations (O(N²) in AUTO_ADAPTIVE mode).

## Decision

### 1. Unified Entity Config

Add a unified `entities:` section to YAML config where each entry has an explicit `type` field:

```yaml
entities:
  - type: agent
    name: robot0
    urdf_path: "robots/mobile_robot.urdf"
    pose: [0.0, 0.0, 0.05]
    yaw: 0.0
    controller_config:
      type: "omni_velocity"

  - type: sim_object
    name: pallet_0
    visual_shape:
      shape_type: box
      half_extents: [0.4, 0.3, 0.15]
      rgba_color: [0.6, 0.4, 0.2, 1.0]
    collision_shape:
      shape_type: box
      half_extents: [0.4, 0.3, 0.15]
    pose: [3.0, 1.0, 0.15]
    mass: 5.0
    collision_mode: "normal_3d"
```

- `type: agent` → parsed into `AgentSpawnParams`, spawned as `Agent`
- `type: sim_object` → parsed into `SimObjectSpawnParams`, spawned as `SimObject`
- `type` field defaults to ``"agent"`` when omitted (backward-compatible)
- Backward-compat: old ``robots:`` section continues to work (agent-only, no ``type`` field needed)

### 2. Batch Spawn Optimization

Wrap batch spawn in a context manager that:

1. **Disables rendering** before spawn loop (`p.configureDebugVisualizer(COV_ENABLE_RENDERING, 0)`)
2. **Defers spatial grid rebuild** — skip per-object `set_collision_spatial_hash_cell_size_mode()` during spawn, do ONE rebuild at end
3. **Re-enables rendering** after spawn loop

This turns O(N²) AUTO_ADAPTIVE overhead into O(N).

### 3. Entity Class Registry

Following the same pattern as the controller registry (`register_controller` / `create_controller`), add an **entity class registry** for custom class support:

```python
from pybullet_fleet.entity_registry import register_entity_class, create_entity

# Register custom class
register_entity_class("forklift", ForkliftAgent)

# YAML-driven spawn
entities:
  - type: forklift
    name: forklift0
    urdf_path: robots/forklift.urdf
```

- Built-in types auto-registered: `"agent"` → `Agent`, `"sim_object"` → `SimObject`
- Custom classes must be `SimObject` subclasses with `from_params(spawn_params, sim_core)` classmethod
- `parse_entity_config()` uses registry to resolve `type` → class
- Same pattern as controller registry — lightweight dict-based

### 4. Spawn Layout Extensibility (Deferred — YAGNI)

Function registry for spawn layouts (grid, random, circle, etc.) is **out of scope**.
Users can freely create `List[Dict]` or `List[SpawnParams]` in Python and pass them to `spawn_from_config()` / `spawn_objects_batch()`. This provides the same flexibility without adding registry machinery.

## Requirements

### Functional

- `parse_entity_config(entities_yaml)` → returns `List[Union[AgentSpawnParams, SimObjectSpawnParams]]`
- `parse_sim_object_config(dict)` → returns `SimObjectSpawnParams` (new, handles YAML dict → params)
- Entity class registry: `register_entity_class(name, cls)` / `create_entity(name, params, sim_core)`
- Built-in auto-registered: `"agent"` → `Agent`, `"sim_object"` → `SimObject`
- `SimObjectManager.spawn_entities_batch(entities_yaml)` → spawns mixed entities from config dicts
- Batch spawn context manager in `MultiRobotSimulationCore`: `batch_spawn()` context manager
- Backward-compat: existing `robots:` key and `parse_agent_config()` unchanged

### Non-functional

- All 815 existing tests pass
- Coverage ≥ 75%
- Batch spawn of 100 objects: spatial grid rebuild called once (not 100 times)

## Out of Scope

- Spawn layout plugins/registry (users generate dicts in Python)
- Grid/layout params in entity config (users pre-compute poses)
