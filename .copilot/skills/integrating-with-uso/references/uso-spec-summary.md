# USO Specification Summary

Condensed reference for the Unified Simulation Orchestrator (USO) specification.
Source: https://github.com/yuokamoto/Unified-Simulation-Orchestrator

## Architecture Overview

USO provides a unified interface layer between simulation engines and business logic:

```
Business Logic / Orchestration
        ↕ (USO API)
   SimulationMaster (orchestrator)
        ↕ (ZeroMQ messages)
   SimulationNode(s) (adapters wrapping individual sims)
```

### Operational Modes

| Mode | Description |
|------|-------------|
| **Single** | One SimulationMaster, one SimulationNode in same process |
| **Distributed** | One SimulationMaster, multiple SimulationNodes across network |

PyBulletFleet integration targets **Single mode** first.

## SimulationNode Interface

The core adapter contract that PyBulletFleet must implement:

```
class SimulationNode:
    # Lifecycle
    initialize(config) → bool
    shutdown() → void

    # Simulation Control
    step(dt: float) → Snapshot
    reset() → Snapshot

    # Asset Management
    add_asset(asset_config) → asset_id
    remove_asset(asset_id) → bool
    get_asset_state(asset_id) → AssetState
    set_asset_state(asset_id, state) → bool

    # World Queries
    get_full_snapshot() → FullSnapshot
    get_delta_snapshot() → DeltaSnapshot
```

### Method Mapping to PyBulletFleet

| USO Method | PyBulletFleet Equivalent |
|-----------|-------------------------|
| `initialize(config)` | `CoreSimulation(**params)` |
| `shutdown()` | `sim_core.close()` |
| `step(dt)` | `sim_core.step_simulation()` |
| `reset()` | `sim_core.reset_simulation()` |
| `add_asset(config)` | `sim_core.add_agent()` / `sim_core.add_object()` |
| `remove_asset(id)` | `sim_core.remove_object()` |
| `get_asset_state(id)` | `obj.get_pose()` + status |
| `set_asset_state(id, st)` | `obj.set_pose()` + state updates |
| `get_full_snapshot()` | Serialize all objects (see snapshot-mapping.md) |
| `get_delta_snapshot()` | Serialize `_moved_this_step` set |

## Snapshot Format

### Full Snapshot

```yaml
timestamp: 1234567890.123
assets:
  robot_0:
    type: "robot"
    model: "mobile_robot.urdf"
    position: [1.0, 2.0, 0.0]
    orientation: [0.0, 0.0, 0.0, 1.0]
    linear_velocity: [0.1, 0.0, 0.0]
    angular_velocity: [0.0, 0.0, 0.05]
    status: "moving"
    connected_to: null
    properties:
      joint_positions: [0.0, 1.57, 0.0]
  box_1:
    type: "object"
    model: "simple_cube.urdf"
    position: [3.0, 4.0, 0.5]
    orientation: [0.0, 0.0, 0.0, 1.0]
    status: "idle"
    connected_to: "robot_0"
    properties: {}
```

### Delta Snapshot

```yaml
timestamp: 1234567890.456
updated_assets:
  robot_0:
    position: [1.1, 2.0, 0.0]
    orientation: [0.0, 0.0, 0.02, 0.9998]
    status: "moving"
new_assets: {}
removed_assets: []
events:
  - type: "collision_start"
    data: {asset_a: "robot_0", asset_b: "box_1"}
```

### Snapshot Types

| Type | When Used | Content |
|------|-----------|---------|
| Full | On `reset()`, seek, initial state | All assets with complete state |
| Delta | Every `step()` return | Only changed assets since last snapshot |

### Supported Serialization

- YAML (human-readable, default for files)
- JSON (wire format, lower overhead)
- MessagePack (binary, via ZeroMQ, lowest overhead)

## Messaging (ZeroMQ)

### Patterns Used

| Pattern | Use Case |
|---------|----------|
| REQ/REP | Command/response (step, reset, add_asset) |
| PUB/SUB | Snapshot streaming, event broadcast |

### Default Ports

| Port | Purpose |
|------|---------|
| 5555 | Command channel (REQ/REP) |
| 5556 | Snapshot stream (PUB/SUB) |

### Message Format

```json
{
  "type": "command|response|snapshot|event",
  "id": "unique-message-id",
  "timestamp": 1234567890.123,
  "payload": { ... }
}
```

## Logging and Replay

### Recording
- Full snapshot at start (or periodic keyframes)
- Delta snapshots every step
- Events logged inline with snapshots

### Replay Modes

| Mode | Description |
|------|-------------|
| **Visual replay** | Load snapshots → set poses kinematically |
| **State restore** | Load full snapshot → reconstruct simulation state |
| **Seek** | Jump to full snapshot keyframe, then apply deltas forward |

### File Format

```yaml
recording:
  metadata:
    simulator: "PyBulletFleet"
    version: "0.1.0"
    start_time: 1234567890.0
    step_dt: 0.016
  keyframes:
    - timestamp: 0.0
      type: full
      snapshot: { ... }
    - timestamp: 0.016
      type: delta
      snapshot: { ... }
```

## Key Design Decisions

1. **Snapshot is the truth** — All state exchange happens via snapshots, not individual property gets/sets
2. **Adapters are thin** — SimulationNode should delegate to existing simulation APIs, not duplicate logic
3. **Delta-first** — Only transmit changes to minimize bandwidth
4. **Serialization-agnostic** — Internal representation is dict; serialize to YAML/JSON/MessagePack at boundaries
5. **Events are optional** — Events enrich snapshots but are not required for basic operation
