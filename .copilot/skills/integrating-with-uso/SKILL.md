---
name: integrating-with-uso
description: "Use when implementing USO (Unified Simulation Orchestrator) integration for PyBulletFleet - SimulationNode adapter, snapshot serialization and deserialization, replay functionality, delta snapshot generation, or ZeroMQ messaging"
---

# Integrating with USO

Guide for integrating PyBulletFleet as a USO (Unified Simulation Orchestrator) simulation node — covering the SimulationNode adapter, snapshot serialization, and replay capability.

**USO Repository:** https://github.com/yuokamoto/Unified-Simulation-Orchestrator

## USO Architecture Overview

```
┌─────────────────────────────┐
│     Simulation Master       │  ← Time sync, snapshot merge, logging
└──────────┬──────────────────┘
           │ ZeroMQ (delta snapshots, control, events)
    ┌──────┼──────────┐
    ▼      ▼          ▼
 SimPy   Gazebo   PyBulletFleet   ← Each implements SimulationNode interface
 Node    Node     Node (NEW)
```

**Two modes:**
- **Single Mode** — Master + Node in one process, no ZeroMQ needed. **Start here.**
- **Distributed Mode** — Multiple nodes across servers, ZeroMQ PUB/SUB, barrier sync.

## SimulationNode Interface

Every USO node must implement:

```python
class SimulationNode:
    def initialize(self, full_snapshot, bt_xml): ...
    def step(self, delta_t): ...
    def collect_asset_states(self) -> dict: ...
    def send_state_update(self, delta_snapshot): ...
    def send_event(self, event): ...
    def shutdown(self): ...
```

### Mapping to PyBulletFleet

| USO Method | PyBulletFleet Implementation |
|-----------|------------------------------|
| `initialize(full_snapshot, bt_xml)` | Parse snapshot → spawn Agents/SimObjects via `from_params()` |
| `step(delta_t)` | Set `_params.timestep = delta_t`, call `step_once()` |
| `collect_asset_states()` | Iterate `sim.sim_objects` → build delta snapshot dict |
| `send_state_update(delta)` | Single mode: local. Distributed: ZeroMQ PUB |
| `send_event(event)` | Action completions, collision events → event dict |
| `shutdown()` | `p.disconnect()`, save final snapshot |

## Snapshot Format

### Full Snapshot (YAML)

```yaml
timestamp: 123.45
world:
  assets:
    robot_1:
      type: "robot"
      model: "robots/mobile_robot.urdf"
      position: [1.0, 2.0, 0.1]
      orientation: [0, 0, 0, 1]  # quaternion xyzw
      linear_velocity: [0.5, 0, 0]
      angular_velocity: [0, 0, 0.1]
      status: "moving"
      connected_to: "pallet_1"  # or null
      properties:
        battery_level: 0.85
```

### Delta Snapshot (JSON)

```json
{
  "type": "delta_snapshot",
  "timestamp": 130.0,
  "updated_assets": {
    "robot_1": {"position": [5.0, 1.0, 0.0], "status": "moving"}
  },
  "removed_assets": [],
  "new_assets": {}
}
```

### Field Mapping: USO → PyBulletFleet

| USO Field | PyBulletFleet Source | How to Get |
|-----------|---------------------|-----------|
| `asset_id` | `SimObject.name` | Direct |
| `type` | Agent→"robot", SimObject→"object" | `isinstance()` check |
| `model` | `Agent._urdf_path` or shape description | Store at spawn time |
| `position` | `SimObject.get_pose().position` | `[x, y, z]` list |
| `orientation` | `SimObject.get_pose().orientation` | `[qx, qy, qz, qw]` list |
| `linear_velocity` | `p.getBaseVelocity()[0]` | **New** — not currently exposed |
| `angular_velocity` | `p.getBaseVelocity()[1]` | **New** — not currently exposed |
| `status` | Action state or "idle"/"moving" | Map from `ActionStatus` |
| `connected_to` | `SimObject.get_attached_objects()` | First parent or null |

See [references/snapshot-mapping.md](references/snapshot-mapping.md) for detailed mapping.

## Implementation Phases

### Phase 1: Snapshot Serialization (Start Here)

No external dependencies. Pure Python.

```
□ SnapshotSerializer class
  ├── to_full_snapshot(sim_core) → dict
  ├── to_delta_snapshot(sim_core, moved_objects) → dict
  └── Uses _moved_this_step for delta detection

□ SnapshotDeserializer class
  ├── from_full_snapshot(snapshot_dict, sim_core) → spawns objects
  ├── apply_delta_snapshot(delta_dict, sim_core) → updates poses
  └── Handles new_assets / removed_assets / updated_assets

□ Tests: round-trip (serialize → deserialize → compare)
```

### Phase 2: Replay Controller

```
□ ReplayController class
  ├── load_log(log_path) → list of snapshots
  ├── play(speed=1.0) → apply snapshots sequentially
  ├── seek(timestamp) → jump to nearest full snapshot
  └── Physics OFF + kinematic control during replay

□ Logging: record snapshots during simulation
  ├── Full snapshot every N seconds (configurable)
  ├── Delta snapshot every step
  └── Event log for action completions
```

### Phase 3: USO Node Adapter (Requires USO Master)

```
□ PyBulletFleetNode(SimulationNode)
  ├── Single mode: direct method calls
  └── Distributed mode: ZeroMQ PUB/SUB

□ ZeroMQ messaging (distributed only)
  ├── Receive: step_start, reset, stop
  ├── Send: delta_snapshot, event
  └── Message format: JSON per USO spec
```

## Delta Snapshot Strategy

Leverage existing `_moved_this_step` set in `MultiRobotSimulationCore`:

```python
def to_delta_snapshot(self, sim_core) -> dict:
    delta = {"updated_assets": {}, "removed_assets": [], "new_assets": {}}

    for obj_id in sim_core._moved_this_step:
        obj = sim_core._id_to_object.get(obj_id)
        if obj is None:
            continue
        pose = obj.get_pose()
        delta["updated_assets"][obj.name] = {
            "position": pose.position.tolist(),
            "orientation": pose.orientation.tolist(),
        }
        # Add status if agent
        if isinstance(obj, Agent):
            delta["updated_assets"][obj.name]["status"] = (
                "moving" if obj.is_moving else "idle"
            )

    return delta
```

## Status Mapping

| PyBulletFleet State | USO Status String |
|--------------------|------------------|
| `agent.is_moving == True` | `"moving"` |
| `agent.is_moving == False`, no actions | `"idle"` |
| `ActionStatus.IN_PROGRESS` (Pick) | `"picking"` |
| `ActionStatus.IN_PROGRESS` (Drop) | `"dropping"` |
| `ActionStatus.FAILED` | `"error"` |
| SimObject (no agent) | `"static"` |

## Cross-References

- **REQUIRED:** working-with-pybullet-fleet — codebase domain knowledge
- **adding-sim-entities** — snapshot type mapping when adding new entity types
- **test-driven-development** — test snapshot round-trip before integration
- **Details:** See [references/snapshot-mapping.md](references/snapshot-mapping.md) for detailed field mapping
- **Details:** See [references/uso-spec-summary.md](references/uso-spec-summary.md) for condensed USO spec
