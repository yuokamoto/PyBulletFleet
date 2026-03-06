# Snapshot Field Mapping

Detailed field-by-field mapping between USO snapshot format and PyBulletFleet data sources.

## Table of Contents

1. [Full Snapshot Mapping](#full-snapshot-mapping)
2. [Delta Snapshot Strategy](#delta-snapshot-strategy)
3. [Replay: Snapshot → World](#replay-snapshot--world)
4. [Event Mapping](#event-mapping)
5. [Implementation Notes](#implementation-notes)

## Full Snapshot Mapping

### Serialization: PyBulletFleet → USO Snapshot

| USO Field | PyBulletFleet Source | Code |
|-----------|---------------------|------|
| `timestamp` | `sim_core.sim_time` | `float` (seconds) |
| `asset_id` | `obj.name` | String, unique per object |
| `type` | Instance type | `"robot"` if Agent, `"object"` if SimObject |
| `model` | URDF path or shape desc | Agent: `agent._urdf_path`; SimObject: reconstruct from spawn params |
| `position` | Pose position | `obj.get_pose().position.tolist()` → `[x, y, z]` |
| `orientation` | Pose quaternion | `obj.get_pose().orientation.tolist()` → `[qx, qy, qz, qw]` |
| `linear_velocity` | PyBullet velocity | `p.getBaseVelocity(obj.object_id)[0]` → `[vx, vy, vz]` |
| `angular_velocity` | PyBullet velocity | `p.getBaseVelocity(obj.object_id)[1]` → `[wx, wy, wz]` |
| `status` | Agent state | See status mapping in SKILL.md |
| `connected_to` | Attachments | `obj.get_attached_objects()` → parent name or null |
| `properties` | Custom | `obj.user_data` dict (arbitrary key-value) |

### Deserialization: USO Snapshot → PyBulletFleet

| USO Field | PyBulletFleet Target | Code |
|-----------|---------------------|------|
| `type: "robot"` | `Agent.from_params()` | Create AgentSpawnParams from snapshot fields |
| `type: "object"` | `SimObject.from_params()` | Create SimObjectSpawnParams |
| `model` | `urdf_path` param | Used to load correct URDF/mesh |
| `position` + `orientation` | `initial_pose` | `Pose(np.array(pos), np.array(orient))` |
| `connected_to` | `agent.attach_object()` | Resolve by name after all objects spawned |
| `properties` | `user_data` | Pass through as dict |

**Velocity handling during deserialization:**
- `physics: true` → `p.resetBaseVelocity(obj_id, linearVelocity, angularVelocity)`
- `physics: false` → Velocities are computed from motion controller, snapshot velocity ignored

## Delta Snapshot Strategy

### What goes in updated_assets

Only objects in `_moved_this_step` set. For each:

```python
entry = {
    "position": [x, y, z],
    "orientation": [qx, qy, qz, qw],
}

# Add velocity only if physics enabled
if sim_core._params.physics:
    lin_vel, ang_vel = p.getBaseVelocity(obj_id)
    entry["linear_velocity"] = list(lin_vel)
    entry["angular_velocity"] = list(ang_vel)

# Add status only if changed
if isinstance(obj, Agent):
    entry["status"] = get_status_string(obj)
```

### What goes in new_assets

Objects spawned since last snapshot. Track via spawn hooks:
- `SimObject.__init__` calls `sim_core.add_object()` → hook here to record new assets
- Build full asset entry (same as full snapshot) for new objects

### What goes in removed_assets

Objects removed since last snapshot. Track via:
- `sim_core.remove_object()` → hook to record removed asset names

## Replay: Snapshot → World

### From Full Snapshot (initialization / seek)

1. Clear current simulation (remove all objects)
2. For each asset in snapshot:
   - Determine type (Agent vs SimObject) from `type` field
   - Create SpawnParams from snapshot fields
   - Spawn via `from_params()`
3. Resolve `connected_to` links (attach objects)
4. Set `sim_time` from snapshot `timestamp`

### From Delta Snapshot (sequential playback)

1. For each `updated_assets`:
   - Find object by name: `sim_core._name_to_object[name]`
   - `obj.set_pose(Pose(position, orientation))`
   - Update status display if applicable
2. For each `new_assets`: spawn new objects
3. For each `removed_assets`: `sim_core.remove_object()`

**Important:** During replay, always use:
- `physics: false` (kinematic control)
- Skip `stepSimulation()`
- Skip action queue processing

## Event Mapping

| PyBulletFleet Event | USO Event Type | Details |
|--------------------|---------------|---------|
| Action completed | `task_completed` | `{"asset_id": agent.name, "task_id": action_type}` |
| Action failed | `task_failed` | `{"asset_id": agent.name, "error": reason}` |
| New collision detected | `collision_start` | `{"asset_a": name_a, "asset_b": name_b}` |
| Collision resolved | `collision_end` | `{"asset_a": name_a, "asset_b": name_b}` |
| Object spawned | `asset_added` | `{"asset_id": name}` |
| Object removed | `asset_removed` | `{"asset_id": name}` |

## Implementation Notes

### Object Naming
USO uses `asset_id` as string keys. PyBulletFleet uses integer `object_id` internally. The `name` property on SimObject/Agent is the bridge. Ensure all objects have unique names — use `sim_core._name_to_object` for lookup.

### Velocity Exposure
PyBulletFleet currently does not expose `linear_velocity` or `angular_velocity` as public properties on SimObject. For snapshot serialization, either:
- Add `get_velocity()` method to SimObject (preferred)
- Call `p.getBaseVelocity()` directly (works but breaks abstraction)

### Joint State in Snapshots
USO snapshot spec supports `properties` dict for custom data. For URDF agents with joints:
```python
properties["joint_positions"] = agent.get_all_joints_state()
```
This enables full state reconstruction including arm configuration.

### Attachment Reconstruction
`connected_to` is a single string (asset_id). PyBulletFleet supports multiple attachments. Strategy:
- Serialize: first attached child's name, or null
- For complex attachment trees, use `properties["attached_objects"]` list
