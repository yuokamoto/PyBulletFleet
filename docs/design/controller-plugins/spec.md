# Controller Plugins — Registry + DifferentialVelocityController

**Date:** 2026-03-21
**Status:** Draft

## Context

Plugin Architecture Phase 2 delivered Controller ABC + OmniOmniVelocityController.
Two gaps remain before the ROS bridge and standalone use are fully pluggable:

1. No registry — controllers cannot be instantiated by name from config/YAML.
2. OmniOmniVelocityController is omni-only — no differential-drive kinematics.
3. `set_velocity()` takes world-frame input, forcing callers to do body→world conversion.

## Decision

- Add a lightweight **controller registry** (`register` / `create` by name).
- Add **DifferentialVelocityController** (unicycle model).
- Unify `set_velocity()` to **body-frame** input; each controller handles its own kinematics.
- Add `from_config(dict)` class method to Controller ABC for config-driven instantiation.
- Add `controller_config` field to `AgentSpawnParams` so config/YAML can specify per-robot controllers.

TPIController extraction is **out of scope** — deferred to a future phase.

## Requirements

### Controller Registry

- Dict-based: `CONTROLLER_REGISTRY: Dict[str, Type[Controller]]`
- `register_controller(name, cls)` + `create_controller(name, params) -> Controller`
- Built-in auto-registered: `"omni_velocity"`, `"differential_velocity"`
- Accessible from YAML config via `controller_config.type`

### Controller ABC Changes

- Add `from_config(cls, config: dict) -> Controller` classmethod (default: `cls()`)
- Add `set_velocity(vx, vy, vz, wz)` to ABC as optional interface (not abstract)

### OmniOmniVelocityController (body-frame)

- `set_velocity(vx, vy, vz, wz)` — accepts **body-frame** velocities
- `compute()` rotates body→world using agent yaw, then Euler integrates
- Breaking change from current world-frame `set_velocity` (only ROS bridge calls it)

### DifferentialVelocityController

- `set_velocity(vx, vy, vz, wz)` — uses `vx` as forward, ignores `vy`, uses `wz`
- Configurable: `max_linear_vel`, `max_angular_vel`, `wheel_separation` (informational)
- `compute()`: unicycle model `x += v*cos(yaw)*dt`, `y += v*sin(yaw)*dt`
- Clamps velocities to configured limits

### AgentSpawnParams

- New optional field: `controller_config: Optional[dict] = None`
- When set, `Agent.from_params()` calls `create_controller(config["type"], config)`

### ROS Bridge Impact

- `RobotHandler.apply_cmd_vel()` passes raw body-frame Twist → `set_velocity()`
- No more `twist_to_world_velocity()` call in RobotHandler
- `bridge_node` creates controller based on config param or defaults to `"omni_velocity"`

## Out of Scope

- TPIController extraction (future phase)
- EventBus integration
- entry_points / pip plugin discovery

## Success Criteria

- [ ] `DifferentialVelocityController` moves robot with unicycle kinematics
- [ ] `OmniOmniVelocityController` accepts body-frame and produces correct omni motion
- [ ] `create_controller("differential_velocity", {...})` works
- [ ] `AgentSpawnParams(controller_config={"type": "omni_velocity"})` auto-creates controller
- [ ] RobotHandler has no controller-type-specific logic
- [ ] All existing tests pass (backward compatible)
- [ ] New tests for DiffVelCtrl + registry
