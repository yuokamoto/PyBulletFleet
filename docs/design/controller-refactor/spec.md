# Controller Refactor — Unified Kinematic Controller

**Status:** Draft
**Date:** 2025-03-25
**Author:** Copilot + Human

## Problem

The current controller architecture has a fundamental split:

- **Velocity mode**: `VelocityController` subclasses (`OmniVelocityController`, `DifferentialVelocityController`) process `cmd_vel` commands.
- **Pose mode**: TPI (TwoPointInterpolation) logic is hardcoded inside `Agent` (~400 lines) and activated by setting `controller = None`.

The ROS bridge (`robot_handler.py`) manually toggles between these modes:
- `cmd_vel` → reattach `VelocityController`
- `goal_pose` / `path` → `set_controller(None)` → TPI mode

This creates several issues:
1. `set_controller(None)` is a hack — "no controller" means "use TPI"
2. TPI logic is tightly coupled to Agent internals
3. Mode switching logic is scattered across robot_handler.py
4. Adding new control modes requires modifying Agent.update() dispatch

## Decision Summary

| Decision | Choice |
|----------|--------|
| Architecture | **B** — Unified controller per kinematic model |
| Mode switching | **B3** — Input-based implicit + interruption rules |
| Class hierarchy | **H1** — Rename `VelocityController` → `KinematicController` |
| TPI ownership | **T1** — Move TPI from Agent into controllers |
| Backward compat | None — clean cut (no aliases) |
| Default controller | `DifferentialController` when none specified |

## Design

### New Class Hierarchy

```
Controller (ABC)                           # unchanged
  └── KinematicController (ABC)            # was VelocityController
        ├── OmniController                 # was OmniVelocityController
        └── DifferentialController         # was DifferentialVelocityController
```

### Controller Modes

Each `KinematicController` operates in one of three internal modes:

```
enum ControllerMode:
    IDLE       — no active command, zero velocity
    VELOCITY   — processing cmd_vel input
    POSE       — processing goal_pose / path via TPI
```

### Mode Switching Rules (B3)

| Current Mode | Input | Action |
|-------------|-------|--------|
| IDLE | `set_velocity()` | → VELOCITY |
| IDLE | `set_goal_pose()` / `set_path()` | → POSE |
| VELOCITY | `set_goal_pose()` / `set_path()` | Stop velocity → POSE |
| VELOCITY | watchdog timeout | → IDLE |
| POSE | `set_velocity()` | Cancel pose navigation → VELOCITY |
| POSE | trajectory complete | → IDLE |
| POSE | `set_goal_pose()` / `set_path()` | Restart POSE with new target |

### KinematicController ABC Interface

```python
class KinematicController(Controller, ABC):
    """Unified controller handling both velocity and pose commands."""

    mode: ControllerMode = IDLE

    # Velocity interface (existing)
    def set_velocity(vx, vy, vz, wx, wy, wz) → None

    # Pose interface (new)
    def set_goal_pose(agent, goal: Pose, **kwargs) → None
    def set_path(agent, path: List[Pose], **kwargs) → None

    # Template method
    def compute(agent, dt) → bool:
        if mode == VELOCITY:
            _check_velocity_timeout()
            return _apply_velocity(agent, dt)
        elif mode == POSE:
            return _apply_pose(agent, dt)
        else:  # IDLE
            return False

    # Abstract methods (subclass-specific)
    @abstractmethod
    def _apply_velocity(agent, dt) → bool
    @abstractmethod
    def _apply_pose(agent, dt) → bool
    @abstractmethod
    def _init_pose_trajectory(agent, goal: Pose) → None
```

### DifferentialController

Merges current `DifferentialVelocityController._apply_velocity()` + Agent's differential TPI logic:

- **VELOCITY mode**: Unicycle kinematics (v·cos(yaw), v·sin(yaw), wz)
- **POSE mode**: Two-phase TPI (ROTATE → FORWARD) with slerp interpolation
  - Owns `_tpi_rotation_angle`, `_tpi_forward`, `_slerp_precomp`
  - Phase state (`DifferentialPhase`) moves from controller-internal, not Agent

### OmniController

Merges current `OmniVelocityController._apply_velocity()` + Agent's omnidirectional TPI logic:

- **VELOCITY mode**: 6-DoF quaternion body→world rotation + Euler integration
- **POSE mode**: Per-axis TPI with distance-ratio velocity scaling
  - Owns `_tpi_pos[3]` instances

### Agent Changes

1. **Remove** all TPI fields (`_tpi_pos`, `_tpi_rotation_angle`, `_tpi_forward`, `_slerp_precomp`, etc.)
2. **Remove** TPI update methods (`_update_omnidirectional`, `_update_differential`, `_init_*_trajectory`)
3. **Simplify** `update()`:
   ```python
   if self._controller is not None:
       moved = self._controller.compute(self, dt)
   # no else — controller is always set
   ```
4. **Default controller**: If no controller specified at construction, set `DifferentialController()`
5. **`set_goal_pose()` / `set_path()`** → delegate to `self._controller.set_goal_pose()` / `set_path()`
6. **`set_motion_mode()`** → create and set the matching controller

### Bridge Changes

`robot_handler.py` simplifies dramatically:
- **`_cmd_vel_cb`**: Just store twist (no controller reattach needed)
- **`apply_cmd_vel`**: Call `controller.set_velocity(...)` — mode auto-switches
- **`_goal_pose_cb`**: Call `controller.set_goal_pose(agent, pose)` — mode auto-switches
- **`_path_cb`**: Call `controller.set_path(agent, waypoints)` — mode auto-switches
- **Remove** all `set_controller(None)` calls
- **Remove** `_vel_controller` field — just use `agent._controller`

`bridge_node.py`:
- Default controller is always set (DifferentialController for differential, OmniController for omni)
- No special-casing needed

### Registry

```python
register_controller("omni", OmniController)
register_controller("differential", DifferentialController)
```

### Config

```yaml
# Before
controller_config: "omni_velocity"

# After
controller_config: "omni"
```

No controller_config → `DifferentialController()` auto-assigned.

## Migration Impact

| File | Change |
|------|--------|
| `controller.py` | Rename + extend classes, add pose mode |
| `agent.py` | Remove ~400 lines TPI code, simplify update() |
| `__init__.py` | Update exports |
| `types.py` | Add `ControllerMode` enum |
| `robot_handler.py` | Simplify — remove mode switching |
| `bridge_node.py` | Simplify — default controller always set |
| All tests | Update imports, class names |
| Config YAMLs | Update registry names |
| Examples | Update if they reference old controller names |

## Success Criteria

- [ ] All existing 866 tests pass (updated for new names)
- [ ] Coverage ≥ 75%
- [ ] `agent.update()` has no TPI branching — single `controller.compute()` call
- [ ] Bridge never calls `set_controller(None)`
- [ ] Both velocity and pose modes work through same controller instance
- [ ] Mode auto-switches on input (B3 rules)
- [ ] Default controller is DifferentialController when none specified
- [ ] `make verify` passes
