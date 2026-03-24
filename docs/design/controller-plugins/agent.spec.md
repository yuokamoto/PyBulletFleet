# Controller Plugins — Agent Specification

## Requirements

### Functional
- Controller registry: name → class mapping, `create_controller(name, params)`
- `DifferentialVelocityController`: unicycle model, configurable vel limits
- `OmniOmniVelocityController`: body-frame input, internal body→world rotation
- `Controller.from_config(dict)`: classmethod factory on ABC
- `AgentSpawnParams.controller_config`: optional dict for auto-creation
- `RobotHandler.apply_cmd_vel()`: pass raw Twist, no type checks

### Non-functional
- Zero overhead when controller is None (legacy path unchanged)
- All 783 existing tests must pass
- Coverage ≥ 75%

## Constraints
- Python ≥ 3.8 (local), 3.12 (Docker)
- No ROS dependency in pybullet_fleet/ (ROS only in ros2_bridge/)
- `Controller` ABC in `pybullet_fleet/controller.py`
- Registry in same file (small, no separate module needed)
- `from_config()` must not import ROS types

## Approach

### File Changes

| File | Change |
|------|--------|
| `pybullet_fleet/controller.py` | Add registry, `from_config()`, `DifferentialVelocityController` |
| `pybullet_fleet/agent.py` | `AgentSpawnParams.controller_config`, auto-create in `from_params()` |
| `pybullet_fleet/__init__.py` | Export `DifferentialVelocityController` |
| `ros2_bridge/.../robot_handler.py` | Simplify `apply_cmd_vel()` |
| `ros2_bridge/.../bridge_node.py` | Use registry for controller creation |
| `tests/test_controller.py` | Add DiffVelCtrl tests, registry tests |

### Implementation Order

**Task 1: Registry + from_config (controller.py)**

```python
# At module level in controller.py
from typing import Dict, Type

CONTROLLER_REGISTRY: Dict[str, Type["Controller"]] = {}

def register_controller(name: str, cls: Type["Controller"]) -> None:
    CONTROLLER_REGISTRY[name] = cls

def create_controller(name: str, config: dict | None = None) -> "Controller":
    if name not in CONTROLLER_REGISTRY:
        raise KeyError(f"Unknown controller: {name!r}. Available: {list(CONTROLLER_REGISTRY)}")
    cls = CONTROLLER_REGISTRY[name]
    return cls.from_config(config or {})
```

Add to `Controller` ABC:
```python
class Controller(ABC):
    @classmethod
    def from_config(cls, config: dict) -> "Controller":
        return cls()
```

**Task 2: OmniOmniVelocityController → body-frame**

Current:
```python
def set_velocity(self, vx=0.0, vy=0.0, vz=0.0, wz=0.0):
    self._velocity_command[:] = [vx, vy, vz]  # world-frame
```

New:
```python
def set_velocity(self, vx=0.0, vy=0.0, vz=0.0, wz=0.0):
    """Set velocity command (body frame). compute() rotates to world."""
    self._body_velocity[:] = [vx, vy, vz]
    self._angular_velocity_command = wz

def compute(self, agent, dt):
    yaw = agent.get_pose().yaw
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    world_vx = self._body_velocity[0] * cos_y - self._body_velocity[1] * sin_y
    world_vy = self._body_velocity[0] * sin_y + self._body_velocity[1] * cos_y
    world_vz = self._body_velocity[2]
    # Euler integrate...
```

**Task 3: DifferentialVelocityController**

```python
class DifferentialVelocityController(Controller):
    def __init__(self, max_linear_vel=2.0, max_angular_vel=1.5, wheel_separation=0.0):
        self._max_linear_vel = max_linear_vel
        self._max_angular_vel = max_angular_vel
        self._wheel_separation = wheel_separation
        self._v = 0.0   # forward velocity (body frame)
        self._wz = 0.0  # angular velocity

    @classmethod
    def from_config(cls, config):
        return cls(
            max_linear_vel=config.get("max_linear_vel", 2.0),
            max_angular_vel=config.get("max_angular_vel", 1.5),
            wheel_separation=config.get("wheel_separation", 0.0),
        )

    def set_velocity(self, vx=0.0, vy=0.0, vz=0.0, wz=0.0):
        """Body-frame: vx = forward, vy ignored, wz = rotation."""
        self._v = max(-self._max_linear_vel, min(vx, self._max_linear_vel))
        self._wz = max(-self._max_angular_vel, min(wz, self._max_angular_vel))

    def compute(self, agent, dt):
        if abs(self._v) < 1e-9 and abs(self._wz) < 1e-9:
            agent._current_velocity[:] = 0.0
            agent._current_angular_velocity = 0.0
            return False

        pose = agent.get_pose()
        yaw = pose.yaw
        dx = self._v * math.cos(yaw) * dt
        dy = self._v * math.sin(yaw) * dt
        dyaw = self._wz * dt
        new_pose = Pose.from_yaw(pose.x + dx, pose.y + dy, pose.z, yaw + dyaw)
        agent.set_pose(new_pose)
        # Set reported velocity in world frame
        agent._current_velocity[:] = [
            self._v * math.cos(yaw),
            self._v * math.sin(yaw),
            0.0,
        ]
        agent._current_angular_velocity = self._wz
        return True

    def on_stop(self, agent):
        self._v = 0.0
        self._wz = 0.0
```

Register both at module bottom:
```python
register_controller("omni_velocity", OmniOmniVelocityController)
register_controller("differential_velocity", DifferentialVelocityController)
```

**Task 4: AgentSpawnParams.controller_config**

```python
@dataclass
class AgentSpawnParams:
    # ... existing fields ...
    controller_config: Optional[Dict[str, Any]] = None
```

In `Agent.from_params()`:
```python
if spawn_params.controller_config:
    from pybullet_fleet.controller import create_controller
    ctrl_config = spawn_params.controller_config
    ctrl = create_controller(ctrl_config["type"], ctrl_config)
    agent.set_controller(ctrl)
```

**Task 5: RobotHandler simplification**

```python
def apply_cmd_vel(self):
    if self._latest_twist is None:
        return
    if self._vel_controller is None:
        self._latest_twist = None
        return
    twist = self._latest_twist
    # Pass body-frame Twist directly — controller handles kinematics
    self._vel_controller.set_velocity(
        vx=twist.linear.x, vy=twist.linear.y,
        vz=twist.linear.z, wz=twist.angular.z,
    )
    self._latest_twist = None
```

Remove `twist_to_world_velocity` import and usage from robot_handler.py.

**Task 6: bridge_node controller selection**

`_spawn_default_robot()` uses `"omni_velocity"` by default (omni, backward compat).
SpawnEntity service can pass `controller_config` if extended later.

### Test Plan

| Test | What it verifies |
|------|-----------------|
| `test_registry_register_and_create` | register + create round-trip |
| `test_registry_unknown_raises` | KeyError for unknown name |
| `test_registry_builtins_registered` | "omni_velocity" and "differential_velocity" exist |
| `test_diff_vel_forward_only` | vx=1 moves along heading, vy ignored |
| `test_diff_vel_rotation_only` | wz rotates in place, no translation |
| `test_diff_vel_combined` | v + wz produces arc |
| `test_diff_vel_clamps_velocity` | Exceeding max_linear_vel clamped |
| `test_diff_vel_from_config` | from_config parses params correctly |
| `test_diff_vel_zero_returns_false` | No motion → False |
| `test_diff_vel_stop_clears` | on_stop() zeros state |
| `test_vel_ctrl_body_frame_forward` | vx=1 at yaw=0 → world x movement |
| `test_vel_ctrl_body_frame_rotated` | vx=1 at yaw=π/2 → world y movement |
| `test_vel_ctrl_body_frame_lateral` | vy=1 at yaw=0 → world y movement |
| `test_spawn_params_controller_config` | AgentSpawnParams auto-creates controller |

### Existing Tests That Must Still Pass
- All 14 tests in `test_controller.py` (OmniOmniVelocityController + SetController + ABC)
- All 783 tests in full suite

### References
- Current controller: `pybullet_fleet/controller.py` (130 lines)
- Agent set_controller: `pybullet_fleet/agent.py` L1783
- Robot handler: `ros2_bridge/.../robot_handler.py` L71-91
- Plugin arch spec: `docs/design/plugin-architecture/spec.md`
