# Plugin & Extensibility Architecture — Agent Specification

## Requirements

### Functional

- Registry class with 4 category-specific instances: action, robot, controller, world
- 3 registration methods: `register()`, `@decorator`, `entry_points` (auto-discovery)
- Lazy loading of entry_points (only on first `get()`)
- Controller ABC with `compute(agent, dt) -> bool` contract
- Built-in controllers: `TPIController` (extract existing logic), `OmniOmniVelocityController` (for cmd_vel)
- EventBus with typed events, priority ordering, FIFO within same priority
- 11 event types integrated into core_simulation.py and action.py
- `register_callback()` backward compatibility via EventBus wrapper
- Agent hook methods: `_on_pre_move()`, `_on_post_move()`, `_on_post_update()`
- Built-in actions/controllers registered in registry at import time
- YAML config support for name-based resolution of robots, controllers, actions, worlds

### Non-Functional

- EventBus.emit() overhead < 10μs per call for 100 robots
- Registry.get() after loading < 1μs (dict lookup)
- Zero impact on existing tests (backward compatible)
- No new required dependencies

## Constraints

- Controller accesses Agent internal state via `_` prefix attributes (`_current_velocity`, `_is_moving`, etc.) — accepted trade-off
- Registry uses global state — must be clearable for tests (autouse fixture)
- entry_points discovery depends on `importlib.metadata` (stdlib, Python ≥ 3.10)
- TYPE_CHECKING guard needed for Controller ↔ Agent circular import avoidance

## Approach

Gymnasium-inspired registry pattern with Strategy-pattern controllers and a lightweight event bus. Phased implementation: Registry first, then Controller ABC, then EventBus. Each phase is independently testable and deployable.

## Design

### Key Components

| Component | Responsibility | Location |
|-----------|---------------|----------|
| `Registry` | Name → class/factory mapping with entry_point discovery | `pybullet_fleet/registry.py` (new) |
| `Controller` ABC | Agent movement control interface | `pybullet_fleet/controller.py` (new) |
| `TPIController` | Existing TPI trajectory logic as Controller | `pybullet_fleet/controller.py` |
| `OmniOmniVelocityController` | Explicit velocity command control | `pybullet_fleet/controller.py` |
| `EventBus` | Simulation lifecycle event pub/sub | `pybullet_fleet/events.py` (new) |
| Modified `Agent` | Controller integration + hook methods | `pybullet_fleet/agent.py` (modify) |
| Modified `MultiRobotSimulationCore` | EventBus integration | `pybullet_fleet/core_simulation.py` (modify) |
| Modified `__init__.py` | Public API expansion + built-in registration | `pybullet_fleet/__init__.py` (modify) |

### Architecture Diagram

```
User Application / External Package
          │
          ▼
    ┌──────────┐     entry_points
    │ Registry │ ◄──── pip install pybullet-fleet-xxx
    │ (4 cats) │
    └────┬─────┘
         │ get("name")
         ▼
┌──────────────────────────────────────────────────────┐
│              MultiRobotSimulationCore                │
│                                                      │
│  events: EventBus ──── emit("pre_step") ────────┐   │
│                        emit("post_step")         │   │
│                        emit("collision")         │   │
│                        emit("agent_added")       │   │
│                                                  │   │
│  ┌─────────┐  ┌────────────┐  ┌──────────────┐  │   │
│  │  Agent  │──│ Controller │  │ Action Queue │  │   │
│  │         │  │  (has-a)   │  │              │  │   │
│  │ hooks:  │  │ .compute() │  │ [Action...]  │  │   │
│  │ _on_pre │  │ .on_goal() │  │ .execute()   │  │   │
│  │ _on_post│  │ .on_stop() │  │ .reset()     │  │   │
│  └─────────┘  └────────────┘  └──────────────┘  │   │
│                                                  │   │
│  register_callback(fn, freq)  ◄── EventBus sugar │   │
└──────────────────────────────────────────────────────┘
```

### Data Flow

#### Registry Resolution

```
YAML config                       Python code
    │                                 │
    ▼                                 ▼
type: "agv_300"               robot_registry.register("agv_300", factory_fn)
    │                                 │
    ▼                                 ▼
robot_registry.get("agv_300") ──► dict lookup ──► entry_points lazy load
    │
    ▼
factory_fn(index=0) → AgentSpawnParams
    │
    ▼
Agent.from_params(params, sim_core=sim, controller=...)
```

#### Controller Flow in Agent.update()

```
Agent.update(dt)
  │
  ├── _update_actions(dt)           ← Action queue (unchanged)
  ├── _on_pre_move(dt)              ← Hook (new, override in subclass)
  ├── _update_kinematic_joints(dt)  ← Joint interpolation (unchanged)
  │
  ├── if self._controller is not None:
  │     └── self._controller.compute(self, dt)    ← Controller mode (new)
  │
  ├── elif self._is_moving and self._goal_pose:
  │     └── existing TPI logic                    ← Legacy mode (unchanged)
  │
  ├── _on_post_move(dt)             ← Hook (new)
  ├── update_attached_objects()     ← Attachment tracking (unchanged)
  └── _on_post_update(dt)          ← Hook (new)
```

#### EventBus Flow in step_once()

```
step_once()
  │
  ├── events.emit("pre_step", dt=dt, sim_time=sim_time)
  │
  ├── for agent in agents:
  │     agent.update(dt)
  │
  ├── collision detection
  │     for a, b in collisions:
  │       events.emit("collision", obj_a=a, obj_b=b)
  │
  ├── callbacks (via EventBus post_step)
  │
  └── events.emit("post_step", dt=dt, sim_time=sim_time)
```

### Code Patterns

#### Registry (new file)

```python
# pybullet_fleet/registry.py
from importlib.metadata import entry_points
from typing import Any, Callable, Dict, Optional, Type

from pybullet_fleet.logging_utils import get_lazy_logger

logger = get_lazy_logger(__name__)


class Registry:
    """Name → class/factory mapping with entry_point auto-discovery.

    Lazy-loads entry_points on first get() to avoid import-time overhead.
    """

    def __init__(self, group: str):
        self._group = group
        self._entries: Dict[str, Any] = {}
        self._loaded_entry_points = False

    def register(self, name: str, entry: Any) -> None:
        """Explicitly register a name → class/factory mapping."""
        if name in self._entries:
            logger.warning(lambda: f"Registry '{self._group}': overwriting '{name}'")
        self._entries[name] = entry

    def get(self, name: str) -> Any:
        """Resolve name to class/factory. Loads entry_points on first miss."""
        if name in self._entries:
            return self._entries[name]
        if not self._loaded_entry_points:
            self._load_entry_points()
            if name in self._entries:
                return self._entries[name]
        available = sorted(self._entries.keys())
        raise KeyError(
            f"'{name}' not found in {self._group}. Available: {available}"
        )

    def _load_entry_points(self) -> None:
        eps = entry_points(group=self._group)
        for ep in eps:
            if ep.name not in self._entries:
                try:
                    self._entries[ep.name] = ep.load()
                except Exception:
                    logger.exception(lambda: f"Failed to load entry_point '{ep.name}' from {self._group}")
        self._loaded_entry_points = True

    def list_available(self) -> list:
        """Return sorted list of registered names (loads entry_points if needed)."""
        if not self._loaded_entry_points:
            self._load_entry_points()
        return sorted(self._entries.keys())

    def __contains__(self, name: str) -> bool:
        try:
            self.get(name)
            return True
        except KeyError:
            return False

    def clear(self) -> None:
        """Clear all entries (for testing)."""
        self._entries.clear()
        self._loaded_entry_points = False


# Global registry instances
action_registry = Registry("pybullet_fleet.actions")
robot_registry = Registry("pybullet_fleet.robots")
controller_registry = Registry("pybullet_fleet.controllers")
world_registry = Registry("pybullet_fleet.worlds")


def register_action(name: str):
    """Decorator to register an Action subclass."""
    def decorator(cls):
        action_registry.register(name, cls)
        return cls
    return decorator


def register_robot(name: str):
    """Decorator to register a robot factory function."""
    def decorator(fn):
        robot_registry.register(name, fn)
        return fn
    return decorator


def register_controller(name: str):
    """Decorator to register a Controller subclass."""
    def decorator(cls):
        controller_registry.register(name, cls)
        return cls
    return decorator


def register_world(name: str):
    """Decorator to register a world setup function."""
    def decorator(fn):
        world_registry.register(name, fn)
        return fn
    return decorator
```

#### Controller ABC (new file)

```python
# pybullet_fleet/controller.py
from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Optional

import numpy as np

from pybullet_fleet.logging_utils import get_lazy_logger

if TYPE_CHECKING:
    from pybullet_fleet.agent import Agent
    from pybullet_fleet.geometry import Pose

logger = get_lazy_logger(__name__)


class Controller(ABC):
    """Agent movement control interface (Strategy pattern).

    Replaces the hardcoded TPI/velocity logic in Agent.update().
    Agent has-a Controller. Controller未設定 = legacy mode (backward compat).

    Usage:
        agent = Agent.from_params(params, controller=PIDController(kp=1.0))
        agent.set_controller(OmniOmniVelocityController())  # dynamic switch
    """

    @abstractmethod
    def compute(self, agent: "Agent", dt: float) -> bool:
        """Compute one step of control.

        Args:
            agent: The agent to control (read state, set pose/velocity)
            dt: Time step in seconds

        Returns:
            True if the agent moved, False otherwise

        Implementation guide:
            - Read: agent.get_pose(), agent.goal_pose, agent.velocity
            - Write: agent.set_pose(), agent.set_pose_raw()
            - Update: agent._current_velocity[:] = ..., agent._current_angular_velocity = ...
        """
        ...

    def on_goal_set(self, agent: "Agent", goal: "Pose") -> None:
        """Called when set_goal_pose() is invoked. Init trajectories here."""
        pass

    def on_path_set(self, agent: "Agent", path: list) -> None:
        """Called when set_path() is invoked."""
        pass

    def on_stop(self, agent: "Agent") -> None:
        """Called when stop() is invoked."""
        pass

    def reset(self) -> None:
        """Reset internal state."""
        pass


class TPIController(Controller):
    """Existing TPI trajectory-following logic as a Controller.

    Extracts _update_omnidirectional() and _update_differential() from Agent
    into the Controller interface. This is the default controller.
    """

    def compute(self, agent, dt):
        if not agent._is_moving or agent._goal_pose is None:
            return False

        if agent._is_final_orientation_aligning:
            agent._update_differential(dt)
        elif agent._motion_mode == MotionMode.OMNIDIRECTIONAL:
            agent._update_omnidirectional(dt)
        elif agent._motion_mode == MotionMode.DIFFERENTIAL:
            agent._update_differential(dt)
        return True

    def on_goal_set(self, agent, goal):
        # Delegate to Agent's existing TPI initialization
        pass  # Agent._init_tpi_* is called inside set_goal_pose()

    def on_stop(self, agent):
        pass  # Agent.stop() clears TPI state internally


class OmniOmniVelocityController(Controller):
    """Explicit velocity command control (for cmd_vel / ROS bridge).

    Usage:
        ctrl = OmniOmniVelocityController()
        agent.set_controller(ctrl)
        ctrl.set_velocity(vx=1.0, vy=0.0, wz=0.2)
    """

    def __init__(self):
        self._velocity_command = np.array([0.0, 0.0, 0.0])
        self._angular_velocity_command = 0.0

    def set_velocity(self, vx: float = 0.0, vy: float = 0.0,
                     vz: float = 0.0, wz: float = 0.0) -> None:
        """Set velocity command (world frame)."""
        self._velocity_command[:] = [vx, vy, vz]
        self._angular_velocity_command = wz

    def compute(self, agent, dt):
        if (np.allclose(self._velocity_command, 0.0)
                and abs(self._angular_velocity_command) < 1e-9):
            agent._current_velocity[:] = 0.0
            agent._current_angular_velocity = 0.0
            return False

        pose = agent.get_pose()
        dx = self._velocity_command[0] * dt
        dy = self._velocity_command[1] * dt
        dz = self._velocity_command[2] * dt
        dyaw = self._angular_velocity_command * dt

        from pybullet_fleet.geometry import Pose
        new_pose = Pose.from_yaw(
            pose.x + dx, pose.y + dy, pose.z + dz, pose.yaw + dyaw
        )
        agent.set_pose(new_pose)

        agent._current_velocity[:] = self._velocity_command
        agent._current_angular_velocity = self._angular_velocity_command
        return True

    def on_stop(self, agent):
        self._velocity_command[:] = 0.0
        self._angular_velocity_command = 0.0
```

#### EventBus (new file)

```python
# pybullet_fleet/events.py
import logging
from collections import defaultdict
from typing import Any, Callable, Dict, List, Optional

from pybullet_fleet.logging_utils import get_lazy_logger

logger = get_lazy_logger(__name__)


class EventBus:
    """Simulation lifecycle event pub/sub.

    Evolution of register_callback(). Register handlers for specific events.
    Handlers execute in priority order (lower = first). Same priority = FIFO.

    Usage:
        bus = EventBus()
        bus.on("collision", my_handler, priority=0)
        bus.emit("collision", obj_a=robot1, obj_b=robot2)
    """

    def __init__(self):
        self._handlers: Dict[str, List[tuple]] = defaultdict(list)
        self._sorted: Dict[str, bool] = {}

    def on(self, event: str, handler: Callable, priority: int = 0) -> None:
        """Register handler. Lower priority = executed first."""
        self._handlers[event].append((priority, handler))
        self._sorted[event] = False

    def off(self, event: str, handler: Callable) -> None:
        """Unregister handler."""
        self._handlers[event] = [
            (p, h) for p, h in self._handlers[event] if h is not handler
        ]

    def emit(self, event: str, **kwargs) -> None:
        """Fire event. Handlers called in priority order."""
        handlers = self._handlers.get(event)
        if not handlers:
            return
        if not self._sorted.get(event, True):
            handlers.sort(key=lambda x: x[0])
            self._sorted[event] = True
        for _, handler in handlers:
            try:
                handler(**kwargs)
            except Exception:
                logger.exception(
                    lambda: f"EventBus handler error for event '{event}'"
                )

    def clear(self, event: Optional[str] = None) -> None:
        """Clear handlers. If event given, clear only that event."""
        if event:
            self._handlers.pop(event, None)
        else:
            self._handlers.clear()
            self._sorted.clear()

    def has_handlers(self, event: str) -> bool:
        """Check if any handlers are registered for an event."""
        return bool(self._handlers.get(event))


# Defined event names (documentation, not enforced)
EVENTS = {
    "pre_step":          "dt: float, sim_time: float",
    "post_step":         "dt: float, sim_time: float",
    "collision":         "obj_a: SimObject, obj_b: SimObject",
    "agent_added":       "agent: Agent",
    "agent_removed":     "agent: Agent",
    "object_added":      "obj: SimObject",
    "object_removed":    "obj: SimObject",
    "action_started":    "agent: Agent, action: Action",
    "action_completed":  "agent: Agent, action: Action, status: ActionStatus",
    "paused":            "(no args)",
    "resumed":           "(no args)",
}
```

#### Agent Modifications (modify existing)

```python
# pybullet_fleet/agent.py — modifications

# In __init__ (after self._current_velocity initialization):
    self._controller: Optional["Controller"] = None

# New method:
    def set_controller(self, controller: Optional["Controller"]) -> None:
        """Set or replace the movement controller (Strategy pattern).

        Args:
            controller: Controller instance, or None to revert to legacy mode
        """
        if self._controller is not None:
            self._controller.on_stop(self)
        self._controller = controller

# In update(dt) — replace movement dispatch:
    moved = False
    if not self.use_fixed_base:
        self._on_pre_move(dt)

        if self._controller is not None:
            moved = self._controller.compute(self, dt)
        elif self._is_moving and self._goal_pose is not None:
            # Legacy mode (backward compatible, unchanged)
            if self._is_final_orientation_aligning:
                self._update_differential(dt)
            elif self._motion_mode == MotionMode.OMNIDIRECTIONAL:
                self._update_omnidirectional(dt)
            elif self._motion_mode == MotionMode.DIFFERENTIAL:
                self._update_differential(dt)
            moved = True

        self._on_post_move(dt)

    # ... (existing attached objects update)
    self._on_post_update(dt)

# Hook methods:
    def _on_pre_move(self, dt: float) -> None:
        """Override point: before movement calculation."""
        pass

    def _on_post_move(self, dt: float) -> None:
        """Override point: after movement, before attachment update."""
        pass

    def _on_post_update(self, dt: float) -> None:
        """Override point: after all updates complete."""
        pass

# In set_goal_pose() — notify controller:
    def set_goal_pose(self, goal, ...):
        # ... existing code ...
        if self._controller is not None:
            self._controller.on_goal_set(self, goal)

# In stop() — notify controller:
    def stop(self):
        if self._controller is not None:
            self._controller.on_stop(self)
        # ... existing code ...
```

#### core_simulation.py Modifications

```python
# pybullet_fleet/core_simulation.py — modifications

# In __init__:
    from pybullet_fleet.events import EventBus
    self.events = EventBus()

# In step_once():
    def step_once(self):
        if self._paused:
            return
        dt = self._params.timestep

        self.events.emit("pre_step", dt=dt, sim_time=self.sim_time)

        # ... existing agent update loop ...

        # After collision detection:
        if self.events.has_handlers("collision"):
            for a, b in self._collisions_this_step:
                self.events.emit("collision", obj_a=a, obj_b=b)

        # ... existing callback loop (kept for backward compat) ...

        self.events.emit("post_step", dt=dt, sim_time=self.sim_time)
        self._step_count += 1

# In add_object():
    def add_object(self, obj):
        # ... existing logic ...
        self.events.emit("object_added", obj=obj)
        if isinstance(obj, Agent):
            self.events.emit("agent_added", agent=obj)

# In remove_object():
    def remove_object(self, obj):
        if isinstance(obj, Agent):
            self.events.emit("agent_removed", agent=obj)
        self.events.emit("object_removed", obj=obj)
        # ... existing logic ...

# In pause()/resume():
    def pause(self):
        self._paused = True
        self.events.emit("paused")

    def resume(self):
        self._paused = False
        self.events.emit("resumed")
```

#### __init__.py Public API Expansion

```python
# pybullet_fleet/__init__.py — additions

# New exports
from pybullet_fleet.action import Action, MoveAction, PickAction, DropAction, WaitAction, JointAction, PoseAction
from pybullet_fleet.controller import Controller, TPIController, OmniOmniVelocityController
from pybullet_fleet.events import EventBus
from pybullet_fleet.registry import (
    Registry,
    action_registry, robot_registry, controller_registry, world_registry,
    register_action, register_robot, register_controller, register_world,
)
from pybullet_fleet.sim_object import ShapeParams, SimObjectSpawnParams
from pybullet_fleet.types import CollisionMode
from pybullet_fleet.config_utils import load_config

# Register built-in actions
action_registry.register("move", MoveAction)
action_registry.register("pick", PickAction)
action_registry.register("drop", DropAction)
action_registry.register("wait", WaitAction)
action_registry.register("joint", JointAction)
action_registry.register("pose", PoseAction)

# Register built-in controllers
controller_registry.register("tpi", TPIController)
controller_registry.register("omni_velocity", OmniOmniVelocityController)
```

### External Package Template

```
pybullet-fleet-{name}/
├── pyproject.toml
├── README.md
├── src/pybullet_fleet_{name}/
│   ├── __init__.py
│   ├── actions/          # Custom Action subclasses
│   ├── robots/           # AgentSpawnParams factories + URDF files
│   │   └── urdf/
│   ├── controllers/      # Custom Controller subclasses
│   └── worlds/           # Scene setup functions
├── tests/
└── examples/
```

```toml
# pyproject.toml template
[project]
name = "pybullet-fleet-{name}"
version = "0.1.0"
dependencies = ["pybullet-fleet>=0.3.0"]

[project.entry-points."pybullet_fleet.actions"]
custom_action = "pybullet_fleet_{name}.actions:CustomAction"

[project.entry-points."pybullet_fleet.robots"]
custom_robot = "pybullet_fleet_{name}.robots:create_custom_robot"

[project.entry-points."pybullet_fleet.controllers"]
custom_ctrl = "pybullet_fleet_{name}.controllers:CustomController"

[project.entry-points."pybullet_fleet.worlds"]
custom_world = "pybullet_fleet_{name}.worlds:setup_custom_world"
```

## File References

Files the plan agent MUST read before planning:

- `pybullet_fleet/__init__.py` — current public API surface (needs expansion)
- `pybullet_fleet/agent.py:320-340` — `__init__` instance variables (add `_controller`)
- `pybullet_fleet/agent.py:460-500` — velocity/angular_velocity properties
- `pybullet_fleet/agent.py:1711-1770` — `update()` method (add controller dispatch)
- `pybullet_fleet/agent.py:995-1010` — `set_goal_pose()` (add controller notification)
- `pybullet_fleet/agent.py:1775-1790` — `stop()` (add controller notification)
- `pybullet_fleet/agent.py:1131-1230` — `_update_omnidirectional()` (extract to TPIController)
- `pybullet_fleet/agent.py:1418-1520` — `_update_differential()` (extract to TPIController)
- `pybullet_fleet/core_simulation.py:52-90` — `SimulationParams` dataclass
- `pybullet_fleet/core_simulation.py:1870-1950` — `step_once()` (add EventBus emit)
- `pybullet_fleet/core_simulation.py:992-1060` — `add_object()` / `remove_object()`
- `pybullet_fleet/core_simulation.py:327-380` — `register_callback()` (keep backward compat)
- `pybullet_fleet/action.py:68-120` — `Action` ABC (ensure exported in __init__)
- `pybullet_fleet/types.py` — `MotionMode`, `ActionStatus` enums
- `pybullet_fleet/sim_object.py:1-50` — `SimObject`, `ShapeParams`, `SimObjectSpawnParams`
- `tests/conftest.py` — autouse fixtures (add registry clear)
- `pyproject.toml` — add entry_points section for built-in registration

## Success Criteria

- [ ] `from pybullet_fleet.registry import action_registry; action_registry.get("move")` returns `MoveAction`
- [ ] Custom Action registered via `register_action("my_action")` decorator is discoverable
- [ ] External package entry_points are discovered on first `registry.get()` call
- [ ] `Agent.from_params(params, controller=OmniOmniVelocityController())` creates agent with velocity control
- [ ] `agent.set_controller(TPIController())` dynamically switches controller
- [ ] `agent.set_controller(None)` reverts to legacy mode
- [ ] `sim.events.on("collision", handler)` receives collision events
- [ ] `sim.events.on("agent_added", handler)` receives add events when `Agent.from_params()` is called
- [ ] `register_callback(fn, frequency=1.0)` still works (backward compat)
- [ ] All existing tests pass without modification
- [ ] EventBus emit overhead < 10μs per call on benchmark
- [ ] Template external package works with `pip install -e .` and entry_points
