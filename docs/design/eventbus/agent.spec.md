# EventBus — Agent Specification

## Requirements

### Functional

- EventBus class in `pybullet_fleet/events.py` with `on()`, `off()`, `emit()`, `clear()`, `has_handlers()`
- Global EventBus on `MultiRobotSimulationCore.events`
- Lazy per-entity EventBus on `SimObject.events` (property, creates on first access)
- 12 event types: pre/post_step, agent_spawned/removed, object_spawned/removed, collision_started/ended, action_started/completed, paused, resumed
- Collision Enter/Exit model with `_prev_collision_pairs` tracking
- `register_callback()` backward compatibility — no changes to existing callback system
- EventBus exported from `pybullet_fleet/__init__.py`
- `tests/test_events.py` with full coverage

### Non-Functional

- Global `emit()` < 10μs per call for 100 robots
- Per-entity `_has_entity_events()` < 50ns per entity (attribute None check)
- Per-entity bus not created until first `.events` access — zero memory for unused entities
- 3 profiling fields (`events_pre_step`, `events_post_step`, `events_collision`) added to `_profiling_stats`
- Zero impact on existing tests
- No new dependencies

## Constraints

- EventBus is per-instance — no global state
- Per-entity bus is lazy (`_events: Optional[EventBus] = None`)
- Collision Enter/Exit requires `_prev_collision_pairs: Set[FrozenSet[int]]` in core_simulation
- Action events access global EventBus via `agent._sim_core.events`
- `TYPE_CHECKING` guard needed for `events.py` type hints

## Approach

Two-layer event system: Global bus for sim-wide events, lazy per-entity bus for entity-specific hooks. Collision uses Enter/Exit model (UE5/Unity pattern). Direct implementation from plugin-architecture agent.spec.md patterns with Enter/Exit extension.

## Design

### Component Table

| Component | File | Change |
|-----------|------|--------|
| `EventBus` | `pybullet_fleet/events.py` | **New file** |
| `SimObject` | `pybullet_fleet/sim_object.py` | Add lazy `events` property, `_has_entity_events()` |
| `MultiRobotSimulationCore` | `pybullet_fleet/core_simulation.py` | Add `self.events`, `_prev_collision_pairs`, emit in step/add/remove/pause/resume |
| Action system | `pybullet_fleet/action.py` | Add action_started/completed emit |
| Public API | `pybullet_fleet/__init__.py` | Export `EventBus` |
| Tests | `tests/test_events.py` | **New file** |

### Architecture

```
step_once()
  │
  ├─ sim.events.emit("pre_step", dt=dt, sim_time=sim_time)
  │
  ├─ for agent in agents:
  │    agent.update(dt)
  │      └─ action transitions:
  │           ├─ sim.events.emit("action_started", agent=agent, action=action)
  │           │  └─ if agent._has_entity_events():
  │           │       agent.events.emit("action_started", action=action)
  │           └─ sim.events.emit("action_completed", ...)
  │              └─ if agent._has_entity_events():
  │                   agent.events.emit("action_completed", ...)
  │
  ├─ callbacks (register_callback)         ← unchanged
  │
  ├─ physics step                          ← unchanged
  │
  ├─ collision detection → current_pairs   ← unchanged
  │    └─ Enter/Exit diff:
  │         started = current_pairs - _prev_collision_pairs
  │         ended   = _prev_collision_pairs - current_pairs
  │         _prev_collision_pairs = current_pairs
  │         │
  │         ├─ for a,b in started:
  │         │    sim.events.emit("collision_started", obj_a=a, obj_b=b)
  │         │    if a._has_entity_events(): a.events.emit("collision_started", other=b)
  │         │    if b._has_entity_events(): b.events.emit("collision_started", other=a)
  │         │
  │         └─ for a,b in ended:
  │              sim.events.emit("collision_ended", obj_a=a, obj_b=b)
  │              if a._has_entity_events(): a.events.emit("collision_ended", other=b)
  │              if b._has_entity_events(): b.events.emit("collision_ended", other=a)
  │
  ├─ monitor update                        ← unchanged
  │
  └─ sim.events.emit("post_step", dt=dt, sim_time=sim_time)

add_object(obj)
  └─ sim.events.emit("object_spawned", obj=obj)
     ├─ if obj._has_entity_events(): obj.events.emit("object_spawned")
     └─ if Agent:
          sim.events.emit("agent_spawned", agent=obj)
          if obj._has_entity_events(): obj.events.emit("agent_spawned")

remove_object(obj)
  └─ if Agent:
       sim.events.emit("agent_removed", agent=obj)
       if obj._has_entity_events(): obj.events.emit("agent_removed")
     sim.events.emit("object_removed", obj=obj)
     if obj._has_entity_events(): obj.events.emit("object_removed")

pause()  → sim.events.emit("paused")
resume() → sim.events.emit("resumed")
```

### Code Patterns

#### events.py (new file)

```python
# pybullet_fleet/events.py
from collections import defaultdict
from typing import Any, Callable, Dict, List, Optional

from pybullet_fleet.logging_utils import get_lazy_logger

logger = get_lazy_logger(__name__)


class EventBus:
    """Lightweight event pub/sub for simulation lifecycle events.

    Two usage levels:
    - **Global bus** (``sim.events``): sim-wide events (pre_step, agent_spawned, etc.)
    - **Per-entity bus** (``agent.events``): entity-specific hooks for subclasses

    Handlers execute in priority order (lower = first). Same priority = FIFO.
    Exceptions in handlers are logged but do not block other handlers.

    Usage::

        # Global
        sim.events.on("collision_started", my_handler, priority=0)

        # Per-entity (in Agent subclass)
        self.events.on("collision_started", self._on_bump)
    """

    def __init__(self) -> None:
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

    def emit(self, event: str, **kwargs: Any) -> None:
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
            self._sorted.pop(event, None)
        else:
            self._handlers.clear()
            self._sorted.clear()

    def has_handlers(self, event: str) -> bool:
        """Check if any handlers are registered for an event."""
        return bool(self._handlers.get(event))


# Defined event names (documentation, not enforced at runtime)
EVENTS = {
    # Simulation step (global only)
    "pre_step": "dt: float, sim_time: float",
    "post_step": "dt: float, sim_time: float",
    # Entity lifecycle (global + per-entity)
    "agent_spawned": "agent: Agent (global) / no args (per-entity)",
    "agent_removed": "agent: Agent (global) / no args (per-entity)",
    "object_spawned": "obj: SimObject (global) / no args (per-entity)",
    "object_removed": "obj: SimObject (global) / no args (per-entity)",
    # Collision Enter/Exit (global + per-entity)
    "collision_started": "obj_a, obj_b (global) / other: SimObject (per-entity)",
    "collision_ended": "obj_a, obj_b (global) / other: SimObject (per-entity)",
    # Action (global + per-entity)
    "action_started": "agent, action (global) / action (per-entity)",
    "action_completed": "agent, action, status (global) / action, status (per-entity)",
    # Simulation control (global only)
    "paused": "(no args)",
    "resumed": "(no args)",
}
```

#### sim_object.py — lazy per-entity EventBus

```python
# In SimObject.__init__():
    self._events: Optional["EventBus"] = None

# New property:
    @property
    def events(self) -> "EventBus":
        """Per-entity EventBus. Created on first access (lazy).

        Use in subclasses to hook into entity-specific events::

            class WarehouseRobot(Agent):
                def __init__(self, ...):
                    super().__init__(...)
                    self.events.on("collision_started", self._on_bump)
                    self.events.on("action_completed", self._on_task_done)
        """
        if self._events is None:
            from pybullet_fleet.events import EventBus
            self._events = EventBus()
        return self._events

# New method:
    def _has_entity_events(self) -> bool:
        """Fast check: True if per-entity EventBus has been created."""
        return self._events is not None
```

#### core_simulation.py — Global EventBus + collision Enter/Exit + profiling

```python
# In __init__():
from pybullet_fleet.events import EventBus
self.events = EventBus()
self._prev_collision_pairs: set = set()  # Set[FrozenSet[int]] for Enter/Exit

# In step_once() — after pause check, before agent updates:
if measure_timing:
    t_ev_pre = time.perf_counter()
self.events.emit("pre_step", dt=self._params.timestep, sim_time=self.sim_time)
if measure_timing:
    self._profiling_stats["events_pre_step"][-1] = (time.perf_counter() - t_ev_pre) * 1000

# In step_once() — after collision detection:
# Collision Enter/Exit diff (with profiling)
if measure_timing:
    t_ev_col = time.perf_counter()
if self.events.has_handlers("collision_started") or self.events.has_handlers("collision_ended") or self._any_entity_collision_handlers():
    current_pairs = {frozenset(pair) for pair in self._collision_pairs}
    started = current_pairs - self._prev_collision_pairs
    ended = self._prev_collision_pairs - current_pairs
    self._prev_collision_pairs = current_pairs

    for pair in started:
        ids = list(pair)
        a = self._sim_objects_dict.get(ids[0])
        b = self._sim_objects_dict.get(ids[1] if len(ids) > 1 else ids[0])
        if a is not None and b is not None:
            self.events.emit("collision_started", obj_a=a, obj_b=b)
            if a._has_entity_events():
                a.events.emit("collision_started", other=b)
            if b._has_entity_events():
                b.events.emit("collision_started", other=a)

    for pair in ended:
        ids = list(pair)
        a = self._sim_objects_dict.get(ids[0])
        b = self._sim_objects_dict.get(ids[1] if len(ids) > 1 else ids[0])
        if a is not None and b is not None:
            self.events.emit("collision_ended", obj_a=a, obj_b=b)
            if a._has_entity_events():
                a.events.emit("collision_ended", other=b)
            if b._has_entity_events():
                b.events.emit("collision_ended", other=a)
if measure_timing:
    self._profiling_stats["events_collision"][-1] = (time.perf_counter() - t_ev_col) * 1000

# In step_once() — at the very end:
if measure_timing:
    t_ev_post = time.perf_counter()
self.events.emit("post_step", dt=self._params.timestep, sim_time=self.sim_time)
if measure_timing:
    self._profiling_stats["events_post_step"][-1] = (time.perf_counter() - t_ev_post) * 1000

# In add_object():
self.events.emit("object_spawned", obj=obj)
if obj._has_entity_events():
    obj.events.emit("object_spawned")
if isinstance(obj, Agent):
    self.events.emit("agent_spawned", agent=obj)
    if obj._has_entity_events():
        obj.events.emit("agent_spawned")

# In remove_object():
if isinstance(obj, Agent):
    self.events.emit("agent_removed", agent=obj)
    if obj._has_entity_events():
        obj.events.emit("agent_removed")
self.events.emit("object_removed", obj=obj)
if obj._has_entity_events():
    obj.events.emit("object_removed")

# In pause():
self.events.emit("paused")

# In resume():
self.events.emit("resumed")

# Helper:
def _any_entity_collision_handlers(self) -> bool:
    """Check if any entity has per-entity collision handlers."""
    return any(obj._has_entity_events() for obj in self._sim_objects_dict.values())
```

#### action.py — action events

```python
# When action transitions to IN_PROGRESS:
if hasattr(agent, '_sim_core') and agent._sim_core is not None:
    agent._sim_core.events.emit("action_started", agent=agent, action=action)
if agent._has_entity_events():
    agent.events.emit("action_started", action=action)

# When action completes/fails/cancels:
if hasattr(agent, '_sim_core') and agent._sim_core is not None:
    agent._sim_core.events.emit(
        "action_completed", agent=agent, action=action, status=action.status
    )
if agent._has_entity_events():
    agent.events.emit("action_completed", action=action, status=action.status)
```

#### __init__.py

```python
from pybullet_fleet.events import EventBus
```

### Test Patterns

```python
# tests/test_events.py

# === EventBus unit tests ===

def test_eventbus_on_emit():
    bus = EventBus()
    results = []
    bus.on("test", lambda **kw: results.append(kw))
    bus.emit("test", x=1)
    assert results == [{"x": 1}]

def test_eventbus_priority_order():
    bus = EventBus()
    order = []
    bus.on("test", lambda: order.append("B"), priority=10)
    bus.on("test", lambda: order.append("A"), priority=0)
    bus.emit("test")
    assert order == ["A", "B"]

def test_eventbus_off():
    bus = EventBus()
    results = []
    handler = lambda **kw: results.append(1)
    bus.on("test", handler)
    bus.off("test", handler)
    bus.emit("test")
    assert results == []

def test_eventbus_handler_exception_isolated():
    bus = EventBus()
    results = []
    bus.on("test", lambda: 1/0, priority=0)
    bus.on("test", lambda: results.append("ok"), priority=1)
    bus.emit("test")
    assert results == ["ok"]

def test_eventbus_clear():
    bus = EventBus()
    bus.on("a", lambda: None)
    bus.on("b", lambda: None)
    bus.clear()
    assert not bus.has_handlers("a")

def test_eventbus_clear_specific():
    bus = EventBus()
    bus.on("a", lambda: None)
    bus.on("b", lambda: None)
    bus.clear("a")
    assert not bus.has_handlers("a")
    assert bus.has_handlers("b")

# === Integration tests (sim_core) ===

def test_pre_post_step_events(sim_core_headless):
    events = []
    sim_core_headless.events.on("pre_step", lambda **kw: events.append(("pre", kw["sim_time"])))
    sim_core_headless.events.on("post_step", lambda **kw: events.append(("post", kw["sim_time"])))
    sim_core_headless.step_once()
    assert len(events) == 2
    assert events[0][0] == "pre"
    assert events[1][0] == "post"

def test_agent_spawned_event(sim_core_headless):
    added = []
    sim_core_headless.events.on("agent_spawned", lambda agent: added.append(agent))
    agent = Agent.from_params(spawn_params, sim_core=sim_core_headless)
    assert len(added) == 1
    assert added[0] is agent

def test_agent_removed_event(sim_core_headless):
    removed = []
    sim_core_headless.events.on("agent_removed", lambda agent: removed.append(agent))
    agent = Agent.from_params(spawn_params, sim_core=sim_core_headless)
    sim_core_headless.remove_object(agent)
    assert len(removed) == 1

def test_collision_started_ended(sim_core_headless):
    """Collision Enter/Exit: started on first overlap, ended on separation."""
    started = []
    ended = []
    sim_core_headless.events.on("collision_started", lambda obj_a, obj_b: started.append((obj_a, obj_b)))
    sim_core_headless.events.on("collision_ended", lambda obj_a, obj_b: ended.append((obj_a, obj_b)))
    # Spawn two overlapping agents → step → collision_started
    # Move apart → step → collision_ended
    ...

def test_collision_started_not_repeated():
    """持続衝突中は collision_started が再発火しない."""
    ...

def test_per_entity_collision_event(sim_core_headless):
    """Per-entity: agent.events.on('collision_started') fires only for that agent."""
    agent_a = Agent.from_params(...)
    agent_b = Agent.from_params(...)
    a_collisions = []
    agent_a.events.on("collision_started", lambda other: a_collisions.append(other))
    # Cause collision between a and b
    # Assert a_collisions contains agent_b
    ...

def test_per_entity_lazy_creation():
    """Bus is not created until .events is accessed."""
    obj = SimObject.from_mesh(...)
    assert obj._events is None  # Not created yet
    _ = obj.events  # Access triggers creation
    assert obj._events is not None

def test_per_entity_has_entity_events_false_when_unused():
    """_has_entity_events() returns False when bus never accessed."""
    obj = SimObject.from_mesh(...)
    assert not obj._has_entity_events()

def test_register_callback_still_works(sim_core_headless):
    results = []
    sim_core_headless.register_callback(lambda sim, dt: results.append(dt))
    sim_core_headless.step_once()
    assert len(results) == 1
```

## File References

Files the plan agent MUST read before planning:

- `pybullet_fleet/core_simulation.py:2441-2620` — `step_once()` (add pre/post emit + collision Enter/Exit)
- `pybullet_fleet/core_simulation.py:355-400` — `register_callback()` (keep unchanged)
- `pybullet_fleet/core_simulation.py:992-1060` — `add_object()` / `remove_object()` (add emit)
- `pybullet_fleet/core_simulation.py:150-200` — `__init__()` (add `self.events`, `_prev_collision_pairs`)
- `pybullet_fleet/core_simulation.py:1180-1220` — `pause()` / `resume()` (add emit)
- `pybullet_fleet/core_simulation.py:2200-2250` — `reset_simulation()` (DO NOT clear events)
- `pybullet_fleet/sim_object.py:1-100` — SimObject class (add lazy `events` property)
- `pybullet_fleet/action.py:68-120` — Action ABC and status transitions
- `pybullet_fleet/agent.py:1700-1770` — `Agent.update()` / `_update_actions()`
- `pybullet_fleet/__init__.py` — current exports (add EventBus)
- `tests/conftest.py` — fixtures
- `pybullet_fleet/core_simulation.py:1700-1800` — `check_collisions()` (collision pairs data structure)

## Success Criteria

- [ ] Global: `sim.events.on("pre_step", handler)` fires every `step_once()`
- [ ] Global: `sim.events.on("post_step", handler)` fires every `step_once()`
- [ ] Global: `sim.events.on("collision_started", handler)` fires on first frame of collision
- [ ] Global: `sim.events.on("collision_ended", handler)` fires when collision pair separates
- [ ] Global: `sim.events.on("agent_spawned", handler)` fires on Agent.from_params()
- [ ] Per-entity: `agent.events.on("collision_started", handler)` fires only for that agent's collisions
- [ ] Per-entity: `agent.events.on("action_completed", handler)` fires only for that agent's actions
- [ ] Per-entity: bus not created until first `.events` access
- [ ] Per-entity: `_has_entity_events()` returns False for unused entities
- [ ] Enter/Exit: sustained collision does NOT re-fire `collision_started`
- [ ] Enter/Exit: separation fires `collision_ended`
- [ ] `register_callback()` unaffected
- [ ] Priority ordering works correctly
- [ ] Handler exceptions are isolated
- [ ] All existing tests pass without modification
- [ ] `make verify` passes
