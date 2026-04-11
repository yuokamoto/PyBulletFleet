# EventBus Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use executing-plans to implement this plan task-by-task.

**Goal:** Add a 2-layer EventBus (global + lazy per-entity) to PyBulletFleet with 12 event types, Enter/Exit collision model, and profiling integration.

**Architecture:** New `events.py` module provides `EventBus` class. Global instance on `MultiRobotSimulationCore.events`. Lazy per-entity instance on `SimObject.events` property. Collision Enter/Exit tracking via `_prev_collision_pairs` set diff. Three profiling fields added to `_profiling_stats`.

**Tech Stack:** Python stdlib only (collections.defaultdict, time.perf_counter). No new dependencies.

---

### Task 1: EventBus class — tests (SERIAL)

**Files:**
- Create: `tests/test_events.py`

**Step 1: Write EventBus unit tests**

```python
# tests/test_events.py
"""Tests for EventBus class and sim-level event integration."""

import pytest

from pybullet_fleet.events import EventBus


# === EventBus unit tests ===


class TestEventBusUnit:
    """Unit tests for EventBus class in isolation."""

    def test_on_emit(self):
        bus = EventBus()
        results = []
        bus.on("test", lambda **kw: results.append(kw))
        bus.emit("test", x=1)
        assert results == [{"x": 1}]

    def test_emit_no_handlers(self):
        """Emit with no handlers does not raise."""
        bus = EventBus()
        bus.emit("nonexistent", x=1)

    def test_priority_order(self):
        bus = EventBus()
        order = []
        bus.on("test", lambda: order.append("B"), priority=10)
        bus.on("test", lambda: order.append("A"), priority=0)
        bus.emit("test")
        assert order == ["A", "B"]

    def test_fifo_within_same_priority(self):
        bus = EventBus()
        order = []
        bus.on("test", lambda: order.append("first"), priority=0)
        bus.on("test", lambda: order.append("second"), priority=0)
        bus.emit("test")
        assert order == ["first", "second"]

    def test_off(self):
        bus = EventBus()
        results = []
        handler = lambda **kw: results.append(1)
        bus.on("test", handler)
        bus.off("test", handler)
        bus.emit("test")
        assert results == []

    def test_off_nonexistent_handler(self):
        """Removing a handler that was never registered doesn't raise."""
        bus = EventBus()
        bus.off("test", lambda: None)

    def test_handler_exception_isolated(self):
        bus = EventBus()
        results = []
        bus.on("test", lambda: 1 / 0, priority=0)
        bus.on("test", lambda: results.append("ok"), priority=1)
        bus.emit("test")
        assert results == ["ok"]

    def test_clear_all(self):
        bus = EventBus()
        bus.on("a", lambda: None)
        bus.on("b", lambda: None)
        bus.clear()
        assert not bus.has_handlers("a")
        assert not bus.has_handlers("b")

    def test_clear_specific(self):
        bus = EventBus()
        bus.on("a", lambda: None)
        bus.on("b", lambda: None)
        bus.clear("a")
        assert not bus.has_handlers("a")
        assert bus.has_handlers("b")

    def test_has_handlers(self):
        bus = EventBus()
        assert not bus.has_handlers("x")
        bus.on("x", lambda: None)
        assert bus.has_handlers("x")

    def test_multiple_events_independent(self):
        bus = EventBus()
        a_results = []
        b_results = []
        bus.on("a", lambda: a_results.append(1))
        bus.on("b", lambda: b_results.append(2))
        bus.emit("a")
        assert a_results == [1]
        assert b_results == []
```

**Step 2: Run test to verify it fails**

```bash
pytest tests/test_events.py -v
```
Expected: FAIL — `ModuleNotFoundError: No module named 'pybullet_fleet.events'`

**Step 3: Commit test file**

```bash
git add tests/test_events.py
git commit -m "test: add EventBus unit tests (red)"
```

---

### Task 2: EventBus class — implementation (SERIAL, depends on Task 1)

**Files:**
- Create: `pybullet_fleet/events.py`

**Step 1: Write EventBus implementation**

```python
# pybullet_fleet/events.py
"""Lightweight event pub/sub for simulation lifecycle events.

Two usage levels:

- **Global bus** (``sim.events``): sim-wide events (pre_step, agent_spawned, etc.)
- **Per-entity bus** (``agent.events``): entity-specific hooks for subclasses

Handlers execute in priority order (lower = first). Same priority = FIFO.
Exceptions in handlers are logged but do not block other handlers.
"""

from collections import defaultdict
from typing import Any, Callable, Dict, List, Optional

from pybullet_fleet.logging_utils import get_lazy_logger

logger = get_lazy_logger(__name__)


class EventBus:
    """Lightweight event pub/sub bus.

    Usage::

        bus = EventBus()
        bus.on("collision_started", my_handler, priority=0)
        bus.emit("collision_started", obj_a=a, obj_b=b)
        bus.off("collision_started", my_handler)
    """

    __slots__ = ("_handlers", "_sorted")

    def __init__(self) -> None:
        self._handlers: Dict[str, List[tuple]] = defaultdict(list)
        self._sorted: Dict[str, bool] = {}

    def on(self, event: str, handler: Callable, priority: int = 0) -> None:
        """Register handler. Lower priority = executed first."""
        self._handlers[event].append((priority, handler))
        self._sorted[event] = False

    def off(self, event: str, handler: Callable) -> None:
        """Unregister handler."""
        self._handlers[event] = [(p, h) for p, h in self._handlers[event] if h is not handler]

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
                logger.exception(lambda: f"EventBus handler error for event '{event}'")

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
```

**Step 2: Run tests to verify they pass**

```bash
pytest tests/test_events.py -v
```
Expected: ALL PASS

**Step 3: Commit**

```bash
git add pybullet_fleet/events.py
git commit -m "feat(events): add EventBus class with priority, off, clear"
```

---

### Task 3: SimObject lazy per-entity EventBus (SERIAL, depends on Task 2)

**Files:**
- Modify: `pybullet_fleet/sim_object.py:270-300` — add `_events` field to `__init__`
- Modify: `pybullet_fleet/sim_object.py` — add `events` property and `_has_entity_events()` after properties section
- Add tests: `tests/test_events.py`

**Step 1: Add per-entity tests to `tests/test_events.py`**

Append to `tests/test_events.py`:

```python
# === Per-entity EventBus tests ===

import pybullet as p
import pybullet_data


@pytest.fixture
def pybullet_direct():
    """Headless PyBullet session for SimObject tests."""
    cid = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=cid)
    yield cid
    p.disconnect(cid)


class TestPerEntityEventBus:
    """Tests for lazy per-entity EventBus on SimObject."""

    def test_lazy_creation_not_created(self, pybullet_direct):
        """Bus is not created until .events is accessed."""
        from pybullet_fleet.sim_object import SimObject, ShapeParams

        obj = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
        )
        assert obj._events is None
        assert not obj._has_entity_events()

    def test_lazy_creation_triggered_on_access(self, pybullet_direct):
        """Accessing .events creates the EventBus."""
        from pybullet_fleet.sim_object import SimObject, ShapeParams

        obj = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
        )
        bus = obj.events
        assert bus is not None
        assert obj._has_entity_events()

    def test_per_entity_on_emit(self, pybullet_direct):
        """Per-entity EventBus on/emit works."""
        from pybullet_fleet.sim_object import SimObject, ShapeParams

        obj = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
        )
        results = []
        obj.events.on("test_event", lambda **kw: results.append(kw))
        obj.events.emit("test_event", value=42)
        assert results == [{"value": 42}]
```

**Step 2: Run test to verify it fails**

```bash
pytest tests/test_events.py::TestPerEntityEventBus -v
```
Expected: FAIL — `AttributeError: 'SimObject' object has no attribute '_events'`

**Step 3: Add `_events` to `SimObject.__init__()` and `events` property**

In `pybullet_fleet/sim_object.py`, add to `__init__()` after `self.name = name` (around line 286):

```python
        # Per-entity EventBus (lazy — created on first .events access)
        self._events: Optional["EventBus"] = None
```

Add property and helper after `_update_log_prefix()` method (around line 310):

```python
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

    def _has_entity_events(self) -> bool:
        """Fast check: True if per-entity EventBus has been created."""
        return self._events is not None
```

Add `"EventBus"` to the TYPE_CHECKING imports at top of file if needed (for type hints in docstrings).

**Step 4: Run tests to verify they pass**

```bash
pytest tests/test_events.py -v
```
Expected: ALL PASS

**Step 5: Run all existing tests to verify no regressions**

```bash
pytest tests/ -x -q
```
Expected: ALL PASS

**Step 6: Commit**

```bash
git add pybullet_fleet/sim_object.py tests/test_events.py
git commit -m "feat(events): add lazy per-entity EventBus on SimObject"
```

---

### Task 4: Global EventBus on core_simulation + pre/post_step (SERIAL, depends on Task 3)

**Files:**
- Modify: `pybullet_fleet/core_simulation.py:28` — add import
- Modify: `pybullet_fleet/core_simulation.py:252-262` — add `events`, `_prev_collision_pairs`, profiling fields to `__init__`
- Modify: `pybullet_fleet/core_simulation.py:2500-2620` — add pre/post_step emit to `step_once()`
- Add tests: `tests/test_events.py`

**Step 1: Add pre/post_step integration tests**

Append to `tests/test_events.py`:

```python
# === core_simulation integration tests ===

from pybullet_fleet import MultiRobotSimulationCore, SimulationParams


@pytest.fixture
def sim_core():
    """Headless sim_core for integration tests."""
    sim = MultiRobotSimulationCore(SimulationParams(gui=False, monitor=False))
    sim.initialize_simulation()
    yield sim
    try:
        p.disconnect(sim.client)
    except p.error:
        pass


class TestGlobalEventBus:
    """Tests for Global EventBus on MultiRobotSimulationCore."""

    def test_has_events_attribute(self, sim_core):
        assert hasattr(sim_core, "events")

    def test_pre_step_fires(self, sim_core):
        events = []
        sim_core.events.on("pre_step", lambda **kw: events.append(("pre", kw)))
        sim_core.step_once()
        assert len(events) == 1
        assert events[0][0] == "pre"
        assert "dt" in events[0][1]
        assert "sim_time" in events[0][1]

    def test_post_step_fires(self, sim_core):
        events = []
        sim_core.events.on("post_step", lambda **kw: events.append(("post", kw)))
        sim_core.step_once()
        assert len(events) == 1
        assert events[0][0] == "post"

    def test_pre_before_post(self, sim_core):
        order = []
        sim_core.events.on("pre_step", lambda **kw: order.append("pre"))
        sim_core.events.on("post_step", lambda **kw: order.append("post"))
        sim_core.step_once()
        assert order == ["pre", "post"]

    def test_pre_step_skipped_when_paused(self, sim_core):
        events = []
        sim_core.events.on("pre_step", lambda **kw: events.append(1))
        sim_core.pause()
        sim_core.step_once()
        assert events == []

    def test_register_callback_still_works(self, sim_core):
        """Backward compatibility: register_callback unaffected."""
        results = []
        sim_core.register_callback(lambda sim, dt: results.append(dt))
        sim_core.step_once()
        assert len(results) == 1
```

**Step 2: Run test to verify it fails**

```bash
pytest tests/test_events.py::TestGlobalEventBus -v
```
Expected: FAIL — `AttributeError: 'MultiRobotSimulationCore' object has no attribute 'events'`

**Step 3: Add import, init fields, and step_once emit**

In `pybullet_fleet/core_simulation.py`:

1. Add import after line 28 (after `from pybullet_fleet._defaults import SIMULATION as _SIM_D`):

```python
from pybullet_fleet.events import EventBus
```

2. After the `_profiling_stats` dict init (around line 260, after `"total": [],`), add:

```python
            "events_pre_step": [],
            "events_post_step": [],
            "events_collision": [],
```

3. After the entire `_profiling_stats` block (around line 262), add:

```python
        # --- EventBus ---
        self.events: EventBus = EventBus()
        self._prev_collision_pairs: set = set()  # Set[FrozenSet[int]] for Enter/Exit tracking
```

4. In `step_once()`, after the `self.sim_time = ...` line (around line 2509) and before the agent update `for agent in self._agents:`, add:

```python
        # EventBus: pre_step
        if measure_timing:
            t_ev_pre = time.perf_counter()
        self.events.emit("pre_step", dt=self._params.timestep, sim_time=self.sim_time)
        if measure_timing:
            self._profiling_stats["events_pre_step"][-1] = (time.perf_counter() - t_ev_pre) * 1000
```

5. In `step_once()`, at the very end — after the monitor timing and before `if return_profiling:` block (around line 2616), add:

```python
        # EventBus: post_step
        if measure_timing:
            t_ev_post = time.perf_counter()
        self.events.emit("post_step", dt=self._params.timestep, sim_time=self.sim_time)
        if measure_timing:
            self._profiling_stats["events_post_step"][-1] = (time.perf_counter() - t_ev_post) * 1000
```

**Step 4: Run tests to verify they pass**

```bash
pytest tests/test_events.py -v
```
Expected: ALL PASS

**Step 5: Run all tests**

```bash
pytest tests/ -x -q
```
Expected: ALL PASS

**Step 6: Commit**

```bash
git add pybullet_fleet/core_simulation.py tests/test_events.py
git commit -m "feat(events): add Global EventBus with pre/post_step emit"
```

---

### Task 5: Entity lifecycle events — agent_spawned/removed, object_spawned/removed (SERIAL, depends on Task 4)

**Files:**
- Modify: `pybullet_fleet/core_simulation.py:1190-1240` — `add_object()` and `remove_object()`
- Add tests: `tests/test_events.py`

**Step 1: Add lifecycle event tests**

Append to `tests/test_events.py`:

```python
from pybullet_fleet import Agent, AgentSpawnParams, Pose
from pybullet_fleet.sim_object import SimObject, ShapeParams
from pybullet_fleet.types import MotionMode


class TestLifecycleEvents:
    """Tests for entity spawn/remove events."""

    def test_object_spawned(self, sim_core):
        spawned = []
        sim_core.events.on("object_spawned", lambda obj: spawned.append(obj))
        obj = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            sim_core=sim_core,
        )
        assert len(spawned) == 1
        assert spawned[0] is obj

    def test_agent_spawned(self, sim_core):
        added = []
        sim_core.events.on("agent_spawned", lambda agent: added.append(agent))
        agent = Agent.from_params(
            AgentSpawnParams(
                urdf_path="robots/mobile_robot.urdf",
                initial_pose=Pose.from_xyz(0, 0, 0.1),
                motion_mode=MotionMode.OMNIDIRECTIONAL,
            ),
            sim_core=sim_core,
        )
        assert len(added) == 1
        assert added[0] is agent

    def test_agent_spawned_also_fires_object_spawned(self, sim_core):
        """Agent is a SimObject, so object_spawned should also fire."""
        spawned = []
        sim_core.events.on("object_spawned", lambda obj: spawned.append(obj))
        agent = Agent.from_params(
            AgentSpawnParams(
                urdf_path="robots/mobile_robot.urdf",
                initial_pose=Pose.from_xyz(0, 0, 0.1),
                motion_mode=MotionMode.OMNIDIRECTIONAL,
            ),
            sim_core=sim_core,
        )
        assert len(spawned) == 1

    def test_object_removed(self, sim_core):
        removed = []
        sim_core.events.on("object_removed", lambda obj: removed.append(obj))
        obj = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            sim_core=sim_core,
        )
        sim_core.remove_object(obj)
        assert len(removed) == 1
        assert removed[0] is obj

    def test_agent_removed(self, sim_core):
        removed = []
        sim_core.events.on("agent_removed", lambda agent: removed.append(agent))
        agent = Agent.from_params(
            AgentSpawnParams(
                urdf_path="robots/mobile_robot.urdf",
                initial_pose=Pose.from_xyz(0, 0, 0.1),
                motion_mode=MotionMode.OMNIDIRECTIONAL,
            ),
            sim_core=sim_core,
        )
        sim_core.remove_object(agent)
        assert len(removed) == 1

    def test_per_entity_object_spawned(self, sim_core):
        """Per-entity bus fires object_spawned if handlers registered before add_object."""
        obj = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
        )
        per_entity_events = []
        obj.events.on("object_spawned", lambda: per_entity_events.append(1))
        sim_core.add_object(obj)
        assert len(per_entity_events) == 1
```

**Step 2: Run test to verify it fails**

```bash
pytest tests/test_events.py::TestLifecycleEvents -v
```
Expected: FAIL

**Step 3: Add emit calls to `add_object()` and `remove_object()`**

In `pybullet_fleet/core_simulation.py`:

In `add_object()`, at the end of the method (after the `_robot_original_colors` block, around line 1218):

```python
        # EventBus: lifecycle events
        self.events.emit("object_spawned", obj=obj)
        if obj._has_entity_events():
            obj.events.emit("object_spawned")
        if isinstance(obj, Agent):
            self.events.emit("agent_spawned", agent=obj)
            if obj._has_entity_events():
                obj.events.emit("agent_spawned")
```

In `remove_object()`, at the start — right after the `try/except ValueError` early-return (around line 1225):

```python
        # EventBus: lifecycle events (emit before actual removal)
        if isinstance(obj, Agent):
            self.events.emit("agent_removed", agent=obj)
            if obj._has_entity_events():
                obj.events.emit("agent_removed")
        self.events.emit("object_removed", obj=obj)
        if obj._has_entity_events():
            obj.events.emit("object_removed")
```

**Step 4: Run tests**

```bash
pytest tests/test_events.py -v
```
Expected: ALL PASS

**Step 5: Commit**

```bash
git add pybullet_fleet/core_simulation.py tests/test_events.py
git commit -m "feat(events): add entity lifecycle events (spawned/removed)"
```

---

### Task 6: Collision Enter/Exit events (SERIAL, depends on Task 5)

**Files:**
- Modify: `pybullet_fleet/core_simulation.py:2580-2600` — add collision diff + emit after collision detection in `step_once()`
- Add tests: `tests/test_events.py`

**Step 1: Add collision Enter/Exit tests**

Append to `tests/test_events.py`:

```python
from pybullet_fleet.types import CollisionMode


class TestCollisionEvents:
    """Tests for collision Enter/Exit events."""

    def test_collision_started_fires(self, sim_core):
        """Two overlapping objects → collision_started fires."""
        started = []
        sim_core.events.on("collision_started", lambda obj_a, obj_b: started.append((obj_a, obj_b)))

        obj_a = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="sphere", radius=0.5),
            collision_shape=ShapeParams(shape_type="sphere", radius=0.5),
            pose=Pose.from_xyz(0, 0, 0.5),
            sim_core=sim_core,
            collision_mode=CollisionMode.NORMAL_3D,
        )
        obj_b = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="sphere", radius=0.5),
            collision_shape=ShapeParams(shape_type="sphere", radius=0.5),
            pose=Pose.from_xyz(0.3, 0, 0.5),
            sim_core=sim_core,
            collision_mode=CollisionMode.NORMAL_3D,
        )
        sim_core.step_once()
        assert len(started) >= 1

    def test_collision_started_not_repeated(self, sim_core):
        """Sustained collision does NOT re-fire collision_started."""
        started = []
        sim_core.events.on("collision_started", lambda obj_a, obj_b: started.append(1))

        SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="sphere", radius=0.5),
            collision_shape=ShapeParams(shape_type="sphere", radius=0.5),
            pose=Pose.from_xyz(0, 0, 0.5),
            sim_core=sim_core,
            collision_mode=CollisionMode.NORMAL_3D,
        )
        SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="sphere", radius=0.5),
            collision_shape=ShapeParams(shape_type="sphere", radius=0.5),
            pose=Pose.from_xyz(0.3, 0, 0.5),
            sim_core=sim_core,
            collision_mode=CollisionMode.NORMAL_3D,
        )
        sim_core.step_once()
        count_after_first = len(started)
        sim_core.step_once()
        assert len(started) == count_after_first  # No re-fire

    def test_collision_ended_fires(self, sim_core):
        """Objects separate → collision_ended fires."""
        ended = []
        sim_core.events.on("collision_ended", lambda obj_a, obj_b: ended.append((obj_a, obj_b)))

        obj_a = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="sphere", radius=0.5),
            collision_shape=ShapeParams(shape_type="sphere", radius=0.5),
            pose=Pose.from_xyz(0, 0, 0.5),
            sim_core=sim_core,
            collision_mode=CollisionMode.NORMAL_3D,
        )
        obj_b = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="sphere", radius=0.5),
            collision_shape=ShapeParams(shape_type="sphere", radius=0.5),
            pose=Pose.from_xyz(0.3, 0, 0.5),
            sim_core=sim_core,
            collision_mode=CollisionMode.NORMAL_3D,
        )
        sim_core.step_once()  # collision_started
        # Move apart
        obj_b.set_pose(Pose.from_xyz(10, 10, 0.5))
        sim_core.step_once()  # collision_ended
        assert len(ended) >= 1

    def test_per_entity_collision_started(self, sim_core):
        """Per-entity collision_started fires with 'other' kwarg."""
        obj_a = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="sphere", radius=0.5),
            collision_shape=ShapeParams(shape_type="sphere", radius=0.5),
            pose=Pose.from_xyz(0, 0, 0.5),
            sim_core=sim_core,
            collision_mode=CollisionMode.NORMAL_3D,
        )
        obj_b = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="sphere", radius=0.5),
            collision_shape=ShapeParams(shape_type="sphere", radius=0.5),
            pose=Pose.from_xyz(0.3, 0, 0.5),
            sim_core=sim_core,
            collision_mode=CollisionMode.NORMAL_3D,
        )
        a_collisions = []
        obj_a.events.on("collision_started", lambda other: a_collisions.append(other))
        sim_core.step_once()
        assert len(a_collisions) == 1
        assert a_collisions[0] is obj_b

    def test_no_collision_events_when_no_handlers(self, sim_core):
        """No Enter/Exit processing when no handlers registered (performance guard)."""
        # Just verify step_once works without collision handlers
        SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="sphere", radius=0.5),
            collision_shape=ShapeParams(shape_type="sphere", radius=0.5),
            pose=Pose.from_xyz(0, 0, 0.5),
            sim_core=sim_core,
            collision_mode=CollisionMode.NORMAL_3D,
        )
        sim_core.step_once()  # Should not fail
```

**Step 2: Run test to verify it fails**

```bash
pytest tests/test_events.py::TestCollisionEvents -v
```
Expected: FAIL

**Step 3: Add collision Enter/Exit logic to `step_once()`**

In `pybullet_fleet/core_simulation.py`, in `step_once()` after the collision check timing (after `self._profiling_stats["collision_check"][-1] = ...`, around line 2600), add:

```python
        # EventBus: collision Enter/Exit diff
        if measure_timing:
            t_ev_col = time.perf_counter()
        _has_collision_handlers = (
            self.events.has_handlers("collision_started")
            or self.events.has_handlers("collision_ended")
            or any(obj._has_entity_events() for obj in self._sim_objects_dict.values())
        )
        if _has_collision_handlers:
            current_pairs = {frozenset(pair) for pair in self._active_collision_pairs}
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
```

**Step 4: Run tests**

```bash
pytest tests/test_events.py -v
```
Expected: ALL PASS

**Step 5: Run all tests**

```bash
pytest tests/ -x -q
```
Expected: ALL PASS

**Step 6: Commit**

```bash
git add pybullet_fleet/core_simulation.py tests/test_events.py
git commit -m "feat(events): add collision Enter/Exit events with profiling"
```

---

### Task 7: Pause/Resume events (SERIAL, depends on Task 4)

**Files:**
- Modify: `pybullet_fleet/core_simulation.py:2429-2437` — `pause()` and `resume()`
- Add tests: `tests/test_events.py`

**Step 1: Add pause/resume tests**

Append to `tests/test_events.py`:

```python
class TestPauseResumeEvents:
    def test_paused_event(self, sim_core):
        events = []
        sim_core.events.on("paused", lambda: events.append("paused"))
        sim_core.pause()
        assert events == ["paused"]

    def test_resumed_event(self, sim_core):
        events = []
        sim_core.events.on("resumed", lambda: events.append("resumed"))
        sim_core.pause()
        sim_core.resume()
        assert events == ["resumed"]
```

**Step 2: Run test to verify it fails**

```bash
pytest tests/test_events.py::TestPauseResumeEvents -v
```

**Step 3: Add emit to `pause()` and `resume()`**

In `pybullet_fleet/core_simulation.py`:

In `pause()` (around line 2430), after `self._simulation_paused = True`:

```python
        self.events.emit("paused")
```

In `resume()` (around line 2434), after `self._simulation_paused = False`:

```python
        self.events.emit("resumed")
```

**Step 4: Run tests**

```bash
pytest tests/test_events.py::TestPauseResumeEvents -v
```
Expected: PASS

**Step 5: Commit**

```bash
git add pybullet_fleet/core_simulation.py tests/test_events.py
git commit -m "feat(events): add paused/resumed events"
```

---

### Task 8: Action events (SERIAL, depends on Task 4)

**Files:**
- Modify: `pybullet_fleet/agent.py:1108-1145` — `_update_actions()`
- Add tests: `tests/test_events.py`

**Step 1: Add action event tests**

Append to `tests/test_events.py`:

```python
from pybullet_fleet.action import MoveAction
from pybullet_fleet.geometry import Path as PbfPath
from pybullet_fleet.types import ActionStatus


class TestActionEvents:
    def test_action_started_global(self, sim_core):
        started = []
        sim_core.events.on("action_started", lambda agent, action: started.append(action))
        agent = Agent.from_params(
            AgentSpawnParams(
                urdf_path="robots/mobile_robot.urdf",
                initial_pose=Pose.from_xyz(0, 0, 0.1),
                motion_mode=MotionMode.OMNIDIRECTIONAL,
            ),
            sim_core=sim_core,
        )
        action = MoveAction(path=PbfPath.from_positions([[5, 5, 0.1]]))
        agent.add_action(action)
        sim_core.step_once()
        assert len(started) == 1
        assert started[0] is action

    def test_action_completed_global(self, sim_core):
        completed = []
        sim_core.events.on(
            "action_completed",
            lambda agent, action, status: completed.append(status),
        )
        agent = Agent.from_params(
            AgentSpawnParams(
                urdf_path="robots/mobile_robot.urdf",
                initial_pose=Pose.from_xyz(0, 0, 0.1),
                motion_mode=MotionMode.OMNIDIRECTIONAL,
                max_linear_vel=100.0,
            ),
            sim_core=sim_core,
        )
        # Very close goal — should complete quickly
        action = MoveAction(path=PbfPath.from_positions([[0.001, 0, 0.1]]))
        agent.add_action(action)
        for _ in range(500):
            sim_core.step_once()
            if action.is_complete():
                break
        assert len(completed) >= 1

    def test_action_started_per_entity(self, sim_core):
        agent = Agent.from_params(
            AgentSpawnParams(
                urdf_path="robots/mobile_robot.urdf",
                initial_pose=Pose.from_xyz(0, 0, 0.1),
                motion_mode=MotionMode.OMNIDIRECTIONAL,
            ),
            sim_core=sim_core,
        )
        per_entity = []
        agent.events.on("action_started", lambda action: per_entity.append(action))
        action = MoveAction(path=PbfPath.from_positions([[5, 5, 0.1]]))
        agent.add_action(action)
        sim_core.step_once()
        assert len(per_entity) == 1
```

**Step 2: Run test to verify it fails**

```bash
pytest tests/test_events.py::TestActionEvents -v
```

**Step 3: Add action event emit to `_update_actions()`**

In `pybullet_fleet/agent.py`, in `_update_actions()`:

After the line `self._current_action = self._action_queue.pop(0)` and the log line (around line 1123), before `else: return`:

No — the emit should happen when the action starts executing (first `execute()` call). Two locations:

1. When action is popped from queue and about to be executed for the first time — add emit right after `self._log.info(...)`:

```python
                # EventBus: action_started
                if hasattr(self, 'sim_core') and self.sim_core is not None and hasattr(self.sim_core, 'events'):
                    self.sim_core.events.emit("action_started", agent=self, action=self._current_action)
                if self._has_entity_events():
                    self.events.emit("action_started", action=self._current_action)
```

2. When `is_complete` is True, after the status logging (around line 1140), before `self._current_action = None`:

```python
            # EventBus: action_completed
            if hasattr(self, 'sim_core') and self.sim_core is not None and hasattr(self.sim_core, 'events'):
                self.sim_core.events.emit(
                    "action_completed", agent=self, action=self._current_action, status=status
                )
            if self._has_entity_events():
                self.events.emit("action_completed", action=self._current_action, status=status)
```

**Step 4: Run tests**

```bash
pytest tests/test_events.py -v
```
Expected: ALL PASS

**Step 5: Run all tests**

```bash
pytest tests/ -x -q
```
Expected: ALL PASS

**Step 6: Commit**

```bash
git add pybullet_fleet/agent.py tests/test_events.py
git commit -m "feat(events): add action_started/completed events"
```

---

### Task 9: Public API export (SERIAL, depends on Task 2)

**Files:**
- Modify: `pybullet_fleet/__init__.py`

**Step 1: Add EventBus export**

Add import:

```python
from pybullet_fleet.events import EventBus
```

Add to `__all__`:

```python
    "EventBus",
```

**Step 2: Verify import works**

```bash
python -c "from pybullet_fleet import EventBus; print('OK')"
```

**Step 3: Commit**

```bash
git add pybullet_fleet/__init__.py
git commit -m "feat(events): export EventBus from public API"
```

---

### Task 10: Final verification (SERIAL, depends on all)

**Step 1: Run full test suite**

```bash
make verify
```
Expected: ALL PASS, lint clean

**Step 2: Check coverage**

```bash
make test
```
Expected: Coverage ≥ 75%

**Step 3: Final commit (if any formatting fixes)**

```bash
make format
git add -A
git commit -m "style: format EventBus code"
```
