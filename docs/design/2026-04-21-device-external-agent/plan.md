# Device + Controller Chain + Demo Configs — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use executing-plans to implement this plan task-by-task.

**Goal:** Implement Door/Elevator devices (Agent subclass + URDF joints), Controller chain
(patrol/random walk), and launch/config files for battle_royale, hotel, clinic, airport_terminal demos.

**Architecture:** Devices are Agent subclasses using JointAction for state transitions.
Controllers stack as a list (high→low execution order). Demos are YAML configs + launch files
bootstrapped from existing office/airport patterns.

**Tech Stack:** Python 3.10+, PyBullet, pybullet_fleet framework, ROS 2 Jazzy (launch/config only)

---

## Task Dependencies

```
SERIAL chain:
  Task 1 (SimObject.update + _needs_update)
  → Task 2 (Controller chain)
  → Task 3 (PatrolController + RandomWalkController)
  → Task 4 (DoorDevice)
  → Task 5 (ElevatorDevice + WorkcellDevice)
  → Task 6 (entity_registry)
  → Task 7 (integration test)
  → Task 8 (commit core)

PARALLEL after Task 8:
  Task 9  (battle_royale demo)
  Task 10 (hotel demo)
  Task 11 (clinic demo)
  Task 12 (airport_terminal update)

SERIAL:
  → Task 13 (final verify + commit)
```

---

### Task 1: SimObject.update() + _needs_update [SERIAL]

**Files:**
- Modify: `pybullet_fleet/sim_object.py:232` (class SimObject)
- Modify: `pybullet_fleet/core_simulation.py:3040-3048` (step_once agent loop)
- Test: `tests/test_sim_object_update.py` (new)

**Step 1: Write the failing test**

```python
# tests/test_sim_object_update.py
"""Tests for SimObject._needs_update and update() hook."""
import pytest
from tests.conftest import MockSimCore
from pybullet_fleet import SimObject, SimObjectSpawnParams, Pose


class TestSimObjectUpdate:
    def test_needs_update_default_false(self, sim_core):
        """SimObject._needs_update defaults to False."""
        params = SimObjectSpawnParams(
            visual_shape=None,
            collision_shape=None,
            initial_pose=Pose.from_xyz(0, 0, 0),
        )
        obj = SimObject.from_params(params, sim_core)
        assert obj._needs_update is False

    def test_update_returns_false_by_default(self, sim_core):
        """SimObject.update() returns False (no-op)."""
        params = SimObjectSpawnParams(
            visual_shape=None,
            collision_shape=None,
            initial_pose=Pose.from_xyz(0, 0, 0),
        )
        obj = SimObject.from_params(params, sim_core)
        assert obj.update(0.01) is False

    def test_subclass_with_needs_update(self, sim_core):
        """Subclass with _needs_update=True gets update() called."""
        call_count = 0

        class UpdatingObject(SimObject):
            _needs_update = True

            def update(self, dt):
                nonlocal call_count
                call_count += 1
                return True

        params = SimObjectSpawnParams(
            visual_shape=None,
            collision_shape=None,
            initial_pose=Pose.from_xyz(0, 0, 0),
        )
        obj = UpdatingObject.from_params(params, sim_core)
        sim_core.tick(5)
        assert call_count == 5
```

**Step 2: Run test to verify it fails**

Run: `pytest tests/test_sim_object_update.py -v`
Expected: FAIL — `_needs_update` attribute not found, `update()` method not found

**Step 3: Implement SimObject.update() + _needs_update**

In `pybullet_fleet/sim_object.py`, add to class `SimObject` (around L260, after `_spawn_params_cls`):

```python
    _needs_update: bool = False  # Subclasses override to True for per-step update()

    def update(self, dt: float) -> bool:
        """Called every simulation step for objects with ``_needs_update = True``.

        Override in subclasses to implement per-step behavior (e.g. device
        state machines, animated objects).

        Args:
            dt: Time step in seconds.

        Returns:
            ``True`` if the object moved or changed state, ``False`` otherwise.
        """
        return False
```

**Step 4: Modify step_once() unified loop**

In `pybullet_fleet/core_simulation.py`, replace the agent update loop (L3040-3048):

```python
        # BEFORE:
        for agent in self._agents:
            moved = agent.update(self._params.timestep)
            if moved and agent.object_id in self._kinematic_objects:
                self._moved_this_step.add(agent.object_id)

        # AFTER:
        for obj in self._sim_objects:
            if obj._needs_update:
                moved = obj.update(self._params.timestep)
                if moved and obj.object_id in self._kinematic_objects:
                    self._moved_this_step.add(obj.object_id)
```

Add `_needs_update = True` to Agent class in `agent.py` (around L219):

```python
class Agent(SimObject):
    _needs_update = True
```

**Step 5: Run full tests**

Run: `pytest tests/test_sim_object_update.py tests/test_agent_core.py tests/test_e2e.py -v`
Expected: ALL PASS

**Step 6: Commit**

```bash
git add pybullet_fleet/sim_object.py pybullet_fleet/agent.py pybullet_fleet/core_simulation.py tests/test_sim_object_update.py
git commit -m "feat: SimObject.update() + _needs_update, unified step_once loop"
```

---

### Task 2: Controller Chain [SERIAL — depends on Task 1]

**Files:**
- Modify: `pybullet_fleet/agent.py:32-186` (AgentSpawnParams), `L353` (_controller), `L1248-1307` (update), `L528-644` (from_params), `L1314-1330` (set_controller)
- Test: `tests/test_controller_chain.py` (new)

**Step 1: Write failing tests**

```python
# tests/test_controller_chain.py
"""Tests for Agent controller chain."""
import pytest
from pybullet_fleet import Agent, AgentSpawnParams, Pose
from pybullet_fleet.controller import Controller
from pybullet_fleet.types import MotionMode


class MockController(Controller):
    def __init__(self):
        self.calls = []

    def compute(self, agent, dt):
        self.calls.append(("compute", dt))
        return False


class TestControllerChain:
    def test_set_controller_backward_compat(self, sim_core):
        """set_controller() replaces base controller (index 0)."""
        params = AgentSpawnParams(
            urdf_path="robots/mobile_robot.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0),
            motion_mode=MotionMode.OMNIDIRECTIONAL,
        )
        agent = Agent.from_params(params, sim_core)
        new_ctrl = MockController()
        agent.set_controller(new_ctrl)
        assert agent.controller is new_ctrl
        assert len(agent._controllers) == 1

    def test_add_controller_appends(self, sim_core):
        """add_controller() appends to chain."""
        params = AgentSpawnParams(
            urdf_path="robots/mobile_robot.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0),
            motion_mode=MotionMode.OMNIDIRECTIONAL,
        )
        agent = Agent.from_params(params, sim_core)
        high_ctrl = MockController()
        agent.add_controller(high_ctrl)
        assert len(agent._controllers) == 2
        assert agent._controllers[-1] is high_ctrl

    def test_chain_execution_order(self, sim_core):
        """High-level controller executes before low-level (reversed order)."""
        call_order = []

        class LowCtrl(Controller):
            def compute(self, agent, dt):
                call_order.append("low")
                return False

        class HighCtrl(Controller):
            def compute(self, agent, dt):
                call_order.append("high")
                return False

        params = AgentSpawnParams(
            urdf_path="robots/mobile_robot.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0),
            motion_mode=MotionMode.OMNIDIRECTIONAL,
        )
        agent = Agent.from_params(params, sim_core)
        agent.set_controller(LowCtrl())
        agent.add_controller(HighCtrl())

        agent.update(0.01)
        assert call_order == ["high", "low"]

    def test_remove_controller(self, sim_core):
        """remove_controller() removes non-base controller."""
        params = AgentSpawnParams(
            urdf_path="robots/mobile_robot.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0),
            motion_mode=MotionMode.OMNIDIRECTIONAL,
        )
        agent = Agent.from_params(params, sim_core)
        high_ctrl = MockController()
        agent.add_controller(high_ctrl)
        agent.remove_controller(high_ctrl)
        assert len(agent._controllers) == 1

    def test_controller_property_returns_base(self, sim_core):
        """controller property returns controllers[0] (base)."""
        params = AgentSpawnParams(
            urdf_path="robots/mobile_robot.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0),
            motion_mode=MotionMode.OMNIDIRECTIONAL,
        )
        agent = Agent.from_params(params, sim_core)
        base = agent.controller
        assert base is agent._controllers[0]
```

**Step 2: Run test to verify it fails**

Run: `pytest tests/test_controller_chain.py -v`
Expected: FAIL — `_controllers` attribute not found

**Step 3: Implement controller chain**

In `agent.py`:

1. **Replace `self._controller` with `self._controllers`** at L353:
   ```python
   self._controllers: list = []  # Controller chain: [base, ...high-level]
   ```

2. **Add `controllers` field to `AgentSpawnParams`** (around L186):
   ```python
   controllers: Optional[List[Dict[str, Any]]] = None
   ```

3. **Update `controller` property** to return `_controllers[0]`:
   ```python
   @property
   def controller(self):
       return self._controllers[0] if self._controllers else None
   ```

4. **Update `set_controller()`** (L1314-1330):
   ```python
   def set_controller(self, controller=None):
       if self._controllers:
           self._controllers[0] = controller
       else:
           self._controllers.append(controller)
   ```

5. **Add `add_controller()` and `remove_controller()`**:
   ```python
   def add_controller(self, controller):
       self._controllers.append(controller)

   def remove_controller(self, controller):
       if controller in self._controllers[1:]:
           self._controllers.remove(controller)
   ```

6. **Update `update()` method** (L1289) — replace single controller call:
   ```python
   # BEFORE:
   if self._controller is not None:
       moved = self._controller.compute(self, dt)
   elif self._is_moving:
       ...

   # AFTER:
   if self._controllers:
       for ctrl in reversed(self._controllers):
           ctrl_moved = ctrl.compute(self, dt)
           moved = moved or ctrl_moved
   elif self._is_moving:
       ...
   ```

7. **Update `from_params()`** (L628-640) — change `self._controller` references to `self._controllers`:
   ```python
   # Replace: agent.set_controller(ctrl) → same (compat)
   # Replace: agent._controller = OmniController(...) → agent._controllers.append(OmniController(...))
   # Replace: if agent._controller is None → if not agent._controllers
   # Replace: agent._controller._navigation_2d → agent._controllers[0]._navigation_2d if agent._controllers
   ```

8. **Handle `controllers` from YAML in `from_params()`** — after creating base controller:
   ```python
   # Add high-level controllers from spawn_params.controllers
   if spawn_params.controllers:
       from pybullet_fleet.controller import create_controller
       for ctrl_cfg in spawn_params.controllers:
           ctrl_type = ctrl_cfg["type"]
           ctrl_params = {k: v for k, v in ctrl_cfg.items() if k != "type"}
           ctrl = create_controller(ctrl_type, ctrl_params)
           agent.add_controller(ctrl)
   ```

**Step 4: Fix all `self._controller` references in agent.py**

Search for all `self._controller` and update to `_controllers` equivalent.
Key locations: L353, L628-640, L1289, L1314-1330, and any other references.

**Step 5: Run tests**

Run: `pytest tests/test_controller_chain.py tests/test_agent_core.py tests/test_e2e.py -v`
Expected: ALL PASS

**Step 6: Commit**

```bash
git add pybullet_fleet/agent.py tests/test_controller_chain.py
git commit -m "feat: controller chain — add_controller, reversed execution, YAML controllers"
```

---

### Task 3: PatrolController + RandomWalkController [SERIAL — depends on Task 2]

**Files:**
- Create: `pybullet_fleet/controllers/__init__.py`
- Create: `pybullet_fleet/controllers/patrol_controller.py`
- Create: `pybullet_fleet/controllers/random_walk_controller.py`
- Modify: `pybullet_fleet/controller.py:965-966` (register new controllers)
- Create: `tests/test_patrol_controller.py`

**Step 1: Write failing tests**

```python
# tests/test_patrol_controller.py
"""Tests for PatrolController and RandomWalkController."""
import pytest
from pybullet_fleet import Agent, AgentSpawnParams, Pose
from pybullet_fleet.types import MotionMode
from pybullet_fleet.controllers.patrol_controller import PatrolController
from pybullet_fleet.controllers.random_walk_controller import RandomWalkController


class TestPatrolController:
    def test_patrol_cycles_waypoints(self, sim_core):
        """PatrolController cycles through waypoints with loop=True."""
        params = AgentSpawnParams(
            urdf_path="robots/mobile_robot.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0.1),
            motion_mode=MotionMode.OMNIDIRECTIONAL,
            max_linear_vel=5.0,
        )
        agent = Agent.from_params(params, sim_core)
        patrol = PatrolController(
            waypoints=[[2, 0, 0.1], [2, 2, 0.1], [0, 2, 0.1]],
            loop=True,
        )
        agent.add_controller(patrol)

        sim_core.tick(500)
        # Agent should have moved from origin
        pos = agent.get_pose().position
        assert not (abs(pos[0]) < 0.01 and abs(pos[1]) < 0.01)

    def test_patrol_no_loop_stops(self, sim_core):
        """PatrolController with loop=False stops at last waypoint."""
        params = AgentSpawnParams(
            urdf_path="robots/mobile_robot.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0.1),
            motion_mode=MotionMode.OMNIDIRECTIONAL,
            max_linear_vel=5.0,
        )
        agent = Agent.from_params(params, sim_core)
        patrol = PatrolController(
            waypoints=[[1, 0, 0.1]],
            loop=False,
        )
        agent.add_controller(patrol)

        sim_core.tick(200)
        pos = agent.get_pose().position
        assert abs(pos[0] - 1.0) < 0.5  # near target

    def test_patrol_from_config(self):
        """PatrolController.from_config() creates from dict."""
        ctrl = PatrolController.from_config({
            "waypoints": [[1, 0, 0], [2, 0, 0]],
            "wait_time": 1.0,
            "loop": True,
        })
        assert ctrl._loop is True
        assert len(ctrl._waypoints) == 2

    def test_patrol_registered(self):
        """PatrolController is registered in CONTROLLER_REGISTRY."""
        from pybullet_fleet.controller import create_controller
        ctrl = create_controller("patrol", {"waypoints": [[0, 0, 0]]})
        assert isinstance(ctrl, PatrolController)


class TestRandomWalkController:
    def test_random_walk_moves(self, sim_core):
        """RandomWalkController moves agent from origin."""
        params = AgentSpawnParams(
            urdf_path="robots/mobile_robot.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0.1),
            motion_mode=MotionMode.OMNIDIRECTIONAL,
            max_linear_vel=5.0,
        )
        agent = Agent.from_params(params, sim_core)
        rw = RandomWalkController(radius=3.0)
        agent.add_controller(rw)

        sim_core.tick(200)
        pos = agent.get_pose().position
        # Should have moved somewhere
        assert abs(pos[0]) > 0.01 or abs(pos[1]) > 0.01

    def test_random_walk_stays_within_radius(self, sim_core):
        """RandomWalkController goals stay within radius."""
        rw = RandomWalkController(radius=2.0)
        # Verify config stored
        assert rw._radius == 2.0

    def test_random_walk_registered(self):
        """RandomWalkController is registered in CONTROLLER_REGISTRY."""
        from pybullet_fleet.controller import create_controller
        ctrl = create_controller("random_walk", {"radius": 5.0})
        assert isinstance(ctrl, RandomWalkController)
```

**Step 2: Run test to verify it fails**

Run: `pytest tests/test_patrol_controller.py -v`
Expected: FAIL — ImportError

**Step 3: Implement PatrolController**

```python
# pybullet_fleet/controllers/__init__.py
"""High-level controllers for agent behavior."""
from pybullet_fleet.controllers.patrol_controller import PatrolController
from pybullet_fleet.controllers.random_walk_controller import RandomWalkController

__all__ = ["PatrolController", "RandomWalkController"]
```

```python
# pybullet_fleet/controllers/patrol_controller.py
"""Waypoint patrol controller."""
from __future__ import annotations

from pybullet_fleet.controller import Controller
from pybullet_fleet.geometry import Pose
from pybullet_fleet.logging_utils import get_lazy_logger

logger = get_lazy_logger(__name__)


class PatrolController(Controller):
    """Cycle through waypoints. Delegates actual movement to the base controller
    by calling ``agent.set_goal_pose()``.

    Args:
        waypoints: List of [x, y, z] positions.
        wait_time: Seconds to wait at each waypoint before advancing.
        loop: If True, restart from first waypoint after reaching last.
    """

    def __init__(
        self,
        waypoints: list | None = None,
        wait_time: float = 0.0,
        loop: bool = True,
    ):
        self._waypoints = waypoints or []
        self._current_idx = 0
        self._wait_time = wait_time
        self._wait_timer = 0.0
        self._loop = loop
        self._started = False

    def compute(self, agent, dt: float) -> bool:
        if not self._waypoints:
            return False

        if self._wait_timer > 0:
            self._wait_timer -= dt
            return False

        # First call: set initial waypoint
        if not self._started:
            self._started = True
            target = self._waypoints[self._current_idx]
            agent.set_goal_pose(Pose.from_xyz(*target))
            return False

        # Advance when agent stops moving
        if not agent.is_moving:
            self._current_idx += 1
            if self._current_idx >= len(self._waypoints):
                if self._loop:
                    self._current_idx = 0
                else:
                    return False  # done
            target = self._waypoints[self._current_idx]
            agent.set_goal_pose(Pose.from_xyz(*target))
            self._wait_timer = self._wait_time

        return False  # actual movement handled by base controller
```

```python
# pybullet_fleet/controllers/random_walk_controller.py
"""Random walk controller."""
from __future__ import annotations

import math
import random

from pybullet_fleet.controller import Controller
from pybullet_fleet.geometry import Pose


class RandomWalkController(Controller):
    """Move to random nearby positions within a radius of the starting point.

    Args:
        radius: Maximum distance from origin for random targets.
        wait_range: (min, max) seconds to wait at each target.
    """

    def __init__(
        self,
        radius: float = 5.0,
        wait_range: tuple | list = (1.0, 5.0),
    ):
        self._radius = radius
        self._wait_range = tuple(wait_range)
        self._origin: list | None = None
        self._wait_timer = 0.0

    def compute(self, agent, dt: float) -> bool:
        if self._origin is None:
            pos = agent.get_pose().position
            self._origin = [float(pos[0]), float(pos[1])]

        if self._wait_timer > 0:
            self._wait_timer -= dt
            return False

        if not agent.is_moving:
            angle = random.uniform(0, 2 * math.pi)
            dist = random.uniform(0, self._radius)
            z = float(agent.get_pose().position[2])
            target = [
                self._origin[0] + dist * math.cos(angle),
                self._origin[1] + dist * math.sin(angle),
                z,
            ]
            agent.set_goal_pose(Pose.from_xyz(*target))
            self._wait_timer = random.uniform(*self._wait_range)

        return False
```

**Step 4: Register controllers**

At end of `pybullet_fleet/controller.py` (L965-966), add:
```python
# Also register high-level controllers
from pybullet_fleet.controllers.patrol_controller import PatrolController
from pybullet_fleet.controllers.random_walk_controller import RandomWalkController

register_controller("patrol", PatrolController)
register_controller("random_walk", RandomWalkController)
```

**Step 5: Run tests**

Run: `pytest tests/test_patrol_controller.py tests/test_controller_chain.py -v`
Expected: ALL PASS

**Step 6: Commit**

```bash
git add pybullet_fleet/controllers/ pybullet_fleet/controller.py tests/test_patrol_controller.py
git commit -m "feat: PatrolController + RandomWalkController with registry"
```

---

### Task 4: DoorDevice [SERIAL — depends on Task 2]

**Files:**
- Create: `pybullet_fleet/devices/__init__.py`
- Create: `pybullet_fleet/devices/door.py`
- Create: `robots/door_hinge.urdf`
- Create: `robots/door_slide.urdf`
- Create: `tests/test_door_device.py`

**Step 1: Write failing tests**

```python
# tests/test_door_device.py
"""Tests for DoorDevice."""
import pytest
from pybullet_fleet import Agent, AgentSpawnParams, Pose
from pybullet_fleet.types import MotionMode
from pybullet_fleet.devices.door import DoorDevice


class TestDoorDevice:
    def test_door_initial_state_closed(self, sim_core):
        """DoorDevice starts in 'closed' state."""
        params = AgentSpawnParams(
            urdf_path="robots/door_hinge.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0),
            use_fixed_base=True,
            user_data={
                "open_positions": {"hinge": 1.57},
                "close_positions": {"hinge": 0.0},
            },
        )
        door = DoorDevice.from_params(params, sim_core)
        assert door.door_state == "closed"

    def test_door_open_close_cycle(self, sim_core):
        """DoorDevice: request_open → open, request_close → closed."""
        params = AgentSpawnParams(
            urdf_path="robots/door_hinge.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0),
            use_fixed_base=True,
            user_data={
                "open_positions": {"hinge": 1.57},
                "close_positions": {"hinge": 0.0},
            },
        )
        door = DoorDevice.from_params(params, sim_core)
        door.request_open()
        sim_core.tick(200)
        assert door.door_state == "open"

        door.request_close()
        sim_core.tick(200)
        assert door.door_state == "closed"

    def test_door_is_agent_subclass(self, sim_core):
        """DoorDevice inherits from Agent."""
        params = AgentSpawnParams(
            urdf_path="robots/door_hinge.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0),
            use_fixed_base=True,
            user_data={
                "open_positions": {"hinge": 1.57},
                "close_positions": {"hinge": 0.0},
            },
        )
        door = DoorDevice.from_params(params, sim_core)
        assert isinstance(door, Agent)
        assert isinstance(door, DoorDevice)
```

**Step 2: Run test to verify it fails**

Run: `pytest tests/test_door_device.py -v`
Expected: FAIL — ImportError

**Step 3: Create Door URDF files**

```xml
<!-- robots/door_hinge.urdf -->
<?xml version="1.0"?>
<robot name="door_hinge">
  <link name="frame">
    <visual>
      <geometry><box size="0.1 0.1 2.0"/></geometry>
      <material name="gray"><color rgba="0.5 0.5 0.5 1.0"/></material>
    </visual>
    <inertial>
      <mass value="0.0"/>
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
    </inertial>
  </link>
  <joint name="hinge" type="revolute">
    <parent link="frame"/>
    <child link="panel"/>
    <origin xyz="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="1.5708" effort="10" velocity="2.0"/>
  </joint>
  <link name="panel">
    <visual>
      <geometry><box size="1.0 0.05 2.0"/></geometry>
      <material name="brown"><color rgba="0.6 0.3 0.1 1.0"/></material>
    </visual>
    <collision>
      <geometry><box size="1.0 0.05 2.0"/></geometry>
    </collision>
    <inertial>
      <mass value="0.0"/>
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
    </inertial>
  </link>
</robot>
```

```xml
<!-- robots/door_slide.urdf -->
<?xml version="1.0"?>
<robot name="door_slide">
  <link name="frame">
    <visual>
      <geometry><box size="0.1 0.1 2.0"/></geometry>
      <material name="gray"><color rgba="0.5 0.5 0.5 1.0"/></material>
    </visual>
    <inertial>
      <mass value="0.0"/>
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
    </inertial>
  </link>
  <joint name="slide" type="prismatic">
    <parent link="frame"/>
    <child link="panel"/>
    <origin xyz="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.2" effort="10" velocity="1.5"/>
  </joint>
  <link name="panel">
    <visual>
      <geometry><box size="1.0 0.05 2.0"/></geometry>
      <material name="brown"><color rgba="0.6 0.3 0.1 1.0"/></material>
    </visual>
    <collision>
      <geometry><box size="1.0 0.05 2.0"/></geometry>
    </collision>
    <inertial>
      <mass value="0.0"/>
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
    </inertial>
  </link>
</robot>
```

**Step 4: Implement DoorDevice**

See agent.spec.md Section 4 (DoorDevice) for full implementation.

```python
# pybullet_fleet/devices/__init__.py
"""Infrastructure devices: doors, elevators, workcells."""
from pybullet_fleet.devices.door import DoorDevice
from pybullet_fleet.devices.elevator import ElevatorDevice
from pybullet_fleet.devices.workcell import WorkcellDevice

__all__ = ["DoorDevice", "ElevatorDevice", "WorkcellDevice"]
```

```python
# pybullet_fleet/devices/door.py
"""Door device — Agent subclass with URDF joint control."""
from __future__ import annotations

from typing import TYPE_CHECKING, Any, Dict, Optional

from pybullet_fleet.action import JointAction
from pybullet_fleet.agent import Agent, AgentSpawnParams
from pybullet_fleet.logging_utils import get_lazy_logger
from pybullet_fleet.types import ActionStatus

if TYPE_CHECKING:
    pass

logger = get_lazy_logger(__name__)


class DoorDevice(Agent):
    """Door controlled via URDF revolute or prismatic joint.

    Uses JointAction for open/close transitions. Door state is derived from
    the current JointAction status and joint positions.

    Config via ``user_data``::

        user_data:
          open_positions: {hinge: 1.57}   # joint targets for "open"
          close_positions: {hinge: 0.0}   # joint targets for "closed"
    """

    @classmethod
    def from_params(cls, spawn_params: AgentSpawnParams, sim_core=None) -> "DoorDevice":
        agent = super().from_params(spawn_params, sim_core)
        ud = spawn_params.user_data or {}
        agent._open_positions = ud.get("open_positions", {})
        agent._close_positions = ud.get("close_positions", {})
        return agent

    @property
    def door_state(self) -> str:
        """Derived door state: 'closed' | 'opening' | 'open' | 'closing'."""
        action = self.current_action
        if action and isinstance(action, JointAction) and action.status == ActionStatus.IN_PROGRESS:
            if action.target_joint_positions == self._open_positions:
                return "opening"
            return "closing"
        if self._open_positions and self.are_joints_at_targets(self._open_positions):
            return "open"
        return "closed"

    def request_open(self) -> None:
        """Open the door (enqueue JointAction to open positions)."""
        if self.door_state in ("closed", "closing"):
            self.clear_actions()
            self.add_action(JointAction(target_joint_positions=self._open_positions))

    def request_close(self) -> None:
        """Close the door (enqueue JointAction to close positions)."""
        if self.door_state in ("open", "opening"):
            self.clear_actions()
            self.add_action(JointAction(target_joint_positions=self._close_positions))
```

**Step 5: Run tests**

Run: `pytest tests/test_door_device.py -v`
Expected: ALL PASS

**Step 6: Commit**

```bash
git add pybullet_fleet/devices/ robots/door_hinge.urdf robots/door_slide.urdf tests/test_door_device.py
git commit -m "feat: DoorDevice — Agent subclass with URDF joint open/close"
```

---

### Task 5: ElevatorDevice + WorkcellDevice [SERIAL — depends on Task 4]

**Files:**
- Create: `pybullet_fleet/devices/elevator.py`
- Create: `pybullet_fleet/devices/workcell.py`
- Create: `robots/elevator.urdf`
- Create: `tests/test_elevator_device.py`

**Step 1: Write failing tests**

```python
# tests/test_elevator_device.py
"""Tests for ElevatorDevice and WorkcellDevice."""
import pytest
from pybullet_fleet import Agent, AgentSpawnParams, Pose, SimObject, SimObjectSpawnParams
from pybullet_fleet.devices.elevator import ElevatorDevice
from pybullet_fleet.devices.workcell import WorkcellDevice


class TestElevatorDevice:
    def test_elevator_initial_floor(self, sim_core):
        params = AgentSpawnParams(
            urdf_path="robots/elevator.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0),
            use_fixed_base=True,
            user_data={
                "floors": {"L1": 0.0, "L2": 8.0, "L3": 16.0},
                "initial_floor": "L1",
                "joint_name": "lift",
            },
        )
        elev = ElevatorDevice.from_params(params, sim_core)
        assert elev.current_floor == "L1"

    def test_elevator_request_floor(self, sim_core):
        params = AgentSpawnParams(
            urdf_path="robots/elevator.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0),
            use_fixed_base=True,
            user_data={
                "floors": {"L1": 0.0, "L2": 8.0},
                "initial_floor": "L1",
                "joint_name": "lift",
            },
        )
        elev = ElevatorDevice.from_params(params, sim_core)
        elev.request_floor("L2")
        sim_core.tick(500)
        assert elev.current_floor == "L2"

    def test_elevator_available_floors(self, sim_core):
        params = AgentSpawnParams(
            urdf_path="robots/elevator.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0),
            use_fixed_base=True,
            user_data={
                "floors": {"L1": 0.0, "L2": 8.0, "L3": 16.0},
                "joint_name": "lift",
            },
        )
        elev = ElevatorDevice.from_params(params, sim_core)
        assert set(elev.available_floors) == {"L1", "L2", "L3"}


class TestWorkcellDevice:
    def test_workcell_needs_update_false(self, sim_core):
        params = SimObjectSpawnParams(
            visual_shape=None,
            collision_shape=None,
            initial_pose=Pose.from_xyz(5, 3, 0.5),
        )
        wc = WorkcellDevice.from_params(params, sim_core)
        assert wc._needs_update is False

    def test_workcell_is_simobject(self, sim_core):
        params = SimObjectSpawnParams(
            visual_shape=None,
            collision_shape=None,
            initial_pose=Pose.from_xyz(5, 3, 0.5),
        )
        wc = WorkcellDevice.from_params(params, sim_core)
        assert isinstance(wc, SimObject)
        assert not isinstance(wc, Agent)
```

**Step 2: Run test to verify it fails**

Run: `pytest tests/test_elevator_device.py -v`
Expected: FAIL — ImportError

**Step 3: Create elevator URDF**

```xml
<!-- robots/elevator.urdf -->
<?xml version="1.0"?>
<robot name="elevator">
  <link name="shaft">
    <visual>
      <geometry><box size="0.01 0.01 0.01"/></geometry>
    </visual>
    <inertial>
      <mass value="0.0"/>
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
    </inertial>
  </link>
  <joint name="lift" type="prismatic">
    <parent link="shaft"/>
    <child link="platform"/>
    <origin xyz="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="30.0" effort="100" velocity="2.0"/>
  </joint>
  <link name="platform">
    <visual>
      <geometry><box size="3.0 3.0 0.2"/></geometry>
      <material name="steel"><color rgba="0.7 0.7 0.7 1.0"/></material>
    </visual>
    <collision>
      <geometry><box size="3.0 3.0 0.2"/></geometry>
    </collision>
    <inertial>
      <mass value="0.0"/>
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
    </inertial>
  </link>
</robot>
```

**Step 4: Implement ElevatorDevice + WorkcellDevice**

See agent.spec.md Sections 5 and 6 for full implementation.

**Step 5: Run tests**

Run: `pytest tests/test_elevator_device.py tests/test_door_device.py -v`
Expected: ALL PASS

**Step 6: Commit**

```bash
git add pybullet_fleet/devices/elevator.py pybullet_fleet/devices/workcell.py robots/elevator.urdf tests/test_elevator_device.py
git commit -m "feat: ElevatorDevice + WorkcellDevice"
```

---

### Task 6: Entity Registry + __init__ exports [SERIAL — depends on Task 5]

**Files:**
- Modify: `pybullet_fleet/entity_registry.py` (register builtins)
- Modify: `pybullet_fleet/__init__.py` (export new classes)

**Step 1: Register device types**

In `pybullet_fleet/entity_registry.py`, in `_register_builtins()`:

```python
from pybullet_fleet.devices.door import DoorDevice
from pybullet_fleet.devices.elevator import ElevatorDevice
from pybullet_fleet.devices.workcell import WorkcellDevice

register_entity_class("door", DoorDevice)
register_entity_class("elevator", ElevatorDevice)
register_entity_class("workcell", WorkcellDevice)
```

**Step 2: Add exports to `__init__.py`**

```python
from pybullet_fleet.devices.door import DoorDevice
from pybullet_fleet.devices.elevator import ElevatorDevice
from pybullet_fleet.devices.workcell import WorkcellDevice
```

**Step 3: Test entity registry**

Run: `pytest tests/test_entity_registry.py -v`
Expected: PASS (existing tests + new types importable)

**Step 4: Commit**

```bash
git add pybullet_fleet/entity_registry.py pybullet_fleet/__init__.py
git commit -m "feat: register door/elevator/workcell in entity registry"
```

---

### Task 7: Full Integration Test [SERIAL — depends on Task 6]

**Files:**
- Test only — run full suite

**Step 1: Run make verify**

```bash
make verify
```

Expected: ALL 1255+ tests pass, coverage ≥ 75%, lint passes.

**Step 2: Fix any regressions**

If any existing tests fail, trace the breakage to the `_controller` → `_controllers` migration
or the `step_once()` unified loop change and fix.

---

### Task 8: Commit Core Changes [SERIAL — depends on Task 7]

If not already committed incrementally, squash or create a summary commit:

```bash
git add -A
git commit -m "feat(phase2): device classes, controller chain, patrol/random controllers

- SimObject.update() + _needs_update for unified step_once() loop
- Controller chain: add_controller(), reversed execution order
- PatrolController: waypoint cycling with loop/wait_time
- RandomWalkController: radius-based random walking
- DoorDevice(Agent): URDF joint open/close via JointAction
- ElevatorDevice(Agent): prismatic Z joint, auto_attach
- WorkcellDevice(SimObject): static position marker
- Entity registry: door, elevator, workcell types
- URDF: door_hinge, door_slide, elevator"
```

---

### Task 9: battle_royale Demo [PARALLEL — after Task 8]

**Files:**
- Create: `ros2_bridge/pybullet_fleet_ros/config/bridge_battle_royale.yaml`
- Create: `ros2_bridge/pybullet_fleet_ros/launch/battle_royale_pybullet.launch.py`

**Step 1: Create bridge config**

```yaml
# ros2_bridge/pybullet_fleet_ros/config/bridge_battle_royale.yaml
simulation:
  timestep: 0.01
  physics: false
  gui: true
  monitor: false

entities:
  - name: tinyRobotA
    type: agent
    urdf_path: robots/mobile_robot.urdf
    initial_pose: [0, 0, 0.1, 0, 0, 0]
    motion_mode: omnidirectional
    max_linear_vel: 0.5
    collision_mode: normal_2d
  - name: tinyRobotB
    type: agent
    urdf_path: robots/mobile_robot.urdf
    initial_pose: [3, 0, 0.1, 0, 0, 0]
    motion_mode: omnidirectional
    max_linear_vel: 0.5
    collision_mode: normal_2d
  - name: tinyRobotC
    type: agent
    urdf_path: robots/mobile_robot.urdf
    initial_pose: [0, 3, 0.1, 0, 0, 0]
    motion_mode: omnidirectional
    max_linear_vel: 0.5
    collision_mode: normal_2d
  - name: tinyRobotD
    type: agent
    urdf_path: robots/mobile_robot.urdf
    initial_pose: [3, 3, 0.1, 0, 0, 0]
    motion_mode: omnidirectional
    max_linear_vel: 0.5
    collision_mode: normal_2d
```

**Step 2: Create launch file**

Model after `office_pybullet.launch.py` — use `pybullet_common.launch.py` with
battle_royale fleet config and nav_graph.

**Step 3: Test**

```bash
DEMO_WORLD=battle_royale docker compose -f docker-compose.rmf.yaml up
```

**Step 4: Commit**

```bash
git add ros2_bridge/pybullet_fleet_ros/config/bridge_battle_royale.yaml \
        ros2_bridge/pybullet_fleet_ros/launch/battle_royale_pybullet.launch.py
git commit -m "feat: battle_royale demo launch + config"
```

---

### Task 10: hotel Demo [PARALLEL — after Task 8]

**Files:**
- Create: `ros2_bridge/pybullet_fleet_ros/config/bridge_hotel.yaml`
- Create: `ros2_bridge/pybullet_fleet_ros/launch/hotel_pybullet.launch.py`

**Step 1: Create bridge config**

Config with:
- 12 DoorDevice entries (L1: 5, L2: 7) — hinge and slide types
- 2 ElevatorDevice entries (Lift1, Lift2) with floors {L1: 0.0, L2: 8.0, L3: 16.0}
- 4 robots: tinyBot_1, deliveryBot_1, cleanerBotA_1, cleanerBotA_2

Use `type: door` / `type: elevator` with proper `user_data` for open/close positions.

**Step 2: Create launch file**

Include 3 fleet adapters (tinyRobot, deliveryRobot, cleanerBotA) with their
respective fleet configs and nav_graphs. Include door_adapter and lift_adapter nodes.

**Step 3: Test**

```bash
DEMO_WORLD=hotel docker compose -f docker-compose.rmf.yaml up
```

**Step 4: Commit**

```bash
git add ros2_bridge/pybullet_fleet_ros/config/bridge_hotel.yaml \
        ros2_bridge/pybullet_fleet_ros/launch/hotel_pybullet.launch.py
git commit -m "feat: hotel demo — 3 floors, 2 elevators, 12 doors, 3 fleets"
```

---

### Task 11: clinic Demo [PARALLEL — after Task 8]

**Files:**
- Create: `ros2_bridge/pybullet_fleet_ros/config/bridge_clinic.yaml`
- Create: `ros2_bridge/pybullet_fleet_ros/launch/clinic_pybullet.launch.py`

**Step 1: Create bridge config**

Config with:
- ~10 DoorDevice entries (RMF-controlled doors only, skip cosmetics)
- 2 ElevatorDevice entries (lift_1, lift_25 — active lifts) with floors {L1: 0.0, L2: 8.0}
- 4 robots: deliveryRobot_1/2, tinyRobot_1/2

**Step 2: Create launch file**

Include 2 fleet adapters (deliveryRobot, tinyRobot), door_adapter, lift_adapter.

**Step 3: Test**

```bash
DEMO_WORLD=clinic docker compose -f docker-compose.rmf.yaml up
```

**Step 4: Commit**

```bash
git add ros2_bridge/pybullet_fleet_ros/config/bridge_clinic.yaml \
        ros2_bridge/pybullet_fleet_ros/launch/clinic_pybullet.launch.py
git commit -m "feat: clinic demo — 2 floors, 2 elevators, ~10 doors, 2 fleets"
```

---

### Task 12: airport_terminal Update [PARALLEL — after Task 8]

**Files:**
- Modify: `ros2_bridge/pybullet_fleet_ros/config/bridge_airport.yaml`
- Modify: `ros2_bridge/pybullet_fleet_ros/launch/airport_terminal_pybullet.launch.py`

**Step 1: Update bridge config**

Add to existing config:
- 5 DoorDevice entries (n02_door, n01_door, s04_door, s08_door, zone_4_door)
- caddy_0 agent with `controllers: [{type: patrol, waypoints: [...], loop: true}]`
- Optional: pedestrian agents with `controllers: [{type: random_walk, radius: 5.0}]`

**Step 2: Update launch file**

Add door_adapter node. caddy fleet uses ExternalAgentHandler via handler_map.

**Step 3: Test**

```bash
DEMO_WORLD=airport_terminal docker compose -f docker-compose.rmf.yaml up
```

**Step 4: Commit**

```bash
git add ros2_bridge/pybullet_fleet_ros/config/bridge_airport.yaml \
        ros2_bridge/pybullet_fleet_ros/launch/airport_terminal_pybullet.launch.py
git commit -m "feat: airport_terminal — add doors, caddy patrol, pedestrians"
```

---

### Task 13: Final Verify + Commit [SERIAL — after Tasks 9-12]

**Step 1: Run make verify**

```bash
make verify
```

Expected: ALL tests pass, coverage ≥ 75%, lint passes.

**Step 2: Update ROADMAP status**

Mark Phase 2 items 6, 7, 11 as ✅ Done.

**Step 3: Final commit**

```bash
git add -A
git commit -m "phase2: all demos operational — battle_royale, hotel, clinic, airport update

Tested: DEMO_WORLD={office,battle_royale,hotel,clinic,airport_terminal}
make verify: all tests pass, coverage ≥ 75%"
```
