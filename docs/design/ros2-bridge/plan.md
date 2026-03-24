# ROS 2 Bridge Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use executing-plans to implement this plan task-by-task.

**Goal:** Build a ROS 2 bridge node that wraps PyBulletFleet's `MultiRobotSimulationCore` and exposes per-robot standard ROS 2 topics/actions plus `simulation_interfaces` services, all running in Docker.

**Architecture:** A single `BridgeNode` drives simulation via a ROS-aware `run_simulation()` override (subclass of `MultiRobotSimulationCore`) and delegates per-robot I/O to `RobotHandler` instances. `SimServices` implements `simulation_interfaces`. `/clock` is published every step. The bridge runs in Docker (Jazzy primary) while PyBulletFleet core remains ROS-independent. Velocity commands (`cmd_vel`) flow through the **OmniOmniVelocityController** (Controller ABC from the Plugin Architecture) rather than a direct `set_velocity()` method on Agent.

**Tech Stack:** Python 3.10+, rclpy, nav2_msgs, control_msgs, tf2_ros, simulation_interfaces (source build — not available via apt), Docker, docker compose

---

## Summary of Changes from v1

This plan incorporates 9 feedback points from the user:

1. **Task 0: Branch creation** — `git checkout -b feat/ros2-bridge` before any work
2. **OmniOmniVelocityController for cmd_vel** — Uses `OmniOmniVelocityController` (Controller ABC from Plugin Architecture) instead of adding `set_velocity()` directly to Agent. Each ROS-controlled agent gets `agent.set_controller(OmniOmniVelocityController())` at spawn time.
3. **Z-axis velocity support** — `OmniOmniVelocityController.set_velocity()` accepts (vx, vy, vz, wz); supports 3D movement
4. **`run_simulation()` override** — Instead of calling `step_once()` from a raw ROS timer, we subclass `MultiRobotSimulationCore` to override `run_simulation()` with a ROS-aware loop that preserves RTF control
5. **simulation_interfaces: source build confirmed** — Not available as apt package in Jazzy or Humble; Docker must build from source
6. **Author email fixed** — `yuokamoto1988@gmail.com`
7. **RobotHandler naming** — confirmed (was already correct)
8. **Task 9: separate ROS client scripts** — standalone Python scripts that interact with the bridge via ROS topics/services/actions
9. **Deferred tasks** — Nav2/arm action servers, Humble Dockerfile, AgentManagerWrapper moved to Phase 2

---

## Critical Design Notes

### Velocity Control via OmniOmniVelocityController (Plugin Architecture Phase 2)

**Current state:** Agent has NO `set_velocity()` method. `velocity` property is read-only. Movement is entirely goal-based via `set_goal_pose()` / `set_path()`. Internal velocity comes from TPI trajectory interpolation.

**Approach:** Instead of adding `set_velocity()` directly to Agent, we use the **OmniOmniVelocityController** from the [Plugin Architecture](../plugin-architecture/agent.spec.md). This is the Controller ABC strategy pattern where Agent "has-a" Controller. When a Controller is set, `Agent.update()` delegates to `controller.compute(agent, dt)` instead of the legacy TPI logic.

**Data flow:** `cmd_vel` → `RobotHandler.apply_cmd_vel()` → `OmniOmniVelocityController.set_velocity(vx, vy, vz, wz)` → (next sim step) → `agent.update(dt)` → `controller.compute(agent, dt)` → Euler integration → `agent.set_pose()`

**Key benefit:** Agent's public API remains clean. Velocity control is a strategy, not a mode flag embedded in Agent core.

```python
# In pybullet_fleet/controller.py (from Plugin Architecture Phase 2)
class OmniOmniVelocityController(Controller):
    """Explicit velocity command control (for cmd_vel / ROS bridge)."""

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

        new_pose = Pose.from_yaw(
            pose.x + dx, pose.y + dy, pose.z + dz, pose.yaw + dyaw
        )
        agent.set_pose(new_pose)
        agent._current_velocity[:] = self._velocity_command
        agent._current_angular_velocity = self._angular_velocity_command
        return True
```

`Agent.update()` dispatches to Controller when set:

```python
# In Agent.update() (modified by Plugin Architecture Phase 2)
moved = False
if not self.use_fixed_base:
    if self._controller is not None:
        # Controller mode (new): delegate to Controller.compute()
        moved = self._controller.compute(self, dt)
    elif self._is_moving and self._goal_pose is not None:
        # Legacy goal-based mode (unchanged)
        if self._is_final_orientation_aligning:
            self._update_differential(dt)
        elif self._motion_mode == MotionMode.OMNIDIRECTIONAL:
            self._update_omnidirectional(dt)
        elif self._motion_mode == MotionMode.DIFFERENTIAL:
            self._update_differential(dt)
        moved = True
```

**Dependency:** This requires Plugin Architecture Phase 2 (Controller ABC + OmniOmniVelocityController) to be implemented first. Task 1 of this plan implements that prerequisite.

### run_simulation() Override for ROS

**Current:** `run_simulation()` is a blocking loop with RTF sync using `time.sleep()`. The ROS bridge needs to integrate `rclpy.spin` with the simulation loop.

**Approach:** Subclass `MultiRobotSimulationCore` in the ROS bridge to replace `run_simulation()` with a ROS-aware version. The ROS timer controls timing and RTF, but the subclass preserves the sim core's internal loop structure.

```python
# In bridge_node.py
class ROSSimulationCore(MultiRobotSimulationCore):
    """Simulation core with ROS-aware run_simulation() override."""

    def run_simulation(self, duration: Optional[float] = None) -> None:
        """Override: use ROS timer-based loop instead of blocking sleep loop.

        RTF is controlled by the ROS timer period + step execution time.
        Use target_rtf parameter to adjust speed.
        """
        self.initialize_simulation()
        # The actual stepping is driven by the ROS timer in BridgeNode.
        # This method returns immediately; rclpy.spin() drives the loop.
```

In `BridgeNode`:

```python
class BridgeNode(Node):
    def __init__(self):
        super().__init__("pybullet_fleet_bridge")
        self.sim = ROSSimulationCore(SimulationParams(...))
        self.sim.initialize_simulation()

        dt = self.sim.params.timestep
        # Timer period controls RTF: period = dt / target_rtf
        target_rtf = self.sim.params.target_rtf or 1.0
        timer_period = dt / target_rtf
        self._step_timer = self.create_timer(timer_period, self._step_callback)

    def _step_callback(self):
        for handler in self._handlers.values():
            handler.apply_cmd_vel(self.sim.params.timestep)
        self.sim.step_once()
        # publish /clock, states, etc.
```

### ROS Package Structure

All ROS code lives under `ros2_bridge/` and is built with `colcon`. It is NOT part of the pip package.

```
ros2_bridge/
├── pybullet_fleet_ros/          # Main ROS package (ament_python)
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   ├── resource/pybullet_fleet_ros
│   ├── pybullet_fleet_ros/
│   │   ├── __init__.py
│   │   ├── bridge_node.py       # BridgeNode + ROSSimulationCore
│   │   ├── robot_handler.py     # Per-robot topic management
│   │   ├── sim_services.py      # simulation_interfaces
│   │   └── conversions.py       # Pose/msg conversion utilities
│   ├── launch/
│   │   ├── bridge.launch.py     # Core bridge launch
│   │   └── multi_robot.launch.py
│   ├── config/
│   │   └── default_bridge.yaml
│   └── test/
│       ├── conftest.py
│       ├── test_conversions.py
│       ├── test_bridge_node.py
│       ├── test_robot_handler.py
│       └── test_sim_services.py
├── scripts/                      # Standalone ROS client scripts
│   ├── teleop_cmd_vel.py         # Send cmd_vel to a robot
│   ├── send_nav_goal.py          # Send NavigateToPose goal
│   ├── spawn_robots.py           # Spawn robots via SpawnEntity service
│   ├── query_entities.py         # Query GetEntities / GetEntityState
│   └── step_simulation.py        # Step simulation via StepSimulation service
└── docker/
    ├── Dockerfile.jazzy
    ├── docker-compose.yaml
    └── ros_entrypoint.sh
```

---

## Task Dependency Graph

```
Task 0 (Branch) ────────────────────────────────────┐
Task 1 (Controller ABC + OmniOmniVelocityController) ───────┤
                                                     │
Task 2 (Docker) ─────────────────────────────┐      │
Task 3 (Package skeleton) ───────────────────┤      │
Task 4 (Conversions) ───────────────────────┤      │
                                              ▼      ▼
                                    Task 5 (RobotHandler, depends on 1,3,4)
                                              │
                                              ▼
                                    Task 6 (BridgeNode, depends on 2,5)
                                              │
                                              ▼
                                    Task 7 (SimServices, depends on 6)
                                              │
                                              ▼
                                    Task 8 (Integration test, depends on 7)
                                              │
                                              ▼
                                    Task 9 (Client scripts, depends on 8)
```

- **PARALLEL:** Tasks 2, 3, 4 (no dependencies on each other)
- **Task 1 (Controller ABC + OmniOmniVelocityController):** Core prerequisite from Plugin Architecture Phase 2 — other tasks depend on it
- **SERIAL:** Tasks 5→6→7→8→9

### Deferred to Phase 2

- Nav2 action servers (`NavigateToPose`, `FollowPath`) in RobotHandler
- `FollowJointTrajectory` action server
- Humble Dockerfile + CI
- AgentManagerROSWrapper (fleet batch operations)
- Custom message types

---

## Task 0: Create Feature Branch (SERIAL, first)

**Step 1: Create branch**

```bash
git checkout -b feat/ros2-bridge
```

**Step 2: Verify**

```bash
git branch --show-current
```
Expected: `feat/ros2-bridge`

---

## Task 1: Controller ABC + OmniOmniVelocityController (SERIAL, depends on Task 0)

**Files:**
- Create: `pybullet_fleet/controller.py`
- Modify: `pybullet_fleet/agent.py` (add `_controller` attribute, `set_controller()`, modify `update()` dispatch)
- Modify: `pybullet_fleet/__init__.py` (export Controller, OmniOmniVelocityController, TPIController)
- Create: `tests/test_controller.py`

This task implements Plugin Architecture Phase 2 — the Controller ABC and built-in controllers. This is a prerequisite for the ROS bridge's `cmd_vel` handling. Agent currently only supports goal-based movement (TPI trajectories). The Controller ABC adds a Strategy pattern where Agent delegates movement to its controller.

**Step 1: Write the failing tests**

```python
# tests/test_controller.py
"""Tests for Controller ABC, OmniOmniVelocityController, and Agent.set_controller()."""

import math
import pytest
from pybullet_fleet import Agent, AgentSpawnParams, Pose
from pybullet_fleet.types import MotionMode


@pytest.fixture
def mobile_agent(sim_core_headless):
    """Spawn a simple mobile agent."""
    params = AgentSpawnParams(
        urdf_path="robots/mobile_robot.urdf",
        initial_pose=Pose.from_xyz(0, 0, 0.05),
        motion_mode=MotionMode.OMNIDIRECTIONAL,
        max_linear_vel=2.0,
    )
    return Agent.from_params(params, sim_core_headless)


class TestOmniOmniVelocityController:
    """OmniOmniVelocityController moves agent at commanded velocity."""

    def test_velocity_moves_robot(self, mobile_agent):
        from pybullet_fleet.controller import OmniOmniVelocityController

        ctrl = OmniOmniVelocityController()
        mobile_agent.set_controller(ctrl)
        ctrl.set_velocity(vx=1.0, vy=0.0)

        initial_pose = mobile_agent.get_pose()
        dt = 0.1
        mobile_agent.update(dt)

        new_pose = mobile_agent.get_pose()
        assert new_pose.x > initial_pose.x
        assert new_pose.x == pytest.approx(initial_pose.x + 1.0 * dt, abs=0.01)

    def test_velocity_with_angular(self, mobile_agent):
        from pybullet_fleet.controller import OmniOmniVelocityController

        ctrl = OmniOmniVelocityController()
        mobile_agent.set_controller(ctrl)
        ctrl.set_velocity(vx=0.0, vy=0.0, wz=1.0)  # 1 rad/s

        initial_yaw = mobile_agent.get_pose().yaw
        dt = 0.1
        mobile_agent.update(dt)

        new_yaw = mobile_agent.get_pose().yaw
        assert abs(new_yaw - initial_yaw) > 0.05

    def test_velocity_with_z(self, mobile_agent):
        from pybullet_fleet.controller import OmniOmniVelocityController

        ctrl = OmniOmniVelocityController()
        mobile_agent.set_controller(ctrl)
        ctrl.set_velocity(vx=0.0, vy=0.0, vz=0.5)

        initial_z = mobile_agent.get_pose().z
        dt = 0.1
        mobile_agent.update(dt)

        assert mobile_agent.get_pose().z > initial_z

    def test_zero_velocity_returns_false(self, mobile_agent):
        from pybullet_fleet.controller import OmniOmniVelocityController

        ctrl = OmniOmniVelocityController()
        mobile_agent.set_controller(ctrl)
        # Default is zero velocity → compute returns False
        result = ctrl.compute(mobile_agent, 0.1)
        assert result is False

    def test_stop_clears_velocity(self, mobile_agent):
        from pybullet_fleet.controller import OmniOmniVelocityController

        ctrl = OmniOmniVelocityController()
        mobile_agent.set_controller(ctrl)
        ctrl.set_velocity(vx=1.0, vy=0.0)

        mobile_agent.stop()
        # on_stop should zero the velocity
        result = ctrl.compute(mobile_agent, 0.1)
        assert result is False


class TestSetController:
    """Agent.set_controller() switches control strategy."""

    def test_set_controller_enables_controller_mode(self, mobile_agent):
        from pybullet_fleet.controller import OmniOmniVelocityController

        ctrl = OmniOmniVelocityController()
        mobile_agent.set_controller(ctrl)
        assert mobile_agent._controller is ctrl

    def test_set_controller_none_restores_legacy(self, mobile_agent):
        from pybullet_fleet.controller import OmniOmniVelocityController

        ctrl = OmniOmniVelocityController()
        mobile_agent.set_controller(ctrl)
        mobile_agent.set_controller(None)
        assert mobile_agent._controller is None

    def test_goal_based_still_works_without_controller(self, mobile_agent):
        """Legacy goal-based mode works when no controller is set."""
        mobile_agent.set_goal_pose(Pose.from_xyz(5, 0, 0.05))
        initial_x = mobile_agent.get_pose().x

        dt = 0.1
        mobile_agent.update(dt)

        # Should move toward goal
        assert mobile_agent.get_pose().x > initial_x
```

Note: `sim_core_headless` fixture from conftest provides `MultiRobotSimulationCore(SimulationParams(gui=False, monitor=False))`.

**Step 2: Run tests to verify they fail**

Run: `pytest tests/test_controller.py -v`
Expected: FAIL — `pybullet_fleet.controller` module not found

**Step 3: Create controller.py**

```python
# pybullet_fleet/controller.py
"""Controller ABC + built-in controllers (Plugin Architecture Phase 2).

Controller is a Strategy pattern for Agent movement. Agent "has-a" Controller.
When controller is set, Agent.update() delegates to controller.compute().
When controller is None, legacy TPI logic runs (backward compatible).

Built-in controllers:
- OmniOmniVelocityController: explicit velocity commands (for ROS cmd_vel)
- TPIController: existing TPI trajectory logic (for goal-based movement)
"""

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

import numpy as np

from pybullet_fleet.logging_utils import get_lazy_logger

if TYPE_CHECKING:
    from pybullet_fleet.agent import Agent
    from pybullet_fleet.geometry import Pose

logger = get_lazy_logger(__name__)


class Controller(ABC):
    """Agent movement control interface (Strategy pattern).

    Usage:
        agent.set_controller(OmniOmniVelocityController())
        agent.set_controller(None)  # revert to legacy mode
    """

    @abstractmethod
    def compute(self, agent: "Agent", dt: float) -> bool:
        """Compute one step of control.

        Args:
            agent: The agent to control
            dt: Time step in seconds

        Returns:
            True if the agent moved, False otherwise
        """
        ...

    def on_goal_set(self, agent: "Agent", goal: "Pose") -> None:
        """Called when set_goal_pose() is invoked."""
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

        from pybullet_fleet.geometry import Pose as PoseClass
        new_pose = PoseClass.from_yaw(
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

**Step 4: Modify Agent to support Controller**

Add to `pybullet_fleet/agent.py`:

1. Add `_controller` attribute in `__init__` (after `_current_velocity` init):

```python
        self._controller: Optional["Controller"] = None  # Plugin Architecture Phase 2
```

2. Add `set_controller()` method:

```python
    def set_controller(self, controller: Optional["Controller"] = None) -> None:
        """Set or clear the movement controller (Strategy pattern).

        When a controller is set, update() delegates to controller.compute().
        When controller is None, legacy TPI goal-based logic is used.

        Args:
            controller: A Controller instance, or None to revert to legacy mode.
        """
        self._controller = controller
```

3. Modify `update()` dispatch (around line ~1750):

```python
    moved = False
    if not self.use_fixed_base:
        if self._controller is not None:
            # Controller mode: delegate to controller.compute()
            moved = self._controller.compute(self, dt)
        elif self._is_moving and self._goal_pose is not None:
            # Legacy goal-based mode (existing code, unchanged)
            if self._is_final_orientation_aligning:
                self._update_differential(dt)
            elif self._motion_mode == MotionMode.OMNIDIRECTIONAL:
                self._update_omnidirectional(dt)
            elif self._motion_mode == MotionMode.DIFFERENTIAL:
                self._update_differential(dt)
            else:
                self._log.warning(f"Unknown motion mode: {self._motion_mode}")
            moved = True
```

4. Modify `stop()` to notify controller:

```python
    def stop(self):
        # ... existing stop logic ...
        if self._controller is not None:
            self._controller.on_stop(self)
```

**Step 5: Run tests to verify they pass**

Run: `pytest tests/test_controller.py -v`
Expected: All tests PASS

Run: `pytest tests/ -v --tb=short`
Expected: All existing tests still pass (no regressions)

**Step 6: Run make verify**

Run: `make verify`
Expected: Lint + tests pass

**Step 7: Commit**

```bash
git add pybullet_fleet/controller.py tests/test_controller.py pybullet_fleet/agent.py
git commit -m "feat: add Controller ABC + OmniOmniVelocityController (Plugin Architecture Phase 2)"
```

---

## Task 2: Docker Environment (PARALLEL with Tasks 3, 4)

**Files:**
- Create: `docker/Dockerfile.jazzy`
- Create: `docker/docker-compose.yaml`
- Create: `docker/ros_entrypoint.sh`
- Create: `docker/.dockerignore`

**Step 1: Create Dockerfile.jazzy**

```dockerfile
# docker/Dockerfile.jazzy
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Install ROS 2 dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-nav2-msgs \
    ros-jazzy-control-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# Build simulation_interfaces from source (NOT available via apt)
WORKDIR /opt/sim_interfaces_ws/src
RUN git clone --depth 1 https://github.com/ros-simulation/simulation_interfaces.git
WORKDIR /opt/sim_interfaces_ws
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --packages-select simulation_interfaces --cmake-args -DCMAKE_BUILD_TYPE=Release

# Install pybullet_fleet (editable for development)
COPY requirements.txt /opt/pybullet_fleet/requirements.txt
COPY pyproject.toml /opt/pybullet_fleet/pyproject.toml
COPY pybullet_fleet/ /opt/pybullet_fleet/pybullet_fleet/
COPY robots/ /opt/pybullet_fleet/robots/
COPY config/ /opt/pybullet_fleet/config/
COPY mesh/ /opt/pybullet_fleet/mesh/
WORKDIR /opt/pybullet_fleet
RUN pip install --break-system-packages -e .

# Build ROS 2 bridge
WORKDIR /opt/bridge_ws/src
COPY ros2_bridge/pybullet_fleet_ros /opt/bridge_ws/src/pybullet_fleet_ros
WORKDIR /opt/bridge_ws
RUN source /opt/ros/jazzy/setup.bash && \
    source /opt/sim_interfaces_ws/install/setup.bash && \
    colcon build --symlink-install

# Entrypoint
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
```

**Step 2: Create ros_entrypoint.sh**

```bash
#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /opt/sim_interfaces_ws/install/setup.bash
source /opt/bridge_ws/install/setup.bash
exec "$@"
```

Save as `docker/ros_entrypoint.sh`.

**Step 3: Create docker-compose.yaml**

```yaml
# docker/docker-compose.yaml
services:
  bridge:
    build:
      context: ..
      dockerfile: docker/Dockerfile.jazzy
    command: >
      ros2 launch pybullet_fleet_ros bridge.launch.py
    network_mode: host
    environment:
      - DISPLAY=${DISPLAY:-}
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      # Dev mode: mount source for live editing
      - ../pybullet_fleet:/opt/pybullet_fleet/pybullet_fleet:ro
      - ../ros2_bridge/pybullet_fleet_ros:/opt/bridge_ws/src/pybullet_fleet_ros:ro
      - ../robots:/opt/pybullet_fleet/robots:ro
      - ../config:/opt/pybullet_fleet/config:ro
    stdin_open: true
    tty: true

  test:
    build:
      context: ..
      dockerfile: docker/Dockerfile.jazzy
    command: bash /opt/pybullet_fleet/docker/test_integration.sh
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-99}
```

**Step 4: Create .dockerignore**

```
# docker/.dockerignore
__pycache__
*.pyc
.git
.github
*.egg-info
build/
dist/
results/
benchmark/
docs/
tests/
```

**Step 5: Verify Docker builds**

Run: `cd docker && docker compose build bridge`
Expected: Image builds successfully (may take 5-10 minutes first time)

**Step 6: Commit**

```bash
git add docker/
git commit -m "feat(ros2): add Docker environment for Jazzy with simulation_interfaces source build"
```

---

## Task 3: ROS Package Skeleton (PARALLEL with Tasks 2, 4)

**Files:**
- Create: `ros2_bridge/pybullet_fleet_ros/package.xml`
- Create: `ros2_bridge/pybullet_fleet_ros/setup.py`
- Create: `ros2_bridge/pybullet_fleet_ros/setup.cfg`
- Create: `ros2_bridge/pybullet_fleet_ros/resource/pybullet_fleet_ros` (empty marker)
- Create: `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/__init__.py`
- Create: `ros2_bridge/pybullet_fleet_ros/config/default_bridge.yaml`

**Step 1: Create package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>pybullet_fleet_ros</name>
  <version>0.1.0</version>
  <description>ROS 2 bridge for PyBulletFleet multi-robot simulation</description>
  <maintainer email="yuokamoto1988@gmail.com">Yu Okamoto</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav2_msgs</depend>
  <depend>control_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>rosgraph_msgs</depend>
  <depend>simulation_interfaces</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**Step 2: Create setup.py**

```python
from setuptools import setup

package_name = 'pybullet_fleet_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/bridge.launch.py',
            'launch/multi_robot.launch.py',
        ]),
        ('share/' + package_name + '/config', ['config/default_bridge.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yu Okamoto',
    maintainer_email='yuokamoto1988@gmail.com',
    description='ROS 2 bridge for PyBulletFleet multi-robot simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = pybullet_fleet_ros.bridge_node:main',
        ],
    },
)
```

**Step 3: Create setup.cfg**

```ini
[develop]
script_dir=$base/lib/pybullet_fleet_ros
[install]
install_scripts=$base/lib/pybullet_fleet_ros
```

**Step 4: Create resource marker and __init__.py**

```bash
mkdir -p ros2_bridge/pybullet_fleet_ros/resource
touch ros2_bridge/pybullet_fleet_ros/resource/pybullet_fleet_ros
```

```python
# ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/__init__.py
"""PyBulletFleet ROS 2 Bridge — connects PyBulletFleet simulation to ROS 2."""
```

**Step 5: Create default_bridge.yaml**

```yaml
# ros2_bridge/pybullet_fleet_ros/config/default_bridge.yaml
pybullet_fleet_bridge:
  ros__parameters:
    config_yaml: ""
    num_robots: 1
    robot_urdf: "robots/mobile_robot.urdf"
    publish_rate: 50.0
    gui: false
    physics: false
    target_rtf: 1.0
    enable_sim_services: true
```

**Step 6: Commit**

```bash
git add ros2_bridge/
git commit -m "feat(ros2): add pybullet_fleet_ros package skeleton"
```

---

## Task 4: Pose/Message Conversion Utilities (PARALLEL with Tasks 2, 3)

**Files:**
- Create: `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/conversions.py`
- Create: `ros2_bridge/pybullet_fleet_ros/test/test_conversions.py`

**Step 1: Write the failing test**

```python
# ros2_bridge/pybullet_fleet_ros/test/test_conversions.py
"""Tests for ROS ↔ PyBulletFleet type conversions."""

import math
import pytest


def test_pbf_pose_to_ros_pose():
    """PyBulletFleet Pose → geometry_msgs/Pose round-trip."""
    from pybullet_fleet.geometry import Pose
    from pybullet_fleet_ros.conversions import pbf_pose_to_ros, ros_pose_to_pbf

    pbf = Pose.from_xyz(1.0, 2.0, 3.0)
    ros_msg = pbf_pose_to_ros(pbf)
    assert ros_msg.position.x == pytest.approx(1.0)
    assert ros_msg.position.y == pytest.approx(2.0)
    assert ros_msg.position.z == pytest.approx(3.0)
    assert ros_msg.orientation.w == pytest.approx(1.0)

    back = ros_pose_to_pbf(ros_msg)
    assert back.x == pytest.approx(1.0)
    assert back.y == pytest.approx(2.0)
    assert back.z == pytest.approx(3.0)


def test_ros_pose_to_pbf_with_orientation():
    """geometry_msgs/Pose with non-identity orientation."""
    from geometry_msgs.msg import Pose as PoseMsg, Quaternion, Point
    from pybullet_fleet_ros.conversions import ros_pose_to_pbf

    msg = PoseMsg()
    msg.position = Point(x=5.0, y=6.0, z=0.0)
    msg.orientation = Quaternion(x=0.0, y=0.0, z=0.7071068, w=0.7071068)

    pbf = ros_pose_to_pbf(msg)
    assert pbf.x == pytest.approx(5.0)
    assert pbf.yaw == pytest.approx(math.pi / 2, abs=0.01)


def test_pbf_pose_to_odom():
    """PyBulletFleet Pose + velocity → nav_msgs/Odometry."""
    from pybullet_fleet.geometry import Pose
    from pybullet_fleet_ros.conversions import make_odom_msg

    pose = Pose.from_xyz(1.0, 2.0, 0.0)
    velocity = [0.5, 0.0, 0.0]
    angular_vel = 0.1

    msg = make_odom_msg(pose, velocity, angular_vel, "odom", "robot_0/base_link")
    assert msg.pose.pose.position.x == pytest.approx(1.0)
    assert msg.twist.twist.linear.x == pytest.approx(0.5)
    assert msg.twist.twist.angular.z == pytest.approx(0.1)
    assert msg.header.frame_id == "odom"
    assert msg.child_frame_id == "robot_0/base_link"


def test_joint_state_msg():
    """Agent joint states → sensor_msgs/JointState."""
    from pybullet_fleet_ros.conversions import make_joint_state_msg

    joint_names = ["joint_0", "joint_1", "joint_2"]
    positions = [0.1, 0.2, 0.3]
    velocities = [0.01, 0.02, 0.03]

    msg = make_joint_state_msg(joint_names, positions, velocities)
    assert list(msg.name) == joint_names
    assert list(msg.position) == pytest.approx(positions)
    assert list(msg.velocity) == pytest.approx(velocities)


def test_twist_to_velocity_world_frame():
    """Twist (body frame) → world-frame velocity conversion."""
    from geometry_msgs.msg import Twist
    from pybullet_fleet_ros.conversions import twist_to_world_velocity

    twist = Twist()
    twist.linear.x = 1.0
    twist.linear.y = 0.5
    twist.linear.z = 0.0
    twist.angular.z = 0.2

    # Robot facing +X (yaw=0)
    vx, vy, vz, wz = twist_to_world_velocity(twist, current_yaw=0.0)
    assert vx == pytest.approx(1.0)
    assert vy == pytest.approx(0.5)
    assert vz == pytest.approx(0.0)
    assert wz == pytest.approx(0.2)


def test_twist_to_velocity_rotated():
    """Twist → world velocity when robot is rotated 90 degrees."""
    from geometry_msgs.msg import Twist
    from pybullet_fleet_ros.conversions import twist_to_world_velocity

    twist = Twist()
    twist.linear.x = 1.0  # forward
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.z = 0.0

    # Robot facing +Y (yaw=90°)
    vx, vy, vz, wz = twist_to_world_velocity(twist, current_yaw=math.pi / 2)
    assert vx == pytest.approx(0.0, abs=1e-6)  # cos(90°) * 1.0 ≈ 0
    assert vy == pytest.approx(1.0, abs=1e-6)  # sin(90°) * 1.0 ≈ 1.0
    assert vz == pytest.approx(0.0)


def test_twist_to_velocity_with_z():
    """Twist with linear.z is passed through to world frame."""
    from geometry_msgs.msg import Twist
    from pybullet_fleet_ros.conversions import twist_to_world_velocity

    twist = Twist()
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.5  # vertical velocity
    twist.angular.z = 0.0

    vx, vy, vz, wz = twist_to_world_velocity(twist, current_yaw=0.0)
    assert vz == pytest.approx(0.5)


def test_sim_time_to_ros_time():
    """Simulation time float → builtin_interfaces/Time."""
    from pybullet_fleet_ros.conversions import sim_time_to_ros_time

    stamp = sim_time_to_ros_time(1.5)
    assert stamp.sec == 1
    assert stamp.nanosec == 500000000
```

**Step 2: Run test to verify it fails**

Run (inside Docker): `cd /opt/bridge_ws && colcon test --packages-select pybullet_fleet_ros`
Expected: FAIL — `pybullet_fleet_ros.conversions` not found

**Step 3: Write conversions.py implementation**

```python
# ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/conversions.py
"""Conversion utilities between PyBulletFleet types and ROS 2 messages."""

import math
from typing import List, Optional, Tuple

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import (
    Point,
    Pose as PoseMsg,
    PoseStamped,
    Quaternion,
    Transform,
    TransformStamped,
    Twist,
    Vector3,
)
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from pybullet_fleet.geometry import Path, Pose


def pbf_pose_to_ros(pbf_pose: Pose) -> PoseMsg:
    """Convert PyBulletFleet Pose → geometry_msgs/Pose."""
    msg = PoseMsg()
    msg.position = Point(
        x=float(pbf_pose.position[0]),
        y=float(pbf_pose.position[1]),
        z=float(pbf_pose.position[2]),
    )
    msg.orientation = Quaternion(
        x=float(pbf_pose.orientation[0]),
        y=float(pbf_pose.orientation[1]),
        z=float(pbf_pose.orientation[2]),
        w=float(pbf_pose.orientation[3]),
    )
    return msg


def ros_pose_to_pbf(ros_pose: PoseMsg) -> Pose:
    """Convert geometry_msgs/Pose → PyBulletFleet Pose."""
    return Pose(
        position=[ros_pose.position.x, ros_pose.position.y, ros_pose.position.z],
        orientation=[
            ros_pose.orientation.x,
            ros_pose.orientation.y,
            ros_pose.orientation.z,
            ros_pose.orientation.w,
        ],
    )


def make_odom_msg(
    pose: Pose,
    velocity: List[float],
    angular_velocity: float,
    frame_id: str,
    child_frame_id: str,
    stamp: Optional[TimeMsg] = None,
) -> Odometry:
    """Create nav_msgs/Odometry from PyBulletFleet state."""
    msg = Odometry()
    msg.header = Header(frame_id=frame_id)
    if stamp:
        msg.header.stamp = stamp
    msg.child_frame_id = child_frame_id
    msg.pose.pose = pbf_pose_to_ros(pose)
    msg.twist.twist.linear = Vector3(
        x=float(velocity[0]), y=float(velocity[1]), z=float(velocity[2])
    )
    msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=float(angular_velocity))
    return msg


def make_joint_state_msg(
    joint_names: List[str],
    positions: List[float],
    velocities: List[float],
    stamp: Optional[TimeMsg] = None,
) -> JointState:
    """Create sensor_msgs/JointState."""
    msg = JointState()
    if stamp:
        msg.header.stamp = stamp
    msg.name = joint_names
    msg.position = [float(p) for p in positions]
    msg.velocity = [float(v) for v in velocities]
    return msg


def make_transform_stamped(
    pose: Pose,
    parent_frame: str,
    child_frame: str,
    stamp: Optional[TimeMsg] = None,
) -> TransformStamped:
    """Create geometry_msgs/TransformStamped from Pose."""
    t = TransformStamped()
    t.header = Header(frame_id=parent_frame)
    if stamp:
        t.header.stamp = stamp
    t.child_frame_id = child_frame
    t.transform.translation = Vector3(
        x=float(pose.position[0]),
        y=float(pose.position[1]),
        z=float(pose.position[2]),
    )
    t.transform.rotation = Quaternion(
        x=float(pose.orientation[0]),
        y=float(pose.orientation[1]),
        z=float(pose.orientation[2]),
        w=float(pose.orientation[3]),
    )
    return t


def twist_to_world_velocity(
    twist: Twist, current_yaw: float
) -> Tuple[float, float, float, float]:
    """Convert Twist (body frame) to world-frame velocity (vx, vy, vz, wz).

    Twist.linear.x/y are in the robot's body frame; rotated by current_yaw.
    Twist.linear.z passes through directly (world-frame vertical).

    Args:
        twist: geometry_msgs/Twist message (body-frame velocities)
        current_yaw: Current robot yaw in world frame (radians)

    Returns:
        (vx, vy, vz, wz) in world frame
    """
    cos_yaw = math.cos(current_yaw)
    sin_yaw = math.sin(current_yaw)
    vx = twist.linear.x * cos_yaw - twist.linear.y * sin_yaw
    vy = twist.linear.x * sin_yaw + twist.linear.y * cos_yaw
    vz = twist.linear.z  # Z velocity passes through directly
    wz = twist.angular.z
    return vx, vy, vz, wz


def nav_path_to_pbf_path(nav_path: NavPath) -> Path:
    """Convert nav_msgs/Path → PyBulletFleet Path."""
    waypoints = []
    for pose_stamped in nav_path.poses:
        waypoints.append(ros_pose_to_pbf(pose_stamped.pose))
    return Path(waypoints=waypoints)


def sim_time_to_ros_time(sim_time: float) -> TimeMsg:
    """Convert simulation time (float seconds) to builtin_interfaces/Time."""
    sec = int(sim_time)
    nanosec = int((sim_time - sec) * 1e9)
    return TimeMsg(sec=sec, nanosec=nanosec)
```

**Step 4: Run test to verify it passes**

Run: `cd /opt/bridge_ws && colcon test --packages-select pybullet_fleet_ros --pytest-args test/test_conversions.py -v`
Expected: All 9 tests PASS

**Step 5: Commit**

```bash
git add ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/conversions.py
git add ros2_bridge/pybullet_fleet_ros/test/test_conversions.py
git commit -m "feat(ros2): add ROS ↔ PyBulletFleet conversion utilities with Z-axis support"
```

---

## Task 5: RobotHandler — Per-Robot Topic Management (SERIAL, depends on Tasks 1, 3, 4)

**Files:**
- Create: `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/robot_handler.py`
- Create: `ros2_bridge/pybullet_fleet_ros/test/test_robot_handler.py`
- Create: `ros2_bridge/pybullet_fleet_ros/test/conftest.py`

**Step 1: Write the failing test**

```python
# ros2_bridge/pybullet_fleet_ros/test/conftest.py
"""Test fixtures for pybullet_fleet_ros tests."""

from unittest.mock import MagicMock, PropertyMock
import pytest

from pybullet_fleet.geometry import Pose
from pybullet_fleet.types import MotionMode


@pytest.fixture
def mock_node():
    """Mock rclpy.node.Node with publisher/subscriber factory methods."""
    node = MagicMock()
    node.get_logger.return_value = MagicMock()

    def make_pub(*args, **kwargs):
        pub = MagicMock()
        pub.topic_name = args[1] if len(args) > 1 else "unknown"
        return pub

    node.create_publisher.side_effect = make_pub

    def make_sub(*args, **kwargs):
        sub = MagicMock()
        sub.topic_name = args[1] if len(args) > 1 else "unknown"
        return sub

    node.create_subscription.side_effect = make_sub
    return node


@pytest.fixture
def mock_agent():
    """Mock Agent with basic attributes for a mobile robot."""
    agent = MagicMock()
    agent.name = "test_robot"
    agent.object_id = 0
    agent.body_id = 0
    agent.is_moving = False
    agent.get_pose.return_value = Pose.from_xyz(0.0, 0.0, 0.0)
    agent.velocity = [0.0, 0.0, 0.0]
    agent.angular_velocity = 0.0
    agent.get_num_joints.return_value = 0
    agent.get_all_joints_state.return_value = []
    agent.get_all_joints_state_by_name.return_value = {}
    agent.spawn_params = MagicMock()
    agent.spawn_params.motion_mode = MotionMode.OMNIDIRECTIONAL
    agent.spawn_params.use_fixed_base = False
    agent.ik_params = None
    agent._controller = None
    return agent
```

```python
# ros2_bridge/pybullet_fleet_ros/test/test_robot_handler.py
"""Tests for RobotHandler — per-robot ROS interface management."""

import pytest


def test_robot_handler_creates_publishers(mock_node, mock_agent):
    """RobotHandler creates odom, joint_states publishers for a mobile robot."""
    from pybullet_fleet_ros.robot_handler import RobotHandler

    handler = RobotHandler(mock_node, mock_agent)

    topic_names = [call[0][1] for call in mock_node.create_publisher.call_args_list]
    assert "/test_robot/odom" in topic_names
    assert "/test_robot/joint_states" in topic_names


def test_robot_handler_creates_cmd_vel_sub(mock_node, mock_agent):
    """RobotHandler subscribes to cmd_vel."""
    from pybullet_fleet_ros.robot_handler import RobotHandler

    handler = RobotHandler(mock_node, mock_agent)

    sub_topics = [call[0][1] for call in mock_node.create_subscription.call_args_list]
    assert "/test_robot/cmd_vel" in sub_topics


def test_cmd_vel_calls_velocity_controller(mock_node, mock_agent):
    """cmd_vel message calls OmniOmniVelocityController.set_velocity() (not agent directly)."""
    from unittest.mock import MagicMock
    from pybullet_fleet_ros.robot_handler import RobotHandler
    from geometry_msgs.msg import Twist

    mock_agent.get_pose.return_value.yaw = 0.0  # facing +X
    mock_vel_ctrl = MagicMock()  # Mock OmniOmniVelocityController

    handler = RobotHandler(mock_node, mock_agent, vel_controller=mock_vel_ctrl)

    twist = Twist()
    twist.linear.x = 1.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.z = 0.0
    handler._cmd_vel_cb(twist)
    handler.apply_cmd_vel()

    # Should call OmniOmniVelocityController.set_velocity() with world-frame values
    mock_vel_ctrl.set_velocity.assert_called_once()
    args = mock_vel_ctrl.set_velocity.call_args
    assert args[1]['vx'] == pytest.approx(1.0)
    assert args[1]['vy'] == pytest.approx(0.0)


def test_publish_state_publishes_odom(mock_node, mock_agent):
    """publish_state sends Odometry message."""
    from pybullet_fleet_ros.robot_handler import RobotHandler
    from pybullet_fleet_ros.conversions import sim_time_to_ros_time

    handler = RobotHandler(mock_node, mock_agent)
    stamp = sim_time_to_ros_time(1.0)
    handler.publish_state(stamp)

    assert handler._odom_pub.publish.called
```

**Step 2: Run test to verify it fails**

Run: `pytest ros2_bridge/pybullet_fleet_ros/test/test_robot_handler.py -v`
Expected: FAIL — `pybullet_fleet_ros.robot_handler` not found

**Step 3: Write RobotHandler implementation**

```python
# ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/robot_handler.py
"""Per-robot ROS interface handler.

Creates publishers, subscribers for a single Agent.
Uses OmniOmniVelocityController.set_velocity() for cmd_vel commands (world-frame velocity).
The OmniOmniVelocityController is created by BridgeNode and passed to RobotHandler.
"""

import logging
from typing import TYPE_CHECKING, Optional

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from .conversions import (
    make_joint_state_msg,
    make_odom_msg,
    make_transform_stamped,
    sim_time_to_ros_time,
    twist_to_world_velocity,
)

if TYPE_CHECKING:
    from pybullet_fleet.agent import Agent
    from rclpy.node import Node
    from tf2_ros import TransformBroadcaster

logger = logging.getLogger(__name__)


class RobotHandler:
    """Manages all ROS 2 interfaces for a single simulated Agent.

    Creates:
    - /{name}/cmd_vel subscriber (Twist) → OmniOmniVelocityController.set_velocity()
    - /{name}/odom publisher (Odometry)
    - /{name}/joint_states publisher (JointState)
    - TF broadcast: odom → {name}/base_link
    """

    def __init__(
        self,
        node: "Node",
        agent: "Agent",
        vel_controller: "OmniOmniVelocityController",
        tf_broadcaster: Optional["TransformBroadcaster"] = None,
    ):
        self._node = node
        self.agent = agent
        self._vel_controller = vel_controller
        self._tf_broadcaster = tf_broadcaster
        self._latest_twist: Optional[Twist] = None

        ns = agent.name or f"robot_{agent.object_id}"
        self._ns = ns

        # Publishers
        self._odom_pub = node.create_publisher(Odometry, f"/{ns}/odom", 10)
        self._joint_pub = node.create_publisher(JointState, f"/{ns}/joint_states", 10)

        # Subscribers
        self._cmd_vel_sub = node.create_subscription(
            Twist, f"/{ns}/cmd_vel", self._cmd_vel_cb, 10
        )

        logger.info("RobotHandler created for '%s' (object_id=%d)", ns, agent.object_id)

    def _cmd_vel_cb(self, msg: Twist) -> None:
        """Store latest cmd_vel for application in next step."""
        self._latest_twist = msg

    def apply_cmd_vel(self) -> None:
        """Apply stored cmd_vel as a velocity command via OmniOmniVelocityController.

        Called once per sim step by BridgeNode.
        Converts body-frame Twist to world-frame velocity and calls
        OmniOmniVelocityController.set_velocity() (NOT agent.set_velocity()).
        """
        if self._latest_twist is None:
            return

        twist = self._latest_twist
        yaw = self.agent.get_pose().yaw

        vx, vy, vz, wz = twist_to_world_velocity(twist, yaw)
        self._vel_controller.set_velocity(vx=vx, vy=vy, vz=vz, wz=wz)

        # Reset twist after applying (require continuous publishing)
        self._latest_twist = None

    def publish_state(self, stamp: TimeMsg) -> None:
        """Publish odom, joint_states, and TF for this agent."""
        pose = self.agent.get_pose()
        velocity = list(self.agent.velocity)
        angular_vel = self.agent.angular_velocity

        # Odometry
        odom_msg = make_odom_msg(
            pose, velocity, angular_vel,
            frame_id="odom",
            child_frame_id=f"{self._ns}/base_link",
            stamp=stamp,
        )
        self._odom_pub.publish(odom_msg)

        # TF: odom → {ns}/base_link
        if self._tf_broadcaster is not None:
            tf_msg = make_transform_stamped(
                pose,
                parent_frame="odom",
                child_frame=f"{self._ns}/base_link",
                stamp=stamp,
            )
            self._tf_broadcaster.sendTransform(tf_msg)

        # Joint states (only if robot has joints)
        num_joints = self.agent.get_num_joints()
        if num_joints > 0:
            self._publish_joint_states(stamp)

    def _publish_joint_states(self, stamp: TimeMsg) -> None:
        """Publish sensor_msgs/JointState for all joints."""
        joints_by_name = self.agent.get_all_joints_state_by_name()
        if not joints_by_name:
            return

        names = list(joints_by_name.keys())
        positions = [joints_by_name[n][0] for n in names]
        velocities = [joints_by_name[n][1] for n in names]

        msg = make_joint_state_msg(names, positions, velocities, stamp=stamp)
        self._joint_pub.publish(msg)

    def destroy(self) -> None:
        """Clean up ROS interfaces."""
        self._node.destroy_publisher(self._odom_pub)
        self._node.destroy_publisher(self._joint_pub)
        self._node.destroy_subscription(self._cmd_vel_sub)
        logger.info("RobotHandler destroyed for '%s'", self._ns)
```

**Step 4: Run test to verify it passes**

Run: `pytest ros2_bridge/pybullet_fleet_ros/test/test_robot_handler.py -v`
Expected: All 4 tests PASS

**Step 5: Commit**

```bash
git add ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/robot_handler.py
git add ros2_bridge/pybullet_fleet_ros/test/test_robot_handler.py
git add ros2_bridge/pybullet_fleet_ros/test/conftest.py
git commit -m "feat(ros2): add RobotHandler with cmd_vel→OmniOmniVelocityController, odom, joint_states, tf"
```

---

## Task 6: BridgeNode + ROSSimulationCore (SERIAL, depends on Tasks 2, 5)

**Files:**
- Create: `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/bridge_node.py`
- Create: `ros2_bridge/pybullet_fleet_ros/test/test_bridge_node.py`

**Step 1: Write the failing test**

```python
# ros2_bridge/pybullet_fleet_ros/test/test_bridge_node.py
"""Tests for BridgeNode — main simulation node."""

from unittest.mock import MagicMock, patch
import pytest


def test_ros_simulation_core_overrides_run_simulation():
    """ROSSimulationCore.run_simulation() initializes but does not block."""
    from pybullet_fleet_ros.bridge_node import ROSSimulationCore
    from pybullet_fleet import SimulationParams

    sim = ROSSimulationCore(SimulationParams(gui=False, monitor=False, target_rtf=0))
    # run_simulation should NOT block (it just calls initialize_simulation)
    # In normal MultiRobotSimulationCore this would block forever
    sim.run_simulation(duration=0)
    # If we get here without hanging, the override works
    assert sim._step_count == 0  # No steps were taken


def test_step_once_increments_sim_time():
    """step_once() increments simulation time."""
    from pybullet_fleet_ros.bridge_node import ROSSimulationCore
    from pybullet_fleet import SimulationParams

    sim = ROSSimulationCore(SimulationParams(gui=False, monitor=False, physics=False))
    sim.initialize_simulation()
    initial_time = sim.sim_time
    sim.step_once()
    assert sim.sim_time > initial_time
```

**Step 2: Write BridgeNode implementation**

```python
# ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/bridge_node.py
"""Main bridge node — wraps MultiRobotSimulationCore as a ROS 2 node.

Uses ROSSimulationCore (subclass) to override run_simulation() with a
ROS-timer-driven loop. RTF is controlled by the timer period.

Usage:
    ros2 run pybullet_fleet_ros bridge_node --ros-args -p num_robots:=5
"""

import logging
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster

from pybullet_fleet import (
    Agent,
    AgentSpawnParams,
    MultiRobotSimulationCore,
    Pose,
    SimulationParams,
)
from pybullet_fleet.controller import OmniOmniVelocityController
from pybullet_fleet.types import MotionMode

from .conversions import sim_time_to_ros_time
from .robot_handler import RobotHandler

logger = logging.getLogger(__name__)


class ROSSimulationCore(MultiRobotSimulationCore):
    """Simulation core with ROS-aware run_simulation() override.

    Instead of blocking in a while-loop with time.sleep() for RTF control,
    this subclass lets the ROS timer drive stepping. run_simulation() just
    initializes and returns immediately.
    """

    def run_simulation(self, duration: Optional[float] = None) -> None:
        """Override: initialize only, do not block.

        The ROS timer in BridgeNode drives step_once() calls.
        RTF is controlled by the timer period (dt / target_rtf).
        """
        self.initialize_simulation()
        # Do NOT enter the blocking while loop.
        # The ROS timer handles stepping and RTF.


class BridgeNode(Node):
    """ROS 2 node that drives PyBulletFleet simulation and exposes per-robot topics.

    Parameters (ROS):
        config_yaml (str): Path to PyBulletFleet YAML config file.
        num_robots (int): Number of mobile robots to spawn at startup.
        robot_urdf (str): URDF path for default mobile robot.
        publish_rate (float): State publish rate in Hz.
        gui (bool): Enable PyBullet GUI window.
        physics (bool): Enable physics simulation.
        target_rtf (float): Target real-time factor (1.0 = real time).
        enable_sim_services (bool): Create simulation_interfaces services.
    """

    def __init__(self):
        super().__init__("pybullet_fleet_bridge")

        # Declare parameters
        self.declare_parameter("config_yaml", "")
        self.declare_parameter("num_robots", 1)
        self.declare_parameter("robot_urdf", "robots/mobile_robot.urdf")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("gui", False)
        self.declare_parameter("physics", False)
        self.declare_parameter("target_rtf", 1.0)
        self.declare_parameter("enable_sim_services", True)

        # Read parameters
        config_yaml = self.get_parameter("config_yaml").get_parameter_value().string_value
        num_robots = self.get_parameter("num_robots").get_parameter_value().integer_value
        robot_urdf = self.get_parameter("robot_urdf").get_parameter_value().string_value
        publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        gui = self.get_parameter("gui").get_parameter_value().bool_value
        physics = self.get_parameter("physics").get_parameter_value().bool_value
        target_rtf = self.get_parameter("target_rtf").get_parameter_value().double_value
        enable_sim_services = self.get_parameter("enable_sim_services").get_parameter_value().bool_value

        # Create simulation core (ROSSimulationCore overrides run_simulation)
        if config_yaml:
            # TODO: load from YAML and wrap as ROSSimulationCore
            self.sim = ROSSimulationCore(SimulationParams(gui=gui, physics=physics, monitor=False))
        else:
            self.sim = ROSSimulationCore(
                SimulationParams(
                    gui=gui,
                    physics=physics,
                    monitor=False,
                    target_rtf=target_rtf,
                )
            )

        # Initialize simulation (calls overridden run_simulation which returns immediately)
        self.sim.run_simulation(duration=0)

        # TF broadcaster (shared by all handlers)
        self._tf_broadcaster = TransformBroadcaster(self)

        # Robot handlers (object_id → RobotHandler)
        self._handlers: Dict[int, RobotHandler] = {}

        # Spawn initial robots
        for i in range(num_robots):
            self._spawn_default_robot(robot_urdf, i)

        # /clock publisher
        self._clock_pub = self.create_publisher(Clock, "/clock", 10)

        # simulation_interfaces services
        self._sim_services = None
        if enable_sim_services:
            from .sim_services import SimServices
            self._sim_services = SimServices(self, self.sim, self)

        # Simulation step timer — period controls RTF
        dt = self.sim.params.timestep
        effective_rtf = target_rtf if target_rtf > 0 else 0.0
        if effective_rtf > 0:
            timer_period = dt / effective_rtf
        else:
            timer_period = 0.0  # As fast as possible
        self._step_timer = self.create_timer(timer_period, self._step_callback)

        # Publish timer (may be slower than step rate)
        publish_period = 1.0 / publish_rate
        self._publish_timer = self.create_timer(publish_period, self._publish_callback)

        self.get_logger().info(
            f"BridgeNode started: {num_robots} robots, dt={dt:.4f}s, "
            f"rtf={target_rtf}, publish_rate={publish_rate}Hz, gui={gui}, physics={physics}"
        )

    def _spawn_default_robot(self, urdf_path: str, index: int) -> Agent:
        """Spawn a default mobile robot and create its RobotHandler."""
        params = AgentSpawnParams(
            urdf_path=urdf_path,
            initial_pose=Pose.from_xyz(index * 2.0, 0.0, 0.05),
            motion_mode=MotionMode.OMNIDIRECTIONAL,
            max_linear_vel=2.0,
            name=f"robot_{index}",
        )
        agent = Agent.from_params(params, sim_core=self.sim)
        # Attach OmniOmniVelocityController for cmd_vel control
        vel_ctrl = OmniOmniVelocityController()
        agent.set_controller(vel_ctrl)
        handler = RobotHandler(
            self, agent,
            vel_controller=vel_ctrl,
            tf_broadcaster=self._tf_broadcaster,
        )
        self._handlers[agent.object_id] = handler
        return agent

    def spawn_robot(self, spawn_params: AgentSpawnParams) -> Agent:
        """Spawn a robot dynamically (e.g., via SpawnEntity service)."""
        agent = Agent.from_params(spawn_params, sim_core=self.sim)
        # Attach OmniOmniVelocityController for cmd_vel control
        vel_ctrl = OmniOmniVelocityController()
        agent.set_controller(vel_ctrl)
        handler = RobotHandler(
            self, agent,
            vel_controller=vel_ctrl,
            tf_broadcaster=self._tf_broadcaster,
        )
        self._handlers[agent.object_id] = handler
        self.get_logger().info(f"Spawned robot '{agent.name}' (id={agent.object_id})")
        return agent

    def remove_robot(self, object_id: int) -> bool:
        """Remove a robot and its ROS interfaces."""
        handler = self._handlers.pop(object_id, None)
        if handler is None:
            return False
        handler.destroy()
        self.sim.remove_object(handler.agent)
        self.get_logger().info(f"Removed robot '{handler.agent.name}' (id={object_id})")
        return True

    def _step_callback(self) -> None:
        """Called every sim timestep — advance simulation."""
        # Apply pending cmd_vel commands
        for handler in self._handlers.values():
            handler.apply_cmd_vel()

        # Step the simulation
        self.sim.step_once()

        # Publish /clock
        clock_msg = Clock()
        clock_msg.clock = sim_time_to_ros_time(self.sim.sim_time)
        self._clock_pub.publish(clock_msg)

    def _publish_callback(self) -> None:
        """Called at publish_rate — publish state for all robots."""
        stamp = sim_time_to_ros_time(self.sim.sim_time)
        for handler in self._handlers.values():
            handler.publish_state(stamp)

    @property
    def handlers(self) -> Dict[int, RobotHandler]:
        """Access robot handlers (used by SimServices for spawn/delete)."""
        return self._handlers


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

**Step 3: Run test to verify**

Run: `pytest ros2_bridge/pybullet_fleet_ros/test/test_bridge_node.py -v`
Expected: PASS

**Step 4: Commit**

```bash
git add ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/bridge_node.py
git add ros2_bridge/pybullet_fleet_ros/test/test_bridge_node.py
git commit -m "feat(ros2): add BridgeNode + ROSSimulationCore with ROS-aware sim loop"
```

---

## Task 7: SimServices — simulation_interfaces Implementation (SERIAL, depends on Task 6)

**Files:**
- Create: `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/sim_services.py`
- Create: `ros2_bridge/pybullet_fleet_ros/test/test_sim_services.py`

**Step 1: Write the failing test**

```python
# ros2_bridge/pybullet_fleet_ros/test/test_sim_services.py
"""Tests for SimServices — simulation_interfaces implementation."""

from unittest.mock import MagicMock
import pytest


def test_get_simulator_features():
    """GetSimulatorFeatures returns supported features."""
    from pybullet_fleet_ros.sim_services import SimServices

    mock_node = MagicMock()
    mock_sim = MagicMock()
    mock_bridge = MagicMock()

    services = SimServices(mock_node, mock_sim, mock_bridge)
    request = MagicMock()
    response = MagicMock()

    result = services._get_features(request, response)
    assert result is not None
    assert response.result.success is True


def test_get_entities_returns_agents():
    """GetEntities returns list of all agents."""
    from pybullet_fleet_ros.sim_services import SimServices

    mock_node = MagicMock()
    mock_sim = MagicMock()
    mock_bridge = MagicMock()

    agent1 = MagicMock()
    agent1.name = "robot_0"
    agent1.object_id = 0
    agent2 = MagicMock()
    agent2.name = "robot_1"
    agent2.object_id = 1
    mock_sim.sim_objects = [agent1, agent2]

    services = SimServices(mock_node, mock_sim, mock_bridge)
    request = MagicMock()
    response = MagicMock()
    response.names = []

    result = services._get_entities(request, response)
    assert len(response.names) == 2


def test_step_simulation():
    """StepSimulation calls sim.step_once()."""
    from pybullet_fleet_ros.sim_services import SimServices

    mock_node = MagicMock()
    mock_sim = MagicMock()
    mock_bridge = MagicMock()

    services = SimServices(mock_node, mock_sim, mock_bridge)
    request = MagicMock()
    request.num_steps = 5
    response = MagicMock()

    services._step_sim(request, response)
    assert mock_sim.step_once.call_count == 5


def test_pause_resume():
    """SetSimulationState can pause and resume."""
    from pybullet_fleet_ros.sim_services import SimServices

    mock_node = MagicMock()
    mock_sim = MagicMock()
    mock_bridge = MagicMock()

    services = SimServices(mock_node, mock_sim, mock_bridge)

    # Pause
    request = MagicMock()
    request.state.state = 0  # PAUSED
    response = MagicMock()
    services._set_sim_state(request, response)
    mock_sim.pause.assert_called_once()

    # Resume
    request.state.state = 1  # PLAYING
    services._set_sim_state(request, response)
    mock_sim.resume.assert_called_once()
```

**Step 2: Run test to verify it fails**

Run: `pytest ros2_bridge/pybullet_fleet_ros/test/test_sim_services.py -v`
Expected: FAIL — `pybullet_fleet_ros.sim_services` not found

**Step 3: Write SimServices implementation**

```python
# ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/sim_services.py
"""simulation_interfaces service and action server implementations.

Implements the simulation_interfaces standard for PyBulletFleet.
See: https://github.com/ros-simulation/simulation_interfaces
"""

import logging
import os
from typing import TYPE_CHECKING

import pybullet as p

from pybullet_fleet import Agent, AgentSpawnParams, Pose
from pybullet_fleet.types import MotionMode

from .conversions import pbf_pose_to_ros, ros_pose_to_pbf

if TYPE_CHECKING:
    from rclpy.node import Node
    from pybullet_fleet.core_simulation import MultiRobotSimulationCore
    from .bridge_node import BridgeNode

logger = logging.getLogger(__name__)

from simulation_interfaces.srv import (
    DeleteEntity,
    GetEntities,
    GetEntitiesStates,
    GetEntityBounds,
    GetEntityInfo,
    GetEntityState,
    GetSimulationState,
    GetSimulatorFeatures,
    GetSpawnables,
    ResetSimulation,
    SetEntityState,
    SetSimulationState,
    SpawnEntity,
    StepSimulation,
)
from simulation_interfaces.action import SimulateSteps
from simulation_interfaces.msg import (
    EntityState,
    SimulationState,
)

from rclpy.action import ActionServer


class SimServices:
    """Implements simulation_interfaces services and actions.

    Services (14):
        /sim/get_simulator_features
        /sim/spawn_entity
        /sim/delete_entity
        /sim/get_entity_state
        /sim/set_entity_state
        /sim/get_entities
        /sim/get_entities_states
        /sim/get_entity_info
        /sim/get_entity_bounds
        /sim/step_simulation
        /sim/get_simulation_state
        /sim/set_simulation_state
        /sim/reset_simulation
        /sim/get_spawnables

    Actions (1):
        /sim/simulate_steps
    """

    def __init__(
        self,
        node: "Node",
        sim: "MultiRobotSimulationCore",
        bridge: "BridgeNode",
    ):
        self._node = node
        self._sim = sim
        self._bridge = bridge

        # Create services
        node.create_service(GetSimulatorFeatures, "/sim/get_simulator_features", self._get_features)
        node.create_service(SpawnEntity, "/sim/spawn_entity", self._spawn_entity)
        node.create_service(DeleteEntity, "/sim/delete_entity", self._delete_entity)
        node.create_service(GetEntityState, "/sim/get_entity_state", self._get_entity_state)
        node.create_service(SetEntityState, "/sim/set_entity_state", self._set_entity_state)
        node.create_service(GetEntities, "/sim/get_entities", self._get_entities)
        node.create_service(GetEntitiesStates, "/sim/get_entities_states", self._get_entities_states)
        node.create_service(GetEntityInfo, "/sim/get_entity_info", self._get_entity_info)
        node.create_service(GetEntityBounds, "/sim/get_entity_bounds", self._get_entity_bounds)
        node.create_service(StepSimulation, "/sim/step_simulation", self._step_sim)
        node.create_service(GetSimulationState, "/sim/get_simulation_state", self._get_sim_state)
        node.create_service(SetSimulationState, "/sim/set_simulation_state", self._set_sim_state)
        node.create_service(ResetSimulation, "/sim/reset_simulation", self._reset_sim)
        node.create_service(GetSpawnables, "/sim/get_spawnables", self._get_spawnables)

        # Action server
        self._simulate_steps_server = ActionServer(
            node, SimulateSteps, "/sim/simulate_steps", self._simulate_steps_cb
        )

        logger.info("SimServices: all simulation_interfaces registered")

    def _find_by_name(self, name: str):
        """Find SimObject or Agent by name."""
        for obj in self._sim.sim_objects:
            if obj.name == name:
                return obj
        return None

    def _get_features(self, request, response):
        """Report supported simulation features."""
        features = response.features
        features.spawn_entity = True
        features.delete_entity = True
        features.get_entity_state = True
        features.set_entity_state = True
        features.step_simulation = True
        features.simulation_state = True
        features.reset_simulation = True
        features.entity_bounds = True
        features.entity_info = True
        features.spawnables = True
        response.result.success = True
        return response

    def _spawn_entity(self, request, response):
        """Spawn a new entity."""
        try:
            name = request.name
            urdf_path = "robots/mobile_robot.urdf"
            if hasattr(request, 'resource') and request.resource and request.resource.uri:
                urdf_path = request.resource.uri

            initial_pose = Pose.from_xyz(0.0, 0.0, 0.05)
            if hasattr(request, 'state') and request.state:
                initial_pose = ros_pose_to_pbf(request.state.pose)

            params = AgentSpawnParams(
                urdf_path=urdf_path,
                initial_pose=initial_pose,
                motion_mode=MotionMode.OMNIDIRECTIONAL,
                name=name or None,
            )
            agent = self._bridge.spawn_robot(params)
            response.result.success = True
            response.result.message = f"Spawned '{agent.name}' (id={agent.object_id})"
        except Exception as e:
            response.result.success = False
            response.result.message = str(e)
            logger.error("SpawnEntity failed: %s", e)
        return response

    def _delete_entity(self, request, response):
        """Delete an entity by name."""
        obj = self._find_by_name(request.name)
        if obj is None:
            response.result.success = False
            response.result.message = f"Entity '{request.name}' not found"
            return response
        self._bridge.remove_robot(obj.object_id)
        response.result.success = True
        return response

    def _get_entity_state(self, request, response):
        """Get pose and velocity of an entity."""
        obj = self._find_by_name(request.name)
        if obj is None:
            response.result.success = False
            response.result.message = f"Entity '{request.name}' not found"
            return response
        pose = obj.get_pose()
        response.state.pose = pbf_pose_to_ros(pose)
        response.result.success = True
        return response

    def _set_entity_state(self, request, response):
        """Set pose of an entity."""
        obj = self._find_by_name(request.name)
        if obj is None:
            response.result.success = False
            response.result.message = f"Entity '{request.name}' not found"
            return response
        if hasattr(request, 'state') and request.state:
            new_pose = ros_pose_to_pbf(request.state.pose)
            obj.set_pose(new_pose)
        response.result.success = True
        return response

    def _get_entities(self, request, response):
        """Return list of all entities."""
        for obj in self._sim.sim_objects:
            name = obj.name or f"object_{obj.object_id}"
            response.names.append(name)
        response.result.success = True
        return response

    def _get_entities_states(self, request, response):
        """Return states for all entities."""
        for obj in self._sim.sim_objects:
            state = EntityState()
            pose = obj.get_pose()
            state.pose = pbf_pose_to_ros(pose)
            response.states.append(state)
            response.names.append(obj.name or f"object_{obj.object_id}")
        response.result.success = True
        return response

    def _get_entity_info(self, request, response):
        """Return metadata about an entity."""
        obj = self._find_by_name(request.name)
        if obj is None:
            response.result.success = False
            response.result.message = f"Entity '{request.name}' not found"
            return response
        response.result.success = True
        return response

    def _get_entity_bounds(self, request, response):
        """Get AABB bounds of an entity."""
        obj = self._find_by_name(request.name)
        if obj is None:
            response.result.success = False
            response.result.message = f"Entity '{request.name}' not found"
            return response
        try:
            aabb_min, aabb_max = p.getAABB(obj.body_id, physicsClientId=self._sim.client)
            response.bounds.min.x = aabb_min[0]
            response.bounds.min.y = aabb_min[1]
            response.bounds.min.z = aabb_min[2]
            response.bounds.max.x = aabb_max[0]
            response.bounds.max.y = aabb_max[1]
            response.bounds.max.z = aabb_max[2]
            response.result.success = True
        except Exception as e:
            response.result.success = False
            response.result.message = str(e)
        return response

    def _step_sim(self, request, response):
        """Execute N simulation steps."""
        num_steps = getattr(request, 'num_steps', 1)
        for _ in range(num_steps):
            self._sim.step_once()
        response.result.success = True
        return response

    def _get_sim_state(self, request, response):
        """Return current simulation state (paused/playing)."""
        if self._sim.is_paused:
            response.state.state = SimulationState.PAUSED
        else:
            response.state.state = SimulationState.PLAYING
        response.result.success = True
        return response

    def _set_sim_state(self, request, response):
        """Set simulation state (pause/resume)."""
        target_state = request.state.state
        if target_state == SimulationState.PAUSED:
            self._sim.pause()
        elif target_state == SimulationState.PLAYING:
            self._sim.resume()
        response.result.success = True
        return response

    def _reset_sim(self, request, response):
        """Reset simulation to initial state."""
        for handler in list(self._bridge.handlers.values()):
            handler.destroy()
        self._bridge.handlers.clear()
        p.resetSimulation(physicsClientId=self._sim.client)
        response.result.success = True
        response.result.message = "Simulation reset. Re-spawn entities as needed."
        return response

    def _get_spawnables(self, request, response):
        """List available robot URDFs."""
        robots_dir = "robots"
        if os.path.isdir(robots_dir):
            for f in os.listdir(robots_dir):
                if f.endswith(".urdf"):
                    spawnable = response.spawnables.add() if hasattr(response.spawnables, 'add') else None
                    if spawnable:
                        spawnable.name = f.replace(".urdf", "")
                        spawnable.resource.uri = os.path.join(robots_dir, f)
        response.result.success = True
        return response

    def _simulate_steps_cb(self, goal_handle):
        """Execute N simulation steps with feedback."""
        num_steps = goal_handle.request.num_steps
        feedback = SimulateSteps.Feedback()

        for i in range(num_steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = SimulateSteps.Result()
                result.success = False
                return result

            self._sim.step_once()
            feedback.steps_done = i + 1
            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        result = SimulateSteps.Result()
        result.success = True
        return result
```

**Note:** Exact field names in `simulation_interfaces` messages should be verified against the installed package. The structure above follows the GitHub repository patterns.

**Step 4: Run tests to verify**

Run: `pytest ros2_bridge/pybullet_fleet_ros/test/test_sim_services.py -v`
Expected: All 4 tests PASS

**Step 5: Commit**

```bash
git add ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/sim_services.py
git add ros2_bridge/pybullet_fleet_ros/test/test_sim_services.py
git commit -m "feat(ros2): add simulation_interfaces services (14) + SimulateSteps action"
```

---

## Task 8: Integration Test with Docker (SERIAL, depends on Task 7)

**Files:**
- Create: `docker/test_integration.sh`
- Create: `ros2_bridge/pybullet_fleet_ros/launch/bridge.launch.py`
- Create: `ros2_bridge/pybullet_fleet_ros/launch/multi_robot.launch.py`

**Step 1: Create bridge.launch.py**

```python
# ros2_bridge/pybullet_fleet_ros/launch/bridge.launch.py
"""Launch core bridge node with configurable parameters."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("num_robots", default_value="1"),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("publish_rate", default_value="50.0"),
        DeclareLaunchArgument("target_rtf", default_value="1.0"),

        Node(
            package="pybullet_fleet_ros",
            executable="bridge_node",
            name="pybullet_fleet_bridge",
            parameters=[{
                "num_robots": LaunchConfiguration("num_robots"),
                "robot_urdf": "robots/mobile_robot.urdf",
                "publish_rate": LaunchConfiguration("publish_rate"),
                "gui": LaunchConfiguration("gui"),
                "target_rtf": LaunchConfiguration("target_rtf"),
            }],
            output="screen",
        ),
    ])
```

**Step 2: Create multi_robot.launch.py**

```python
# ros2_bridge/pybullet_fleet_ros/launch/multi_robot.launch.py
"""Launch bridge with N mobile robots."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("num_robots", default_value="10"),
        DeclareLaunchArgument("gui", default_value="false"),

        Node(
            package="pybullet_fleet_ros",
            executable="bridge_node",
            name="pybullet_fleet_bridge",
            parameters=[{
                "num_robots": LaunchConfiguration("num_robots"),
                "robot_urdf": "robots/mobile_robot.urdf",
                "publish_rate": 20.0,
                "gui": LaunchConfiguration("gui"),
            }],
            output="screen",
        ),
    ])
```

**Step 3: Create integration test script**

```bash
#!/bin/bash
# docker/test_integration.sh
# Run inside Docker container to verify full stack
set -e

echo "=== Integration Test: PyBulletFleet ROS 2 Bridge ==="

# Source ROS
source /opt/ros/jazzy/setup.bash
source /opt/sim_interfaces_ws/install/setup.bash
source /opt/bridge_ws/install/setup.bash

# Start bridge in background
ros2 run pybullet_fleet_ros bridge_node \
    --ros-args -p num_robots:=3 -p gui:=false -p publish_rate:=10.0 &
BRIDGE_PID=$!
sleep 3

echo "--- Checking topics ---"
TOPICS=$(ros2 topic list)
echo "$TOPICS"

for topic in /clock /robot_0/odom /robot_0/cmd_vel /robot_0/joint_states \
             /robot_1/odom /robot_2/odom /tf; do
    if echo "$TOPICS" | grep -q "$topic"; then
        echo "  ✓ $topic"
    else
        echo "  ✗ $topic MISSING"
        kill $BRIDGE_PID 2>/dev/null
        exit 1
    fi
done

echo "--- Checking /clock publishes ---"
timeout 5 ros2 topic echo /clock --once || { echo "✗ /clock not publishing"; kill $BRIDGE_PID; exit 1; }
echo "  ✓ /clock publishing"

echo "--- Checking odom publishes ---"
timeout 5 ros2 topic echo /robot_0/odom --once || { echo "✗ odom not publishing"; kill $BRIDGE_PID; exit 1; }
echo "  ✓ /robot_0/odom publishing"

echo "--- Checking services ---"
SERVICES=$(ros2 service list)
for svc in /sim/get_simulator_features /sim/spawn_entity /sim/get_entities \
           /sim/step_simulation /sim/get_simulation_state; do
    if echo "$SERVICES" | grep -q "$svc"; then
        echo "  ✓ $svc"
    else
        echo "  ✗ $svc MISSING"
        kill $BRIDGE_PID 2>/dev/null
        exit 1
    fi
done

echo "--- Testing cmd_vel → OmniOmniVelocityController ---"
ros2 topic pub --once /robot_0/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 1
echo "  ✓ cmd_vel published"

# Cleanup
kill $BRIDGE_PID 2>/dev/null
wait $BRIDGE_PID 2>/dev/null

echo ""
echo "=== All integration tests PASSED ==="
```

**Step 4: Run integration test**

Run: `cd docker && docker compose run --rm test`
Expected: All checks pass

**Step 5: Commit**

```bash
git add docker/test_integration.sh
git add ros2_bridge/pybullet_fleet_ros/launch/
git commit -m "test(ros2): add integration test + launch files"
```

---

## Task 9: Client Scripts (SERIAL, depends on Task 8)

**Files:**
- Create: `ros2_bridge/scripts/teleop_cmd_vel.py`
- Create: `ros2_bridge/scripts/send_nav_goal.py`
- Create: `ros2_bridge/scripts/spawn_robots.py`
- Create: `ros2_bridge/scripts/query_entities.py`
- Create: `ros2_bridge/scripts/step_simulation.py`

Standalone Python scripts that interact with the bridge purely via ROS topics/services/actions. These demonstrate the ROS API without importing any PyBulletFleet code.

**Step 1: teleop_cmd_vel.py**

```python
#!/usr/bin/env python3
"""Send cmd_vel commands to a robot via ROS 2 topic.

Usage:
    # In Docker:
    python3 scripts/teleop_cmd_vel.py --robot robot_0 --vx 1.0 --vy 0.0 --wz 0.2 --duration 5.0

    # Or with ros2 topic pub directly:
    ros2 topic pub /robot_0/cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 1.0}, angular: {z: 0.2}}" --rate 10
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelPublisher(Node):
    def __init__(self, robot_name: str, vx: float, vy: float, vz: float, wz: float, rate: float):
        super().__init__("teleop_cmd_vel")
        self._pub = self.create_publisher(Twist, f"/{robot_name}/cmd_vel", 10)
        self._twist = Twist()
        self._twist.linear.x = vx
        self._twist.linear.y = vy
        self._twist.linear.z = vz
        self._twist.angular.z = wz
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f"Publishing cmd_vel to /{robot_name}/cmd_vel: "
            f"vx={vx}, vy={vy}, vz={vz}, wz={wz} at {rate}Hz"
        )

    def _publish(self):
        self._pub.publish(self._twist)


def main():
    parser = argparse.ArgumentParser(description="Send cmd_vel to a robot")
    parser.add_argument("--robot", default="robot_0", help="Robot name")
    parser.add_argument("--vx", type=float, default=0.5, help="Linear X velocity (m/s)")
    parser.add_argument("--vy", type=float, default=0.0, help="Linear Y velocity (m/s)")
    parser.add_argument("--vz", type=float, default=0.0, help="Linear Z velocity (m/s)")
    parser.add_argument("--wz", type=float, default=0.0, help="Angular Z velocity (rad/s)")
    parser.add_argument("--rate", type=float, default=10.0, help="Publish rate (Hz)")
    parser.add_argument("--duration", type=float, default=0.0, help="Duration (0=infinite)")
    args = parser.parse_args()

    rclpy.init()
    node = CmdVelPublisher(args.robot, args.vx, args.vy, args.vz, args.wz, args.rate)

    if args.duration > 0:
        start = time.time()
        while rclpy.ok() and (time.time() - start) < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
        # Send zero velocity to stop
        stop_twist = Twist()
        node._pub.publish(stop_twist)
        time.sleep(0.1)
    else:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

**Step 2: spawn_robots.py**

```python
#!/usr/bin/env python3
"""Spawn robots via the SpawnEntity service.

Usage:
    python3 scripts/spawn_robots.py --name arm_0 --urdf robots/arm_robot.urdf --x 1.0 --y 2.0
    python3 scripts/spawn_robots.py --name mobile_5 --x 5.0
"""

import argparse

import rclpy
from rclpy.node import Node
from simulation_interfaces.srv import SpawnEntity


def main():
    parser = argparse.ArgumentParser(description="Spawn a robot via SpawnEntity service")
    parser.add_argument("--name", required=True, help="Entity name")
    parser.add_argument("--urdf", default="robots/mobile_robot.urdf", help="URDF path")
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.05)
    args = parser.parse_args()

    rclpy.init()
    node = Node("spawn_robots_client")
    client = node.create_client(SpawnEntity, "/sim/spawn_entity")

    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("SpawnEntity service not available")
        return

    request = SpawnEntity.Request()
    request.name = args.name
    request.resource.uri = args.urdf
    request.state.pose.position.x = args.x
    request.state.pose.position.y = args.y
    request.state.pose.position.z = args.z
    request.state.pose.orientation.w = 1.0

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    result = future.result()
    if result.result.success:
        node.get_logger().info(f"Spawned: {result.result.message}")
    else:
        node.get_logger().error(f"Failed: {result.result.message}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

**Step 3: query_entities.py**

```python
#!/usr/bin/env python3
"""Query entities and their states via simulation_interfaces services.

Usage:
    python3 scripts/query_entities.py                     # List all entities
    python3 scripts/query_entities.py --name robot_0      # Get specific entity state
"""

import argparse

import rclpy
from rclpy.node import Node
from simulation_interfaces.srv import GetEntities, GetEntityState


def main():
    parser = argparse.ArgumentParser(description="Query simulation entities")
    parser.add_argument("--name", default=None, help="Get state of specific entity")
    args = parser.parse_args()

    rclpy.init()
    node = Node("query_entities_client")

    if args.name:
        client = node.create_client(GetEntityState, "/sim/get_entity_state")
        if not client.wait_for_service(timeout_sec=5.0):
            node.get_logger().error("GetEntityState service not available")
            return

        request = GetEntityState.Request()
        request.name = args.name
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        result = future.result()

        if result.result.success:
            p = result.state.pose.position
            node.get_logger().info(f"Entity '{args.name}': pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f})")
        else:
            node.get_logger().error(f"Failed: {result.result.message}")
    else:
        client = node.create_client(GetEntities, "/sim/get_entities")
        if not client.wait_for_service(timeout_sec=5.0):
            node.get_logger().error("GetEntities service not available")
            return

        request = GetEntities.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        result = future.result()

        if result.result.success:
            node.get_logger().info(f"Entities ({len(result.names)}): {list(result.names)}")
        else:
            node.get_logger().error(f"Failed: {result.result.message}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

**Step 4: step_simulation.py**

```python
#!/usr/bin/env python3
"""Step the simulation via StepSimulation service.

Usage:
    python3 scripts/step_simulation.py --steps 100
    python3 scripts/step_simulation.py --pause
    python3 scripts/step_simulation.py --resume
"""

import argparse

import rclpy
from rclpy.node import Node
from simulation_interfaces.srv import StepSimulation, SetSimulationState, GetSimulationState
from simulation_interfaces.msg import SimulationState


def main():
    parser = argparse.ArgumentParser(description="Control simulation stepping")
    parser.add_argument("--steps", type=int, default=0, help="Number of steps to execute")
    parser.add_argument("--pause", action="store_true", help="Pause simulation")
    parser.add_argument("--resume", action="store_true", help="Resume simulation")
    parser.add_argument("--status", action="store_true", help="Get simulation state")
    args = parser.parse_args()

    rclpy.init()
    node = Node("step_simulation_client")

    if args.pause:
        client = node.create_client(SetSimulationState, "/sim/set_simulation_state")
        client.wait_for_service(timeout_sec=5.0)
        request = SetSimulationState.Request()
        request.state.state = SimulationState.PAUSED
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        node.get_logger().info("Simulation paused")

    elif args.resume:
        client = node.create_client(SetSimulationState, "/sim/set_simulation_state")
        client.wait_for_service(timeout_sec=5.0)
        request = SetSimulationState.Request()
        request.state.state = SimulationState.PLAYING
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        node.get_logger().info("Simulation resumed")

    elif args.status:
        client = node.create_client(GetSimulationState, "/sim/get_simulation_state")
        client.wait_for_service(timeout_sec=5.0)
        request = GetSimulationState.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        result = future.result()
        state_name = "PAUSED" if result.state.state == SimulationState.PAUSED else "PLAYING"
        node.get_logger().info(f"Simulation state: {state_name}")

    elif args.steps > 0:
        client = node.create_client(StepSimulation, "/sim/step_simulation")
        client.wait_for_service(timeout_sec=5.0)
        request = StepSimulation.Request()
        request.num_steps = args.steps
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        node.get_logger().info(f"Executed {args.steps} simulation steps")

    else:
        node.get_logger().info("No action specified. Use --steps, --pause, --resume, or --status")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

**Step 5: send_nav_goal.py (placeholder — nav action deferred)**

```python
#!/usr/bin/env python3
"""Send a NavigateToPose goal to a robot (Phase 2 — requires nav action server).

Usage:
    python3 scripts/send_nav_goal.py --robot robot_0 --x 5.0 --y 5.0

NOTE: This requires the NavigateToPose action server which is deferred to Phase 2.
For now, use cmd_vel or set_entity_state service to move robots.
"""

import argparse
import sys


def main():
    parser = argparse.ArgumentParser(description="Send NavigateToPose goal (Phase 2)")
    parser.add_argument("--robot", default="robot_0")
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    args = parser.parse_args()

    print(f"NavigateToPose action server is deferred to Phase 2.")
    print(f"For now, use cmd_vel or SetEntityState service:")
    print(f"  python3 scripts/teleop_cmd_vel.py --robot {args.robot} --vx 1.0 --duration 5")
    print(f'  ros2 service call /sim/set_entity_state simulation_interfaces/srv/SetEntityState '
          f'"{{name: \'{args.robot}\', state: {{pose: {{position: {{x: {args.x}, y: {args.y}, z: 0.05}}}}}}}}"')


if __name__ == "__main__":
    main()
```

**Step 6: Commit**

```bash
git add ros2_bridge/scripts/
git commit -m "feat(ros2): add standalone ROS client scripts (teleop, spawn, query, step, nav_goal)"
```

---

## Deferred Tasks (Phase 2)

The following tasks are deferred from Phase 1. They should be implemented after the core bridge is stable:

### Deferred: Nav2 + Arm Action Servers
- Add `NavigateToPose`, `FollowPath` to RobotHandler._setup_nav_actions
- Add `FollowJointTrajectory` to RobotHandler._setup_joint_actions
- Requires polling action completion with ROS executor yield

### Deferred: Humble Dockerfile + CI
- Create `docker/Dockerfile.humble` (same as Jazzy but `ros:humble-ros-base`, `ros-humble-*`)
- Add `humble` service to docker-compose.yaml
- CI matrix: `{jazzy, humble}` × `{test, integration}`

### Deferred: AgentManagerROSWrapper
- Wraps `AgentManager` for fleet-level batch operations
- Batch cmd_vel, batch spawn, fleet status topics
- Depends on Open-RMF integration planning

---

## Checklist Before Claiming Complete

- [ ] `make verify` passes (lint + tests, for core Agent changes)
- [ ] Docker image builds: `cd docker && docker compose build bridge`
- [ ] Integration test passes: `cd docker && docker compose run --rm test`
- [ ] `/clock` publishes advancing sim time
- [ ] `cmd_vel` → `OmniOmniVelocityController.set_velocity()` → robot moves
- [ ] All 14 simulation_interfaces services respond
- [ ] Client scripts work inside Docker container
