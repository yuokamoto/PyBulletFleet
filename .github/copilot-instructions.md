# PyBulletFleet — Copilot Instructions

Kinematics-first PyBullet simulation framework for large-scale multi-robot fleets.
Targets 100+ robots at real-time or faster. Python ≥ 3.10, PyBullet backend.

**Design philosophy:** Speed over fidelity. System integration over low-level control.
Physics is optional — default mode teleports robots kinematically (no `stepSimulation()`).

## Architecture

```
User Application
    │
    ▼
MultiRobotSimulationCore  (core_simulation.py)
    ├── Agent (agent.py)           — URDF, navigation, action queue, joint control
    ├── SimObject (sim_object.py)  — Single rigid body, pose, attachments
    ├── AgentManager               — Batch spawn, goal management
    ├── Action System (action.py)  — MoveAction, PickAction, DropAction, WaitAction, JointAction
    ├── Tools (tools.py)           — Offset pose calculation utilities
    ├── CollisionVisualizer        — Collision pair rendering
    └── DataMonitor                — Real-time FPS/step-time display
```

| Component | File | Key Class |
|-----------|------|-----------|
| Sim Core | `core_simulation.py` | `MultiRobotSimulationCore`, `SimulationParams` |
| Agent | `agent.py` | `Agent`, `AgentSpawnParams`, `IKParams` |
| SimObject | `sim_object.py` | `SimObject` |
| Actions | `action.py` | `MoveAction`, `PickAction`, `DropAction`, `WaitAction`, `JointAction` |
| Types | `types.py` | `CollisionMode`, `MotionMode`, `ActionStatus`, `CollisionDetectionMethod` |
| Geometry | `geometry.py` | `Pose`, `Path` |
| Config | `config_utils.py` | YAML loading with merge |

## Code Conventions

- **Formatting:** black, line-length 127. Run `make format` to auto-fix.
- **Type checking:** pyright in basic mode. Relaxed for PyBullet's dynamic API.
- **Linting:** flake8 (line-length 127).
- **Pre-commit:** black + pyright + flake8 + general hooks. Run `make lint`.
- **Logging:** Always `get_lazy_logger(__name__)` from `pybullet_fleet.logging_utils`. Never `logging.getLogger()` or `print()`.
- **Factory methods:** Always use `from_params()`, `from_yaml()`, `from_dict()`, `from_urdf()`, `from_mesh()`. Never call `__init__()` directly on Agent, SimObject, or SimulationParams.
- **Shared shape cache:** `SimObject._shared_shapes` caches PyBullet mesh shape IDs. Must clear between tests (autouse fixture handles this).
- **Import guards:** Use `if TYPE_CHECKING:` for imports that cause circular dependencies (common between action.py ↔ agent.py).

## Testing Rules

- **Always `p.DIRECT`** — never `p.GUI` in tests. Headless CI cannot open windows.
- **Headless params:** `SimulationParams(gui=False, monitor=False)`.
- **Factory setup:** Use `Agent.from_params(spawn_params, sim_core)` in tests.
- **Coverage:** `--cov-fail-under=75`. Run `make test` for full CI-equivalent test run.
- **Strict markers:** `--strict-markers --strict-config` enforced.
- **Autouse fixtures** in `tests/conftest.py`:
  - `_clear_shared_shapes` — clears shape cache (stale IDs after `p.disconnect()`)
  - `_disable_monitor_gui` — patches `DataMonitor.start` to no-op
- **MockSimCore** in conftest — lightweight sim_core with `tick(n)` for stepping.
- **arm_sim fixture** — parametrized (physics / kinematic / physics_off) for arm tests.

## Guard Rails — DO NOT

1. **Never call `p.stepSimulation()` when `physics: false`** — kinematic mode teleports objects; physics stepping breaks positions.
2. **Never use `p.GUI` in tests** — always `p.DIRECT`.
3. **Never mutate `_agents` or `_objects` lists directly** — use `add_object()` / `remove_object()`.
4. **Never skip `_shared_shapes` cleanup** — stale shape IDs crash after `p.disconnect()`.
5. **Never import without `TYPE_CHECKING` guard** when it would cause circular deps.
6. **Never use `logging.getLogger()`** — use `get_lazy_logger(__name__)`.
7. **Never call `__init__()` directly** on Agent, SimObject, SimulationParams — use factory methods.
8. **Always run `make verify` before claiming work is done.**

## Common Patterns

### Spawn and run

```python
from pybullet_fleet import (
    MultiRobotSimulationCore, SimulationParams,
    Agent, AgentSpawnParams, Pose, MotionMode,
)
from pybullet_fleet.types import CollisionMode

sim = MultiRobotSimulationCore.from_yaml("config/config.yaml")
params = AgentSpawnParams(
    urdf_path="robots/mobile_robot.urdf",
    initial_pose=Pose.from_xyz(0, 0, 0.1),
    motion_mode=MotionMode.OMNIDIRECTIONAL,
    max_linear_vel=2.0,
    collision_mode=CollisionMode.NORMAL_2D,
)
agent = Agent.from_params(params, sim)
sim.register_callback(my_callback, frequency=1.0)
sim.run_simulation()
```

### Action lifecycle

```python
from pybullet_fleet.action import MoveAction
from pybullet_fleet.geometry import Path

action = MoveAction(path=Path.from_points([[5, 5, 0]]))
agent.add_action(action)
# Status: NOT_STARTED → IN_PROGRESS → COMPLETED / FAILED / CANCELLED
while agent.update(dt):
    pass
```

### IK-controlled arm

```python
params = AgentSpawnParams(
    urdf_path="robots/arm_robot.urdf",
    ik_params=IKParams(ee_link_name="end_effector"),
)
agent = Agent.from_params(params, sim)
agent.set_ee_target((0.3, 0.0, 0.4))  # IK solves joint angles
```

## Entry Points

| Command | Purpose |
|---------|---------|
| `make verify` | **Full CI-equivalent check** (lint + test) |
| `make test` | Tests with coverage (75% threshold) |
| `make test-fast` | Quick test (stop on first failure) |
| `make lint` | All pre-commit hooks |
| `make format` | Auto-format with black |
| `make typecheck` | Pyright type check |
| `make docs` | Sphinx docs (warnings = errors) |
| `make bench-smoke` | Quick benchmark (~10s) |
| `make bench-full` | Full benchmark sweep |
| `make clean` | Remove caches and build artifacts |
| `make help` | List all targets |
