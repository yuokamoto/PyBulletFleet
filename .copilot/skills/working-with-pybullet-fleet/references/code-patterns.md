# Code Patterns Reference

Detailed code patterns for PyBulletFleet development. Read when you need specific implementation examples beyond the quick reference in SKILL.md.

## Table of Contents

1. [Factory Method Pattern](#factory-method-pattern)
2. [Configuration Pattern](#configuration-pattern)
3. [Agent Navigation Pattern](#agent-navigation-pattern)
4. [Action System Pattern](#action-system-pattern)
5. [Pick/Drop Pattern](#pickdrop-pattern)
6. [Joint Control Pattern](#joint-control-pattern)
7. [Callback Pattern](#callback-pattern)
8. [Demo Script Pattern](#demo-script-pattern)
9. [Discovering Current Patterns](#discovering-current-patterns)

## Factory Method Pattern

All major classes use `from_*` factory methods. Never call `__init__` directly.

```python
# Pattern: from_dict / from_config / from_params
# Config loading chain:
config = load_config("config.yaml")     # config_utils.py
params = SimulationParams.from_dict(config)
sim = MultiRobotSimulationCore(params)

# Or shorthand:
sim = MultiRobotSimulationCore.from_yaml("config.yaml")
sim = MultiRobotSimulationCore.from_dict(config_dict)
```

SimObject/Agent auto-register to sim_core in `__init__`:
```python
# SimObject.__init__ calls sim_core.add_object(self)
# No manual registration needed
obj = SimObject.from_params(spawn_params, sim_core)  # auto-registered
```

## Configuration Pattern

Config is YAML → dict → dataclass:

```python
# from_dict maps string keys to dataclass fields
# Unknown keys are silently ignored
# Special handling: enums from strings, nested dicts

# SimulationParams auto-selects collision method:
#   physics=False → CLOSEST_POINTS
#   physics=True  → CONTACT_POINTS
```

Layered config:
```python
from pybullet_fleet.config_utils import load_config
config = load_config("base.yaml")  # single file
# Override specific values:
config["physics"] = True
params = SimulationParams.from_dict(config)
```

## Agent Navigation Pattern

```python
# Single goal
agent.set_goal_pose(Pose.from_xyz(5, 5, 0))

# Multi-waypoint path
path = Path.from_points([[1,1,0], [3,3,0], [5,5,0]])
agent.set_path(path, auto_approach=True)

# Check status
if not agent.is_moving:
    agent.set_goal_pose(next_goal)

# Stop movement
agent.stop()
```

## Action System Pattern

Actions are queued and processed in `agent.update(dt)`:

```python
from pybullet_fleet.action import MoveAction, WaitAction, PickAction, DropAction

# Queue actions
agent.add_action(MoveAction(path=Path.from_points([[5,5,0]])))
agent.add_action(WaitAction(duration=1.0))

# Or as sequence
agent.add_action_sequence([
    MoveAction(path=Path.from_points([[5,5,0]])),
    WaitAction(duration=1.0),
    MoveAction(path=Path.from_points([[0,0,0]])),
])

# Monitor
action = agent.get_current_action()
if action and action.is_complete():
    print("Done!")
```

## Pick/Drop Pattern

```python
# Pick with approach
pick = PickAction(
    target_object_id=box.object_id,
    target_position=box.get_pose().position,
    use_approach=True,
    approach_offset=[0, 0, 0.3],  # 30cm above
    pick_offset=[0, 0, 0.01],
)

# Drop at location
drop = DropAction(
    drop_pose=Pose.from_xyz(5, 5, 0.1),
    use_approach=True,
    approach_offset=[0, 0, 0.3],
)

agent.add_action_sequence([pick, drop])
```

## Joint Control Pattern

For URDF robots with joints:

```python
# By index
agent.set_joint_target(0, target_position=1.57)

# By name
agent.set_joint_target_by_name("joint_1", 1.57)

# Multiple joints at once
agent.set_joints_targets_by_name({
    "joint_1": 1.57,
    "joint_2": -0.5,
    "joint_3": 0.0,
})

# Check convergence
if agent.are_joints_at_targets_by_name({"joint_1": 1.57}, tolerance=0.01):
    print("Joint reached target")

# JointAction for action queue
from pybullet_fleet.action import JointAction
action = JointAction(
    target_joint_positions={"joint_1": 1.57, "joint_2": -0.5},
    tolerance=0.01,
)
agent.add_action(action)
```

## Callback Pattern

```python
# Global callback (called by sim loop)
def my_update(sim_core):
    for agent in sim_core.agents:
        if not agent.is_moving:
            agent.set_goal_pose(random_pose())

sim.register_callback(my_update, frequency=1.0)  # 1 Hz

# Agent-level callback
def agent_callback(agent):
    print(f"{agent.name} at {agent.get_pose()}")

agent.register_callback(agent_callback, frequency=0.5)
```

## Demo Script Pattern

Standard structure for examples:

```python
#!/usr/bin/env python3
"""Demo description."""

from pybullet_fleet import (
    MultiRobotSimulationCore, SimulationParams,
    Agent, AgentSpawnParams, AgentManager, GridSpawnParams,
    Pose, Path,
)
from pybullet_fleet.types import CollisionMode, MotionMode

# 1. Config
params = SimulationParams(gui=True, physics=False, speed=1.0)
sim = MultiRobotSimulationCore(params)

# 2. Spawn
spawn_params = AgentSpawnParams(urdf_path="robots/mobile_robot.urdf", ...)
manager = AgentManager(sim)
agents = manager.spawn_agents_grid(
    num_objects=10,
    grid_params=GridSpawnParams(spacing=[2, 2, 0]),
    spawn_params=spawn_params,
)

# 3. Callback
def update(sim_core):
    ...

sim.register_callback(update, frequency=1.0)

# 4. Run
sim.run_simulation()
```

## Discovering Current Patterns

When patterns may have evolved, search the source:

```bash
# Find all factory methods
grep -rn "def from_" pybullet_fleet/

# Find all Action subclasses
grep -rn "class.*Action.*:" pybullet_fleet/action.py

# Find all enums
grep -rn "class.*Enum" pybullet_fleet/types.py

# Find config parameter handling
grep -rn "from_dict" pybullet_fleet/core_simulation.py

# Find test fixtures
grep -rn "@pytest.fixture" tests/conftest.py

# Find callback registration
grep -rn "register_callback" pybullet_fleet/

# Find public API exports
cat pybullet_fleet/__init__.py
```
