# Tutorial 3: Managing a 100-Robot Fleet

<table width="100%">
<tr>
<td align="center"><b>Mixed Fleet Grid</b><br><small>100robots_grid_demo.py</small><br>
<video src="../100robots_grid_mixed.mp4" width="100%" autoplay loop muted playsinline></video></td>
<td align="center"><b>Cube Patrol</b><br><small>100robots_cube_patrol_demo.py</small><br>
<video src="../100robots_cube_patrol.mp4" width="100%" autoplay loop muted playsinline></video></td>
</tr>
<tr>
<td align="center"><b>Mobile Pick & Drop</b><br><small>pick_drop_mobile_100robots_demo.py</small><br>
<video src="../pick_drop_mobile_100robots.mp4" width="100%" autoplay loop muted playsinline></video></td>
<td align="center"><b>Arm Pick & Drop</b><br><small>pick_drop_arm_100robots_demo.py</small><br>
<video src="../pick_drop_arm_100robots.mp4" width="100%" autoplay loop muted playsinline></video></td>
</tr>
</table>

**Source file:** [`examples/scale/100robots_cube_patrol_demo.py`](https://github.com/yuokamoto/PyBulletFleet/blob/main/examples/scale/100robots_cube_patrol_demo.py)

This tutorial scales up from a single agent to a fleet of 100 robots.
You will learn how to:

- Load simulation parameters from a YAML config file
- Create and use `AgentManager` to manage many agents as a group
- Batch-spawn 100 robots in a grid with mixed types using `GridSpawnParams`
- Build `Path` objects with waypoints and assign them with `set_path`
- Iterate over all agents to read state or assign individual paths
- Write an efficient monitoring callback for large fleets

After this tutorial, combine what you've learned with
[Tutorial 2 — Action System](action-system) to send each agent an action sequence
via `manager.add_action_sequence_all`.

---

## 1. When to Use AgentManager

| | Direct `Agent` objects | `AgentManager` |
|---|---|---|
| Fleet size | 1–10 | 10–10,000 |
| Bulk ops | Manual loops | `set_goal_pose_all`, `set_pose_all`, `add_action_sequence_all` |
| Grid spawn | Manual | `spawn_agents_grid`, `spawn_agents_grid_mixed` |
| Iteration | `agent.get_pose()` per agent | `manager.objects` generator |

`AgentManager` is a thin wrapper — each item in `manager.objects` is a regular `Agent`
and supports all the same methods from Tutorial 1 and 2.

---

## 2. Load Config from YAML

For larger simulations, externalise parameters to a YAML file
so you can change settings (number of agents, speed limits, collision mode)
without editing Python code:

```python
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams

params = SimulationParams.from_config("config/100robots_config.yaml")
sim = MultiRobotSimulationCore(params)
```

`from_config` reads the YAML and maps every key to the corresponding `SimulationParams` field.
Any parameter not present in the file gets its default value.

> The config files live in `config/`. See [Configuration Reference](../configuration/index)
> for the full parameter list.

---

## 3. Create an AgentManager

```python
from pybullet_fleet.agent_manager import AgentManager

manager = AgentManager(sim_core=sim)
```

`AgentManager` does not spawn any agents on its own — it manages agents that are
added to it, either via `spawn_agents_grid` / `spawn_agents_grid_mixed`
or by manually calling `manager.add_agent(agent)`.

---

## 4. Define a Grid Layout with GridSpawnParams

`GridSpawnParams` describes a regular grid of positions:

```python
from pybullet_fleet.agent_manager import GridSpawnParams

grid = GridSpawnParams(
    x_min=0,
    x_max=9,          # 10 columns: indices 0, 1, …, 9
    y_min=0,
    y_max=9,          # 10 rows:    indices 0, 1, …, 9
    spacing=[10.0, 10.0, 0.0],   # 10 m between agents in X and Y
    offset=[-15.0, -15.0, 0.3],  # world-frame offset of the grid origin
)
```

Total spawned = `(x_max - x_min + 1) × (y_max - y_min + 1)`.
With the above settings: 10 × 10 = 100 agents.

The actual world position of grid cell `(i, j)` is:

```
x = offset[0] + i * spacing[0]
y = offset[1] + j * spacing[1]
z = offset[2]
```

---

## 5. Define Agent Types

Define `AgentSpawnParams` for each robot type you want in the fleet.
Here we use two: omnidirectional and differential drive.

```python
from pybullet_fleet.agent import AgentSpawnParams, MotionMode
import os

urdf_path = os.path.abspath("robots/mobile_robot.urdf")

spawn_omni = AgentSpawnParams(
    urdf_path=urdf_path,
    mass=0.0,                            # kinematic (teleport-based, no physics)
    max_linear_vel=2.0,                  # m/s
    max_linear_accel=1.0,                # m/s²
    max_angular_vel=2.0,                 # rad/s
    max_angular_accel=5.0,
    motion_mode=MotionMode.OMNIDIRECTIONAL,
    use_fixed_base=False,
)

spawn_diff = AgentSpawnParams(
    urdf_path=urdf_path,
    mass=0.0,
    max_linear_vel=2.0,
    max_linear_accel=1.0,
    max_angular_vel=2.0,
    max_angular_accel=5.0,
    motion_mode=MotionMode.DIFFERENTIAL,
    use_fixed_base=False,
)
```

**`mass=0.0` (kinematic mode)** is the key performance choice. Kinematic control
teleports each robot to its next pose without calling `stepSimulation()` — this is
why PyBulletFleet can run 100 robots at 40× real time. See the
[Benchmark Results](../benchmarking/results) for measured throughput.

---

## 6. Spawn 100 Agents in a Mixed Grid

`spawn_agents_grid_mixed` takes a list of `(SpawnParams, probability)` tuples.
Probabilities must sum to 1.0:

```python
manager.spawn_agents_grid_mixed(
    num_agents=100,
    grid_params=grid,
    spawn_params_list=[
        (spawn_omni, 0.5),   # 50% omnidirectional
        (spawn_diff, 0.5),   # 50% differential
    ],
)

print(f"Spawned: {len(manager.objects)} agents")
```

After spawning, each agent is accessible through `manager.objects`:

> **Note:** The snippets below are not in the demo script. They show
> additional usage patterns you can use in your own code.

```python
for agent in manager.objects:
    print(agent.motion_mode, agent.get_pose().position)
```

For a uniform grid (one type only), use `spawn_agents_grid` instead:

```python
manager.spawn_agents_grid(
    num_agents=100,
    grid_params=grid,
    spawn_params=spawn_omni,
)
```

> `spawn_agents_grid` is used in
> `examples/scale/pick_drop_mobile_100robots_demo.py`, which also demonstrates
> `SimObjectManager` for batch-spawning pickable objects alongside agents.

---

## 7. Build a Path and Assign it to an Agent

A `Path` is a sequence of `Pose` waypoints the agent follows in order,
looping back to the start when it reaches the end.

```python
from pybullet_fleet.geometry import Path, Pose

# Build waypoints using Pose.from_euler for full 6-DOF control
waypoints = [
    Pose.from_euler(x, y, z, roll=0, pitch=0, yaw=0)
    for (x, y, z) in [
        [5, 5, 0.3],    # corner 1
        [-5, 5, 0.3],   # corner 2
        [-5, -5, 0.3],  # corner 3
        [5, -5, 0.3],   # corner 4
    ]
]
path = Path(waypoints=waypoints)

# Or build from positions only (orientation = identity on all waypoints).
# This shorthand is not used in the demo, but is handy for quick tests:
path = Path.from_positions([
    [5, 5, 0.3], [-5, 5, 0.3], [-5, -5, 0.3], [5, -5, 0.3]
])
```

Assign to an agent:

```python
# Omnidirectional robot (direction parameter ignored)
agent.set_path(path.waypoints)

# Differential drive — specify movement direction
from pybullet_fleet.agent import MovementDirection

agent.set_path(path.waypoints, direction=MovementDirection.FORWARD)
agent.set_path(path.waypoints, direction=MovementDirection.BACKWARD)
```

**`FORWARD`**: the robot faces toward the next waypoint (normal driving).
**`BACKWARD`**: the robot keeps its heading fixed and reverses toward the waypoint —
useful for robots that should always face a particular way (e.g., a forklift mast).

> **Tip:** Call `path.get_total_distance()` to inspect the path length before assigning,
> and `path.visualize(...)` to draw it in the GUI.

---

## 8. Assign Individual Paths to All 100 Agents

Here each robot patrols a cube centred on its own spawn position.
The key is getting each robot's spawn pose to build a per-robot path:

```python
for robot in manager.objects:
    spawn_pos = robot.get_pose().position    # current (= spawn) position

    # Build a path centred at this robot's position
    cx, cy = spawn_pos[0], spawn_pos[1]
    half = 2.5
    patrol_waypoints = [
        Pose.from_euler(cx + half, cy + half, 0.3, roll=0, pitch=0, yaw=0),
        Pose.from_euler(cx - half, cy + half, 0.3, roll=0, pitch=0, yaw=0),
        Pose.from_euler(cx - half, cy - half, 0.3, roll=0, pitch=0, yaw=0),
        Pose.from_euler(cx + half, cy - half, 0.3, roll=0, pitch=0, yaw=0),
    ]

    if robot.motion_mode == MotionMode.DIFFERENTIAL:
        import random
        direction = random.choice([MovementDirection.FORWARD, MovementDirection.BACKWARD])
        robot.set_path(patrol_waypoints, direction=direction)
    else:
        robot.set_path(patrol_waypoints)
```

This pattern — iterate `manager.objects`, get pose, compute per-robot data, call
single-agent API — is the standard way to initialize heterogeneous fleets.

---

## 9. Bulk Operations with the Manager

`AgentManager` provides vectorised versions of the most common per-agent operations.
These are more readable (and slightly faster) than manual loops:

```python
# Set a random goal for every agent
import numpy as np

manager.set_goal_pose_all(
    lambda agent: Pose.from_xyz(
        np.random.uniform(-10, 10),
        np.random.uniform(-10, 10),
        0.3,
    )
)

# Get all poses at once
poses = manager.get_poses_dict()   # {agent_index: Pose}

# Teleport all agents to new poses
manager.set_pose_all(
    lambda agent: Pose.from_xyz(agent.get_pose().position[0], 0, 0.3)
)
```

All bulk methods accept a **factory callable** `(Agent) → result` that is called
once per agent.

> **Note:** These snippets are not in the demo script `100robots_cube_patrol_demo.py`.
> They show additional `AgentManager` APIs you can use in your own code.

---

## 10. Write a Monitoring Callback

For large fleets, avoid per-step prints (they dominate step time).
The `step_count` modulo pattern is efficient:

```python
def monitoring_callback(sim_core, dt):
    # Print only every 300 steps (≈ every 30 simulated seconds at timestep=0.1)
    if sim_core.step_count % 300 != 0:
        return

    moving = sum(1 for r in manager.objects if r.is_moving)
    speeds = [np.linalg.norm(r.velocity) for r in manager.objects if r.is_moving]
    avg_speed = np.mean(speeds) if speeds else 0.0

    print(
        f"[t={sim_core.sim_time:.0f}s] "
        f"Moving: {moving}/{len(manager.objects)} | "
        f"Avg speed: {avg_speed:.2f} m/s"
    )

sim.register_callback(monitoring_callback, frequency=None)
```

**`r.is_moving`** — `True` while the agent has a goal or path to follow.
**`r.velocity`** — current velocity vector `[vx, vy, vz]` in world frame.

> Printing every step at 100 agents adds ~5–10% overhead.
> Throttling to every 300 steps costs nothing measurable.

---

## 11. Camera Setup

For large grids, let the simulation auto-fit the camera to all spawned agents:

```python
sim.setup_camera()   # auto-scales to bounding box of all objects
```

Or pass an explicit config (the demo uses YAML config instead, but you can
override programmatically):

```python
sim.setup_camera(camera_config={
    "camera_mode": "manual",
    "camera_distance": 80.0,
    "camera_yaw": 45,
    "camera_pitch": -35,
    "camera_target": [0, 0, 0],
})
```

---

## 12. Run

```python
sim.run_simulation()
```

At 100 agents with `physics=False`, you should see ~40× RTF (≈ 2.4 ms per step).
See [Benchmark Results](../benchmarking/results) for the full throughput table.

### Scale Demos

All four scale demo scripts live in `examples/scale/`:

| Script | What it demonstrates |
|--------|---------------------|
| [`100robots_cube_patrol_demo.py`](https://github.com/yuokamoto/PyBulletFleet/blob/main/examples/scale/100robots_cube_patrol_demo.py) | 100 mobile robots patrolling cube paths (this tutorial) |
| [`100robots_grid_demo.py`](https://github.com/yuokamoto/PyBulletFleet/blob/main/examples/scale/100robots_grid_demo.py) | Mixed fleet (mobile + arm) in a grid with `--mode mixed\|single` |
| [`pick_drop_mobile_100robots_demo.py`](https://github.com/yuokamoto/PyBulletFleet/blob/main/examples/scale/pick_drop_mobile_100robots_demo.py) | 100 mobile robots with pick/drop action sequences and `SimObjectManager` |
| [`pick_drop_arm_100robots_demo.py`](https://github.com/yuokamoto/PyBulletFleet/blob/main/examples/scale/pick_drop_arm_100robots_demo.py) | 100 fixed-base arms with `JointAction` pick/drop cycles |

```bash
python examples/scale/100robots_cube_patrol_demo.py
python examples/scale/100robots_grid_demo.py
python examples/scale/pick_drop_mobile_100robots_demo.py
python examples/scale/pick_drop_arm_100robots_demo.py
```

### Switching Robot Models

All scale demos accept a `--robot` argument to swap the robot model at runtime.
Pass a model name resolved by `resolve_urdf()` or a direct URDF path:

```bash
# Mobile demos — use mobile models
python examples/scale/100robots_cube_patrol_demo.py --robot racecar
python examples/scale/pick_drop_mobile_100robots_demo.py --robot mobile_robot

# Arm demo — use arm models
python examples/scale/pick_drop_arm_100robots_demo.py --robot kuka_iiwa

# Grid demo — has both mobile and arm robots
python examples/scale/100robots_grid_demo.py --robot racecar --arm-robot kuka_iiwa
```

| Script | Argument | Default | Alternatives |
|--------|----------|---------|-------------|
| `100robots_grid_demo.py` | `--robot` (mobile) | `husky` | `racecar`, `mobile_robot` |
| `100robots_grid_demo.py` | `--arm-robot` (arm) | `panda` | `kuka_iiwa`, `arm_robot` |
| `100robots_cube_patrol_demo.py` | `--robot` (mobile) | `husky` | `racecar`, `mobile_robot` |
| `pick_drop_mobile_100robots_demo.py` | `--robot` (mobile) | `husky` | `racecar`, `mobile_robot` |
| `pick_drop_arm_100robots_demo.py` | `--robot` (arm) | `panda` | `kuka_iiwa`, `arm_robot` |

See [Tutorial 6 — Robot Models](robot-models) for the full model resolution system
and `python examples/models/resolve_urdf_demo.py --list` for all available names.

---

## Performance Notes

- **`physics=False`** is the single most important setting for fleet-scale throughput.
  Physics stepping is O(n) even with kinematic control.
- **`collision_check_frequency`** — set to `1.0` or lower for offline use; `null` (every step)
  for real-time collision monitoring. See the [Optimization Guide](../benchmarking/optimization-guide).
- **Motion mode matters** — DIFFERENTIAL robots are ~5× more expensive to update than
  OMNIDIRECTIONAL due to heading alignment computation.
  Mixed fleets pay a weighted-average cost.

---

## See Also

- [Tutorial 1 — Spawning Objects](spawning-objects): single-agent basics
- [Tutorial 2 — Action System](action-system): `add_action_sequence_all` for fleet-scale tasks
- [Benchmark Results](../benchmarking/results): measured throughput at 100–2000 agents
- [Optimization Guide](../benchmarking/optimization-guide): tuning `collision_check_frequency`, motion mode, physics flag
- [Configuration Reference](../configuration/index): full YAML parameter list
