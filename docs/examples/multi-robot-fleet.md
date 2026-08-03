# Tutorial 3: Managing a 100-Robot Fleet

<table width="100%">
<tr>
<td align="center"><b>Cube Patrol</b><br><small>100robots_cube_patrol_demo.py</small><br>
<video src="../100robots_cube_patrol.mp4" width="100%" autoplay loop muted playsinline></video></td>
<td align="center"><b>Mixed Fleet Grid</b><br><small>100robots_mixed_demo.py</small><br>
<video src="../100robots_grid_mixed.mp4" width="100%" autoplay loop muted playsinline></video></td>
</tr>
<tr>
<td align="center"><b>Mobile Pick & Drop</b><br><small>pick_drop_mobile_100robots_demo.py</small><br>
<video src="../pick_drop_mobile_100robots.mp4" width="100%" autoplay loop muted playsinline></video></td>
<td align="center"><b>Arm Pick & Drop</b><br><small>pick_drop_arm_100robots_demo.py</small><br>
<video src="../pick_drop_arm_100robots.mp4" width="100%" autoplay loop muted playsinline></video></td>
</tr>
</table>

**Primary walkthrough:** [`100robots_cube_patrol_demo.py`](https://github.com/yuokamoto/PyBulletFleet/blob/main/pybullet_fleet/examples/scale/100robots_cube_patrol_demo.py)

**Related configuration-driven variants:** [`100robots_grid_demo.py`](https://github.com/yuokamoto/PyBulletFleet/blob/main/pybullet_fleet/examples/scale/100robots_grid_demo.py) and
[`100robots_mixed_demo.py`](https://github.com/yuokamoto/PyBulletFleet/blob/main/pybullet_fleet/examples/scale/100robots_mixed_demo.py)

This tutorial follows the Cube Patrol demo as it scales from one agent to two
50-robot fleets. Grid and Mixed demos are related variants that use the same
`from_dict()` configuration pattern with scene YAML files.
You will learn how to:

- Declare named managers with `managers:` config and route entity grids to them
- Select per-agent or batch execution with `fleet_controller.type`
- Set fleet-wide controller defaults alongside that execution choice
- Retrieve managers at runtime with `sim.get_manager()`
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

## 2. Two Fleet Concepts, Two Layers

The similar names describe different jobs. They can be combined, but none is a
replacement for another.

| Concept | Layer | Purpose | Typical use |
| --- | --- | --- | --- |
| `fleet_controller:` | Manager configuration | Selects batch execution with `type`, and supplies shared `ControllerParams` defaults | Choose `batch_omni` / `batch_differential` and set common limits |
| Fleet command interface | Command ingress API | Resolves names, validates a command, records an ack, then invokes public Agent operations | External clients, ROS/RMF adapters, or comparing command submission paths |

`fleet_controller.type` selects how a manager executes movement; omitting
`type` keeps per-agent execution. It does not define a command API; use the
fleet command interface when commands need name-based targeting and
acknowledgement semantics.

---

## 3. Load, Adapt, and Instantiate Config

For larger simulations, build the final configuration dictionary before
creating the core. Cube Patrol loads the bundled base YAML, adds its runtime
overrides and fleet definitions in Python, then calls `from_dict()`.
This abbreviated control-flow snippet names the definitions expanded in the
next section; use the linked runnable demo for the complete script.

```python
from pybullet_fleet.config_utils import load_yaml_config, merge_configs
from pybullet_fleet.core_simulation import MultiRobotSimulationCore

overrides = {"simulation": {"target_rtf": 5}}
cfg = merge_configs(load_yaml_config("config/config.yaml"), overrides)

# The following sections define these lists from CLI choices and resolved models.
cfg["managers"] = manager_definitions
cfg["entities"] = entity_definitions

sim = MultiRobotSimulationCore.from_dict(cfg)
```

This keeps runtime choices explicit and inspectable in Python. The Grid and
Mixed variants instead merge `config/config.yaml` with their selected
`--config` scene YAML, adjust the resulting dict, and also call `from_dict()`.

> The bundled config files live in `pybullet_fleet/config/`. See [Configuration Reference](../configuration/index)
> for the full parameter list.

`MultiRobotSimulationCore.from_yaml("my_scene.yaml")` remains convenient for a
completely static scenario, but it is not the initialization path used by these
scale demos.

---

## 4. Declare Named Managers in Config

The YAML/dict schema for a multi-fleet simulation has `managers:` and
`entities:` sections. Once passed to `from_dict()`, the simulation core creates
the managers, spawns the agents, and wires them together.

The following YAML shows the `managers:` / `entities:` schema. Cube Patrol
builds this same structure in its Python `cfg` dictionary:

```yaml
managers:
  - name: omni_fleet
    fleet_controller:
      type: batch_omni                 # omit type for per-agent execution
      max_linear_vel: 2.0
      max_linear_accel: 1.0
      max_angular_vel: 2.0
      max_angular_accel: 5.0

  - name: diff_fleet
    fleet_controller:
      type: batch_differential
      max_linear_vel: 2.0
      max_linear_accel: 1.0
      max_angular_vel: 2.0
      max_angular_accel: 5.0

entities:
  - urdf_path: "robots/mobile_robot.urdf"
    mass: 0.0
    use_fixed_base: false
    manager: omni_fleet          # route this group to omni_fleet
    grid:
      x_min: 0
      x_max: 4
      y_min: 0
      y_max: 9
      spacing: [10.0, 10.0, 0.0]
      offset: [-15.0, -15.0, 0.3]

  - urdf_path: "robots/mobile_robot.urdf"
    mass: 0.0
    use_fixed_base: false
    manager: diff_fleet          # route this group to diff_fleet
    grid:
      x_min: 5
      x_max: 9
      y_min: 0
      y_max: 9
      spacing: [10.0, 10.0, 0.0]
      offset: [-15.0, -15.0, 0.3]
```

Instantiate the composed dictionary and retrieve its managers:

```python
from pybullet_fleet.core_simulation import MultiRobotSimulationCore

sim = MultiRobotSimulationCore.from_dict(cfg)
omni_manager = sim.get_manager("omni_fleet")
diff_manager  = sim.get_manager("diff_fleet")
```

> **How Cube Patrol builds this schema:**
> `100robots_cube_patrol_demo.py` accepts a `--robot` argument that resolves the URDF
> path at runtime via `resolve_model()`. It then constructs `managers:` and
> `entities:` in a Python dict, conditionally enables the batch controllers from
> `--controller`, and passes the result to `from_dict()`. Treat it as the
> reference example for programmatically composing a config-driven fleet; the
> structure is identical to the YAML above.
>
> The runnable demo keeps an explicit entity `controller.type` even though this
> batch-focused YAML omits it: when `--controller per_agent` removes
> `fleet_controller.type`, that entity value selects the per-agent fallback.

### fleet_controller — execution type and shared parameter defaults

`fleet_controller.type` selects the manager's execution path:

| Value | Execution |
| --- | --- |
| omitted | Each Agent runs its own controller |
| `batch_omni` | Shared vectorised omnidirectional controller |
| `batch_differential` | Shared vectorised differential-drive controller |

The remaining `fleet_controller` fields set `ControllerParams` defaults for
the whole manager. Per-agent controller values take precedence for fields they
set explicitly.

See [Controller Config](../how-to/controller-config) for the full list of
`ControllerParams` fields.

With a batch `type`, one `BatchKinematicController` advances every agent in the
manager in a vectorised NumPy pass instead of calling `compute()` per agent.
The performance benefit depends on the
controller, movement workload, command path, and host environment; use it for
large homogeneous fleets to reduce per-agent Python dispatch, then measure the
target scenario. See [Benchmark Results](../benchmarking/results.md).
for the current reproducible comparison.

> Agents with different controller types must be in separate batch managers,
> because each batch type supports one motion model.

### Controller vs command interface

The Grid scale demo exposes two independent comparison axes. These are separate
from `fleet_controller:` configuration:

| Axis | Values | Meaning |
|------|--------|---------|
| Controller implementation | `per_agent`, `batch` | How robot motion is computed inside the simulation. |
| Command interface | `per_agent`, `fleet` | How goals are submitted to the simulation. |

`per_agent` controllers use each `Agent` controller. `batch` controllers use one
vectorised manager-level controller. `per_agent` command submission calls each
`Agent` directly, while `fleet` submits name-based commands through
`FleetCommandDispatcher`.

The fastest default is usually `--controller batch --command-interface fleet`.
Use `per_agent` command submission when you want to compare the API overhead
while keeping the scene and controller implementation unchanged.

```bash
# User-facing scale demo: repeated goal commands.
python examples/scale/100robots_grid_demo.py \
  --movement goal --controller batch --command-interface fleet --duration 10

python examples/scale/100robots_grid_demo.py \
  --movement goal --controller batch --command-interface per_agent --duration 10

# Focused setup-time and state-snapshot smoke. Robot count is configurable.
python examples/scale/batch_controller_scale_demo.py \
  --no-gui --duration 5 --robots 500 --command-interface fleet
```

`100robots_grid_demo.py` uses `config/100robots_config.yaml` for its standard
mobile-only scene. In `--movement goal` mode, it sends repeated goals to those
mobile robots. Use `100robots_mixed_demo.py` for the separate Husky + Panda
mixed scene; it selects a scene with `--config`, rather than per-model CLI
arguments.

### Alternative: Python API

If you prefer to construct managers imperatively (e.g. when URDF paths are resolved
at runtime):

```python
from pybullet_fleet.agent_manager import AgentManager, GridSpawnParams
from pybullet_fleet.agent import AgentSpawnParams

omni_manager = AgentManager(sim_core=sim, fleet_controller={"type": "batch_omni"})
omni_manager.spawn_agents_grid(
    num_agents=50,
    grid_params=GridSpawnParams(
        x_min=0, x_max=4, y_min=0, y_max=9,
        spacing=[10.0, 10.0, 0.0],
        offset=[-15.0, -15.0, 0.3],
    ),
    spawn_params=AgentSpawnParams(
        urdf_path=urdf_path, mass=0.0,
        use_fixed_base=False,
        controller={"type": "omni", "max_linear_vel": 2.0, "max_linear_accel": 1.0},
    ),
)
```

**`mass=0.0` (kinematic mode)** is the key performance choice. Kinematic control
teleports each robot to its next pose without calling `stepSimulation()` — this is
why PyBulletFleet can run 100 robots at 40× real time. See the
[Benchmark Results](../benchmarking/results) for measured throughput.

> `spawn_agents_grid` is used in
> `examples/scale/pick_drop_mobile_100robots_demo.py`, which also demonstrates
> `SimObjectManager` for batch-spawning pickable objects alongside agents.

---

## 5. Build a Path and Assign it to an Agent

A `Path` is a sequence of `Pose` waypoints the agent follows in order,
looping back to the start when it reaches the end.

This is a short API primer. The Cube Patrol-specific nine-waypoint path and
manager-specific assignment used by the runnable demo are shown in the next
section.

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

## 6. Assign Individual Paths to All 100 Agents

Cube Patrol gives every robot a 5 m × 5 m × 5 m path centred on its own spawn
position. The following is the same path construction and manager-specific
assignment flow as `100robots_cube_patrol_demo.py` (formatting only is
simplified):

```python
import random

from pybullet_fleet.agent import MovementDirection
from pybullet_fleet.geometry import Path, Pose


def create_cube_patrol_path(cube_center, cube_size=5.0):
    """Bottom XY circuit → climb → top XY circuit → descend."""
    half = cube_size / 2.0
    cx, cy, cz = cube_center
    z_bot = cz - half + 0.3
    z_top = cz + half
    corners = [
        [cx + half, cy + half, z_bot],
        [cx - half, cy + half, z_bot],
        [cx - half, cy - half, z_bot],
        [cx + half, cy - half, z_bot],
        [cx + half, cy - half, z_top],
        [cx + half, cy + half, z_top],
        [cx - half, cy + half, z_top],
        [cx - half, cy - half, z_top],
        [cx + half, cy - half, z_bot],
    ]
    return Path(waypoints=[Pose.from_euler(x, y, z, 0, 0, 0) for x, y, z in corners])


def cube_path_for(robot):
    spawn_pos = robot.get_pose().position    # current (= spawn) position
    path = create_cube_patrol_path([spawn_pos[0], spawn_pos[1], spawn_pos[2] + 2.5])
    path.visualize(
        show_lines=True,
        line_color=[0.5, 0.5, 0.5],
        line_width=1.0,
        show_waypoints=True,
        show_axes=False,
        show_points=False,
        lifetime=0,
    )
    return path

for robot in omni_manager.objects:
    robot.set_path(cube_path_for(robot).waypoints)

for robot in diff_manager.objects:
    direction = random.choice([MovementDirection.FORWARD, MovementDirection.BACKWARD])
    robot.set_path(cube_path_for(robot).waypoints, direction=direction)
```

This pattern — iterate `manager.objects`, get pose, compute per-robot data, call
single-agent API — is the standard way to initialize heterogeneous fleets.

---

## 7. Write a Monitoring Callback

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

## 8. Camera Setup

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

## 9. Run

```python
sim.run_simulation()
```

At 100 agents with `physics=False`, performance depends heavily on the host and
display stack. The 2026-07-24 WSL2 benchmark measured ~143× RTF
with the default batch controller + fleet command interface.
See [Benchmark Results](../benchmarking/results) for the full throughput table.

### Scale Demos

The scale demos in `examples/scale/` have different roles:

| Script | What it demonstrates |
|--------|---------------------|
| [`100robots_cube_patrol_demo.py`](https://github.com/yuokamoto/PyBulletFleet/blob/main/pybullet_fleet/examples/scale/100robots_cube_patrol_demo.py) | 100 mobile robots patrolling cube paths; programmatically composes `managers:`/`entities:` and uses `from_dict()` (this tutorial) |
| [`100robots_grid_demo.py`](https://github.com/yuokamoto/PyBulletFleet/blob/main/pybullet_fleet/examples/scale/100robots_grid_demo.py) | Mobile-only grid demo; merges YAML, applies controller/command choices, then uses `from_dict()` |
| [`100robots_mixed_demo.py`](https://github.com/yuokamoto/PyBulletFleet/blob/main/pybullet_fleet/examples/scale/100robots_mixed_demo.py) | Husky + Panda mixed-fleet grid scene; merges YAML, adjusts mobile controllers, then uses `from_dict()` |
| [`batch_controller_scale_demo.py`](https://github.com/yuokamoto/PyBulletFleet/blob/main/pybullet_fleet/examples/scale/batch_controller_scale_demo.py) | Focused batch-controller scale demo; `--robots` controls robot count and defaults to 500 |
| [`pick_drop_mobile_100robots_demo.py`](https://github.com/yuokamoto/PyBulletFleet/blob/main/pybullet_fleet/examples/scale/pick_drop_mobile_100robots_demo.py) | 100 mobile robots with pick/drop action sequences and `SimObjectManager` |
| [`pick_drop_arm_100robots_demo.py`](https://github.com/yuokamoto/PyBulletFleet/blob/main/pybullet_fleet/examples/scale/pick_drop_arm_100robots_demo.py) | 100 fixed-base arms with `JointAction` pick/drop cycles |

```bash
python examples/scale/100robots_cube_patrol_demo.py
python examples/scale/100robots_grid_demo.py
python examples/scale/100robots_mixed_demo.py
python examples/scale/batch_controller_scale_demo.py --no-gui --duration 5
python examples/scale/pick_drop_mobile_100robots_demo.py
python examples/scale/pick_drop_arm_100robots_demo.py
```

### Switching Robot Models

Most scale demos accept a `--robot` argument to swap the robot model at runtime.
Pass a model name resolved by `resolve_model()` or a direct URDF path:

```bash
# Mobile demos — use mobile models
python examples/scale/100robots_cube_patrol_demo.py --robot racecar
python examples/scale/batch_controller_scale_demo.py --robot racecar
python examples/scale/pick_drop_mobile_100robots_demo.py --robot mobile_robot

# Arm demo — use arm models
python examples/scale/pick_drop_arm_100robots_demo.py --robot kuka_iiwa

# Grid demos select entities through a config file instead of model arguments.
python examples/scale/100robots_grid_demo.py --robot racecar
python examples/scale/100robots_grid_demo.py --config config/100robots_config.yaml
python examples/scale/100robots_mixed_demo.py --mobile-robot racecar --arm-robot kuka_iiwa
python examples/scale/100robots_mixed_demo.py --config config/100robots_mixed_config.yaml
```

| Script | Argument | Default | Alternatives |
|--------|----------|---------|-------------|
| `100robots_grid_demo.py` | `--robot`, `--config` | `mobile_robot` grid | `husky`, `racecar`; another `entities[].grid` YAML scene |
| `100robots_mixed_demo.py` | `--mobile-robot`, `--arm-robot`, `--config` | Husky + Panda grid | `racecar`, `mobile_robot`; `kuka_iiwa`, `arm_robot`; another scene |
| `100robots_cube_patrol_demo.py` | `--robot` (mobile) | `husky` | `racecar`, `mobile_robot` |
| `batch_controller_scale_demo.py` | `--robot` (mobile) | `simple_cube` | `husky`, `racecar`, `mobile_robot` |
| `pick_drop_mobile_100robots_demo.py` | `--robot` (mobile) | `husky` | `racecar`, `mobile_robot` |
| `pick_drop_arm_100robots_demo.py` | `--robot` (arm) | `panda` | `kuka_iiwa`, `arm_robot` |

See [Tutorial 6 — Robot Models](robot-models) for the full model resolution system
and `python examples/models/resolve_model_demo.py --list` for all available names.

---

## Performance Notes

- **`physics=False`** is the single most important setting for fleet-scale throughput.
  Physics stepping is O(n) even with kinematic control.
- **`collision_check_frequency`** — set to `1.0` or lower for offline use; `null` (every step)
  for real-time collision monitoring. See the [Optimization Guide](../benchmarking/optimization-guide).
- **Controller type matters** — differential movement has additional
  heading-alignment work compared with omnidirectional movement. Measure the
representative mix on the target host; see [Benchmark Results](../benchmarking/results.md).

---

## See Also

- [Tutorial 1 — Spawning Objects](spawning-objects): single-agent basics
- [Tutorial 2 — Action System](action-system): action queues for fleet-scale tasks
- [Benchmark Results](../benchmarking/results): measured throughput at 100–2000 agents
- [Optimization Guide](../benchmarking/optimization-guide): tuning `collision_check_frequency`, motion mode, physics flag
- [Configuration Reference](../configuration/index): full YAML parameter list
