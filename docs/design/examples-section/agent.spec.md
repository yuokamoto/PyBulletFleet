# Examples Section — Agent Specification

## Requirements

1. Three tutorial pages under `docs/examples/` (plus `index.md`)
2. Tutorial style: prose before each code block (what/why), prose after (key points/gotchas)
3. Each page must be independently readable — minimal assumed prior knowledge
4. Cross-references to other example pages and API docs where relevant
5. `docs/index.md` toctree and "How to read" table updated

## Constraints

- Do NOT edit any file under `examples/` — source files are read-only reference
- Code shown in docs can be excerpts (not full file dumps), but must accurately reflect what's in the source
- Use `python` fenced code blocks, not `{code-block}` directives (MyST plain markdown)

## File Locations

| File | Purpose |
|------|---------|
| `docs/examples/index.md` | Section index with toctree |
| `docs/examples/spawning-objects.md` | Tutorial 1 |
| `docs/examples/action-system.md` | Tutorial 2 |
| `docs/examples/multi-robot-fleet.md` | Tutorial 3 |

## Source Files to Read Before Writing

- `examples/robot_demo.py` — Tutorial 1 source
- `examples/action_system_demo.py` — Tutorial 2 source
- `examples/100robots_cube_patrol_demo.py` — Tutorial 3 source
- `examples/path_following_demo.py` — Reference for set_path with waypoints
- `pybullet_fleet/agent.py` — Agent class for accurate method signatures
- `pybullet_fleet/sim_object.py` — SimObject class for accurate method signatures
- `pybullet_fleet/agent_manager.py` — AgentManager for accurate method signatures
- `pybullet_fleet/geometry.py` — Path, Pose for accurate method signatures
- `pybullet_fleet/action.py` — Action classes for accurate method signatures
- `docs/getting-started/quickstart.md` — Avoid duplicating this

---

## Page 1: `docs/examples/spawning-objects.md`

### Title: Spawning Objects and Controlling Agents

### Sections (in order)

**1. Overview box** (2-3 sentences): what this tutorial teaches, which script it uses, link to source.

**2. Setup: SimulationParams and MultiRobotSimulationCore**
Code:
```python
params = SimulationParams(gui=True, timestep=0.01, physics=True)
sim_core = MultiRobotSimulationCore(params)
```
Explain: `timestep` controls sim resolution; `physics=True` enables joint dynamics (needed for arm).
For pure kinematics (mobile robots only) use `physics=False` for better performance.

**3. Spawning SimObjects**

3a. From mesh (`SimObject.from_mesh`):
```python
pallet_sim = SimObject.from_mesh(
    visual_shape=ShapeParams(shape_type="mesh", mesh_path=..., mesh_scale=[0.5,0.5,0.5], rgba_color=[...]),
    collision_shape=ShapeParams(shape_type="box", half_extents=[0.5, 0.4, 0.1]),
    pose=Pose.from_xyz(-2, 0, 0.1),
    sim_core=sim_core,
)
```
Explain: `ShapeParams` decouples visual from collision geometry. `Pose.from_xyz` for position-only placement.

3b. Wrapping an existing PyBullet body (`SimObject(body_id=...)`):
```python
box_body = p.createMultiBody(...)
box_sim = SimObject(body_id=box_body, sim_core=sim_core)
```
Explain: useful when integrating with existing PyBullet code or using shapes not yet in ShapeParams.

**4. Spawning Agents**

4a. From mesh (`Agent.from_mesh`) — mobile robot without URDF:
```python
cube_agent = Agent.from_mesh(
    visual_shape=ShapeParams(...),
    collision_shape=ShapeParams(...),
    pose=Pose.from_xyz(0, 0, 0.5),
    max_linear_vel=1.5,
    sim_core=sim_core,
)
```

4b. From URDF (`Agent.from_urdf`) — articulated robot:
```python
arm_agent = Agent.from_urdf(
    urdf_path="robots/arm_robot.urdf",
    pose=Pose.from_xyz(3, 0, 0),
    use_fixed_base=True,
    sim_core=sim_core,
)
```
Explain: `use_fixed_base=True` for arms/fixed infrastructure. `from_urdf` automatically imports joint structure.

**5. Reading and Writing Pose**
```python
current_pose = cube_agent.get_pose()
print(current_pose.position)   # [x, y, z]
print(current_pose.orientation) # quaternion [x, y, z, w]

cube_agent.set_pose(Pose.from_xyz(1, 2, 0.5))  # teleport immediately
```
Also show `Pose.from_euler(x, y, z, roll, pitch, yaw)` variant.

**6. Setting a Goal Pose**
```python
goal = Pose.from_xyz(3, 2, 0.5)
cube_agent.set_goal_pose(goal)
```
Explain: agent moves toward the goal over multiple steps using its kinematic model;
velocity/acceleration limits defined at spawn time via `max_linear_vel`, `max_linear_accel`.

**7. Registering Callbacks**
```python
def cube_random_movement_callback(sim_core, dt):
    if sim_core.step_count % 10 == 0:
        new_goal = Pose.from_xyz(np.random.uniform(-3, 4), np.random.uniform(-2, 4), 0.5)
        cube_agent.set_goal_pose(new_goal)

sim_core.register_callback(cube_random_movement_callback, frequency=10)
```
Explain: `frequency=10` means the callback fires at most 10 times per simulated second.
`frequency=None` fires every step. `sim_core.step_count` and `sim_core.sim_time` are available.

**8. Joint Control (arm)**
```python
def arm_joint_control_callback(sim_core, dt):
    t = sim_core.sim_time
    targets = [0.5 * np.sin(t + j * 0.5) for j in range(arm_agent.get_num_joints())]
    arm_agent.set_all_joints_targets(targets)
```
Explain: `get_num_joints()` returns number of controllable joints from URDF.

**9. Running**
```python
sim_core.run_simulation()
```
Explain: blocks until window closed or Ctrl+C. Keyboard shortcuts (SPACE, v, c, t) recap.

**10. See Also**
- `examples/path_following_demo.py` — `set_path()` with multiple waypoints
- Tutorial 2: Action System — for sequenced, high-level tasks
- `docs/api/` — full Agent and SimObject API reference

---

## Page 2: `docs/examples/action-system.md`

### Title: High-Level Actions: Pick, Drop, Move, Wait

### Sections (in order)

**1. Overview** — explain the difference between `set_goal_pose` (single goal, no sequencing)
and the action queue (ordered tasks with type-specific logic).

**2. Setup** — brief SimulationParams + agent spawn (DIFFERENTIAL, mass=0.0 for kinematic).

**3. Spawning Pickable Objects**
```python
pallet_params = SimObjectSpawnParams(
    visual_shape=...,
    collision_shape=...,
    initial_pose=Pose.from_euler(5, 0, 0.1, roll=np.pi/2, pitch=0, yaw=0),
    mass=0.0,
    pickable=True,   # ← enables pick/drop interaction
)
pallet = SimObject.from_params(pallet_params, sim_core=sim)
```
Explain: `pickable=True` required for PickAction to work. `mass=0.0` keeps it kinematic.

**4. MoveAction — navigate along a path**
```python
from pybullet_fleet.geometry import Path
path = Path.from_positions([[2.5, 2.5, 0.3], [0, 0, 0.3]])
move = MoveAction(path=path, final_orientation_align=False)
```

**5. PickAction — approach and attach**
```python
pick = PickAction(
    target_object_id=pallet.body_id,
    use_approach=True,
    approach_offset=1.5,
    pick_offset=1.0,
    attach_relative_pose=Pose.from_euler(0.6, 0, -0.2, roll=np.pi/2, pitch=0, yaw=0),
)
```
Explain: `use_approach=True` inserts an intermediate approach waypoint automatically.
`attach_relative_pose` defines where the object sits relative to the robot after pickup.

**6. DropAction — approach and release**
```python
drop = DropAction(
    drop_pose=Pose(position=[5,5,0.1], orientation=[...]),
    place_gently=True,
    use_approach=True,
    approach_offset=1.5,
    drop_offset=1.0,
)
```

**7. WaitAction — pause for a fixed duration**
```python
wait = WaitAction(duration=3.0, action_type="charge")
```
Explain: useful for simulating recharging, loading delays, human interaction time.

**8. Queueing and Running**
```python
agent.add_action_sequence([pick, drop, move, wait])
print(agent.get_action_queue_size())  # 4
```

**9. Monitoring action state via callback**
```python
def status_callback(sim_core, dt):
    action = agent.get_current_action()
    if action:
        print(f"{action.__class__.__name__} — {action.status.value}")

sim.register_callback(status_callback, frequency=None)
```

**10. Running**
```python
sim.run_simulation()
```

**11. See Also**
- Tutorial 1 (spawn, pose, callback basics)
- Tutorial 3 (fleet scale with actions)
- `docs/api/action` — full Action API

---

## Page 3: `docs/examples/multi-robot-fleet.md`

### Title: Managing a 100-Robot Fleet

### Sections (in order)

**1. Overview** — when to use AgentManager vs spawning manually;
scales to 1000+ agents; covers config from YAML.

**2. Loading Config from YAML**
```python
params = SimulationParams.from_config("config/100robots_config.yaml")
sim = MultiRobotSimulationCore(params)
```
Explain: externalising parameters lets you swap configs for benchmarking without code changes.

**3. Creating AgentManager**
```python
from pybullet_fleet.agent_manager import AgentManager
manager = AgentManager(sim_core=sim)
```

**4. GridSpawnParams — batch spawn in a grid**
```python
from pybullet_fleet.agent_manager import GridSpawnParams
grid = GridSpawnParams(
    x_min=0, x_max=9,
    y_min=0, y_max=9,
    spacing=[10.0, 10.0, 0.0],
    offset=[-15.0, -15.0, 0.3],
)
```
Explain: `x_max=9` means 10 columns (0..9); total = (x_max-x_min+1) × (y_max-y_min+1).

**5. Mixed-type batch spawn**
```python
spawn_omni = AgentSpawnParams(motion_mode=MotionMode.OMNIDIRECTIONAL, ...)
spawn_diff = AgentSpawnParams(motion_mode=MotionMode.DIFFERENTIAL, ...)

manager.spawn_agents_grid_mixed(
    num_agents=100,
    grid_params=grid,
    spawn_params_list=[
        (spawn_omni, 0.5),  # 50% omnidirectional
        (spawn_diff, 0.5),  # 50% differential
    ],
)
```
Explain: probabilities must sum to 1.0. Use `manager.objects` to iterate spawned agents.

**6. Building a Path and calling set_path**
```python
from pybullet_fleet.geometry import Path, Pose

waypoints = [
    Pose.from_euler(x, y, z, roll=0, pitch=0, yaw=0),
    # ... more waypoints
]
path = Path(waypoints=waypoints)

robot.set_path(path.waypoints)                             # omnidirectional
robot.set_path(path.waypoints, direction=MovementDirection.FORWARD)  # differential
```
Explain: `set_path()` loops continuously. `MovementDirection.BACKWARD` is for differential
robots that travel in reverse (facing away from the target direction).

**7. Iterating over all agents**
```python
for robot in manager.objects:
    pose = robot.get_pose()
    robot.set_path(build_path_for(pose.position))
```
Also show bulk velocity read:
```python
speeds = [np.linalg.norm(r.velocity) for r in manager.objects if r.is_moving]
avg = np.mean(speeds) if speeds else 0.0
```

**8. Monitoring callback**
```python
def monitor(sim_core, dt):
    moving = sum(1 for r in manager.objects if r.is_moving)
    print(f"[{sim_core.sim_time:.1f}s] Moving: {moving}/100")

sim.register_callback(monitor, frequency=None)
```

**9. Running**
```python
sim.run_simulation()
```

**10. Performance note**: at 100 agents with `physics=False`, expect ~40× RTF.
Link to Benchmark Results page.

**11. See Also**
- Tutorial 1 (single-agent basics)
- Tutorial 2 (action queue)
- [Benchmark Results](../benchmarking/results)
- [Optimization Guide](../benchmarking/optimization-guide)

---

## docs/examples/index.md structure

```markdown
# Examples

Three step-by-step tutorials covering the core APIs.

## Which tutorial should I read?

| I want to... | Start here |
|---|---|
| Spawn objects and move a single robot | Tutorial 1 |
| Execute pick/drop/charge task sequences | Tutorial 2 |
| Simulate 100+ robots at once | Tutorial 3 |

{toctree}
:maxdepth: 1

spawning-objects
action-system
multi-robot-fleet
```

---

## docs/index.md changes

1. Add `examples/index` to toctree between `getting-started/quickstart` and `architecture/index`
2. Add row to "How to read" table:
   | **Examples** | Step-by-step tutorials: spawn, pose, actions, fleet management | First simulation you write |

## Success Criteria

- [ ] `sphinx-build -b html . _build/html` exits 0 with no warnings for new pages
- [ ] All three pages render with correct code highlighting
- [ ] All cross-links resolve (no broken references in Sphinx output)
- [ ] `manager.objects`, `agent.get_pose()`, `Pose.from_euler`, `Path(waypoints=...)`,
      `add_action_sequence`, `GridSpawnParams`, `register_callback` all shown with accurate signatures
