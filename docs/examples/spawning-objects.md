# Tutorial 1: Spawning Objects and Controlling Agents

**Source file:** [`examples/robot_demo.py`](https://github.com/yuokamoto/PyBulletFleet/blob/main/examples/robot_demo.py)

This tutorial walks through the fundamental building blocks of every PyBulletFleet simulation:

- Creating a simulation world (`SimulationParams`, `MultiRobotSimulationCore`)
- Spawning passive objects (`SimObject`)
- Spawning active agents (`Agent`) — from mesh, URDF, or raw PyBullet body
- Getting and setting poses (`get_pose`, `set_pose`)
- Setting a movement goal (`set_goal_pose`)
- Controlling arm joints (`set_all_joints_targets`)
- Running per-step logic with callbacks (`register_callback`)

After this tutorial you can move on to
[Tutorial 2 — Action System](action-system) for sequenced high-level tasks, or
[Tutorial 3 — Managing a Fleet](multi-robot-fleet) for 100+ robots.

---

## 1. Initialize the Simulation

Every simulation starts with two objects: `SimulationParams` and `MultiRobotSimulationCore`.

```python
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams

params = SimulationParams(gui=True, timestep=0.01, physics=True)
sim_core = MultiRobotSimulationCore(params)
```

**Key parameters:**

| Parameter | What it controls | Typical values |
|-----------|-----------------|----------------|
| `gui` | Open the PyBullet GUI window | `True` for dev, `False` for headless/benchmark |
| `timestep` | Seconds of simulation time per step | `0.1` (fast, kinematic); `0.01` (fine-grained, physics) |
| `physics` | Enable full PyBullet rigid-body dynamics | `True` for joints/contacts; `False` for pure kinematics |

> **Why `physics=True` here?** This demo includes an articulated arm robot that needs joint
> dynamics. For simulations with only mobile robots the default `physics=False` gives
> 5–10× better throughput — see the [Optimization Guide](../benchmarking/optimization-guide).

---

## 2. Spawn a SimObject from a Mesh

`SimObject` represents a passive object in the scene (a pallet, a shelf, a box).
The most flexible way to create one is `SimObject.from_mesh`, which lets you specify
the visual and collision geometry independently.

```python
import os
from pybullet_fleet.sim_object import SimObject, ShapeParams
from pybullet_fleet.geometry import Pose

pallet_mesh_path = os.path.abspath("mesh/11pallet.obj")

pallet = SimObject.from_mesh(
    visual_shape=ShapeParams(
        shape_type="mesh",
        mesh_path=pallet_mesh_path,
        mesh_scale=[0.5, 0.5, 0.5],
        rgba_color=[0.8, 0.6, 0.4, 1.0],
    ),
    collision_shape=ShapeParams(shape_type="box", half_extents=[0.5, 0.4, 0.1]),
    pose=Pose.from_xyz(-2, 0, 0.1),
    sim_core=sim_core,
)
```

**What's happening:**

- `ShapeParams(shape_type="mesh", ...)` defines a detailed visual appearance using a `.obj` file.
- `ShapeParams(shape_type="box", half_extents=[...])` defines a simpler box collision shape —
  this is intentional: collision geometry is cheaper than visual geometry, so they are kept separate.
- `Pose.from_xyz(x, y, z)` positions the object. Z is typically the height above the floor.

**`Pose` — the core position/orientation type:**

```python
# Position only (orientation = identity quaternion)
pose = Pose.from_xyz(x, y, z)

# Position + Euler angles (roll, pitch, yaw in radians)
pose = Pose.from_euler(x, y, z, roll=0, pitch=0, yaw=1.57)

# Direct construction (orientation as quaternion [x, y, z, w])
pose = Pose(position=[x, y, z], orientation=[0, 0, 0, 1])
```

---

## 3. Wrap an Existing PyBullet Body

If you already have a PyBullet body ID — for example from `p.createMultiBody` or
from loading a URDF directly — you can wrap it in a `SimObject` to get the same
`get_pose` / `set_pose` interface:

```python
import pybullet as p

box_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.3, 0.3, 0.3])
box_visual    = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.3, 0.3, 0.3],
                                    rgbaColor=[1.0, 0.0, 0.0, 1.0])
box_body      = p.createMultiBody(0.0, box_collision, box_visual, [-2, 2, 0.3])

box_sim = SimObject(body_id=box_body, sim_core=sim_core)
```

> This is useful when integrating PyBulletFleet into an existing PyBullet project,
> or when you need a geometry type (e.g., `GEOM_CYLINDER`) that `ShapeParams` does not yet cover.

---

## 4. Spawn an Agent from a Mesh

`Agent` extends `SimObject` with kinematics: velocity limits, goal-following, path-following,
and (optionally) an action queue. Use `Agent.from_mesh` when you want a custom visual shape
but don't have a URDF:

```python
from pybullet_fleet.agent import Agent

cube_mesh_path = os.path.abspath("mesh/cube.obj")

cube_agent = Agent.from_mesh(
    visual_shape=ShapeParams(
        shape_type="mesh",
        mesh_path=cube_mesh_path,
        mesh_scale=[0.3, 0.3, 0.3],
        rgba_color=[0.0, 1.0, 0.0, 1.0],
    ),
    collision_shape=ShapeParams(shape_type="box", half_extents=[0.3, 0.3, 0.3]),
    pose=Pose.from_xyz(0, 0, 0.5),
    max_linear_vel=1.5,     # m/s
    sim_core=sim_core,
)
```

**Key Agent parameters:**

| Parameter | Description |
|-----------|-------------|
| `max_linear_vel` | Maximum speed in m/s |
| `max_linear_accel` | Acceleration limit in m/s² (smooth ramp-up) |
| `max_angular_vel` | Maximum rotation speed in rad/s (differential drive) |
| `motion_mode` | `MotionMode.OMNIDIRECTIONAL` (default) or `MotionMode.DIFFERENTIAL` |
| `mass` | `0.0` for kinematic (teleport-based), `> 0` for physics-driven |

---

## 5. Spawn an Agent from a URDF

Use `Agent.from_urdf` for articulated robots (arms, mobile manipulators) whose joint
structure is defined in a URDF file:

```python
arm_agent = Agent.from_urdf(
    urdf_path=os.path.join(os.path.dirname(__file__), "../robots/arm_robot.urdf"),
    pose=Pose.from_xyz(3, 0, 0),
    use_fixed_base=True,   # mount the base to the world
    sim_core=sim_core,
)
```

**`use_fixed_base=True`** — the robot's base link is pinned to the world frame.
Use this for fixed arms, cameras on stands, or any infrastructure that shouldn't slide.
Leave it `False` for mobile robots.

> **Tip:** `from_urdf` automatically discovers all joints in the URDF.
> Call `arm_agent.get_num_joints()` to find out how many controllable joints were loaded.

---

## 6. Getting and Setting Poses

All `SimObject` and `Agent` instances expose `get_pose()` and `set_pose()`:

> **Note:** The snippets below are not in `robot_demo.py`. They show API usage
> you can add to your own scripts.

```python
# Get current pose
pose = cube_agent.get_pose()
print(pose.position)    # [x, y, z] as a list
print(pose.orientation) # quaternion [x, y, z, w]

# Teleport to a new pose immediately
cube_agent.set_pose(Pose.from_xyz(1, 2, 0.5))
```

> **`set_pose` vs `set_goal_pose`:**
> `set_pose` teleports the object instantly — useful for resetting state or discrete
> event-driven placement. `set_goal_pose` (see next section) makes the agent move there
> smoothly under its velocity and acceleration limits.

---

## 7. Setting a Goal Pose (Smooth Movement)

`set_goal_pose` tells the agent to navigate to a target pose over multiple simulation steps,
respecting its velocity and acceleration limits:

> **Note:** The standalone call below is not in `robot_demo.py` (the demo uses
> `set_goal_pose` inside a callback in Section 8).

```python
goal = Pose.from_xyz(3, 2, 0.5)
cube_agent.set_goal_pose(goal)
```

The agent moves toward the goal on each call to `sim_core.step()` (or automatically inside
`run_simulation()`). Once it arrives, it stays there until a new goal is set.

> For following a sequence of waypoints through a space, use `set_path()` instead.
> See [Tutorial 3 — Managing a Fleet](multi-robot-fleet) for a worked example.

---

## 8. Periodic Logic with Callbacks

Callbacks are the main hook for per-step simulation logic.
Register a function with `register_callback` and it will be called automatically
during `run_simulation()`:

```python
import numpy as np

def cube_random_movement_callback(sim_core, dt):
    """Assign a new random goal every 10 simulated steps."""
    if sim_core.step_count % 10 == 0:
        x = np.random.uniform(-3, 4)
        y = np.random.uniform(-2, 4)
        cube_agent.set_goal_pose(Pose.from_xyz(x, y, 0.5))

sim_core.register_callback(cube_random_movement_callback, frequency=10)
```

**Callback signature:** `callback(sim_core, dt)` — always two arguments.

**`frequency` parameter:**

| Value | When the callback fires |
|-------|------------------------|
| `10` | At most 10 times per simulated second |
| `1` | Once per simulated second |
| `None` | Every simulation step (no throttling) |

**Useful `sim_core` attributes inside a callback:**

```python
sim_core.step_count   # int — total steps since start
sim_core.sim_time     # float — total simulated seconds
```

---

## 9. Controlling Arm Joints

For URDF-loaded agents with joints, use `set_all_joints_targets` to set target angles
for all controllable joints at once:

```python
def arm_joint_control_callback(sim_core, dt):
    """Sinusoidal sweep of all arm joints."""
    t = sim_core.sim_time
    num_joints = arm_agent.get_num_joints()
    targets = [0.5 * np.sin(t + j * 0.5) for j in range(num_joints)]
    arm_agent.set_all_joints_targets(targets)

sim_core.register_callback(arm_joint_control_callback, frequency=10)
```

- `get_num_joints()` — returns the number of controllable joints (as found in the URDF).
- `set_all_joints_targets(positions, max_force=500.0)` — sets position targets for joints
  in joint-index order. Joint control requires `physics=True` in `SimulationParams`.

---

## 10. Running the Simulation

```python
sim_core.run_simulation()
```

This blocks until the GUI window is closed or `Ctrl+C` is pressed.
All registered callbacks fire automatically each step.

**GUI keyboard shortcuts** (when `gui=True`):

| Key | Action |
|-----|--------|
| `Space` | Pause / Resume |
| `v` | Toggle visual shapes |
| `c` | Toggle collision wireframes |

---

## Full Example

The complete script is at `examples/robot_demo.py`.
Run it from the project root:

```bash
python examples/robot_demo.py
```

---

## See Also

- [Tutorial 2 — Action System](action-system): sequence Pick, Drop, Move, and Wait tasks
- [Tutorial 3 — Managing a Fleet](multi-robot-fleet): `AgentManager`, grid spawn, `set_path`
- [`examples/path_following_demo.py`](https://github.com/yuokamoto/PyBulletFleet/blob/main/examples/path_following_demo.py): detailed `set_path` demo with 2D and 3D waypoints
- [API Reference](../api/index): full Agent, SimObject, and related class documentation
