# Tutorial 2: High-Level Actions — Pick, Drop, Move, Wait

**Source file:** [`examples/action_system_demo.py`](https://github.com/yuokamoto/PyBulletFleet/blob/main/examples/action_system_demo.py)

This tutorial introduces PyBulletFleet's **action queue system**: a way to give an agent
an ordered list of tasks — pick up a pallet, carry it to a drop-off, drive to a charging
station, wait — and let the simulation execute them automatically.

**What you'll learn:**

- The difference between `set_goal_pose` and the action queue
- Spawning pickable objects with `SimObjectSpawnParams`
- The four built-in action types: `MoveAction`, `PickAction`, `DropAction`, `WaitAction`
- Queueing a task sequence with `add_action_sequence`
- Monitoring progress via `get_current_action`

If you haven't read [Tutorial 1 — Spawning Objects](spawning-objects) yet,
start there to understand `SimulationParams`, `Agent`, and `register_callback`.

---

## 1. Why Use Actions Instead of `set_goal_pose`?

`set_goal_pose` is great for a single navigation goal, but it has no concept of sequencing.
If you set a new goal before the previous one is reached, the old goal is discarded.

The action queue solves this:

| | `set_goal_pose` | Action queue |
|---|---|---|
| Multiple steps | ❌ — one goal at a time | ✅ — ordered task list |
| Mixed task types | ❌ — navigation only | ✅ — move, pick, drop, wait |
| State tracking | ❌ | ✅ — each action has `status` |
| Re-use / replay | ❌ | ✅ — actions are reset-able |

---

## 2. Setup

```python
from pybullet_fleet.agent import Agent, AgentSpawnParams, MotionMode
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.geometry import Pose, Path
from pybullet_fleet.sim_object import SimObject, SimObjectSpawnParams, ShapeParams
from pybullet_fleet.action import MoveAction, WaitAction, PickAction, DropAction

params = SimulationParams(gui=True, timestep=0.1, target_rtf=3, physics=False)
sim = MultiRobotSimulationCore(params)
```

**`target_rtf=3`** — the simulation runs at most 3× real time, making the GUI easy to follow.
For maximum speed (benchmarking / headless) use `target_rtf=0`.

```python
import os
import numpy as np

urdf_path = os.path.join(os.path.dirname(__file__), "../robots/mobile_robot.urdf")

agent = Agent.from_params(
    AgentSpawnParams(
        urdf_path=urdf_path,
        initial_pose=Pose.from_xyz(0, 0, 0.3),
        motion_mode=MotionMode.DIFFERENTIAL,
        max_linear_vel=2.0,
        max_linear_accel=5.0,
        max_angular_vel=2.0,
        max_angular_accel=5.0,
        mass=0.0,  # kinematic — no physics forces
    ),
    sim_core=sim,
)

# Optional: visualise the planned path as a coloured line in the GUI
agent.path_visualize = True
agent.path_visualize_width = 3.0
```

> **`AgentSpawnParams` vs `Agent.from_urdf`:** `from_params` takes a dataclass, which is more
> convenient when you have many parameters or want to pass spawn configurations around.
> Internally, both routes go through the same code.

---

## 3. Spawn a Pickable Object

For `PickAction` to work, the target object must be created with `pickable=True`.
Use `SimObjectSpawnParams` + `SimObject.from_params` for this:

```python
pallet_params = SimObjectSpawnParams(
    visual_shape=ShapeParams(
        shape_type="mesh",
        mesh_path=os.path.join(os.path.dirname(__file__), "../mesh/11pallet.obj"),
        mesh_scale=[0.5, 0.5, 0.5],
        rgba_color=[0.8, 0.6, 0.4, 1.0],
    ),
    collision_shape=ShapeParams(shape_type="box", half_extents=[0.5, 0.4, 0.1]),
    initial_pose=Pose.from_euler(5, 0, 0.1, roll=np.pi / 2, pitch=0, yaw=0),
    mass=0.0,
    pickable=True,   # ← required for PickAction
)
pallet = SimObject.from_params(pallet_params, sim_core=sim)
```

**`Pose.from_euler` with roll/pitch/yaw** — the pallet is rotated 90° around the X axis
(`roll=np.pi/2`) so it lies flat (horizontal) rather than standing upright. Adding a rotation
at spawn time is often easier than editing the mesh or URDF.

---

## 4. MoveAction — Navigate Along a Path

Use `MoveAction` when you want the agent to follow a path as part of a task sequence:

```python
path_to_charge = Path.from_positions([[2.5, 2.5, 0.3], [0, 0, 0.3]])
move_to_charger = MoveAction(
    path=path_to_charge,
    final_orientation_align=False,  # don't rotate at the end
)
```

`Path.from_positions` is a convenience factory that creates a `Path` from a list of
`[x, y, z]` positions with identity orientations. For full orientation control, build
`Pose` objects directly and pass `Path(waypoints=[...])`.

> For a single-step navigation goal outside an action sequence, `agent.set_goal_pose(pose)`
> is simpler. Use `MoveAction` when it is one step among several.

---

## 5. PickAction — Approach and Attach

`PickAction` moves the agent to a pickup approach pose, then attaches the target object
to the robot as if it were physically holding it:

```python
pick = PickAction(
    target_object_id=pallet.body_id,   # which object to pick
    use_approach=True,                  # move to an approach waypoint first
    approach_offset=1.5,                # approach from 1.5 m away
    pick_offset=1.0,                    # final pick position: 1.0 m from object centre
    attach_relative_pose=Pose.from_euler(
        0.6, 0, -0.2,                  # attach point: 0.6 m forward, 0.2 m below robot centre
        roll=np.pi / 2, pitch=0, yaw=0
    ),
)
```

**What happens step by step:**

1. Agent navigates to the **approach waypoint** (auto-calculated from target position + `approach_offset`).
2. Agent navigates to the **pick pose** (`pick_offset` from object).
3. The target object is kinematically **attached** to the agent at `attach_relative_pose`.
   It moves with the agent for all subsequent steps.

> `use_approach=True` is recommended when the agent may be coming from any direction —
> it inserts an intermediate waypoint that prevents the agent from colliding with the object
> during approach.

---

## 6. DropAction — Approach and Release

`DropAction` moves the agent to a drop-off area and detaches the currently held object:

```python
import pybullet as p

pallet_quat = p.getQuaternionFromEuler([np.pi / 2, 0, 0])  # horizontal orientation

drop = DropAction(
    drop_pose=Pose(position=[5, 5, 0.1], orientation=list(pallet_quat)),
    place_gently=True,      # set object velocity to 0 on release
    use_approach=True,
    approach_offset=1.5,
    drop_offset=1.0,
)
```

The `drop_pose` defines where the object is placed in the world after release —
not the agent's final position. The `orientation` in `drop_pose` sets the object's
orientation when placed, which is why we pass the same `pallet_quat` used at spawn time.

```{tip}
For EE-attached objects (arm robots, mobile manipulators), `drop_relative_pose`
lets you drop relative to the object's current position instead of an absolute
world coordinate.  See [Tutorial 5 §6 — drop_relative_pose](arm-ee-control) for details.
```

---

## 7. WaitAction — Pause for a Fixed Duration

`WaitAction` keeps the agent stationary for a given number of simulated seconds.
Use it to simulate charging, loading time, or synchronisation delays:

```python
charge = WaitAction(
    duration=3.0,           # simulated seconds to wait
    action_type="charge",   # label (used in logs); options: "idle", "charge", "loading", "custom"
)
```

> Time here is **simulated time**, not wall-clock time. With `target_rtf=3`, a 3-second
> `WaitAction` completes in ~1 second of real time.

---

## 8. Queue the Task Sequence

Pass all actions as a list to `add_action_sequence`. They execute one after another,
each starting only when the previous one reports completion:

```python
agent.add_action_sequence([pick, drop, move_to_charger, charge])
print(f"Queued: {agent.get_action_queue_size()} actions")  # → 4
```

> **Repeated tasks:** After all actions complete, the queue is empty and the agent becomes
> idle. To repeat the sequence, call `add_action_sequence` again (e.g., from a callback
> that detects the queue is empty).

---

## 9. Monitor Action Progress

Use a callback to display or react to the current action state:

```python
last_print = [0.0]  # list trick: allows mutation inside a nested function

def status_callback(sim_core, dt):
    if sim_core.sim_time - last_print[0] < 2.0:
        return
    last_print[0] = sim_core.sim_time

    current = agent.get_current_action()
    if current:
        print(f"[t={sim_core.sim_time:.1f}s] "
              f"{current.__class__.__name__} — {current.status.value} "
              f"(queue: {agent.get_action_queue_size()})")
    else:
        print(f"[t={sim_core.sim_time:.1f}s] All tasks completed")

sim.register_callback(status_callback, frequency=None)
```

**Useful methods:**

| Method | Returns |
|--------|---------|
| `agent.get_current_action()` | The currently executing `Action`, or `None` |
| `agent.get_action_queue_size()` | Number of actions remaining (including current) |
| `action.status.value` | `"not_started"`, `"in_progress"`, or `"completed"` |

---

## 10. Run

```python
sim.run_simulation()
```

The expected sequence you'll see in the GUI:

1. Agent approaches pallet → picks it up (pallet moves with agent)
2. Agent carries pallet to drop-off zone → releases it
3. Agent navigates to charging station
4. Agent waits in place for 3 simulated seconds
5. Console prints "All tasks completed"

---

## Full Example

```bash
python examples/action_system_demo.py
```

---

## See Also

- [Tutorial 1 — Spawning Objects](spawning-objects): SimObject, Agent, Pose, callbacks
- [Tutorial 3 — Managing a Fleet](multi-robot-fleet): apply action queues to 100+ robots via `add_action_sequence_all`
- [API Reference](../api/index): full Action, Agent, and SimObject class documentation
