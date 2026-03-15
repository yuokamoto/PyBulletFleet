# Examples

Three step-by-step tutorials that walk through the core PyBulletFleet APIs.
Each tutorial is based on a runnable script in `examples/` — you can open the file,
run it, and follow along in the docs at the same time.

## Which tutorial should I read?

| I want to… | Start here |
|---|---|
| Spawn objects in a scene, get/set poses, and move a single robot | [Tutorial 1 — Spawning Objects](spawning-objects) |
| Execute a sequence of high-level tasks (pick, drop, move, charge) | [Tutorial 2 — Action System](action-system) |
| Simulate 100 robots at once with a manager and per-robot paths | [Tutorial 3 — Managing a Fleet](multi-robot-fleet) |
| Simulate a robot arm picking and dropping objects | [Tutorial 4 — Arm Pick & Drop](arm-pick-drop) |

```{toctree}
:maxdepth: 1

spawning-objects
action-system
multi-robot-fleet
arm-pick-drop
```

## API Quick-Reference

| API | Covered in |
|-----|-----------|
| `SimObject.from_mesh` / `from_params` | Tutorial 1 |
| `Agent.from_mesh` / `from_urdf` / `from_params` | Tutorials 1, 4 |
| `Pose.from_xyz` / `from_euler` | Tutorials 1–4 |
| `agent.get_pose()` / `set_pose()` | Tutorial 1 |
| `agent.set_goal_pose()` | Tutorial 1 |
| `agent.set_path()` | Tutorial 3 |
| `sim_core.register_callback()` | Tutorials 1–3 |
| `MoveAction` / `PickAction` / `DropAction` / `WaitAction` | Tutorials 2, 4 |
| `JointAction` | Tutorial 4 |
| `agent.add_action_sequence()` | Tutorials 2, 4 |
| `set_all_joints_targets` / `set_joint_target` | Tutorials 1, 4 |
| `attach_object` with `parent_link_index` | Tutorial 4 |
| `AgentManager` / `GridSpawnParams` | Tutorial 3 |
| `manager.spawn_agents_grid_mixed()` | Tutorial 3 |
