# Examples

Six tutorials that walk through the core PyBulletFleet APIs.
Each tutorial is based on a runnable script in `examples/` — you can open the file,
run it, and follow along in the docs at the same time.

```{tip}
Tutorials are numbered for reference, **not as a required reading order**.
Jump directly to the topic you need — each tutorial lists its prerequisites
at the top so you can backtrack only when necessary.
```

## Which tutorial should I read?

| I want to… | Start here |
|---|---|
| Spawn objects in a scene, get/set poses, and move a single robot | [Tutorial 1 — Spawning Objects](spawning-objects) |
| Execute a sequence of high-level tasks (pick, drop, move, charge) | [Tutorial 2 — Action System](action-system) |
| Simulate 100 robots at once with a manager and per-robot paths | [Tutorial 3 — Managing a Fleet](multi-robot-fleet) |
| Simulate a robot arm picking and dropping objects | [Tutorial 4 — Arm Pick & Drop](arm-pick-drop) |
| Control an arm by end-effector position (IK) | [Tutorial 5 — EE Control & IK](arm-ee-control) |
| Use IK on a mobile manipulator (base + arm) | [Tutorial 5 §6 — Mobile Manipulator IK](arm-ee-control) |
| Load a robot by name from multiple sources | [Tutorial 6 — Robot Models](robot-models) |

```{toctree}
:maxdepth: 1
:hidden:

spawning-objects
action-system
multi-robot-fleet
arm-pick-drop
arm-ee-control
robot-models
```

## API Quick-Reference

| API | Covered in |
|-----|-----------|
| `SimObject.from_mesh` / `from_params` | Tutorial 1 |
| `Agent.from_mesh` / `from_urdf` / `from_params` | Tutorials 1, 4, 6 |
| `resolve_urdf` / `list_all_models` / `discover_models` / `auto_detect_profile` | Tutorial 6 |
| `register_model` / `unregister_model` / `add_search_path` | Tutorial 6 |
| `Pose.from_xyz` / `from_euler` | Tutorials 1–4 |
| `agent.get_pose()` / `set_pose()` | Tutorial 1 |
| `agent.set_goal_pose()` | Tutorial 1 |
| `agent.set_path()` | Tutorial 3 |
| `sim_core.register_callback()` | Tutorials 1–3 |
| `MoveAction` / `PickAction` / `DropAction` / `WaitAction` | Tutorials 2, 4 |
| `JointAction` | Tutorial 4 |
| `JointAction` tolerance (scalar, dict, agent-level) | Tutorial 4 |
| `PoseAction` | Tutorial 5 |
| `agent.move_end_effector()` | Tutorial 5 |
| `IKParams` | Tutorial 5 |
| `IKParams(ik_joint_names=...)` | Tutorial 5 |
| `DropAction(drop_relative_pose=...)` | Tutorials 2, 5 |
| `PickAction(ee_target_position=...)` / `DropAction(ee_target_position=...)` | Tutorial 5 |
| `agent.add_action_sequence()` | Tutorials 2, 4, 5 |
| `Agent.joint_tolerance` / per-joint tolerance | Tutorial 4 |
| `set_all_joints_targets` / `set_joint_target` | Tutorials 1, 4 |
| `attach_object` with `parent_link_index` | Tutorial 4 |
| `AgentManager` / `GridSpawnParams` | Tutorial 3 |
| `manager.spawn_agents_grid_mixed()` | Tutorial 3 |
| `SimulationParams.enable_floor` | Tutorial 1 |
