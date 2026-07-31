# Examples

Nine tutorials and runnable companion examples that walk through the core
PyBulletFleet APIs.
Each tutorial is based on a runnable script in `pybullet_fleet/examples/` — you
can run it from the installed package or copy it to `examples/` before editing.

```{tip}
Tutorials are numbered for reference, **not as a required reading order**.
Jump directly to the topic you need — each tutorial lists its prerequisites
at the top so you can backtrack only when necessary.
```

```{toctree}
:maxdepth: 1
:hidden:

spawning-objects
action-system
multi-robot-fleet
arm-pick-drop
arm-ee-control
robot-models
sdf-mesh-loading
event-bus
plugins
```

## Tutorials

| # | Tutorial | Script | What you'll learn |
|---|----------|--------|-------------------|
| 1 | [Spawning Objects](spawning-objects) | `basics/robot_demo.py` | Create sim, spawn agents & objects, `set_goal_pose()` |
| 2 | [Action System](action-system) | `basics/action_system_demo.py` | MoveAction, PickAction, DropAction lifecycle |
| 3 | [Multi-Robot Fleet](multi-robot-fleet) | `scale/100robots_cube_patrol_demo.py` | AgentManager, grid spawn, 100+ robots |
| 4 | [Arm Pick & Drop](arm-pick-drop) | `arm/pick_drop_arm_demo.py` | Joint-space IK, JointAction, tolerance tuning |
| 5 | [Arm EE Control](arm-ee-control) | `arm/pick_drop_arm_ee_demo.py` | End-effector IK, PoseAction, mobile manipulators |
| 6 | [Robot Models](robot-models) | `models/resolve_model_demo.py` | `resolve_model`, model registry, `robot_descriptions` |
| 7 | [SDF and Mesh Assets](sdf-mesh-loading) | `models/sdf_demo.py` | Multi-model SDF and bulk mesh loading |
| 8 | [Event Bus](event-bus) | `basics/event_bus_demo.py` | EventBus, lifecycle and custom events |
| 9 | [Plugins](plugins) | `basics/plugin_demo.py` | SimPlugin lifecycle and AgentPlugin configuration |

## Run bundled examples

Examples are included in the PyPI wheel. After `pip install pybullet-fleet`,
you can run a demo without cloning the repository:

```bash
pybullet-fleet examples --list
pybullet-fleet examples --run batch_controller_scale_demo.py -- --no-gui --duration 10
pybullet-fleet examples --copy ./examples
```

Use `--copy` before editing a demo. The source-checkout commands below remain
useful once examples have been copied or when developing the repository.

## Switching Robot Models with `--robot`

Most demo scripts accept a `--robot` argument to swap the robot model at runtime.
Pass a model name (resolved via `resolve_model()`) or a direct URDF path.
The model should be compatible with the demo category — arm models for arm demos,
mobile models for mobile demos:

```bash
# Arm demos — pass arm models (default: panda)
python examples/arm/pick_drop_arm_demo.py --robot kuka_iiwa

# Mobile demos — pass mobile models (default: husky)
python examples/mobile/path_following_demo.py --robot racecar

# Scale demos
python examples/scale/100robots_cube_patrol_demo.py --robot mobile_robot
python examples/scale/pick_drop_arm_100robots_demo.py --robot kuka_iiwa

# The grid demo is mobile-only. Select another entities[].grid scene with --config.
python examples/scale/100robots_grid_demo.py --config config/100robots_config.yaml
```

| Category | Scripts | Argument | Default | Alternatives |
|----------|---------|----------|---------|-------------|
| Arm demos | `examples/arm/pick_drop_arm_*.py`, `rail_arm_demo.py` | `--robot` | `panda` | `kuka_iiwa`, `arm_robot` |
| Mobile demos | `examples/mobile/path_following_demo.py` | `--robot` | `husky` | `racecar`, `mobile_robot` |
| Scale (mobile) | `100robots_cube_patrol_demo.py`, `pick_drop_mobile_100robots_demo.py` | `--robot` | `husky` | `racecar`, `mobile_robot` |
| Scale (arm) | `pick_drop_arm_100robots_demo.py` | `--robot` | `panda` | `kuka_iiwa`, `arm_robot` |
| Grid demo | `100robots_grid_demo.py` | `--config` | mobile-only grid | another `entities[].grid` YAML scene |
| Mixed grid demo | `100robots_mixed_demo.py` | `--config` | Husky + Panda | another mixed `entities[].grid` YAML scene |
| Model demos | `resolve_model_demo.py` | `--robot` | `panda` | any registered model |
| Model demos | `robot_descriptions_demo.py` | `--robot` | `tiago` | any `robot_descriptions` model |

Model names are resolved by `resolve_model()` — see [Tutorial 6 — Robot Models](robot-models)
for the full resolution system and `python examples/models/resolve_model_demo.py --list`
for all available names.

## API Quick-Reference

| API | Covered in |
|-----|-----------|
| `SimObject.from_mesh` / `from_params` | Tutorial 1 |
| `Agent.from_mesh` / `from_urdf` / `from_params` | Tutorials 1, 4, 6 |
| `resolve_model` / `list_all_models` / `discover_models` / `auto_detect_profile` | Tutorial 6 |
| `register_model` / `unregister_model` / `add_search_path` | Tutorial 6 |
| `Pose.from_xyz` / `from_euler` | Tutorials 1–4 |
| `agent.get_pose()` / `set_pose()` | Tutorial 1 |
| `agent.set_goal_pose()` | Tutorial 1 |
| `agent.set_path()` | Tutorial 3 |
| `FleetCommandDispatcher.navigate()` | Tutorial 3 |
| `FleetStateProvider.get_states()` | Tutorial 3 |
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
| `SimObject.from_sdf()` / `load_mesh_directory()` | Tutorial 7 |
| `sim.events` / `agent.events` | Tutorial 8 |
| `SimPlugin` / `AgentPlugin` / `BatteryPlugin` | Tutorial 9 |
