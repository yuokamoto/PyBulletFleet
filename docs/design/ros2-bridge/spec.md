# ROS 2 Bridge Specification

**Date:** 2026-03-21 (updated: 2026-03-21)
**Status:** Draft
**Depends on:** [Plugin Architecture](../plugin-architecture/spec.md) Phase 2 (Controller ABC)

## Context

PyBulletFleet is a kinematics-first multi-robot simulation framework targeting 100+ robots.
It has no ROS dependency today but was designed with ROS 2 compatibility in mind
(Pose follows `geometry_msgs/Pose` conventions, Agent API follows ROS message patterns).
Users need to integrate simulated fleets with ROS 2 navigation stacks, visualization tools,
and fleet management systems like Open-RMF.

## Decision

Build a ROS 2 bridge as a single-node architecture that wraps `MultiRobotSimulationCore`
and `AgentManager`, exposing per-robot standard ROS 2 topics/actions and
simulation-level `simulation_interfaces` services.

Velocity commands (`cmd_vel`) flow through the **OmniOmniVelocityController** (from the Plugin Architecture)
rather than a direct `set_velocity()` method on Agent. This keeps Agent's API clean and leverages
the Controller ABC strategy pattern.

Start in the same repository (`ros2_bridge/` directory) for development velocity.
Split to a separate repository when the API stabilizes.

## Requirements

### Phase 1 (this spec)

- Single bridge node wrapping `MultiRobotSimulationCore` in-process
- Per-robot standard topics: `cmd_vel` (sub), `odom` (pub), `joint_states` (pub), `tf` (pub)
- Per-robot Nav2 actions: `NavigateToPose`, `FollowPath`
- Per-robot arm actions: `FollowJointTrajectory`
- `/clock` publisher (`rosgraph_msgs/Clock`) for `use_sim_time` support
- `simulation_interfaces` — broadly support all applicable services and actions (see table below)
- Docker-based development (Ubuntu 24.04 + Jazzy primary, Humble secondary)
- Examples covering mobile robots, arms, and mobile manipulators
- No custom messages — standard ROS 2 messages only

### Phase 2 (future)

- Open-RMF fleet adapter sample (`rmf_fleet_adapter` Python API)
- Fleet-level batch topics for efficient multi-robot command dispatch

### Phase 3 (future)

- Custom messages for PyBulletFleet-specific actions (Pick, Drop, ActionQueue)
- `bloom-release` for `apt install ros-jazzy-pybullet-fleet-*`

## Architecture

```
┌───────────────────────────────────────────────────────────────┐
│                   pybullet_fleet_ros (1 node)                 │
│                                                               │
│  ┌───────────────────┐  ┌──────────────────────────────────┐  │
│  │ Per-Robot Handlers │  │ Sim-Level Services               │  │
│  │                   │  │                                  │  │
│  │ /{name}/cmd_vel ──┼──┼► OmniOmniVelocityController.set_velocity │  │
│  │ /{name}/odom      │  │   SpawnEntity                    │  │
│  │ /{name}/tf        │  │   DeleteEntity                   │  │
│  │ /{name}/joint_st  │  │   Get/SetEntityState             │  │
│  │                   │  │   SimulationStep                 │  │
│  │ Nav2 Actions:     │  │   SetSimulationPaused            │  │
│  │  NavigateToPose   │  │   GetSimulatorFeatures           │  │
│  │  FollowPath       │  │                                  │  │
│  │                   │  └──────────────────────────────────┘  │
│  │ Arm Actions:      │                                        │
│  │  FollowJointTraj  │  ┌──────────────────────────────────┐  │
│  └───────────────────┘  │ AgentManager ROS Wrapper         │  │
│           │              │ (future batch I/F)               │  │
│           ▼              └──────────────┬───────────────────┘  │
│  ┌────────────────────────────────────────────────────────┐   │
│  │      MultiRobotSimulationCore (in-process)             │   │
│  │      sim.step() driven by ROS timer callback           │   │
│  │                                                        │   │
│  │  Agent ── Controller (has-a, Strategy pattern)         │   │
│  │         ├─ OmniOmniVelocityController (cmd_vel → pose update)  │   │
│  │         └─ TPIController (goal-based, legacy default)  │   │
│  │                                                        │   │
│  │  EventBus ── agent_added / agent_removed events        │   │
│  │              (auto RobotHandler lifecycle, Phase 2+)    │   │
│  └────────────────────────────────────────────────────────┘   │
└───────────────────────────────────────────────────────────────┘
```

### Key design choices

1. **Single node** — avoids DDS Discovery overhead with 100+ robots.
   One node handles all per-robot topics via namespace prefixing.
2. **In-process** — bridge imports `pybullet_fleet` directly (no IPC).
   `sim.step()` is called from a ROS timer callback.
3. **Per-robot standard topics** — compatible with teleop_twist_keyboard, Nav2,
   MoveIt, RViz out of the box.
4. **OmniOmniVelocityController for cmd_vel** — `cmd_vel` Twist messages are converted to
   `OmniOmniVelocityController.set_velocity(vx, vy, vz, wz)` calls. The Controller ABC
   strategy pattern keeps velocity logic out of Agent core. Each ROS-controlled robot
   gets a `OmniOmniVelocityController` attached at spawn time.
5. **AgentManager wrapper** — `AgentManager.set_goal_pose_by_object_id()` and
   batch methods map directly to fleet-level ROS interfaces.
6. **simulation_interfaces** — broadly support all applicable interfaces, not just a subset.
7. **`/clock` publisher** — enables `use_sim_time` for Nav2, Open-RMF, and RViz integration.
8. **EventBus integration (Phase 2+)** — When the plugin architecture's EventBus is available,
   `agent_added` / `agent_removed` events can automatically create/destroy `RobotHandler`
   instances. Phase 1 manages handlers manually in `BridgeNode`.

### Plugin Architecture Integration

This design depends on the [Plugin Architecture](../plugin-architecture/spec.md):

| Plugin Phase | ROS Bridge Usage |
|-------------|------------------|
| Phase 2 (Controller ABC) | **Required for Phase 1.** `OmniOmniVelocityController` provides `cmd_vel` → pose update. Each ROS-controlled agent gets `agent.set_controller(OmniOmniVelocityController())` at spawn. |
| Phase 3 (EventBus) | **Optional for Phase 1.** `agent_added`/`agent_removed` events could automate `RobotHandler` lifecycle. Phase 1 manages handlers manually. |
| Phase 1 (Registry) | **Not used in ROS Bridge Phase 1.** Future: resolve robot types from YAML by name. |

## ROS 2 Distro Support

| Distro | Ubuntu | Role | EOL |
|--------|--------|------|-----|
| **Jazzy** | 24.04 | Primary | May 2029 |
| **Humble** | 22.04 | Secondary (CI tested) | May 2027 |

## Repository Layout

```
PyBulletFleet/
├── pybullet_fleet/              # Core (pip, ROS-independent)
├── ros2_bridge/                 # colcon workspace overlay
│   ├── pybullet_fleet_ros/      # Bridge node package
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── pybullet_fleet_ros/
│   │   │   ├── __init__.py
│   │   │   ├── bridge_node.py           # Main node
│   │   │   ├── robot_handler.py         # Per-robot topic/action mgmt
│   │   │   ├── sim_services.py          # simulation_interfaces impl
│   │   │   └── agent_manager_wrapper.py # AgentManager ROS wrapper
│   │   ├── launch/
│   │   │   ├── teleop_mobile.launch.py
│   │   │   ├── nav2_goal.launch.py
│   │   │   ├── multi_robot_fleet.launch.py
│   │   │   ├── arm_control.launch.py
│   │   │   └── mobile_manipulator.launch.py
│   │   └── config/
│   │       └── default_bridge.yaml
│   └── pybullet_fleet_bringup/  # Launch + RViz configs (optional)
│       ├── package.xml
│       ├── launch/
│       └── rviz/
├── docker/
│   ├── Dockerfile.jazzy
│   ├── Dockerfile.humble
│   └── docker-compose.yaml
└── examples/                    # Existing Python-only examples (unchanged)
```

## Development Environment

- **Docker-based**: `Dockerfile.jazzy` (primary) and `Dockerfile.humble` (secondary)
- Host runs PyBulletFleet natively; Docker container runs ROS 2 bridge
- OR: entire stack (PyBulletFleet + ROS 2) runs inside Docker
- `docker-compose.yaml` for one-command startup
- X11 forwarding for RViz / PyBullet GUI when needed

## Topic & Service Naming Convention

```
/{robot_name}/cmd_vel                  # geometry_msgs/Twist
/{robot_name}/odom                     # nav_msgs/Odometry
/{robot_name}/joint_states             # sensor_msgs/JointState
/tf                                    # tf2_msgs/TFMessage (all robots)
/{robot_name}/navigate_to_pose         # nav2_msgs/NavigateToPose (action)
/{robot_name}/follow_path              # nav2_msgs/FollowPath (action)
/{robot_name}/follow_joint_trajectory  # control_msgs/FollowJointTrajectory (action)

/clock                                 # rosgraph_msgs/Clock (sim time)

# --- simulation_interfaces services (Phase 1) ---
/sim/get_simulator_features            # GetSimulatorFeatures (implement first)
/sim/spawn_entity                      # SpawnEntity
/sim/delete_entity                     # DeleteEntity
/sim/get_entity_state                  # GetEntityState
/sim/set_entity_state                  # SetEntityState
/sim/get_entities                      # GetEntities
/sim/get_entities_states               # GetEntitiesStates
/sim/get_entity_info                   # GetEntityInfo
/sim/get_entity_bounds                 # GetEntityBounds (via p.getAABB)
/sim/step_simulation                   # StepSimulation
/sim/get_simulation_state              # GetSimulationState
/sim/set_simulation_state              # SetSimulationState (pause/resume)
/sim/reset_simulation                  # ResetSimulation
/sim/get_spawnables                    # GetSpawnables (scan URDF dir)

# --- simulation_interfaces actions (Phase 1) ---
/sim/simulate_steps                    # SimulateSteps (N-step with feedback)

# --- simulation_interfaces (deferred / low priority) ---
# /sim/set_entity_info                 # SetEntityInfo (partial support)
# /sim/get_named_poses                 # GetNamedPoses (future)
# /sim/get_available_worlds            # GetAvailableWorlds (future)
# /sim/load_world / unload_world       # LoadWorld/UnloadWorld (future)
# /sim/get_current_world               # GetCurrentWorld (future)
```

## Examples (Phase 1)

| # | Example | Robot Type | Key ROS I/F | Description |
|---|---------|-----------|-------------|-------------|
| 1 | `teleop_mobile` | Mobile | cmd_vel, odom, tf | Drive 1 robot with teleop_twist_keyboard |
| 2 | `nav2_goal` | Mobile | NavigateToPose | Send Nav2 goal, robot navigates autonomously |
| 3 | `multi_robot_fleet` | Mobile ×N | cmd_vel ×N, odom ×N, tf | Visualize fleet in RViz |
| 4 | `arm_control` | Arm | joint_states, FollowJointTrajectory | Control arm joints via ROS |
| 5 | `mobile_manipulator` | Mobile + Arm | cmd_vel + joint_states + tf | Combined mobile base + arm |

## Constraints

- PyBulletFleet core (`pybullet_fleet/`) must remain ROS-independent
- `ros2_bridge/` is NOT included in the PyPI package
- No custom messages in Phase 1
- Docker images should work on machines without ROS installed natively
- Bridge must not significantly degrade PyBulletFleet's 100+ robot performance

## Out of Scope

- Open-RMF integration (Phase 2)
- Custom message types / bloom release (Phase 3)
- Physics-mode specific features (e.g., force/torque sensors)
- Gazebo replacement — this is complementary, not competitive
- ROS 1 support

## Open Questions

- [x] ~~Should `sim.step()` be driven by a ROS timer or by `simulation_interfaces/SimulationStep` service calls?~~ → Timer for continuous sim. `ROSSimulationCore` subclass overrides `run_simulation()` to be non-blocking; ROS timer drives `step_once()`.
- [ ] TF publishing strategy for 100+ robots: single `/tf` topic vs per-robot? (Single is standard but may be high-bandwidth)
- [ ] Should we support `ros2 launch` directly from host, or always via Docker?
- [ ] simulation_interfaces is still evolving — pin to a specific tag or track main?
- [x] ~~How does cmd_vel reach Agent without adding set_velocity() to Agent?~~ → OmniOmniVelocityController (Controller ABC strategy pattern). cmd_vel → `OmniOmniVelocityController.set_velocity()` → `Controller.compute()` → `Agent.set_pose()`.

## Success Criteria

- [ ] `docker compose up` launches PyBulletFleet + ROS bridge with zero native ROS install
- [ ] `ros2 topic echo /clock` shows advancing simulation time
- [ ] `ros2 topic echo /{robot}/odom` shows live odometry from a simulated robot
- [ ] `teleop_twist_keyboard` drives a simulated robot via cmd_vel
- [ ] NavigateToPose action completes (robot arrives at goal)
- [ ] FollowJointTrajectory action moves simulated arm joints
- [ ] RViz displays all robots via /tf + /odom markers
- [ ] 10+ robots running simultaneously without bridge becoming bottleneck
- [ ] CI passes on both Jazzy and Humble Docker images
- [ ] All 5 example launch files work out of the box

## References

- [simulation_interfaces](https://github.com/ros-simulation/simulation_interfaces) — ROS 2 standard sim interfaces
- [Open-RMF](https://github.com/open-rmf/rmf) — Fleet management framework
- [rmf_demos_fleet_adapter](https://github.com/open-rmf/rmf_demos/tree/main/rmf_demos_fleet_adapter) — Reference fleet adapter
- [Nav2 action definitions](https://github.com/ros-navigation/navigation2/tree/main/nav2_msgs) — NavigateToPose, FollowPath
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) — Primary target distro
