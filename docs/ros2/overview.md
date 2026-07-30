# Bridge Interfaces

`bridge_node` owns a PyBulletFleet simulation core and exposes selected ROS 2
interfaces. It can export a conventional per-robot API, a compact fleet API, or
both. Exported endpoints are independent from how an RMF adapter controls the
simulation.

## Per-robot API

Enable this API when existing tools expect one ROS namespace per robot. A robot
can publish odometry, joint state, path, current goal, diagnostics, and TF; it
can accept velocity, goal-pose, path, trajectory, and joint commands. It also
provides navigation, path-following, trajectory, attach, and generic-action
services/actions as appropriate for the enabled interface groups.

The full endpoint and message-type table is maintained in the
[bridge README](https://github.com/yuokamoto/PyBulletFleet/blob/main/ros2_bridge/README.md#pybullet_fleet_ros--ros-2-bridge).

## Fleet API

Use the fleet API when an external client works on a whole fleet. It reduces
ROS graph size by replacing repeated per-robot state and command endpoints with
batched endpoints:

| Endpoint | Purpose |
| --- | --- |
| `/fleet/states` | One `FleetState` message containing all current Agent states |
| `/fleet/navigate` | Batched navigation command, as topic or service |
| `/fleet/stop` | Batched stop command, as topic or service |
| `/fleet/attach` | Batched attach/detach command, as topic or service |
| `/fleet/execute_action` | Batched generic action command, as topic or service |
| `/fleet/joint_command` | Batched joint-position command, as topic or service |

Pose-bearing fleet commands include a ROS header. `header.frame_id` identifies
the command frame (normally `odom`); `header.stamp` records the command issue
time. Clients should treat service responses and command acknowledgements as
the command-ingress result, not as proof that every robot has completed motion.

### Actor visibility and current scope

An `Agent` represents any active simulated actor, not only a robot owned by the
ROS/RMF client. This includes mock people, robots owned by another system, and
jointed devices. Today the fleet API is global: `FleetStateProvider` publishes
every `sim.agents` entry in `/fleet/states`, and `FleetCommandDispatcher` can
resolve every uniquely named Agent. A separate `AgentManager` does not alter
that scope.

For per-robot endpoints, use `per_robot_api.include_robots` or
`exclude_robots` to avoid creating ROS interfaces for mock actors. If a fleet
state must contain only a controlled fleet, disable the fleet state endpoint or
provide that filtered state through an integration/plugin for now. Selective
fleet-state publication and named fleet API scopes are planned; see the
[roadmap](../roadmap).

## Simulation services

The bridge can also expose `simulation_interfaces` services for spawning,
deleting, querying, moving, stepping, pausing, and resetting entities. These
are useful for test harnesses and scenario tools, but are separate from the
fleet command API.

### Dynamic spawning

`/sim/spawn_entity` creates an **Agent from a URDF** at runtime.  It accepts
the entity `name`, a URDF `uri` (including `package://` URIs or a known model
name), `initial_pose`, and `allow_renaming`.  The bridge resolves the URI and
creates the agent with its default dynamic-spawn configuration.

It is intentionally a small interoperability service, not a serialized
`AgentSpawnParams` API: it cannot set a controller, motion limits, mass,
collision settings, `user_data`, or arbitrary mesh/shape parameters.  It also
does not currently create a passive `SimObject`, despite the generic
`SpawnEntity` service name.  For a fully configured robot or object, define it
in the bridge YAML scenario (or create it through the Python API) before
starting the simulation.

## Scale guidance

Fleet endpoints are the preferred public interface for high robot counts. The
latest 1000-robot Docker measurement reached 9.37x real time in fleet mode;
per-robot and hybrid modes impose substantial DDS graph overhead. The benchmark
also records that per-robot publication latency is not equivalent to end-to-end
motion completion. Read the [benchmark results](../benchmarking/results) and
the [bridge performance notes](https://github.com/yuokamoto/PyBulletFleet/blob/main/ros2_bridge/PERFORMANCE.md)
before setting production thresholds.

For framework developers, the transport-neutral state and command abstractions
behind these endpoints are described in the Fleet API architecture page.
