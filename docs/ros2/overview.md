# Bridge Interfaces

`bridge_node` owns a PyBulletFleet simulation core and exposes selected ROS 2
interfaces. It can export a conventional per-robot API, a compact fleet API, or
both. Exported endpoints are independent from how an RMF adapter controls the
simulation.

## ROS communication patterns

Choose the public ROS boundary by the client that owns the command loop.  The
per-robot and fleet paths may be enabled together for debugging, but they are
alternative ways for a client to communicate with the same simulation core.

```{mermaid}
flowchart LR
  subgraph clients[External ROS processes]
    app1["Robot App 1"]
    appn["Robot App N"]
    fleet[Fleet application]
  end
  subgraph bridge[bridge_node]
    per_robot[Per-robot handlers]
    fleet_api[Fleet ROS API]
  end
  core[PyBulletFleet simulation core]

  app1 <-->|ROS 2 per-robot endpoints| per_robot
  appn <-->|ROS 2 per-robot endpoints| per_robot
  fleet <-->|ROS 2 fleet endpoints| fleet_api
  per_robot <-->|In-process Python| core
  fleet_api <-->|In-process Python| core

  linkStyle 0,1,2 stroke:#1976d2,stroke-width:3px
  linkStyle 3,4 stroke:#616161,stroke-width:2px
```

Blue links are ROS 2 communication; gray links are in-process Python calls.
The arrows are bidirectional because commands flow toward the simulator while
state, feedback, and diagnostics flow back to the client.
`Robot App 1` and `Robot App N` represent the per-robot pattern: one ROS
namespace and handler set is created for every selected robot.

| Boundary | ROS graph shape | Typical client | Choose it when |
| --- | --- | --- | --- |
| Per-robot API | One namespace and handler set per selected robot | Nav2-like Robot App, RViz, manual debugging | The client expects standard robot topics, services, or actions |
| Fleet ROS API (`fleet_api`) | One batched state stream and named command endpoints | Fleet manager or load-test client | The client commands or observes many robots together |

`per_robot_api.include_robots` and `exclude_robots` limit which Agents receive
per-robot endpoints. Fleet endpoints can instead be partitioned explicitly by
named managers; see [Bridge Configuration](configuration).

## Per-robot API

Enable this API when existing tools expect one ROS namespace per robot. A robot
can publish odometry, joint state, path, current goal, diagnostics, and TF; it
can accept velocity, goal-pose, path, trajectory, and joint commands. It also
provides navigation, path-following, trajectory, attach, and generic-action
services/actions as appropriate for the enabled interface groups.

| Direction | Topic | Type | Description |
| --- | --- | --- | --- |
| Pub | `/{robot}/odom` | `nav_msgs/Odometry` | Odometry and `odom → base_link` TF |
| Pub | `/{robot}/joint_states` | `sensor_msgs/JointState` | Joint positions and velocities |
| Pub | `/{robot}/plan` | `nav_msgs/Path` | Current planned path |
| Pub | `/{robot}/current_goal` | `geometry_msgs/PoseStamped` | Active navigation goal |
| Pub | `/{robot}/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Robot diagnostics |
| Sub | `/{robot}/cmd_vel` | `geometry_msgs/Twist` | Velocity command |
| Sub | `/{robot}/goal_pose` | `geometry_msgs/PoseStamped` | Fire-and-forget navigation goal |
| Sub | `/{robot}/path` | `nav_msgs/Path` | Path to follow |
| Sub | `/{robot}/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Joint trajectory command |
| Sub | `/{robot}/joint_commands` | `std_msgs/Float64MultiArray` | Direct joint-position command |

Per-robot action servers are `navigate_to_pose`, `follow_path`,
`follow_joint_trajectory`, and `execute_action_blocking`; the per-robot
services are `toggle_attach` and `attach_object`.  The non-blocking generic
action topic is `/{robot}/execute_action`.

## Fleet ROS API

Use the Fleet ROS API (`fleet_api`) when an external client works on a whole
fleet. It reduces
ROS graph size by replacing repeated per-robot state and command endpoints with
batched endpoints. In the table, `/fleet[/<manager>]` means either the global
compatibility endpoint (`/fleet`) or an endpoint for one configured manager
(`/fleet/<manager>`):

| Endpoint | Purpose |
| --- | --- |
| `/fleet[/<manager>]/states` | One `FleetState` message for the global scope or that manager's Agents |
| `/fleet[/<manager>]/navigate` | Batched navigation command, as topic or service |
| `/fleet[/<manager>]/stop` | Batched stop command, as topic or service |
| `/fleet[/<manager>]/attach` | Batched attach/detach command, as topic or service |
| `/fleet[/<manager>]/execute_action` | Batched generic action command, as topic or service |
| `/fleet[/<manager>]/joint_command` | Batched joint-position command, as topic or service |

Pose-bearing fleet commands include a ROS header. `header.frame_id` identifies
the command frame (normally `odom`); `header.stamp` records the command issue
time. Clients should treat service responses and command acknowledgements as
the command-ingress result, not as proof that every robot has completed motion.

### Agent scope

An `Agent` represents any active simulated actor, not only a robot owned by the
ROS/RMF client. This includes mock people, robots owned by another system, and
jointed devices. The global `/fleet/*` endpoints retain their compatibility
scope by default. Configure `state_scope` for a filtered global state stream,
or use `manager_interfaces` for manager-namespaced state and command endpoints.
See [manager-scoped Fleet API endpoints](configuration.md#manager-scoped-fleet-api-endpoints).

For per-robot endpoints, use `per_robot_api.include_robots` or
`exclude_robots` to avoid creating ROS interfaces for mock actors.

## Simulation services

The bridge can also expose `simulation_interfaces` services for spawning,
deleting, querying, moving, stepping, pausing, and resetting entities. These
are useful for test harnesses and scenario tools, but are separate from the
fleet command API.

### Choose the control boundary

Use the interface that reflects what is being controlled, rather than treating
the Fleet API as a general-purpose simulation API:

| Target and intent | Recommended boundary |
| --- | --- |
| An `Agent` moving as part of a fleet | Fleet ROS API (`/fleet/*` or `/fleet/<manager>/*`) |
| An `Agent` used with Nav2-like, robot-by-robot tooling | Per-robot API |
| A general `SimObject` queried, moved, spawned, or deleted from another ROS node | `simulation_interfaces` services such as `/sim/get_entity_state`, `/sim/set_entity_state`, and `/sim/delete_entity` |
| Doors, lifts, or workcells | Their dedicated ROS interfaces |
| Scenario logic, physics tuning, or continuous custom object behavior | In-process Python API |

`SimObject` is intentionally not a Fleet API target: fleet commands model
robot/actor operations such as navigation, stop, attachment, and joint
commands. `simulation_interfaces` is the appropriate ROS boundary when an
external node must manipulate a non-Agent entity. A client can perform
periodic `/sim/set_entity_state` calls for simple continuous control, but
complex behavior and physics-oriented logic are generally clearer and more
efficient in Python.

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
per-robot and hybrid modes add substantial ROS interface and bridge overhead.
The measurement does not isolate DDS graph cost from message handling,
serialization, callbacks, TF, or endpoint handlers. The benchmark also records
that per-robot publication latency is not equivalent to end-to-end motion
completion. Read the [benchmark results](../benchmarking/results) and
the [bridge performance notes](https://github.com/yuokamoto/PyBulletFleet/blob/main/ros2_bridge/PERFORMANCE.md)
before setting production thresholds.

For framework developers, the transport-neutral state and command abstractions
behind these endpoints are described in the Fleet API architecture page.
