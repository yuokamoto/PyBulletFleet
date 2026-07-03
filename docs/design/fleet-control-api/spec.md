# Fleet Control API and Selective ROS Interfaces

**Date:** 2026-07-02
**Status:** Draft
**Related:** [ROS 2 Bridge](../ros2-bridge/spec.md),
[Controller Plugins](../controller-plugins/spec.md),
[Snapshot Replay](../snapshot-replay/spec.md),
[Unified Simulation Orchestrator](https://github.com/yuokamoto/Unified-Simulation-Orchestrator)

## Context

The current ROS 2 bridge started with a per-robot interface: each simulated robot
gets its own publishers, subscribers, services, and action servers. This is
easy to integrate with ROS tools and Open-RMF, but it scales poorly when the
fleet has hundreds or thousands of robots. The highest-frequency endpoints are
the state publishers (`odom`, TF, `joint_states`, diagnostics, battery), which
produce work every simulation step or at a fixed publish rate.

Closed PR #24 explored the first Pattern 2 batch API:

- `/fleet/states` publishes one aggregated fleet snapshot.
- `/fleet/navigate` accepts one or many named navigation goals.

That direction is useful, but a pure "batch mode replaces all per-robot
interfaces" design is too restrictive. It breaks existing RMF integration and
per-robot debugging even though low-frequency command/service/action endpoints
are not the main performance problem.

The bridge also has future deployment patterns that are separate from Pattern 2:

- **Pattern 3: Plugin Only** — direct Python/EventBus integration with no ROS
  bridge in the control path.
- **Pattern 4: Plugin + Bridge** — direct Python control path with ROS kept for
  visualization, debugging, and non-RMF external control.

Pattern 2 should therefore be designed as a ROS adapter over the fleet control
API that can coexist with selected per-robot interfaces, not as a mutually
exclusive replacement.

The same fleet-level state and command model should also serve the non-ROS
deployment patterns:

- **Pattern 2: Batch ROS 2** wraps the fleet model in ROS topics/services.
- **Pattern 3: Plugin Only** calls the fleet model directly from Python.
- **Pattern 4: Plugin + Bridge** uses direct Python as the control path while
  ROS wraps the same fleet model for debugging, visualization, and external
  non-RMF control.

## Decision

Make fleet-level APIs and per-robot APIs independently configurable.

The source of truth should be a Python fleet abstraction in the bridge/core
layer, not the ROS topics themselves. ROS is one transport adapter over that
abstraction.

Proposed internal interfaces:

```python
class FleetStateProvider:
    def get_states(self) -> list[RobotStateSnapshot]: ...


class FleetCommandDispatcher:
    def navigate(self, goals: list[RobotGoalCommand]) -> CommandAck: ...
    def stop(self, names: list[str]) -> CommandAck: ...
    def execute_action(self, commands: list[RobotActionCommand]) -> CommandAck: ...
    def attach(self, commands: list[RobotAttachCommand]) -> CommandAck: ...
    def set_charging(self, commands: list[RobotChargingCommand]) -> CommandAck: ...
```

`FleetHandler` should become a thin ROS wrapper around these interfaces:

```text
/fleet/states         -> FleetStateProvider.get_states()
/fleet/navigate       -> FleetCommandDispatcher.navigate()
/fleet/stop           -> FleetCommandDispatcher.stop()
/fleet/execute_action -> FleetCommandDispatcher.execute_action()
/fleet/attach         -> FleetCommandDispatcher.attach()
```

Pattern 3 can call the same Python dispatcher directly from an RMF adapter
plugin. Pattern 4 can have both the plugin and ROS bridge share the same
dispatcher instance.

The fleet APIs are the preferred scalable path:

- `/fleet/states`
- `/fleet/navigate`
- `/fleet/joint_command`
- future `/fleet/stop`
- future `/fleet/execute_action`
- future `/fleet/attach`
- future `/fleet/set_charging`

Per-robot APIs remain available for compatibility, RMF migration, and debugging,
but each group can be disabled separately. If no per-robot interface group is
enabled for a robot, the bridge should not create a per-robot handler for that
robot.

Fleet-level command callbacks should enqueue commands into the dispatcher and
return acknowledgements. They should not run long blocking robot actions inside
the ROS callback thread.

## Configuration Model

Use explicit capability groups rather than one exclusive mode flag.

```yaml
fleet_api:
  enabled: true
  states: true
  navigate: true
  joint_command: true
  stop: false
  execute_action: false
  attach: false
  charging: false

per_robot_api:
  enabled: true
  state_publishers: false   # odom, TF, joint_states, diagnostics, battery_state
  command_topics: true      # cmd_vel, goal_pose, path, joint commands
  services: true            # toggle_attach, attach_object, set_charging
  actions: false            # NavigateToPose, FollowPath, FollowJointTrajectory, ExecuteAction
  include_robots: []        # empty means all robots unless exclude_robots is set
  exclude_robots: []
```

`include_robots` and `exclude_robots` only decide whether a robot gets any
per-robot ROS handler. They do not define different group settings per robot.
When a small number of robots need different per-robot settings, prefer
entity-local overrides after handler decomposition. This follows the same style
as RMF and the existing entity config model: defaults are global, and only the
special robots carry local overrides.

```yaml
per_robot_api:
  enabled: true
  state_publishers: false
  command_topics: false
  services: false
  actions: false

entities:
  - name: robot_debug
    urdf_path: robots/mobile_robot.urdf
    per_robot_api:
      state_publishers: true
      services: true

  - name: robot_commands
    urdf_path: robots/mobile_robot.urdf
    per_robot_api:
      command_topics: true
```

Do not add this override schema in Phase 0. The first implementation should keep
one global `per_robot_api` profile plus `include_robots` / `exclude_robots`.
Per-robot group overrides become useful after the handler groups exist as
separate objects.

## Recommended Deployment Profiles

### Backward-Compatible ROS/RMF

Use this for today's Open-RMF demos and ROS teleoperation.

```yaml
fleet_api:
  enabled: false
per_robot_api:
  enabled: true
  state_publishers: true
  command_topics: true
  services: true
  actions: true
```

### Hybrid Batch State

Use this as the first scalable RMF migration step. High-frequency state is
batched, while low-frequency per-robot commands/services remain available.

```yaml
fleet_api:
  enabled: true
  states: true
  navigate: true
  joint_command: true
per_robot_api:
  enabled: true
  state_publishers: false
  command_topics: true
  services: true
  actions: true
```

### Full Fleet API

Use this for 500-1000+ robot workloads once fleet-level stop/action/attach APIs
exist.

```yaml
fleet_api:
  enabled: true
  states: true
  navigate: true
  joint_command: true
  stop: true
  execute_action: true
  attach: true
per_robot_api:
  enabled: false
```

### Debug Subset

Use this when the fleet is mostly batch-controlled but a few robots need normal
ROS tools.

```yaml
fleet_api:
  enabled: true
  states: true
  navigate: true
  joint_command: true
per_robot_api:
  enabled: true
  state_publishers: false
  command_topics: true
  services: true
  actions: false
  include_robots: ["robot0", "robot1"]
```

## RMF Integration Strategy

The near-term RMF path should not require all commands to become batch commands
at once.

The practical sequence is:

1. Use `/fleet/states` as the main robot-state source.
2. Keep per-robot command/service/action endpoints for RMF compatibility.
3. Add `/fleet/navigate` support in `RobotClientAPI` for navigation dispatch.
4. Add fleet-level stop/action/attach APIs.
5. Move RMF adapter command paths from per-robot endpoints to fleet-level
   endpoints as those APIs become available.

For Pattern 3/4, replace "use `/fleet/states`" with "call
`FleetStateProvider.get_states()`". Replace `/fleet/navigate` and future fleet
services with direct calls to `FleetCommandDispatcher`. The ROS and Python paths
should share command semantics and acknowledgement structures so RMF integration
does not fork by transport.

`FleetState` should eventually carry all state RMF needs without requiring
per-robot publishers:

- name
- pose (`x`, `y`, `yaw`)
- velocity (`vx`, `vy`, `vyaw`)
- moving / mode
- battery state of charge
- charging state
- optional diagnostic status / error text
- optional task/action progress summary

Keep the published fleet state minimal enough for high-rate use. Large or
sparse details should be available through request/response APIs instead of
being published every frame:

- detailed diagnostics
- full joint states for large articulated fleets
- extended battery health / charger metadata
- attached-object details
- controller internals

These can be exposed through future services such as `/fleet/get_robot_info`,
`/fleet/get_diagnostics`, or `/fleet/get_joint_states`, and still be included in
snapshot/replay records where message size is less critical.

Patrol-style RMF tasks can use `/fleet/states` and `/fleet/navigate`.
Delivery-style tasks also need stop/cancel and attach/drop or generic action
support, so `/fleet/execute_action` and/or `/fleet/attach` should be added
before claiming full RMF batch support.

## Message Shape: 2D and 3D

Fleet APIs should support 3D state and command data as the canonical internal
model. RMF mostly consumes 2D navigation (`x`, `y`, `yaw`), but non-RMF users
may need z, roll/pitch, full quaternions, aerial robots, lifts, mobile
manipulators, or cross-floor simulation.

Recommended approach:

- Internal Python snapshots and commands use 3D pose data.
- ROS messages can provide explicit 2D and 3D variants where size matters.
- 2D messages are optimization/compatibility surfaces, not the internal source
  of truth.

Candidate message families:

```text
RobotState2D  # name, x, y, yaw, vx, vy, vyaw, mode/minimal status
RobotState3D  # name, geometry_msgs/Pose, geometry_msgs/Twist, mode/minimal status
RobotGoal2D   # name, x, y, yaw
RobotGoal3D   # name, geometry_msgs/Pose
```

`FleetState` can either choose one family (`RobotState3D[]`) or expose separate
topics:

```text
/fleet/states_2d
/fleet/states_3d
```

Do not overload 2D fields with hidden conventions for z or orientation. If 3D
is needed, use a 3D message explicitly.

## TF Strategy

TF is special: many transforms can be carried in a single `tf2_msgs/TFMessage`
on `/tf`. Unlike per-robot odometry topics, TF does not require one topic per
robot.

For batch/fleet modes:

- Keep a single `/tf` publisher.
- Publish all enabled agents' base transforms in one `TFMessage` when TF output
  is enabled.
- Gate TF separately from odometry and `joint_states` because users may want
  RViz visualization without all per-robot state topics.

Configuration should distinguish:

```yaml
per_robot_api:
  state_publishers: false
  tf: true
```

TF still has O(N) message size, but it avoids O(N) ROS publishers/topics and is
valuable for visualization. For 1000+ robots, users may still disable it or
restrict it to a debug subset.

## Fleet Joint Commands

Fleet-level navigation is not enough for arms and mobile manipulators. Add a
fleet-level joint command API so batch/fleet control can cover articulated
robots without per-robot joint command topics.

Candidate APIs:

```text
/fleet/joint_command         # topic, fire-and-forget
/fleet/set_joint_targets     # service, acknowledgement
```

Candidate command shape:

```text
RobotJointCommand
  string name
  string[] joint_names
  float64[] positions
  float64[] velocities   # optional / empty
  float64[] efforts      # optional / empty
  builtin_interfaces/Duration time_from_start
```

As with navigation, provide both topic and service forms where useful:

- topic for high-rate streaming or best-effort external controllers
- service for low-rate commands that need an acknowledgement
- future action only when progress/cancel semantics are required

## Performance Expectations

The hot path is high-frequency publication, not the mere existence of endpoints.

High-impact at scale:

- per-robot `odom`
- per-robot TF
- per-robot `joint_states`
- per-robot diagnostics/battery publishers

Lower-impact unless heavily used:

- command subscribers
- services
- action servers

However, 1000 action servers can still cause ROS graph, memory, and discovery
overhead because each action expands into multiple ROS entities. For 1000-robot
deployments, command APIs should also move toward fleet-level servers.

Fleet-level services should not perform long-running work in the callback.
Callbacks should validate requests, enqueue commands into the simulation thread
or EventBus, and return an acknowledgement. Progress should be observed through
`/fleet/states`.

For every fleet-level service that may be used at high rate or may target many
robots, provide a topic form with the same payload where possible. A single
service can become an unnecessary bottleneck if many clients send frequent
requests. Topic input plus state-based progress observation is often a better
fit for fleet commands. Use actions only for commands that truly require
goal-handle lifecycle, cancellation, and feedback semantics.

Plugin Only should reduce control-path overhead because it bypasses ROS
serialization, DDS discovery, topic/service/action entities, and ROS callback
dispatch. Plugin + Bridge can keep that direct Python path for RMF while still
exposing ROS for debugging and external control.

Do not claim a measured ROS/RMF performance improvement until bridge-specific
benchmarks exist. Current benchmarks focus on core simulation performance. Add
ROS bridge measurements for endpoint creation, memory, publish cost, fleet
message size, command latency, and RMF adapter latency before treating Phase 5
as a performance-verified optimization.

## Snapshot, Replay, and Trace

The Python fleet abstraction should be traceable by construction. This connects
the ROS bridge roadmap to the Snapshot/Replay and future trace/audit work.

State snapshots:

- `FleetStateProvider.get_states()` returns a deterministic fleet snapshot.
- The ROS `/fleet/states` message is a transport encoding of the same snapshot.
- Snapshot/replay can persist the Python snapshot without requiring ROS.
- The snapshot model should align with the Unified Simulation Orchestrator
  direction: snapshot-centric state, optional delta snapshots, reusable
  scenario/logic across engines, and replay without binding the record format
  to ROS transport.

Command trace:

- Every `FleetCommandDispatcher` call should produce a command event:
  - command id
  - simulation time
  - source (`ros`, `plugin`, `replay`, `test`)
  - command type (`navigate`, `stop`, `execute_action`, `attach`, ...)
  - target names
  - accepted/rejected names and reason
- The event should be emitted on EventBus or appended to a simulation recorder
  trace before the command mutates agent state.

Replay:

- Replaying a fleet trace should call the same `FleetCommandDispatcher`
  methods with recorded command payloads at recorded simulation times.
- This avoids duplicating ROS-specific replay logic.
- ROS bag replay remains possible, but Python trace replay should be the
  canonical deterministic path.
- Recorder format is intentionally not decided here. ROSCon work does not need
  snapshot/replay to be complete. Candidate formats include JSONL for simple
  event logs, SQLite for indexed state/command queries, MCAP for ROS-adjacent
  tooling, or a future USO-compatible snapshot/delta format.

Audit:

- State snapshots plus command events form an auditable causal record:
  command -> state transition -> resulting fleet snapshot.
- The record is useful for debugging RMF behavior, verifying large-fleet
  scheduling, and reproducing CI failures without a ROS graph.

## Handler Decomposition

Selective per-robot APIs require splitting the current all-in-one
`RobotHandler`.

Proposed handler groups:

- `StatePublisherHandler`
  - `/{robot}/odom`
  - TF
  - `/{robot}/joint_states`
  - `/{robot}/diagnostics`
  - `/{robot}/battery_state`
- `CommandTopicHandler`
  - `/{robot}/cmd_vel`
  - `/{robot}/goal_pose`
  - `/{robot}/path`
  - `/{robot}/joint_trajectory`
  - `/{robot}/joint_commands`
- `TfPublisherHandler`
  - `/tf` batch publication for all enabled agents
- `NavigationActionHandler`
  - `/{robot}/navigate_to_pose`
  - `/{robot}/follow_path`
  - `/{robot}/follow_joint_trajectory`
- `ServiceHandler`
  - `/{robot}/toggle_attach`
  - `/{robot}/attach_object`
  - `/{robot}/set_charging`
- `ExecuteActionHandler`
  - `/{robot}/execute_action`
  - `/{robot}/execute_action_blocking`

`BridgeNode` should instantiate only the handler groups enabled by config. If
all groups are disabled for a robot, no per-robot handler should be created.

This makes Handler Decomposition a prerequisite for a clean selective-interface
implementation. A temporary implementation can gate calls inside the existing
`RobotHandler`, but that keeps unnecessary objects alive and makes the code
harder to reason about.

## Open Questions

- Should `RobotGoal` remain `name, x, y, yaw`, or should it contain
  `geometry_msgs/Pose` / `Pose2D`?
- Should the ROS API expose separate `RobotState2D` / `RobotState3D` and
  `RobotGoal2D` / `RobotGoal3D` messages, or should it standardize on 3D
  messages and accept the larger payload?
- Should the high-rate `/fleet/states` topic remain minimal, with detailed
  diagnostics/joint/controller state fetched through services on demand?
- Should `FleetState` include a string diagnostic message, an enum status, or
  both?
- Should every fleet-level service also have a topic form by default, with
  services reserved for acknowledgement-oriented clients?
- How should requests be sharded if one fleet-level service becomes a bottleneck
  for very high command rates?
- Which concrete recorder format should store fleet snapshots and command
  traces: JSONL, SQLite, MCAP, the existing recorder output, or a
  USO-compatible snapshot/delta format?
- Should command ids be generated by the dispatcher, accepted from callers, or
  both?

## Success Criteria

- Existing per-robot ROS/RMF configurations continue to work unchanged.
- Batch fleet state can run without per-robot state publishers.
- Per-robot commands/services/actions can coexist with fleet state.
- A single `/tf` publisher can broadcast all enabled agents without per-robot TF
  publishers.
- Fleet-level navigation and joint commands both support one-or-many robot
  targets.
- A robot with no enabled per-robot groups creates no per-robot ROS interfaces.
- RMF can consume fleet state while retaining existing per-robot command paths.
- Future fleet-level action/attach APIs can replace per-robot endpoints without
  changing the configuration model.
- Pattern 2, 3, and 4 use the same Python fleet state and command semantics.
- Fleet commands and snapshots can be recorded and replayed without ROS.
