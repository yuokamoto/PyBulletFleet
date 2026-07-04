# Fleet Control API Implementation Plan

**Date:** 2026-07-03
**Status:** Draft
**Spec:** [spec.md](spec.md)

## Goal

Implement a transport-independent fleet control API and ROS 2 adapter without
breaking the existing per-robot ROS/RMF interface. The end state is:

- fleet state and command semantics live in Python abstractions;
- ROS `/fleet/*` endpoints are thin wrappers over those abstractions;
- per-robot APIs can be enabled or disabled by group;
- no per-robot handler is created when all per-robot groups are disabled;
- Pattern 2, 3, and 4 share the same state/command model;
- future snapshot/replay can record the same fleet snapshots and command events.

## Non-Goals

- Do not rewrite the RMF adapter in the first PR.
- Do not implement full snapshot/replay before ROSCon.
- Do not remove existing per-robot ROS APIs.
- Do not require custom batch commands for all existing per-robot services in
  the first iteration.

## Phase 0 — Preserve Current Behavior

Purpose: introduce configuration and test scaffolding with zero behavior change.

Tasks:

- Add `fleet_api` / `per_robot_api` config parsing helpers.
- Default config must expand to today's behavior:
  - fleet API disabled;
  - all per-robot groups enabled.
- Add tests for config expansion and precedence.
- Add docs examples for backward-compatible, hybrid, full-fleet, and debug
  subset profiles.

Exit criteria:

- Existing bridge configs behave unchanged.
- Invalid config values produce clear warnings or errors.

## Phase 1 — Handler Decomposition Skeleton

Purpose: split the all-in-one `RobotHandler` into group boundaries without
changing externally visible behavior.

Tasks:

- Introduce per-robot handler group classes:
  - `StatePublisherHandler`
  - `CommandTopicHandler`
  - `NavigationActionHandler`
  - `ServiceHandler`
  - `ExecuteActionHandler`
  - `TfPublisherHandler` or a bridge-level TF batch publisher
- Keep `RobotHandler` as a compatibility facade that instantiates all groups.
- Move resource creation/destruction into the groups.
- Preserve current topic/service/action names.
- Preserve existing tests by keeping compatibility attributes or updating tests
  to target the new group classes.

Exit criteria:

- Full per-robot mode creates the same interfaces as today.
- `RobotHandler` remains import-compatible for custom handler users.
- Unit tests cover each group in isolation.

## Phase 2 — Selective Per-Robot Interfaces

Purpose: allow performance-oriented profiles without removing compatibility.

Tasks:

- Teach `BridgeNode` to instantiate only enabled per-robot handler groups.
- Implement `include_robots` / `exclude_robots`.
- If no group is enabled for a robot, create no per-robot handler for that robot.
- Keep per-robot group settings global in this phase. If a few robots need
  different settings, add future entity-local `per_robot_api` overrides after
  handler decomposition instead of encoding per-robot behavior into
  include/exclude filters.
- Separate TF from state publishers:
  - `state_publishers: false`
  - `tf: true`
- Add tests for:
  - all groups enabled;
  - state publishers disabled but services/actions enabled;
  - no groups enabled;
  - debug subset.

Exit criteria:

- Hybrid batch-state mode can run with no per-robot state publishers.
- Per-robot command/service/action paths can coexist with fleet state.

## Phase 3 — Python Fleet State and Command Abstractions

Purpose: define the transport-independent API used by ROS, plugins, and replay.

Tasks:

- Add Python data structures:
  - `RobotStateSnapshot2D`
  - `RobotStateSnapshot3D`
  - `RobotGoalCommand2D`
  - `RobotGoalCommand3D`
  - `RobotJointCommand`
  - `CommandAck`
  - `CommandEvent`
- Add `FleetStateProvider`.
- Add `FleetCommandDispatcher`.
- Dispatcher should:
  - resolve robot names through a cached name index;
  - validate commands;
  - enqueue or apply commands on the simulation-safe path;
  - emit command events before mutating state.
- Keep ROS out of these data structures.

Exit criteria:

- Python tests can navigate one or many robots without importing ROS.
- State snapshots can be generated without ROS.
- Command events include source, simulation time, command id, target names, and
  accepted/rejected names.

## Phase 4 — ROS Fleet Wrappers

Purpose: expose the Python fleet model through ROS.

Tasks:

- Implement `/fleet/states` from `FleetStateProvider`.
- Implement `/fleet/navigate` topic and service from `FleetCommandDispatcher`.
- Implement `/fleet/joint_command` topic and service.
- Decide first ROS message set:
  - either 3D-only messages;
  - or explicit 2D/3D variants.
- Keep high-rate state messages minimal.
- Add detail services later for diagnostics, joint state, and controller internals.

Exit criteria:

- Batch state and navigate work for one and many robots.
- Topic and service variants share payload semantics.
- Docker smoke covers `/fleet/states` and `/fleet/navigate`.

## Phase 5 — RMF Migration and Plugin-Only Path

Purpose: reduce RMF scaling pressure and make Plugin Only / Plugin + Bridge
deployment possible without duplicating RMF adapter logic.

The subphase labels keep the RMF work grouped under one architectural change:
the RMF adapter stops depending directly on one ROS transport shape. The
recommended PR order still splits these subphases into separate PRs, but they
share the same success condition and should be evaluated together.

### Phase 5a — RMF Client Abstraction

Tasks:

- Extract the RMF-facing client contract from `RobotClientAPI`.
- Define a transport-independent client interface:
  - `get_states()`
  - `navigate(goals)`
  - `stop(names)`
  - `execute_action(commands)`
  - `attach(commands)`
- Implement `RosFleetClient` using ROS `/fleet/*` endpoints.
- Implement `PythonFleetClient` using `FleetStateProvider` and
  `FleetCommandDispatcher` directly.
- Keep the existing per-robot ROS client path as a compatibility implementation.

Exit criteria:

- RMF adapter logic depends on the client abstraction, not directly on ROS
  topics/actions/services.
- Existing per-robot ROS RMF demos still work.

### Phase 5b — RMF over Fleet State and Navigation

Tasks:

- Let RMF consume fleet state snapshots instead of per-robot state publishers.
- Add `/fleet/navigate` / dispatcher-backed navigation support to the client
  abstraction.
- Keep existing per-robot command/service/action paths initially for delivery
  and compatibility.

Exit criteria:

- Patrol can run with fleet state and fleet navigation.

### Phase 5c — Plugin Only Launch Path

Tasks:

- Add a launch/runtime path where RMF adapter, simulation core,
  `FleetStateProvider`, and `FleetCommandDispatcher` run in one Python process.
- No ROS bridge is required in the control path.
- Use the same RMF client abstraction with `PythonFleetClient`.

Exit criteria:

- A basic RMF patrol scenario can run without bridge topics/services in the
  control path.

### Phase 5d — Plugin + Bridge Launch Path

Tasks:

- Run the direct Python fleet client as the RMF control path.
- Run ROS bridge wrappers over the same provider/dispatcher for debugging,
  visualization, and non-RMF external control.
- Ensure ROS-origin commands and plugin-origin commands share command ids,
  acknowledgements, and trace semantics.

Exit criteria:

- Plugin + Bridge can observe and optionally command the same simulation without
  maintaining a separate control implementation.
- Delivery remains supported through per-robot compatibility paths until
  fleet-level `stop`, `execute_action`, and `attach` APIs are ready.

### Phase 5 Performance Notes

Expected direction:

- Plugin Only should reduce ROS serialization, DDS graph, and callback overhead
  on the control path.
- Hybrid ROS should reduce high-frequency publisher overhead if per-robot state
  publishers are disabled.
- Per-robot service/action compatibility may still consume memory and discovery
  resources, especially at 1000 robots.

Current gap:

- The repository has core simulation benchmarks, but no dedicated ROS bridge
  performance measurement for endpoint count, publish cost, action/server
  creation, or RMF adapter latency.

Required before claiming a performance win:

- Add ROS bridge benchmarks for at least:
  - startup time and memory vs robot count;
  - per-step publish time with per-robot state publishers on/off;
  - `/fleet/states` publish time and message size;
  - command latency for per-robot action/service vs fleet dispatcher;
  - Plugin Only RMF control latency.

## RMF Performance Measurement Pattern

Add this before or alongside Phase 5 so later changes have a baseline. It should
be an optional benchmark, not a blocking CI gate at first.

### Benchmark Modes

Measure the same RMF scenario through multiple transport/configuration modes:

1. **Per-Robot ROS Baseline**
   - current bridge behavior;
   - per-robot state publishers/actions/services enabled.
2. **Hybrid Fleet State**
   - `/fleet/states` enabled;
   - per-robot high-frequency publishers disabled;
   - per-robot command/service/action compatibility retained.
3. **Fleet ROS Commands**
   - `/fleet/states`;
   - `/fleet/navigate` and later `/fleet/execute_action` / `/fleet/attach`;
   - minimal per-robot ROS interfaces.
4. **Plugin Only**
   - RMF adapter calls `PythonFleetClient`;
   - no ROS bridge in the control path.
5. **Plugin + Bridge**
   - RMF adapter uses `PythonFleetClient`;
   - ROS bridge remains active for observation/debug/external control.

### Scenarios

Start with deterministic low-cardinality scenarios, then scale:

- office patrol: one robot, one task;
- office delivery: dispenser -> ingestor;
- hotel patrol: cross-lift route;
- N-robot idle state publication: 10, 100, 500, 1000 robots;
- N-robot navigation dispatch: 10, 100, 500 robots where feasible.

### Metrics

Collect both wall-clock and sim-time metrics:

- launch/startup time until RMF stack ready;
- ROS graph entity count;
- process RSS memory;
- CPU utilization;
- `/fleet_states` or `/fleet/states` publish rate and message size;
- per-step bridge publish time if instrumented;
- dispatch submit -> task accepted latency;
- task accepted -> first robot motion latency;
- task accepted -> completed latency;
- command request -> acknowledgement latency;
- missed deadlines/timeouts;
- real-time factor during the scenario.

### Tooling

Extend existing smoke scripts instead of replacing them:

- keep `docker/test_rmf_smoke.sh` and `docker/test_rmf_e2e.sh` as correctness
  gates;
- add `docker/rmf_perf_check.py` for metric collection;
- add `docker/test_rmf_perf.sh` as an opt-in benchmark wrapper;
- write JSON results to `benchmark/results/rmf_*.json`;
- summarize results in a future `docs/benchmarking/rmf-results.md`.

The benchmark should accept a scenario and mode:

```bash
docker/test_rmf_perf.sh office_pybullet per_robot "patrol:lounge,coe"
docker/test_rmf_perf.sh office_pybullet hybrid_fleet_state "patrol:lounge,coe"
docker/test_rmf_perf.sh office_pybullet plugin_only "patrol:lounge,coe"
```

### Non-Blocking Policy

Do not make RMF performance benchmarks blocking in CI until they are stable and
have clear thresholds. Initially they should publish artifacts or local JSON
files only. Correctness gates remain the existing smoke/E2E tests.

## Phase 6 — Trace, Snapshot, and Replay Hooks

Purpose: make fleet APIs replayable without ROS.

Tasks:

- Persist `CommandEvent` records from `FleetCommandDispatcher`.
- Persist fleet snapshots from `FleetStateProvider`.
- Keep recorder format open:
  - JSONL;
  - SQLite;
  - MCAP;
  - existing recorder output;
  - future USO-compatible snapshot/delta format.
- Add replay entry point that calls `FleetCommandDispatcher` directly.

Exit criteria:

- A recorded command trace can be replayed without a ROS graph.
- Snapshot and command event schemas are stable enough for CI regression tests.

## Recommended PR Order

1. Config parsing helpers and tests.
2. Handler decomposition skeleton with compatibility facade.
3. Selective per-robot interface creation.
4. Python fleet state/command abstractions.
5. ROS `/fleet/states`, `/fleet/navigate`, `/fleet/joint_command` wrappers.
6. RMF client abstraction (`RosFleetClient`, `PythonFleetClient`, legacy client).
7. RMF state-source and navigation migration.
8. Plugin Only launch path.
9. Plugin + Bridge launch path.
10. ROS bridge performance benchmarks.
11. Fleet action/attach/stop APIs.
12. Trace/replay hooks.

## Implementation Notes

- Prefer small PRs with behavior-preserving steps first.
- Keep `RobotHandler` import compatibility until custom handler users have a
  migration path.
- Avoid long-running work in ROS service callbacks; return acknowledgements and
  observe progress through fleet state.
