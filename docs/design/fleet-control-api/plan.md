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
  - `RobotState2D`
  - `RobotState3D`
  - `RobotGoalCommand2D`
  - `RobotGoalCommand3D`
  - `RobotJointPositionsCommand`
  - `RobotNamedJointPositionsCommand`
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

## Phase 4.5 — Fleet API Examples and Integration Checks

Purpose: prove the Phase 3/4 fleet APIs through the existing examples and
smoke tests before starting the RMF migration.

This phase should not add a separate `fleet_api_demo.py` unless an existing
example cannot exercise the API cleanly. For performance comparison, the same
scene, robot count, loop timing, and measurement path should be reusable with
per-agent, batch, and fleet API control paths.

Tasks:

- Add a fleet API mode to an existing non-ROS scale example:
  - default to the batch controller plus fleet command interface;
  - add a CLI or config switch such as `--command-interface per_agent|fleet`;
  - use `FleetStateProvider` and `FleetCommandDispatcher` in fleet mode;
  - keep the existing performance counters usable for both modes.
- Add a non-ROS benchmark path through `benchmark/run_benchmark.py`:
  - keep `mobile` / `arm` as benchmark types and compare per-agent, batch, and
    fleet dispatcher command paths as separate mobile controller and command
    ingress axes;
  - keep worker process isolation and JSON result output.
- Add or document a ROS bridge config/profile that enables the fleet API
  wrappers while preserving the existing per-robot compatibility interfaces.
- Add an optional ROS fleet scale check:
  - generate a temporary bridge config with many robots;
  - disable per-robot ROS interfaces for the scale path;
  - command all robots through `/fleet/navigate`.
- Update integration checks so fleet APIs are verified explicitly:
  - use a Python checker for multi-step assertions instead of large inline
    shell snippets;
  - verify `/fleet/states`, `/fleet/navigate`, and `/fleet/joint_command` when
    the bridge config enables them;
  - keep the smoke test fast and deterministic enough for CI.
- Document manual pre-merge commands for Yu to run:
  - non-ROS batch example;
  - non-ROS fleet API example;
  - ROS bridge fleet API smoke check;
  - any optional local performance comparison command.

Exit criteria:

- Existing examples still run with their default behavior.
- The same non-ROS example can compare batch control and fleet API control.
- The benchmark runner can compare per-agent, batch, and fleet control paths.
- Docker integration confirms `FleetStateProvider`,
  `FleetCommandDispatcher`, and `FleetRosInterface` are wired together.
- Optional Docker scale check can command many robots through `/fleet/navigate`
  without per-robot ROS interfaces.
- Manual verification steps are written down before merge.

## Phase 5 — RMF Migration and Plugin-Only Path

Purpose: reduce RMF scaling pressure and make Plugin Only / Plugin + Bridge
deployment possible without duplicating RMF adapter logic.

The subphase labels keep the RMF work grouped under one architectural change:
the RMF adapter stops depending directly on one ROS transport shape. The
recommended PR order still splits these subphases into separate PRs, but they
share the same success condition and should be evaluated together.

### Phase 5a — RMF Client Abstraction

Status: in progress.

Tasks:

- Extract the RMF-facing client contract from the per-robot ROS client.
- Define a transport-independent per-robot facade consumed by
  `RobotAdapter`:
  - `get_data()`
  - `navigate(cmd_id, position, map_name, speed_limit)`
  - `stop()`
  - `start_charge(cmd_id)` / `stop_charge()`
  - `toggle_attach(attach, cmd_id)`
- Add a client factory so the adapter can choose the transport without
  changing RMF callback logic.
- Implement `RosFleetClient` using ROS `/fleet/states` and `/fleet/navigate`
  for patrol/navigation.
- Implement `PythonFleetClient` using `FleetStateProvider` and
  `FleetCommandDispatcher` directly.
- Keep the existing per-robot ROS client path as a compatibility implementation.

Current implementation notes:

- `per_robot_ros` remains the default and preserves existing demos.
- `fleet_ros` is available as an experimental RMF client mode for
  `/fleet/states` + `/fleet/navigate` + `/fleet/stop` + `/fleet/attach` +
  `/fleet/execute_action`; charging still uses per-robot compatibility
  services until a fleet-level charge API exists.
- `PythonFleetClient` is still pending and should reuse the same client
  factory/interface.

Exit criteria:

- RMF adapter logic depends on the client abstraction, not directly on ROS
  topics/actions/services.
- Existing per-robot ROS RMF demos still work.

### Phase 5b — RMF over Typed Fleet Commands

Tasks:

- Let RMF consume fleet state snapshots instead of per-robot state publishers.
- Add `/fleet/navigate` / dispatcher-backed navigation support to the client
  abstraction.
- Add `/fleet/stop` / dispatcher-backed stop support to the client abstraction.
- Add `/fleet/attach` / dispatcher-backed attach support to the client
  abstraction.
- Add `/fleet/execute_action` / dispatcher-backed generic action enqueue
  support.
- Keep existing per-robot command/service/action paths initially for delivery
  and compatibility.

Exit criteria:

- Patrol can run with fleet state, fleet navigation, and fleet stop.
- Delivery attach/drop can use fleet attach instead of per-robot attach
  services.
- Generic PyBulletFleet actions can be queued through a fleet endpoint; RMF
  category mapping remains explicit in the adapter.

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
- Delivery remains supported through fleet attach; remaining per-robot
  compatibility paths are mainly charging and any RMF custom categories that
  are not yet mapped onto fleet endpoints.

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
- Use the Docker fleet scale checker for early diagnosis before RMF integration:
  - compare `fleet`, `per_robot`, and `hybrid` interface modes with motion
    verification enabled so topic-only per-robot commands are not measured only
    by publish-loop time;
  - in `hybrid`, sweep per-robot groups (`state_publishers`, `tf`,
    `command_topics`, `services`, `actions`) to isolate endpoint families that
    add service latency or executor pressure;
  - treat `--target-rtf 0.0` as a stress mode. Also record `target_rtf=1.0`
    because an unconstrained simulation thread can starve ROS callbacks when
    high-cardinality per-robot publishers are enabled.

Current observations from the Docker scale checker:

- Fleet-only ROS remains the expected baseline for large fleets. In a 1000 robot
  check, `/fleet/navigate` returned in tens of milliseconds at `target_rtf=1.0`
  and motion verification completed shortly after.
- Hybrid mode is sensitive to per-robot endpoint count. Exposing per-robot state
  publishers, TF, command topics, services, or actions can add seconds of fleet
  service latency at 1000 robots.
- Per-robot action servers are not viable at 1000 robots in the current Docker
  environment; creating them can abort in the DDS/RCL layer.
- Per-robot topic commands are not equivalent to fleet service commands for
  performance claims: they are fire-and-forget and need motion verification.
  At hundreds of robots, not all commands were applied reliably in the current
  bridge setup.
- `publish_rate` must throttle fleet state and per-robot state/TF publishers,
  not just `/clock`; this is required before comparing hybrid modes.

## RMF Performance Measurement Pattern

Track this under Phase 6c. It should be an optional benchmark, not a blocking
CI gate at first.

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

## Phase 6 — Extension, Optimization, and Replay

Purpose: keep Phase 5 focused on RMF migration while moving extension hooks,
optimization work, benchmark refreshes, and replayability into a follow-up
phase.

### Phase 6a — Generic Fleet Command Extension

Purpose: let users add fleet-level commands without requiring a new built-in
ROS message and dispatcher method for every project-specific behavior.

Tasks:

- Add a generic fleet command surface, separate from the typed standard APIs:
  - candidate ROS endpoint: `/fleet/custom_command` or `/fleet/command`;
  - candidate payload fields: `command_id`, `source`, `command_type`,
    `names`, and `params_json`.
- Add a Python-side command handler registry on or beside
  `FleetCommandDispatcher`:
  - `register_command_handler(command_type, handler)`;
  - handler receives resolved agents, command params, source, and command id;
  - handler returns accepted/rejected details or raises a validation error.
- Keep typed APIs as the recommended path for standard behavior:
  - `/fleet/navigate`;
  - `/fleet/stop`;
  - `/fleet/attach`;
  - `/fleet/execute_action`;
  - `/fleet/joint_command`.
- Ensure unknown command types are rejected explicitly and logged.
- Decide whether generic command payloads should use JSON only, or allow a
  typed envelope plus JSON parameters.

Exit criteria:

- A user/plugin can register and execute one custom fleet command without
  modifying core message definitions.
- Typed standard fleet APIs remain unchanged and continue to be the RMF-facing
  default.
- Custom command events are recorded with enough metadata for trace/replay.

### Phase 6b — RMF Command Coalescing

Purpose: reduce burst load when RMF issues many per-robot callbacks in a short
window while preserving RMF's per-robot callback contract.

Tasks:

- Add optional command coalescing in `RosFleetClient`:
  collect per-robot RMF `navigate()` callbacks that arrive within a short flush
  window and send them as one `/fleet/navigate` request.
- Keep coalescing conservative or disabled by default until ack/rejection and
  latency semantics are validated.
- Measure RMF per-robot navigate callback -> coalesced `/fleet/navigate` flush
  latency.
- Compare coalescing on/off for command latency and task-completion impact.

Exit criteria:

- Optional coalescing can be enabled without changing `RobotAdapter` callback
  code.
- Single-robot navigation remains the default behavior.

### Phase 6c — Post Fleet-API Performance Refresh

Purpose: refresh performance results after the fleet API and batch-controller
paths are in place, using comparable conditions instead of mixing older
PyBulletFleet-only numbers with ROS bridge measurements.

Tasks:

- Define a common 1000-robot benchmark matrix:
  - PyBulletFleet only, per-agent command path;
  - PyBulletFleet only, batch controller with per-agent command ingress;
  - PyBulletFleet only, batch controller with fleet command ingress;
  - ROS bridge fleet-only with `/fleet/navigate` service;
  - ROS bridge fleet-only with `/fleet/navigate` topic and motion verification;
  - ROS bridge hybrid profiles for selected per-robot groups;
  - Plugin Only;
  - Plugin + Bridge.
- Keep the Docker fleet scale checker as a smoke/diagnostic tool before the
  full benchmark refresh:
  - service `/fleet/navigate` should verify ack and motion;
  - topic `/fleet/navigate` should verify publish matching and motion;
  - detailed service-vs-topic performance comparison remains part of this
    refresh.
- Add a separate joint-command benchmark case instead of folding it into the
  mobile navigation path:
  - PyBulletFleet only, arm or mixed mobile-manipulator joint commands;
  - update `benchmark/arm_benchmark.py` with a mobile-benchmark-equivalent
    command-interface switch once fleet joint commands are implemented;
  - reuse the existing benchmark CLI vocabulary where applicable
    (`--agents`, `--collision-freq`, `--mode`, `--controller`,
    `--command-interface`) so mobile and arm control-path measurements stay
    comparable;
  - ROS bridge `/fleet/joint_command` topic/service;
  - hybrid mode with any required per-robot compatibility interfaces;
  - Plugin Only joint command dispatch through `FleetCommandDispatcher`.
- Keep key conditions explicit and aligned:
  - robot model and controller;
  - `physics=false`;
  - `gui=false`;
  - `timestep=0.1`;
  - `target_rtf=0` for maximum RTF;
  - matching movement workload and command cadence;
  - matching robot count and grid spacing.
- Record both maximum RTF and latency metrics:
  - sim step time;
  - service command request -> ack latency;
  - topic command publish time and publish -> first motion latency;
  - command request -> first motion latency;
  - joint command request -> ack latency and first joint motion latency;
  - fleet state snapshot/publish time;
  - memory and startup time where available.
- Update or replace stale benchmark summaries in:
  - `docs/benchmarking/results.md`;
  - `benchmark/README.md`;
  - future ROS 2 Bridge ReadTheDocs performance page;
  - `ros2_bridge/PERFORMANCE.md` until the ReadTheDocs page exists.
- Clearly label historical results that use different hardware, timestep,
  controller, or workload so they are not used for direct ROS-vs-non-ROS
  claims.

Exit criteria:

- PyBulletFleet-only and ROS bridge fleet API numbers are measured on the same
  machine with the same 1000-robot workload.
- Any claim about ROS overhead is backed by an aligned PyBulletFleet-only
  baseline.
- ReadTheDocs and local benchmark docs no longer present mixed-condition values
  as directly comparable.

### Phase 6d — Scale Example Config Cleanup

Purpose: align non-ROS scale examples with the newer `entities[].grid` config
pattern used by the core config loader and ROS bridge examples, without
breaking existing example configs immediately.

Tasks:

- Update `pybullet_fleet/config/100robots_config.yaml` to make
  `entities[].grid` the primary example scene definition.
- Keep the old top-level `num_robots`, `grid`, `spacing`, `offset`, and
  robot-specific sections as commented legacy examples or compatibility notes.
- Update `pybullet_fleet/examples/scale/100robots_grid_demo.py` to prefer
  `entities[]` when present while retaining fallback support for the current
  top-level example format.
- Document that new scale examples should use `entities[].grid` so scene
  definitions can be shared more easily across non-ROS examples, benchmarks,
  and ROS bridge configs.
- Review other `pybullet_fleet/examples/scale/*` demos for ad hoc grid setup
  and leave follow-up notes where converting to `entities[]` would reduce
  duplication.

Exit criteria:

- The 100-robot grid demo still runs with existing configs.
- The default example config uses `entities[].grid` as the visible primary
  pattern.
- Legacy top-level scale keys remain understandable but are no longer the
  recommended pattern.

### Phase 6e — Trace, Snapshot, and Replay Hooks

Purpose: make fleet APIs replayable without ROS.

Tasks:

- Persist `CommandEvent` records from `FleetCommandDispatcher`.
- Persist fleet snapshots from `FleetStateProvider`.
- Include typed fleet commands and generic custom commands in the same trace
  stream.
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
6. Fleet API examples and integration checks.
7. RMF client abstraction (`RosFleetClient`, `PythonFleetClient`, legacy client).
8. RMF state-source and typed fleet command migration.
9. Plugin Only launch path.
10. Plugin + Bridge launch path.
11. Generic custom fleet command extension.
12. RMF command coalescing.
13. ROS bridge performance benchmark refresh.
14. Scale example config cleanup.
15. Trace/replay hooks.

## Implementation Notes

- Prefer small PRs with behavior-preserving steps first.
- Keep `RobotHandler` import compatibility until custom handler users have a
  migration path.
- Avoid long-running work in ROS service callbacks; return acknowledgements and
  observe progress through fleet state.
