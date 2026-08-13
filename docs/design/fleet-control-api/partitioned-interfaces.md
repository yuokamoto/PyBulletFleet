# Manager-Scoped Fleet ROS Interfaces

**Date:** 2026-08-08
**Status:** Implemented
**Parent:** [Fleet Control API](spec.md)

## Problem

The scalable bridge publishes one `FleetState` on `/fleet/states` and accepts
batch commands on `/fleet/*`. This remains the compatibility and whole-fleet
interface, but it has two limitations for a large operational deployment:

- A client interested in one operating group still receives every robot state.
- Internal NPCs and simulation-only agents are converted into the external
  state message even when no external client needs them.

The main deployment model already has named managers. A manager owns the agent
set used by a batch controller, and entity grid spawns are routed to it through
the existing `entities[].manager` setting. Reuse that established grouping for
the first selective ROS interface rather than adding a second grouping model.

RMF fleet names are a separate scheduling/business-domain boundary. They must
not be inferred from a manager name, although an application may deliberately
use matching names.

## Decision

Add optional **manager interface groups**. Every exposed named manager owns one
namespaced Fleet API endpoint set. The manager name is both the membership
source and the stable ROS namespace in v1.

```text
/fleet/states                         # existing filtered global snapshot
/fleet/navigate                       # existing global topic and service

/fleet/<manager>/states               # selected manager-owned agents
/fleet/<manager>/navigate             # topic and service; manager-owned agents
/fleet/<manager>/stop
/fleet/<manager>/joint_command
/fleet/<manager>/attach
/fleet/<manager>/execute_action
```

Existing message and service definitions stay unchanged: a message on
`/fleet/delivery_fleet/states` is identified by its topic, rather than by
adding a manager field to every robot state or command. Numeric namespaces such
as `/fleet/1/states` are explicitly rejected: their meaning depends on YAML
order and they are unsuitable for dashboards, access-control policy, and client
configuration.

## Configuration Shape

This schema is implemented by `bridge_node`.

```yaml
managers:
  - name: delivery_fleet
    fleet_controller:
      type: batch_differential
  - name: inspection_fleet
    fleet_controller:
      type: batch_omni
  - name: internal_npc
    fleet_controller:
      type: batch_omni

fleet_api:
  enabled: true
  states: true
  navigate: true

  # /fleet/states contains only agents from these managers. Omit this field to
  # retain today's all-agent global snapshot.
  state_scope: managers
  state_include_managers: [delivery_fleet, inspection_fleet]

  # Each entry creates /fleet/<manager>/* endpoints for that manager's agents.
  manager_interfaces:
    - manager: delivery_fleet
      states: true
      state_publish_rate: 5.0
      state_qos:
        reliability: reliable
        history: keep_last
        depth: 10
        durability: volatile
      navigate: true
      stop: true

    - manager: inspection_fleet
      states: true
      state_publish_rate: 1.0
      state_qos:
        reliability: best_effort
        history: keep_last
        depth: 1
        durability: volatile
      navigate: true

entities:
  - name: delivery_robot
    manager: delivery_fleet
    urdf_path: package://pybullet_fleet/robots/simple_cube.urdf
    grid:
      count: 100
      spacing: [2.0, 2.0]

  - name: npc
    manager: internal_npc
    urdf_path: package://pybullet_fleet/robots/simple_cube.urdf
    grid:
      count: 20
      spacing: [2.0, 2.0]
```

`state_scope` controls the compatibility `/fleet/states` snapshot membership.
`all_agents` includes every Agent, including an Agent with no manager;
`managers` requires `state_include_managers` and prevents agents owned only by
other managers from being collected, converted, or allocated in that message.
This covers the primary case of excluding internal agents. A future
`state_exclude_managers` option can support the inverse policy without changing
the model.

`manager_interfaces` is independent of the global API. A deployment can expose
only manager state streams, retain `/fleet/states` as a compatibility stream,
or expose global state and manager state concurrently during migration. Global
command endpoints remain unrestricted compatibility endpoints; deployments
requiring command isolation must disable or not expose them in the restricted
ROS domain.

## Validation and Runtime Model

After simulation construction, the bridge resolves each configured manager to
its current Agent members and creates a `FleetRosInterface` for every configured
manager endpoint. It rejects an unknown manager and an Agent that belongs to
more than one exposed manager interface.

An empty, known manager emits a warning but does not prevent startup. Its
manager-scoped state stream is empty and its commands have no valid targets.
Membership is deliberately resolved at startup: changing a manager's members
requires restarting the bridge before the endpoint scope changes.

A manager may technically contain non-Agent objects or overlap another manager,
so this validation is required even though the normal YAML spawn path routes
each grid into one named manager.

Each endpoint set wraps its own `FleetStateProvider` and
`FleetCommandDispatcher` over the same simulation core:

1. The global endpoint set requests states for every agent or for
   `state_include_managers`.
2. A manager endpoint set requests only its manager's agent names through
   `FleetStateProvider.get_states_3d(names=...)`.
3. Before dispatch, manager command endpoints reject targets outside their
   manager's resolved agent set.
4. Every message emitted in one bridge publish cycle uses the same simulation
   timestamp. Receiving all manager streams is not an atomic replacement for a
   global snapshot.

The rate comparison uses the shared `ros_time_to_seconds()` conversion utility,
the inverse of `sim_time_to_ros_time()`. This keeps ROS timestamp handling out
of the endpoint implementation and avoids a clock lookup on the publish path.

Before constructing a state message, an endpoint set checks its matched
subscriber count. With zero subscribers it skips state collection, ROS message
conversion, and publication. A publisher still has a small fixed DDS/discovery
cost, but avoiding the per-agent work is what makes manager selection useful at
scale.

## Expected Benefits and Limits

Manager-scoped interfaces provide:

- state omission for internal/simulation-only managers;
- selective subscriptions for clients interested in one manager;
- different state rate, QoS, and history depth for each manager; and
- command boundaries that can be paired with DDS access-control policy.

They do not inherently improve total bridge throughput. When all manager
streams are published and one client subscribes to all of them, state work is
approximately the global snapshot cost plus per-message overhead. They also do
not reduce the work of dispatching a full-fleet command. Commands remain global
by default because they are low frequency; manager command endpoints exist for
ownership and isolation, not as a latency remedy.

## Usage Guidance

Choose a manager according to the agent's external observation and control
needs, rather than merely its robot model:

| Agent category | Manager and ROS exposure | Normal access path |
|---|---|---|
| Main externally controlled fleet | Named operating manager with a manager interface | Continuous `/fleet/<manager>/states` and manager/global fleet commands |
| NPC whose state affects external decisions | Named manager with a state interface at the required rate | Continuous manager state stream; commands only when needed |
| Internal or rarely inspected NPC | `npc` manager, or no named manager | On-demand simulation services such as `/sim/get_entity_state` and `/sim/set_entity_state` |
| Legacy whole-fleet client | Add only the required operating managers to `state_include_managers` | Filtered `/fleet/states` plus existing global fleet endpoints |

The simulation services are independent of `fleet_api` and
`manager_interfaces`. In particular, `/sim/get_entity_state` accepts an entity
name and can inspect an Agent or SimObject even when it is absent from all
continuous FleetState streams. `/sim/get_entities` discovers available names;
`/sim/get_entities_states` is intentionally on-demand because it returns every
entity and should not substitute for a high-rate state topic.

Do not expose a low-frequency NPC manager solely to support occasional state
inspection. A service call has a smaller steady-state cost and makes that
operational intent explicit. Promote the NPC to a state-exposed manager only
when an external client must continuously make decisions from its state.

## Documentation

The user-facing documentation covers the implemented surface:

- `docs/ros2/configuration.md`: document `state_include_managers` and
  `manager_interfaces`, their endpoint names, validation rules, QoS/rate
  settings, and the distinction from RMF fleet names.
- `docs/ros2/quickstart.md` and the ROS demo catalog: provide one example with
  an externally exposed operating manager and an omitted `npc` manager.
- RMF integration documentation: the direct `python_fleet` client does not
  require manager endpoints, while `fleet_ros` selects its endpoint namespace
  explicitly.
- `ros2_bridge/PERFORMANCE.md`: manager-scoped endpoints are not a throughput
  optimisation when every stream is subscribed; their benefit is selective
  observation and command ownership.

## Validation Plan

Extend the scale checker to compare, at 100, 500, and 1000 robots:

| Case | State subscribers | What it establishes |
|---|---|---|
| Global | one `/fleet/states` subscriber | baseline whole-fleet cost |
| Manager selective | one manager state subscriber | selective-client benefit |
| Manager complete | one subscriber to every manager stream | total-work overhead |
| Manager distributed | one subscriber per manager stream | executor/DDS fan-out behavior |

For every case record achieved RTF, CPU, publish-to-callback p50/p99,
per-manager serialized size, missed samples, and full-snapshot completion
latency. Start with the current same-host profile. Network and slow-subscriber
tests are separate follow-up work before recommending `BEST_EFFORT` or
`KEEP_LAST(1)` defaults.

## Non-Goals

- Do not replace `/fleet/*` or change existing custom message definitions.
- Do not split a single command merely to reduce its size.
- Do not model RMF fleet identity inside `FleetState`.
- Do not introduce tags, regular expressions, or arbitrary robot-name
  selectors in the initial implementation.
- Do not add per-robot ROS handlers; this remains a fleet-level interface.
