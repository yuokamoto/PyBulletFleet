# Bridge Configuration

This page is for ROS users selecting which endpoints `bridge_node` exports. It
does not change the core simulation's controller or batch-execution settings.

## Exported API groups

`fleet_api` controls batched fleet endpoints. `per_robot_api` controls groups of
endpoints under each robot namespace. Both may be enabled for debugging, but
for scale-sensitive deployments enable only the API that the client needs.

```yaml
fleet_api:
  enabled: true
  states: true
  navigate: true
  stop: true
  attach: true
  execute_action: true
  joint_command: false

per_robot_api:
  enabled: false
  state_publishers: false
  tf: false
  command_topics: false
  services: false
  actions: false
```

The per-robot groups deliberately avoid a single all-or-nothing handler. For
example, a debugging configuration can publish state while omitting actions and
services that would otherwise add ROS entities for every robot.

### Limit per-robot interfaces to controlled robots

When a scene includes mock people, external robots, or devices represented as
`Agent` objects, do not leave the per-robot API at its all-Agent default. Use
an allow-list for the robots controlled through ROS:

```yaml
per_robot_api:
  enabled: true
  state_publishers: true
  tf: true
  command_topics: true
  services: false
  actions: false
  include_robots: [delivery_01, delivery_02]
```

`exclude_robots` is the inverse form when most Agents should be exposed. These
filters prevent handler, publisher, subscriber, service, and action-server
creation for excluded actors.

### Manager-scoped Fleet API endpoints

Use named `AgentManager` instances to expose a selective state stream and a
manager-owned command boundary. A manager name is an execution grouping and a
stable ROS namespace; it is not an RMF fleet name.

```yaml
fleet_api:
  enabled: true
  states: true
  # Keep the compatibility stream, while avoiding work for internal NPCs.
  state_scope: managers
  state_include_managers: [delivery]
  manager_interfaces:
    - manager: delivery
      states: true
      state_publish_rate: 5.0
      state_qos:
        reliability: reliable
        history: keep_last
        depth: 10
        durability: volatile
      navigate: true
      stop: true
      joint_command: true

managers:
  - name: delivery
  - name: npc
```

This creates `/fleet/delivery/states`, `/fleet/delivery/navigate`,
`/fleet/delivery/stop`, and `/fleet/delivery/joint_command` (each command has
both its existing topic and service form). `attach` and `execute_action` may be
enabled in the same way. Manager commands reject targets outside their resolved
Agent membership. The global `/fleet/*` commands remain unrestricted for
compatibility.

`state_scope` makes the global `/fleet/states` membership explicit:

- `all_agents` includes every Agent, including Agents with no manager. This is
  the default when the field is omitted for compatibility.
- `managers` requires `state_include_managers` and includes only those named
  managers' Agents.

`state_include_managers` affects only the global compatibility snapshot, not
the manager streams. The global filtered stream and every manager stream use
the same complete `state_qos` structure. A state stream with no matched
subscribers skips state collection and ROS-message conversion. Manager state
rate can only reduce the bridge's configured `publish_rate`, which is the
shared scheduling ceiling.

For a manager-only deployment, use `fleet_api.states: false` and retain only
the `manager_interfaces[].states` streams; neither `state_scope` nor
`state_include_managers` is then needed.

Each listed manager must exist. A manager with no Agents logs a warning rather
than preventing the bridge from starting; its manager-scoped state stream is
empty and its commands have no valid targets. Membership is resolved at bridge
startup, so add Agents before starting the bridge (or restart it after changing
membership). An Agent cannot belong to more than one exposed
`manager_interfaces` entry. Put rarely inspected NPCs in an unexposed `npc`
manager and use the on-demand `/sim/get_entity_state` or
`/sim/get_entities_states` services instead.

For a copyable complete configuration with `delivery_fleet`,
`inspection_fleet`, and an unexposed `npc` manager, start with
[`bridge_fleet_multi_manager_demo.yaml`](https://github.com/yuokamoto/PyBulletFleet/blob/main/ros2_bridge/pybullet_fleet_ros/config/bridge_fleet_multi_manager_demo.yaml).

## Common deployment choices

| Scenario | `fleet_api` | `per_robot_api` |
| --- | --- | --- |
| Fleet manager at scale | Enable required fleet commands and state | Disable |
| Existing Nav2-like tooling | Disable or enable for observation | Enable required groups |
| Hybrid debugging | Enable selected endpoints | Enable only inspected groups |
| Plugin-only RMF path | Disable | Disable |

`client_mode` is a separate RMF adapter setting. It chooses the RMF-to-simulator
control path; it does not automatically enable or disable exported bridge APIs.
For example, `python_fleet` can still export `/fleet/*` for observation, while
a plugin-only deployment can disable both groups.

## Configuration files and assets

The bridge receives the core simulation YAML through its `config_yaml`
parameter. For installed ROS packages, resolve bundled files using the ament
prefix rather than a source-tree path:

```bash
ros2 run pybullet_fleet_ros bridge_node --ros-args \
  -p config_yaml:="$(ros2 pkg prefix pybullet_fleet_ros)/share/pybullet_fleet_ros/config/bridge_test.yaml"
```

Bridge and RMF configs may use `package://` URIs for worlds, SDFs, URDFs, and
meshes. `bridge_node` resolves these through the ROS ament index, which lets
the same configuration work in Docker, a native overlay, and an installed
package.

For complete parameter defaults and launch examples, consult the
[bridge README](https://github.com/yuokamoto/PyBulletFleet/blob/main/ros2_bridge/README.md)
and the environment-specific setup guide linked from the [ROS 2 landing page](index).
