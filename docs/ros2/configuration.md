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

They do **not** filter the fleet API: its state provider currently includes all
Agents, and its dispatcher resolves all uniquely named Agents. A manager is an
execution group, not an API visibility group. Until the planned fleet API scope
feature is available, disable unwanted fleet endpoints or expose a filtered
view from an integration/plugin.

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
