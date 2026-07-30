# Tutorial 9: Plugins

Plugins package reusable behavior without placing scenario logic in a callback.
Use a `SimPlugin` for behavior owned by the whole simulation, and an
`AgentPlugin` for state or behavior owned by one Agent. This tutorial uses a
small status reporter and the built-in `BatteryPlugin`.

**Source file:** [`examples/basics/plugin_demo.py`](https://github.com/yuokamoto/PyBulletFleet/blob/main/pybullet_fleet/examples/basics/plugin_demo.py)

## Run the demo

```bash
python examples/basics/plugin_demo.py
```

The headless demo creates one moving robot. A configuration-loaded
`StatusReporterPlugin` prints its lifecycle and observes the robot at 2 Hz.
The Agent receives `BatteryPlugin` from its `plugins:` entry, so the reported
battery state decreases while it moves.

## Choose the plugin scope

| Scope | Base class | Owned by | Typical use |
| --- | --- | --- | --- |
| Simulation | `SimPlugin` | `MultiRobotSimulationCore` | Workcells, traffic monitoring, external integration |
| Per agent | `AgentPlugin` | One `Agent` | Battery state, robot-specific sensors, custom status |

Do not use a plugin merely to write a short scenario-specific sequence. A
`register_callback()` callback is simpler for that case. Use a plugin when the
behavior needs a lifecycle, configuration, or reuse across scenarios.

## Simulation plugin lifecycle

`SimPlugin` receives the simulation core and can implement only the hooks it
needs:

```text
construct -> on_init -> on_step (zero or more times) -> on_reset -> on_shutdown
```

`on_init()` runs after config-driven world and entities are created. `on_step()`
runs in the update portion of `step_once()`; a `frequency` in Hz reduces how
often it runs and its `dt` is the elapsed time since the previous plugin call.
`on_shutdown()` runs when `run_simulation()` finishes, before PyBullet
disconnects.

The demo's configuration is equivalent to this application YAML. Replace the
dotted class path with an importable class in your package:

```yaml
plugins:
  - class: my_project.plugins.StatusReporterPlugin
    frequency: 2.0
    config:
      label: status
```

The same plugin can be registered from Python when the application owns its
construction:

```python
plugin = sim.register_plugin(StatusReporterPlugin, config={"label": "status"}, frequency=2.0)
plugin.on_init()  # register_plugin() does not invoke lifecycle hooks itself
```

For config-loaded plugins, `from_dict()` / `from_yaml()` performs lifecycle
initialization after entities have been created.

## Per-agent plugins

Declare an `AgentPlugin` under one entity's `plugins:` key. The built-in battery
plugin is registered as `battery`:

```yaml
entities:
  - name: delivery_01
    urdf_path: robots/mobile_robot.urdf
    controller:
      type: omni
    plugins:
      - type: battery
        config:
          initial_soc: 0.8
          discharge_rate: 0.002
          charge_rate: 0.005
```

The agent exposes convenience accessors for the built-in plugin:

```python
print(agent.battery_soc)
agent.set_charging(True)
```

For a custom per-agent plugin, subclass `AgentPlugin` and add it through the
same `type:` registry entry or a dotted `class:` path. Its `on_update(dt)` runs
as part of the owning Agent's update.

## Step-order rule

`SimPlugin.on_step()`, AgentPlugin `on_update()`, and normal callbacks run in
the update part of a step. Framework `set_pose()` writes there are buffered;
use framework pose getters for same-step state. See
[Two-Phase Step](../architecture/two-phase-step) before calling raw PyBullet
pose or AABB APIs from a plugin.

## See also

- [Plugins and Events](../architecture/plugins-events): extension-point choice and lifecycle contract
- [Tutorial 8 — Event Bus](event-bus): react to transitions without step polling
- [Tutorial 1 — Spawning Objects](spawning-objects): config-driven entity creation
