# Open-RMF Integration

`pybullet_fleet_rmf` connects PyBulletFleet robots to Open-RMF through
`rmf_adapter.easy_full_control`. RMF demo launches default to `python_fleet`,
the in-process path that avoids ROS messages on the RMF command path.

## RMF client modes

| Mode | Command/state path | Use it when |
| --- | --- | --- |
| `python_fleet` | RMF bridge plugin -> `FleetStateProvider` / `FleetCommandDispatcher` -> simulation core | You want the lowest overhead or a plugin-only deployment |
| `fleet_ros` | RMF adapter -> Fleet ROS API -> bridge node | The RMF adapter itself must use ROS fleet endpoints |
| `per_robot_ros` | RMF adapter -> per-robot ROS actions/services/topics -> bridge node | You need compatibility with per-robot ROS integrations |

`python_fleet` requires the adapter to run as a bridge plugin so it shares the
same simulation core. It is not a standalone `fleet_adapter` process mode.
`fleet_ros` requires the matching `fleet_api` endpoints to be enabled. See
[Bridge configuration](configuration).

## Two independent choices

Do not use `fleet_ros` to mean “publish fleet ROS endpoints.”  These are
separate settings:

| Choice | Values | What it controls |
| --- | --- | --- |
| RMF `client_mode` | `python_fleet`, `fleet_ros`, `per_robot_ros` | How the RMF adapter reads state and sends commands to the simulation |
| Bridge exported API | `fleet_api`, `per_robot_api` | Which ROS endpoints external tools can use or observe |

For example, `client_mode:=python_fleet` can still enable `fleet_api` and
publish `/fleet/*` for RViz, monitoring, or debugging.  Conversely,
`client_mode:=fleet_ros` makes the RMF adapter use that Fleet ROS API; the
launch config enables the required endpoint groups for it.

The three modes are alternatives for the RMF adapter's command/state boundary;
they do not describe three stages that are chained together.

```{mermaid}
flowchart TB
  rmf["Open-RMF schedule and tasks"]

  subgraph python_mode["python_fleet (default)"]
    plugin["RmfAdapterBridgePlugin"]
    python_api["Fleet state and command API"]
    sim_python["Simulation core"]
    plugin <-->|In-process Python| python_api
    python_api <-->|In-process Python| sim_python
  end

  subgraph fleet_mode["fleet_ros"]
    adapter_fleet["Standalone fleet adapter"]
    fleet_endpoints["Fleet ROS API"]
    bridge_fleet["bridge node"]
    sim_fleet["Simulation core"]
    adapter_fleet <-->|ROS 2| fleet_endpoints
    fleet_endpoints <-->|ROS 2| bridge_fleet
    bridge_fleet <-->|In-process Python| sim_fleet
  end

  subgraph robot_mode["per_robot_ros"]
    adapter_robot["Standalone fleet adapter"]
    robot_endpoints["Per-robot ROS endpoints"]
    bridge_robot["bridge node"]
    sim_robot["Simulation core"]
    adapter_robot <-->|ROS 2| robot_endpoints
    robot_endpoints <-->|ROS 2| bridge_robot
    bridge_robot <-->|In-process Python| sim_robot
  end

  rmf <-->|RMF APIs| plugin
  rmf <-->|RMF APIs| adapter_fleet
  rmf <-->|RMF APIs| adapter_robot

  linkStyle 0,1,4,7 stroke:#616161,stroke-width:2px
  linkStyle 2,3,5,6 stroke:#1976d2,stroke-width:3px
  linkStyle 8,9,10 stroke:#7b1fa2,stroke-width:3px
```

`python_fleet` bypasses ROS only on the RMF command/state path.  The same
bridge can still export the Fleet ROS API or selected per-robot endpoints for
observation or debugging.
Purple links are RMF-facing APIs, blue links are ROS 2 communication, and gray
links are in-process Python calls.  Each is bidirectional: commands and task
updates travel in opposite directions.

## Supported scenarios and task behavior

The repository provides office, hotel, airport terminal, clinic, campus, and
battle-royale launch configurations. Patrol, delivery, and configured clean
coverage tasks are supported. Delivery uses the bridge's attach/drop path.

Charging is not physically simulated: battery state is fixed at 100%, so an RMF
fleet configuration should use `finishing_request: nothing` or `park` to avoid
charging-related deadlock. Unknown custom action categories are logged and
finished until a simulator-side mapping is provided.

Door, lift, and workcell handlers connect RMF infrastructure protocols to the
simulated environment. The complete capability matrix and launch-file list are
kept in the [bridge README](https://github.com/yuokamoto/PyBulletFleet/blob/main/ros2_bridge/README.md#pybullet_fleet_rmf--open-rmf-integration).

## Run and validate

Use Docker for the fully provisioned RMF demo environment, including the
`rmf_demos` assets. Native and apt installs require the documented Jazzy setup;
RMF demo launch files additionally require an `rmf_demos` source overlay.

Start an office demo with the default `python_fleet` client mode:

```bash
cd docker
docker compose run --rm bridge \
  ros2 launch pybullet_fleet_rmf office_pybullet.launch.py
```

In another terminal, dispatch a patrol:

```bash
cd docker
docker compose run --rm bridge \
  ros2 run rmf_demos_tasks dispatch_patrol -p pantry lounge -n 3
```

Use `client_mode:=fleet_ros` only when the RMF adapter itself must communicate
through the Fleet ROS API.  Keep `client_mode:=python_fleet` for the in-process
control path, even when `fleet_api` is separately enabled for observation or
debugging.  Use `client_mode:=per_robot_ros` for per-robot compatibility.  The
Docker guide documents the runnable demo, dispatch flow check, RMF stack check,
and client-mode matrix; these are the recommended validation path after
changing a bridge configuration or selecting a new client mode.
