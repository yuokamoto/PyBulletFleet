# Open-RMF Integration

`pybullet_fleet_rmf` connects PyBulletFleet robots to Open-RMF through
`rmf_adapter.easy_full_control`. RMF demo launches default to `python_fleet`,
the in-process path that avoids ROS messages on the RMF command path.

## RMF client modes

| Mode | Command/state path | Use it when |
| --- | --- | --- |
| `python_fleet` | RMF bridge plugin -> `FleetStateProvider` / `FleetCommandDispatcher` -> simulation core | You want the lowest overhead or a plugin-only deployment |
| `fleet_ros` | RMF adapter -> `/fleet/states` and fleet command services -> bridge node | You need ROS-visible batched fleet control |
| `per_robot_ros` | RMF adapter -> per-robot ROS actions/services/topics -> bridge node | You need compatibility with per-robot ROS integrations |

`python_fleet` requires the adapter to run as a bridge plugin so it shares the
same simulation core. It is not a standalone `fleet_adapter` process mode.
`fleet_ros` requires the matching `fleet_api` endpoints to be enabled. See
[Bridge configuration](configuration).

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

The Docker guide documents the runnable demo, dispatch flow check, RMF stack
check, and client-mode matrix. These checks are the recommended validation path
after changing a bridge configuration or selecting a new client mode.
