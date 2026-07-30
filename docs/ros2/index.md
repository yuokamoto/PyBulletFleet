# ROS 2 Bridge and Open-RMF

PyBulletFleet provides a ROS 2 Jazzy bridge for external tools and an Open-RMF
adapter for fleet-management scenarios. These are optional integrations: the
Python simulation core remains usable without ROS 2.

This section is for users integrating a simulator with ROS applications, RMF,
or fleet-management tools. Framework extension contracts belong in the
[architecture](../architecture/index) section.

```{toctree}
:maxdepth: 2

overview
configuration
rmf
```

## Choose an environment

| Environment | Best for | Start here |
| --- | --- | --- |
| Docker | Reproducible bridge and RMF demos; no native ROS install | [Docker bridge guide](https://github.com/yuokamoto/PyBulletFleet/blob/main/docker/README.md) |
| Native Jazzy | Ubuntu 24.04 development, apt packages, and overlays | [Native Jazzy setup](https://github.com/yuokamoto/PyBulletFleet/blob/main/ros2_bridge/NATIVE_ROS2.md) |
| Jazzy apt packages | Running an installed bridge outside a source checkout | [Bridge installation reference](https://github.com/yuokamoto/PyBulletFleet/blob/main/ros2_bridge/README.md#jazzy-apt-installation) |

The ROS packages provide interfaces and nodes. Install the separately released
`pybullet-fleet` Python package in the Python environment that launches ROS
nodes; the package includes the simulation core and PyBullet dependency.

## Which control path should I use?

| Need | Recommended path |
| --- | --- |
| Inspect or control individual robots with standard ROS interfaces | Per-robot API |
| Send one command or receive one state snapshot for a large fleet | Fleet API |
| Run an RMF demo with the lowest ROS control-path overhead | `python_fleet` |
| Keep RMF commands and state visible on ROS fleet endpoints | `fleet_ros` |
| Preserve an existing per-robot ROS integration | `per_robot_ros` |

See [Bridge overview](overview) for endpoint semantics, [configuration](configuration)
for enabling interfaces, and [Open-RMF](rmf) for the RMF client modes.
