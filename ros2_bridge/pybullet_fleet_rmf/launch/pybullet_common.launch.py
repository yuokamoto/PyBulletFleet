"""Reusable PyBulletFleet simulation nodes for Open-RMF demos.

Launches the nodes that replace Gazebo in an rmf_demos scenario:

1. **bridge_node** — PyBulletFleet simulation backend.
   Door, lift, and workcell handlers are loaded inside this node
   via ``handler_map`` / ``handler_registry`` in the bridge YAML.
2. **fleet_adapter** — Open-RMF EasyFullControl fleet adapter, either as a
   standalone node or as an in-process bridge plugin for ``python_fleet``

Include this from a demo-specific launch file and pass the required
arguments to configure it for each map/scenario.

Required launch arguments (must be declared by the parent launch):
    config_yaml:  Path to PyBulletFleet bridge YAML config file
    fleet_config: Path to RMF fleet adapter config YAML
    nav_graph:    Path to RMF navigation graph YAML

Optional launch arguments (defaults provided):
    gui:          Enable PyBullet GUI (default: use YAML config)
    target_rtf:   Target real-time factor (default: use YAML config)
    server_uri:   API server WebSocket URI (default: ``""``)
    client_mode:  RMF client transport (default: ``per_robot_ros``)

Example (from a demo launch file)::

    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, "launch", "pybullet_common.launch.py")
        ),
        launch_arguments={
            "config_yaml": bridge_config,
            "fleet_config": fleet_config,
            "nav_graph": nav_graph,
        }.items(),
    )
"""

import os
import tempfile

import yaml
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

RMF_ADAPTER_PLUGIN_CLASS = "pybullet_fleet_rmf.rmf_adapter_plugin.RmfAdapterBridgePlugin"


def _as_bool(value: str, default: bool = False) -> bool:
    if value == "":
        return default
    return value.lower() in ("true", "1", "yes")


def _append_in_process_rmf_plugin(
    bridge_config: dict,
    *,
    config_file: str,
    nav_graph: str = "",
    client_mode: str = "python_fleet",
    server_uri: str = "",
    use_sim_time: bool = True,
) -> dict:
    """Return bridge config with the in-process RMF adapter plugin added."""
    updated = dict(bridge_config)
    plugins = list(updated.get("bridge_plugins", []))
    if any(entry.get("class") == RMF_ADAPTER_PLUGIN_CLASS for entry in plugins if isinstance(entry, dict)):
        return updated

    plugins.append(
        {
            "class": RMF_ADAPTER_PLUGIN_CLASS,
            "config": {
                "config_file": config_file,
                "nav_graph": nav_graph,
                "client_mode": client_mode,
                "server_uri": server_uri,
                "use_sim_time": bool(use_sim_time),
            },
        }
    )
    updated["bridge_plugins"] = plugins
    return updated


def _bridge_config_for_client_mode(
    *,
    config_yaml: str,
    fleet_config: str,
    nav_graph: str,
    client_mode: str,
    server_uri: str = "",
    use_sim_time: bool = True,
) -> str:
    """Return bridge config path, generating a merged temp config if needed."""
    if client_mode != "python_fleet":
        return config_yaml

    with open(config_yaml, "r") as f:
        bridge_config = yaml.safe_load(f) or {}
    bridge_config = _append_in_process_rmf_plugin(
        bridge_config,
        config_file=fleet_config,
        nav_graph=nav_graph,
        client_mode=client_mode,
        server_uri=server_uri,
        use_sim_time=use_sim_time,
    )
    fd, path = tempfile.mkstemp(prefix="pbf_rmf_bridge_", suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump(bridge_config, f, sort_keys=False)
    return path


def _bridge_node_setup(context: LaunchContext):
    """Build bridge_node with conditional parameter overrides."""
    use_sim_time = context.launch_configurations.get("use_sim_time", "true")
    client_mode = context.launch_configurations.get("client_mode", "per_robot_ros").strip().lower()
    effective_use_sim_time = _as_bool(use_sim_time, True)
    config_yaml = _bridge_config_for_client_mode(
        config_yaml=context.launch_configurations["config_yaml"],
        fleet_config=context.launch_configurations["fleet_config"],
        nav_graph=context.launch_configurations["nav_graph"],
        client_mode=client_mode,
        server_uri=context.launch_configurations.get("server_uri", ""),
        use_sim_time=effective_use_sim_time,
    )

    params = {
        "config_yaml": config_yaml,
        "use_sim_time": effective_use_sim_time,
    }
    gui = context.launch_configurations.get("gui", "")
    if gui:
        params["gui"] = _as_bool(gui)
    target_rtf = context.launch_configurations.get("target_rtf", "")
    if target_rtf:
        params["target_rtf"] = float(target_rtf)

    return [
        Node(
            package="pybullet_fleet_ros",
            executable="bridge_node",
            name="pybullet_fleet_bridge",
            parameters=[params],
            output="screen",
        ),
    ]


def _fleet_adapter_setup(context: LaunchContext):
    """Build the standalone RMF adapter unless bridge_node owns it in-process."""
    client_mode = context.launch_configurations.get("client_mode", "per_robot_ros").strip().lower()
    if client_mode == "python_fleet":
        return []

    use_sim_time = context.launch_configurations.get("use_sim_time", "true")
    arguments = [
        "-c",
        context.launch_configurations["fleet_config"],
        "-n",
        context.launch_configurations["nav_graph"],
    ]
    if _as_bool(use_sim_time, True):
        arguments.append("-sim")
    arguments.extend(["--client-mode", client_mode])

    return [
        Node(
            package="pybullet_fleet_rmf",
            executable="fleet_adapter",
            name="pybullet_fleet_adapter",
            arguments=arguments,
            parameters=[
                {
                    "server_uri": context.launch_configurations.get("server_uri", ""),
                    "use_sim_time": _as_bool(use_sim_time, True),
                }
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    """Launch PyBulletFleet bridge + RMF adapters."""

    return LaunchDescription(
        [
            # ── Arguments ────────────────────────────────────────────
            DeclareLaunchArgument(
                "config_yaml",
                description="Path to PyBulletFleet bridge YAML config file",
            ),
            DeclareLaunchArgument(
                "fleet_config",
                description="Path to RMF fleet adapter config YAML",
            ),
            DeclareLaunchArgument(
                "nav_graph",
                description="Path to RMF navigation graph YAML",
            ),
            DeclareLaunchArgument("gui", default_value=""),
            DeclareLaunchArgument("target_rtf", default_value=""),
            DeclareLaunchArgument(
                "server_uri",
                default_value="",
                description="API server WebSocket URI",
            ),
            DeclareLaunchArgument(
                "client_mode",
                default_value="per_robot_ros",
                description="RMF client transport: per_robot_ros, fleet_ros, or python_fleet",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock (/clock) for all nodes. Default true for acceleration.",
            ),
            OpaqueFunction(function=_bridge_node_setup),
            OpaqueFunction(function=_fleet_adapter_setup),
        ]
    )
