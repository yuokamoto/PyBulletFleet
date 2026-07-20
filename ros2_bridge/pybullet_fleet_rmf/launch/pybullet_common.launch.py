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
    rmf_adapters: YAML/JSON list of RMF adapter configs. Each entry needs
                  ``config_file`` and may include ``nav_graph``, ``name``,
                  ``server_uri``, ``client_mode``, and ``use_sim_time``.

Optional launch arguments (defaults provided):
    gui:          Enable PyBullet GUI (default: use YAML config)
    target_rtf:   Target real-time factor (default: use YAML config)
    server_uri:   API server WebSocket URI (default: ``""``)
    client_mode:  RMF client transport (default: ``python_fleet``)

Example (from a demo launch file)::

    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, "launch", "pybullet_common.launch.py")
        ),
        launch_arguments={
            "config_yaml": bridge_config,
            "rmf_adapters": json.dumps(
                [{"config_file": fleet_config, "nav_graph": nav_graph}]
            ),
        }.items(),
    )
"""

import os
import subprocess
import tempfile

import yaml
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from pybullet_fleet_ros.uri_utils import resolve_package_uris

RMF_ADAPTER_PLUGIN_CLASS = "pybullet_fleet_rmf.rmf_adapter_plugin.RmfAdapterBridgePlugin"


def _as_bool(value: str, default: bool = False) -> bool:
    if value == "":
        return default
    return value.lower() in ("true", "1", "yes")


def _native_python_env() -> dict:
    """Expose the repo venv to ROS console scripts in native development."""
    paths = []
    repo_root = os.environ.get("PBF_REPO_ROOT", "")
    if repo_root:
        paths.append(repo_root)
    venv = os.environ.get("PBF_VENV", "")
    if venv:
        python = os.path.join(venv, "bin", "python")
        try:
            site_packages = subprocess.check_output(
                [
                    python,
                    "-c",
                    "import sysconfig; print(sysconfig.get_paths()['purelib'])",
                ],
                text=True,
                stderr=subprocess.DEVNULL,
            ).strip()
        except (OSError, subprocess.SubprocessError, KeyError):
            site_packages = ""
        if site_packages:
            paths.append(site_packages)
    if not paths:
        return {}
    pythonpath = os.environ.get("PYTHONPATH", "")
    if pythonpath:
        paths.append(pythonpath)
    return {"PYTHONPATH": ":".join(paths)}


def _append_in_process_rmf_plugin(
    bridge_config: dict,
    *,
    config_file: str,
    nav_graph: str = "",
    client_mode: str = "python_fleet",
    server_uri: str = "",
    use_sim_time: bool = True,
    rmf_frame_offset=None,
) -> dict:
    """Return bridge config with the in-process RMF adapter plugin added."""
    updated = dict(bridge_config)
    plugins = list(updated.get("bridge_plugins", []))
    if any(
        entry.get("class") == RMF_ADAPTER_PLUGIN_CLASS
        and entry.get("config", {}).get("config_file") == config_file
        and entry.get("config", {}).get("nav_graph", "") == nav_graph
        for entry in plugins
        if isinstance(entry, dict)
    ):
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
                "rmf_frame_offset": list(rmf_frame_offset or [0.0, 0.0]),
            },
        }
    )
    updated["bridge_plugins"] = plugins
    return updated


def _enable_fleet_api_for_rmf_client(bridge_config: dict) -> dict:
    """Return bridge config with the fleet ROS endpoints required by RMF enabled."""
    updated = dict(bridge_config)
    fleet_api = dict(updated.get("fleet_api") or {})
    fleet_api.update(
        {
            "enabled": True,
            "states": True,
            "navigate": True,
            "stop": True,
            "attach": True,
        }
    )
    updated["fleet_api"] = fleet_api
    return updated


def _rmf_adapters_from_yaml(value: str) -> list[dict]:
    """Parse the required YAML/JSON adapter list from a launch argument."""
    if not value:
        raise ValueError("rmf_adapters must contain at least one adapter")
    loaded = yaml.safe_load(value)
    if not isinstance(loaded, list):
        raise ValueError("rmf_adapters must be a YAML/JSON list")
    if not loaded:
        raise ValueError("rmf_adapters must contain at least one adapter")
    adapters = []
    for item in loaded:
        if not isinstance(item, dict):
            raise ValueError("each rmf_adapters entry must be a mapping")
        if not item.get("config_file"):
            raise ValueError("each rmf_adapters entry needs config_file")
        adapters.append(dict(item))
    return adapters


def _validate_adapter_client_modes(adapters: list[dict], client_mode: str) -> None:
    """Reject per-adapter client_mode overrides that cannot be routed safely."""
    for adapter in adapters:
        adapter_mode = str(adapter.get("client_mode", client_mode)).strip().lower()
        if adapter_mode != client_mode:
            raise ValueError(
                "rmf_adapters client_mode overrides must match the global "
                f"client_mode ({client_mode!r}); got {adapter_mode!r}"
            )


def _bridge_config_for_client_mode(
    *,
    config_yaml: str,
    rmf_adapters: str,
    client_mode: str,
    server_uri: str = "",
    use_sim_time: bool = True,
) -> str:
    """Return bridge config path, generating a merged temp config if needed."""
    adapters = _rmf_adapters_from_yaml(rmf_adapters)
    _validate_adapter_client_modes(adapters, client_mode)
    if client_mode not in {"python_fleet", "fleet_ros"}:
        return config_yaml

    with open(config_yaml, "r") as f:
        bridge_config = yaml.safe_load(f) or {}
    bridge_config = resolve_package_uris(bridge_config)
    if client_mode == "python_fleet":
        rmf_frame_offset = bridge_config.get("rmf_frame_offset", [0.0, 0.0])
        for adapter in adapters:
            bridge_config = _append_in_process_rmf_plugin(
                bridge_config,
                config_file=adapter["config_file"],
                nav_graph=adapter.get("nav_graph", ""),
                client_mode=adapter.get("client_mode", client_mode),
                server_uri=adapter.get("server_uri", server_uri),
                use_sim_time=adapter.get("use_sim_time", use_sim_time),
                rmf_frame_offset=adapter.get("rmf_frame_offset", rmf_frame_offset),
            )
    else:
        bridge_config = _enable_fleet_api_for_rmf_client(bridge_config)
    fd, path = tempfile.mkstemp(prefix="pbf_rmf_bridge_", suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump(bridge_config, f, sort_keys=False)
    return path


def _bridge_node_setup(context: LaunchContext):
    """Build bridge_node with conditional parameter overrides."""
    use_sim_time = context.launch_configurations.get("use_sim_time", "true")
    client_mode = context.launch_configurations.get("client_mode", "python_fleet").strip().lower()
    effective_use_sim_time = _as_bool(use_sim_time, True)
    config_yaml = _bridge_config_for_client_mode(
        config_yaml=context.launch_configurations["config_yaml"],
        rmf_adapters=context.launch_configurations["rmf_adapters"],
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
            additional_env=_native_python_env(),
            output="screen",
        ),
    ]


def _fleet_adapter_setup(context: LaunchContext):
    """Build the standalone RMF adapter unless bridge_node owns it in-process."""
    client_mode = context.launch_configurations.get("client_mode", "python_fleet").strip().lower()
    if client_mode == "python_fleet":
        return []

    use_sim_time = context.launch_configurations.get("use_sim_time", "true")
    default_use_sim_time = _as_bool(use_sim_time, True)
    nodes = []
    adapters = _rmf_adapters_from_yaml(context.launch_configurations["rmf_adapters"])
    _validate_adapter_client_modes(adapters, client_mode)
    for index, adapter in enumerate(adapters):
        adapter_use_sim_time = _as_bool(str(adapter.get("use_sim_time", use_sim_time)), default_use_sim_time)
        arguments = ["-c", adapter["config_file"]]
        nav_graph = adapter.get("nav_graph", "")
        if nav_graph:
            arguments.extend(["-n", nav_graph])
        if adapter_use_sim_time:
            arguments.append("-sim")
        arguments.extend(["--client-mode", client_mode])
        nodes.append(
            Node(
                package="pybullet_fleet_rmf",
                executable="fleet_adapter",
                name=adapter.get("name", f"pybullet_fleet_adapter_{index}"),
                arguments=arguments,
                parameters=[
                    {
                        "server_uri": adapter.get("server_uri", context.launch_configurations.get("server_uri", "")),
                        "use_sim_time": adapter_use_sim_time,
                    }
                ],
                additional_env=_native_python_env(),
                output="screen",
            )
        )
    return nodes


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
                "rmf_adapters",
                description="YAML/JSON list of RMF adapter configs",
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
                default_value="python_fleet",
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
