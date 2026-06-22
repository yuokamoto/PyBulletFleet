"""Reusable PyBulletFleet simulation nodes for Open-RMF demos.

Launches the nodes that replace Gazebo in an rmf_demos scenario:

1. **bridge_node** — PyBulletFleet simulation backend.
   Door, lift, and workcell handlers are loaded inside this node
   via ``handler_map`` / ``handler_registry`` in the bridge YAML.
2. **fleet_adapter** — Open-RMF EasyFullControl fleet adapter

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

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bridge_node_setup(context: LaunchContext):
    """Build bridge_node with conditional parameter overrides."""
    use_sim_time = context.launch_configurations.get("use_sim_time", "true")

    params = {
        "config_yaml": context.launch_configurations["config_yaml"],
        "use_sim_time": use_sim_time.lower() in ("true", "1", "yes"),
    }
    gui = context.launch_configurations.get("gui", "")
    if gui:
        params["gui"] = gui.lower() in ("true", "1", "yes")
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


def generate_launch_description():
    """Launch PyBulletFleet bridge + RMF adapters."""

    use_sim_time = LaunchConfiguration("use_sim_time")

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
                "use_sim_time",
                default_value="true",
                description="Use simulation clock (/clock) for all nodes. Default true for acceleration.",
            ),
            OpaqueFunction(function=_bridge_node_setup),
            # ── Fleet adapter (wall clock) ──────────────────────────
            # Launched when use_sim_time is false
            Node(
                package="pybullet_fleet_rmf",
                executable="fleet_adapter",
                name="pybullet_fleet_adapter",
                condition=UnlessCondition(use_sim_time),
                arguments=[
                    "-c",
                    LaunchConfiguration("fleet_config"),
                    "-n",
                    LaunchConfiguration("nav_graph"),
                ],
                parameters=[
                    {
                        "server_uri": LaunchConfiguration("server_uri"),
                        "use_sim_time": use_sim_time,
                    }
                ],
                output="screen",
            ),
            # ── Fleet adapter (sim time) ────────────────────────────
            # Launched when use_sim_time is true — passes -sim flag
            Node(
                package="pybullet_fleet_rmf",
                executable="fleet_adapter",
                name="pybullet_fleet_adapter",
                condition=IfCondition(use_sim_time),
                arguments=[
                    "-c",
                    LaunchConfiguration("fleet_config"),
                    "-n",
                    LaunchConfiguration("nav_graph"),
                    "-sim",
                ],
                parameters=[
                    {
                        "server_uri": LaunchConfiguration("server_uri"),
                        "use_sim_time": use_sim_time,
                    }
                ],
                output="screen",
            ),
        ]
    )
