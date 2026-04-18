"""Reusable PyBulletFleet simulation nodes for Open-RMF demos.

Launches the four nodes that replace Gazebo in an rmf_demos scenario:

1. **bridge_node** — PyBulletFleet simulation backend
2. **door_adapter** — instant open/close (replaces Gazebo door plugin)
3. **workcell_adapter** — instant dispenser/ingestor (replaces Gazebo
   TeleportDispenser / TeleportIngestor plugins)
4. **fleet_adapter** — Open-RMF EasyFullControl fleet adapter

Include this from a demo-specific launch file and pass the required
arguments to configure it for each map/scenario.

Required launch arguments (must be declared by the parent launch):
    config_yaml:  Path to PyBulletFleet bridge YAML config file
    fleet_config: Path to RMF fleet adapter config YAML
    nav_graph:    Path to RMF navigation graph YAML

Optional launch arguments (defaults provided):
    gui:          Enable PyBullet GUI (default: ``true``)
    target_rtf:   Target real-time factor (default: ``1.0``)
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("target_rtf", default_value="1.0"),
            DeclareLaunchArgument(
                "server_uri",
                default_value="",
                description="API server WebSocket URI",
            ),
            # ── Door adapter ────────────────────────────────────────
            # Instant open/close (replaces Gazebo door plugin)
            Node(
                package="pybullet_fleet_ros",
                executable="door_adapter",
                name="pybullet_door_adapter",
                output="screen",
            ),
            # ── Workcell adapter ────────────────────────────────────
            # Instant dispenser/ingestor (replaces Gazebo
            # TeleportDispenser/TeleportIngestor plugins)
            Node(
                package="pybullet_fleet_ros",
                executable="workcell_adapter",
                name="pybullet_workcell_adapter",
                output="screen",
            ),
            # ── PyBulletFleet simulation ────────────────────────────
            Node(
                package="pybullet_fleet_ros",
                executable="bridge_node",
                name="pybullet_fleet_bridge",
                parameters=[
                    {
                        "config_yaml": LaunchConfiguration("config_yaml"),
                        "gui": LaunchConfiguration("gui"),
                        "target_rtf": LaunchConfiguration("target_rtf"),
                    }
                ],
                output="screen",
            ),
            # ── Fleet adapter ───────────────────────────────────────
            # Open-RMF EasyFullControl (replaces rmf_demos_fleet_adapter)
            Node(
                package="pybullet_fleet_ros",
                executable="fleet_adapter",
                name="pybullet_fleet_adapter",
                arguments=[
                    "-c",
                    LaunchConfiguration("fleet_config"),
                    "-n",
                    LaunchConfiguration("nav_graph"),
                ],
                parameters=[
                    {
                        "server_uri": LaunchConfiguration("server_uri"),
                    }
                ],
                output="screen",
            ),
        ]
    )
