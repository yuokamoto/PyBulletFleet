"""Launch Open-RMF airport_terminal demo with PyBulletFleet and multiple fleets.

Demonstrates multi-fleet support: four RMF fleets sharing a single bridge_node.
Each fleet uses its own rmf_demos fleet config and nav graph. In
``python_fleet`` mode they run as in-process bridge plugins over the shared
simulation core; ROS-backed modes keep standalone fleet_adapter nodes.

Usage::

    ros2 launch pybullet_fleet_rmf airport_terminal_pybullet.launch.py
    ros2 launch pybullet_fleet_rmf airport_terminal_pybullet.launch.py gui:=true

    # Dispatch task to a specific fleet:
    ros2 run rmf_demos_tasks dispatch_patrol -- -p A -n 1 -F tinyRobot

    # Dispatch cleaning task:
    ros2 run rmf_demos_tasks dispatch_clean -cs zone_1 --use_sim_time
"""

import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate airport demo launch: common infra + bridge + 4 fleet adapters."""
    pkg_dir = get_package_share_directory("pybullet_fleet_rmf")
    rmf_demos_dir = get_package_share_directory("rmf_demos")
    rmf_demos_maps_dir = get_package_share_directory("rmf_demos_maps")

    # Bridge config — all fleets' robots combined
    bridge_config = os.path.join(pkg_dir, "config", "bridge_airport.yaml")

    # rmf_demos airport maps
    building_yaml = os.path.join(rmf_demos_maps_dir, "airport_terminal", "airport_terminal.building.yaml")
    rviz_config = os.path.join(rmf_demos_dir, "include", "airport_terminal", "airport_terminal.rviz")
    airport_config_dir = os.path.join(rmf_demos_dir, "config", "airport_terminal")
    airport_nav_dir = os.path.join(rmf_demos_maps_dir, "maps", "airport_terminal", "nav_graphs")

    # Fleet configs
    # Nav graph assignments must match rmf_demos airport_terminal.launch.xml:
    #   tinyRobot → 2.yaml, deliveryRobot → 1.yaml,
    #   cleanerBotA → 0.yaml, cleanerBotE → 4.yaml
    fleet_configs = {
        "tinyRobot": (os.path.join(airport_config_dir, "tinyRobot_config.yaml"), os.path.join(airport_nav_dir, "2.yaml")),
        "deliveryRobot": (
            os.path.join(airport_config_dir, "deliveryRobot_config.yaml"),
            os.path.join(airport_nav_dir, "1.yaml"),
        ),
        "cleanerBotA": (
            os.path.join(airport_config_dir, "cleanerBotA_config.yaml"),
            os.path.join(airport_nav_dir, "0.yaml"),
        ),
        "cleanerBotE": (
            os.path.join(airport_config_dir, "cleanerBotE_config.yaml"),
            os.path.join(airport_nav_dir, "4.yaml"),
        ),
    }

    rmf_adapters = [
        {
            "name": f"{fleet_name}_fleet_adapter",
            "config_file": config_path,
            "nav_graph": nav_path,
        }
        for fleet_name, (config_path, nav_path) in fleet_configs.items()
    ]

    launch_items = [
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("target_rtf", default_value=""),
        DeclareLaunchArgument("server_uri", default_value="", description="API server WebSocket URI"),
        DeclareLaunchArgument("headless", default_value="false", description="Skip rviz launch"),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation clock"),
        DeclareLaunchArgument(
            "client_mode",
            default_value="python_fleet",
            description="RMF client transport: per_robot_ros, fleet_ros, or python_fleet",
        ),
        # ── RMF common infra ────────────────────────────────────────
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(rmf_demos_dir, "common.launch.xml")),
            launch_arguments={
                "config_file": building_yaml,
                "viz_config_file": rviz_config,
                "headless": LaunchConfiguration("headless"),
                "server_uri": LaunchConfiguration("server_uri"),
                "use_reservation_node": "true",
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
        ),
        # ── PyBulletFleet bridge + RMF fleet adapters ───────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_dir, "launch", "pybullet_common.launch.py")),
            launch_arguments={
                "config_yaml": bridge_config,
                "rmf_adapters": json.dumps(rmf_adapters),
                "gui": LaunchConfiguration("gui"),
                "target_rtf": LaunchConfiguration("target_rtf"),
                "server_uri": LaunchConfiguration("server_uri"),
                "client_mode": LaunchConfiguration("client_mode"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
        ),
    ]

    return LaunchDescription(launch_items)
