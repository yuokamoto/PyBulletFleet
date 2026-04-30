"""Launch Open-RMF office demo with PyBulletFleet replacing Gazebo.

Reuses ``common.launch.xml`` from ``rmf_demos`` for all shared RMF
infrastructure (traffic schedule, blockade, map server, task dispatcher,
door/lift/mutex supervisors, and RMF visualization/rviz).

Only the simulation backend (Gazebo → PyBulletFleet bridge) and the
fleet adapter (rmf_demos_fleet_adapter → pybullet_fleet_rmf) are replaced.
These nodes are provided by ``pybullet_common.launch.py`` and included
here for reuse across different demo maps.

Usage::

    # Start the demo:
    ros2 launch pybullet_fleet_rmf office_pybullet.launch.py
    ros2 launch pybullet_fleet_rmf office_pybullet.launch.py gui:=true

    # With web dashboard (start api-server + dashboard via docker compose):
    ros2 launch pybullet_fleet_rmf office_pybullet.launch.py \\
        server_uri:=ws://localhost:8000/_internal

    # Dispatch a patrol (from another terminal):
    ros2 run rmf_demos_tasks dispatch_patrol -- -p patrol_A1 patrol_D1 -n 3

    # Go to a specific place:
    ros2 run rmf_demos_tasks dispatch_go_to_place -- -p pantry
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate office demo launch: rmf_demos common infra + PyBulletFleet bridge + fleet adapter."""
    pkg_dir = get_package_share_directory("pybullet_fleet_rmf")
    rmf_demos_dir = get_package_share_directory("rmf_demos")
    rmf_demos_maps_dir = get_package_share_directory("rmf_demos_maps")

    # Config paths — use our custom fleet config to avoid charging deadlock.
    # The original rmf_demos config uses finishing_request="charge" +
    # recharge_soc=1.0 which deadlocks because our sim doesn't drain battery.
    bridge_config = os.path.join(pkg_dir, "config", "bridge_office.yaml")
    fleet_config = os.path.join(pkg_dir, "config", "tinyRobot_office.yaml")
    nav_graph = os.path.join(rmf_demos_maps_dir, "maps", "office", "nav_graphs", "0.yaml")
    building_yaml = os.path.join(rmf_demos_maps_dir, "office", "office.building.yaml")
    rviz_config = os.path.join(rmf_demos_dir, "include", "office", "office.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("target_rtf", default_value=""),
            DeclareLaunchArgument(
                "server_uri",
                default_value="",
                description="API server WebSocket URI (e.g. ws://localhost:8000/_internal)",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Skip rviz launch",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock",
            ),
            # ============================================================
            # RMF common infra — reuse rmf_demos common.launch.xml
            # (traffic schedule, blockade, map server, task dispatcher,
            #  door/lift/mutex supervisors, rviz visualization)
            # ============================================================
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
            # ============================================================
            # PyBulletFleet simulation + RMF adapters
            # (bridge_node + workcell_handler, door_adapter, fleet_adapter)
            # ============================================================
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_dir, "launch", "pybullet_common.launch.py")),
                launch_arguments={
                    "config_yaml": bridge_config,
                    "fleet_config": fleet_config,
                    "nav_graph": nav_graph,
                    "gui": LaunchConfiguration("gui"),
                    "target_rtf": LaunchConfiguration("target_rtf"),
                    "server_uri": LaunchConfiguration("server_uri"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ]
    )
