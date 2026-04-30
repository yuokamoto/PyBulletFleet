"""Launch Open-RMF battle_royale demo with PyBulletFleet."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory("pybullet_fleet_ros")
    rmf_demos_dir = get_package_share_directory("rmf_demos")
    rmf_demos_maps_dir = get_package_share_directory("rmf_demos_maps")

    bridge_config = os.path.join(pkg_dir, "config", "bridge_battle_royale.yaml")
    fleet_config = os.path.join(rmf_demos_dir, "config", "battle_royale", "tinyRobot_config.yaml")
    nav_graph = os.path.join(rmf_demos_maps_dir, "maps", "battle_royale", "nav_graphs", "0.yaml")
    building_yaml = os.path.join(rmf_demos_maps_dir, "battle_royale", "battle_royale.building.yaml")
    rviz_config = os.path.join(rmf_demos_dir, "include", "battle_royale", "battle_royale.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("target_rtf", default_value=""),
            DeclareLaunchArgument("server_uri", default_value=""),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
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
