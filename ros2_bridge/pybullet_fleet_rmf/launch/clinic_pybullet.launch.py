"""Launch Open-RMF clinic demo with PyBulletFleet — 2 fleets, 2 elevators, 10 doors.

Defaults to ``python_fleet`` so all RMF fleet adapters run as in-process bridge
plugins over the shared simulation core. ROS-backed modes keep standalone
fleet_adapter nodes for the additional fleets.
"""

import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory("pybullet_fleet_rmf")
    rmf_demos_dir = get_package_share_directory("rmf_demos")
    rmf_demos_maps_dir = get_package_share_directory("rmf_demos_maps")

    bridge_config = os.path.join(pkg_dir, "config", "bridge_clinic.yaml")
    building_yaml = os.path.join(rmf_demos_maps_dir, "clinic", "clinic.building.yaml")
    rviz_config = os.path.join(rmf_demos_dir, "include", "clinic", "clinic.rviz")
    clinic_config_dir = os.path.join(rmf_demos_dir, "config", "clinic")
    clinic_nav_dir = os.path.join(rmf_demos_maps_dir, "maps", "clinic", "nav_graphs")

    fleet_configs = {
        "deliveryRobot": (
            os.path.join(clinic_config_dir, "deliveryRobot_config.yaml"),
            os.path.join(clinic_nav_dir, "0.yaml"),
        ),
        "tinyRobot": (
            os.path.join(clinic_config_dir, "tinyRobot_config.yaml"),
            os.path.join(clinic_nav_dir, "1.yaml"),
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
        DeclareLaunchArgument("server_uri", default_value=""),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "client_mode",
            default_value="python_fleet",
            description="RMF client transport: per_robot_ros, fleet_ros, or python_fleet",
        ),
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
