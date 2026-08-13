"""Launch the copyable multi-manager Fleet ROS demonstration.

Usage::

    ros2 launch pybullet_fleet_ros multi_manager_fleet_demo.launch.py gui:=true
    ros2 launch pybullet_fleet_ros multi_manager_fleet_demo.launch.py \
        gui:=false rviz:=true view_manager:=inspection_fleet

The scene comes directly from ``bridge_fleet_multi_manager_demo.yaml``: two
manager-scoped Fleet endpoints are exposed and the ``npc`` manager is omitted
from continuous FleetState publication.
"""

from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from pybullet_fleet_ros.fleet_endpoints import manager_fleet_namespace


def _as_bool(value: str) -> bool:
    return value.lower() in ("true", "1", "yes")


def _launch_setup(context: LaunchContext):
    package_dir = Path(get_package_share_directory("pybullet_fleet_ros"))
    config_path = package_dir / "config" / "bridge_fleet_multi_manager_demo.yaml"
    rviz_config = package_dir / "config" / "fleet_demo.rviz"
    gui = _as_bool(context.launch_configurations["gui"])
    target_rtf = float(context.launch_configurations["target_rtf"])
    view_manager = context.launch_configurations["view_manager"]

    return [
        Node(
            package="pybullet_fleet_ros",
            executable="bridge_node",
            name="pybullet_fleet_bridge",
            parameters=[
                {
                    "config_yaml": str(config_path),
                    "gui": gui,
                    "target_rtf": target_rtf,
                }
            ],
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                sys.executable,
                "-m",
                "pybullet_fleet_ros.fleet_rviz",
                "--fleet-namespace",
                manager_fleet_namespace(view_manager),
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", str(rviz_config)],
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ]


def generate_launch_description():
    """Launch the fixed, directly editable multi-manager example."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("target_rtf", default_value="1.0"),
            DeclareLaunchArgument(
                "view_manager",
                default_value="delivery_fleet",
                description="Manager stream displayed in RViz",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
