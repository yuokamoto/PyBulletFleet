"""Launch core bridge node with configurable parameters."""

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def _launch_setup(context: LaunchContext):
    params = {
        "config_yaml": context.launch_configurations["config_yaml"],
    }
    gui = context.launch_configurations.get("gui", "")
    if gui:
        params["gui"] = gui.lower() in ("true", "1", "yes")
    enable_monitor_gui = context.launch_configurations.get("enable_monitor_gui", "")
    if enable_monitor_gui:
        params["enable_monitor_gui"] = enable_monitor_gui.lower() in ("true", "1", "yes")
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
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_yaml", description="Path to PyBulletFleet YAML config file"),
            DeclareLaunchArgument("gui", default_value=""),
            DeclareLaunchArgument(
                "enable_monitor_gui",
                default_value="",
                description="Show the tkinter monitor window. Empty = follow gui (hidden when gui:=false).",
            ),
            DeclareLaunchArgument("target_rtf", default_value=""),
            OpaqueFunction(function=_launch_setup),
        ]
    )
