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
            DeclareLaunchArgument("target_rtf", default_value=""),
            OpaqueFunction(function=_launch_setup),
        ]
    )
