"""Launch core bridge node with configurable parameters."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_yaml", description="Path to PyBulletFleet YAML config file"),
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("publish_rate", default_value="50.0"),
            DeclareLaunchArgument("target_rtf", default_value="1.0"),
            Node(
                package="pybullet_fleet_ros",
                executable="bridge_node",
                name="pybullet_fleet_bridge",
                parameters=[
                    {
                        "config_yaml": LaunchConfiguration("config_yaml"),
                        "publish_rate": LaunchConfiguration("publish_rate"),
                        "gui": LaunchConfiguration("gui"),
                        "target_rtf": LaunchConfiguration("target_rtf"),
                    }
                ],
                output="screen",
            ),
        ]
    )
