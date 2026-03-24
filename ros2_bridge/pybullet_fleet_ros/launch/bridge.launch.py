"""Launch core bridge node with configurable parameters."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("num_robots", default_value="1"),
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("publish_rate", default_value="50.0"),
            DeclareLaunchArgument("target_rtf", default_value="1.0"),
            Node(
                package="pybullet_fleet_ros",
                executable="bridge_node",
                name="pybullet_fleet_bridge",
                parameters=[
                    {
                        "num_robots": LaunchConfiguration("num_robots"),
                        "robot_urdf": "robots/mobile_robot.urdf",
                        "publish_rate": LaunchConfiguration("publish_rate"),
                        "gui": LaunchConfiguration("gui"),
                        "target_rtf": LaunchConfiguration("target_rtf"),
                    }
                ],
                output="screen",
            ),
        ]
    )
