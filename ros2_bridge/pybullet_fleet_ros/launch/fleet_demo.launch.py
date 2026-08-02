"""Launch a configurable fleet-only ROS API demo.

Usage::

    ros2 launch pybullet_fleet_ros fleet_demo.launch.py robots:=100 gui:=true
    ros2 launch pybullet_fleet_ros fleet_demo.launch.py robot_model:=tb3_burger
"""

from pathlib import Path
import os
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from pybullet_fleet_ros.fleet_demo_config import SUPPORTED_ROBOT_MODELS, write_fleet_demo_config


def _as_bool(value: str) -> bool:
    return value.lower() in ("true", "1", "yes")


def _launch_setup(context: LaunchContext):
    robots = int(context.launch_configurations["robots"])
    robot_model = context.launch_configurations["robot_model"]
    gui = _as_bool(context.launch_configurations["gui"])
    target_rtf = float(context.launch_configurations["target_rtf"])

    package_dir = Path(get_package_share_directory("pybullet_fleet_ros"))
    template_path = package_dir / "config" / "bridge_fleet_scale.yaml"
    rviz_config = package_dir / "config" / "fleet_demo.rviz"
    descriptor, config_name = tempfile.mkstemp(prefix="pybullet_fleet_fleet_demo_", suffix=".yaml")
    # The bridge reads this after the launch setup has returned.
    # Close the descriptor before writing the generated YAML by path.
    os.close(descriptor)
    config_path = Path(config_name)
    write_fleet_demo_config(
        template_path,
        config_path,
        robots=robots,
        robot_model=robot_model,
        gui=gui,
        target_rtf=target_rtf,
    )

    return [
        Node(
            package="pybullet_fleet_ros",
            executable="bridge_node",
            name="pybullet_fleet_bridge",
            parameters=[{"config_yaml": str(config_path)}],
            output="screen",
        ),
        ExecuteProcess(
            cmd=[sys.executable, "-m", "pybullet_fleet_ros.fleet_rviz"],
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
    return LaunchDescription(
        [
            DeclareLaunchArgument("robots", default_value="100", description="Number of robots to spawn"),
            DeclareLaunchArgument(
                "robot_model",
                default_value="simple_cube",
                description=f"Robot model: {', '.join(SUPPORTED_ROBOT_MODELS)}",
            ),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="false", description="Launch lightweight Fleet RViz view"),
            DeclareLaunchArgument("target_rtf", default_value="1.0"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
