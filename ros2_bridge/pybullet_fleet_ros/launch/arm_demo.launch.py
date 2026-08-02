"""Launch bridge with a Franka Panda and RViz for arm control demo.

Usage::

    ros2 launch pybullet_fleet_ros arm_demo.launch.py
    ros2 launch pybullet_fleet_ros arm_demo.launch.py gui:=true
    ros2 launch pybullet_fleet_ros arm_demo.launch.py gui:=true rviz:=false
"""

import os
from pathlib import Path
import re

import pybullet_data
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context: LaunchContext):
    pkg_dir = get_package_share_directory("pybullet_fleet_ros")
    src_config = "/rmf_demos_ws/src/pybullet_fleet_ros/config"

    rviz_config = os.path.join(pkg_dir, "config", "arm_demo.rviz")
    if not os.path.exists(rviz_config):
        rviz_config = os.path.join(src_config, "arm_demo.rviz")

    config_yaml = os.path.join(pkg_dir, "config", "bridge_arm.yaml")
    if not os.path.exists(config_yaml):
        config_yaml = os.path.join(src_config, "bridge_arm.yaml")

    # PyBullet ships Panda, but its generated URDF uses package://meshes.
    # Expand that package-relative URI to the installed pybullet_data path for RViz.
    urdf_path = Path(pybullet_data.getDataPath()) / "franka_panda" / "panda.urdf"
    try:
        mesh_dir = urdf_path.parent / "meshes"
        robot_description = urdf_path.read_text(encoding="utf-8").replace("package://meshes", f"file://{mesh_dir}")
        # RViz does not need collision geometry. Removing it also avoids a
        # PyBullet collision MTL that contains a development-machine texture path.
        robot_description = re.sub(r"\s*<collision>.*?</collision>", "", robot_description, flags=re.DOTALL)
        # Panda's base and wrist have no separate visual mesh and point to the
        # same collision assets, so omit only those two RViz visual elements.
        robot_description = re.sub(
            r"\s*<visual>.*?meshes/collision/.*?</visual>",
            "",
            robot_description,
            flags=re.DOTALL,
        )
    except FileNotFoundError:
        robot_description = ""

    rviz = context.launch_configurations.get("rviz", "true")
    gui = context.launch_configurations.get("gui", "")
    target_rtf = context.launch_configurations.get("target_rtf", "")

    bridge_params = {
        "config_yaml": config_yaml,
    }
    if gui:
        bridge_params["gui"] = gui.lower() in ("true", "1", "yes")
    enable_monitor_gui = context.launch_configurations.get("enable_monitor_gui", "")
    if enable_monitor_gui:
        bridge_params["enable_monitor_gui"] = enable_monitor_gui.lower() in ("true", "1", "yes")
    if target_rtf:
        bridge_params["target_rtf"] = float(target_rtf)

    nodes = [
        Node(
            package="pybullet_fleet_ros",
            executable="bridge_node",
            name="pybullet_fleet_bridge",
            parameters=[bridge_params],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ]

    if robot_description:
        nodes.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                namespace="panda0",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "frame_prefix": "panda0/",
                    }
                ],
                remappings=[("joint_states", "/panda0/joint_states")],
                output="screen",
                condition=IfCondition(LaunchConfiguration("rviz")),
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value=""),
            DeclareLaunchArgument("rviz", default_value="true", description="Launch RViz"),
            DeclareLaunchArgument(
                "enable_monitor_gui",
                default_value="",
                description="Show the tkinter monitor window. Empty = follow gui (hidden when gui:=false).",
            ),
            DeclareLaunchArgument("target_rtf", default_value=""),
            OpaqueFunction(function=_launch_setup),
        ]
    )
