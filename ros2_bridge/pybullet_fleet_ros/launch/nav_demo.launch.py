"""Launch bridge with a mobile robot and RViz for navigation demo.

Usage::

    ros2 launch pybullet_fleet_ros nav_demo.launch.py
    ros2 launch pybullet_fleet_ros nav_demo.launch.py gui:=true
    ros2 launch pybullet_fleet_ros nav_demo.launch.py gui:=true rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context: LaunchContext):
    pkg_dir = get_package_share_directory("pybullet_fleet_ros")
    src_config = "/rmf_demos_ws/src/pybullet_fleet_ros/config"

    rviz_config = os.path.join(pkg_dir, "config", "nav_demo.rviz")
    if not os.path.exists(rviz_config):
        rviz_config = os.path.join(src_config, "nav_demo.rviz")

    config_yaml = os.path.join(pkg_dir, "config", "bridge_nav.yaml")
    if not os.path.exists(config_yaml):
        config_yaml = os.path.join(src_config, "bridge_nav.yaml")

    # Read URDF for robot_state_publisher (RViz model visualisation)
    urdf_path = "/opt/pybullet_fleet/robots/mobile_robot.urdf"
    try:
        with open(urdf_path, "r") as f:
            robot_description = f.read()
    except FileNotFoundError:
        robot_description = ""

    gui = context.launch_configurations.get("gui", "")
    target_rtf = context.launch_configurations.get("target_rtf", "")

    bridge_params = {
        "config_yaml": config_yaml,
    }
    if gui:
        bridge_params["gui"] = gui.lower() in ("true", "1", "yes")
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
                namespace="robot0",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "frame_prefix": "robot0/",
                    }
                ],
                remappings=[("joint_states", "/robot0/joint_states")],
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
            DeclareLaunchArgument("target_rtf", default_value=""),
            OpaqueFunction(function=_launch_setup),
        ]
    )
